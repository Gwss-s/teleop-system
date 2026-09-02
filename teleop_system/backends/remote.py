"""Cross-process teleop bridge: run the DEVICE layer in whatever env has the
vendor SDK, stream TeleopState over TCP to a client in another env.

Why this exists: xrobotoolkit_sdk is a compiled cp312 pybind module (built from
source, not on PyPI) — a py3.10 sim/robot env cannot import it. The 3-layer
design reserves this seam: the device layer talks to hardware in the SDK env,
mapping + sim/robot client live in the consumer env, joined by a localhost
socket. Any future cross-machine integration reuses the same shape.

Wire format: plain dicts of numpy arrays (policy/protocol.py framing). No
teleop classes cross the boundary -> no cross-env pickle identity issues.

Publisher (env/OS WITH the device, e.g. SDK env, or Windows-native for gamepad):
  PYTHONPATH=<repo> python -m teleop_system.backends.remote \
      --serve --port 5570 [--backend pico|gamepad] [--sides right] [--tap raw_tap_XXX.npz]
  (raw tap in remote mode is recorded HERE, on the publisher side; typical
  Windows+WSL2 split: gamepad publisher on Windows, sim consumer in WSL2)

Client (any env): RemoteBackend("localhost", 5570).read() -> TeleopState
"""
import argparse
import socket

import numpy as np

from teleop_system.policy.protocol import recv_msg, send_msg
from ..types import SideState, TeleopState


def _encode(ts):
    return {"t": ts.t_wall, "ts_ns": ts.ts_dev_ns, "buttons": dict(ts.buttons),
            "sides": {k: {"pos": s.pos, "rot": s.rot, "grip": s.grip, "trigger": s.trigger}
                      for k, s in ts.sides.items()}}


def _decode(d):
    return TeleopState(
        sides={k: SideState(pos=np.asarray(v["pos"], dtype=np.float64),
                            rot=np.asarray(v["rot"], dtype=np.float64),
                            grip=float(v["grip"]), trigger=float(v["trigger"]))
               for k, v in d["sides"].items()},
        buttons=dict(d["buttons"]), t_wall=float(d["t"]),
        ts_dev_ns=int(d.get("ts_ns", 0)))   # 旧发布端无此键 -> 0(向后兼容)


class RemoteBackend:
    """Layer-1 backend over the wire: read() -> TeleopState from a publisher."""

    def __init__(self, host="localhost", port=5570, timeout=3.0):
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.settimeout(timeout)

    def read(self):
        send_msg(self._sock, {"kind": "read"})
        r = recv_msg(self._sock)
        if r is None:
            raise ConnectionResetError("teleop bridge publisher gone")
        return _decode(r)

    def close(self):
        try:
            send_msg(self._sock, {"kind": "bye"})
            self._sock.close()
        except OSError:
            pass


def serve(backend, host="0.0.0.0", port=5570, log=print):
    """Poll server: one read() of `backend` per client request. Survives client
    disconnects (next client can attach); Ctrl-C to stop."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(2)
    log(f"[teleop-bridge] serving TeleopState on {host}:{port}")
    while True:
        conn, addr = srv.accept()
        log(f"[teleop-bridge] client {addr}")
        try:
            while True:
                m = recv_msg(conn)
                if m is None or m.get("kind") == "bye":
                    break
                send_msg(conn, _encode(backend.read()))
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()
            log("[teleop-bridge] client disconnected")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serve", action="store_true", required=True)
    ap.add_argument("--port", type=int, default=5570)
    ap.add_argument("--backend", choices=("pico", "gamepad"), default="pico",
                    help="发布哪个设备(gamepad 典型用法: Windows 原生读手柄,发给 WSL2/其他环境)")
    ap.add_argument("--gamepad-config", default="configs/gamepad.yaml")
    ap.add_argument("--sides", default="right", help="comma list: right,left(仅 pico)")
    ap.add_argument("--tap", default="", help="raw-stream npz path (publisher-side,仅 pico)")
    args = ap.parse_args()
    from .factory import make_backend  # noqa: PLC0415
    backend = make_backend(args.backend, sides=tuple(args.sides.split(",")),
                           tap_path=args.tap or None,
                           gamepad_config=args.gamepad_config)
    try:
        serve(backend, port=args.port)
    except KeyboardInterrupt:
        pass
    finally:
        backend.close()


if __name__ == "__main__":
    main()
