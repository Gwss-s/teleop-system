"""GR00T-N1.7 adapter: serve NVIDIA GR00T behind the PolicyServer protocol.

Runs INSIDE Isaac-GR00T's own env (uv .venv) as an isolated process
(third_party rule #1: heavy VLA stacks never share an env; the protocol is
the only contact surface).  Launch inside that env (cf. scripts/server_libero_policy.sh).

Conventions verified against the official integration
(Isaac-GR00T gr00t/eval/sim/LIBERO/libero_env.py, N1.7):
  * images: video.image / video.wrist_image = agentview / eye-in-hand,
    both 180-deg rotated ([::-1, ::-1]) -- IDENTICAL to our LIBERO client
    (envs/libero/eval_client.obs_to_request already rotates), so images
    pass through untouched;
  * state(8) = [eef_xyz(3), quat2axisangle(3), gripper_qpos(2)] -- their
    "roll/pitch/yaw" naming is actually the same axis-angle formula our
    client uses; direct slice mapping, no conversion;
  * policy IO: flat keys via Gr00tSimPolicyWrapper, batched (B=1, T=1);
    returns dict 'action.{x,y,z,roll,pitch,yaw,gripper}';
  * gripper chain (their LiberoEnv.step): model emits [0,1] ->
    normalize to [-1,1] + binarize(sign) -> INVERT (RLDS 1=open vs
    robosuite -1=open).  Replicated here so the emitted chunk is directly
    executable, same contract as server_lerobot (pi05) output.

Response: f32[chunk, 7].  Client should execute --n-action-steps 8
(GR00T's official rollout horizon).

Run (inside Isaac-GR00T .venv, repo roots on PYTHONPATH):
  python -m teleop_system.policy.adapters.groot \
      --ckpt <GR00T-N1.7-LIBERO>/libero_spatial --port 5573
"""
import argparse
import socket

import numpy as np

from ..protocol import recv_msg, send_msg

ACTION_KEYS = ("x", "y", "z", "roll", "pitch", "yaw", "gripper")


def build_obs(req):
    """PolicyServer request -> Gr00tSimPolicyWrapper flat obs (B=1, T=1)."""
    st = np.asarray(req["state"], np.float32)
    imgs = req["images"]
    obs = {
        "video.image": np.ascontiguousarray(imgs["agentview"])[None, None],
        "video.wrist_image": np.ascontiguousarray(imgs["wrist"])[None, None],
        "state.x": st[0:1][None, None],
        "state.y": st[1:2][None, None],
        "state.z": st[2:3][None, None],
        "state.roll": st[3:4][None, None],
        "state.pitch": st[4:5][None, None],
        "state.yaw": st[5:6][None, None],
        "state.gripper": st[6:8][None, None],
        "task": [req["task"]],
    }
    return obs


def assemble_chunk(action):
    """Flat action dict -> f32[chunk, 7] with the official gripper chain."""
    comps = []
    for k in ACTION_KEYS:
        a = np.asarray(action[f"action.{k}"], dtype=np.float32)
        while a.ndim > 2:          # (B,T,d) -> (T,d)
            a = a[0]
        if a.ndim == 1:            # (T,) -> (T,1)
            a = a[:, None]
        comps.append(a)
    chunk = np.concatenate(comps, axis=-1)          # (T, 7)
    # official gripper chain (libero_env.step): [0,1] -> [-1,1], binarize, invert
    g = 2.0 * chunk[:, -1] - 1.0
    g = np.sign(g)
    g[g == 0.0] = 1.0
    chunk[:, -1] = -g
    return np.ascontiguousarray(chunk, dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True,
                    help="per-suite checkpoint dir, e.g. .../GR00T-N1.7-LIBERO/libero_spatial")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5573)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--embodiment-tag", default="LIBERO_PANDA")
    args = ap.parse_args()

    # heavy imports inside main: gr00t only exists in its own env
    from gr00t.data.embodiment_tags import EmbodimentTag
    from gr00t.policy.gr00t_policy import Gr00tPolicy, Gr00tSimPolicyWrapper

    print(f"[groot-server] loading {args.ckpt}", flush=True)
    policy = Gr00tPolicy(
        embodiment_tag=EmbodimentTag.resolve(args.embodiment_tag),
        model_path=args.ckpt,
        device=args.device,
    )
    policy = Gr00tSimPolicyWrapper(policy)

    def infer(req):
        out = policy.get_action(build_obs(req))
        action = out[0] if isinstance(out, tuple) else out
        return assemble_chunk(action)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(4)
    print(f"[groot-server] ready on {args.host}:{args.port} "
          f"(embodiment={args.embodiment_tag})", flush=True)
    while True:
        conn, addr = srv.accept()
        print(f"[groot-server] client {addr}", flush=True)
        try:
            n = 0
            while True:
                req = recv_msg(conn)
                if req is None:
                    break
                chunk = infer(req)
                send_msg(conn, chunk)
                n += 1
                if n % 20 == 1:
                    print(f"[groot-server] inference #{n} -> {chunk.shape}", flush=True)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()
            print("[groot-server] client disconnected", flush=True)


if __name__ == "__main__":
    main()
