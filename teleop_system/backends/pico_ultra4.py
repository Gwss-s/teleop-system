"""Layer-1 device backend: Pico Ultra 4 via XRoboToolkit (`xrobotoolkit_sdk`).

Only knows the device. Reads controller poses/analogs/buttons, transforms
XR (Y-up) -> canonical world (Z-up), and emits a TeleopState. Knows NOTHING
about robots, clutches, scaling, or engagement — that is the mapping layer.

Raw-stream tap: every read() can be recorded verbatim (raw XR
pose + analogs, BEFORE any mapping), so a collection session can later be
re-mapped with different yaml configs without re-teleoperating. Enable by
passing tap_path; rows are flushed to a compressed npz on close().

Runtime deps: `xrobotoolkit_sdk` importable; XRoboToolkit PC Service running;
headset app connected with Controller tracking + Send enabled.
"""
import time

import numpy as np

from ..geometry import xr_pose_to_world
from ..types import SideState, TeleopState


class PicoUltra4:
    """Poll-based: call read() once per control beat -> TeleopState."""

    def __init__(self, sides=("right",), xrt=None, tap_path=None):
        if xrt is None:
            import xrobotoolkit_sdk as xrt  # noqa: PLC0415 -- optional dep
            xrt.init()
        self.xrt = xrt
        self.sides = tuple(sides)
        self.tap_path = tap_path
        self._tap = []          # rows: (t_wall, side_idx, pose7, grip, trigger, A, B)

    # -- raw SDK reads ---------------------------------------------------------
    def _raw(self, side):
        x = self.xrt
        if side == "right":
            return (np.asarray(x.get_right_controller_pose(), dtype=np.float64),
                    float(x.get_right_grip()), float(x.get_right_trigger()))
        return (np.asarray(x.get_left_controller_pose(), dtype=np.float64),
                float(x.get_left_grip()), float(x.get_left_trigger()))

    # -- layer-1 API -----------------------------------------------------------
    def read(self):
        """One beat: raw device state -> TeleopState (canonical world frame)."""
        t = time.time()
        a, b = bool(self.xrt.get_A_button()), bool(self.xrt.get_B_button())
        try:
            ts_ns = int(self.xrt.get_time_stamp_ns())   # 设备样本时间戳(堵塞检测)
        except Exception:
            ts_ns = 0
        st = TeleopState(sides={}, buttons={"A": a, "B": b}, t_wall=t, ts_dev_ns=ts_ns)
        for i, side in enumerate(self.sides):
            pose7, grip, trigger = self._raw(side)
            pos, rot = xr_pose_to_world(pose7)
            st.sides[side] = SideState(pos=pos, rot=rot, grip=grip, trigger=trigger)
            if self.tap_path is not None:
                self._tap.append((t, i, pose7.copy(), grip, trigger, a, b, ts_ns))
        return st

    def save_tap(self, path=None):
        path = path or self.tap_path
        if path is None or not self._tap:
            return None
        t, si, pose, grip, trig, a, b, ts = zip(*self._tap)
        np.savez_compressed(
            path, t_wall=np.array(t), side_idx=np.array(si, dtype=np.int8),
            sides=np.array(self.sides), pose_xr=np.stack(pose),
            grip=np.array(grip), trigger=np.array(trig),
            btn_a=np.array(a), btn_b=np.array(b),
            ts_dev_ns=np.array(ts, dtype=np.int64))
        return path

    def close(self):
        p = self.save_tap()
        if p is not None:
            print(f"[pico_ultra4] raw tap saved: {p} ({len(self._tap)} rows)", flush=True)
        try:
            self.xrt.close()
        except AttributeError:
            pass
