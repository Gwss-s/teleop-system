"""Layer-1 device backend: Pico Ultra 4 via XRoboToolkit (`xrobotoolkit_sdk`).

Only knows the device. Reads controller poses/analogs/buttons, transforms
XR (Y-up) -> canonical world (Z-up), and emits a TeleopState. Knows NOTHING
about robots, clutches, scaling, or engagement — that is the mapping layer.

Spare inputs -> TeleopState.aux (备用输入,标准映射不消费,留给自制末端执行器):
  right_axis_x / right_axis_y   右手柄摇杆 [-1,1]      right_axis_click  摇杆按下 0/1
  x / y                         左手柄 X/Y 键 0/1(A/B 在右手柄,已用于裁决)
  left_grip / left_trigger      左手柄模拟量 [0,1]     left_axis_x / left_axis_y / left_axis_click
  (left_* 只在左手没有作为遥操手参与时导出;SDK 缺某个读取函数时该项省略)

Raw-stream tap: every read() can be recorded verbatim (raw XR
pose + analogs + aux, BEFORE any mapping), so a collection session can later
be re-mapped with different yaml configs without re-teleoperating. Enable by
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
        self._tap = []          # rows: (t_wall, side_idx, pose7, grip, trigger, A, B, ts_ns)
        self._tap_aux = []      # 与 _tap 逐行对应的 aux dict
        # 备用输入读取表: aux 名 -> (SDK 函数名, 取值方式);缺的函数启动时剔除
        readers = {
            "right_axis_x": ("get_right_axis", 0), "right_axis_y": ("get_right_axis", 1),
            "right_axis_click": ("get_right_axis_click", None),
            "x": ("get_X_button", None), "y": ("get_Y_button", None),
        }
        if "left" not in self.sides:
            readers.update({
                "left_grip": ("get_left_grip", None), "left_trigger": ("get_left_trigger", None),
                "left_axis_x": ("get_left_axis", 0), "left_axis_y": ("get_left_axis", 1),
                "left_axis_click": ("get_left_axis_click", None),
            })
        self._aux_readers = {k: (getattr(xrt, fn), idx) for k, (fn, idx) in readers.items()
                             if callable(getattr(xrt, fn, None))}

    # -- raw SDK reads ---------------------------------------------------------
    def _raw(self, side):
        x = self.xrt
        if side == "right":
            return (np.asarray(x.get_right_controller_pose(), dtype=np.float64),
                    float(x.get_right_grip()), float(x.get_right_trigger()))
        return (np.asarray(x.get_left_controller_pose(), dtype=np.float64),
                float(x.get_left_grip()), float(x.get_left_trigger()))

    def _aux(self):
        aux = {}
        for name, (fn, idx) in self._aux_readers.items():
            try:
                v = fn()
                aux[name] = float(v[idx] if idx is not None else v)
            except Exception:
                continue
        return aux

    # -- layer-1 API -----------------------------------------------------------
    def read(self):
        """One beat: raw device state -> TeleopState (canonical world frame)."""
        t = time.time()
        a, b = bool(self.xrt.get_A_button()), bool(self.xrt.get_B_button())
        try:
            ts_ns = int(self.xrt.get_time_stamp_ns())   # 设备样本时间戳(堵塞检测)
        except Exception:
            ts_ns = 0
        aux = self._aux()
        st = TeleopState(sides={}, buttons={"A": a, "B": b}, t_wall=t, ts_dev_ns=ts_ns, aux=aux)
        for i, side in enumerate(self.sides):
            pose7, grip, trigger = self._raw(side)
            pos, rot = xr_pose_to_world(pose7)
            st.sides[side] = SideState(pos=pos, rot=rot, grip=grip, trigger=trigger)
            if self.tap_path is not None:
                self._tap.append((t, i, pose7.copy(), grip, trigger, a, b, ts_ns))
                self._tap_aux.append(aux)
        return st

    def save_tap(self, path=None):
        path = path or self.tap_path
        if path is None or not self._tap:
            return None
        t, si, pose, grip, trig, a, b, ts = zip(*self._tap)
        data = dict(
            t_wall=np.array(t), side_idx=np.array(si, dtype=np.int8),
            sides=np.array(self.sides), pose_xr=np.stack(pose),
            grip=np.array(grip), trigger=np.array(trig),
            btn_a=np.array(a), btn_b=np.array(b),
            ts_dev_ns=np.array(ts, dtype=np.int64))
        names = sorted({k for row in self._tap_aux for k in row})
        if names:            # 备用输入按列存,缺失补 0;旧 tap 无此键 -> 重放 aux 为空
            data["aux_names"] = np.array(names)
            data["aux"] = np.array([[row.get(k, 0.0) for k in names] for row in self._tap_aux])
        np.savez_compressed(path, **data)
        return path

    def close(self):
        p = self.save_tap()
        if p is not None:
            print(f"[pico_ultra4] raw tap saved: {p} ({len(self._tap)} rows)", flush=True)
        try:
            self.xrt.close()
        except AttributeError:
            pass
