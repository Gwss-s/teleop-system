"""Layer-1 device backend: Xbox-class gamepad via pygame joystick.

A gamepad is a RATE device (stick deflection = velocity), not a pose device.
This backend integrates stick rates into a VIRTUAL 6-DoF hand pose and emits
a standard TeleopState — so the entire downstream stack (EEDeltaMapper clutch
deltas, calibration yaml, recording, takeover, tap replay) works unchanged,
exactly like the Pico backend.

Sources (facts verified, not guessed):
  * pygame official joystick docs: Xbox 360 layout under SDL2 = left stick
    axes 0/1, right stick 3/4, LT/RT axes 2/5, A/B/X/Y buttons 0-3, LB/RB 4/5,
    D-pad is a hat; axis range [-1,1] with drift (deadzone required); the event
    queue must be pumped for reads to update; hot-plug via JOYDEVICEADDED/
    JOYDEVICEREMOVED. (pygame.org/docs/ref/joystick.html)
  * Reading/deadzone/dual-backend patterns follow lerobot's gamepad
    teleoperator (Apache-2.0); per-model axis tables follow gym-hil's
    controller_config.json (Apache-2.0); the 6-DoF binding convention follows
    ur5_teleop_collection (MIT): LS=XY, RT/LT=Z, RS=pitch/yaw, D-pad=roll.
  * SDL2 trigger pitfall (well-known, not in pygame docs): LT/RT axes REST at
    -1.0 but read 0.0 until first touched -> normalised via (v+1)/2 with a
    per-axis "touched" guard so an untouched trigger contributes 0.

Default bindings (all overridable in configs/gamepad.yaml):
  LS = XY translate | RT/LT = Z up/down | RS = pitch/yaw | D-pad ←→ = roll
  LB (hold) = clutch/dead-man -> grip 1.0   (松手即停,真机安全语义)
  X  (hold) = gripper close   -> trigger 1.0
  B = save episode, A = discard  (与 Pico 手柄的 B/A 语义完全一致)

Heavy import (pygame) stays inside this file per the L1/L2 numpy-only rule.
"""
import os
import time

import numpy as np

from ..geometry import axisangle_to_mat
from ..types import SideState, TeleopState

# pygame SDL2 下的 Xbox 360 布局(官方文档);其余型号表可在 yaml 的 models: 里加
DEFAULT_MODELS = {
    "default": {
        "axes": {"left_x": 0, "left_y": 1, "right_x": 3, "right_y": 4, "lt": 2, "rt": 5},
        "buttons": {"a": 0, "b": 1, "x": 2, "y": 3, "lb": 4, "rb": 5},
    },
}
DEFAULT_BINDINGS = {"clutch": "lb", "gripper": "x", "save": "b", "discard": "a"}


class GamepadBackend:
    """Poll-based: call read() once per control beat -> TeleopState."""

    def __init__(self, config=None, pygame_mod=None, joystick_index=0):
        """config: dict (parsed configs/gamepad.yaml) or None for defaults;
        pygame_mod: injectable fake for tests (same pattern as PicoUltra4.xrt)."""
        cfg = config or {}
        self.deadzone = float(cfg.get("deadzone", 0.15))
        self.lin_speed = float(cfg.get("lin_speed", 0.15))   # m/s at full deflection
        self.rot_speed = float(cfg.get("rot_speed", 0.8))    # rad/s at full deflection
        self.max_dt = float(cfg.get("max_dt", 0.1))          # 防长暂停后虚拟位姿瞬移
        self.bindings = {**DEFAULT_BINDINGS, **(cfg.get("bindings") or {})}
        self._models = {**DEFAULT_MODELS, **(cfg.get("models") or {})}

        if pygame_mod is None:
            # 无窗口也能用 joystick;失焦仍收事件(pygame 文档注明的环境变量)
            os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            import pygame as pygame_mod  # noqa: PLC0415 -- heavy dep stays here
            pygame_mod.init()
            pygame_mod.joystick.init()
            if pygame_mod.joystick.get_count() <= joystick_index:
                raise ConnectionError(
                    f"未检测到手柄(joystick {joystick_index});已连数={pygame_mod.joystick.get_count()}")
        self.pg = pygame_mod
        self.js = self.pg.joystick.Joystick(joystick_index)
        try:
            self.js.init()
        except AttributeError:      # pygame 2.x 的 Joystick 自动 init
            pass
        name = self.js.get_name()
        layout = self._models.get(name, self._models["default"])
        self.axes = layout["axes"]
        self.btns = layout["buttons"]
        print(f"[gamepad] \"{name}\" 已连接: 布局={'default' if name not in self._models else name} "
              f"lin={self.lin_speed}m/s rot={self.rot_speed}rad/s deadzone={self.deadzone}", flush=True)

        # 虚拟手部位姿(绝对值无意义,L2 只消费增量;锚定发生在 ENGAGE 沿)
        self._pos = np.zeros(3)
        self._rot = np.eye(3)
        self._t_last = None
        self._trig_touched = {"lt": False, "rt": False}   # SDL2 扳机未触碰读 0.0 的守卫
        self._removed = False

    # -- raw helpers -----------------------------------------------------------
    def _axis(self, name):
        v = float(self.js.get_axis(self.axes[name]))
        return 0.0 if abs(v) < self.deadzone else v

    def _trigger(self, name):
        """LT/RT: SDL2 静止 -1.0,未触碰 0.0 -> 触碰守卫 + (v+1)/2 归一到 [0,1]。"""
        raw = float(self.js.get_axis(self.axes[name]))
        if not self._trig_touched[name]:
            if raw == 0.0:
                return 0.0
            self._trig_touched[name] = True
        return max(0.0, min(1.0, (raw + 1.0) / 2.0))

    def _button(self, name):
        return bool(self.js.get_button(self.btns[name]))

    def _hat_x(self):
        if self.js.get_numhats() < 1:
            return 0.0
        return float(self.js.get_hat(0)[0])     # D-pad 左右 = roll

    # -- layer-1 API -----------------------------------------------------------
    def read(self):
        """One beat: pump events, integrate rates -> TeleopState."""
        for ev in self.pg.event.get():           # 文档要求:必须常泵事件队列
            if ev.type == getattr(self.pg, "JOYDEVICEREMOVED", -1):
                self._removed = True
        if self._removed:
            raise ConnectionError("手柄已断开(JOYDEVICEREMOVED)")

        now = time.time()
        dt = 0.0 if self._t_last is None else min(now - self._t_last, self.max_dt)
        self._t_last = now

        # 速率(手柄系,右手系: 前推=+x, 左推=+y, RT=+z)
        v = np.array([-self._axis("left_y"),
                      -self._axis("left_x"),
                      self._trigger("rt") - self._trigger("lt")]) * self.lin_speed
        w = np.array([self._hat_x(),                    # roll  (D-pad ←→)
                      -self._axis("right_y"),           # pitch (右摇杆上下)
                      -self._axis("right_x")]) * self.rot_speed   # yaw (右摇杆左右)

        self._pos = self._pos + v * dt
        drot = w * dt
        if float(np.linalg.norm(drot)) > 0.0:
            self._rot = axisangle_to_mat(drot) @ self._rot   # 世界系增量左乘

        grip = 1.0 if self._button(self.bindings["clutch"]) else 0.0
        trigger = 1.0 if self._button(self.bindings["gripper"]) else 0.0
        return TeleopState(
            sides={"right": SideState(pos=self._pos.copy(), rot=self._rot.copy(),
                                      grip=grip, trigger=trigger)},
            buttons={"A": self._button(self.bindings["discard"]),
                     "B": self._button(self.bindings["save"])},
            t_wall=now,
            ts_dev_ns=time.time_ns())   # 本地同步轮询:采样时刻=读取时刻,永远新鲜

    def close(self):
        try:
            self.pg.joystick.quit()
        except Exception:
            pass
