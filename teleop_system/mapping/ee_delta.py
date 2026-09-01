"""Layer-2 semantic mapping: TeleopState -> ControlIntent + TakeoverEvent.

Device-agnostic and env-agnostic. This file is the ONE place where takeover
semantics is defined (grip threshold -> ENGAGE/DISENGAGE, verdict-button
rising edges -> SAVE/DISCARD); every client and downstream consumer keys off these
events — no other engagement notion may exist in the framework.

Per-arm clutched EE-delta mapping (the feel-calibrated scheme, moved here
verbatim from legacy/pico.py LiberoEEDeltaMapper + PicoTeleop):
  * clutch-in (grip crosses threshold): anchor at current pose, first delta 0;
  * while engaged: dpos = pos - prev, drot = axisangle(rot @ prev.T), then
    per-axis sign flip (mirror calibration) and gain -> METRIC ArmIntent;
  * clutch-out: clear anchor; re-engaging re-anchors (no jump by construction
    — increments never reference an absolute target, so takeover/handback can
    never cause an EE jump for VR increments).

Config is yaml-driven (configs/teleop/*.yaml): axis signs / gains / engage
threshold / verdict buttons / side->arm assignment. numpy-only + pyyaml.
"""
import numpy as np

from ..geometry import mat_to_axisangle
from ..types import DISCARD, DISENGAGE, ENGAGE, SAVE, ArmIntent, ControlIntent, TakeoverEvent


class ArmConfig:
    """Mapping parameters for one arm (mutable — live gain tuning in teleop_test)."""

    def __init__(self, side="right", pos_scale=4.0, rot_scale=2.0,
                 pos_sign=(-1.0, 1.0, 1.0), rot_sign=(1.0, -1.0, -1.0),
                 world_yaw_deg=0.0):
        self.side = side
        self.pos_scale, self.rot_scale = float(pos_scale), float(rot_scale)
        self.pos_sign = np.asarray(pos_sign, dtype=np.float64)
        self.rot_sign = np.asarray(rot_sign, dtype=np.float64)
        # 操作员站位对齐: 头显世界系 -> 机器人基座系的水平偏航(度)。
        # 头显世界系朝向 = app 启动时头的朝向, 与基座轴不对齐时逐轴符号修不了
        # 45° 斜移——先绕 z 转正, 再做逐轴符号×增益。0 = 恒等(等价回归不受影响)。
        self.world_yaw_deg = float(world_yaw_deg)
        c = np.cos(np.radians(self.world_yaw_deg))
        s = np.sin(np.radians(self.world_yaw_deg))
        self.world_rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class EEDeltaMapper:
    """Poll-based: call step(TeleopState) once per beat."""

    def __init__(self, arms=None, engage_threshold=0.9, disengage_threshold=None,
                 save_button="B", discard_button="A"):
        # arms: {arm_name: ArmConfig}; default = single right arm, calibrated values
        # disengage_threshold < engage_threshold gives hysteresis (anti-jitter:
        # grip noise around the threshold would otherwise storm rebase/replan).
        # None = no hysteresis (legacy-equivalent).
        self.arms = arms or {"right": ArmConfig()}
        self.engage_threshold = float(engage_threshold)
        self.disengage_threshold = (self.engage_threshold if disengage_threshold is None
                                    else float(disengage_threshold))
        self.save_button, self.discard_button = save_button, discard_button
        self._engaged = {a: False for a in self.arms}
        self._prev = {a: None for a in self.arms}      # (pos, rot) anchor/prev beat
        self._btn_prev = {}

    @classmethod
    def from_yaml(cls, path, pos_scale=None, rot_scale=None):
        """Build from a configs/teleop/*.yaml; optional CLI gain overrides
        (None = keep yaml value) apply to every arm."""
        import yaml  # noqa: PLC0415 -- config-time only
        with open(path) as f:
            cfg = yaml.safe_load(f)
        arms = {}
        for name, a in cfg["arms"].items():
            arms[name] = ArmConfig(
                side=a.get("side", name),
                pos_scale=pos_scale if pos_scale is not None else a.get("pos_scale", 4.0),
                rot_scale=rot_scale if rot_scale is not None else a.get("rot_scale", 2.0),
                pos_sign=a.get("pos_sign", (-1.0, 1.0, 1.0)),
                rot_sign=a.get("rot_sign", (1.0, -1.0, -1.0)),
                world_yaw_deg=a.get("world_yaw_deg", 0.0))
        return cls(arms=arms,
                   engage_threshold=cfg.get("engage_threshold", 0.9),
                   disengage_threshold=cfg.get("disengage_threshold"),
                   save_button=cfg.get("save_button", "B"),
                   discard_button=cfg.get("discard_button", "A"))

    def engaged(self, arm=None):
        """True while the human has taken over (any arm, or a specific one)."""
        if arm is not None:
            return self._engaged[arm]
        return any(self._engaged.values())

    def _button_edge(self, buttons, name):
        cur = bool(buttons.get(name, False))
        edge = cur and not self._btn_prev.get(name, False)
        self._btn_prev[name] = cur
        return edge

    def step(self, ts):
        """One beat: TeleopState -> (ControlIntent, [TakeoverEvent])."""
        intent = ControlIntent(arms={}, t_wall=ts.t_wall)
        events = []
        for arm, cfg in self.arms.items():
            s = ts.sides.get(cfg.side)
            thr = self.disengage_threshold if self._engaged[arm] else self.engage_threshold
            engaged_now = s is not None and s.grip > thr
            if engaged_now:
                if not self._engaged[arm] or self._prev[arm] is None:
                    dpos, drot = np.zeros(3), np.zeros(3)   # clutch-in: anchor
                    events.append(TakeoverEvent(ENGAGE, arm, ts.t_wall))
                else:
                    ppos, prot = self._prev[arm]
                    dpos = s.pos - ppos
                    drot = mat_to_axisangle(s.rot @ prot.T)
                    if cfg.world_yaw_deg != 0.0:   # 站位对齐(0 时跳过,保逐位等价)
                        dpos = cfg.world_rot @ dpos
                        drot = cfg.world_rot @ drot
                self._prev[arm] = (s.pos, s.rot)
                intent.arms[arm] = ArmIntent(
                    dpos=dpos * cfg.pos_sign * cfg.pos_scale,
                    drot=drot * cfg.rot_sign * cfg.rot_scale,
                    gripper=s.trigger)
            else:
                if self._engaged[arm]:
                    events.append(TakeoverEvent(DISENGAGE, arm, ts.t_wall))
                self._prev[arm] = None
            self._engaged[arm] = engaged_now
        if self._button_edge(ts.buttons, self.save_button):
            events.append(TakeoverEvent(SAVE, "", ts.t_wall))
        if self._button_edge(ts.buttons, self.discard_button):
            events.append(TakeoverEvent(DISCARD, "", ts.t_wall))
        return intent, events
