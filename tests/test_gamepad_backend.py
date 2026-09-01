"""Unit tests for the gamepad L1 backend (no hardware: injected FakePygame).

Covers: deadzone, stick->velocity integration, SDL2 trigger touched-guard,
clutch/gripper/verdict buttons, world-frame rotation left-multiply, max_dt
clamp, hot-unplug raising, and the FULL pipeline gamepad -> EEDeltaMapper ->
intent_to_osc (proves L2/L3 need zero changes for the new device).

Run: PYTHONPATH=. python tests/test_gamepad_backend.py
"""
import sys
from types import SimpleNamespace

import numpy as np

import teleop_system.backends.gamepad as gp_mod
from teleop_system.backends.gamepad import GamepadBackend
from teleop_system.geometry import axisangle_to_mat
from teleop_system.mapping.ee_delta import ArmConfig, EEDeltaMapper
from teleop_system.types import ENGAGE, SAVE
from envs.libero.teleop_adapter import intent_to_osc

JOYDEVICEREMOVED = 999


class FakeJoystick:
    def __init__(self):
        self.axes = {i: 0.0 for i in range(6)}
        self.buttons = {i: 0 for i in range(8)}
        self.hat = (0, 0)

    def init(self):
        pass

    def get_name(self):
        return "Fake Xbox 360 Controller"

    def get_axis(self, i):
        return self.axes[i]

    def get_button(self, i):
        return self.buttons[i]

    def get_numhats(self):
        return 1

    def get_hat(self, i):
        return self.hat


class FakeClock:
    """可控时钟:替换模块内 time,让积分 dt 完全确定。"""

    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def time_ns(self):
        return int(self.t * 1e9)


def make(clock=None, **cfg):
    js = FakeJoystick()
    events = []
    fake_pg = SimpleNamespace(
        JOYDEVICEREMOVED=JOYDEVICEREMOVED,
        joystick=SimpleNamespace(Joystick=lambda i: js, init=lambda: None,
                                 quit=lambda: None, get_count=lambda: 1),
        event=SimpleNamespace(get=lambda: [events.pop(0)] if events else []),
    )
    be = GamepadBackend(config=cfg or None, pygame_mod=fake_pg)
    if clock is not None:
        gp_mod.time = clock          # 模块级时钟注入
    return be, js, events


def teardown():
    import time
    gp_mod.time = time               # 还原真实时钟


def test_idle_and_deadzone():
    clock = FakeClock()
    be, js, _ = make(clock)
    s0 = be.read()
    clock.t += 0.05
    js.axes[1] = -0.1                # 低于死区 0.15
    s1 = be.read()
    assert np.allclose(s0.sides["right"].pos, s1.sides["right"].pos)
    assert s1.sides["right"].grip == 0.0 and s1.sides["right"].trigger == 0.0
    teardown()


def test_stick_integration():
    clock = FakeClock()
    be, js, _ = make(clock, lin_speed=0.2)
    be.read()                        # 首拍 dt=0
    js.axes[1] = -1.0                # 左摇杆满前推 -> +x
    clock.t += 0.04
    s = be.read()
    assert np.allclose(s.sides["right"].pos, [0.2 * 0.04, 0, 0], atol=1e-12), \
        f"got {s.sides['right'].pos}"
    teardown()


def test_trigger_touched_guard():
    clock = FakeClock()
    be, js, _ = make(clock, lin_speed=0.2)
    be.read()
    clock.t += 0.04                  # RT 原始 0.0(未触碰)必须不产生 z 运动
    s = be.read()
    assert s.sides["right"].pos[2] == 0.0
    js.axes[5] = 1.0                 # RT 满按 -> +z 满速
    clock.t += 0.04
    s = be.read()
    assert np.isclose(s.sides["right"].pos[2], 0.2 * 0.04)
    js.axes[5] = -1.0                # 已触碰后的静止值 -1.0 -> 归一 0
    clock.t += 0.04
    z_prev = s.sides["right"].pos[2]
    s = be.read()
    assert np.isclose(s.sides["right"].pos[2], z_prev)
    teardown()


def test_buttons_and_verdicts():
    clock = FakeClock()
    be, js, _ = make(clock)
    js.buttons[4] = 1                # LB = clutch
    js.buttons[2] = 1                # X  = gripper
    js.buttons[1] = 1                # B  = save
    js.buttons[0] = 1                # A  = discard
    s = be.read()
    assert s.sides["right"].grip == 1.0 and s.sides["right"].trigger == 1.0
    assert s.buttons["B"] and s.buttons["A"]
    assert s.ts_dev_ns > 0           # 本地设备必须带时间戳(看门狗走输入龄)
    teardown()


def test_rotation_left_multiply():
    clock = FakeClock()
    be, js, _ = make(clock, rot_speed=1.0)
    be.read()
    js.axes[3] = -1.0                # 右摇杆左满 -> +yaw
    clock.t += 0.05
    r1 = be.read().sides["right"].rot
    js.axes[3] = 0.0
    js.hat = (1, 0)                  # D-pad 右 -> +roll
    clock.t += 0.05
    r2 = be.read().sides["right"].rot
    expect = axisangle_to_mat([0.05, 0, 0]) @ axisangle_to_mat([0, 0, 0.05])
    assert np.allclose(r2, expect, atol=1e-12), "世界系增量必须左乘"
    assert np.allclose(r1, axisangle_to_mat([0, 0, 0.05]), atol=1e-12)
    teardown()


def test_max_dt_clamp():
    clock = FakeClock()
    be, js, _ = make(clock, lin_speed=0.2, max_dt=0.1)
    be.read()
    js.axes[1] = -1.0
    clock.t += 5.0                   # 长暂停:位移必须被钳到 max_dt
    s = be.read()
    assert np.isclose(s.sides["right"].pos[0], 0.2 * 0.1)
    teardown()


def test_unplug_raises():
    clock = FakeClock()
    be, _, events = make(clock)
    events.append(SimpleNamespace(type=JOYDEVICEREMOVED))
    try:
        be.read()
        raise AssertionError("拔线必须抛异常(入口看门狗依赖它)")
    except ConnectionError:
        pass
    teardown()


def test_full_pipeline_to_osc():
    """gamepad -> EEDeltaMapper(identity 标定) -> intent_to_osc,零改动全链路。"""
    clock = FakeClock()
    be, js, _ = make(clock, lin_speed=0.2)
    mapper = EEDeltaMapper(arms={"right": ArmConfig(side="right", pos_scale=1.0, rot_scale=1.0,
                                                    pos_sign=(1, 1, 1), rot_sign=(1, 1, 1))})
    be.read()
    js.buttons[4] = 1                                  # 捏合离合
    intent, events = mapper.step(be.read())
    assert ENGAGE in [e.kind for e in events]
    assert intent_to_osc(intent, "right") is not None  # 锚定拍,零增量
    js.axes[1] = -1.0                                  # 满速前推 0.05s
    clock.t += 0.05
    intent, _ = mapper.step(be.read())
    a = intent_to_osc(intent, "right")                 # 0.2*0.05=0.01m -> OSC 0.01/0.05=0.2
    assert np.isclose(a[0], 0.2, atol=1e-6), f"OSC x={a[0]}"
    assert a[6] == -1.0                                # 夹爪未按 -> 开
    js.buttons[1] = 1                                  # B 沿 -> SAVE
    clock.t += 0.05
    _, events = mapper.step(be.read())
    assert SAVE in [e.kind for e in events]
    teardown()


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"[ok] {t.__name__}")
    print(f"[ok] all {len(tests)} gamepad backend tests passed")


if __name__ == "__main__":
    sys.exit(main())
