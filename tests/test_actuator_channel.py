"""Unit tests for the actuator channel (自制末端执行器接口), no hardware.

Covers the whole seam: L1 spare inputs -> TeleopState.aux (gamepad + Pico,
incl. SDKs that lack the aux getters), raw-tap save/replay of aux, remote
bridge round-trip, L2 ActuatorChannel modes (hold / rate / toggle / button
pair) and yaml parsing, and the invariant that an unconfigured mapper emits
actuators=None (downstream unchanged).

Run: PYTHONPATH=. python tests/test_actuator_channel.py
"""
import os
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

import teleop_system.backends.gamepad as gp_mod
from teleop_system.backends.gamepad import GamepadBackend
from teleop_system.backends.pico_ultra4 import PicoUltra4
from teleop_system.backends.remote import _decode, _encode
from teleop_system.backends.tap_replay import TapReplay
from teleop_system.mapping.ee_delta import ActuatorChannel, ArmConfig, EEDeltaMapper
from teleop_system.types import SideState, TeleopState

# ---- fakes ---------------------------------------------------------------------

class FakeJoystick:
    def __init__(self, n_buttons=8):
        self.axes = {i: 0.0 for i in range(6)}
        self.buttons = {i: 0 for i in range(n_buttons)}
        self.hat = (0, 0)

    def init(self): pass
    def get_name(self): return "Fake Xbox 360 Controller"
    def get_axis(self, i): return self.axes[i]
    def get_button(self, i): return self.buttons[i]          # 越界 KeyError = 模拟 pygame 报错
    def get_numbuttons(self): return len(self.buttons)
    def get_numhats(self): return 1
    def get_hat(self, i): return self.hat


def make_gamepad(n_buttons=8):
    js = FakeJoystick(n_buttons)
    fake_pg = SimpleNamespace(
        JOYDEVICEREMOVED=999,
        joystick=SimpleNamespace(Joystick=lambda i: js, init=lambda: None,
                                 quit=lambda: None, get_count=lambda: 1),
        event=SimpleNamespace(get=lambda: []))
    return GamepadBackend(config=None, pygame_mod=fake_pg), js


class FakeXrtBasic:
    """只有位姿/grip/trigger/A/B 的旧 SDK 形态(等价回归测试同款)。"""
    pose = [0.0, 1.0, -0.5, 0.0, 0.0, 0.0, 1.0]
    def get_right_controller_pose(self): return self.pose
    def get_right_grip(self): return 0.0
    def get_right_trigger(self): return 0.0
    def get_A_button(self): return False
    def get_B_button(self): return False
    def get_time_stamp_ns(self): return 123
    def close(self): pass


class FakeXrtFull(FakeXrtBasic):
    """带备用输入读取的 SDK 形态。"""
    def __init__(self):
        self.axis = [0.0, 0.0]; self.click = False; self.x = False; self.y = False
    def get_right_axis(self): return self.axis
    def get_right_axis_click(self): return self.click
    def get_X_button(self): return self.x
    def get_Y_button(self): return self.y
    def get_left_grip(self): return 0.25
    def get_left_trigger(self): return 0.75
    def get_left_axis(self): return [0.1, -0.2]
    def get_left_axis_click(self): return False


def ts_with(aux, t=0.0):
    return TeleopState(sides={"right": SideState(pos=np.zeros(3), rot=np.eye(3))},
                       buttons={"A": False, "B": False}, t_wall=t, aux=aux)

# ---- L1: gamepad ---------------------------------------------------------------

def test_gamepad_aux_exports_unbound_only():
    be, js = make_gamepad(n_buttons=8)
    js.buttons[3] = 1; js.buttons[5] = 1; js.hat = (0, -1)
    aux = be.read().aux
    assert aux["y"] == 1.0 and aux["rb"] == 1.0 and aux["dpad_y"] == -1.0
    assert aux["back"] == 0.0 and aux["start"] == 0.0
    for bound in ("a", "b", "x", "lb"):                # 已绑语义的键不进 aux
        assert bound not in aux
    assert "ls_click" not in aux and "rs_click" not in aux   # 实机只有 8 键,越界项跳过


def test_gamepad_aux_follows_rebinding():
    js = FakeJoystick(11)
    fake_pg = SimpleNamespace(JOYDEVICEREMOVED=999, event=SimpleNamespace(get=lambda: []),
                              joystick=SimpleNamespace(Joystick=lambda i: js, init=lambda: None,
                                                       quit=lambda: None, get_count=lambda: 1))
    be = GamepadBackend(config={"bindings": {"gripper": "y"}}, pygame_mod=fake_pg)
    aux = be.read().aux
    assert "x" in aux and "y" not in aux               # X 被释放进 aux,Y 被占用移出
    assert "ls_click" in aux and "rs_click" in aux     # 11 键手柄: 摇杆按下可用

# ---- L1: pico ------------------------------------------------------------------

def test_pico_aux_full_sdk():
    xrt = FakeXrtFull(); xrt.axis = [0.5, -1.0]; xrt.click = True; xrt.y = True
    aux = PicoUltra4(sides=("right",), xrt=xrt).read().aux
    assert aux["right_axis_x"] == 0.5 and aux["right_axis_y"] == -1.0
    assert aux["right_axis_click"] == 1.0 and aux["y"] == 1.0 and aux["x"] == 0.0
    assert aux["left_grip"] == 0.25 and aux["left_axis_y"] == -0.2   # 左手没参与遥操 -> 导出


def test_pico_aux_left_side_in_use_not_exported():
    class Xrt(FakeXrtFull):
        def get_left_controller_pose(self): return self.pose
    aux = PicoUltra4(sides=("right", "left"), xrt=Xrt()).read().aux
    assert "left_grip" not in aux and "left_axis_x" not in aux and "right_axis_x" in aux


def test_pico_aux_old_sdk_is_empty():
    st = PicoUltra4(sides=("right",), xrt=FakeXrtBasic()).read()
    assert st.aux == {} and "right" in st.sides       # 缺函数不报错,aux 空


def test_tap_roundtrip_carries_aux():
    xrt = FakeXrtFull()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "tap.npz")
        be = PicoUltra4(sides=("right",), xrt=xrt, tap_path=path)
        xrt.axis = [0.3, 0.0]; be.read()
        xrt.axis = [0.0, 0.9]; xrt.click = True; be.read()
        be.save_tap()
        rp = TapReplay(path)
        a1 = rp.read().aux; a2 = rp.read().aux
        assert a1["right_axis_x"] == 0.3 and a1["right_axis_click"] == 0.0
        assert a2["right_axis_y"] == 0.9 and a2["right_axis_click"] == 1.0
        assert rp.read() is None


def test_old_tap_without_aux_still_replays():
    assert TapReplay("tests/data/sample_tap.npz").read().aux == {}

# ---- L1: remote bridge ---------------------------------------------------------

def test_remote_roundtrip_aux():
    st = ts_with({"y": 1.0, "dpad_y": -1.0}, t=5.0)
    back = _decode(_encode(st))
    assert back.aux == {"y": 1.0, "dpad_y": -1.0} and back.t_wall == 5.0
    old = _encode(st); old.pop("aux")                  # 旧发布端没有 aux 键
    assert _decode(old).aux == {}

# ---- L2: channels --------------------------------------------------------------

def test_channel_hold():
    ch = ActuatorChannel("y", mode="hold")
    assert ch.step({"y": 1.0}, 0.04) == 1.0 and ch.step({"y": 0.0}, 0.04) == 0.0
    assert ch.step({"y": -0.7}, 0.04) == 0.0           # 摇杆负半轴截到 0
    assert ActuatorChannel("t", "hold").step({"t": 0.4}, 0.04) == 0.4   # 模拟量直通


def test_channel_rate_and_pair():
    ch = ActuatorChannel("axis", mode="rate", speed=0.5, init=0.5)
    assert np.isclose(ch.step({"axis": 1.0}, 0.2), 0.6)          # 0.5 + 0.5*1.0*0.2
    assert np.isclose(ch.step({"axis": -1.0}, 0.4), 0.4)
    for _ in range(100): ch.step({"axis": 1.0}, 0.1)
    assert ch.value == 1.0                                       # 饱和
    pair = ActuatorChannel(["up", "down"], mode="rate", speed=1.0, init=0.0)
    assert np.isclose(pair.step({"up": 1.0, "down": 0.0}, 0.1), 0.1)
    assert np.isclose(pair.step({"up": 1.0, "down": 1.0}, 0.1), 0.1)   # 同按 = 不动
    assert np.isclose(pair.step({"down": 1.0}, 0.05), 0.05)           # 缺键当 0


def test_channel_toggle_edges():
    ch = ActuatorChannel("btn", mode="toggle", init=0.0)
    assert ch.step({"btn": 1.0}, 0.04) == 1.0          # 上升沿翻
    assert ch.step({"btn": 1.0}, 0.04) == 1.0          # 按住不再翻
    assert ch.step({"btn": 0.0}, 0.04) == 1.0
    assert ch.step({"btn": 1.0}, 0.04) == 0.0          # 再按翻回


def test_channel_rejects_bad_config():
    for bad in (dict(source="a", mode="pwm"), dict(source=["a", "b", "c"])):
        try:
            ActuatorChannel(**bad); assert False, bad
        except AssertionError as e:
            assert "actuator" in str(e)

# ---- L2: mapper ----------------------------------------------------------------

def arm():
    return {"right": ArmConfig(pos_sign=(1, 1, 1), rot_sign=(1, 1, 1))}


def test_mapper_without_actuators_emits_none():
    intent, _ = EEDeltaMapper(arms=arm()).step(ts_with({"y": 1.0}))
    assert intent.actuators is None


def test_mapper_updates_actuators_regardless_of_clutch():
    m = EEDeltaMapper(arms=arm(), actuators=[ActuatorChannel("y", "toggle"),
                                             ActuatorChannel("ax", "rate", speed=1.0, init=0.0)])
    i0, _ = m.step(ts_with({"y": 0.0, "ax": 0.0}, t=0.0))
    assert i0.actuators.shape == (2,) and "right" not in i0.arms   # 没捏 grip
    i1, _ = m.step(ts_with({"y": 1.0, "ax": 1.0}, t=0.1))
    assert i1.actuators[0] == 1.0 and np.isclose(i1.actuators[1], 0.1)
    i2, _ = m.step(ts_with({"y": 1.0, "ax": 1.0}, t=5.0))          # 长暂停: dt 截到 max_dt
    assert np.isclose(i2.actuators[1], 0.2)


def test_mapper_from_yaml_actuators():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "t.yaml")
        with open(p, "w") as f:
            f.write("device: gamepad\narms:\n  right: {side: right, pos_scale: 1.0, rot_scale: 1.0,\n"
                    "    pos_sign: [1,1,1], rot_sign: [1,1,1]}\n"
                    "actuators:\n  - {source: y, mode: toggle}\n"
                    "  - {source: [rb, back], mode: rate, speed: 0.5, init: 0.5}\n")
        m = EEDeltaMapper.from_yaml(p)
        assert len(m.actuators) == 2 and m.actuators[1].source == ("rb", "back")
        assert m.actuators_yaml[0]["source"] == "y"                # 原样保留供存盘
        intent, _ = m.step(ts_with({"y": 1.0}))
        assert intent.actuators.tolist() == [1.0, 0.5]
        m2 = EEDeltaMapper.from_yaml("configs/teleop/gamepad_ur5e.yaml")
        assert m2.actuators == [] and m2.actuators_yaml is None    # 仓库模板默认未启用


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"[ok] {t.__name__}")
    print(f"[ok] all {len(tests)} actuator channel tests passed")


if __name__ == "__main__":
    sys.exit(main())
