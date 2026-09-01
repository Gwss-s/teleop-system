"""Regression: the NEW 3-layer teleop stack must reproduce the LEGACY
PicoTeleop+LiberoEEDeltaMapper (feel-verified calibration) bit-for-bit.

A scripted FakeXrt drives both stacks through the same trajectory covering:
idle -> engage (translation+rotation+trigger) -> release -> re-engage
(re-anchor) -> B/A verdict edges. Per beat we assert:
  * engagement flags equal;
  * mapped 7-dim OSC actions allclose (same float ops, same order);
  * SAVE/DISCARD events fire exactly on the legacy b/a rising edges.

No hardware, no sim, numpy-only. Run with any python that has numpy:
  PYTHONPATH=. python tests/test_teleop_equivalence.py
"""
import sys

import numpy as np

from teleop_system.legacy.pico import LiberoEEDeltaMapper, PicoTeleop
from teleop_system.backends.pico_ultra4 import PicoUltra4
from teleop_system.mapping.ee_delta import ArmConfig, EEDeltaMapper
from teleop_system.types import DISCARD, SAVE
from envs.libero.teleop_adapter import intent_to_osc


def axisangle_to_quat_xyzw(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    s = np.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0)])


class FakeXrt:
    """Scripted xrobotoolkit_sdk stand-in: set_frame() then both stacks read.
    Repeated getter calls within one beat return the same values."""

    def __init__(self):
        self.pose = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float64)  # x,y,z,qx,qy,qz,qw
        self.grip = 0.0
        self.trigger = 0.0
        self.a = False
        self.b = False

    def set_frame(self, pos, quat, grip, trigger, a=False, b=False):
        self.pose = np.concatenate([pos, quat]).astype(np.float64)
        self.grip, self.trigger, self.a, self.b = grip, trigger, a, b

    # SDK surface used by both stacks
    def get_right_controller_pose(self):
        return self.pose.copy()

    def get_right_grip(self):
        return self.grip

    def get_right_trigger(self):
        return self.trigger

    def get_A_button(self):
        return self.a

    def get_B_button(self):
        return self.b

    def close(self):
        pass


def build_script(n=80, seed=0):
    """Beats of (pos, quat, grip, trigger, a, b) exercising every transition."""
    rng = np.random.default_rng(seed)
    frames = []
    pos = np.array([0.1, -0.2, 0.9])
    ang, axis = 0.0, np.array([0.3, -0.5, 0.8])
    for t in range(n):
        if t < 5:                       # idle
            grip, trig = 0.0, 0.0
        elif t < 30:                    # engaged: move + rotate + trigger play
            grip = 0.95 + 0.04 * rng.random()
            trig = 0.8 if 12 <= t < 20 else 0.1
            pos = pos + rng.normal(0, 0.004, 3)          # ~4mm/beat hand motion
            ang += 0.02
        elif t < 36:                    # released (hold), press B once at t=32
            grip, trig = 0.2, 0.0
        elif t < 60:                    # re-engage (re-anchor), different motion
            grip, trig = 0.99, 0.6
            pos = pos + np.array([0.006, -0.002, 0.003])
            ang -= 0.015
        else:                           # idle tail, press A once at t=65
            grip, trig = 0.0, 0.0
        b = (t == 32) or (t == 33)      # held 2 beats -> exactly one edge
        a = (t == 65)
        frames.append((pos.copy(), axisangle_to_quat_xyzw(axis, ang), grip, trig, a, b))
    return frames


def main():
    xrt = FakeXrt()
    # legacy stack (untouched reference implementation)
    legacy = PicoTeleop(side="right", mapper=LiberoEEDeltaMapper(pos_scale=4.0, rot_scale=2.0),
                        xrt=xrt)
    # new 3-layer stack, same calibration values (as in configs/teleop/pico_libero.yaml)
    backend = PicoUltra4(sides=("right",), xrt=xrt)
    mapper = EEDeltaMapper(arms={"right": ArmConfig(side="right", pos_scale=4.0, rot_scale=2.0)})

    n_eng = n_act = n_save = n_discard = 0
    for t, (pos, quat, grip, trig, a, b) in enumerate(build_script()):
        xrt.set_frame(pos, quat, grip, trig, a=a, b=b)
        legacy.update()
        intent, events = mapper.step(backend.read())
        kinds = [e.kind for e in events]

        assert legacy.engaged() == mapper.engaged("right"), f"beat {t}: engagement mismatch"
        if legacy.engaged():
            n_eng += 1
            a_old = legacy.get_action()
            a_new = intent_to_osc(intent, "right")
            assert a_new is not None, f"beat {t}: new stack lost engaged intent"
            assert np.allclose(a_old, a_new, atol=1e-9), \
                f"beat {t}: action mismatch\n old={a_old}\n new={a_new}"
            n_act += 1
        else:
            assert intent_to_osc(intent, "right") is None, f"beat {t}: intent while idle"

        assert legacy.b_pressed() == (SAVE in kinds), f"beat {t}: SAVE edge mismatch"
        assert legacy.a_pressed() == (DISCARD in kinds), f"beat {t}: DISCARD edge mismatch"
        n_save += SAVE in kinds
        n_discard += DISCARD in kinds

    assert n_eng >= 40 and n_act == n_eng, "script did not exercise engagement enough"
    assert n_save == 1 and n_discard == 1, f"verdict edges wrong: save={n_save} discard={n_discard}"
    print(f"[ok] legacy == new stack over 80 beats "
          f"({n_act} engaged actions bit-matched, {n_save} save, {n_discard} discard)")


if __name__ == "__main__":
    sys.exit(main())
