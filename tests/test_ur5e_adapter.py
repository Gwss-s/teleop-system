"""Unit tests for the UR5e layer-3 adapter (numpy-only, no hardware).

Covers the safety-critical behaviours that must hold before the real robot:
  1. anchored-equivalence: integrating per-beat deltas == official
     XRoboToolkit anchored composition (ref_pose ⊕ total offset);
  2. world-frame rotation left-multiply composition;
  3. per-beat step caps (hand flick bounded);
  4. workspace box clamp (target can never leave the box);
  5. divergence clamp (target can never lead the measured TCP beyond limit);
  6. reset() re-anchors with zero jump;
  7. pose6 <-> (pos, rot) roundtrip.

Run: PYTHONPATH=. python tests/test_ur5e_adapter.py
"""
import sys

import numpy as np

from teleop_system.geometry import axisangle_to_mat, mat_to_axisangle
from teleop_system.types import ArmIntent
from envs.ur5e.teleop_adapter import (SafetyLimits, ServoTargetTracker,
                                      pos_rot_to_pose, pose_to_pos_rot)

HOME = np.array([0.10, -0.30, 0.30, 0.0, np.pi, 0.0])


def loose_limits():
    """Limits wide enough not to interfere (for pure-maths tests)."""
    return SafetyLimits(workspace_min=np.array([-5.0, -5.0, -5.0]),
                        workspace_max=np.array([5.0, 5.0, 5.0]),
                        max_lin_step=1.0, max_rot_step=1.0,
                        max_target_offset=100.0, max_target_rot_offset=100.0)


def ai(dpos=(0, 0, 0), drot=(0, 0, 0), grip=0.0):
    return ArmIntent(dpos=np.asarray(dpos, float), drot=np.asarray(drot, float),
                     gripper=grip)


def test_roundtrip():
    pos, rot = pose_to_pos_rot(HOME)
    back = pos_rot_to_pose(pos, rot)
    p2, r2 = pose_to_pos_rot(back)
    assert np.allclose(pos, p2) and np.allclose(rot, r2, atol=1e-12)


def test_anchored_equivalence():
    """Per-beat integration == anchored composition (actual tracks target)."""
    rng = np.random.default_rng(0)
    tr = ServoTargetTracker(loose_limits())
    tr.reset(HOME)
    p0, r0 = pose_to_pos_rot(HOME)
    total_dpos = np.zeros(3)
    R_total = np.eye(3)
    target = HOME
    for _ in range(200):
        dpos = rng.normal(0, 0.003, 3)
        drot = rng.normal(0, 0.01, 3)
        target = tr.step(ai(dpos, drot), target)   # actual == target (ideal servo)
        total_dpos += dpos
        R_total = axisangle_to_mat(drot) @ R_total
    tp, trot = pose_to_pos_rot(target)
    assert np.allclose(tp, p0 + total_dpos, atol=1e-9), "位置积分 != 锚定合成"
    assert np.allclose(trot, R_total @ r0, atol=1e-9), "旋转左乘链 != 总左乘"


def test_rotation_left_multiply():
    tr = ServoTargetTracker(loose_limits())
    tr.reset(HOME)
    _, r0 = pose_to_pos_rot(HOME)
    d1, d2 = np.array([0.1, 0, 0]), np.array([0, 0.2, 0])
    t1 = tr.step(ai(drot=d1), HOME)
    t2 = tr.step(ai(drot=d2), t1)
    _, rot = pose_to_pos_rot(t2)
    expect = axisangle_to_mat(d2) @ axisangle_to_mat(d1) @ r0
    assert np.allclose(rot, expect, atol=1e-12), "世界系增量必须左乘"


def test_per_beat_caps():
    lim = loose_limits()
    lim.max_lin_step, lim.max_rot_step = 0.01, 0.05
    tr = ServoTargetTracker(lim)
    tr.reset(HOME)
    t = tr.step(ai(dpos=(1.0, 1.0, 0), drot=(0, 0, 2.0)), HOME)   # 手甩动
    assert np.linalg.norm(t[:3] - HOME[:3]) <= 0.01 + 1e-12
    _, rot = pose_to_pos_rot(t)
    _, r0 = pose_to_pos_rot(HOME)
    ang = np.linalg.norm(mat_to_axisangle(rot @ r0.T))
    assert ang <= 0.05 + 1e-12


def test_workspace_clamp():
    lim = loose_limits()
    lim.workspace_min = np.array([-0.2, -0.4, 0.05])
    lim.workspace_max = np.array([0.2, -0.1, 0.5])
    lim.max_target_offset = 100.0
    tr = ServoTargetTracker(lim)
    start = np.array([0.19, -0.30, 0.30, 0.0, np.pi, 0.0])
    tr.reset(start)
    t = start
    for _ in range(50):                       # 持续往 +x 推,必须停在盒边
        t = tr.step(ai(dpos=(0.05, 0, 0)), t)
    assert t[0] <= 0.2 + 1e-12, f"目标越出工作空间盒: x={t[0]}"
    assert lim.inside_workspace(t)


def test_divergence_clamp():
    lim = loose_limits()
    lim.max_target_offset = 0.05
    tr = ServoTargetTracker(lim)
    tr.reset(HOME)
    actual = HOME                              # 机器人卡住不动(极端情形)
    t = HOME
    for _ in range(100):
        t = tr.step(ai(dpos=(0.01, 0, 0)), actual)
    div = np.linalg.norm(t[:3] - actual[:3])
    assert div <= 0.05 + 1e-12, f"目标领先实测超限: {div}"
    # 旋转同理
    lim.max_target_rot_offset = 0.2
    tr.reset(HOME)
    for _ in range(100):
        t = tr.step(ai(drot=(0, 0, 0.05)), actual)
    _, trot = pose_to_pos_rot(t)
    _, arot = pose_to_pos_rot(actual)
    ang = np.linalg.norm(mat_to_axisangle(trot @ arot.T))
    assert ang <= 0.2 + 1e-12, f"目标姿态领先超限: {ang}"


def test_reset_no_jump():
    tr = ServoTargetTracker(loose_limits())
    tr.reset(HOME)
    tr.step(ai(dpos=(0.01, 0.01, 0)), HOME)
    somewhere = np.array([0.0, -0.25, 0.35, 0.1, np.pi - 0.1, 0.05])
    tr.reset(somewhere)                        # 脱开/重捏: 钉在实测位姿
    t = tr.step(ai(), somewhere)               # 首拍零增量
    assert np.allclose(t, somewhere, atol=1e-9), "重锚定后首拍必须零跳变"


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"[ok] {t.__name__}")
    print(f"[ok] all {len(tests)} UR5e adapter tests passed")


if __name__ == "__main__":
    sys.exit(main())
