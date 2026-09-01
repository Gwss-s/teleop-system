"""Layer-3 robot adapter: ControlIntent -> UR5e servoL target pose.

The ONLY place that knows UR conventions: TCP pose = [x, y, z, rx, ry, rz]
(base frame, metres, rotation vector / axis-angle in radians — the official
UR representation, same as ur_rtde getActualTCPPose / servoL).

Unlike the LIBERO adapter (per-beat OSC increments, saturation silently DROPS
over-limit motion), the UR5e path keeps a PERSISTENT servo target: per-beat
metric increments from ControlIntent integrate into the target and servoL
chases it with error retention — the official XRoboToolkit real-robot
semantics (anchored target, unexecuted distance is caught up, never lost).

Runaway protection (mandatory on a real robot, and the lesson of the 2026-08
anchored-mode incident where an unclamped persistent target got thrown metres
outside the workspace):
  1. per-beat step caps (hand-flick / tracking-jump protection);
  2. workspace box clamp on the target position;
  3. max divergence clamp: the target may never lead the MEASURED TCP pose by
     more than a configured offset (position and rotation separately).

World-frame increment composition (verbatim the official XRoboToolkit
apply_delta_pose semantics):
    pos_new = pos + dpos
    R_new   = R(drot) @ R          # LEFT-multiply = world/base-frame delta

numpy-only: ur_rtde stays in the entry file (envs/ur5e/teleop_record.py).
"""
from dataclasses import dataclass, field

import numpy as np

from teleop_system.geometry import axisangle_to_mat, mat_to_axisangle


def pose_to_pos_rot(pose6):
    """UR pose [x,y,z,rx,ry,rz] -> (pos(3,), rot(3,3))."""
    pose6 = np.asarray(pose6, dtype=np.float64)
    return pose6[0:3].copy(), axisangle_to_mat(pose6[3:6])


def pos_rot_to_pose(pos, rot):
    """(pos(3,), rot(3,3)) -> UR pose [x,y,z,rx,ry,rz]."""
    return np.concatenate([np.asarray(pos, dtype=np.float64),
                           mat_to_axisangle(np.asarray(rot, dtype=np.float64))])


@dataclass
class SafetyLimits:
    """Cartesian safety envelope for the servo target (base frame, metres/rad).

    workspace_* MUST be re-checked for every physical cell before first run;
    defaults are deliberately conservative for a table-mounted UR5e.
    """
    workspace_min: np.ndarray = field(
        default_factory=lambda: np.array([-0.40, -0.40, 0.05]))
    workspace_max: np.ndarray = field(
        default_factory=lambda: np.array([0.40, 0.40, 0.55]))
    max_lin_step: float = 0.010        # m/beat   (0.25 m/s @ 25 Hz)
    max_rot_step: float = 0.06         # rad/beat (1.5 rad/s @ 25 Hz)
    max_target_offset: float = 0.10    # m, target may lead measured TCP by this much
    max_target_rot_offset: float = 0.50  # rad, same for orientation

    @classmethod
    def from_dict(cls, d):
        """Build from the `safety:` section of configs/ur5e.yaml."""
        ws = d.get("workspace", {})
        kw = {}
        if ws:
            lo, hi = zip(*(ws[k] for k in ("x", "y", "z")))
            kw["workspace_min"], kw["workspace_max"] = np.array(lo, float), np.array(hi, float)
        for k in ("max_lin_step", "max_rot_step",
                  "max_target_offset", "max_target_rot_offset"):
            if k in d:
                kw[k] = float(d[k])
        return cls(**kw)

    def inside_workspace(self, pose6):
        p = np.asarray(pose6, dtype=np.float64)[0:3]
        return bool(np.all(p >= self.workspace_min) and np.all(p <= self.workspace_max))


def _cap_norm(v, cap):
    """Scale vector v down so ||v|| <= cap (direction preserved)."""
    n = float(np.linalg.norm(v))
    if n > cap and n > 0.0:
        return v * (cap / n)
    return v


class ServoTargetTracker:
    """Persistent servoL target with integrated safety clamps.

    Lifecycle (driven by the entry loop):
      * reset(actual)  on ENGAGE, on DISENGAGE, and after any safety trip —
        target snaps to the measured TCP pose, so the robot never chases a
        stale target while the human is not commanding it (release = hold,
        the real-robot clutch semantics);
      * step(ai, actual) once per engaged beat -> clamped target pose6.
    """

    def __init__(self, limits=None):
        self.limits = limits or SafetyLimits()
        self._pos = None
        self._rot = None

    @property
    def active(self):
        return self._pos is not None

    def reset(self, actual_pose6):
        """Anchor the target at the measured TCP pose (delta 0 by construction)."""
        self._pos, self._rot = pose_to_pos_rot(actual_pose6)

    def target_pose(self):
        assert self.active, "reset() before target_pose()"
        return pos_rot_to_pose(self._pos, self._rot)

    def step(self, ai, actual_pose6):
        """One engaged beat: ArmIntent (metric dpos/drot, already sign/gain
        mapped by layer 2) -> new clamped servo target pose6."""
        assert self.active, "reset() before step()"
        lim = self.limits
        # 1. per-beat caps (hand flick / XR tracking jump -> bounded motion)
        dpos = _cap_norm(np.asarray(ai.dpos, dtype=np.float64), lim.max_lin_step)
        drot = _cap_norm(np.asarray(ai.drot, dtype=np.float64), lim.max_rot_step)
        # 2. integrate (world-frame delta: position adds, rotation left-multiplies)
        pos = self._pos + dpos
        rot = axisangle_to_mat(drot) @ self._rot
        # 3. workspace box clamp
        pos = np.clip(pos, lim.workspace_min, lim.workspace_max)
        # 4. divergence clamp vs the MEASURED pose (anti-runaway: servoL error
        #    retention is bounded; a dropped-frame burst can never fling the
        #    target far from the physical arm)
        apos, arot = pose_to_pos_rot(actual_pose6)
        off = pos - apos
        n = float(np.linalg.norm(off))
        if n > lim.max_target_offset:
            pos = apos + off * (lim.max_target_offset / n)
        err = mat_to_axisangle(rot @ arot.T)
        ang = float(np.linalg.norm(err))
        if ang > lim.max_target_rot_offset:
            rot = axisangle_to_mat(err * (lim.max_target_rot_offset / ang)) @ arot
        self._pos, self._rot = pos, rot
        return pos_rot_to_pose(pos, rot)


def intent_to_servo_target(intent, tracker, actual_pose6, arm="right"):
    """ControlIntent -> servoL target pose6 for `arm`; None if the arm is not
    engaged this beat (caller keeps servoing the held target = hold pose)."""
    ai = intent.arms.get(arm)
    if ai is None:
        return None
    return tracker.step(ai, actual_pose6)
