"""Pico VR teleop backend via XRoboToolkit (pybind module `xrobotoolkit_sdk`).

Implements the Teleop ABC with TAKEOVER semantics:
  engaged()    -- analog grip held past threshold (0.9, same as the official
                  XRoboToolkit-Teleop-Sample-Python BaseTeleopController);
  get_action() -- env-convention action from clutched controller deltas.

Design follows the official sample (base_teleop_controller.py) closely:
  * clutch: on engage, anchor at the current controller pose; on release,
    clear it -- re-engaging re-anchors (index/re-center, their scheme);
  * frame transform: XR (OpenXR, Y-up) -> sim world (Z-up) with their exact
    matrix R_HEADSET_TO_WORLD = [[0,0,-1],[-1,0,0],[0,1,0]]; positions p'=R@p,
    orientations R' = R @ R_ctrl @ R^T (conjugation).

Unlike the sample (absolute IK targets), the default mapper here emits
*per-step delta* actions, matching robosuite/LIBERO OSC_POSE "relative" mode
(the convention of robosuite's own Keyboard/SpaceMouse devices, see
robosuite/utils/input_utils.py input2action).

Runtime deps: `xrobotoolkit_sdk` importable; XRoboToolkit PC Service running;
headset app connected with Controller tracking + Send enabled.
"""
import numpy as np

from .base import Teleop

# XR (Y-up) -> world (Z-up); exact matrix from the official sample
# (xrobotoolkit_teleop/utils/geometry.py R_HEADSET_TO_WORLD).
R_HEADSET_TO_WORLD = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

GRIP_ENGAGE_THRESHOLD = 0.9   # official sample: active = grip > 0.9


def _quat_xyzw_to_mat(q):
    """(x,y,z,w) quaternion -> 3x3 rotation matrix (normalised)."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_axisangle(R):
    """3x3 rotation matrix -> axis-angle rotation vector (rad)."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cos))
    if angle < 1e-8:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-10:            # angle ~ pi: recover axis from the diagonal
        axis = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None))
        axis[axis.argmax()] = max(axis.max(), 1e-6)
        return axis / np.linalg.norm(axis) * angle
    return axis / n * angle


class LiberoEEDeltaMapper:
    """World-frame per-step deltas -> LIBERO OSC_POSE relative action (7,).

    LIBERO conventions (robosuite osc_pose.json + lerobot libero env):
      action = [dx,dy,dz, drx,dry,drz, grip], each in [-1,1];
      1.0 == 0.05 m translation / 0.5 rad axis-angle rotation per step (20 Hz);
      grip: +1 close, -1 open (trigger analog thresholded at 0.5).
    """
    TRANS_PER_UNIT = 0.05   # m,  osc_pose.json output_max
    ROT_PER_UNIT = 0.5      # rad, osc_pose.json output_max

    def __init__(self, pos_scale=4.0, rot_scale=2.0,
                 pos_sign=(-1.0, 1.0, 1.0), rot_sign=(1.0, -1.0, -1.0)):
        # Empirically calibrated on LIBERO (agentview camera faces the robot).
        # Position is mirrored through the y-z plane (flip world x = forward/back);
        # pos_sign=(-1,1,1). For a consistent mirror, the axis-angle rotation delta
        # (a pseudovector) transforms as (wx,wy,wz)->(wx,-wy,-wz), i.e.
        # rot_sign=(1,-1,-1) -- matches teleop feel: left/right-turn (x) unchanged,
        # pitch (y) and EE-Z roll (z) flipped. All 6 axes verified against Panda EE.
        # pos 1.0==0.05m/step, rot 1.0==0.5rad/step (hard 1 m/s cap).
        self.pos_scale, self.rot_scale = pos_scale, rot_scale
        self.pos_sign = np.asarray(pos_sign, dtype=np.float64)
        self.rot_sign = np.asarray(rot_sign, dtype=np.float64)

    def __call__(self, dpos_w, drot_w, trigger):
        a = np.zeros(7, dtype=np.float32)
        a[0:3] = np.clip(dpos_w * self.pos_sign * self.pos_scale / self.TRANS_PER_UNIT, -1, 1)
        a[3:6] = np.clip(drot_w * self.rot_sign * self.rot_scale / self.ROT_PER_UNIT, -1, 1)
        a[6] = 1.0 if trigger > 0.5 else -1.0
        return a


class PicoTeleop(Teleop):
    """One-controller takeover teleop (default: right controller).

    Call update() once per control beat; get_action() then returns the
    env-convention action for that beat (world-frame controller displacement
    since the previous beat, i.e. per-step delta), or None if not engaged.
    """

    def __init__(self, side="right", mapper=None, xrt=None):
        if xrt is None:
            import xrobotoolkit_sdk as xrt  # noqa: PLC0415 -- optional dep
            xrt.init()
        self.xrt = xrt
        self.side = side
        self.mapper = mapper or LiberoEEDeltaMapper()
        self._engaged = False
        self._prev_pos = None    # world frame, previous beat
        self._prev_rot = None
        self._dpos = np.zeros(3)
        self._drot = np.zeros(3)
        self._trigger = 0.0
        self._prev_b = False
        self._b_edge = False
        self._prev_a = False
        self._a_edge = False

    # -- raw reads -----------------------------------------------------------
    def _pose_world(self):
        p = (self.xrt.get_right_controller_pose() if self.side == "right"
             else self.xrt.get_left_controller_pose())
        pos = R_HEADSET_TO_WORLD @ np.asarray(p[0:3], dtype=np.float64)
        R_ctrl = _quat_xyzw_to_mat(np.asarray(p[3:7], dtype=np.float64))
        rot = R_HEADSET_TO_WORLD @ R_ctrl @ R_HEADSET_TO_WORLD.T
        return pos, rot

    def _grip(self):
        return (self.xrt.get_right_grip() if self.side == "right"
                else self.xrt.get_left_grip())

    def _trig(self):
        return (self.xrt.get_right_trigger() if self.side == "right"
                else self.xrt.get_left_trigger())

    # -- Teleop interface ------------------------------------------------------
    def update(self):
        engaged_now = self._grip() > GRIP_ENGAGE_THRESHOLD
        # A/B rising edges (episode verdict markers, official sample pattern)
        b = bool(self.xrt.get_B_button())
        a = bool(self.xrt.get_A_button())
        self._b_edge = b and not self._prev_b
        self._a_edge = a and not self._prev_a
        self._prev_b, self._prev_a = b, a

        if engaged_now:
            pos, rot = self._pose_world()
            if not self._engaged or self._prev_pos is None:
                # clutch-in: anchor = current pose -> first delta is zero
                self._dpos, self._drot = np.zeros(3), np.zeros(3)
            else:
                self._dpos = pos - self._prev_pos
                self._drot = _mat_to_axisangle(rot @ self._prev_rot.T)
            self._prev_pos, self._prev_rot = pos, rot
            self._trigger = self._trig()
        else:
            self._prev_pos = self._prev_rot = None
            self._dpos, self._drot = np.zeros(3), np.zeros(3)
        self._engaged = engaged_now

    def engaged(self):
        return self._engaged

    def get_action(self):
        if not self._engaged:
            return None
        return self.mapper(self._dpos, self._drot, self._trigger)

    # extras (not part of the ABC)
    def b_pressed(self):
        """Rising edge of B since last update() -- e.g. episode save."""
        return self._b_edge

    def a_pressed(self):
        """Rising edge of A since last update() -- e.g. episode discard."""
        return self._a_edge

    def close(self):
        self.xrt.close()
