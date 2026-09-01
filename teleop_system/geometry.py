"""SO(3) helpers shared by teleop backends and mapping (numpy-only).

The XR->world matrix and both conversions are copied verbatim from the
calibrated implementation in legacy/pico.py (kept as frozen reference);
the equivalence regression test (tests/test_teleop_equivalence.py) pins the
new stack to the legacy maths bit-for-bit.
"""
import numpy as np

# XR (OpenXR, Y-up) -> world (Z-up); exact matrix from the official
# XRoboToolkit sample (xrobotoolkit_teleop/utils/geometry.py R_HEADSET_TO_WORLD).
R_HEADSET_TO_WORLD = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def quat_xyzw_to_mat(q):
    """(x,y,z,w) quaternion -> 3x3 rotation matrix (normalised).
    Zero/garbage quaternion (SDK before first packet) -> identity."""
    n = np.linalg.norm(q)
    if n < 1e-8:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_axisangle(R):
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


def axisangle_to_mat(v):
    """Axis-angle rotation vector (rad) -> 3x3 rotation matrix (Rodrigues)."""
    v = np.asarray(v, dtype=np.float64)
    angle = float(np.linalg.norm(v))
    if angle < 1e-10:
        return np.eye(3)
    k = v / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def xr_pose_to_world(pose7):
    """Raw XR pose [x,y,z,qx,qy,qz,qw] -> (pos(3,), rot(3,3)) canonical world.
    Single source of truth — used by live backends AND tap replay, so a
    replayed stream is guaranteed to transform identically to the live one."""
    pose7 = np.asarray(pose7, dtype=np.float64)
    pos = R_HEADSET_TO_WORLD @ pose7[0:3]
    rot = R_HEADSET_TO_WORLD @ quat_xyzw_to_mat(pose7[3:7]) @ R_HEADSET_TO_WORLD.T
    return pos, rot
