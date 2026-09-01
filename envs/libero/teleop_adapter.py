"""Layer-3 setting adapter: ControlIntent -> LIBERO/robosuite OSC_POSE action.

The ONLY place that knows LIBERO's action convention (robosuite osc_pose.json
+ lerobot libero env): action = [dx,dy,dz, drx,dry,drz, grip], each in [-1,1];
1.0 == 0.05 m translation / 0.5 rad axis-angle rotation per step (20 Hz);
grip +1 close / -1 open (analog trigger thresholded at 0.5).

Shared by LIBERO-Plus and robomimic (same robosuite OSC bridge) — write once,
serve both benches. Upstream layers (teleop/backends, teleop/mapping) never
see these constants.
"""
import numpy as np



TRANS_PER_UNIT = 0.05    # m,   osc_pose.json output_max
ROT_PER_UNIT = 0.5       # rad, osc_pose.json output_max
GRIP_CLOSE_THRESHOLD = 0.5


def intent_to_osc(intent, arm="right", trans_per_unit=TRANS_PER_UNIT,
                  rot_per_unit=ROT_PER_UNIT, grip_threshold=GRIP_CLOSE_THRESHOLD):
    """ControlIntent -> OSC_POSE action (7,) float32 for `arm`;
    None if the arm is not engaged this beat (caller decides fallback).
    Defaults = LIBERO; robomimic (same robosuite OSC bridge) reuses this
    function, overriding per-unit constants only if its controller config
    differs from osc_pose.json defaults."""
    ai = intent.arms.get(arm)
    if ai is None:
        return None
    a = np.zeros(7, dtype=np.float32)
    a[0:3] = np.clip(ai.dpos / trans_per_unit, -1, 1)
    a[3:6] = np.clip(ai.drot / rot_per_unit, -1, 1)
    a[6] = 1.0 if ai.gripper > grip_threshold else -1.0
    return a

