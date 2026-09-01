"""Canonical teleop message types — the seams of the 3-layer pipeline.

Design goal: N devices x M robots must cost N + M files, not N x M.
These three types ARE the seam:

    device backend --TeleopState--> mapping --ControlIntent + TakeoverEvent--> env adapter
    (teleop/backends/)              (teleop/mapping/)         (envs/<bench>/)

Hard constraint (deploy targets may lack heavy deps): layers 1-2 may
depend on nothing heavier than numpy (+ yaml for configs). Heavy imports live
only inside concrete backend files.

Conventions:
  * canonical world frame: Z-up, metres, rotation matrices (3,3);
  * TeleopState carries RAW button levels (no edge detection) — edges are the
    mapping layer's job (single place where takeover semantics is defined);
  * ControlIntent is METRIC (m / rad per beat) and env-agnostic — converting
    to a specific action space (OSC [-1,1], joint targets, ...) is layer 3.
"""
from dataclasses import dataclass, field

import numpy as np

# TakeoverEvent kinds — the ONLY takeover vocabulary the framework knows.
# Keyboard z/x, VR grip, physically grabbing a leader arm all normalise to these.
ENGAGE = "engage"        # human takes over (this arm)
DISENGAGE = "disengage"  # human hands back (this arm)
SAVE = "save"            # episode verdict: keep     (global, arm="")
DISCARD = "discard"      # episode verdict: void     (global, arm="")


@dataclass
class SideState:
    """One controller / hand / leader-arm, in the canonical world frame."""
    pos: np.ndarray          # (3,) position, metres
    rot: np.ndarray          # (3,3) rotation matrix
    grip: float = 0.0        # analog grip  [0,1]  (engagement source)
    trigger: float = 0.0     # analog trigger [0,1] (gripper source)


@dataclass
class TeleopState:
    """Layer-1 output: everything the device knows this beat, device-agnostic."""
    sides: dict = field(default_factory=dict)    # side name -> SideState
    buttons: dict = field(default_factory=dict)  # button name -> bool (raw level)
    t_wall: float = 0.0
    ts_dev_ns: int = 0       # 设备侧样本时间戳(ns,设备时钟域);0=后端不支持。
                             # 用途:wall−ts 的增长=上游链路排队(堵塞)的直接证据


@dataclass
class ArmIntent:
    """Per-arm command for one control beat. Two channels (either or both):
    EE-delta (VR/keyboard style) and absolute joints (leader-follower style,
    e.g. SO101 dual leader arms — follower mirrors leader joints directly)."""
    dpos: np.ndarray         # (3,) EE displacement target, metres
    drot: np.ndarray         # (3,) EE rotation target, axis-angle, radians
    gripper: float = 0.0     # analog [0,1]; env adapter thresholds/uses as-is
    joints: np.ndarray = None  # optional (n_joints,) absolute joint targets


@dataclass
class ControlIntent:
    """Layer-2 output: env-agnostic control command. Arms absent from `arms`
    are NOT engaged this beat (env adapter must not move them)."""
    arms: dict = field(default_factory=dict)     # arm name -> ArmIntent
    t_wall: float = 0.0


@dataclass
class TakeoverEvent:
    """Layer-2 output: normalised takeover/verdict event (see kinds above).
    Downstream consumers (recorders, takeover arbiters) key off
    ENGAGE/DISENGAGE exclusively — no client may invent its own engagement notion."""
    kind: str
    arm: str = ""            # "" for global events (save/discard)
    t_wall: float = 0.0
