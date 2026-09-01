"""Teleop interface with TAKEOVER semantics (the thing LeRobot's Teleoperator lacks).

Takeover-style collection (DAgger family) needs three things from a teleop device:
  1. engagement edge events (human takes over / hands back)  -> z/x, grip button, ...
  2. the human action stream while engaged                    -> a_human
  3. episode verdict keys (save/discard) are NOT teleop's job -> recorder owns them

Implementations wrap concrete devices; the collection client only sees this
interface, so keyboard / Pico-VR / (later) LeRobot leader-arm are swappable.
"""
from abc import ABC, abstractmethod

import numpy as np


class Teleop(ABC):
    """Poll-based: call update() once per control beat."""

    @abstractmethod
    def update(self) -> None:
        """Pump device events (non-blocking)."""

    @abstractmethod
    def engaged(self) -> bool:
        """True while the human has taken over."""

    @abstractmethod
    def get_action(self) -> np.ndarray | None:
        """Latest human action (action_dim,) while engaged; None if unavailable.
        Semantics must match the dataset's action convention (e.g. absolute
        joint targets), because delta* = clip(a_human - a_base, +-B)."""


# Planned backends:
#   keyboard.py  — keyboard EE-teleop (z/x engage keys; Isaac-side RMPflow or joint jog)
#   pico.py      — Pico VR via XRoboToolkit (grip button = engage, 6DoF wrist tracking)
#   lerobot.py   — thin wrapper over lerobot.teleoperators (leader arms / gamepad) for
#                  real-robot setups; engagement mapped from a dedicated button.
