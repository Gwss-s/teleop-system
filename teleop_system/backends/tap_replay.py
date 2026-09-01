"""Layer-1 backend that REPLAYS a recorded raw tap (see pico_ultra4 tap_path).

Makes "re-map without re-teleoperating" a fact: feed a recorded session
through a NEW mapping config and regenerate the action stream offline.

    backend = TapReplay("outputs/teleop_demos/.../raw_tap_1724....npz")
    mapper = EEDeltaMapper.from_yaml("configs/teleop/new_tuning.yaml")
    while (ts := backend.read()) is not None:
        intent, events = mapper.step(ts)
        ...

Uses geometry.xr_pose_to_world — the same single-source transform as the live
backend, so a replayed stream is bit-identical to what the live session saw.
"""
import numpy as np

from ..geometry import xr_pose_to_world
from ..types import SideState, TeleopState


class TapReplay:
    """read() returns the next recorded beat's TeleopState, None at end."""

    def __init__(self, path):
        d = np.load(path, allow_pickle=False)
        self.sides = tuple(str(s) for s in d["sides"])
        self._t = d["t_wall"]
        self._pose = d["pose_xr"]
        self._grip, self._trigger = d["grip"], d["trigger"]
        self._a, self._b = d["btn_a"], d["btn_b"]
        self._side_idx = d["side_idx"]
        self._i = 0                       # row cursor; one beat = len(sides) rows

    def __len__(self):
        return len(self._t) // len(self.sides)

    def read(self):
        n = len(self.sides)
        if self._i + n > len(self._t):
            return None
        i = self._i
        st = TeleopState(sides={}, t_wall=float(self._t[i]),
                         buttons={"A": bool(self._a[i]), "B": bool(self._b[i])})
        for k in range(n):
            r = i + k
            side = self.sides[int(self._side_idx[r])]
            pos, rot = xr_pose_to_world(self._pose[r])
            st.sides[side] = SideState(pos=pos, rot=rot,
                                       grip=float(self._grip[r]),
                                       trigger=float(self._trigger[r]))
        self._i += n
        return st

    def close(self):
        pass
