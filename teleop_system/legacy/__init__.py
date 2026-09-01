"""Frozen legacy implementations — the equivalence-regression baseline.

pico.py (single-file PicoTeleop + LiberoEEDeltaMapper) is the feel-verified,
sign-calibrated original that the 3-layer stack was extracted from.
tests/test_teleop_equivalence.py pins the new stack to this code bit-for-bit:
DO NOT edit the maths here; if mapping maths must change, update the test
consciously (it is the fuse protecting the calibration).

base.py is the old ABC interface (update/engaged/get_action), superseded by
the duck-typed `read() -> TeleopState` backend contract.
"""
