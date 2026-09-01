"""Layer-1 device backends: one file per hardware, device-only knowledge.

  pico_ultra4.py   Pico Ultra 4 via XRoboToolkit           (done)
  remote.py        cross-process bridge (SDK env <-> sim env, TCP)
  tap_replay.py    replay a recorded raw tap (offline re-mapping / tests)

Adding a device = one new file implementing `read() -> TeleopState`.
Heavy vendor imports (SDK / pygame / hidapi) stay INSIDE that file;
the rest of the framework only depends on numpy.
"""
