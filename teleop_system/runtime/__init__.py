"""Env-agnostic client runtime: everything between "got an observation" and
"publish an action" that does NOT depend on the simulator or robot.

  chunk_source.py — AsyncChunkSource: sim-clock chunk consumption for the
      takeover client (official-eval sync mode, shadow queries during human
      takeover, rebase-on-handback). Only takeover-mode entries import this
      package; pure-teleop entries never touch it.
"""
