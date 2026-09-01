#!/usr/bin/env python
"""Teleop latency diagnosis: per-beat timing breakdown of the feel-check loop.

Splits every beat into its four suspects and prints percentile stats every 5 s:
  read  = backend.read() (SDK getter IPC cost)
  step  = env.step() (MuJoCo physics + offscreen render)
  gui   = cv2 convert/resize/imshow/waitKey
  stale = beats where the CONTROLLER DATA TIMESTAMP did not advance
          (xrt.get_time_stamp_ns unchanged -> no new packet arrived over WiFi
          since last beat -> network stall, independent of local compute)
  over  = beats whose total exceeded the 50 ms budget (20 Hz)

Verdict guide:
  stale spikes while read/step/gui stay flat  -> WiFi pose stream (hotspot)
  step p95 >> p50                             -> sim/render contention
  gui  p95 >> p50                             -> X11/compositor
  read p95 >> p50                             -> SDK/service IPC

Run (conda `libero` env, PC Service + headset connected, wave the controller
both engaged and idle for ~60 s):
  PYTHONPATH=<repo> MUJOCO_GL=egl python envs/libero/teleop_latency_diag.py
"""
import argparse
import time

import numpy as np

from teleop_system.backends.pico_ultra4 import PicoUltra4
from teleop_system.mapping.ee_delta import EEDeltaMapper
from envs.libero.teleop_adapter import intent_to_osc

NOOP = np.array([0, 0, 0, 0, 0, 0, -1], dtype=np.float32)
CTRL_HZ = 20.0
TELEOP_CONFIG = "configs/teleop/pico_libero.yaml"


def pct(a):
    a = np.asarray(a) * 1000.0
    return f"{np.percentile(a, 50):5.1f}/{np.percentile(a, 95):5.1f}/{a.max():6.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_spatial")
    ap.add_argument("--task-id", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--teleop-config", default=TELEOP_CONFIG)
    ap.add_argument("--no-gui", action="store_true", help="drop cv2 to isolate it")
    args = ap.parse_args()

    import cv2
    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv

    suite = benchmark.get_benchmark_dict()[args.suite]()
    env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(args.task_id),
                             camera_heights=256, camera_widths=256, ignore_done=True)
    env.seed(0)
    env.reset()
    obs = env.set_init_state(suite.get_task_init_states(args.task_id)[0])

    backend = PicoUltra4(sides=("right",))
    mapper = EEDeltaMapper.from_yaml(args.teleop_config)
    xrt = backend.xrt

    hist = {"read": [], "step": [], "gui": [], "total": []}
    stale = over = n = 0
    prev_ts = None
    stale_runs = []          # lengths of consecutive-stale bursts
    run = 0
    grip_hold = -1.0
    t_end = time.time() + args.seconds
    print(f"[diag] {args.seconds:.0f}s @20Hz — 挥动手柄,握/松 grip 交替 "
          f"(单位 ms, p50/p95/max)", flush=True)

    while time.time() < t_end:
        t0 = time.time()
        ts_dev = xrt.get_time_stamp_ns()
        st = backend.read()
        t1 = time.time()
        intent, _ = mapper.step(st)
        if mapper.engaged("right"):
            a = intent_to_osc(intent, "right")
            grip_hold = a[6]
        else:
            a = NOOP.copy(); a[6] = grip_hold
        obs, _, _, _ = env.step(a)
        t2 = time.time()
        if not args.no_gui:
            view = cv2.cvtColor(obs["agentview_image"][::-1, ::-1], cv2.COLOR_RGB2BGR)
            view = cv2.resize(view, (512, 512))
            cv2.putText(view, "DIAG " + ("HUMAN" if mapper.engaged("right") else "idle"),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            cv2.imshow("latency-diag", view)
            cv2.waitKey(1)
        t3 = time.time()

        hist["read"].append(t1 - t0)
        hist["step"].append(t2 - t1)
        hist["gui"].append(t3 - t2)
        if ts_dev == prev_ts:
            stale += 1; run += 1
        else:
            if run: stale_runs.append(run)
            run = 0
        prev_ts = ts_dev

        dt = time.time() - t0
        hist["total"].append(dt)
        if dt > 1.0 / CTRL_HZ:
            over += 1
        else:
            time.sleep(1.0 / CTRL_HZ - dt)
        n += 1
        if n % 100 == 0:
            print(f"[diag] n={n:4d} read {pct(hist['read'][-100:])} | "
                  f"step {pct(hist['step'][-100:])} | gui {pct(hist['gui'][-100:])} | "
                  f"total {pct(hist['total'][-100:])} | stale {stale} over {over}", flush=True)

    if run: stale_runs.append(run)
    print("\n========== 总结 ==========")
    for k in ("read", "step", "gui", "total"):
        print(f"  {k:5s} p50/p95/max = {pct(hist[k])} ms")
    print(f"  预算超支拍数: {over}/{n} ({100*over/max(n,1):.1f}%)")
    print(f"  位姿数据未更新拍数(stale): {stale}/{n} ({100*stale/max(n,1):.1f}%)")
    if stale_runs:
        print(f"  停顿爆发: {len(stale_runs)} 次, 最长连续 {max(stale_runs)} 拍 "
              f"(={max(stale_runs)*50} ms 无新位姿)")
    print("判定: stale高+其余平 → WiFi位姿流 | step尖 → 渲染 | gui尖 → X11 | read尖 → SDK IPC")
    backend.close(); env.close()
    if not args.no_gui:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
