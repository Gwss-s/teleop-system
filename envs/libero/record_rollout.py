#!/usr/bin/env python
"""Rollout 视频录制:基座策略跑指定任务,存 mp4(agentview|wrist 并排,含步数与结果标注)。

用途:任务选型时"眼见为实"——观察策略的失败形态,判断是否适合人工接管纠正
(系统性偏移/时机错误=适合;完全不知所措/感知崩溃=不适合)。

Run (liberoplus_sim env, 打在空闲的 5557 采集基座上, 不占评测池):
  MUJOCO_GL=osmesa PYTHONPATH=$REPO python envs/libero/record_rollout.py \
      --port 5557 --suite libero_spatial --task-ids 438,446 --out-dir /tmp/videos
"""
import argparse
import socket
import time

import numpy as np

from teleop_system.policy.protocol import recv_msg, send_msg
from envs.libero.eval_client import NOOP, WARMUP_STEPS, obs_to_request

FPS = 20


def main():
    # 同 collect_client:LIBERO env 每次重建泄漏 ~100 fd,批量录制需提软上限防 EMFILE
    import resource
    _soft, _hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    if _soft < _hard:
        resource.setrlimit(resource.RLIMIT_NOFILE, (_hard, _hard))

    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--suite", default=None)
    ap.add_argument("--task-ids", default=None, help="comma list (single-suite mode)")
    ap.add_argument("--tasks-json", default=None,
                    help='批量模式: JSON 文件 [{"suite","task_id","label"},...];'
                         "输出 out_dir/<suite>/<label>_t<id>_<成功|失败>.mp4")
    ap.add_argument("--max-steps", type=int, default=400)
    ap.add_argument("--n-action-steps", type=int, default=10)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    import json as jsonlib
    import cv2
    from pathlib import Path
    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv

    if args.tasks_json:
        jobs = jsonlib.load(open(args.tasks_json))
    else:
        jobs = [{"suite": args.suite, "task_id": int(x), "label": f"task{x}"}
                for x in args.task_ids.split(",")]

    out_root = Path(args.out_dir)
    conn = socket.create_connection((args.host, args.port))
    suites = {}

    for job in jobs:
        sname, tid, label = job["suite"], int(job["task_id"]), job["label"]
        init_idx = int(job.get("init", 0))
        if sname not in suites:
            suites[sname] = benchmark.get_benchmark_dict()[sname]()
        suite = suites[sname]
        task = suite.get_task(tid)
        out = out_root / sname.replace("libero_", "")
        out.mkdir(parents=True, exist_ok=True)

        env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(tid),
                                 camera_heights=256, camera_widths=256)
        env.seed(0)
        env.reset()
        obs = env.set_init_state(suite.get_task_init_states(tid)[init_idx])
        for _ in range(WARMUP_STEPS):
            obs, _, _, _ = env.step(NOOP)

        tmp = out / f".rec_t{tid}_i{init_idx}.mp4"
        vw = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (1024, 512))

        def frame(o, step, verdict=None):
            a = cv2.cvtColor(o["agentview_image"][::-1, ::-1], cv2.COLOR_RGB2BGR)
            w = cv2.cvtColor(o["robot0_eye_in_hand_image"][::-1, ::-1], cv2.COLOR_RGB2BGR)
            f = cv2.resize(np.hstack([a, w]), (1024, 512))
            cv2.putText(f, f"{sname} task{tid}  step {step}", (12, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 255), 2)
            if verdict:
                cv2.putText(f, verdict, (12, 84), cv2.FONT_HERSHEY_SIMPLEX, 1.4,
                            (0, 255, 0) if verdict == "SUCCESS" else (0, 0, 255), 3)
            return f

        done, steps, t0 = False, 0, time.time()
        while not done and steps < args.max_steps:
            send_msg(conn, obs_to_request(obs, task.language))
            chunk = np.asarray(recv_msg(conn), dtype=np.float32)
            for a in chunk[:args.n_action_steps]:
                obs, _, done, _ = env.step(a)
                vw.write(frame(obs, steps))
                steps += 1
                if done or steps >= args.max_steps:
                    break
        verdict = "SUCCESS" if done else "FAIL (timeout)"
        for _ in range(FPS * 2):                       # 结尾定格2秒标注结果
            vw.write(frame(obs, steps, verdict))
        vw.release()
        env.close()
        final = out / f"{label}_t{tid}_{'成功' if done else '失败'}.mp4"
        tmp.rename(final)
        print(f"[rec] {sname}/{final.name}: {verdict} steps={steps} wall={time.time()-t0:.0f}s", flush=True)
    conn.close()


if __name__ == "__main__":
    main()
