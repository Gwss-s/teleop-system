#!/usr/bin/env python
"""LIBERO autonomous eval client: local sim + remote frozen base policy.

Runs LIBERO episodes with actions served by a PolicyServer
(teleop_system.policy.server_lerobot, e.g. pi05_libero_base on the GPU box).

All conventions mirror the official lerobot LIBERO integration
(lerobot/envs/libero.py + lerobot/processor/env_processor.py):
  * obs.state (8,) = [eef_pos(3), eef_axisangle(3), gripper_qpos(2)],
    quat given as (x,y,z,w);
  * images agentview / robot0_eye_in_hand, 256x256, rotated 180 deg
    (img[::-1, ::-1]) to match the training-data orientation;
  * after reset: 10 warmup steps with the no-op action [0,...,0,-1];
  * execute n_action_steps(=10) actions of each chunk, then re-query.

Run (conda env `liberoplus_sim`, repo root on PYTHONPATH):
  PYTHONPATH=$PWD python envs/libero/eval_client.py \
      --host localhost --port 5557 --suite libero_spatial --task-id 0 --episodes 5
(Use an SSH tunnel for the GPU box: ssh -N -L 5557:localhost:5557 <gpu-host>)
"""
import argparse
import socket
import time

import numpy as np

from teleop_system.policy.protocol import recv_msg, send_msg

NOOP = np.array([0, 0, 0, 0, 0, 0, -1], dtype=np.float32)
WARMUP_STEPS = 10          # lerobot libero.py num_steps_wait
N_ACTION_STEPS = 10        # pi05-on-LIBERO official override (chunk_size=50)
CTRL_HZ = 20.0             # LIBERO control_freq (sim seconds = steps / CTRL_HZ)


def quat2axisangle(q):
    """(x,y,z,w) -> axis-angle vector; robosuite/lerobot formula."""
    q = np.asarray(q, dtype=np.float64)
    w = float(np.clip(q[3], -1.0, 1.0))
    den = np.sqrt(max(1.0 - w * w, 0.0))
    if den < 1e-10:
        return np.zeros(3)
    return q[:3] * (2.0 * np.arccos(w)) / den


def obs_to_request(obs, task_str):
    """LIBERO raw obs -> PolicyServer request dict."""
    state = np.concatenate([
        obs["robot0_eef_pos"],
        quat2axisangle(obs["robot0_eef_quat"]),
        obs["robot0_gripper_qpos"],
    ]).astype(np.float32)
    # 180-degree rotation: HuggingFaceVLA/libero camera orientation convention
    images = {
        "agentview": np.ascontiguousarray(obs["agentview_image"][::-1, ::-1]),
        "wrist": np.ascontiguousarray(obs["robot0_eye_in_hand_image"][::-1, ::-1]),
    }
    return {"state": state, "images": images, "task": task_str}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--suite", default="libero_spatial",
                    choices=["libero_spatial", "libero_object", "libero_goal",
                             "libero_90", "libero_10"])
    ap.add_argument("--task-id", type=int, default=0)
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--max-steps", type=int, default=400)
    ap.add_argument("--n-action-steps", type=int, default=N_ACTION_STEPS)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--show", action="store_true",
                    help="live agentview window (cv2), for watching the rollout")
    ap.add_argument("--out-jsonl", default="",
                    help="append one JSON record per episode (batch eval)")
    args = ap.parse_args()

    cv2 = None
    if args.show:
        import cv2  # noqa: PLC0415

    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv

    suite = benchmark.get_benchmark_dict()[args.suite]()
    task = suite.get_task(args.task_id)
    task_str = task.language
    init_states = suite.get_task_init_states(args.task_id)
    print(f"[eval] {args.suite}/{args.task_id}: \"{task_str}\"")

    env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(args.task_id),
                             camera_heights=256, camera_widths=256)
    env.seed(args.seed)

    conn = socket.create_connection((args.host, args.port))
    print(f"[eval] policy server connected {args.host}:{args.port}")

    successes = 0
    for ep in range(args.episodes):
        env.reset()
        obs = env.set_init_state(init_states[ep % len(init_states)])
        for _ in range(WARMUP_STEPS):
            obs, _, _, _ = env.step(NOOP)

        done, steps, t0 = False, 0, time.time()
        while not done and steps < args.max_steps:
            send_msg(conn, obs_to_request(obs, task_str))
            chunk = np.asarray(recv_msg(conn), dtype=np.float32)   # [chunk, 7]
            for a in chunk[:args.n_action_steps]:
                obs, _, done, _ = env.step(a)
                steps += 1
                if cv2 is not None:
                    view = cv2.cvtColor(obs["agentview_image"][::-1, ::-1], cv2.COLOR_RGB2BGR)
                    view = cv2.resize(view, (512, 512))
                    cv2.putText(view, f"ep{ep} step {steps}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)
                    cv2.imshow("libero-eval", view)
                    cv2.waitKey(1)
                if done or steps >= args.max_steps:
                    break
        successes += bool(done)
        print(f"[eval] ep{ep}: {'SUCCESS' if done else 'fail'} "
              f"steps={steps} wall={time.time()-t0:.1f}s "
              f"running={successes}/{ep+1}", flush=True)
        if args.out_jsonl:
            import json
            with open(args.out_jsonl, "a") as f:
                f.write(json.dumps({
                    "suite": args.suite, "task_id": args.task_id, "ep": ep,
                    "success": bool(done), "steps": steps,
                    "wall_s": round(time.time() - t0, 1), "seed": args.seed,
                    "wall_clock": time.time(),
                }) + "\n")

    print(f"[eval] final success rate: {successes}/{args.episodes}")
    conn.close()
    env.close()


if __name__ == "__main__":
    main()
