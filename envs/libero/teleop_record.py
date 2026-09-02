#!/usr/bin/env python
"""纯遥操作 + 回合录制(本系统的主入口;无策略依赖、完全单机)。

三层栈(docs/architecture.md):
    PicoUltra4 (L1 设备后端) -> EEDeltaMapper (L2 yaml 映射) -> intent_to_osc (L3 OSC)
与 collect_client.py 走同一条映射路径,手感/标定完全通用;区别只是动作 100% 来自人,
不连策略服务器。

操作(右手柄): 捏住 GRIP 移动,松开保持;TRIGGER 夹爪;B=保存本回合,A=作废重来。
回合在任务成功(done)时自动保存,超时(--max-steps)自动作废。

录制格式与 collect_client.py 的 episode npz 同构(mode/state/a_base/a_applied/
a_base_age/speed/wrist/agentview/sim_step),其中 a_base 恒为 NOOP、mode 为
human(接管拍)/idle(松手保持拍)——下游数据管线可以无差别读取。

运行(pixi shell,PC Service + 头显已连接并 Send On):
  PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
      --suite libero_goal --task-id 0 --out-dir outputs/teleop_demos
"""
import argparse
import time
from pathlib import Path

import numpy as np

from teleop_system.backends.factory import make_backend
from teleop_system.mapping.ee_delta import EEDeltaMapper
from teleop_system.types import DISCARD, SAVE
from envs.libero.eval_client import NOOP, WARMUP_STEPS, obs_to_request
from envs.libero.teleop_adapter import intent_to_osc

CTRL_HZ = 20.0
TELEOP_CONFIG = "configs/teleop/pico_libero.yaml"


def save_episode(buf, out_dir, ep_idx, tid):
    """与 collect_client.save_episode 同构的 npz(a_base=NOOP, age=0, speed=1)。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"episode_{ep_idx:04d}_t{tid}_{int(time.time())}.npz"
    np.savez_compressed(
        path,
        mode=np.array([r["mode"] for r in buf]),
        state=np.stack([r["state"] for r in buf]),
        a_base=np.stack([np.asarray(NOOP, np.float32)] * len(buf)),
        a_applied=np.stack([r["a_applied"] for r in buf]),
        a_base_age=np.zeros(len(buf), np.int64),
        speed=np.ones(len(buf), np.float64),
        wrist=np.stack([r["wrist"] for r in buf]),
        agentview=np.stack([r["agent"] for r in buf]),
        sim_step=np.array([r["t"] for r in buf]),
    )
    n_h = sum(1 for r in buf if r["mode"] == "human")
    print(f"[record] saved {path.name}: {len(buf)} frames ({n_h} human)", flush=True)


def main():
    # LIBERO env 重建泄漏 fd,提软上限(同 collect_client)
    try:
        import resource                 # Unix-only 模块;Windows 无此模块也无此问题
        _soft, _hard = resource.getrlimit(resource.RLIMIT_NOFILE)
        if _soft < _hard:
            resource.setrlimit(resource.RLIMIT_NOFILE, (_hard, _hard))
    except ImportError:
        pass

    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_goal")
    ap.add_argument("--task-id", type=int, default=0)
    ap.add_argument("--teleop-config", default=TELEOP_CONFIG)
    ap.add_argument("--backend", choices=("pico", "gamepad", "remote"), default="pico",
                    help="输入设备(gamepad=Xbox 手柄, remote=跨进程桥)")
    ap.add_argument("--gamepad-config", default="configs/gamepad.yaml")
    ap.add_argument("--pos-scale", type=float, default=None)
    ap.add_argument("--rot-scale", type=float, default=None)
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--out-dir", default="outputs/teleop_demos")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-raw-tap", action="store_true",
                    help="关闭设备原始流录制(tap 支持事后换 yaml 重映射)")
    args = ap.parse_args()

    import cv2
    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv

    suite = benchmark.get_benchmark_dict()[args.suite]()
    task = suite.get_task(args.task_id)
    task_str = task.language
    init_states = suite.get_task_init_states(args.task_id)
    env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(args.task_id),
                             camera_heights=256, camera_widths=256)
    env.seed(args.seed)

    out_dir = Path(args.out_dir) / f"{args.suite}_{args.task_id}"
    tap = None
    if args.backend == "pico" and not args.no_raw_tap:
        out_dir.mkdir(parents=True, exist_ok=True)
        tap = out_dir / f"raw_tap_{int(time.time())}.npz"
    if args.backend == "gamepad" and args.teleop_config == TELEOP_CONFIG:
        args.teleop_config = "configs/teleop/gamepad_libero.yaml"   # 手柄专用标定
        print(f"[record] backend=gamepad -> teleop-config 自动切换: {args.teleop_config}", flush=True)
    backend = make_backend(args.backend, tap_path=tap, gamepad_config=args.gamepad_config)
    mapper = EEDeltaMapper.from_yaml(args.teleop_config,
                                     pos_scale=args.pos_scale, rot_scale=args.rot_scale)
    print(f"[record] task {args.task_id}: \"{task_str}\"", flush=True)
    print("[record] grip=移动 松手=保持 trigger=夹爪 | B=保存 A=作废 | 成功自动保存 超时自动作废", flush=True)

    ep_idx, n_saved, n_discard = 0, 0, 0
    try:
        while True:                                    # ---- episode loop ----
            env.reset()
            obs = env.set_init_state(init_states[ep_idx % len(init_states)])
            for _ in range(WARMUP_STEPS):
                obs, _, _, _ = env.step(NOOP)
            buf, done, steps = [], False, 0
            gripper_hold = -1.0
            while not done and steps < args.max_steps:
                beat = time.time()
                intent, tevents = mapper.step(backend.read())
                ev = {e.kind for e in tevents}
                if SAVE in ev:
                    save_episode(buf, out_dir, ep_idx, args.task_id)
                    n_saved += 1; ep_idx += 1; buf = []; break
                if DISCARD in ev:
                    print(f"[record] discarded ({len(buf)} frames)")
                    n_discard += 1; buf = []; break

                if mapper.engaged("right"):
                    a = intent_to_osc(intent, "right"); gripper_hold = a[6]; mode = "human"
                else:
                    a = NOOP.copy(); a[6] = gripper_hold; mode = "idle"

                req = obs_to_request(obs, task_str)
                buf.append(dict(mode=mode, state=req["state"],
                                a_applied=np.asarray(a, np.float32).copy(),
                                wrist=obs["robot0_eye_in_hand_image"][::-1, ::-1].copy(),
                                agent=obs["agentview_image"][::-1, ::-1].copy(), t=steps))
                obs, _, done, _ = env.step(a)
                steps += 1

                view = np.hstack([
                    cv2.resize(cv2.cvtColor(obs["agentview_image"][::-1, ::-1], cv2.COLOR_RGB2BGR), (512, 512)),
                    cv2.resize(cv2.cvtColor(obs["robot0_eye_in_hand_image"][::-1, ::-1], cv2.COLOR_RGB2BGR), (512, 512))])
                eng = mapper.engaged("right")
                cv2.putText(view, f"{'HUMAN' if eng else 'idle'}  step {steps}  ep {ep_idx}"
                                  f"  saved {n_saved} disc {n_discard}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255) if eng else (170, 170, 170), 2)
                cv2.imshow("teleop-record", view)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    raise KeyboardInterrupt
                dt = time.time() - beat
                if dt < 1.0 / CTRL_HZ:
                    time.sleep(1.0 / CTRL_HZ - dt)

            if done and buf:                           # 任务成功:自动保存
                print(f"[record] SUCCESS at step {steps}")
                save_episode(buf, out_dir, ep_idx, args.task_id)
                n_saved += 1; ep_idx += 1
            elif buf:                                  # 超时:自动作废
                print(f"[record] timeout at step {steps}: discarded")
                n_discard += 1
    except KeyboardInterrupt:
        print(f"\n[record] stopped: saved {n_saved} discarded {n_discard}")
    finally:
        for closer in (backend.close, env.close, cv2.destroyAllWindows):
            try:
                closer()
            except Exception:
                pass


if __name__ == "__main__":
    main()
