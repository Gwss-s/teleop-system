#!/usr/bin/env python
"""LIBERO takeover client: async chunks + clutch-tied time dilation.

Design synthesized from HIL-SERL/gym-hil, Sirius, TRANSIC, LeRobot rollout-
DAgger and the RTC paper:

  * AUTO = OFFICIAL-EVAL protocol (runtime/chunk_source.py sync mode): each
    chunk conditioned on the obs of its first executed step, only the first
    exec_horizon actions run (age 0..h-1), and the per-step delta is the
    blocking reply to THIS frame's submit -- step-for-step what eval_client
    measures. The world pauses (sim-time invisible) while a fresh chunk is in
    flight. Async prefetch is NOT used here: over a tunnel its round-trip
    exceeds the horizon and silently degrades to 20-45-step open loop, i.e. a
    different (much worse) policy than the evaluated one;
  * TIME DILATION tied to the clutch: sim runs 1.0x while autonomous, ramps
    to --human-speed (default 0.4x, slow motion) while grip is held, ramps
    back on release. Sirius ships slow-motion (--sleep_time) and TRANSIC
    pauses sim during correction; deliberate clutch-tied dilation is our
    controlled version (--human-speed 1.0 disables, for A/B comparison).
    Position-anchored Pico deltas are dilation-invariant, so pacing never
    enters the recorded data (speed_factor is logged per frame anyway);
  * HARD SWITCH arbitration (grip hold = human, release = policy), no
    blending -- blended actions are useless as supervision;
  * ENGAGE: chunks keep being queried but only logged (shadow query ->
    counterfactual a_base for delta* = a_human - a_base, Sirius pattern);
  * HANDBACK: flush stale plans + fresh-obs requery (lerobot #3747/#4398
    fix), and the sim is FROZEN (not stepped) until the new chunk lands --
    invisible in sim time, TRANSIC semantics for free;
  * B = save episode, A = discard (episode verdict).

Teleop is the 3-layer stack (docs/teleop_usage.md): PicoUltra4 backend ->
EEDeltaMapper (yaml: configs/teleop/pico_libero.yaml, calibrated) ->
envs/libero/teleop_adapter.intent_to_osc. A raw device tap is recorded per
session (re-map later without re-teleoperating); disable with --no-raw-tap.

Run (conda env `liberoplus_sim`, PC Service + headset connected & sending;
policy server on the GPU box, tunnel 5557):
  PYTHONPATH=$PWD python envs/libero/collect_client.py \
      --host localhost --port 5557 --suite libero_spatial --task-id 0
"""
import argparse
import collections
import gc
import json
import random
import socket
import time
from pathlib import Path

import numpy as np

from teleop_system.policy.protocol import recv_msg, send_msg
from teleop_system.runtime.chunk_source import AsyncChunkSource
from teleop_system.backends.pico_ultra4 import PicoUltra4
from teleop_system.mapping.ee_delta import EEDeltaMapper
from teleop_system.types import DISCARD, SAVE
from envs.libero.eval_client import NOOP, WARMUP_STEPS, obs_to_request
from envs.libero.teleop_adapter import intent_to_osc

CTRL_HZ = 20.0             # LIBERO control_freq (sim beat)
TELEOP_CONFIG = "configs/teleop/pico_libero.yaml"   # calibrated mapping
# 默认(未扰动)agentview 位姿 —— 源码 _setup_camera 常量;--ref-cam 时把 frontview
# 重定位到此,作为固定方向参考(相机扰动只动 agentview,不动这个参考 → 现有校准直接匹配)。
REF_CAM_POS = [0.6586131746834771, 0.0, 1.6103500240372423]
REF_CAM_QUAT = [0.6380177736282349, 0.3048497438430786, 0.30484986305236816, 0.6380177736282349]


def save_episode(buf, out_dir, ep_idx, tid):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"episode_{ep_idx:04d}_t{tid}_{int(time.time())}.npz"
    np.savez_compressed(
        path,
        mode=np.array([r["mode"] for r in buf]),
        state=np.stack([r["state"] for r in buf]),
        a_base=np.stack([r["a_base"] for r in buf]),
        a_applied=np.stack([r["a_applied"] for r in buf]),
        a_base_age=np.array([r["age"] for r in buf]),      # chunk staleness (steps)
        speed=np.array([r["speed"] for r in buf]),         # dilation factor per frame
        wrist=np.stack([r["wrist"] for r in buf]),
        agentview=np.stack([r["agent"] for r in buf]),
        sim_step=np.array([r["t"] for r in buf]),
    )
    n_h = sum(1 for r in buf if r["mode"] == "human")
    print(f"[collect] saved {path.name}: {len(buf)} frames ({n_h} human)", flush=True)


def main():
    # LIBERO OffScreenRenderEnv 每次重建泄漏恒定 ~100 个 fd(EGL/nvidia 句柄,close 不回收;
    # 实测 envs/libero 轮转 6 次 71->571)。终端默认软上限 1024 => ~10 局后 EMFILE,
    # MuJoCo 误报 "resource not found"。软上限提到硬上限(通常 1M),千局量级无忧。
    import resource
    _soft, _hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    if _soft < _hard:
        resource.setrlimit(resource.RLIMIT_NOFILE, (_hard, _hard))

    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--suite", default="libero_spatial")
    ap.add_argument("--task-id", type=int, default=0)
    ap.add_argument("--task-ids", default="",
                    help="逗号分隔的变体任务清单(cell 采集模式):每回合轮转一个任务,"
                         "覆盖 --task-id。失败变体优先排前面。")
    ap.add_argument("--no-shuffle", action="store_true",
                    help="关闭每 epoch 自动打乱(默认开启:每跑完一遍 task-ids 就用 --seed 重新洗牌"
                         "再开新 epoch,消除采集顺序偏置)")
    ap.add_argument("--max-steps", type=int, default=600)
    ap.add_argument("--n-action-steps", type=int, default=10)
    # -- teleop (3-layer stack; see docs/teleop_usage.md) --
    ap.add_argument("--teleop-config", default=TELEOP_CONFIG,
                    help="yaml mapping config (calibrated axis signs/gains)")
    ap.add_argument("--pos-scale", type=float, default=None,
                    help="override yaml pos gain (None = use yaml)")
    ap.add_argument("--rot-scale", type=float, default=None,
                    help="override yaml rot gain (None = use yaml)")
    ap.add_argument("--no-raw-tap", action="store_true",
                    help="disable raw device-stream recording (tap enables re-mapping)")
    ap.add_argument("--view-size", type=int, default=0,
                    help="每个视角的渲染边长; 0=自动(min(win-h, win-w/2))")
    ap.add_argument("--win-w", type=int, default=1900,
                    help="窗口宽(px)。1900x950=手感验证过的默认;大屏可试 2800x1400"
                         "(显示开销 5->9ms/拍,预算50ms;1880/视角实测16ms,不建议)")
    ap.add_argument("--win-h", type=int, default=950,
                    help="窗口高(px)")
    ap.add_argument("--ref-cam", action="store_true",
                    help="加一个固定在【默认 agentview 位姿】的参考相机(仅显示,不进策略/录制);"
                         "相机扰动下方向错乱时开,现有 agentview 校准直接匹配,免重标定")
    ap.add_argument("--human-speed", type=float, default=0.4,
                    help="接管时的时间膨胀系数(0.4=慢动作,标准配置; 1.0=关闭,消融用)")
    ap.add_argument("--ramp-beats", type=int, default=5,
                    help="速度渐变节拍数(~0.25s @20Hz)")
    ap.add_argument("--out-dir", default="outputs/takeover_demos")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resume-collection", action="store_true",
                    help="从 out_dir/collect_progress.json 恢复 ep_idx/order/rng,续采上次没做完的 task")
    args = ap.parse_args()

    import cv2
    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv

    suite = benchmark.get_benchmark_dict()[args.suite]()
    # cell 采集模式: 多变体轮转(每回合换一个任务,env 按需重建)
    task_ids = ([int(x) for x in args.task_ids.split(",") if x.strip()]
                if args.task_ids else [args.task_id])
    env, cur_tid, task_str, init_states = None, None, "", None

    def load_task(tid):
        nonlocal env, cur_tid, task_str, init_states
        if tid == cur_tid:
            return
        if env is not None:
            env.close()
            env = None
            gc.collect()          # LIBERO 已知泄漏:重建前尽量回收
        task = suite.get_task(tid)
        task_str = task.language
        init_states = suite.get_task_init_states(tid)
        for attempt in (1, 2):
            try:
                cams = ["agentview", "robot0_eye_in_hand"]
                if args.ref_cam:
                    cams.append("frontview")     # 复用现成相机作固定参考
                env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(tid),
                                         camera_heights=256, camera_widths=256,
                                         camera_names=cams)
                break
            except Exception as e:
                # 已知形态:多局重建后内存耗尽 -> MuJoCo 误报 "resource not found"
                if attempt == 2:
                    raise RuntimeError(
                        f"env 重建失败(task {tid}),多为 LIBERO 重建泄漏累积内存耗尽;"
                        f"重启 collect_client 即可续采(数据已保存,不丢)") from e
                print(f"[collect] env build failed ({e}); gc + retry...", flush=True)
                gc.collect()
                time.sleep(2.0)
        env.seed(args.seed)
        if args.ref_cam:                         # 把 frontview 重定位到默认 agentview 位姿
            m = env.sim.model
            fid = m.camera_name2id("frontview")
            m.cam_pos[fid] = REF_CAM_POS
            m.cam_quat[fid] = REF_CAM_QUAT
            env.sim.forward()
        cur_tid = tid
        print(f"[collect] task {tid}: \"{task_str}\"", flush=True)

    load_task(task_ids[0])

    conn = socket.create_connection((args.host, args.port))
    # -- teleop 3-layer stack: device backend -> yaml mapper -> OSC adapter --
    out_dir = Path(args.out_dir) / (f"{args.suite}_{task_ids[0]}" if len(task_ids) == 1
                                    else f"{args.suite}_cell{len(task_ids)}x")
    tap = None
    if not args.no_raw_tap:
        out_dir.mkdir(parents=True, exist_ok=True)
        tap = out_dir / f"raw_tap_{int(time.time())}.npz"
    backend = PicoUltra4(sides=("right",), tap_path=tap)
    mapper = EEDeltaMapper.from_yaml(args.teleop_config,
                                     pos_scale=args.pos_scale, rot_scale=args.rot_scale)
    source = AsyncChunkSource(conn, send_msg, recv_msg,
                              exec_horizon=args.n_action_steps, chunk_len=50)
    print(f"[collect] \"{task_str}\"", flush=True)
    print(f"[collect] grip=接管(慢动作 x{args.human_speed}) 松手=交还 B=保存 A=作废 Ctrl-C=退出", flush=True)

    # 全阶段延时日志(复盘用,独立于回合数据,绝不进 npz)
    out_dir.mkdir(parents=True, exist_ok=True)
    perf_path = out_dir / f"perf_{int(time.time())}.jsonl"
    perf_f = open(perf_path, "w")
    print(f"[collect] 每拍分阶段延时 -> {perf_path}", flush=True)

    # 可缩放窗口(WINDOW_NORMAL,可拖角/最大化);内容按窗口分辨率直接渲染
    # (Qt 后端不给上采样,窗口大内容小 -> 自己 resize 到满窗)
    cv2.namedWindow("libero-takeover", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("libero-takeover", args.win_w, args.win_h)
    if not args.view_size:
        args.view_size = min(args.win_h, args.win_w // 2)
    K = args.view_size / 950.0                # HUD 字号/坐标随分辨率缩放

    def dual_view(o, S):
        va = cv2.resize(cv2.cvtColor(o["agentview_image"][::-1, ::-1], cv2.COLOR_RGB2BGR), (S, S))
        vw = cv2.resize(cv2.cvtColor(o["robot0_eye_in_hand_image"][::-1, ::-1], cv2.COLOR_RGB2BGR), (S, S))
        panes = [va, vw]
        if args.ref_cam and "frontview_image" in o:   # 固定参考相机置最左,给操作员看方向
            vr = cv2.resize(cv2.cvtColor(o["frontview_image"][::-1, ::-1], cv2.COLOR_RGB2BGR), (S, S))
            hud_text(vr, "REF 看这里操作", 14, 42, 0.9, (0, 255, 255))
            panes = [vr, va, vw]
        return np.hstack(panes)

    def hud_text(img, text, x, y, scale, color):
        cv2.putText(img, text, (int(x * K), int(y * K)), cv2.FONT_HERSHEY_SIMPLEX,
                    scale * K, color, max(2, int(round(2 * K))))

    def panel(img, x, y, lines, scale=0.6, lh=30):
        """半透明黑底板 + 多行彩色文字(ASCII,cv2 渲染不了中文)。lines=[(text,color),...]"""
        x0, y0 = int(x * K), int(y * K)
        ncol = max((len(t) for t, _ in lines), default=0)
        w = int((ncol * scale * 12.5 + 14) * K); h = int((len(lines) * lh + 8) * K)
        ov = img.copy()
        cv2.rectangle(ov, (x0 - 6, y0 - int(20 * K)), (x0 - 6 + w, y0 - int(20 * K) + h), (0, 0, 0), -1)
        cv2.addWeighted(ov, 0.5, img, 0.5, 0, img)
        for i, (t, c) in enumerate(lines):
            cv2.putText(img, t, (x0, y0 + int(i * lh * K)), cv2.FONT_HERSHEY_SIMPLEX,
                        scale * K, c, max(1, int(round(1.6 * K))))

    ep_idx = 0
    perf_work, perf_disp = [], []      # 每拍工作耗时/显示耗时(实时性能仪表)
    stale_win = collections.deque(maxlen=100)   # 设备陈旧帧滑窗
    age_off = None                     # min(wall-ts_dev) 时钟域偏移(输入龄归一化)
    age_ms = 0.0
    prev_pose_bytes, last_disp_ms = None, 0.0
    # 每 epoch 打乱: order 是 task_ids 的一个排列;每跑完一遍就用固定种子重洗(可复现)
    order_rng = random.Random(args.seed)
    order = list(range(len(task_ids)))
    last_epoch = -1                                    # 只在真正进入新 epoch 时洗牌一次(防重采触发重洗)
    # ---- 仪表盘计数器(全程 + 本 epoch) ----
    t_sess0 = None                                     # 首拍墙钟,算采集时长
    n_beat_all = n_human_all = 0                       # 全程拍数 / 人接管拍数
    n_saved = n_discard = 0                            # 回合裁决累计(B/成功=存,A/超时=弃)
    ep_beat = ep_human = 0                             # 本 epoch 拍数 / 人接管拍数(每 epoch 清零)

    # ---- 续采:进度落盘 + 恢复(从上次没做完的 task 接着采) ----
    prog_path = out_dir / "collect_progress.json"

    def save_progress():
        tmp = prog_path.with_suffix(".json.tmp")
        with open(tmp, "w") as f:
            json.dump({"version": 1, "suite": args.suite, "task_ids": task_ids,
                       "seed": args.seed, "no_shuffle": args.no_shuffle,
                       "ep_idx": ep_idx, "last_epoch": last_epoch, "order": order,
                       "rng_state": order_rng.getstate(),
                       "n_saved": n_saved, "n_discard": n_discard,
                       "wall": time.time()}, f)
        tmp.replace(prog_path)                          # 原子替换,防 Ctrl-C 半截

    if args.resume_collection and prog_path.exists():
        with open(prog_path) as f:
            st = json.load(f)
        assert st["suite"] == args.suite and st["task_ids"] == task_ids, (
            f"续采文件与本次 suite/task_ids 不符,拒绝续采"
            f"(存档={st['suite']}/{len(st['task_ids'])}变体)")
        ep_idx, last_epoch, order = st["ep_idx"], st["last_epoch"], st["order"]
        n_saved, n_discard = st["n_saved"], st["n_discard"]
        rs = st["rng_state"]                            # JSON 把 tuple 拍平成 list,需还原
        order_rng.setstate((rs[0], tuple(rs[1]), rs[2]))
        print(f"[collect] 续采: ep_idx={ep_idx} epoch={last_epoch} "
              f"saved={n_saved} disc={n_discard} order={order}", flush=True)
    elif args.resume_collection:
        print("[collect] --resume-collection 但无进度文件,从头开始", flush=True)
    try:
        while True:                                   # ---- episode loop (无限,自动开新 epoch) ----
            pos = ep_idx % len(task_ids)
            epoch = ep_idx // len(task_ids)
            if epoch != last_epoch:                   # 新 epoch 边界(重采同一 task 时 epoch 不变,不重洗)
                if not args.no_shuffle:
                    order_rng.shuffle(order)          # 每 epoch 重新打乱
                ep_beat = ep_human = 0                # 本 epoch 接管率清零(看接管率逐 epoch 变化)
                print(f"[collect] ===== epoch {epoch} 开始"
                      f"{'(已打乱)' if not args.no_shuffle else '(固定序)'}, "
                      f"共 {len(task_ids)} 变体 =====", flush=True)
                last_epoch = epoch
            load_task(task_ids[order[pos]])
            env.reset()
            obs = env.set_init_state(init_states[ep_idx % len(init_states)])
            for _ in range(WARMUP_STEPS):
                obs, _, _, _ = env.step(NOOP)
            source.start_episode(obs_to_request(obs, task_str), step=0)
            buf, done, steps, was_engaged = [], False, 0, False
            advance = False        # 只有 B 保存 / 自动成功才前进;A 舍弃 / 超时 → 重来同一 task
            speed = 1.0                               # dilation state

            while not done and steps < args.max_steps:
                beat = time.time()
                tm = {}                                   # 本拍分阶段耗时(ms)
                tstate = backend.read()
                tm["read"] = (time.time() - beat) * 1e3
                # 输入龄: (墙钟-设备样本时间戳)-会话最小值;增长=上游链路排队(堵塞)
                if tstate.ts_dev_ns:
                    diff = beat - tstate.ts_dev_ns / 1e9
                    age_off = diff if age_off is None else min(age_off, diff)
                    age_ms = (diff - age_off) * 1e3
                # 设备新鲜度:位姿与上一拍逐位相同=陈旧帧(头显没送到新包)
                sr = tstate.sides["right"]
                pb = sr.pos.tobytes() + sr.rot.tobytes()
                stale = (pb == prev_pose_bytes)
                prev_pose_bytes = pb
                stale_win.append(stale)
                t0 = time.time()
                intent, tevents = mapper.step(tstate)
                tm["map"] = (time.time() - t0) * 1e3
                engaged = mapper.engaged("right")
                ev_kinds = {e.kind for e in tevents}

                # 回合裁决优先处理(在任何分支之前,世界暂停期间按键也有效)
                if SAVE in ev_kinds:
                    save_episode(buf, out_dir, ep_idx, cur_tid); buf = []; advance = True; n_saved += 1; break
                if DISCARD in ev_kinds:
                    print(f"[collect] episode discarded ({len(buf)} frames) → 重来同一 task {cur_tid}"); buf = []; n_discard += 1; break

                # -- time-dilation state machine (ramped, clutch-tied) --
                target = args.human_speed if engaged else 1.0
                speed += (target - speed) / max(args.ramp_beats, 1)

                req = obs_to_request(obs, task_str)
                t0 = time.time()
                if engaged:
                    a_base = source.next_a_base(req, steps)   # 接管期:影子查询,永不阻塞
                else:
                    if was_engaged:                       # 交还:清旧计划,喂新观测
                        source.rebase(req, steps)
                        was_engaged = False
                    # 官方评测协议(eval_client 同款):chunk 以段首当帧 obs 为条件,
                    # 只执行前 exec_horizon 步(age 恒 0..h-1);新 chunk 未到 → None,
                    # 下面世界暂停等它。异步预取在隧道 RTT 下会退化成 20-45 步开环,
                    # 跑的根本不是测评那个策略 —— 这里必须与测评逐步一致
                    a_base = source.next_a_base(req, steps, sync=True)
                tm["chunk"] = (time.time() - t0) * 1e3

                if engaged:
                    if not was_engaged:
                        print(f"[age] 接管沿到达,此刻输入龄 {age_ms:.0f}ms "
                              f"(体感'捏了很久才接管'≈此数)", flush=True)
                    t0 = time.time()
                    a_applied = intent_to_osc(intent, "right")
                    tm["map"] += (time.time() - t0) * 1e3
                    mode = "human"
                    was_engaged = True
                    if a_base is None:                    # rebase 结果未到:影子暂缺
                        a_base = NOOP
                else:
                    if a_base is None:
                        # ---- 世界暂停:重规划期间不步进仿真(sim 时间无痕) ----
                        view = dual_view(obs, args.view_size)
                        hud_text(view, f"REPLAN (world paused)  step {steps}",
                                 (964 if args.ref_cam else 14), 42, 1.0, (0, 200, 255))
                        cv2.imshow("libero-takeover", view)
                        cv2.waitKey(1)
                        time.sleep(0.02)
                        continue                          # steps 不增长,世界冻结
                    a_applied, mode = a_base, "auto"

                buf.append(dict(mode=mode, state=req["state"], a_base=np.asarray(a_base, np.float32).copy(),
                                a_applied=np.asarray(a_applied, np.float32).copy(),
                                age=steps - source.origin, speed=speed,
                                wrist=obs["robot0_eye_in_hand_image"][::-1, ::-1].copy(),
                                agent=obs["agentview_image"][::-1, ::-1].copy(),
                                t=steps))

                t0 = time.time()
                obs, _, done, _ = env.step(np.asarray(a_applied, np.float32))
                tm["sim"] = (time.time() - t0) * 1e3
                steps += 1
                if t_sess0 is None:
                    t_sess0 = beat                    # 首个真正步进的拍 = 采集起点
                n_beat_all += 1; ep_beat += 1
                if mode == "human":
                    n_human_all += 1; ep_human += 1

                t_disp = time.time()
                S = args.view_size
                view = dual_view(obs, S)
                # ---- 仪表盘(操作台):进度 / 接管率 / 时长 / 链路健康 ----
                #   分阶段延时(read/map/chunk/sim/disp)仍写进 perf_*.jsonl,不上屏避免拥挤
                stale_pct = 100.0 * sum(stale_win) / max(len(stale_win), 1)
                work_ms = (time.time() - beat) * 1e3
                budget = (1.0 / CTRL_HZ) / max(speed, 0.05)
                el = int(beat - (t_sess0 or beat))
                ovr = 100.0 * n_human_all / max(n_beat_all, 1)
                epr = 100.0 * ep_human / max(ep_beat, 1)
                bad = age_ms > 120 or work_ms > budget * 1e3 or stale_pct > 15
                panel(view, (964 if args.ref_cam else 14), 40, [
                    (f"{'HUMAN takeover' if mode == 'human' else 'AUTO policy'}  step {steps}",
                     (0, 0, 255) if mode == "human" else (0, 210, 0)),
                    (f"epoch {epoch}   task {pos + 1}/{len(task_ids)}   id {cur_tid}", (255, 255, 255)),
                    (f"takeover  {epr:.0f}% ep   {ovr:.0f}% all", (0, 255, 160)),
                    (f"saved {n_saved}  disc {n_discard}   time {el // 60:02d}:{el % 60:02d}", (230, 230, 230)),
                    (f"age {age_ms:.0f}ms  stale {stale_pct:.0f}%  x{speed:.2f}",
                     (0, 0, 255) if bad else (180, 180, 180)),
                ])
                cv2.imshow("libero-takeover", view)
                cv2.waitKey(1)
                last_disp_ms = (time.time() - t_disp) * 1e3
                tm["disp"] = last_disp_ms

                # -- pacing: wall dt = sim_dt / speed (dilation) --
                dt = time.time() - beat
                perf_f.write(json.dumps({
                    "t": round(beat, 3), "ep": ep_idx, "step": steps, "mode": mode,
                    "stale": int(stale), "age_ms": round(age_ms, 1),
                    **{k: round(v, 1) for k, v in tm.items()},
                    "work": round(dt * 1e3, 1), "budget": round(budget * 1e3, 1),
                    "speed": round(speed, 2)}) + "\n")
                perf_work.append(dt)
                perf_disp.append(tm["disp"] / 1e3)
                if len(perf_work) >= 200:              # 每 ~10s 报一次节拍健康度
                    w, d = sorted(perf_work), sorted(perf_disp)
                    miss = sum(1 for x in perf_work if x > budget) / len(perf_work)
                    print(f"[perf] 每拍 p50={w[100]*1e3:.0f}ms p95={w[190]*1e3:.0f}ms "
                          f"(显示 p50={d[100]*1e3:.0f}ms) 超预算 {miss*100:.0f}% "
                          f"陈旧帧 {stale_pct:.0f}%", flush=True)
                    perf_work.clear(), perf_disp.clear()
                    perf_f.flush()
                if dt < budget:
                    time.sleep(budget - dt)

            if done and buf:
                print(f"[collect] task SUCCESS at step {steps}")
                save_episode(buf, out_dir, ep_idx, cur_tid); advance = True; n_saved += 1
            elif buf:                                     # timeout, no verdict
                print(f"[collect] episode timeout at step {steps}: discarded ({len(buf)} frames) → 重来同一 task {cur_tid}")
                n_discard += 1
            if advance:
                ep_idx += 1
            else:
                print(f"[collect] ↩ 不前进,重采 task {cur_tid}", flush=True)
            save_progress()                               # 每回合裁决后落盘续采进度
    except KeyboardInterrupt:
        print("\n[collect] stopped")
        save_progress()                                   # ep_idx 未变,落盘保证续采点
    finally:
        # 逐个关,任何一个失败不阻塞其余(env 半构造时 close 会抛 AttributeError)
        for closer in (perf_f.close, source.close, backend.close, conn.close,
                       (env.close if env is not None else (lambda: None))):
            try:
                closer()
            except Exception:
                pass


if __name__ == "__main__":
    main()
