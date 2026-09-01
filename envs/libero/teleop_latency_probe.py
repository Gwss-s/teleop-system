#!/usr/bin/env python
"""Rigorous teleop latency probe — decomposes END-TO-END lag segment by segment.

The beat-level diag (teleop_latency_diag.py) only measures THROUGHPUT (stream
stalls). Felt latency is AGE: hand moves at T, sim arm moves at T+L. L =
  (1) 头显采样周期     ~11 ms @90Hz          (不可测下界)
  (2) 头显→PC 管线 age  Phase STREAM 直接测量 (含 WiFi/服务缓冲/批传)
  (3) 拍量化           0-50 ms @20Hz, 均值 25 (固有)
  (4) 新旧栈计算差异    Phase AB 活体对比      (回答"是否重构引入")
  (5) OSC 控制器收敛    Phase STEP 阶跃响应    (sim 侧固有"跟手性")
  (6) 渲染/显示        ~1-2 帧

Phases (all default on, skip with flags):
  STREAM  1 kHz 紧轮询 30 s(无 sim 无 GUI):流速率/到达间隔/批传检测/数据 age。
          age 依据 xrt.get_time_stamp_ns 与本机钟的差;若时钟不同源,报告
          "min 校正抖动"(变化量仍然有效)。期间请连续快速挥动手柄。
  AB      20 Hz 30 s:同一数据流同时喂旧 PicoTeleop 与新三层栈,逐拍断言输出
          一致 + 对比各自耗时。期间握住 grip 挥动。
  STEP    自动(无需手柄):LIBERO 里发方波 delta 指令,互相关测 EE 响应滞后
          (OSC 收敛拍数)——重构前后完全相同的固有延迟,量化"跟手性"基线。

Run (conda `libero`, PC Service + headset ON, worn on forehead):
  PYTHONPATH=<repo> MUJOCO_GL=egl python envs/libero/teleop_latency_probe.py
"""
import argparse
import time

import numpy as np

NS = 1e9


def pct_ms(a, unit=1e-3):
    a = np.asarray(a, dtype=np.float64) / unit
    if len(a) == 0:
        return "n/a"
    return (f"p50 {np.percentile(a, 50):6.1f} | p90 {np.percentile(a, 90):6.1f} | "
            f"p99 {np.percentile(a, 99):6.1f} | max {a.max():7.1f}")


# ---------------------------------------------------------------- STREAM ----
def phase_stream(xrt, seconds):
    print(f"\n{'='*68}\nPHASE STREAM — {seconds:.0f}s 1kHz 紧轮询。请【连续快速挥动手柄】\n{'='*68}")
    arrivals = []          # (t_local_ns, ts_dev_ns, pos)
    prev_ts = None
    t_end = time.time() + seconds
    while time.time() < t_end:
        ts = xrt.get_time_stamp_ns()
        if ts != prev_ts:
            p = xrt.get_right_controller_pose()
            arrivals.append((time.time_ns(), int(ts), (p[0], p[1], p[2])))
            prev_ts = ts
        time.sleep(0.0005)

    if len(arrivals) < 50:
        print("[stream] 样本太少,检查连接"); return
    tl = np.array([a[0] for a in arrivals], dtype=np.float64)
    td = np.array([a[1] for a in arrivals], dtype=np.float64)
    # 垃圾时间戳过滤: 设备戳应该单调递增且节奏 ~90Hz;
    # 偏离样本中位 age 超过 5s 的视为坏包(ts=0/回绕/未初始化)
    age_raw = (tl - td) / NS
    med = np.median(age_raw)
    good = np.abs(age_raw - med) < 5.0
    n_bad = int((~good).sum())
    tl, td, age = tl[good], td[good], age_raw[good]
    n = len(tl)
    dur = (tl[-1] - tl[0]) / NS
    inter = np.diff(tl) / NS                      # 到达间隔(本机钟)
    inter_dev = np.diff(td) / NS                  # 设备戳间隔

    print(f"[stream] 有效更新 {n} 个 / {dur:.1f}s = {n/dur:.1f} Hz "
          f"(设备戳节奏 {1.0/max(np.median(inter_dev),1e-9):.1f} Hz, 剔除坏包 {n_bad})")
    print(f"[stream] 到达间隔 ms: {pct_ms(inter)}")
    # 批传检测:大量间隔≈0 + 周期性大间隔 = 打包到达
    burst = float((inter < 0.002).mean())
    print(f"[stream] 批传指数: {burst*100:.0f}% 的样本与前一个几乎同时到达"
          f"{' ← 存在打包/缓冲!' if burst > 0.3 else ' (基本逐个到达)'}")
    gaps = inter[inter > 0.05]
    if len(gaps):
        print(f"[stream] >50ms 断口 {len(gaps)} 次, 最长 {gaps.max()*1000:.0f} ms")
    # 数据 age:稳健基线 = p05(min 会被单个坏包毁掉)
    base = np.percentile(age, 5)
    adj = age - base
    if abs(base) < 0.5:
        print(f"[stream] 时钟基本同源(offset {base*1000:.1f}ms) → 管线 age 直接可信:")
    else:
        print(f"[stream] 时钟不同源(offset {base:.3f}s) → 报告 p05 校正抖动(下界):")
    print(f"[stream]   age-p05 ms: {pct_ms(adj)}")
    # 时间线分桶: age 是"常驻高位"(缓冲堆积)还是"偶发尖峰"(断口后追赶)
    nb = 6
    edges = np.linspace(tl[0], tl[-1], nb + 1)
    row = []
    for k in range(nb):
        m = (tl >= edges[k]) & (tl < edges[k + 1])
        row.append(f"{np.median(adj[m])*1000:5.0f}" if m.any() else "  n/a")
    print(f"[stream]   age 中位数时间线(每{dur/nb:.0f}s一桶,ms): {' '.join(row)}")
    print(f"[stream]   判读: 各桶平稳≈0 → 无常驻延迟 | 逐桶抬升 → 缓冲堆积 | 个别桶高 → 偶发拥塞")
    return


# -------------------------------------------------------------------- AB ----
def phase_ab(xrt, seconds):
    print(f"\n{'='*68}\nPHASE AB — {seconds:.0f}s 20Hz 新旧栈同流对比。请【握住 grip 挥动】\n{'='*68}")
    from teleop_system.legacy.pico import LiberoEEDeltaMapper, PicoTeleop
    from teleop_system.backends.pico_ultra4 import PicoUltra4
    from teleop_system.mapping.ee_delta import EEDeltaMapper
    from envs.libero.teleop_adapter import intent_to_osc

    legacy = PicoTeleop(side="right", mapper=LiberoEEDeltaMapper(), xrt=xrt)
    backend = PicoUltra4(sides=("right",), xrt=xrt)
    mapper = EEDeltaMapper.from_yaml("configs/teleop/pico_libero.yaml")

    t_old, t_new, diffs, engaged_ct = [], [], [], 0
    n = 0
    t_end = time.time() + seconds
    while time.time() < t_end:
        beat = time.time()
        a0 = time.time(); legacy.update(); a1 = time.time()
        intent, _ = mapper.step(backend.read()); a2 = time.time()
        t_old.append(a1 - a0); t_new.append(a2 - a1)
        if legacy.engaged() != mapper.engaged("right"):
            diffs.append((n, "engage-mismatch"))
        elif legacy.engaged():
            engaged_ct += 1
            va, vb = legacy.get_action(), intent_to_osc(intent, "right")
            d = float(np.abs(va - vb).max()) if vb is not None else 1e9
            if d > 1e-6:
                diffs.append((n, d))
        n += 1
        dt = time.time() - beat
        if dt < 0.05:
            time.sleep(0.05 - dt)
    print(f"[ab] {n} 拍, 接管 {engaged_ct} 拍")
    print(f"[ab] 旧栈耗时 ms: {pct_ms(t_old)}")
    print(f"[ab] 新栈耗时 ms: {pct_ms(t_new)}")
    if diffs:
        print(f"[ab] ❌ 输出不一致 {len(diffs)} 拍: {diffs[:5]}")
    else:
        print("[ab] ✅ 全部拍输出逐位一致 → 重构未改变任何指令值")
    if engaged_ct < 100:
        print("[ab] ⚠️ 接管拍太少,建议重测并全程握住 grip")


# ------------------------------------------------------------------ STEP ----
def phase_step(args):
    print(f"\n{'='*68}\nPHASE STEP — 自动 OSC 阶跃响应(无需手柄,手别碰)\n{'='*68}")
    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv
    suite = benchmark.get_benchmark_dict()[args.suite]()
    env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(args.task_id),
                             camera_heights=128, camera_widths=128, ignore_done=True)
    env.seed(0); env.reset()
    obs = env.set_init_state(suite.get_task_init_states(args.task_id)[0])
    noop = np.array([0, 0, 0, 0, 0, 0, -1], dtype=np.float32)
    for _ in range(20):
        obs, _, _, _ = env.step(noop)

    # 方波:x 方向 ±0.3 每 10 拍翻转,4 个周期
    cmd, disp = [], []
    prev = obs["robot0_eef_pos"].copy()
    for k in range(80):
        c = 0.3 if (k // 10) % 2 == 0 else -0.3
        a = noop.copy(); a[0] = c
        obs, _, _, _ = env.step(a)
        cur = obs["robot0_eef_pos"].copy()
        cmd.append(c); disp.append(cur[0] - prev[0]); prev = cur
    env.close()

    cmd = np.asarray(cmd) - np.mean(cmd)
    disp = np.asarray(disp) - np.mean(disp)
    lags = range(0, 8)
    xc = [float(np.dot(cmd[:len(cmd)-L], disp[L:])) for L in lags]
    best = int(np.argmax(xc))
    gain = float(np.dot(cmd, disp) / np.dot(cmd, cmd)) / 0.05  # 实现位移/指令位移
    print(f"[step] OSC 响应滞后 ≈ {best} 拍 = {best*50} ms (互相关峰)")
    print(f"[step] 单拍跟踪增益 ≈ {gain*100:.0f}% (指令 0.3→{0.3*0.05*1000:.0f}mm/拍, "
          f"实际 {gain*0.3*0.05*1000:.1f}mm/拍)")
    print(f"[step] 解读: 增益<100% 意味着 EE 用多拍才走完一拍的指令 → 固有'跟手'滞后,"
          f"\n[step]       该滞后与遥操栈无关(重构前后同一 robosuite 控制器)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_spatial")
    ap.add_argument("--task-id", type=int, default=0)
    ap.add_argument("--stream-seconds", type=float, default=30)
    ap.add_argument("--ab-seconds", type=float, default=30)
    ap.add_argument("--skip-stream", action="store_true")
    ap.add_argument("--skip-ab", action="store_true")
    ap.add_argument("--skip-step", action="store_true")
    args = ap.parse_args()

    xrt = None
    if not (args.skip_stream and args.skip_ab):
        import xrobotoolkit_sdk as xrt_mod
        xrt_mod.init()
        xrt = xrt_mod
        time.sleep(0.5)

    if not args.skip_stream:
        phase_stream(xrt, args.stream_seconds)
    if not args.skip_ab:
        input("\n>>> 准备好后回车进入 AB 阶段(全程握住 grip 挥动) ")
        phase_ab(xrt, args.ab_seconds)
    if not args.skip_step:
        phase_step(args)

    print(f"\n{'='*68}\n延迟链汇总(把各段相加即端到端感受):\n"
          f"  头显采样(~11ms) + 管线age(STREAM) + 拍量化(均值25ms)\n"
          f"  + OSC收敛(STEP) + 渲染(~1-2帧)\n"
          f"  重构贡献 = AB 阶段的差异(预期恒为 0)\n{'='*68}")
    if xrt is not None:
        xrt.close()


if __name__ == "__main__":
    main()
