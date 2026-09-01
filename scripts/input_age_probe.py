#!/usr/bin/env python
"""遥操输入链路堵塞探针（独立运行，不依赖 LIBERO/策略服务器）。

背景（2026-08-25）：握住手柄很久才接管 = grip 值到达 PC 时已是旧数据 =>
数据在 头显 -> (WiFi) -> PC Service -> SDK 某环节排队。本探针用
`xrt.get_time_stamp_ns()`（官方 xr_client 同款）逐拍测量：

  输入龄 age = (本机墙钟 - 样本时间戳) - 会话最小值
    * 最小值归一化消掉时钟域偏移；age 的【增长】= 队列在变深（堵塞铁证）；
    * age 恒定且小(<50ms) = 链路干净，延迟感来自别处。

用法（戴好头显、手柄持续小幅移动，期间捏几次 grip 到底）：
  python scripts/input_age_probe.py --seconds 60
输出：每秒一行实况 + 结束判决 + jsonl 明细(/tmp/teleop_probe_<ts>.jsonl)。
"""
import argparse
import json
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=20.0)
    args = ap.parse_args()

    import numpy as np
    import xrobotoolkit_sdk as xrt
    xrt.init()

    path = f"/tmp/teleop_probe_{int(time.time())}.jsonl"
    f = open(path, "w")
    beat_dt = 1.0 / args.hz
    t_end = time.time() + args.seconds
    offset = None                    # min(wall - ts) 时钟域偏移估计
    prev_pose, prev_ts, prev_grip = None, None, 0.0
    ages, rates, stales = [], [], []
    sec_ages, sec_new, sec_reads = [], 0, 0
    t_sec = time.time()

    print(f"[probe] {args.seconds:.0f}s @ {args.hz:.0f}Hz -> {path}")
    print("[probe] 手柄保持小幅移动;期间捏几次 grip 到底(测接管龄)")
    while time.time() < t_end:
        t0 = time.time()
        ts = int(xrt.get_time_stamp_ns())
        pose = tuple(xrt.get_right_controller_pose())
        grip = float(xrt.get_right_grip())
        diff = t0 - ts / 1e9
        offset = diff if offset is None else min(offset, diff)
        age_ms = (diff - offset) * 1e3
        stale = int(pose == prev_pose)
        new_ts = int(ts != prev_ts)
        ages.append(age_ms); stales.append(stale)
        sec_ages.append(age_ms); sec_new += new_ts; sec_reads += 1
        if grip > 0.9 and prev_grip <= 0.9:
            print(f"  >>> GRIP 沿到达: 此刻输入龄 {age_ms:.0f}ms "
                  f"(你感到的'很久才接管'≈这个数+一拍)", flush=True)
        f.write(json.dumps({"t": round(t0, 3), "age_ms": round(age_ms, 1),
                            "ts_new": new_ts, "stale": stale,
                            "grip": round(grip, 2)}) + "\n")
        prev_pose, prev_ts, prev_grip = pose, ts, grip
        if t0 - t_sec >= 1.0:
            a = sorted(sec_ages)
            rate = sec_new / (t0 - t_sec)
            rates.append(rate)
            print(f"[probe] age p50={a[len(a)//2]:6.0f}ms max={a[-1]:6.0f}ms | "
                  f"新样本 {rate:4.0f}/s | 陈旧读 {100*sum(stales[-sec_reads:])/sec_reads:3.0f}%",
                  flush=True)
            sec_ages, sec_new, sec_reads, t_sec = [], 0, 0, t0
        dt = time.time() - t0
        if dt < beat_dt:
            time.sleep(beat_dt - dt)

    f.close()
    a = sorted(ages)
    n = len(a)
    head = sorted(ages[: n // 6]) or [0]
    tail = sorted(ages[-n // 6:]) or [0]
    growth = tail[len(tail) // 2] - head[len(head) // 2]
    print("\n========== 判决 ==========")
    print(f"输入龄: p50={a[n//2]:.0f}ms p95={a[int(n*.95)]:.0f}ms max={a[-1]:.0f}ms")
    print(f"前1/6段 p50={head[len(head)//2]:.0f}ms -> 后1/6段 p50={tail[len(tail)//2]:.0f}ms "
          f"(增长 {growth:+.0f}ms)")
    print(f"陈旧读占比: {100*sum(stales)/n:.1f}%   新样本率中位: {sorted(rates)[len(rates)//2]:.0f}/s")
    if growth > 150:
        print("=> 【堵塞确认】输入龄随时间增长=链路排队(头显->PC)。优先治链路:"
              "USB 直连(gnirehtet)/独占5G热点/PC Service 重启。")
    elif a[n // 2] > 100:
        print("=> 【恒定高延迟】链路有固定排队/缓冲。查热点带宽、PC Service 设置。")
    else:
        print("=> 【链路干净】输入龄小且稳——延迟感不在输入链路,回头查执行/显示侧。")
    print(f"明细: {path}")


if __name__ == "__main__":
    main()
