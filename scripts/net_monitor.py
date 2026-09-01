#!/usr/bin/env python
"""独立网络延迟可视化监视器(与遥操作完全解耦,纯 ICMP ping)。

用途:遥操作前/中,独立验证 PC<->头显 无线链路质量。与 SDK/遥操进程零耦合
(不占 XRoboToolkit 连接,可与任何遥操客户端并行常开)。

原理: 以 5Hz ping 头显(默认自动发现热点上的客户端 IP),实时窗口显示:
  * 当前 RTT(大字,绿<30ms/黄<100ms/红>=100ms 或丢包)
  * 60 秒滚动曲线(对数刻度,一眼看出堵塞爬坡)
  * p50/p95/max/丢包率(滚动窗口)
判读: RTT 稳定个位数=链路干净; RTT 数百 ms 且持续爬升=bufferbloat(发快于传);
     成段丢包=射频断流。ping 测的是 ICMP 往返,与 SDK 的"输入龄"分层互证:
     ping 高 => 链路层就堵(与遥操软件无关); ping 低但输入龄高 => 查服务/应用层。

启动(独立终端,可一直开着):
  ~/miniconda3/envs/liberoplus_sim/bin/python scripts/net_monitor.py
  # 指定目标: --host 10.42.0.246   纯终端模式: --no-gui
"""
import argparse
import collections
import re
import subprocess
import time


def find_headset_ip(dev="wlo1"):
    """热点网卡邻居表里的第一个客户端 = 头显。"""
    try:
        out = subprocess.run(["ip", "neigh", "show", "dev", dev],
                             capture_output=True, text=True, timeout=3).stdout
        for line in out.splitlines():
            m = re.match(r"(\d+\.\d+\.\d+\.\d+)\s+lladdr", line)
            if m:
                return m.group(1)
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None, help="头显 IP(默认自动发现热点客户端)")
    ap.add_argument("--dev", default="wlo1", help="热点网卡(自动发现用)")
    ap.add_argument("--interval", type=float, default=0.2, help="ping 间隔(s),非root最小0.2")
    ap.add_argument("--no-gui", action="store_true", help="纯终端模式(每秒一行)")
    args = ap.parse_args()

    host = args.host or find_headset_ip(args.dev)
    if not host:
        print(f"[net] {args.dev} 上没发现客户端;头显连上热点后重试,或 --host 指定")
        return
    print(f"[net] 监视 {host} (ping {1/args.interval:.0f}Hz), Ctrl-C 退出")

    proc = subprocess.Popen(
        ["ping", "-O", "-i", str(args.interval), host],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

    N = int(60 / args.interval)               # 60 秒窗口
    rtts = collections.deque(maxlen=N)        # None=丢包
    t_line = time.time()

    gui = not args.no_gui
    if gui:
        try:
            import cv2
            import numpy as np
            cv2.namedWindow("net-monitor", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("net-monitor", 760, 380)
        except Exception as e:
            print(f"[net] GUI 不可用({e}),转终端模式")
            gui = False

    def stats():
        ok = sorted(r for r in rtts if r is not None)
        loss = 100.0 * sum(1 for r in rtts if r is None) / max(len(rtts), 1)
        if not ok:
            return None, None, None, loss
        return (ok[len(ok) // 2], ok[int(len(ok) * 0.95)], ok[-1], loss)

    try:
        for line in proc.stdout:
            m = re.search(r"time=([\d.]+)\s*ms", line)
            if m:
                rtts.append(float(m.group(1)))
            elif "no answer yet" in line or "Unreachable" in line:
                rtts.append(None)
            else:
                continue

            p50, p95, mx, loss = stats()
            cur = rtts[-1]

            if gui:
                import cv2
                import numpy as np
                img = np.zeros((380, 760, 3), np.uint8)
                if cur is None:
                    color, txt = (0, 0, 255), "LOST"
                else:
                    color = ((0, 220, 0) if cur < 30 else
                             (0, 220, 255) if cur < 100 else (0, 0, 255))
                    txt = f"{cur:.0f} ms"
                cv2.putText(img, txt, (24, 90), cv2.FONT_HERSHEY_SIMPLEX, 2.6, color, 5)
                if p50 is not None:
                    cv2.putText(img, f"p50 {p50:.0f}  p95 {p95:.0f}  max {mx:.0f} ms   loss {loss:.0f}%",
                                (24, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(img, f"{host}  60s window", (24, 370),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1)
                # 对数刻度曲线: 1ms->底部, 3000ms->顶部; 丢包画红柱
                x0, y0, w_, h_ = 24, 160, 712, 190
                cv2.rectangle(img, (x0, y0), (x0 + w_, y0 + h_), (60, 60, 60), 1)
                import math
                for gy, lbl in ((10, "10"), (100, "100"), (1000, "1000")):
                    yy = y0 + h_ - int(h_ * (math.log10(gy) - 0) / (math.log10(3000)))
                    cv2.line(img, (x0, yy), (x0 + w_, yy), (45, 45, 45), 1)
                    cv2.putText(img, lbl, (x0 + w_ - 40, yy - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
                for i, r in enumerate(rtts):
                    x = x0 + int(i * w_ / N)
                    if r is None:
                        cv2.line(img, (x, y0), (x, y0 + h_), (0, 0, 160), 1)
                    else:
                        v = max(min(r, 3000.0), 1.0)
                        y = y0 + h_ - int(h_ * math.log10(v) / math.log10(3000))
                        c = ((0, 220, 0) if r < 30 else
                             (0, 220, 255) if r < 100 else (0, 0, 255))
                        cv2.circle(img, (x, y), 1, c, -1)
                cv2.imshow("net-monitor", img)
                cv2.waitKey(1)
            elif time.time() - t_line >= 1.0:
                t_line = time.time()
                cs = "LOST" if cur is None else f"{cur:.0f}ms"
                print(f"[net] now={cs:>6}  p50={p50 or 0:.0f}ms p95={p95 or 0:.0f}ms "
                      f"max={mx or 0:.0f}ms loss={loss:.0f}%", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        p50, p95, mx, loss = stats()
        if p50 is not None:
            print(f"\n[net] 汇总: p50={p50:.0f}ms p95={p95:.0f}ms max={mx:.0f}ms loss={loss:.0f}%")


if __name__ == "__main__":
    main()
