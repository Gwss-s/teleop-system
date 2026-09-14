"""Freedrive 工作空间标定: 持续采样 TCP, 输出 min/max 供 configs/ur5e.yaml workspace 使用.

用法: python scripts/workspace_calib.py [控制柜IP] [采样秒数]
  IP 不填就用 configs/ur5e.yaml 的 robot.host;秒数默认 120。
  示教器切手动模式+自由驱动, 拖末端扫过预期工作空间边界, Ctrl-C 或超时结束。
"""
import sys
import time
from pathlib import Path

import numpy as np
import rtde_receive
import yaml

_cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "configs/ur5e.yaml").read_text())
HOST = sys.argv[1] if len(sys.argv) > 1 else str(_cfg["robot"]["host"])
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

r = rtde_receive.RTDEReceiveInterface(HOST)
lo = np.full(3, np.inf)
hi = np.full(3, -np.inf)
t0 = time.time()
n = 0
print(f"[calib] 采样 {DURATION:.0f}s, 现在拖动机械臂扫工作空间边界 ...", flush=True)
try:
    while time.time() - t0 < DURATION:
        p = np.array(r.getActualTCPPose()[:3])
        lo = np.minimum(lo, p)
        hi = np.maximum(hi, p)
        n += 1
        if n % 50 == 0:
            print(f"[{time.time()-t0:5.1f}s] cur={np.round(p,3).tolist()} "
                  f"lo={np.round(lo,3).tolist()} hi={np.round(hi,3).tolist()}", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
print("\n===== 采样结果(基座系,米) =====")
for i, ax in enumerate("xyz"):
    print(f"  {ax}: [{lo[i]:+.3f}, {hi[i]:+.3f}]  跨度 {hi[i]-lo[i]:.3f}")
print("(写 yaml 前建议向内收 1-2cm 余量;z 下限尤其注意桌面高度)")
