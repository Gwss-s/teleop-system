"""连通性探针: 打印右手柄位姿/grip/trigger 和备用输入(aux), 验证 Pico → PC Service → pybind 全链路.

用法(pixi shell 里, 仓库根目录): python scripts/pico_probe.py
  * 挥手柄 pos 变化、捏 grip 值升到 1.0 = 链路通;
  * aux 一行列出所有备用输入的实时值——给自制末端执行器绑键时, 从这里抄名字填进
    configs/teleop/pico_ur5e.yaml 的 actuators: source。
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop_system.backends.pico_ultra4 import PicoUltra4  # noqa: E402

backend = PicoUltra4(sides=("right",))
print("[probe] SDK 已初始化, 等待头显连接+Send On ...", flush=True)
t0 = time.time()
last_ts = 0
try:
    while time.time() - t0 < 120:
        st = backend.read()
        s = st.sides["right"]
        fresh = "NEW" if st.ts_dev_ns != last_ts else "stale"
        last_ts = st.ts_dev_ns
        aux = " ".join(f"{k}={v:+.2f}" for k, v in sorted(st.aux.items()))
        print(f"[{time.time()-t0:5.1f}s] {fresh:5s} pos=({s.pos[0]:+.3f},{s.pos[1]:+.3f},{s.pos[2]:+.3f}) "
              f"grip={s.grip:.2f} trig={s.trigger:.2f} A={int(st.buttons['A'])} B={int(st.buttons['B'])}",
              flush=True)
        print(f"          aux: {aux or '(SDK 未提供备用输入)'}", flush=True)
        time.sleep(1.0)
finally:
    backend.close()
