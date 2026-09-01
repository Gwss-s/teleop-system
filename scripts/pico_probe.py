"""连通性探针: 打印右手柄位姿/grip/trigger, 验证 Pico → PC Service → pybind 全链路."""
import time
import xrobotoolkit_sdk as xrt

xrt.init()
print("[probe] xrt.init() OK, 等待头显连接+Send On ...", flush=True)
t0 = time.time()
last_ts = 0
while time.time() - t0 < 120:
    ts = xrt.get_time_stamp_ns()
    p = xrt.get_right_controller_pose()
    g = xrt.get_right_grip()
    tr = xrt.get_right_trigger()
    fresh = "NEW" if ts != last_ts else "stale"
    last_ts = ts
    print(f"[{time.time()-t0:5.1f}s] {fresh:5s} pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
          f"grip={g:.2f} trig={tr:.2f} ts={ts}", flush=True)
    time.sleep(1.0)
xrt.close()
