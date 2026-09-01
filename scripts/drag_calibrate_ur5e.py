"""拖动式自动标定(UR5e + Pico): 手柄固定在末端 -> freedrive 拖臂 -> Kabsch 解对齐.

原理: 手柄刚性随末端运动时, TCP 轨迹(基座系)与手柄轨迹(头显世界系)是同一条
曲线在两个坐标系下的表达; 中心化后 SVD(Kabsch) 解最优旋转 R(det=+1).
两系都重力对齐 => R 应为纯偏航; 解完检查倾斜与残差即是标定质量报告.
平移与旋转共用同一 R, 所以 pos_sign/rot_sign 全部 +1, 站位信息全进 world_yaw_deg.

用法(先退出 teleop_record):
    PYTHONPATH=$PWD python scripts/drag_calibrate_ur5e.py
步骤: 手柄绑/握在末端法兰上(尽量贴近 TCP, 全程别松) -> 回车进入拖动模式 ->
拖末端画一个大的立体轨迹(比如 20cm+ 的三维 L 形/方框, 尽量少转动手柄朝向) ->
40s 自动结束并写回 configs/teleop/pico_ur5e.yaml.
"""
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop_system.backends.factory import make_backend  # noqa: E402

ROBOT_CONFIG = "configs/ur5e.yaml"
TELEOP_CONFIG = "configs/teleop/pico_ur5e.yaml"
DURATION = 40.0      # 采样时长(s)
HZ = 50.0
MIN_SPREAD2 = 0.03   # 第二主轴 RMS 跨度下限(m): 轨迹不能近似一条直线
MAX_TILT_DEG = 8.0   # R 的非偏航(倾斜)分量超过此值 -> 标定可疑
MAX_RMS = 0.02       # 对齐残差 RMS 上限(m)


def kabsch(A, B):
    """最优旋转 R: R @ A_i ~= B_i(A,B 已中心化, (N,3))."""
    C = A.T @ B
    U, S, Vt = np.linalg.svd(C)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, S


def wait_trigger_press(backend, side, prompt):
    """等一次扳机按下(>0.7 上升沿); 返回时扳机已松开(等下降沿, 防止连按)."""
    print(prompt, flush=True)
    prev = 1.0   # 初始当作按着, 强制先看到松开状态
    while True:
        s = backend.read().sides.get(side)
        cur = s.trigger if s is not None else 0.0
        if cur > 0.7 and prev <= 0.7:
            break
        prev = cur
        time.sleep(0.02)
    while True:  # 等松开
        s = backend.read().sides.get(side)
        if s is None or s.trigger < 0.3:
            return
        time.sleep(0.02)


def main():
    import rtde_control
    import rtde_receive

    rcfg = yaml.safe_load(Path(ROBOT_CONFIG).read_text())
    host = rcfg["robot"]["host"]
    tcfg = yaml.safe_load(Path(TELEOP_CONFIG).read_text())
    arm_name, arm = next(iter(tcfg["arms"].items()))
    side = arm.get("side", arm_name)

    print(f"[标定] 连接 {host} ...", flush=True)
    rtde_r = rtde_receive.RTDEReceiveInterface(host)
    backend = make_backend("pico", sides=(side,))
    rtde_c = None
    try:
        rtde_c = rtde_control.RTDEControlInterface(host)
        rtde_c.teachMode()
        print("[标定] 已进入拖动示教(freedrive)", flush=True)
    except Exception as e:
        print(f"[标定] teachMode 不可用({e}); 请在示教器上切手动模式并按住自由驱动键拖动",
              flush=True)

    print("\n⚠ 手柄靠头显摄像头追踪: 戴着头显看着臂拖, 或让头显'看得见'手柄;"
          "\n  摘下头显时给距离传感器贴胶带, 否则它睡眠后追踪冻结(上次失败的原因)。",
          flush=True)
    wait_trigger_press(backend, side,
                       "\n手柄固定在末端(贴近 TCP)后, 【按一下扳机】开始采样 >>>")
    print(f"[标定] 采样中(最长 {DURATION:.0f}s, 再按一下扳机提前结束): "
          "拖末端画一个大的立体轨迹, 尽量少转手柄朝向 ...", flush=True)

    P, H = [], []   # TCP(基座系) / 手柄(世界系)
    t0 = time.time()
    period = 1.0 / HZ
    warned_frozen = False
    trig_prev = 1.0
    while True:
        el = time.time() - t0
        if el >= DURATION:
            break
        p = np.array(rtde_r.getActualTCPPose()[:3])
        s = backend.read().sides.get(side)
        if s is not None:
            if el > 2.0 and s.trigger > 0.7 and trig_prev <= 0.7:
                print(f"    [{el:4.1f}s] 扳机结束采样", flush=True)
                break
            trig_prev = s.trigger
            P.append(p)
            H.append(np.array(s.pos))
        if len(P) % 100 == 0 and P:
            hs = float(np.linalg.norm(np.ptp(np.asarray(H), axis=0)))
            ps_ = float(np.linalg.norm(np.ptp(np.asarray(P), axis=0)))
            print(f"    [{el:4.1f}s] 样本 {len(P)} | 轨迹跨度 TCP {ps_:.2f}m / "
                  f"手柄 {hs:.2f}m", flush=True)
            if not warned_frozen and ps_ > 0.05 and hs < 0.01:
                warned_frozen = True
                print("    ⚠ 臂在动但手柄轨迹不动 -> 追踪疑似冻结! 检查头显是否睡眠/"
                      "能否看到手柄", flush=True)
        time.sleep(period)

    if rtde_c is not None:
        rtde_c.endTeachMode()
        rtde_c.stopScript()
        print("[标定] 已退出拖动示教", flush=True)
    try:
        backend.xrt.close()
    except Exception:
        pass

    P = np.asarray(P)
    H = np.asarray(H)
    A = H - H.mean(axis=0)
    B = P - P.mean(axis=0)
    spread_p = np.sqrt(np.linalg.eigvalsh(B.T @ B / len(B)))[::-1]  # 主轴 RMS
    spread_h = np.sqrt(np.linalg.eigvalsh(A.T @ A / len(A)))[::-1]
    print(f"\n[标定] 样本 {len(P)}, 主轴 RMS 跨度: TCP {np.round(spread_p,3).tolist()} m"
          f" / 手柄 {np.round(spread_h,3).tolist()} m", flush=True)
    assert spread_p[1] >= MIN_SPREAD2, \
        f"TCP 轨迹太扁(第二主轴 {spread_p[1]:.3f} < {MIN_SPREAD2}), 请画更立体的轨迹重跑"
    assert spread_h[1] >= MIN_SPREAD2, \
        (f"手柄轨迹太扁(第二主轴 {spread_h[1]:.3f} < {MIN_SPREAD2}): 追踪冻结或手柄没绑紧,"
         " 不写盘, 修复后重跑")
    assert spread_h[0] >= 0.5 * spread_p[0], \
        "手柄轨迹跨度远小于 TCP: 追踪断续或手柄松动, 不写盘, 修复后重跑"

    R, _ = kabsch(A, B)
    rms = float(np.sqrt(np.mean(np.sum((A @ R.T - B) ** 2, axis=1))))
    yaw_deg = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    tilt_deg = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    print(f"[标定] 对齐残差 RMS = {rms*1000:.1f} mm, 偏航 = {yaw_deg:+.1f}°, "
          f"倾斜 = {tilt_deg:.1f}°", flush=True)
    if tilt_deg > MAX_TILT_DEG:
        print(f"  ⚠ 倾斜 > {MAX_TILT_DEG}°: 两系应都重力对齐, 检查手柄是否松动/追踪丢失",
              flush=True)
    if rms > MAX_RMS:
        print(f"  ⚠ 残差 > {MAX_RMS*1000:.0f}mm: 手柄可能没绑紧, 或离 TCP 太远且转动了朝向",
              flush=True)

    pos_scale = arm.get("pos_scale", 0.8)
    rot_scale = arm.get("rot_scale", 1.0)
    engage_threshold = tcfg.get("engage_threshold", 0.9)
    Path(TELEOP_CONFIG).rename(TELEOP_CONFIG + ".bak")
    Path(TELEOP_CONFIG).write_text(f"""\
# Pico Ultra 4 -> UR5e 真机(scripts/drag_calibrate_ur5e.py 拖动标定写入)
# 残差 RMS {rms*1000:.1f}mm / 倾斜 {tilt_deg:.1f}°; world_yaw_deg 绑定头显 app
# 启动朝向 -- 重启 app 后需要重标!
device: pico_ultra4
engage_threshold: {engage_threshold}
save_button: B
discard_button: A
arms:
  right:
    side: right
    pos_scale: {pos_scale}
    rot_scale: {rot_scale}
    pos_sign: [1.0, 1.0, 1.0]
    rot_sign: [1.0, 1.0, 1.0]
    world_yaw_deg: {yaw_deg:.1f}
""")
    print(f"[标定] 已写回 {TELEOP_CONFIG}(旧文件 -> .bak)", flush=True)


if __name__ == "__main__":
    main()
