"""引导式自动标定(UR5e 真机 + Pico): 机器人逐轴演示 -> 人捏 grip 模仿 -> 自动解出
站位偏航角(world_yaw_deg) + 逐轴符号, 写回 configs/teleop/pico_ur5e.yaml.

为什么有偏航角: 头显世界系朝向 = app 启动时头的朝向, 与机器人基座水平轴一般不
对齐(斜 45° 站位会让"只朝一个轴动"的手势散到两个轴上), 符号翻转修不了旋转——
所以 x 轴手势直接用来解偏航, 之后所有轴在"转正后"的坐标里判符号。

用法(先退出 teleop_record, RTDE 控制通道单客户端):
    PYTHONPATH=$PWD python scripts/auto_calibrate_ur5e.py
每轴: 机器人慢速演示一小段并回位 -> 你捏住 grip 朝"臂刚才动的方向"移手/转手 ->
松手。动作幅度大一点(平移≥10cm/旋转≥30°), 方向凭直觉即可, 不需要理解坐标系。
"""
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop_system.backends.factory import make_backend           # noqa: E402
from teleop_system.geometry import axisangle_to_mat, mat_to_axisangle  # noqa: E402

ROBOT_CONFIG = "configs/ur5e.yaml"
TELEOP_CONFIG = "configs/teleop/pico_ur5e.yaml"
LIN_DEMO = 0.10     # 平移演示幅度(m); 单向, 不回位(往返会让人跟错方向)
ROT_DEMO = 0.80     # 旋转演示幅度(rad, ~46°); 单向, 不回位
MOVE_SPEED = 0.08   # 演示速度(m/s), moveL
MIN_HAND_LIN = 0.06 # 有效模仿的最小手柄位移(m)
MIN_HAND_ROT = 0.20 # 有效模仿的最小手柄旋转(rad)
DOMINANCE = 0.55    # 转正后目标轴分量须占的比例
AXIS_NAME = ("x", "y", "z")
ROT_DESC = ("绕x(类似手腕上下翻/俯仰)", "绕y(类似手腕左右翻/侧倾)", "绕z(类似手腕拧螺丝/自转)")


def wait_gesture(backend, side, kind, engage_thr=0.9, release_thr=0.8, timeout=45.0):
    """等一次 捏合->移动->松开, 返回世界系位移 np3(平移) 或 旋转向量 np3."""
    anchor = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        ts = backend.read()
        s = ts.sides.get(side)
        if s is None:
            time.sleep(0.02)
            continue
        if anchor is None:
            if s.grip > engage_thr:
                anchor = (np.array(s.pos), np.array(s.rot))
                print("    [标定] 已捏合, 开始移动 ...", flush=True)
        else:
            if s.grip < release_thr:
                if kind == "lin":
                    return np.array(s.pos) - anchor[0]
                return mat_to_axisangle(np.array(s.rot) @ anchor[1].T)
        time.sleep(0.02)
    print("    [标定] 超时(45s 未完成一次捏合-松开), 跳过本轴", flush=True)
    return None


def get_gesture(backend, side, kind, tries=3):
    """采一次达标幅度的手势向量; 不判方向(方向由调用方在转正后判)."""
    min_mag = MIN_HAND_LIN if kind == "lin" else MIN_HAND_ROT
    for i in range(tries):
        d = wait_gesture(backend, side, kind)
        if d is None:
            return None
        mag = float(np.linalg.norm(d))
        if mag >= min_mag:
            return d
        print(f"    [标定] 动作太小({mag:.3f} < {min_mag}), 幅度大一点重试 ({i+1}/{tries})",
              flush=True)
    return None


def demo_lin(rtde_c, rtde_r, k, lo, hi):
    """机器人沿基座轴 k 单向演示 LIN_DEMO(停在终点, 不回位); 返回演示方向 ±1."""
    p0 = np.array(rtde_r.getActualTCPPose())
    demo_dir = 1.0 if p0[k] + LIN_DEMO + 0.03 <= hi[k] else -1.0
    if demo_dir < 0:
        assert p0[k] - LIN_DEMO - 0.03 >= lo[k], f"轴{AXIS_NAME[k]}两头都出盒, 先挪臂"
    pt = p0.copy()
    pt[k] += demo_dir * LIN_DEMO
    rtde_c.moveL(pt.tolist(), MOVE_SPEED, 0.4)
    return demo_dir


def demo_rot(rtde_c, rtde_r, k):
    """机器人绕基座轴 k 单向演示 +ROT_DEMO(停在终点; 基座系左乘=官方增量语义)."""
    p0 = np.array(rtde_r.getActualTCPPose())
    R0 = axisangle_to_mat(p0[3:6])
    e = np.zeros(3)
    e[k] = ROT_DEMO
    pt = np.concatenate([p0[:3], mat_to_axisangle(axisangle_to_mat(e) @ R0)])
    rtde_c.moveL(pt.tolist(), MOVE_SPEED, 0.4)


def yaw_from_x_gesture(v, demo_dir):
    """x 轴手势的水平分量 -> 偏航角(度): R_z(yaw) 把手势转到 demo_dir*+x."""
    h = v[:2]
    h_mag = float(np.linalg.norm(h))
    if h_mag < 0.6 * np.linalg.norm(v):
        print(f"    [标定] 手势竖直分量过大(水平占比 {h_mag/np.linalg.norm(v):.0%}),"
              " x 轴请尽量水平移动", flush=True)
        return None
    target = 0.0 if demo_dir > 0 else np.pi
    yaw = target - np.arctan2(h[1], h[0])
    yaw_deg = float(np.degrees((yaw + np.pi) % (2 * np.pi) - np.pi))
    return yaw_deg


def judge_sign(v, k, demo_dir, R, what):
    """转正后判轴 k 的符号; 不达标返回 None."""
    va = R @ v
    mag = float(np.linalg.norm(va))
    frac = abs(va[k]) / mag
    if frac < DOMINANCE:
        print(f"    [标定] {what}: 转正后目标轴占比 {frac:.0%} < {DOMINANCE:.0%}, 无效",
              flush=True)
        return None
    sign = 1.0 if demo_dir * va[k] > 0 else -1.0
    print(f"    [标定] {what}: 有效(幅度 {mag:.3f}, 轴占比 {frac:.0%}) -> sign={sign:+.0f}",
          flush=True)
    return sign


def rz(yaw_deg):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def main():
    import rtde_control
    import rtde_receive

    rcfg = yaml.safe_load(Path(ROBOT_CONFIG).read_text())
    host = rcfg["robot"]["host"]
    ws = rcfg["safety"]["workspace"]
    lo = np.array([ws["x"][0], ws["y"][0], ws["z"][0]])
    hi = np.array([ws["x"][1], ws["y"][1], ws["z"][1]])

    tcfg = yaml.safe_load(Path(TELEOP_CONFIG).read_text())
    arm_name, arm = next(iter(tcfg["arms"].items()))
    side = arm.get("side", arm_name)
    pos_scale = arm.get("pos_scale", 0.8)
    rot_scale = arm.get("rot_scale", 1.0)
    engage_threshold = tcfg.get("engage_threshold", 0.9)
    pos_sign = [1.0, 1.0, 1.0]
    rot_sign = [1.0, 1.0, 1.0]
    yaw_deg = 0.0

    print(f"[标定] 连接 {host} ...", flush=True)
    rtde_c = rtde_control.RTDEControlInterface(host)
    rtde_r = rtde_receive.RTDEReceiveInterface(host)
    backend = make_backend("pico", sides=(side,))

    try:
        # ---- ① x 轴: 手势直接解站位偏航(在此之后所有判定都在转正坐标里) ----
        print(f"\n== 平移 x(1/6) ==  观察臂的动作 ...", flush=True)
        while True:
            demo_dir = demo_lin(rtde_c, rtde_r, 0, lo, hi)
            print("    请捏住 grip, 把手柄朝【臂刚才去的方向】水平平移 10cm 以上, 然后松手",
                  flush=True)
            v = get_gesture(backend, side, "lin")
            if v is None:
                print("[标定] x 轴是站位对齐的基准, 没有它无法继续; 重新演示 ...", flush=True)
                continue
            got = yaw_from_x_gesture(v, demo_dir)
            if got is not None:
                yaw_deg = got
                print(f"    [标定] 站位偏航 = {yaw_deg:+.1f}°(已吸收进映射, x 符号恒 +1)",
                      flush=True)
                break
        R = rz(yaw_deg)

        # ---- ② y / z 轴: 转正后判符号(y 顺带捕捉镜像站位) ----
        for k in (1, 2):
            print(f"\n== 平移 {AXIS_NAME[k]}({k+1}/6) ==  观察臂的动作 ...", flush=True)
            demo_dir = demo_lin(rtde_c, rtde_r, k, lo, hi)
            print("    请捏住 grip, 把手柄朝【臂刚才去的方向】平移 10cm 以上, 然后松手",
                  flush=True)
            v = get_gesture(backend, side, "lin")
            s = judge_sign(v, k, demo_dir, R, f"平移{AXIS_NAME[k]}") if v is not None else None
            if s is not None:
                pos_sign[k] = s
            else:
                print(f"    [标定] 平移{AXIS_NAME[k]} 保留 +1(之后手感不对可用 --calibrate 微调)",
                      flush=True)

        # ---- ③ 旋转 3 轴: 同一转正 ----
        for k in range(3):
            print(f"\n== 旋转 {ROT_DESC[k]}({k+4}/6) ==  观察末端的转动 ...", flush=True)
            demo_rot(rtde_c, rtde_r, k)
            print("    请捏住 grip, 把手柄朝【末端刚才转的方向】转 30° 以上, 然后松手\n"
                  "    (不需要标这轴的话, 不捏 grip 等 45s 自动跳过)", flush=True)
            v = get_gesture(backend, side, "rot")
            s = judge_sign(v, k, 1.0, R, f"旋转{AXIS_NAME[k]}") if v is not None else None
            if s is not None:
                rot_sign[k] = s
    finally:
        rtde_c.stopScript()
        try:
            backend.xrt.close()   # 不关会在解释器退出时 abort(gRPC 线程)
        except Exception:
            pass

    print(f"\n[标定] 结果: world_yaw_deg={yaw_deg:+.1f} pos_sign={pos_sign} "
          f"rot_sign={rot_sign}", flush=True)
    Path(TELEOP_CONFIG).rename(TELEOP_CONFIG + ".bak")
    ps = ", ".join(f"{v:.1f}" for v in pos_sign)
    rs = ", ".join(f"{v:.1f}" for v in rot_sign)
    Path(TELEOP_CONFIG).write_text(f"""\
# Pico Ultra 4 -> UR5e 真机(scripts/auto_calibrate_ur5e.py 自动标定写入)
# world_yaw_deg = 头显世界系->基座系的水平对齐角, 跟操作员站位/app 启动朝向绑定,
# 换站位或重启头显 app 后需要重标!
device: pico_ultra4
engage_threshold: {engage_threshold}
save_button: B
discard_button: A
arms:
  right:
    side: right
    pos_scale: {pos_scale}
    rot_scale: {rot_scale}
    pos_sign: [{ps}]
    rot_sign: [{rs}]
    world_yaw_deg: {yaw_deg:.1f}
""")
    print(f"[标定] 已写回 {TELEOP_CONFIG}(旧文件 -> .bak). 用 teleop_record.py 验证手感.",
          flush=True)


if __name__ == "__main__":
    main()
