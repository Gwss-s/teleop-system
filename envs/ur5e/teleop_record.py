#!/usr/bin/env python
"""UR5e 真机纯遥操作 + 回合录制入口(主循环 20-50Hz 算目标,servoL 机器人侧插值)。

三层栈: Pico backend(--backend pico|remote|replay) -> EEDeltaMapper(yaml 标定)
        -> envs/ur5e/teleop_adapter.ServoTargetTracker(持久目标+安全钳制) -> servoL

安全层(全部默认开启,configs/ur5e.yaml 配置):
  * 工作空间盒钳制 + 每拍步长限幅 + 目标-实测偏差钳位(见 teleop_adapter 模块头);
  * 设备流看门狗: read() 异常 / 设备时间戳停走 / 位姿字节冻结超过 watchdog_timeout
    -> servoStop + 强制脱开(必须松手重捏才恢复,防"grip 冻在 0.9"的失控);
  * protective/emergency stop 轮询: 触发 -> servoStop,等待示教器解锁后自动
    reuploadScript 恢复,同样要求重新捏合。
离合语义: 捏 grip 移动;松手 = 目标钉在当前实测位姿(主动保持,不回零);
          B = 保存回合, A = 作废。

分步验收(详见 docs/ur5e_setup.md):
  1) 离线:  --dry-run --backend replay --tap <raw_tap.npz> --fast --no-gui
     (重放已录手柄流,检查目标序列连续性/限幅,零硬件)
  2) URSim: docker 起 universalrobots/ursim_e-series(端口映射),--robot-host localhost
  3) 真机空载低速: --pos-scale 0.4,自由空间画方框
  4) --calibrate 现场翻符号(键 1-6,s 存 configs/teleop/pico_ur5e.yaml)
  5) 加夹爪(configs/ur5e.yaml gripper.type: robotiq) -> 抓放泡沫块

运行(任何装有 ur_rtde 的环境;Pico SDK 不在同环境时用 --backend remote):
  PYTHONPATH=$PWD python envs/ur5e/teleop_record.py \
      --robot-config configs/ur5e.yaml --out-dir outputs/ur5e_demos
"""
import argparse
import time
from pathlib import Path

import numpy as np
import yaml

from teleop_system.mapping.ee_delta import EEDeltaMapper
from teleop_system.types import DISCARD, ENGAGE, SAVE
from envs.ur5e.teleop_adapter import SafetyLimits, ServoTargetTracker, pose_to_pos_rot

TELEOP_CONFIG = "configs/teleop/pico_ur5e.yaml"
ROBOT_CONFIG = "configs/ur5e.yaml"


# ---------------------------------------------------------------------------
# robot interfaces (real ur_rtde vs dry-run simulator, same duck type)
# ---------------------------------------------------------------------------

class URRobot:
    """ur_rtde 薄壳: servoL 伺服 + 状态读取 + 异常恢复。仅此类接触 ur_rtde。"""

    def __init__(self, host, ctrl_hz):
        import rtde_control  # noqa: PLC0415 -- heavy/optional dep stays here
        import rtde_receive  # noqa: PLC0415
        self.rtde_c = rtde_control.RTDEControlInterface(host)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
        self.dt = 1.0 / ctrl_hz

    def tcp_pose(self):
        return np.asarray(self.rtde_r.getActualTCPPose(), dtype=np.float64)

    def q(self):
        return np.asarray(self.rtde_r.getActualQ(), dtype=np.float64)

    def servo(self, target, lookahead, gain):
        # speed/acceleration 参数官方注明"NOT used in current version",占位 0.5
        self.rtde_c.servoL(list(target), 0.5, 0.5, self.dt, lookahead, gain)

    def servo_stop(self):
        try:
            self.rtde_c.servoStop()
        except Exception as e:
            print(f"[ur5e] servoStop failed: {e}", flush=True)

    def faulted(self):
        """(bool, reason) — protective/emergency stop 或 RTDE 断连。"""
        try:
            if self.rtde_r.isEmergencyStopped():
                return True, "EMERGENCY STOP"
            if self.rtde_r.isProtectiveStopped():
                return True, "protective stop"
            if not self.rtde_r.isConnected():
                return True, "RTDE receive disconnected"
        except Exception as e:
            return True, f"receive error: {e}"
        return False, ""

    def recover(self):
        """protective stop 在示教器/Dashboard 解锁后: 重传控制脚本恢复伺服。"""
        try:
            if not self.rtde_r.isConnected():
                self.rtde_r.reconnect()
            self.rtde_c.reuploadScript()
            return True
        except Exception as e:
            print(f"[ur5e] recover failed: {e}", flush=True)
            return False

    def close(self):
        for f in (self.rtde_c.servoStop, self.rtde_c.stopScript,
                  self.rtde_c.disconnect, self.rtde_r.disconnect):
            try:
                f()
            except Exception:
                pass


class DryRunRobot:
    """零硬件模拟器: 实测位姿以受限速度追踪 servo 目标(一阶伺服近似),
    用于离线验证目标序列与偏差钳位行为。"""

    HOME = np.array([0.0, -0.35, 0.30, 0.0, np.pi, 0.0])  # 盒内起始位姿

    def __init__(self, host=None, ctrl_hz=25.0, lin_vel=0.20, rot_vel=1.2):
        self.dt = 1.0 / ctrl_hz
        self._pose = self.HOME.copy()
        self._lin_vel, self._rot_vel = lin_vel, rot_vel

    def tcp_pose(self):
        return self._pose.copy()

    def q(self):
        return np.zeros(6)

    def servo(self, target, lookahead, gain):
        from teleop_system.geometry import axisangle_to_mat, mat_to_axisangle  # noqa: PLC0415
        tp, tr = pose_to_pos_rot(target)
        ap, ar = pose_to_pos_rot(self._pose)
        step = tp - ap
        n = float(np.linalg.norm(step))
        cap = self._lin_vel * self.dt
        if n > cap:
            step *= cap / n
        err = mat_to_axisangle(tr @ ar.T)
        a = float(np.linalg.norm(err))
        rcap = self._rot_vel * self.dt
        if a > rcap:
            err *= rcap / a
        self._pose[0:3] = ap + step
        self._pose[3:6] = mat_to_axisangle(axisangle_to_mat(err) @ ar)

    def servo_stop(self):
        pass

    def faulted(self):
        return False, ""

    def recover(self):
        return True

    def close(self):
        pass


# ---------------------------------------------------------------------------

def make_backend(args):
    from teleop_system.backends.factory import make_backend as factory  # noqa: PLC0415
    tap_path = None
    if args.backend == "pico" and not args.no_raw_tap:
        out = Path(args.out_dir)
        out.mkdir(parents=True, exist_ok=True)
        tap_path = out / f"raw_tap_{int(time.time())}.npz"
    return factory(args.backend, tap_path=tap_path, tap=args.tap,
                   remote_host=args.remote_host, remote_port=args.remote_port,
                   gamepad_config=args.gamepad_config)


def save_episode(buf, out_dir, ep_idx):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"episode_{ep_idx:04d}_{int(time.time())}.npz"
    data = dict(
        mode=np.array([r["mode"] for r in buf]),
        t_wall=np.array([r["t"] for r in buf]),
        tcp_actual=np.stack([r["actual"] for r in buf]),
        tcp_target=np.stack([r["target"] for r in buf]),
        q=np.stack([r["q"] for r in buf]),
        gripper=np.array([r["grip"] for r in buf], dtype=np.float64),
    )
    if buf and buf[0].get("cam") is not None:
        data["cam"] = np.stack([r["cam"] for r in buf])
    np.savez_compressed(path, **data)
    n_h = sum(1 for r in buf if r["mode"] == "human")
    print(f"[ur5e] saved {path.name}: {len(buf)} frames ({n_h} human)", flush=True)


def save_mapping_yaml(cfg, path, engage_threshold):
    ps = ", ".join(f"{v:.1f}" for v in cfg.pos_sign)
    rs = ", ".join(f"{v:.1f}" for v in cfg.rot_sign)
    txt = f"""# Pico Ultra 4 -> UR5e(teleop_record.py --calibrate 's' 键写入)
device: pico_ultra4
engage_threshold: {engage_threshold}
save_button: B
discard_button: A
arms:
  right:
    side: right
    pos_scale: {cfg.pos_scale}
    rot_scale: {cfg.rot_scale}
    pos_sign: [{ps}]
    rot_sign: [{rs}]
    world_yaw_deg: {getattr(cfg, "world_yaw_deg", 0.0):.1f}
"""
    Path(path).write_text(txt)
    print(f"[ur5e] 标定已存盘 -> {path}  pos_sign=[{ps}] rot_sign=[{rs}] "
          f"pos x{cfg.pos_scale} rot x{cfg.rot_scale}", flush=True)


class Watchdog:
    """设备流健康监视: 时间戳停走 / 位姿字节冻结 / read() 异常 -> tripped。"""

    def __init__(self, timeout):
        self.timeout = timeout
        self.tripped = False
        self.reason = ""
        self._last_change = time.time()
        self._prev_bytes = None
        self._age_off = None
        self.age_ms = 0.0

    def check(self, tstate, side="right"):
        now = time.time()
        has_ts = bool(tstate.ts_dev_ns)
        if not has_ts:
            # 无设备时间戳的后端:用"位姿字节冻结"作陈旧代理
            s = tstate.sides.get(side)
            if s is not None:
                pb = s.pos.tobytes() + s.rot.tobytes() + bytes([int(s.grip * 100)])
                if pb != self._prev_bytes:
                    self._prev_bytes = pb
                    self._last_change = now
        else:
            # 有设备时间戳:以输入龄为准。本地轮询设备(手柄)静止时位姿字节
            # 合法地不变,字节冻结在这里只会误报
            diff = now - tstate.ts_dev_ns / 1e9
            self._age_off = diff if self._age_off is None else min(self._age_off, diff)
            self.age_ms = (diff - self._age_off) * 1e3
        frozen = (not has_ts) and (now - self._last_change) > self.timeout
        stalled = has_ts and (self.age_ms / 1e3 > self.timeout)
        if frozen or stalled:
            self.trip("设备流冻结" if frozen else f"输入龄 {self.age_ms:.0f}ms")
        elif self.tripped:
            self.tripped, self.reason = False, ""      # 流恢复(是否重捏由外层管)
        return self.tripped

    def trip(self, reason):
        if not self.tripped:
            print(f"[watchdog] ⚠ {reason} -> servoStop + 脱开(松手重捏恢复)", flush=True)
        self.tripped, self.reason = True, reason


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-config", default=ROBOT_CONFIG)
    ap.add_argument("--robot-host", default=None, help="覆盖 yaml 的 robot.host")
    ap.add_argument("--teleop-config", default=TELEOP_CONFIG)
    ap.add_argument("--save-config", default=TELEOP_CONFIG,
                    help="--calibrate 's' 键存盘目标")
    ap.add_argument("--pos-scale", type=float, default=None)
    ap.add_argument("--rot-scale", type=float, default=None)
    ap.add_argument("--backend", choices=("pico", "gamepad", "remote", "replay"), default="pico")
    ap.add_argument("--gamepad-config", default="configs/gamepad.yaml",
                    help="手柄键位/速度配置(--backend gamepad)")
    ap.add_argument("--remote-host", default="localhost")
    ap.add_argument("--remote-port", type=int, default=5570)
    ap.add_argument("--tap", default="", help="--backend replay 的 raw_tap npz")
    ap.add_argument("--no-raw-tap", action="store_true")
    ap.add_argument("--dry-run", action="store_true",
                    help="零硬件: 用内置一阶伺服模拟器代替真机,打印/录制目标序列")
    ap.add_argument("--fast", action="store_true", help="不按节拍睡眠(离线重放用)")
    ap.add_argument("--calibrate", action="store_true",
                    help="启用标定键: 1-6 翻符号 +/-/[/] 调增益 s 存盘(需 GUI)")
    ap.add_argument("--no-gui", action="store_true", help="无 HUD 窗口(headless)")
    ap.add_argument("--camera", type=int, default=-1,
                    help="录制用相机的 /dev/video 序号; -1 = 不用相机")
    ap.add_argument("--out-dir", default="outputs/ur5e_demos")
    args = ap.parse_args()

    rcfg = yaml.safe_load(Path(args.robot_config).read_text())
    # 手柄后端: 若映射配置仍是 Pico 默认,自动切到手柄标定文件(符号/增益语义不同)
    if args.backend == "gamepad" and args.teleop_config == TELEOP_CONFIG:
        args.teleop_config = "configs/teleop/gamepad_ur5e.yaml"
        if args.save_config == TELEOP_CONFIG:
            args.save_config = args.teleop_config
        print(f"[ur5e] backend=gamepad -> teleop-config 自动切换: {args.teleop_config}", flush=True)
    host = args.robot_host or rcfg["robot"]["host"]
    ctrl_hz = float(rcfg["robot"].get("ctrl_hz", 25.0))
    servo_cfg = rcfg["robot"].get("servo", {})
    lookahead = float(servo_cfg.get("lookahead_time", 0.1))
    gain = float(servo_cfg.get("gain", 300))
    limits = SafetyLimits.from_dict(rcfg.get("safety", {}))
    wd_timeout = float(rcfg.get("safety", {}).get("watchdog_timeout", 0.3))

    robot = DryRunRobot(ctrl_hz=ctrl_hz) if args.dry_run else URRobot(host, ctrl_hz)
    backend = make_backend(args)
    mapper = EEDeltaMapper.from_yaml(args.teleop_config,
                                     pos_scale=args.pos_scale, rot_scale=args.rot_scale)
    acfg = mapper.arms["right"]
    tracker = ServoTargetTracker(limits)
    tracker.reset(robot.tcp_pose())              # 启动即钉在当前位姿(主动保持)
    if not limits.inside_workspace(robot.tcp_pose()):
        print(f"[ur5e] ⚠ 当前 TCP {np.round(robot.tcp_pose()[:3], 3)} 在工作空间盒外!"
              f" 盒={limits.workspace_min}..{limits.workspace_max} — 目标会被缓慢拉回盒内,"
              f"确认 configs 的 workspace 与实际单元一致", flush=True)

    gripper = None
    gcfg = rcfg.get("gripper", {})
    if gcfg.get("type") == "robotiq" and not args.dry_run:
        from envs.ur5e.gripper import RobotiqGripper  # noqa: PLC0415
        gripper = RobotiqGripper(host, port=int(gcfg.get("port", 63352)),
                                 speed=int(gcfg.get("speed", 255)),
                                 force=int(gcfg.get("force", 128)))
        gripper.activate()
        print("[ur5e] Robotiq gripper 激活完成", flush=True)

    cv2 = cam = None
    gui = not args.no_gui
    if gui or args.camera >= 0:
        import cv2  # noqa: PLC0415
    if args.camera >= 0:
        cam = cv2.VideoCapture(args.camera)

    watchdog = Watchdog(wd_timeout)
    dt = 1.0 / ctrl_hz
    out_dir = Path(args.out_dir)
    buf, ep_idx, n_saved, n_discard, steps = [], 0, 0, 0, 0
    need_reengage = False        # 看门狗/故障后必须松手重捏
    was_allowed = False          # 上一拍是否在人控(脱开沿检测用)
    servo_stopped = False        # 看门狗期间只发一次 servoStop
    grip_hold = 0.0
    # dry-run 统计(离线验收: 连续性/限幅证据)
    # 连续人控段内目标步长必须 <= max_lin_step;捏合/脱开沿的重锚定跳变(目标钉回
    # 实测位姿,朝机器人本体收缩,受偏差钳位保护)单独计数,不算连续性违规
    stat_max_step = stat_max_div = 0.0
    stat_reanchor = 0
    prev_target = tracker.target_pose()
    prev_mode = "idle"

    print(f"[ur5e] robot={'DRY-RUN' if args.dry_run else host} ctrl={ctrl_hz}Hz "
          f"lookahead={lookahead} gain={gain}", flush=True)
    print(f"[ur5e] workspace={limits.workspace_min.tolist()}..{limits.workspace_max.tolist()} "
          f"step<={limits.max_lin_step}m/{limits.max_rot_step}rad "
          f"offset<={limits.max_target_offset}m", flush=True)
    print("[ur5e] grip=移动 松手=保持 trigger=夹爪 | B=保存 A=作废 | Ctrl-C 退出", flush=True)

    try:
        while True:
            beat = time.time()
            # ---- L1: 设备读取(看门狗包裹) ----
            try:
                tstate = backend.read()
            except Exception as e:
                watchdog.trip(f"read() 异常: {e}")
                tstate = None
            if tstate is None:
                if args.backend == "replay":
                    break                                  # 重放结束
                robot.servo_stop()
                time.sleep(dt)
                continue
            watchdog.check(tstate)

            # ---- L2: 映射 + 事件 ----
            intent, tevents = mapper.step(tstate)
            kinds = [e.kind for e in tevents]
            if SAVE in kinds and buf:
                save_episode(buf, out_dir, ep_idx)
                buf = []; ep_idx += 1; n_saved += 1
            if DISCARD in kinds and buf:
                print(f"[ur5e] discarded ({len(buf)} frames)", flush=True)
                buf = []; n_discard += 1

            # ---- 机器人故障轮询 ----
            fault, why = robot.faulted()
            if fault:
                robot.servo_stop()
                print(f"[ur5e] ✋ {why} — 在示教器解锁后自动恢复...", flush=True)
                while True:
                    time.sleep(0.5)
                    fault, _ = robot.faulted()
                    if not fault and robot.recover():
                        break
                tracker.reset(robot.tcp_pose())
                need_reengage = True
                print("[ur5e] 已恢复;松手重捏后继续", flush=True)
                continue

            # ---- 接管仲裁(看门狗/故障后强制重捏) ----
            if not mapper.engaged("right"):
                need_reengage = False                      # 已松手,解除强制
            allowed = mapper.engaged("right") and not watchdog.tripped and not need_reengage
            if watchdog.tripped and mapper.engaged("right"):
                need_reengage = True                       # 冻结时 grip 卡在高位: 恢复后仍需重捏

            actual = robot.tcp_pose()
            if ENGAGE in kinds and allowed:
                tracker.reset(actual)                      # 捏合沿: 锚定,首拍增量 0
            if allowed and "right" in intent.arms:
                target = tracker.step(intent.arms["right"], actual)
                grip_hold = intent.arms["right"].gripper
                mode = "human"
                was_allowed = True
            else:
                if was_allowed:
                    tracker.reset(actual)                  # 脱开沿: 目标钉在当前实测位姿
                    was_allowed = False                    # 之后主动保持这个冻结目标
                target = tracker.target_pose()
                mode = "idle"

            if watchdog.tripped:
                if not servo_stopped:
                    robot.servo_stop()
                    servo_stopped = True
            else:
                if servo_stopped:                          # 流恢复: 重新锚定再伺服
                    tracker.reset(actual)
                    target = tracker.target_pose()
                    servo_stopped = False
                robot.servo(target, lookahead, gain)
            if gripper is not None:
                gripper.move_trigger(grip_hold)

            # ---- 录制 ----
            frame = None
            if cam is not None:
                ok, im = cam.read()
                frame = cv2.resize(im, (320, 240)) if ok else None
            buf.append(dict(mode=mode, t=beat, actual=actual,
                            target=target.copy(), q=robot.q(),
                            grip=grip_hold, cam=frame))
            steps += 1
            step_mm = float(np.linalg.norm(target[:3] - prev_target[:3]))
            if mode == "human" and prev_mode == "human":
                stat_max_step = max(stat_max_step, step_mm)
            elif step_mm > 1e-9:
                stat_reanchor += 1
            stat_max_div = max(stat_max_div, float(np.linalg.norm(target[:3] - actual[:3])))
            prev_target, prev_mode = target, mode

            # ---- HUD / 标定键 ----
            if gui:
                S = 560
                view = np.zeros((300, S, 3), np.uint8)
                if frame is not None:
                    view = cv2.resize(frame, (S, int(S * frame.shape[0] / frame.shape[1])))
                    view = np.vstack([view, np.zeros((150, S, 3), np.uint8)])
                col = (0, 0, 255) if mode == "human" else (170, 170, 170)
                if watchdog.tripped or need_reengage:
                    col = (0, 165, 255)
                y0 = view.shape[0] - 130
                for i, (txt, c) in enumerate([
                    (f"{'HUMAN' if mode == 'human' else 'idle'}"
                     + ("  [WATCHDOG]" if watchdog.tripped else "")
                     + ("  [RE-ENGAGE]" if need_reengage else ""), col),
                    (f"step {steps}  saved {n_saved}  disc {n_discard}  "
                     f"age {watchdog.age_ms:.0f}ms", (230, 230, 230)),
                    (f"tcp  {np.round(actual[:3], 3).tolist()}", (200, 255, 200)),
                    (f"tgt  {np.round(target[:3], 3).tolist()}  "
                     f"div {1e3 * np.linalg.norm(target[:3] - actual[:3]):.0f}mm", (200, 255, 200)),
                    (f"pos_sign={[int(v) for v in acfg.pos_sign]} "
                     f"rot_sign={[int(v) for v in acfg.rot_sign]} "
                     f"x{acfg.pos_scale:.2f}/{acfg.rot_scale:.2f}", (255, 255, 0)),
                ]):
                    cv2.putText(view, txt, (10, y0 + 25 * i),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 1)
                cv2.imshow("ur5e-teleop", view)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    raise KeyboardInterrupt
                if args.calibrate:
                    if k in (ord('+'), ord('=')):
                        acfg.pos_scale = round(acfg.pos_scale + 0.1, 2)
                    elif k == ord('-'):
                        acfg.pos_scale = max(0.1, round(acfg.pos_scale - 0.1, 2))
                    elif k == ord(']'):
                        acfg.rot_scale = round(acfg.rot_scale + 0.1, 2)
                    elif k == ord('['):
                        acfg.rot_scale = max(0.1, round(acfg.rot_scale - 0.1, 2))
                    elif k in (ord('1'), ord('2'), ord('3')):
                        acfg.pos_sign[k - ord('1')] *= -1
                        print(f"[calib] pos_sign -> {[int(v) for v in acfg.pos_sign]}", flush=True)
                    elif k in (ord('4'), ord('5'), ord('6')):
                        acfg.rot_sign[k - ord('4')] *= -1
                        print(f"[calib] rot_sign -> {[int(v) for v in acfg.rot_sign]}", flush=True)
                    elif k == ord('s'):
                        save_mapping_yaml(acfg, args.save_config, mapper.engage_threshold)

            if not args.fast:
                el = time.time() - beat
                if el < dt:
                    time.sleep(dt - el)
    except KeyboardInterrupt:
        pass
    finally:
        robot.servo_stop()
        for closer in (robot.close, backend.close,
                       (gripper.close if gripper is not None else lambda: None),
                       (cam.release if cam is not None else lambda: None)):
            try:
                closer()
            except Exception:
                pass
        if cv2 is not None:
            cv2.destroyAllWindows()
        print(f"[ur5e] 结束: beats={steps} saved={n_saved} discarded={n_discard} "
              f"未裁决缓存 {len(buf)} 帧", flush=True)
        if args.dry_run:
            print(f"[dry-run] beats={steps} 连续人控段最大单拍步长={1e3 * stat_max_step:.1f}mm "
                  f"(上限 {1e3 * limits.max_lin_step:.0f}mm) "
                  f"重锚定跳变 {stat_reanchor} 次(受偏差钳位保护,非违规) "
                  f"最大目标-实测偏差={1e3 * stat_max_div:.1f}mm "
                  f"(上限 {1e3 * limits.max_target_offset:.0f}mm)", flush=True)
            if buf:
                save_episode(buf, out_dir, ep_idx)


if __name__ == "__main__":
    main()
