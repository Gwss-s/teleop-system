#!/usr/bin/env python
"""标定入口: pure Pico -> LIBERO teleop (no policy) to validate / calibrate the mapping.

Drives the LIBERO Panda EE through the 3-layer teleop stack (docs/teleop_usage.md):
    PicoUltra4 (backend) -> EEDeltaMapper (yaml mapping) -> intent_to_osc (OSC)
so a feel-check here validates exactly the path collect_client.py uses.

Controls (right controller): hold GRIP to move; release to hold; TRIGGER = gripper.
In the window:
  q         退出
  +/-       位移增益 ±0.25
  ] / [     旋转增益 ±0.25
  1/2/3     翻转 pos_sign 的 x/y/z（方向标定）
  4/5/6     翻转 rot_sign 的 rx/ry/rz
  s         把当前 signs+scales 存到 --save-config（默认 wrist yaml）
  r         重置场景

腕视角标定用法（相机扰动下方向错乱时）：
  --view wrist（或 both）显示腕部相机；--teleop-config 指向 wrist 配置；边看腕视角边用
  1-6 翻符号直到"手往哪动、腕视角里末端就往哪动"，按 s 存盘。之后采集时用
  collect_client.py --teleop-config configs/teleop/pico_libero_wrist.yaml 启用。

Run (pixi sim env；相机扰动变体用 goal + 对应 task-id):
  PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_test.py \
    --suite libero_goal --task-id 809 --view both \
    --teleop-config configs/teleop/pico_libero_wrist.yaml
"""
import argparse
import time

import numpy as np

from teleop_system.backends.pico_ultra4 import PicoUltra4
from teleop_system.mapping.ee_delta import EEDeltaMapper
from envs.libero.teleop_adapter import intent_to_osc

NOOP = np.array([0, 0, 0, 0, 0, 0, -1], dtype=np.float32)
CTRL_HZ = 20.0
TELEOP_CONFIG = "configs/teleop/pico_libero.yaml"


def save_yaml(cfg, path, engage_threshold=0.9):
    """把当前 ArmConfig 写回 yaml(仅覆盖 sign/scale/阈值,格式与手写一致)。"""
    ps = ", ".join(f"{v:.1f}" for v in cfg.pos_sign)
    rs = ", ".join(f"{v:.1f}" for v in cfg.rot_sign)
    txt = f"""# Pico Ultra 4 -> LIBERO 腕部相机视角校准(teleop_test.py 's' 键写入)
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
"""
    with open(path, "w") as f:
        f.write(txt)
    print(f"[calib] 已存盘 -> {path}  pos_sign=[{ps}] rot_sign=[{rs}] "
          f"pos_scale={cfg.pos_scale} rot_scale={cfg.rot_scale}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_spatial")
    ap.add_argument("--task-id", type=int, default=0)
    ap.add_argument("--teleop-config", default=TELEOP_CONFIG)
    ap.add_argument("--save-config", default="configs/teleop/pico_libero_wrist.yaml",
                    help="'s' 键存盘目标(默认 wrist 配置,不覆盖 agentview 那份)")
    ap.add_argument("--view", choices=("both", "wrist", "agentview"), default="both",
                    help="显示哪个相机(腕视角标定用 wrist 或 both)")
    ap.add_argument("--pos-scale", type=float, default=None)
    ap.add_argument("--rot-scale", type=float, default=None)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    import cv2
    from libero.libero import benchmark
    from libero.libero.envs import OffScreenRenderEnv

    suite = benchmark.get_benchmark_dict()[args.suite]()
    env = OffScreenRenderEnv(bddl_file_name=suite.get_task_bddl_file_path(args.task_id),
                             camera_heights=256, camera_widths=256, ignore_done=True)
    env.seed(args.seed)
    init_states = suite.get_task_init_states(args.task_id)

    def fresh_reset():
        env.reset()
        return env.set_init_state(init_states[0])

    backend = PicoUltra4(sides=("right",))
    mapper = EEDeltaMapper.from_yaml(args.teleop_config,
                                     pos_scale=args.pos_scale, rot_scale=args.rot_scale)
    cfg = mapper.arms["right"]
    print(f"[calib] config={args.teleop_config} view={args.view}", flush=True)
    print("[calib] grip=移动 trigger=夹爪 | +/- 位移增益 [/] 旋转增益 | 1/2/3 翻pos符号 "
          "4/5/6 翻rot符号 | s 存盘 r 重置 q 退出", flush=True)

    def render(o):
        S = 512
        def one(key, label):
            im = cv2.cvtColor(o[key][::-1, ::-1], cv2.COLOR_RGB2BGR)
            im = cv2.resize(im, (S, S))
            cv2.putText(im, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
            return im
        if args.view == "wrist":
            return one("robot0_eye_in_hand_image", "WRIST(腕)")
        if args.view == "agentview":
            return one("agentview_image", "AGENTVIEW(外部)")
        return np.hstack([one("agentview_image", "AGENTVIEW(外部)"),
                          one("robot0_eye_in_hand_image", "WRIST(腕)")])

    obs, gripper_hold, n = fresh_reset(), -1.0, 0
    last_a = np.zeros(7, np.float32)
    try:
        while True:
            beat = time.time()
            intent, _ = mapper.step(backend.read())
            if mapper.engaged("right"):
                a = intent_to_osc(intent, "right"); gripper_hold = a[6]; last_a = a
                if n % 10 == 0:
                    print(f"[calib] HUMAN dpos*={a[:3].round(3)} drot*={a[3:6].round(3)} "
                          f"grip={a[6]:+.0f}", flush=True)
            else:
                a = NOOP.copy(); a[6] = gripper_hold
            obs, _, done, _ = env.step(a); n += 1
            if done:
                obs = fresh_reset(); gripper_hold = -1.0

            view = render(obs)
            eng = mapper.engaged("right")
            cv2.putText(view, "HUMAN(grip)" if eng else "idle", (10, view.shape[0] - 66),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if eng else (150, 150, 150), 2)
            cv2.putText(view, f"pos_sign={[int(v) for v in cfg.pos_sign]} "
                              f"rot_sign={[int(v) for v in cfg.rot_sign]}",
                        (10, view.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
            cv2.putText(view, f"pos x{cfg.pos_scale:.2f} rot x{cfg.rot_scale:.2f} | "
                              f"a=[{last_a[0]:+.2f},{last_a[1]:+.2f},{last_a[2]:+.2f}]",
                        (10, view.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
            cv2.imshow("s4-pico-libero", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k in (ord('+'), ord('=')):
                cfg.pos_scale = round(cfg.pos_scale + 0.25, 2)
            elif k == ord('-'):
                cfg.pos_scale = max(0.25, round(cfg.pos_scale - 0.25, 2))
            elif k == ord(']'):
                cfg.rot_scale = round(cfg.rot_scale + 0.25, 2)
            elif k == ord('['):
                cfg.rot_scale = max(0.25, round(cfg.rot_scale - 0.25, 2))
            elif k in (ord('1'), ord('2'), ord('3')):
                i = k - ord('1'); cfg.pos_sign[i] *= -1
                print(f"[calib] pos_sign -> {[int(v) for v in cfg.pos_sign]}", flush=True)
            elif k in (ord('4'), ord('5'), ord('6')):
                i = k - ord('4'); cfg.rot_sign[i] *= -1
                print(f"[calib] rot_sign -> {[int(v) for v in cfg.rot_sign]}", flush=True)
            elif k == ord('s'):
                save_yaml(cfg, args.save_config, mapper.engage_threshold)
            elif k == ord('r'):
                obs = fresh_reset(); gripper_hold = -1.0

            dt = time.time() - beat
            if dt < 1.0 / CTRL_HZ:
                time.sleep(1.0 / CTRL_HZ - dt)
    except KeyboardInterrupt:
        pass
    finally:
        backend.close(); env.close(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
