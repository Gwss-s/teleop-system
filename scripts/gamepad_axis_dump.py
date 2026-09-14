#!/usr/bin/env python
"""手柄逐轴/逐键自查工具(跨平台): 实时打印所有 axis/button/hat 的原始值 + 备用输入名。

用途:
  * 手柄型号不在 configs/gamepad.yaml models: 表里时,30 秒确认各轴/键序号——
    逐个拨动摇杆/扳机/按键,看第一行哪个编号在动,填表即可。也用于验证扳机静止值
    (SDL2 惯例: 扳机静止 -1.0,但未触碰前读 0.0——后端已做"触碰守卫");
  * 给自制末端执行器绑键时,第二行 aux 列出所有"备用输入"的名字与实时值——按下
    某个键看哪个名字变成 1,把它填进 configs/teleop/*.yaml 的 actuators: source。
    (已绑给离合/夹爪/保存/作废的键不在 aux 里)

运行: python scripts/gamepad_axis_dump.py    (Ctrl-C 退出)
"""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame  # noqa: E402
import yaml  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from teleop_system.backends.gamepad import GamepadBackend  # noqa: E402

pygame.init()
pygame.joystick.init()
n = pygame.joystick.get_count()
if n == 0:
    raise SystemExit("未检测到手柄:检查连接(Windows 上 Xbox 手柄免驱;蓝牙需先配对)")
js = pygame.joystick.Joystick(0)
print(f"手柄: \"{js.get_name()}\"  axes={js.get_numaxes()} "
      f"buttons={js.get_numbuttons()} hats={js.get_numhats()}")
cfg_path = ROOT / "configs/gamepad.yaml"
backend = GamepadBackend(config=yaml.safe_load(cfg_path.read_text()) if cfg_path.exists() else None,
                         pygame_mod=pygame)
print("前半=原始编号(填 models: 表用);aux: 后=备用输入名(填 actuators: source 用)。"
      "逐个拨动摇杆/扳机/按键观察;Ctrl-C 退出\n")
try:
    while True:
        st = backend.read()                 # 顺带泵事件队列
        axes = " ".join(f"a{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
        btns = "".join(str(js.get_button(i)) for i in range(js.get_numbuttons()))
        hats = " ".join(f"h{i}:{js.get_hat(i)}" for i in range(js.get_numhats()))
        aux = " ".join(f"{k}={v:+.0f}" for k, v in sorted(st.aux.items()))
        print(f"\r{axes}  btn:{btns}  {hats}  | aux: {aux}   ", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nbye")
