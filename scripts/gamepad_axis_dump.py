#!/usr/bin/env python
"""手柄逐轴/逐键自查工具(跨平台): 实时打印所有 axis/button/hat 的值。

用途: 手柄型号不在 configs/gamepad.yaml models: 表里时,30 秒确认各轴/键序号——
逐个拨动摇杆/扳机/按键,看哪个编号在动,填表即可。也用于验证扳机静止值
(SDL2 惯例: 扳机静止 -1.0,但未触碰前读 0.0——后端已做"触碰守卫")。

运行: python scripts/gamepad_axis_dump.py    (Ctrl-C 退出)
"""
import os
import time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame  # noqa: E402

pygame.init()
pygame.joystick.init()
n = pygame.joystick.get_count()
if n == 0:
    raise SystemExit("未检测到手柄:检查连接(Windows 上 Xbox 手柄免驱;蓝牙需先配对)")
js = pygame.joystick.Joystick(0)
print(f"手柄: \"{js.get_name()}\"  axes={js.get_numaxes()} "
      f"buttons={js.get_numbuttons()} hats={js.get_numhats()}")
print("逐个拨动摇杆/扳机/按键,观察哪个编号变化;Ctrl-C 退出\n")
try:
    while True:
        pygame.event.pump()
        axes = " ".join(f"a{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
        btns = "".join(str(js.get_button(i)) for i in range(js.get_numbuttons()))
        hats = " ".join(f"h{i}:{js.get_hat(i)}" for i in range(js.get_numhats()))
        print(f"\r{axes}  btn:{btns}  {hats}   ", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nbye")
