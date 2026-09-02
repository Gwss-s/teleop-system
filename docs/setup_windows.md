# Windows 学生指南(从零到遥操 UR5e)

> 教学主线(手柄/VR → URSim/真机 UR5e)在 Windows 上**原生可行**,不需要 WSL。
> 本文只写 Windows 特有的部分;命令与 Linux 相同的环节直接引用 `runbook.md`。
> Linux 用户看 `runbook.md` §0 即可,不用读本文。

## 0. 路线总览

| 环节 | Windows 方案 | 状态 |
|---|---|---|
| 环境安装 | pixi 原生(见 §1) | ✅ 锁文件已含 win-64 全部依赖 |
| 零硬件数据流演示 | `pixi run demo` | ✅ 跨平台 |
| Xbox 手柄 | 原生免驱,pygame 2.x 映射表**与 Linux 相同** | ✅(轴表 30 秒可自查) |
| URSim 仿真 UR5e | Docker Desktop + 端口映射(见 §4) | ✅ 机制已用同款命令验证 |
| UR5e 真机 | 原生,网线直连(ur_rtde 官方 win 轮子) | ✅ |
| Pico VR 头显 | 官方有 Windows 版 PC Service + pybind | ⚠ 官方支持,本项目未实测(§6) |
| LIBERO 仿真 | ❌ 官方不支持 Windows | 选修内容,需 Linux(README 附录) |

## 1. 安装(PowerShell)

```powershell
# 装 pixi(一次;装完重开终端)
powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"

git clone git@github.com:Gwss-s/teleop-system.git
cd teleop-system
pixi install          # python3.10 + numpy/ur_rtde/pygame/opencv(win-64 已锁定)
pixi run test         # 24 项单测全过 = 安装 OK
pixi shell            # 进环境(PYTHONPATH 已自动配好,后续命令都在这里面跑)
```

建议用 Windows Terminal;若中文/符号输出乱码,先 `set PYTHONUTF8=1`。

## 2. 零硬件演示(第一次看到数据流)

```powershell
pixi run demo    # 重放示例手柄流 -> L1→L2→L3 -> 内置伺服模拟器,打印安全钳制统计
```

## 3. Xbox 手柄

USB 插上即用(免驱);蓝牙需先在系统设置里配对。**键位与 Linux 完全一致**
(pygame 2.x/SDL2 归一化,官方同一张表):左摇杆=XY,RT/LT=Z,右摇杆=pitch/yaw,
十字键=roll,LB 按住=离合,X 按住=夹爪,B=保存,A=作废。

不放心或用了非 Xbox 手柄时,30 秒自查:

```powershell
python scripts/gamepad_axis_dump.py    # 逐个拨动,看哪个编号在动;必要时填 configs/gamepad.yaml
```

## 4. URSim 仿真 UR5e(无真机练习)

装 [Docker Desktop](https://www.docker.com/products/docker-desktop/)(会自动
启用 WSL2 后端,无需手动折腾)。**Windows 上必须用端口映射连 localhost**,
不要用 Linux 文档里的"自定义子网直连容器 IP"方案(Windows/macOS 下宿主机
到容器 IP 不可直达,Docker 已知限制):

```powershell
mkdir $HOME\ursim_programs
docker run -d --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 -p "30001-30004:30001-30004" `
    -v $HOME\ursim_programs:/ursim/programs universalrobots/ursim_e-series:5.12.6
```

1. 浏览器开 `http://localhost:6080/vnc.html` → PolyScope 会弹
   "Confirmation of applied Safety Configuration" → 点 **Confirm Safety
   Configuration**(挂空 programs 卷时是一键确认,无需密码;若没弹出走
   `ur5e_setup.md` §2 的完整流程);
2. 上电+松刹车(免 VNC,脚本化):

```powershell
python -c "import dashboard_client,time; d=dashboard_client.DashboardClient('localhost'); d.connect(); d.powerOn(); time.sleep(6); d.brakeRelease(); time.sleep(8); print(d.robotmode())"
```

3. 手柄遥操仿真臂(浏览器 VNC 里看机器人动):

```powershell
python envs/ur5e/teleop_record.py --robot-host localhost --backend gamepad
```

## 5. UR5e 真机

与 Linux 完全同一套命令(`runbook.md` §2),仅组网方式不同:
- 网线直连时,Windows 侧在 控制面板→网络适配器→以太网→IPv4 设静态
  `192.168.10.1/255.255.255.0`,控制柜(示教器 设置→网络)设静态
  `192.168.10.18`,`ping 192.168.10.18` 通即可;
- 示教器开 Remote Control 模式、payload/TCP 已设(同 Linux);
- 之后标定/遥操命令全部一致:

```powershell
python scripts/auto_calibrate_ur5e.py            # 引导式标定(需 Pico;手柄用户跳过)
python envs/ur5e/teleop_record.py --robot-host 192.168.10.18 --backend gamepad --pos-scale 0.4
```

## 6. Pico VR 头显(Windows,待实测)

官方提供 Windows 组件:PC Service 的 `XRoboToolkit-PC-Service.win.zip`
(github XR-Robotics/XRoboToolkit-PC-Service releases v1.0.0)+ pybind 仓库的
`setup_windows.bat`。**本项目未在 Windows 实测过这条链路**;两条路任选:
- A. 按官方 README 装 Windows 版 PC Service + 构建 pybind,之后入口
  `--backend pico` 与 Linux 完全一致;
- B. 跨机桥接(绕开 Windows SDK 构建):实验室 Linux 机跑
  `python -m teleop_system.backends.remote --serve --backend pico`,
  你的 Windows 机在同一局域网跑入口 `--backend remote --remote-host <Linux机IP>`。

## 7. Windows 已知差异与限制

| 项 | 说明 |
|---|---|
| `scripts/*.sh` | Linux 实验室基础设施(热点重置/策略服),Windows 不适用 |
| `net_monitor.py` | 可用,但需 `--host <头显IP>`(无 Linux 的邻居表自动发现);ping 固定 1Hz(系统限制) |
| `MUJOCO_GL=egl` 前缀 | 教学主线用不到(那是 LIBERO/Linux 的);pixi shell 内也不需要 PYTHONPATH 前缀 |
| LIBERO 仿真 | robosuite 官方仅支持 Linux/macOS;属选修内容,要玩去 Linux 机或 WSL2(手柄需经 remote 桥,WSL2 内核无手柄驱动) |
| 中文输出乱码 | `set PYTHONUTF8=1` |

## 8. 待 Windows 实机确认清单(第一批学生跑通后可删)

- [ ] `pixi install && pixi run test` 全绿(依赖已锁定,预期直过)
- [ ] 手柄轴表:`gamepad_axis_dump.py` 确认与 default 表一致(官方表跨平台,预期一致)
- [ ] URSim Docker Desktop 全流程(端口映射机制已在 Linux 用同款命令验证)
- [ ] Pico Windows 链路(§6 路线 A)
