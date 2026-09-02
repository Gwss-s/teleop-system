# UR5e 遥操作实验指南(URSim 仿真 → 真机)

> 实验路线:零硬件数据流 → URSim 仿真练习 → 真机。**严格按顺序做**,每步有
> 明确的通过标准。命令都在仓库根目录、`pixi shell` 环境内执行(Linux/Windows
> 同一套命令;有差异的地方用 🪟 标出)。控制原理见 `architecture.md` §4。

## 0. 你在控制什么(2 分钟读懂)

主循环 25Hz:每拍把你的手部增量积分进一个**持久伺服目标**,`servoL` 让机械臂
朝目标运动(没走完的距离下一拍继续追,不丢运动)。三层安全钳制随时生效:

1. **每拍步长限幅**——手甩动/头显跟踪跳变被限速;
2. **工作空间盒**——目标位置永远出不了 `configs/ur5e.yaml` 的 `workspace` 盒;
3. **偏差钳位**——目标最多领先机械臂实际位置 10cm/0.5rad,防止"目标甩飞"。

离合语义:**捏住 grip(手柄为按住 LB)= 移动;松手 = 机械臂钉在原地保持**。
看门狗/保护停后必须松手重捏才能继续(防按键卡死导致失控)。

## 1. 零硬件:重放数据流(通过标准:打印安全钳制统计)

```bash
pixi run demo
```

重放一段真实录制的手柄流,走完整 L1→L2→L3 到内置伺服模拟器。结束时打印:
连续人控段最大单拍步长(应 ≤10mm,说明限幅生效)、最大目标-实测偏差
(应 ≤100mm,说明钳位生效)。

## 2. URSim 仿真(通过标准:浏览器里看到机械臂跟你的输入动)

URSim 是 UR 官方的仿真控制柜——对程序来说它就是一台真 UR5e(同一套 RTDE 接口),
是上真机前的必经练习。需要 Docker。

> 🪟 **Windows**:先装 [Docker Desktop](https://www.docker.com/products/docker-desktop/)
> (自动配好后端);下面命令把 `~/ursim_programs` 换成 `$HOME\ursim_programs`,
> 其余完全一致。

```bash
# 版本用 5.12.x(5.26 与 ur_rtde 1.6.5 不兼容);端口映射 + localhost,双系统通用
mkdir -p ~/ursim_programs
docker run -d --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 \
    -p 30001-30004:30001-30004 -v ~/ursim_programs:/ursim/programs \
    universalrobots/ursim_e-series:5.12.6      # 默认型号即 UR5e
```

1. 浏览器开 `http://localhost:6080/vnc.html` → 弹出 "Confirmation of applied
   Safety Configuration" → 点 **Confirm Safety Configuration**(只需一次,状态
   持久;跳过这步会报 "Failed to start RTDE data synchronization",见 FAQ Q8);
2. 上电+松刹车(脚本化,不用点界面):

```bash
python -c "import dashboard_client,time; d=dashboard_client.DashboardClient('localhost'); d.connect(); d.powerOn(); time.sleep(6); d.brakeRelease(); time.sleep(8); print(d.robotmode())"
# 期望输出 Robotmode: RUNNING
```

3. 遥操作(浏览器 VNC 里看机械臂动):

```bash
# 手柄(键位见 README §4.2):
python envs/ur5e/teleop_record.py --robot-host localhost --backend gamepad
# VR 头显(需先完成 docs/pico_teleop_setup.md 部署):
python envs/ur5e/teleop_record.py --robot-host localhost
# 没有任何设备时,用示例流重放看链路:
python envs/ur5e/teleop_record.py --robot-host localhost \
    --backend replay --tap tests/data/sample_tap.npz --no-gui
```

日常再次启动:`docker start ursim` + 上面第 2 步即可(安全确认不用重做)。

## 3. 真机(必须有指导教师在场)

### 3.1 组网

网线连接 PC 与控制柜底部网口,两端同网段:

- **Linux**:PC 网口设静态 `192.168.10.1/24`(或配 DHCP 服务),控制柜
  (示教器 设置→网络)设 DHCP 或静态 `192.168.10.18`;
- 🪟 **Windows**:控制面板 → 网络适配器 → 以太网 → IPv4 设静态
  `192.168.10.1 / 255.255.255.0`;控制柜设静态 `192.168.10.18`。

通过标准:`ping 192.168.10.18` 通。然后示教器上:开 **Remote Control** 模式、
确认 payload/TCP 设置。控制柜 IP 写进 `configs/ur5e.yaml` 的 `robot.host`。

### 3.2 安全检查(每次换场地必做,教师签字项)

1. **⚠ 核对 `configs/ur5e.yaml` 的 `workspace` 盒**与实际桌面/围栏一致——
   这是最重要的一道防线,仓库里的数值只对应标定时那张桌子。重标方法:
   示教器切手动模式+自由驱动,拖末端扫过预期边界,同时跑:
   ```bash
   python scripts/workspace_calib.py 192.168.10.18
   ```
   把输出的 min/max 写进 yaml(各方向再留 2-3cm 余量);
2. 急停按钮在操作员手边;首跑必须低速(`--pos-scale 0.4`)。

### 3.3 标定(VR 头显;手柄用户跳过)

**首次 / 换站位 / 重启头显 app 后必做**(头显世界系朝向 = app 启动瞬间头的
朝向,斜站位下偏航可达上百度,纯翻符号无解):

```bash
python scripts/auto_calibrate_ur5e.py
```

机械臂逐轴慢速演示一小段,你捏住 grip 朝**同方向**模仿(平移≥10cm/旋转≥30°,
凭直觉即可),脚本自动解出站位偏航角+逐轴符号,写回 `configs/teleop/pico_ur5e.yaml`。
x 轴手势尽量水平移动(它是解偏航的基准)。**启动头显 app 时人面向机器人**,
可让偏航角接近 0。

个别轴不顺手时手动微调(不必整套重标):

```bash
python envs/ur5e/teleop_record.py --robot-host 192.168.10.18 --calibrate
# 键 1/2/3 翻平移符号, 4/5/6 翻旋转符号, +/- 和 [/] 调增益, s 存盘
```

通过标准:"手往哪动,末端就往哪动"。

### 3.4 遥操作与录制

```bash
# 首跑低速;手柄加 --backend gamepad
python envs/ur5e/teleop_record.py --robot-host 192.168.10.18 --pos-scale 0.4
```

- 操作:grip(LB)按住=移动,松手=保持;trigger(X)=夹爪;**B=保存回合,A=作废**;
  q 或 Ctrl-C 退出(自动 servoStop,机械臂原地停);
- 通过标准:自由空间画一个 20cm 见方的立体方框,松手即停、无保护停;
- 录制产物:`outputs/ur5e_demos/episode_*.npz`
  (键:`mode/t_wall/tcp_actual/tcp_target/q/gripper[/cam]`);
  `--camera <N>` 可附带一路 USB 相机画面;
- 夹爪(选配 Robotiq):`configs/ur5e.yaml` 里 `gripper.type: robotiq`,
  经控制柜 63352 端口,与伺服互不干扰。验收:抓放泡沫块。

## 4. 安全行为一览(理解每个"停下来"的含义)

| 触发 | 行为 | 恢复 |
|---|---|---|
| 手甩动/跟踪跳变 | 每拍步长限幅(方向不变,限速) | 自动 |
| 目标出工作空间盒 | 钳到盒边 | 自动 |
| 机械臂跟不上 | 目标最多领先实测 10cm/0.5rad | 自动 |
| 手柄流断/冻结 >0.3s | 看门狗:servoStop + 强制脱开 | 流恢复后**松手重捏** |
| protective/emergency stop | servoStop + 等待解锁 | 示教器解锁 → 程序自动恢复 → **松手重捏**(频发时见 FAQ Q10) |
| q / Ctrl-C / 程序异常 | servoStop → stopScript → 断开 | — |

出问题先查 `docs/faq.md`。
