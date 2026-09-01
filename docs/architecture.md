# 系统架构与设计说明(完整介绍)

> 本文是 teleop-system 的系统级介绍:三层架构、数据流、设计决策与理由、
> 数据格式、与官方 XRoboToolkit 遥操样例的对照、版本清单。
> 日常操作看 `runbook.md`;踩坑与调试史看 `troubleshooting.md`。

## 1. 定位与能力

**可随时接管策略的通用遥操作框架。** 四种工作形态:

1. **纯遥操 + 回合录制**(主功能):人 100% 驱动机器人,B/A 键裁决回合存废;
2. **标定**:现场翻轴符号/调增益,存 yaml;
3. **接管模式**:VLA 策略自动执行,捏 grip 随时硬切换为人控,松手交还;
4. **离线重放**:录制的原始设备流(raw tap)可换配置重新映射,不用重新遥操。

已接入:LIBERO/robosuite 仿真(Franka, OSC_POSE)、UR5e 真机(ur_rtde, servoL)。

## 2. 三层架构

设计目标:**N 种输入设备 × M 种机器人 = N+M 份代码,而不是 N×M。**
三层之间的"缝"由 `teleop_system/types.py` 的三个 dataclass 钉死:

```
输入设备 ──L1──> TeleopState ──L2──> ControlIntent + [TakeoverEvent] ──L3──> 机器人动作
```

| 层 | 位置 | 职责 | 不许知道 |
|---|---|---|---|
| **L1 设备后端** | `teleop_system/backends/` | 读设备:位姿/模拟量/按键,设备系→世界系(Z-up)变换,设备时间戳 | 机器人、离合、接管语义 |
| **L2 语义映射** | `teleop_system/mapping/ee_delta.py` | 离合锚定→逐拍增量、迟滞防抖、逐轴符号×增益(yaml)、接管语义(grip阈值→ENGAGE/DISENGAGE)、裁决键(B/A→SAVE/DISCARD) | 具体设备、具体机器人 |
| **L3 机器人适配** | `envs/<robot>/teleop_adapter.py` | 唯一懂该机器人动作约定的文件 | 设备 |

三个数据类型(全部 numpy,详见 `types.py` 源码注释):

- `TeleopState`:sides(每手 pos(3)/rot(3,3)/grip/trigger) + buttons 原始电平 + 双时间戳;
- `ControlIntent`:每臂 `ArmIntent(dpos 米, drot 弧度轴角, gripper)`——**度量单位,与环境无关**;
  另有 `joints` 通道备用(主从臂关节直传形态);
- `TakeoverEvent`:`ENGAGE/DISENGAGE/SAVE/DISCARD` 四种,全框架唯一的接管词汇表。

**硬约束**:L1/L2 只准依赖 numpy(+yaml)。重依赖(厂商 SDK、ur_rtde、cv2、仿真器)
只能出现在具体 backend 文件或入口文件里。

### 换设备 / 换机器人的成本

- **换输入设备**(键盘/SpaceMouse/主从臂):新建 `backends/<device>.py`,实现一个方法
  `read() -> TeleopState`,完。L2/L3 原样复用。已被两种真实设备证明:Pico(6DoF 跟踪)
  与 Xbox 手柄(`backends/gamepad.py`,速率→虚拟位姿积分,~180 行,下游零改动)。
- **换机器人**:新建 `envs/<robot>/teleop_adapter.py`(度量增量 → 该机器人动作)+ 入口。
  参照 UR5e:适配器 ~150 行(含全部安全钳制),入口复用现成模式。

## 3. 数据流

### 3.1 Pico → LIBERO 仿真

```
[Pico Ultra 4 头显 APK] --WiFi 90Hz 位姿流--> PC 热点 10.42.0.1
[XRoboToolkit PC Service] --gRPC 127.0.0.1:60061--> pybind 模块 xrobotoolkit_sdk
[L1 PicoUltra4.read()] --xr_pose_to_world--> TeleopState     (20Hz 节拍轮询)
[L2 EEDeltaMapper.step()] --离合+增量+符号增益--> ControlIntent + TakeoverEvent
[L3 intent_to_osc()] --> OSC_POSE [-1,1]^7 --> env.step()
```

### 3.2 Pico → UR5e 真机

```
...同上到 ControlIntent...                                    (25Hz 节拍)
[L3 ServoTargetTracker.step()] --持久目标积分+三重安全钳制--> 目标位姿[x,y,z,rx,ry,rz]
[ur_rtde servoL(target, dt, lookahead=0.1, gain=300)] --> 机器人侧 500Hz 插值伺服
```

跨进程形态(SDK 与消费端 python ABI 不兼容时):L1 在 SDK 环境跑
`backends/remote.py --serve`(TCP 5570),消费端用 `RemoteBackend` 接收 TeleopState。

## 4. 关键设计决策与理由

### 4.1 仿真用逐拍增量,真机用持久目标——为什么不同

- **LIBERO(OSC_POSE)**:控制器接口本身是逐拍增量([-1,1] 归一化,0.05m/0.5rad 满偏)。
  增量超限部分会被丢弃,但仿真里 20Hz 本地闭环 + 慢动作膨胀下丢弃可忽略;曾误判
  "增量模式导致延迟"而开发锚定方案,最终证明延迟真因是无线链路老化
  (详见 `troubleshooting.md` #1/#2),增量模式回归唯一标准。
- **UR5e(servoL)**:servoL 语义就是"朝持久目标伺服",误差自然保留(没走完的距离
  下一拍继续追,不丢运动)——这与官方 XRoboToolkit 真机实现的锚定语义同构。
  持久目标必须配**偏差钳位**(目标最多领先实测 TCP 10cm/0.5rad),否则丢帧/甩手
  会把目标甩出工作空间(历史事故,见 `troubleshooting.md` #2)。

### 4.2 L2 输出度量单位(米/弧度),不做归一化

归一化是具体控制器的约定(OSC 的 [-1,1]),放进 L2 就把机器人知识泄漏进了设备无关层。
L2 输出物理量,L3 各自换算:LIBERO 除以满偏,UR5e 直接积分。**好处**:同一份标定
yaml 的增益语义在任何机器人上一致(1.0 = 手动 1cm 末端动 1cm)。

### 4.3 raw tap:录制原始设备流,支持离线重映射

L1 每拍可把**映射前**的原始 XR 位姿/模拟量/按键记进 npz(默认开)。价值:
标定改了、增益改了、甚至换机器人,历史操作数据重放一遍即可重新生成动作流,
不用人重新遥操。`tap_replay.py` 与在线后端走同一个 `xr_pose_to_world`(单一真源),
保证重放逐位一致。这也是无真机验证的基础(UR5e 离线/URSim 验收全靠它)。

### 4.4 接管语义事件化

grip 阈值(0.9,官方值)+迟滞防抖 → ENGAGE/DISENGAGE 沿;B/A 上升沿 → SAVE/DISCARD。
所有入口只认这四种事件,不许自造接管判断——保证任何设备(键盘 z/x、VR grip、
拉动主从臂)接入后,录制器/接管仲裁的行为完全一致。

### 4.5 UR5e 安全层(真机必须,仿真没有的东西)

1. **每拍步长限幅**(方向保持):手甩动/XR 跟踪跳变被限速;
2. **工作空间盒**:目标位置硬钳进配置的盒;
3. **偏差钳位**:目标最多领先实测 TCP `max_target_offset`;
4. **看门狗**:设备流断/冻结/输入龄超时 → servoStop + 强制脱开,**必须松手重捏**
   才恢复(防"grip 冻在 0.9"的失控);
5. **protective/emergency stop 轮询**:触发 → servoStop → 等解锁 → reuploadScript
   → 同样要求重捏;
6. **离合语义**:松手 = 目标钉在当前实测位姿并主动保持(不回零、不追旧目标)。

### 4.6 无 IK 的取舍

本框架在笛卡尔增量层工作,直接用机器人自带控制器(OSC/servoL),**不含 IK**。
好处:零 URDF/求解器依赖,接入新机器人不用建模。代价:接近奇异位形/关节限位时
的行为由机器人控制器决定(servoL 可能保护停),不如官方 Placo QP-IK 的
manipulability 正则那样"软着陆"。缓解:工作空间盒把操作限制在远离奇异的区域。

## 5. 录制数据格式

| 流 | 文件 | 键 |
|---|---|---|
| LIBERO 回合 | `episode_*.npz` | `mode(human/auto/idle), state(8), a_base(7), a_applied(7), a_base_age, speed, wrist(256²), agentview(256²), sim_step`(纯遥操时 a_base=NOOP) |
| UR5e 回合 | `episode_*.npz` | `mode, t_wall, tcp_actual(6), tcp_target(6), q(6), gripper[, cam(240×320)]` |
| 原始设备流 | `raw_tap_*.npz` | `t_wall, side_idx, pose_xr(7), grip, trigger, btn_a, btn_b, ts_dev_ns` |

裁决规则:B=保存,A=作废;LIBERO 任务成功自动保存、超时自动作废;UR5e 只认 B/A。

## 6. 与官方 XRoboToolkit 遥操样例的对照

官方 `XRoboToolkit-Teleop-Sample-Python`(XR-Robotics org)是本框架多处默认值的
出处(逐项见 HANDOVER §4)。架构差异:

| 维度 | 官方样例 | 本框架 |
|---|---|---|
| 输入设备 | 仅 Pico(SDK 绑定) | L1 抽象,任何设备一个文件接入 |
| 机器人接入 | 每机器人一个 Controller 类,Placo QP-IK + servoJ,需 URDF | L3 薄适配,直接用机器人自带控制器,无 IK/URDF |
| 跟踪结构 | 锚定绝对目标 | 仿真:逐拍增量;真机:持久目标(同构)+偏差钳位 |
| 录制/重放 | 无重映射概念 | raw tap + 离线重映射 |
| 接管/裁决 | 无 | ENGAGE/DISENGAGE/SAVE/DISCARD 一等公民 |
| 笛卡尔安全层 | 无(靠 IK manipulability 正则) | 三重钳制+看门狗+pstop 恢复 |
| 标定 | scale 固定传参 | 现场翻符号/调增益/存 yaml 工作流 |

## 7. 版本清单(2026-08-31)

| 组件 | 版本 | 说明 |
|---|---|---|
| XRoboToolkit PC Service | 1.0.0(deb, ubuntu 22.04) | github XR-Robotics/XRoboToolkit-PC-Service release v1.0.0 |
| xrobotoolkit_sdk(pybind) | 1.0.2 | 源码构建,装在 `liberoplus_sim` 与 `xr-robotics` 两个 conda 环境 |
| 官方遥操样例(对照基准) | commit `79e5cb8`(2025-12-31) | XRoboToolkit-Teleop-Sample-Python |
| ur_rtde | 1.6.5(PyPI) | UR5e 控制 |
| URSim | e-series **5.12.6**(docker) | ⚠不要用 5.26,ur_rtde 版本解析失败 |
| robosuite / mujoco | 1.4.1 / 3.1.6 | LIBERO 仿真(conda `liberoplus_sim` 锁定清单见 freeze.txt) |
| pygame | ≥2.1(SDL2) | Xbox 手柄后端;键位依据 pygame 官方 joystick 文档 |
| LIBERO-plus | github sylvestf/LIBERO-plus(pip -e) | 仿真任务集 |
