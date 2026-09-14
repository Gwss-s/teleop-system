# 系统架构与设计说明

> 本文讲清楚这套系统**是什么、为什么这样设计**。读代码前先读这里;
> 动手操作看 `ur5e_setup.md`,设备部署看 `pico_teleop_setup.md`。

## 1. 系统能力

一个通用遥操作框架,当前接入:**UR5e**(仿真 URSim 与真机,`servoL` 伺服)——
本课程的主线;另有 `envs/libero/`(robosuite 仿真,OSC 控制)作为"换机器人只写
一层"的第二个适配示例。四种工作形态:

1. **纯遥操 + 回合录制**(主功能):人 100% 驱动机器人,B/A 键裁决回合存废;
2. **标定**:自动/手动求解坐标系对齐,存 yaml;
3. **重放**:录制的原始设备流(raw tap)可换配置重新映射,不用重新遥操;
4. **策略接管**(进阶能力):自动策略执行中,捏 grip 随时切换为人控。

## 2. 三层架构(核心思想)

设计目标:**N 种输入设备 × M 种机器人 = N+M 份代码,而不是 N×M。**
三层之间的"缝"由 `teleop_system/types.py` 的三个 dataclass 钉死:

```
输入设备 ──L1──> TeleopState ──L2──> ControlIntent + [TakeoverEvent] ──L3──> 机器人动作
(Pico/手柄/重放)  (每手位姿+按键)    (离合/增量/标定,度量单位)          (servoL 或 OSC)
```

| 层 | 位置 | 职责 | 不许知道 |
|---|---|---|---|
| **L1 设备后端** | `teleop_system/backends/` | 只懂设备:读位姿/模拟量/按键,变换到世界系(Z-up) | 机器人、离合、接管语义 |
| **L2 语义映射** | `teleop_system/mapping/ee_delta.py` | 离合锚定→逐拍增量、站位偏航对齐(world_yaw)、迟滞防抖、逐轴符号×增益(yaml)、接管语义(grip阈值→ENGAGE/DISENGAGE)、裁决键(B/A→SAVE/DISCARD) | 具体设备、具体机器人 |
| **L3 机器人适配** | `envs/<robot>/teleop_adapter.py` | 唯一懂该机器人动作约定的文件 | 设备 |

三个数据类型(全部 numpy,详见 `types.py` 源码注释):

- `TeleopState`:sides(每手 pos(3)/rot(3,3)/grip/trigger)+ buttons 原始电平 + 时间戳
  + `aux`(设备的**备用输入**:没被标准映射用掉的按键/摇杆,名→数值,留给扩展);
- `ControlIntent`:每臂 `ArmIntent(dpos 米, drot 弧度轴角, gripper)`——**物理量,与机器人无关**;
  另有可选 `actuators`(自制末端执行器各通道 [0,1],见 5.8);
- `TakeoverEvent`:`ENGAGE/DISENGAGE/SAVE/DISCARD` 四种,全框架唯一的接管词汇表。

**硬约束**:L1/L2 只准依赖 numpy(+yaml)。重依赖(厂商 SDK、ur_rtde、cv2)只能
出现在具体 backend 文件或入口文件里。

### 换设备 / 换机器人的成本

- **换输入设备**:新建 `backends/<device>.py`,实现一个方法
  `read() -> TeleopState`,完。已有两种真实设备证明:Pico(6DoF 跟踪直读)与
  Xbox 手柄(摇杆速率→虚拟位姿积分,约 180 行,下游零改动);
- **换机器人**:新建 `envs/<robot>/teleop_adapter.py`(度量增量→该机器人动作)+
  入口。UR5e 适配器含全部安全钳制约 150 行。

## 3. 目录结构(每个文件夹是哪一层、先看哪个)

```
teleop-system/
├── README.md                 实验路线图(纯导航,不含命令)
├── pixi.toml / pixi.lock     环境定义与锁定(Linux/Windows);pixi run test / demo 任务在这里
│
├── teleop_system/            ★ 框架本体 = L1 + L2(只依赖 numpy + yaml,不碰任何硬件 SDK)
│   ├── types.py              三层之间的"缝":TeleopState / ControlIntent / TakeoverEvent  ← 读代码从这开始
│   ├── geometry.py           旋转数学:四元数↔矩阵↔轴角,XR(Y-up)→世界(Z-up) 变换(单一真源)
│   ├── backends/             L1 设备后端,每种输入设备一个文件,只实现 read() -> TeleopState
│   │   ├── pico_ultra4.py       Pico 头显手柄(6DoF 直读)+ raw tap 录制
│   │   ├── gamepad.py           Xbox 手柄(摇杆速率积分成虚拟位姿)
│   │   ├── tap_replay.py        重放 raw tap(零硬件,测试与 demo 的基础)
│   │   ├── remote.py            跨进程/跨机桥:设备在别的环境或电脑上时用
│   │   └── factory.py           --backend 名字 → 后端类,加设备在这登记
│   ├── mapping/ee_delta.py   L2 语义映射:离合锚定、逐拍增量、站位偏航对齐、符号×增益、
│   │                         接管/裁决事件、自制执行器通道
│   ├── policy/ runtime/      进阶能力(策略接管)用:策略服务器协议、异步 chunk 消费。课程主线不涉及
│   └── legacy/               旧版单文件实现,只用作等价回归测试的对照基准,不要改
│
├── envs/                     ★ L3 机器人适配 + 运行入口,每种机器人一个文件夹
│   ├── ur5e/                 本课程主线
│   │   ├── teleop_adapter.py    度量增量 → servoL 目标位姿,含三重安全钳制(有单测)
│   │   ├── teleop_record.py     遥操 + 录制入口(主循环、看门狗、保护停恢复、HUD、--calibrate)
│   │   ├── gripper.py           Robotiq 夹爪(选配),也是写硬件驱动的参考样例
│   │   └── actuator.py          自制末端执行器驱动壳 ← 课程小组实现的地方
│   └── libero/               第二个适配示例(robosuite 仿真,OSC 控制),证明"换机器人只写 L3";
│                             另含策略接管/评测客户端。仅 Linux,课程主线不涉及
│
├── configs/                  三类配置,详见 configs/README.md
│   ├── ur5e.yaml             机器人参数 + 安全层(workspace 盒!)+ 夹爪/执行器硬件参数
│   ├── gamepad.yaml          手柄键位/速度/型号表
│   └── teleop/<设备>_<机器人>.yaml   映射标定产物(符号/增益/偏航角)+ 可选执行器通道表
│
├── scripts/                  工具,每个独立运行
│   ├── auto_calibrate_ur5e.py   引导式标定(机械臂演示、人模仿)
│   ├── drag_calibrate_ur5e.py   拖动式标定(手柄固定在末端,Kabsch 对齐)
│   ├── workspace_calib.py       freedrive 拖臂扫工作空间边界 → 填 ur5e.yaml
│   ├── pico_probe.py / gamepad_axis_dump.py   看设备原始数据(链路验收、查备用输入名)
│   ├── net_monitor.py / input_age_probe.py    链路延迟体检
│   └── reset_teleop_link.sh / server_libero_policy.sh   链路复位 / 策略服务器(进阶)
│
├── tests/                    pixi run test 跑的全部单测;data/sample_tap.npz 是 demo 用的示例手柄流
├── docs/                     本文 + 三份跟着做的教程 + faq
└── outputs/                  运行时生成(录制的 npz、raw tap),不入库
```

三层对应关系一句话:`teleop_system/backends/` 是 L1,`teleop_system/mapping/` 是 L2,
`envs/<robot>/teleop_adapter.py` 是 L3,`envs/<robot>/teleop_record.py` 把三层串成主循环。

建议阅读顺序:`types.py` → `mapping/ee_delta.py` → `backends/gamepad.py`(最短的
真实后端)→ `envs/ur5e/teleop_adapter.py` → `envs/ur5e/teleop_record.py`。
要接自制执行器,只需要动 `configs/teleop/*.yaml`、`configs/ur5e.yaml` 和
`envs/ur5e/actuator.py` 三处(见 5.8)。

## 4. 数据流(Pico → UR5e)

```
[Pico 4 Ultra 头显 app] --WiFi 90Hz 位姿流--> 同一局域网
[XRoboToolkit PC Service] --gRPC--> pybind 模块 xrobotoolkit_sdk
[L1 PicoUltra4.read()] --XR系→世界系变换--> TeleopState        (25Hz 节拍轮询)
[L2 EEDeltaMapper.step()] --离合+增量+对齐+符号增益--> ControlIntent
[L3 ServoTargetTracker.step()] --持久目标积分+三重钳制--> 目标位姿 [x,y,z,rx,ry,rz]
[ur_rtde servoL(target, lookahead=0.1, gain=300)] --> 机器人侧 500Hz 插值伺服
```

Xbox 手柄路径只换 L1(`backends/gamepad.py` 把摇杆速率积分成虚拟手部位姿),
其余逐层相同。跨进程/跨机形态:设备侧跑 `backends/remote.py --serve`,消费侧
用 `--backend remote` 接收——SDK 与运行环境不兼容、或设备在另一台机器时使用。

## 5. 关键设计决策与理由

### 5.1 持久目标 + 三重钳制(真机控制的核心)

`servoL` 的语义是"朝一个目标位姿伺服":把每拍手部增量**积分进持久目标**,
机械臂没走完的距离下一拍继续追,运动不丢失(这也是官方 XRoboToolkit 真机实现
的锚定语义)。但持久目标必须配**钳制**,否则丢帧/甩手会把目标甩到机械臂追不上
的地方:① 每拍步长限幅(限速);② 工作空间盒(限界);③ 目标-实测偏差钳位
(限领先量)。三者都在 `envs/ur5e/teleop_adapter.py`,有独立单元测试。

(对照:`envs/libero` 的 OSC 接口是逐拍增量式,超限部分被控制器丢弃——本地
仿真闭环下可接受,真机决不可用这种"丢运动"的语义。)

### 5.2 L2 输出物理量(米/弧度),不做归一化

归一化(如 OSC 的 [-1,1])是具体控制器的约定,放进 L2 就把机器人知识泄漏进了
设备无关层。L2 输出物理量,L3 各自换算。好处:同一份标定 yaml 的增益语义在
任何机器人上一致(1.0 = 手动 1cm,末端动 1cm)。

### 5.3 raw tap:录制原始设备流,支持离线重映射

L1 每拍可把**映射前**的原始位姿/模拟量/按键记进 npz(默认开启)。价值:标定、
增益、甚至机器人换了,历史操作数据重放一遍即可重新生成动作流,不用人重新遥操。
重放与在线走同一个坐标变换函数(单一真源),保证逐位一致——这也是
`pixi run demo` 和回归测试的基础。

### 5.4 接管语义事件化

grip 阈值(0.9,官方值)+迟滞防抖 → ENGAGE/DISENGAGE 沿;B/A 上升沿 →
SAVE/DISCARD。所有入口只认这四种事件——任何设备(VR grip、手柄 LB)接入后,
录制器/接管仲裁的行为完全一致。

### 5.5 站位偏航对齐(为什么标定不只是翻符号)

头显世界系的朝向 = app 启动瞬间头的朝向,与机器人基座水平轴一般**斜着差一个
角度**。斜站位下"只朝一个轴动"的手势会散到两个轴上——逐轴符号翻转(镜像)修
不了旋转。所以 L2 先绕 z 轴转正(`world_yaw_deg`),再做逐轴符号×增益。
两种标定法(`scripts/`):

- **引导式**(`auto_calibrate_ur5e.py`):机械臂逐轴演示、人模仿,x 轴手势解
  偏航,其余轴在转正后的坐标里判符号;
- **拖动式**(`drag_calibrate_ur5e.py`):手柄固定在末端,freedrive 拖臂——
  TCP 轨迹与手柄轨迹是同一条曲线在两个坐标系下的表达,中心化后
  SVD(Kabsch)解最优旋转。

### 5.6 真机安全层(仿真没有、真机必须)

除 5.1 的三重钳制外:**看门狗**(设备流断/冻结超时 → servoStop + 强制脱开,
必须松手重捏——防"grip 卡在按下状态"的失控)、**protective stop 轮询**
(触发 → 停 → 等解锁 → 自动恢复+重捏)、**离合语义**(松手 = 目标钉在当前
实测位姿主动保持,不回零、不追旧目标)。行为总表见 `ur5e_setup.md` 实验 4.5。

### 5.7 无 IK 的取舍

本框架在笛卡尔增量层工作,直接用机器人自带控制器(servoL/OSC),不含 IK。
好处:零 URDF/求解器依赖,接新机器人不用建模。代价:接近奇异位形/关节限位时
的行为由机器人控制器兜底(servoL 可能保护停)。缓解:工作空间盒选在远离奇异
的区域。

### 5.8 扩展点:自制末端执行器(舵机组)

课程要在末端装一个自己做的执行器。框架沿三层各留了一个口,把"手柄上剩下的
按键/摇杆 → 每路舵机的指令"接通,**唯独硬件通信留给你写**:

```
L1  TeleopState.aux        后端把没被标准映射占用的输入全部导出(名→数值)。手柄:
                           y/rb/back/start/…/dpad_y;Pico:摇杆、摇杆按下、X/Y、整只左手柄。
                           改绑了离合/夹爪键,被释放的键自动出现在 aux 里
L2  ControlIntent.actuators  yaml `actuators:` 列 N 路通道,每路指定 source(aux 名或一对
                           名)+ mode(hold 跟随 / rate 当速度积分 / toggle 按一下切换)
                           → 每拍算出 (N,) 的 [0,1]。与离合无关:停臂时也能开合工具
L3  envs/ur5e/actuator.py  ServoActuator.command(values) 每拍被调用一次——把 [0,1]
                           变成串口帧/PWM 发给你的驱动板。这是唯一要你实现的文件
```

为什么是 [0,1] 而不是角度:和 5.2 同一个理由——角度范围、正反向是硬件知识,只准
出现在 L3(或 `configs/ur5e.yaml` 的 `actuator:` 块),上游对任何舵机都一样。
按键怎么绑写在 `configs/README.md`;通道值随回合一起录进 npz(`actuators` 键),
所以可以先不接硬件:在 URSim 仿真里按绑好的键,看 `command()` 的打印或 npz 里的
`actuators` 列,验证绑定对不对再动串口。

## 6. 录制数据格式

| 流 | 文件 | 键 |
|---|---|---|
| UR5e 回合 | `episode_*.npz` | `mode(human/idle), t_wall, tcp_actual(6), tcp_target(6), q(6), gripper[, cam(240×320)][, actuators(N)]` |
| 原始设备流 | `raw_tap_*.npz` | `t_wall, side_idx, pose_xr(7), grip, trigger, btn_a, btn_b, ts_dev_ns[, aux_names, aux]` |

裁决规则:B=保存,A=作废。方括号里的键只在对应功能启用时出现。

## 7. 与官方 XRoboToolkit 遥操样例的设计对照

官方 `XRoboToolkit-Teleop-Sample-Python` 是本框架多处默认值的出处
(坐标变换矩阵、grip 阈值 0.9、真机增益 0.8、servoL lookahead/gain)。
架构上的差异:

| 维度 | 官方样例 | 本框架 |
|---|---|---|
| 输入设备 | 仅 Pico(SDK 绑定) | L1 抽象,任何设备一个文件接入 |
| 机器人接入 | 每机器人一个 Controller 类,Placo QP-IK + servoJ,需 URDF | L3 薄适配,直接用机器人自带控制器,无 IK/URDF |
| 录制/重放 | 无重映射概念 | raw tap + 离线重映射 |
| 接管/裁决 | 无 | ENGAGE/DISENGAGE/SAVE/DISCARD 一等公民 |
| 笛卡尔安全层 | 无(靠 IK 正则) | 三重钳制+看门狗+pstop 恢复 |
| 标定 | scale 固定传参 | 偏航角自动求解 + 符号/增益 yaml 工作流 |

## 8. 版本清单

| 组件 | 版本 | 说明 |
|---|---|---|
| XRoboToolkit PC Service | 1.0.0 | Linux .deb / Windows zip,官方 release |
| xrobotoolkit_sdk(pybind) | 1.0.2 | 源码构建,见 `pico_teleop_setup.md` §1 |
| ur_rtde | 1.6.5(PyPI) | UR5e 控制,Linux/Windows 轮子均有 |
| URSim | e-series **5.12.6**(docker) | ⚠不要用 5.26,ur_rtde 版本解析失败 |
| pygame | ≥2.1 | 手柄后端;2.x 映射表 Windows/Linux 一致(官方文档) |
| python / numpy | 3.10 / ≥1.24 | pixi 双平台锁定(`pixi.lock`) |
