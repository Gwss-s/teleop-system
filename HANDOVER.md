# teleop-system 总交接文档(HANDOVER)

> **定位:可随时接管策略的通用遥操作框架,主功能是"不接管的纯遥操作 + 回合录制"。**
> 支持 LIBERO 仿真(已标定)与 UR5e 真机(代码+离线验收完成,待真机现场验收)。
> 面向读者:接手继续通用化改造/真机验收的下一个会话/工程师。

---

## 0. 一页速览

```
仿真主入口(纯遥操+录制): envs/libero/teleop_record.py   ← 单机,零服务依赖
仿真标定入口:            envs/libero/teleop_test.py     ← 翻符号/调增益/存 yaml
接管模式(策略+接管+录制): envs/libero/collect_client.py  ← 需策略服务器(5557)
真机入口(UR5e):          envs/ur5e/teleop_record.py     ← ur_rtde/servoL,
                                                          含 --dry-run/--calibrate

三层栈:  输入设备 → L1 teleop_system/backends → L2 mapping → L3 envs/<robot>/teleop_adapter → 执行器
输入设备: --backend pico(头显跟踪) | gamepad(Xbox手柄) | remote(跨进程桥) | replay(tap重放)
设备部署: docs/pico_teleop_setup.md(照做即可,含全部坑)
UR5e:    docs/ur5e_setup.md(分步验收:离线→URSim→空载低速→标定→夹爪→录制)
手感/延迟: docs/teleop_usage.md + scripts/net_monitor.py 体检 + reset_teleop_link.sh 重置
测试:    pixi run test = 等价回归(映射保险丝) + UR5e 安全钳制 + gamepad 后端,共 24 项
```

---

## 1. 三层架构(通用化的地基,已验证可支撑)

三层的"缝"由 `teleop_system/types.py` 的三个 dataclass 定义,
设计目标:**N 个设备 × M 个机器人 = N+M 份代码,而不是 N×M**。

| 层 | 模块 | 职责 | 输出 |
|---|---|---|---|
| **L1 设备后端** | `teleop_system/backends/`(pico_ultra4 / gamepad / remote / tap_replay) | 只懂设备:读位姿/模拟量/按键,XR(Y-up)→世界(Z-up) 变换。**不懂机器人、离合、接管** | `TeleopState`(sides: 每手 pos/rot/grip/trigger;buttons 原始电平;设备时间戳) |
| **L2 语义映射** | `teleop_system/mapping/ee_delta.py`(`EEDeltaMapper`) | 设备无关+机器人无关:离合锚定→逐拍增量、迟滞防抖、逐轴符号×增益(yaml 标定)、接管语义(grip 阈值→ENGAGE/DISENGAGE)、裁决按键(B/A 沿→SAVE/DISCARD) | `ControlIntent`(每臂 dpos/drot **米/弧度度量单位** + gripper;另有 joints 通道备用)+ `TakeoverEvent` 列表 |
| **L3 机器人适配** | `envs/libero/teleop_adapter.py`(OSC)/ `envs/ur5e/teleop_adapter.py`(servoL) | 唯一懂各机器人动作约定的文件 | LIBERO: OSC `[-1,1]^7`;UR5e: servoL 目标位姿(米/旋转向量) |

**硬约束**(`types.py` 模块头):L1/L2 只准依赖 numpy(+yaml);重依赖(SDK/pygame/hidapi/ur_rtde)只能进具体 backend/入口文件。

**数据流(Pico→LIBERO 仿真)**:

```
[Pico Ultra 4 头显 APK] --WiFi 90Hz 位姿流--> 热点 10.42.0.1
[XRoboToolkit PC Service] --gRPC 127.0.0.1:60061--> pybind 模块 xrobotoolkit_sdk
[L1 PicoUltra4.read()] --xr_pose_to_world--> TeleopState
   (可选跨进程: backends/remote.py, TCP 5570, 解决 SDK env 与消费 env 的 ABI 冲突)
[L2 EEDeltaMapper.step()] 离合+增量+符号增益 --> ControlIntent + TakeoverEvent
[L3 intent_to_osc()] --> OSC 动作 [-1,1]^7  → env.step()
```

UR5e 真机把最后一行换成:`ServoTargetTracker.step()`(持久目标+安全钳制)→ `servoL`。
两种 L3 的本质差异与安全设计见 `docs/ur5e_setup.md` §0。

**换输入设备(键盘/SpaceMouse/主从臂)只需**:
1. 新建 `teleop_system/backends/<device>.py`,一个类实现 `read() -> TeleopState`(唯一必写);
2. L2/L3 原样复用;
3. 入口接入(`backends/factory.py` 统一工厂;UR5e 入口与 LIBERO teleop_record 已接 `--backend`)。

---

## 2. 入口功能矩阵

| 入口 | 功能 | 依赖 | 录制 |
|---|---|---|---|
| `envs/libero/teleop_record.py` | **纯遥操**(人 100% 驱动)+ 录制 | 无(单机) | npz(B存/A弃/成功自动存/超时自动弃) |
| `envs/libero/teleop_test.py` | 标定:实时翻轴符号(1-6)/调增益(+-[])/s 存 yaml | 无(单机) | 无 |
| `envs/libero/collect_client.py` | **接管模式**:策略自动跑,捏 grip 接管,松手交还(官方评测协议消费 chunk、时间膨胀、世界暂停重规划、断点续采) | 策略服务器(5557) | npz + 续采进度 |
| `envs/ur5e/teleop_record.py` | **真机纯遥操+录制**;`--dry-run` 零硬件 /`--calibrate` 现场标定 /`--backend replay` 重放 | ur_rtde(真机/URSim) | npz(tcp/q/gripper/相机可选) |

LIBERO npz 格式:`mode/state/a_base/a_applied/a_base_age/speed/wrist/agentview/sim_step`
(纯遥操时 a_base 恒 NOOP);UR5e npz:`mode/t_wall/tcp_actual/tcp_target/q/gripper[/cam]`。

---

## 3. 文件地图

```
teleop-system/
├── README.md / HANDOVER.md          ← 快速开始 / 本文档
├── teleop_system/                   ← 框架包(pip 依赖只有 numpy+yaml)
│   ├── types.py                     ← 三层接口数据类型(TeleopState/ControlIntent/TakeoverEvent)★自研
│   ├── geometry.py                  ← XR→世界系变换 + SO(3) 工具(单一真源,tap重放逐位一致)
│   ├── backends/                    ← L1:pico_ultra4(★SDK官方壳)/gamepad(Xbox)/remote/tap_replay/factory
│   ├── mapping/ee_delta.py          ← L2:离合/增量/迟滞/符号增益/接管语义 ★自研
│   ├── legacy/                      ← 旧单文件实现(等价回归基准,勿改数学)
│   ├── runtime/chunk_source.py      ← 接管模式的 chunk 消费(官方评测协议对位)★自研
│   └── policy/                      ← 策略服协议(8字节长度前缀+pickle)+ π0.5/LingBot 服务器壳
├── envs/libero/                     ← 仿真入口(见 §2)+ eval_client + 延迟探针
├── envs/ur5e/                       ← 真机:teleop_adapter(★安全钳制)/teleop_record/gripper(Robotiq)
├── configs/
│   ├── README.md                    ← 三类配置约定(设备/机器人/映射标定)
│   ├── teleop/pico_libero.yaml      ← 仿真标定产物(agentview;_wrist=腕视角) ⚠符号勿手改
│   ├── teleop/pico_ur5e.yaml        ← 真机映射(官方参照增益,符号待现场标定)
│   └── ur5e.yaml                    ← 机器人/安全参数 ⚠workspace 必须按实际单元核对
├── scripts/                         ← net_monitor(链路体检)/input_age_probe(输入龄)/reset_teleop_link/策略服启动
├── pixi.toml                        ← 环境清单(核心+UR5e 一键;sim feature 为 LIBERO)
├── docs/
│   ├── architecture.md              ← 系统完整介绍 ★新读者从这里开始
│   ├── runbook.md                   ← 日常操作手册(安装/开工/录制规范/故障速查)
│   ├── troubleshooting.md           ← 踩坑与调试记录(11 条)
│   ├── pico_teleop_setup.md         ← 设备部署手册 ★必读
│   ├── ur5e_setup.md                ← UR5e 部署+分步验收 ★真机前必读
│   ├── teleop_usage.md              ← 数据通路/仪表工具箱/延迟判读
│   └── teleop_official_comparison.md← 延迟排查历史归档(增量vs锚定的教训)
└── tests/                           ← 等价回归(映射保险丝) + UR5e 安全钳制单测
```

---

## 4. 开源出处 vs 自研(逐项)

### 来自开源/官方(遵其许可,升级时对官方仓库看齐)

| 内容 | 来源 | 本仓库位置 |
|---|---|---|
| Pico SDK 全链路(APK、PC Service、pybind `xrobotoolkit_sdk`) | **XRoboToolkit**(XR-Robotics org,GitHub;清单见 `docs/pico_teleop_setup.md` 末尾) | 系统依赖,不在仓库内 |
| XR(Y-up)→世界(Z-up) 矩阵 `R_HEADSET_TO_WORLD`、grip>0.9 接管阈值、离合相对模式 | 官方示例 `xrobotoolkit_teleop/common/base_teleop_controller.py`、`utils/geometry.py`(逐字复制) | `teleop_system/geometry.py`、`ee_delta.py` 默认值 |
| 世界系增量左乘合成(`apply_delta_pose`)、真机 pos 缩放 0.8/rot 不缩放、servoJ/L lookahead 0.1 gain 300 | 官方 XRoboToolkit UR5e 真机实现(`hardware/dual_arm_ur_controller.py` 等) | `envs/ur5e/teleop_adapter.py` 语义与默认值 |
| servoL 参数语义/范围、initPeriod-waitPeriod 循环、servoStop→stopScript 收尾、Robotiq 63352 接法 | **ur_rtde 官方文档**(sdurobotics.gitlab.io/ur_rtde) | `envs/ur5e/teleop_record.py`、`gripper.py` |
| OSC_POSE 动作约定(0.05m/0.5rad 满偏)、8 维 state、256² 双相机、`[::-1,::-1]` 图像旋转、`quat2axisangle` | **robosuite / LIBERO / lerobot 官方 LIBERO 集成** | `envs/libero/teleop_adapter.py`、`eval_client.py` |
| π0.5 / LingBot-VLA 模型本体 | lerobot(`pi05_libero_finetuned`)/ Robbyant lingbot-vla-v2(Apache-2.0) | 服务器侧,不在仓库内 |
| 设计思路引用(实现全自研):异步 chunk/时间膨胀/接管仲裁 | HIL-SERL、gym-hil、Sirius、TRANSIC、LeRobot rollout-DAgger、RTC 论文(见 `collect_client.py` 模块头) | — |
| 手柄读取模式(pygame/死区)、型号映射表模式、6DoF 键位约定 | lerobot teleoperators/gamepad(Apache-2.0)、gym-hil controller_config(Apache-2.0)、ur5_teleop_collection(MIT);轴/键序号依据 pygame 官方文档 | `backends/gamepad.py`、`configs/gamepad.yaml` |

### 本仓库自研(改造时可自由重构)

- **三层架构与三个 seam 数据类型**(`types.py` 全部)——通用化的核心资产;
- `EEDeltaMapper`(离合+迟滞+多臂+yaml 标定)、`intent_to_osc`;
- **UR5e 安全层**(`ServoTargetTracker` 三重钳制 + 看门狗 + protective-stop 恢复);
- `remote.py` 跨进程桥、raw tap + `tap_replay.py`(事后重映射不用重新遥操);
- `collect_client.py` 全部回路(硬切换仲裁、影子查询、交还重规划、世界暂停、时间膨胀、
  续采、HUD、fd 泄漏 workaround);`chunk_source.py`;`protocol.py`;
- 标定流(teleop_test / ur5e --calibrate)、两个 teleop_record 入口。

### 历史教训(改造前必读 `docs/teleop_official_comparison.md`)

仿真侧曾因"增量×OSC限幅=运动丢弃"误判开发过锚定模式,最终定论:**手感劣化的真因是
链路老化**(重启后 3ms/0% 丢包),仿真 V1 增量模式是唯一标准。
**但真机侧持久目标(=锚定语义)是官方正确结构**——区别在于 servoL 误差保留 vs OSC
丢弃,且必须配偏差钳位(该文档记录的"目标甩飞"事故就是没钳位的后果)。
文档还留有各层延迟基准(env.step 7.5ms、显示 5.1ms、好手感节拍 50ms)——UR5e
调手感用同一套方法论。

---

## 5. 通用化改造 backlog(给下一个会话)

1. **两套实现并存**:`legacy/base.py`(旧 ABC)与新栈(duck-typing `read()->TeleopState`)
   接口不一致。建议把 `read()` 写成正式 Protocol,legacy 仅作等价测试基准保留;
2. **backend 工厂**:已建 `backends/factory.py`,UR5e 入口与 LIBERO teleop_record
   已接(--backend pico|gamepad|remote|replay);teleop_test/collect_client 待接;
3. **入口重复代码**:四个入口的主循环共性(read→map→adapt→step→pace)可抽
   `TeleopLoop` 骨架,差异注入(动作来源仲裁、录制器、HUD);
4. **录制器独立**:save_episode + B/A 裁决抽成 `EpisodeRecorder`,与 env/机器人解耦;
5. **配置单一化**:CTRL_HZ、相机键、state 组装分散在 LIBERO 入口/eval_client,
   应进 per-robot yaml(UR5e 已做:configs/ur5e.yaml);
6. **UR5e 剩余**:Dashboard 自动解锁 protective stop(163 代码需等5s);双臂支持
   (L2 本就多臂,入口只接了右手);真实相机(RealSense)接入录制。

## 6. UR5e 真机验收状态(接力点)

- ✅ 代码/单测/离线重放验收(真实 tap 1419 拍:连续段步长压限 10mm,偏差 34mm<100mm)
- ✅ URSim 全链路(2026-08-31:真实 tap 驱动仿真 UR5e 移动,无 protective stop,
  看门狗在真实断流段正确触发;首次需 GUI 确认安全配置,攻略见 `docs/ur5e_setup.md` §2)
- ✅ 真机空载低速遥操 + 自动标定(2026-09-01,新机器部署:Pico 全链路 + 真机
  跟手验收通过;新增 L2 `world_yaw_deg` 站位对齐与 `scripts/auto_calibrate_ur5e.py`
  引导式标定——纯符号标定在斜站位下无解,实测偏航 119°,见 runbook §2)
- ⬜ 正式 workspace 扫界(当前 configs/ur5e.yaml 是临时验证盒!用
  `scripts/workspace_calib.py` freedrive 扫边界后写正式值)→ 夹爪 → 录制回路

## 7. 依赖与环境

- **pixi(推荐,新机器)**:`pixi install && pixi run test` 覆盖核心框架+UR5e
  (python3.10/numpy/yaml/cv2/ur_rtde);`pixi install -e sim` 加 robosuite/mujoco,
  再 `pixi run -e sim install-libero` 装 LIBERO-plus。手动部分(Pico SDK/PC Service)
  见 `docs/runbook.md` §0.2;
- 本机历史环境:conda `liberoplus_sim`(py3.10;锁定清单 `docs/liberoplus_sim.freeze.txt`,
  robosuite 1.4.1 / mujoco 3.1.6 / numpy 2.2.6;xrobotoolkit_sdk 1.0.2 与 ur_rtde 1.6.5 已装);
- Pico SDK:pybind `xrobotoolkit_sdk` 1.0.2(源码构建,`docs/pico_teleop_setup.md` §2);
  与消费环境不兼容时用 `backends/remote.py` 跨进程桥(TCP 5570);
- UR5e:`pip install ur_rtde`;URSim 用 docker(5.12.x);
- 策略服务器(仅接管模式):GPU 机上的 lerobot(π0.5)或 lingbot 环境。

## 8. 已知坑速查

| 坑 | 处置 |
|---|---|
| 手感突然变差/延迟秒级 | 链路老化:`scripts/net_monitor.py` 体检 → `reset_teleop_link.sh` + 头显整机重启(不要去改映射!) |
| grip 是模拟量不是布尔 | 官方阈值 0.9;`ee_delta` 有迟滞参数防抖 |
| LIBERO env 重建泄漏 fd | 入口已内置 RLIMIT_NOFILE 提升;千局量级无忧 |
| SDK 与消费环境 ABI 冲突 | `backends/remote.py` publisher/client 跨进程 |
| 相机扰动任务方向错乱 | `teleop_test.py --view wrist` 现场翻符号存 `_wrist.yaml`,或 collect `--ref-cam` |
| 运行顺序 | PC Service 先起 → 消费端 → 头显 app 连 10.42.0.1 → Send On(顺序错=连不上) |
| UR5e 首跑 | ⚠先核对 `configs/ur5e.yaml` workspace;首跑 `--pos-scale 0.4` 空载低速 |
| protective stop 频发(163) | UR 要求等 ≥5s 再解锁;目前走示教器手动解锁 |
