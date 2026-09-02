# teleop-system — 机器人遥操作教学项目

> 机器人学课程配套项目。**目标:用 Pico 4 Ultra VR 头显和 Xbox 手柄遥操作
> UR5e 机械臂**(先仿真 URSim,后真机),完成回合数据录制。
> Windows 与 Linux 均可完成全部实验,跟着文档一步步做即可。

---

## 1. 你将学到什么

| 主题 | 对应模块 |
|---|---|
| **分层解耦设计**:N 设备 × M 机器人 = N+M 份代码,而不是 N×M | `teleop_system/types.py` 三个接口数据类型 |
| **SO(3) 旋转数学**:四元数/旋转矩阵/轴角,坐标系变换(XR Y-up → 世界 Z-up) | `teleop_system/geometry.py` |
| **离合式增量映射**:锚定、逐拍增量、迟滞防抖、符号/增益标定 | `teleop_system/mapping/ee_delta.py` |
| **坐标系对齐标定**:站位偏航角求解、Kabsch 轨迹对齐 | `scripts/auto_calibrate_ur5e.py`、`scripts/drag_calibrate_ur5e.py` |
| **真机安全设计**:步长限幅、工作空间盒、目标-实测偏差钳位、看门狗 | `envs/ur5e/teleop_adapter.py` |
| **实时控制**:servoL 伺服语义、控制频率与插值、延迟诊断 | `envs/ur5e/teleop_record.py`、`scripts/net_monitor.py` |
| **数据采集**:回合录制、裁决(保存/作废)、raw tap 事后重映射 | `envs/ur5e/teleop_record.py` |

## 2. 三层架构(本项目的核心思想)

```
输入设备 ──L1 backends──> TeleopState ──L2 mapping──> ControlIntent ──L3 adapter──> 机器人动作
 (Pico/手柄/重放)        (每手位姿+按键)   (离合/增量/标定)  (物理量+夹爪)      (servoL)
```

**换输入设备只写 L1**(一个类实现 `read() -> TeleopState`),**换机器人只写 L3**。
接口由 `teleop_system/types.py` 的三个 dataclass 钉死——先读这个文件再读其他代码。
完整设计与理由:`docs/architecture.md`。

## 3. 环境安装(Windows / Linux 同一套 pixi)

```bash
# 装 pixi(一次;装完重开终端):
#   Linux/macOS:  curl -fsSL https://pixi.sh/install.sh | bash
#   Windows(PowerShell):
#                 powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"

git clone git@github.com:Gwss-s/teleop-system.git && cd teleop-system
pixi install          # 双平台锁定:python3.10/numpy/ur_rtde/pygame/opencv
pixi run test         # 24 项单测全过 = 安装 OK
pixi shell            # 进环境(之后所有命令都在这里面跑,无需配任何环境变量)
```

## 4. 实验路线(按顺序做)

### 4.1 零硬件:重放数据流

```bash
pixi run demo    # 重放示例手柄流走完整 L1→L2→L3,结束打印安全钳制统计
```

### 4.2 手柄 + URSim 仿真(第一次亲手遥操)

先按 `docs/ur5e_setup.md` §2 起 URSim(约 5 分钟,浏览器里看机械臂),然后:

```bash
python envs/ur5e/teleop_record.py --robot-host localhost --backend gamepad
# 左摇杆=XY  RT/LT=Z  右摇杆=pitch/yaw  十字键=roll
# LB按住=离合(松手即停)  X按住=夹爪  B=保存回合  A=作废
```

### 4.3 VR 头显 + URSim 仿真

先按 `docs/pico_teleop_setup.md` 完成头显部署(一次性),然后:

```bash
# 顺序固定: PC Service → 本程序 → 头显 app 连 PC 的 IP → Send On
python envs/ur5e/teleop_record.py --robot-host localhost
#   grip=移动 松手=保持 trigger=夹爪 | B=保存 A=作废
```

### 4.4 UR5e 真机(必须有指导教师在场)

**⚠ 安全第一**:上真机前**必须**核对 `configs/ur5e.yaml` 的 `workspace` 盒与
实际工作单元一致;首跑用 `--pos-scale 0.4` 低速;急停按钮在手边。
组网、标定、操作、录制的完整流程:`docs/ur5e_setup.md` §3。

```bash
python scripts/auto_calibrate_ur5e.py                 # ① 标定(VR;换站位后必重做)
python envs/ur5e/teleop_record.py --pos-scale 0.4     # ② 遥操+录制(手柄加 --backend gamepad)
```

## 5. 文档地图(共 4 份,按需读)

| 文档 | 什么时候读 |
|---|---|
| `docs/architecture.md` | 想弄懂系统怎么设计的(读代码前) |
| `docs/ur5e_setup.md` | 做实验时的主指南(URSim → 真机,一步步跟着做) |
| `docs/pico_teleop_setup.md` | 部署 VR 头显链路时(一次性;含 Windows 节) |
| `docs/faq.md` | 碰到问题时(现象→原因→怎么办) |

配置文件的分类与红线:`configs/README.md`。

## 6. 测试与自查

```bash
pixi run test                              # 映射数学等价回归 + 安全钳制 + 手柄,24 项
python scripts/gamepad_axis_dump.py        # 手柄逐轴自查
python scripts/net_monitor.py --host <头显IP> --no-gui   # 头显链路体检(绿 <30ms)
```

手感不对、连不上、报错——先查 `docs/faq.md`。
