# teleop-system — 机器人遥操作教学项目

> 机器人学课程配套项目。**教学目标:用 Pico 4 Ultra VR 头显和 Xbox 手柄遥操作
> UR5e 机械臂**(先仿真 URSim,后真机),完成回合数据录制。核心是一个三层架构的
> 通用遥操作框架;**Windows 与 Linux 均可完成教学主线**(Windows 看
> `docs/setup_windows.md`)。LIBERO 仿真为系统前身,属选修内容(仅 Linux,见附录)。

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
| **数据采集**:回合录制、裁决(保存/作废)、raw tap 事后重映射 | 两个 `teleop_record.py` 入口 |

## 2. 三层架构(本项目的核心思想)

```
输入设备 ──L1 backends──> TeleopState ──L2 mapping──> ControlIntent ──L3 adapter──> 机器人动作
 (Pico/手柄/重放)        (每手位姿+按键)   (离合/增量/标定)  (度量增量+夹爪)   (servoL 或 OSC)
```

| 层 | 职责 | 关键约束 |
|---|---|---|
| **L1 设备后端** `teleop_system/backends/` | 只懂设备:读位姿/模拟量/按键,变换到世界系 | 不懂机器人、离合、接管 |
| **L2 语义映射** `teleop_system/mapping/` | 设备无关+机器人无关:离合锚定→逐拍增量、站位对齐(world_yaw)、符号×增益、接管语义 | 只准依赖 numpy+yaml |
| **L3 机器人适配** `envs/<robot>/teleop_adapter.py` | 唯一懂各机器人动作约定的文件 | UR5e: servoL 目标 + 三重安全钳制;LIBERO: OSC `[-1,1]^7` |

**换输入设备只写 L1**(一个类实现 `read() -> TeleopState`),**换机器人只写 L3**。
接口由 `teleop_system/types.py` 的三个 dataclass 钉死——先读这个文件再读其他代码。

## 3. 环境安装(Windows / Linux 同一套 pixi)

```bash
# Linux / macOS:
curl -fsSL https://pixi.sh/install.sh | bash
# Windows(PowerShell;装完重开终端,细节与差异看 docs/setup_windows.md):
#   powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"

git clone git@github.com:Gwss-s/teleop-system.git && cd teleop-system
pixi install          # 双平台锁定:python3.10/numpy/ur_rtde/pygame/opencv
pixi run test         # 24 项单测全过 = 安装 OK
pixi shell            # 进环境干活(PYTHONPATH 已自动配好,下文命令都在这里面跑)
```

VR 头显链路(仅头显实验需要,一次性):PC Service + APK + pybind SDK,
Linux 照 `docs/pico_teleop_setup.md`;Windows 见 `docs/setup_windows.md` §6。

## 4. 快速上手(按难度递进)

### 4.1 零硬件:重放驱动模拟器(理解数据流)

```bash
pixi run demo    # 重放示例手柄流(tests/data/sample_tap.npz)走完整 L1→L2→L3
```

`--dry-run` 用内置一阶伺服模拟器代替真机;结束打印安全钳制统计(步长压限/偏差钳位)。

### 4.2 手柄 + URSim 仿真(第一次亲手遥操,双系统)

先起 URSim(UR 官方仿真控制柜,浏览器里看机器人):Linux 照 `docs/ur5e_setup.md` §2,
Windows 照 `docs/setup_windows.md` §4。然后:

```bash
python envs/ur5e/teleop_record.py --robot-host localhost --backend gamepad
# 左摇杆=XY  RT/LT=Z  右摇杆=pitch/yaw  十字键=roll
# LB按住=离合(松手即停)  X按住=夹爪  B=保存回合  A=作废
```

### 4.3 VR 头显 + URSim 仿真

```bash
# 顺序固定: PC Service → 本程序 → 头显 app 连 PC 的 IP → Send On
python envs/ur5e/teleop_record.py --robot-host localhost
#   grip=移动 松手=保持 trigger=夹爪 | B=保存 A=作废
```

### 4.4 UR5e 真机(必须有指导教师在场)

**⚠ 安全第一**:上真机前**必须**核对 `configs/ur5e.yaml` 的 `workspace` 盒与
实际工作单元一致;首跑用 `--pos-scale 0.4` 低速;急停按钮在手边。
分步验收流程(离线→URSim 仿真→真机)见 `docs/ur5e_setup.md`;组网(Linux DHCP /
Windows 静态 IP)见 `docs/runbook.md` §2 与 `docs/setup_windows.md` §5。

```bash
# ① 标定(首次/换站位/重启头显 app 后必做): 臂逐轴演示,你捏 grip 模仿,自动解出
#    站位偏航角+逐轴符号(原理见 docs/ur5e_setup.md §3 第 4 条;手柄用户跳过)
python scripts/auto_calibrate_ur5e.py
# ② 遥操+录制(手柄加 --backend gamepad)
python envs/ur5e/teleop_record.py --pos-scale 0.4
```

**启动头显 app 时人面向机器人**——头显世界系朝向 = app 启动瞬间头的朝向,
面向机器人可让站位偏航角接近 0(偏了也能标出来,但没必要)。

### 附录:LIBERO 仿真(系统前身,选修,仅 Linux)

```bash
pixi install -e sim && pixi run -e sim install-libero   # 重依赖(含 torch)
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0 --backend gamepad   # 或不加 --backend 用 Pico
```

## 5. 文档地图

| 文档 | 内容 |
|---|---|
| `docs/architecture.md` | **系统完整介绍**:三层设计/数据流/设计决策与理由/数据格式 ★新同学从这里开始 |
| `docs/setup_windows.md` | **Windows 学生指南**(路线总览/安装/URSim/真机组网/已知差异) |
| `docs/runbook.md` | **日常操作手册**:安装/开工 checklist/录制规范/故障速查 |
| `docs/ur5e_setup.md` | UR5e 部署与分步验收(离线→URSim→真机)★真机前必读 |
| `docs/pico_teleop_setup.md` | Pico 头显部署(PC Service/APK/pybind/组网,Linux) |
| `docs/troubleshooting.md` | 踩坑与调试记录(现象→根因→修复→教训) |
| `docs/teleop_usage.md` | 数据通路与延迟判读 |

## 6. 测试与调试

```bash
pixi run test    # 等价回归(映射数学保险丝) + UR5e 安全钳制 + gamepad,共 24 项
```

手感不对时先跑 `python scripts/net_monitor.py --host <头显IP> --no-gui` 体检
链路——**手感劣化的头号原因是链路老化,不是映射**(教训见
`docs/troubleshooting.md` #1);头显摘下会睡眠断流,距离传感器贴胶带。
手柄轴/键不确定时:`python scripts/gamepad_axis_dump.py` 逐轴自查。
