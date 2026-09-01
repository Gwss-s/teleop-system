# teleop-system — 三层通用遥操作框架

> **主功能:纯遥操作 + 回合录制**;附带:随时接管一个 VLA 策略(接管模式)。
> 支持 LIBERO 仿真(Franka/OSC,已标定)与 **UR5e 真机**(ur_rtde/servoL,已过 URSim 验收)。

## 文档地图

| 文档 | 内容 |
|---|---|
| `docs/architecture.md` | **系统完整介绍**:三层设计/数据流/设计决策与理由/数据格式/版本清单 |
| `docs/runbook.md` | **日常操作手册**:安装(pixi)/开工 checklist/录制规范/故障速查 |
| `docs/troubleshooting.md` | **踩坑与调试记录**(11 条,现象→根因→修复→教训) |
| `docs/pico_teleop_setup.md` | Pico 设备部署(PC Service/APK/pybind/热点) |
| `docs/ur5e_setup.md` | UR5e 部署与分步验收(离线→URSim→真机) |
| `docs/teleop_usage.md` | 数据通路与延迟判读 |
| `HANDOVER.md` | 交接:现状/开源出处 vs 自研/backlog/接力点 |

## 三层架构(一句话)

`输入设备 →(L1 backends: 设备→TeleopState)→(L2 mapping: 离合/增量/标定→ControlIntent)
→(L3 envs/<robot>/teleop_adapter: →机器人动作)→ 执行器`。
**换设备只写 L1,换机器人只写 L3**(N+M 而非 N×M)。

## 快速开始

```bash
# 环境(新机器,详见 docs/runbook.md §0):
curl -fsSL https://pixi.sh/install.sh | bash
pixi install && pixi run test            # 核心+UR5e;LIBERO 仿真: pixi install -e sim
pixi shell

# 设备部署(一次性): docs/pico_teleop_setup.md
# 开工顺序: runService.sh → 本程序 → 头显 app 连 10.42.0.1 → Send On

# ── LIBERO 仿真:纯遥操+录制(主功能) ──────────────────────────
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0
#   grip=移动 松手=保持 trigger=夹爪 | B=保存 A=作废 | 成功自动保存

# 标定: 1-6 翻轴符号, +/-/[/] 调增益, s 存 yaml
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_test.py \
    --suite libero_goal --task-id 0 --view both

# Xbox 手柄遥操(免头显;LIBERO 与 UR5e 入口均支持 --backend gamepad)
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0 --backend gamepad
#   左摇杆=XY RT/LT=Z 右摇杆=pitch/yaw 十字键=roll | LB按住=离合 X按住=夹爪 | B=保存 A=作废

# 接管模式(策略自动跑,捏 grip 随时接管;需策略服务器)
bash scripts/server_libero_policy.sh <GPU> 5557        # GPU 机
ssh -N -L 5557:localhost:5557 <gpu-host> &             # 本地隧道
PYTHONPATH=$PWD python envs/libero/collect_client.py \
    --host localhost --port 5557 --suite libero_goal --task-id 0

# ── UR5e 真机(分步验收: docs/ur5e_setup.md) ──────────────────
# 第一步永远是离线验证(零硬件):
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py \
    --dry-run --backend replay --tap <raw_tap.npz> --fast --no-gui
# 真机(先核对 configs/ur5e.yaml 的 workspace!):
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --robot-host <控制柜IP>
```

## 测试

```bash
pixi run test
# 或: PYTHONPATH=$PWD python tests/test_teleop_equivalence.py  (映射数学保险丝)
#     PYTHONPATH=$PWD python tests/test_ur5e_adapter.py        (UR5e 安全钳制 7 项)
```

## 手感不对时

先跑 `python scripts/net_monitor.py` 体检链路——**手感劣化的头号原因是链路老化,
不是映射**(教训见 `docs/troubleshooting.md` #1);红了跑
`bash scripts/reset_teleop_link.sh` 并重启头显。
