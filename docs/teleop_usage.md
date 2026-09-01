# 数据通路与延迟判读(仪表专篇)

> 定位:回答"延迟发生在哪一段、用哪个仪表看"。
> **日常启动流程在 `runbook.md` §1**(本文不再重复命令);
> 历史排查记录与基准数字见 `troubleshooting.md`。

## 1. 数据通路(哪一段延迟由哪个仪表负责)

```
[Pico Ultra 4 头显]
   | (1) WiFi 空口: PC与头显所在局域网, 位姿流 90Hz  <- net_monitor(ping) 测这段
   v
[XRoboToolkit PC Service]  RoboticsServiceProcess, 官方脚本 runService.sh 启动
   | (2) gRPC 127.0.0.1:60061, 状态缓存
   v
[SDK xrobotoolkit_sdk]                              <- input_age_probe(输入龄) 测 (1)+(2) 合计
   | (3) 每拍 read(): 位姿/grip/trigger/AB/时间戳
   v
[三层遥操栈] backend -> mapping(ee_delta) -> env adapter
   | (4) 机器人动作(LIBERO: OSC [-1,1]^7 / UR5e: servoL 目标位姿)
   v
[执行器: LIBERO env.step ~8-12ms / UR5e servoL] -> [显示 ~5ms]  <- 客户端 HUD 的 perf 字段测 (3)(4)
   |
   +--旁路(仅接管模式): 策略服务器(隧道5557, 影子查询/自动段, 异步不阻塞)
```

**关键事实**:人手→机械臂的通路上只有 (1) 一段真网络;(3)(4) 全在本机
(实测合计 <20ms/拍,预算 50ms)。Xbox 手柄后端无 (1)(2) 段(本地 USB 轮询,
输入龄恒 ~0),延迟问题只可能在 (4)(5)。

## 2. 手感版本史(为什么增量模式是唯一标准)

| 版本 | 形态 | 状态 |
|---|---|---|
| **V1 增量模式** | 逐拍增量 + 离合锚定(pos×4 rot×2, LIBERO 标定值) | **✅ 唯一标准,默认参数** |
| ~~V2 锚定/V3 橡皮筋~~ | 误判"遥操代码导致延迟"时期的改动 | **已删除(2026-08-26 定夺)**。真因=链路开太久整体退化,全链路重启后 V1 完全正常 |

**教训固化**:手感突然变差,先跑链路体检(runbook §1 第 2 步),不要改映射代码。
(真机 UR5e 的持久目标≠这里的 V2:servoL 语义天然误差保留,且带偏差钳位,
见 `architecture.md` §4.1。)

## 3. 仪表工具箱(按"从外到内"排查顺序)

| 工具 | 测什么 | 何时用 |
|---|---|---|
| `scripts/net_monitor.py` | (1) 空口:ICMP ping 实时曲线/丢包(与遥操零耦合,可常开) | **每次开工 30 秒仪式**;手感变差第一查 |
| `scripts/input_age_probe.py --seconds 60` | (1)+(2) 合计:输入龄(样本时间戳 vs 墙钟),捏 grip 打印"沿到达龄"(=体感接管延迟) | ping 干净但仍觉得"隔了一拍"时 |
| `envs/libero/teleop_latency_diag.py --seconds 60` | 流吞吐:stale%/最长断流(量化验收门:stale<2% 且断流<1s) | 正式录制前的验收 |
| `envs/libero/teleop_latency_probe.py` | 端到端分段(三阶段):STREAM 流龄 / AB 新旧栈活体对比 / STEP OSC 阶跃响应 | 深度排查、改栈后的回归 |
| 采集客户端 HUD + `perf_*.jsonl` | (3)(4) 逐拍分阶段耗时(read/map/chunk/sim/disp)、age、stale% | 运行中常态监控;每拍明细落盘复盘 |

## 4. 延迟判读表(分层定位,不再猜)

| 现象 | 结论 | 治法 |
|---|---|---|
| ping 高/丢包(net_monitor 红) | (1) 空口堵。**已证实的头号原因:AP/Service/头显连跑数日后整体退化**(实证:空载 ping 3.8s/52%丢包 → 全链路重启后 3ms/0%) | **`bash scripts/reset_teleop_link.sh`** + 头显整机重启;测量时保持头显清醒(睡眠=WiFi省电=假性堵塞)。持续不行再换 5G 热点 |
| ping 低 但 输入龄高 | (2) Service/App 层积压 | 重启头显 App,再不行重启 PC Service(官方顺序) |
| 输入龄低 但仍觉得跟不上 | (4) 执行器欠账(LIBERO OSC 上限~0.15m/s,手超速) | 放慢手速 / 接管模式的慢动作膨胀 |
| 全部仪表绿 仍觉得卡 | 显示呈现层或体感预期 | 报 perf 行来分析 |

**每次开工前的 30 秒仪式**:开 net_monitor 瞄一眼 → 绿则开工;红则 reset_teleop_link.sh + 头显重启。
