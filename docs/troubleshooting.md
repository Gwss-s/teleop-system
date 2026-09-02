# 踩坑与调试记录(按事件,持续追加)

> 每条:现象 → 根因 → 修复 → 教训。新坑往后加编号,不删旧条目。
> 相关深度文档:
> UR5e 排障 `ur5e_setup.md` §2。

## #1 手感反复恶化到不可用(2026-08 下旬,头号教训)

- **现象**:遥操延迟逐日恶化,最差秒级;一度怀疑映射代码,开发了"锚定/橡皮筋"模式。
- **根因**:**无线链路"开太久"整体退化**——AP/PC Service/头显连跑数日后,空载 ping
  达 3.8s、丢包 52%;全链路重启后 3ms/0%。
- **修复**:`scripts/reset_teleop_link.sh`(热点+PC Service 重启)+ 头显整机重启;
  锚定模式确认为误判产物,整体删除,增量模式回归唯一标准。
- **教训**:手感变差**先跑 `scripts/net_monitor.py` 体检,不要改映射**。开工前
  30 秒体检仪式写进 runbook。测量时保持头显清醒(睡眠=WiFi 省电=假性堵塞)。

## #2 锚定模式首跑"完全不可用"——增益带错 + 目标甩飞(2026-08-25)

- **现象**:锚定(持久目标)模式首跑机械臂顶死限位零响应。
- **根因**:增量模式的补偿性增益(pos×4/rot×3)被带进锚定模式 → 持久目标被甩出
  工作空间(实测目标-实际误差峰值 1290mm,远超臂展);且当时无偏差钳位。
- **修复**:锚定语义下增益必须 1:1(官方 scale 位置 ≤1.5、旋转从不放大)+
  目标-实际偏差钳位。
- **教训**:**持久目标必须配偏差钳位**——这条直接成为 UR5e 适配器的安全设计
  (`max_target_offset`);增益是"模式相关"的,换控制语义必须重标定。

## #3 增量×OSC 限幅 = 运动丢弃(架构级认知)

- **现象**:手快/丢帧时机械臂永久落后,永不追平。
- **机理**:OSC 单步限幅 ±0.05m,超限部分**丢弃**;丢帧把多拍位移挤进一拍必然超限。
  官方锚定+IK 结构误差保留,只表现为短暂滞后后追平。
- **处置**:仿真里可接受(本地闭环+慢动作膨胀);真机 UR5e 用 servoL 持久目标,
  误差天然保留。逐项对照见 docs/architecture.md §6。

## #4 LIBERO env 重建泄漏 fd → ~10 局后 EMFILE

- **现象**:多回合轮转任务后 MuJoCo 误报 "resource not found"。
- **根因**:OffScreenRenderEnv 每次重建泄漏 ~100 个 fd(EGL/nvidia 句柄,close 不回收),
  默认软上限 1024。
- **修复**:入口启动时 RLIMIT_NOFILE 提到硬上限(已内置);env 重建前 gc + 重试一次。

## #5 SDK pybind 与仿真环境 ABI 冲突

- **现象**:cp312 编译的 `xrobotoolkit_sdk` 进不了 py3.10 环境。
- **修复**:`backends/remote.py` 跨进程桥——L1 在 SDK 环境跑 publisher(TCP 5570),
  消费端 `RemoteBackend.read()` 拿 TeleopState。当前本机两个环境都装了 1.0.2,
  桥是备用路径;新机器上若版本对不齐,桥就是正解。

## #6 相机扰动任务方向错乱

- **现象**:agentview 被扰动旋转后,"手往前推、臂往别处走"。
- **根因**:位置符号标定是**参考视角相关**的(镜像标定)。
- **修复**:`teleop_test.py --view wrist` 以腕视角现场翻符号存 `_wrist.yaml`;
  或接管模式 `--ref-cam` 加固定参考相机,现有标定直接匹配。

## #7 grip 是模拟量不是布尔

- **现象**:阈值附近抖动导致接管/交还风暴(反复 rebase/重规划)。
- **修复**:官方阈值 0.9 + `ee_delta` 迟滞参数(disengage 阈值低于 engage)。

## #8 2.4G 热点间歇性缓冲堆积

- **现象**:输入龄分桶实测 12→186ms 波动,差时段端到端 ~300ms;5G AP 受
  Intel iwlwifi LAR 限制开不出。
- **缓解**:头显额头佩戴(摄像头须看见手柄!)、离主机 1-2m、距离传感器贴胶带
  防熄屏断流;开工前跑 `teleop_latency_diag.py --seconds 60`,stale<2% 且
  最长断流<1s 才开工。录制中撞上断流的回合按 A 作废。

## #9 URSim + ur_rtde:"Failed to start RTDE data synchronization"(2026-08-31)

- **现象**:RTDEReceive 正常、RTDEControl 必超时;verbose 打出
  "PolyScope major version: 0";换 URSim 版本、换 ur_rtde 版本、容器内跑客户端全部无效。
- **定位手法**:写原始 RTDE 协议探针直连 30004 发 GET_URCONTROL_VERSION,拿到明文:
  **"SafetySetup has not been confirmed yet, therefore no data will be send yet"**。
- **根因**:URSim 出厂**安全配置未确认**;用 Dashboard powerOn/brakeRelease 绕过了
  GUI 初始化,导致确认步骤被跳过。Dashboard 的 closeSafetyPopup 关不掉它。
- **修复**(GUI 一次性,状态存容器里):VNC 进 PolyScope → Settings→Password→Safety
  设密码 → Installation→Safety→Robot Limits 解锁 → Apply and restart → 重启后
  点 "Confirm Safety Configuration"。之后上电/松刹车全可 Dashboard 脚本化。
- **教训**:receive 通 ≠ control 通;RTDE 层的真实报错要用原始协议探针看,
  ur_rtde 客户端会把它吞成通用超时。
- **更简修复(2026-09-02)**:起容器时挂一个**空的 programs 卷**
  (`-v ~/ursim_programs:/ursim/programs`),PolyScope 首屏会直接弹确认屏,
  **一键 Confirm 即可,免设密码/解锁全流程**;确认状态随卷持久。上面的密码流程
  仅在确认屏没有自动弹出时使用。

## #10 URSim 5.26 过新,ur_rtde 1.6.5 解析不了

- **现象**:5.26(2026-07 发布)下 ur_rtde 报 PolyScope version 0.0(在 #9 修复之后
  依然如此)。
- **修复**:用 `universalrobots/ursim_e-series:5.12.6`(与 ur_rtde 兼容性久经验证)。

## #11 看门狗在真实数据上的首次触发(2026-08-31,按设计工作)

- **现象**:URSim 全链路验收时,重放的真实 tap 里两段 >0.3s 的连续冻结帧触发看门狗
  (servoStop+强制重捏)。
- **判读**:这些冻结段是 #8 描述的录制时代链路劣化的真实产物——看门狗行为正确。
- **提示**:真机实操若看门狗频繁触发,先按 #1/#8 治链路;`watchdog_timeout`(0.3s)
  在 configs/ur5e.yaml 可调,放宽前先想清楚"冻结时 grip 卡在高位"意味着什么。
