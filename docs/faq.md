# 常见问题(FAQ)

> 按"现象 → 原因 → 怎么办"整理。碰到问题先在这里找;找不到再问助教。

## 遥操作手感

**Q1. 手感突然变差 / 机械臂反应慢了一拍以上**
头号原因是**无线链路老化**(路由器/PC Service/头显连续运行数日后整体退化),
不是代码问题,不要去改映射参数。处置:
1. 体检:`python scripts/net_monitor.py --host <头显IP> --no-gui`
   (绿 <30ms 正常;红/丢包 → 下一步);
2. 重启链路:重启 PC Service、头显整机重启、必要时重启路由器,再复查。

> 🪟 **Windows**:`net_monitor` 必须带 `--host`(无自动发现);ping 固定 1Hz。

**Q2. 机械臂无故停顿后猛跟一下**
位姿流瞬时断流(WiFi 抖动)。检查:头显是否戴在额头且摄像头能看见手柄
(手柄 6DoF 追踪依赖头显视野)、头显是否离 PC/路由器太远。录制中碰到这种情况,
该回合按 A 作废。

**Q3. 头显放下一会儿就断连**
头显摘下触发距离传感器 → 睡眠 → WiFi 省电断流。**距离传感器贴一小块胶带**。

**Q4. 手往一个方向动,机械臂却斜着走 / 方向反了**
坐标系没对齐。方向"反了"是符号问题(`--calibrate` 键 1-6 翻);"斜着走"是
站位偏航问题,纯符号修不了 → 重跑 `python scripts/auto_calibrate_ur5e.py`。
**换站位或重启头显 app 后必须重标**(头显世界系朝向 = app 启动瞬间头的朝向)。

**Q5. 屏幕出现橙色 [WATCHDOG] 且机械臂停了**
看门狗保护:手柄数据流断/冻结超过 0.3s,机械臂已 servoStop。这是设计行为
(防"grip 卡在按下状态"的失控)。流恢复后**松开 grip 再重新捏合**即恢复;
频繁触发说明链路差,回 Q1。

## 连接问题

**Q6. 头显 app 连不上 PC**
- 顺序必须是:PC Service → 消费端程序 → 头显 app 连 PC 的 IP → Send On;
- PC 与头显必须在**同一局域网且能互 ping**(校园网有 AP 隔离,用自有路由器);
- PC Service 是单实例,确认没有开第二份。

**Q7. 手柄(Xbox)没反应**
- USB 直插免驱;蓝牙需先在系统设置配对;
- 跑 `python scripts/gamepad_axis_dump.py` 逐轴自查:拨动摇杆看哪个编号在动。
  若编号与 `configs/gamepad.yaml` 的表不一致(非 Xbox 手柄),照实测值在
  `models:` 下加一张表;
- 手柄要**按住 LB(离合)**机械臂才会动,松手即停——这是安全设计,不是故障。

**Q8. URSim 连接报 "Failed to start RTDE data synchronization"**
URSim 的安全配置还没确认:浏览器开 `http://localhost:6080/vnc.html`,点
**Confirm Safety Configuration**(见 `ur5e_setup.md` §2)。另外镜像版本用
5.12.x,不要用 5.26(ur_rtde 1.6.5 解析不了它的版本号)。

**Q9. 真机 ping 不通控制柜**
- 网线插控制柜底部网口;两端 IP 在同一网段(组网方法见 `ur5e_setup.md` §3);
- 示教器上确认已开 **Remote Control** 模式(本地模式下拒绝外部控制)。

## 真机安全

**Q10. 机械臂触发 protective stop(保护性停止)**
程序会提示并自动等待:在示教器上解锁 → 程序自动恢复 → **松手重捏**继续。
短时间频繁触发(错误代码 163)时,UR 要求等 5 秒以上再解锁。
反复触发说明动作太猛或撞到限位:降低增益(`--pos-scale 0.4`)、检查工作空间盒。

**Q11. 为什么必须先核对 `configs/ur5e.yaml` 的 workspace?**
它是伺服目标的硬钳制盒——机械臂的目标永远出不了这个盒。仓库里的数值对应
的是某一张具体桌子,**换场地不改它,保护就是错的**。标定方法:
`python scripts/workspace_calib.py`(freedrive 拖末端扫边界)。

## 其他

**Q12. grip 捏了没反应 / 时灵时不灵**
grip 是 0~1 的模拟量不是开关,阈值 0.9:要**捏到底**。

**Q13. 录制的数据在哪?格式是什么?**
`outputs/` 下,npz 每回合一个文件;键与含义见 `architecture.md` §5。
B=保存本回合,A=作废重来。

**Q14. Windows 终端中文/符号乱码**
`set PYTHONUTF8=1` 后重跑。

**Q15. 想看每一拍的延迟分解**
`python envs/libero/teleop_latency_diag.py --seconds 60` 测流吞吐(stale%/断流);
`python scripts/input_age_probe.py --seconds 60` 测输入龄(数据从头显到 PC 的排队
时间;期间捏几次 grip 会打印"体感接管延迟")。判读:ping 高=链路堵(Q1);
ping 低但输入龄高=重启头显 app 和 PC Service。
