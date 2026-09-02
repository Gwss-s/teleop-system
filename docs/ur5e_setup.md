# 实验指南:遥操作 UR5e(实验 1-4)

> **前提:已完成 `setup.md`(实验 0)**。本文所有命令都在
> "开终端 → `cd` 进项目目录 → `pixi shell`"之后执行(setup.md §0.7 的三步)。
> 每个实验都写了"预期现象"和"通过标准";卡住看括号里的 FAQ 编号(`docs/faq.md`)。
> 全文 🪟 = 只在 Windows 做,🐧 = 只在 Linux 做,没有标记 = 两个系统都一样。

## 0. 你在控制什么(2 分钟读懂再动手)

程序每秒 25 拍:每拍把你的手部移动量累加进一个**伺服目标**,机械臂持续朝目标
运动(没走完的距离下一拍继续追,不丢运动)。三层安全钳制随时生效:

1. **每拍步长限幅**——手甩太快会被限速;
2. **工作空间盒**——目标位置永远出不了 `configs/ur5e.yaml` 里划定的盒子;
3. **偏差钳位**——目标最多领先机械臂实际位置 10cm,防止"目标甩飞"。

离合语义:**捏住 grip(手柄=按住 LB)= 机械臂动;松手 = 停在原地**。
看门狗/保护停触发后,必须**松手再重捏**才能继续(防按键卡死失控)。
更多设计原理见 `architecture.md`。

---

## 实验 1:零硬件,重放数据流(5 分钟)

不需要任何硬件。重放一段真实录制的手柄操作,走完整的三层管线到内置模拟器:

```bash
pixi run demo    # 用示例手柄数据跑完整遥操流程(内置模拟器代替真机),结束打印安全钳制统计
```

**预期**:滚动打印若干行后,结尾出现类似
`[dry-run] beats=1279 连续人控段最大单拍步长=10.0mm (上限 10mm) ... 最大目标-实测偏差=100.0mm (上限 100mm)`。

**通过标准**:两个统计值都不超过括号里的上限——这就是安全钳制在起作用。
(报"找不到 python" → 没进 pixi shell,回 setup.md §0.7;详见 FAQ Q17。)

---

## 实验 2:手柄遥操 URSim 仿真机械臂(约 30 分钟)

URSim 是 UR 官方的"仿真控制柜"——对程序来说它就是一台真 UR5e。
需要:Docker 已装好(setup.md §0.8)、Xbox 手柄。

### 2.1 启动 URSim(首次)

**🐧 Linux**:

```bash
mkdir -p ~/ursim_programs    # 建一个目录,让仿真器把设置存在这里(下次启动不用重新确认)
# 下载并后台启动 URSim 仿真控制柜,把它的各端口映射到本机 localhost
docker run -d --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 -v ~/ursim_programs:/ursim/programs universalrobots/ursim_e-series:5.12.6
```

**🪟 Windows**(先确认 Docker Desktop 正在运行——左下角绿色):

```powershell
mkdir $HOME\ursim_programs   # 建一个目录存仿真器设置(提示"已存在"可忽略)
# 下载并后台启动 URSim 仿真控制柜,端口映射到本机 localhost(注意端口段带引号)
docker run -d --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 -p "30001-30004:30001-30004" -v $HOME\ursim_programs:/ursim/programs universalrobots/ursim_e-series:5.12.6
```

**预期**:首次会下载镜像(约 1GB,几分钟),结束打印一长串字母数字(容器 ID)。
验证:

```bash
docker ps        # 列出正在运行的容器;能看到一行 ursim ... Up 即启动成功
```

(报 "Cannot connect to the Docker daemon" → Docker 没在运行,见 FAQ Q16。
版本必须用 5.12.6,不要换新版,原因见 FAQ Q8。)

### 2.2 确认安全配置(仅首次,点一下就好)

浏览器打开 `http://localhost:6080/vnc.html` → 点 **Connect** → 看到机械臂操作
界面(叫 PolyScope),正中弹着 "Confirmation of applied Safety Configuration"
窗口 → 点底部的 **Confirm Safety Configuration** 按钮。这个网页标签页先留着别关。

### 2.3 上电 + 松刹车(每次启动 URSim 后都要做一次)

回到终端(在 pixi shell 里):

```bash
# 给仿真机械臂上电并松开抱闸,让它进入可控状态
python -c "import dashboard_client,time; d=dashboard_client.DashboardClient('localhost'); d.connect(); d.powerOn(); time.sleep(6); d.brakeRelease(); time.sleep(8); print(d.robotmode())"
```

**预期**:等约 15 秒,最后打印 `Robotmode: RUNNING`。

### 2.4 手柄遥操!

手柄用 USB 插上,然后:

```bash
# 用手柄遥操 URSim 里的仿真机械臂(--robot-host localhost 指向仿真器)
python envs/ur5e/teleop_record.py --robot-host localhost --backend gamepad
```

**键位**:`左摇杆`=前后左右,`RT/LT`=上/下,`右摇杆`=俯仰/偏航,`十字键←→`=滚转;
**按住 `LB`=离合(松手立刻停)**;按住 `X`=夹爪合;`B`=保存本回合;`A`=作废;`q`=退出。

**预期**:终端打印 `[gamepad] "..." 已连接`;按住 LB 推左摇杆,浏览器 VNC 画面里
的机械臂跟着动。
**通过标准**:能画一个大致的立体方框,松开 LB 机械臂立刻停。
(手柄没反应 → FAQ Q7;方向不顺手是正常的,真机实验才需要标定。)

### 2.5 收尾与日常再启动

```bash
docker stop ursim     # 停止仿真器(结束当天实验时)
docker start ursim    # 下次再玩:先启动仿真器,再做一次 §2.3 上电(安全确认不用重做)
```

录下的数据在 `outputs/ur5e_demos/`(按过 B 才有保存)。

---

## 实验 3:VR 头显遥操 URSim(约 1 小时,含设备部署)

1. **部署头显链路**(一次性,装好后以后跳过):按 `pico_teleop_setup.md`
   从头做到底,里面的探针验收通过后再回来;
2. 启动 URSim 并上电(同 §2.1-§2.3;已确认过安全配置就跳过 §2.2);
3. **按固定顺序启动**(顺序错了连不上,见 FAQ Q6):
   ① 先在 PC 上启动 PC Service → ② 再运行下面的命令 → ③ 头显开 app 连 PC 的 IP
   → ④ 头显面板把 Send 拨到 On:

```bash
# 用 VR 头显遥操 URSim 里的仿真机械臂(不加 --backend,默认用 Pico 头显)
python envs/ur5e/teleop_record.py --robot-host localhost
```

**操作**:捏住手柄侧面的 **grip 键**(要捏到底)=移动,松手=停;**扳机**=夹爪;
`B`=保存回合,`A`=作废。
**通过标准**:同实验 2——画方框、松手即停。方向不对是正常的(下个实验标定)。

---

## 实验 4:UR5e 真机(必须有指导教师在场!)

### 4.1 组网(接网线)

网线一头插控制柜底部网口,一头插电脑,两端设成同一网段:

- 🐧 **Linux**:电脑网口设静态 IP `192.168.10.1`,子网掩码 `255.255.255.0`;
- 🪟 **Windows**:控制面板 → 网络和 Internet → 网络连接 → 右键"以太网" →
  属性 → 双击"Internet 协议版本 4" → 选"使用下面的 IP 地址",填
  IP `192.168.10.1`、掩码 `255.255.255.0`,确定;
- 示教器(机械臂的手持平板):设置 → 网络 → 静态地址,IP 填 `192.168.10.18`,
  掩码 `255.255.255.0`。

验证(电脑终端):

```bash
ping 192.168.10.18     # 测试能否连通控制柜;有回复即网络通(不通见 FAQ Q9)
```

示教器上还要:打开 **Remote Control**(远程控制)模式;确认 payload/TCP 已按实际
末端设置(请助教确认)。

### 4.2 安全检查(每次换场地必做,教师确认项)

1. **⚠ 核对 `configs/ur5e.yaml` 的 `workspace` 盒**与实际桌面/围栏一致——这是最
   重要的一道防线,配置里的数值只对应标定它时的那张桌子。重标方法:示教器切手动
   模式并开自由驱动,一人拖着机械臂末端扫过工作范围的边界,同时电脑上跑:

   ```bash
   # 记录机械臂末端扫过的范围,输出 workspace 盒的 min/max 供填入 configs/ur5e.yaml
   python scripts/workspace_calib.py 192.168.10.18
   ```

   结束后把打印的 min/max 填进 `configs/ur5e.yaml`(每个方向再往里收 2-3cm 余量);
2. 急停按钮放在操作员手边;所有人站在工作空间盒之外。

### 4.3 标定(VR 头显用户做;手柄用户跳过)

**首次 / 换了站位 / 重启过头显 app,都必须重做**(头显的"世界方向"= app 启动瞬间
你头朝的方向):

```bash
# 引导式标定:机械臂逐轴演示,你捏 grip 模仿,自动算出坐标对齐参数并写入配置
python scripts/auto_calibrate_ur5e.py
```

流程:机械臂逐轴慢速演示一小段 → 你捏住 grip 朝**同一个方向**移动/转动手柄
(平移 10cm 以上、旋转 30° 以上,凭直觉即可)→ 松手 → 下一轴。跑完自动把标定结果
写进 `configs/teleop/pico_ur5e.yaml`。小技巧:**启动头显 app 时人面向机器人**,
标定会更准。

个别轴反了不用整套重标,手动微调:

```bash
# 手动微调标定:进遥操界面,用数字键翻转某个轴的方向
python envs/ur5e/teleop_record.py --robot-host 192.168.10.18 --calibrate
# 界面里:键 1/2/3 翻平移方向,4/5/6 翻旋转方向,s 保存,q 退出
```

**通过标准**:"手往哪动,机械臂末端就往哪动"。

### 4.4 遥操作与录制

```bash
# 遥操真机并录制(--pos-scale 0.4 = 速度减半,首跑必须低速;手柄用户加 --backend gamepad)
python envs/ur5e/teleop_record.py --robot-host 192.168.10.18 --pos-scale 0.4
```

操作与实验 2/3 完全一致(grip/LB=动,松手=停,B=保存,A=作废,q=退出——退出时
机械臂自动刹停)。
**通过标准**:在自由空间画 20cm 见方的立体方框,松手即停、全程无保护性停止。
录制产物:`outputs/ur5e_demos/episode_*.npz`(数据格式见 `architecture.md` §5);
命令加 `--camera 0` 可同时录一路 USB 相机。
夹爪(选配 Robotiq):把 `configs/ur5e.yaml` 里 `gripper.type` 改成 `robotiq`。

### 4.5 这些"停下来"都是什么意思

| 现象 | 含义 | 怎么继续 |
|---|---|---|
| 松手后停 | 正常:离合语义 | 重捏 grip / LB |
| 屏幕橙色 [WATCHDOG] | 手柄数据流断了(保护) | 松手重捏;频繁出现见 FAQ Q1/Q5 |
| 示教器弹保护性停止 | 动作太猛/撞限位 | 示教器上解锁 → 程序自动恢复 → 松手重捏(见 FAQ Q10) |
| 终端按 q / Ctrl-C 退出 | 程序刹停并断开 | 重新运行命令 |
