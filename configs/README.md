# configs 目录约定(三类配置,各司其职)

| 类 | 文件 | 内容 | 谁读 |
|---|---|---|---|
| **设备配置** | `gamepad.yaml` | 手柄键位/轴序号/速度/死区/型号表 | L1 后端(`backends/gamepad.py`) |
| **机器人配置** | `ur5e.yaml` | 控制柜 IP、节拍、servo 参数、**安全层**(工作空间盒/限幅/看门狗)、夹爪、自制执行器硬件参数 | 机器人入口(`envs/ur5e/teleop_record.py`) |
| **映射标定** | `teleop/<设备>_<机器人>.yaml` | 逐轴符号×增益、engage 阈值、裁决键——**设备×机器人的标定产物**;可选 `actuators:` 通道表 | L2(`EEDeltaMapper.from_yaml`) |

命名规则:`teleop/` 下一律 `<设备>_<机器人>[_变体].yaml`
(如 `pico_ur5e.yaml` = Pico→UR5e 的标定)。

⚠ 注意:
- 标定 yaml 由标定流程写入(`auto_calibrate_ur5e.py` / `--calibrate` 的 s 键),
  一般不手改;`world_yaw_deg` 绑定操作员站位与头显 app 启动朝向,换了就要重标;
- `ur5e.yaml` 的 workspace 盒**每换一个物理单元必须重新核对**(见 `docs/faq.md` Q11)。

## 自制末端执行器(舵机组)怎么配

分两处,缺一不可:

**① `teleop/<设备>_ur5e.yaml` 的 `actuators:`**——每项一路舵机,顺序就是舵机编号,
决定"哪个备用键 → 哪路 → 怎么变":

```yaml
actuators:
  - {source: <备用输入名>, mode: toggle, init: 0.0}                      # 按一下开、再按合
  - {source: [<正向输入名>, <负向输入名>], mode: rate, speed: 0.5, init: 0.5}  # 两个键当摇杆,匀速走
  - {source: <摇杆轴名>, mode: rate, speed: 0.8, init: 0.5}              # 摇杆推多远走多快
  - {source: <模拟量名>, mode: hold}                                      # 直接跟随(扳机/按住)
```

| 字段 | 取值 |
|---|---|
| `source` | 一个备用输入名,或 `[正向, 负向]` 一对(值 = 正 − 负)。可用名字:手柄见 `gamepad.yaml` 注释,Pico 见 `backends/pico_ultra4.py` 模块头;实时查看用 `scripts/gamepad_axis_dump.py` / `scripts/pico_probe.py` |
| `mode` | `hold` 直接跟随 · `rate` 输入当速度(每秒走 `speed`×输入,量程 0..1) · `toggle` 上升沿 0/1 切换 |
| `init` | 启动初值 [0,1] |

通道值与是否捏合离合无关——停臂时也能操作工具。不写 `actuators:` = 不启用,
一切照旧。标定脚本重写 yaml 时会原样保留这一块。

**② `ur5e.yaml` 的 `actuator:`**——`type: custom` 启用硬件驱动,`channels` 必须等于①的
通道数;其余键(串口、波特率、角度范围……)是你的驱动参数,在
`envs/ur5e/actuator.py` 的 `ServoActuator(cfg)` 里读。`type: none` 时通道值只录进
npz(`actuators` 键),不驱动硬件——先这样验证按键绑定,再接硬件。
