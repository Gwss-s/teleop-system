"""自制末端执行器(舵机组)驱动 —— 课程小组在这里实现硬件通信。

框架已经把"手柄上的备用按键/摇杆 → 每路舵机的 [0,1] 指令"全部接好,每个控制
节拍(默认 25Hz)会调用一次 command(values);你只需要把 values 变成你的硬件听得懂
的东西(串口帧 / PWM / CAN ...)。夹爪的参考实现在同目录 gripper.py。

接口约定(envs/ur5e/teleop_record.py 按此调用):
  * __init__(cfg)        cfg = configs/ur5e.yaml 的 actuator: 块(dict),参数随你定义
  * command(values)      values: (n,) numpy 数组,每路 [0,1];n = yaml 里 channels 的值,
                         也等于 configs/teleop/*.yaml 中 actuators: 列表的长度。
                         每拍都会被调用(值不变时也会),要不要去重/限速由你决定
                         (gripper.py 的 deadband 是一个现成例子)
  * close()              退出时调用,释放串口等资源;必须能安全重复调用

约束:
  * 只准 import 标准库或你自己加进 pixi 环境的驱动库(如串口用 `pixi add pyserial`),
    并且 import 放在本文件里——不要泄漏到 teleop_system/ 的 L1/L2
  * 不要在 command() 里阻塞(sleep / 等回包超过几毫秒):它跑在机械臂伺服的主循环里,
    卡住就会触发看门狗
  * 舵机角度范围、正反向、速度限制等硬件常识写进这里(或 yaml),不要改上游

实现步骤建议:
  1. 先不接硬件,让 command() 只 print(values);configs/ur5e.yaml 的 actuator.type 改成
     custom,在 URSim 仿真里(docs/ur5e_setup.md 实验 2/3)按你绑好的键,确认打印的数值
     按预期变化(dry-run 也会调用本类,cfg["dry_run"] 为 True,可据此跳过开串口);
  2. 再接串口,把 [0,1] 线性映到每路舵机的 [min_angle, max_angle];
  3. 最后加保护:上电初值、断线重连、超时不阻塞。
"""


class ServoActuator:
    """N 路舵机执行器。未实现前,启用 configs/ur5e.yaml actuator.type=custom 会在
    第一拍抛 NotImplementedError,提醒你还没写。"""

    def __init__(self, cfg):
        self.cfg = dict(cfg or {})
        self.n = int(self.cfg.get("channels", 1))
        # TODO(课程小组): 在这里打开串口 / 初始化你的驱动板。参数从 self.cfg 取,例如
        #   self.cfg.get("port", "/dev/ttyUSB0"), self.cfg.get("baud", 115200)
        # 并把每路舵机的角度范围也放进 yaml 读出来,不要写死在代码里。

    def command(self, values):
        """values: (n,) 每路 [0,1] -> 发给硬件。每拍调用一次,禁止阻塞。"""
        raise NotImplementedError(
            "envs/ur5e/actuator.py: ServoActuator.command() 还没实现——"
            "把每路 [0,1] 换算成你的舵机角度并发出去;先用 print(values) 验证按键绑定")

    def close(self):
        # TODO(课程小组): 关串口等。允许被重复调用。
        pass
