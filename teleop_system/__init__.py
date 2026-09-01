"""teleop_system: 三层通用遥操作框架(设备后端 -> 语义映射 -> 机器人适配)。

    L1 backends/  设备 -> TeleopState              (每种输入设备一个文件)
    L2 mapping/   离合/增量/标定 -> ControlIntent   (设备无关+机器人无关)
    L3 envs/<robot>/teleop_adapter.py  ControlIntent -> 机器人动作

换输入设备只写 L1,换机器人只写 L3;三层的缝由 types.py 的三个 dataclass 定义。
附带:policy/(策略服务协议+服务器,接管模式用)、runtime/(chunk 消费)、
legacy/(旧单文件实现,等价回归测试的冻结基准)。
"""
__version__ = "1.0.0"
