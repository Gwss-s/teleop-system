# configs 目录约定(三类配置,各司其职)

| 类 | 文件 | 内容 | 谁读 |
|---|---|---|---|
| **设备配置** | `gamepad.yaml` | 手柄键位/轴序号/速度/死区/型号表 | L1 后端(`backends/gamepad.py`) |
| **机器人配置** | `ur5e.yaml` | 控制柜 IP、节拍、servo 参数、**安全层**(工作空间盒/限幅/看门狗)、夹爪 | 机器人入口(`envs/ur5e/teleop_record.py`) |
| **映射标定** | `teleop/<设备>_<机器人>.yaml` | 逐轴符号×增益、engage 阈值、裁决键——**设备×机器人的标定产物** | L2(`EEDeltaMapper.from_yaml`) |

命名规则:`teleop/` 下一律 `<设备>_<机器人>[_变体].yaml`
(如 `pico_ur5e.yaml` = Pico→UR5e 的标定)。

⚠ 注意:
- 标定 yaml 由标定流程写入(`auto_calibrate_ur5e.py` / `--calibrate` 的 s 键),
  一般不手改;`world_yaw_deg` 绑定操作员站位与头显 app 启动朝向,换了就要重标;
- `ur5e.yaml` 的 workspace 盒**每换一个物理单元必须重新核对**(见 `docs/faq.md` Q11)。
