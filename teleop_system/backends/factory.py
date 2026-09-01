"""Backend factory: the ONE place mapping --backend names to L1 classes.

Keeps entries free of per-device imports (heavy vendor deps stay lazy inside
each branch), so adding a device = one backend file + one branch here.
"""
from pathlib import Path

BACKEND_CHOICES = ("pico", "gamepad", "remote", "replay")


def make_backend(kind, *, tap_path=None, tap=None, remote_host="localhost",
                 remote_port=5570, gamepad_config="configs/gamepad.yaml",
                 joystick_index=0, sides=("right",)):
    """kind: one of BACKEND_CHOICES.
    tap_path: raw-tap 录制目标(仅 pico 支持);tap: 重放源 npz(仅 replay)。"""
    if kind == "pico":
        from .pico_ultra4 import PicoUltra4  # noqa: PLC0415
        return PicoUltra4(sides=sides, tap_path=tap_path)
    if kind == "gamepad":
        cfg = None
        p = Path(gamepad_config)
        if p.exists():
            import yaml  # noqa: PLC0415
            cfg = yaml.safe_load(p.read_text())
        from .gamepad import GamepadBackend  # noqa: PLC0415
        return GamepadBackend(config=cfg, joystick_index=joystick_index)
    if kind == "remote":
        from .remote import RemoteBackend  # noqa: PLC0415
        return RemoteBackend(remote_host, remote_port)
    if kind == "replay":
        assert tap, "--backend replay 需要 --tap <raw_tap.npz>"
        from .tap_replay import TapReplay  # noqa: PLC0415
        return TapReplay(tap)
    raise ValueError(f"未知 backend: {kind!r} (可选: {BACKEND_CHOICES})")
