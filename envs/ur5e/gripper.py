"""Robotiq gripper client over the Robotiq_Grippers URCap socket (port 63352).

The ur_rtde official guide lists three ways to drive a Robotiq gripper; this
is the recommended one (and what the official XRoboToolkit UR5e teleop uses):
string commands to the URCap's socket server on the control box — does NOT
interrupt the rtde_control script, so arm and gripper move simultaneously.

Protocol: "SET <VAR> <val> [<VAR> <val> ...]\n" -> "ack";
          "GET <VAR>\n" -> "<VAR> <val>".  Key vars:
  ACT activate(1)   GTO go-to(1)   POS 0(open)-255(closed)
  SPE speed 0-255   FOR force 0-255   STA status(3 = activation done)

Analog trigger [0,1] -> POS with a small change deadband (avoid socket spam
at the control rate). stdlib-only.
"""
import socket
import time


class RobotiqGripper:
    def __init__(self, host, port=63352, speed=255, force=128, timeout=2.0):
        self.speed, self.force = int(speed), int(force)
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.settimeout(timeout)
        self._last_pos = None

    # -- wire helpers ----------------------------------------------------------
    def _set(self, **vars_):
        cmd = "SET " + " ".join(f"{k} {v}" for k, v in vars_.items()) + "\n"
        self._sock.sendall(cmd.encode())
        return self._sock.recv(1024).strip() == b"ack"

    def _get(self, var):
        self._sock.sendall(f"GET {var}\n".encode())
        r = self._sock.recv(1024).strip().decode()   # "VAR value"
        return int(r.split()[1])

    # -- API -------------------------------------------------------------------
    def activate(self, wait=10.0):
        """Activate (runs a close/open calibration cycle on first power-up)."""
        if self._get("STA") == 3:
            self._set(GTO=1, SPE=self.speed, FOR=self.force)
            return True
        self._set(ACT=0)
        self._set(ACT=1)
        t0 = time.time()
        while time.time() - t0 < wait:
            if self._get("STA") == 3:
                self._set(GTO=1, SPE=self.speed, FOR=self.force)
                return True
            time.sleep(0.1)
        raise TimeoutError("Robotiq activation did not reach STA 3")

    def move(self, pos):
        """Command position 0(open)-255(closed); returns commanded value."""
        pos = max(0, min(255, int(round(pos))))
        if self._last_pos is not None and abs(pos - self._last_pos) < 3:
            return self._last_pos                     # deadband: skip no-op traffic
        self._set(POS=pos)
        self._last_pos = pos
        return pos

    def move_trigger(self, trigger):
        """Analog trigger [0,1] -> gripper position (1.0 = fully closed)."""
        return self.move(round(float(trigger) * 255))

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass
