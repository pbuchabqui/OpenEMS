"""Stim Link — ESP32 stimulator communication (serial or TCP).

Extracted from tools/hil_test/hil_test.py. Supports both the legacy
esp32_stimulator (serial/TCP) and the new esp32_combined (serial).

Usage:
    from tools.lib.stim_link import StimLink

    stim = StimLink.serial("/dev/ttyUSB0")
    stim.set_rpm(1500)
    stim.set_map(55)
    stim.set_clt(90)
    stim.preset("IDLE")
"""

from __future__ import annotations
import time
import socket
import select


class StimLink:
    """ESP32 stimulator connection."""

    def __init__(self):
        self._sock = None
        self._ser = None

    @classmethod
    def serial(cls, port: str) -> StimLink:
        import serial
        s = cls()
        s._ser = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(0.3)
        s._ser.reset_input_buffer()
        return s

    @classmethod
    def tcp(cls, host: str, port: int = 3333) -> StimLink:
        s = cls()
        s._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s._sock.settimeout(2.0)
        s._sock.connect((host, port))
        return s

    def _send(self, cmd: str) -> None:
        line = (cmd + '\n').encode()
        if self._ser:
            self._ser.reset_input_buffer()
            self._ser.write(line)
            time.sleep(0.02)
        elif self._sock:
            self._sock.sendall(line)
            time.sleep(0.02)

    def _send_read(self, cmd: str, timeout: float = 0.3) -> str:
        self._send(cmd)
        if self._ser:
            t0 = time.time()
            while (time.time() - t0) < timeout:
                if self._ser.in_waiting:
                    return self._ser.readline().decode(errors='replace').strip()
                time.sleep(0.01)
        elif self._sock:
            ready, _, _ = select.select([self._sock], [], [], timeout)
            if ready:
                return self._sock.recv(256).decode(errors='replace').strip()
        return ""

    def set_rpm(self, rpm: int) -> None:
        self._send(f"RPM {rpm}")

    def set_map(self, kpa: int) -> None:
        self._send(f"MAP {kpa}")

    def set_clt(self, degc: int) -> None:
        self._send(f"CLT {degc}")

    def set_iat(self, degc: int) -> None:
        self._send(f"IAT {degc}")

    def set_tps(self, pct: int) -> None:
        self._send(f"TPS {pct}")

    def preset(self, name: str) -> None:
        self._send(name.upper())

    def status(self) -> str:
        return self._send_read("STATUS")

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        if self._sock:
            self._sock.close()
