"""Scope Link — ESP32 logic scope serial reader.

Extracted from tools/esp32_scope/scope_host.py. Works with both
esp32_scope.ino and esp32_combined.ino.

Usage:
    from tools.lib.scope_link import ScopeLink

    scope = ScopeLink("/dev/ttyUSB0")
    rows = scope.read_live_table()
    for r in rows:
        print(f"{r['ch']}: PW={r['pw_us']}us Period={r['period_us']}us")
"""

from __future__ import annotations
import time
import re
from dataclasses import dataclass


@dataclass
class LiveRow:
    ch: int = 0
    name: str = ""
    status: str = ""
    pw_us: float = 0.0
    period_us: float = 0.0
    freq_hz: float = 0.0
    duty_pct: float = 0.0
    count: int = 0


class ScopeLink:
    """ESP32 scope serial connection."""

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def send_cmd(self, cmd: str) -> None:
        self.ser.write((cmd + '\r\n').encode())
        time.sleep(0.05)

    def read_line(self) -> str:
        return self.ser.readline().decode(errors='replace').strip()

    def read_lines(self, timeout_s: float = 2.0) -> list[str]:
        lines = []
        t0 = time.time()
        while (time.time() - t0) < timeout_s:
            line = self.read_line()
            if line:
                lines.append(line)
            elif lines:
                break
        return lines

    def run_live(self) -> list[LiveRow]:
        """Switch to LIVE mode and read one table snapshot."""
        self.send_cmd('l')
        time.sleep(1.2)
        rows = []
        for line in self.read_lines(1.5):
            m = re.match(
                r'CH(\d+)\s+(\S+)\s+(\S+)\s+([\d.]+)\s+([\d.]+)\s+'
                r'([\d.]+)\s+([\d.]+)\s+(\d+)', line)
            if m:
                rows.append(LiveRow(
                    ch=int(m.group(1)), name=m.group(2), status=m.group(3),
                    pw_us=float(m.group(4)), period_us=float(m.group(5)),
                    freq_hz=float(m.group(6)), duty_pct=float(m.group(7)),
                    count=int(m.group(8))))
        return rows

    def run_timing(self) -> str:
        """Run timing 720° analysis and return report text."""
        self.send_cmd('t')
        time.sleep(2.0)
        return '\n'.join(self.read_lines(3.0))

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
