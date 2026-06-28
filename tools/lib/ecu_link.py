"""ECU Link — STM32 CDC communication for OpenEMS test scripts.

Extracted from tools/hil_test/hil_test.py to eliminate code duplication
across diag/*.py, hil_test.py, and other test scripts.

Usage:
    from tools.lib.ecu_link import ECULink, Snapshot

    ecu = ECULink("/dev/ttyACM0")
    snap = ecu.snapshot()
    print(f"RPM: {snap.rpm_x10 / 10.0:.1f}")
    cal = ecu.read_calibration()
"""

from __future__ import annotations
import struct
import time
from dataclasses import dataclass

# ── CDC Protocol helpers ────────────────────────────────────────────────────

def _crlf_cmd(ser, cmd: bytes) -> bytes:
    """Send command, read until CR/LF, return response."""
    ser.reset_input_buffer()
    ser.write(cmd + b'\r')
    time.sleep(0.05)
    return ser.read(128)

def _read_cmd(ser, cmd: bytes, size: int, retries: int = 2) -> bytes | None:
    """Send single-char command, read exactly `size` bytes."""
    for _ in range(retries + 1):
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.08)
        data = ser.read(size)
        if len(data) == size:
            return data
        time.sleep(0.05)
    return None


# ── Snapshot dataclass ──────────────────────────────────────────────────────

@dataclass
class Snapshot:
    """66-byte realtime snapshot from ECU page 3."""
    rpm_x10: int = 0
    map_bar_x100: int = 0
    tps_x10: int = 0
    clt_x10: int = 0
    iat_x10: int = 0
    lambda_x1000: int = 0
    pw_us: int = 0
    advance_deg: int = 0
    ve_live: int = 0
    stft_pct: int = 0
    status: int = 0
    reserved: list[int] = None

    def __post_init__(self):
        if self.reserved is None:
            self.reserved = [0] * 32

    def full_sync(self) -> bool:    return bool(self.status & 0x0001)
    def phase_a(self) -> bool:      return bool(self.status & 0x0002)
    def sensor_fault(self) -> bool: return bool(self.status & 0x0004)
    def limp_mode(self) -> bool:    return bool(self.status & 0x0008)

    @classmethod
    def parse(cls, data: bytes) -> Snapshot:
        """Parse 66-byte snapshot from ECU."""
        if len(data) < 64:
            return cls()
        vals = struct.unpack_from('<HHHHHHHHHHHHBBBBBBBB', data, 0)
        return cls(
            rpm_x10=vals[0], map_bar_x100=vals[1], tps_x10=vals[2],
            clt_x10=vals[3], iat_x10=vals[4], lambda_x1000=vals[5],
            pw_us=vals[6], advance_deg=vals[7],
            ve_live=data[63] if len(data) > 63 else 0,
            status=vals[11],
        )


# ── ECULink class ───────────────────────────────────────────────────────────

class ECULink:
    """STM32 CDC connection for OpenEMS ECU."""

    def __init__(self, port: str):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=1.5)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def ping(self) -> bool:
        data = _crlf_cmd(self.ser, b'C')
        return len(data) >= 2 and data[0] == 0 and data[1] == 0xAA

    def snapshot(self) -> Snapshot:
        data = _read_cmd(self.ser, b'A', 66)
        return Snapshot.parse(data) if data else Snapshot()

    def read_page(self, page: int) -> bytes | None:
        assert 0 <= page <= 10
        return _read_cmd(self.ser, bytes([ord('r'), page, 0, 0]), 512)

    def bench_mode(self, enable: bool = True) -> None:
        self.ser.write(b'B' + (b'\x01' if enable else b'\x00'))
        time.sleep(0.02)

    def firmware_version(self) -> str:
        data = _crlf_cmd(self.ser, b'S')
        return data.decode('ascii', errors='replace').strip()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
