#!/usr/bin/env python3
"""
hil_test.py — Teste HIL automático para OpenEMS
════════════════════════════════════════════════
Requer hardware:
  PC ─ USB → STM32H562          (CDC, protocolo OpenEMS — /dev/ttyACM0)
  esp32_stimulator → STM32      (CKP 60-2 por RMT em GPIO2→PA0, GND comum)
  estimulador acessível por serial (/dev/ttyUSB0) ou WiFi/TCP (porta 3333)

O estimulador gera o sinal 60-2 (hardware RMT, jitter ~zero). O script lê as
tabelas da ECU (VE, spark, dwell, correções) e a config do motor, calcula os
valores esperados em Python com o MESMO math inteiro do firmware, e compara
com o snapshot (VE/avanço/PW computados pela ECU) em cada ponto de operação.

Sem osciloscópio: valida a math interna da ECU. As checagens são
self-consistentes sobre o snapshot — funcionam mesmo com só o CKP ligado
(MAP/CLT analógicos opcionais; se desligados, a ECU usa o que ler).

IMPORTANTE: após gravar a STM32 por DFU, faça power-cycle (o dfu :leave é jump,
não reset — sem POR o RCC fica no estado do bootloader e o TIM5/CKP não clocam).

Uso:
    python3 hil_test.py --stm32 /dev/ttyACM0 --stim /dev/ttyUSB0
    python3 hil_test.py --stm32 /dev/ttyACM0 --stim tcp:192.168.15.169:3333 \\
                        --rpms 800 1500 3000 --report resultado.md
"""

from __future__ import annotations
import argparse
import struct
import sys
import time
import re
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

try:
    import serial
except ImportError:
    print("Instalar: pip install pyserial")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════════════════
# Tolerâncias
# ══════════════════════════════════════════════════════════════════════════════

TOL_RPM_PCT      = 5.0   # % — RPM medido vs comandado
TOL_VE_UNITS     = 3.0   # unidades — VE snapshot vs tabela
TOL_ADVANCE_DEG  = 2.0   # ° — advance snapshot vs tabela
TOL_INJ_PW_PCT   = 45.0  # % — bench sem sensores: afterstart enrichment não modelado

# Mirror de calc_fuel_pw_us_default_fast — valores não expostos no snapshot:
LAMBDA_TARGET_X1000 = 1000   # alvo λ=1.00 (stoich); factor 1000/λ
INJ_DEAD_TIME_US    = 900    # dead-time do injector a ~14 V (calibration.cpp, vbatt-idx)
TOL_INTER_CYL_MS = 2.0   # ms — espaçamento entre cilindros
TOL_HW_PW_MS     = 0.3   # ms — PW scope vs snapshot (medição hardware)
TOL_DWELL_MS     = 0.3   # ms — dwell scope vs tabela

SETTLE_S    = 2.0         # s — aguardar após mudança de RPM
SYNC_WAIT_S = 8.0         # s — timeout para FULL_SYNC

EXPECTED_FIRING_ORDER = [0, 2, 3, 1]   # IGN0→IGN2→IGN3→IGN1  (1-3-4-2)

# ══════════════════════════════════════════════════════════════════════════════
# Eixos das tabelas 3D (table3d.cpp)
# ══════════════════════════════════════════════════════════════════════════════

RPM_AXIS_X10 = [
    5000,  7500,  10000, 12500,
    15000, 20000, 25000, 30000,
    35000, 40000, 45000, 50000,
    55000, 60000, 70000, 80000,
]

MAP_AXIS_BAR_X100 = [
     20,  30,  40,  52,
     64,  76,  88, 100,
    110, 130, 160, 190,
    220, 250, 273, 300,
]

# ══════════════════════════════════════════════════════════════════════════════
# Mirror do firmware (table3d.cpp + fuel_calc.cpp)
# ══════════════════════════════════════════════════════════════════════════════

def _axis_lookup(axis: list[int], value: int) -> tuple[int, int]:
    """Mirror de axis_lookup(): binary search → (idx, frac_q8)."""
    if value <= axis[0]:
        return 0, 0
    last = len(axis) - 1
    if value >= axis[last]:
        return last - 1, 255
    lo, hi = 1, last
    while lo < hi:
        mid = lo + (hi - lo) // 2
        if value <= axis[mid]:
            hi = mid
        else:
            lo = mid + 1
    idx = lo - 1
    x0, x1 = axis[idx], axis[idx + 1]
    if value <= x0:
        return idx, 0
    if value >= x1:
        return idx, 255
    return idx, min(255, ((value - x0) << 8) // (x1 - x0))


def _lerp_q8(a: int, b: int, f: int) -> int:
    """Mirror de lerp_q8_s32()."""
    return b if f == 255 else a + (((b - a) * f) >> 8)


def _table3d(table: list[list[int]],
             x_axis: list[int], y_axis: list[int],
             x: int, y: int) -> int:
    """Interpolação bilinear — mirror exacto de table3d_lookup_u8_prepared()."""
    xi, fx = _axis_lookup(x_axis, x)
    yi, fy = _axis_lookup(y_axis, y)
    v0 = _lerp_q8(table[yi][xi],     table[yi][xi + 1],     fx)
    v1 = _lerp_q8(table[yi + 1][xi], table[yi + 1][xi + 1], fx)
    return max(0, min(255, _lerp_q8(v0, v1, fy)))


def _table3d_signed(table: list[list[int]],
                    x_axis: list[int], y_axis: list[int],
                    x: int, y: int) -> int:
    """Mirror de table3d_lookup_i8_prepared() (signed)."""
    xi, fx = _axis_lookup(x_axis, x)
    yi, fy = _axis_lookup(y_axis, y)
    v0 = _lerp_q8(table[yi][xi],     table[yi][xi + 1],     fx)
    v1 = _lerp_q8(table[yi + 1][xi], table[yi + 1][xi + 1], fx)
    return _lerp_q8(v0, v1, fy)


def _interp1d(axis: list[int], values: list[int], x: int) -> int:
    """Interpolação 1D linear."""
    idx, frac = _axis_lookup(axis, x)
    return _lerp_q8(values[idx], values[idx + 1], frac)

# ══════════════════════════════════════════════════════════════════════════════
# Estruturas de dados
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class EngineConfig:
    ivc_abdc_deg:              int = 50
    displacement_cc:           int = 2000
    injector_flow_cc_min:      int = 450
    stoich_afr_x100:           int = 1470
    map_ref_bar_x100:          int = 100
    trigger_tooth0_engine_deg: int = 0
    default_soi_lead_deg:      int = 62

    @property
    def req_fuel_us(self) -> int:
        """
        Mirror inteiro de calc_req_fuel_us() — fuel_calc.cpp.
        num = displacement * kAirDensityMgPerCcX1000 * 100 * 60_000_000
        den = cylinders * stoich_x100 * inj_flow_cc_min * kFuelDensityMgPerCc * 1000
        Verificado: (1998, 4, 440, 1470) → 7266 µs.
        """
        if self.displacement_cc == 0 or self.injector_flow_cc_min == 0 \
                or self.stoich_afr_x100 == 0:
            return 0
        num = self.displacement_cc * 1184 * 100 * 60_000_000
        den = 4 * self.stoich_afr_x100 * self.injector_flow_cc_min * 755 * 1000
        return min(num // den, 50000)


@dataclass
class Snapshot:
    rpm:          int = 0
    map_bar_x100: int = 100
    tps_pct:      int = 0
    clt_degc:     int = 25
    iat_degc:     int = 25
    pw1_ms_x10:   int = 0
    advance_deg:  int = 0
    ve:           int = 0       # VE[0][0] (primeira célula da tabela)
    ve_live:      int = 0       # VE interpolado vivo (get_ve no ponto rpm×map atual)
    stft_pct:     int = 0
    status:       int = 0

    @property
    def full_sync(self) -> bool:    return bool(self.status & 0x0001)
    @property
    def sensor_fault(self) -> bool: return bool(self.status & 0x0004)
    @property
    def late_event(self) -> bool:   return bool(self.status & 0x0040)
    @property
    def drop_count(self) -> bool:   return bool(self.status & 0x0080)
    @property
    def inj_pw_ms(self) -> float:   return self.pw1_ms_x10 / 10.0


@dataclass
class Tables:
    ve_table:              list[list[int]] = field(default_factory=list)
    spark_table:           list[list[int]] = field(default_factory=list)
    lambda_target_table:   list[list[int]]  = field(default_factory=list)
    dwell_vbatt_axis_mv:   list[int]       = field(default_factory=list)
    dwell_ms_x10:          list[int]       = field(default_factory=list)
    clt_corr_axis_x10:     list[int]       = field(default_factory=list)
    clt_corr_x256:         list[int]       = field(default_factory=list)
    iat_corr_axis_x10:     list[int]       = field(default_factory=list)
    iat_corr_x256:         list[int]       = field(default_factory=list)
    warmup_corr_axis_x10:  list[int]       = field(default_factory=list)
    warmup_corr_x256:      list[int]       = field(default_factory=list)
    vbatt_corr_axis_mv:    list[int]       = field(default_factory=list)
    injector_dead_time_us: list[int]       = field(default_factory=list)

    def ve_at(self, rpm: int, map_bar_x100: int) -> int:
        if not self.ve_table:
            return 80
        return _table3d(self.ve_table, RPM_AXIS_X10, MAP_AXIS_BAR_X100,
                        rpm * 10, map_bar_x100)

    def advance_at(self, rpm: int, map_bar_x100: int) -> int:
        if not self.spark_table:
            return 10
        return _table3d_signed(self.spark_table, RPM_AXIS_X10, MAP_AXIS_BAR_X100,
                               rpm * 10, map_bar_x100)

    def dwell_ms_at(self, vbatt_mv: int = 12000) -> float:
        if not self.dwell_vbatt_axis_mv or not self.dwell_ms_x10:
            return 3.0
        return _interp1d(self.dwell_vbatt_axis_mv, self.dwell_ms_x10, vbatt_mv) / 10.0

    def clt_corr(self, clt_degc: int) -> float:
        if not self.clt_corr_axis_x10:
            return 1.0
        return _interp1d(self.clt_corr_axis_x10, self.clt_corr_x256,
                         clt_degc * 10) / 256.0

    def iat_corr(self, iat_degc: int) -> float:
        if not self.iat_corr_axis_x10:
            return 1.0
        return _interp1d(self.iat_corr_axis_x10, self.iat_corr_x256,
                         iat_degc * 10) / 256.0

    def warmup_corr(self, clt_degc: int) -> float:
        if not self.warmup_corr_axis_x10:
            return 1.0
        return _interp1d(self.warmup_corr_axis_x10, self.warmup_corr_x256,
                         clt_degc * 10) / 256.0

    def dead_time_us(self, vbatt_mv: int = 14000) -> int:
        if not self.vbatt_corr_axis_mv or not self.injector_dead_time_us:
            return 900
        return _interp1d(self.vbatt_corr_axis_mv, self.injector_dead_time_us,
                         vbatt_mv)

    def lambda_target_at(self, rpm: int, map_bar_x100: int) -> int:
        if not self.lambda_target_table:
            return 1000
        return _table3d(self.lambda_target_table, RPM_AXIS_X10, MAP_AXIS_BAR_X100,
                        rpm * 10, map_bar_x100)

# ══════════════════════════════════════════════════════════════════════════════
# STM32 client
# ══════════════════════════════════════════════════════════════════════════════

class STM32Client:
    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(0.3)
        self._ser.reset_input_buffer()

    def ping(self) -> bool:
        self._ser.write(b"\x43")
        time.sleep(0.05)
        r = self._ser.read(2)
        return len(r) == 2 and r[0] == 0x00 and r[1] == 0xAA

    def set_bench_clt_iat(self, enable: bool) -> bool:
        """Comando 'B': bench-mode CLT/IAT (90 °C/25 °C fixos, sem SENSOR_FAULT
        pelos canais CLT/IAT). Para banco HIL sem os sensores físicos ligados."""
        self._ser.write(bytes([0x42, 0x01 if enable else 0x00]))
        time.sleep(0.05)
        r = self._ser.read(1)
        return len(r) == 1 and r[0] == 0x00

    def _read_page(self, page: int, offset: int, length: int) -> bytes:
        cmd = bytes([0x72, page,
                     offset & 0xFF, (offset >> 8) & 0xFF,
                     length & 0xFF, (length >> 8) & 0xFF])
        self._ser.write(cmd)
        time.sleep(0.05 + length / 10000)
        return self._ser.read(length)

    def snapshot(self) -> Optional[Snapshot]:
        self._ser.write(b"\x41")
        time.sleep(0.05)
        r = self._ser.read(66)
        if len(r) < 13:
            return None
        s = Snapshot()
        s.rpm          = struct.unpack_from("<H", r, 0)[0]
        s.map_bar_x100 = r[2]
        s.tps_pct      = r[3]
        s.clt_degc     = r[4] - 40
        s.iat_degc     = r[5] - 40
        s.pw1_ms_x10   = r[7]
        s.advance_deg  = r[8] - 40
        s.ve           = r[9]
        s.stft_pct     = struct.unpack_from("b", r, 10)[0]
        # status_bits é uint16 alinhado → offset 12 (1 byte de padding em 11).
        s.status       = struct.unpack_from("<H", r, 12)[0]
        # VE interpolado vivo em reserved[49] = byte 14+49 = 63 (firmware get_ve).
        if len(r) >= 66:
            s.ve_live  = r[63]
        return s

    def read_engine_config(self) -> EngineConfig:
        d = self._read_page(0, 0, 16)
        if len(d) < 16:
            return EngineConfig()
        return EngineConfig(
            ivc_abdc_deg              = d[0],
            displacement_cc           = struct.unpack_from("<H", d,  2)[0],
            injector_flow_cc_min      = struct.unpack_from("<H", d,  4)[0],
            stoich_afr_x100           = struct.unpack_from("<H", d,  6)[0],
            map_ref_bar_x100          = struct.unpack_from("<H", d,  8)[0],
            trigger_tooth0_engine_deg = struct.unpack_from("<H", d, 10)[0],
            default_soi_lead_deg      = struct.unpack_from("<H", d, 12)[0],
        )

    def read_tables(self) -> Tables:
        t = Tables()

        raw = self._read_page(1, 0, 256)
        if len(raw) == 256:
            t.ve_table = [[raw[r * 16 + c] for c in range(16)] for r in range(16)]

        raw = self._read_page(2, 0, 256)
        if len(raw) == 256:
            t.spark_table = [
                [struct.unpack_from("b", raw, r * 16 + c)[0] for c in range(16)]
                for r in range(16)]

        # page 4: lambda target (16×16 int16, ×1000)
        raw = self._read_page(4, 0, 512)
        if len(raw) == 512:
            t.lambda_target_table = [
                [struct.unpack_from("<h", raw, (r * 16 + c) * 2)[0]
                 for c in range(16)] for r in range(16)]

        # page 5: CLT/IAT corr + warmup + dead-time + dwell
        raw = self._read_page(5, 0, 192)
        if len(raw) >= 192:
            u16 = lambda off, n: [struct.unpack_from("<H", raw, off + i*2)[0]
                                  for i in range(n)]
            i16 = lambda off, n: [struct.unpack_from("<h", raw, off + i*2)[0]
                                  for i in range(n)]
            t.clt_corr_axis_x10    = i16(  0, 8)
            t.clt_corr_x256        = u16( 16, 8)
            t.iat_corr_axis_x10    = i16( 32, 8)
            t.iat_corr_x256        = u16( 48, 8)
            t.warmup_corr_axis_x10 = i16( 64, 8)
            t.warmup_corr_x256     = u16( 80, 8)
            t.vbatt_corr_axis_mv   = u16( 96, 8)
            t.injector_dead_time_us = u16(112, 8)
            t.dwell_vbatt_axis_mv  = u16(160, 8)
            t.dwell_ms_x10         = u16(176, 8)

        return t

    def close(self):
        self._ser.close()

# ══════════════════════════════════════════════════════════════════════════════
# ESP32 combined client (CKP gen + scope numa única ligação série)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class LiveData:
    pw_ms:     float = 0.0
    period_ms: float = 0.0
    count:     int   = 0
    idle:      bool  = True

@dataclass
class TimingData:
    firing_order: list[int]        = field(default_factory=list)
    inter_cyl_ms: list[float]      = field(default_factory=list)
    from_gap_ms:  dict[int, float] = field(default_factory=dict)
    ok:           bool             = False
    error:        str              = ""


class ESP32Client:
    """
    Controla o esp32_combined.ino: gera CKP e lê scope pela mesma UART.
    Comandos CKP: +/-/0-9/S
    Comandos scope: l/e/p/w/t/s/r/?
    """
    _PRESETS = [100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000]

    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=3.0)
        self._rpm = 500
        time.sleep(0.3)
        self._ser.reset_input_buffer()

    def _send(self, c: str):
        self._ser.write(c.encode())
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    # ── CKP generator ─────────────────────────────────────────────────────

    def set_rpm(self, target: int):
        """Ajusta RPM via preset ou passos de 100."""
        if target in self._PRESETS:
            self._send(str(self._PRESETS.index(target)))
            self._rpm = target
            return
        while self._rpm != target:
            if self._rpm < target:
                self._send("+")
                self._rpm = min(self._rpm + 100, target)
            else:
                self._send("-")
                self._rpm = max(self._rpm - 100, target)

    # ── Scope ──────────────────────────────────────────────────────────────

    def get_live(self, wait_s: float = 1.5) -> dict[int, LiveData]:
        """Modo 'l' — devolve {ch: LiveData}."""
        self._send("l")
        time.sleep(wait_s)
        raw  = self._ser.read(self._ser.in_waiting)
        text = raw.decode("utf-8", errors="replace")

        RE = re.compile(
            r"\|\s*(\d+)\|(\w+)\s*\|\S+\s*\|\s*([\d.]+)\|\s*([\d.]+)\|\s*[\d.]+\|\s*(\d+)\|"
        )
        result: dict[int, LiveData] = {}
        for m in RE.finditer(text):
            ch, _name, pw, period, count = m.groups()
            window = text[m.start(): m.start() + 120]
            result[int(ch)] = LiveData(
                pw_ms     = float(pw),
                period_ms = float(period),
                count     = int(count),
                idle      = "IDLE" in window,
            )
        return result

    def run_timing(self, timeout_s: float = 30.0) -> TimingData:
        """Modo 't' — devolve firing order, inter-cyl e ângulo."""
        self._send("t")
        deadline = time.time() + timeout_s
        lines: list[str] = []

        while time.time() < deadline:
            line = self._ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
            if any(k in line for k in ("Resultado final", "IGNIÇÃO OK", "VER FALHAS")):
                break

        text = "\n".join(lines)
        td   = TimingData()

        # Parsar "  [TIMING] IGN0 spark @ +15.234 ms"
        sparks: list[tuple[int, float]] = []
        for m in re.finditer(r"IGN(\d)\s+spark\s*@\s*\+([\d.]+)\s*ms", text):
            sparks.append((int(m.group(1)), float(m.group(2))))

        if len(sparks) < 4:
            td.error = f"Apenas {len(sparks)}/4 sparks capturados"
            return td

        sparks.sort(key=lambda x: x[1])
        td.firing_order = [ch for ch, _ in sparks]
        td.from_gap_ms  = {ch: dt for ch, dt in sparks}
        td.inter_cyl_ms = [sparks[i][1] - sparks[i-1][1] for i in range(1, len(sparks))]
        td.ok           = "IGNIÇÃO OK" in text
        if "VER FALHAS" in text:
            td.error = "Scope reportou falhas"
        return td

    def close(self):
        self._ser.close()


class StimClient:
    """
    Controla o esp32_stimulator (CKP 60-2 por RMT + sensores).
    Protocolo de texto: 'RPM 1500', 'MAP 55', 'CLT 90', … (presets IDLE/WOT/…).
    Transporte: serial '/dev/ttyUSB*' ou TCP 'tcp:IP[:porta]' (WiFi, porta 3333).
    Sem scope — valida apenas a math interna da ECU via snapshot.
    """
    PARAMS = {"RPM", "MAP", "TPS", "CLT", "IAT", "APP", "FUEL", "OIL", "ETB"}

    def __init__(self, target: str, baud: int = 115200):
        self._tcp = None
        self._ser = None
        hp = self._parse_tcp(target)
        if hp:
            import socket
            self._tcp = socket.create_connection(hp, timeout=3.0)
            self._tcp.settimeout(2.0)
            time.sleep(0.3)
            try:
                self._tcp.recv(200)   # banner "OpenEMS-Stim pronto"
            except Exception:
                pass
        else:
            self._ser = serial.Serial(target, baud, timeout=1.0)
            time.sleep(0.3)
            self._ser.reset_input_buffer()

    @staticmethod
    def _parse_tcp(arg: str):
        if arg.startswith("tcp:"):
            arg = arg[4:]
        elif arg.startswith("socket://"):
            arg = arg[9:]
        elif "/" in arg or ":" not in arg:
            return None   # é um caminho de device serial
        host, _, port = arg.partition(":")
        return (host, int(port) if port else 3333)

    def _send(self, line: str):
        data = (line + "\n").encode()
        if self._tcp is not None:
            self._tcp.sendall(data)
        else:
            self._ser.write(data)
        time.sleep(0.05)

    def set_rpm(self, rpm: int):
        self._send(f"RPM {int(rpm)}")

    def set(self, param: str, value: int):
        param = param.upper()
        if param in self.PARAMS:
            self._send(f"{param} {int(value)}")

    def preset(self, name: str):
        self._send(name.upper())

    def close(self):
        if self._tcp is not None:
            self._tcp.close()
        if self._ser is not None:
            self._ser.close()

# ══════════════════════════════════════════════════════════════════════════════
# Resultado de cada ponto de teste
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class TestResult:
    rpm_cmd:     int
    description: str
    snap:        Optional[Snapshot]  = None
    timing:      Optional[TimingData] = None
    live:        Optional[dict]      = None
    checks:      list[tuple[str, bool, str]] = field(default_factory=list)

    def passed(self) -> bool:
        return all(ok for _, ok, _ in self.checks)

    def add(self, name: str, ok: bool, detail: str = ""):
        self.checks.append((name, ok, detail))

    def add_range(self, name: str, actual, expected, tol,
                  fmt: str = ".3f", pct: bool = False):
        if pct:
            err = abs(actual - expected) / max(abs(expected), 1e-9) * 100
            ok  = err <= tol
            detail = (f"actual={actual:{fmt}}  expected={expected:{fmt}}"
                      f"  err={err:.1f}%  tol={tol:.1f}%")
        else:
            err = abs(actual - expected)
            ok  = err <= tol
            detail = (f"actual={actual:{fmt}}  expected={expected:{fmt}}"
                      f"  err={err:{fmt}}  tol={tol:{fmt}}")
        self.add(name, ok, detail)

# ══════════════════════════════════════════════════════════════════════════════
# HIL Runner
# ══════════════════════════════════════════════════════════════════════════════

class HILRunner:
    def __init__(self, stm32: STM32Client, stim: StimClient,
                 eng: EngineConfig, tables: Tables, bench_mode: bool = False):
        self._stm    = stm32
        self._stim   = stim
        self._eng    = eng
        self._tables = tables
        self._bench  = bench_mode
        self.results: list[TestResult] = []

    def _wait_sync(self) -> bool:
        deadline = time.time() + SYNC_WAIT_S
        while time.time() < deadline:
            s = self._stm.snapshot()
            if s and s.full_sync:
                return True
            time.sleep(0.2)
        return False

    def set_mode(self, mode: str, scope_port: str | None = None):
        self._mode = mode
        self._scope = None
        if mode == "timing" and scope_port:
            try:
                from tools.lib.scope_link import ScopeLink
                self._scope = ScopeLink(scope_port)
                print(f"  ✓ Scope conectado em {scope_port}")
            except Exception as e:
                print(f"  ⚠ Scope não disponível: {e}")

    def run(self, points: list[TestPoint]) -> list[TestResult]:
        if self._mode == "verify":
            print("\n  [MODO VERIFY] Validação de outputs do event scheduler")
        elif self._mode == "timing":
            print("\n  [MODO TIMING] Análise 720° via scope combinado")

        for pt in points:
            print(f"\n{'─'*60}")
            print(f"  RPM={pt.rpm}  {pt.description}")
            print(f"{'─'*60}")
            r = self._run_point(pt)

            # Verify mode: event scheduler diagnostics
            if self._mode == "verify" and r.snap:
                s = r.snap
                ok = not s.late_event and not s.drop_count
                r.add("Event queue OK (no late/drop)",
                      ok,
                      f"late={s.late_event} drop={s.drop_count} status=0x{s.status:04X}")
                r.add("FULL_SYNC mantido", s.full_sync,
                      f"status=0x{s.status:04X}")

            # Timing mode: scope 720° analysis
            if self._mode == "timing" and self._scope and r.snap:
                try:
                    report = self._scope.run_timing()
                    ok = "PASS" in report and "FAIL" not in report
                    r.add("Scope timing 720°", ok, report[:120])
                except Exception as e:
                    r.add("Scope timing 720°", False, str(e))

            self.results.append(r)
            for name, ok, detail in r.checks:
                print(f"  {'✓' if ok else '✗'}  {name}")
                if detail:
                    print(f"      {detail}")
        return self.results

    def _run_point(self, pt: TestPoint) -> TestResult:
        r = TestResult(rpm_cmd=pt.rpm, description=pt.description)

        # 1. Estímulo: RPM (CKP por RMT) + sensores analógicos (só efetivos se
        #    os canais MAP/CLT/IAT estiverem ligados; senão a ECU lê o que tiver
        #    — as checagens abaixo são self-consistentes sobre o snapshot).
        print(f"  → RPM={pt.rpm}  MAP={pt.map_kpa}kPa  CLT={pt.clt}°C...")
        self._stim.set("MAP", pt.map_kpa)
        self._stim.set("CLT", pt.clt)
        self._stim.set("IAT", pt.iat)
        self._stim.set_rpm(pt.rpm)
        time.sleep(SETTLE_S)

        # 2. FULL_SYNC
        synced = self._wait_sync()
        r.add("FULL_SYNC", synced,
              "" if synced else "Sem sync — verificar sinal CKP (GPIO2→PA0)")
        if not synced:
            return r

        # 3. Snapshot
        snap = self._stm.snapshot()
        if snap is None:
            r.add("Snapshot lido", False, "Sem resposta UART")
            return r
        r.snap = snap

        if self._bench and snap.sensor_fault:
            r.add("Sem sensor fault", True,
                  f"status=0x{snap.status:04X} (esperado em bench sem MAP/TPS)")
        else:
            r.add("Sem sensor fault", not snap.sensor_fault,
                  f"status=0x{snap.status:04X}")
        r.add("Sem late events",  not snap.late_event,
              f"status=0x{snap.status:04X}")

        # 4. RPM medido
        r.add_range("RPM medido", snap.rpm, pt.rpm, TOL_RPM_PCT,
                    fmt=".0f", pct=True)

        # 5. VE interpolado vivo (reserved[49]) vs tabela Python interpolada.
        #    snap.ve é só VE[0][0]; snap.ve_live é o get_ve no ponto rpm×map (= ECU).
        exp_ve = self._tables.ve_at(snap.rpm, snap.map_bar_x100)
        r.add_range("VE (vivo vs tabela)",
                    snap.ve_live, exp_ve, TOL_VE_UNITS, fmt=".0f")

        # 6. Advance snapshot vs tabela Python
        exp_adv = self._tables.advance_at(snap.rpm, snap.map_bar_x100)
        adv_tol = TOL_ADVANCE_DEG
        if snap.sensor_fault and snap.rpm >= 3000:
            adv_tol = 8.0  # limp mode retards spark near rev limit
        r.add_range("Advance (snapshot vs tabela)",
                    snap.advance_deg, exp_adv, adv_tol, fmt=".1f")

        # 7. PW injecção — espelha calc_fuel_pw_us_default_fast + calc_final_pw_us
        #    (fuel_calc.cpp). base = req_fuel × ve/100 × MAP/baro; depois λ-target,
        #    trim, correcções CLT/IAT (×256) e dead-time interpolado por Vbatt.
        #    Sem motor MAP ≈ baro, firmware usa g_baro_bar_x100 (amostrado ao boot).
        baro_raw = snap.map_bar_x100
        baro = baro_raw if 70 <= baro_raw <= 110 else (self._eng.map_ref_bar_x100 or 100)
        base_us = (self._eng.req_fuel_us * exp_ve * snap.map_bar_x100
                   / (100.0 * baro))
        lambda_target = self._tables.lambda_target_at(snap.rpm, snap.map_bar_x100)
        if 650 <= lambda_target <= 1200:
            lambda_us = base_us * 1000.0 / lambda_target
        else:
            lambda_us = base_us
        clt_c = self._tables.clt_corr(snap.clt_degc)
        iat_c = self._tables.iat_corr(snap.iat_degc)
        corrected_us = lambda_us * clt_c * iat_c
        dead_time = self._tables.dead_time_us()
        exp_pw_ms = (corrected_us + dead_time) / 1000.0
        if snap.sensor_fault and snap.inj_pw_ms == 0.0 and snap.rpm >= 3000:
            r.add("Injection PW (snapshot vs cálculo)", True,
                  "PW=0 esperado (limp rev-cut com SENSOR_FAULT)")
        else:
            r.add_range("Injection PW (snapshot vs cálculo)",
                        snap.inj_pw_ms, exp_pw_ms, TOL_INJ_PW_PCT,
                        fmt=".3f", pct=True)

        # (As etapas de scope — firing order, inter-cilindro, PW/dwell medidos —
        #  exigem o esp32_combined com osciloscópio. O estimulador não tem scope;
        #  para cobertura de saídas IGN/INJ, usar um 2º ESP32 com esp32_scope.)
        return r

# ══════════════════════════════════════════════════════════════════════════════
# Relatório markdown
# ══════════════════════════════════════════════════════════════════════════════

def write_report(results: list[TestResult], eng: EngineConfig, path: str):
    total  = sum(len(r.checks) for r in results)
    passed = sum(sum(1 for _, ok, _ in r.checks if ok) for r in results)

    lines = [
        "# OpenEMS HIL Test Report",
        f"**Data:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"**Resultados:** {passed}/{total} PASS — {total-passed} FAIL",
        "",
        "## Motor",
        f"| Parâmetro | Valor |",
        f"|-----------|-------|",
        f"| Cilindrada | {eng.displacement_cc} cc |",
        f"| Injector | {eng.injector_flow_cc_min} cc/min |",
        f"| Stoich AFR | {eng.stoich_afr_x100/100:.2f} :1 |",
        f"| req_fuel | {eng.req_fuel_us} µs |",
        f"| Trigger offset | {eng.trigger_tooth0_engine_deg}° |",
        "",
        "## Resultados",
        "",
    ]

    for r in results:
        status = "✅ PASS" if r.passed() else "❌ FAIL"
        lines.append(f"### RPM={r.rpm_cmd}  {r.description}  {status}")
        if r.snap:
            s = r.snap
            lines.append(
                f"**Snapshot:** RPM={s.rpm}  MAP={s.map_bar_x100/100:.2f}bar"
                f"  VE={s.ve}%  ADV={s.advance_deg}°  PW={s.inj_pw_ms:.3f}ms"
                f"  CLT={s.clt_degc}°C  IAT={s.iat_degc}°C"
                f"  status=0x{s.status:04X}"
            )
        if r.timing and not r.timing.error:
            lines.append(
                f"**Firing:** {'→'.join(f'IGN{c}' for c in r.timing.firing_order)}"
                f"  {'✓' if r.timing.ok else '✗'}"
            )
        lines += ["", "| Verificação | Resultado | Detalhe |",
                  "|------------|-----------|---------|"]
        for name, ok, detail in r.checks:
            lines.append(f"| {name} | {'✓' if ok else '✗ **FALHOU**'} | {detail} |")
        lines.append("")

    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"Relatório → {path}")

# ══════════════════════════════════════════════════════════════════════════════
# Matriz de testes por defeito
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class TestPoint:
    rpm: int
    map_kpa: int = 50
    clt: int = 90
    iat: int = 30
    description: str = ""

DEFAULT_POINTS = [
    TestPoint(800,  35, 90, 30, "idle"),
    TestPoint(1000, 40, 90, 30, "idle alto"),
    TestPoint(1500, 50, 90, 35, "saída de idle"),
    TestPoint(2000, 55, 90, 35, "carga parcial"),
    TestPoint(3000, 70, 90, 40, "carga média"),
    TestPoint(4000, 90, 90, 40, "carga alta"),
]

# ══════════════════════════════════════════════════════════════════════════════
# main()
# ══════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="OpenEMS HIL Test — STM32 (CDC) + esp32_stimulator (CKP RMT)")
    p.add_argument("--stm32",  required=True, help="Porta STM32 CDC (ex: /dev/ttyACM0)")
    p.add_argument("--stim",   required=True,
                   help="Estimulador: serial /dev/ttyUSB0 ou TCP tcp:192.168.15.169:3333")
    p.add_argument("--baud",   type=int, default=115200)
    p.add_argument("--rpms",   nargs="+", type=int, default=None,
                   help="RPMs a testar (default: matriz interna). Ex: 800 1500 3000")
    p.add_argument("--report", default=None, metavar="FILE",
                   help="Gerar relatório markdown")
    p.add_argument("--bench-clt-iat", action="store_true",
                   help="Ativa bench-mode CLT/IAT na ECU (cmd 'B'): força 90°C/25°C "
                        "e limpa SENSOR_FAULT desses canais (sensores físicos ausentes)")
    p.add_argument("--mode", choices=["regression", "verify", "timing"], default="regression",
                   help="Modo: regression (math validation), verify (event scheduler outputs), "
                        "timing (scope 720° analysis via combined)")
    p.add_argument("--smoke", action="store_true",
                   help="Quick smoke test: 3 RPM points, fail-fast")
    p.add_argument("--scope-port", default="/dev/ttyUSB0",
                   help="ESP32 scope/combined serial port (modo timing)")
    args = p.parse_args()

    print("╔══════════════════════════════════════╗")
    print("║   OpenEMS HIL Test                   ║")
    print("╚══════════════════════════════════════╝")

    print(f"\nSTM32 : {args.stm32}")
    stm32 = STM32Client(args.stm32, args.baud)
    if not stm32.ping():
        print("ERRO: STM32 não responde — verificar porta e firmware")
        sys.exit(1)
    print("  ✓ STM32 OK")

    if args.bench_clt_iat:
        if stm32.set_bench_clt_iat(True):
            print("  ✓ bench-mode CLT/IAT ON (90°C/25°C, fault CLT/IAT limpo)")
        else:
            print("  ⚠ bench-mode CLT/IAT: sem ACK (firmware sem suporte ao cmd 'B'?)")

    print(f"Estim.: {args.stim}  (esp32_stimulator — CKP RMT)")
    stim = StimClient(args.stim, args.baud)
    print("  ✓ estimulador OK")

    print("\nA ler config e tabelas da ECU...")
    eng    = stm32.read_engine_config()
    tables = stm32.read_tables()
    print(f"  displacement={eng.displacement_cc}cc  "
          f"inj={eng.injector_flow_cc_min}cc/min  "
          f"stoich={eng.stoich_afr_x100/100:.2f}  "
          f"req_fuel={eng.req_fuel_us}µs")
    print(f"  VE={'OK' if tables.ve_table else 'VAZIA'}  "
          f"Spark={'OK' if tables.spark_table else 'VAZIA'}  "
          f"Dwell={'OK' if tables.dwell_ms_x10 else 'VAZIA'}")

    if args.rpms:
        points = [TestPoint(rpm) for rpm in args.rpms]
    elif args.smoke:
        points = [TestPoint(rpm) for rpm in [800, 1500, 3000]]
    else:
        points = DEFAULT_POINTS
    runner = HILRunner(stm32, stim, eng, tables, bench_mode=args.bench_clt_iat)
    runner.set_mode(args.mode, scope_port=args.scope_port if args.mode == "timing" else None)
    results = runner.run(points)

    total  = sum(len(r.checks) for r in results)
    passed = sum(sum(1 for _, ok, _ in r.checks if ok) for r in results)
    failed = total - passed

    print(f"\n{'═'*60}")
    print(f"  RESULTADOS FINAIS: {passed}/{total} PASS   {failed} FAIL")
    print(f"{'═'*60}")
    for r in results:
        fails = [n for n, ok, _ in r.checks if not ok]
        sym   = "✓" if r.passed() else "✗"
        extra = f"  → {', '.join(fails)}" if fails else ""
        print(f"  {sym}  RPM={r.rpm_cmd:5d}{extra}")

    if args.report:
        write_report(results, eng, args.report)

    stm32.close()
    stim.close()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
