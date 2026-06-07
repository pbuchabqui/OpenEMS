#!/usr/bin/env python3
"""
hil_test.py — Teste HIL (Hardware-in-the-Loop) automatizado para OpenEMS
══════════════════════════════════════════════════════════════════════════
Controla o ESP32 CKP generator para variar RPM, lê o snapshot do STM32
via UART e verifica que os valores calculados pelo firmware coincidem com
os valores esperados calculados em Python a partir das tabelas lidas da ECU.

Opcionalmente lê o ESP32 scope para verificação dos pulsos no hardware.

Uso mínimo (sem scope):
    python3 hil_test.py --stm32 /dev/ttyUSB0 --gen /dev/ttyUSB1

Com scope:
    python3 hil_test.py --stm32 /dev/ttyUSB0 --gen /dev/ttyUSB1 \\
                        --scope /dev/ttyUSB2 --report report.md

Arquitectura de verificação (por ponto de teste):
    1. CKP generator → RPM comandado
    2. Aguardar steady-state (2 s)
    3. STM32 snapshot → {rpm, map, ve, advance, pw, dwell, status}
    4. Tabelas lidas da ECU + sensor values do snapshot
       → calcular expected_{ve, advance, pw, dwell} em Python
    5. Scope timing 't' → firing order + inter-cylinder offset (se disponível)
    6. Scope LIVE → dwell PW + injection PW medidos (se disponível)
    7. Comparar: PASS se dentro das tolerâncias
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

# ═══════════════════════════════════════════════════════════════════════════════
# Configuração de tolerâncias
# ═══════════════════════════════════════════════════════════════════════════════

TOL_RPM_PCT       = 5.0   # % de tolerância no RPM medido vs comandado
TOL_DWELL_MS      = 0.3   # ms de tolerância no dwell medido vs esperado
TOL_INJ_PW_PCT    = 15.0  # % de tolerância no PW injecção (firmware aplica corr. não modeladas)
TOL_ADVANCE_DEG   = 2.0   # ° de tolerância no avanço de ignição
TOL_INTER_CYL_MS  = 2.0   # ms de tolerância no espaçamento entre cilindros
EXPECTED_ORDER    = [0, 2, 3, 1]  # IGN0→IGN2→IGN3→IGN1 (1-3-4-2)

SETTLE_S = 2.0    # segundos a aguardar após mudança de RPM

# ═══════════════════════════════════════════════════════════════════════════════
# Eixos das tabelas 3D (kRpmAxisX10 e kLoadAxisBarX100, de table3d.cpp)
# ═══════════════════════════════════════════════════════════════════════════════

RPM_AXIS_X10 = [
    5000, 7500, 10000, 12500,
    15000, 20000, 25000, 30000,
    40000, 50000, 60000, 70000,
    80000, 90000, 100000, 120000,
]

MAP_AXIS_BAR_X100 = [
    20, 30, 40, 50,
    60, 70, 80, 90,
    100, 115, 130, 145,
    160, 180, 215, 300,
]

# ═══════════════════════════════════════════════════════════════════════════════
# Mirror exacto do math do firmware (table3d.cpp + fuel_calc.cpp)
# ═══════════════════════════════════════════════════════════════════════════════

def _axis_lookup(axis: list[int], value: int) -> tuple[int, int]:
    """
    Devolve (idx, frac_q8) — mirror de axis_lookup() em table3d.cpp.
    idx: índice do intervalo inferior.
    frac_q8: posição fraccionária em Q8 (0-255).
    """
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
    span = x1 - x0
    frac = ((value - x0) << 8) // span
    return idx, min(255, frac)


def _lerp_q8(a: int, b: int, frac_q8: int) -> int:
    """Mirror de lerp_q8_s32()."""
    if frac_q8 == 255:
        return b
    return a + (((b - a) * frac_q8) >> 8)


def table3d_lookup_u8(table: list[list[int]],
                      x_axis: list[int], y_axis: list[int],
                      x: int, y: int) -> int:
    """
    Interpolação bilinear — mirror exacto de table3d_lookup_u8_prepared().
    table[yi][xi], x=RPM (cols), y=MAP/load (rows).
    """
    xi, fx = _axis_lookup(x_axis, x)
    yi, fy = _axis_lookup(y_axis, y)
    v00 = table[yi][xi];     v10 = table[yi][xi + 1]
    v01 = table[yi + 1][xi]; v11 = table[yi + 1][xi + 1]
    v0 = _lerp_q8(v00, v10, fx)
    v1 = _lerp_q8(v01, v11, fx)
    v  = _lerp_q8(v0,  v1,  fy)
    return max(0, min(255, v))


def table3d_lookup_i8(table: list[list[int]],
                      x_axis: list[int], y_axis: list[int],
                      x: int, y: int) -> int:
    """Mirror de table3d_lookup_i8_prepared() (signed)."""
    xi, fx = _axis_lookup(x_axis, x)
    yi, fy = _axis_lookup(y_axis, y)
    v00 = table[yi][xi];     v10 = table[yi][xi + 1]
    v01 = table[yi + 1][xi]; v11 = table[yi + 1][xi + 1]
    v0 = _lerp_q8(v00, v10, fx)
    v1 = _lerp_q8(v01, v11, fx)
    return _lerp_q8(v0, v1, fy)


def interp_1d(axis: list[int], values: list[int], x: int) -> int:
    """Interpolação 1D linear simples (para tabelas de correcção)."""
    idx, frac = _axis_lookup(axis, x)
    return _lerp_q8(values[idx], values[idx + 1], frac)


# ═══════════════════════════════════════════════════════════════════════════════
# Estruturas de dados
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class EngineConfig:
    ivc_abdc_deg:              int = 50
    displacement_cc:           int = 2000
    injector_flow_cc_min:      int = 450
    stoich_afr_x100:           int = 1300
    map_ref_bar_x100:          int = 100
    trigger_tooth0_engine_deg: int = 0
    default_soi_lead_deg:      int = 62

    @property
    def req_fuel_us(self) -> int:
        """
        Mirror EXACTO de calc_req_fuel_us() — integer arithmetic idêntica.
        num = displacement * kAirDensityMgPerCcX1000 * 100 * 60_000_000
        den = cylinders * stoich_x100 * inj_flow_cc_min * kFuelDensityMgPerCc * 1000
        """
        if (self.displacement_cc == 0 or self.injector_flow_cc_min == 0
                or self.stoich_afr_x100 == 0):
            return 0
        AIR_X1000    = 1184   # kAirDensityMgPerCcX1000
        FUEL_DENSITY = 755    # kFuelDensityMgPerCc
        CYLINDERS    = 4
        num = (self.displacement_cc * AIR_X1000 * 100 * 60_000_000)
        den = (CYLINDERS * self.stoich_afr_x100
               * self.injector_flow_cc_min * FUEL_DENSITY * 1000)
        return min(num // den, 50000)


@dataclass
class Snapshot:
    rpm:            int   = 0
    map_bar_x100:   int   = 100   # MAP em bar × 100
    tps_pct:        int   = 0
    clt_degc:       int   = 25    # (clt_p40 - 40)
    iat_degc:       int   = 25    # (iat_p40 - 40)
    pw1_ms_x10:     int   = 0
    advance_deg:    int   = 0     # (advance_p40 - 40)
    ve:             int   = 0
    stft_pct:       int   = 0
    status:         int   = 0

    @property
    def full_sync(self) -> bool: return bool(self.status & 0x0001)
    @property
    def sensor_fault(self) -> bool: return bool(self.status & 0x0004)
    @property
    def late_event(self) -> bool: return bool(self.status & 0x0040)
    @property
    def inj_pw_ms(self) -> float: return self.pw1_ms_x10 / 10.0


@dataclass
class Tables:
    ve_table:     list[list[int]] = field(default_factory=list)  # [16][16] uint8
    spark_table:  list[list[int]] = field(default_factory=list)  # [16][16] int8 signed
    dwell_vbatt_axis_mv:  list[int] = field(default_factory=list)  # 16 values
    dwell_ms_x10:         list[int] = field(default_factory=list)  # 16 values
    clt_corr_axis_x10:    list[int] = field(default_factory=list)
    clt_corr_x256:        list[int] = field(default_factory=list)
    iat_corr_axis_x10:    list[int] = field(default_factory=list)
    iat_corr_x256:        list[int] = field(default_factory=list)
    vbatt_corr_axis_mv:   list[int] = field(default_factory=list)
    injector_dead_time_us: list[int] = field(default_factory=list)

    def ve_at(self, rpm: int, map_bar_x100: int) -> int:
        """VE% na tabela 3D para (rpm, MAP)."""
        return table3d_lookup_u8(
            self.ve_table,
            RPM_AXIS_X10, MAP_AXIS_BAR_X100,
            rpm * 10, map_bar_x100)

    def advance_at(self, rpm: int, map_bar_x100: int) -> int:
        """Avanço em graus na tabela 3D para (rpm, MAP). signed int8."""
        return table3d_lookup_i8(
            self.spark_table,
            RPM_AXIS_X10, MAP_AXIS_BAR_X100,
            rpm * 10, map_bar_x100)

    def dwell_ms_at(self, vbatt_mv: int) -> float:
        """Dwell em ms para a tensão de bateria actual."""
        if not self.dwell_vbatt_axis_mv or not self.dwell_ms_x10:
            return 3.0   # default
        return interp_1d(self.dwell_vbatt_axis_mv,
                         self.dwell_ms_x10, vbatt_mv) / 10.0

    def clt_corr_at(self, clt_degc: int) -> float:
        """Correcção CLT em fracção (1.0 = sem correcção)."""
        if not self.clt_corr_axis_x10:
            return 1.0
        v = interp_1d(self.clt_corr_axis_x10, self.clt_corr_x256,
                      clt_degc * 10)
        return v / 256.0

    def iat_corr_at(self, iat_degc: int) -> float:
        if not self.iat_corr_axis_x10:
            return 1.0
        v = interp_1d(self.iat_corr_axis_x10, self.iat_corr_x256,
                      iat_degc * 10)
        return v / 256.0


@dataclass
class TestPoint:
    rpm:         int
    description: str = ""

@dataclass
class TestResult:
    rpm_cmd:       int
    description:   str
    snap:          Optional[Snapshot]  = None
    timing_order:  Optional[list[int]] = None
    inter_cyl_ms:  Optional[float]     = None
    scope_dwell_ms: Optional[float]    = None
    scope_pw_ms:    Optional[float]    = None
    checks:        list[tuple[str, bool, str]] = field(default_factory=list)

    def passed(self) -> bool:
        return all(ok for _, ok, _ in self.checks)

    def add_check(self, name: str, ok: bool, detail: str = ""):
        self.checks.append((name, ok, detail))

    def add_range(self, name: str, actual, expected, tol,
                  fmt=".3f", pct=False):
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
        self.add_check(name, ok, detail)


# ═══════════════════════════════════════════════════════════════════════════════
# STM32 client (protocolo UART OpenEMS)
# ═══════════════════════════════════════════════════════════════════════════════

class STM32Client:
    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(0.3)
        self._ser.reset_input_buffer()

    def _cmd(self, data: bytes, read_n: int, delay: float = 0.05) -> bytes:
        self._ser.write(data)
        time.sleep(delay)
        return self._ser.read(read_n)

    def ping(self) -> bool:
        r = self._cmd(b"\x43", 2)       # 'C' → ACK + magic
        return len(r) == 2 and r[0] == 0x00 and r[1] == 0xAA

    def read_page(self, page: int, offset: int, length: int) -> bytes:
        cmd = bytes([0x72, page,
                     offset & 0xFF, (offset >> 8) & 0xFF,
                     length & 0xFF, (length >> 8) & 0xFF])
        r = self._cmd(cmd, length, delay=0.05 + length / 10000)
        return r

    def snapshot(self) -> Optional[Snapshot]:
        r = self._cmd(b"\x41", 64, delay=0.05)
        if len(r) < 13:
            return None
        s = Snapshot()
        s.rpm           = struct.unpack_from("<H", r, 0)[0]
        s.map_bar_x100  = r[2]
        s.tps_pct       = r[3]
        s.clt_degc      = r[4] - 40
        s.iat_degc      = r[5] - 40
        s.pw1_ms_x10    = r[7]
        s.advance_deg   = r[8] - 40
        s.ve            = r[9]
        s.stft_pct      = struct.unpack_from("b", r, 10)[0]
        s.status        = struct.unpack_from("<H", r, 11)[0]
        return s

    def read_engine_config(self) -> EngineConfig:
        d = self.read_page(0, 0, 16)
        if len(d) < 16:
            return EngineConfig()
        return EngineConfig(
            ivc_abdc_deg              = d[0],
            displacement_cc           = struct.unpack_from("<H", d, 2)[0],
            injector_flow_cc_min      = struct.unpack_from("<H", d, 4)[0],
            stoich_afr_x100           = struct.unpack_from("<H", d, 6)[0],
            map_ref_bar_x100          = struct.unpack_from("<H", d, 8)[0],
            trigger_tooth0_engine_deg = struct.unpack_from("<H", d, 10)[0],
            default_soi_lead_deg      = struct.unpack_from("<H", d, 12)[0],
        )

    def read_tables(self) -> Tables:
        t = Tables()

        # VE table — page 1, 256 bytes, layout [16 rows][16 cols] = [MAP][RPM]
        raw = self.read_page(1, 0, 256)
        if len(raw) == 256:
            t.ve_table = [[raw[r * 16 + c] for c in range(16)] for r in range(16)]

        # Spark table — page 2, 256 bytes, signed int8
        raw = self.read_page(2, 0, 256)
        if len(raw) == 256:
            t.spark_table = [
                [struct.unpack_from("b", raw, r * 16 + c)[0] for c in range(16)]
                for r in range(16)]

        # Corrections — page 5
        # offsets: 96=dwell_vbatt_axis, 160=dwell_vbatt_axis (re-check),
        # Conforme sync_table_from_page() em ui_protocol.cpp:
        #   p+  0: clt_corr_axis_x10   (16B)
        #   p+ 16: clt_corr_x256       (16B)
        #   p+ 32: iat_corr_axis_x10   (16B)
        #   p+ 48: iat_corr_x256       (16B)
        #   p+ 96: vbatt_corr_axis_mv  (16B u16 LE → 8 values)
        #   p+112: injector_dead_time_us(16B)
        #   p+160: dwell_vbatt_axis_mv (16B u16 LE → 8 values)
        #   p+176: dwell_ms_x10_table  (16B u16 LE → 8 values)
        raw = self.read_page(5, 0, 192)
        if len(raw) >= 192:
            def u16s(buf, off, n):
                return [struct.unpack_from("<H", buf, off + i * 2)[0]
                        for i in range(n)]
            def u8s(buf, off, n):
                return list(buf[off:off + n])

            t.clt_corr_axis_x10   = u8s(raw,   0, 16)
            t.clt_corr_x256       = u8s(raw,  16, 16)
            t.iat_corr_axis_x10   = u8s(raw,  32, 16)
            t.iat_corr_x256       = u8s(raw,  48, 16)
            t.vbatt_corr_axis_mv  = u16s(raw, 96, 8)
            t.injector_dead_time_us = u16s(raw, 112, 8)
            t.dwell_vbatt_axis_mv = u16s(raw, 160, 8)
            t.dwell_ms_x10        = u16s(raw, 176, 8)

        return t

    def close(self):
        self._ser.close()


# ═══════════════════════════════════════════════════════════════════════════════
# CKP generator client (esp32_ckp_gen.ino)
# ═══════════════════════════════════════════════════════════════════════════════

class CKPGenClient:
    _PRESETS = [100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000]

    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=1.0)
        self._rpm = 500
        time.sleep(0.3)
        self._ser.reset_input_buffer()

    def _send(self, c: str):
        self._ser.write(c.encode())
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def set_rpm(self, target_rpm: int):
        """Ajusta RPM. Usa presets quando possível, senão usa +/-."""
        # Tentar preset exacto
        if target_rpm in self._PRESETS:
            idx = self._PRESETS.index(target_rpm)
            self._send(str(idx))
            self._rpm = target_rpm
            return

        # Ajustar em passos de 100
        current = self._rpm
        while current != target_rpm:
            if current < target_rpm:
                self._send("+")
                current = min(current + 100, target_rpm)
            else:
                self._send("-")
                current = max(current - 100, target_rpm)
        self._rpm = target_rpm

    def close(self):
        self._ser.close()


# ═══════════════════════════════════════════════════════════════════════════════
# Scope client (esp32_scope.ino)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class LiveData:
    pw_ms:     float = 0.0
    period_ms: float = 0.0
    count:     int   = 0
    idle:      bool  = True

@dataclass
class TimingData:
    firing_order:  list[int]         = field(default_factory=list)
    inter_cyl_ms:  list[float]       = field(default_factory=list)
    from_gap_ms:   dict[int, float]  = field(default_factory=dict)
    ok:            bool              = False
    error:         str               = ""


class ScopeClient:
    def __init__(self, port: str, baud: int = 115200):
        self._ser = serial.Serial(port, baud, timeout=3.0)
        time.sleep(0.3)
        self._ser.reset_input_buffer()

    def _send(self, c: str):
        self._ser.write(c.encode())
        time.sleep(0.05)

    def get_live(self, wait_s: float = 1.5) -> dict[int, LiveData]:
        """Lê uma tabela LIVE do scope (modo 'l')."""
        self._send("l")
        time.sleep(wait_s)
        raw = self._ser.read(self._ser.in_waiting)
        text = raw.decode("utf-8", errors="replace")

        # Parsa linhas: "| 0|IGN0  |PC6    |   3.021|  240.00| ..."
        RE = re.compile(
            r"\|\s*(\d+)\|(\w+)\s*\|\w+\s*\|\s*([\d.]+)\|\s*([\d.]+)\|\s*[\d.]+\|\s*(\d+)\|"
        )
        result: dict[int, LiveData] = {}
        for m in RE.finditer(text):
            ch, name, pw, period, count = m.groups()
            idle = ("IDLE" in text[m.start():m.start()+120])
            result[int(ch)] = LiveData(
                pw_ms     = float(pw),
                period_ms = float(period),
                count     = int(count),
                idle      = idle,
            )
        return result

    def run_timing(self, timeout_s: float = 30.0) -> TimingData:
        """Executa o modo 't' e parseia o relatório de timing."""
        self._send("t")
        deadline = time.time() + timeout_s
        lines = []

        # Acumular output até ver "Resultado final" ou timeout
        while time.time() < deadline:
            line = self._ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
            if "Resultado final" in line or "IGNIÇÃO OK" in line or "VER FALHAS" in line:
                break

        text = "\n".join(lines)

        td = TimingData()

        # Parsar linhas de spark: "  IGN0    15.234 ms ..."
        RE_SPARK = re.compile(r"IGN(\d)\s+([\d.]+)\s*ms")
        spark_entries = []
        for m in RE_SPARK.finditer(text):
            ch, dt = int(m.group(1)), float(m.group(2))
            spark_entries.append((ch, dt))

        if len(spark_entries) < 4:
            td.error = f"Spark entries insuficientes: {len(spark_entries)}/4"
            return td

        # Ordenar por timestamp para obter firing order
        spark_entries.sort(key=lambda x: x[1])
        td.firing_order = [ch for ch, _ in spark_entries]

        # from_gap_ms
        for ch, dt in spark_entries:
            td.from_gap_ms[ch] = dt

        # inter-cylinder offsets
        for i in range(1, len(spark_entries)):
            dt = spark_entries[i][1] - spark_entries[i-1][1]
            td.inter_cyl_ms.append(dt)

        td.ok = "IGNIÇÃO OK" in text
        if "VER FALHAS" in text:
            td.error = "Scope reportou falhas"

        return td

    def close(self):
        self._ser.close()


# ═══════════════════════════════════════════════════════════════════════════════
# HIL Runner
# ═══════════════════════════════════════════════════════════════════════════════

class HILRunner:
    def __init__(self,
                 stm32:  STM32Client,
                 gen:    CKPGenClient,
                 scope:  Optional[ScopeClient],
                 eng:    EngineConfig,
                 tables: Tables):
        self._stm  = stm32
        self._gen  = gen
        self._scope = scope
        self._eng   = eng
        self._tables = tables
        self.results: list[TestResult] = []

    def _wait_sync(self, timeout_s: float = 8.0) -> bool:
        """Aguarda FULL_SYNC no snapshot."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            snap = self._stm.snapshot()
            if snap and snap.full_sync:
                return True
            time.sleep(0.2)
        return False

    def run(self, points: list[TestPoint]) -> list[TestResult]:
        for pt in points:
            print(f"\n{'─'*60}")
            print(f"  RPM={pt.rpm}  {pt.description}")
            print(f"{'─'*60}")
            r = self._test_point(pt)
            self.results.append(r)
            for name, ok, detail in r.checks:
                sym = "✓" if ok else "✗"
                print(f"  {sym}  {name}")
                if detail:
                    print(f"      {detail}")
        return self.results

    def _test_point(self, pt: TestPoint) -> TestResult:
        r = TestResult(rpm_cmd=pt.rpm, description=pt.description)

        # 1. Comandar RPM
        print(f"  → Comandar RPM={pt.rpm}...")
        self._gen.set_rpm(pt.rpm)

        # 2. Aguardar steady-state e FULL_SYNC
        time.sleep(SETTLE_S)
        synced = self._wait_sync()
        r.add_check("FULL_SYNC", synced,
                    "" if synced else "Sem sync após 8 s — verificar sinal CKP")
        if not synced:
            return r

        # 3. Ler snapshot
        snap = self._stm.snapshot()
        if not snap:
            r.add_check("Snapshot lido", False, "Sem resposta da ECU")
            return r
        r.snap = snap

        # 4. Verificações de status
        r.add_check("Sem sensor fault", not snap.sensor_fault)
        r.add_check("Sem late events", not snap.late_event,
                    f"status=0x{snap.status:04X}")

        # 5. RPM medido vs comandado
        r.add_range("RPM medido", snap.rpm, pt.rpm, TOL_RPM_PCT,
                    fmt=".0f", pct=True)

        # 6. VE: valor do snapshot vs lookup Python na tabela
        expected_ve = self._tables.ve_at(snap.rpm, snap.map_bar_x100)
        r.add_range("VE da tabela",
                    snap.ve, expected_ve, 3.0,   # ±3 % (arredondamento Q8)
                    fmt=".0f")

        # 7. Avanço de ignição: snapshot vs lookup Python
        expected_adv = self._tables.advance_at(snap.rpm, snap.map_bar_x100)
        r.add_range("Advance da tabela",
                    snap.advance_deg, expected_adv, TOL_ADVANCE_DEG,
                    fmt=".1f")

        # 8. PW de injecção: snapshot vs cálculo Python
        #    expected_pw = req_fuel × VE/100 × clt_corr × iat_corr
        #    (STFT não modelado — tolerância mais larga)
        req_us   = self._eng.req_fuel_us
        clt_corr = self._tables.clt_corr_at(snap.clt_degc)
        iat_corr = self._tables.iat_corr_at(snap.iat_degc)
        ve_frac  = snap.ve / 100.0
        expected_pw_ms = (req_us * ve_frac * clt_corr * iat_corr) / 1000.0

        r.add_range("Injection PW",
                    snap.inj_pw_ms, expected_pw_ms, TOL_INJ_PW_PCT,
                    fmt=".3f", pct=True)

        # 9. Scope: timing de ignição (se disponível)
        if self._scope:
            print("  → Scope timing analysis...")
            td = self._scope.run_timing(timeout_s=25.0)

            if td.error:
                r.add_check("Scope timing", False, td.error)
            else:
                r.timing_order   = td.firing_order
                r.inter_cyl_ms   = sum(td.inter_cyl_ms) / max(len(td.inter_cyl_ms), 1)

                order_ok = (td.firing_order == EXPECTED_ORDER)
                r.add_check("Firing order",
                            order_ok,
                            f"detectada={'→'.join(f'IGN{c}' for c in td.firing_order)}"
                            f"  esperada={'→'.join(f'IGN{c}' for c in EXPECTED_ORDER)}")

                expected_inter = snap.rpm and (2 * 60_000 / snap.rpm / 4)
                for i, ic in enumerate(td.inter_cyl_ms):
                    r.add_range(f"Inter-cil [cyl{i+1}]",
                                ic, expected_inter, TOL_INTER_CYL_MS, fmt=".3f")

            # 10. Scope: PW medido no hardware
            print("  → Scope live data...")
            live = self._scope.get_live(wait_s=1.5)
            if 4 in live and not live[4].idle:     # INJ0
                r.scope_pw_ms = live[4].pw_ms
                r.add_range("INJ0 PW (scope vs snapshot)",
                            r.scope_pw_ms, snap.inj_pw_ms, TOL_DWELL_MS,
                            fmt=".3f")
            if 0 in live and not live[0].idle:     # IGN0 dwell
                r.scope_dwell_ms = live[0].pw_ms
                # Dwell esperado da tabela (vbatt não disponível no snapshot → usar 12000 mV default)
                expected_dwell = self._tables.dwell_ms_at(12000)
                r.add_range("IGN0 Dwell (scope vs tabela)",
                            r.scope_dwell_ms, expected_dwell, TOL_DWELL_MS,
                            fmt=".3f")

        return r


# ═══════════════════════════════════════════════════════════════════════════════
# Relatório markdown
# ═══════════════════════════════════════════════════════════════════════════════

def generate_report(results: list[TestResult], eng: EngineConfig,
                    path: Optional[str] = None) -> str:
    total  = sum(len(r.checks) for r in results)
    passed = sum(sum(1 for _, ok, _ in r.checks if ok) for r in results)
    failed = total - passed

    lines = [
        "# OpenEMS HIL Test Report",
        f"**Data:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"**Resultados:** {passed}/{total} PASS  —  {failed} FAIL",
        "",
        "## Configuração do Motor",
        f"| Parâmetro | Valor |",
        f"|-----------|-------|",
        f"| Cilindrada | {eng.displacement_cc} cc |",
        f"| Injector | {eng.injector_flow_cc_min} cc/min |",
        f"| Stoich AFR | {eng.stoich_afr_x100/100:.2f} :1 |",
        f"| req_fuel | {eng.req_fuel_us:.0f} µs |",
        f"| Trigger offset | {eng.trigger_tooth0_engine_deg}° |",
        "",
        "## Resultados por Ponto de Teste",
        "",
    ]

    for r in results:
        status = "✅ PASS" if r.passed() else "❌ FAIL"
        lines.append(f"### RPM={r.rpm_cmd} — {r.description}  {status}")
        if r.snap:
            s = r.snap
            lines += [
                f"**Snapshot:** RPM={s.rpm}  MAP={s.map_bar_x100/100:.2f}bar"
                f"  VE={s.ve}%  ADV={s.advance_deg}°  PW={s.inj_pw_ms:.3f}ms"
                f"  CLT={s.clt_degc}°C  IAT={s.iat_degc}°C  "
                f"status=0x{s.status:04X}",
                "",
            ]
        if r.timing_order:
            lines.append(f"**Firing order:** "
                         f"{'→'.join(f'IGN{c}' for c in r.timing_order)}")
        if r.scope_dwell_ms:
            lines.append(f"**Dwell (scope):** {r.scope_dwell_ms:.3f} ms")
        if r.scope_pw_ms:
            lines.append(f"**INJ PW (scope):** {r.scope_pw_ms:.3f} ms")

        lines.append("")
        lines.append("| Verificação | Resultado | Detalhe |")
        lines.append("|------------|-----------|---------|")
        for name, ok, detail in r.checks:
            sym = "✓" if ok else "✗ **FALHOU**"
            lines.append(f"| {name} | {sym} | {detail} |")
        lines.append("")

    text = "\n".join(lines)
    if path:
        with open(path, "w") as f:
            f.write(text)
        print(f"\nRelatório guardado em {path}")
    return text


# ═══════════════════════════════════════════════════════════════════════════════
# Matrix de testes
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_TEST_MATRIX = [
    TestPoint(500,  "cranking — idle baixo"),
    TestPoint(750,  "fast idle"),
    TestPoint(1000, "idle normal"),
    TestPoint(1500, "saída de idle"),
    TestPoint(2000, "carga parcial"),
    TestPoint(3000, "carga média"),
]


# ═══════════════════════════════════════════════════════════════════════════════
# main()
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(description="OpenEMS HIL Test")
    p.add_argument("--stm32",  required=True,  help="Porta STM32 UART")
    p.add_argument("--gen",    required=True,  help="Porta ESP32 CKP generator")
    p.add_argument("--scope",  default=None,   help="Porta ESP32 scope (opcional)")
    p.add_argument("--baud",   type=int, default=115200)
    p.add_argument("--rpms",   nargs="+", type=int,
                   default=[pt.rpm for pt in DEFAULT_TEST_MATRIX],
                   help="Lista de RPMs a testar")
    p.add_argument("--report", default=None, help="Ficheiro markdown de saída")
    args = p.parse_args()

    print("╔══════════════════════════════════════════╗")
    print("║   OpenEMS HIL Test                       ║")
    print("╚══════════════════════════════════════════╝")

    # Ligar dispositivos
    print(f"\nSTM32  : {args.stm32}")
    stm32 = STM32Client(args.stm32, args.baud)
    if not stm32.ping():
        print("ERRO: STM32 não responde ao handshake 'C'")
        sys.exit(1)
    print("  ✓ STM32 OK")

    print(f"CKP Gen: {args.gen}")
    gen = CKPGenClient(args.gen, args.baud)
    print("  ✓ CKP generator OK")

    scope = None
    if args.scope:
        print(f"Scope  : {args.scope}")
        scope = ScopeClient(args.scope, args.baud)
        print("  ✓ Scope OK")
    else:
        print("Scope  : não configurado (só verificação software)")

    # Ler configuração e tabelas da ECU
    print("\nA ler configuração da ECU...")
    eng = stm32.read_engine_config()
    print(f"  displacement={eng.displacement_cc}cc  "
          f"inj={eng.injector_flow_cc_min}cc/min  "
          f"stoich={eng.stoich_afr_x100/100:.2f}  "
          f"req_fuel={eng.req_fuel_us:.0f}µs")

    print("A ler tabelas (VE, spark, dwell, correcções)...")
    tables = stm32.read_tables()
    print(f"  VE table: {'OK' if tables.ve_table else 'VAZIA (defaults)'}")
    print(f"  Spark:    {'OK' if tables.spark_table else 'VAZIA'}")
    print(f"  Dwell:    {'OK' if tables.dwell_ms_x10 else 'VAZIA'}")

    # Criar matriz de testes
    test_pts = [TestPoint(rpm) for rpm in args.rpms]

    # Correr
    runner = HILRunner(stm32, gen, scope, eng, tables)
    results = runner.run(test_pts)

    # Resumo
    total  = sum(len(r.checks) for r in results)
    passed = sum(sum(1 for _, ok, _ in r.checks if ok) for r in results)
    failed = total - passed

    print(f"\n{'═'*60}")
    print(f"  RESULTADOS: {passed}/{total} PASS   {failed} FAIL")
    print(f"{'═'*60}")
    for r in results:
        sym = "✓" if r.passed() else "✗"
        failed_names = [n for n, ok, _ in r.checks if not ok]
        detail = f"  FALHOU: {', '.join(failed_names)}" if failed_names else ""
        print(f"  {sym} RPM={r.rpm_cmd:5d}{detail}")

    # Relatório
    if args.report:
        generate_report(results, eng, args.report)

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
