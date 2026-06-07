#!/usr/bin/env python3
"""
hil_test.py — Teste HIL automático para OpenEMS
════════════════════════════════════════════════
Requer hardware:
  PC ─ USB0 → STM32H562  (UART protocolo OpenEMS)
  PC ─ USB1 → ESP32       (esp32_combined.ino: CKP gen + scope)

O ESP32 gera o sinal 60-2 para o STM32, monitoriza IGN/INJ e
reporta firing order + PW por série.

O script lê as tabelas da ECU (VE, spark, dwell), calcula os valores
esperados em Python com o mesmo math do firmware (integer arithmetic),
e compara com os valores medidos pelo scope e o snapshot UART.

Uso:
    python3 hil_test.py --stm32 /dev/ttyUSB0 --esp32 /dev/ttyUSB1
    python3 hil_test.py --stm32 /dev/ttyUSB0 --esp32 /dev/ttyUSB1 \\
                        --rpms 500 1000 2000 --report resultado.md
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
TOL_INJ_PW_PCT   = 15.0  # % — PW snapshot vs cálculo (STFT + dead time não modelados)
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
    40000, 50000, 60000, 70000,
    80000, 90000, 100000, 120000,
]

MAP_AXIS_BAR_X100 = [
    20,  30,  40,  50,
    60,  70,  80,  90,
    100, 115, 130, 145,
    160, 180, 215, 300,
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
    ve:           int = 0
    stft_pct:     int = 0
    status:       int = 0

    @property
    def full_sync(self) -> bool:    return bool(self.status & 0x0001)
    @property
    def sensor_fault(self) -> bool: return bool(self.status & 0x0004)
    @property
    def late_event(self) -> bool:   return bool(self.status & 0x0040)
    @property
    def inj_pw_ms(self) -> float:   return self.pw1_ms_x10 / 10.0


@dataclass
class Tables:
    ve_table:              list[list[int]] = field(default_factory=list)
    spark_table:           list[list[int]] = field(default_factory=list)
    dwell_vbatt_axis_mv:   list[int]       = field(default_factory=list)
    dwell_ms_x10:          list[int]       = field(default_factory=list)
    clt_corr_axis_x10:     list[int]       = field(default_factory=list)
    clt_corr_x256:         list[int]       = field(default_factory=list)
    iat_corr_axis_x10:     list[int]       = field(default_factory=list)
    iat_corr_x256:         list[int]       = field(default_factory=list)

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
        r = self._ser.read(64)
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
        s.status       = struct.unpack_from("<H", r, 11)[0]
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

        # page 5: CLT/IAT corr + dwell
        raw = self._read_page(5, 0, 192)
        if len(raw) >= 192:
            u8  = lambda off, n: list(raw[off:off + n])
            u16 = lambda off, n: [struct.unpack_from("<H", raw, off + i*2)[0]
                                  for i in range(n)]
            t.clt_corr_axis_x10  = u8(  0, 16)
            t.clt_corr_x256      = u8( 16, 16)
            t.iat_corr_axis_x10  = u8( 32, 16)
            t.iat_corr_x256      = u8( 48, 16)
            t.dwell_vbatt_axis_mv = u16(160, 8)
            t.dwell_ms_x10        = u16(176, 8)

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
    def __init__(self, stm32: STM32Client, esp32: ESP32Client,
                 eng: EngineConfig, tables: Tables):
        self._stm    = stm32
        self._esp    = esp32
        self._eng    = eng
        self._tables = tables
        self.results: list[TestResult] = []

    def _wait_sync(self) -> bool:
        deadline = time.time() + SYNC_WAIT_S
        while time.time() < deadline:
            s = self._stm.snapshot()
            if s and s.full_sync:
                return True
            time.sleep(0.2)
        return False

    def run(self, points: list[TestPoint]) -> list[TestResult]:
        for pt in points:
            print(f"\n{'─'*60}")
            print(f"  RPM={pt.rpm}  {pt.description}")
            print(f"{'─'*60}")
            r = self._run_point(pt)
            self.results.append(r)
            for name, ok, detail in r.checks:
                print(f"  {'✓' if ok else '✗'}  {name}")
                if detail:
                    print(f"      {detail}")
        return self.results

    def _run_point(self, pt: TestPoint) -> TestResult:
        r = TestResult(rpm_cmd=pt.rpm, description=pt.description)

        # 1. RPM
        print(f"  → RPM={pt.rpm}...")
        self._esp.set_rpm(pt.rpm)
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

        r.add("Sem sensor fault", not snap.sensor_fault)
        r.add("Sem late events",  not snap.late_event,
              f"status=0x{snap.status:04X}")

        # 4. RPM medido
        r.add_range("RPM medido", snap.rpm, pt.rpm, TOL_RPM_PCT,
                    fmt=".0f", pct=True)

        # 5. VE snapshot vs tabela Python
        exp_ve = self._tables.ve_at(snap.rpm, snap.map_bar_x100)
        r.add_range("VE (snapshot vs tabela)",
                    snap.ve, exp_ve, TOL_VE_UNITS, fmt=".0f")

        # 6. Advance snapshot vs tabela Python
        exp_adv = self._tables.advance_at(snap.rpm, snap.map_bar_x100)
        r.add_range("Advance (snapshot vs tabela)",
                    snap.advance_deg, exp_adv, TOL_ADVANCE_DEG, fmt=".1f")

        # 7. PW injecção: req_fuel × VE × correcções CLT/IAT
        clt_c = self._tables.clt_corr(snap.clt_degc)
        iat_c = self._tables.iat_corr(snap.iat_degc)
        exp_pw_ms = self._eng.req_fuel_us * (snap.ve / 100.0) * clt_c * iat_c / 1000.0
        r.add_range("Injection PW (snapshot vs cálculo)",
                    snap.inj_pw_ms, exp_pw_ms, TOL_INJ_PW_PCT,
                    fmt=".3f", pct=True)

        # 8. Timing analysis (scope interno do ESP32)
        print("  → Scope timing analysis...")
        td = self._esp.run_timing(timeout_s=25.0)
        r.timing = td

        if td.error:
            r.add("Scope timing capturado", False, td.error)
        else:
            order_ok = (td.firing_order == EXPECTED_FIRING_ORDER)
            r.add("Firing order",
                  order_ok,
                  f"detectada={'→'.join(f'IGN{c}' for c in td.firing_order)}"
                  f"  esperada={'→'.join(f'IGN{c}' for c in EXPECTED_FIRING_ORDER)}")

            exp_ic = 2 * 60_000 / max(snap.rpm, 1) / 4   # ms
            for i, ic in enumerate(td.inter_cyl_ms):
                r.add_range(f"Inter-cil [{i+1}]",
                            ic, exp_ic, TOL_INTER_CYL_MS, fmt=".3f")

        # 9. PW medidos no hardware (scope LIVE)
        print("  → Scope LIVE...")
        live = self._esp.get_live(wait_s=1.5)
        r.live = live

        if 4 in live and not live[4].idle:   # INJ0
            r.add_range("INJ0 PW (scope vs snapshot)",
                        live[4].pw_ms, snap.inj_pw_ms, TOL_HW_PW_MS, fmt=".3f")

        if 0 in live and not live[0].idle:   # IGN0 dwell
            exp_dwell = self._tables.dwell_ms_at()
            r.add_range("IGN0 Dwell (scope vs tabela)",
                        live[0].pw_ms, exp_dwell, TOL_DWELL_MS, fmt=".3f")

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
    description: str = ""

DEFAULT_POINTS = [
    TestPoint(500,  "cranking / idle baixo"),
    TestPoint(750,  "fast idle"),
    TestPoint(1000, "idle normal"),
    TestPoint(1500, "saída de idle"),
    TestPoint(2000, "carga parcial"),
    TestPoint(3000, "carga média"),
]

# ══════════════════════════════════════════════════════════════════════════════
# main()
# ══════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="OpenEMS HIL Test — 1× STM32 + 1× ESP32 (combined)")
    p.add_argument("--stm32",  required=True, help="Porta STM32 (ex: /dev/ttyUSB0)")
    p.add_argument("--esp32",  required=True, help="Porta ESP32 combined (ex: /dev/ttyUSB1)")
    p.add_argument("--baud",   type=int, default=115200)
    p.add_argument("--rpms",   nargs="+", type=int,
                   default=[pt.rpm for pt in DEFAULT_POINTS],
                   help="RPMs a testar (ex: 500 1000 2000)")
    p.add_argument("--report", default=None, metavar="FILE",
                   help="Gerar relatório markdown")
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

    print(f"ESP32 : {args.esp32}  (CKP gen + scope)")
    esp32 = ESP32Client(args.esp32, args.baud)
    print("  ✓ ESP32 OK")

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

    points = [TestPoint(rpm) for rpm in args.rpms]
    runner = HILRunner(stm32, esp32, eng, tables)
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
    esp32.close()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
