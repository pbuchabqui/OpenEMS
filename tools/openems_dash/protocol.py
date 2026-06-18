#!/usr/bin/env python3
"""
protocol.py — biblioteca do protocolo serial OpenEMS (CDC, mestre-único).

Comandos (ui_protocol.cpp):
  Q/S/F  → strings (signature / fw version / protocol version)
  C      → 0x00 0xAA (comms test)
  A      → página realtime (64B)
  d      → dirty page mask (1B)
  r <page> <off u16le> <len u16le>          → bytes da página
  x <page> <off u16le> <len u16le> <data>   → escreve só RAM, ACK 1B
  w ...                                      → escreve RAM + flash, ACK 1B
  b <page>                                   → burn página p/ flash, ACK 1B
"""

from __future__ import annotations

import glob
import struct
import threading
from dataclasses import dataclass, asdict

import serial

# Eixos fixos do firmware (table3d.cpp) — mesmos de tools/hil_test/hil_test.py
RPM_AXIS_X10 = [
    5000, 7500, 10000, 12500, 15000, 20000, 25000, 30000,
    35000, 40000, 45000, 50000, 55000, 60000, 70000, 80000,
]
MAP_AXIS_BAR_X100 = [
     20,  30,  40,  52,  64,  76,  88, 100,
    110, 130, 160, 190, 220, 250, 273, 300,
]
RPM_AXIS = [v // 10 for v in RPM_AXIS_X10]
MAP_AXIS_KPA = [v for v in MAP_AXIS_BAR_X100]  # bar×100 == kPa

PAGE_SIZES = {0: 512, 1: 256, 2: 256, 3: 66, 4: 512, 5: 256, 6: 80, 7: 32, 8: 80, 9: 112, 10: 320}

STATUS_BITS = {
    "FULL_SYNC":    0x0001,
    "PHASE_A":      0x0002,
    "SENSOR_FAULT": 0x0004,
    "SCHED_LATE":   0x0008,
    "SCHED_DROP":   0x0010,
    "SCHED_CLAMP":  0x0020,
    "WBO2_FAULT":   0x0040,
}


@dataclass
class RealtimeData:
    rpm: int
    map_kpa: float
    tps_pct: int
    clt_c: int
    iat_c: int
    lambda_x1000: int
    pw_ms: float
    advance_deg: int
    ve: int
    stft_pct: int
    status_bits: int
    status: dict
    late_events: int
    sched_drops: int
    cal_clamps: int
    seed_loaded: int
    seed_confirmed: int
    seed_rejected: int
    sync_state: int
    ivc_clamps: int
    loop2ms_last_us: int
    loop2ms_max_us: int
    an1_raw: int      # APP1 (pedal 1) ADC bruto
    an2_raw: int      # APP2 (pedal 2)
    an3_raw: int      # ETB TPS1
    an4_raw: int      # ETB TPS2

    def to_dict(self) -> dict:
        return asdict(self)


def parse_realtime(buf: bytes) -> RealtimeData:
    """Decodifica UiRealtimeData (ui_protocol.h, 66 bytes)."""
    if len(buf) != 66:
        raise ValueError(f"realtime page: esperado 66B, recebido {len(buf)}B")
    # 'x' = byte de padding: status_bits (uint16) é alinhado p/ offset 12.
    (rpm, map_x100, tps, clt_p40, iat_p40, o2_d4, pw_x10,
     adv_p40, ve, stft, status) = struct.unpack_from("<HBBbbBBBBbxH", buf, 0)
    r = buf[14:66]  # reserved[52] começa no offset 14
    return RealtimeData(
        rpm=rpm,
        map_kpa=map_x100,           # bar×100 == kPa
        tps_pct=tps,
        clt_c=clt_p40 - 40,
        iat_c=iat_p40 - 40,
        lambda_x1000=o2_d4 * 4,
        pw_ms=pw_x10 / 10.0,
        advance_deg=adv_p40 - 40,
        ve=ve,
        stft_pct=stft,
        status_bits=status,
        status={name: bool(status & bit) for name, bit in STATUS_BITS.items()},
        late_events=struct.unpack_from("<I", r, 0)[0],
        sched_drops=struct.unpack_from("<I", r, 10)[0],
        cal_clamps=struct.unpack_from("<I", r, 14)[0],
        seed_loaded=struct.unpack_from("<I", r, 18)[0],
        seed_confirmed=struct.unpack_from("<I", r, 22)[0],
        seed_rejected=struct.unpack_from("<I", r, 26)[0],
        sync_state=r[30],
        ivc_clamps=struct.unpack_from("<I", r, 31)[0],
        loop2ms_last_us=struct.unpack_from("<I", r, 35)[0],
        loop2ms_max_us=struct.unpack_from("<I", r, 39)[0],
        an1_raw=struct.unpack_from("<H", r, 43)[0],
        an2_raw=struct.unpack_from("<H", r, 45)[0],
        an3_raw=struct.unpack_from("<H", r, 47)[0],
        an4_raw=struct.unpack_from("<H", r, 50)[0],
    )


def autodetect_port() -> str:
    ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        raise RuntimeError("nenhuma /dev/ttyACM* encontrada — placa conectada?")
    return ports[0]


class OpenEMSLink:
    """Acesso serial thread-safe (uma transação por vez)."""

    def __init__(self, port: str | None = None, timeout: float = 1.0):
        self.port = port or autodetect_port()
        # exclusive=True: segunda instância falha na hora em vez de disputar
        # bytes da mesma porta (transações corrompidas, "network error" na UI)
        self._ser = serial.Serial(self.port, 115200, timeout=timeout, exclusive=True)
        self._lock = threading.Lock()

    def close(self) -> None:
        self._ser.close()

    def _txn(self, tx: bytes, rx_len: int) -> bytes:
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(tx)
            data = self._ser.read(rx_len)
        if len(data) != rx_len:
            raise TimeoutError(
                f"cmd {tx[:1]!r}: esperado {rx_len}B, recebido {len(data)}B")
        return data

    # ── comandos simples ────────────────────────────────────────────────
    def query(self) -> str:
        return self._txn(b"Q", 12).decode()

    def fw_version(self) -> str:
        return self._txn(b"S", 14).decode()

    def protocol_version(self) -> str:
        return self._txn(b"F", 3).decode()

    def comms_test(self) -> bool:
        return self._txn(b"C", 2) == b"\x00\xaa"

    def dirty_mask(self) -> int:
        return self._txn(b"d", 1)[0]

    def read_realtime(self) -> RealtimeData:
        return parse_realtime(self._txn(b"A", PAGE_SIZES[3]))

    # ── páginas ─────────────────────────────────────────────────────────
    def read_page(self, page: int, off: int = 0, length: int | None = None) -> bytes:
        size = PAGE_SIZES[page]
        if length is None:
            length = size - off
        # O ring TX do firmware tem 512B (capacidade útil 511) — uma leitura de
        # 512B (páginas 0 e 4) não cabe e perde bytes. Ler em blocos de 256B.
        out = b""
        while length > 0:
            n = min(length, 256)
            cmd = b"r" + struct.pack("<BHH", page, off, n)
            out += self._txn(cmd, n)
            off += n
            length -= n
        return out

    def write_page_ram(self, page: int, off: int, data: bytes) -> None:
        cmd = b"x" + struct.pack("<BHH", page, off, len(data)) + data
        ack = self._txn(cmd, 1)
        if ack != b"\x00":
            raise IOError(f"write page {page} off {off}: ACK {ack.hex()}")

    def burn_page(self, page: int) -> None:
        ack = self._txn(b"b" + bytes([page]), 1)
        if ack != b"\x00":
            raise IOError(f"burn page {page}: ACK {ack.hex()}")


# ── codecs de página ────────────────────────────────────────────────────────

def decode_grid_u8(buf: bytes) -> list[list[int]]:
    """Página 1 (VE): 16×16 uint8, row-major [map][rpm]."""
    return [list(buf[r * 16:(r + 1) * 16]) for r in range(16)]


def decode_grid_i8(buf: bytes) -> list[list[int]]:
    """Página 2 (spark): 16×16 int8."""
    vals = struct.unpack("<256b", buf)
    return [list(vals[r * 16:(r + 1) * 16]) for r in range(16)]


def decode_grid_i16(buf: bytes) -> list[list[int]]:
    """Página 4 (lambda target ×1000): 16×16 int16 LE."""
    vals = struct.unpack("<256h", buf)
    return [list(vals[r * 16:(r + 1) * 16]) for r in range(16)]


def decode_ltft(buf: bytes) -> dict:
    """Page 10: LTFT mult 16×16 int8 (%) + LTFT add 8×8 int8 (50µs/count)."""
    mult = struct.unpack("<256b", buf[:256])
    mult_grid = [list(mult[r * 16:(r + 1) * 16]) for r in range(16)]
    add_vals = struct.unpack("<64b", buf[256:320])
    add_grid = [list(add_vals[r * 8:(r + 1) * 8]) for r in range(8)]
    return {"ltft_pct": mult_grid, "ltft_add_50us": add_grid}


def encode_cell_u8(value: int) -> bytes:
    return bytes([max(0, min(255, int(value)))])


def encode_cell_i8(value: int) -> bytes:
    return struct.pack("<b", max(-128, min(127, int(value))))


def encode_cell_i16(value: int) -> bytes:
    return struct.pack("<h", max(-32768, min(32767, int(value))))


GRID_PAGES = {
    1: {"name": "VE", "fmt": "u8", "decode": decode_grid_u8, "encode": encode_cell_u8,
        "cell_size": 1, "unit": "%VE"},
    2: {"name": "Spark", "fmt": "i8", "decode": decode_grid_i8, "encode": encode_cell_i8,
        "cell_size": 1, "unit": "°BTDC"},
    4: {"name": "Lambda", "fmt": "i16", "decode": decode_grid_i16, "encode": encode_cell_i16,
        "cell_size": 2, "unit": "λ×1000"},
}

# Página 5: layout de sync_page_from_table (ui_protocol.cpp)
# Tupla: (nome, offset, count, fmt, scale)
# scale: fator aplicado na leitura (display = raw * scale); na escrita: raw = round(display / scale)
# scale=1.0 → valor já está em unidade natural.

PAGE5_FIELDS = [
    ("clt_corr_axis_x10",            0, 8, "h",  0.1),   # °C
    ("clt_corr_x256",               16, 8, "H",  1.0),   # fator ×256 (adimensional)
    ("iat_corr_axis_x10",           32, 8, "h",  0.1),   # °C
    ("iat_corr_x256",               48, 8, "H",  1.0),
    ("warmup_corr_axis_x10",        64, 8, "h",  0.1),   # °C
    ("warmup_corr_x256",            80, 8, "H",  1.0),
    ("vbatt_corr_axis_mv",          96, 8, "H",  0.001), # V
    ("injector_dead_time_us",      112, 8, "H",  0.001), # ms
    ("ae_clt_corr_axis_x10",       128, 8, "h",  0.1),   # °C
    ("ae_clt_sens",                144, 8, "H",  1.0),
    ("dwell_vbatt_axis_mv",        160, 8, "H",  0.001), # V
    ("dwell_ms_x10_table",         176, 8, "H",  0.1),   # ms
    ("lambda_delay_rpm_axis_x10",  192, 3, "I",  0.1),   # RPM
    ("lambda_delay_load_axis_bar_x100", 204, 3, "I", 0.01), # bar
    ("lambda_delay_ms_table",      216, 9, "H",  1.0),   # ms
    ("ae_tpsdot_threshold_x10",    234, 1, "H",  0.1),   # %/s
    ("ae_taper_cycles",            236, 1, "H",  1.0),
    ("ae_max_pw_us",               238, 1, "H",  0.001), # ms
    ("idle_spark_tps_max_x10",     240, 1, "H",  0.1),   # %
    ("idle_spark_map_max_bar_x100", 242, 1, "H", 0.01),  # bar
    ("idle_spark_rpm_min_x10",     244, 1, "H",  0.1),   # RPM
    ("idle_spark_window_above_target_x10", 246, 1, "H", 0.1), # RPM
    ("idle_spark_deadband_rpm_x10", 248, 1, "H", 0.1),   # RPM
    ("idle_spark_rpm_per_deg_x10", 250, 1, "H",  0.1),   # RPM/°
    ("idle_spark_retard_limit_deg", 252, 1, "h", 1.0),   # °
    ("idle_spark_advance_limit_deg", 254, 1, "h", 1.0),  # °
]

PAGE6_FIELDS = [
    ("xtau_clt_axis_x10",      0, 8, "h",  0.1),   # °C
    ("xtau_x_fraction_q8",    16, 8, "H",  1.0),   # Q8 adimensional
    ("xtau_tau_cycles",       32, 8, "H",  1.0),
    ("ae_tpsdot_axis_x10",    48, 4, "H",  0.1),   # %/s
    ("ae_pw_adder_us",        56, 4, "H",  0.001), # ms
    ("crank_enter_rpm_x10",   64, 1, "H",  0.1),   # RPM
    ("crank_exit_rpm_x10",    66, 1, "H",  0.1),   # RPM
    ("crank_spark_deg",       68, 1, "h",  1.0),   # °
    ("crank_min_pw_us",       70, 1, "H",  0.001), # ms
    ("crank_prime_tooth",     72, 1, "H",  1.0),
    ("crank_prime_max_pw_us", 74, 1, "H",  0.001), # ms
]

PAGE7_FIELDS = [
    ("dwell_rpm_axis_rpm",   0, 4, "H", 1.0),   # RPM
    ("dwell_rpm_factor_q8",  8, 4, "H", 1.0),   # Q8 adimensional
]

# Página 0 — bytes 0-15: engine config; bytes 16+: calibração e dirigibilidade.
PAGE0_FIELDS = [
    ("ivc_abdc_deg",              0, 1, "B",  1.0),  # °
    ("displacement_cc",           2, 1, "H",  1.0),  # cc
    ("injector_flow_cc_min",      4, 1, "H",  1.0),  # cc/min
    ("stoich_afr_x100",           6, 1, "H",  0.01), # AFR
    ("map_ref_bar_x100",          8, 1, "H",  0.01), # bar
    ("trigger_tooth0_engine_deg", 10, 1, "H", 1.0),  # °
    ("default_soi_lead_deg",     12, 1, "H",  1.0),  # °
    ("config_magic",             14, 1, "H",  1.0),
    # bytes 16-55: sensores APP/ETB
    ("app1_raw_min",             16, 1, "H",  1.0),
    ("app1_raw_max",             18, 1, "H",  1.0),
    ("app2_raw_min",             20, 1, "H",  1.0),
    ("app2_raw_max",             22, 1, "H",  1.0),
    ("etb_tps1_raw_min",         24, 1, "H",  1.0),
    ("etb_tps1_raw_max",         26, 1, "H",  1.0),
    ("etb_tps2_raw_min",         28, 1, "H",  1.0),
    ("etb_tps2_raw_max",         30, 1, "H",  1.0),
    ("app_max_delta_pct_x10",    32, 1, "H",  0.1),  # %
    ("etb_max_delta_pct_x10",    34, 1, "H",  0.1),  # %
    ("etb_max_open_pct_x10_limp", 36, 1, "H", 0.1), # %
    ("etb_max_rate_pct_per_s",   38, 1, "H",  1.0),  # %/s
    ("etb_idle_open_pct_x10",    40, 1, "H",  0.1),  # %
    ("etb_cal_valid",            42, 1, "B",  1.0),
    ("etb_harness_present",      43, 1, "B",  1.0),
    ("etb_kp_x10",               44, 1, "H",  0.1),
    ("etb_ki_x10",               46, 1, "H",  0.1),
    ("etb_kd_x10",               48, 1, "H",  0.1),
    # bytes 56-65: trim + CMP
    ("cyl_fuel_trim_pct_0",    56, 1, "b",  1.0),  # %
    ("cyl_fuel_trim_pct_1",    57, 1, "b",  1.0),
    ("cyl_fuel_trim_pct_2",    58, 1, "b",  1.0),
    ("cyl_fuel_trim_pct_3",    59, 1, "b",  1.0),
    ("cyl_ign_trim_deg_0",     60, 1, "b",  1.0),  # °
    ("cyl_ign_trim_deg_1",     61, 1, "b",  1.0),
    ("cyl_ign_trim_deg_2",     62, 1, "b",  1.0),
    ("cyl_ign_trim_deg_3",     63, 1, "b",  1.0),
    ("cmp_window_open_tooth",  64, 1, "B",  1.0),
    ("cmp_window_close_tooth", 65, 1, "B",  1.0),
    # bytes 66-99: dirigibilidade
    ("antijerk_tpsdot_threshold_x10", 66, 1, "H", 0.1),  # %/s
    ("antijerk_retard_deg",           68, 1, "h", 1.0),  # °
    ("antijerk_decay_cycles",         70, 1, "B", 1.0),
    ("rev_limit_rpm_x10",             72, 1, "L", 0.1),  # RPM
    ("rev_limit_soft_window_x10",     76, 1, "L", 0.1),  # RPM
    ("rev_limit_spark_window_x10",    80, 1, "L", 0.1),  # RPM
    ("rev_limit_max_retard_deg",      84, 1, "h", 1.0),  # °
    ("ltft_add_pw_threshold_us",      86, 1, "H", 0.001),# ms
    ("decel_cut_tps_threshold_x10",   88, 1, "H", 0.1),  # %
    ("decel_cut_entry_rpm_x10",       90, 1, "L", 0.1),  # RPM
    ("decel_cut_exit_rpm_x10",        94, 1, "L", 0.1),  # RPM
    ("decel_cut_min_clt_x10",         98, 1, "h", 0.1),  # °C
    # bytes 100-105: marcha lenta ETB
    ("etb_idle_rpm_target",      100, 1, "H", 1.0),  # RPM
    ("etb_idle_min_opening_x10", 102, 1, "H", 0.1),  # %
    ("etb_idle_max_opening_x10", 104, 1, "H", 0.1),  # %
    # bytes 106-121: idle RPM target vs CLT
    ("iac_clt_axis_x10",         106, 8, "h", 0.1),  # °C
    ("iac_idle_target_rpm_x10",  122, 8, "H", 0.1),  # RPM
    # byte 138: WBO2 CAN ID
    ("wbo2_can_id",              138, 1, "H", 1.0),  # CAN ID 11-bit
]

FIELD_PAGES = {0: PAGE0_FIELDS, 5: PAGE5_FIELDS, 6: PAGE6_FIELDS, 7: PAGE7_FIELDS}

# Pedal map (page 8): 4 modes × 10 uint16 LE, units pct×10.
# Mode order: 0=ECO, 1=NORMAL, 2=SPORT, 3=RAIN. Axis fixed: 0%,10%,...,100%.
PEDAL_MAP_MODES = ["ECO", "NORMAL", "SPORT", "RAIN"]
PEDAL_MAP_AXIS  = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]  # pedal %

def decode_pedal_maps(buf: bytes) -> list[list[float]]:
    """80 bytes → list of 4 lists of 10 floats (throttle %)."""
    maps = []
    for m in range(4):
        row = list(struct.unpack_from("<10H", buf, m * 20))
        maps.append([v / 10.0 for v in row])
    return maps

# ── Boost map (page 9) ────────────────────────────────────────────────────────
# 7 marchas × 8 RPM × uint16 LE; unidade: bar × 1000
BOOST_RPM_AXIS   = [1500, 2000, 2500, 3000, 4000, 5000, 6500, 8000]
BOOST_GEAR_LABELS = ["Neutro/Desc.", "1ª", "2ª", "3ª", "4ª", "5ª", "6ª"]

def decode_boost_map(buf: bytes) -> list[list[int]]:
    rows = []
    for g in range(7):
        row = list(struct.unpack_from("<8H", buf, g * 16))
        rows.append(row)
    return rows

def encode_boost_map(rows: list[list[int]]) -> bytes:
    out = bytearray(112)
    for g, row in enumerate(rows):
        struct.pack_into("<8H", out, g * 16, *[int(v) for v in row])
    return bytes(out)


def encode_pedal_maps(maps: list[list[float]]) -> bytes:
    """4 × 10 floats (throttle %) → 80 bytes for page 8."""
    out = bytearray(80)
    for m, row in enumerate(maps):
        struct.pack_into("<10H", out, m * 20, *[int(round(v * 10)) for v in row])
    return bytes(out)


# ── CAN RX Map ────────────────────────────────────────────────────────────────
# Configuração RAM-only (sem NVM por enquanto); enviada ao firmware via protocolo
# futuro. Armazenada no servidor e exposta ao dashboard via REST.

CAN_RX_SIGNALS = ["GEAR", "SPEED_KMH"]

CAN_SIGNAL_DEFAULTS = {
    "GEAR":      {"id": 0, "byte_lo": 0, "byte_hi": 255, "shift_right": 0, "mask": 255, "offset": 0, "timeout_ms": 500},
    "SPEED_KMH": {"id": 0, "byte_lo": 0, "byte_hi": 255, "shift_right": 0, "mask": 255, "offset": 0, "timeout_ms": 500},
}

# Estado em memória (o servidor é single-process)
_can_rx_map: dict = {k: dict(v) for k, v in CAN_SIGNAL_DEFAULTS.items()}

def can_rx_map_get() -> dict:
    return {k: dict(v) for k, v in _can_rx_map.items()}

def can_rx_map_set(signal: str, fields: dict) -> None:
    if signal not in _can_rx_map:
        raise ValueError(f"sinal desconhecido: {signal}")
    for k, v in fields.items():
        if k in _can_rx_map[signal]:
            _can_rx_map[signal][k] = int(v)


def _apply_scale(vals: list, scale: float) -> list:
    if scale == 1.0:
        return vals
    return [round(v * scale, 6) for v in vals]


def _remove_scale(vals: list, scale: float) -> list:
    if scale == 1.0:
        return [int(v) for v in vals]
    return [round(float(v) / scale) for v in vals]


def decode_fields(page: int, buf: bytes) -> dict:
    out = {}
    for entry in FIELD_PAGES[page]:
        name, off, count, fmt = entry[0], entry[1], entry[2], entry[3]
        scale = entry[4] if len(entry) > 4 else 1.0
        vals = list(struct.unpack_from(f"<{count}{fmt}", buf, off))
        vals = _apply_scale(vals, scale)
        out[name] = vals[0] if count == 1 else vals
    return out


def encode_field(page: int, name: str, values) -> tuple[int, bytes]:
    """Devolve (offset, bytes) para escrever um campo via 'x'."""
    for entry in FIELD_PAGES[page]:
        fname, off, count, fmt = entry[0], entry[1], entry[2], entry[3]
        scale = entry[4] if len(entry) > 4 else 1.0
        if fname == name:
            vals = values if isinstance(values, list) else [values]
            if len(vals) != count:
                raise ValueError(f"{name}: esperado {count} valores")
            raw = _remove_scale(vals, scale)
            return off, struct.pack(f"<{count}{fmt}", *raw)
    raise KeyError(f"página {page}: campo {name} desconhecido")
