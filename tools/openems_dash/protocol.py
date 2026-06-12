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
    20, 30, 40, 50, 60, 70, 80, 90,
    100, 115, 130, 145, 160, 180, 215, 300,
]
RPM_AXIS = [v // 10 for v in RPM_AXIS_X10]
MAP_AXIS_KPA = [v for v in MAP_AXIS_BAR_X100]  # bar×100 == kPa

PAGE_SIZES = {0: 512, 1: 256, 2: 256, 3: 64, 4: 512, 5: 256, 6: 80, 7: 32}

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

    def to_dict(self) -> dict:
        return asdict(self)


def parse_realtime(buf: bytes) -> RealtimeData:
    """Decodifica UiRealtimeData (ui_protocol.h, 64 bytes)."""
    if len(buf) != 64:
        raise ValueError(f"realtime page: esperado 64B, recebido {len(buf)}B")
    (rpm, map_x100, tps, clt_p40, iat_p40, o2_d4, pw_x10,
     adv_p40, ve, stft, status) = struct.unpack_from("<HBBbbBBBBbH", buf, 0)
    r = buf[12:62]  # reserved[50]
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
        self._ser = serial.Serial(self.port, 115200, timeout=timeout)
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
        return parse_realtime(self._txn(b"A", 64))

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
PAGE5_FIELDS = [
    # (nome, offset, count, struct fmt por elemento)
    ("clt_corr_axis_x10",            0, 8, "h"),
    ("clt_corr_x256",               16, 8, "H"),
    ("iat_corr_axis_x10",           32, 8, "h"),
    ("iat_corr_x256",               48, 8, "H"),
    ("warmup_corr_axis_x10",        64, 8, "h"),
    ("warmup_corr_x256",            80, 8, "H"),
    ("vbatt_corr_axis_mv",          96, 8, "H"),
    ("injector_dead_time_us",      112, 8, "H"),
    ("ae_clt_corr_axis_x10",       128, 8, "h"),
    ("ae_clt_sens",                144, 8, "H"),
    ("dwell_vbatt_axis_mv",        160, 8, "H"),
    ("dwell_ms_x10_table",         176, 8, "H"),
    ("lambda_delay_rpm_axis_x10",  192, 3, "I"),
    ("lambda_delay_load_axis_bar_x100", 204, 3, "I"),
    ("lambda_delay_ms_table",      216, 9, "H"),
    ("ae_tpsdot_threshold_x10",    234, 1, "H"),
    ("ae_taper_cycles",            236, 1, "H"),
    ("ae_max_pw_us",               238, 1, "H"),
    ("idle_spark_tps_max_x10",     240, 1, "H"),
    ("idle_spark_map_max_bar_x100", 242, 1, "H"),
    ("idle_spark_rpm_min_x10",     244, 1, "H"),
    ("idle_spark_window_above_target_x10", 246, 1, "H"),
    ("idle_spark_deadband_rpm_x10", 248, 1, "H"),
    ("idle_spark_rpm_per_deg_x10", 250, 1, "H"),
    ("idle_spark_retard_limit_deg", 252, 1, "h"),
    ("idle_spark_advance_limit_deg", 254, 1, "h"),
]

PAGE6_FIELDS = [
    ("xtau_clt_axis_x10",      0, 8, "h"),
    ("xtau_x_fraction_q8",    16, 8, "H"),
    ("xtau_tau_cycles",       32, 8, "H"),
    ("ae_tpsdot_axis_x10",    48, 4, "H"),
    ("ae_pw_adder_us",        56, 4, "H"),
    ("crank_enter_rpm_x10",   64, 1, "H"),
    ("crank_exit_rpm_x10",    66, 1, "H"),
    ("crank_spark_deg",       68, 1, "h"),
    ("crank_min_pw_us",       70, 1, "H"),
    ("crank_prime_tooth",     72, 1, "H"),
    ("crank_prime_max_pw_us", 74, 1, "H"),
]

PAGE7_FIELDS = [
    ("dwell_rpm_axis_rpm",   0, 4, "H"),
    ("dwell_rpm_factor_q8",  8, 4, "H"),
]

# Página 0, bytes 0-15: engine config (engine_config.cpp). Byte 1 é padding;
# bytes 14-15 são o magic 0x4543 — exposto read-only para diagnóstico.
PAGE0_FIELDS = [
    ("ivc_abdc_deg",              0, 1, "B"),
    ("displacement_cc",           2, 1, "H"),
    ("injector_flow_cc_min",      4, 1, "H"),
    ("stoich_afr_x100",           6, 1, "H"),
    ("map_ref_bar_x100",          8, 1, "H"),
    ("trigger_tooth0_engine_deg", 10, 1, "H"),
    ("default_soi_lead_deg",     12, 1, "H"),
    ("config_magic",             14, 1, "H"),
]

FIELD_PAGES = {0: PAGE0_FIELDS, 5: PAGE5_FIELDS, 6: PAGE6_FIELDS, 7: PAGE7_FIELDS}


def decode_fields(page: int, buf: bytes) -> dict:
    out = {}
    for name, off, count, fmt in FIELD_PAGES[page]:
        vals = list(struct.unpack_from(f"<{count}{fmt}", buf, off))
        out[name] = vals[0] if count == 1 else vals
    return out


def encode_field(page: int, name: str, values) -> tuple[int, bytes]:
    """Devolve (offset, bytes) para escrever um campo via 'x'."""
    for fname, off, count, fmt in FIELD_PAGES[page]:
        if fname == name:
            vals = values if isinstance(values, list) else [values]
            if len(vals) != count:
                raise ValueError(f"{name}: esperado {count} valores")
            return off, struct.pack(f"<{count}{fmt}", *[int(v) for v in vals])
    raise KeyError(f"página {page}: campo {name} desconhecido")
