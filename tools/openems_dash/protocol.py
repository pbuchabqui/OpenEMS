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

# Dimensão do grid principal (espelha kTableAxisSize do firmware)
N = 20

# Eixos default do firmware (table3d.cpp) — mesmos de tools/hil_test/hil_test.py
RPM_AXIS_X10 = [
    5000, 7500, 10000, 12500, 15000, 17500, 20000, 22500, 25000, 27500,
    30000, 35000, 40000, 45000, 50000, 55000, 60000, 65000, 70000, 80000,
]
MAP_AXIS_BAR_X100 = [
     20,  30,  40,  46,  52,  58,  64,  70,  76,  88,
     94, 100, 110, 130, 160, 190, 220, 250, 273, 300,
]
RPM_AXIS = [v // 10 for v in RPM_AXIS_X10]
MAP_AXIS_KPA = [v for v in MAP_AXIS_BAR_X100]  # bar×100 == kPa

PAGE_SIZES = {0: 512, 1: N*N, 2: N*N, 3: 86, 4: 2*N*N, 5: 256, 6: 80, 7: 32, 8: 80,
              9: 112, 10: N*N + ((N+1)//2)**2, 11: 4*N,
              12: 2 * N * N}  # LTFT accum: hits_wire u8 + mean_stft i8

STATUS_BITS = {
    "FULL_SYNC":        0x0001,  # bit 0
    "PHASE_A":          0x0002,  # bit 1
    "SENSOR_FAULT":     0x0004,  # bit 2
    "LIMP_MODE":        0x0008,  # bit 3
    "ETB_LIMP":         0x0010,  # bit 4
    "XTAU_LEARN":       0x0020,  # bit 5
    "SCHED_LATE":       0x0040,  # bit 6
    "SCHED_DROP":       0x0080,  # bit 7
    "SCHED_CLAMP":      0x0100,  # bit 8
    "WBO2_FAULT":       0x0200,  # bit 9
    "TLE8888_FAULT":    0x0400,  # bit 10
    "IGN_SEQUENTIAL":   0x0800,  # bit 11 — 1=sequencial, 0=wasted-spark (presync)
    "REV_LIMIT":        0x1000,  # bit 12 — fuel cut active
    "LAUNCH_ACTIVE":    0x2000,  # bit 13 — launch control holding
    "TC_ACTIVE":        0x4000,  # bit 14 — traction control reducing torque
    "BENCH_MODE":       0x8000,  # bit 15 — bench 'B' activo na ECU (RAM; cai no reset)
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
    dc_pct: float      # injector duty cycle % (720° cycle)
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
    tc_reduction_pct: float   # 0–100 % throttle cut from TC (reserved[31..32])
    torque_spark_retard_deg: int  # 0–30° retard from TC/launch (reserved[33])
    loop2ms_last_us: int
    loop2ms_max_us: int
    an1_raw: int      # APP1 (pedal 1) ADC bruto
    an2_raw: int      # APP2 (pedal 2)
    an3_raw: int      # ETB TPS1
    an4_raw: int      # ETB TPS2
    lambda_target_x1000: int
    ltft_pct: int
    tle8888_fault_bm: int
    sensor_fault_bits: int  # bitmask SensorId (reserved[34]): b0=MAP b1=MAF b2=TPS b3=CLT b4=IAT b5=O2 b6=FUEL b7=OIL
    ethanol_pct: int
    cmp_confirms: int   # gate do sequencial (0/1/2); 2 = CMP confirmado → sequencial
    cmp_glitch: int     # bordas CMP rejeitadas pela validação temporal (saturado 255)
    inj_mode: int       # 0=simultaneous, 1=semi_seq, 2=sequential
    map_fused_kpa: float  # MAP fundido (sensor+modelo) no tick de 2ms do cálculo de fuel
    net_pw_us: int         # PW de fluxo líquido (µs, pré-dead-time/xtau/ΔP/S-curve), mesmo tick
    ckp_edge_count: int    # bordas CKP cruas acumuladas (pré-filtro — ruído conta)
    cmp_edge_count: int    # bordas CMP cruas acumuladas (pré-validação)
    tooth_period_ns: int   # período do último dente aceite (ns); 0 = nenhum
    ckp_edge_age_ms: int   # idade da última borda CKP (ms, satura 65535; 65535 = nunca)
    cmp_edge_age_ms: int   # idade da última borda CMP (ms, idem)

    def to_dict(self) -> dict:
        return asdict(self)


def parse_realtime(buf: bytes) -> RealtimeData:
    """Decodifica UiRealtimeData (ui_protocol.h, 86 bytes)."""
    if len(buf) != 86:
        raise ValueError(f"realtime page: esperado 86B, recebido {len(buf)}B")
    # 'x' = byte de padding: status_bits (uint16) é alinhado p/ offset 12.
    (rpm, map_x100, tps, clt_p40, iat_p40, o2_d4, pw_x10,
     adv_p40, ve, stft, status) = struct.unpack_from("<HBBbbBBBBbxH", buf, 0)
    r = buf[14:66]  # reserved[52] começa no offset 14
    map_fused_bar_x100, net_pw_us = struct.unpack_from("<HH", buf, 66)
    (ckp_edges, cmp_edges, tooth_ns,
     ckp_age, cmp_age) = struct.unpack_from("<IIIHH", buf, 70)
    # Injector duty cycle (contra ciclo 720°): DC% = PW_ms × RPM / 1200.
    # pw_x10 está em décimos de ms → PW_ms = pw_x10/10 → divisor 12000.
    dc_pct = round(pw_x10 * rpm / 12000.0, 1) if rpm > 0 else 0.0
    return RealtimeData(
        rpm=rpm,
        map_kpa=map_x100,           # bar×100 == kPa
        tps_pct=tps,
        clt_c=clt_p40 - 40,
        iat_c=iat_p40 - 40,
        lambda_x1000=o2_d4 * 5,  # ÷5 no fw (cobre até 1.275)
        pw_ms=pw_x10 / 10.0,
        advance_deg=adv_p40 - 40,
        dc_pct=dc_pct,
        # VE interpolado vivo (get_ve no ponto rpm×map atual) — o campo 've'
        # da struct é o VE[0][0] estático (nunca muda); o vivo vai em r[49].
        ve=r[49],
        stft_pct=stft,
        status_bits=status,
        status={name: bool(status & bit) for name, bit in STATUS_BITS.items()},
        late_events=struct.unpack_from("<I", r, 0)[0],
        sched_drops=struct.unpack_from("<I", r, 10)[0],
        cal_clamps=struct.unpack_from("<I", r, 14)[0],
        seed_loaded=struct.unpack_from("<I", r, 18)[0],
        seed_confirmed=struct.unpack_from("<I", r, 22)[0],
        seed_rejected=struct.unpack_from("<I", r, 26)[0],
        sync_state=r[30] & 0x0F,
        inj_mode=r[30] >> 4,
        # r+31..34: tc_reduction_pct_x10 (u16) + spark_retard (u8) + pad
        tc_reduction_pct=struct.unpack_from("<H", r, 31)[0] / 10.0,
        torque_spark_retard_deg=r[33],
        loop2ms_last_us=struct.unpack_from("<I", r, 35)[0],
        loop2ms_max_us=struct.unpack_from("<I", r, 39)[0],
        an1_raw=struct.unpack_from("<H", r, 43)[0],
        an2_raw=struct.unpack_from("<H", r, 45)[0],
        an3_raw=struct.unpack_from("<H", r, 47)[0],
        an4_raw=struct.unpack_from("<H", r, 50)[0],
        lambda_target_x1000=r[4] * 5,
        ltft_pct=struct.unpack_from("<b", r, 5)[0],
        tle8888_fault_bm=r[8],
        sensor_fault_bits=r[34],
        ethanol_pct=r[9],
        cmp_confirms=r[7],
        cmp_glitch=r[6],
        map_fused_kpa=map_fused_bar_x100,  # bar×100 == kPa
        net_pw_us=net_pw_us,
        ckp_edge_count=ckp_edges,
        cmp_edge_count=cmp_edges,
        tooth_period_ns=tooth_ns,
        ckp_edge_age_ms=ckp_age,
        cmp_edge_age_ms=cmp_age,
    )


def autodetect_port() -> str:
    ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        raise RuntimeError("nenhuma /dev/ttyACM* encontrada — placa conectada?")
    return ports[0]


class OpenEMSLink:
    """Acesso serial thread-safe (uma transação por vez)."""

    def __init__(self, port: str | None = None, timeout: float = 0.3):
        self.port = port or autodetect_port()
        # exclusive=True: segunda instância falha na hora em vez de disputar
        # bytes da mesma porta (transações corrompidas, "network error" na UI)
        self._ser = serial.Serial(self.port, 115200, timeout=timeout, exclusive=True)
        self._lock = threading.Lock()

    def close(self) -> None:
        self._ser.close()

    def _txn(self, tx: bytes, rx_len: int, timeout: float | None = None) -> bytes:
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(tx)
            if timeout is None:
                data = self._ser.read(rx_len)
            else:
                # Timeout pontual (ex.: burn — erase+program de flash bloqueia
                # o firmware antes do ACK); restaura o timeout por-defeito no
                # finally para não afetar as próximas transações (poll 30Hz).
                orig = self._ser.timeout
                self._ser.timeout = timeout
                try:
                    data = self._ser.read(rx_len)
                finally:
                    self._ser.timeout = orig
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

    # ── bench-mode ('B': força CLT=90°C/IAT=25°C p/ HIL sem sondas) ──────
    def bench_mode(self, on: bool) -> None:
        ack = self._txn(b"B" + bytes([1 if on else 0]), 1)
        if ack != b"\x00":
            raise IOError(f"bench_mode: ACK {ack.hex()}")

    # ── reset LEARN session ('Z': STFT+LEARN+LTFT NVM-shadow zero) ─────
    def reset_adaptives(self) -> None:
        """Zera STFT, acumulador LEARN e shadows LTFT (marca dirty p/ flush
        adaptativo). Não é burn de página de calibração (VE/page0)."""
        ack = self._txn(b"Z", 1)
        if ack != b"\x00":
            raise IOError(f"reset_adaptives: ACK {ack.hex()}")

    # ── apply LEARN accumulated → VE ('Y': bake-in manual, RAM) ──────────
    def apply_ltft_ready(self) -> int:
        """Aplica bake-in em todas as células com hits>0. Retorna n_commits (0..255)."""
        resp = self._txn(b"Y", 2)
        if len(resp) < 2 or resp[0] != 0x00:
            raise IOError(f"apply_ltft_ready: resp {resp.hex() if resp else 'empty'}")
        return int(resp[1])

    # ── contadores de debug ('D': 41 × u32 LE = 164 B) ──────────────────
    # Índices 31-32 (stft_last_err, stft_integ_x1000) são i32 no wire.
    # 37-40: discriminação dos gatilhos de perda de FULL_SYNC (blip PW=0).
    DEBUG_FIELDS = [
        "late_events", "sched_drops", "inj1_arm", "seq_calls", "evt_overflow",
        "clear_all", "presync_count", "dwell_watchdog", "ckp_isr_count",
        "tc_gap", "tc_spike", "tc_normal", "phase_skip", "phase_fire",
        "evt_inserted", "evt_dispatched", "presync_revs", "seq_revs",
        "diag_clear_all", "gap_accepted", "gap_premature", "gap_last_tc",
        "loss_missing_gap", "loss_stall", "loss_avg", "loss_delta",
        "stft_blk_clt", "stft_blk_o2", "stft_blk_ae", "stft_blk_cut",
        "stft_runs", "stft_last_err", "stft_integ_x1000",
        "ltft_accum_accepted", "ltft_accum_rejected", "ltft_accum_commits",
        "ltft_learn_flags",  # b0-7 reserved pad, b8-15 burn_ve, b16 burn_pending
        "loss_histogram", "loss_wrap", "loss_hist_mn", "loss_hist_mx",
        "rev_limit_trips", "rev_limit_rpm_x10", "rev_limit_rpm_max",
    ]
    DEBUG_SIZE = 44 * 4  # must match FW diag[44]

    def read_debug(self) -> dict:
        assert len(self.DEBUG_FIELDS) * 4 == self.DEBUG_SIZE
        buf = self._txn(b"D", self.DEBUG_SIZE)
        # 31 u32 + 2 i32 + 11 u32
        vals = (
            struct.unpack("<31I", buf[:124])
            + struct.unpack("<2i", buf[124:132])
            + struct.unpack("<11I", buf[132:176])
        )
        return dict(zip(self.DEBUG_FIELDS, vals))

    # ── osciloscópio CKP/CMP ('K': 294 bytes) ────────────────────────────
    def read_scope(self) -> dict:
        """Rings de timestamps TIM5 (62.5 MHz) das bordas cruas CKP/CMP +
        âncora angular (tooth_index/fase/sync no instante do dump).
        Devolve listas ordenadas da mais antiga → mais recente, em ticks."""
        buf = self._txn(b"K", 294)
        ckp_idx, cmp_idx, cmp_ref_tooth = buf[0], buf[1], buf[2]
        ckp = list(struct.unpack_from("<64I", buf, 3))
        cmp = list(struct.unpack_from("<8I", buf, 259))
        tooth_index, phase_a, sync_state = buf[291], buf[292], buf[293]
        # idx aponta para a próxima escrita (mais antiga) → rotaciona
        ckp = ckp[ckp_idx:] + ckp[:ckp_idx]
        cmp = cmp[cmp_idx:] + cmp[:cmp_idx]
        return {"ckp_ts": [t for t in ckp if t != 0],
                "cmp_ts": [t for t in cmp if t != 0],
                "cmp_ref_tooth": cmp_ref_tooth,
                "tooth_index": tooth_index,
                "phase_a": bool(phase_a),
                "sync_state": sync_state}

    # ── teste de saídas ('T') ────────────────────────────────────────────
    # 'T' + subcmd(1) + arg1(1) + arg2(u16 LE) → ACK 1B (STATUS → 4B)
    def _test_cmd(self, sub: int, a1: int = 0, a2: int = 0) -> None:
        ack = self._txn(b"T" + struct.pack("<BBH", sub, a1, a2 & 0xFFFF), 1)
        if ack != b"\x00":
            raise IOError(f"test cmd 0x{sub:02x}: ACK {ack.hex()}")

    def test_enter(self) -> None:
        self._test_cmd(0x01, 0, 0xA55A)

    def test_exit(self) -> None:
        self._test_cmd(0x00)

    def test_keepalive(self) -> None:
        self._test_cmd(0x02)

    def test_status(self) -> dict:
        st = self._txn(b"T\x03\x00\x00\x00", 4)
        return {"active": bool(st[0]), "abort_reason": st[1],
                "keepalive_s": st[2], "busy": bool(st[3])}

    def test_fire_inj(self, cyl: int, pw_us: int) -> None:
        self._test_cmd(0x10, cyl, pw_us)

    def test_fire_ign(self, cyl: int, dwell_us: int) -> None:
        self._test_cmd(0x11, cyl, dwell_us)

    def test_pump(self, on: bool) -> None:
        self._test_cmd(0x20, 1 if on else 0)

    def test_fan(self, on: bool) -> None:
        self._test_cmd(0x21, 1 if on else 0)

    def test_vvt(self, ch: int, duty_pct_x10: int) -> None:
        self._test_cmd(0x30, ch, duty_pct_x10)

    def test_etb(self, pwm: int) -> None:
        self._test_cmd(0x40, 0, pwm & 0xFFFF)  # int16 como u16

    def test_ewg(self, pwm: int) -> None:
        self._test_cmd(0x41, 0, pwm & 0xFFFF)

    def burn_page(self, page: int) -> None:
        # burn_page_to_flash (ui_protocol.cpp) apaga o setor (8KB) e programa
        # antes de responder — bloqueante no firmware. O timeout por-defeito
        # (0.3s, dimensionado para o poll de telemetria a 30Hz) estoura em
        # erase+program real; 2s dá margem folgada sem travar a UI para sempre
        # se a placa realmente não responder.
        ack = self._txn(b"b" + bytes([page]), 1, timeout=2.0)
        if ack != b"\x00":
            raise IOError(f"burn page {page}: ACK {ack.hex()}")


# ── codecs de página ────────────────────────────────────────────────────────

def decode_grid_u8(buf: bytes) -> list[list[int]]:
    """Página 1 (VE): N×N uint8, row-major [map][rpm]."""
    return [list(buf[r * N:(r + 1) * N]) for r in range(N)]


def decode_grid_i8(buf: bytes) -> list[list[int]]:
    """Página 2 (spark): N×N int8."""
    vals = struct.unpack(f"<{N*N}b", buf)
    return [list(vals[r * N:(r + 1) * N]) for r in range(N)]


def decode_grid_i16(buf: bytes) -> list[list[int]]:
    """Página 4 (lambda target ×1000): N×N int16 LE."""
    vals = struct.unpack(f"<{N*N}h", buf)
    return [list(vals[r * N:(r + 1) * N]) for r in range(N)]


def decode_ltft(buf: bytes) -> dict:
    """Page 10: LTFT mult N×N int8 (%) + LTFT add (N+1)//2 int8 (50µs/count)."""
    na = (N + 1) // 2
    mult = struct.unpack(f"<{N*N}b", buf[:N*N])
    mult_grid = [list(mult[r * N:(r + 1) * N]) for r in range(N)]
    add_vals = struct.unpack(f"<{na*na}b", buf[N*N:N*N + na*na])
    add_grid = [list(add_vals[r * na:(r + 1) * na]) for r in range(na)]
    return {"ltft_pct": mult_grid, "ltft_add_50us": add_grid}


def decode_ltft_accum(buf: bytes) -> dict:
    """Page 12: [hits_wire u8 N×N][mean_stft_x10 i8 N×N], row-major [map][rpm].

    hits_wire: bits0-6 = min(hits,127); bit7 = ready from firmware.
    ready is the single source of truth — host must not reimplement thresholds.
    """
    cells = N * N
    if len(buf) < 2 * cells:
        raise ValueError(f"page 12: esperado {2*cells}B, got {len(buf)}")
    hits_wire = list(buf[:cells])
    stft_flat = list(struct.unpack(f"<{cells}b", buf[cells:cells * 2]))
    hits = [[hits_wire[r * N + c] & 0x7F for c in range(N)] for r in range(N)]
    ready = [[bool(hits_wire[r * N + c] & 0x80) for c in range(N)] for r in range(N)]
    mean_stft = [stft_flat[r * N:(r + 1) * N] for r in range(N)]
    return {
        "hits": hits,
        "mean_stft_x10": mean_stft,
        "ready": ready,
    }


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
        "cell_size": 2, "unit": "λ"},
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
# offset 0 reserved (IVC ABDC removed from wire; always 0)
# targeting (ecu_sched.cpp — "mantido por compatibilidade de API e
# protocolo, contador de clamp permanece 0"); scheduler já não o lê.
PAGE0_FIELDS = [
    ("displacement_cc",           2, 1, "H",  1.0),  # cc
    ("injector_flow_cc_min",      4, 1, "H",  1.0),  # cc/min
    ("stoich_afr_x100",           6, 1, "H",  0.01), # AFR
    ("map_ref_bar_x100",          8, 1, "H",  0.01), # bar
    ("trigger_tooth0_engine_deg", 10, 1, "H", 1.0),  # °
    ("default_eoi_lead_deg",     12, 1, "H",  1.0),  # ° (EOI targeting — magic v2 0x4544)
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
    ("closed_loop_enable",            80, 1, "B", 1.0),  # 0=open-loop freeze 1=on
    ("ltft_apply_burn_ve",            81, 1, "B", 1.0),  # after APPLY: 0=RAM 1=burn page1
    ("closed_loop_post_start_s",      82, 1, "H", 1.0),  # seconds after warm CLT+O2
    ("ltft_adapt_min_rpm_x10",        84, 1, "H", 0.1),  # RPM; LTFT/LEARN only below
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
    # bytes 140-145: STFT closed-loop tuning
    ("stft_kp_x100",            140, 1, "H", 1.0),
    ("stft_ki_x1000",           142, 1, "H", 1.0),
    ("stft_clamp_pct_x10",      144, 1, "H", 1.0),
    # bytes 176-183: LTFT authority / rates (após layout version @175)
    ("ltft_mult_clamp_pct_x10", 176, 1, "H", 0.1),   # ±%
    ("ltft_add_clamp_us",       178, 1, "H", 0.001),  # ms display scale
    ("ltft_learn_div",          180, 1, "B", 1.0),
    ("ltft_commit_gain_pct",    181, 1, "B", 1.0),
    ("ltft_max_step_x10",       182, 1, "H", 0.1),    # %/tick; 0=unlimited
    ("ltft_adapt_enable",       184, 1, "B", 1.0),    # 0=STFT only 1=LTFT adapt
    ("ltft_learn_ready_hits",   185, 1, "H", 1.0),
    ("ltft_learn_max_err_x1000", 187, 1, "B", 0.001),  # λ err
    ("ltft_learn_ready_max_mean_err", 188, 1, "B", 0.001),
    ("ltft_learn_ready_min_stft_x10", 189, 1, "B", 0.1),
    ("ltft_learn_ready_max_stft_x10", 190, 1, "B", 0.1),
    # bytes 146-153: X-τ auto-calibration limits
    ("xtau_x_min_q8",          146, 1, "H", 1.0),
    ("xtau_x_max_q8",          148, 1, "H", 1.0),
    ("xtau_tau_min",            150, 1, "H", 1.0),
    ("xtau_tau_max",            152, 1, "H", 1.0),
    # bytes 154-163: EWG position PID + sensor calibration
    ("ewg_kp_x10",             154, 1, "H", 1.0),
    ("ewg_ki_x10",             156, 1, "H", 1.0),
    ("ewg_kd_x10",             158, 1, "H", 1.0),
    ("ewg_pos_min_raw",        160, 1, "H", 1.0),
    ("ewg_pos_max_raw",        162, 1, "H", 1.0),
    ("eoi_idle_deg",           164, 1, "H", 1.0),  # ° BTDC — EOI em idle (blend)
    ("eoi_blend_rpm_lo",       166, 1, "H", 1.0),  # RPM início do blend (0/0=off)
    ("eoi_blend_rpm_hi",       168, 1, "H", 1.0),  # RPM fim do blend
    ("mspark_max_rpm_x10",       170, 1, "H", 0.1),  # RPM max p/ multi-spark
    ("mspark_count",              172, 1, "B", 1.0),  # sparks adicionais (0-3)
    ("mspark_inter_dwell_ms_x10", 173, 1, "H", 0.1),  # dwell entre sparks (ms)
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
# page0 216–245: 3 signals × 10 B (can_rx_map.h). Dash mirrors + writes FW.

CAN_RX_SIGNALS = ["GEAR", "SPEED_KMH", "WHEEL_SPEED_KMH"]
# Offsets match firmware kCanRxMapPage0Off + i * 12
CAN_RX_PAGE0_OFF = {
    "GEAR":            216,
    "SPEED_KMH":       228,
    "WHEEL_SPEED_KMH": 240,
}

CAN_SIGNAL_DEFAULTS = {
    "GEAR":            {"id": 0, "byte_lo": 0, "byte_hi": 255, "shift_right": 0, "mask": 65535, "offset": 0, "timeout_ms": 500},
    "SPEED_KMH":       {"id": 0, "byte_lo": 0, "byte_hi": 255, "shift_right": 0, "mask": 65535, "offset": 0, "timeout_ms": 500},
    "WHEEL_SPEED_KMH": {"id": 0, "byte_lo": 0, "byte_hi": 255, "shift_right": 0, "mask": 65535, "offset": 0, "timeout_ms": 500},
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

def can_rx_map_pack_signal(signal: str) -> bytes:
    """12-byte LE wire blob for one CanSignalDef."""
    if signal not in _can_rx_map:
        raise ValueError(f"sinal desconhecido: {signal}")
    d = _can_rx_map[signal]
    return struct.pack(
        "<HBBBxHhH",
        int(d["id"]) & 0x7FF,
        int(d["byte_lo"]) & 0xFF,
        int(d["byte_hi"]) & 0xFF,
        int(d["shift_right"]) & 0xFF,
        # pad byte inserted by 'x'
        int(d["mask"]) & 0xFFFF,
        int(d["offset"]),
        int(d["timeout_ms"]) & 0xFFFF,
    )

def can_rx_map_unpack_signal(blob: bytes) -> dict:
    if len(blob) < 12:
        raise ValueError("can_rx signal wire needs 12 bytes")
    id_, blo, bhi, sh, mask, off, to = struct.unpack_from("<HBBBxHhH", blob, 0)
    return {
        "id": id_ & 0x7FF,
        "byte_lo": blo,
        "byte_hi": bhi,
        "shift_right": sh,
        "mask": mask & 0xFFFF,
        "offset": off,
        "timeout_ms": to & 0xFFFF,
    }

def can_rx_map_from_page0(buf: bytes) -> None:
    """Load server-side map from page0 RAM blob."""
    for sig, off in CAN_RX_PAGE0_OFF.items():
        if len(buf) >= off + 12:
            _can_rx_map[sig] = can_rx_map_unpack_signal(buf[off:off + 12])


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
