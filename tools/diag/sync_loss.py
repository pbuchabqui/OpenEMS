#!/usr/bin/env python3
"""Parser do dump 'D' (diag 41×u32) — discrimina os gatilhos de perda de FULL_SYNC.

Uso:
    python3 tools/diag/sync_loss.py [PORT]            # snapshot único, tudo etiquetado
    python3 tools/diag/sync_loss.py [PORT] --watch    # polling, mostra DELTAS ao vivo
    python3 tools/diag/sync_loss.py [PORT] --raw      # imprime os 41 índices crus

Comando 'D' = 1 byte (0x44) a partir de IDLE → 164 bytes crus (41×u32 LE, sem envelope).
Fonte do layout: src/app/ui_protocol.cpp (array diag[41]).

Foco: correlacionar blips de PW=0 com o gatilho de sync-loss (índices 0-based):
  [20] gap_premature   count<55          → gap prematuro (candidato a debounce)
  [37] loss_histogram  mx > 1.5×mn        → gate de dispersão do hist
  [38] loss_wrap       tooth_index 57→0   → gap real classificado NORMAL
  [22] loss_missing    tooth_count>63     → overrun (gap ausente)
  [39]/[40] hist_mn/mx = par do último trip de histograma (decide se gate está apertado)
"""
import serial, struct, sys, time

CMD = b'D'
NFIELDS = 44
SIZE = NFIELDS * 4  # 176

# (índice, chave, rótulo) — ordem = layout do firmware
FIELDS = [
    (0,  'late_event',       'late_event_count'),
    (1,  'cyc_drop',         'cycle_schedule_drop'),
    (2,  'inj1_arm',         'inj1_arm'),
    (3,  'seq_calls',        'seq_calls'),
    (4,  'evt_overflow',     'evt_overflow'),
    (5,  'clear_all',        'clear_all_count (flush 60 unsync)'),
    (6,  'presync',          'presync_count'),
    (7,  'dwell_wdog',       'dwell_watchdog_count'),
    (8,  'isr_count',        'ckp_isr_count'),
    (9,  'tc_gap',           'tooth_class GAP'),
    (10, 'tc_spike',         'tooth_class SPIKE_NOISE (ruído tolerado)'),
    (11, 'tc_normal',        'tooth_class NORMAL'),
    (12, 'phase_skip',       'phase_skip'),
    (13, 'phase_fire',       'phase_fire'),
    (14, 'evt_inserted',     'evt_inserted'),
    (15, 'evt_dispatched',   'evt_dispatched'),
    (16, 'presync_revs',     'diag_presync_revs'),
    (17, 'seq_revs',         'diag_seq_revs'),
    (18, 'clear_all2',       'diag_clear_all_count'),
    (19, 'gap_accepted',     'gap_accepted (FULL_SYNC ok)'),
    (20, 'gap_premature',    'gap_premature  [GATILHO: count<55]'),
    (21, 'gap_last_tc',      'gap_last_tc (tooth_count do último prematuro)'),
    (22, 'loss_missing',     'loss_missing_gap  [GATILHO: overrun >63]'),
    (23, 'loss_stall',       'loss_stall (watchdog motor parado)'),
    (24, 'loss_avg',         'loss_avg (hist_avg no último missing/wrap)'),
    (25, 'loss_delta',       'loss_delta (período no último missing/wrap)'),
    (26, 'stft_blk_clt',     'stft_blocked_clt'),
    (27, 'stft_blk_o2',      'stft_blocked_o2'),
    (28, 'stft_blk_ae',      'stft_blocked_ae'),
    (29, 'stft_blk_cut',     'stft_blocked_cut'),
    (30, 'stft_runs',        'stft_runs'),
    (31, 'stft_last_err',    'stft_last_err (i32)'),
    (32, 'stft_integ',       'stft_integrator_x1000 (i32)'),
    (33, 'ltft_accept',      'ltft_accum_accepted'),
    (34, 'ltft_reject',      'ltft_accum_rejected'),
    (35, 'ltft_commits',     'ltft_accum_commits'),
    (36, 'ltft_flags',       'ltft flags (b8-15 burn_ve, b16 pending)'),
    (37, 'loss_histogram',   'loss_histogram  [GATILHO: mx>1.5×mn]'),
    (38, 'loss_wrap',        'loss_wrap  [GATILHO: tooth_index 57→0]'),
    (39, 'hist_mn',          'hist_mn (min do último trip de histograma, ticks)'),
    (40, 'hist_mx',          'hist_mx (max do último trip de histograma, ticks)'),
    (41, 'rev_limit_trips',  'rev_limit_trips (bordas de subida do rev-limiter)'),
    (42, 'rev_limit_rpm_x10','rev_limit_rpm_x10 (rpm×10 no último trip)'),
    (43, 'rev_limit_rpm_max','rev_limit_rpm_max (maior rpm×10 já visto — glitch?)'),
]

# Campos com sinal (i32) — reinterpretar
SIGNED = {31, 32}

# Os quatro gatilhos de perda de FULL_SYNC (contadores acumulativos)
TRIGGERS = [(20, 'gap_premature'), (37, 'loss_histogram'),
            (38, 'loss_wrap'), (22, 'loss_missing(overrun)')]


def read_dump(s):
    s.reset_input_buffer()
    s.write(CMD)
    deadline = time.time() + 2.0
    buf = bytearray()
    while len(buf) < SIZE and time.time() < deadline:
        chunk = s.read(SIZE - len(buf))
        if chunk:
            buf.extend(chunk)
    if len(buf) != SIZE:
        raise IOError(f"got {len(buf)}/{SIZE} bytes (comando 'D' respondeu curto)")
    vals = list(struct.unpack('<%dI' % NFIELDS, buf))
    for i in SIGNED:
        if vals[i] >= 0x80000000:
            vals[i] -= 0x100000000
    return vals


def interpret_hist(mn, mx):
    if mn == 0:
        return "sem trip de histograma registado"
    ratio = mx / mn
    if ratio < 1.8:
        verdict = "GATE NO LIMIAR → dispersão de roda real; candidato a relaxar/fix-histograma (sem risco angular)"
    else:
        verdict = "CONTAMINAÇÃO GROSSEIRA → provável dente perdido real; drop está correto (problema de sinal/mecânico)"
    return f"mn={mn} mx={mx} ticks  ratio={ratio:.2f}×  → {verdict}"


def print_snapshot(vals, raw=False):
    if raw:
        for i in range(NFIELDS):
            print(f"[{i:2d}] {vals[i]}")
        return
    print("── DIAG dump 'D' (41×u32) ──")
    for idx, _key, label in FIELDS:
        mark = " ◄" if idx in {t[0] for t in TRIGGERS} else ""
        print(f"  [{idx:2d}] {label:52s} = {vals[idx]}{mark}")
    print("\n── Gatilhos de perda de FULL_SYNC (acumulado) ──")
    total = sum(vals[i] for i, _ in TRIGGERS)
    for i, name in TRIGGERS:
        share = (100.0 * vals[i] / total) if total else 0.0
        print(f"  {name:26s} = {vals[i]:8d}  ({share:5.1f}%)")
    print(f"  {'flush duro (60 unsync)':26s} = {vals[5]:8d}")
    print(f"  {'ruído tolerado (SPIKE)':26s} = {vals[10]:8d}  (não derruba sync)")
    print("\n── Histograma (contexto do último trip) ──")
    print("  " + interpret_hist(vals[39], vals[40]))
    if total == 0:
        print("\n  Nenhum gatilho disparou ainda — reproduz a condição de PW=0 e relê.")


def watch(s, period=0.5):
    print("WATCH: mostra DELTAS por amostra. Reproduz a condição intermitente. Ctrl-C p/ sair.\n")
    prev = read_dump(s)
    hdr = "  ".join(f"{name.split('(')[0]:>14s}" for _, name in TRIGGERS)
    print(f"{'t(s)':>6s}  {hdr}   {'spike':>7s}  {'flush':>6s}   hist(mn/mx)")
    t0 = time.time()
    try:
        while True:
            time.sleep(period)
            cur = read_dump(s)
            dt = "  ".join(f"{cur[i]-prev[i]:>14d}" for i, _ in TRIGGERS)
            dsp = cur[10] - prev[10]
            dfl = cur[5] - prev[5]
            hist = f"{cur[39]}/{cur[40]}" if cur[39] else "-"
            flag = ""
            for i, name in TRIGGERS:
                if cur[i] - prev[i] > 0:
                    flag = f"  ← {name} disparou"
            print(f"{time.time()-t0:6.1f}  {dt}  {dsp:7d}  {dfl:6d}   {hist}{flag}")
            prev = cur
    except KeyboardInterrupt:
        print("\n(fim)")


def main():
    args = [a for a in sys.argv[1:]]
    port = next((a for a in args if not a.startswith('-')), '/dev/ttyACM0')
    s = serial.Serial(port, 115200, timeout=1.5)
    time.sleep(0.3)
    try:
        if '--watch' in args:
            watch(s)
        else:
            vals = read_dump(s)
            print_snapshot(vals, raw=('--raw' in args))
    finally:
        s.close()


if __name__ == '__main__':
    main()
