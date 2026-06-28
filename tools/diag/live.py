#!/usr/bin/env python3
"""
OpenEMS Live Diagnostic Tool
─────────────────────────────
Replaces oscilloscope for CKP/CMP/IGN/INJ signal verification.
Reads realtime data + trace buffers via USB CDC and presents a
continuously-updating terminal dashboard.

Usage:
    python3 tools/diag_live.py [PORT]       # default /dev/ttyACM0
    python3 tools/diag_live.py --once       # single snapshot, no loop
"""

import serial, struct, sys, time, os

PORT = '/dev/ttyACM0'
ONCE = False
for arg in sys.argv[1:]:
    if arg == '--once':
        ONCE = True
    elif arg.startswith('/dev') or arg.startswith('COM'):
        PORT = arg

# ── Protocol helpers ──────────────────────────────────────────────────────

ENTRY_SIZE = 12
N_TRACE = 32
CHUNK = 1 + N_TRACE * ENTRY_SIZE

ACT_NAMES = {0: 'INJ_ON ', 1: 'INJ_OFF', 2: 'DWELL  ', 3: 'SPARK  '}
FLAG_BITS = {0: 'LATE', 1: 'INHIB', 2: 'DELTA'}

def decode_flags(f):
    if f == 0: return 'OK'
    return '|'.join(v for k, v in FLAG_BITS.items() if f & (1 << k))

def decode_oc1m(ccmr1):
    return ((ccmr1 >> 16) & 1) << 3 | (ccmr1 >> 4) & 7

OC_NAMES = {
    0: 'Frozen', 1: 'Active', 2: 'Inactive', 3: 'Toggle',
    4: 'ForceInact', 5: 'ForceAct', 6: 'PWM1', 7: 'PWM2',
}

def read_cmd(s, cmd, size, retries=2):
    """Send a single-byte command and read response."""
    for attempt in range(retries):
        try:
            s.reset_input_buffer()
            s.write(cmd)
            time.sleep(0.4)
            data = s.read(size)
            if len(data) >= size:
                return data
        except Exception:
            time.sleep(0.5)
    return None

def parse_trace(raw):
    """Parse a trace buffer: [idx_byte][N×entry]"""
    if raw is None or len(raw) < CHUNK:
        return None, []
    idx = raw[0]
    entries = []
    for j in range(N_TRACE):
        i = (idx + j) % N_TRACE
        off = 1 + i * ENTRY_SIZE
        cnt, ccmr1, tooth, action, flags = struct.unpack_from('<IIHBB', raw, off)
        entries.append({
            'cnt': cnt, 'ccmr1': ccmr1, 'tooth': tooth,
            'action': action, 'flags': flags,
            'oc1m': decode_oc1m(ccmr1),
        })
    return idx, entries

def parse_realtime(raw):
    """Parse realtime page (66 bytes)."""
    if raw is None or len(raw) < 10:
        return {}
    rt = {}
    rt['rpm'] = struct.unpack_from('<H', raw, 0)[0]
    rt['map_bar_x100'] = raw[2] if len(raw) > 2 else 0
    rt['tps_pct'] = raw[3] if len(raw) > 3 else 0
    rt['clt_p40'] = struct.unpack_from('<b', raw, 4)[0] if len(raw) > 4 else 0
    rt['iat_p40'] = struct.unpack_from('<b', raw, 5)[0] if len(raw) > 5 else 0
    return rt

def parse_diag(raw):
    """Parse diagnostic packet (20 bytes = 5×uint32)."""
    if raw is None or len(raw) < 20:
        return {}
    vals = struct.unpack('<5I', raw[:20])
    return {
        'late': vals[0],
        'drop': vals[1],
        'dwell_wdog': vals[2],
        'tim3_isr': vals[3],
        'tim1cc_isr': vals[4],
    }

# ── Dashboard rendering ──────────────────────────────────────────────────

def clear_screen():
    os.system('clear' if os.name != 'nt' else 'cls')

def render_dashboard(rt, diag, diag_prev, pc6_entries, pe9_entries, dt):
    """Render a full diagnostic dashboard."""
    lines = []
    lines.append('╔══════════════════════════════════════════════════════════════╗')
    lines.append('║  OpenEMS Live Diagnostics                  Ctrl+C to exit  ║')
    lines.append('╚══════════════════════════════════════════════════════════════╝')
    lines.append('')

    # ── Engine state ──
    rpm = rt.get('rpm', 0)
    clt = rt.get('clt_p40', 40) - 40
    iat = rt.get('iat_p40', 40) - 40
    lines.append(f'  Engine:  RPM={rpm}  MAP={rt.get("map_bar_x100",0)} kPa  '
                 f'TPS={rt.get("tps_pct",0)}%  CLT={clt}°C  IAT={iat}°C')
    lines.append('')

    # ── Scheduler counters (delta per interval) ──
    d_late = diag.get('late', 0) - diag_prev.get('late', 0)
    d_drop = diag.get('drop', 0) - diag_prev.get('drop', 0)
    d_wdog = diag.get('dwell_wdog', 0) - diag_prev.get('dwell_wdog', 0)
    d_tim3 = diag.get('tim3_isr', 0) - diag_prev.get('tim3_isr', 0)
    d_tim1 = diag.get('tim1cc_isr', 0) - diag_prev.get('tim1cc_isr', 0)

    status_ok = (d_late == 0 and d_drop == 0 and d_wdog == 0)
    status = '✓ OK' if status_ok else '✗ FAULT'

    lines.append(f'  Scheduler [{status}]:')
    lines.append(f'    Late events:   {d_late:+d}/s    Drops: {d_drop:+d}/s    '
                 f'Dwell wdog: {d_wdog:+d}/s')
    lines.append(f'    CC ISRs:  TIM3(INJ)={d_tim3}/s  TIM1(IGN)={d_tim1}/s')
    lines.append('')

    # ── Trace analysis ──
    for name, entries, is_inj in [('PC6 INJ1', pc6_entries, True),
                                    ('PE9 IGN1', pe9_entries, False)]:
        valid = [e for e in entries if e['cnt'] != 0 or e['flags'] != 0]
        if not valid:
            lines.append(f'  {name}: no data')
            continue

        # Extract teeth and actions
        on_teeth = set()
        off_teeth = set()
        flag_issues = []
        oc_modes = set()
        for e in valid:
            if e['action'] in (0, 2):  # ON/DWELL
                on_teeth.add(e['tooth'])
            elif e['action'] in (1, 3):  # OFF/SPARK
                off_teeth.add(e['tooth'])
            if e['flags'] != 0:
                flag_issues.append(f"{ACT_NAMES.get(e['action'],'?')}@t{e['tooth']}:{decode_flags(e['flags'])}")
            oc_modes.add(OC_NAMES.get(e['oc1m'], f'?{e["oc1m"]}'))

        on_count = sum(1 for e in valid if e['action'] in (0, 2) and e['flags'] == 0)
        off_count = sum(1 for e in valid if e['action'] in (1, 3) and e['flags'] == 0)

        # Duty cycle estimation
        on_label = 'ON' if is_inj else 'DWELL'
        off_label = 'OFF' if is_inj else 'SPARK'
        on_t_str = ','.join(str(t) for t in sorted(on_teeth))
        off_t_str = ','.join(str(t) for t in sorted(off_teeth))

        tooth_span = 0
        if on_teeth and off_teeth:
            on_t = min(on_teeth)
            off_t = min(off_teeth)
            tooth_span = (off_t - on_t) % 58

        lines.append(f'  {name}:')
        lines.append(f'    {on_label}@tooth[{on_t_str}]  {off_label}@tooth[{off_t_str}]  '
                     f'span={tooth_span} teeth ({tooth_span*6}°)')
        lines.append(f'    Events: {on_count} {on_label} / {off_count} {off_label}  '
                     f'OC modes: {", ".join(sorted(oc_modes))}')

        if flag_issues:
            lines.append(f'    ⚠ Issues: {", ".join(flag_issues[:5])}')

        # Check for consecutive same-action (stuck pattern)
        for j in range(1, len(valid)):
            if (valid[j-1]['action'] == valid[j]['action'] and
                valid[j-1]['flags'] == 0 and valid[j]['flags'] == 0):
                act = ACT_NAMES.get(valid[j]['action'], '?')
                lines.append(f'    ⚠ STUCK: two {act} in a row '
                             f'(tooth {valid[j-1]["tooth"]}→{valid[j]["tooth"]})')
                break

        lines.append('')

    # ── Timing analysis ──
    if rpm > 0:
        rev_ms = 60000.0 / rpm
        tooth_ms = rev_ms / 58.0
        lines.append(f'  Timing @ {rpm} RPM:')
        lines.append(f'    Revolution: {rev_ms:.1f} ms    Tooth: {tooth_ms:.3f} ms    '
                     f'6°={tooth_ms:.3f} ms')
        lines.append('')

    return '\n'.join(lines)

# ── Main loop ─────────────────────────────────────────────────────────────

def main():
    # Find port
    port = PORT
    s = None
    for p in [port, '/dev/ttyACM0', '/dev/ttyACM1']:
        try:
            s = serial.Serial(p, 115200, timeout=3)
            time.sleep(0.3)
            s.reset_input_buffer()
            s.write(b'Q')
            time.sleep(0.3)
            sig = s.read(50)
            if b'OpenEMS' in sig:
                port = p
                break
            s.close()
            s = None
        except Exception:
            pass

    if s is None:
        print('ERROR: No OpenEMS device found')
        sys.exit(1)

    print(f'Connected to {port}')

    diag_prev = parse_diag(read_cmd(s, b'D', 20))
    iteration = 0

    try:
        while True:
            t_start = time.time()

            # Read all data
            rt_raw = read_cmd(s, b'A', 66)
            diag_raw = read_cmd(s, b'D', 20)
            pc6_raw = read_cmd(s, b'T', CHUNK) if CHUNK <= 512 else None
            pe9_raw = read_cmd(s, b'U', CHUNK) if CHUNK <= 512 else None

            rt = parse_realtime(rt_raw)
            diag = parse_diag(diag_raw)

            _, pc6_entries = parse_trace(pc6_raw) if pc6_raw else (None, [])
            _, pe9_entries = parse_trace(pe9_raw) if pe9_raw else (None, [])

            dt = time.time() - t_start

            # Render
            clear_screen()
            print(render_dashboard(rt, diag, diag_prev, pc6_entries, pe9_entries, dt))

            diag_prev = diag.copy()
            iteration += 1

            if ONCE:
                break

            time.sleep(2.0)

    except KeyboardInterrupt:
        print('\nDiagnostics stopped.')
    except Exception as e:
        print(f'\nError: {e}')
    finally:
        try:
            s.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()
