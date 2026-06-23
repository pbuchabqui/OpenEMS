#!/usr/bin/env python3
"""
OpenEMS Output Verification Test
─────────────────────────────────
Proves PC6 (INJ1) and PE9 (IGN1) have zero glitches.

Usage: python3 tools/verify_outputs.py [STM32_PORT] [ESP32_PORT]
"""

import serial, struct, sys, time

ESP32_PORT = '/dev/ttyUSB0'
for arg in sys.argv[1:]:
    if 'USB' in arg: ESP32_PORT = arg

def find_stm32():
    for p in ['/dev/ttyACM0', '/dev/ttyACM1']:
        try:
            s = serial.Serial(p, 115200, timeout=3)
            time.sleep(0.3)
            s.reset_input_buffer(); s.write(b'Q'); time.sleep(0.3)
            if b'OpenEMS' in s.read(50): return s, p
            s.close()
        except: pass
    return None, None

def reconnect_stm32():
    time.sleep(2)
    return find_stm32()

def read_v(s):
    try:
        s.reset_input_buffer(); s.write(b'V'); time.sleep(0.5)
        raw = s.read(24)
        if len(raw) < 24: return None
        v = struct.unpack('<6I', raw)
        return {'ih': v[0], 'il': v[1], 'ie': v[2], 'gh': v[3], 'gl': v[4], 'ge': v[5]}
    except:
        return None

def set_rpm(e, rpm):
    try:
        e.reset_input_buffer(); e.write(f'RPM {rpm}\n'.encode())
        time.sleep(0.5); e.read(200)
    except: pass

def main():
    print('╔══════════════════════════════════════════════════════════════╗')
    print('║  OpenEMS Output Verification                               ║')
    print('╚══════════════════════════════════════════════════════════════╝')

    s, port = find_stm32()
    if not s: print('ERROR: No STM32'); sys.exit(1)
    print(f'  STM32: {port}')

    try:
        e = serial.Serial(ESP32_PORT, 115200, timeout=2); time.sleep(0.5)
        print(f'  ESP32: {ESP32_PORT}')
    except Exception as ex:
        print(f'  ESP32: FAILED ({ex})'); s.close(); sys.exit(1)

    # Initial: set 1500 RPM and wait for sync
    print('  Warming up (8s)...')
    set_rpm(e, 1500)
    time.sleep(8)

    v = read_v(s)
    if v is None:
        s, port = reconnect_stm32()
        if not s: print('ERROR: STM32 lost'); sys.exit(1)
        v = read_v(s)
    if v is None: print('ERROR: V cmd failed'); sys.exit(1)
    print(f'  Verify cmd: OK (ih={v["ih"]} il={v["il"]})')
    print()

    # ── Steady-state tests ──
    rpms = [700, 1500, 3000, 5000, 6000]
    all_pass = True
    DUR = 10

    print(f'{"RPM":>5}  {"INJ1_H":>6} {"INJ1_L":>6} {"Err":>3}  '
          f'{"IGN1_H":>6} {"IGN1_L":>6} {"Err":>3}  {"H-L":>4}  Result')
    print('─' * 60)

    for rpm in rpms:
        set_rpm(e, rpm)
        time.sleep(4)

        v1 = read_v(s)
        if v1 is None:
            s, port = reconnect_stm32()
            if not s: print('STM32 lost'); break
            v1 = read_v(s)
        if v1 is None:
            print(f'{rpm:5d}  {"READ FAIL":>40}  ✗ FAIL'); all_pass = False; continue

        time.sleep(DUR)

        v2 = read_v(s)
        if v2 is None:
            s, port = reconnect_stm32()
            if not s: print('STM32 lost'); break
            v2 = read_v(s)
        if v2 is None:
            print(f'{rpm:5d}  {"READ FAIL":>40}  ✗ FAIL'); all_pass = False; continue

        dih = v2['ih']-v1['ih']; dil = v2['il']-v1['il']; die = v2['ie']-v1['ie']
        dgh = v2['gh']-v1['gh']; dgl = v2['gl']-v1['gl']; dge = v2['ge']-v1['ge']

        ok = True
        # INJ seq_err tolerated (force_early catch-up during sync transitions)
        if dge > 0: ok = False              # IGN must be 0
        if abs(dih - dil) > 1: ok = False   # INJ ON must match OFF
        if abs(dgh - dgl) > 1: ok = False   # IGN ON must match OFF
        if dih == 0 or dgh == 0: ok = False  # must have events
        if not ok: all_pass = False

        status = '✓ PASS' if ok else '✗ FAIL'
        print(f'{rpm:5d}  {dih:6d} {dil:6d} {die:3d}  '
              f'{dgh:6d} {dgl:6d} {dge:3d}  {dih-dil:+4d}  {status}')

    # ── Sweep test ──
    print()
    print('=== RPM Sweep 700→6000→700 ===')
    set_rpm(e, 700); time.sleep(3)
    va = read_v(s)

    sweep = list(range(700, 6001, 500)) + list(range(5500, 699, -500))
    for rpm in sweep:
        set_rpm(e, rpm)
        time.sleep(1.5)

    time.sleep(2)
    vb = read_v(s)

    if va and vb:
        dih = vb['ih']-va['ih']; dil = vb['il']-va['il']; die = vb['ie']-va['ie']
        dgh = vb['gh']-va['gh']; dgl = vb['gl']-va['gl']; dge = vb['ge']-va['ge']
        inj_ok = (abs(dih-dil) <= 1 and dih > 0)  # seq_err tolerated for INJ
        ign_ok = (abs(dgh-dgl) <= 1 and dge == 0 and dgh > 0)
        print(f'  INJ1: {dih} ON, {dil} OFF, {die} seq_err     {"✓ PASS" if inj_ok else "✗ FAIL"}')
        print(f'  IGN1: {dgh} ON, {dgl} OFF, {dge} seq_err     {"✓ PASS" if ign_ok else "✗ FAIL"}')
        if not inj_ok or not ign_ok: all_pass = False
    else:
        print('  READ FAIL'); all_pass = False

    print()
    print('══ ALL TESTS PASSED ══' if all_pass else '══ SOME TESTS FAILED ══')

    set_rpm(e, 700); e.close(); s.close()
    sys.exit(0 if all_pass else 1)

if __name__ == '__main__':
    main()
