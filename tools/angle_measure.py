#!/usr/bin/env python3
"""Measure sub-tooth angle precision of INJ1 via CDC 'G' command."""
import serial, struct, sys, time

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
for p in [PORT, '/dev/ttyACM0', '/dev/ttyACM1']:
    try:
        s = serial.Serial(p, 115200, timeout=3); time.sleep(0.3)
        s.reset_input_buffer(); s.write(b'Q'); time.sleep(0.3)
        if b'OpenEMS' in s.read(50): break
        s.close()
    except: pass

# Get RPM for tooth period
s.reset_input_buffer(); s.write(b'A'); time.sleep(0.5)
rt = s.read(66)
rpm = struct.unpack_from('<H', rt, 0)[0]
if rpm == 0:
    print("RPM=0, no CKP signal"); sys.exit(1)

tooth_us = 60e6 / (rpm * 60)  # µs per tooth position
tooth_ticks = tooth_us / 0.016  # TIM5 ticks per tooth (62.5MHz = 16ns)
print(f"RPM={rpm}  tooth={tooth_us:.1f}µs = {tooth_ticks:.0f} TIM5 ticks")

# Collect N snapshots
N = 20
on_fracs = []
off_fracs = []

for sample in range(N):
    time.sleep(0.15)
    s.reset_input_buffer(); s.write(b'G'); time.sleep(0.5)
    raw = s.read(45)
    if len(raw) < 45:
        continue

    gap_ts = struct.unpack_from('<I', raw, 0)[0]
    ridx = raw[4]

    for i in range(8):
        off = 5 + i * 5
        ts = struct.unpack_from('<I', raw, off)[0]
        high = raw[off + 4]
        if ts == 0: continue

        dt = (ts - gap_ts) & 0xFFFFFFFF  # 32-bit circular
        teeth_from_gap = dt / tooth_ticks
        tooth_in_rev = teeth_from_gap % 58.0
        tooth_int = int(tooth_in_rev)
        frac = tooth_in_rev - tooth_int

        if high:
            on_fracs.append((tooth_int, frac))
        else:
            off_fracs.append((tooth_int, frac))

s.close()

print(f"\nCollected: {len(on_fracs)} ON, {len(off_fracs)} OFF edges")
print()

for label, fracs in [("ON (rising)", on_fracs), ("OFF (falling)", off_fracs)]:
    if not fracs:
        print(f"{label}: no data")
        continue
    teeth = [t for t, f in fracs]
    fs = [f for t, f in fracs]
    avg_tooth = sum(teeth) / len(teeth)
    avg_frac = sum(fs) / len(fs)
    min_frac = min(fs)
    max_frac = max(fs)
    jitter_frac = max_frac - min_frac
    jitter_deg = jitter_frac * 6.0

    print(f"{label}:")
    print(f"  Avg tooth: {avg_tooth:.1f}  Avg frac: {avg_frac:.3f}")
    print(f"  Jitter: {jitter_frac:.4f} frac = {jitter_deg:.3f}°")
    if avg_frac < 0.05 or avg_frac > 0.95:
        print(f"  >> BOUNDARY (force_oc mode)")
    else:
        print(f"  >> SUB-TOOTH: frac={avg_frac:.3f}")
        if jitter_deg < 0.5:
            print(f"  >> PRECISION < 0.5° VALIDATED ✓")
        else:
            print(f"  >> Jitter {jitter_deg:.3f}° exceeds 0.5° ✗")
    print()
