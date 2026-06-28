#!/usr/bin/env python3
"""Dump and decode PC6/PE9 debug trace ring buffers via CDC 'T' command."""
import serial, struct, sys, time

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
ENTRY_SIZE = 12  # uint32 tim_cnt, uint32 ccmr1, uint16 tooth, uint8 action, uint8 flags
N = 32

ACT_NAMES = {0: 'INJ_ON ', 1: 'INJ_OFF', 2: 'DWELL  ', 3: 'SPARK  '}
FLAG_BITS = {0: 'LATE', 1: 'INHIBIT', 2: 'DELTA_DROP'}

def decode_flags(f):
    parts = [v for k, v in FLAG_BITS.items() if f & (1 << k)]
    return '|'.join(parts) if parts else 'OK'

def decode_oc1m(ccmr1):
    bits_2_0 = (ccmr1 >> 4) & 0x7
    bit_3 = (ccmr1 >> 16) & 0x1
    return (bit_3 << 3) | bits_2_0

OC_NAMES = {
    0: 'Frozen', 1: 'Active', 2: 'Inactive', 3: 'Toggle',
    4: 'ForceInact', 5: 'ForceAct', 6: 'PWM1', 7: 'PWM2',
}

def print_ring(name, idx, data):
    print(f'\n{"="*70}')
    print(f'  {name}  (write cursor={idx}, oldest=entry {idx})')
    print(f'{"="*70}')
    print(f'  #  {"TIM_CNT":>10}  {"Tooth":>5}  {"Action":>7}  {"OC1M":>12}  {"CCMR1":>10}  Flags')
    print(f'  {"─"*65}')
    for j in range(N):
        i = (idx + j) % N
        off = i * ENTRY_SIZE
        tim_cnt, ccmr1, tooth, action, flags = struct.unpack_from('<IIHBB', data, off)
        if tim_cnt == 0 and ccmr1 == 0 and tooth == 0 and action == 0 and flags == 0:
            continue
        oc1m = decode_oc1m(ccmr1)
        marker = ' ◄' if j == N - 1 else ''
        print(f'  {j:2d}  {tim_cnt:10d}  {tooth:5d}  {ACT_NAMES.get(action, f"?{action}"):>7}  '
              f'{OC_NAMES.get(oc1m, f"?{oc1m}"):>12}  0x{ccmr1:08x}  {decode_flags(flags)}{marker}')

CHUNK = 1 + N * ENTRY_SIZE  # 385 bytes per channel

def read_trace(s, cmd):
    s.reset_input_buffer()
    s.write(cmd)
    time.sleep(0.5)
    raw = s.read(CHUNK)
    if len(raw) < CHUNK:
        print(f'ERROR: cmd {cmd!r} got {len(raw)}/{CHUNK} bytes')
        sys.exit(1)
    return raw[0], raw[1:]

s = serial.Serial(PORT, 115200, timeout=3)
time.sleep(0.3)
pc6_idx, pc6_data = read_trace(s, b'T')
pe9_idx, pe9_data = read_trace(s, b'U')
s.close()

print_ring('PC6 (INJ1 — TIM3_CH1)', pc6_idx, pc6_data)
print_ring('PE9 (IGN1 — TIM1_CH1)', pe9_idx, pe9_data)

# Summary
print(f'\n── Summary ──')
for name, data in [('PC6', pc6_data), ('PE9', pe9_data)]:
    actions = []
    for i in range(N):
        off = i * ENTRY_SIZE
        _, _, _, action, flags = struct.unpack_from('<IIHBB', data, off)
        if flags == 0:
            actions.append(action)
    on_count = sum(1 for a in actions if a in (0, 2))
    off_count = sum(1 for a in actions if a in (1, 3))
    print(f'  {name}: {on_count} ON/DWELL, {off_count} OFF/SPARK in last {len(actions)} good events')
