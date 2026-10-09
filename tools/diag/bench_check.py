#!/usr/bin/env python3
"""Bench check: one line per poll from the realtime page, flags counter jumps.

Uses the dashboard's protocol module (tools/openems_dash/protocol.py), the
one the dashboard tests keep in step with the firmware. Close the dashboard
first: the serial port is opened exclusively.

    python3 tools/diag/bench_check.py [PORT] [--period 0.5] [--count N]

Exit code 1 if a late / drop / clamp counter moved or sync was lost while
the script ran (docs/bench_test_manual.md, tests T2-T6).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "openems_dash"))
from protocol import OpenEMSLink  # noqa: E402

SYNC = {0: "WAIT_GAP", 1: "HALF", 2: "FULL", 3: "LOSS"}
INJ = {0: "simult", 1: "semi", 2: "seq"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--count", type=int, default=0, help="polls, 0 = until Ctrl-C")
    a = ap.parse_args()

    link = OpenEMSLink(a.port)
    first = None
    bad = False
    was_full = False
    n = 0
    try:
        while a.count == 0 or n < a.count:
            d = link.read_realtime()
            if first is None:
                first = d
            full = d.sync_state == 2
            if was_full and not full:
                bad = True
                print("!! FULL_SYNC lost")
            was_full = was_full or full
            late = d.late_events - first.late_events
            drop = d.sched_drops - first.sched_drops
            clamp = d.cal_clamps - first.cal_clamps
            if late or drop or clamp:
                bad = True
            tooth_us = d.tooth_period_ns / 1000.0
            print(f"rpm {d.rpm:5d}  sync {SYNC.get(d.sync_state, d.sync_state):8s} "
                  f"inj {INJ.get(d.inj_mode, d.inj_mode):6s} cmp {d.cmp_confirms} "
                  f"adv {d.advance_deg_fine:6.1f}  pw {d.pw_ms:5.1f} ms  "
                  f"tooth {tooth_us:8.1f} us  loop max {d.loop2ms_max_us:4d} us  "
                  f"late +{late} drop +{drop} clamp +{clamp}  "
                  f"faults 0x{d.sensor_fault_bits:02x} status 0x{d.status_bits:04x}",
                  flush=True)
            n += 1
            time.sleep(a.period)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    print("FAIL" if bad else "OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
