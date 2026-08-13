#!/usr/bin/env python3
"""
hil_encoder_test.py — Smoke HIL para OpenEMS em modo encoder (EMS_MT6835_ENCODER=1)

Requer:
  PC ─ USB → STM32H562 VGT6   (CDC OpenEMS, /dev/ttyACM0)
  esp32_encoder_stim → STM32  (AB GPIO2/4→PA0/PA1, CMP GPIO5→PC6)

Não substitui hil_test.py (CKP 60-2). Critérios mínimos:
  - ping ECU
  - RPM reportado > 0 após IDLE
  - late_event_count (UART 'D'[0]) não sobe em IDLE ~30 s
  - opcional: sweep RPM 800/1500/3000 sem stall (rpm volta a 0)

Uso:
  python3 hil_encoder_test.py --stm32 /dev/ttyACM0 --stim /dev/ttyUSB0
  python3 hil_encoder_test.py --stm32 /dev/ttyACM0 --stim tcp:192.168.15.169:3333 \\
      --bench-clt-iat --idle-s 30
"""

from __future__ import annotations
import argparse
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.ecu_link import ECULink  # noqa: E402
from tools.lib.stim_link import StimLink  # noqa: E402


def _open_stim(spec: str) -> StimLink:
    if spec.startswith("tcp:"):
        hostport = spec[4:]
        if ":" in hostport:
            host, port_s = hostport.rsplit(":", 1)
            return StimLink.tcp(host, int(port_s))
        return StimLink.tcp(hostport, 3333)
    return StimLink.serial(spec)


def _read_diag_late(ecu: ECULink) -> int | None:
    """UART 'D' → 52×u32; [0] = late_event_count (TIM2 no build encoder)."""
    data = ecu.ser  # type: ignore[attr-defined]
    data.reset_input_buffer()
    data.write(b"D")
    time.sleep(0.12)
    raw = data.read(52 * 4)
    if len(raw) < 4:
        return None
    return struct.unpack_from("<I", raw, 0)[0]


def main() -> int:
    p = argparse.ArgumentParser(description="Smoke HIL encoder stim")
    p.add_argument("--stm32", required=True, help="CDC port, e.g. /dev/ttyACM0")
    p.add_argument("--stim", required=True,
                   help="ESP32 encoder stim: /dev/ttyUSB0 or tcp:host:3333")
    p.add_argument("--bench-clt-iat", action="store_true",
                   help="Força CLT/IAT de bancada na ECU (comando 'B')")
    p.add_argument("--idle-s", type=float, default=30.0,
                   help="Segundos em IDLE a vigiar late count")
    p.add_argument("--rpms", type=int, nargs="*", default=[800, 1500, 3000],
                   help="Sweep opcional após IDLE")
    args = p.parse_args()

    fails = 0
    ecu = ECULink(args.stm32)
    stim = _open_stim(args.stim)

    try:
        if not ecu.ping():
            print("FAIL  ping ECU")
            return 1
        print("PASS  ping ECU")

        if args.bench_clt_iat:
            ecu.bench_mode(True)
            print("INFO  bench CLT/IAT ON")

        stim.preset("IDLE")
        stim.set_rpm(800)
        time.sleep(3.0)

        snap = ecu.snapshot()
        rpm = snap.rpm_x10 / 10.0
        if rpm <= 0.0:
            print(f"FAIL  RPM={rpm:.1f} (esperado > 0 com AB a girar)")
            fails += 1
        else:
            print(f"PASS  RPM={rpm:.1f} (snapshot)")

        late0 = _read_diag_late(ecu)
        if late0 is None:
            print("FAIL  ler diag 'D' (late)")
            fails += 1
            late0 = 0
        else:
            print(f"INFO  late_event_count inicial={late0}")

        print(f"INFO  IDLE {args.idle_s:.0f}s — vigiar late…")
        t_end = time.time() + args.idle_s
        while time.time() < t_end:
            time.sleep(2.0)
            snap = ecu.snapshot()
            if snap.rpm_x10 == 0:
                print("FAIL  RPM caiu a 0 durante IDLE (stall?)")
                fails += 1
                break

        late1 = _read_diag_late(ecu)
        if late1 is None:
            print("FAIL  ler diag 'D' final")
            fails += 1
        else:
            delta = late1 - late0
            # Em IDLE estável o late não deve disparar em rajada; tolerância
            # pequena para ruído de arranque/presync.
            if delta > 5:
                print(f"FAIL  late subiu {delta} em IDLE ({late0}→{late1})")
                fails += 1
            else:
                print(f"PASS  late estável (Δ={delta}, {late0}→{late1})")

        for rpm_cmd in args.rpms:
            stim.set_rpm(rpm_cmd)
            time.sleep(2.5)
            snap = ecu.snapshot()
            got = snap.rpm_x10 / 10.0
            # Encoder Ω→RPM tem folga; smoke só exige movimento coerente.
            if got < rpm_cmd * 0.3:
                print(f"FAIL  RPM cmd={rpm_cmd} got={got:.0f}")
                fails += 1
            else:
                print(f"PASS  RPM sweep cmd={rpm_cmd} got={got:.0f}")

    finally:
        stim.close()
        ecu.close()

    if fails:
        print(f"\nRESULT  {fails} FAIL")
        return 1
    print("\nRESULT  ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
