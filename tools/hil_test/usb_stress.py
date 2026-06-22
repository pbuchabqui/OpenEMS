#!/usr/bin/env python3
"""
usb_stress.py — Stress test USB CDC com sweep realista de RPM.

Simula aceleração/desaceleração gradual (incrementos de 50 RPM a cada 200ms)
e verifica que o USB CDC se mantém estável durante toda a rampa.

Cenários:
  1. Warmup:      200 → 800 RPM  (cranking → idle)
  2. Acceleration: 800 → 4000 RPM (aceleração normal)
  3. Deceleration: 4000 → 800 RPM (desaceleração)
  4. Tip-in:       800 → 3000 RPM (aceleração brusca, step 100 RPM)
  5. Flameout:     800 → 0 RPM    (motor apaga)
  6. Re-crank:     0 → 200 → 800  (re-arranque)

Uso:
    python3 usb_stress.py --stm32 /dev/ttyACM0 --stim /dev/ttyUSB0
    python3 usb_stress.py --stm32 /dev/ttyACM0 --stim tcp:192.168.15.169:3333
"""
from __future__ import annotations

import argparse
import serial
import socket
import struct
import sys
import time


class ECULink:
    def __init__(self, port: str):
        self._ser = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(0.5)
        self._ser.reset_input_buffer()

    def ping(self) -> bool:
        try:
            self._ser.reset_input_buffer()
            self._ser.write(b"\x43")
            r = self._ser.read(2)
            return len(r) == 2 and r[0] == 0x00 and r[1] == 0xAA
        except Exception:
            return False

    def bench_mode(self):
        self._ser.write(b"\x42\x01")
        time.sleep(0.2)

    def snapshot(self) -> dict | None:
        try:
            self._ser.reset_input_buffer()
            self._ser.write(b"\x41")
            time.sleep(0.15)
            r = self._ser.read(66)
            if len(r) < 14:
                if len(r) > 0:
                    return {"_partial": len(r)}
                return {"_timeout": True}
            rpm = struct.unpack_from("<H", r, 0)[0]
            status = struct.unpack_from("<H", r, 12)[0]
            return {
                "rpm": rpm,
                "pw_x10": r[7],
                "adv": r[8] - 40,
                "status": status,
                "sync": bool(status & 1),
                "fault": bool(status & 4),
            }
        except serial.SerialException:
            return {"_disconnect": True}
        except Exception:
            return None

    def close(self):
        self._ser.close()


class StimLink:
    def __init__(self, target: str):
        self._tcp = None
        self._ser = None
        if target.startswith("tcp:"):
            hp = target[4:].partition(":")
            host = hp[0]
            port = int(hp[2]) if hp[2] else 3333
            self._tcp = socket.create_connection((host, port), timeout=3)
            self._tcp.settimeout(2)
            try:
                self._tcp.recv(200)
            except Exception:
                pass
        else:
            self._ser = serial.Serial(target, 115200, timeout=1)
            time.sleep(0.3)
            self._ser.reset_input_buffer()

    def set_rpm(self, rpm: int):
        data = f"RPM {rpm}\n".encode()
        if self._tcp:
            self._tcp.sendall(data)
        else:
            self._ser.write(data)

    def close(self):
        if self._tcp:
            self._tcp.close()
        if self._ser:
            self._ser.close()


def ramp(stim: StimLink, ecu: ECULink, start: int, end: int,
         step: int = 50, interval_ms: int = 200, label: str = "") -> dict:
    """Rampa de RPM gradual. Retorna métricas."""
    direction = 1 if end > start else -1
    step = abs(step) * direction
    rpm = start
    usb_ok = 0
    usb_fail = 0
    rpm_err_max = 0.0
    sync_losses = 0
    samples = 0

    print(f"  {label}: {start} → {end} RPM (step {abs(step)}, {interval_ms}ms)")

    while (direction > 0 and rpm <= end) or (direction < 0 and rpm >= end):
        stim.set_rpm(max(0, rpm))
        time.sleep(interval_ms / 1000.0)

        snap = ecu.snapshot()
        if snap is None or snap.get("_disconnect") or snap.get("_timeout") or snap.get("_partial"):
            usb_fail += 1
            reason = "disconnect" if snap and snap.get("_disconnect") else \
                     f"partial({snap['_partial']}B)" if snap and snap.get("_partial") else \
                     "timeout" if snap and snap.get("_timeout") else "null"
            if usb_fail <= 5:
                print(f"      [{reason} @ RPM {rpm}]")
        else:
            usb_ok += 1
            samples += 1
            if rpm > 0 and snap["sync"]:
                err = abs(snap["rpm"] - rpm) / max(rpm, 1) * 100
                if err > rpm_err_max:
                    rpm_err_max = err
            if snap.get("status", 0) & 0x0008:
                sync_losses += 1

        rpm += step

    status = "✓" if usb_fail == 0 else "✗"
    print(f"    {status} USB: {usb_ok} ok / {usb_fail} fail | "
          f"RPM err max: {rpm_err_max:.1f}% | sync losses: {sync_losses}")

    return {
        "label": label,
        "usb_ok": usb_ok,
        "usb_fail": usb_fail,
        "rpm_err_max": rpm_err_max,
        "sync_losses": sync_losses,
        "samples": samples,
    }


def main():
    p = argparse.ArgumentParser(description="USB CDC stress test with RPM sweep")
    p.add_argument("--stm32", required=True)
    p.add_argument("--stim", required=True)
    p.add_argument("--duration", type=int, default=0,
                   help="Continuous sweep duration in seconds (0 = run scenarios only)")
    args = p.parse_args()

    print("╔══════════════════════════════════════╗")
    print("║   USB Stress Test — RPM Sweep        ║")
    print("╚══════════════════════════════════════╝")

    ecu = ECULink(args.stm32)
    for attempt in range(5):
        if ecu.ping():
            break
        time.sleep(1)
    else:
        print("ERRO: STM32 não responde")
        sys.exit(1)
    print(f"  ✓ STM32 OK ({args.stm32})")

    ecu.bench_mode()
    print("  ✓ Bench mode ON")

    stim = StimLink(args.stim)
    print(f"  ✓ Estimulador OK ({args.stim})")

    # Iniciar em idle
    stim.set_rpm(700)
    time.sleep(3)

    scenarios = [
        ("Warmup",       200, 800,  50, 200),
        ("Acceleration", 800, 4000, 50, 200),
        ("Deceleration", 4000, 800, 50, 200),
        ("Tip-in",       800, 3000, 100, 100),
        ("Tip-out",      3000, 800, 100, 100),
        ("Flameout",     800, 0,   50, 200),
        ("Re-crank",     0,   800, 50, 200),
    ]

    print(f"\n{'─'*60}")
    print(f"  Cenários de sweep ({len(scenarios)} rampas)")
    print(f"{'─'*60}")

    results = []
    for label, start, end, step, interval in scenarios:
        stim.set_rpm(start)
        time.sleep(1)
        r = ramp(stim, ecu, start, end, step, interval, label)
        results.append(r)

    # Continuous sweep
    if args.duration > 0:
        print(f"\n{'─'*60}")
        print(f"  Sweep contínuo ({args.duration}s)")
        print(f"{'─'*60}")
        t0 = time.time()
        cycle = 0
        total_ok = 0
        total_fail = 0
        while time.time() - t0 < args.duration:
            r1 = ramp(stim, ecu, 800, 4000, 50, 150, f"cycle {cycle} up")
            r2 = ramp(stim, ecu, 4000, 800, 50, 150, f"cycle {cycle} down")
            total_ok += r1["usb_ok"] + r2["usb_ok"]
            total_fail += r1["usb_fail"] + r2["usb_fail"]
            cycle += 1
        results.append({
            "label": f"Continuous ({cycle} cycles)",
            "usb_ok": total_ok, "usb_fail": total_fail,
            "rpm_err_max": 0, "sync_losses": 0, "samples": total_ok,
        })

    # Summary
    stim.set_rpm(0)
    stim.close()
    ecu.close()

    total_ok = sum(r["usb_ok"] for r in results)
    total_fail = sum(r["usb_fail"] for r in results)
    max_err = max(r["rpm_err_max"] for r in results)
    total_sync = sum(r["sync_losses"] for r in results)

    print(f"\n{'═'*60}")
    passed = total_fail == 0
    print(f"  {'PASS' if passed else 'FAIL'}: "
          f"{total_ok} snapshots OK, {total_fail} USB fails")
    print(f"  RPM tracking max error: {max_err:.1f}%")
    print(f"  Sync losses: {total_sync}")
    print(f"{'═'*60}")

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
