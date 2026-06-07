#!/usr/bin/env python3
"""
scope_host.py — Leitor de dados do ESP32 Scope para OpenEMS
════════════════════════════════════════════════════════════
Recebe linhas série do esp32_scope.ino e:
  - Mostra tabela formatada em tempo real (modo terminal)
  - Exporta CSV para análise posterior

Uso:
    python3 scope_host.py --port /dev/ttyUSB0
    python3 scope_host.py --port COM3 --csv log.csv
    python3 scope_host.py --port /dev/ttyUSB0 --mode pulse
"""

import argparse
import csv
import re
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("Instalar: pip install pyserial")
    sys.exit(1)


# ── Parsing de linhas do scope ────────────────────────────────────────────────

RE_PULSE = re.compile(
    r"CH(\d+)\s+(\w+)\s+PW=\s*([\d.]+)\s*ms\s+Period=\s*([\d.]+)\s*ms\s+Count=(\d+)"
)
RE_EDGE = re.compile(
    r"(\d+)\s*µs\s+CH(\d+)\s+(\w+)\s+(RISE|FALL)(?:\s+PW=([\d.]+)\s*ms)?"
)
RE_LIVE = re.compile(
    r"\|\s*(\d+)\|(\w+)\s*\|(\w+)\s*\|\s*([\d.]+)\|\s*([\d.]+)\|\s*([\d.]+)\|\s*(\d+)\|"
)


def parse_live_row(line: str) -> dict | None:
    m = RE_LIVE.search(line)
    if not m:
        return None
    ch, name, pin, pw, period, freq, count = m.groups()
    return {
        "ts": datetime.now().isoformat(timespec="milliseconds"),
        "ch": int(ch),
        "name": name,
        "pin": pin,
        "pw_ms": float(pw),
        "period_ms": float(period),
        "freq_hz": float(freq),
        "count": int(count),
    }


def parse_pulse_row(line: str) -> dict | None:
    m = RE_PULSE.search(line)
    if not m:
        return None
    ch, name, pw, period, count = m.groups()
    return {
        "ts": datetime.now().isoformat(timespec="milliseconds"),
        "ch": int(ch),
        "name": name,
        "pw_ms": float(pw),
        "period_ms": float(period),
        "count": int(count),
    }


def parse_edge_row(line: str) -> dict | None:
    m = RE_EDGE.search(line)
    if not m:
        return None
    ts_us, ch, name, edge, pw = m.groups()
    return {
        "ts_us": int(ts_us),
        "ch": int(ch),
        "name": name,
        "edge": edge,
        "pw_ms": float(pw) if pw else None,
    }


# ── Formatação de tabela ──────────────────────────────────────────────────────

LIVE_FIELDS = ["ts", "ch", "name", "pin", "pw_ms", "period_ms", "freq_hz", "count"]
PULSE_FIELDS = ["ts", "ch", "name", "pw_ms", "period_ms", "count"]
EDGE_FIELDS  = ["ts_us", "ch", "name", "edge", "pw_ms"]


def clear_screen():
    print("\033[2J\033[H", end="")


def print_live_header():
    print(f"{'Timestamp':<26} {'CH':>2} {'Name':<6} {'Pin':<5} "
          f"{'PW (ms)':>8} {'Per (ms)':>9} {'Freq Hz':>8} {'Count':>7}")
    print("─" * 80)


def print_live_row(r: dict):
    print(f"{r['ts']:<26} {r['ch']:>2} {r['name']:<6} {r['pin']:<5} "
          f"{r['pw_ms']:>8.3f} {r['period_ms']:>9.3f} {r['freq_hz']:>8.2f} "
          f"{r['count']:>7}")


# ── Loop principal ────────────────────────────────────────────────────────────

def run(port: str, baud: int, mode: str, csv_path: str | None):
    csv_file = None
    csv_writer = None

    if csv_path:
        csv_file = open(csv_path, "w", newline="")
        fields = {"live": LIVE_FIELDS, "pulse": PULSE_FIELDS, "edge": EDGE_FIELDS}
        csv_writer = csv.DictWriter(csv_file, fieldnames=fields[mode])
        csv_writer.writeheader()
        print(f"A gravar em {csv_path}")

    print(f"Ligando a {port} @ {baud}…")
    with serial.Serial(port, baud, timeout=0.5) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()

        # Enviar comando de modo ao ESP32
        mode_cmd = {"live": b"l", "pulse": b"p", "edge": b"e"}
        ser.write(mode_cmd.get(mode, b"l"))

        print(f"Modo: {mode.upper()}  (Ctrl+C para parar)")

        if mode == "live":
            print_live_header()

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip()

                row = None
                if mode == "live":
                    row = parse_live_row(line)
                    if row:
                        print_live_row(row)
                elif mode == "pulse":
                    row = parse_pulse_row(line)
                    if row:
                        print(f"[{row['ts']}] CH{row['ch']} {row['name']:<5}  "
                              f"PW={row['pw_ms']:.3f} ms  "
                              f"T={row['period_ms']:.3f} ms  "
                              f"#{row['count']}")
                elif mode == "edge":
                    row = parse_edge_row(line)
                    if row:
                        pw_str = f"  PW={row['pw_ms']:.3f} ms" if row["pw_ms"] else ""
                        print(f"{row['ts_us']:>12} µs  CH{row['ch']} "
                              f"{row['name']:<5} {row['edge']}{pw_str}")
                else:
                    print(line)

                if row and csv_writer:
                    csv_writer.writerow(row)
                    csv_file.flush()

        except KeyboardInterrupt:
            print("\nParado.")
        finally:
            if csv_file:
                csv_file.close()
                print(f"Dados guardados em {csv_path}")


# ── Análise de CSV (verificação das medições) ─────────────────────────────────

def analyse_csv(csv_path: str):
    """
    Lê um CSV gravado em modo 'pulse' e verifica se os valores estão
    dentro dos limites esperados para uma ECU a funcionar.
    """
    import statistics

    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append(r)

    by_channel: dict[str, list] = {}
    for r in rows:
        name = r.get("name", "?")
        by_channel.setdefault(name, []).append(float(r["pw_ms"]))

    print(f"\nAnálise de {csv_path}  ({len(rows)} amostras)\n")
    print(f"{'Canal':<8} {'N':>6} {'Média':>9} {'Min':>9} {'Max':>9} {'σ':>8}  Avaliação")
    print("─" * 65)

    for name in sorted(by_channel):
        data = by_channel[name]
        n    = len(data)
        avg  = statistics.mean(data)
        mn   = min(data)
        mx   = max(data)
        sd   = statistics.stdev(data) if n > 1 else 0.0

        # Limites empíricos (ajustar conforme o motor)
        if name.startswith("IGN"):
            ok = 1.0 <= avg <= 5.0 and sd < 0.5
            lim = "dwell 1–5 ms"
        elif name.startswith("INJ"):
            ok = 1.0 <= avg <= 30.0 and sd < 1.0
            lim = "PW 1–30 ms"
        else:
            ok = True
            lim = ""

        status = "✓ OK" if ok else "✗ FORA"
        print(f"{name:<8} {n:>6} {avg:>8.3f}ms {mn:>8.3f}ms "
              f"{mx:>8.3f}ms {sd:>7.3f}ms  {status}  ({lim})")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="OpenEMS ESP32 Scope — host reader")
    p.add_argument("--port",    default="/dev/ttyUSB0",
                   help="Porta série (default: /dev/ttyUSB0)")
    p.add_argument("--baud",    type=int, default=115200)
    p.add_argument("--mode",    choices=["live", "pulse", "edge"], default="live",
                   help="Modo de captura (default: live)")
    p.add_argument("--csv",     metavar="FICHEIRO",
                   help="Guardar medições em CSV")
    p.add_argument("--analyse", metavar="FICHEIRO",
                   help="Analisar CSV existente (sem ligar ao ESP32)")
    args = p.parse_args()

    if args.analyse:
        analyse_csv(args.analyse)
    else:
        run(args.port, args.baud, args.mode, args.csv)


if __name__ == "__main__":
    main()
