#!/usr/bin/env python3
"""Lê a latência da ISR do CKP e os monitores do ICACHE pelo comando serial 'k' (e zera os máximos com 'l').

Uso (placa em /dev/ttyACM0, só leitura; nenhum comando mexe em saída ou calibração):

    python3 -I tools/isr_diag.py                    # uma leitura
    python3 -I tools/isr_diag.py --reset            # zera máximos e monitores do ICACHE antes de ler
    python3 -I tools/isr_diag.py --watch 2          # lê a cada 2 s até Ctrl+C
    python3 -I tools/isr_diag.py --port /dev/ttyACM1

Unidade: ticks do TIM5 a 62,5 MHz (16 ns). `gap` é o dente do gap, onde roda o rebuild da tabela
angular (pior caso). Procedimento de medição: docs/bench_test_manual.md.
"""
import argparse
import os
import select
import struct
import termios
import time
import tty

TICK_NS = 16.0  # TIM5 a 62,5 MHz
ACK_OK = None   # o valor do ACK do 'l' não é conferido aqui; basta receber 1 byte


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def read_exact(fd, n, timeout):
    buf = b""
    end = time.time() + timeout
    while time.time() < end and len(buf) < n:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                buf += os.read(fd, 4096)
            except BlockingIOError:
                pass
    return buf


def drain(fd, t=0.2):
    read_exact(fd, 1 << 30, t)


def command(fd, cmd, n, timeout=1.0):
    drain(fd)  # descarta telemetria espontânea pendente
    os.write(fd, cmd)
    return read_exact(fd, n, timeout)


def fmt_ticks(t):
    return "%9d ticks = %8.2f us" % (t, t * TICK_NS / 1000.0)


def read_k(fd):
    b = command(fd, b"k", 41)
    if len(b) < 25:
        raise SystemExit("resposta curta do 'k': %d bytes (firmware sem o comando?)" % len(b))
    tooth_max, tooth_last, gap_max, gap_last, hits, misses = struct.unpack("<6I", b[:24])
    # Firmware com 41 bytes traz o custo das partes do gap; o de 25 bytes não (None).
    parts = struct.unpack("<4I", b[25:41]) if len(b) >= 41 else None
    return tooth_max, tooth_last, gap_max, gap_last, hits, misses, b[24], parts


def show(vals):
    tooth_max, tooth_last, gap_max, gap_last, hits, misses, on, parts = vals
    print("dente normal  max %s   ultimo %s" % (fmt_ticks(tooth_max), fmt_ticks(tooth_last)))
    print("dente do gap  max %s   ultimo %s" % (fmt_ticks(gap_max), fmt_ticks(gap_last)))
    if parts is not None:
        reb_max, reb_last, arm_max, arm_last = parts
        print("  gap: rebuild  max %s   ultimo %s" % (fmt_ticks(reb_max), fmt_ticks(reb_last)))
        print("  gap: arma d0  max %s   ultimo %s" % (fmt_ticks(arm_max), fmt_ticks(arm_last)))
        # "ultimo" é do mesmo gap nas três linhas: o que sobra é o restante do tratamento (process_gap,
        # sensores, ...) mais o fetch frio e a latência de entrada, que o rebuild/arm0 não incluem.
        print("  gap: resto    ultimo %s   (gap - rebuild - arma d0)" % fmt_ticks(gap_last - reb_last - arm_last))
    # Os monitores SATURAM (hit em 2^32-1, miss em 65535) e não dão a volta (RM0481 §8.4.12): a 250 MHz o de
    # hit enche em ~17 s. Depois de saturar, a taxa de acerto não vale; use --reset e leia logo em seguida.
    saturated = hits == 0xFFFFFFFF or misses == 0xFFFF
    total = hits + misses
    rate = (100.0 * hits / total) if total else 0.0
    print("ICACHE %s  hits=%d misses=%d  acerto=%s" %
          ("ligado" if on else "DESLIGADO", hits, misses,
           ("%.2f%%" % rate) if not saturated else "n/d (monitor saturou: rode com --reset e leia em poucos s)"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--reset", action="store_true", help="zera os máximos ('l') antes de ler")
    ap.add_argument("--watch", type=float, metavar="S", help="repete a leitura a cada S segundos")
    args = ap.parse_args()

    fd = open_port(args.port)
    try:
        if args.reset:
            ack = command(fd, b"l", 1)
            if len(ack) != 1:
                raise SystemExit("sem ACK do 'l'")
            print("maximos e monitores do ICACHE zerados")
        while True:
            show(read_k(fd))
            if args.watch is None:
                break
            print("-" * 60)
            time.sleep(args.watch)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
