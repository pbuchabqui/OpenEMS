#!/usr/bin/env python3
"""Erro angular e jitter das saídas IGN da ECU, medidos pelo scope do estimulador (tools/stm32_bench).

Lê o log do modo EDGE (comando 'e') do estimulador: cada linha traz o instante de uma borda em µs (base de tempo
única, TIM5 a 1 MHz) e o canal. O CKP vem com o instante exato do compare do gerador; IGN/INJ vêm de EXTI. O script
acha o dente 0 (primeiro dente depois do gap), mede onde cai cada faísca (borda de descida da IGN) em graus depois do
dente 0 e compara com o ângulo esperado. Procedimento completo: docs/bench_test_manual.md, T7d.

Uso (só leitura: o único byte que envia à placa é 'e' no início e 'l' no fim, que trocam o modo do scope):

    # captura 5 min da bancada e analisa (salva o log bruto em --save)
    python3 -I tools/ign_jitter.py --port /dev/ttyACM1 --seconds 300 --save /tmp/ign_3000.log --advance 10

    # analisa um log já gravado
    python3 -I tools/ign_jitter.py --log /tmp/ign_3000.log --advance 10 --csv /tmp/ign.csv --label 3000rpm

    # calibra o scope: jumper PA0 do gerador -> entrada INJ4 (PB13), com o fio da INJ4 da ECU desligado
    python3 -I tools/ign_jitter.py --log /tmp/calib_3000.log --calib INJ4

Conversão: 1° = T_rev / 360, com T_rev o tempo entre dois dentes 0 seguidos. Erro positivo = faísca atrasada.
Os ângulos esperados (PMS de cada IGN, módulo 360°) vêm da tabela do T3; mude com --pms se a ordem de ignição mudar.
"""
import argparse
import bisect
import math
import os
import re
import select
import statistics
import sys
import termios
import time
import tty

EDGE_RE = re.compile(r"^\s*(-?\d+)\s+\S+\s+CH(\d+)\s+(\w+)\s+(RISE|FALL)")
TEETH_PER_REV = 58          # roda 60-2: dentes reais por volta
DEFAULT_PMS = {"IGN1": 0.0, "IGN2": 180.0, "IGN3": 180.0, "IGN4": 0.0}   # PMS mod 360° (T3: 1-3-4-2)


def parse(lines):
    """-> (eventos [(t_us, nome, nível)], avisos de overflow do scope)"""
    ev, warn = [], 0
    for ln in lines:
        m = EDGE_RE.match(ln)
        if m:
            ev.append((int(m.group(1)), m.group(3), 1 if m.group(4) == "RISE" else 0))
        elif "AVISO" in ln or "overflow" in ln.lower():
            warn += 1
    ev.sort(key=lambda e: e[0])   # EXTI e gerador chegam ao buffer em ordem levemente diferente da real
    return ev, warn


def pct(sorted_vals, p):
    """Percentil por posto mais próximo (sem interpolar: um valor medido de verdade)."""
    if not sorted_vals:
        return float("nan")
    k = max(0, min(len(sorted_vals) - 1, int(math.ceil(p / 100.0 * len(sorted_vals))) - 1))
    return sorted_vals[k]


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def find_tooth0(rises):
    """Instantes do dente 0: a subida cujo intervalo desde a anterior é > 2x a mediana dos 5 intervalos antes dela."""
    out = []
    for i in range(6, len(rises)):
        prev = [rises[j] - rises[j - 1] for j in range(i - 5, i)]
        if rises[i] - rises[i - 1] > 2.0 * statistics.median(prev):
            out.append(rises[i])
    return out


def revolutions(ev, max_dev):
    """Voltas íntegras: [(t0_inicio, t0_fim)] com 58 subidas de CKP e T_rev dentro de max_dev da mediana."""
    rises = [t for t, n, lv in ev if n == "CKP" and lv == 1]
    t0 = find_tooth0(rises)
    if len(t0) < 3:
        return [], [], len(rises)
    trev = [b - a for a, b in zip(t0, t0[1:])]
    med = statistics.median(trev)
    good, bad = [], 0
    for a, b in zip(t0, t0[1:]):
        n = bisect.bisect_left(rises, b) - bisect.bisect_left(rises, a)
        if n == TEETH_PER_REV and abs((b - a) - med) <= max_dev * med:
            good.append((a, b))
        else:
            bad += 1
    return good, bad, len(rises)


def describe(vals):
    s = sorted(vals)
    return (len(s), statistics.fmean(s), statistics.pstdev(s) if len(s) > 1 else 0.0,
            pct(s, 50), pct(s, 99), pct(s, 99.9), s[0], s[-1])


def analyse_angle(ev, adv, pms, bias_us, max_dev, csv_path, label):
    revs, bad, nrise = revolutions(ev, max_dev)
    if not revs:
        sys.exit("sem voltas íntegras: o log não tem CKP com gap reconhecível (modo 'e' ligado? motor girando?)")
    starts = [a for a, _ in revs]
    trevs = [b - a for a, b in revs]
    rpm = 60e6 / statistics.median(trevs)
    print("voltas íntegras: %d   descartadas (≠58 dentes ou T_rev fora de ±%.0f%%): %d   rpm ≈ %.0f"
          % (len(revs), max_dev * 100, bad, rpm))
    # Piso de ruído: a dispersão do próprio CKP entra em todo erro abaixo.
    print("piso do gerador: T_rev média %.2f µs  desvio %.3f µs  pico a pico %d µs  (= %.4f° de desvio)"
          % (statistics.fmean(trevs), statistics.pstdev(trevs), max(trevs) - min(trevs),
             statistics.pstdev(trevs) / statistics.fmean(trevs) * 360.0))
    rows = []
    print("\navanço %.2f°, viés subtraído %.2f µs. Erro = posição medida − esperada; positivo = faísca atrasada\n"
          % (adv, bias_us))
    print("%-5s %7s %9s %9s %9s %9s %10s %10s %11s" %
          ("saída", "n", "média(°)", "desvio(°)", "p50|e|(°)", "p99|e|(°)", "p99,9|e|(°)", "max|e|(°)", "max|e|(µs)"))
    for name in sorted(n for n in pms if any(e[1] == n for e in ev)):
        errs, errs_us, a_mod = [], [], []
        for t, n, lv in ev:
            if n != name or lv != 0:
                continue
            k = bisect.bisect_right(starts, t) - 1
            if k < 0 or t >= revs[k][1]:
                continue                       # fora de uma volta íntegra
            trev = revs[k][1] - revs[k][0]
            deg_per_us = 360.0 / trev
            a = (t - revs[k][0]) * deg_per_us
            e = wrap180(a - ((pms[name] - adv) % 360.0)) - bias_us * deg_per_us
            errs.append(e)
            errs_us.append(e / deg_per_us)
            a_mod.append(a)
        if not errs:
            print("%-5s   sem faíscas em voltas íntegras" % name)
            continue
        n_, mean, std, _, _, _, _, _ = describe(errs)
        ab = sorted(abs(x) for x in errs)
        abu = sorted(abs(x) for x in errs_us)
        print("%-5s %7d %9.4f %9.4f %9.4f %9.4f %10.4f %10.4f %11.2f" %
              (name, n_, mean, std, pct(ab, 50), pct(ab, 99), pct(ab, 99.9), ab[-1], abu[-1]))
        if abs(mean) > 3.0:
            print("      AVISO: média de %.1f° — fase ou ordem de ignição diferente do esperado (veja T3), não é jitter" % mean)
        rows.append((name, n_, mean, std, pct(ab, 99), ab[-1], abu[-1]))
    if csv_path and rows:
        new = not os.path.exists(csv_path)
        with open(csv_path, "a", encoding="utf-8") as f:
            if new:
                f.write("label,advance_deg,rpm,saida,n,media_deg,desvio_deg,p99_abs_deg,max_abs_deg,max_abs_us\n")
            for r in rows:
                f.write("%s,%.2f,%.0f,%s,%d,%.5f,%.5f,%.5f,%.5f,%.3f\n" % (label, adv, rpm, r[0], r[1], r[2], r[3], r[4], r[5], r[6]))
        print("\n%d linhas acrescentadas a %s" % (len(rows), csv_path))


def analyse_calib(ev, name):
    """Latência e jitter do EXTI de `name`: a mesma borda aparece no CKP (instante exato do compare) e no canal."""
    ref = {1: [t for t, n, lv in ev if n == "CKP" and lv == 1], 0: [t for t, n, lv in ev if n == "CKP" and lv == 0]}
    mine = {1: [t for t, n, lv in ev if n == name and lv == 1], 0: [t for t, n, lv in ev if n == name and lv == 0]}
    lat = []
    for lv in (0, 1):
        r, m = ref[lv], mine[lv]
        for t in m:
            i = bisect.bisect_left(r, t)
            cands = [r[j] for j in (i - 1, i) if 0 <= j < len(r)]
            if not cands:
                continue
            c = min(cands, key=lambda x: abs(x - t))
            if abs(t - c) <= 25:           # só a mesma borda: meio dente a 8500 rpm = 59 µs
                lat.append(t - c)
    if len(lat) < 100:
        sys.exit("poucos pares CKP/%s (%d): jumper PA0 -> entrada do %s ligado? modo 'e' ligado?" % (name, len(lat), name))
    n, mean, std, p50, p99, p999, lo, hi = describe(lat)
    print("calibração do scope (%s contra o CKP exato do gerador): n=%d" % (name, n))
    print("  latência  média %.2f µs   p50 %.1f   p99 %.1f   p99,9 %.1f   min %d   max %d" % (mean, p50, p99, p999, lo, hi))
    print("  jitter    desvio %.3f µs   (max − min) = %d µs" % (std, hi - lo))
    spread = hi - lo
    verdict = ("bom: ≤ 1 µs, o scope resolve o erro de 0,1° mesmo a 8500 rpm" if spread <= 1 else
               "aceitável: ≤ 3 µs; a 8500 rpm isso é ≤ 0,15° de incerteza — inclua na leitura" if spread <= 3 else
               "ruim: > 3 µs, o jitter do EXTI mascara o erro de 0,1°; use analisador lógico externo")
    print("  " + verdict)
    print("  viés para --bias-us: %.2f µs (latência média; o resto é jitter)" % mean)
    print("  pares só até ±25 µs: latência maior que isso não é calibrável aqui (e já seria falha do scope).")


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def capture(port, seconds, save):
    fd = open_port(port)
    buf = b""
    try:
        os.write(fd, b"e\n")                 # modo EDGE do scope
        end = time.time() + seconds
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                try:
                    buf += os.read(fd, 65536)
                except BlockingIOError:
                    pass
        os.write(fd, b"l\n")                 # volta ao modo LIVE
    finally:
        os.close(fd)
    text = buf.decode("utf-8", errors="replace")
    if save:
        with open(save, "w", encoding="utf-8") as f:
            f.write(text)
        print("log bruto salvo em %s (%d bytes)" % (save, len(buf)))
    return text.splitlines()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--log", help="log do modo EDGE já gravado")
    src.add_argument("--port", help="porta do estimulador (USB CDC), ex.: /dev/ttyACM1")
    ap.add_argument("--seconds", type=float, default=300.0, help="duração da captura com --port (padrão 300)")
    ap.add_argument("--save", help="grava o log bruto da captura")
    ap.add_argument("--advance", type=float, default=10.0, help="avanço do modo lâmpada, graus (padrão 10; positivo = antes do PMS)")
    ap.add_argument("--pms", default=None, help="PMS mod 360 por saída, ex.: IGN1=0,IGN2=180,IGN3=180,IGN4=0")
    ap.add_argument("--bias-us", type=float, default=0.0, help="viés constante a subtrair, µs (veja --calib)")
    ap.add_argument("--max-rev-dev", type=float, default=0.25, help="descarta voltas com T_rev fora desta fração da mediana")
    ap.add_argument("--calib", metavar="CANAL", help="modo calibração: latência/jitter do EXTI de CANAL (ex.: INJ4) contra o CKP")
    ap.add_argument("--csv", help="acrescenta uma linha por saída a este CSV (varredura do avanço)")
    ap.add_argument("--label", default="", help="rótulo da corrida no CSV")
    args = ap.parse_args()

    pms = dict(DEFAULT_PMS)
    if args.pms:
        pms = {k: float(v) % 360.0 for k, v in (p.split("=") for p in args.pms.split(","))}

    if args.log:
        with open(args.log, encoding="utf-8", errors="replace") as f:
            lines = f.read().splitlines()
    else:
        lines = capture(args.port, args.seconds, args.save)
    ev, warn = parse(lines)
    if not ev:
        sys.exit("nenhuma borda no log (formato '  <t> µs  CH<n> <NOME> RISE|FALL'; o scope está em modo EDGE?)")
    if warn:
        print("AVISO: %d linhas de overflow/aviso do scope; eventos perdidos viram voltas descartadas abaixo" % warn)
    if args.calib:
        analyse_calib(ev, args.calib)
    else:
        analyse_angle(ev, args.advance, pms, args.bias_us, args.max_rev_dev, args.csv, args.label)


if __name__ == "__main__":
    main()
