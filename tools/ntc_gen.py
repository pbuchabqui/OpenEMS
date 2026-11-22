#!/usr/bin/env python3
"""ntc_gen.py — gera a página 0x0D (curvas NTC de CLT/IAT + escala de pressão) do OpenEMS.

O firmware não tem float/libm: a LUT de 128 baldes (raw>>5, °C×10) é calculada aqui, no host,
a partir do modelo Beta do NTC e da topologia do divisor, e gravada na página 0x0D (NVM slot 10).
Sem página válida o firmware usa a reta -40..150 °C (placeholder).

Modelo (ADC de 12 bits, referência --vref):
    5 V (--vsup) --[--rpull--]--+-- NTC -- GND
                                +-- (opcional) --ra--+-- ADC
                                                     +-- rb -- GND
O atenuador ra/rb carrega o nó do NTC (Ra+Rb em paralelo com o NTC); o modelo inclui isso.
Os parâmetros do sensor (--r25, --beta) e do divisor DEVEM vir do datasheet e do hardware real
da bancada; nenhum valor padrão do NTC é "o do sensor".

Uso:
  python3 tools/ntc_gen.py --rpull 2490 --vsup 3.3 --r25 2500 --beta 3500 -o ntc_page.bin
  python3 tools/ntc_gen.py ... --send /dev/ttyACM0      # grava na RAM da ECU e queima (motor parado)

Escala de pressão (combustível/óleo): reta por dois pontos raw->bar, p.ex. transdutor 0,5-4,5 V
(0-10 bar) atrás de um divisor: --fuel-cal RAW_A BAR_A RAW_B BAR_B (bar com decimais) e --fuel-nominal BAR
(pressão nominal do ΔP; 0 desliga a compensação). Sem esses argumentos, --send preserva o que já
está na ECU e -o grava o default (0..2,5 bar em 0..4095, nominal 3,0 bar).

A escrita da página é uma única 'x' de 544 B (a página é validada inteira; fatias parciais
seriam rejeitadas por não-monotonicidade), seguida de 'b' se --send.
Os baldes têm 32 contagens e o firmware interpola entre eles; o relatório mostra o erro da
interpolação no meio do balde perto dos limiares de DFCO (70 °C) e overtemp (105/115 °C).
"""
import argparse
import math
import struct
import sys

N = 128
BUCKET = 32
T_MIN_X10, T_MAX_X10 = -400, 1500


def raw_to_temp_x10(raw: float, a) -> int:
    vadc = raw / 4095.0 * a.vref
    atten = a.rb / (a.ra + a.rb) if a.ra > 0 else 1.0
    vn = vadc / atten
    if vn <= 1e-6:
        return T_MAX_X10                           # nó em curto a GND = NTC ~0 ohm => quente
    if vn >= a.vsup - 1e-6:
        return T_MIN_X10                           # NTC aberto => frio/aberto
    rpar = a.rpull * vn / (a.vsup - vn)
    if a.ra > 0:
        inv = 1.0 / rpar - 1.0 / (a.ra + a.rb)
        if inv <= 0:
            return T_MIN_X10
        rntc = 1.0 / inv
    else:
        rntc = rpar
    t_k = 1.0 / (1.0 / 298.15 + math.log(rntc / a.r25) / a.beta)
    t = round((t_k - 273.15) * 10)
    return max(T_MIN_X10, min(T_MAX_X10, t))


def build(a):
    # amostra na borda esquerda do balde (raw = i*32): o firmware interpola entre baldes
    return [raw_to_temp_x10(i * BUCKET, a) for i in range(N)]


def report(curve, a):
    down = all(curve[i] >= curve[i + 1] for i in range(N - 1))
    up = all(curve[i] <= curve[i + 1] for i in range(N - 1))
    print(f"monótona: {'sim' if (up or down) else 'NÃO'}  amplitude: {abs(curve[-1] - curve[0]) / 10:.0f} °C")
    for target in (700, 1050, 1150):
        # balde cuja temperatura (°C×10) fica mais perto do limiar
        i = min(range(N), key=lambda k: abs(curve[k] - target))
        i = min(i, N - 2)
        mid = raw_to_temp_x10(i * BUCKET + BUCKET // 2, a)
        lin = (curve[i] + curve[i + 1]) / 2
        print(f"  ~{target / 10:5.1f} °C: balde {i:3d}  {curve[i] / 10:6.1f}..{curve[i + 1] / 10:6.1f} °C "
              f"(erro da interpolação no meio do balde {abs(mid - lin) / 10:.2f} °C)")
    return up or down


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rpull", type=float, required=True, help="pull-up do divisor, ohms")
    p.add_argument("--vsup", type=float, required=True, help="tensão do pull-up, V")
    p.add_argument("--vref", type=float, default=3.3, help="referência do ADC, V (padrão 3,3)")
    p.add_argument("--ra", type=float, default=0.0, help="atenuador: resistor série, ohms (0 = sem)")
    p.add_argument("--rb", type=float, default=0.0, help="atenuador: resistor para GND, ohms")
    p.add_argument("--r25", type=float, required=True, help="R do CLT a 25 °C, ohms")
    p.add_argument("--beta", type=float, required=True, help="B do CLT (K)")
    p.add_argument("--iat-r25", type=float, help="R do IAT a 25 °C (padrão = CLT)")
    p.add_argument("--iat-beta", type=float, help="B do IAT (padrão = CLT)")
    p.add_argument("--fuel-cal", nargs=4, type=float, metavar=("RAW_A", "BAR_A", "RAW_B", "BAR_B"))
    p.add_argument("--oil-cal", nargs=4, type=float, metavar=("RAW_A", "BAR_A", "RAW_B", "BAR_B"))
    p.add_argument("--fuel-nominal", type=float, metavar="BAR", help="pressão nominal do ΔP, bar")
    p.add_argument("-o", "--out", help="grava o binário de 544 B (CLT, IAT, escala de pressão)")
    p.add_argument("--send", metavar="PORTA", help="escreve na ECU (x) e queima (b); motor parado")
    a = p.parse_args()
    if a.ra > 0 and a.rb <= 0:
        p.error("--rb é obrigatório com --ra")

    clt = build(a)
    ia = argparse.Namespace(**vars(a))
    ia.r25 = a.iat_r25 or a.r25
    ia.beta = a.iat_beta or a.beta
    iat = build(ia)

    print("CLT:")
    ok = report(clt, a)
    print("IAT:")
    ok = report(iat, ia) and ok
    if not ok or abs(clt[-1] - clt[0]) < 500 or abs(iat[-1] - iat[0]) < 500:
        print("curva rejeitada pelo firmware (não monótona ou amplitude < 50 °C): confira os parâmetros",
              file=sys.stderr)
        return 1

    def cal_bytes(v):
        ra, ba, rb, bb = int(round(v[0])), int(round(v[1] * 1000)), int(round(v[2])), int(round(v[3] * 1000))
        if not (0 <= ra < rb <= 4095 and 0 <= ba < bb <= 20000):
            print("escala de pressão inválida (precisa raw_a < raw_b <= 4095 e bar_a < bar_b <= 20)",
                  file=sys.stderr)
            sys.exit(1)
        return struct.pack("<4H", ra, ba, rb, bb)

    curves = struct.pack(f"<{N}h", *clt) + struct.pack(f"<{N}h", *iat)
    have_press = a.fuel_cal or a.oil_cal or a.fuel_nominal is not None
    default_press = (struct.pack("<4H", 0, 0, 4095, 2500) * 2 + struct.pack("<H", 3000)).ljust(32, b"\0")
    link = None
    if a.send:
        sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent / "openems_dash"))
        from protocol import OpenEMSLink  # noqa: E402
        link = OpenEMSLink(a.send)
    try:
        if have_press or link is None:
            press = bytearray(default_press)
        else:
            press = bytearray(link.read_page(0x0D, 512, 32))   # preserva o que a ECU já tem
        if a.fuel_cal:
            press[0:8] = cal_bytes(a.fuel_cal)
        if a.oil_cal:
            press[8:16] = cal_bytes(a.oil_cal)
        if a.fuel_nominal is not None:
            nominal = int(round(a.fuel_nominal * 1000))
            if nominal != 0 and not 500 <= nominal <= 20000:
                print("--fuel-nominal fora de 0,5..20 bar (ou 0)", file=sys.stderr)
                return 1
            press[16:18] = struct.pack("<H", nominal)
        page = curves + bytes(press)
        if a.out:
            with open(a.out, "wb") as f:
                f.write(page)
            print(f"gravado {a.out} ({len(page)} B)")
        if link is not None:
            link.write_page_ram(0x0D, 0, page)
            link.burn_page(0x0D)
            print("página 0x0D enviada e queimada")
    finally:
        if link is not None:
            link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
