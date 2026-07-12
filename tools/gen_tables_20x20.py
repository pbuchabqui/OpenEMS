#!/usr/bin/env python3
"""gen_tables_20x20.py — gera os initializers 20×20 de calibration.cpp.

Resample bilinear dos defaults 16×16 (embutidos abaixo, copiados de
calibration.cpp/table3d.cpp antes da migração) para os eixos novos de 20
pontos. Como os eixos antigos são subconjunto dos novos, o resample é exato
nas 16 linhas/colunas originais e interpolação 1D pura nas 4 inseridas por
eixo. Proveniência dos números da migração 16→20 (commit "flip").

Uso: python3 tools/gen_tables_20x20.py > /tmp/tables20.txt
"""

OLD_RPM = [500, 750, 1000, 1250, 1500, 2000, 2500, 3000,
           3500, 4000, 4500, 5000, 5500, 6000, 7000, 8000]
NEW_RPM = [500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750,
           3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 8000]

OLD_LOAD = [20, 30, 40, 52, 64, 76, 88, 100, 110, 130, 160, 190, 220, 250, 273, 300]
NEW_LOAD = [20, 30, 40, 46, 52, 58, 64, 70, 76, 88, 94, 100,
            110, 130, 160, 190, 220, 250, 273, 300]

VE = [
    [45, 47, 50, 52, 53, 54, 55, 56, 58, 60, 62, 63, 64, 66, 68, 70],
    [48, 52, 55, 57, 58, 59, 60, 61, 63, 65, 67, 68, 70, 72, 74, 76],
    [52, 56, 60, 62, 63, 64, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82],
    [56, 61, 64, 66, 67, 68, 69, 70, 72, 74, 76, 78, 80, 82, 84, 86],
    [60, 65, 68, 70, 72, 73, 74, 75, 77, 79, 81, 83, 85, 87, 89, 91],
    [64, 68, 72, 75, 77, 79, 80, 81, 83, 85, 87, 89, 91, 93, 95, 96],
    [68, 72, 76, 79, 81, 83, 84, 85, 87, 89, 91, 93, 95, 97, 99, 100],
    [70, 75, 79, 82, 84, 86, 87, 88, 90, 92, 94, 96, 98, 100, 102, 103],
    [73, 78, 82, 85, 88, 90, 91, 92, 94, 96, 98, 100, 102, 104, 106, 107],
    [90, 96, 102, 107, 111, 114, 116, 118, 121, 124, 127, 130, 133, 136, 139, 142],
    [105, 113, 120, 126, 131, 135, 138, 141, 145, 149, 153, 157, 161, 165, 169, 173],
    [120, 130, 138, 146, 152, 157, 161, 165, 170, 175, 180, 185, 190, 195, 200, 205],
    [135, 146, 156, 165, 173, 179, 184, 189, 195, 201, 207, 213, 219, 225, 231, 237],
    [150, 163, 175, 186, 195, 203, 209, 215, 222, 229, 236, 243, 250, 252, 253, 254],
    [165, 180, 194, 207, 218, 227, 235, 242, 250, 252, 253, 254, 254, 254, 254, 254],
    [180, 197, 213, 228, 241, 250, 252, 253, 254, 254, 254, 254, 254, 254, 254, 254],
]

LAMBDA = [
    [1050]*16,
    [1030]*16,
    [1010]*16,
    [1000]*16,
    [1000]*16,
    [1000]*16,
    [1000]*16,
    [990]*16,
    [970, 970, 970, 970, 960, 950, 940, 930, 920, 920, 920, 920, 920, 930, 940, 950],
    [930, 930, 925, 920, 915, 910, 900, 895, 890, 890, 890, 895, 900, 905, 910, 920],
    [900, 900, 895, 890, 885, 880, 870, 865, 860, 860, 865, 870, 875, 885, 895, 905],
    [880, 880, 875, 870, 865, 855, 845, 840, 835, 835, 840, 845, 855, 865, 875, 890],
    [860, 860, 855, 850, 845, 835, 825, 820, 815, 815, 820, 830, 840, 850, 865, 880],
    [845, 845, 840, 835, 830, 820, 810, 805, 800, 800, 805, 815, 825, 840, 855, 875],
    [825, 825, 820, 815, 810, 800, 790, 785, 780, 780, 785, 795, 810, 830, 850, 870],
    [810, 810, 805, 800, 795, 785, 775, 770, 765, 765, 775, 790, 805, 825, 850, 870],
]

SPARK = [
    [12, 15, 18, 22, 26, 30, 33, 32, 34, 35, 38, 40, 40, 39, 37, 35],
    [10, 13, 16, 20, 24, 28, 31, 30, 32, 33, 36, 38, 38, 37, 35, 33],
    [8, 11, 14, 18, 22, 26, 29, 28, 30, 31, 34, 36, 36, 35, 33, 31],
    [6, 9, 12, 16, 20, 23, 26, 25, 27, 28, 31, 33, 33, 32, 30, 28],
    [4, 7, 10, 14, 18, 21, 24, 23, 25, 26, 29, 31, 31, 30, 28, 26],
    [2, 5, 8, 12, 15, 18, 21, 20, 22, 23, 26, 28, 28, 27, 25, 23],
    [1, 3, 6, 10, 13, 16, 19, 18, 20, 21, 24, 26, 26, 25, 23, 21],
    [0, 2, 4, 8, 11, 14, 17, 16, 18, 19, 22, 24, 24, 23, 21, 19],
    [0, 1, 3, 6, 9, 12, 15, 14, 16, 17, 20, 22, 22, 21, 19, 17],
    [0, 0, 2, 5, 8, 11, 13, 12, 14, 15, 18, 20, 20, 19, 17, 15],
    [0, 0, 1, 4, 7, 9, 11, 10, 12, 13, 16, 18, 18, 17, 15, 13],
    [0, 0, 0, 3, 5, 7, 9, 8, 10, 11, 14, 16, 16, 15, 13, 11],
    [0, 0, 0, 2, 4, 6, 8, 7, 9, 10, 13, 15, 15, 14, 12, 10],
    [2, 2, 2, 4, 6, 8, 9, 8, 10, 11, 14, 16, 16, 15, 13, 11],
    [1, 1, 2, 3, 5, 7, 8, 7, 9, 10, 13, 15, 15, 14, 12, 10],
    [0, 1, 1, 2, 4, 6, 7, 6, 8, 9, 12, 14, 14, 13, 11, 9],
]


def interp_1d(x, xs, ys):
    """Interpolação linear com clamp nas pontas."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            f = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + f * (ys[i + 1] - ys[i])
    return ys[-1]


def resample(table, lo=None, hi=None):
    """16×16 [load][rpm] → 20×20 nos eixos novos (separável: RPM, depois load)."""
    # 1) cada linha (load fixo) para o eixo RPM novo
    rows_rpm = [[interp_1d(r, OLD_RPM, row) for r in NEW_RPM] for row in table]
    # 2) cada coluna (rpm fixo) para o eixo de load novo
    out = []
    for ld in NEW_LOAD:
        col_src = list(zip(*rows_rpm))
        row = [interp_1d(ld, OLD_LOAD, [rows_rpm[i][c] for i in range(16)])
               for c in range(20)]
        out.append(row)
    # arredonda e clampa
    res = []
    for row in out:
        r = [int(round(v)) for v in row]
        if lo is not None:
            r = [max(lo, min(hi, v)) for v in r]
        res.append(r)
    return res


def emit(name, decl, table, suffix=""):
    print(f"{decl} = {{")
    for row in table:
        print("    {" + ", ".join(f"{v}{suffix}" for v in row) + "},")
    print("};\n")


if __name__ == "__main__":
    ve20 = resample(VE, 0, 255)
    la20 = resample(LAMBDA, 0, 32767)
    sp20 = resample(SPARK, -128, 127)
    print("// Gerado por tools/gen_tables_20x20.py — resample bilinear dos")
    print("// defaults 16×16 para os eixos de 20 pontos (exato nas linhas antigas).\n")
    emit("ve", "uint8_t ve_table[kTableAxisSize][kTableAxisSize]", ve20, "u")
    emit("lambda", "int16_t lambda_target_table_x1000[kTableAxisSize][kTableAxisSize]", la20)
    emit("spark", "int8_t spark_table[kTableAxisSize][kTableAxisSize]", sp20)
    rpm_x10 = [r * 10 for r in NEW_RPM]
    print("uint32_t kRpmAxisX10[kTableAxisSize] = {")
    print("    " + ", ".join(str(v) + "u" for v in rpm_x10[:10]) + ",")
    print("    " + ", ".join(str(v) + "u" for v in rpm_x10[10:]) + ",\n};\n")
    print("uint32_t kLoadAxisBarX100[kTableAxisSize] = {")
    print("    " + ", ".join(str(v) + "u" for v in NEW_LOAD[:10]) + ",")
    print("    " + ", ".join(str(v) + "u" for v in NEW_LOAD[10:]) + ",\n};")
