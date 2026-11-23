"""Gera hardware/interface_vgt6/*.kicad_sch (KiCad 10). Uso: python3 gen/build.py  (a partir de hardware/interface_vgt6/)

Gerador de uma passada: depois de editar no KiCad, nao rode de novo sobre os mesmos arquivos.
Conexoes por global label e simbolo de alimentacao; layout so organiza por blocos.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicadgen as k
from kicadgen import Sheet, Q, U

PROJECT = "interface_vgt6"
OUT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---- ajustes do gerador ------------------------------------------------------------------------
k.RAIL_NETS.add("+5V_F")
k.RAIL_NETS.discard("VREF+")      # VREF+ nao e alimentado pela carrier: so desacoplamento
_part = Sheet.part


def G(v):
    return round(round(v / 2.54) * 2.54, 2)


def _snap(self, libname, symname, ref_prefix, value, fp, x, y, nets, **kw):
    return _part(self, libname, symname, ref_prefix, value, fp, G(x), G(y), nets, **kw)


Sheet.part = _snap

FP_R = "Resistor_SMD:R_0603_1608Metric"
FP_R12 = "Resistor_SMD:R_1206_3216Metric"
FP_C = "Capacitor_SMD:C_0603_1608Metric"
FP_C8 = "Capacitor_SMD:C_0805_2012Metric"
FP_C12 = "Capacitor_SMD:C_1206_3216Metric"
FP_TSSOP14 = "Package_SO:TSSOP-14_4.4x5mm_P0.65mm"
FP_SOT235 = "Package_TO_SOT_SMD:SOT-23-5"
FP_DPAK = "Package_TO_SOT_SMD:TO-252-3_TabPin2"
FP_SOT23 = "Package_TO_SOT_SMD:SOT-23"
FP_SMA = "Diode_SMD:D_SMA"
FP_SMB = "Diode_SMD:D_SMB"
FP_SOD123 = "Diode_SMD:D_SOD-123"
FP_SOD323 = "Diode_SMD:D_SOD-323"
FP_LED = "LED_SMD:LED_0603_1608Metric"
FP_HDR = "Connector_PinHeader_2.54mm:PinHeader_1x%02d_P2.54mm_Vertical"
FP_MSTB = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_%d-G-5,08_1x%02d_P5.08mm_Horizontal"
FP_MC = "Connector_Phoenix_MC:PhoenixContact_MC_1,5_%d-G-3.81_1x%02d_P3.81mm_Horizontal"


def R(s, x, y, v, a, b, fp=FP_R):
    return s.part("Device", "R", "R", v, fp, x, y, {"1": a, "2": b})


def C(s, x, y, v, a, b, fp=FP_C):
    return s.part("Device", "C", "C", v, fp, x, y, {"1": a, "2": b})


def CP(s, x, y, v, a, b, fp):
    return s.part("Device", "C_Polarized", "C", v, fp, x, y, {"1": a, "2": b})


def PD(s, x, y, net, gnd="SGND", v="10k"):
    return R(s, x, y, v, net, gnd)


def conn(s, x, y, ref, value, fp, nets, nc=()):
    n = len(nets)
    d = {str(i + 1): net for i, net in enumerate(nets) if net}
    return s.part("Connector_Generic", "Conn_01x%02d" % n, "J", value, fp, x, y, d, ref=ref,
                  nc=tuple(str(i + 1) for i, net in enumerate(nets) if not net))


def tvs_uni(s, x, y, val, fp, net, gnd):
    """TVS unidirecional: catodo (pino 1, pad 1 do D_SMA/SMB) na linha protegida"""
    return s.part("Device", "D_Zener", "D", val, fp, x, y, {"1": net, "2": gnd})


def esd(s, x, y, net, gnd="SGND"):
    # SMF6.0A: tensao de trabalho 6 V (acima do sinal de 5 V; o PESD5V0 trabalharia no limite). Valor de memoria, conferir a ficha.
    return s.part("Device", "D_Zener", "D", "SMF6.0A", "Diode_SMD:D_SOD-123F", x, y, {"1": net, "2": gnd})


def clamp(s, x, y, node, hi="+3V3", lo="SGND"):
    return s.part("Diode", "BAT54S", "D", "BAT54S", FP_SOT23, x, y, {"1": lo, "2": hi, "3": node})


def title(s, x, y, txt, size=3.0):
    s.text(txt, G(x), G(y), size)


def note(s, x, y, txt):
    s.text(txt, G(x), G(y), 1.5)


# buffer 74AHCT125: unidade -> (OE, A, Y)
BUFPIN = {1: ("1", "2", "3"), 2: ("4", "5", "6"), 3: ("10", "9", "8"), 4: ("13", "12", "11")}


def buf(s, ref, unit, x, y, a, ynet, oe="OE_N", ncy=False):
    po, pa, py = BUFPIN[unit]
    nets = {po: oe, pa: a}
    nc = ()
    if ynet is None:
        nc = (py,)
    else:
        nets[py] = ynet
    s.part("74xx", "74AHCT125", "U", "74AHCT125", FP_TSSOP14, x, y, nets, unit=unit, ref=ref, nc=nc)


def buf_pwr(s, ref, x, y):
    s.part("74xx", "74AHCT125", "U", "74AHCT125", FP_TSSOP14, x, y, {"7": "SGND", "14": "+5V"}, unit=5, ref=ref)
    C(s, x + 20, y, "100n", "+5V", "SGND")


# ================================================================================================
# 1. ALIMENTACAO
# ================================================================================================
def sheet_power(s):
    title(s, 20, 15, "1. Alimentação: duas entradas (J1A INJ+IGN, J1B ETB+AUX+lógica), buck 5 V, 5 V de sensores")
    note(s, 20, 20, "Fusível geral 15 A por entrada; PFET de proteção de inversão e, depois dele, TVS SMBJ20A; fusíveis por banco (INJ 7,5 A, IGN 10 A, ETB 5 A, AUX 10 A).")

    def entrada(y, tag, raw, fused, prot, J, ref_j, banks):
        conn(s, 30, y, ref_j, "Entrada " + tag, FP_MSTB % (4, 4), [raw, raw, "PGND", "PGND"])
        s.part("Device", "Fuse", "F", "15A", "Fuse:Fuseholder_Blade_ATO_Littelfuse_Pudenz_2_Pin", 110, y, {"1": raw, "2": fused})
        s.part("Transistor_FET", "Q_PMOS_GDS", "Q", "AOD4189", FP_DPAK, 150, y, {"1": tag + "_PG", "2": fused, "3": prot})
        R(s, 180, y, "10k", tag + "_PG", "PGND")
        tvs_uni(s, 70, y + 18, "SMBJ20A", FP_SMB, prot, "PGND")
        s.part("Device", "D_Zener", "D", "BZT52C15", FP_SOD123, 205, y, {"1": prot, "2": tag + "_PG"})
        x = 250
        for (val, net, cap, capfp, capv) in banks:
            s.part("Device", "Fuse", "F", val, "Fuse:Fuseholder_Blade_Mini_Keystone_3568", x, y, {"1": prot, "2": net})
            CP(s, x + 25, y, cap, net, "PGND", capfp)
            C(s, x + 45, y, capv, net, "PGND", FP_C12)
            x += 70

    entrada(50, "A", "VBAT_RAW_A", "VBAT_A_F", "VBAT_A_P", None, "J1A",
            [("7.5A", "VBAT_INJ", "220u/35V", "Capacitor_SMD:CP_Elec_8x10", "10u/50V"),
             ("10A", "VBAT_IGN", "470u/35V", "Capacitor_SMD:CP_Elec_10x10.5", "10u/50V")])
    entrada(110, "B", "VBAT_RAW_B", "VBAT_B_F", "VBAT_PROT", None, "J1B",
            [("5A", "VBAT_ETB", "220u/35V", "Capacitor_SMD:CP_Elec_8x10", "10u/50V"),
             ("10A", "VBAT_AUX", "100u/35V", "Capacitor_SMD:CP_Elec_8x10", "10u/50V")])
    note(s, 20, 75, "VBAT_PROT = entrada B depois da proteção de inversão (alimenta buck, VBATT e as pontes).")

    # buck 5 V
    title(s, 20, 135, "Buck 12 V → 5 V (AP63205, circuito da Fig. 1 da ficha: L 4,7 µH, C1 10 µF, C2 2×22 µF, C3 100 nF, FB direto em VOUT)", 2.0)
    u = k.refs.next("U")
    s.part("Regulator_Switching", "AP63205WU", "U", "AP63205WU-7", "Package_TO_SOT_SMD:TSOT-23-6", 60, 160,
           {"1": "+5V", "2": "VBAT_PROT", "3": "VBAT_PROT", "4": "PGND", "5": "BUCK_SW", "6": "BUCK_BST"}, ref=u)
    C(s, 95, 160, "100n", "BUCK_BST", "BUCK_SW")
    s.part("Device", "L", "L", "4.7uH", "Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm", 120, 160, {"1": "BUCK_SW", "2": "+5V"})
    C(s, 145, 160, "10u/50V", "VBAT_PROT", "PGND", FP_C12)
    C(s, 165, 160, "100n/50V", "VBAT_PROT", "PGND")
    C(s, 185, 160, "22u/10V", "+5V", "PGND", FP_C8)
    C(s, 205, 160, "22u/10V", "+5V", "PGND", FP_C8)
    C(s, 225, 160, "10u/10V", "+5V", "SGND", FP_C8)

    # 5 V de sensores
    title(s, 20, 195, "5 V de sensores: ferrite + TPS2553 (limite ≈350 mA nominal com RILIM 75 kΩ: 314–397 mA pela ficha), LED acende em falha", 2.0)
    s.part("Device", "FerriteBead", "FB", "BLM21 600R/2A", "Inductor_SMD:L_0805_2012Metric", 40, 220, {"1": "+5V", "2": "+5V_F"})
    k.box_symbol("TPS2553DBV", "U", 12.7, left=[("1", "IN", "passive"), ("3", "EN", "input"), ("5", "ILIM", "passive")],
                 right=[("6", "OUT", "passive"), ("4", "FAULT", "open_collector")], bottom=[("2", "GND", "passive")])
    u = k.refs.next("U")
    s.part("OpenEMS", "TPS2553DBV", "U", "TPS2553DBV", "Package_TO_SOT_SMD:SOT-23-6", 85, 220,
           {"1": "+5V_F", "3": "+5V_F", "5": "SENS_ILIM", "6": "+5V_SENS", "4": "SENS_FAULT", "2": "SGND"}, ref=u)
    R(s, 120, 220, "75k 1%", "SENS_ILIM", "SGND")
    C(s, 140, 220, "100n", "+5V_F", "SGND")
    C(s, 160, 220, "10u/10V", "+5V_SENS", "SGND", FP_C8)
    C(s, 180, 220, "100n", "+5V_SENS", "SGND")
    R(s, 205, 220, "1k", "+5V", "SENS_LEDA")
    s.part("Device", "LED_Small", "D", "LED vermelho (falha 5V sens.)", FP_LED, 230, 220, {"1": "SENS_FAULT", "2": "SENS_LEDA"})

    # headers da WeAct P3 / P4, nettie, furos, VREF+
    title(s, 20, 255, "Headers de alimentação da WeAct, ponto único PGND–SGND, VREF+", 2.0)
    s.part("Connector_Generic", "Conn_02x02_Odd_Even", "J", "WeAct P3 (VDD33 x2, GND x2)", "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical", 40, 280,
           {"1": "+3V3", "2": "+3V3", "3": "SGND", "4": "SGND"}, ref="P3")
    s.part("Connector_Generic", "Conn_02x02_Odd_Even", "J", "WeAct P4 (VCC x2, GND x2)", "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical", 90, 280,
           {"1": "+5V", "2": "+5V", "3": "SGND", "4": "SGND"}, ref="P4")
    note(s, 25, 292, "P3/P4 2×2 da WeAct: pinos 1 e 2 = trilho, 3 e 4 = GND (SchDoc). 5 V entra no VCC; o D4 da WeAct bloqueia o USB; S1 aberto; nunca ligar VB.")
    s.part("Device", "NetTie_2", "NT", "NetTie PGND-SGND", "NetTie:NetTie-2_SMD_Pad0.5mm", 150, 280, {"1": "PGND", "2": "SGND"})
    note(s, 135, 292, "Único ponto de junção PGND–SGND: junto à entrada (layout).")
    C(s, 200, 280, "100n", "VREF+", "SGND")
    C(s, 220, 280, "1u", "VREF+", "SGND")
    note(s, 190, 292, "VREF+ só desacoplado: a carrier não o alimenta (confirmar na WeAct se VREF+ = VDDA).")
    for i, (hx, hy) in enumerate([(300, 280), (330, 280), (360, 280), (390, 280)]):
        s.part("Mechanical", "MountingHole", "H", "M3", "MountingHole:MountingHole_3.2mm_M3", hx, hy, {})

    # PWR_FLAGs de todos os trilhos
    title(s, 20, 320, "PWR_FLAG por trilho (fontes externas ao esquemático: conectores, WeAct, buck)", 2.0)
    for i, net in enumerate(["VBAT_RAW_A", "VBAT_RAW_B", "PGND", "SGND", "+5V", "+5V_F", "+5V_SENS", "+3V3", "VBAT_PROT", "VBAT_INJ", "VBAT_IGN",
                             "VBAT_ETB", "VBAT_AUX"]):
        s.rail(net, G(30 + i * 22), G(340), flag=True)


# ================================================================================================
# 2. SAIDAS INJ / IGN / ARM
# ================================================================================================
def sheet_outputs(s):
    title(s, 20, 15, "2. Saídas INJ1–4 / IGN1–4 e chave ARM (OE# dos 74AHCT125)")
    note(s, 20, 20, "Sem ARM, OE# fica alto (pull-up): buffers em alta impedância e saídas em LOW pelos pull-downs. INJ e IGN seguem out_pins.h (PE0/2/4/6, PE9/11/13/15).")

    # ARM
    R(s, 40, 50, "10k", "+5V", "OE_N")
    s.part("Switch", "SW_SPST", "SW", "ARM (jumper/chave)", FP_HDR % 2, 70, 50, {"1": "SGND", "2": "OE_N"})
    R(s, 100, 50, "1k", "+5V", "ARM_LEDA")
    s.part("Device", "LED_Small", "D", "LED vermelho ARM", FP_LED, 125, 50, {"1": "OE_N", "2": "ARM_LEDA"})
    R(s, 160, 50, "100k", "OE_N", "ARM_SENSE")
    R(s, 180, 50, "150k", "ARM_SENSE", "SGND")
    note(s, 150, 65, "ARM_SENSE (PE14, opcional): divisor 100k/150k: PE14 ≈ 2,9 V com OE_N ≈ 4,8 V (não carrega o pull-up de 10 kΩ). Firmware não usa hoje.")

    u1, u2 = k.refs.next("U"), k.refs.next("U")
    # INJ
    conn(s, 395, 95, "J2", "Injetores", FP_MSTB % (5, 5), ["VBAT_INJ", "INJ1_CON", "INJ2_CON", "INJ3_CON", "INJ4_CON"])
    for i in range(4):
        n = i + 1
        y = 100 + i * 45
        PD(s, 30, y, "INJ%d_MCU" % n)
        buf(s, u1, n, 70, y, "INJ%d_MCU" % n, "INJ%d_G" % n)
        R(s, 105, y, "220", "INJ%d_G" % n, "INJ%d_GG" % n)
        PD(s, 125, y, "INJ%d_GG" % n, "PGND")
        s.part("Diode", "1N4148W", "D", "1N4148W", FP_SOD123, 95, y + 16, {"1": "INJ%d_GD" % n, "2": "INJ%d_G" % n})
        R(s, 125, y + 16, "47", "INJ%d_GD" % n, "INJ%d_GG" % n)
        s.part("Transistor_FET", "Q_NMOS_GDS", "Q", "IRLR3110Z", FP_DPAK, 160, y, {"1": "INJ%d_GG" % n, "2": "INJ%d_CON" % n, "3": "PGND"})
        tvs_uni(s, 205, y - 8, "SMAJ58A", FP_SMA, "INJ%d_CON" % n, "INJ%d_TV" % n)
        s.part("Diode", "1N4148W", "D", "1N4148W", FP_SOD123, 245, y - 8, {"1": "INJ%d_GG" % n, "2": "INJ%d_TV" % n})
    buf_pwr(s, u1, 290, 100)
    note(s, 190, 85, "Clamp ativo dreno→gate: TVS 58 V + 1N4148W em série. Gate 220 Ω no desligamento; diodo + 47 Ω em paralelo aceleram só o turn-on. Gate 220 Ω limita a corrente que o 74AHCT125 absorve durante o clamp (ficha SN74AHCT125: IO contínuo máx. ±25 mA). IRLR3110Z (100 V) escolhida; IRLR2905 (55 V) NÃO serve com este clamp de ≈ 70 V.")
    # IGN
    conn(s, 395, 290, "J3", "Ignição", FP_MSTB % (8, 8),
         ["VBAT_IGN", "VBAT_IGN", "PGND", "PGND", "IGN1_CON", "IGN2_CON", "IGN3_CON", "IGN4_CON"])
    for i in range(4):
        n = i + 1
        y = 290 + i * 28
        PD(s, 30, y, "IGN%d_MCU" % n)
        buf(s, u2, n, 70, y, "IGN%d_MCU" % n, "IGN%d_G" % n)
        R(s, 105, y, "33", "IGN%d_G" % n, "IGN%d_CON" % n)
        PD(s, 125, y, "IGN%d_CON" % n, "PGND")
        esd(s, 150, y, "IGN%d_CON" % n, "PGND")
    buf_pwr(s, u2, 290, 290)
    note(s, 20, 280, "IGN: só buffer 74AHCT125 (5 V) → 33 Ω → conector; 10 kΩ para PGND; ESD no conector. Potência fica no ignitor da bobina.")


# ================================================================================================
# 3. ETB + AUXILIARES
# ================================================================================================
def sheet_etb_aux(s):
    title(s, 20, 15, "3. ETB (DRV8871) e auxiliares (bomba, ventoinha, VVT)")
    note(s, 20, 20, "ETB: PWM PE5, IN1 PE7 (abrir), IN2 PE8 (fechar). IN1 = PWM & ABRIR, IN2 = PWM & FECHAR; sem ARM ou PWM baixo = coast.")
    u = k.refs.next("U")
    for i, (mcu, b) in enumerate([("ETB_PWM_MCU", "ETB_PWM_B"), ("ETB_OPEN_MCU", "ETB_OPEN_B"), ("ETB_CLOSE_MCU", "ETB_CLOSE_B")]):
        y = 50 + i * 30
        PD(s, 30, y, mcu)
        buf(s, u, i + 1, 70, y, mcu, b)
    buf(s, u, 4, 70, 140, "SGND", None)
    buf_pwr(s, u, 110, 140)
    # AND 74AHCT1G08 x2
    for i, (b, o) in enumerate([("ETB_OPEN_B", "ETB_IN1"), ("ETB_CLOSE_B", "ETB_IN2")]):
        ua = k.refs.next("U")
        y = 60 + i * 45
        s.part("74xGxx", "74AHCT1G08", "U", "74AHCT1G08", FP_SOT235, 150, y, {"1": "ETB_PWM_B", "2": b, "3": "SGND", "4": o, "5": "+5V"}, ref=ua)
        PD(s, 190, y, o, "PGND")
    # DRV8871
    ud = k.refs.next("U")
    s.part("Driver_Motor", "DRV8871DDA", "U", "DRV8871DDA", "Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm", 250, 70,
           {"1": "PGND", "2": "ETB_IN2", "3": "ETB_IN1", "4": "ETB_ILIM", "5": "VBAT_ETB", "6": "ETB_M+", "7": "PGND", "8": "ETB_M-", "9": "PGND"}, ref=ud)
    R(s, 215, 100, "20k 1%", "ETB_ILIM", "PGND")
    C(s, 290, 100, "100n/50V", "VBAT_ETB", "PGND")
    CP(s, 310, 100, "220u/35V", "VBAT_ETB", "PGND", "Capacitor_SMD:CP_Elec_8x10")
    note(s, 205, 115, "RILIM 20 kΩ → ITRIP ≈ 3,2 A nominal (ITRIP = 64 kV / RILIM; com VILIM 59–69 kV: 2,95–3,45 A). Ficha: mín. 15 kΩ.")
    conn(s, 360, 60, "J4", "Corpo de borboleta (ETB)", FP_MC % (6, 6),
         ["ETB_M+", "ETB_M-", "ETB_TPS1_SIG", "ETB_TPS2_SIG", "+5V_SENS", "SGND"])
    note(s, 340, 85, "J4 também leva os dois potenciômetros do ETB (TPS1 PA2, TPS2 PC5), condicionados na folha 4.")

    # AUXILIARES
    title(s, 20, 175, "Auxiliares: bomba (PE10), ventoinha (PE12) — MOSFET + diodo, relé fora da placa; VVT (PB6/PB7) — MOSFET + Schottky")
    for i, (mcu, drv, nm) in enumerate([("PUMP_MCU", "PUMP_DRV", "BOMBA"), ("FAN_MCU", "FAN_DRV", "VENTOINHA")]):
        y = 205 + i * 40
        R(s, 40, y, "100", mcu, mcu + "_G")
        PD(s, 60, y, mcu + "_G", "PGND")
        s.part("Transistor_FET", "Q_NMOS_GSD", "Q", "AO3400A", FP_SOT23, 90, y, {"1": mcu + "_G", "2": "PGND", "3": drv})
        s.part("Diode", "1N4148W", "D", "1N4148W", FP_SOD123, 130, y, {"1": "VBAT_AUX", "2": drv})
        note(s, 150, y + 3, "bobina do relé " + nm + " entre VBAT_AUX e " + drv)
    for i, (mcu, drv, nm) in enumerate([("VVT_INT_MCU", "VVT_INT", "admissão"), ("VVT_EXH_MCU", "VVT_EXH", "escape")]):
        y = 295 + i * 45
        R(s, 40, y, "100", mcu, mcu + "_G")
        PD(s, 60, y, mcu + "_G", "PGND")
        PD(s, 20, y, mcu, "SGND")
        s.part("Transistor_FET", "Q_NMOS_GSD", "Q", "AO3400A", FP_SOT23, 100, y, {"1": mcu + "_G", "2": "PGND", "3": drv})
        s.part("Device", "D_Schottky", "D", "SS34", FP_SMA, 140, y - 8, {"1": "VBAT_AUX", "2": drv})
        note(s, 160, y + 3, "solenoide VVT " + nm + " entre VBAT_AUX e " + drv)
    note(s, 20, 275, "Entradas PE10/PE12 e PB6/PB7 com pull-down de gate (10 kΩ) e de entrada (10 kΩ): reset = tudo LOW.")
    PD(s, 20, 205, "PUMP_MCU", "SGND")
    PD(s, 20, 245, "FAN_MCU", "SGND")
    conn(s, 360, 250, "J7", "Auxiliares", FP_MSTB % (6, 6), ["VBAT_AUX", "PGND", "PUMP_DRV", "FAN_DRV", "VVT_INT", "VVT_EXH"])


# ================================================================================================
# 4. ENTRADAS: CKP / CMP / FLEX / ANALOGICAS
# ================================================================================================
def sheet_inputs(s):
    title(s, 20, 15, "4. Entradas: CKP (VR MAX9924 ou Hall, por jumper), CMP Hall, flex fuel e analógicas")
    note(s, 20, 20, "J5: CKP_VR+, CKP_VR-, CKP_HALL, CMP_HALL, +5V_SENS, SGND, blindagem (SGND só no lado da ECU).")
    conn(s, 30, 50, "J5", "Roda 60-2 e fase", FP_MC % (7, 7), ["CKP_VR+", "CKP_VR-", "CKP_HALL", "CMP_HALL", "+5V_SENS", "SGND", "SGND"])
    # --- VR: MAX9924 modo A2 ---
    k.box_symbol("MAX9924", "U", 12.7,
                 left=[("1", "IN+", "input"), ("2", "IN-", "input"), ("4", "BIAS", "passive")],
                 right=[("7", "COUT", "open_collector"), ("6", "ZERO_EN", "input"), ("9", "INT_THRS", "input"),
                        ("8", "EXT", "passive"), ("3", "N.C.", "passive")],
                 top=[("10", "VCC", "power_in")], bottom=[("5", "GND", "power_in")])
    um = k.refs.next("U")
    R(s, 60, 50, "10k", "CKP_VR+", "VR_INP")
    R(s, 80, 50, "10k", "CKP_VR-", "VR_INN")
    C(s, 100, 50, "1n", "VR_INP", "VR_INN")
    s.part("OpenEMS", "MAX9924", "U", "MAX9924UAUB+", "Package_SO:MSOP-10_3x3mm_P0.5mm", 150, 60,
           {"1": "VR_INP", "2": "VR_INN", "4": "SGND", "7": "CKP_VR_OUT", "6": "SGND", "9": "SGND", "10": "+5V_SENS", "5": "SGND"},
           nc=("8", "3"), ref=um)
    C(s, 190, 30, "10n", "+5V_SENS", "SGND")
    C(s, 205, 30, "100n", "+5V_SENS", "SGND")
    C(s, 220, 30, "1u", "+5V_SENS", "SGND")
    R(s, 215, 60, "10k", "+3V3", "CKP_VR_OUT")
    note(s, 120, 90, "Modo A2 (ficha, Fig. 3): ZERO_EN, INT_THRS e BIAS em GND, referência interna 2,5 V, EXT aberto. COUT é dreno aberto: pull-up 10 kΩ em 3,3 V.")
    note(s, 120, 95, "Entradas: 10 kΩ em série (limita corrente nos diodos ESD; abs. máx. ±20 mA) + 1 nF diferencial.")

    # --- Hall CKP ---
    esd(s, 60, 125, "CKP_HALL")
    R(s, 85, 125, "4.7k", "+5V_SENS", "CKP_PU")
    s.part("Connector_Generic", "Conn_01x02", "JP", "JP_CKP_PU (fechado = pull-up)", FP_HDR % 2, 110, 125, {"1": "CKP_PU", "2": "CKP_HALL"}, ref="JP2")
    R(s, 140, 125, "1k", "CKP_HALL", "CKP_HF")
    C(s, 160, 125, "100p", "CKP_HF", "SGND")
    us = k.refs.next("U")
    s.part("74xGxx", "74LVC1G17", "U", "74LVC1G17", FP_SOT235, 200, 125, {"2": "CKP_HF", "3": "SGND", "4": "CKP_H_S", "5": "+3V3"}, ref=us)
    C(s, 225, 125, "100n", "+3V3", "SGND")
    # jumper VR/Hall
    s.part("Jumper", "Jumper_3_Open", "JP", "JP_CKP (1-2 VR, 2-3 Hall)", FP_HDR % 3, 270, 90, {"1": "CKP_VR_OUT", "2": "CKP_SEL", "3": "CKP_H_S"}, ref="JP1")
    ux = k.refs.next("U")
    s.part("74xGxx", "74LVC1G86", "U", "74LVC1G86", FP_SOT235, 320, 90, {"1": "CKP_SEL", "2": "CKP_POL", "3": "SGND", "4": "CKP_X", "5": "+3V3"}, ref=ux)
    C(s, 345, 90, "100n", "+3V3", "SGND")
    PD(s, 300, 125, "CKP_POL")
    s.part("Connector_Generic", "Conn_01x02", "JP", "JP_POL (fechado = inverte)", FP_HDR % 2, 325, 125, {"1": "CKP_POL", "2": "+3V3"}, ref="JP3")
    R(s, 375, 90, "1k", "CKP_X", "CKP_MCU")
    note(s, 255, 108, "XOR com JP_POL inverte a borda se a polaridade VR e Hall diferirem (medir no scope).")

    # --- CMP Hall ---
    esd(s, 60, 165, "CMP_HALL")
    R(s, 85, 165, "4.7k", "+5V_SENS", "CMP_HALL")
    R(s, 110, 165, "1k", "CMP_HALL", "CMP_HF")
    C(s, 130, 165, "100p", "CMP_HF", "SGND")
    uc = k.refs.next("U")
    s.part("74xGxx", "74LVC1G17", "U", "74LVC1G17", FP_SOT235, 170, 165, {"2": "CMP_HF", "3": "SGND", "4": "CMP_H_S", "5": "+3V3"}, ref=uc)
    C(s, 195, 165, "100n", "+3V3", "SGND")
    R(s, 225, 165, "1k", "CMP_H_S", "CMP_MCU")

    # --- FLEX ---
    esd(s, 60, 205, "FLEX_IN")
    R(s, 85, 205, "3.3k", "+5V_SENS", "FLEX_IN")
    R(s, 110, 205, "1k", "FLEX_IN", "FLEX_F")
    C(s, 130, 205, "100p", "FLEX_F", "SGND")
    uf = k.refs.next("U")
    s.part("74xGxx", "74LVC1G17", "U", "74LVC1G17", FP_SOT235, 170, 205, {"2": "FLEX_F", "3": "SGND", "4": "FLEX_MCU", "5": "+3V3"}, ref=uf)
    C(s, 195, 205, "100n", "+3V3", "SGND")
    note(s, 40, 220, "Flex fuel (PB5): pull-up 3,3 kΩ em 5 V de sensores → Schmitt em 3,3 V (entrada tolerante a 5 V).")

    # --- Analogicas ---
    title(s, 20, 240, "Analógicas (ADC 0–3,3 V): divisor 10k/20k + 1k + C + BAT54S para 3V3/SGND", 2.0)
    chans = [("MAP", "10n"), ("TPS", "39n"), ("APP1", "39n"), ("APP2", "39n"),
             ("FUEL", "220n"), ("OIL", "220n"), ("ETB_TPS1", "39n"), ("ETB_TPS2", "39n")]
    for i, (nm, cv) in enumerate(chans):
        x = 30 + (i % 4) * 85
        y = 265 + (i // 4) * 40
        R(s, x, y, "10k", nm + "_SIG", nm + "_N1")
        R(s, x + 15, y, "20k", nm + "_N1", "SGND")
        R(s, x + 30, y, "1k", nm + "_N1", nm + "_MCU")
        C(s, x + 45, y, cv, nm + "_MCU", "SGND")
        clamp(s, x + 65, y, nm + "_MCU")
    y = 355
    for i, nm in enumerate(["CLT", "IAT"]):
        x = 30 + i * 85
        R(s, x, y, "2.49k", "+3V3", nm + "_SIG")
        R(s, x + 15, y, "1k", nm + "_SIG", nm + "_MCU")
        C(s, x + 30, y, "100n", nm + "_MCU", "SGND")
        clamp(s, x + 50, y, nm + "_MCU")
    x = 200
    R(s, x, y, "100k", "VBAT_PROT", "VBATT_N1")
    R(s, x + 15, y, "22k", "VBATT_N1", "SGND")
    R(s, x + 30, y, "1k", "VBATT_N1", "VBATT_MCU")
    C(s, x + 45, y, "100n", "VBATT_MCU", "SGND")
    clamp(s, x + 65, y, "VBATT_MCU")
    note(s, 30, 385, "KNOCK (PA5): DNP, estágio não montado (EMS_KNOCK_HW_PRESENT=0). Pino fica NC no header.")
    conn(s, 355, 355, "J6", "Sensores", FP_MC % (13, 13),
         ["MAP_SIG", "TPS_SIG", "APP1_SIG", "APP2_SIG", "CLT_SIG", "IAT_SIG", "FUEL_SIG", "OIL_SIG", "FLEX_IN",
          "+5V_SENS", "+5V_SENS", "SGND", "SGND"])


# ================================================================================================
# 5. CAN, SERVICO, HEADERS DA WEACT
# ================================================================================================
P1 = {1: "APP1_MCU", 2: "OIL_MCU", 3: "APP2_MCU", 4: "VBATT_MCU", 5: "SGND", 6: "VREF+", 7: "CKP_MCU", 8: "CMP_MCU",
      9: "ETB_TPS1_MCU", 10: "MAP_MCU", 11: "TPS_MCU", 15: "FUEL_MCU", 16: "ETB_TPS2_MCU", 17: "CLT_MCU", 18: "IAT_MCU",
      19: "LED_STATUS", 20: "ETB_OPEN_MCU", 21: "ETB_CLOSE_MCU", 22: "IGN1_MCU", 23: "PUMP_MCU", 24: "IGN2_MCU",
      25: "FAN_MCU", 26: "IGN3_MCU", 27: "ARM_SENSE", 28: "IGN4_MCU", 30: "SGND"}
P2 = {2: "SGND", 3: "INJ4_MCU", 5: "INJ3_MCU", 6: "ETB_PWM_MCU", 7: "INJ2_MCU", 9: "INJ1_MCU", 10: "SGND",
      11: "CAN_RX_MCU", 12: "CAN_TX_MCU", 13: "VVT_INT_MCU", 14: "VVT_EXH_MCU", 16: "FLEX_MCU", 31: "UART_RX", 34: "UART_TX"}


def sheet_comms(s):
    title(s, 20, 15, "5. CAN (TJA1051T/3), serviço (UART) e headers P1/P2 da WeAct")
    # CAN
    note(s, 20, 20, "CAN 500 kbps: PB8 RX, PB9 TX. TJA1051T/3: VCC 5 V, VIO 3,3 V, S em GND (modo normal).")
    ut = k.refs.next("U")
    s.part("Interface_CAN_LIN", "TJA1051T-3", "U", "TJA1051T/3", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", 60, 55,
           {"1": "CAN_TX_MCU", "2": "SGND", "3": "+5V", "4": "CAN_RX_MCU", "5": "+3V3", "6": "CANL_T", "7": "CANH_T", "8": "CAN_S"}, ref=ut)
    PD(s, 100, 55, "CAN_S")
    C(s, 120, 55, "100n", "+5V", "SGND")
    C(s, 140, 55, "100n", "+3V3", "SGND")
    s.part("Device", "L_Coupled_1423", "L", "choke CAN (ACM2012-900-2P)", "Inductor_SMD:L_CommonModeChoke_Coilank_ACM2012", 190, 55,
           {"1": "CANH_T", "4": "CAN_H", "2": "CANL_T", "3": "CAN_L"})
    s.part("Diode", "SM712_SOT23", "D", "SM712 (TVS CAN)", FP_SOT23, 250, 55, {"1": "CAN_H", "2": "CAN_L", "3": "SGND"})
    R(s, 285, 55, "120", "CAN_H", "CAN_TERM")
    s.part("Connector_Generic", "Conn_01x02", "JP", "JP_TERM (fechado = 120 Ω)", FP_HDR % 2, 310, 55, {"1": "CAN_TERM", "2": "CAN_L"}, ref="JP4")
    conn(s, 365, 55, "J8", "CAN e WBO2", FP_MSTB % (4, 4), ["CAN_H", "CAN_L", "VBAT_AUX", "PGND"])
    note(s, 340, 75, "WBO2 só por CAN; VBAT_AUX alimenta o sensor (3 A de pico).")

    # servico
    conn(s, 60, 120, "J9", "Serviço (UART)", FP_HDR % 4, ["+3V3", "UART_TX", "UART_RX", "SGND"])
    R(s, 110, 120, "1k", "LED_STATUS", "LED_A")
    s.part("Device", "LED_Small", "D", "LED verde status", FP_LED, 135, 120, {"1": "SGND", "2": "LED_A"})
    note(s, 100, 135, "LED_STATUS (PB2). UART: PA9 TX, PA10 RX.")

    # headers
    title(s, 20, 160, "Headers P1/P2 da WeAct (2×20): pino a pino conforme docs/hw/weact_h562_schematic.md", 2.0)
    for tag, tbl, x in (("P1", P1, 100), ("P2", P2, 300)):
        nets = {}
        nc = []
        for pin in range(1, 41):
            if pin in tbl:
                nets[str(pin)] = tbl[pin]
            else:
                nc.append(str(pin))
        s.part("Connector_Generic", "Conn_02x20_Odd_Even", "J", "WeAct " + tag, "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical",
               x, 200, nets, ref=tag, nc=tuple(nc))
    note(s, 215, 215, "PA5 (KNOCK) NC; PB11, PA13, PA14, NRST não saem em P1/P2.")
    note(s, 215, 222, "P2-1 = VB (backup RTC): NÃO LIGAR. P2: PC8..PC12, PD2, PA8 = SD da WeAct: NC.")


# ================================================================================================
def build():
    root = U()
    sheets = [("Alimentação", "power", sheet_power), ("Saídas INJ/IGN", "outputs", sheet_outputs),
              ("ETB e auxiliares", "etb_aux", sheet_etb_aux), ("Entradas", "inputs", sheet_inputs),
              ("CAN e headers", "comms", sheet_comms)]
    blocks, errs = [], []
    r = Sheet(PROJECT, root, root, "OpenEMS interface VGT6", 1, paper="A3", is_root=True)
    for i, (title_, fname, fn) in enumerate(sheets):
        uid = U()
        s = Sheet(PROJECT, root, uid, title_, i + 2, paper="A2")
        fn(s)
        s.write(os.path.join(OUT, fname + ".kicad_sch"))
        errs += ["[%s] %s" % (fname, e) for e in s.errors]
        x, y = 25.4 + (i % 3) * 101.6, 76.2 + (i // 3) * 50.8
        blocks.append(["sheet", ["at", x, y], ["size", 76.2, 12.7], ["exclude_from_sim", "no"], ["in_bom", "yes"],
                       ["on_board", "yes"], ["dnp", "no"], ["fields_autoplaced", "yes"],
                       ["stroke", ["width", 0.1524], ["type", "solid"]], ["fill", ["color", 0, 0, 0, 0.0]],
                       ["uuid", Q(uid)],
                       ["property", Q("Sheetname"), Q(title_), ["at", x, y - 1, 0],
                        ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left", "bottom"]]],
                       ["property", Q("Sheetfile"), Q(fname + ".kicad_sch"), ["at", x, y + 13.7, 0],
                        ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left", "top"]]],
                       ["instances", ["project", Q(PROJECT), ["path", Q("/" + root), ["page", Q(str(i + 2))]]]]])
    r.text("OpenEMS: placa de interface para a WeAct H562 VGT6 (carrier de potência)", 25.4, 20.0, 3.0)
    r.text("PCB de 4 camadas: L1 sinal+potência, L2 plano PGND/SGND (junção única), L3 potência, L4 sinal. Veja docs/hw/interface_vgt6.md", 25.4, 30.0, 1.6)
    r.write(os.path.join(OUT, PROJECT + ".kicad_sch"), children=blocks)
    with open(os.path.join(OUT, PROJECT + ".kicad_pro"), "w") as f:
        f.write('{\n  "meta": {"filename": "%s.kicad_pro", "version": 3},\n  "sheets": []\n}\n' % PROJECT)
    k.write_syslib(os.path.join(OUT, "OpenEMS.kicad_sym"), ["MAX9924", "TPS2553DBV"])
    with open(os.path.join(OUT, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n  (version 7)\n  (lib (name "OpenEMS")(type "KiCad")(uri "${KIPRJMOD}/OpenEMS.kicad_sym")(options "")(descr "simbolos da carrier"))\n)\n')
    for e in errs:
        print("ERRO:", e)
    print("erros do gerador:", len(errs))


if __name__ == "__main__":
    build()
