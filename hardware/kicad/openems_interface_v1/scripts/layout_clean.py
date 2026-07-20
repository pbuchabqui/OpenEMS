#!/usr/bin/env python3
"""Clean PCB layout: organised placement + local bridges only (no spaghetti).

Philosophy
----------
Auto-routing hundreds of stub tracks made the board unreadable. This script
rebuilds a *placement-first* scaffold:

  • Clear zones (connectors / WeAct / power / TLE)
  • Pad nets from pinmap (authoritative)
  • Only *local* copper: pin-pair shorts on connectors + TLE OUT A+B
  • PGND zones (fill in Pcbnew)
  • Zone labels — route the rest by hand in KiCad

Run:  python3 scripts/layout_clean.py
  or: bash scripts/build_all.sh
"""
from __future__ import annotations

import re
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRETTY = ROOT / "libs" / "OpenEMS.pretty"
PCB = ROOT / "openems_interface_v1.kicad_pcb"
PCB_VER = "20221018"
W, H = 130.0, 100.0

# ---------------------------------------------------------------------------
# Pin maps (docs/hw/netlist_v1.md)
# ---------------------------------------------------------------------------
J2 = {
    1: "VBAT_RAW", 2: "VBAT_RAW",
    3: "PGND", 4: "PGND", 5: "PGND",
    6: "INJ1", 7: "INJ2", 8: "INJ3", 9: "INJ4",
    10: "IGN1", 11: "IGN2", 12: "IGN3", 13: "IGN4",
    14: "VVT_EXH", 15: "VVT_INT",
    16: "PUMP_RLY", 17: "FAN_RLY", 18: "MAIN_RLY",
    19: "ETB_MOTOR_P", 20: "ETB_MOTOR_P",
    21: "ETB_MOTOR_N", 22: "ETB_MOTOR_N",
}
J1 = {
    1: "CKP_P", 2: "CKP_N", 3: "SHIELD_GND",
    4: "CMP_SIG", 5: "CMP_5V", 6: "CMP_GND",
    7: "MAP", 8: "CLT", 9: "IAT",
    10: "APP1", 11: "APP2",
    12: "FUEL_PRESS", 13: "OIL_PRESS",
    14: "ETB_TPS1", 15: "ETB_TPS2",
    16: "+5V_SENS_A", 17: "+5V_SENS_B",
    18: "SGND", 19: "SGND",
    20: "CANH", 21: "CANL", 22: "SHIELD_GND",
    23: "FLEX_12V", 24: "FLEX_SIG", 25: "FLEX_GND",
    26: "KNOCK_SIG", 27: "SHIELD_GND",
    28: "CMP2_SIG", 29: "CMP2_5V", 30: "CMP2_GND",
    31: "TPS_INDEP",
}
J3 = {  # WeAct P1 critical
    1: "MCU_PC0", 2: "MCU_PC1", 3: "MCU_PC2", 4: "MCU_PC3",
    5: "AGND", 6: "VREF_P",
    7: "MCU_PA0", 8: "MCU_PA1",
    9: "MCU_PA2", 10: "MCU_PA3", 11: "MCU_PA4", 12: "MCU_PA5",
    15: "MCU_PC4", 16: "MCU_PC5", 17: "MCU_PB0", 18: "MCU_PB1",
    20: "MCU_PE7", 21: "MCU_PE8", 22: "MCU_PE9",
    23: "MCU_PE10", 24: "MCU_PE11", 25: "MCU_PE12", 26: "MCU_PE13",
    29: "MCU_PB12", 30: "MCU_PB13", 31: "MCU_PB14", 32: "MCU_PB15",
}
J4 = {  # WeAct P2 critical
    2: "MCU_PE6", 3: "MCU_PE4", 4: "MCU_PE5",
    5: "MCU_PE2", 6: "MCU_PE3", 7: "MCU_PE0",
    9: "MCU_PB8", 10: "MCU_PB9",
    11: "MCU_PB6", 12: "MCU_PB7",
    28: "MCU_PA11", 30: "MCU_PA12",
}
NETTIE = {1: "PGND", 2: "SGND", 3: "AGND", 4: "SHIELD_GND"}
TLE = {
    3: "MCU_PB12", 4: "MCU_PB14", 5: "MCU_PB15",
    6: "AGND", 7: "MCU_PB13", 8: "+3V3",
    9: "+5V_SENS_A", 10: "+5V_SENS_B",
    20: "+3V3", 21: "MCU_PA0",
    24: "MCU_PE1", 25: "PGND", 27: "MCU_PE3",
    28: "MCU_PE0", 29: "MCU_PE2", 30: "MCU_PE4", 31: "MCU_PE6",
    32: "MCU_PE9", 33: "MCU_PE11", 34: "MCU_PE13", 35: "MCU_PE15",
    36: "MCU_PE10", 37: "MCU_PE12", 38: "MCU_PB6", 39: "MCU_PB7",
    43: "MCU_PB8", 44: "MCU_PB9", 46: "CANH", 47: "CANL",
    50: "PGND", 51: "CKP_N", 52: "CKP_P",
    54: "VBAT", 55: "MAIN_RLY",
    59: "INJ1", 60: "INJ1", 61: "INJ2", 62: "INJ2",
    63: "INJ3", 64: "INJ3", 65: "INJ4", 66: "INJ4",
    67: "FAN_RLY", 68: "PUMP_RLY", 75: "PGND",
    83: "VVT_EXH", 84: "VVT_EXH", 85: "VVT_EXH",
    87: "VBAT", 90: "VBAT",
    92: "VVT_INT", 93: "VVT_INT", 94: "VVT_INT",
    96: "IGN1", 97: "IGN2", 98: "IGN3", 99: "IGN4", 100: "AGND",
}
Q1 = {1: "VBAT_RAW", 2: "VBAT_RAW", 3: "VBAT"}
F1 = {1: "VBAT", 2: "VBAT"}
D1 = {1: "PGND", 2: "VBAT"}
C1 = {1: "VBAT", 2: "PGND"}
C2 = {1: "VBAT", 2: "PGND"}
U1 = {1: "PGND", 2: "+5V_MAIN", 3: "VBAT", 4: "+5V_MAIN", 5: "VBAT", 6: "+5V_MAIN"}
U2 = {1: "+5V_MAIN", 2: "AGND", 3: "+5V_MAIN", 4: "AGND", 5: "+3V3"}
FB1 = {1: "+3V3", 2: "VDDA"}
C3 = {1: "VDDA", 2: "AGND"}
C4 = {1: "+3V3", 2: "AGND"}


def uid() -> str:
    return str(uuid.uuid4())


def load_mod(name: str) -> str:
    return (PRETTY / f"{name}.kicad_mod").read_text(encoding="utf-8")


def embed(mod_text: str, lib_id: str, ref: str, at: tuple[float, float], rot: int = 0) -> str:
    m = re.match(r'\(footprint\s+"[^"]+"\s*(.*)\)$', mod_text.strip(), re.S)
    if not m:
        m = re.match(r"\(footprint\s+(\S+)\s*(.*)\)$", mod_text.strip(), re.S)
        if not m:
            raise ValueError(lib_id)
        body = m.group(2)
    else:
        body = m.group(1)
    body = re.sub(r"^\(version [^)]+\)\s*", "", body)
    body = re.sub(r"^\(generator [^)]+\)\s*", "", body)
    body = body.replace('"REF**"', f'"{ref}"')
    body = re.sub(r"\(fp_text reference REF\*\*", f'(fp_text reference "{ref}"', body)
    body = re.sub(r'\(fp_text reference "REF\*\*"', f'(fp_text reference "{ref}"', body)
    return (
        f'(footprint "{lib_id}" (layer "F.Cu")\n'
        f"    (tstamp {uid()})\n"
        f"    (at {at[0]} {at[1]} {rot})\n"
        f'    (path "/{uid()}")\n'
        f"{body}\n"
        f")"
    )


def assign_nets(block: str, pinmap: dict[int, str], net_code: dict[str, int]) -> str:
    out: list[str] = []
    i = 0
    while i < len(block):
        m = re.match(r'\(pad ("?\d+"?)\s', block[i:])
        if not m:
            out.append(block[i])
            i += 1
            continue
        raw = m.group(1).strip('"')
        if not raw.isdigit():
            out.append(block[i])
            i += 1
            continue
        pn = int(raw)
        start = i
        depth = 0
        j = i
        while j < len(block):
            if block[j] == "(":
                depth += 1
            elif block[j] == ")":
                depth -= 1
                if depth == 0:
                    j += 1
                    break
            j += 1
        pad = block[start:j]
        if pn in pinmap and pinmap[pn] in net_code:
            netn = pinmap[pn]
            code = net_code[netn]
            if re.search(r"\(net \d+", pad):
                pad = re.sub(r'\(net \d+ "[^"]*"\)', f'(net {code} "{netn}")', pad)
            else:
                pad = pad[:-1] + f' (net {code} "{netn}")' + ")"
        out.append(pad)
        i = j
    return "".join(out)


def pad_xy(block: str, at: tuple[float, float], pin: int) -> tuple[float, float] | None:
    m = re.search(rf'\(pad "?{pin}"? \w+ \w+ \(at ([-\d.]+) ([-\d.]+)', block)
    if not m:
        return None
    return at[0] + float(m.group(1)), at[1] + float(m.group(2))


def segment(a, b, w, layer, net):
    return (
        f"  (segment (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) "
        f'(width {w}) (layer "{layer}") (net {net}) (tstamp {uid()}))\n'
    )


def mounting_hole(ref: str, at: tuple[float, float], drill: float = 3.2) -> str:
    return f"""  (footprint "MountingHole:{drill}mm" (layer "F.Cu")
    (tstamp {uid()})
    (at {at[0]} {at[1]})
    (attr through_hole exclude_from_pos_files exclude_from_bom)
    (fp_text reference "{ref}" (at 0 {-drill - 1}) (layer "F.SilkS")
      (effects (font (size 0.8 0.8) (thickness 0.12)))
      (tstamp {uid()})
    )
    (fp_text value "MH" (at 0 {drill + 1}) (layer "F.Fab")
      (effects (font (size 0.8 0.8) (thickness 0.12)))
      (tstamp {uid()})
    )
    (fp_circle (center 0 0) (end {drill / 2 + 0.4} 0)
      (stroke (width 0.12) (type solid)) (fill none) (layer "F.CrtYd") (tstamp {uid()}))
    (pad "" np_thru_hole circle (at 0 0) (size {drill} {drill}) (drill {drill}) (layers "*.Cu" "*.Mask") (tstamp {uid()}))
  )
"""


def zone_rect(net: int, name: str, layer: str, x0, y0, x1, y1) -> str:
    return f"""  (zone (net {net}) (net_name "{name}") (layer "{layer}") (tstamp {uid()}) (hatch edge 0.5)
    (connect_pads (clearance 0.25))
    (min_thickness 0.25)
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon
      (pts
        (xy {x0} {y0})
        (xy {x1} {y0})
        (xy {x1} {y1})
        (xy {x0} {y1})
      )
    )
  )
"""


def label(txt: str, at: tuple[float, float], size: float = 1.2) -> str:
    return f"""  (gr_text "{txt}" (at {at[0]} {at[1]} 0) (layer "Cmts.User") (tstamp {uid()})
    (effects (font (size {size} {size}) (thickness 0.15)))
  )
"""


def box(x0, y0, x1, y1, layer="Dwgs.User") -> str:
    lines = []
    for a, b in [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]:
        lines.append(
            f"  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]})\n"
            f'    (stroke (width 0.15) (type dash)) (layer "{layer}") (tstamp {uid()}))'
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    # ---- collect all nets ----
    maps = [J1, J2, J3, J4, NETTIE, TLE, Q1, F1, D1, C1, C2, U1, U2, FB1, C3, C4]
    all_nets: set[str] = set()
    for m in maps:
        all_nets |= set(m.values())
    all_nets |= {"", "VBAT_RAW", "VBAT", "+5V_MAIN", "+3V3", "VDDA", "VREF_P"}

    net_code: dict[str, int] = {"": 0}
    net_lines = ['  (net 0 "")']
    for i, n in enumerate(sorted(n for n in all_nets if n), 1):
        net_code[n] = i
        net_lines.append(f'  (net {i} "{n}")')

    # ---- placement (clean zones) ----
    # Bottom edge connectors
    # J1 rusEFI origin = center; pad1 ~ (-22,-4) → place so pad row near y=88
    j1_at = (35.0, 88.0)
    j2_at = (85.0, 85.0)  # our 23 origin = pad1

    # WeAct center-top
    weact_sw = (45.69, 8.0)
    j3_at = (weact_sw[0] - 2.54, weact_sw[1] + 5.0)
    j4_at = (weact_sw[0] + 38.62 + 2.54, weact_sw[1] + 5.0)
    holes = [
        (weact_sw[0] + 2.80, weact_sw[1] + 2.80),
        (weact_sw[0] + 33.28, weact_sw[1] + 2.80),
        (weact_sw[0] + 2.80, weact_sw[1] + 66.30),
        (weact_sw[0] + 33.28, weact_sw[1] + 66.30),
    ]
    nettie_at = (weact_sw[0] - 10.0, weact_sw[1] + 35.0)

    # TLE right of WeAct, above connectors
    u3_at = (100.0, 45.0)

    # Power chain: neat row left of TLE, mid board
    #   Q1  F1  C1  C2     then U1 buck / U2 LDO below
    q1_at = (58.0, 72.0)
    f1_at = (66.0, 72.0)
    d1_at = (66.0, 78.0)
    c1_at = (74.0, 72.0)
    c2_at = (80.0, 72.0)
    u1_at = (58.0, 62.0)
    u2_at = (50.0, 62.0)
    fb1_at = (50.0, 55.0)
    c3_at = (45.0, 55.0)
    c4_at = (45.0, 62.0)

    parts: list[tuple[str, str, str, tuple[float, float], int, dict]] = [
        ("rusEFI_AMPSEAL_35_RA_776180", "OpenEMS:rusEFI_AMPSEAL_35_RA_776180", "J1", j1_at, 0, J1),
        ("TE_770669_AMPSEAL_23_RA", "OpenEMS:TE_770669_AMPSEAL_23_RA", "J2", j2_at, 0, J2),
        ("WeAct_PinHeader_2x25_P2.54mm", "OpenEMS:WeAct_PinHeader_2x25_P2.54mm", "J3", j3_at, 0, J3),
        ("WeAct_PinHeader_2x25_P2.54mm", "OpenEMS:WeAct_PinHeader_2x25_P2.54mm", "J4", j4_at, 0, J4),
        ("NetTie-4_THT_Pad1.0mm", "OpenEMS:NetTie-4_THT_Pad1.0mm", "NT1", nettie_at, 0, NETTIE),
        ("LQFP-100_14x14mm_P0.5mm", "OpenEMS:LQFP-100_14x14mm_P0.5mm", "U3", u3_at, 0, TLE),
        ("SOT-23", "OpenEMS:SOT-23", "Q1", q1_at, 0, Q1),
        ("Fuse_1206_3216Metric", "OpenEMS:Fuse_1206_3216Metric", "F1", f1_at, 0, F1),
        ("D_SMB", "OpenEMS:D_SMB", "D1", d1_at, 90, D1),
        ("CP_Elec_6.3x5.8", "OpenEMS:CP_Elec_6.3x5.8", "C1", c1_at, 0, C1),
        ("C_0805_2012Metric", "OpenEMS:C_0805_2012Metric", "C2", c2_at, 0, C2),
        ("SOT-23-6", "OpenEMS:SOT-23-6", "U1", u1_at, 0, U1),
        ("SOT-23-5", "OpenEMS:SOT-23-5", "U2", u2_at, 0, U2),
        ("L_0805_2012Metric", "OpenEMS:L_0805_2012Metric", "FB1", fb1_at, 0, FB1),
        ("C_0805_2012Metric", "OpenEMS:C_0805_2012Metric", "C3", c3_at, 0, C3),
        ("C_0805_2012Metric", "OpenEMS:C_0805_2012Metric", "C4", c4_at, 0, C4),
    ]

    fp_blocks: dict[str, tuple[str, tuple[float, float]]] = {}
    fp_sexprs = []
    for mod, lib, ref, at, rot, pmap in parts:
        block = embed(load_mod(mod), lib, ref, at, rot)
        block = assign_nets(block, pmap, net_code)
        fp_sexprs.append(block)
        fp_blocks[ref] = (block, at)

    for i, (hx, hy) in enumerate(holes, 1):
        fp_sexprs.append(mounting_hole(f"H{i}", (hx, hy)))

    # ---- local bridges only ----
    routes = []
    n_vbat_raw = net_code["VBAT_RAW"]
    n_pgnd = net_code["PGND"]
    n_vbat = net_code["VBAT"]
    n_etbp = net_code["ETB_MOTOR_P"]
    n_etbn = net_code["ETB_MOTOR_N"]

    def p(ref: str, pin: int):
        return pad_xy(fp_blocks[ref][0], fp_blocks[ref][1], pin)

    # J2 pair shorts
    for a, b, net in [
        (1, 2, n_vbat_raw),
        (3, 4, n_pgnd),
        (4, 5, n_pgnd),
        (19, 20, n_etbp),
        (21, 22, n_etbn),
    ]:
        pa, pb = p("J2", a), p("J2", b)
        if pa and pb:
            routes.append(segment(pa, pb, 1.2 if net != n_etbp and net != n_etbn else 1.5, "F.Cu", net))

    # Power chain local: Q1 drain → F1 → C1 (only if close)
    q1d, f1a, f1b, c1p = p("Q1", 3), p("F1", 1), p("F1", 2), p("C1", 1)
    if q1d and f1a:
        routes.append(segment(q1d, f1a, 1.2, "F.Cu", n_vbat))
    if f1b and c1p:
        routes.append(segment(f1b, c1p, 1.2, "F.Cu", n_vbat))
    # C1 → C2
    c2p = p("C2", 1)
    if c1p and c2p:
        routes.append(segment(c1p, c2p, 0.8, "F.Cu", n_vbat))
    # C1 → TLE BAT (54)
    tbat = p("U3", 54)
    if c1p and tbat:
        routes.append(segment(c1p, tbat, 1.0, "F.Cu", n_vbat))
    # BATPA/B short to BAT
    for pin in (87, 90):
        pt = p("U3", pin)
        if pt and tbat:
            routes.append(segment(pt, tbat, 0.8, "F.Cu", n_vbat))

    # TLE OUT A+B / A+B+C local shorts only
    for a, b in ((59, 60), (61, 62), (63, 64), (65, 66)):
        pa, pb = p("U3", a), p("U3", b)
        if pa and pb:
            routes.append(segment(pa, pb, 0.6, "F.Cu", net_code[TLE[a]]))
    for group in ((83, 84, 85), (92, 93, 94)):
        pts = [p("U3", g) for g in group]
        pts = [x for x in pts if x]
        for i in range(len(pts) - 1):
            routes.append(segment(pts[i], pts[i + 1], 0.6, "F.Cu", net_code[TLE[group[0]]]))

    # SPI straps local: pin 6 AGND and 8 +3V3 already netted — tiny via stubs ok
    for pin, net in ((6, "AGND"), (8, "+3V3"), (20, "+3V3"), (25, "PGND"), (50, "PGND"), (75, "PGND"), (100, "AGND")):
        pt = p("U3", pin)
        if pt and net in net_code:
            # thermal via next to pad (0.6mm offset)
            routes.append(
                f'  (via (at {pt[0] + 1.2:.3f} {pt[1]:.3f}) (size 0.7) (drill 0.35) '
                f'(layers "F.Cu" "B.Cu") (net {net_code[net]}) (tstamp {uid()}))\n'
            )

    # LDO local: U1 pin2 → U2 pin1; U2 out → FB1
    u1o, u2i, u2o = p("U1", 2), p("U2", 1), p("U2", 5)
    fb1a, fb1b = p("FB1", 1), p("FB1", 2)
    if u1o and u2i:
        routes.append(segment(u1o, u2i, 0.6, "F.Cu", net_code["+5V_MAIN"]))
    if u2o and fb1a:
        routes.append(segment(u2o, fb1a, 0.5, "F.Cu", net_code["+3V3"]))
    if u2o:
        t20 = p("U3", 20)
        if t20:
            routes.append(segment(u2o, t20, 0.5, "F.Cu", net_code["+3V3"]))

    # ---- graphics: zones + zone labels ----
    graphics = []
    # outline
    for a, b in [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]:
        graphics.append(
            f"  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]})\n"
            f'    (stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (tstamp {uid()}))'
        )
    # WeAct keepout
    wx0, wy0 = weact_sw
    graphics.append(box(wx0, wy0, wx0 + 38.62, wy0 + 69.10))
    graphics.append(label("WeAct H562\\nkeepout", (wx0 + 12, wy0 + 32), 1.3))
    # Zone labels
    graphics.append(label("J1 AMPSEAL 35 (signals)", (20, 96), 1.1))
    graphics.append(label("J2 AMPSEAL 23 (power)", (85, 96), 1.1))
    graphics.append(label("POWER\\nQ1 F1 C bulk\\nbuck + LDO", (52, 80), 1.1))
    graphics.append(label("U3 TLE8888", (100, 32), 1.2))
    graphics.append(label("NT1 star GND", (nettie_at[0] - 5, nettie_at[1] - 4), 1.0))
    graphics.append(
        label(
            "CLEAN LAYOUT — only local bridges + PGND pours\\n"
            "Route remaining nets by hand in Pcbnew\\n"
            "Edit → Fill All Zones  |  Inspect → DRC\\n"
            "NAO fabricar sem revisao pin1 / FET / DRC",
            (65, 20),
            1.15,
        )
    )

    # PGND zones F+B
    zones = zone_rect(net_code["PGND"], "PGND", "F.Cu", 1, 1, 129, 99)
    zones += zone_rect(net_code["PGND"], "PGND", "B.Cu", 1, 1, 129, 99)

    # ---- assemble PCB ----
    pcb = f"""(kicad_pcb (version {PCB_VER}) (generator openems_layout_clean)

  (general
    (thickness 1.6)
  )

  (paper "A4")
  (title_block
    (title "OpenEMS Interface Board v1")
    (date "2026-07-20")
    (rev "v1-clean")
    (company "OpenEMS")
    (comment 1 "Placement-first scaffold. Local bridges only. Fill zones + hand-route.")
  )

  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive")
    (33 "F.Adhes" user "F.Adhesive")
    (34 "B.Paste" user)
    (35 "F.Paste" user)
    (36 "B.SilkS" user "B.Silkscreen")
    (37 "F.SilkS" user "F.Silkscreen")
    (38 "B.Mask" user)
    (39 "F.Mask" user)
    (40 "Dwgs.User" user "User.Drawings")
    (41 "Cmts.User" user "User.Comments")
    (42 "Eco1.User" user "User.Eco1")
    (43 "Eco2.User" user "User.Eco2")
    (44 "Edge.Cuts" user)
    (45 "Margin" user)
    (46 "B.CrtYd" user "B.Courtyard")
    (47 "F.CrtYd" user "F.Courtyard")
    (48 "B.Fab" user)
    (49 "F.Fab" user)
  )

  (setup
    (pad_to_mask_clearance 0)
    (pcbplotparams
      (layerselection 0x00010fc_ffffffff)
      (plot_on_all_layers_selection 0x0000000_00000000)
      (disableapertmacros false)
      (usegerberextensions false)
      (usegerberattributes true)
      (usegerberadvancedattributes true)
      (creategerberjobfile true)
      (dashed_line_dash_ratio 12.000000)
      (dashed_line_gap_ratio 3.000000)
      (svgprecision 4)
      (plotframeref false)
      (viasonmask false)
      (mode 1)
      (useauxorigin false)
      (hpglpennumber 1)
      (hpglpenspeed 20)
      (hpglpendiameter 15.000000)
      (dxfpolygonmode true)
      (dxfimperialunits true)
      (dxfusepcbnewfont true)
      (psnegative false)
      (psa4output false)
      (plotreference true)
      (plotvalue true)
      (plotinvisibletext false)
      (sketchpadsonfab false)
      (subtractmaskfromsilk false)
      (outputformat 1)
      (mirror false)
      (drillshape 1)
      (scaleselection 1)
      (outputdirectory "")
    )
  )

{chr(10).join(net_lines)}

{chr(10).join(graphics)}

{zones}

{chr(10).join(fp_sexprs)}

{"".join(routes)}
)
"""
    PCB.write_text(pcb, encoding="utf-8")
    nseg = pcb.count("(segment ")
    print(f"Clean layout → {PCB}")
    print(f"  footprints: {len(parts) + 4}  segments: {nseg} (local only)")
    print("  Open Pcbnew → Home → Edit → Fill All Zones")


if __name__ == "__main__":
    main()
