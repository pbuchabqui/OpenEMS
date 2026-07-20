#!/usr/bin/env python3
"""Place TLE8888 + input power chain on the PCB and wire critical nets.

Placement (board 130×100, connectors on bottom edge):
  U3  TLE8888 LQFP-100     @ (92, 52)   — power/TLE zone
  Q1  P-FET SOT-23         @ (62, 70)   — reverse polarity
  F1  Fuse 1206            @ (70, 70)
  D1  TVS SMB              @ (70, 76)
  C1  bulk 100u            @ (78, 70)
  C2  100n                 @ (82, 70)
  U1  buck SOT-23-6        @ (62, 58)

Nets follow tle8888_pinout.md + netlist_v1.md (subset on copper).
"""
from __future__ import annotations

import re
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRETTY = ROOT / "libs" / "OpenEMS.pretty"
PCB = ROOT / "openems_interface_v1.kicad_pcb"

# Key TLE pins → nets (OpenEMS v1)
TLE_NETS = {
    3: "MCU_PB12",  # CSN — stub only until WeAct routed
    4: "MCU_PB14",  # SDO
    5: "MCU_PB15",  # SIP
    6: "AGND",  # SIN strap
    7: "MCU_PB13",  # FCLP
    8: "+3V3",  # FCLN strap
    9: "+5V_SENS_A",
    10: "+5V_SENS_B",
    20: "+3V3",  # VDDIO
    21: "MCU_PA0",  # VROUT → PA0 (0Ω jumper R0 optional later)
    24: "MCU_PE1",  # INJEN
    25: "PGND",
    27: "MCU_PE3",  # IGNEN
    28: "MCU_PE0",  # IN1
    29: "MCU_PE2",
    30: "MCU_PE4",
    31: "MCU_PE6",
    32: "MCU_PE9",  # IN5 IGN
    33: "MCU_PE11",
    34: "MCU_PE13",
    35: "MCU_PE15",
    36: "MCU_PE10",  # pump
    37: "MCU_PE12",  # fan
    38: "MCU_PB6",  # VVT
    39: "MCU_PB7",
    43: "MCU_PB8",  # CANRX
    44: "MCU_PB9",  # CANTX
    46: "CANH",
    47: "CANL",
    50: "PGND",
    51: "CKP_N",  # VRIN2
    52: "CKP_P",  # VRIN1
    54: "VBAT",  # BAT
    55: "MAIN_RLY",  # MR DNP path
    59: "INJ1",
    60: "INJ1",
    61: "INJ2",
    62: "INJ2",
    63: "INJ3",
    64: "INJ3",
    65: "INJ4",
    66: "INJ4",
    67: "FAN_RLY",  # OUT15
    68: "PUMP_RLY",  # OUT14
    75: "PGND",
    83: "VVT_EXH",
    84: "VVT_EXH",
    85: "VVT_EXH",
    87: "VBAT",  # BATPA
    90: "VBAT",  # BATPB
    92: "VVT_INT",
    93: "VVT_INT",
    94: "VVT_INT",
    96: "IGN1",
    97: "IGN2",
    98: "IGN3",
    99: "IGN4",
    100: "AGND",
}

# Discrete power chain nets (by ref pin)
# SOT-23 standard: 1=G, 2=S, 3=D for many P-FET GSD symbols — confirm with DS
# We use: pin1 gate (to VBAT_RAW via R later), pin2 source=VBAT_RAW, pin3 drain=VBAT_PROT
Q1_NETS = {1: "VBAT_RAW", 2: "VBAT_RAW", 3: "VBAT"}  # simplified: reverse FET placeholder
F1_NETS = {1: "VBAT", 2: "VBAT"}  # fuse in VBAT rail after FET (simplified single rail)
D1_NETS = {1: "PGND", 2: "VBAT"}  # TVS cathode VBAT (pin order footprint-dependent)
C1_NETS = {1: "VBAT", 2: "PGND"}
C2_NETS = {1: "VBAT", 2: "PGND"}
# TPS54302 SOT-23-6: 1=GND 2=SW 3=VIN 4=FB 5=EN 6=BOOT
U1_NETS = {1: "PGND", 2: "+5V_MAIN", 3: "VBAT", 4: "+5V_MAIN", 5: "VBAT", 6: "+5V_MAIN"}


def uid() -> str:
    return str(uuid.uuid4())


def load_mod(name: str) -> str:
    return (PRETTY / f"{name}.kicad_mod").read_text(encoding="utf-8")


def embed_footprint(mod_text: str, lib_id: str, ref: str, at: tuple[float, float], rot: int = 0) -> str:
    m = re.match(r'\(footprint\s+"[^"]+"\s*(.*)\)$', mod_text.strip(), re.S)
    if not m:
        # unquoted name
        m = re.match(r"\(footprint\s+(\S+)\s*(.*)\)$", mod_text.strip(), re.S)
        if not m:
            raise ValueError(f"bad mod {lib_id}")
        body = m.group(2)
    else:
        body = m.group(1)
    body = re.sub(r"^\(version [^)]+\)\s*", "", body)
    body = re.sub(r"^\(generator [^)]+\)\s*", "", body)
    body = body.replace('"REF**"', f'"{ref}"')
    body = re.sub(r"\(fp_text reference REF\*\*", f'(fp_text reference "{ref}"', body)
    body = re.sub(
        r'\(fp_text reference "REF\*\*"',
        f'(fp_text reference "{ref}"',
        body,
    )
    return (
        f'(footprint "{lib_id}" (layer "F.Cu")\n'
        f"    (tstamp {uid()})\n"
        f"    (at {at[0]} {at[1]} {rot})\n"
        f'    (path "/{uid()}")\n'
        f"{body}\n"
        f")"
    )


def strip_power_stage(text: str) -> str:
    """Remove previously placed power-stage footprints by reference."""
    refs = {"U3", "Q1", "F1", "D1", "C1", "C2", "U1"}
    out = []
    pos = 0
    pattern = re.compile(r'\(footprint "')
    while True:
        m = pattern.search(text, pos)
        if not m:
            out.append(text[pos:])
            break
        start = m.start()
        out.append(text[pos:start])
        depth = 0
        for i in range(start, len(text)):
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        block = text[start:end]
        ref_m = re.search(r'\(fp_text reference "([^"]+)"', block)
        ref = ref_m.group(1) if ref_m else ""
        if ref not in refs:
            out.append(block)
        pos = end
    return "".join(out)


def assign_nets_in_block(block: str, pinmap: dict[int, str], net_code: dict[str, int]) -> str:
    """Assign nets to pads; process pad-by-pad with balanced parentheses."""
    out: list[str] = []
    i = 0
    while i < len(block):
        m = re.match(r'\(pad ("?\d+"?)\s', block[i:])
        if not m:
            out.append(block[i])
            i += 1
            continue
        # parse pad number
        raw_num = m.group(1).strip('"')
        if not raw_num.isdigit():
            out.append(block[i])
            i += 1
            continue
        pn = int(raw_num)
        # find end of this pad s-expr
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
        pad_sexpr = block[start:j]
        if pn in pinmap and pinmap[pn] in net_code:
            netn = pinmap[pn]
            code = net_code[netn]
            if re.search(r"\(net \d+", pad_sexpr):
                pad_sexpr = re.sub(
                    r'\(net \d+ "[^"]*"\)', f'(net {code} "{netn}")', pad_sexpr
                )
            else:
                pad_sexpr = pad_sexpr[:-1] + f' (net {code} "{netn}")' + ")"
        out.append(pad_sexpr)
        i = j
    return "".join(out)


def ensure_nets(text: str, needed: set[str]) -> tuple[str, dict[str, int]]:
    existing = {name: int(c) for c, name in re.findall(r'\(net (\d+) "([^"]*)"\)', text)}
    existing.setdefault("", 0)
    next_id = max(existing.values(), default=0) + 1
    additions = []
    for n in sorted(needed):
        if n not in existing:
            existing[n] = next_id
            additions.append(f'  (net {next_id} "{n}")\n')
            next_id += 1
    if additions:
        text = re.sub(
            r'(  \(net 0 ""\)\n)((?:  \(net \d+ "[^"]*"\)\n)*)',
            lambda m: m.group(1) + m.group(2) + "".join(additions),
            text,
            count=1,
        )
    return text, existing


def segment(a, b, width, layer, net):
    return (
        f"  (segment (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) "
        f'(width {width}) (layer "{layer}") (net {net}) (tstamp {uid()}))\n'
    )


def via(pt, net, size=0.8, drill=0.4):
    return (
        f"  (via (at {pt[0]:.4f} {pt[1]:.4f}) (size {size}) (drill {drill}) "
        f'(layers "F.Cu" "B.Cu") (net {net}) (tstamp {uid()}))\n'
    )


def pad_abs(block: str, at: tuple[float, float], pin: int) -> tuple[float, float] | None:
    ox, oy = at
    m = re.search(
        rf'\(pad "?{pin}"? \w+ \w+ \(at ([-\d.]+) ([-\d.]+)',
        block,
    )
    if not m:
        return None
    return ox + float(m.group(1)), oy + float(m.group(2))


def main() -> None:
    text = PCB.read_text(encoding="utf-8")
    text = strip_power_stage(text)

    needed = set(TLE_NETS.values()) | set(Q1_NETS.values()) | set(F1_NETS.values())
    needed |= set(D1_NETS.values()) | set(C1_NETS.values()) | set(U1_NETS.values())
    needed |= {"VBAT_RAW", "VBAT", "+5V_MAIN", "+3V3", "PGND", "AGND", "CKP_DIG"}
    text, net_code = ensure_nets(text, needed)

    placements = [
        ("LQFP-100_14x14mm_P0.5mm", "OpenEMS:LQFP-100_14x14mm_P0.5mm", "U3", (92.0, 52.0), 0, TLE_NETS),
        ("SOT-23", "OpenEMS:SOT-23", "Q1", (62.0, 70.0), 0, Q1_NETS),
        ("Fuse_1206_3216Metric", "OpenEMS:Fuse_1206_3216Metric", "F1", (70.0, 70.0), 0, F1_NETS),
        ("D_SMB", "OpenEMS:D_SMB", "D1", (70.0, 76.0), 90, D1_NETS),
        ("CP_Elec_6.3x5.8", "OpenEMS:CP_Elec_6.3x5.8", "C1", (78.0, 70.0), 0, C1_NETS),
        ("C_0805_2012Metric", "OpenEMS:C_0805_2012Metric", "C2", (83.0, 70.0), 0, C2_NETS),
        ("SOT-23-6", "OpenEMS:SOT-23-6", "U1", (62.0, 58.0), 0, U1_NETS),
    ]

    parts = []
    blocks_for_route = {}
    for mod, lib_id, ref, at, rot, pmap in placements:
        raw = load_mod(mod)
        block = embed_footprint(raw, lib_id, ref, at, rot)
        block = assign_nets_in_block(block, pmap, net_code)
        parts.append(block)
        blocks_for_route[ref] = (block, at)

    # Insert footprints before final )
    blob = "\n".join(parts) + "\n"
    text = text.rstrip()
    if not text.endswith(")"):
        raise SystemExit("bad pcb")
    text = text[:-1] + "\n" + blob

    # --- Routes: power chain + TLE BAT/PGND/VDDIO + OUT shorts + SPI stubs ---
    routes = []
    routes.append(
        f'  (gr_text "Power stage: Q1→F1→VBAT bulk→TLE BAT; buck→+5V_MAIN\\n'
        f'TLE U3 LQFP100 @ (92,52). Straps SIN=AGND FCLN=+3V3 on copper."\n'
        f'    (at 65 48 0) (layer "Cmts.User") (tstamp {uid()})\n'
        f"    (effects (font (size 1.0 1.0) (thickness 0.12))))\n"
    )

    def p(ref: str, pin: int):
        block, at = blocks_for_route[ref]
        return pad_abs(block, at, pin)

    # VBAT_RAW bay (70,68) already from route_power — connect to Q1
    q1s = p("Q1", 2)
    q1d = p("Q1", 3)
    f1a, f1b = p("F1", 1), p("F1", 2)
    c1p = p("C1", 1)
    if q1s and f1a:
        # source region ← existing VBAT_RAW stub near (70,68)
        routes.append(
            f"  (segment (start 70.0000 68.0000) (end {q1s[0]:.4f} {q1s[1]:.4f}) "
            f'(width 1.5) (layer "F.Cu") (net {net_code["VBAT_RAW"]}) (tstamp {uid()}))\n'
        )
    if q1d and f1a:
        routes.append(
            f"  (segment (start {q1d[0]:.4f} {q1d[1]:.4f}) (end {f1a[0]:.4f} {f1a[1]:.4f}) "
            f'(width 1.5) (layer "F.Cu") (net {net_code["VBAT"]}) (tstamp {uid()}))\n'
        )
    if f1b and c1p:
        routes.append(
            f"  (segment (start {f1b[0]:.4f} {f1b[1]:.4f}) (end {c1p[0]:.4f} {c1p[1]:.4f}) "
            f'(width 1.5) (layer "F.Cu") (net {net_code["VBAT"]}) (tstamp {uid()}))\n'
        )

    # TLE BAT pins 54, 87, 90 → VBAT pour via short
    bat_pins = []
    for pin in (54, 87, 90):
        pt = p("U3", pin)
        if pt:
            bat_pins.append(pt)
    if bat_pins and c1p:
        # star to bulk
        for pt in bat_pins:
            routes.append(
                f"  (segment (start {pt[0]:.4f} {pt[1]:.4f}) (end {c1p[0]:.4f} {c1p[1]:.4f}) "
                f'(width 1.2) (layer "F.Cu") (net {net_code["VBAT"]}) (tstamp {uid()}))\n'
            )

    # TLE PGND 25,50,75 + AGND 100
    for pin, net in ((25, "PGND"), (50, "PGND"), (75, "PGND"), (100, "AGND")):
        pt = p("U3", pin)
        if not pt:
            continue
        # via to pour
        routes.append(via(pt, net_code[net], size=0.7, drill=0.35))

    # VDDIO + straps
    for pin, net in ((20, "+3V3"), (8, "+3V3"), (6, "AGND"), (9, "+5V_SENS_A"), (10, "+5V_SENS_B")):
        pt = p("U3", pin)
        if pt:
            routes.append(via(pt, net_code[net], size=0.6, drill=0.3))

    # OUT1 A+B short on copper (already same net) — add bridge if both found
    for a, b in ((59, 60), (61, 62), (63, 64), (65, 66)):
        pa, pb = p("U3", a), p("U3", b)
        if pa and pb:
            netn = TLE_NETS[a]
            routes.append(
                f"  (segment (start {pa[0]:.4f} {pa[1]:.4f}) (end {pb[0]:.4f} {pb[1]:.4f}) "
                f'(width 0.8) (layer "F.Cu") (net {net_code[netn]}) (tstamp {uid()}))\n'
            )

    # OUT5/6 triples
    for group in ((83, 84, 85), (92, 93, 94)):
        pts = [p("U3", g) for g in group]
        pts = [x for x in pts if x]
        netn = TLE_NETS[group[0]]
        for i in range(len(pts) - 1):
            routes.append(
                f"  (segment (start {pts[i][0]:.4f} {pts[i][1]:.4f}) "
                f"(end {pts[i+1][0]:.4f} {pts[i+1][1]:.4f}) "
                f'(width 0.8) (layer "F.Cu") (net {net_code[netn]}) (tstamp {uid()}))\n'
            )

    # Connect TLE OUT stubs toward J2 (INJ pads already have stubs at y=72)
    for pin, j2_stub_x in ((59, 98.0), (61, 102.0), (63, 106.0), (65, 110.0)):
        pt = p("U3", pin)
        if not pt:
            continue
        netn = TLE_NETS[pin]
        routes.append(
            f"  (segment (start {pt[0]:.4f} {pt[1]:.4f}) (end {j2_stub_x:.4f} 72.0000) "
            f'(width 0.6) (layer "F.Cu") (net {net_code[netn]}) (tstamp {uid()}))\n'
        )

    # IGN toward J2
    for pin, j2x in ((96, 114.0), (97, 118.0), (98, 122.0), (99, 126.0)):
        pt = p("U3", pin)
        if not pt or net_code.get(TLE_NETS[pin]) is None:
            continue
        routes.append(
            f"  (segment (start {pt[0]:.4f} {pt[1]:.4f}) (end {min(j2x, 128):.4f} 74.0000) "
            f'(width 0.4) (layer "F.Cu") (net {net_code[TLE_NETS[pin]]}) (tstamp {uid()}))\n'
        )

    # CKP: VRIN to J1 corridor (15,40)
    for pin, dest in ((52, (15.0, 40.0)), (51, (17.0, 40.0))):
        pt = p("U3", pin)
        if not pt:
            continue
        netn = TLE_NETS[pin]
        routes.append(
            f"  (segment (start {pt[0]:.4f} {pt[1]:.4f}) (end {dest[0]:.4f} {dest[1]:.4f}) "
            f'(width 0.35) (layer "F.Cu") (net {net_code[netn]}) (tstamp {uid()}))\n'
        )

    # SPI stubs toward WeAct corridor center (66, 38)
    spi_hub = (66.0, 38.0)
    for pin in (3, 4, 5, 7):
        pt = p("U3", pin)
        if not pt:
            continue
        netn = TLE_NETS[pin]
        if netn not in net_code:
            continue
        routes.append(
            f"  (segment (start {pt[0]:.4f} {pt[1]:.4f}) (end {spi_hub[0]:.4f} {spi_hub[1]:.4f}) "
            f'(width 0.3) (layer "F.Cu") (net {net_code[netn]}) (tstamp {uid()}))\n'
        )

    # Buck VIN from VBAT bulk
    u1vin = p("U1", 3)
    if u1vin and c1p:
        routes.append(
            f"  (segment (start {c1p[0]:.4f} {c1p[1]:.4f}) (end {u1vin[0]:.4f} {u1vin[1]:.4f}) "
            f'(width 1.0) (layer "F.Cu") (net {net_code["VBAT"]}) (tstamp {uid()}))\n'
        )

    text = text + "".join(routes) + ")\n"
    PCB.write_text(text, encoding="utf-8")
    print(f"Placed power stage on {PCB} ({PCB.stat().st_size} bytes)")
    print("  U3 TLE8888 LQFP-100 + Q1 F1 D1 C1 C2 U1 buck")


if __name__ == "__main__":
    main()
