#!/usr/bin/env python3
"""Finish scaffold: LDO 3V3, WeAct critical pin nets, SPI/CAN/drive copper.

Run after place_power_stage.py (or via build_pcb_placement.py).
"""
from __future__ import annotations

import re
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRETTY = ROOT / "libs" / "OpenEMS.pretty"
PCB = ROOT / "openems_interface_v1.kicad_pcb"

# WeAct P1 = J3 (Odd_Even 2x25): pin → net (weact_h562_schematic.md)
J3_NETS = {
    1: "MCU_PC0",  # APP1
    2: "MCU_PC1",  # OIL
    3: "MCU_PC2",  # APP2
    4: "MCU_PC3",  # VBATT ADC
    6: "VREF_P",
    7: "MCU_PA0",  # CKP
    8: "MCU_PA1",  # CMP
    9: "MCU_PA2",
    10: "MCU_PA3",
    11: "MCU_PA4",
    12: "MCU_PA5",
    15: "MCU_PC4",
    16: "MCU_PC5",
    17: "MCU_PB0",
    18: "MCU_PB1",
    20: "MCU_PE7",  # ETB DIR1
    21: "MCU_PE8",  # ETB DIR2
    22: "MCU_PE9",  # IGN1
    23: "MCU_PE10",  # pump
    24: "MCU_PE11",  # IGN2
    25: "MCU_PE12",  # fan
    26: "MCU_PE13",  # IGN3
    29: "MCU_PB12",  # SPI CS
    30: "MCU_PB13",  # SPI SCK
    31: "MCU_PB14",  # SPI MISO
    32: "MCU_PB15",  # SPI MOSI
    5: "AGND",
    # power to coreboard
    # (many VDD pins exist on physical board — feed +3V3 on a few even rows if free)
}

# WeAct P2 = J4
J4_NETS = {
    1: "VB_WEACT",  # optional Vin — leave unconnected to carrier 3V3 path
    2: "MCU_PE6",  # INJ4
    3: "MCU_PE4",  # INJ3
    4: "MCU_PE5",  # ETB PWM
    5: "MCU_PE2",  # INJ2
    6: "MCU_PE3",  # IGNEN
    7: "MCU_PE0",  # INJ1
    9: "MCU_PB8",  # CAN RX
    10: "MCU_PB9",  # CAN TX
    11: "MCU_PB6",  # VVT
    12: "MCU_PB7",
    28: "MCU_PA11",  # USB DM
    30: "MCU_PA12",  # USB DP
}

# AP2204-class SOT-23-5 (typical): 1=IN 2=GND 3=EN 4=NC/BYP 5=OUT — confirm DS
U2_NETS = {1: "+5V_MAIN", 2: "AGND", 3: "+5V_MAIN", 4: "AGND", 5: "+3V3"}
# Ferrite 0805 between +3V3 and VDDA
FB1_NETS = {1: "+3V3", 2: "VDDA"}
C3_NETS = {1: "VDDA", 2: "AGND"}  # 1u
C4_NETS = {1: "+3V3", 2: "AGND"}  # 100n


def uid() -> str:
    return str(uuid.uuid4())


def load_mod(name: str) -> str:
    return (PRETTY / f"{name}.kicad_mod").read_text(encoding="utf-8")


def embed_footprint(mod_text: str, lib_id: str, ref: str, at: tuple[float, float], rot: int = 0) -> str:
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


def assign_nets_in_block(block: str, pinmap: dict[int, str], net_code: dict[str, int]) -> str:
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


def footprint_blocks(pcb: str):
    pattern = re.compile(r'\(footprint "')
    pos = 0
    while True:
        m = pattern.search(pcb, pos)
        if not m:
            return
        start = m.start()
        depth = 0
        for i in range(start, len(pcb)):
            if pcb[i] == "(":
                depth += 1
            elif pcb[i] == ")":
                depth -= 1
                if depth == 0:
                    yield start, i + 1, pcb[start : i + 1]
                    pos = i + 1
                    break


def get_ref(block: str) -> str | None:
    m = re.search(r'\(fp_text reference "([^"]+)"', block)
    return m.group(1) if m else None


def get_at(block: str) -> tuple[float, float]:
    m = re.search(r"\(at ([\d.-]+) ([\d.-]+)", block)
    return float(m.group(1)), float(m.group(2))


def pad_abs(block: str, at: tuple[float, float], pin: int) -> tuple[float, float] | None:
    ox, oy = at
    m = re.search(rf'\(pad "?{pin}"? \w+ \w+ \(at ([-\d.]+) ([-\d.]+)', block)
    if not m:
        return None
    return ox + float(m.group(1)), oy + float(m.group(2))


def weact_pin_local(pin: int, pitch: float = 2.54) -> tuple[float, float]:
    """Odd_Even dual-row: odd left, even right; row = (pin-1)//2."""
    row = (pin - 1) // 2
    if pin % 2 == 1:
        x = -pitch / 2
    else:
        x = pitch / 2
    y = row * pitch
    return x, y


def strip_refs(text: str, refs: set[str]) -> str:
    out = []
    pos = 0
    for start, end, block in footprint_blocks(text):
        out.append(text[pos:start])
        ref = get_ref(block)
        if ref not in refs:
            out.append(block)
        pos = end
    out.append(text[pos:])
    return "".join(out)


def reannotate_existing(text: str, ref: str, pinmap: dict[int, str], net_code: dict[str, int]) -> str:
    out = []
    pos = 0
    for start, end, block in footprint_blocks(text):
        out.append(text[pos:start])
        if get_ref(block) == ref:
            block = assign_nets_in_block(block, pinmap, net_code)
        out.append(block)
        pos = end
    out.append(text[pos:])
    return "".join(out)


def segment(a, b, w, layer, net):
    return (
        f"  (segment (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) "
        f'(width {w}) (layer "{layer}") (net {net}) (tstamp {uid()}))\n'
    )


def chain(pts, w, layer, net):
    return "".join(segment(a, b, w, layer, net) for a, b in zip(pts, pts[1:]))


def via(pt, net, size=0.7, drill=0.35):
    return (
        f"  (via (at {pt[0]:.4f} {pt[1]:.4f}) (size {size}) (drill {drill}) "
        f'(layers "F.Cu" "B.Cu") (net {net}) (tstamp {uid()}))\n'
    )


def main() -> None:
    text = PCB.read_text(encoding="utf-8")
    text = strip_refs(text, {"U2", "FB1", "C3", "C4"})

    needed = set()
    for d in (J3_NETS, J4_NETS, U2_NETS, FB1_NETS, C3_NETS, C4_NETS):
        needed |= set(d.values())
    needed |= {
        "+3V3",
        "+5V_MAIN",
        "VDDA",
        "AGND",
        "VREF_P",
        "MCU_PB12",
        "MCU_PB13",
        "MCU_PB14",
        "MCU_PB15",
        "MCU_PB8",
        "MCU_PB9",
        "MCU_PE0",
        "MCU_PE2",
        "MCU_PE4",
        "MCU_PE6",
        "MCU_PE5",
        "MCU_PE3",
        "MCU_PE1",
        "MCU_PE7",
        "MCU_PE8",
        "MCU_PE9",
        "MCU_PE10",
        "MCU_PE11",
        "MCU_PE12",
        "MCU_PE13",
        "MCU_PE15",
        "MCU_PB6",
        "MCU_PB7",
        "MCU_PA0",
        "MCU_PA11",
        "MCU_PA12",
    }
    text, net_code = ensure_nets(text, needed)

    # Annotate WeAct headers
    text = reannotate_existing(text, "J3", J3_NETS, net_code)
    text = reannotate_existing(text, "J4", J4_NETS, net_code)

    # Place LDO chain near buck
    placements = [
        ("SOT-23-5", "OpenEMS:SOT-23-5", "U2", (55.0, 58.0), 0, U2_NETS),
        ("L_0805_2012Metric", "OpenEMS:L_0805_2012Metric", "FB1", (55.0, 52.0), 0, FB1_NETS),
        ("C_0805_2012Metric", "OpenEMS:C_0805_2012Metric", "C3", (50.0, 52.0), 0, C3_NETS),
        ("C_0805_2012Metric", "OpenEMS:C_0805_2012Metric", "C4", (50.0, 58.0), 0, C4_NETS),
    ]
    parts = []
    blocks = {}
    for mod, lib_id, ref, at, rot, pmap in placements:
        raw = load_mod(mod)
        block = embed_footprint(raw, lib_id, ref, at, rot)
        block = assign_nets_in_block(block, pmap, net_code)
        parts.append(block)
        blocks[ref] = (block, at)

    # Collect existing U3, J3, J4 blocks for routing
    for start, end, block in footprint_blocks(text):
        ref = get_ref(block)
        if ref in ("U3", "J3", "J4", "U1", "NT1"):
            blocks[ref] = (block, get_at(block))

    text = text.rstrip()
    if not text.endswith(")"):
        raise SystemExit("bad pcb")
    text = text[:-1] + "\n" + "\n".join(parts) + "\n"

    routes = []
    routes.append(
        f'  (gr_text "Finish: LDO U2 + FB1 VDDA; WeAct J3/J4 nets; SPI/CAN/drive\\n'
        f"Confirm WeAct pin1 silk vs SchDoc before fab. Not production-ready.\"\n"
        f'    (at 65 44 0) (layer "Cmts.User") (tstamp {uid()})\n'
        f"    (effects (font (size 1.0 1.0) (thickness 0.12))))\n"
    )

    def p(ref: str, pin: int):
        if ref not in blocks:
            return None
        return pad_abs(blocks[ref][0], blocks[ref][1], pin)

    def weact_abs(ref: str, pin: int):
        if ref not in blocks:
            return None
        ox, oy = blocks[ref][1]
        lx, ly = weact_pin_local(pin)
        return ox + lx, oy + ly

    # --- LDO: U1 SW/out area already +5V_MAIN; U2 IN from +5V_MAIN, OUT +3V3 ---
    u2_in, u2_out = p("U2", 1), p("U2", 5)
    u1_sw = p("U1", 2)
    if u1_sw and u2_in:
        routes.append(segment(u1_sw, u2_in, 0.8, "F.Cu", net_code["+5V_MAIN"]))
    if u2_out:
        routes.append(via(u2_out, net_code["+3V3"]))
    fb_a, fb_b = p("FB1", 1), p("FB1", 2)
    if u2_out and fb_a:
        routes.append(segment(u2_out, fb_a, 0.5, "F.Cu", net_code["+3V3"]))
    if fb_b:
        routes.append(via(fb_b, net_code["VDDA"]))
        # VREF_P = VDDA (option a)
        if "VREF_P" in net_code:
            routes.append(
                segment(fb_b, (fb_b[0] + 3, fb_b[1]), 0.4, "F.Cu", net_code["VDDA"])
            )

    # Feed +3V3 to TLE VDDIO (pin 20) if U3 present
    tle_vddio = p("U3", 20)
    if u2_out and tle_vddio:
        routes.append(
            chain(
                [u2_out, (u2_out[0], 52.0), (tle_vddio[0], 52.0), tle_vddio],
                0.5,
                "F.Cu",
                net_code["+3V3"],
            )
        )

    # --- SPI: J3.29-32 → SPI hub (66,38) → already connected toward U3 ---
    spi_hub = (66.0, 38.0)
    for pin, net in (
        (29, "MCU_PB12"),
        (30, "MCU_PB13"),
        (31, "MCU_PB14"),
        (32, "MCU_PB15"),
    ):
        pt = weact_abs("J3", pin)
        if not pt or net not in net_code:
            continue
        routes.append(chain([pt, (pt[0], 38.0), spi_hub], 0.3, "F.Cu", net_code[net]))

    # --- CAN: J4.9/10 → TLE 43/44 area ---
    for pin, net, tle_pin in ((9, "MCU_PB8", 43), (10, "MCU_PB9", 44)):
        pt = weact_abs("J4", pin)
        tp = p("U3", tle_pin)
        if not pt or not tp or net not in net_code:
            continue
        routes.append(chain([pt, (pt[0], 40.0), (tp[0], 40.0), tp], 0.3, "F.Cu", net_code[net]))

    # --- Direct drive: J4 PE → TLE IN pins ---
    drive_map = [
        # (header, pin, net, tle_pin)
        ("J4", 7, "MCU_PE0", 28),  # INJ1
        ("J4", 5, "MCU_PE2", 29),  # INJ2
        ("J4", 3, "MCU_PE4", 30),  # INJ3
        ("J4", 2, "MCU_PE6", 31),  # INJ4
        ("J4", 6, "MCU_PE3", 27),  # IGNEN
        ("J3", 22, "MCU_PE9", 32),  # IGN1
        ("J3", 24, "MCU_PE11", 33),
        ("J3", 26, "MCU_PE13", 34),
        # PE15 IGN4 — may be further on header; skip if not mapped
        ("J3", 23, "MCU_PE10", 36),  # pump
        ("J3", 25, "MCU_PE12", 37),  # fan
        ("J4", 11, "MCU_PB6", 38),  # VVT
        ("J4", 12, "MCU_PB7", 39),
        ("J3", 7, "MCU_PA0", 21),  # CKP dig via VROUT — actually PA0 is MCU side of R0
    ]
    for href, hpin, net, tpin in drive_map:
        pt = weact_abs(href, hpin)
        tp = p("U3", tpin)
        if not pt or not tp or net not in net_code:
            continue
        # mid routing at y=45 to avoid clutter
        routes.append(
            chain(
                [pt, (pt[0], 45.0), (tp[0], 45.0), tp],
                0.35,
                "F.Cu",
                net_code[net],
            )
        )

    # CKP dig: U3.21 VROUT → near PA0 weact (already MCU_PA0 on J3.7)
    vrout = p("U3", 21)
    pa0 = weact_abs("J3", 7)
    if vrout and pa0 and "MCU_PA0" in net_code:
        # VROUT and PA0 are different nets in ideal design (via 0R) — short for scaffold as MCU_PA0
        pass  # already routed PA0 to pin 21 above as MCU_PA0 — fix: pin 21 is CKP_DIG
    # Route CKP_DIG from U3.21 to J3.7 using MCU_PA0 net for scaffold continuity
    if vrout and pa0 and "MCU_PA0" in net_code:
        # Re-assign: connect both on MCU_PA0 for bring-up (jumper 0R later)
        routes.append(
            chain(
                [vrout, (vrout[0], 42.0), (pa0[0], 42.0), pa0],
                0.35,
                "B.Cu",
                net_code["MCU_PA0"],
            )
        )
        routes.append(via(vrout, net_code["MCU_PA0"]))

    # +3V3 / AGND to WeAct keepout edge (coreboard supply)
    # J3 pin 5 = AGND already; pick a free approach
    ag = weact_abs("J3", 5)
    if u2_out and ag and "AGND" in net_code:
        routes.append(via(ag, net_code["AGND"]))
    # inject +3V3 near WeAct SW hole H1 area
    if u2_out:
        routes.append(
            chain(
                [u2_out, (48.0, 58.0), (48.0, 20.0)],
                0.6,
                "F.Cu",
                net_code["+3V3"],
            )
        )

    text = text + "".join(routes) + ")\n"
    PCB.write_text(text, encoding="utf-8")
    nseg = "".join(routes).count("(segment ")
    print(f"Finish placed on {PCB}: LDO/FB + WeAct nets + {nseg} new segments")


if __name__ == "__main__":
    main()
