#!/usr/bin/env python3
"""Add power/GND copper tracks and signal corridors on the PCB.

Assumes build_pcb_placement.py + annotate_pcb_nets.py already ran.
Strategy (docs/hw):
  - Thick VBAT_RAW from J2.1–2 into power zone (left of J2, above edge)
  - Thick PGND from J2.3–5 to NetTie NT1.1 and along bottom pour
  - Pair shorts for ETB motor dual pins
  - CKP_P/N pair routed north away from J2 power (twisted-pair corridor)
  - Keepout drawings for SPI (near WeAct) and CKP (near J1 left)
"""
from __future__ import annotations

import math
import re
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PCB = ROOT / "openems_interface_v1.kicad_pcb"

# Track widths (mm)
W_PWR = 1.5  # ~3A class with margin @ 1oz (conservative)
W_GND = 1.5
W_ETB = 2.0  # stall current dual pins
W_SIG = 0.3
W_PAIR = 0.35


def uid() -> str:
    return str(uuid.uuid4())


def load_nets(text: str) -> dict[str, int]:
    return {name: int(code) for code, name in re.findall(r'\(net (\d+) "([^"]*)"\)', text)}


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


def get_at(block: str) -> tuple[float, float, float]:
    m = re.search(r"\(at ([\d.-]+) ([\d.-]+)(?: ([\d.-]+))?\)", block)
    if not m:
        return 0.0, 0.0, 0.0
    return float(m.group(1)), float(m.group(2)), float(m.group(3) or 0)


def parse_pads(block: str, at: tuple[float, float, float]) -> dict[int, tuple[float, float, str | None]]:
    """pin -> (abs_x, abs_y, net_name)."""
    ox, oy, rot = at
    pads: dict[int, tuple[float, float, str | None]] = {}
    # pad N or "N" ... (at x y) ... optional (net code "name")
    for m in re.finditer(
        r'\(pad "?(\d+)"? \w+ \w+ \(at ([\d.-]+) ([\d.-]+)(?: [\d.-]+)?\)'
        r"((?:[^()]|\([^()]*\))*)\)",
        block,
    ):
        pn = int(m.group(1))
        lx, ly = float(m.group(2)), float(m.group(3))
        rest = m.group(4)
        nm = re.search(r'\(net \d+ "([^"]+)"\)', rest)
        net = nm.group(1) if nm else None
        # rotation 0 only
        ax, ay = ox + lx, oy + ly
        pads[pn] = (ax, ay, net)
    return pads


def segment(
    a: tuple[float, float],
    b: tuple[float, float],
    width: float,
    layer: str,
    net: int,
) -> str:
    return (
        f"  (segment (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) "
        f'(width {width}) (layer "{layer}") (net {net}) (tstamp {uid()}))\n'
    )


def via(pt: tuple[float, float], net: int, size: float = 0.8, drill: float = 0.4) -> str:
    return (
        f"  (via (at {pt[0]:.4f} {pt[1]:.4f}) (size {size}) (drill {drill}) "
        f'(layers "F.Cu" "B.Cu") (net {net}) (tstamp {uid()}))\n'
    )


def keepout_rect(x0: float, y0: float, x1: float, y1: float, label: str) -> str:
    lines = []
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    for a, b in zip(corners, corners[1:]):
        lines.append(
            f"  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]})\n"
            f'    (stroke (width 0.12) (type dash)) (layer "Eco1.User") (tstamp {uid()}))'
        )
    lines.append(
        f'  (gr_text "{label}" (at {(x0 + x1) / 2:.2f} {(y0 + y1) / 2:.2f} 0) '
        f'(layer "Eco1.User") (tstamp {uid()})\n'
        f"    (effects (font (size 1.0 1.0) (thickness 0.12))))\n"
    )
    return "\n".join(lines) + "\n"


def strip_previous_routes(text: str) -> str:
    """Remove segments/vias we previously generated (net-tagged power routes).

    Conservative: only strip all segments/vias if marker comment present.
    """
    text = re.sub(
        r"\n  \(segment \(start[\s\S]*?\(tstamp [0-9a-f-]+\)\)\n",
        "\n",
        text,
    )
    text = re.sub(
        r"\n  \(via \(at[\s\S]*?\(tstamp [0-9a-f-]+\)\)\n",
        "\n",
        text,
    )
    text = re.sub(
        r'\n  \(gr_line \(start[\s\S]*?\(layer "Eco1\.User"\)[\s\S]*?\)\n',
        "\n",
        text,
    )
    text = re.sub(
        r'\n  \(gr_text "(?:PWR|CKP|SPI|ETB)[\s\S]*?\(layer "Eco1\.User"\)[\s\S]*?\)\n',
        "\n",
        text,
    )
    text = re.sub(
        r"\n  \(gr_text \"Routing:[\s\S]*?\n  \)\n",
        "\n",
        text,
    )
    return text


def chain(pts: list[tuple[float, float]], width: float, layer: str, net: int) -> str:
    out = []
    for a, b in zip(pts, pts[1:]):
        out.append(segment(a, b, width, layer, net))
    return "".join(out)


def main() -> None:
    text = PCB.read_text(encoding="utf-8")
    text = strip_previous_routes(text)
    nets = load_nets(text)

    fps: dict[str, dict[int, tuple[float, float, str | None]]] = {}
    for _, _, block in footprint_blocks(text):
        ref = get_ref(block)
        if not ref:
            continue
        at = get_at(block)
        pads = parse_pads(block, at)
        if pads:
            fps[ref] = pads

    j1 = fps.get("J1", {})
    j2 = fps.get("J2", {})
    nt = fps.get("NT1", {})
    if not j2 or not nt:
        raise SystemExit(f"missing footprints J2/NT1: {list(fps)}")

    def pad(ref_pads: dict, n: int) -> tuple[float, float]:
        return ref_pads[n][0], ref_pads[n][1]

    routes: list[str] = []
    routes.append(
        '  (gr_text "Routing: VBAT_RAW/PGND thick; ETB dual; CKP pair N; SPI keepout\\n'
        'Fill zones after open. TLE/power parts not placed yet — stubs only."\n'
        f'    (at 65 58 0) (layer "Cmts.User") (tstamp {uid()})\n'
        "    (effects (font (size 1.1 1.1) (thickness 0.15))))\n"
    )

    # ---- VBAT_RAW: short J2.1–J2.2, stub into power bay (x~70, y~70) ----
    n_vbat = nets["VBAT_RAW"]
    p1, p2 = pad(j2, 1), pad(j2, 2)
    routes.append(segment(p1, p2, W_PWR, "F.Cu", n_vbat))
    mid = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    power_bay = (70.0, 68.0)  # reverse FET / fuse / bulk area (TBD place)
    routes.append(
        chain(
            [mid, (mid[0], 75.0), power_bay],
            W_PWR,
            "F.Cu",
            n_vbat,
        )
    )
    # local pour hook via on B.Cu
    routes.append(via(power_bay, n_vbat, size=1.0, drill=0.5))
    routes.append(segment(power_bay, (60.0, 68.0), W_PWR, "B.Cu", n_vbat))

    # ---- PGND: short J2.3–5, run to NT1.1, spine along bottom ----
    n_pgnd = nets["PGND"]
    g3, g4, g5 = pad(j2, 3), pad(j2, 4), pad(j2, 5)
    routes.append(segment(g3, g4, W_GND, "F.Cu", n_pgnd))
    routes.append(segment(g4, g5, W_GND, "F.Cu", n_pgnd))
    gmid = ((g3[0] + g5[0]) / 2, g3[1])
    nt1 = pad(nt, 1)  # PGND on NetTie
    # go north then west to NetTie
    routes.append(
        chain(
            [gmid, (gmid[0], 78.0), (nt1[0], 78.0), nt1],
            W_GND,
            "F.Cu",
            n_pgnd,
        )
    )
    # bottom GND spine
    routes.append(segment((10.0, 95.0), (120.0, 95.0), W_GND, "B.Cu", n_pgnd))
    routes.append(via((gmid[0], 95.0), n_pgnd, size=0.9, drill=0.45))
    routes.append(segment(gmid, (gmid[0], 95.0), W_GND, "F.Cu", n_pgnd))
    routes.append(via((nt1[0], nt1[1]), n_pgnd, size=0.8, drill=0.4))

    # ---- ETB dual pins shorted (same net already) + fat stubs ----
    n_etbp = nets["ETB_MOTOR_P"]
    n_etbn = nets["ETB_MOTOR_N"]
    # find pads by number 19-22
    if 19 in j2 and 20 in j2:
        routes.append(segment(pad(j2, 19), pad(j2, 20), W_ETB, "F.Cu", n_etbp))
        ep = ((pad(j2, 19)[0] + pad(j2, 20)[0]) / 2, pad(j2, 19)[1])
        routes.append(segment(ep, (ep[0], 78.0), W_ETB, "F.Cu", n_etbp))
    if 21 in j2 and 22 in j2:
        routes.append(segment(pad(j2, 21), pad(j2, 22), W_ETB, "F.Cu", n_etbn))
        en = ((pad(j2, 21)[0] + pad(j2, 22)[0]) / 2, pad(j2, 21)[1])
        routes.append(segment(en, (en[0], 76.0), W_ETB, "F.Cu", n_etbn))

    # ---- INJ1–4 short stubs north (to TLE OUT zone TBD) ----
    tle_zone_y = 72.0
    for pin, netname in [(6, "INJ1"), (7, "INJ2"), (8, "INJ3"), (9, "INJ4")]:
        if pin not in j2 or netname not in nets:
            continue
        p = pad(j2, pin)
        routes.append(
            segment(p, (p[0], tle_zone_y), W_SIG * 2, "F.Cu", nets[netname])
        )

    # ---- IGN stubs ----
    for pin, netname in [(10, "IGN1"), (11, "IGN2"), (12, "IGN3"), (13, "IGN4")]:
        if pin not in j2 or netname not in nets:
            continue
        p = pad(j2, pin)
        routes.append(
            segment(p, (p[0], tle_zone_y + 2), W_SIG, "F.Cu", nets[netname])
        )

    # ---- CKP differential pair from J1 → north (quiet zone left) ----
    if j1 and 1 in j1 and 2 in j1:
        n_ckpp, n_ckpn = nets["CKP_P"], nets["CKP_N"]
        ckp, ckn = pad(j1, 1), pad(j1, 2)
        # parallel run up left side
        routes.append(
            chain(
                [ckp, (ckp[0], 70.0), (15.0, 70.0), (15.0, 40.0)],
                W_PAIR,
                "F.Cu",
                n_ckpp,
            )
        )
        routes.append(
            chain(
                [ckn, (ckn[0], 71.0), (17.0, 71.0), (17.0, 40.0)],
                W_PAIR,
                "F.Cu",
                n_ckpn,
            )
        )
        # shield → NetTie SHIELD_GND (pad 4)
        if 3 in j1 and "SHIELD_GND" in nets:
            s = pad(j1, 3)
            if 4 in nt:
                routes.append(
                    chain(
                        [s, (s[0], 78.0), (pad(nt, 4)[0], 78.0), pad(nt, 4)],
                        W_SIG,
                        "B.Cu",
                        nets["SHIELD_GND"],
                    )
                )
                routes.append(via(s, nets["SHIELD_GND"]))

    # ---- CAN pair light stubs from J1 ----
    if j1 and 20 in j1 and 21 in j1 and "CANH" in nets:
        ch, cl = pad(j1, 20), pad(j1, 21)
        routes.append(segment(ch, (ch[0], 78.0), W_PAIR, "F.Cu", nets["CANH"]))
        routes.append(segment(cl, (cl[0], 77.0), W_PAIR, "F.Cu", nets["CANL"]))

    # ---- Keepout / corridor drawings ----
    routes.append(
        keepout_rect(5, 30, 28, 78, "CKP quiet\\nno INJ/ETB")
    )
    routes.append(
        keepout_rect(48, 20, 85, 55, "SPI2 corridor\\nPB12-15 ↔ TLE")
    )
    routes.append(
        keepout_rect(55, 60, 115, 78, "TLE power zone\\n(BAT/OUT stubs)")
    )
    routes.append(
        keepout_rect(58, 62, 75, 75, "PWR bay\\nFET/fuse/bulk")
    )

    # Inject before final )
    blob = "".join(routes)
    text = text.rstrip()
    if not text.endswith(")"):
        raise SystemExit("pcb format")
    text = text[:-1] + "\n" + blob + ")\n"
    PCB.write_text(text, encoding="utf-8")
    nseg = blob.count("(segment ")
    nvia = blob.count("(via ")
    print(f"Routed {PCB}: {nseg} segments, {nvia} vias")


if __name__ == "__main__":
    main()
