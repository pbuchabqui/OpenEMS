#!/usr/bin/env python3
"""Assign J1/J2 pad nets from pinmap_logical.md (authoritative).

Schematic global-label connectivity on dense 1.27 mm connectors is unreliable
in generated files; PCB pad nets follow docs/hw/netlist_v1.md.
"""
from __future__ import annotations

import re
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PCB = ROOT / "openems_interface_v1.kicad_pcb"

J2 = {
    1: "VBAT_RAW",
    2: "VBAT_RAW",
    3: "PGND",
    4: "PGND",
    5: "PGND",
    6: "INJ1",
    7: "INJ2",
    8: "INJ3",
    9: "INJ4",
    10: "IGN1",
    11: "IGN2",
    12: "IGN3",
    13: "IGN4",
    14: "VVT_EXH",
    15: "VVT_INT",
    16: "PUMP_RLY",
    17: "FAN_RLY",
    18: "MAIN_RLY",
    19: "ETB_MOTOR_P",
    20: "ETB_MOTOR_P",
    21: "ETB_MOTOR_N",
    22: "ETB_MOTOR_N",
}
J1 = {
    1: "CKP_P",
    2: "CKP_N",
    # shields join SHIELD_GND at star (NetTie) — same net on copper
    3: "SHIELD_GND",
    4: "CMP_SIG",
    5: "CMP_5V",
    6: "CMP_GND",
    7: "MAP",
    8: "CLT",
    9: "IAT",
    10: "APP1",
    11: "APP2",
    12: "FUEL_PRESS",
    13: "OIL_PRESS",
    14: "ETB_TPS1",
    15: "ETB_TPS2",
    16: "+5V_SENS_A",
    17: "+5V_SENS_B",
    18: "SGND",
    19: "SGND",
    20: "CANH",
    21: "CANL",
    22: "SHIELD_GND",
    23: "FLEX_12V",
    24: "FLEX_SIG",
    25: "FLEX_GND",
    26: "KNOCK_SIG",
    27: "SHIELD_GND",
    28: "CMP2_SIG",
    29: "CMP2_5V",
    30: "CMP2_GND",
    31: "TPS_INDEP",
}
# Star ground NetTie NT1 (KiCad NetTie-4): pads 1..4
NETTIE = {
    1: "PGND",
    2: "SGND",
    3: "AGND",
    4: "SHIELD_GND",
}


def uid() -> str:
    return str(uuid.uuid4())


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


def main() -> None:
    text = PCB.read_text(encoding="utf-8")
    nets = sorted(
        set(J1.values())
        | set(J2.values())
        | {
            "PGND",
            "SGND",
            "AGND",
            "SHIELD_GND",
            "+3V3",
            "VBAT",
            "+5V_MAIN",
            "VDDA",
            "VREF_P",
        }
    )
    net_code = {"": 0}
    net_lines = ['  (net 0 "")']
    for i, n in enumerate(nets, 1):
        net_code[n] = i
        net_lines.append(f'  (net {i} "{n}")')

    if re.search(r'\(net 0 ""\)', text):
        text = re.sub(
            r'(  \(net 0 ""\)\n)(?:  \(net \d+ "[^"]*"\)\n)*',
            "\n".join(net_lines) + "\n",
            text,
            count=1,
        )

    def get_ref(block: str) -> str | None:
        m = re.search(r'\(fp_text reference "([^"]+)"', block)
        return m.group(1) if m else None

    out: list[str] = []
    pos = 0
    for start, end, block in footprint_blocks(text):
        out.append(text[pos:start])
        ref = get_ref(block)
        pinmap = {"J1": J1, "J2": J2, "NT1": NETTIE}.get(ref or "")
        if pinmap:

            def repl_pad(m: re.Match[str]) -> str:
                pnum = m.group(1)
                full = m.group(0)
                if not pnum.isdigit():
                    return full
                pn = int(pnum)
                if pn not in pinmap:
                    return full
                netn = pinmap[pn]
                code = net_code[netn]
                if re.search(r"\(net \d+", full):
                    return re.sub(
                        r'\(net \d+ "[^"]*"\)', f'(net {code} "{netn}")', full
                    )
                return full[:-1] + f' (net {code} "{netn}")' + ")"

            # Quoted pad numbers (OpenEMS generator) and bare (rusEFI legacy)
            block = re.sub(
                r'\(pad "(\d+)" ((?:[^()]|\([^()]*\))*)\)',
                repl_pad,
                block,
            )
            block = re.sub(
                r"\(pad (\d+) ((?:[^()]|\([^()]*\))*)\)",
                repl_pad,
                block,
            )
        out.append(block)
        pos = end
    out.append(text[pos:])
    text = "".join(out)

    text = re.sub(
        r'\n  \(zone \(net \d+\) \(net_name "PGND"\)[\s\S]*?\n  \)\n',
        "\n",
        text,
    )
    text = re.sub(
        r'\n  \(gr_text "PGND zones[\s\S]*?\n  \)\n',
        "\n",
        text,
    )

    pgnd = net_code["PGND"]
    z1, z2, zt = uid(), uid(), uid()
    zones = f"""
  (zone (net {pgnd}) (net_name "PGND") (layer "F.Cu") (tstamp {z1}) (hatch edge 0.5)
    (connect_pads (clearance 0.2))
    (min_thickness 0.25)
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon
      (pts
        (xy 1 1)
        (xy 129 1)
        (xy 129 99)
        (xy 1 99)
      )
    )
  )
  (zone (net {pgnd}) (net_name "PGND") (layer "B.Cu") (tstamp {z2}) (hatch edge 0.5)
    (connect_pads (clearance 0.2))
    (min_thickness 0.25)
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon
      (pts
        (xy 1 1)
        (xy 129 1)
        (xy 129 99)
        (xy 1 99)
      )
    )
  )
  (gr_text "PGND zones F+B (B: Fill All Zones in Pcbnew)\\nJ1/J2 nets = pinmap_logical.md (authoritative)\\nStar NetTie: PGND-SGND-AGND-SHIELD near WeAct\\nAGND pour island near ADC later"
    (at 65 52 0) (layer "Cmts.User") (tstamp {zt})
    (effects (font (size 1.3 1.3) (thickness 0.18)))
  )
"""
    text = text.rstrip()
    if text.endswith(")"):
        text = text[:-1] + zones + ")\n"
    PCB.write_text(text, encoding="utf-8")
    print(f"Annotated {PCB} ({PCB.stat().st_size} bytes), {len(nets)} nets")


if __name__ == "__main__":
    main()
