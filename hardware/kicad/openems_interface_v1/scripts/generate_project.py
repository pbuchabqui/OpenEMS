#!/usr/bin/env python3
"""Generate OpenEMS interface board v1 KiCad 7 hierarchical project.

Produces a project openable in KiCad 7.0.x with:
  - root schematic + 10 modular sheets (docs/hw/schematic/*)
  - custom OpenEMS symbol lib (TLE8888 multi-unit, power nets, modules)
  - .kicad_pro / placeholder .kicad_pcb / sym-lib-table / LEIA-ME.md

Authority for nets/pin maps remains docs/hw/*.md — this materialises them.
"""
from __future__ import annotations

import json
import re
import textwrap
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # openems_interface_v1/
SHEETS = ROOT / "sheets"
LIBS = ROOT / "libs"
PROJECT = "openems_interface_v1"
SCH_VER = "20230121"
SYM_VER = "20220914"
PCB_VER = "20221018"


def uid() -> str:
    return str(uuid.uuid4())


def g(x: float) -> float:
    """Snap to 1.27 mm (50 mil) grid."""
    return round(x / 1.27) * 1.27


# ---------------------------------------------------------------------------
# S-expression primitives
# ---------------------------------------------------------------------------

def text_block(s: str, at: tuple[float, float], size: float = 1.27) -> str:
    escaped = s.replace("\\", "\\\\").replace('"', '\\"')
    # multi-line texts use \n
    return f"""  (text "{escaped}" (at {at[0]} {at[1]} 0)
    (effects (font (size {size} {size})) (justify left top))
    (uuid {uid()})
  )"""


def label(name: str, at: tuple[float, float], rot: int = 0, justify: str = "left") -> str:
    j = "left" if justify == "left" else "right"
    if rot in (0, 180):
        j2 = "bottom"
    else:
        j2 = "right" if rot == 90 else "left"
    return f"""  (label "{name}" (at {at[0]} {at[1]} {rot})
    (effects (font (size 1.27 1.27)) (justify {j} {j2}))
    (uuid {uid()})
  )"""


def global_label(name: str, at: tuple[float, float], rot: int = 0, shape: str = "input") -> str:
    return f"""  (global_label "{name}" (shape {shape}) (at {at[0]} {at[1]} {rot}) (fields_autoplaced)
    (effects (font (size 1.27 1.27)) (justify left))
    (uuid {uid()})
    (property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {at[0]} {at[1]} 0)
      (effects (font (size 1.27 1.27)) hide)
    )
  )"""


def wire(a: tuple[float, float], b: tuple[float, float]) -> str:
    return f"""  (wire (pts (xy {a[0]} {a[1]}) (xy {b[0]} {b[1]}))
    (stroke (width 0) (type default))
    (uuid {uid()})
  )"""


def no_connect(at: tuple[float, float]) -> str:
    return f"""  (no_connect (at {at[0]} {at[1]}) (uuid {uid()}))"""


def power_flag(at: tuple[float, float]) -> str:
    """Inline power-flag symbol instance (lib must be available)."""
    return symbol_inst(
        "power:PWR_FLAG",
        "PWR_FLAG",
        "#FLG",
        at,
        pins=["1"],
        hide_ref=True,
    )


def symbol_inst(
    lib_id: str,
    value: str,
    ref: str,
    at: tuple[float, float],
    unit: int = 1,
    rot: int = 0,
    pins: list[str] | None = None,
    footprint: str = "",
    dnp: bool = False,
    hide_ref: bool = False,
) -> str:
    pin_lines = ""
    if pins:
        pin_lines = "\n".join(
            f'    (pin "{p}" (uuid {uid()}))' for p in pins
        )
    ref_hide = " hide" if hide_ref or ref.startswith("#") else ""
    dnp_s = "yes" if dnp else "no"
    return f"""  (symbol (lib_id "{lib_id}") (at {at[0]} {at[1]} {rot}) (unit {unit})
    (in_bom yes) (on_board yes) (dnp {dnp_s})
    (uuid {uid()})
    (property "Reference" "{ref}" (at {at[0]} {at[1] - 5.08} 0)
      (effects (font (size 1.27 1.27)){ref_hide})
    )
    (property "Value" "{value}" (at {at[0]} {at[1] + 5.08} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "{footprint}" (at {at[0]} {at[1]} 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "~" (at {at[0]} {at[1]} 0)
      (effects (font (size 1.27 1.27)) hide)
    )
{pin_lines}
  )"""


def title_block(title: str, rev: str = "v1", comment: str = "") -> str:
    c = f'\n    (comment 1 "{comment}")' if comment else ""
    return f"""  (paper "A3")
  (title_block
    (title "{title}")
    (date "2026-07-20")
    (rev "{rev}")
    (company "OpenEMS")
    (comment 2 "docs/hw/schematic — fonte de verdade nos .md"){c}
  )"""


def sch_header(title: str, lib_symbols: str = "", body: str = "", page: str = "1") -> str:
    return f"""(kicad_sch (version {SCH_VER}) (generator openems_gen)

  (uuid {uid()})

{title_block(title)}

  (lib_symbols
{lib_symbols}
  )

{body}

  (sheet_instances
    (path "/" (page "{page}"))
  )
)
"""


# ---------------------------------------------------------------------------
# Custom symbol library
# ---------------------------------------------------------------------------

def power_sym(name: str) -> str:
    """Simple power port symbol (arrow style like KiCad power)."""
    safe = name.replace("+", "p").replace("-", "n").replace(".", "_")
    return f"""  (symbol "{name}" (power) (pin_names (offset 0)) (in_bom yes) (on_board yes)
    (property "Reference" "#PWR" (at 0 -3.81 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Value" "{name}" (at 0 3.556 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_keywords" "power {name}" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_description" "Power net {name}" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (symbol "{name}_0_1"
      (polyline
        (pts (xy -0.762 1.27) (xy 0 2.54))
        (stroke (width 0) (type default)) (fill (type none))
      )
      (polyline
        (pts (xy 0 0) (xy 0 2.54))
        (stroke (width 0) (type default)) (fill (type none))
      )
      (polyline
        (pts (xy 0 2.54) (xy 0.762 1.27))
        (stroke (width 0) (type default)) (fill (type none))
      )
    )
    (symbol "{name}_1_1"
      (pin power_in line (at 0 0 90) (length 0) hide
        (name "{name}" (effects (font (size 1.27 1.27))))
        (number "1" (effects (font (size 1.27 1.27))))
      )
    )
  )"""


def gnd_sym(name: str) -> str:
    return f"""  (symbol "{name}" (power) (pin_names (offset 0)) (in_bom yes) (on_board yes)
    (property "Reference" "#PWR" (at 0 -6.35 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Value" "{name}" (at 0 -3.81 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_keywords" "power {name} ground" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (symbol "{name}_0_1"
      (polyline
        (pts (xy 0 0) (xy 0 -1.27) (xy 1.27 -1.27) (xy 0 -2.54) (xy -1.27 -1.27) (xy 0 -1.27))
        (stroke (width 0) (type default)) (fill (type none))
      )
    )
    (symbol "{name}_1_1"
      (pin power_in line (at 0 0 270) (length 0) hide
        (name "{name}" (effects (font (size 1.27 1.27))))
        (number "1" (effects (font (size 1.27 1.27))))
      )
    )
  )"""


def pin_def(etype: str, name: str, number: str, x: float, y: float, rot: int, length: float = 2.54) -> str:
    return f"""      (pin {etype} line (at {x} {y} {rot}) (length {length})
        (name "{name}" (effects (font (size 1.016 1.016))))
        (number "{number}" (effects (font (size 1.016 1.016))))
      )"""


def tle8888_symbol() -> str:
    """Multi-unit TLE8888-2QK (LQFP-100). Units A–G match schematic modules."""
    # Parent
    units = []

    # Unit A — Power / trackers / supplies
    pins_a = [
        ("power_in", "BAT", "54", -15.24, 10.16, 0),
        ("power_in", "BATPA", "87", -15.24, 7.62, 0),
        ("power_in", "BATPB", "90", -15.24, 5.08, 0),
        ("power_in", "BATSTBY", "53", -15.24, 2.54, 0),
        ("power_in", "VDDIO", "20", -15.24, -2.54, 0),
        ("power_in", "V5VCAN", "45", -15.24, -5.08, 0),
        ("power_in", "KEY", "49", -15.24, -7.62, 0),
        ("power_out", "T5V1", "9", 15.24, 10.16, 180),
        ("power_out", "T5V2", "10", 15.24, 7.62, 180),
        ("power_out", "V5V", "11", 15.24, 5.08, 180),
        ("power_out", "V5VSTBY", "41", 15.24, 2.54, 180),
        ("passive", "CP", "95", 15.24, -2.54, 180),
        ("open_collector", "MR", "55", 15.24, -5.08, 180),
        ("passive", "RST", "1", 15.24, -7.62, 180),
        ("passive", "MON", "2", 15.24, -10.16, 180),
        ("power_in", "PGND", "25", 0, -15.24, 90),
        ("power_in", "PGND_2", "50", 2.54, -15.24, 90),
        ("power_in", "PGND_3", "75", 5.08, -15.24, 90),
        ("power_in", "AGND", "100", -2.54, -15.24, 90),
    ]
    units.append(("A", "Power/Trackers", pins_a, 17.78, 17.78))

    # Unit B — SPI
    pins_b = [
        ("input", "CSN", "3", -10.16, 5.08, 0),
        ("output", "SDO", "4", 10.16, 5.08, 180),
        ("input", "SIP", "5", -10.16, 2.54, 0),
        ("input", "SIN", "6", -10.16, 0, 0),
        ("input", "FCLP", "7", -10.16, -2.54, 0),
        ("input", "FCLN", "8", -10.16, -5.08, 0),
    ]
    units.append(("B", "SPI", pins_b, 12.7, 7.62))

    # Unit C — Injectors IN1-4 OUT1-4 + INJEN
    pins_c = [
        ("input", "INJEN", "24", -12.7, 10.16, 0),
        ("input", "IN1", "28", -12.7, 7.62, 0),
        ("input", "IN2", "29", -12.7, 5.08, 0),
        ("input", "IN3", "30", -12.7, 2.54, 0),
        ("input", "IN4", "31", -12.7, 0, 0),
        ("open_collector", "OUT1A", "59", 12.7, 7.62, 180),
        ("open_collector", "OUT1B", "60", 12.7, 5.08, 180),
        ("open_collector", "OUT2A", "61", 12.7, 2.54, 180),
        ("open_collector", "OUT2B", "62", 12.7, 0, 180),
        ("open_collector", "OUT3A", "63", 12.7, -2.54, 180),
        ("open_collector", "OUT3B", "64", 12.7, -5.08, 180),
        ("open_collector", "OUT4A", "65", 12.7, -7.62, 180),
        ("open_collector", "OUT4B", "66", 12.7, -10.16, 180),
    ]
    units.append(("C", "Injectors", pins_c, 15.24, 12.7))

    # Unit D — Ignition IN5-8 IGN1-4 + IGNEN
    pins_d = [
        ("input", "IGNEN", "27", -12.7, 7.62, 0),
        ("input", "IN5", "32", -12.7, 5.08, 0),
        ("input", "IN6", "33", -12.7, 2.54, 0),
        ("input", "IN7", "34", -12.7, 0, 0),
        ("input", "IN8", "35", -12.7, -2.54, 0),
        ("output", "IGN1", "96", 12.7, 5.08, 180),
        ("output", "IGN2", "97", 12.7, 2.54, 180),
        ("output", "IGN3", "98", 12.7, 0, 180),
        ("output", "IGN4", "99", 12.7, -2.54, 180),
    ]
    units.append(("D", "Ignition", pins_d, 15.24, 10.16))

    # Unit E — Relays / VVT IN9-12 OUT14/15 OUT5/6
    pins_e = [
        ("input", "IN9", "36", -15.24, 10.16, 0),
        ("input", "IN10", "37", -15.24, 7.62, 0),
        ("input", "IN11", "38", -15.24, 5.08, 0),
        ("input", "IN12", "39", -15.24, 2.54, 0),
        ("open_collector", "OUT14", "68", 15.24, 10.16, 180),
        ("open_collector", "OUT15", "67", 15.24, 7.62, 180),
        ("open_collector", "OUT5A", "83", 15.24, 2.54, 180),
        ("open_collector", "OUT5B", "84", 15.24, 0, 180),
        ("open_collector", "OUT5C", "85", 15.24, -2.54, 180),
        ("open_collector", "OUT6A", "92", 15.24, -5.08, 180),
        ("open_collector", "OUT6B", "93", 15.24, -7.62, 180),
        ("open_collector", "OUT6C", "94", 15.24, -10.16, 180),
    ]
    units.append(("E", "Relays/VVT", pins_e, 17.78, 12.7))

    # Unit F — CAN
    pins_f = [
        ("input", "CANTX", "44", -10.16, 2.54, 0),
        ("output", "CANRX", "43", -10.16, 0, 0),
        ("bidirectional", "CANH", "46", 10.16, 2.54, 180),
        ("bidirectional", "CANL", "47", 10.16, 0, 180),
        ("input", "CANWKEN", "42", -10.16, -2.54, 0),
        ("input", "WK", "48", -10.16, -5.08, 0),
    ]
    units.append(("F", "CAN", pins_f, 12.7, 7.62))

    # Unit G — VR
    pins_g = [
        ("input", "VRIN1", "52", -10.16, 2.54, 0),
        ("input", "VRIN2", "51", -10.16, 0, 0),
        ("output", "VROUT", "21", 10.16, 1.27, 180),
    ]
    units.append(("G", "VR", pins_g, 12.7, 5.08))

    unit_letters = "ABCDEFG"
    parts = []
    # Parent shell
    parts.append(f"""  (symbol "TLE8888" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
    (property "Reference" "U" (at 0 20.32 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Value" "TLE8888-2QK" (at 0 17.78 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "Package_QFP:LQFP-100_14x14mm_P0.5mm" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "https://www.infineon.com/tle8888" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_keywords" "TLE8888 Infineon ECU power hub" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_description" "Engine management power IC LQFP-100, -2QK WD off" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_fp_filters" "LQFP*14x14*P0.5*" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )""")

    for i, (letter, uname, pins, half_w, half_h) in enumerate(units):
        # unit index 1-based
        u = i + 1
        # graphics
        parts.append(f"""    (symbol "TLE8888_{u}_1"
      (rectangle (start {-half_w} {-half_h}) (end {half_w} {half_h})
        (stroke (width 0.254) (type default)) (fill (type background))
      )
      (text "{uname}" (at 0 {half_h - 1.27} 0)
        (effects (font (size 1.27 1.27)))
      )""")
        for et, nm, num, x, y, rot in pins:
            parts.append(pin_def(et, nm, num, x, y, rot))
        parts.append("    )")

    parts.append("  )")
    return "\n".join(parts)


def bts7960_symbol() -> str:
    pins = [
        ("power_in", "VCC", "1", -12.7, 7.62, 0),
        ("power_in", "GND", "2", -12.7, -7.62, 0),
        ("input", "PWM", "3", -12.7, 2.54, 0),
        ("input", "IN1", "4", -12.7, 0, 0),
        ("input", "IN2", "5", -12.7, -2.54, 0),
        ("passive", "OUT1", "6", 12.7, 2.54, 180),
        ("passive", "OUT2", "7", 12.7, -2.54, 180),
    ]
    lines = [f"""  (symbol "BTS7960_Module" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
    (property "Reference" "U" (at 0 12.7 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Value" "BTS7960_Module" (at 0 10.16 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "~" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_description" "BTS7960 H-bridge module (confirm PWM/DIR pinout)" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (symbol "BTS7960_Module_0_1"
      (rectangle (start -10.16 -10.16) (end 10.16 10.16)
        (stroke (width 0.254) (type default)) (fill (type background))
      )
    )
    (symbol "BTS7960_Module_1_1\""""]
    for et, nm, num, x, y, rot in pins:
        lines.append(pin_def(et, nm, num, x, y, rot))
    lines.append("    )\n  )")
    return "\n".join(lines)


def conn_symbol(name: str, n: int, prefix: str = "P") -> str:
    """Vertical connector 1xn pin, pins on left."""
    half_h = (n - 1) * 1.27 / 2 + 2.54
    lines = [f"""  (symbol "{name}" (pin_names (offset 1.016) hide) (in_bom yes) (on_board yes)
    (property "Reference" "J" (at 0 {half_h + 1.27} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Value" "{name}" (at 0 {-half_h - 1.27} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "~" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (symbol "{name}_0_1"
      (rectangle (start 0 {-half_h}) (end 1.27 {half_h})
        (stroke (width 0.254) (type default)) (fill (type background))
      )
    )
    (symbol "{name}_1_1\""""]
    y0 = (n - 1) * 1.27 / 2
    for i in range(n):
        y = y0 - i * 1.27
        lines.append(f"""      (pin passive line (at -2.54 {y} 0) (length 2.54)
        (name "Pin_{i+1}" (effects (font (size 1.016 1.016))))
        (number "{i+1}" (effects (font (size 1.016 1.016))))
      )""")
    lines.append("    )\n  )")
    return "\n".join(lines)


def weact_header_symbol(name: str, n_rows: int = 25) -> str:
    """Dual-row 2.54 mm header abstract (odd/even)."""
    half_h = (n_rows - 1) * 1.27 / 2 + 1.27
    lines = [f"""  (symbol "{name}" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
    (property "Reference" "J" (at 0 {half_h + 2.54} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Value" "{name}" (at 0 {-half_h - 2.54} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "Connector_PinHeader_2.54mm:PinHeader_2x{n_rows}_P2.54mm_Vertical" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "docs/hw/weact_h562_schematic.md" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (symbol "{name}_0_1"
      (rectangle (start -1.27 {-half_h}) (end 1.27 {half_h})
        (stroke (width 0.254) (type default)) (fill (type background))
      )
    )
    (symbol "{name}_1_1\""""]
    y0 = (n_rows - 1) * 1.27 / 2
    for i in range(n_rows):
        y = y0 - i * 1.27
        # odd left, even right (Odd_Even convention)
        odd = 2 * i + 1
        even = 2 * i + 2
        lines.append(f"""      (pin passive line (at -5.08 {y} 0) (length 3.81)
        (name "P{odd}" (effects (font (size 0.762 0.762))))
        (number "{odd}" (effects (font (size 0.762 0.762))))
      )
      (pin passive line (at 5.08 {y} 180) (length 3.81)
        (name "P{even}" (effects (font (size 0.762 0.762))))
        (number "{even}" (effects (font (size 0.762 0.762))))
      )""")
    lines.append("    )\n  )")
    return "\n".join(lines)


def build_symbol_lib() -> str:
    parts = [f"(kicad_symbol_lib (version {SYM_VER}) (generator openems_gen)"]
    for n in ["VBAT", "VBAT_RAW", "+5V_MAIN", "+5V_SENS_A", "+5V_SENS_B",
              "+3V3", "VDDA", "VREF_P", "+5V_CAN"]:
        parts.append(power_sym(n))
    for n in ["PGND", "AGND", "SGND", "SHIELD_GND"]:
        parts.append(gnd_sym(n))
    parts.append(tle8888_symbol())
    parts.append(bts7960_symbol())
    parts.append(conn_symbol("AMPSEAL_23", 23))
    parts.append(conn_symbol("AMPSEAL_35", 35))
    parts.append(weact_header_symbol("WeAct_P1", 25))
    parts.append(weact_header_symbol("WeAct_P2", 25))
    # USB isolator placeholder
    parts.append("""  (symbol "USB_Isolator" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
    (property "Reference" "U" (at 0 10.16 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Value" "USB_Isolator" (at 0 7.62 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" "" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "Datasheet" "~" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (property "ki_description" "USB 2.0 isolator (ADuM3160 class)" (at 0 0 0)
      (effects (font (size 1.27 1.27)) hide)
    )
    (symbol "USB_Isolator_0_1"
      (rectangle (start -7.62 -7.62) (end 7.62 7.62)
        (stroke (width 0.254) (type default)) (fill (type background))
      )
    )
    (symbol "USB_Isolator_1_1"
      (pin bidirectional line (at -10.16 2.54 0) (length 2.54)
        (name "UD+" (effects (font (size 1.016 1.016))))
        (number "1" (effects (font (size 1.016 1.016))))
      )
      (pin bidirectional line (at -10.16 0 0) (length 2.54)
        (name "UD-" (effects (font (size 1.016 1.016))))
        (number "2" (effects (font (size 1.016 1.016))))
      )
      (pin power_in line (at -10.16 -5.08 0) (length 2.54)
        (name "VBUS" (effects (font (size 1.016 1.016))))
        (number "3" (effects (font (size 1.016 1.016))))
      )
      (pin power_in line (at -10.16 -7.62 0) (length 2.54)
        (name "GND_USB" (effects (font (size 1.016 1.016))))
        (number "4" (effects (font (size 1.016 1.016))))
      )
      (pin bidirectional line (at 10.16 2.54 180) (length 2.54)
        (name "D+" (effects (font (size 1.016 1.016))))
        (number "5" (effects (font (size 1.016 1.016))))
      )
      (pin bidirectional line (at 10.16 0 180) (length 2.54)
        (name "D-" (effects (font (size 1.016 1.016))))
        (number "6" (effects (font (size 1.016 1.016))))
      )
      (pin power_in line (at 10.16 -5.08 180) (length 2.54)
        (name "VDD" (effects (font (size 1.016 1.016))))
        (number "7" (effects (font (size 1.016 1.016))))
      )
      (pin power_in line (at 10.16 -7.62 180) (length 2.54)
        (name "GND" (effects (font (size 1.016 1.016))))
        (number "8" (effects (font (size 1.016 1.016))))
      )
    )
  )""")
    parts.append(")")
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# Sheet bodies
# ---------------------------------------------------------------------------

def sheet_01_power() -> str:
    body = []
    body.append(text_block(
        "SHEET 01 — ALIMENTAÇÃO\\n"
        "J2.1-2 VBAT_RAW → P-FET reverse → F1 30A → VBAT\\n"
        "VBAT → TLE BAT (sheet 04) + buck ≥500 kHz → +5V_MAIN → LDO → +3V3\\n"
        "VDDA via ferrite; trackers 5 V NÃO saem daqui (TLE T5V1/T5V2)\\n"
        "Ref: docs/hw/schematic/01_power.md | Rejeitar AMS1117 / LM2596",
        (25.4, 25.4), 1.524,
    ))
    # Power chain left → right
    y = 80
    body.append(global_label("VBAT_RAW", (40, y), 0, "input"))
    body.append(text_block("← J2 pos 1-2", (40, y - 8), 1.0))
    body.append(wire((40, y), (50.8, y)))
    body.append(symbol_inst("Device:Q_PMOS_GSD", "P-FET rev", "Q1", (63.5, y),
                            pins=["1", "2", "3"], footprint="Package_TO_SOT_SMD:SOT-23"))
    body.append(wire((76.2, y), (88.9, y)))
    body.append(symbol_inst("Device:Fuse", "30A", "F1", (101.6, y),
                            pins=["1", "2"], footprint="Fuse:Fuse_1206_3216Metric"))
    body.append(wire((114.3, y), (127, y)))
    body.append(global_label("VBAT", (127, y), 0, "output"))
    body.append(text_block("→ TLE BAT + buck", (127, y - 8), 1.0))

    # TVS on VBAT
    body.append(wire((120, y), (120, y + 15.24)))
    body.append(symbol_inst("Device:D_TVS", "SMBJ24CA", "D1", (120, y + 22.86),
                            pins=["1", "2"], footprint="Diode_SMD:D_SMB"))
    body.append(wire((120, y + 30.48), (120, y + 38.1)))
    body.append(global_label("PGND", (120, y + 38.1), 270, "passive"))

    # Bulk
    body.append(symbol_inst("Device:C_Polarized", "100u", "C1", (140, y + 15.24),
                            pins=["1", "2"], footprint="Capacitor_SMD:CP_Elec_6.3x5.8"))
    body.append(symbol_inst("Device:C", "100n", "C2", (152.4, y + 15.24),
                            pins=["1", "2"], footprint="Capacitor_SMD:C_0805_2012Metric"))
    body.append(global_label("VBAT", (140, y + 5.08), 90, "passive"))
    body.append(global_label("PGND", (140, y + 25.4), 270, "passive"))

    # Buck
    body.append(text_block("Buck ≥500 kHz (TPS54302 class)", (50.8, 130), 1.27))
    body.append(symbol_inst("Regulator_Switching:TPS54302", "TPS54302", "U1",
                            (80, 150), pins=["1", "2", "3", "4", "5", "6"]))
    body.append(global_label("VBAT", (55, 147.46), 0, "input"))
    body.append(global_label("+5V_MAIN", (105, 150), 0, "output"))
    body.append(global_label("PGND", (80, 165), 270, "passive"))

    # LDO 3V3
    body.append(text_block("LDO high-PSRR → +3V3 / VDDA", (150, 130), 1.27))
    body.append(symbol_inst("Regulator_Linear:AP2204RA-3.3", "AP2204RA-3.3", "U2",
                            (180, 150), pins=["1", "2", "3"]))
    body.append(global_label("+5V_MAIN", (155, 150), 0, "input"))
    body.append(global_label("+3V3", (210, 150), 0, "output"))

    # Ferrite VDDA
    body.append(symbol_inst("Device:FerriteBead", "600R@100M", "FB1", (240, 150),
                            pins=["1", "2"], footprint="Inductor_SMD:L_0805_2012Metric"))
    body.append(global_label("+3V3", (225, 150), 0, "passive"))
    body.append(global_label("VDDA", (255, 150), 0, "output"))
    body.append(global_label("VREF_P", (255, 160), 0, "output"))
    body.append(text_block("VREF_P = VDDA (opção a)\\nAGND star → sheet 09", (225, 170), 1.27))
    body.append(global_label("AGND", (240, 175), 270, "passive"))

    body.append(text_block(
        "CHECKLIST:\\n"
        "[ ] Reverse polarity só na entrada\\n"
        "[ ] Trackers 5 V no sheet 04 (TLE)\\n"
        "[ ] Star PGND/AGND no sheet 09\\n"
        "[ ] MR main-relay footprint DNP (sheet 04/09)",
        (50.8, 200), 1.27,
    ))
    return "\n".join(body)


def sheet_02_ckp() -> str:
    body = []
    body.append(text_block(
        "SHEET 02 — CKP (VR via TLE8888)\\n"
        "J1.CKP+/− → TLE VRIN1/2 → VROUT → R0 0Ω → MCU.PA0\\n"
        "Shield single-ended ECU. Sem MAX9924.\\n"
        "Ref: docs/hw/schematic/02_ckp.md",
        (25.4, 25.4), 1.524,
    ))
    # VR path labels + TLE unit G
    body.append(global_label("CKP_P", (50.8, 80), 0, "input"))
    body.append(global_label("CKP_N", (50.8, 90), 0, "input"))
    body.append(global_label("CKP_SHLD", (50.8, 100), 0, "input"))
    body.append(global_label("SHIELD_GND", (80, 100), 0, "passive"))

    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", (130, 90),
                            unit=7,  # G = VR
                            pins=["52", "51", "21"]))
    body.append(global_label("CKP_P", (100, 87.46), 0, "passive"))
    body.append(global_label("CKP_N", (100, 90), 0, "passive"))
    body.append(global_label("CKP_DIG", (160, 88.73), 0, "output"))

    body.append(symbol_inst("Device:R", "0R", "R0", (190, 88.73),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(global_label("MCU_PA0", (220, 88.73), 0, "output"))
    body.append(text_block(
        "TP-VR+/− nos nós VRIN (bancada)\\n"
        "TP-DIG em VROUT (estimulador digital)\\n"
        "R0 DNP = bancada ESP32 no TP-DIG",
        (50.8, 130), 1.27,
    ))
    body.append(global_label("AGND", (130, 120), 270, "passive"))
    return "\n".join(body)


def sheet_03_cmp() -> str:
    body = []
    body.append(text_block(
        "SHEET 03 — CMP (Hall)\\n"
        "Sensor Hall open-collector → pull-up 5V_SENS → divider/clamp → MCU\\n"
        "Ref: docs/hw/schematic/03_cmp.md",
        (25.4, 25.4), 1.524,
    ))
    body.append(global_label("CMP_SIG", (50.8, 80), 0, "input"))
    body.append(global_label("CMP_5V", (50.8, 70), 0, "bidirectional"))
    body.append(global_label("CMP_GND", (50.8, 90), 0, "passive"))
    body.append(global_label("+5V_SENS_A", (80, 70), 0, "passive"))
    body.append(global_label("SGND", (80, 90), 0, "passive"))

    body.append(symbol_inst("Device:R", "1k", "R10", (110, 80),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(global_label("+5V_SENS_A", (110, 65), 90, "passive"))
    body.append(text_block("pull-up Hall", (120, 65), 1.0))

    body.append(symbol_inst("Device:R", "10k", "R11", (150, 80),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:R", "15k", "R12", (150, 100),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:C", "100n", "C10", (175, 100),
                            pins=["1", "2"], footprint="Capacitor_SMD:C_0805_2012Metric"))
    body.append(symbol_inst("Device:D_TVS", "3V3", "D10", (200, 100),
                            pins=["1", "2"], footprint="Diode_SMD:D_SOD-323"))
    body.append(global_label("MCU_PA1", (230, 80), 0, "output"))
    body.append(global_label("AGND", (175, 115), 270, "passive"))
    body.append(global_label("SGND", (150, 115), 270, "passive"))
    body.append(text_block(
        "Confirmar pino MCU CMP no pinout.md (PA1 típico Hall).\\n"
        "CMP2 reservado no AMPSEAL (sheet 09).",
        (50.8, 140), 1.27,
    ))
    return "\n".join(body)


def _gl_wire(body: list, net: str, pin_xy: tuple[float, float],
             label_xy: tuple[float, float], shape: str = "bidirectional") -> None:
    """Global label + wire to a pin coordinate (approximate)."""
    body.append(global_label(net, label_xy, 0, shape))
    body.append(wire(label_xy, pin_xy))


def sheet_04_tle() -> str:
    """TLE hub — labels align with netlist_v1 + tle8888_pinout.md."""
    body = []
    body.append(text_block(
        "SHEET 04 — TLE8888-2QK HUB (direct-drive)\\n"
        "SPI ★ SIN→AGND FCLN→+3V3 | OUT1-4 A+B shorted | OUT5/6 A+B+C | BATPA=BATPB\\n"
        "Ref: docs/hw/schematic/04_tle8888_hub.md | tle8888_pinout.md | netlist_v1.md",
        (25.4, 15.24), 1.524,
    ))

    # --- Unit A Power ---
    body.append(text_block("Unit A — Power / Trackers (p54 BAT, p9/10 T5V, p20 VDDIO)", (30, 40), 1.27))
    ua = (110, 75)
    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", ua, unit=1,
                            pins=["54", "87", "90", "53", "20", "45", "49",
                                  "9", "10", "11", "41", "95", "55", "1", "2",
                                  "25", "50", "75", "100"]))
    # Left power ins (approx pin y from symbol layout)
    for net, dy in [("VBAT", -10.16), ("VBAT", -7.62), ("VBAT", -5.08), ("+3V3", 2.54)]:
        _gl_wire(body, net, (ua[0] - 17.78, ua[1] + dy), (ua[0] - 40, ua[1] + dy), "passive")
    _gl_wire(body, "+5V_SENS_A", (ua[0] + 17.78, ua[1] - 10.16), (ua[0] + 40, ua[1] - 10.16), "output")
    _gl_wire(body, "+5V_SENS_B", (ua[0] + 17.78, ua[1] - 7.62), (ua[0] + 40, ua[1] - 7.62), "output")
    _gl_wire(body, "MAIN_RLY", (ua[0] + 17.78, ua[1] + 5.08), (ua[0] + 40, ua[1] + 5.08), "output")
    body.append(text_block("MR DNP v1", (ua[0] + 45, ua[1] + 7), 1.0))
    # Charge pump cap note
    body.append(symbol_inst("Device:C", "100n_CP", "C_CP", (ua[0] + 50, ua[1] + 2.54),
                            pins=["1", "2"], footprint="Capacitor_SMD:C_0805_2012Metric"))
    body.append(global_label("PGND", (ua[0] + 50, ua[1] + 12), 270, "passive"))
    body.append(global_label("PGND", (ua[0], ua[1] + 20), 270, "passive"))
    body.append(global_label("AGND", (ua[0] - 5, ua[1] + 20), 270, "passive"))
    # Local bulk trackers
    body.append(symbol_inst("Device:C", "100n", "C_T5A", (ua[0] + 55, ua[1] - 10.16),
                            pins=["1", "2"], footprint="Capacitor_SMD:C_0805_2012Metric"))
    body.append(symbol_inst("Device:C", "100n", "C_T5B", (ua[0] + 55, ua[1] - 5),
                            pins=["1", "2"], footprint="Capacitor_SMD:C_0805_2012Metric"))

    # --- Unit B SPI with explicit straps ---
    body.append(text_block("Unit B — SPI2 single-ended ★ STRAPS OBRIGATÓRIOS", (200, 40), 1.27))
    ub = (270, 75)
    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", ub, unit=2,
                            pins=["3", "4", "5", "6", "7", "8"]))
    # pin y: CSN +5.08, SDO right, SIP +2.54, SIN 0, FCLP -2.54, FCLN -5.08
    _gl_wire(body, "MCU_PB12", (ub[0] - 12.7, ub[1] - 5.08), (ub[0] - 45, ub[1] - 5.08), "input")  # CSN
    _gl_wire(body, "MCU_PB15", (ub[0] - 12.7, ub[1] - 2.54), (ub[0] - 45, ub[1] - 2.54), "input")  # SIP MOSI
    _gl_wire(body, "MCU_PB13", (ub[0] - 12.7, ub[1] + 2.54), (ub[0] - 45, ub[1] + 2.54), "input")  # FCLP SCK
    _gl_wire(body, "MCU_PB14", (ub[0] + 12.7, ub[1] - 5.08), (ub[0] + 40, ub[1] - 5.08), "output")  # SDO MISO
    # SIN → AGND strap
    _gl_wire(body, "AGND", (ub[0] - 12.7, ub[1]), (ub[0] - 45, ub[1]), "passive")
    # FCLN → +3V3 strap
    _gl_wire(body, "+3V3", (ub[0] - 12.7, ub[1] + 5.08), (ub[0] - 45, ub[1] + 5.08), "passive")
    body.append(text_block(
        "p6 SIN = AGND\\np8 FCLN = +3V3/VDDIO\\nsem isto SPI morto (MSC/LVDS)",
        (ub[0] - 45, ub[1] + 15), 1.0,
    ))

    # --- Unit C Injectors ---
    body.append(text_block("Unit C — INJ DD: PE0/2/4/6 + PE1 INJEN → OUT1-4 A+B", (30, 115), 1.27))
    uc = (110, 160)
    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", uc, unit=3,
                            pins=["24", "28", "29", "30", "31",
                                  "59", "60", "61", "62", "63", "64", "65", "66"]))
    for net, dy in [("MCU_PE1", -10.16), ("MCU_PE0", -7.62), ("MCU_PE2", -5.08),
                    ("MCU_PE4", -2.54), ("MCU_PE6", 0)]:
        _gl_wire(body, net, (uc[0] - 15.24, uc[1] + dy), (uc[0] - 45, uc[1] + dy), "input")
    # OUT pairs shorted on same net
    for net, dy_a, dy_b in [("INJ1", -7.62, -5.08), ("INJ2", -2.54, 0),
                            ("INJ3", 2.54, 5.08), ("INJ4", 7.62, 10.16)]:
        _gl_wire(body, net, (uc[0] + 15.24, uc[1] + dy_a), (uc[0] + 40, uc[1] + dy_a), "output")
        body.append(wire((uc[0] + 15.24, uc[1] + dy_b), (uc[0] + 30, uc[1] + dy_b)))
        body.append(wire((uc[0] + 30, uc[1] + dy_b), (uc[0] + 30, uc[1] + dy_a)))

    # --- Unit D Ignition ---
    body.append(text_block("Unit D — IGN gate: PE9/11/13/15 + PE3 IGNEN → IGN1-4", (200, 115), 1.27))
    ud = (280, 160)
    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", ud, unit=4,
                            pins=["27", "32", "33", "34", "35", "96", "97", "98", "99"]))
    for net, dy in [("MCU_PE3", -7.62), ("MCU_PE9", -5.08), ("MCU_PE11", -2.54),
                    ("MCU_PE13", 0), ("MCU_PE15", 2.54)]:
        _gl_wire(body, net, (ud[0] - 15.24, ud[1] + dy), (ud[0] - 45, ud[1] + dy), "input")
    for net, dy in [("IGN1", -5.08), ("IGN2", -2.54), ("IGN3", 0), ("IGN4", 2.54)]:
        _gl_wire(body, net, (ud[0] + 15.24, ud[1] + dy), (ud[0] + 40, ud[1] + dy), "output")

    # --- Unit E Relays/VVT ---
    body.append(text_block("Unit E — PE10/12 pump/fan, PB6/7 VVT; OUT5/6 A+B+C shorted", (30, 210), 1.27))
    ue = (110, 250)
    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", ue, unit=5,
                            pins=["36", "37", "38", "39",
                                  "68", "67", "83", "84", "85", "92", "93", "94"]))
    for net, dy in [("MCU_PE10", -10.16), ("MCU_PE12", -7.62), ("MCU_PB6", -5.08), ("MCU_PB7", -2.54)]:
        _gl_wire(body, net, (ue[0] - 17.78, ue[1] + dy), (ue[0] - 50, ue[1] + dy), "input")
    _gl_wire(body, "PUMP_RLY", (ue[0] + 17.78, ue[1] - 10.16), (ue[0] + 45, ue[1] - 10.16), "output")
    _gl_wire(body, "FAN_RLY", (ue[0] + 17.78, ue[1] - 7.62), (ue[0] + 45, ue[1] - 7.62), "output")
    # VVT OUT5 short A+B+C
    _gl_wire(body, "VVT_EXH", (ue[0] + 17.78, ue[1] - 2.54), (ue[0] + 45, ue[1] - 2.54), "output")
    body.append(wire((ue[0] + 17.78, ue[1]), (ue[0] + 35, ue[1])))
    body.append(wire((ue[0] + 35, ue[1]), (ue[0] + 35, ue[1] - 2.54)))
    body.append(wire((ue[0] + 17.78, ue[1] + 2.54), (ue[0] + 35, ue[1] + 2.54)))
    body.append(wire((ue[0] + 35, ue[1] + 2.54), (ue[0] + 35, ue[1] - 2.54)))
    _gl_wire(body, "VVT_INT", (ue[0] + 17.78, ue[1] + 5.08), (ue[0] + 45, ue[1] + 5.08), "output")
    body.append(wire((ue[0] + 17.78, ue[1] + 7.62), (ue[0] + 35, ue[1] + 7.62)))
    body.append(wire((ue[0] + 35, ue[1] + 7.62), (ue[0] + 35, ue[1] + 5.08)))
    body.append(wire((ue[0] + 17.78, ue[1] + 10.16), (ue[0] + 35, ue[1] + 10.16)))
    body.append(wire((ue[0] + 35, ue[1] + 10.16), (ue[0] + 35, ue[1] + 5.08)))

    # --- Unit F CAN + 120R ---
    body.append(text_block("Unit F — CAN + terminador 120Ω (jumper JP_CAN)", (200, 210), 1.27))
    uf = (280, 250)
    body.append(symbol_inst("OpenEMS:TLE8888", "TLE8888-2QK", "U3", uf, unit=6,
                            pins=["44", "43", "46", "47", "42", "48"]))
    _gl_wire(body, "MCU_PB9", (uf[0] - 12.7, uf[1] - 2.54), (uf[0] - 45, uf[1] - 2.54), "input")
    _gl_wire(body, "MCU_PB8", (uf[0] - 12.7, uf[1]), (uf[0] - 45, uf[1]), "output")
    _gl_wire(body, "CANH", (uf[0] + 12.7, uf[1] - 2.54), (uf[0] + 50, uf[1] - 2.54), "bidirectional")
    _gl_wire(body, "CANL", (uf[0] + 12.7, uf[1]), (uf[0] + 50, uf[1]), "bidirectional")
    body.append(symbol_inst("Device:R", "120R", "R_CAN", (uf[0] + 35, uf[1] + 12.7),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(text_block("JP_CAN: popular se ECU for extremo do bus", (uf[0] + 20, uf[1] + 25), 1.0))
    body.append(no_connect((uf[0] - 12.7, uf[1] + 2.54)))  # CANWKEN unused
    body.append(no_connect((uf[0] - 12.7, uf[1] + 5.08)))  # WK unused

    body.append(text_block(
        "CHECKLIST: SPI straps | A+B copper short | BATPA=BATPB | INJEN/IGNEN default LOW | "
        "sem +12V coil/inj na ECU | V5VCAN supply por DS",
        (30, 300), 1.27,
    ))
    return "\n".join(body)


def adc_channel(body: list, name: str, mcu: str, x: float, y: float, n: int, rail: str = "A") -> None:
    body.append(global_label(name, (x, y), 0, "input"))
    body.append(symbol_inst("Device:R", "10k", f"R{n}", (x + 25.4, y),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:R", "15k", f"R{n+1}", (x + 25.4, y + 12.7),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:R", "1k", f"R{n+2}", (x + 50.8, y),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:C", "100n", f"C{n}", (x + 63.5, y + 12.7),
                            pins=["1", "2"], footprint="Capacitor_SMD:C_0805_2012Metric"))
    body.append(symbol_inst("Device:D_TVS", "3V3", f"D{n}", (x + 76.2, y + 12.7),
                            pins=["1", "2"], footprint="Diode_SMD:D_SOD-323"))
    body.append(global_label(mcu, (x + 95, y), 0, "output"))
    body.append(global_label("SGND", (x + 25.4, y + 22.86), 270, "passive"))
    body.append(global_label("AGND", (x + 63.5, y + 22.86), 270, "passive"))
    body.append(text_block(f"rail {rail}", (x + 30, y - 6), 1.0))


def sheet_05_analog() -> str:
    body = []
    body.append(text_block(
        "SHEET 05 — ADC conditioning (0.5–4.5 V → ~0.3–2.7 V)\\n"
        "Rede: R1 10k + R2 15k + Rf 1k + C 100n + TVS 3V3\\n"
        "APP1 rail A / APP2 rail B. NTC CLT/IAT pull-up 2k49 → 5V_SENS_A\\n"
        "Ref: docs/hw/schematic/05_analog.md",
        (25.4, 20.32), 1.524,
    ))
    channels = [
        ("MAP", "MCU_PA3", 40, 55, 100, "A"),
        ("APP1", "MCU_PC0", 40, 100, 110, "A"),
        ("APP2", "MCU_PC2", 40, 145, 120, "B"),
        ("ETB_TPS1", "MCU_PA2", 40, 190, 130, "A"),
        ("ETB_TPS2", "MCU_PC5", 180, 55, 140, "B"),
        ("FUEL_PRESS", "MCU_PC4", 180, 100, 150, "B"),
        ("OIL_PRESS", "MCU_PC1", 180, 145, 160, "B"),
        ("TPS_INDEP", "MCU_PA4", 180, 190, 170, "A"),
    ]
    for name, mcu, x, y, n, rail in channels:
        adc_channel(body, name, mcu, x, y, n, rail)

    # NTC
    body.append(text_block("NTC CLT/IAT — pull-up 2k49 a +5V_SENS_A", (40, 240), 1.27))
    body.append(global_label("CLT", (40, 255), 0, "input"))
    body.append(symbol_inst("Device:R", "2k49", "R200", (70, 255),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(global_label("+5V_SENS_A", (70, 240), 90, "passive"))
    body.append(global_label("MCU_PB0", (110, 255), 0, "output"))
    body.append(global_label("IAT", (40, 280), 0, "input"))
    body.append(symbol_inst("Device:R", "2k49", "R201", (70, 280),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(global_label("+5V_SENS_A", (70, 265), 90, "passive"))
    body.append(global_label("MCU_PB1", (110, 280), 0, "output"))

    # VBATT
    body.append(text_block("VBATT 0–18 V → PC3 (Rhi 47k / Rlo 10k)", (180, 240), 1.27))
    body.append(global_label("VBAT", (180, 255), 0, "input"))
    body.append(symbol_inst("Device:R", "47k", "R210", (210, 255),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:R", "10k", "R211", (210, 275),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(global_label("MCU_PC3", (250, 255), 0, "output"))
    body.append(global_label("PGND", (210, 290), 270, "passive"))
    body.append(global_label("KNOCK_SIG", (320, 255), 0, "bidirectional"))
    body.append(text_block("KNOCK_SIG → sheet 10 DNP → MCU_PA5", (320, 265), 1.0))
    return "\n".join(body)


def sheet_06_etb() -> str:
    body = []
    body.append(text_block(
        "SHEET 06 — ETB BTS7960 @ 10 kHz\\n"
        "MCU PE5 PWM / PE7 DIR1 / PE8 DIR2. Feedback TPS no sheet 05.\\n"
        "Confirmar pinout do módulo concreto (RPWM/LPWM vs PWM+DIR).\\n"
        "Ref: docs/hw/schematic/06_etb.md",
        (25.4, 25.4), 1.524,
    ))
    body.append(symbol_inst("OpenEMS:BTS7960_Module", "BTS7960", "U4", (140, 100),
                            pins=["1", "2", "3", "4", "5", "6", "7"]))
    body.append(global_label("VBAT", (100, 90), 0, "input"))
    body.append(global_label("PGND", (100, 110), 0, "passive"))
    body.append(global_label("ETB_PWM", (100, 97.5), 0, "input"))
    body.append(global_label("ETB_DIR1", (100, 100), 0, "input"))
    body.append(global_label("ETB_DIR2", (100, 102.5), 0, "input"))
    body.append(global_label("MCU_PE5", (60, 97.5), 0, "input"))
    body.append(global_label("MCU_PE7", (60, 100), 0, "input"))
    body.append(global_label("MCU_PE8", (60, 102.5), 0, "input"))
    body.append(global_label("ETB_MOTOR_P", (180, 97.5), 0, "output"))
    body.append(global_label("ETB_MOTOR_N", (180, 102.5), 0, "output"))
    body.append(symbol_inst("Device:C_Polarized", "220u", "C400", (140, 140),
                            pins=["1", "2"], footprint="Capacitor_SMD:CP_Elec_8x10"))
    body.append(global_label("VBAT", (140, 125), 90, "passive"))
    body.append(global_label("PGND", (140, 155), 270, "passive"))
    body.append(text_block(
        "Segurança: mola default-closed, corte duro, batente, validar bancada.\\n"
        "Corrente motor NÃO partilha SGND/AGND.",
        (50.8, 180), 1.27,
    ))
    return "\n".join(body)


def sheet_07_flex() -> str:
    body = []
    body.append(text_block(
        "SHEET 07 — Flex fuel → MCU.PB5 (EXTI)\\n"
        "J1 FLEX_12V=VBAT, FLEX_GND=SGND, FLEX_SIG → divisor + TVS.\\n"
        "Ref: docs/hw/schematic/07_flex.md | netlist_v1.md bloco 14",
        (25.4, 25.4), 1.524,
    ))
    body.append(global_label("FLEX_12V", (50.8, 70), 0, "output"))
    body.append(wire((50.8, 70), (80, 70)))
    body.append(global_label("VBAT", (80, 70), 0, "passive"))
    body.append(global_label("FLEX_SIG", (50.8, 90), 0, "input"))
    body.append(symbol_inst("Device:R", "10k", "R300", (90, 90),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:R", "3k3", "R301", (90, 110),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(symbol_inst("Device:R", "10k_PU", "R302", (70, 75),
                            pins=["1", "2"], footprint="Resistor_SMD:R_0805_2012Metric"))
    body.append(global_label("+5V_SENS_B", (70, 60), 90, "passive"))
    body.append(symbol_inst("Device:D_TVS", "3V3", "D300", (130, 110),
                            pins=["1", "2"], footprint="Diode_SMD:D_SOD-323"))
    body.append(global_label("MCU_PB5", (160, 90), 0, "output"))
    body.append(global_label("FLEX_GND", (50.8, 120), 0, "passive"))
    body.append(wire((50.8, 120), (80, 120)))
    body.append(global_label("SGND", (80, 120), 0, "passive"))
    body.append(global_label("AGND", (130, 125), 270, "passive"))
    return "\n".join(body)


def sheet_08_usb() -> str:
    body = []
    body.append(text_block(
        "SHEET 08 — USB isolado (PA11 DM / PA12 DP)\\n"
        "Host USB ↔ isolator ↔ MCU. Isolador junto ao conector.\\n"
        "NÃO alimentar sensores pelo USB 5 V.\\n"
        "Ref: docs/hw/schematic/08_usb.md | netlist_v1.md bloco 16",
        (25.4, 25.4), 1.524,
    ))
    body.append(symbol_inst("OpenEMS:USB_Isolator", "ADuM3160_class", "U5", (140, 100),
                            pins=["1", "2", "3", "4", "5", "6", "7", "8"]))
    _gl_wire(body, "USB_DP_ISO", (127.84, 97.46), (90, 97.46), "bidirectional")
    _gl_wire(body, "USB_DM_ISO", (127.84, 100), (90, 100), "bidirectional")
    _gl_wire(body, "VBUS", (127.84, 105.08), (90, 105.08), "input")
    _gl_wire(body, "USB_GND_ISO", (127.84, 107.62), (90, 107.62), "passive")
    _gl_wire(body, "MCU_PA12", (152.16, 97.46), (190, 97.46), "bidirectional")  # DP
    _gl_wire(body, "MCU_PA11", (152.16, 100), (190, 100), "bidirectional")  # DM
    _gl_wire(body, "+3V3", (152.16, 105.08), (190, 105.08), "passive")
    _gl_wire(body, "AGND", (152.16, 107.62), (190, 107.62), "passive")
    body.append(text_block(
        "USB conector no edge. ESD TVS em D+/D− lado host.\\n"
        "USB_GND_ISO isolado de PGND (sem pour por baixo da barreira).",
        (50.8, 150), 1.27,
    ))
    return "\n".join(body)


def sheet_09_connectors() -> str:
    """Full logical pin maps from netlist_v1.md §§J1/J2 + WeAct critical pins."""
    body = []
    body.append(text_block(
        "SHEET 09 — AMPSEAL + WeAct + star GND\\n"
        "Mapa lógico = netlist_v1.md (cavidade TE 3 filas a validar no PDF layout).\\n"
        "J2 header 1-770669-1 / plug 770680-1 | J1 header 1-776180-1 / plug 776164-1\\n"
        "WeAct V1.0 38.62×69.10 mm — weact_h562_schematic.md",
        (25.4, 12.7), 1.27,
    ))

    # ----- J2 23-pos power -----
    # AMPSEAL symbol: pin length 2.54 left of origin; pin1 at y = +(n-1)*1.27/2
    j2_at = (100.0, 160.0)
    n2 = 23
    body.append(symbol_inst(
        "OpenEMS:AMPSEAL_23", "1-770669-1", "J2", j2_at,
        pins=[str(i) for i in range(1, n2 + 1)],
        footprint="OpenEMS:TE_770669_AMPSEAL_23_RA",
    ))
    # netlist: J2.VBAT+ is battery in → VBAT_RAW into reverse-polarity FET (sheet 01)
    j2_nets = {
        1: "VBAT_RAW", 2: "VBAT_RAW",
        3: "PGND", 4: "PGND", 5: "PGND",
        6: "INJ1", 7: "INJ2", 8: "INJ3", 9: "INJ4",
        10: "IGN1", 11: "IGN2", 12: "IGN3", 13: "IGN4",
        14: "VVT_EXH", 15: "VVT_INT",
        16: "PUMP_RLY", 17: "FAN_RLY", 18: "MAIN_RLY",
        19: "ETB_MOTOR_P", 20: "ETB_MOTOR_P",
        21: "ETB_MOTOR_N", 22: "ETB_MOTOR_N",
        23: "NC_J2_23",
    }
    y0 = j2_at[1] + (n2 - 1) * 1.27 / 2
    pin_x = j2_at[0] - 2.54
    for pin, net in j2_nets.items():
        y = y0 - (pin - 1) * 1.27
        if net.startswith("NC_"):
            body.append(no_connect((pin_x, y)))
            body.append(text_block(f"p{pin} livre", (pin_x - 25, y - 0.5), 0.9))
        else:
            _gl_wire(body, net, (pin_x, y), (pin_x - 30, y), "bidirectional")
            body.append(text_block(f"{pin}", (pin_x - 8, y - 0.8), 0.8))

    body.append(text_block(
        "J2 potência (lógico): 1-2 VBAT_RAW | 3-5 PGND | 6-9 INJ | 10-13 IGN |\\n"
        "14-15 VVT | 16-18 RLY (MAIN DNP) | 19-20 ETB+ | 21-22 ETB- | 23 livre\\n"
        "⚠️ +12V bobina/injetor NÃO entra neste conector",
        (30, 40), 1.0,
    ))

    # ----- J1 35-pos signals -----
    j1_at = (250.0, 175.0)
    n1 = 35
    body.append(symbol_inst(
        "OpenEMS:AMPSEAL_35", "1-776180-1", "J1", j1_at,
        pins=[str(i) for i in range(1, n1 + 1)],
        footprint="OpenEMS:TE_776180_AMPSEAL_35_RA",
    ))
    j1_nets = {
        1: "CKP_P", 2: "CKP_N", 3: "CKP_SHLD",
        4: "CMP_SIG", 5: "CMP_5V", 6: "CMP_GND",
        7: "MAP", 8: "CLT", 9: "IAT",
        10: "APP1", 11: "APP2",
        12: "FUEL_PRESS", 13: "OIL_PRESS",
        14: "ETB_TPS1", 15: "ETB_TPS2",
        16: "+5V_SENS_A", 17: "+5V_SENS_B",
        18: "SGND", 19: "SGND",
        20: "CANH", 21: "CANL", 22: "CAN_SHLD",
        23: "FLEX_12V", 24: "FLEX_SIG", 25: "FLEX_GND",
        26: "KNOCK_SIG", 27: "KNOCK_SHLD",
        28: "CMP2_SIG", 29: "CMP2_5V", 30: "CMP2_GND",
        31: "TPS_INDEP",
        32: "NC_J1_32", 33: "NC_J1_33", 34: "NC_J1_34", 35: "NC_J1_35",
    }
    y1 = j1_at[1] + (n1 - 1) * 1.27 / 2
    pin1_x = j1_at[0] - 2.54
    reserved = {26, 27, 28, 29, 30, 31}
    for pin, net in j1_nets.items():
        y = y1 - (pin - 1) * 1.27
        if net.startswith("NC_"):
            body.append(no_connect((pin1_x, y)))
        elif pin in reserved:
            _gl_wire(body, net, (pin1_x, y), (pin1_x - 35, y), "bidirectional")
            body.append(text_block("DNP/res", (pin1_x - 55, y - 0.5), 0.8))
        else:
            _gl_wire(body, net, (pin1_x, y), (pin1_x - 35, y), "bidirectional")
        body.append(text_block(f"{pin}", (pin1_x - 8, y - 0.8), 0.8))

    # CMP_5V / CMP_GND aliases to rails
    body.append(text_block(
        "Aliases sheet 03: CMP_5V=+5V_SENS_A, CMP_GND=SGND\\n"
        "CKP_SHLD / CAN_SHLD / KNOCK_SHLD → SHIELD_GND no star",
        (280, 40), 1.0,
    ))
    body.append(global_label("CMP_5V", (300, 55), 0, "passive"))
    body.append(wire((300, 55), (330, 55)))
    body.append(global_label("+5V_SENS_A", (330, 55), 0, "passive"))
    body.append(global_label("CMP_GND", (300, 65), 0, "passive"))
    body.append(wire((300, 65), (330, 65)))
    body.append(global_label("SGND", (330, 65), 0, "passive"))
    body.append(global_label("CKP_SHLD", (300, 75), 0, "passive"))
    body.append(wire((300, 75), (340, 75)))
    body.append(global_label("SHIELD_GND", (340, 75), 0, "passive"))
    body.append(global_label("CAN_SHLD", (300, 85), 0, "passive"))
    body.append(wire((300, 85), (340, 85)))
    body.append(global_label("SHIELD_GND", (340, 85), 0, "passive"))

    # ----- WeAct P1/P2 critical OpenEMS pins -----
    body.append(text_block(
        "WeAct P1/P2 — pin map crítico (weact_h562_schematic.md).\\n"
        "Odd/Even dual-row 2.54 mm. Cruzar pin1 com silkscreen físico.",
        (30, 320), 1.27,
    ))
    # Compact label tables instead of full 50-pin connectors (keeps sheet readable)
    # P1 critical
    p1_map = [
        (1, "MCU_PC0", "APP1"), (2, "MCU_PC1", "OIL"),
        (3, "MCU_PC2", "APP2"), (4, "MCU_PC3", "VBATT"),
        (6, "VREF_P", "VREF+"),
        (7, "MCU_PA0", "CKP"), (8, "MCU_PA1", "CMP"),
        (9, "MCU_PA2", "ETB_TPS1"), (10, "MCU_PA3", "MAP"),
        (11, "MCU_PA4", "TPS"), (12, "MCU_PA5", "KNOCK"),
        (15, "MCU_PC4", "FUEL"), (16, "MCU_PC5", "ETB_TPS2"),
        (17, "MCU_PB0", "CLT"), (18, "MCU_PB1", "IAT"),
        (20, "MCU_PE7", "ETB_DIR1"),
        (21, "MCU_PE8", "ETB_DIR2"), (22, "MCU_PE9", "IGN1"),
        (23, "MCU_PE10", "PUMP"), (24, "MCU_PE11", "IGN2"),
        (25, "MCU_PE12", "FAN"), (26, "MCU_PE13", "IGN3"),
        (29, "MCU_PB12", "SPI_CS"), (30, "MCU_PB13", "SPI_SCK"),
        (31, "MCU_PB14", "SPI_MISO"), (32, "MCU_PB15", "SPI_MOSI"),
    ]
    p2_map = [
        (1, "VB_WEACT", "Vin board"),
        (2, "MCU_PE6", "INJ4"),
        (3, "MCU_PE4", "INJ3"), (4, "MCU_PE5", "ETB_PWM"),
        (5, "MCU_PE2", "INJ2"), (6, "MCU_PE3", "IGNEN"),
        (7, "MCU_PE0", "INJ1"),
        (9, "MCU_PB8", "CAN_RX"), (10, "MCU_PB9", "CAN_TX"),
        (11, "MCU_PB6", "VVT_EXH"), (12, "MCU_PB7", "VVT_INT"),
        (27, "MCU_PA10", "UART_RX_TP"), (28, "MCU_PA11", "USB_DM"),
        (29, "MCU_PA8", "—"), (30, "MCU_PA12", "USB_DP"),
    ]
    body.append(text_block("P1 (J3) nets → MCU", (30, 350), 1.27))
    for i, (pin, net, note) in enumerate(p1_map):
        col = i // 14
        row = i % 14
        x = 30 + col * 90
        y = 365 + row * 5.08
        body.append(global_label(net, (x, y), 0, "bidirectional"))
        body.append(text_block(f"P1.{pin} {note}", (x + 25, y - 0.5), 0.85))

    body.append(text_block("P2 (J4) nets → MCU", (220, 350), 1.27))
    for i, (pin, net, note) in enumerate(p2_map):
        y = 365 + i * 5.08
        body.append(global_label(net, (220, y), 0, "bidirectional"))
        body.append(text_block(f"P2.{pin} {note}", (245, y - 0.5), 0.85))

    # Placeholder dual headers (mechanical) without overcrowding nets
    body.append(symbol_inst(
        "OpenEMS:WeAct_P1", "WeAct_P1_2x25", "J3", (400, 420),
        pins=[str(i) for i in range(1, 51)],
        footprint="OpenEMS:WeAct_PinHeader_2x25_P2.54mm",
    ))
    body.append(symbol_inst(
        "OpenEMS:WeAct_P2", "WeAct_P2_2x25", "J4", (450, 420),
        pins=[str(i) for i in range(1, 51)],
        footprint="OpenEMS:WeAct_PinHeader_2x25_P2.54mm",
    ))
    body.append(text_block(
        "J3/J4 footprints mecânicos — ligar pin-a-pin no KiCad\\n"
        "usando a tabela P1/P2 acima e o SchDoc WeAct.",
        (380, 330), 1.0,
    ))

    # ----- Star ground -----
    body.append(text_block(
        "★ STAR GND — único ponto de união\\n"
        "PGND + SGND + AGND + SHIELD_GND\\n"
        "Correntes INJ/IGN/ETB NUNCA atravessam SGND/AGND",
        (30, 470), 1.27,
    ))
    star = (120.0, 520.0)
    for i, net in enumerate(["PGND", "SGND", "AGND", "SHIELD_GND"]):
        x = 60 + i * 40
        body.append(global_label(net, (x, star[1]), 0, "passive"))
        body.append(wire((x, star[1]), star))
    body.append(text_block("STAR", (star[0] - 5, star[1] + 8), 1.524))

    # Power to WeAct
    body.append(text_block(
        "Alimentação coreboard: +3V3 da carrier → headers VDD\\n"
        "(ou VB_WEACT se LDO onboard — escolher uma e documentar)",
        (220, 500), 1.0,
    ))
    body.append(global_label("+3V3", (220, 530), 0, "passive"))
    body.append(global_label("AGND", (260, 530), 0, "passive"))
    return "\n".join(body)


def sheet_10_knock() -> str:
    body = []
    body.append(text_block(
        "SHEET 10 — Knock FOOTPRINT ONLY / DNP v1\\n"
        "J1.26 KNOCK_SIG → (IC DNP) → MCU_PA5 | J1.27 KNOCK_SHLD → SHIELD_GND\\n"
        "Ref: docs/hw/schematic/10_knock_dnp.md | netlist bloco 13",
        (25.4, 25.4), 1.524,
    ))
    body.append(global_label("KNOCK_SIG", (80, 80), 0, "bidirectional"))
    body.append(wire((80, 80), (120, 80)))
    body.append(no_connect((120, 80)))
    body.append(text_block("IC knock DNP", (125, 78), 1.0))
    body.append(wire((140, 80), (180, 80)))
    body.append(global_label("MCU_PA5", (180, 80), 0, "bidirectional"))
    body.append(global_label("KNOCK_SHLD", (80, 100), 0, "bidirectional"))
    body.append(wire((80, 100), (140, 100)))
    body.append(global_label("SHIELD_GND", (140, 100), 0, "passive"))
    body.append(text_block(
        "Não popular CI na v1. Silk 'DNP v1'.\\n"
        "Longe de IGN/VVT no layout. Envelope, não portadora.",
        (80, 130), 1.27,
    ))
    return "\n".join(body)


# ---------------------------------------------------------------------------
# Root hierarchical schematic
# ---------------------------------------------------------------------------

SHEET_DEFS = [
    ("01_power", "01 — Alimentação", "sheets/01_power.kicad_sch"),
    ("02_ckp", "02 — CKP VR", "sheets/02_ckp.kicad_sch"),
    ("03_cmp", "03 — CMP Hall", "sheets/03_cmp.kicad_sch"),
    ("04_tle8888_hub", "04 — TLE8888 Hub", "sheets/04_tle8888_hub.kicad_sch"),
    ("05_analog", "05 — ADC", "sheets/05_analog.kicad_sch"),
    ("06_etb", "06 — ETB", "sheets/06_etb.kicad_sch"),
    ("07_flex", "07 — Flex fuel", "sheets/07_flex.kicad_sch"),
    ("08_usb", "08 — USB isolado", "sheets/08_usb.kicad_sch"),
    ("09_connectors_weact", "09 — Conectores/WeAct", "sheets/09_connectors_weact.kicad_sch"),
    ("10_knock_dnp", "10 — Knock DNP", "sheets/10_knock_dnp.kicad_sch"),
]


def build_root() -> tuple[str, dict[str, str]]:
    """Return root sch text and map sheet_name -> sheet uuid."""
    sheet_uuids = {}
    root_uuid = uid()
    parts = []
    parts.append(f"""(kicad_sch (version {SCH_VER}) (generator openems_gen)

  (uuid {root_uuid})

  (paper "A3")
  (title_block
    (title "OpenEMS Interface Board v1")
    (date "2026-07-20")
    (rev "v1")
    (company "OpenEMS")
    (comment 1 "Hierarquia modular — ver docs/hw/schematic/")
    (comment 2 "MCU: WeAct STM32H562VGT6 | Hub: TLE8888-2QK | AMPSEAL 35+23")
  )

  (lib_symbols
  )

""")
    parts.append(text_block(
        "OpenEMS Interface Board v1 — esquemático hierárquico\\n"
        "Firmware: STM32H562 + TLE8888 driver | Branch: feat/interface-board-v1\\n"
        "\\n"
        "Como usar (sem saber KiCad):\\n"
        "1. Abrir openems_interface_v1.kicad_pro no KiCad 7\\n"
        "2. Duplo-clique num sheet para entrar no módulo\\n"
        "3. Nets globais (MCU_*, VBAT, +3V3, …) ligam entre sheets\\n"
        "4. Autoridade: docs/hw/*.md  |  Netlist: docs/hw/netlist_v1.md\\n"
        "\\n"
        "Estado: componentes + labels prontos; completar fios pin-a-pin e footprints TE.",
        (30, 25), 1.524,
    ))

    # 2 columns x 5 sheets
    for i, (fname, sname, spath) in enumerate(SHEET_DEFS):
        col = i % 2
        row = i // 2
        x = 40 + col * 140
        y = 90 + row * 40
        w, h = 110, 28
        su = uid()
        sheet_uuids[fname] = su
        page = str(i + 2)
        parts.append(f"""  (sheet (at {x} {y}) (size {w} {h})
    (stroke (width 0.1524) (type solid))
    (fill (color 0 0 0 0.0000))
    (uuid {su})
    (property "Sheetname" "{sname}" (at {x} {y - 1.27} 0)
      (effects (font (size 1.524 1.524)) (justify left bottom))
    )
    (property "Sheetfile" "{spath}" (at {x} {y + h + 1.27} 0)
      (effects (font (size 1.27 1.27)) (justify left top))
    )
    (instances
      (project "{PROJECT}"
        (path "/{root_uuid}" (page "{page}"))
      )
    )
  )
""")

    parts.append(f"""  (sheet_instances
    (path "/" (page "1"))
""")
    for i, (fname, _, _) in enumerate(SHEET_DEFS):
        parts.append(f'    (path "/{root_uuid}/{sheet_uuids[fname]}" (page "{i+2}"))\n')
    parts.append("  )\n)\n")
    return "".join(parts), {"root": root_uuid, **sheet_uuids}


def _pcb_module(lib_id: str, ref: str, at: tuple[float, float], rot: int = 0) -> str:
    """Minimal footprint instance (pads resolved by KiCad on open/save)."""
    return f"""  (footprint "{lib_id}" (layer "F.Cu")
    (tstamp {uid()})
    (at {at[0]} {at[1]} {rot})
    (descr "OpenEMS placed outline — run Update from PCB Editor")
    (path "/{uid()}")
    (attr through_hole)
    (fp_text reference "{ref}" (at 0 -4) (layer "F.SilkS")
      (effects (font (size 1 1) (thickness 0.15)))
      (tstamp {uid()})
    )
    (fp_text value "{lib_id.split(':')[-1]}" (at 0 4) (layer "F.Fab")
      (effects (font (size 1 1) (thickness 0.15)))
      (tstamp {uid()})
    )
  )
"""


def build_pcb() -> str:
    """Board outline ≥130 mm connector edge + AMPSEAL/WeAct keepouts.

    Footprints live in libs/OpenEMS.pretty; full pad geometry appears after
    opening in Pcbnew (Update Footprints) or re-import from schematic.
    """
    # Edge length: 60.5 (35) + 8 gap + 50.4 (23) ≈ 119 → project 130 mm
    w, h = 130.0, 100.0
    t1, t2, t3, t4, t5 = uid(), uid(), uid(), uid(), uid()
    return f"""(kicad_pcb (version {PCB_VER}) (generator openems_gen)

  (general
    (thickness 1.6)
  )

  (paper "A4")
  (title_block
    (title "OpenEMS Interface Board v1")
    (rev "v1")
    (company "OpenEMS")
    (comment 1 "AMPSEAL RA same edge; WeAct center; NAO fabricar sem layout")
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

  (net 0 "")

  (gr_line (start 0 0) (end {w} 0)
    (stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (tstamp {t1}))
  (gr_line (start {w} 0) (end {w} {h})
    (stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (tstamp {t2}))
  (gr_line (start {w} {h}) (end 0 {h})
    (stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (tstamp {t3}))
  (gr_line (start 0 {h}) (end 0 0)
    (stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (tstamp {t4}))

  (gr_text "OpenEMS interface v1 — 130x100 mm outline\\nBottom edge: J1 AMPSEAL35 + gap + J2 AMPSEAL23\\nCenter: WeAct 38.62x69.10 keepout\\nFootprints: libs/OpenEMS.pretty — NAO fabricar ainda"
    (at {w/2} 45 0) (layer "Cmts.User") (tstamp {t5})
    (effects (font (size 1.8 1.8) (thickness 0.25)))
  )

  (gr_text "J1 35-pos signals (1-776180-1)" (at 35 92 0) (layer "Dwgs.User") (tstamp {uid()})
    (effects (font (size 1.2 1.2) (thickness 0.15)))
  )
  (gr_text "J2 23-pos power (1-770669-1)" (at 100 92 0) (layer "Dwgs.User") (tstamp {uid()})
    (effects (font (size 1.2 1.2) (thickness 0.15)))
  )
  (gr_text "WeAct keepout 38.62 x 69.10" (at 65 40 0) (layer "Dwgs.User") (tstamp {uid()})
    (effects (font (size 1.2 1.2) (thickness 0.15)))
  )
  (gr_line (start 45.69 5.45) (end 84.31 5.45)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 84.31 5.45) (end 84.31 74.55)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 84.31 74.55) (end 45.69 74.55)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 45.69 74.55) (end 45.69 5.45)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 5 82) (end 70 82)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 70 82) (end 70 98)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 70 98) (end 5 98)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 5 98) (end 5 82)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 78 82) (end 125 82)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 125 82) (end 125 98)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 125 98) (end 78 98)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
  (gr_line (start 78 98) (end 78 82)
    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))
)
"""


def build_pro() -> str:
    """Minimal KiCad 7 project file."""
    pro = {
        "board": {
            "design_settings": {
                "defaults": {},
                "diff_pair_dimensions": [],
                "drc_exclusions": [],
                "meta": {"version": 2},
                "rule_severities": {},
                "rules": {
                    "max_error": 0.005,
                    "min_clearance": 0.0,
                    "min_connection": 0.0,
                    "min_copper_edge_clearance": 0.0,
                    "min_hole_clearance": 0.0,
                    "min_hole_to_hole": 0.25,
                    "min_microvia": 0.2,
                    "min_microvia_drill": 0.1,
                    "min_resolved_spokes": 2,
                    "min_silk_clearance": 0.0,
                    "min_text_height": 0.8,
                    "min_text_thickness": 0.08,
                    "min_through_hole_diameter": 0.3,
                    "min_track_width": 0.0,
                    "min_via": 0.5,
                    "min_via_annular_width": 0.1,
                    "min_via_drill": 0.3,
                    "solder_mask_to_copper_clearance": 0.0,
                    "use_height_for_length_calcs": True,
                },
                "teardrop_options": [],
                "teardrop_parameters": [],
                "track_widths": [0.0],
                "tuning_pattern_settings": {},
                "via_dimensions": [{"diameter": 0.0, "drill": 0.0}],
                "zones_allow_external_fillets": False,
            },
            "layer_presets": [],
            "viewports": [],
        },
        "boards": [],
        "cvpcb": {"equivalence_files": []},
        "erc": {
            "erc_exclusions": [],
            "meta": {"version": 0},
            "pin_map": [
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
                [0, 2, 0, 1, 0, 0, 1, 0, 2, 2, 2, 2],
                [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 2],
                [0, 1, 0, 0, 0, 0, 1, 1, 2, 1, 1, 2],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
                [1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 2],
                [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 2],
                [0, 2, 1, 2, 0, 0, 1, 0, 2, 2, 2, 2],
                [0, 2, 0, 1, 0, 0, 1, 0, 2, 0, 0, 2],
                [0, 2, 1, 1, 0, 0, 1, 0, 2, 0, 0, 2],
                [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
            ],
            "rule_severities": {
                "bus_definition_conflict": "error",
                "bus_entry_needed": "error",
                "bus_to_bus_conflict": "error",
                "bus_to_net_conflict": "error",
                "different_unit_footprint": "error",
                "different_unit_net": "warning",
                "duplicate_reference": "error",
                "duplicate_sheet_names": "error",
                "endpoint_off_grid": "warning",
                "extra_units": "error",
                "global_label_dangling": "warning",
                "hier_label_mismatch": "error",
                "label_dangling": "error",
                "lib_symbol_issues": "warning",
                "missing_bidi_pin": "warning",
                "missing_input_pin": "warning",
                "missing_power_pin": "error",
                "missing_unit": "warning",
                "multiple_net_names": "warning",
                "net_not_bus_member": "warning",
                "no_connect_connected": "warning",
                "no_connect_dangling": "warning",
                "pin_not_connected": "error",
                "pin_not_driven": "error",
                "pin_to_pin": "warning",
                "power_pin_not_driven": "error",
                "similar_labels": "warning",
                "simulation_model_issue": "ignore",
                "unannotated": "error",
                "unit_value_mismatch": "error",
                "unresolved_variable": "error",
                "wire_dangling": "error",
            },
        },
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {
            "filename": f"{PROJECT}.kicad_pro",
            "version": 1,
        },
        "net_settings": {
            "classes": [
                {
                    "bus_width": 12,
                    "clearance": 0.2,
                    "diff_pair_gap": 0.25,
                    "diff_pair_via_gap": 0.25,
                    "diff_pair_width": 0.2,
                    "line_style": 0,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "name": "Default",
                    "pcb_color": "rgba(0, 0, 0, 0.000)",
                    "schematic_color": "rgba(0, 0, 0, 0.000)",
                    "track_width": 0.25,
                    "via_diameter": 0.8,
                    "via_drill": 0.4,
                    "wire_width": 6,
                }
            ],
            "meta": {"version": 3},
            "net_colors": None,
            "netclass_assignments": None,
            "netclass_patterns": [],
        },
        "pcbnew": {
            "last_paths": {
                "gencad": "",
                "idf": "",
                "netlist": "",
                "plot": "",
                "pos_files": "",
                "specctra_dsn": "",
                "step": "",
                "svg": "",
                "vrml": "",
            },
            "page_layout_descr_file": "",
        },
        "schematic": {
            "annotate_start_num": 0,
            "drawing": {
                "dashed_lines_dash_length_ratio": 12.0,
                "dashed_lines_gap_length_ratio": 3.0,
                "default_line_thickness": 6.0,
                "default_text_size": 50.0,
                "field_names": [],
                "intersheets_ref_own_page": False,
                "intersheets_ref_prefix": "",
                "intersheets_ref_short": False,
                "intersheets_ref_show": False,
                "intersheets_ref_suffix": "",
                "junction_size_choice": 3,
                "label_size_ratio": 0.375,
                "pin_symbol_size": 25.0,
                "text_offset_ratio": 0.15,
            },
            "legacy_lib_dir": "",
            "legacy_lib_list": [],
            "meta": {"version": 1},
            "net_format_name": "",
            "page_layout_descr_file": "",
            "plot_directory": "",
            "spice_current_sheet_as_root": False,
            "spice_external_command": 'spice "%I"',
            "spice_model_current_sheet_as_root": True,
            "spice_save_all_currents": False,
            "spice_save_all_voltages": False,
            "subpart_first_id": 65,
            "subpart_id_separator": 0,
        },
        "sheets": [
            ["root", "Root"],
        ],
        "text_variables": {},
    }
    return json.dumps(pro, indent=2) + "\n"


def build_readme() -> str:
    return textwrap.dedent("""\
    # OpenEMS Interface Board v1 — projecto KiCad 7

    ## Abrir (sem saber KiCad)

    1. Instalar **KiCad 7** (já tens 7.0.x).
    2. Duplo-clique em:

       `hardware/kicad/openems_interface_v1/openems_interface_v1.kicad_pro`

       ou: `kicad openems_interface_v1.kicad_pro`
    3. **Schematic Editor** → folha raiz com 10 módulos → duplo-clique para entrar.
    4. **Backspace** para voltar à raiz.

    ## O que está pronto

    | Item | Estado |
    |------|--------|
    | Hierarquia 10 sheets | ✅ |
    | TLE8888 multi-unit A–G + straps SPI + shorts A/B | ✅ |
    | J2 23-pos pin→net completo (netlist) | ✅ lógicos |
    | J1 35-pos pin→net completo (netlist) | ✅ lógicos |
    | WeAct P1/P2 tabela crítica MCU | ✅ labels |
    | Star GND + aliases shield/CMP | ✅ |
    | Flex PB5, USB PA11/12, ADC nets | ✅ |
    | Footprints TE AMPSEAL 23/35 RA | ✅ `OpenEMS.pretty` (TE PDF + padrão KiCad) |
    | Footprint WeAct 2×25 | ✅ |
    | PCB outline 130×100 + keepouts | ✅ (não fabricar) |
    | Fios TLE pin-exact no canvas | ⚠️ labels+fios aprox. — afinar no KiCad |
    | Layout copper / pours | ❌ |

    ## Mapa sheets ↔ docs

    | Sheet | Doc |
    |-------|-----|
    | 01–10 | `docs/hw/schematic/0N_*.md` |
    | pinos J1/J2 | `docs/hw/netlist_v1.md` (secção Conector) |
    | TLE package | `docs/hw/tle8888_pinout.md` |
    | WeAct headers | `docs/hw/weact_h562_schematic.md` |
    | Pinmap KiCad | `resources/pinmap_logical.md` |

    ## Atalhos

    | Ação | Tecla |
    |------|-------|
    | Fio | `W` |
    | Global label | menu Place |
    | Zoom fit | `Home` |
    | ERC | Inspect → Electrical Rules Checker |
    | Annotate | Tools → Annotate Schematic |

    ## Regenerar

    ```bash
    python3 hardware/kicad/openems_interface_v1/scripts/generate_project.py
    ```

    ⚠️ Reescreve os `.kicad_sch`. Commit ou backup antes se editaste à mão.

    ## Próximos passos manuais

    1. Annotate + ERC (muitos avisos de pin não ligado nos headers WeAct são esperados).
    2. Ligar J3/J4 pin-a-pin com a tabela P1/P2 do sheet 09.
    3. Footprints TE (página RECOMMENDED PCB LAYOUT dos PDFs).
    4. Layout: star, pours, CKP longe de potência.
    5. Gerber só depois.

    ## Bibliotecas

    - `libs/OpenEMS.kicad_sym` + `sym-lib-table`
    - Sistema: Device, Regulator_*, …
    """)


# ---------------------------------------------------------------------------
# Embed system/local symbols into lib_symbols (required for kicad-cli / portable)
# ---------------------------------------------------------------------------

_SYS = Path("/usr/share/kicad/symbols")
_LIB_FILES = {
    "Device": _SYS / "Device.kicad_sym",
    "power": _SYS / "power.kicad_sym",
    "Regulator_Switching": _SYS / "Regulator_Switching.kicad_sym",
    "Regulator_Linear": _SYS / "Regulator_Linear.kicad_sym",
    "Connector_Generic": _SYS / "Connector_Generic.kicad_sym",
    "Connector": _SYS / "Connector.kicad_sym",
}


def _extract_symbol_raw(path: Path, name: str) -> str | None:
    """Extract a top-level symbol, following (extends ...) once to the base."""
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8")
    seen: set[str] = set()
    cur = name
    while cur not in seen:
        seen.add(cur)
        pat = rf'\(symbol "{re.escape(cur)}"(?:\s|\()'
        m = re.search(pat, text)
        if not m:
            return None
        start = m.start()
        rest = text[start + 1 :]
        m2 = re.search(r'\n  \(symbol "', rest)
        end = start + 1 + m2.start() if m2 else len(text)
        block = text[start:end].rstrip()
        depth = 0
        for i, c in enumerate(block):
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    block = block[: i + 1]
                    break
        em = re.search(r'\(extends "([^"]+)"\)', block)
        if em:
            cur = em.group(1)
            continue
        return block
    return None


def _embed_lib_id(lib_id: str, openems_path: Path) -> str | None:
    if ":" not in lib_id:
        return None
    lib, name = lib_id.split(":", 1)
    path = openems_path if lib == "OpenEMS" else _LIB_FILES.get(lib)
    if path is None:
        print(f"  WARN: unknown lib {lib}")
        return None
    src = _extract_symbol_raw(path, name)
    if not src:
        print(f"  WARN: missing symbol {lib_id}")
        return None
    src = re.sub(r'^\(symbol "[^"]+"', f'(symbol "{lib_id}"', src, count=1)
    return "\n".join("    " + ln if ln else ln for ln in src.splitlines())


def inject_embedded_symbols(sch_path: Path, openems_path: Path) -> None:
    text = sch_path.read_text(encoding="utf-8")
    ids = sorted(set(re.findall(r'\(lib_id "([^"]+)"\)', text)))
    if not ids:
        return
    embeds = []
    for lid in ids:
        e = _embed_lib_id(lid, openems_path)
        if e:
            embeds.append(e)
    block = "\n".join(embeds)
    new_text, n = re.subn(
        r"\(lib_symbols\s*\n\s*\)",
        f"(lib_symbols\n{block}\n  )",
        text,
        count=1,
    )
    if n == 0:
        new_text, n = re.subn(
            r"\(lib_symbols\n[\s\S]*?\n  \)",
            f"(lib_symbols\n{block}\n  )",
            text,
            count=1,
        )
    if n == 0:
        print(f"  FAIL embed inject {sch_path.name}")
        return
    sch_path.write_text(new_text, encoding="utf-8")
    print(f"  embed {sch_path.name}: {len(embeds)} symbols")


def main() -> None:
    ROOT.mkdir(parents=True, exist_ok=True)
    SHEETS.mkdir(parents=True, exist_ok=True)
    LIBS.mkdir(parents=True, exist_ok=True)
    (ROOT / "scripts").mkdir(parents=True, exist_ok=True)

    openems_path = LIBS / "OpenEMS.kicad_sym"
    openems_path.write_text(build_symbol_lib(), encoding="utf-8")

    (ROOT / "sym-lib-table").write_text(
        """(sym_lib_table
  (version 7)
  (lib (name "OpenEMS")(type "KiCad")(uri "${KIPRJMOD}/libs/OpenEMS.kicad_sym")(options "")(descr "OpenEMS interface board symbols"))
)
""",
        encoding="utf-8",
    )
    (ROOT / "fp-lib-table").write_text(
        """(fp_lib_table
  (version 7)
  (lib (name "OpenEMS")(type "KiCad")(uri "${KIPRJMOD}/libs/OpenEMS.pretty")(options "")(descr "OpenEMS AMPSEAL + WeAct footprints"))
)
""",
        encoding="utf-8",
    )

    sheet_builders = {
        "01_power": sheet_01_power,
        "02_ckp": sheet_02_ckp,
        "03_cmp": sheet_03_cmp,
        "04_tle8888_hub": sheet_04_tle,
        "05_analog": sheet_05_analog,
        "06_etb": sheet_06_etb,
        "07_flex": sheet_07_flex,
        "08_usb": sheet_08_usb,
        "09_connectors_weact": sheet_09_connectors,
        "10_knock_dnp": sheet_10_knock,
    }

    for i, (fname, sname, _) in enumerate(SHEET_DEFS):
        body = sheet_builders[fname]()
        content = sch_header(f"OpenEMS — {sname}", body=body, page=str(i + 2))
        path = SHEETS / f"{fname}.kicad_sch"
        path.write_text(content, encoding="utf-8")
        inject_embedded_symbols(path, openems_path)

    root_sch, _ = build_root()
    root_path = ROOT / f"{PROJECT}.kicad_sch"
    root_path.write_text(root_sch, encoding="utf-8")
    inject_embedded_symbols(root_path, openems_path)

    (ROOT / f"{PROJECT}.kicad_pro").write_text(build_pro(), encoding="utf-8")
    (ROOT / f"{PROJECT}.kicad_pcb").write_text(build_pcb(), encoding="utf-8")
    (ROOT / "LEIA-ME.md").write_text(build_readme(), encoding="utf-8")

    print(f"Generated project in {ROOT}")
    for p in sorted(ROOT.rglob("*")):
        if p.is_file():
            print(f"  {p.relative_to(ROOT)} ({p.stat().st_size} bytes)")


if __name__ == "__main__":
    main()


