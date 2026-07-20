#!/usr/bin/env python3
"""Place AMPSEAL + WeAct footprints on openems_interface_v1.kicad_pcb.

Reads libs/OpenEMS.pretty and writes a 130x100 mm board with:
  J1 TE 35 RA, J2 TE 23 RA (bottom edge), WeAct keepout + 2x25 headers + MH Φ3.2.
"""
from __future__ import annotations

import re
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRETTY = ROOT / "libs" / "OpenEMS.pretty"
PCB_VER = "20221018"
W, H = 130.0, 100.0


def uid() -> str:
    return str(uuid.uuid4())


def load_mod(name: str) -> str:
    return (PRETTY / f"{name}.kicad_mod").read_text(encoding="utf-8")


def embed_footprint(mod_text: str, lib_id: str, ref: str, at: tuple[float, float], rot: int = 0) -> str:
    m = re.match(r'\(footprint\s+"[^"]+"\s*(.*)\)$', mod_text.strip(), re.S)
    if not m:
        raise ValueError(f"bad mod {lib_id}")
    body = m.group(1)
    body = re.sub(r"^\(version [^)]+\)\s*", "", body)
    body = re.sub(r"^\(generator [^)]+\)\s*", "", body)
    body = body.replace('"REF**"', f'"{ref}"')
    # rusEFI legacy: (fp_text reference REF** …) without quotes
    body = re.sub(r"\(fp_text reference REF\*\*", f'(fp_text reference "{ref}"', body)
    body = re.sub(r"\(fp_text value [^\n(]+", f'(fp_text value "{lib_id.split(":")[-1]}"', body, count=1)
    return (
        f'(footprint "{lib_id}" (layer "F.Cu")\n'
        f"    (tstamp {uid()})\n"
        f"    (at {at[0]} {at[1]} {rot})\n"
        f'    (path "/{uid()}")\n'
        f"{body}\n"
        f")"
    )


def mounting_hole(ref: str, at: tuple[float, float], drill: float = 3.2) -> str:
    return f"""  (footprint "MountingHole:{drill}mm" (layer "F.Cu")
    (tstamp {uid()})
    (at {at[0]} {at[1]})
    (descr "Mounting Hole {drill}mm WeAct")
    (tags "mounting hole")
    (path "/{uid()}")
    (attr through_hole exclude_from_pos_files exclude_from_bom)
    (fp_text reference "{ref}" (at 0 {-drill - 1}) (layer "F.SilkS")
      (effects (font (size 0.8 0.8) (thickness 0.12)))
      (tstamp {uid()})
    )
    (fp_text value "MH{drill}" (at 0 {drill + 1}) (layer "F.Fab")
      (effects (font (size 0.8 0.8) (thickness 0.12)))
      (tstamp {uid()})
    )
    (fp_circle (center 0 0) (end {drill / 2 + 0.5} 0)
      (stroke (width 0.12) (type solid)) (fill none) (layer "F.CrtYd") (tstamp {uid()}))
    (fp_circle (center 0 0) (end {drill / 2 + 0.3} 0)
      (stroke (width 0.12) (type solid)) (fill none) (layer "F.SilkS") (tstamp {uid()}))
    (pad "" np_thru_hole circle (at 0 0) (size {drill} {drill}) (drill {drill}) (layers *.Cu *.Mask) (tstamp {uid()}))
  )
"""


def outline_rect(w: float, h: float) -> str:
    lines = []
    corners = [(0, 0), (w, 0), (w, h), (0, h), (0, 0)]
    for a, b in zip(corners, corners[1:]):
        lines.append(
            f"  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]})\n"
            f'    (stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (tstamp {uid()}))'
        )
    return "\n".join(lines)


def gr_text(s: str, at: tuple[float, float], layer: str = "Cmts.User", size: float = 1.5) -> str:
    return f"""  (gr_text "{s}" (at {at[0]} {at[1]} 0) (layer "{layer}") (tstamp {uid()})
    (effects (font (size {size} {size}) (thickness 0.2)))
  )"""


def build() -> str:
    # J1: rusEFI AMPSEAL_35 origin is geometric center (pad1 at -22,-4).
    # Place so pad1 ≈ (10, 82.5): at = (10+22, 82.5+4) = (32, 86.5)
    j1_x, j1_y = 32.0, 86.5
    # J2: our TE_770669 origin at pad1; keep pin1 near (78, 82.5)
    j2_x, j2_y = 78.0, 82.5
    weact_sw = ((W - 38.62) / 2, 8.0)
    hdr_y = weact_sw[1] + 4.0
    j3_x = weact_sw[0] - 2.54
    j4_x = weact_sw[0] + 38.62 + 2.54
    holes = [
        (weact_sw[0] + 2.80, weact_sw[1] + 2.80),
        (weact_sw[0] + 33.28, weact_sw[1] + 2.80),
        (weact_sw[0] + 2.80, weact_sw[1] + 66.30),
        (weact_sw[0] + 33.28, weact_sw[1] + 66.30),
    ]
    # Star NetTie near WeAct SW (outside keepout)
    nettie_at = (weact_sw[0] - 8.0, weact_sw[1] + 2.0)

    parts = [
        # Prefer rusEFI production footprint for J1 35 (tags 776180 / 1-776180)
        embed_footprint(
            load_mod("rusEFI_AMPSEAL_35_RA_776180"),
            "OpenEMS:rusEFI_AMPSEAL_35_RA_776180",
            "J1",
            (j1_x, j1_y),
        ),
        embed_footprint(load_mod("TE_770669_AMPSEAL_23_RA"), "OpenEMS:TE_770669_AMPSEAL_23_RA", "J2", (j2_x, j2_y)),
        embed_footprint(load_mod("WeAct_PinHeader_2x25_P2.54mm"), "OpenEMS:WeAct_PinHeader_2x25_P2.54mm", "J3", (j3_x, hdr_y)),
        embed_footprint(load_mod("WeAct_PinHeader_2x25_P2.54mm"), "OpenEMS:WeAct_PinHeader_2x25_P2.54mm", "J4", (j4_x, hdr_y)),
        embed_footprint(
            load_mod("NetTie-4_THT_Pad1.0mm"),
            "OpenEMS:NetTie-4_THT_Pad1.0mm",
            "NT1",
            nettie_at,
        ),
    ]
    for i, (hx, hy) in enumerate(holes, 1):
        parts.append(mounting_hole(f"H{i}", (hx, hy), 3.2))

    wx0, wy0 = weact_sw
    wx1, wy1 = wx0 + 38.62, wy0 + 69.10
    keep = []
    for a, b in [((wx0, wy0), (wx1, wy0)), ((wx1, wy0), (wx1, wy1)), ((wx1, wy1), (wx0, wy1)), ((wx0, wy1), (wx0, wy0))]:
        keep.append(
            f"  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]})\n"
            f'    (stroke (width 0.15) (type dash)) (layer "Dwgs.User") (tstamp {uid()}))'
        )

    return f"""(kicad_pcb (version {PCB_VER}) (generator openems_pcb)

  (general
    (thickness 1.6)
  )

  (paper "A4")
  (title_block
    (title "OpenEMS Interface Board v1")
    (date "2026-07-20")
    (rev "v1")
    (company "OpenEMS")
    (comment 1 "J1+J2 AMPSEAL RA bottom edge; WeAct center; NAO fabricar sem layout copper")
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

{outline_rect(W, H)}

{chr(10).join(keep)}

{gr_text("OpenEMS interface v1 — 130x100 mm\\\\nBottom: J1 35 + J2 23 AMPSEAL RA\\\\nCenter: WeAct 38.62x69.10 + MH Phi3.2\\\\nNAO fabricar sem copper/GND pours", (W / 2, 50), size=1.6)}
{gr_text("J1 signals 1-776180-1", (j1_x + 20, 78), "Dwgs.User", 1.1)}
{gr_text("J2 power 1-770669-1", (j2_x + 10, 78), "Dwgs.User", 1.1)}
{gr_text("WeAct keepout", (wx0 + 8, wy0 + 35), "Dwgs.User", 1.1)}

{chr(10).join(parts)}
)
"""


def main() -> None:
    out = ROOT / "openems_interface_v1.kicad_pcb"
    out.write_text(build(), encoding="utf-8")
    print(f"Wrote {out} ({out.stat().st_size} bytes)")
    # Pad nets + PGND zones from authoritative pinmap
    from annotate_pcb_nets import main as annotate

    annotate()


if __name__ == "__main__":
    main()
