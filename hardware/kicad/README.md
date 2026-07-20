# OpenEMS — KiCad hardware

## Interface board v1

**Project:** `openems_interface_v1/openems_interface_v1.kicad_pro`  
**Guide:** `openems_interface_v1/LEIA-ME.md`  
**Pinmap:** `openems_interface_v1/resources/pinmap_logical.md`  
**Vendor review (Speeduino/rusEFI):** `../docs/hw/kicad_vendor_review.md`

### Rebuild PCB (layout limpo)

```bash
bash hardware/kicad/openems_interface_v1/scripts/build_all.sh
```

Isto corre `layout_clean.py`: zonas organizadas + **só bridges locais**  
(sem auto-route emaranhado). Routing fino = manual no Pcbnew.

### Status

| Area | State |
|------|--------|
| Hierarchical schematic (10 sheets) | ✅ |
| Placement J1/J2/WeAct/TLE/power/LDO | ✅ limpo |
| Pad nets (pinmap) | ✅ |
| Local bridges (A+B, dual pins, power chain) | ✅ |
| PGND pours | ✅ Fill Zones |
| Full signal routing | ❌ hand-route |
| Fab | ❌ DRC + pin1 first |

⚠️ **Do not order PCBs** without manual DRC and pin1 verification.
