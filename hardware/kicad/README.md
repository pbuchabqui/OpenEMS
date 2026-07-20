# OpenEMS — KiCad hardware

## Interface board v1

**Project:** `openems_interface_v1/openems_interface_v1.kicad_pro`  
**Guide:** `openems_interface_v1/LEIA-ME.md`  
**Pinmap:** `openems_interface_v1/resources/pinmap_logical.md`  
**Vendor review (Speeduino/rusEFI):** `../docs/hw/kicad_vendor_review.md`

### Rebuild PCB copper scaffold

```bash
bash hardware/kicad/openems_interface_v1/scripts/build_all.sh
```

Pipeline: placement → annotate nets → power routes → TLE/power stage → LDO/WeAct finish.

### Status (scaffold)

| Area | State |
|------|--------|
| Hierarchical schematic (10 sheets) | ✅ |
| AMPSEAL J1 (rusEFI) + J2 (TE) | ✅ |
| WeAct headers + MH + NetTie star | ✅ |
| TLE8888 LQFP-100 + power chain | ✅ |
| LDO 3V3 + VDDA ferrite | ✅ |
| SPI/CAN/drive stubs WeAct↔TLE | ✅ |
| PGND pours | ✅ (fill in Pcbnew) |
| Production DRC / fab | ❌ manual review required |

⚠️ **Do not order PCBs** until pin1 silk, FET pinout, buck FB network, and full DRC are verified by hand.
