# Excluded — experimental VGT6 PCB (do not fab)

Parked at user request (2026-08-04).

These Gerbers / board snapshot came from the **OpenEMS VGT6 PE\* remap +
manual bus routing** experiment on `openems_ecu.kicad_pcb`. High DRC error
count; **not** the production path.

## Active production path (use this)

| | |
|--|--|
| PCB | `hardware/openems_ecu/openems_ecu.kicad_pcb` (= mRE copper + H562 pads) |
| Also | `hardware/openems_ecu/production/openems_mre_h562.kicad_pcb` |
| Gerbers | `hardware/openems_ecu/gerbers/mre_h562/` |
| ZIP | `hardware/openems_ecu/gerbers/openems_mre_h562_*.zip` |
| Firmware | `make firmware BOARD=mre` |
| Pinout | `docs/hw/pinout_mre_bringup.md` |

## Restore experimental board (if needed)

```bash
cp -a hardware/openems_ecu/openems_ecu.kicad_pcb.bak-experimental-vgt6-* \
      hardware/openems_ecu/openems_ecu.kicad_pcb
```
