# Gerbers — openems_ecu (STM32H562 / LQFP100)

**Source:** `hardware/openems_ecu/openems_ecu.kicad_pcb`  
**Stackup:** 2-layer (F.Cu / B.Cu)  
**Generated:** via `kicad-cli` (KiCad 10)

## ⚠️ Fabrication readiness

This export is from the **current working tree** board, which includes:

- H562 power pad rework (VCAP, VDDUSB, VSSA, …)
- Partial OpenEMS VGT6 net renames and experimental MCU–TLE routing

**DRC at export:** 566 error-level violations, 50 unconnected items.

**Do not order production PCBs until DRC is clean** (or you accept scrap risk).  
The `BOARD=mre` firmware and `docs/hw/pinout_mre_bringup.md` were removed; no current
firmware build matches this copper.

## Contents

- `*.gbr` / KiCad extensions — copper, mask, paste, silk, edge, fab
- Drill Excellon + map PDF
- `openems_ecu-all-pos.csv` — pick & place
- `openems_ecu-drc.json` — full DRC report

## Typical fab upload

Zip this directory and upload to JLCPCB / PCBWay / etc. as a **2-layer** board.
