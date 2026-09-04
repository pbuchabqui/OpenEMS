# Gerbers — microRusEFI copper + STM32H562

**Intent:** production / bring-up board = [rusefi/hw_microRusEfi](https://github.com/rusefi/hw_microRusEfi) layout
with **H562 drop-in** (6 power pads), **not** OpenEMS VGT6 PE* re-route.

| | |
|--|--|
| PCB source | `hardware/openems_ecu/production/openems_mre_h562.kicad_pcb` |
| Base | `hardware/vendor/hw_microRusEfi/micro_rusEFI.kicad_pcb` |
| MCU | **STM32H562VGT6** LQFP100 (same land as F427) |
| TLE / routing | **mRE original** (PD12–15 IGN, PE14–11 INJ, SPI PD5+PB3/4/5, …) |
| Firmware | `make firmware BOARD=mre` → `openems-mre.bin` |
| Docs | `docs/hw/pinout_mre_bringup.md` |

## H562 electrical deltas vs F427 on this board

| Pad | F427 | H562 treatment |
|-----|------|----------------|
| 19 | VDD | **GND** (VSSA) |
| 20 | VSSA | **/ADC/AGND** (VREF-) |
| 48 | PB11 | **VCAP** + C25 2.2µF → GND |
| 49 | VCAP_1 | **GND** (VSS) |
| 73 | VCAP_2 | **/VDD** (VDDUSB) + C26 bulk |
| 98 | PE1 | **VCAP** + C100 2.2µF → GND |

## DRC at export

- Error violations: **649**
- Unconnected items: **2**

Review `drc.json` before ordering. Baseline mRE may already carry residual DRC;
this is **not** the VGT6 experimental bus (that had 500+ target-net shorts).

## Fab package

2-layer. Upload the zip of this folder (or parent `openems_mre_h562_*.zip`).

## Silk branding

- `microRusEFI` / `RUSEFI.COM` → **OpenEMS**
- `STM32F407VG` → **STM32H562VG**
- rusEFI QR logo removed
- Design history credits retained under OpenEMS / H562 header
