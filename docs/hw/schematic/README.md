# Esquemático modular — OpenEMS interface board v1

## Porque módulos e não “um PDF gigante”

O netlist (`../netlist_v1.md`) diz **o que liga a quê**. Estes ficheiros dizem **como desenhar
cada bloco no KiCad**: topologia, valores, refs Speeduino/rusEFI, traps, e lista de
componentes. Um sheet por módulo = revisão barata e paralelizável.

| Ordem | Sheet | Ficheiro | Estado |
|------:|-------|----------|--------|
| 0 | Raiz / hierarquia | este README | ✅ |
| 1 | Alimentação | `01_power.md` | ✅ |
| 2 | CKP (VR / TLE) | `02_ckp.md` | ✅ |
| 3 | CMP (Hall) | `03_cmp.md` | ✅ |
| 4 | Hub TLE8888 (SPI, INJ, IGN, VVT, relés, CAN, trackers) | `04_tle8888_hub.md` | ✅ |
| 5 | Condicionamento ADC | `05_analog.md` | ✅ |
| 6 | ETB (BTS7960) | `06_etb.md` | ✅ |
| 7 | Flex fuel | `07_flex.md` | ✅ |
| 8 | USB isolado | `08_usb.md` | ✅ |
| 9 | Conectores + WeAct | `09_connectors_weact.md` | ✅ |
| 10 | Knock (footprint only) | `10_knock_dnp.md` | ✅ |

**Fonte de verdade cruzada:** porquê → `interface_board_v1.md`; pinos → `pinout.md` /
`tle8888_pinout.md`; nets → `netlist_v1.md`. **Não duplicar racional** aqui.

## Hierarquia KiCad (projecto gerado)

**Abrir (produção):** `hardware/openems_ecu/openems_ecu.kicad_pro`  
**Arquitectura:** `docs/hw/microruseefi_as_base.md`  
**Pinmap lógico:** `docs/hw/pinmap_logical.md`

```
openems_ecu.kicad_sch (produção)          (root)
├── sheets/01_power.kicad_sch
├── sheets/02_ckp.kicad_sch
├── sheets/03_cmp.kicad_sch
├── sheets/04_tle8888_hub.kicad_sch
├── sheets/05_analog.kicad_sch
├── sheets/06_etb.kicad_sch
├── sheets/07_flex.kicad_sch
├── sheets/08_usb.kicad_sch
├── sheets/09_connectors_weact.kicad_sch
└── sheets/10_knock_dnp.kicad_sch
```

Nets globais (power / hierarchical labels):  
`VBAT`, `+5V_MAIN`, `+3V3`, `VDDA`, `VREF_P`, `+5V_SENS_A`, `+5V_SENS_B`,  
`PGND`, `SGND`, `AGND`, `SHIELD_GND`, e todos os `MCU.*` / `J1_*` / `J2_*`.

## Processo por módulo

1. Ler o `.md` do módulo  
2. Consultar Speeduino/rusEFI (secção “Referência”)  
3. Desenhar no KiCad **só** esse sheet  
4. ERC local: cada net do módulo termina em pino ou hierarquia  
5. Marcar checkbox no sheet  

## O que NÃO fazer ainda

- Layout / copper  
- Escolher caixa final  
- Popular knock  
