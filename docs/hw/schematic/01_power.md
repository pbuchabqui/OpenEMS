# Sheet 01 — Alimentação

## Função
Entrada de bateria → proteção → `VBAT` → (a) TLE8888 BAT + (b) buck → LDO → `+3V3` / `VDDA` / `VREF_P`.  
Trackers 5 V dos sensores vêm do **TLE** (sheet 04), não deste buck.

## Referência (Speeduino / rusEFI)
- rusEFI / microRusEFI: main relay + BAT rail + TVS no 12 V; sensores em rail limpo separado.  
- Speeduino boards: fusível + diodo/P-FET reverse; LDO 5 V ruidoso comum — **rejeitamos** AMS1117 e buck 150 kHz.  
**Adoptamos:** P-FET reverse + fusível + TVS + bulk.  
**Adaptamos:** buck ≥500 kHz + LDO high-PSRR só para MCU; 5 V sensores = T5V1/T5V2 do TLE.  
**Rejeitamos:** LM2596, AMS1117, alimentar sensores pelo USB 5 V.

## Topologia

```
J2.VBAT+ (×2) ──► Q1 P-MOSFET (rev. polarity) ──► F1 30A ──► VBAT
                                                         │
                              SMBJ24CA ──► PGND          │
                              C_bulk 100µF + 100nF       │
                    ┌────────────────────────────────────┤
                    │                                    │
                    ▼                                    ▼
            TLE BAT (p54)                    buck ≥500 kHz (ex. TPS54302)
            BATPA/B (87/90)                           │
            + bulk local                         +5V_MAIN
                                                      │
                                                 LDO 3V3 low-noise
                                                      │
                                    ┌─────────────────┼─────────────────┐
                                    ▼                 ▼                 ▼
                                  +3V3            ferrite            (DNP VREF opt c)
                                MCU VDD          +1µ+100n           5V_SENS→3.0V buf
                                                    │
                                                  VDDA ──► VREF+ (pino sep. LQFP100)
                                                  VSSA = AGND
```

Key-on: **VBAT directo** (sem MR na v1). `TLE.MR` (p55) footprint DNP → J2.MAIN_RLY.

## Componentes

| Ref | Valor / PN | Notas |
|-----|------------|-------|
| Q1 | P-MOSFET automotive logic-level | gate pull-up a VBAT_RAW; source=RAW, drain=prot |
| F1 | 30 A blade / poly | acessível |
| D_tvs | SMBJ24CA | CATHODE a VBAT, anode PGND |
| C_bulk | 100 µF low-ESR + 100 nF | em VBAT e em BAT do TLE |
| U_buck | TPS54302 ou ≥500 kHz | Cin/Cout por DS |
| U_ldo | TPS7A20 / ADP7118 class | high PSRR @ f_sw |
| L_ferrite | 600 Ω @ 100 MHz | VDDA |
| C_vdda | 1 µF + 100 nF | VSSA local |
| C_vdd | 100 nF / pin VDD | no coreboard + extras se carrier alimenta 3V3 |

## Nets exportadas
`VBAT`, `VBAT_RAW`, `+5V_MAIN`, `+3V3`, `VDDA`, `VREF_P`, `PGND`, `AGND`

## Checklist
- [ ] Reverse polarity só no caminho de entrada  
- [ ] Trackers 5 V **não** saem deste sheet  
- [ ] Star: PGND vs AGND unidos num ponto  
- [ ] VREF+ ligado a VDDA (opção a); footprint DNP (c) opcional  
