# Sheet 09 — Conectores AMPSEAL + socket WeAct

## Função
Fronteira mecânica/eléctrica: J1/J2 AMPSEAL RA, headers dual-row WeAct, TP, star GND.

## Referência
- `../ampseal_connectors.md`, `../weact_h562_coreboard.md`, `../weact_h562_schematic.md`  
**Adoptamos:** 35+23 tamanhos diferentes; RA gold; WeAct V1.0 38,62×69,10 mm.  
**Rejeitamos:** +12 V bobina/inj no conector da ECU.

## J2 — 23-pos potência (`1-770669-1` / plug `770680-1`)

| Pos lógicas | Net |
|-------------|-----|
| 1–2 | VBAT+ |
| 3–5 | PGND |
| 6–9 | INJ1–4 LS |
| 10–13 | IGN1–4 trigger |
| 14–15 | VVT_EXH / VVT_INT |
| 16–18 | PUMP_RLY / FAN_RLY / MAIN_RLY (DNP) |
| 19–20 | ETB_MOTOR_P ×2 |
| 21–22 | ETB_MOTOR_N ×2 |
| 23 | livre |

## J1 — 35-pos sinais (`1-776180-1` / plug `776164-1`)

| Grupo | Nets |
|-------|------|
| CKP | CKP_P, CKP_N, CKP_SHLD |
| CMP | CMP_SIG, CMP_5V, CMP_GND |
| Analógicos | MAP, CLT, IAT, APP1, APP2, FUEL_P, OIL_P, ETB_TPS1/2 |
| Rails | +5V_SENS_A/B, SGND×2 |
| CAN | CANH, CANL, CAN_SHLD |
| Flex | FLEX_12V, FLEX_SIG, FLEX_GND |
| Reserva | KNOCK×2, CMP2×3, TPS_INDEP, livres |

⚠️ Cavidades físicas 1…N: mapear após orientação pin 1 do header (drawing TE).

## WeAct P1/P2
Dual-row 2,54 mm; nets em `weact_h562_schematic.md`.  
Furos Φ3,2 @ offset 2,80 mm.

## Star ground
Único ponto: PGND + SGND + AGND + SHIELD_GND.  
Correntes INJ/IGN/ETB **nunca** atravessam SGND/AGND.

## Checklist
- [ ] Footprints TE das páginas de layout  
- [ ] Pin 1 silk + key color black  
- [ ] WeAct pin 1 confere com silkscreen físico  
