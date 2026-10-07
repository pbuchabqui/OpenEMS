# Sheet 01 — Alimentação (entrada / protecção)

## Função
Entrada de bateria no AMPSEAL J2 → **protecção de polaridade + fusível + TVS + bulk**
→ rail `VBAT` limpo para o resto da placa.

Na arquitectura **v2**, buck / LDO / trackers 5 V de sensor vivem no **TPS65381A-Q1**
(sheet `04_pmic_tps65381`, ainda por desenhar). Este sheet **não** regula 3,3 V nem 5 V.

## Referência (Speeduino / rusEFI)
- rusEFI / microRusEFI: TVS no 12 V, bulk, main-relay / key-on.
- Speeduino: fusível + diodo ou P-FET reverse.

**Adoptamos:** P-FET reverse + fusível + SMBJ24CA + bulk 100 µF + 100 nF.  
**Adaptamos:** sem buck/LDO aqui (vão para o PMIC); key-on directo (sem power-latch MCU).  
**Rejeitamos:** AMS1117 / LM2596; alimentar sensores por USB; TLE8888 BAT/trackers.

## Topologia

```
J2.VBAT+ ── F1 30A ── VBAT_RAW ──┬── Q1 P-MOS source
                                 │         │
                                 │        gate ── R_g 100k ── PGND
                                 │         │
                                 │        (R_gs 1M opcional source–gate)
                                 │         │
                                 └──────── drain ── VBAT ──┬── C_bulk 100µF ── PGND
                                                           ├── C 100nF ── PGND
                                                           └── D_tvs SMBJ24CA ── PGND
                                                                    (cátodo a VBAT)

J2.VBAT- / chassis ── PGND

Star (layout): PGND · SGND · AGND unem-se num ponto perto desta entrada
(net-tie no layout; no esquemático partilham labels + PWR_FLAG).
```

Key-on: **VBAT directo** (sem main-relay na path de alimentação da ECU).  
Main-relay de actuadores: GPIO `MAIN_RELAY` / `PE14` (sheet de drive / conectores).

## Componentes

| Ref | Valor / PN | Notas |
|-----|------------|-------|
| F1 | 30 A blade / poly | acessível; `Device:Fuse` |
| Q1 | P-MOSFET automotive logic-level | `Device:Q_PMOS_GSD` — PN BOM (ex. IRF7416 class / 40 V+) |
| R1 | 100 k | gate → PGND (liga o FET com polaridade correcta) |
| R2 | 1 M (opc.) | gate–source, anti-flutuação |
| D1 | **SMBJ24CA** | cátodo a `VBAT`, ânodo `PGND` |
| C1 | 100 µF low-ESR ≥35 V | bulk |
| C2 | 100 nF 50 V | HF |

## Nets exportadas
`VBAT`, `VBAT_RAW`, `PGND`, `SGND`, `AGND`  
(`+3V3`, `VDDA`, `+5V_SENS_*` → sheet PMIC)

## Checklist
- [x] Doc v2 sem buck/TLE (2026-08-08)
- [x] Sheet desenhada + ERC
- [ ] PN exacto do P-FET e do fusível no BOM
- [ ] Star GND no layout (net-tie)
- [ ] PMIC sheet produz `+3V3` / trackers
