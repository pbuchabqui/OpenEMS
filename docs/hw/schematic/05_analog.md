# Sheet 05 — Condicionamento ADC

## Função
Sensores ratiométricos, NTC e VBATT → ADC1 do H562. **Mapa de pinos só de
`pinout_v2.md`** (não da tabela pré-v2).

## Referência
- Speeduino / rusEFI: divisor + RC + clamp; ADC a **3,3 V** → divisor 10k/15k
  (0–5 V → 0–3 V).
- APP1/APP2 em rails 5 V **diferentes** (plausibilidade de rail).
- LAMBDA_UA/UR vêm do **CJ125** (sheet 04d); aqui só RC leve + TVS no lado MCU.

**Adoptamos:** TVS 3,3 V por canal no nó ADC; pull-up NTC 2,49 k a `+5V_SENS_A`;
VBATT divisor 47k/10k (0–18 V → ~0–3,1 V).  
**Adaptamos:** canais v2 (`LAMBDA_*`, `EWG_POS`); dual tracker A/B (não TLE).  
**Rejeitamos:** knock activo (sheet 10 DNP); mapa pré-v2 (MAP em PA3, etc.).

## Rede padrão ratiométrica (0,5–4,5 V → ~0,3–2,7 V)

```
J1.SIG ── Rdiv 10k ──┬── Rf 1k ──┬── MCU.Pxx
                     │           │
                  Rlo 15k     C 100n
                     │           │
                   SGND        AGND
                     │
                  TVS 3V3 ── AGND
```

Sensor supply: `+5V_SENS_A` ou `_B` + retorno `SGND` (mapa no conector, sheet 09).

## Canais (`pinout_v2`)

| Sinal | GPIO | Pino | Rail 5 V | Tipo |
|-------|------|-----:|----------|------|
| `PRESS_OIL` | `PA2` | 25 | A | ratiométrico |
| `PRESS_FUEL` | `PA3` | 26 | A | ratiométrico |
| `LAMBDA_UA` | `PA4` | 29 | — (CJ125) | RC+TVS só |
| `LAMBDA_UR` | `PA5` | 30 | — (CJ125) | RC+TVS só |
| `EWG_POS` | `PA7` | 32 | A | ratiométrico |
| `APP1` | `PB0` | 35 | **A** | ratiométrico |
| `APP2` | `PB1` | 36 | **B** | ratiométrico (rail ≠ APP1) |
| `TPS1` | `PC0` | 15 | A | ratiométrico |
| `ETB_TPS2` | `PC1` | 16 | **B** | ratiométrico |
| `MAP` | `PC2` | 17 | A | ratiométrico (fc ~1,6 kHz) |
| `VBATT` | `PC3` | 18 | — | divisor VBAT |
| `CLT` | `PC4` | 33 | pull-up → A | NTC → SGND |
| `IAT` | `PC5` | 34 | pull-up → A | NTC → SGND |

13 canais ADC. Knock (`PA5` no doc v1) **não** — `PA5` é `LAMBDA_UR` na v2.

## NTC (CLT / IAT)

```
+5V_SENS_A ── Rpu 2k49 ──┬── Rf 1k ──┬── MCU.PCx
                         │           │
                      J1.NTC      C 100n
                         │           │
                       SGND        AGND
                         │
                      TVS 3V3 ── AGND
```

## VBATT (interno)

```
VBAT ── Rhi 47k ──┬── Rf 1k ──┬── MCU.PC3
                  │           │
               Rlo 10k     C 1µF
                  │           │
                PGND        AGND
```

0–18 V → ~0–3,1 V (`raw × 18000 / 4095` no firmware). RC forte OK (VBATT lenta).

## Componentes (refs sheet)

| Grupo | Refs | Valores |
|-------|------|---------|
| Ratiométricos ×8 | R100–R131, D10–D17, C20–C27 | 10k / 15k / 1k / TVS / 100n |
| Lambda ×2 | R132–R135, D18–D19, C28–C29 | Rf 1k + C 100n + TVS (sem divisor 5 V) |
| NTC ×2 | R136–R141, D20–D21, C30–C31 | 2k49 / 1k / TVS / 100n |
| VBATT | R142–R144, C32 | 47k / 10k / 1k / 1µF |

## Nets
`MCU.PA2`…`PA5`, `MCU.PA7`, `MCU.PB0`/`PB1`, `MCU.PC0`…`PC5`,  
`J1.*_SIG`, `N_*_DIV`, `+5V_SENS_A`/`_B`, `SGND`, `AGND`, `VBAT`, `PGND`

## Checklist
- [x] Canais = `pinout_v2` (2026-08-08)
- [x] APP1 rail A / APP2 rail B
- [x] Sheet regenerada — ERC **0** (2026-08-08)
- [x] Globals ADC na sheet 11 — netlist **13/13** U1↔canais
- [ ] PN TVS / passivos no BOM
- [ ] Title block formal
- [ ] `+5V_SENS_B` no APP2 (supply no conector; condicionamento partilha SGND)
