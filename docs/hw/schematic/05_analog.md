# Sheet 05 — Condicionamento ADC

## Função
Todos os sensores ratiométricos e NTC → ADC do H562. Rede padrão + VBATT interno.

## Referência
- Speeduino / rusEFI: divisor + RC + clamp 5 V boards; nós usamos **3,3 V ADC** → divisor 10k/15k.  
- APP1/APP2 em rails 5 V **diferentes** (plausibilidade de rail).  
**Adoptamos:** TVS 3,3 V por canal; pull-up NTC 2,49 k a 5 V sens.  
**Adaptamos:** dual tracker TLE; VBATT em PC3 com divisor 0–18 V.  
**Rejeitamos:** knock activo neste sheet (só net reservada).

## Rede padrão (0,5–4,5 V → ~0,3–2,7 V)

```
J1.SIG ── R1 10k ──┬── Rf 1k ──┬── MCU.ADx
                   │           │
                R2 15k      C 100n
                   │           │
                 SGND        AGND
                   │
                TVS 3V3 ── AGND
```

Alimentação sensor: `+5V_SENS_A` ou `_B` + retorno `SGND`.

## Canais

| Sinal | MCU | 5 V rail | Notas |
|-------|-----|----------|-------|
| MAP | PA3 | A | fc 1,6 kHz — rever se MAP “rápido” |
| TPS indep. | PA4 | A | reserva |
| APP1 | PC0 | A | |
| APP2 | PC2 | **B** | rail ≠ APP1 |
| ETB_TPS1 | PA2 | A | |
| ETB_TPS2 | PC5 | **B** | |
| FUEL_P | PC4 | B | |
| OIL_P | PC1 | B | |
| CLT | PB0 | pull-up 2k49 → A | NTC to SGND |
| IAT | PB1 | pull-up 2k49 → A | NTC to SGND |
| VBATT | PC3 | — | ver abaixo |
| KNOCK | PA5 | — | sheet 10 DNP |

## VBATT (interno)

```
VBAT ── Rhi ──┬── Rf ──┬── PC3
              │        │
           Rlo       C large
              │        │
            PGND/SGND AGND
```

Razão: 0–18 V → 0–3,3 V → `raw × 18000 / 4095` no firmware.  
Ex.: Rhi=47k, Rlo=10k → ≈ 0–18 V a 0–3,1 V (calibrar no passo 5).  
RC forte OK (VBATT lenta).

## Checklist
- [ ] APP1/APP2 rails diferentes  
- [ ] TVS e C no lado 3,3 V  
- [ ] SGND ≠ PGND até ao star  
- [ ] Knock não populado  
