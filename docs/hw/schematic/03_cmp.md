# Sheet 03 — CMP (Hall)

## Função
Sensor Hall open-collector de came → pull-up 5 V → divisor 3,3 V → `PA1` (TIM5_CH2).

## Referência
- rusEFI VR_Board / hall notes: OC sink → pull-up ~1 k–10 k, ~5 mA.  
- Speeduino: `TrigEdge` / cam edge configurável; pull-up common.  
**Adoptamos:** pull-up a **+5V_SENS_A** + divisor 10k/15k + TVS 3,3 V.  
**Adaptamos:** polaridade em firmware page0[258] bit1 + PUPDR (pull-up se falling).  
**Rejeitamos:** partilhar o único canal VR do TLE com o CMP.

## Topologia

```
+5V_SENS_A ── Rpu 10k ──┬── J1.CMP_SIG (N_CMP_RAW)
                        │
                        ├── R1 10k ──┬── N_CMP_DIV ── Rfilt 100–1k ──┬── MCU.PA1
                        │            │                                │
                        │         R2 15k                           C 100p–1n
                        │            │                                │
                        │          SGND                            AGND/SGND
                        │
                     TVS 3.3V ── AGND

J1.CMP_5V ← +5V_SENS_A
J1.CMP_GND ← SGND
```

Valores: divisor 10k/15k mapeia 0–5 V → 0–3 V (margem ao 3,3 V).  
RC “leve”: não atrasar dezenas de µs (CMP só resolve fase).

**Firmware:** se Hall idle HIGH / pulso LOW → page0[258] bit1=1 (falling) + pull-up interno.

## Componentes
| Ref | Valor |
|-----|-------|
| Rpu | 10 k (ou 1–4k7 se sensor pedir mais corrente) |
| R1/R2 | 10 k / 15 k 1% |
| Rfilt | 100–1 k |
| C | 100 pF–1 nF |
| TVS | 3,3 V unidirectional |

## Nets
`N_CMP_RAW`, `N_CMP_DIV`, `CMP_DIG`, `MCU.PA1`, `+5V_SENS_A`, `SGND`

## Checklist
- [ ] Pull-up a 5 V sensores, não a 3,3 V MCU (nível Hall típico 5 V)  
- [ ] TVS no nó 3,3 V  
- [ ] Documentar part number Hall → bit polaridade  
