# Sheet 03 — CMP (Hall)

## Função
Sensor Hall open-collector de came → pull-up 5 V → divisor 3,3 V → **`PA1`**
(`TIM5_CH2`). Irmão do CKP (`02_ckp.md` → `PA0`/`TIM5_CH1`); mesmo timebase TIM5.

Autoridade: [`../pinout_v2.md`](../pinout_v2.md) (`CMP_IN`).

## Referência
- rusEFI VR_Board / hall notes: OC sink → pull-up ~1 k–10 k, ~5 mA.
- Speeduino: `TrigEdge` / cam edge configurável; pull-up common.
- **CKP v2** (`02_ckp.md`) — mesma topologia e valores.

**Adoptamos:** pull-up a **+5V_SENS_A** + divisor 10k/15k + Rfilt 330 + C 330pF + TVS 3,3 V.  
**Adaptamos:** polaridade em firmware `page0[258]` bit1 + PUPDR (pull-up se falling).  
**Rejeitamos:** partilhar canal VR (TLE saiu); RC agressivo (CMP só resolve fase, mas
não deve atrasar dezenas de µs).

## Topologia

```
+5V_SENS_A ── Rpu 10k ──┬── J1.CMP_SIG (N_CMP_RAW)
                        │
                        ├── R1 10k ──┬── N_CMP_DIV ── Rfilt 330 ──┬── MCU.PA1
                        │            │                              │
                        │         R2 15k                         C 330pF
                        │            │                              │
                        │          SGND                          AGND
                        │
                     TVS 3.3V ── AGND

J1.CMP_5V ← +5V_SENS_A
J1.CMP_GND ← SGND
```

Valores alinhados com `02_ckp` (2026-08-08): divisor 10k/15k → 0–5 V mapeia 0–3 V;
Rfilt 330 + 330 pF C0G (τ ~100 ns).

**Firmware:** se Hall idle HIGH / pulso LOW → page0[258] bit1=1 (falling) + pull-up
interno. RISING/FALLING final pendente do PN do Hall.

## Componentes (sheet actual)

| Ref | Valor | Footprint |
|-----|-------|-----------|
| R1 | 10 k (Rpu) | R_0603 |
| R2 | 10 k (div hi) | R_0603 |
| R3 | 15 k (div lo) | R_0603 |
| R4 | 330 (Rfilt) | R_0603 |
| D1 | TVS 3V3 uni | — PN BOM depois |
| C1 | 330 pF C0G | C_0603 |

## Nets
`N_CMP_DIV`, `MCU.PA1`, `J1.CMP_SIG`, `+5V_SENS_A`, `SGND`, `AGND`

## Checklist
- [x] Topologia Hall (não VR)
- [x] Pino = `PA1` / TIM5_CH2
- [x] Valores = espelho do CKP (10k/15k, 330, 330pF, TVS 3V3)
- [x] Sheet rascunho existe (`03_cmp.kicad_sch`) + título no canvas
- [ ] ERC / polish profissional (title block formal)
- [x] Global `MCU.PA1` na sheet 11 — netlist projecto: U1/24 + R4 + C1 (2026-08-08)
- [ ] Part number Hall CMP → bit polaridade
