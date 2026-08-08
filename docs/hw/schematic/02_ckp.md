# Sheet 02 — CKP (Hall)

## Função
Sensor Hall de roda dentada (60-2) → pull-up 5 V → divisor 3,3 V → **`PA0`**
(`TIM5_CH1`). Mesmo timebase do scheduler que o CMP (`PA1`/`TIM5_CH2`).

Autoridade de pino: [`../pinout_v2.md`](../pinout_v2.md) (`CKP_IN`).  
Decisão Hall (não VR): [`../architecture_v2.md`](../architecture_v2.md).

⚠️ **Isto muda o sensor exigido ao motor:** roda lida por VR deixa de servir na placa
— precisa de Hall (ou VR + condicionador **externo**, fora do âmbito desta sheet).

## Referência
- **CMP deste projecto** (`03_cmp.md`) — topologia irmã; mesmos valores.
- rusEFI / Speeduino hall notes: open-collector → pull-up ~1 k–10 k a 5 V.
- ~~Interface VR do TLE8888~~ — **rejeitada** com a v2 (TLE saiu).

**Adoptamos:** pull-up a **+5V_SENS_A** + divisor 10k/15k + Rfilt + C + TVS 3,3 V
(igual ao CMP).  
**Adaptamos:** pino `PA0` (TIM5_CH1); polaridade em firmware `page0[258]` bit0 + PUPDR.  
**Rejeitamos:** interface VR discreta na placa; partilhar condicionamento com o CMP
num único canal; filtro RC agressivo (atrasa o dente e o gap 60-2).

## Topologia

```
+5V_SENS_A ── Rpu 10k ──┬── J1.CKP_SIG (N_CKP_RAW)
                        │
                        ├── Rdiv1 10k ──┬── N_CKP_DIV ── Rfilt 330 ──┬── MCU.PA0
                        │               │                              │
                        │            Rdiv2 15k                      C 330pF
                        │               │                              │
                        │             SGND                          AGND
                        │
                     TVS 3.3V ── AGND

J1.CKP_5V  ← +5V_SENS_A   (alimentação do sensor no conector)
J1.CKP_GND ← SGND
```

Valores (iguais ao CMP, 2026-08-07 no rascunho `03_cmp`):

| Função | Valor | Nota |
|---|---|---|
| Rpu | 10 k | ~0,5 mA se OC a GND; subir corrente (1–4k7) se o Hall o pedir |
| Divisor | 10 k / 15 k 1% | 0–5 V → 0–3 V (margem ao 3,3 V ADC/GPIO) |
| Rfilt | 330 Ω | com 330 pF → τ ≈ 100 ns — leve, não come dentes |
| C | 330 pF C0G | no lado 3,3 V |
| TVS | 3,3 V unidirectional | no nó 3,3 V (após divisor), não no raw 5 V |

**Firmware:** polaridade RISING/FALLING ainda **pendente** do datasheet do Hall
(mecanismo de calibração já existe — `page0[258]` bit0). Default actual do firmware
é subida; confirmar antes do motor real.

## Componentes (sheet)

| Ref | Valor | Footprint |
|-----|-------|-----------|
| R10 | 10 k (Rpu) | R_0603 |
| R11 | 10 k (div hi) | R_0603 |
| R12 | 15 k (div lo) | R_0603 |
| R13 | 330 (Rfilt) | R_0603 |
| D2 | TVS 3V3 uni | SOD-323 / SMB class — PN BOM depois |
| C2 | 330 pF C0G | C_0603 |

Refs `R10–R13` / `D2` / `C2` para não colidir com o rascunho `03_cmp` (`R1–R4`/`D1`/`C1`).

## Nets
`N_CKP_RAW`, `N_CKP_DIV`, `MCU.PA0`, `J1.CKP_SIG`, `+5V_SENS_A`, `SGND`, `AGND`

## Checklist
- [x] Doc reescrito como Hall (não VR/TLE) — 2026-08-08
- [x] Pino = `PA0` / TIM5_CH1 (`pinout_v2`)
- [x] Topologia = espelho de `03_cmp` (mesmos valores R/C/TVS)
- [ ] Pull-up a 5 V sensores, não a 3,3 V MCU
- [ ] TVS no nó 3,3 V (pós-divisor)
- [ ] Part number Hall CKP → fechar bit polaridade RISING/FALLING
- [x] Sheet `02_ckp.kicad_sch` desenhada (2026-08-08) — clone de `03_cmp` com nets
      `J1.CKP_SIG` / `MCU.PA0` / `N_CKP_DIV`, refs R10–R13/D2/C2; ERC sheet **0 erros**
- [ ] Title block formal (hoje: texto de título no canvas)
- [x] Global `MCU.PA0` na sheet 11 — netlist projecto: U1/23 + R13 + C2 (2026-08-08)
