# Sheet 07 — Flex fuel

## Função
Sensor GM/Continental 50–150 Hz → **`PB4`** (`TIM3_CH1` / captura) — autoridade:
[`../pinout_v2.md`](../pinout_v2.md) (`FLEX_IN`). Duty = temperatura combustível.

⚠️ **Não usar `PB5`.** Um rascunho anterior tinha `MCU.PB5`; o pinout v2 fixa flex em
`PB4` (pino 90). Corrigido 2026-08-07.

## Referência
- Speeduino flex input; rusEFI flex.
- Pull-up ~3k3 onboard no sensor; **pull-up interno MCU insuficiente**.
**Adoptamos:** divisor 10k/3k3 (0–12 V → ~0–3 V) + TVS + pull-up 10k a 5 V se OC.
**Adaptamos:** pino `PB4` (não o EXTI genérico do doc pré-v2).
**Rejeitamos:** alimentar o sensor a 5 V de lógica sem path de 12 V quando o sensor o exige.

## Topologia

```
J1.FLEX_12V ← VBAT (fused se possível)
J1.FLEX_GND ← SGND
J1.FLEX_SIG ── R1 10k ──┬── R2 3k3 ── SGND
                        │
                     Rpu 10k ← +5V_SENS_B (se open-collector)
                        │
                     TVS 3V3 ── AGND
                        │
                     MCU.PB4   (pinout_v2 FLEX_IN)
```

Sem RC agressivo (entrada digital lenta).

## Nets
`N_FLEX_RAW`, `FLEX_DIG`, `MCU.PB4`, `+5V_SENS_B`, `SGND`, `AGND`, `J1.FLEX_*`

## Checklist
- [x] Pino MCU = `PB4` (pinout_v2) — doc + sheet corrigidos (2026-08-07)
- [x] Global `MCU.PB4` na sheet 11 — netlist: U1/90 + flex (2026-08-08)
- [ ] 12 V sensor separado do rail de lógica
- [ ] Divisor dimensionado a 12 V + margem TVS
- [ ] Sheet com title_block + ERC local limpo (qualidade profissional)
