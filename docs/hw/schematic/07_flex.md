# Sheet 07 — Flex fuel

## Função
Sensor GM/Continental 50–150 Hz → PB5 EXTI; duty = temperatura combustível.

## Referência
- Speeduino flex input; rusEFI flex.  
- Pull-up ~3k3 onboard no sensor; **pull-up interno MCU insuficiente**.  
**Adoptamos:** divisor 10k/3k3 (0–12 V → ~0–3 V) + TVS + pull-up 10k a 5 V se OC.

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
                     MCU.PB5
```

Sem RC agressivo (entrada digital lenta).

## Nets
`N_FLEX_RAW`, `FLEX_DIG`, `MCU.PB5`

## Checklist
- [ ] 12 V sensor separado do rail de lógica  
- [ ] Divisor dimensionado a 12 V + margem TVS  
