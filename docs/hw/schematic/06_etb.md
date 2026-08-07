# Sheet 06 — ETB (BTS7960 @ 10 kHz)

## Função
Ponte-H do motor da borboleta; feedback TPS no sheet 05.

## Referência
- rusEFI / DBW boards: H-bridge externo, feedback dual TPS.  
- Firmware OpenEMS: PWM + IN1 + IN2, ambos LOW = travagem (`etb_driver`).  
**Adoptamos:** módulo BTS7960 (loops de comutação internos).  
**Adaptamos:** 10 kHz (não 20 kHz); PE5/PE7/PE8.  
**Rejeitamos:** DRV8701 (interface 2 pinos ≠ firmware 3 pinos) sem mudança de SW.

## Topologia

```
MCU.PE5 (TIM15 PWM) ── ETB_PWM  ── BTS7960.PWM (ou RPWM/LPWN conforme módulo)
MCU.PE7             ── ETB_DIR1 ── IN1 / R_EN
MCU.PE8             ── ETB_DIR2 ── IN2 / L_EN

VBAT ── bulk ── BTS7960.VCC
PGND ────────── BTS7960.GND   (retorno próprio ao star — não partilhar SGND)

BTS7960.OUT1 ──┬── J2.ETB_M+ (pin ×2)
BTS7960.OUT2 ──┴── J2.ETB_M− (pin ×2)
```

⚠️ Confirmar pinout do **módulo** concreto (RPWM/LPWM vs PWM+DIR).  
Gate de segurança (procedimento): mola default-closed, corte de energia duro, batente, validar
em bancada antes do motor.

## Componentes
| Ref | Notas |
|-----|-------|
| U_etb | BTS7960 module |
| C_bulk | 100–470 µF em VBAT junto do módulo |
| F_etb | fusível dedicado opcional no chicote |

## Nets
`ETB_PWM`, `ETB_DIR1`, `ETB_DIR2`, `ETB_MOTOR_P/N`, `VBAT`, `PGND`

## Checklist
- [ ] 10 kHz no firmware (path etb_driver)  
- [ ] Motor pins duplicados no AMPSEAL 23  
- [ ] Feedback TPS longe do par de potência  
