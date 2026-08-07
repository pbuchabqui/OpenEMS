# Sheet 10 — Knock (footprint DNP v1)

## Função
Reservar espaço e vias; **não popular**.

## Referência
- rusEFI TPIC8101 / knock wiki; detecção OpenEMS é 1 amostra/dente → precisa de **envelope**.  
**Adoptamos:** footprint TPIC8101 + SPI CS futuro + PA5 + J1 knock.  
**Rejeitamos:** bandpass discreto “à bruta” no ADC sem integrador.

## Topologia (não montar)

```
J1.KNOCK_SIG / SHLD → TPIC8101 → MCU.PA5
SPI2 + CS_TBD (não partilhar CSN do TLE sem buffer)
```

Longe de bobinas e VVT.

## Checklist
- [ ] Copper keepout / zona no layout  
- [ ] Via conector reservada, não cablada no chicote v1  
