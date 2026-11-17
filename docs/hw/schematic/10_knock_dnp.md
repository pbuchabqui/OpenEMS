# Sheet 10 — Knock (footprint DNP v1)

> **Status:** fora-de-escopo — placa v2 (MC33810, L9960T, CJ125, TPS65381A) e seus esquemáticos/conectores não fazem parte do projeto (decisão do usuário, 2026-11-17). Escopo atual: bancada WeAct H562 VGT6 + ESP32 (`docs/ROADMAP.md`).

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
