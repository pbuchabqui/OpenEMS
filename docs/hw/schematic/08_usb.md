# Sheet 08 — USB isolado

> **Status:** fora-de-escopo — placa v2 (MC33810, L9960T, CJ125, TPS65381A) e seus esquemáticos/conectores não fazem parte do projeto (decisão do usuário, 2026-11-17). Escopo atual: bancada WeAct H562 VGT6 + ESP32 (`docs/ROADMAP.md`).

## Função
CDC de calibração sem laço de terra veículo↔laptop.

## Referência
- Prática EMS aftermarket: isolador USB em ECU de bancada/veículo.  
**Adoptamos:** isolador full-speed + DC-DC isolado lado host.  
**Adaptamos:** PA11/PA12 do H562; isolador **junto ao conector USB**, barreira sem pour GND.

## Topologia

```
MCU.PA11 (DM) ── ESD ── Isolator logic side ── Isolator USB side ── USB-C/B
MCU.PA12 (DP) ── ESD ──          │                      │
+3V3 / GND MCU ─────────────────┘                      │
                                        DC-DC isolado ─┤
                                        +5V_USB_ISO / USB_GND_ISO
```

Candidatos: ADuM3160 / ISOUSB211 + B0505S-class.

## Checklist
- [ ] Creepage/clearance da barreira  
- [ ] Sem plano de terra a atravessar a barreira  
- [ ] ESD no lado conector  
