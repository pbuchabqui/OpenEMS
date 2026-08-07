# Sheet 08 — USB isolado

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
