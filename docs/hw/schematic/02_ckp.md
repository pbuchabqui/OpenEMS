# Sheet 02 — CKP (VR via TLE8888)

## Função
Sensor VR 60-2 → `VRIN1/2` → interface VR do TLE → `VROUT` → `PA0` (TIM5_CH1).

## Referência
- rusEFI: TLE8888 VR interface (zero-crossing + peak arm); `tle8888.cpp` VRSConfig.  
- Speeduino / rusEFI VR boards: par trançado blindado, shield só no ECU.  
- MAX9924 **fora** da BOM (atraso e open-drain — supersedido).  
**Adoptamos:** VR nativo do TLE, clamp interno, push-pull out.  
**Adaptamos:** TP-VR + jumper 0 Ω TP-DIG para bancada (ESP32 digital).  
**Rejeitamos:** filtro RC agressivo na entrada diferencial; pull-up em VROUT.

## Topologia

```
J1.CKP+ ────┬──── TLE pin 52 VRIN1
            │
J1.CKP- ────┴──── TLE pin 51 VRIN2     (par trançado no chicote)
J1.CKP_SHLD ────── SHIELD_GND          (só lado ECU)

TLE pin 21 VROUT ──┬── R0 0Ω (jumper) ── MCU.PA0  (CKP_DIG)
                   │
                   └── TP-DIG (header 2.54, para estimulador)

TP-VR+ / TP-VR- nos nós VRIN (antes do IC), para gerador/DAC.
```

**MCU:** PA0 pull-down interno **fica** (falso-sync). Captura: page0[258] bit0 (default subida).

## Componentes
| Ref | Valor | Notas |
|-----|-------|-------|
| R0 | 0 Ω 0805 | DNP = bancada ESP32 no TP-DIG |
| TP×3 | test points | VROUT, VRIN1, VRIN2 |

Sem diodos de clamp externos (50 mA internos). Sem R série se o DS do sensor não exigir.

## Nets
`CKP_P`, `CKP_N`, `CKP_DIG`, `SHIELD_GND`, `MCU.PA0`

## Checklist
- [ ] Shield single-end ECU  
- [ ] Layout: par CKP longe de INJ/IGN/ETB (sheet layout notes)  
- [ ] R0 acessível para remove-before-flight motor  
