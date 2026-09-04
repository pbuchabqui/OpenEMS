# Sheet 02 — posição angular (MT6835, fork encoder)

> **Este ficheiro é do fork `feat/mt6835-encoder`.** A PCB de produção
> (`hw/v1-clean-board`, `hardware/openems_v1/`) usa Hall 60-2 em `PA0`.
> Aqui `PA0`/`PA1` são o par AB do encoder. Não copiar o Hall da v1 para cima.

## Função
Encoder magnético absoluto **MT6835** (SPI + ABZ) → `TIM2` em modo encoder.
`CH1`/`CH2` decodificam quadratura AB (16384 counts/volta). O CMP Hall
continua separado, capturado em `TIM3_CH1` (`PC6`) — grava `TIM2->CNT`,
não tempo. `TIM5` fica só como timebase (watchdogs / ω).

Autoridade: [`../../dev/mt6835_encoder_fork.md`](../../dev/mt6835_encoder_fork.md).
Datasheet: MagnTek MT6835 Rev 1.3 (2022.12) — ABFreq máx 2,048 MHz;
`RS_MAX` a 16384 PPR = 7500 RPM (INL não garantido acima); TDelay típ. 10 µs.

## Referência
- rusEFI / Speeduino: Hall de came em captura, não em TIM encoder.
- STM32H562 RM: TIM2 encoder mode, CH1/CH2 only.

**Adoptamos:** TIM2 hardware encoder (zero ISR por borda AB); CMP em TIM3.
**Adaptamos:** Z (index) para âncora/correção de drift AB vs SPI; CMP ainda
é Hall (fase 720°).
**Rejeitamos:** VR via TLE8888; Hall 60-2 em `PA0`; TIM6 como timebase.

## Topologia (bancada WeAct)

```
MT6835  A  ──── MCU.PA0  (TIM2_CH1)
        B  ──── MCU.PA1  (TIM2_CH2)
        Z  ──── MCU.<Z>  (captura / EXTI — ver fork doc)
        SPI ─── SPI2 (TLE8888 ausente neste fork, EMS_TLE8888_PRESENT=0)

CMP Hall ──── MCU.PC6  (TIM3_CH1, captura de TIM2_CNT)
```

## Nets
`ENC_A`, `ENC_B`, `ENC_Z`, `MCU.PA0`, `MCU.PA1`, `MCU.PC6`

SPI, PPR e pinos exactos de Z/CS ficam no firmware (`src/hal/mt6835.*`)
e no fork doc — esta sheet não inventa footprint de produção.

## Checklist
- [ ] Shield single-end ECU  
- [ ] Layout: par CKP longe de INJ/IGN/ETB (sheet layout notes)  
- [ ] R0 acessível para remove-before-flight motor  
