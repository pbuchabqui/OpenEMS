# Pinout v2 — STM32H562VGT6 LQFP100 (arquitectura MC33810 / L9960T / TPS65381A / CJ125)

> **AUTORIDADE de números de pino e funções de periférico.** Toda a tabela abaixo foi
> gerada e **validada programaticamente** contra `MCU_ST_STM32H5:STM32H562VGTx`
> (`/usr/share/kicad/symbols/`), 100 pinos, 1..100 completos. 53 atribuições, **0 falhas**.
> Ver `../../docs/hw/README.md` §1 — código ganha a documento; este ficheiro ganha a
> qualquer prosa sobre pinos.

⚠️ **Não é a matriz da spec Gemini.** Aquela tinha 25 de 45 números errados (deslocamentos
sistemáticos: `PE9/10/11` −2, `PD12–15` −1, `PB8/PB9` −2), punha `VBAT_SENSE` no `VDDA` e o
`IAT` no `VREF+`, e usava periféricos que o GPIOE não tem (TIM2/TIM3/TIM5 em `PE*`).
Foi **descartada e refeita**.

## Restrições que fixaram a alocação

| Restrição | Consequência |
|---|---|
| `TIM5_CH1..CH4` só sai em `PA0..PA3`, e o scheduler usa `TIM5_CH3` | **CKP=`PA0`, CMP=`PA1`** — capturas e eventos partilham o contador, sem conversão de timebase |
| GPIOE tem 15 pinos bondados (`PE1` **não** — pino 98 = `VCAP`) | Os 8 direct inputs do MC33810 cabem numa só porta |
| `kOutBsrrPin[] = {4,6,0,2, 15,13,11,9}` já existe e está validado em HW | INJ/IGN **mantêm** `PE0/2/4/6` + `PE9/11/13/15` → **zero alterações ao caminho quente** |
| `FDCAN2` **não existe** neste package | Barramento único `FDCAN1` + expansor de I/O por SPI (`CS_IOEXP`) |
| `USB_DM/DP` só em `PA11`/`PA12` | Não colide — `FDCAN1` fica em `PB8`/`PB9` |
| VBATT decidido em `PC3`/INP13 | Mantido, apesar de a spec o querer no `PA6` (que é `VDDA`) |

## Tabela

| Bloco | Sinal | GPIO | Pino | Função |
|---|---|---|---:|---|
| adc | `PRESS_OIL` | `PA2` | 25 | ADC1_INP14 |
| adc | `PRESS_FUEL` | `PA3` | 26 | ADC1_INP15 |
| adc | `LAMBDA_UA` | `PA4` | 29 | ADC1_INP18 |
| adc | `LAMBDA_UR` | `PA5` | 30 | ADC1_INP19 |
| adc | `EWG_POS` | `PA7` | 32 | ADC1_INP7 |
| adc | `APP1` | `PB0` | 35 | ADC1_INP9 |
| adc | `APP2` | `PB1` | 36 | ADC1_INP5 |
| adc | `TPS1` | `PC0` | 15 | ADC1_INP10 |
| adc | `ETB_TPS2` | `PC1` | 16 | ADC1_INP11 |
| adc | `MAP` | `PC2` | 17 | ADC1_INP12 |
| adc | `VBATT` | `PC3` | 18 | ADC1_INP13 |
| adc | `CLT` | `PC4` | 33 | ADC1_INP4 |
| adc | `IAT` | `PC5` | 34 | ADC1_INP8 |
| aux | `VVT1` | `PE10` | 41 | GPIO |
| aux | `VVT2` | `PE12` | 43 | GPIO |
| aux | `MAIN_RELAY` | `PE14` | 45 | GPIO |
| aux | `PUMP` | `PE7` | 38 | GPIO |
| aux | `FAN` | `PE8` | 39 | GPIO |
| can | `CAN1_RX` | `PB8` | 95 | FDCAN1_RX |
| can | `CAN1_TX` | `PB9` | 96 | FDCAN1_TX |
| dbg | `SWDIO` | `PA13` | 72 | GPIO |
| dbg | `SWCLK` | `PA14` | 76 | GPIO |
| drive | `MC33810_RSTB` | `PD10` | 57 | GPIO |
| drive | `MC33810_EN` | `PD11` | 58 | GPIO |
| drive | `MC33810_FAULTB` | `PD9` | 56 | GPIO |
| drive | `INJ1` | `PE0` | 97 | GPIO |
| drive | `IGN2` | `PE11` | 42 | GPIO |
| drive | `IGN3` | `PE13` | 44 | GPIO |
| drive | `IGN4` | `PE15` | 46 | GPIO |
| drive | `INJ2` | `PE2` | 1 | GPIO |
| drive | `INJ3` | `PE4` | 3 | GPIO |
| drive | `INJ4` | `PE6` | 5 | GPIO |
| drive | `IGN1` | `PE9` | 40 | GPIO |
| etb | `ETB_PWM` | `PD12` | 59 | TIM4_CH1 |
| etb | `ETB_DIR` | `PD13` | 60 | GPIO |
| etb | `EWG_PWM` | `PD14` | 61 | TIM4_CH3 |
| etb | `EWG_DIR` | `PD15` | 62 | GPIO |
| flex | `FLEX_IN` | `PB4` | 90 | TIM3_CH1 |
| o2 | `O2_HEATER_PWM` | `PE5` | 4 | TIM15_CH1 |
| pmic | `TPS65381_NRES_IN` | `PB2` | 37 | GPIO |
| pmic | `TPS65381_ENDRV` | `PE3` | 2 | GPIO |
| spi | `CS_PMIC` | `PB12` | 51 | GPIO |
| spi | `SPI2_SCK` | `PB13` | 52 | SPI2_SCK |
| spi | `SPI2_MISO` | `PB14` | 53 | SPI2_MISO |
| spi | `SPI2_MOSI` | `PB15` | 54 | SPI2_MOSI |
| spi | `CS_MC33810` | `PD5` | 86 | GPIO |
| spi | `CS_L9960T` | `PD6` | 87 | GPIO |
| spi | `CS_CJ125` | `PD7` | 88 | GPIO |
| spi | `CS_IOEXP` | `PD8` | 55 | GPIO |
| sync | `CKP_IN` | `PA0` | 23 | TIM5_CH1 |
| sync | `CMP_IN` | `PA1` | 24 | TIM5_CH2 |
| usb | `USB_DM` | `PA11` | 70 | USB_DM |
| usb | `USB_DP` | `PA12` | 71 | USB_DP |

## Orçamento

- **53 pinos atribuídos**, 27 GPIO livres para crescimento.
- ADC: 13 canais em uso dos 16 `ADC1_INP` bondados.
- `EWG_POS` (`PA7`/INP7) é a **dívida nova** de reactivar o EWG: o PID precisa de
  realimentação de posição real, que estava fixada em 0 quando o bloco foi guardado.

## Pendente

- Polaridade de captura CKP/CMP continua em aberto (ver memória
  `cmp-ckp-capture-edge-polarity`) — não é resolvida por esta tabela.
- `VVT1`/`VVT2`, `PUMP`, `FAN` estão como GPIO puro. Se algum precisar de PWM real em vez
  de on/off, tem de sair do GPIOE para um pino com timer — o GPIOE não tem TIM2/3/5.
- O expansor de I/O por SPI ainda não tem part number.
