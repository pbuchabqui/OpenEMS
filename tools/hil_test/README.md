# OpenEMS HIL Test

Teste automatizado Hardware-in-the-Loop.

## Hardware necessário

```
PC ─ USB ──→ STM32H562 VGT6   (CDC protocolo OpenEMS, /dev/ttyACM0)
PC ─ WiFi ─→ ESP32             (esp32_stimulator, TCP 3333)
   ou USB ─→ ESP32             (/dev/ttyUSB0)
```

Um único ESP32 gera o sinal CKP e (opcionalmente) monitoriza IGN/INJ.

## Ligações (LQFP100 — WeAct STM32H562VGT6)

```
                          CKP / CMP (entrada)
ESP32 GPIO 2  ────────────────→  STM32 PA0   (CKP 60-2, TIM5_CH1)
ESP32 GPIO 4  ────────────────→  STM32 PA1   (CMP, TIM5_CH2)

                          IGN (saída — TIM1)
ESP32 GPIO 32 ←────────────────  STM32 PA8   (IGN0 TIM1_CH1)
ESP32 GPIO 33 ←────────────────  STM32 PE11  (IGN1 TIM1_CH2)
ESP32 GPIO 25 ←────────────────  STM32 PE13  (IGN2 TIM1_CH3)
ESP32 GPIO 26 ←────────────────  STM32 PE14  (IGN3 TIM1_CH4)

                          INJ (saída — TIM2)
ESP32 GPIO 27 ←────────────────  STM32 PC6   (INJ0 TIM2_CH1)
ESP32 GPIO 14 ←────────────────  STM32 PC7   (INJ1 TIM2_CH2)
ESP32 GPIO 12 ←────────────────  STM32 PB10  (INJ2 TIM2_CH3)
ESP32 GPIO 13 ←────────────────  STM32 PB11  (INJ3 TIM2_CH4)

ESP32 GND     ─────────────────  STM32 GND   (OBRIGATÓRIO)
```

Mínimo para rodar: **CKP (GPIO2→PA0) + GND**. Os demais são opcionais.

## Pré-requisitos

```bash
# Kernel: desabilitar USB autosuspend (causa disconnect da CDC em ~1s)
sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'

# Python
pip install pyserial
```

## Uso

```bash
# Estimulador via WiFi/TCP (recomendado)
python3 hil_test.py --stm32 /dev/ttyACM0 --stim tcp:192.168.15.169:3333

# Estimulador via USB serial
python3 hil_test.py --stm32 /dev/ttyACM0 --stim /dev/ttyUSB0

# Bench sem sensores MAP/TPS (força CLT/IAT, aceita SENSOR_FAULT)
python3 hil_test.py --stm32 /dev/ttyACM0 --stim tcp:192.168.15.169:3333 \
    --bench-clt-iat

# RPMs específicos + relatório
python3 hil_test.py --stm32 /dev/ttyACM0 --stim tcp:192.168.15.169:3333 \
    --bench-clt-iat --rpms 800 1500 3000 --report resultado.md
```

Exit code: `0` se todos PASS, `1` se algum FAIL.

## Flash

```bash
# DFU (sem :leave — fazer power-cycle manual depois)
dfu-util -a 0 -s 0x08000000 -D /tmp/openems-build/bin/openems.bin

# IMPORTANTE: após DFU, power-cycle (despluga/repluga USB com BOOT0 solto).
# O :leave faz jump sem POR — RCC fica no estado do bootloader.
```

## Verificações por ponto de RPM

| # | Verificação | Fonte | Tolerância |
|---|-------------|-------|-----------|
| 1 | FULL_SYNC atingido | STM32 snapshot | — |
| 2 | Sem sensor fault | STM32 snapshot | — (skip com --bench-clt-iat) |
| 3 | Sem late events | STM32 snapshot | — |
| 4 | RPM medido vs comandado | Snapshot | ±5% |
| 5 | VE snapshot vs tabela Python | Snapshot + page 1 | ±3 |
| 6 | Advance snapshot vs tabela Python | Snapshot + page 2 | ±2° (±8° com limp) |
| 7 | Injection PW vs cálculo | Snapshot + pages 4/5 | ±45% bench / ±15% full |

## Notas bench (sem sensores MAP/TPS)

- `--bench-clt-iat` força CLT=90°C / IAT=25°C via comando 'B' e aceita
  SENSOR_FAULT como esperado (MAP/TPS não conectados leem lixo).
- **Limp rev-cut**: com SENSOR_FAULT, a ECU corta injeção acima de 3000 RPM
  (PW=0 é esperado e o teste aceita automaticamente).
- **PW ~40% acima do calculado**: afterstart enrichment e cranking multiplier
  não são modelados no cálculo Python — tolerância alargada para 45%.
- **Advance limitado em altas RPMs**: limp mode retarda spark perto do rev
  limiter — tolerância alargada para ±8° acima de 3000 RPM com fault.
