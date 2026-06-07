# OpenEMS HIL Test

Teste automatizado Hardware-in-the-Loop.

## Hardware necessário

```
PC ─ USB ──→ STM32H562        (UART protocolo OpenEMS)
PC ─ USB ──→ ESP32             (esp32_combined.ino)
```

Um único ESP32 gera o sinal CKP e monitoriza os sinais IGN/INJ.
Não são necessários dois ESP32.

## Ligações

```
ESP32 GPIO 2  ────────────────→  STM32 PA0   (CKP 60-2)
ESP32 GPIO 4  ────────────────→  STM32 PA1   (CMP)
ESP32 GPIO 32 ←────────────────  STM32 PC6   (IGN0 TIM8_CH1)
ESP32 GPIO 33 ←────────────────  STM32 PC7   (IGN1 TIM8_CH2)
ESP32 GPIO 25 ←────────────────  STM32 PC8   (IGN2 TIM8_CH3)
ESP32 GPIO 26 ←────────────────  STM32 PC9   (IGN3 TIM8_CH4)
ESP32 GPIO 27 ←────────────────  STM32 PA15  (INJ0 TIM2_CH1)
ESP32 GPIO 14 ←────────────────  STM32 PB3   (INJ1 TIM2_CH2)
ESP32 GPIO 12 ←────────────────  STM32 PB10  (INJ2 TIM2_CH3)
ESP32 GPIO 13 ←────────────────  STM32 PB11  (INJ3 TIM2_CH4)
ESP32 GND     ─────────────────  STM32 GND   (OBRIGATÓRIO)
```

Não ligar GPIO36 — o canal CKP (CH8) é gerado internamente.

## Setup

```bash
# 1. Instalar dependência
pip install pyserial

# 2. Flash ESP32 com esp32_combined.ino (Arduino IDE)
#    Ficheiro: tools/esp32_combined/esp32_combined.ino

# 3. Flash STM32 com firmware OpenEMS
#    make flash
```

## Uso

```bash
# Teste com RPMs por defeito (500, 750, 1000, 1500, 2000, 3000)
python3 hil_test.py --stm32 /dev/ttyUSB0 --esp32 /dev/ttyUSB1

# RPMs específicos
python3 hil_test.py --stm32 /dev/ttyUSB0 --esp32 /dev/ttyUSB1 \
    --rpms 500 1000 2000

# Com relatório markdown
python3 hil_test.py --stm32 /dev/ttyUSB0 --esp32 /dev/ttyUSB1 \
    --report resultado.md
```

Exit code: `0` se todos PASS, `1` se algum FAIL.

## Verificações por ponto de RPM

| # | Verificação | Fonte | Tolerância |
|---|-------------|-------|-----------|
| 1 | FULL_SYNC atingido | STM32 snapshot | — |
| 2 | Sem sensor fault | STM32 snapshot | — |
| 3 | Sem late events | STM32 snapshot | — |
| 4 | RPM medido vs comandado | Snapshot | ±5% |
| 5 | VE snapshot vs tabela Python | Snapshot + page 1 | ±3 |
| 6 | Advance snapshot vs tabela Python | Snapshot + page 2 | ±2° |
| 7 | Injection PW vs req_fuel×VE×corr | Snapshot | ±15% |
| 8 | Firing order IGN0→IGN2→IGN3→IGN1 | ESP32 scope modo 't' | exacto |
| 9 | Inter-cylinder offset = T_cycle/4 | ESP32 scope modo 't' | ±2 ms |
| 10 | INJ0 PW medido vs snapshot PW | ESP32 scope LIVE | ±0.3 ms |
| 11 | IGN0 Dwell medido vs tabela dwell | ESP32 scope LIVE | ±0.3 ms |

## Saída esperada

```
╔══════════════════════════════════════╗
║   OpenEMS HIL Test                   ║
╚══════════════════════════════════════╝

STM32 : /dev/ttyUSB0
  ✓ STM32 OK
ESP32 : /dev/ttyUSB1  (CKP gen + scope)
  ✓ ESP32 OK

A ler config e tabelas da ECU...
  displacement=2000cc  inj=450cc/min  stoich=14.70  req_fuel=7273µs
  VE=OK  Spark=OK  Dwell=OK

────────────────────────────────────────────────────────────
  RPM=1000  idle normal
────────────────────────────────────────────────────────────
  → RPM=1000...
  → Scope timing analysis...
  → Scope LIVE...
  ✓  FULL_SYNC
  ✓  Sem sensor fault
  ✓  Sem late events
  ✓  RPM medido         actual=1003   expected=1000   err=0.3%
  ✓  VE (snapshot vs tabela)   actual=82  expected=82  err=0
  ✓  Advance            actual=12     expected=12     err=0.0°
  ✓  Injection PW       actual=6.100  expected=6.200  err=1.6%
  ✓  Firing order       IGN0→IGN2→IGN3→IGN1
  ✓  Inter-cil [1]      60.000 ms ≈ 60.000 ms
  ✓  INJ0 PW (scope)    6.100 ms ≈ 6.100 ms
  ✓  IGN0 Dwell (scope) 3.000 ms ≈ 3.000 ms

════════════════════════════════════════════════════════════
  RESULTADOS FINAIS: 66/66 PASS   0 FAIL
════════════════════════════════════════════════════════════
  ✓  RPM=  500
  ✓  RPM=  750
  ✓  RPM= 1000
  ✓  RPM= 1500
  ✓  RPM= 2000
  ✓  RPM= 3000
```

## Como funciona o modo combinado

O timer ISR que gera o sinal CKP no `GPIO2` escreve simultaneamente
no ring buffer do scope com o mesmo timestamp:

```cpp
// ckp_isr() — ISR context
gpio_set_level(CKP_GPIO, 1);       // → STM32 PA0
buf_push(ts, kCkpChan, 1u);        // → ring buffer scope (sem GPIO físico)
```

A FSM de timing analysis detecta o gap (intervalo fall→rise > 1.8×T)
e captura os 4 sparks IGN exactamente como se viesse de um GPIO externo.
