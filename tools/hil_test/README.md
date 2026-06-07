# OpenEMS HIL Test

Teste automatizado Hardware-in-the-Loop. Varia RPM via ESP32 generator,
lê o snapshot da ECU via UART e verifica que os valores calculados pelo
firmware coincidem com os valores esperados calculados em Python a partir
das mesmas tabelas.

## Dependências

```bash
pip install pyserial
```

## Ligações físicas

```
PC UART  → STM32H562 PA9/PA10  (ou USB-CDC PA11/PA12)
PC UART  → ESP32 CKP generator  (esp32_ckp_gen.ino)
PC UART  → ESP32 scope          (esp32_scope.ino)  ← opcional
```

## Uso mínimo (sem scope)

```bash
python3 hil_test.py --stm32 /dev/ttyUSB0 --gen /dev/ttyUSB1
```

## Com scope

```bash
python3 hil_test.py \
    --stm32 /dev/ttyUSB0 \
    --gen   /dev/ttyUSB1 \
    --scope /dev/ttyUSB2 \
    --report report.md
```

## RPMs personalizados

```bash
python3 hil_test.py --stm32 /dev/ttyUSB0 --gen /dev/ttyUSB1 \
    --rpms 500 1000 2000 3000
```

## O que é verificado por ponto de RPM

| Verificação | Fonte | Tolerância |
|-------------|-------|-----------|
| FULL_SYNC atingido | STM32 snapshot status | — |
| RPM medido vs comandado | Snapshot | ±5% |
| VE calculado vs tabela Python | Snapshot + page 1 | ±3 unidades |
| Advance calculado vs tabela Python | Snapshot + page 2 | ±2° |
| Injection PW vs req_fuel × VE × correcções | Snapshot | ±15% |
| Firing order IGN0→IGN2→IGN3→IGN1 | ESP32 scope modo 't' | exacto |
| Inter-cylinder offset = T_cycle/4 | ESP32 scope modo 't' | ±2 ms |
| Dwell medido vs tabela dwell | ESP32 scope modo LIVE | ±0.3 ms |
| INJ PW medido vs snapshot PW | ESP32 scope modo LIVE | ±0.3 ms |

## Math mirror

`hil_test.py` reimplementa em Python (com inteiros) as funções críticas
do firmware:

- `_axis_lookup(axis, value)` → mirror de `axis_lookup()` em table3d.cpp
- `_lerp_q8(a, b, frac)` → mirror de `lerp_q8_s32()`
- `table3d_lookup_u8/i8()` → interpolação bilinear idêntica ao firmware
- `EngineConfig.req_fuel_us` → mirror inteiro de `calc_req_fuel_us()`
- `Tables.ve_at(rpm, map)` → lookup na tabela VE lida da ECU
- `Tables.advance_at(rpm, map)` → lookup na tabela spark
- `Tables.dwell_ms_at(vbatt_mv)` → lookup 1D na tabela dwell

Os eixos (RPM e MAP) são os mesmos do firmware:
```
RPM (rpm×10): 500, 750, 1000, 1250, 1500, 2000, 2500, 3000,
              4000, 5000, 6000, 7000, 8000, 9000, 10000, 12000
MAP (bar×100): 20, 30, 40, 50, 60, 70, 80, 90,
               100, 115, 130, 145, 160, 180, 215, 300
```

## Saída esperada

```
╔══════════════════════════════════════════╗
║   OpenEMS HIL Test                       ║
╚══════════════════════════════════════════╝

STM32  : /dev/ttyUSB0
  ✓ STM32 OK
CKP Gen: /dev/ttyUSB1
  ✓ CKP generator OK

A ler configuração da ECU...
  displacement=2000cc  inj=450cc/min  stoich=14.70  req_fuel=7273µs
A ler tabelas (VE, spark, dwell, correcções)...
  VE table: OK
  Spark:    OK
  Dwell:    OK

────────────────────────────────────────────────────────────
  RPM=500  cranking — idle baixo
────────────────────────────────────────────────────────────
  → Comandar RPM=500...
  ✓  FULL_SYNC
  ✓  Sem sensor fault
  ✓  Sem late events
  ✓  RPM medido       actual=498   expected=500   err=0.4%
  ✓  VE da tabela     actual=85    expected=85    err=0
  ✓  Advance          actual=8     expected=8     err=0.0°
  ✓  Injection PW     actual=6.1ms expected=6.2ms err=1.6%

════════════════════════════════════════════════════════════
  RESULTADOS: 28/28 PASS   0 FAIL
════════════════════════════════════════════════════════════
  ✓ RPM=  500
  ✓ RPM=  750
  ✓ RPM= 1000
  ✓ RPM= 1500
  ✓ RPM= 2000
  ✓ RPM= 3000
```
