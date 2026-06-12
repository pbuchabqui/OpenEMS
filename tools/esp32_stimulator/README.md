# ESP32 Sensor Stimulator — OpenEMS

Gera todos os sinais de **entrada** do STM32H562: roda fônica CKP 60-2, cam CMP,
e 10 canais analógicos de sensor. Permite testar o ECU em bancada sem motor.

---

## Ligações

### Sinais digitais (conexão directa)

| ESP32 GPIO | STM32 | Sinal | Notas |
|-----------|-------|-------|-------|
| GPIO2 | PA0 | CKP 60-2 | 58 dentes + gap de 2, 3.3 V |
| GPIO4 | PA1 | CMP | 1 pulso por ciclo completo (2 rotações) |
| GND | GND | Referência | **Obrigatório** |

### Sensores analógicos — DAC interno (sem filtro externo)

| ESP32 GPIO | STM32 | Sensor | Tipo |
|-----------|-------|--------|------|
| GPIO25 | PA2 (ADC1_IN3) | MAP | DAC1 — analógico verdadeiro 8-bit |
| GPIO26 | PA4 (ADC1_IN5) | TPS legacy | DAC2 — analógico verdadeiro 8-bit |

### Sensores analógicos — LEDC PWM + filtro RC

**Filtro por canal:** série 10 kΩ + 100 nF para GND  
Frequência de corte: fc = 1 / (2π × 10 kΩ × 100 nF) ≈ **159 Hz**  
Atenuação a 39 kHz PWM: ≥ 48 dB — ripple < 0.5 mV

```
ESP32 GPIO ──[10 kΩ]──┬── STM32 ADC pin
                       │
                    [100 nF]
                       │
                      GND
```

| ESP32 GPIO | STM32 | Sensor | Range simulado |
|-----------|-------|--------|----------------|
| GPIO13 | PC2 (ADC2_IN1) | CLT | −40 a +150 °C |
| GPIO12 | PC3 (ADC2_IN2) | IAT | −40 a +150 °C |
| GPIO14 | PB0 (ADC1_IN7) | APP1 | 0–100% |
| GPIO27 | PB1 (ADC1_IN8) | APP2 | 0–100% (espelho APP1) |
| GPIO16 | PC4 (ADC2_IN13) | FUEL_PRESS | 0–5 bar |
| GPIO17 | PC5 (ADC2_IN14) | OIL_PRESS | 0–5 bar |
| GPIO18 | PC0 (ADC1_IN9) | ETB_TPS1 | 0–100% |
| GPIO19 | PC1 (ADC1_IN10) | ETB_TPS2 | 0–100% invertido |

---

## Nota sobre CLT / IAT

Se a PCB do ECU tiver **pull-up** para 3.3 V nos pinos CLT/IAT (circuito NTC típico):

```
3.3V ──[R_pull]──┬── ADC pin
                  │
              [NTC sensor]
                  │
                 GND
```

O pull-up irá **conflituar** com a tensão forçada pelo ESP32.

**Solução A (recomendada):** remover o resistor de pull-up na PCB (DNP) durante
os testes em bancada.

**Solução B:** interpor um buffer rail-to-rail (ex: TLV9001, LM358) entre o GPIO
e o pino ADC, com alimentação 3.3 V.

**Solução C:** se o valor do pull-up for conhecido (ex: R_pull = 2.2 kΩ), calcular
a tensão necessária na saída do ESP32 para compensar — o firmware usa uma tabela
linear, por isso a relação é simples, mas exige calibração manual.

---

## Calibração (mirrors de `sensors.cpp`)

| Sensor | Fórmula raw → valor físico | Inversa (valor → raw) |
|--------|---------------------------|----------------------|
| MAP | `kPa = raw × 300 / 4095` | `raw = kPa × 4095 / 300` |
| TPS/APP | `pct = (raw − 200) × 100 / 3695` | `raw = 200 + pct × 3695 / 100` |
| CLT/IAT | tabela linear −40..+150 °C, 128 entradas | `raw = ((T×10+400)×127/1900)×32+16` |
| FUEL/OIL | `bar = raw × 2.5 / 4095` | `raw = bar×1000×4095/2500` |

A tabela CLT/IAT usada é a tabela de **teste por defeito** (`init_tables()` em
`sensors.cpp`). Se o firmware for carregado com uma tabela NTC real (Steinhart-Hart),
é necessário recalibrar os valores de temperatura para raw.

---

## Comandos série (115200 baud)

### Parâmetros individuais

```
RPM 1500       → set RPM (50-6000)
MAP 55         → MAP em kPa (0-300)
TPS 20         → TPS em % (0-100)
CLT 90         → temperatura refrigerante °C (-40 a +150)
IAT 35         → temperatura ar admissão °C (-40 a +150)
APP 30         → pedal acelerador % (0-100)
FUEL 35        → pressão combustível bar×10 (35 = 3.5 bar)
OIL 25         → pressão óleo bar×10
ETB 25         → borboleta ETB % (TPS2 = 100-n, anti-phase)
```

### Presets

| Preset | RPM | MAP | TPS | CLT | IAT | APP | FUEL | OIL |
|--------|-----|-----|-----|-----|-----|-----|------|-----|
| `IDLE` | 700 | 35 kPa | 3% | 90°C | 25°C | 0% | 3.5 bar | 2.0 bar |
| `CRANK` | 200 | 101 kPa | 0% | 20°C | 15°C | 0% | 2.0 bar | 0.5 bar |
| `CRUISE` | 2000 | 55 kPa | 20% | 90°C | 35°C | 20% | 3.5 bar | 3.0 bar |
| `WOT` | 4000 | 100 kPa | 100% | 90°C | 40°C | 100% | 3.8 bar | 4.0 bar |
| `COAST` | 2000 | 25 kPa | 0% | 90°C | 35°C | 0% | 3.5 bar | 2.5 bar |

### Outros

```
STATUS   → imprimir estado actual + raw ADC + tensão calculada
?        → ajuda
```

---

## Diferença em relação ao `esp32_combined`

| Ferramenta | Função | GPIO25/26 |
|-----------|--------|-----------|
| `esp32_combined` | CKP gen + scope (monitora saídas ECU) | Entradas: IGN2, IGN3 |
| `esp32_stimulator` | Estimulador (gera entradas para ECU) | Saídas: MAP (DAC1), TPS (DAC2) |

São sketches **mutuamente exclusivos** — usar um segundo ESP32 se se quiser
estimulação + scope simultâneos.

---

## Exemplo de sessão

```
> IDLE
  [STIM] Preset: IDLE

> STATUS
  RPM:       700
  MAP:  PA2   35 kPa   raw= 478  V=0.385 V
  TPS:  PA4    3 %     raw= 311  V=0.250 V
  CLT:  PC2   90 °C   raw=2800  V=2.256 V
  ...

> RPM 1000
  [STIM] RPM=1000

> MAP 45
  [STIM] MAP=45 kPa

> CLT 20
  [STIM] CLT=20°C
```

---

## Dependências Arduino

- ESP32 Arduino Core ≥ 2.0
- `driver/dac.h` (incluído no ESP-IDF ≥ 4.4)
- `driver/ledc.h` (incluído)
- Nenhuma biblioteca externa necessária
