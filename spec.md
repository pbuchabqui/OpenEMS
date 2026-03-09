# OpenEMS — Especificação Funcional

**Versão do firmware:** OpenEMS_v1.1
**Plataforma alvo:** Teensy 3.5 (NXP MK64FX512VMD12, ARM Cortex-M4 @ 120 MHz)
**Data:** 2026-03-09

---

## Índice

1. [Visão Geral](#1-visão-geral)
2. [Arquitetura de Software](#2-arquitetura-de-software)
3. [Loop de Fundo e Temporização](#3-loop-de-fundo-e-temporização)
4. [Decodificação de Posição de Virabrequim (CKP)](#4-decodificação-de-posição-de-virabrequim-ckp)
5. [Aquisição de Sensores](#5-aquisição-de-sensores)
6. [Cálculo de Combustível](#6-cálculo-de-combustível)
7. [Cálculo de Ignição](#7-cálculo-de-ignição)
8. [Detecção de Detonação (Knock)](#8-detecção-de-detonação-knock)
9. [Controles Auxiliares](#9-controles-auxiliares)
10. [Partida a Frio e Pós-Partida](#10-partida-a-frio-e-pós-partida)
11. [Scheduler de Injeção e Ignição](#11-scheduler-de-injeção-e-ignição)
12. [Protocolo TunerStudio (UART)](#12-protocolo-tunerstudio-uart)
13. [Protocolo CAN Bus](#13-protocolo-can-bus)
14. [Armazenamento em Flash (FlexNVM)](#14-armazenamento-em-flash-flexnvm)
15. [Status, Diagnóstico e Limp Mode](#15-status-diagnóstico-e-limp-mode)
16. [Periféricos de Hardware](#16-periféricos-de-hardware)
17. [Convenções de Código](#17-convenções-de-código)

---

## 1. Visão Geral

O OpenEMS é um sistema de gerenciamento de motor (EMS) em tempo real escrito em C++17, destinado a motores de 4 cilindros. Toda a lógica roda sem alocação dinâmica de memória.

| Parâmetro | Valor |
|-----------|-------|
| MCU | NXP MK64FX512VMD12 (Teensy 3.5) |
| Núcleo | ARM Cortex-M4 @ 120 MHz |
| Clock de barramento | 60 MHz |
| Linguagem | C++17 |
| Biblioteca padrão | Mínima (sem `new`/`delete`) |
| Runtime | Teensyduino (API Arduino) |
| Número de cilindros | 4 |
| Roda fônica | 60-2 (dentes faltantes) |
| Ordem de ignição | 1–3–4–2 |
| Offsets de TDC por cilindro | 0°, 180°, 360°, 540° (em ciclo de 720°) |

---

## 2. Arquitetura de Software

O firmware é organizado em quatro camadas. Dependências fluem exclusivamente de cima para baixo.

```
┌─────────────────────────────────────────┐
│  APP   (ems::app)                        │  TunerStudio, CAN stack
├─────────────────────────────────────────┤
│  ENGINE (ems::engine)                    │  Combustível, ignição, auxiliares
├─────────────────────────────────────────┤
│  DRV   (ems::drv)                        │  CKP, scheduler, sensores
├─────────────────────────────────────────┤
│  HAL   (ems::hal)                        │  ADC, CAN, UART, FTM, FlexNVM
└─────────────────────────────────────────┘
                   ▼
           Hardware Teensy 3.5
```

### Módulos por camada

| Camada | Arquivo | Responsabilidade |
|--------|---------|-----------------|
| HAL | `hal/adc.h/cpp` | ADC0/ADC1 com PDB |
| HAL | `hal/ftm.h/cpp` | FlexTimer (PWM, output compare, input capture) |
| HAL | `hal/can.h/cpp` | Driver FlexCAN |
| HAL | `hal/uart.h/cpp` | UART0 via USB CDC |
| HAL | `hal/flexnvm.h/cpp` | Armazenamento em flash |
| DRV | `drv/ckp.h/cpp` | Decodificação de posição de virabrequim |
| DRV | `drv/sensors.h/cpp` | Agregação e validação de sensores |
| ENGINE | `engine/fuel_calc.h/cpp` | Cálculo de injeção de combustível |
| ENGINE | `engine/ign_calc.h/cpp` | Cálculo de avanço de ignição |
| ENGINE | `engine/knock.h/cpp` | Detecção e retardo de detonação |
| ENGINE | `engine/auxiliaries.h/cpp` | IACV, wastegate, VVT, ventilador |
| ENGINE | `engine/quick_crank.h/cpp` | Enriquecimento de partida/pós-partida |
| ENGINE | `engine/table3d.h/cpp` | Interpolação bilinear 16×16 |
| ENGINE | `engine/cycle_sched.h/cpp` | Planejamento angular de ciclo |
| ENGINE | `engine/ecu_sched.h/cpp` | Scheduler hardware (FTM0) |
| APP | `app/tuner_studio.h/cpp` | Protocolo TunerStudio |
| APP | `app/can_stack.h/cpp` | Aplicação CAN (WBO2, diagnóstico) |

---

## 3. Loop de Fundo e Temporização

`main()` configura o NVIC e executa um scheduler cooperativo baseado em `millis()`:

| Período | Tarefa |
|---------|--------|
| 2 ms | Recálculo de combustível e ignição; `ecu_sched_commit_calibration` |
| 10 ms | Atualização de PID: IACV, VVT, wastegate |
| 20 ms | Serviço do protocolo TunerStudio; tarefas auxiliares de 20 ms |
| 50 ms | Sensores lentos (pressão de combustível, óleo); envio de frame CAN 0x400 |
| 100 ms | Sensores muito lentos (CLT, IAT); frame CAN 0x401; atualização de STFT; save de sync seed |
| 500 ms | Flush de calibração para FlexNVM; save do mapa de knock |

### Watchdog

Um watchdog de hardware roda no **PIT1** com período de **100 ms**. A função `pit1_kick()` deve ser chamada em toda iteração do loop principal ou o MCU resetará.

### Hierarquia de prioridade de interrupções

| IRQ | Periférico | Prioridade | Finalidade |
|-----|-----------|------------|-----------|
| 71 | FTM3 | 1 (mais alta) | Captura de dente CKP |
| 42 | FTM0 | 4 | Disparo de injeção/ignição |
| 39 | ADC0 | 5 | Amostragem ADC completa |
| 68 | PIT0 | 11 | Contador de timestamp em µs |
| 69 | PIT1 | 12 | Reload do watchdog |

---

## 4. Decodificação de Posição de Virabrequim (CKP)

**Arquivo:** `drv/ckp.h/cpp`
**Periférico:** FTM3 CH0 (PTD0, ALT4), captura de borda de subida
**Clock FTM3:** 60 MHz (PS=2) → 16,67 ns/tick

### Roda fônica

| Parâmetro | Valor |
|-----------|-------|
| Dentes totais (posições) | 60 |
| Dentes físicos (reais) | 58 |
| Dentes faltantes | 2 (gap) |
| Ângulo por posição | 6,0° |

### Máquina de estados de sincronização

```
WAIT_GAP ──(gap detectado, ≥55 dentes)──► HALF_SYNC
    ▲                                           │
    │                               (2º gap detectado, ≥55 dentes)
    │                                           ▼
LOSS_OF_SYNC ◄──(gap <55 dentes)────── FULL_SYNC
    │                                           ▲
    └──(gap detectado)──► re-entra HALF_SYNC ──┘
```

| Transição | Condição |
|-----------|----------|
| WAIT_GAP → HALF_SYNC | Gap detectado + ≥ 55 dentes contados |
| HALF_SYNC → FULL_SYNC | 2º gap detectado + ≥ 55 dentes |
| HALF_SYNC → LOSS_OF_SYNC | Gap antes de 55 dentes (falso gap) |
| FULL_SYNC → LOSS_OF_SYNC | Gap antes de 55 dentes **ou** > 63 dentes sem gap |
| LOSS_OF_SYNC → HALF_SYNC | Gap detectado (tentativa de re-sync) |

### Detecção de gap

- **Critério:** `período_atual × 2 > média × 3` (período > 1,5× a média)
- **Tolerância de dente normal:** ±20% da média (janela: 0,8× a 1,2×)
- **Janela de histórico:** 3 amostras (FIFO deslizante)
- **Rejeição de ruído EMC:** mínimo 50 ticks FTM3 por período

### Cálculo de RPM

```
RPM_x10 = 600.000.000.000 / (60 × tooth_period_ns)
```

### Sensor de fase (CAM)

- **Periférico:** FTM3 CH1 (PTD1, ALT4), borda de subida
- Cada borda alterna `phase_A`, identificando qual revolução do ciclo de 720°
- Timeout de confirmação: 70 dentes; se expirado sem pulso CAM, retorna a HALF_SYNC

### Seed de sincronização rápida

Ao desligar a ignição, o estado de sincronização é salvo em FlexNVM. No próximo cold-start:

- Se o seed for compatível (`tooth_index = 0`, flags FULL_SYNC + PHASE_A), a sincronização é atingida em apenas 1 gap, pulando HALF_SYNC
- 8 slots rotativos no FlexRAM com CRC32 para validação
- Contadores de diagnóstico: `seed_loaded`, `seed_confirmed`, `seed_rejected`

---

## 5. Aquisição de Sensores

**Arquivo:** `drv/sensors.h/cpp`
**Periférico:** ADC0 + ADC1 (12 bits, 0–4095), com PDB sincronizado ao FTM3

### Canais de sensor

| Sensor | ADC | Canal | Descrição |
|--------|-----|-------|-----------|
| MAP | ADC0 | SE10 | Pressão absoluta do coletor |
| MAF | ADC0 | SE11 | Fluxo de ar mássico (frequência) |
| TPS | ADC0 | SE12 | Posição do acelerador |
| O2 | ADC0 | SE4B | Sensor lambda de banda estreita |
| AN1–AN4 | ADC0 | SE6B–SE9B | Entradas analógicas auxiliares |
| CLT | ADC1 | SE14 | Temperatura do líquido de arrefecimento |
| IAT | ADC1 | SE15 | Temperatura do ar de admissão |
| Pressão de combustível | ADC1 | SE5B | Pressão absoluta |
| Pressão de óleo | ADC1 | SE6B | Pressão absoluta |

> **Nota:** O campo `o2_mv` foi removido do struct `SensorData`. O sinal de O2 de banda estreita é usado apenas para detecção de falha de sensor (range check). O controle em malha fechada de lambda usa exclusivamente o sensor WBO2 via CAN (ID 0x180).

### Estratégia de amostragem

| Cadência | Sensores | Método |
|----------|----------|--------|
| A cada dente (~12×/rev) | MAP, TPS, O2 | PDB sincronizado ao FTM3 |
| 50 ms | Pressão de combustível, pressão de óleo | Polling no loop |
| 100 ms | CLT, IAT | Média de 8 amostras para suavizar resposta térmica |

### Filtros IIR

| Sensor | Fator α | Comportamento |
|--------|---------|---------------|
| MAP | 0,3 | Resposta rápida (motor) |
| O2 | 0,1 | Resposta lenta (menos jitter) |

### Conversão de unidades (ADC raw → unidade física)

| Sensor | Fórmula | Escala |
|--------|---------|--------|
| MAP | `raw × 2500 / 4095` | kPa × 10 (0–250 kPa) |
| O2 | `raw × 5000 / 4095` | mV (0–5000 mV, referência ADC = 5 V) |
| TPS | `(raw − min) × 1000 / (max − min)` | % × 10 (0–100%) |
| CLT / IAT | LUT 128 entradas lineares | °C × 10 (−40 a +150°C) |
| Pressão combustível/óleo | `raw × 2500 / 4095` | kPa × 10 |

### Detecção de falha

- Qualquer sensor com **3 amostras consecutivas fora dos limites** ativa bit de falha
- Limites de range em raw ADC:

| Sensor | min_raw | max_raw |
|--------|---------|---------|
| MAP | 50 | 4095 |
| TPS | 50 | 4095 |
| CLT | 100 | 3800 |
| IAT | 100 | 3900 |
| O2 | 10 | 4095 |
| Pressão combustível | 50 | 4050 |
| Pressão de óleo | 50 | 4050 |

### Valores de fallback em caso de falha

| Sensor | Valor de fallback | Justificativa |
|--------|-------------------|---------------|
| MAP | 101,0 kPa | Pressão atmosférica (seguro) |
| TPS | 0% | Sem enriquecimento de aceleração |
| CLT | 90°C | Evita enriquecimento excessivo de warmup |
| IAT | 25°C | Temperatura ambiente amena |

---

## 6. Cálculo de Combustível

**Arquivo:** `engine/fuel_calc.h/cpp`

### Pipeline de cálculo

```
VE (tabela 16×16)
        │
        ▼
PW_base = REQ_FUEL × (VE/100) × (MAP/MAP_REF)
        │
        ├─── × corr_clt_x256 / 256    (enriquecimento por temperatura)
        ├─── × corr_iat_x256 / 256    (correção de densidade do ar)
        ├─── × corr_warmup_x256 / 256 (enriquecimento de warmup)
        │
        ▼
PW_corrigido + dead_time(Vbatt) + AE_pw
        │
        ▼
PW_final × corr_stft_x256 / 256 × corr_ltft_x256 / 256
```

### Parâmetros de calibração padrão

| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `REQ_FUEL` | 8000 µs | Tempo base de injeção (VE=100%, MAP=100 kPa) |
| `MAP_REF` | 100 kPa | Pressão de referência |

#### Derivação de REQ_FUEL

`REQ_FUEL` é uma constante de calibração (não calculada em runtime). A fórmula teórica de derivação é:

```
REQ_FUEL [µs] = (Vd [cc] / N_cyl) × AFR_stoich × ρ_fuel [g/cc]
                ──────────────────────────────────────────────────
                Q_inj [cc/min] × (1/60 × 10⁶)
```

Onde:
- `Vd` = cilindrada total (cm³)
- `N_cyl` = número de cilindros
- `AFR_stoich` = relação ar/combustível estequiométrica
- `ρ_fuel` = densidade do combustível (g/cm³)
- `Q_inj` = vazão do injetor (cc/min)

**Referência para combustíveis comuns:**

| Combustível | AFR estequiométrico | Densidade (g/cc) | Nota |
|-------------|--------------------|--------------------|------|
| Gasolina E0 | 14,7 | 0,720 | Padrão |
| Gasolina E30 | ~13,5 | ~0,752 | Mistura 30% etanol |
| Etanol E100 | 9,0 | 0,789 | Flex puro |

**Exemplo (E0):** Motor 1600 cc / 4 cil = 400 cc/cil; injetor 240 cc/min:
```
REQ_FUEL = 400 × 14,7 × 0,720 / (240 × 16667) ≈ 8000 µs
```

**Para E30:** REQ_FUEL deve ser recalculado com AFR=13,5 e ρ=0,752, resultando em ~7900 µs (≈1,2% menor). O firmware **não** aplica correção automática por fração de etanol; o usuário deve ajustar `REQ_FUEL` conforme o combustível utilizado.

### Tabela VE

- Dimensão: **16×16** (RPM × MAP)
- Eixo RPM (×10): 500, 750, 1000, 1250, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 12000
- Eixo MAP (kPa): 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 175, 200
- Interpolação: **bilinear**, aritmética em inteiros (Q8)
- Faixa de valores: 50–110%

### Tabelas de correção

**Correção de CLT (enriquecimento por temperatura d'água):**

| CLT (°C) | Correção (×256) | Equivalente |
|----------|-----------------|-------------|
| −40 | 384 | 1,50× |
| −10 | 352 | 1,38× |
| 0 | 320 | 1,25× |
| 20 | 288 | 1,13× |
| 40 | 272 | 1,06× |
| 70 | 256 | 1,00× (neutro) |
| 90 | 248 | 0,97× |
| 110 | 240 | 0,94× |

**Correção de IAT (densidade do ar):**

| IAT (°C) | Correção (×256) | Equivalente |
|----------|-----------------|-------------|
| −20 | 272 | 1,06× |
| 0 | 264 | 1,03× |
| 20 | 256 | 1,00× (neutro) |
| 40 | 248 | 0,97× |
| 60 | 240 | 0,94× |
| 80 | 232 | 0,91× |
| 100 | 224 | 0,88× |
| 120 | 216 | 0,84× |

**Enriquecimento de warmup (adicional para motor frio):**

| CLT (°C) | Multiplicador (×256) | Equivalente |
|----------|----------------------|-------------|
| −40 | 420 | 1,64× |
| −10 | 380 | 1,48× |
| 0 | 350 | 1,37× |
| 20 | 320 | 1,25× |
| 40 | 290 | 1,13× |
| ≥ 70 | 256 | 1,00× (inativo) |

**Dead-time por tensão de bateria:**

| Vbatt (mV) | Dead-time (µs) |
|------------|----------------|
| 9000 | 1400 |
| 10000 | 1200 |
| 11000 | 1000 |
| 12000 | 900 |
| 13000 | 800 |
| 14000 | 700 |
| 15000 | 650 |
| 16000 | 600 |

### Enriquecimento em aceleração (AE)

- **Gatilho:** variação de TPS (TPS-dot) acima de um limiar configurável (padrão: 0,5 %/ms)
- **Magnitude:** proporcional à taxa de variação do TPS
- **Sensibilidade por CLT:** fator reduzido em temperaturas mais altas (11 a 4, de −40°C a 110°C)
- **Decaimento:** exponencial ao longo de 8 ciclos configuráveis (`taper_cycles`)

### Trim adaptativo de combustível (STFT e LTFT)

**STFT (Short-Term Fuel Trim):**
- Controlador PID sobre erro de lambda: Kp = 3/100, Ki = 1/200
- Limite: ±25,0% (±250 em unidades ×10)
- Atualizado a cada **100 ms**
- Condições de habilitação: CLT > 70°C, O2 válido, sem AE ativo, sem corte de rotação
- Unidade: `int16_t`, % × 10

**LTFT (Long-Term Fuel Trim):**
- Integração do STFT por célula (16×16), com decaimento de 1/64 por atualização
- Persiste em FlexRAM (256 bytes) e é carregado no cold-start
- Funções de acesso: `fuel_ltft_load_cell()` / `fuel_ltft_store_cell()`

---

## 7. Cálculo de Ignição

**Arquivo:** `engine/ign_calc.h/cpp`

### Pipeline de cálculo

```
Avanço_base (tabela 16×16)
        │
        + corr_clt_deg     (avanço adicional em motor frio)
        + corr_iat_deg     (retardo em ar quente)
        − knock_retard_deg (retardo por detonação)
        │
        ▼
clamp [−10°, +40°] BTDC
        │
        ▼
Ângulo de centelha final
```

### Tabela de avanço

- Dimensão: **16×16** (RPM × carga)
- Mesmos eixos da tabela VE
- Armazenamento: offset −40° (valor 0 na tabela = −40° BTDC)
- Faixa típica: 40–70° BTDC no storage; clampado para −10° a +40° na saída
- Interpolação: bilinear Q10

### Dwell por tensão de bateria

| Vbatt (mV) | Dwell (ms) |
|------------|------------|
| 9000 | 4,2 |
| 10000 | 3,8 |
| 11000 | 3,4 |
| 12000 | 3,2 |
| 13000 | 3,0 |
| 14000 | 2,8 |
| 15000 | 2,5 |
| 16000 | 2,2 |

### Ângulo de início de dwell

```
dwell_angle_x10 = (dwell_ms_x10 × RPM × 360) / 600000
dwell_start_deg = spark_deg − dwell_angle_deg
```

- Dwell máximo: 3599 (359,9°) — nunca maior que 1 revolução

### Ordem de ignição e offsets

| Cilindro | Posição na ordem | Offset TDC |
|----------|-----------------|------------|
| 1 | 1ª | 0° |
| 3 | 2ª | 180° |
| 4 | 3ª | 360° |
| 2 | 4ª | 540° |

---

## 8. Detecção de Detonação (Knock)

**Arquivo:** `engine/knock.h/cpp`
**Periférico:** CMP0 com DAC de 6 bits (VOSEL 0–63) como limiar de tensão

### Janela de detecção

- **Ativa:** 10° a 90° ATDC (após TDC) por cilindro
- Controlada por `knock_window_open(cyl)` / `knock_window_close(cyl)`
- Eventos do CMP0 ISR são contabilizados **somente** quando a janela está aberta
- Limiar padrão de eventos: **3 eventos** por janela

### Algoritmo de retardo

| Parâmetro | Valor |
|-----------|-------|
| Passo de retardo por evento | 2,0° (incremento de 20 em unidades ×10) |
| Retardo máximo por cilindro | 10,0° (100 em unidades ×10) |
| Passo de recuperação | 0,1° por ciclo limpo |
| Delay antes de iniciar recuperação | 10 ciclos limpos |

### Ajuste do limiar (VOSEL)

- Cada evento de knock decrementa VOSEL em 2 (limiar mais sensível)
- A cada 100 ciclos limpos, VOSEL incrementa em 1 (limiar menos sensível)
- Faixa: 0–63; padrão inicial: 32
- Estado persiste em FlexNVM (slot dedicado no knock map)

### Saída

```cpp
uint16_t knock_get_retard_x10(cyl)  // retardo em 0,1°, por cilindro
```

---

## 9. Controles Auxiliares

**Arquivo:** `engine/auxiliaries.h/cpp`
**Periférico PWM:** FTM1 (IACV + wastegate) e FTM2 (VVT), ambos a **15 Hz**

### 9.1 Válvula de controle de ar em marcha lenta (IACV)

**Pinos:** FTM1 CH0 (PTA8) — IACV, FTM1 CH1 (PTA9) — wastegate

**Dois modos de operação:**

**Modo warmup (CLT < 60°C):** Tabela de feedforward por CLT

| CLT (°C) | Duty cycle (% × 10) |
|----------|---------------------|
| −40 | 620 (62%) |
| −10 | 560 (56%) |
| 10 | 500 (50%) |
| 30 | 440 (44%) |
| ≥ 60 | Transição para PID |

**Modo PID (CLT ≥ 60°C):**
- Kp = 2, Kd = 5/2, integrador limitado a ±300 (×10)
- Inibição por taxa de variação de RPM: spike > 40 RPM/tick (20 ms) desativa PID
- Anti-windup: duty clampado a [0, 1000]

**RPM alvo por CLT:**

| CLT (°C) | RPM alvo |
|----------|----------|
| −40 | 1200 |
| −10 | 1100 |
| 20 | 1000 |
| 50 | 900 |
| ≥ 70 | 800 |

### 9.2 Wastegate (controle de boost)

**Parâmetros PID:** Kp = 8/100, Ki = 1/100, limite ±250

**Proteção de overboost:**
- Ativada se pressão exceder alvo em mais de **20 kPa** por **500 ms**
- Ação de failsafe: duty = 0% (wastegate totalmente aberta)

**Tabela de alvo de boost (8×8, RPM × TPS):** 105–180 kPa × 10

### 9.3 VVT (Variable Valve Timing)

**Pinos:** FTM2 CH0 (PTA10) — escape, FTM2 CH1 (PTA11) — admissão

- PID independente para admissão e escape: Kp = 1,2, Ki = 1/20
- Offset base: 50%; saída clampada a [0%, 100%]
- Habilitação: requer FULL_SYNC + pulso CAM recente
- Timeout de confirmação: **200 ms**

**Tabelas de alvo (12×12, RPM × carga):**

| Atuador | Faixa do alvo |
|---------|---------------|
| Escape (ESC) | 60–160° (×10) |
| Admissão (ADM) | 80–330° (×10) |

### 9.4 Ventilador de arrefecimento

| Evento | Temperatura |
|--------|-------------|
| Liga | 95°C |
| Desliga | 90°C (histerese) |

### 9.6 Prime Pulse de injeção

O prime pulse é uma injeção **simultânea única** em todos os 4 injetores, disparada logo no início do cranking para preencher o coletor de admissão.

| Parâmetro | Valor |
|-----------|-------|
| Condição de disparo | 5º dente CKP recebido com RPM < 700 |
| Requisito de sincronização | **Nenhum** — dispara independente de sync state |
| Tipo | Injeção simultânea (todos os 4 injetores) |
| Duração | `REQ_FUEL × kCrankFuelMult(CLT)` (mesma tabela de cranking) |
| Clamp máximo | 30 000 µs (proteção contra afogamento) |
| One-shot | Sim — `s_prime_done` reseta apenas em `quick_crank_reset()` (key-off) |
| Mecanismo | ISR CKP → `prime_on_tooth()` → flag → loop de fundo → `ecu_sched_fire_prime_pulse()` |

**Fluxo de disparo:**

```
ISR FTM3 (prioridade 1)          Loop de fundo (2 ms)
──────────────────────           ────────────────────────────
ckp_ftm3_ch0_isr()               quick_crank_consume_prime()
  → prime_on_tooth(snap)            → retorna pw_us (atomic)
      conta dentes                ecu_sched_fire_prime_pulse(pw)
      no 5º: calcula PW              → arma CH0-CH3 ON em FTM0
      g_prime_pending = true         FTM0_IRQHandler → re-arma OFF
```

**Tabela de PW de prime pulse por CLT** (idêntica à tabela de cranking):

| CLT (°C) | PW prime típica (µs) |
|----------|---------------------|
| −40 | ~24 000 (3,0× REQ_FUEL) |
| 0 | ~19 200 (2,4×) |
| 20 | ~16 000 (2,0×) |
| 40 | ~13 600 (1,7×) |
| ≥ 70 | ~10 000 (1,25×) |

### 9.5 Bomba de combustível

| Evento | Comportamento |
|--------|---------------|
| Key-on | Priming por **2 s** |
| Motor girando | Bomba ligada continuamente |
| Queda de RPM a zero | Desliga após **3 s** |

---

## 10. Partida a Frio e Pós-Partida

**Arquivo:** `engine/quick_crank.h/cpp`

> **Prime pulse de injeção:** veja seção 9.6. O prime pulse é disparado no 5º dente CKP (sem requisito de sincronização), antes mesmo da detecção de cranking ser confirmada. O enriquecimento cíclico descrito abaixo começa após o FULL_SYNC.

### Detecção de cranking

| Parâmetro | Valor |
|-----------|-------|
| Entrada em cranking | RPM < 450 |
| Saída de cranking | RPM > 700 (histerese) |
| Requer | FULL_SYNC ativo |

### Enriquecimento de cranking por CLT

| CLT (°C) | Multiplicador (×256) | Equivalente |
|----------|----------------------|-------------|
| −40 | 768 | 3,00× |
| −20 | 614 | 2,40× |
| 0 | 512 | 2,00× |
| 20 | 435 | 1,70× |
| 40 | 358 | 1,40× |
| 60 | 320 | 1,25× |
| ≥ 80 | 294 | 1,15× |

**Parâmetros adicionais no cranking:**
- Avanço de centelha: **fixo em 8° BTDC**
- Pulsewidth mínima: **2500 µs** (garante combustível na câmara em baixo RPM)

### Janela de afterstart

Ao sair do cranking, o modo de afterstart é ativado por um período e com multiplicadores dependentes de CLT:

| CLT (°C) | Duração (ms) | Mult. inicial (×256) |
|----------|-------------|----------------------|
| −40 | 2400 | 346 (1,35×) |
| −20 | 2000 | 333 (1,30×) |
| 0 | 1700 | 320 (1,25×) |
| 20 | 1400 | 307 (1,20×) |
| 40 | 1000 | 294 (1,15×) |
| 60 | 700 | 281 (1,10×) |
| ≥ 80 | 500 | 269 (1,05×) |

- Decaimento: **linear** do multiplicador inicial até 1,0× ao longo da duração
- Limite: clampado em [1,0×, 2,0×]

---

## 11. Scheduler de Injeção e Ignição

**Arquivos:** `engine/cycle_sched.h/cpp`, `engine/ecu_sched.h/cpp`
**Periférico:** FTM0, output compare, PS=64 → **1,875 MHz** (533 ns/tick)

### Canais FTM0

| Canal FTM0 | Função | Pino |
|------------|--------|------|
| CH2 | INJ1 | — |
| CH3 | INJ2 | — |
| CH4 | IGN4 | PTD4 |
| CH5 | IGN3 | PTD5 |
| CH6 | IGN2 | PTD6 |
| CH7 | IGN1 | PTD7 |

### Domínio angular

Eventos são armazenados como `(tooth_index, sub-fração_x256, phase)`, não como timestamps absolutos. A cada dente capturado pelo ISR do FTM3, o offset FTM0 é calculado:

```
offset_ftm0 = (sub_frac × tooth_period_ftm0) >> 8
```

### Eventos por cilindro por ciclo

| Evento | Descrição |
|--------|-----------|
| `INJ_ON` | Abre injetor (início da injeção) |
| `INJ_OFF` | Fecha injetor (fim da pulsewidth) |
| `DWELL_START` | Inicia carga da bobina |
| `SPARK` | Dispara centelha (output compare → clear) |

### Modos de operação por estado de sincronização

| Estado CKP | Modo injeção | Modo ignição |
|------------|-------------|-------------|
| HALF_SYNC | Simultânea (4 cilindros juntos) ou semi-sequencial (bancos alternados) | Wasted spark |
| FULL_SYNC | Sequencial por cilindro, por fase | Sequencial por cilindro |

### Lead de acionamento

| Evento | Antecipação em relação ao TDC |
|--------|-------------------------------|
| SOI (início de injeção) | 10 dentes antes do TDC |
| Dwell start | 4 dentes antes do ângulo calculado |
| Spark | 4 dentes antes do ângulo calculado |

### Limites e proteções

| Parâmetro | Limite | Ação |
|-----------|--------|------|
| Avanço máximo | 60° BTDC | Clamp + contador de clamps |
| Dwell máximo | 18750 ticks (10 ms) | Clamp |
| PW máxima de injeção | 37500 ticks (20 ms) | Clamp |
| Evento tardio | Dente já passou ao armar | Não arma; incrementa `g_late_event_count` |
| Perda de sincronização | LOSS_OF_SYNC | Limpa tabela de ângulos; saídas passivas |

### Estratégia de fim de injeção (EOI)

O EOI (End of Injection) é calculado puramente como:

```
EOI_deg = SOI_deg + PW_deg   (mod 720°)
```

**Não há parâmetro de IVC (Intake Valve Closing) implementado.** O scheduler não verifica se o EOI ultrapassa o ângulo de fechamento de admissão.

| Implicação | Descrição |
|------------|-----------|
| PW curta (< 5 ms @ 1500 RPM) | EOI tipicamente antes do IVC — injeção sobre válvula aberta |
| PW longa (cranking, carga alta) | EOI pode ultrapassar IVC — injeção parcialmente no coletor fechado |
| Efeito prático | Aceitável em motores com MAP; leve perda de eficiência de mistura em alta carga |

**Ponto de extensão futuro:** adicionar parâmetro de calibração `ivc_deg` e validação `assert(EOI_deg ≤ ivc_deg + margem)` no `cycle_sched.cpp`.

### Double-buffering

O background escreve em `g_pending[4]` com protocolo de invalidação atômica. O ISR lê apenas após o bit `valid` estar setado, prevenindo leitura parcial.

---

## 12. Protocolo TunerStudio (UART)

**Arquivo:** `app/tuner_studio.h/cpp`
**Transporte:** USB CDC (UART0), **115200 bps**
**Assinatura:** `"OpenEMS_v1.1"`

### Comandos suportados

| Byte | Comando | Resposta |
|------|---------|----------|
| `Q` | Consulta de assinatura | `"OpenEMS_v1.1"` (13 bytes) |
| `H` | Idem (legado) | `"OpenEMS_v1.1"` |
| `S` | Versão de firmware | `"OpenEMS_fw_1.1"` |
| `F` | Versão de protocolo | `"001"` |
| `C` | Teste de comunicação | `0x00` + `0xAA` (2 bytes) |
| `A` / `O` | Bloco de dados em tempo real | 64 bytes |
| `r` | Leitura de página de calibração | `r[page:u8][offset:u16 LE][len:u16 LE]` → dados |
| `w` | Escrita de página de calibração | `w[page:u8][offset:u16 LE][len:u16 LE][dados]` → `0x00` OK / `0x01` erro |

### Páginas de calibração

| Página | Tamanho | Conteúdo |
|--------|---------|---------|
| 0 | 512 bytes | Configuração geral |
| 1 | 256 bytes | Tabela VE (16×16 = 256 bytes) |
| 2 | 256 bytes | Tabela de avanço (16×16) |

### Bloco de dados em tempo real (64 bytes)

| Offset | Tipo | Campo | Escala / Offset |
|--------|------|-------|-----------------|
| 0–1 | U16 LE | RPM | ÷ 1 (valor bruto) |
| 2 | U8 | MAP (kPa) | ÷ 10 |
| 3 | U8 | TPS (%) | ÷ 10 |
| 4 | S8 | CLT (°C) | valor + 40 |
| 5 | S8 | IAT (°C) | valor + 40 |
| 6 | U8 | Lambda O2 | λ × 1000 ÷ 4 |
| 7 | U8 | PW injetor | ms × 10 |
| 8 | U8 | Avanço (°) | valor + 40 |
| 9 | U8 | Célula VE (%) | valor bruto |
| 10 | S8 | STFT (%) | valor + 100 |
| 11 | U8 | Status bits | veja seção 15 |
| 12–15 | U32 LE | `late_events` | contador |
| 16–19 | U32 LE | `late_max_delay_ticks` | ticks FTM0 |
| 20 | U8 | `queue_depth_peak` | — |
| 21 | U8 | `queue_depth_last_cycle` | — |
| 22–25 | U32 LE | `cycle_schedule_drops` | contador |
| 26–29 | U32 LE | `calibration_clamps` | contador |
| 30 | U8 | `sync_state_raw` | enum CkpSyncState |

---

## 13. Protocolo CAN Bus

**Arquivo:** `app/can_stack.h/cpp`
**Driver:** `hal/can.h/cpp` (FlexCAN)
**Pinos:** PTA12 (CAN0_TX), PTA13 (CAN0_RX) → ALT2
**Velocidade:** 500 kbps

### Configuração de bit timing

| Parâmetro | Valor |
|-----------|-------|
| Clock fonte | Bus clock (60 MHz) |
| PRESDIV | 5 (÷6 → 10 MHz) |
| PROPSEG | 5 |
| PSEG1 | 7 |
| PSEG2 | 4 |
| RJW | 3 |
| Tq por bit | 20 → 2 µs/bit = 500 kbps |

### Frame de recepção — WBO2 lambda (ID 0x180)

| Byte(s) | Conteúdo | Escala |
|---------|---------|--------|
| 0–1 | λ × 1000 (U16 LE) | Divide por 1000 para obter λ |
| 2 | Byte de status | — |

- **Timeout:** 500 ms; após expirar, `can_stack_wbo2_fault()` retorna `true`
- **Fallback:** λ = 1050 (λ = 1,05) via `can_stack_lambda_milli_safe()`
- **ID configurável** via `can_stack_set_wbo2_rx_id()` (padrão: 0x180)

### Frame de transmissão A — Diagnóstico (ID 0x400, 50 ms)

| Byte | Conteúdo | Escala |
|------|---------|--------|
| 0–1 | RPM (U16 LE) | — |
| 2 | MAP (kPa) | ÷ 10 |
| 3 | TPS (%) | ÷ 10 |
| 4 | CLT (°C) | valor + 40 |
| 5 | Avanço (°) | valor + 40 |
| 6 | PW injetor | ms × 10 |
| 7 | Status bits | veja seção 15 |

### Frame de transmissão B — Diagnóstico (ID 0x401, 100 ms)

| Byte | Conteúdo | Escala |
|------|---------|--------|
| 0–1 | Pressão de combustível (U16 LE) | kPa × 10 |
| 2–3 | Pressão de óleo (U16 LE) | kPa × 10 |
| 4 | IAT (°C) | valor + 40 |
| 5 | STFT (%) | valor + 100 |
| 6 | VVT admissão (%) | — |
| 7 | VVT escape (%) | — |

---

## 14. Armazenamento em Flash (FlexNVM)

**Arquivo:** `hal/flexnvm.h/cpp`

### Layout de memória

| Região | Base | Conteúdo | Tamanho |
|--------|------|---------|---------|
| FlexRAM | `0x14000000` | LTFT 16×16 | 256 bytes (offset 0) |
| FlexRAM | `0x14000100` | Knock map 8×8 | 64 bytes (offset 256) |
| FlexRAM | `0x14000140` | Sync seeds (8 slots × 32 bytes) | 256 bytes (offset 320) |
| FlexNVM D-Flash | `0x10000000` | Páginas de calibração | 32 páginas × 4 KB |

### LTFT (Long-Term Fuel Trim)

- Tabela 16×16 de `int8_t` (256 células)
- Índices: `rpm_i` ∈ [0, 15], `load_i` ∈ [0, 15]
- Leitura: `nvm_read_ltft(rpm_i, load_i)` → `int8_t`
- Escrita: `nvm_write_ltft(rpm_i, load_i, val)` → `bool`

### Mapa de detonação (Knock Map)

- Tabela 8×8 de `int8_t` (64 células), retardo em 0,1°
- Índices: `rpm_i` ∈ [0, 7], `load_i` ∈ [0, 7]
- Reset: `nvm_reset_knock_map()` zera todas as células

### Páginas de calibração (D-Flash)

- Tamanho de setor: 4 KB
- 32 páginas disponíveis
- `nvm_save_calibration(page, src, len)`: apaga setor → programa → verifica readback
- `nvm_load_calibration(page, dst, len)`: leitura direta

### Sync seed de runtime

Estrutura `RuntimeSyncSeed` (32 bytes):

| Campo | Tipo | Descrição |
|-------|------|---------|
| `magic` | U16 | `0x5343` ("SC") |
| `version` | U8 | `1` |
| `flags` | U8 | VALID, FULL_SYNC, PHASE_A |
| `tooth_index` | U16 | 0–57 |
| `decoder_tag` | U16 | `0x3C02` (60-2) |
| `sequence` | U32 | Contador crescente (slot mais recente vence) |
| `crc32` | U32 | Validação de integridade |

- 8 slots rotativos; leitura retorna o slot com maior `sequence` e CRC32 válido
- Para reaquização rápida no boot: `tooth_index` deve ser 0

---

## 15. Status, Diagnóstico e Limp Mode

**Arquivo:** `app/status_bits.h`

### Byte de status (8 bits)

| Bit | Máscara | Nome | Significado |
|-----|---------|------|-------------|
| 0 | `0x01` | `STATUS_SYNC_FULL` | Sincronização CKP completa (FULL_SYNC) |
| 1 | `0x02` | `STATUS_PHASE_A` | Fase CAM detectada |
| 2 | `0x04` | `STATUS_SENSOR_FAULT` | Falha em algum sensor |
| 3 | `0x08` | `STATUS_LIMP_MODE` | Modo limp ativo |
| 4 | `0x10` | `STATUS_SCHED_LATE` | Evento de scheduler atrasado detectado |
| 5 | `0x20` | `STATUS_SCHED_DROP` | Ciclo de scheduler descartado |
| 6 | `0x40` | `STATUS_SCHED_CLAMP` | Clamp de calibração ativado |
| 7 | `0x80` | `STATUS_WBO2_FAULT` | Timeout do sensor WBO2 (> 500 ms) |

### Limp Mode

| Parâmetro | Valor |
|-----------|-------|
| Condição de ativação | Falha crítica (MAP **ou** CLT) + RPM > 3000 |
| Ação | Corte de injeção e ignição acima de 3000 RPM |
| Limite de RPM em limp | 3000 RPM (30000 × 10) |

### Watchdog de hardware

| Parâmetro | Valor |
|-----------|-------|
| Periférico | PIT1 |
| Período | 100 ms |
| Ação no timeout | `system_reset()` (reset do MCU) |

---

## 16. Periféricos de Hardware

| Periférico | Finalidade | Configuração |
|-----------|-----------|-------------|
| FTM3 CH0 (PTD0) | Captura CKP (borda de subida) | PS=2, 60 MHz, prioridade IRQ=1 |
| FTM3 CH1 (PTD1) | Captura fase CAM | PS=2, 60 MHz, prioridade IRQ=2 |
| FTM0 CH2–7 | Output compare: injeção + ignição | PS=64, 1,875 MHz, prioridade IRQ=4 |
| FTM1 CH0 (PTA8) | PWM IACV | 15 Hz |
| FTM1 CH1 (PTA9) | PWM wastegate | 15 Hz |
| FTM2 CH0 (PTA10) | PWM VVT escape | 15 Hz |
| FTM2 CH1 (PTA11) | PWM VVT admissão | 15 Hz |
| ADC0 | MAP, MAF, TPS, O2, AN1–4 | 12 bits, média de 4 amostras |
| ADC1 | CLT, IAT, pressões | 12 bits |
| PDB0 | Trigger de ADC sincronizado ao FTM3 | TRGSEL=FTM3 |
| PIT0 | Contador de timestamp | 1 µs (60 MHz / 60), prioridade IRQ=11 |
| PIT1 | Watchdog | 100 ms (60 MHz / 6.000.000), prioridade IRQ=12 |
| FlexCAN (PTA12/13) | CAN 0 | 500 kbps, ALT2 |
| UART0 / USB CDC | TunerStudio | 115200 bps |
| FlexNVM | Calibração, LTFT, knock, seed | FlexRAM + D-Flash |
| CMP0 | Detecção de knock (comparador) | DAC 6 bits (VOSEL) |

---

## 17. Convenções de Código

### Namespaces

| Namespace | Camada |
|-----------|--------|
| `ems::hal` | HAL |
| `ems::drv` | Driver |
| `ems::engine` | Engine |
| `ems::app` | Application |

### Prefixos de variáveis

| Prefixo | Significado |
|---------|-------------|
| `g_` | Global de módulo ou arquivo |
| `k` | Constante em tempo de compilação |
| `s_` | Local estático |

### Sufixos de unidade (obrigatórios em quantidades físicas)

| Sufixo | Significado |
|--------|-------------|
| `_x10` | × 10 (uma casa decimal) |
| `_x100` | × 100 |
| `_x256` | Ponto fixo Q8.8 (× 256) |
| `_kpa` | Quilopascal |
| `_degc` | Grau Celsius |
| `_mv` | Milivolt |
| `_ms` | Milissegundo |
| `_us` | Microssegundo |
| `_ns` | Nanossegundo |
| `_pct` | Porcentagem |
| `_rpm` | Rotações por minuto |

### Aritmética de ponto fixo

Ponto flutuante é evitado. Correções e razões são carregadas como inteiros escalados:

```cpp
// Q8.8: divisor = 256
int16_t corr_clt_x256 = 312;  // 312/256 ≈ 1.22 (+22% enriquecimento)
uint32_t pw_us = (base_pw_us * corr_clt_x256) >> 8;
```

Subtração circular de timer (16 bits unsigned):

```cpp
uint16_t delta = (uint16_t)(current - previous);  // correto mesmo com overflow
```

### Compilação condicional

APIs exclusivas para teste ficam guardadas por `#ifdef EMS_HOST_TEST`. Isso permite testes unitários em host sem poluir o firmware de produção.

---

*Gerado em 2026-03-09 a partir do código-fonte do OpenEMS.*
