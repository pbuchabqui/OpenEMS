# OpenEMS — Análise de Reúso de Código
## Fontes: `pbuchabqui/ESP32_S3_EMS` + rusefi (conceitos arquiteturais)

---

## Resumo executivo

O repositório `pbuchabqui/ESP32_S3_EMS` é significativamente mais maduro do que o esperado.
Ele já implementa ~65% do núcleo de tempo real do OpenEMS, com qualidade de código alta e
abordagens técnicas corretas. A estratégia é **fork + adaptação**, não reescrita.

O rusefi contribui com conceitos arquiteturais (não código direto, já que está em STM32/C++),
especialmente no scheduler por ângulo de virabrequim e no modelo de qualidade de sincronismo.

---

## 1. Mapeamento: ESP32_S3_EMS → OpenEMS

### ✅ REUSAR DIRETAMENTE (zero ou mínima adaptação)

| Arquivo fonte | Módulo OpenEMS | Tamanho | Observação |
|---|---|---|---|
| `src/sync.c` + `include/sync.h` | `decoder/trigger_60_2` | 834 linhas | **Melhor implementação disponível.** PCNT + ETM + GPTimer. Fallback GPIO ISR para ESP32 sem ETM. |
| `src/high_precision_timing.c/h` | `scheduler/hp_timing` | 238 linhas | Phase predictor + jitter measurer + latency comp. Padrão rusefi adaptado para ESP32. |
| `src/hp_state.c/h` | `scheduler/hp_state` | 71 linhas | Estado centralizado compartilhado entre módulos de timing. Manter como está. |
| `src/mcpwm_injection_hp.c` | `scheduler/injector_driver` | 292 linhas | Timer contínuo + compare absoluto. Zero jitter. Reusar. |
| `src/mcpwm_ignition_hp.c` | `scheduler/ignition_driver` | 241 linhas | Idem para ignição. Reusar. |
| `src/control/table_16x16.c/h` | `tables/table_3d` | ~150 linhas | 16x16 bilinear com checksum. Ampliar para outras dimensões se necessário. |
| `src/control/fuel_calc.c/h` | `control/fuel_calc` | ~200 linhas | VE lookup + cache de interpolação + enrichments. Reusar. |
| `src/control/lambda_pid.c/h` | `control/closed_loop_fuel` | ~100 linhas | PID lambda + STFT/LTFT. Reusar. |
| `src/control/map_storage.c/h` | `config/config_manager` | ~120 linhas | Persistência de mapas em NVS. Reusar. |
| `src/config_manager.c/h` | `config/engine_config` | 138 linhas | Parâmetros de configuração em NVS. Reusar. |
| `src/sensor_processing.c/h` | `sensors/` (todos) | 350 linhas | MAP, TPS, CLT, IAT, VBAT. Reusar, adicionar flex/knock/VSS. |
| `src/twai_lambda.c/h` | `comms/can_wideband` | 303 linhas | Leitura lambda wideband via TWAI/CAN. Reusar. |
| `src/espnow_link.c/h` | `comms/espnow_link` | 811 linhas | Link ESP-NOW dashboard + tunagem. Reusar. |
| `src/tuning_protocol.c/h` | `comms/tunerstudio` | 546 linhas | Protocolo TunerStudio-like. Reusar. |
| `src/data_logger.c/h` | `logging/sd_logger` | 687 linhas | SD card logger. Reusar. |
| `src/safety_monitor.c/h` | `diagnostics/fault_manager` | 260 linhas | Limp mode + watchdog + sensor validation. Reusar e expandir. |
| `src/cli_interface.c/h` | `tools/cli` | 1048 linhas | Interface CLI serial. Reusar. |
| `src/logger.c/h` | `utils/logger` | 139 linhas | Sistema de logging interno. Reusar. |
| `src/test_framework.c/h` | `tests/` | 498 linhas | Framework de testes. Adaptar para Google Test no host. |
| `src/math_utils.h` | `utils/math_utils` | - | Utilitários matemáticos. Reusar. |
| `firmware/s3/sdkconfig` | `sdkconfig.defaults` | - | Configuração ESP-IDF testada. Adaptar. |
| `firmware/partitions.csv` | `partitions.csv` | - | Layout de partições com NVS. Reusar. |

**Total reusável: ~6.400 linhas de código já funcional e testado.**

---

### ⚠️ ADAPTAR (requer modificação significativa)

| Arquivo fonte | Problema | Adaptação necessária |
|---|---|---|
| `src/sync.c` | Hardcodado para ESP32-S3 (ETM via `SOC_GPTIMER_SUPPORT_ETM`) | Já tem fallback GPIO ISR — validar e testar fallback no ESP32 original |
| `include/s3_control_config.h` | Nome "S3", mistura config de hardware e engine | Separar em `hal/hal_pins.h` (hardware) e `config/engine_config.h` (motor) |
| `src/control/engine_control.c` | Monolítico — mistura planner + executor + monitor em tasks | Refatorar seguindo separação Core0/Core1 definida no OpenEMS |
| `src/control/ignition_timing.c/h` | Não encontrado (stub?) | Verificar — pode precisar ser implementado |
| `src/control/fuel_injection.c/h` | Wrapper sobre MCPWM — verificar se usa scheduling por CAD | Adaptar para scheduling baseado em ângulo via event_scheduler |

---

### ❌ IMPLEMENTAR DO ZERO (não existe no ESP32_S3_EMS)

Estes módulos existem na especificação OpenEMS mas não no repositório fonte:

| Módulo OpenEMS | Prioridade | Inspiração rusefi |
|---|---|---|
| `scheduler/event_scheduler` | **CRÍTICO** | `trigger/scheduler.cpp` — fila de eventos por ângulo de virabrequim |
| `control/vvt_control` | Alta | `controllers/actuators/vvt.cpp` — PID dual cam |
| `control/idle_control` | Alta | `controllers/idle/idle_controller.cpp` — PID IAC |
| `control/boost_control` | Média | `controllers/actuators/boost.cpp` — PID wastegate |
| `control/flex_compensation` | Média | Correção E% em VE + avanço |
| `control/knock_control` | Alta | `sensors/knock/knock_controller.cpp` — retardo por cilindro |
| `sensors/flex_sensor` | Média | Frequência → E% (PWM input) |
| `sensors/knock_sensor` | Alta | Janela por CAD + integração |
| `sensors/vss_sensor` | Baixa | Pulse counter → velocidade |
| `control/launch_control` | Baixa | RPM limiter + ignition cut |
| `control/traction_control` | Baixa | Delta VSS → cylinder cut |
| `control/anti_lag` | Baixa | Retardo + enriquecimento overrun |
| `comms/obd2` | Média | PIDs padrão via CAN (sobre twai_lambda) |
| `diagnostics/dtc_codes` | Média | Enum + NVS store + freeze frame |
| `hal/hal_*.h` | **CRÍTICO** | Wrappers inline sobre periféricos existentes |

---

## 2. Contribuições arquiteturais do rusefi

O rusefi (GPL v3, STM32) não pode ser copiado diretamente mas seus conceitos são essenciais:

### 2.1 Event Scheduler por ângulo (o mais importante)

rusefi usa uma fila de eventos ordenada por ângulo de virabrequim (não por tempo).
A cada dente da roda fônica, converte os próximos eventos de grau → tempo e agenda via timer.
Isso elimina o problema de acumulação de erro de tempo quando o RPM varia.

```
Conceito rusefi (adaptar para ESP32):
- Engine state: { tooth_index, tooth_period_us, rpm, revolution_index }
- Event queue: [ {cyl, event_type, angle_deg, callback}, ... ] ordenado por angle
- On each tooth: para cada evento cujo angle está no próximo dente:
    fire_time_us = current_tooth_time + (angle_offset / tooth_angle) * tooth_period
    schedule_hw_timer(fire_time_us, callback)
```

No OpenEMS, isso é implementado em `scheduler/event_scheduler.c` usando o MCPWM
em modo de compare absoluto (exatamente o que `mcpwm_injection_hp.c` já faz para injeção).

### 2.2 Sync quality counter

rusefi usa um contador de qualidade de sincronismo: incrementa a cada gap detectado
corretamente, decrementa em caso de erro. Só considera sincronizado após N acertos consecutivos.

O `sync.c` do ESP32_S3_EMS não implementa isso explicitamente — adicionar.

### 2.3 Per-cylinder trim

rusefi mantém um array de correções por cilindro para knock retard e traction cut.
O `safety_monitor.c` tem knock mas não per-cylinder. Adicionar ao OpenEMS.

### 2.4 TDC calibration offset

rusefi tem `globalTriggerAngleOffset` — offset configurável entre o gap da roda fônica e o TDC
real do cilindro 1. Essencial para calibração. Adicionar como parâmetro em `engine_config.h`.

---

## 3. Decisão técnica crítica: PCNT vs RMT

O ESP32_S3_EMS usa **PCNT + ETM + GPTimer** (não RMT como planejado inicialmente no OpenEMS).

**Por que PCNT é melhor aqui:**
- PCNT conta pulsos em hardware — sem CPU para cada dente
- ETM captura timestamp exato no GPTimer quando PCNT atinge threshold — zero latência de ISR
- Funciona mesmo com burst de pulsos em alta RPM
- RMT seria melhor para análise de forma de onda (VR passivo), mas o circuito usa condicionador externo

**Decisão: manter PCNT + ETM + GPTimer do ESP32_S3_EMS.**

Para suporte a sensor VR passivo (sem condicionador), adicionar opção de threshold adaptativo
no software baseado no pico anterior — conceito do rusefi `VRThresholdCallback`.

---

## 4. Estrutura adaptada do repositório OpenEMS

Com base na análise, a estrutura do repositório muda de "do zero" para "fork organizado":

```
openems/
├── firmware/
│   ├── hal/                          → NOVO — wrappers inline (abstraem sync.c, mcpwm_*)
│   │   ├── hal_gpio.h                → inline, acesso direto GPIO
│   │   ├── hal_mcpwm.h               → inline, wrap mcpwm_injection_hp + ignition_hp
│   │   ├── hal_pcnt_etm.h            → inline, wrap sync PCNT+ETM
│   │   └── hal_can.h                 → wrap twai_lambda
│   │
│   ├── decoder/                      → ADAPTADO de sync.c
│   │   ├── trigger_60_2.c/h          → sync.c refatorado + sync quality counter
│   │   └── cam_decoder.c/h           → parte do sync.c (CMP) extraída
│   │
│   ├── scheduler/                    → PARCIAL REÚSO + NOVO
│   │   ├── event_scheduler.c/h       → NOVO — conceito rusefi, angle→time scheduling
│   │   ├── hp_timing.c/h             → high_precision_timing.c REUTILIZADO
│   │   ├── hp_state.c/h              → hp_state.c REUTILIZADO
│   │   ├── injector_driver.c/h       → mcpwm_injection_hp.c REUTILIZADO
│   │   └── ignition_driver.c/h       → mcpwm_ignition_hp.c REUTILIZADO
│   │
│   ├── sensors/                      → ADAPTADO de sensor_processing.c
│   │   ├── sensor_processing.c/h     → REUTILIZADO (MAP, TPS, CLT, IAT, VBAT)
│   │   ├── flex_sensor.c/h           → NOVO — frequência → E%
│   │   ├── knock_sensor.c/h          → NOVO — janela CAD + integração
│   │   └── vss_sensor.c/h            → NOVO — velocidade
│   │
│   ├── control/                      → PARCIAL REÚSO + NOVO
│   │   ├── fuel_calc.c/h             → REUTILIZADO
│   │   ├── closed_loop_fuel.c/h      → lambda_pid.c REUTILIZADO
│   │   ├── knock_control.c/h         → NOVO (+ safety_monitor adaptado)
│   │   ├── vvt_control.c/h           → NOVO
│   │   ├── idle_control.c/h          → NOVO
│   │   ├── boost_control.c/h         → NOVO
│   │   ├── flex_compensation.c/h     → NOVO
│   │   ├── launch_control.c/h        → NOVO
│   │   └── traction_control.c/h      → NOVO
│   │
│   ├── tables/                       → REUTILIZADO
│   │   ├── table_16x16.c/h           → REUTILIZADO
│   │   └── table_2d.c/h              → NOVO (curvas CLT/IAT NTC)
│   │
│   ├── comms/                        → REUTILIZADO + NOVO
│   │   ├── can_wideband.c/h          → twai_lambda.c REUTILIZADO
│   │   ├── espnow_link.c/h           → REUTILIZADO
│   │   ├── tunerstudio.c/h           → tuning_protocol.c REUTILIZADO
│   │   └── obd2.c/h                  → NOVO (sobre TWAI existente)
│   │
│   ├── diagnostics/                  → ADAPTADO
│   │   ├── fault_manager.c/h         → safety_monitor.c ADAPTADO + DTC store
│   │   ├── limp_mode.c/h             → extraído de safety_monitor
│   │   └── dtc_codes.h               → NOVO — enum de códigos
│   │
│   ├── logging/                      → REUTILIZADO
│   │   └── sd_logger.c/h             → data_logger.c REUTILIZADO
│   │
│   ├── config/                       → REUTILIZADO
│   │   ├── config_manager.c/h        → REUTILIZADO
│   │   └── engine_config.h           → s3_control_config.h REFATORADO
│   │
│   └── utils/                        → REUTILIZADO
│       ├── math_utils.h              → REUTILIZADO
│       ├── logger.c/h                → REUTILIZADO
│       ├── atomic_buffer.h           → NOVO — Core0↔Core1
│       └── ring_buffer.h             → NOVO
│
└── tests/                            → test_framework.c ADAPTADO para Google Test host
```

---

## 5. Plano de implementação por sprint

### Sprint 1 — Base funcional (fork + reorganização)
1. Fork do ESP32_S3_EMS, renomear estrutura para OpenEMS
2. Criar `hal/` com wrappers inline sobre periféricos existentes
3. Refatorar `s3_control_config.h` → separar hardware e engine config
4. Validar compilação ESP-IDF sem regressão
5. Implementar `scheduler/event_scheduler.c` — peça central missing

### Sprint 2 — Completar sensores e controle
6. `sensors/flex_sensor.c` — frequência → E%
7. `sensors/knock_sensor.c` — janela CAD + integração
8. `control/knock_control.c` — retardo por cilindro
9. `control/vvt_control.c` — PID dual cam

### Sprint 3 — Features avançadas
10. `control/idle_control.c` — PID IAC
11. `control/boost_control.c` — PID wastegate
12. `comms/obd2.c` — PIDs OBD2
13. `diagnostics/dtc_codes.h` + DTC store NVS

### Sprint 4 — Testes e validação
14. Adaptar `test_framework.c` para Google Test no host
15. Testes de simulação para decoder e scheduler
16. Documentação e .ini TunerStudio

---

## 6. Licença

`pbuchabqui/ESP32_S3_EMS` — verificar licença antes de fork público.
Se não houver licença declarada, contatar o autor para clarificação ou request de GPL v3.
rusefi — GPL v3 (apenas conceitos arquiteturais, sem código copiado direto).
OpenEMS — GPL v3.
