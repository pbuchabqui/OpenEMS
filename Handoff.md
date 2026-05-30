# Handoff — OpenEMS MVP Bancada (STM32H562)

**Data:** 26 de maio de 2026  
**Branch:** `feat/oem-complete`  
**Status:** Pronto para MVP de bancada segura (sem atuadores energizados no primeiro boot)

> **Documentação canônica:** `README.md` é a fonte única da verdade do projeto.  
> Este handoff resume o estado atual da branch para transição de contexto; detalhes técnicos duráveis devem ir para o `README.md` ou comentários no código.

---

## 1. Resumo executivo

OpenEMS é uma ECU embarcada para motores a combustão, direcionada ao **STM32H562RGT6**. A branch `feat/oem-complete` consolida o pipeline de sincronismo, agendamento angular, cálculo de combustível/ignição, auxiliares e calibração via UI proprietária.

**Escopo real desta branch:** MVP de bancada com CKP/CMP, scheduler por output compare, telemetria UART e persistência NVM — **não** inclui borboleta eletrônica (ETB), gestão centralizada de torque, ou auto-aprendizado X-τ fechado em lambda.

### Conquistas principais

- **Sincronismo:** decode CKP/CMP via TIM5 input capture, FSM 60-2, quick crank e runtime seed para re-aquisição rápida.
- **Agendamento:** injeção (TIM2) e ignição (TIM8) por hardware OC, sem bit-bang no caminho crítico.
- **Combustível/ignição:** VE, lambda alvo, STFT/LTFT, AE, compensação X-τ (tabelas estáticas), correção de marcha lenta na ignição.
- **Auxiliares:** IAC (TIM3), wastegate com failsafe de overboost, VVT estimado por fase CKP/CMP, ventoinha e bomba.
- **Calibração:** protocolo UI via UART (`PA9`/`PA10`), bridge Python e persistência em Flash Bank2.
- **Qualidade:** `make firmware` e `make host-test` passam; regressão host cobre CKP, scheduler, fuel/ign, UI e auxiliares.

---

## 2. Arquitetura implementada

### Camadas

```text
APP     ui_protocol, can_stack
ENGINE  fuel_calc, ign_calc, ecu_sched, quick_crank, transient_fuel, auxiliaries, calibration
DRV     ckp, sensors
HAL     adc, flash, uart, can, stm32h562/{timer,gpio,system}
```

### Módulos principais

| Módulo | Arquivo | Função |
|--------|---------|--------|
| Entry point | `src/main_stm32.cpp` | Loops 2/10/20/50/100/500 ms; orquestração de motor e NVM |
| CKP/CMP | `src/drv/ckp.cpp` | Input capture TIM5, sync 60-2, seed de runtime |
| Scheduler | `src/engine/ecu_sched.cpp` | OC TIM2/TIM8, tabela angular 4 cilindros |
| Combustível | `src/engine/fuel_calc.cpp` | VE, trims, AE, cadeia de PW |
| Ignição | `src/engine/ign_calc.cpp` | Avanço, dwell, knock retard, idle spark |
| Transiente | `src/engine/transient_fuel.cpp` | Modelo X-τ (tabelas em `calibration.cpp`) |
| Quick crank | `src/engine/quick_crank.cpp` | Enriquecimento/prime em partida |
| Auxiliares | `src/engine/auxiliaries.cpp` | IAC, WG, VVT, fan, pump; init de TIM3/TIM4 @ 15 Hz |
| Calibração | `src/engine/calibration.cpp`, `src/app/ui_protocol.cpp` | Tabelas + protocolo de leitura/escrita |
| Sensores | `src/drv/sensors.cpp`, `src/hal/adc.cpp` | ADC1/ADC2 + TIM6, validação e fault bits |
| NVM | `src/hal/flash.cpp` | LTFT, knock, páginas de calibração, runtime seed |

### Pinout ativo (MVP)

| Função | Periférico | Pino |
|--------|------------|------|
| CKP | TIM5 CH1 | PA0 |
| CMP | TIM5 CH2 | PA1 |
| INJ1–4 | TIM2 CH1–4 | PA15, PB3, PB10, PB11 |
| IGN1–4 | TIM8 CH1–4 | PC6–PC9 |
| IACV / WG | TIM3 CH1/CH2 | PA6 / PA7 |
| VVT esc/adm | TIM4 CH1/CH2 | PB6 / PB7 |
| UI UART | USART1 | PA9 TX, PA10 RX |
| Fan / Pump | GPIO | PB12 / PB13 |

**Fora do escopo atual:** ETB, dual TPS, driver ponte H, modos Eco/Sport/Rain de pedal.

---

## 3. Funcionalidades chave

### Combustível e ignição (loop 2 ms)

- Lookup 2D preparado (`table3d`) para VE, lambda e avanço.
- STFT por lambda (quando válido) + LTFT por célula persistida.
- AE por derivada de TPS; X-τ habilitado acima de 700 rpm (`rpm_x10 >= 7000`).
- Quick crank desabilita AE/X-τ/idle spark durante janela de partida.
- LIMP básico: falha MAP ou CLT → corte de rotação acima de 3000 rpm.

### Auxiliares

- **IAC:** PID de RPM com warmup por CLT; duty limitado por slew rate.
- **Wastegate:** alvo 2D RPM×TPS; failsafe após 500 ms de overboost.
- **VVT:** posição estimada por `tooth_index` + `phase_A`; só atua com FULL_SYNC estável.
- **Fan/Pump:** histerese por CLT; bomba com prime de 2 s e delay de 3 s após RPM zero.

### Calibração e NVM

- UI proprietária (`scripts/bridge.py` + `scripts/ui/index.html`).
- Flush adaptativo LTFT/knock a cada 500 ms quando dirty.
- Calibração página 0 com intervalo mínimo de 5 min entre gravações.
- Runtime seed salvo 100 ms após motor parar (gap sync válido).

---

## 4. Correções recentes (branch)

1. ADC/DMA one-shot com re-arm correto; SMPR2 para IN10; canal GPDMA ADC2.
2. Race conditions e `volatile` em estado compartilhado (scheduler, sensores, auxiliares).
3. Retornos de erro NVM rastreados (`g_flash_write_faults`).
4. Restauração de `ecu_sched.cpp` e ordem de clear de status register.
5. GPIO auxiliar e persistência de calibração endurecidos no último commit.

---

## 5. Checklist de validação (pré-bancada)

Antes de energizar bobinas/injetores:

- [ ] `make firmware` e `make host-test` passam localmente.
- [ ] Confirmar revisão de silício (Rev X) e evidência em bancada.
- [ ] Validar pinout físico contra tabela acima (conflitos PC8/PC9, JTAG em PA15/PB3).
- [ ] Primeiro boot **sem** cargas em INJ/IGN; verificar ausência de reset loop / hard fault.
- [ ] TIM5 com CKP/CMP sintético 200–8500 rpm; transições WAIT_GAP → HALF_SYNC → FULL_SYNC.
- [ ] TIM2/TIM8 em osciloscópio (cranking, half-sync, full-sync, loss-of-sync).
- [ ] UART 115200 8N1 em PA9/PA10 com bridge UI; assinatura `OpenEMS_v1.1`.
- [ ] Monitorar `loop2_max`, `late_event_count`, `cycle_schedule_drop_count`, `g_flash_write_faults`.
- [ ] Simular falha MAP/CLT e confirmar LIMP + rev cut.

---

## 6. Próximos passos sugeridos

1. **Bancada segura:** validar `.bin` via ST-Link; medir jitter real do scheduler.
2. **Hardening NVM:** medir latência do loop 2 ms durante flush de Flash (errata ES0565).
3. **Cobertura de testes:** expandir HIL para sensores e perda de sync.
4. **Futuro (não implementado):** ETB + torque manager, MAP estimator, auto-calib X-τ por lambda, TCS, cruise.

---

## 7. Build e testes

```bash
make firmware    # .elf / .hex / .bin em /tmp/openems-build
make host-test   # regressão CKP, scheduler, fuel/ign, UI, auxiliares
make clean
```

**Repositório:** github.com/pbuchabqui/OpenEMS  
**Fim do handoff.**
