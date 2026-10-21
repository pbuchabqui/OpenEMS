# Arquitetura

## Regras De Arquitetura

O fluxo de dependencias e descendente:

```text
APP     protocolos, calibracao, diagnostico
ENGINE  calculos de motor, agendamento angular, tabelas
DRV     decodificacao CKP/CMP, sensores, servicos independentes de MCU
HAL     registradores e perifericos STM32H562
HW      timers, ADC, GPIO, USB, CAN
```

Regras praticas:

- `APP` nao deve entrar no caminho critico de captura/agendamento.
- `ENGINE` decide tempo, angulo, pulso, dwell e estrategia.
- `DRV` transforma sinais fisicos em eventos de motor confiaveis.
- `HAL` contem apenas detalhe de periferico.
- O codigo ativo deve usar somente perifericos e nomes STM32H562 reais, sem aliases de compatibilidade.
- **`ENGINE` e `DRV` nao incluem headers `app/`.** Sinais de veiculo (marcha, velocidade, roda) entram via `engine/vehicle_inputs.h` (implementacao em `app/vehicle_inputs_bridge.cpp` → `can_rx_map`).
- **INJ/IGN GPIO:** mapas BSRR e write hot-path em `hal/out_pins.h` (`out_pin_write` inline). Init/safe em `out_pins_hw_init()`. `ecu_sched` nao reimplementa tabelas de pinos.
- Decisao tecnica duravel vai neste conjunto de `docs/*.md`, no arquivo do subsistema; o `README.md` e so visao geral (ver `CLAUDE.md`).

## Escopo: so o firmware Hall

Este repo mantem um unico firmware: roda 60-2 + sensor de fase Hall, captura
em TIM5. A antiga regra "dual-firmware" (espelhar toda mudanca num tree de
encoder MT6835) foi abandonada na revisao de 2026-10; o tree encoder, se
retomado, porta as mudancas por conta propria.


### Camadas — contratos pos-higiene

**Vehicle inputs (ENGINE ← APP):**

```text
can_stack → can_rx_map_process (APP)
torque_manager / auxiliaries → vehicle_gear|speed|wheel (engine/vehicle_inputs.h)
                            → vehicle_inputs_bridge.cpp (APP) → can_rx_*
```

Page0 serialize/apply do mapa CAN permanece em APP (`can_rx_map_*_page0`).

**INJ/IGN outputs (HAL):**

```text
ecu_sched gpio_set_pin → ems::hal::out_pin_write  (inline em out_pins.h)
ecu_sched_outputs_safe_early → out_pins_hw_init()  (clocks + MODER + BSRR LOW)
```

Ordem de canais BSRR = `ECU_CH_*`:
`[INJ3, INJ4, INJ1, INJ2, IGN4, IGN3, IGN2, IGN1]`. Actuadores active-high;
safe = LOW. Testes de polaridade RGT6: `test_out_pins_bsrr_rgt6`.

Hot-path rule (`ecu_sched_internal.h`): nao mover `evt_insert` / arm / dispatch
para outro `.cpp`; write GPIO so e permitido se **header-inline** (como
`out_pin_write`).

**NVM boot:** loaders de tabelas/correccoes em `app/nvm_boot.cpp`
(`nvm_boot_load_tables(cal_layout_ok)`), chamados de `main_stm32` — mesma
ordem e gate de layout de antes.

**UI protocol:** `ui_protocol.cpp` (parse + API) + `ui_protocol_state.cpp` +
`ui_protocol_pages.cpp` + `ui_protocol_envelope.cpp` + `ui_protocol_internal.h`.

**ETB PWM:** API preferida `etb_pwm_init` / `etb_pwm_set_duty_x10`. Aliases
`tim15_etb_*` deprecated em `hal/timer.h`.

