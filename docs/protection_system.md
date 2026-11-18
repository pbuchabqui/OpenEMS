# Sistema de proteção

> **Status:** vigente · validado 2026-11-18 (doc-check + conferência pontual contra o código)

Duas peças: o `limp_gating` decide os cortes de injeção, ignição e ETB; o `DiagnosticManager` guarda as falhas e alimenta o corte por CRITICAL.

## limp_gating (`src/engine/limp_gating.{h,cpp}`)

`limp_gating_update()` é o único caminho que calcula `allow_injection`/`allow_ignition`, escreve `g_fuel_cut_reasons` e `g_spark_cut_reasons` e grava as máscaras de inibição de INJ/IGN. Nada mais escreve esses campos (o DFCO entra por `limp_gating_or_fuel_reason`).

- Cada corte tem um bit de motivo; o `ui_protocol.cpp` os publica ao dashboard e ao host.
- Os cortes por valor (óleo, pressão de trilho, sobretemperatura, lambda, ETB, limp a 3000 rpm, boost, duty do injetor) podem ser desligados em RAM por `limp_gating_set_protect_disable` (bits `kProtectDis*`, perdidos no reset). Rev limit, flood, fatal, stall, watchdogs e perda de sincronismo nunca entram nessa máscara.
- Os limites de bancada (`boost_cut_map_bar_x100`, `oil_min_after_start_bar_x1000`, `lambda_protect_*`) valem 0 = desligado, para a bancada não disparar corte.
- Corte fatal (`fatal_cut`): hoje só o clock. Se o SYSCLK não está no PLL1 (`system_clock_ok()` falso: HSE/PLL não travou, core no HSI), o TIM5 conta ~7,8× mais devagar e dwell/PW sairiam 7,8× mais longos, com o watchdog de dwell contando nos mesmos ticks. O boot não habilita a captura CKP/CMP, reporta `CLOCK_FAULT` em CRITICAL, o `limp_gating` corta injeção e ignição sem passar pelo PROTECT, e o teste de saídas é recusado. Só o reset com o clock de volta reabilita.
- O problema de ETB trava num latch (`limp_gating_report_etb_problem`) até `limp_gating_clear_etb_problem`. Se a causa persiste, o latch trava de novo.

## DiagnosticManager (`src/engine/diagnostic_manager.{h,cpp}`)

Tabela de 16 eventos em RAM, com código, severidade, contador de ocorrências, `freeze_frame[4]` e estado de recuperação.

- `report_fault` cria o evento ou incrementa o existente. A severidade só sobe por reporte, nunca desce. Com a tabela cheia, descarta o evento não crítico mais antigo; se todos são críticos, o novo não entra.
- `clear_fault` remove o evento. Os chamadores limpam quando a condição some (sobretemperatura com histerese em `engine_calc.cpp`; faixa de MAP/TPS e plausibilidade MAP×TPS em `sensors_diag.cpp`).
- `diagnostic_tick(100)` roda no bloco de 100 ms do `main_stm32.cpp` e carimba o tempo dos eventos.
- Único efeito sobre o motor: `is_system_ready()` é falso se houver um evento CRITICAL, e `engine_calc.cpp` o passa ao `limp_gating` como `diag_critical` (bit `kProtectDisDiag` desliga esse corte). WARNING e ERROR só aparecem na tabela.
- `clear_fault_if_not_critical` remove a falha só se ela não estiver em CRITICAL; é o que limpa estados transitórios sem soltar um CRITICAL.
- `ADC_RECOVERY_FAILED` em CRITICAL não é limpo por ninguém: o corte fica até o reset. O estado transitório "em recuperação" é reportado como WARNING e é limpo quando a recuperação dá certo. Decidido em 2026-10-22 que é intencional: o ADC esgotou as tentativas de recuperação e sem leitura confiável de MAP/TPS o motor não deve voltar sozinho; só reiniciar a ECU reabilita.

## Diagnóstico de sensores (`src/engine/sensors_diag.{h,cpp}`)

`sensors_diag_poll()` roda no bloco de 50 ms do `main_stm32.cpp`, depois de `sensors_tick_50ms()`, e traduz o snapshot **commitado** dos sensores em DTCs: `MAP_SENSOR_RANGE` e `TPS_SENSOR_RANGE` (a partir dos `fault_bits`), `MAP_TPS_CORRELATION` (plausibilidade) e `ADC_RECOVERY_FAILED`. Isso não roda na ISR do dente: cada report/clear varre a tabela de eventos sob seção crítica, e dentro da ISR isso atrasava o agendamento de INJ/IGN.

- O corte e o limp continuam imediatos: são guiados pelos `fault_bits` e pelo fallback que a ISR ainda aplica. O DTC chega à tabela com até um período do poll (50 ms) de atraso; para o CRITICAL de ADC, que alimenta `diag_critical`, o corte por DTC também pode atrasar esse tanto.
- Com o ADC em recuperação ou falhado, o poll não avalia faixa nem plausibilidade (os dados são fallback, não medição).
- A plausibilidade (`check_sensor_plausibility`) só aplica as regras que assumem motor funcionando (vácuo em marcha lenta; MAP não atmosférico com a borboleta fechada) com RPM ≥ 400. Com a chave ligada e o motor parado, ou em cranking, o MAP é atmosférico e isso é normal.

Sem debounce na tabela: cada chamador já filtra a falha antes de reportar (timers de falha de sensor, histerese de sobretemperatura), e um debounce no CRITICAL atrasaria o corte. Não há persistência em NVM: nada lê os códigos depois do reset, e gravar Flash exige motor parado (errata ES0565). As duas coisas estão no Backlog de `docs/ROADMAP.md`.

## Como adicionar uma proteção

1. Escolha o motivo: um bit novo em `g_fuel_cut_reasons` ou `g_spark_cut_reasons`, e um bit `kProtectDis*` se o corte puder ser desligado pelo PROTECT.
2. Calcule a condição em `limp_gating_update` (limiar e tempo como variável de bancada com 0 = desligado, no mesmo padrão dos `lambda_protect_*`).
3. Exponha o motivo no protocolo do host e, se for novo canal, no `openems.ini` (`make ini-check`).
4. Teste em `test/` (cortes: `test_sched.cpp` e `test_etb.cpp`; faltas: `test_engine_misc.cpp`).
