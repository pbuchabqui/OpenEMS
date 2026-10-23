# Sistema de proteção

Duas peças: o `limp_gating` decide os cortes de injeção, ignição e ETB; o `DiagnosticManager` guarda as falhas e alimenta o corte por CRITICAL.

## limp_gating (`src/engine/limp_gating.{h,cpp}`)

`limp_gating_update()` é o único caminho que calcula `allow_injection`/`allow_ignition`, escreve `g_fuel_cut_reasons` e `g_spark_cut_reasons` e grava as máscaras de inibição de INJ/IGN. Nada mais escreve esses campos (o DFCO entra por `limp_gating_or_fuel_reason`).

- Cada corte tem um bit de motivo; o `ui_protocol.cpp` os publica ao dashboard e ao host.
- Os cortes por valor (óleo, pressão de trilho, sobretemperatura, lambda, ETB, limp a 3000 rpm, boost, duty do injetor) podem ser desligados em RAM por `limp_gating_set_protect_disable` (bits `kProtectDis*`, perdidos no reset). Rev limit, flood, fatal, stall, watchdogs e perda de sincronismo nunca entram nessa máscara.
- Os limites de bancada (`boost_cut_map_bar_x100`, `oil_min_after_start_bar_x1000`, `lambda_protect_*`) valem 0 = desligado, para a bancada não disparar corte.
- O problema de ETB trava num latch (`limp_gating_report_etb_problem`) até `limp_gating_clear_etb_problem`. Se a causa persiste, o latch trava de novo.

## DiagnosticManager (`src/engine/diagnostic_manager.{h,cpp}`)

Tabela de 16 eventos em RAM, com código, severidade, contador de ocorrências, `freeze_frame[4]` e estado de recuperação.

- `report_fault` cria o evento ou incrementa o existente. A severidade só sobe por reporte, nunca desce. Com a tabela cheia, descarta o evento não crítico mais antigo; se todos são críticos, o novo não entra.
- `clear_fault` remove o evento. Os chamadores limpam quando a condição some (sobretemperatura com histerese em `engine_calc.cpp`; faixa de MAP/TPS/MAF e plausibilidade MAP×TPS em `sensors.cpp`).
- `diagnostic_tick(100)` roda no bloco de 100 ms do `main_stm32.cpp` e carimba o tempo dos eventos.
- Único efeito sobre o motor: `is_system_ready()` é falso se houver um evento CRITICAL, e `engine_calc.cpp` o passa ao `limp_gating` como `diag_critical` (bit `kProtectDisDiag` desliga esse corte). WARNING e ERROR só aparecem na tabela.
- `ADC_RECOVERY_FAILED` em CRITICAL não é limpo por ninguém: o corte fica até o reset. Decidido em 2026-10-22 que é intencional: o ADC esgotou as tentativas de recuperação e sem leitura confiável de MAP/TPS o motor não deve voltar sozinho; só reiniciar a ECU reabilita.

Sem debounce na tabela: cada chamador já filtra a falha antes de reportar (timers de falha de sensor, histerese de sobretemperatura), e um debounce no CRITICAL atrasaria o corte. Não há persistência em NVM: nada lê os códigos depois do reset, e gravar Flash exige motor parado (errata ES0565). As duas coisas estão no Backlog de `docs/ROADMAP.md`.

## Como adicionar uma proteção

1. Escolha o motivo: um bit novo em `g_fuel_cut_reasons` ou `g_spark_cut_reasons`, e um bit `kProtectDis*` se o corte puder ser desligado pelo PROTECT.
2. Calcule a condição em `limp_gating_update` (limiar e tempo como variável de bancada com 0 = desligado, no mesmo padrão dos `lambda_protect_*`).
3. Exponha o motivo no protocolo do host e, se for novo canal, no `openems.ini` (`make ini-check`).
4. Teste em `test/` (cortes: `test_sched.cpp` e `test_etb.cpp`; faltas: `test_engine_misc.cpp`).
