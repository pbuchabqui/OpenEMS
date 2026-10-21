# OpenEMS

ECU para motores a combustao interna (STM32H562, C++17 embarcado). Prioridade: baixa latencia, baixo jitter e previsibilidade temporal. Escopo: so o firmware Hall (roda 60-2 + sensor de fase, captura em TIM5); o tree encoder MT6835 foi abandonado.

## Seguranca

- Nunca assumir atuadores energizados. Ensaio real exige confirmar a revisao fisica do chip (Rev X, `REV_ID` em `DBGMCU_IDCODE`); bancada e sem bobinas/injetores.
- Saidas INJ/IGN sao active-high; estado seguro = LOW.

## Antes de dar algo como pronto

- `make ci-local` (secrets, host-test, precision-test, ini-check, dash-check, firmware, lint-includes). `make host-test WERROR=1` deve fechar com 0 FAIL.
- Mudou pino ou HAL: compilar as duas placas, `make firmware-rgt6 WERROR=1` e `make firmware-vgt6 WERROR=1`.
- Novo teste: arquivo `test/test_*.cpp` + registro em `run_all.cpp` (ordem fixa das suites) e `suite_registry.h`. Nao recriar `mvp_bench_tests.cpp`.

## Restricoes de codigo

- Sem STL no caminho critico, sem alocacao dinamica em runtime, sem excecoes, sem aliases de compatibilidade (so perifericos e nomes STM32H562 reais).
- Camadas descendentes: APP > ENGINE > DRV > HAL > HW. `ENGINE`/`DRV` nao incluem `app/` (sinais de veiculo entram por `engine/vehicle_inputs.h`). `hal/regs.h` so em engine via `tools/include_allowlist.txt`.
- Hot-path de `ecu_sched` (`ecu_sched_internal.h`): nao mover `evt_insert`/arm/dispatch para outro `.cpp`. Escrita de GPIO so se for header-inline (`out_pin_write`).
- Base de tempo do motor e TIM5. Nunca usar USB SOF como referencia temporal.

## Errata ES0565 que mordem

- Flash: a primeira erase/program apos power-on congela fetch ~120 us; nao gravar Flash com motor girando nem em Bank2 durante janela de injecao/ignicao/CKP. Tratar NVM como baixa taxa, commit explicito.
- ADC: so amostragem regular disparada por TIM6 (sem fila injetada, dual interleaved ou watchdog analogico misturado).
- FDCAN: sem edge filtering (`EFBI`); fila/buffer unico.
- USB OUT: atraso minimo de 800 ns (Full Speed) antes de ler o buffer.
- IWDG nao acorda de Stop mode; nao depender de Stop para recuperacao.

## Docs e segredos

- O `README.md` e so visao geral. Decisao tecnica duravel vai no `docs/*.md` do subsistema (fuel, idle, ignition, protection, output_contract); nao criar Markdown paralelo fora de `docs/`.
- `wifi_credentials.h` nunca e commitado (copiar de `wifi_credentials.example.h`). `graphify-out/` nao e versionado.
