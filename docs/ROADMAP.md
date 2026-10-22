# Roadmap e progresso OpenEMS

Atualizado em 2026-10-21, `main` em `6aeb3ea`. Reúne o antigo `progresso.md` (revisão e correção de `src/`, branch `fix/output-contract-cuts`, já mesclada em `main`) e o roadmap de produto. Nada aqui foi validado em hardware. `main` está igual a `origin/main`. `tools/ts/OpenEMS_model.msq` foi versionado em 2026-10-21.

## Verificação em `main` (2026-10-21)

- `make host-test WERROR=1`: 1830 PASS / 0 FAIL.
- `make precision-test`: 162 PASS / 0 FAIL / 0 XFAIL / 0 XPASS.
- `make firmware-rgt6 WERROR=1` e `make firmware-vgt6 WERROR=1`: os dois terminaram com `Firmware build successful`.
- `make ini-check`: 188 constants, 45 channels, 0 findings.
- `make dash-check`: não rodado nesta data. Em 2026-10-20: 11 testes, OK, com `ResourceWarning` de arquivo não fechado em `tools/openems_dash/tests/test_tune.py`.
- A saída longa de `make host-test` pode vir truncada no terminal. O resultado completo está no log do binário, em `/tmp/openems-build/host/mvp_bench_tests`.

## Roadmap

Escopo: firmware e bancada (WeAct H562 VGT6 + ESP32). A placa v2 (MC33810, L9960T, CJ125, TPS65381A) e os drivers dos CIs novos estão fora deste roadmap. Ordem vinda de `docs/hw/README.md` §6: bancada, depois ETB, depois motor. Cada fase só começa quando a anterior cumpre o seu critério de pronto.

### Fase 0 — Consistência da documentação

Sem hardware. Reflete a divisão do README e a remoção dos projetos KiCad em 2026-10-21 (`a1e26fc`, recuperáveis no git).

- Feito em 2026-10-21: `docs/bench_test_manual.md` (esquemático e errata), `hardware/README.md` (estrutura sem os projetos KiCad removidos) e as referências a "README §" em `docs/hw/pinout.md`, `weact_h562_coreboard.md` e `interface_board_v1.md` agora apontam para `docs/estado_e_hardening.md`, `docs/errata_stm32h562.md` e `docs/hw/README.md`.
- O suporte a TLE8888 foi removido do firmware em 2026-10-07 (`9e7b309`: `src/hal/tle8888.{cpp,h}` e `tle8888_regs.h`). Feito em 2026-10-21: `docs/hw/README.md` marca como histórico o TLE8888 (§4) e os caminhos para os projetos KiCad removidos; `microruseefi_as_base.md` e `mcu_f407_to_h562_pin_map.md` ganharam a nota de links quebrados. `tle8888_crosscheck.md` e `tle8888_pinout.md` já estavam marcados como históricos e `vr_input_conditioning.md` como supersedido. Revisadas as menções restantes: `architecture_v2.md` (linha do driver atualizada para "removido"), `bom_v1_candidates.md`, `kicad_vendor_review.md` e `netlist_v1.md` ganharam aviso de histórico, `weact_h562_schematic.md` ganhou uma ressalva, e `interface_board_v1.md` e `pinout.md` já estavam corretos. Os itens "TLE8888" de Concluído são histórico e ficam.
- `docs/wiring_diagram.md` já está marcado como histórico (TLE8888 fora do projeto); o pinout vigente é `docs/hw/pinout.md`. Nada a atualizar.
- `.graphifyignore` agora exclui `tools/esp32_combined/ardustim_gui/assets/`.
- `tools/ts/OpenEMS_model.msq` (tune de exemplo do TunerStudio, gerado em 18/10) foi versionado em 2026-10-21.
- Pronto quando: `grep` por `hardware/openems_` e `README.md §` só acha menções históricas e `make ci-local` passa. Verificado em 2026-10-21 (`ci-local` OK). Fase 0 concluída.

### Fase 1 — Firmware sem silício novo

Itens de "Registrado e deixado de propósito", resolvidos com host-tests.

1. ~~USART1~~ feito em 2026-10-21: `system.cpp` seleciona PCLK2 (`RCC_CCIPR1` USART1SEL = 000) depois que o PLL assume o SYSCLK, e `uart.cpp` calcula o BRR a partir do SYSCLK real (125 MHz com PLL, 16 MHz se ficou em HSI16). Compila nas duas placas; o baud real só se confirma na bancada (Fase 2).
2. ~~FDCAN~~ feito em 2026-10-21, com as fontes da ST (`stm32h562xx.h` e `stm32h5xx_hal_rcc_ex.h`): `FDCANSEL` está em `RCC_CCIPR5` (0xE8) bits [9:8], com 00 = HSE (reset, 8 MHz aqui), 01 = PLL1Q, 10 = PLL2Q. `system.cpp` agora liga `PLL1QEN`, programa `PLL1Q` = ÷8 (VCO 500 MHz, 62,5 MHz) e seleciona PLL1Q depois do lock. `can.cpp` ganhou um `static_assert` do bit timing (500 kbps). Compila nas duas placas; o bit rate real só se confirma na bancada.
3. ~~`can0_tx`~~ feito em 2026-10-21: recusa o envio (retorna `false`) enquanto `FDCAN_CCCR_INIT` estiver ativo. Os chamadores em `can_stack.cpp` já tratam o retorno.
4. ~~Gauge de lambda~~ feito em 2026-10-21: firmware (÷5) e dash (×5) já concordavam; o `openems.ini` escalava `lambda` e `lambdaTarget` com 0,004 e agora usa 0,005 (λ=1,000 aparece 1,000 no TunerStudio). A resolução citada em `server.py` passou de ±0,004 para ±0,005.
5. ~~Páginas 0x0A e 0x0C~~ feito em 2026-10-21: escrita recusada (`page_is_fw_owned`, mesmo caminho da página 0x03) nos protocolos legado e envelope, com teste `test_ltft_pages_reject_writes`. O LTFT canónico vive em `g_ltft_*`/NVM adaptativo, não na página 10.
6. ~~CAN 0x402 e status bits~~ feito em 2026-10-21: o 0x402 agora usa `g_eng_cfg.injector_flow_cc_min` e `kCylinderCount` em vez de 450 cc/min e 4 cilindros fixos. O layout dos status bits (bits 0-7 em 0x400, 8-15 em 0x401 data[6]) estava certo, mas o override de `WBO2_FAULT` ficava só no 0x400, que não leva o bit 9, e o 0x401 enviava o status cru: a falha do WBO2 nunca chegava ao barramento. O override agora vale para a palavra inteira. Testes `test_can_stack_fco_uses_configured_flow` e `test_can_stack_wbo2_fault_reaches_0x401`.
7. ~~CKP, pulso substituto~~ decidido em 2026-10-21: mantém o limiar de 0,9, sem mudança de código. Ruído em f ≥ 0,9 é tomado como o próprio dente (adiantado em até 0,1 dente = 0,6°) e a borda real que vem depois cai como ruído, então o `tooth_index` fica exato. Evidência nova: `test_ckp_noise_above_substitute_threshold` (f = 0,95 e 0,99) e o cenário `noise f=0.95,0.99` do `precision-test` (erro de ignição máximo 0,006°, 0 faíscas perdidas ou espúrias; precision-test em 165 PASS).
8. ~~`DiagnosticManager`~~ revisado em 2026-10-21: o `diagnostic_tick` já estava ligado e testado (bloco de 100 ms do `main_stm32.cpp`); o debounce e o timeout de recuperação eram constantes mortas (removidas); o único efeito sobre o motor é o corte por CRITICAL (`is_system_ready` → `diag_critical`), que não deve ter debounce. Corrigido: as falhas de faixa de MAP/TPS/MAF e `MAP_TPS_CORRELATION` agora são limpas quando a condição some (antes ficavam presas). `docs/protection_system.md` descrevia o `LimpManager` do rusEFI, não este código, e foi reescrito. Debounce de WARNING/ERROR e persistência em NVM foram para o Backlog.
- Pronto quando: `make ci-local` verde, `host-test WERROR=1` com 0 FAIL, `precision-test` com 0 FAIL, `ini-check` e `dash-check` sem findings e as duas placas compilando.

### Fase 2 — Bancada WeAct VGT6 + ESP32

Bloqueia as fases seguintes. Roteiro em `docs/bench_test_manual.md`.

- Confirmar a Rev X do chip (marcação e `REV_ID` em `DBGMCU_IDCODE`) antes de qualquer ensaio.
- Gravar o `.bin` por DFU (`dfu-util`, § 2 do manual), sem bobinas nem injetores, sem reset loop nem hard fault.
- Scope das saídas INJ/IGN (BSRR, TIM5_CH3): latência e jitter. CKP/CMP sintético de 200 a 8500 rpm, com perda e retomada de sync.
- USB CDC real: IRQ e endpoints, sem bloquear o caminho crítico, RX ≥ envelope.
- Registrar `loop2_last`, `loop2_max`, `late_event_count`, `cycle_schedule_drop_count` e `calibration_clamp_count`.
- IWDG com o gate do ETB, e Flash sem escrita com o motor girando.
- Ferramentas: `tools/diag/` (`sync_loss.py`, `bench_check.py`, `angle.py`) e `tools/hil_test/hil_test.py`.
- Modelo do relatório: `docs/bench_report_template.md` (inclui as verificações das correções de baud da UART, bit rate do CAN, gauge de lambda, bit de falha do WBO2 e páginas 0x0A/0x0C).
- Pronto quando: há um relatório de bancada com T3 e T4 medidos e sem falhas.

### Fase 3 — Medidas e calibração

Depende da Fase 2.

- Curva de CLT/IAT com pull-up de 2,49 kΩ. Escala dos sensores de pressão de combustível e de óleo. Fase do VVT.
- PWM real do ETB, ADC com o motor parado, rodapé de calibração na Flash.
- Offsets EXTI do flex (RPR1 0x00C, FPR1 0x010) e registradores do CAN.
- Ângulo na luz de ponto durante o rev limit.
- Confirmar `PA2 = ETB_TPS1 / INP14` no package real (auditoria ADC de 2026-07-16, tabela abaixo).
- CMP em LOW (FULL_SYNC só no crank) e 2º canal RMT para a phase_A: backlog de sincronismo, não de canal ADC.

| Sensor | Pino | INP (`adc.cpp`) |
|--------|------|-----------------|
| MAP | PA3 | INP15 |
| TPS | PA4 | INP18 |
| KNOCK | PA5 | INP19 |
| APP1 / APP2 | PC0 / PC2 | INP10 / INP12 |
| ETB_TPS1 / TPS2 | PA2 / PC5 | INP14 / INP8 |
| CLT / IAT | PB0 / PB1 | INP9 / INP5 |
| FUEL / OIL / EWG | PC4 / PC1 / PC3 | INP4 / INP11 / INP13 |

### Fase 4 — Gate pré-motor

Depende de hardware fora do escopo atual. Só critérios.

- Polaridade CKP/CMP contra o datasheet do sensor Hall. Ruído sob carga. Gate do ETB, com autocal e PID desacoplados do motor.
- Depois: cranking sem faísca, faísca sem combustível, partida.

### Backlog

- Datalog em SD: implementado e testado no host (VGT6, blocos brutos, 100 Hz; ver [`datalog.md`](datalog.md)). O driver SDMMC compila mas nunca rodou em hardware: validar na Fase 2 (itens no relatório de bancada). O SDMMC1 conflita com INJ3/4 e IGN3/4 na RGT6, então só existe na VGT6. Taxa até 1 kHz, comando `L` (liga/desliga/taxa) e contadores no realtime já existem. Falta um botão no dash (hoje só `/api/datalog`).
- `DiagnosticManager`: debounce de WARNING/ERROR (o CRITICAL continua imediato), timeout de recuperação e persistência dos DTCs em NVM. Só vale quando houver um consumidor dos códigos (dashboard ou serviço); a gravação em Flash exige motor parado (errata ES0565). `ADC_RECOVERY_FAILED` em CRITICAL não é limpo até o reset: decidir se isso é desejado.
- STFT com ganhos escalonados pelo atraso: implementado (`fuel_stft_gain_scale_q8`, calibração `stft_gain_ref_delay_ms`, página 0 offset 348, `stftGainRefDelay` no TunerStudio). Vem desligado (0 = ganhos fixos). Falta afinar a referência com motor e WBO2: kp/ki valem para o atraso de referência e são multiplicados por ref / atraso (25% a 200%).
- Pesquisa: redes neurais int8, só nos blocos lentos e depois do datalog em SD e da medida de `loop2_max`. Ordem: parâmetros X-τ (antes, testar a tabela com mais eixos), feedforward do STFT junto com o X-τ e saúde da sonda, diagnóstico/anomalia, feedforward aprendido no ETB/EWG (o PID continua mandando). Estudo em [`nn_viabilidade.md`](nn_viabilidade.md).
- Desenho em aberto: proxy de TC por `rpm_dot`, cascata STFT/LTFT, taper de partida do ETB, interlock de `'Y'` e `'Z'`.
- Fora do escopo por decisão: placa v2 e seus drivers (`etb_driver.cpp` de 3 pinos para 2 pinos + SPI do L9960T, passo 0 de diagnóstico dos CIs novos).

## Registrado e deixado de propósito

Não viraram código na revisão de `src/`.

- CKP (`src/drv/ckp.cpp`, `classify_edge` por volta da linha 308 e o substituto por volta de 598): decidido em 2026-10-21 (Fase 1, item 7). O limiar de 0,9 fica; f ≥ 0,9 tem teste e cenário no simulador.
- Lambda (`limp_gating.cpp`, por volta de 247). A restauração exige λ válido, λ ≤ alvo+desvio, e a banda de saída (RPM, carga ou TPS). Injetor fechado mantém λ pobre, então a condição pobre bloqueia a saída. `engine_calc_reset` não zera `g_lambda_cut`. O latch sobrevive a um stall. Zeram o corte o reset de teste, timeout 0, lambda desligado e o próprio caminho de restauração.
- Falha de CLT publica 90,0 °C (`kFallbackCltDegcX10 = 900`) e falha de IAT publica 25,0 °C (`kFallbackIatDegcX10 = 250`), em `sensors.h` e no tick de 100 ms. O fallback de MAP é 101,0 kPa. A curva NTC não foi inventada. O fallback de CLT continua alimentando partida e DFCO.
- USART: resolvido em 2026-10-21 (Fase 1, item 1). Falta medir o baud na bancada.
- FDCAN: resolvido em 2026-10-21 (Fase 1, item 2). Falta medir o bit rate na bancada.
- `can0_tx` recusa o envio quando o bit 0 de `TXBRP` está pendente. Não consulta `FDCAN_CCCR_INIT`. A init conta falha se INIT não entra ou não sai. O TX ignora essa falha.
- Gauge de lambda: resolvido em 2026-10-21 (Fase 1, item 4).
- `map_window_enable` nasce 0. A janela angular de MAP fica desligada até a calibração ligar.
- Páginas 0x0A e 0x0C: resolvido em 2026-10-21 (Fase 1, item 5): a escrita é recusada.
- CAN 0x401: bytes 0–3 são `fuel_press_bar_x1000` e `oil_press_bar_x1000`. CAN 0x402 acumula combustível com `kFcoFlowCcMin = 450`, constante, e não lê `injector_flow_cc_min`. O bit de falha WBO2 é forçado dentro de `tx_0x400` e só os bits 0–7 desse word saem no 0x400. O 0x401[6] leva `status_bits >> 8` do argumento da chamada.
- Não reabertos, e por isso sem confirmação nova: `g_t_etb_ms` depois de `output_test`, perda de tick em `g.time_ms` das auxiliares, `micros()` sob ISR de prioridade maior.

## Concluído (referência)

### Revisão e correção de `src/` (2026-10, mesclada em `main`)

- 20 commits em cima de `c4a83df`. Os 7 do primeiro passe podem não compilar sozinhos. Os 13 seguintes foram feitos um por tema, e cada um compilou e passou `make host-test` na árvore que o contém.
- Primeiro passe, do mais antigo ao mais novo: `9b6e91f` HAL · `4f94a28` drivers · `50b99ee` agendador · `e104299` núcleo do motor · `440a5ee` borboleta · `d8adc43` aplicação · `d8f2461` main e registro de testes.
- Continuação, do mais antigo ao mais novo:
  - `b0939b3` `fix(drv):` APP e TPS do ETB a cada 2 ms, falha de faixa acima do raw 4000.
  - `118b1d9` `fix(drv):` baro em repouso recusa vácuo e não empurra a leitura para 70 kPa.
  - `a9e62f6` `fix(throttle):` uma rearme automática do ETB, só com RPM zero, pedal abaixo do idle e a lâmina fora da contagem da falha. O DTC sai depois de seguimento bom. A segunda falha fica latente até o reset.
  - `7e787ab` `fix(hal):` sem USB enumerado o orçamento de TX é só o da UART. Com USB enumerado vale o mínimo dos dois e o espelho.
  - `c7fa5de` `fix(hal):` flex válido só entre 45 e 155 Hz. Etanol 0% até 50 Hz e 100% a partir de 150 Hz.
  - `c8b0084` `fix(hal):` a segunda `adc_init` volta na hora quando a primeira já completou sem recuperação.
  - `76bb214` `fix(engine):` o slot de 100 ms chama `diagnostic_tick(100)`.
  - `7bb0621` `fix(main):` removido o ramo morto de auto-burn da page 0. `g_calib_dirty` saiu. A page 0 queima pelo comando `b`. O flush adaptativo de LTFT/knock no slot de 500 ms permanece, ainda limitado pelo RPM seguro.
  - `8e32871` `test:` bake de VE 254.
  - `e57f17e` `test:` STFT parado a 39,0 °C e em movimento a 60,0 °C.
  - `d611fb8` `test:` X-tau no eixo de MAP atual (25 e 150 bar×100).
  - `2b2943d` `test:` inibição de ignição e stall a partir da fila.
  - `58e9500` `test:` uma família de corte do `limp_gating_update` por caso, e o re-latch do ETB.
- O que o primeiro passe já tinha corrigido:
  - Críticos: corrida dos watchdogs de dwell e injetor, mapa de pedal que dava a volta, watchdog de seguimento do ETB, PWM com base de clock 250 MHz, LTFT que rebaixava VE de boost.
  - Altos: ADC sem dentes, TIM6, rearme de DMA, SMPR 47,5 ciclos, teardown no stall do CKP, prime de partida, debounce de plausibilidade, APP em falha contra DFCO, CAN TX/RX, loaders do NVM, rodapé BEGIN/END+CRC, parser com timeout, flex no EXTI, relógio da marcha, VVT parado.
  - O corte por sobretemperatura crítica do diagnóstico continua independente do bit PROTECT. O teste de precisão "diag critical only" trata isso como contrato.
- Segundo passe, fechado: os itens 1, 3, 4, 5, 7, 8 e os dois menores que o plano aceitou (UART, flex, `adc_init`, `diagnostic_tick`, ramo morto da page 0) estão nos commits acima. O IWDG ficou como está escrito no `main`: o kick só ocorre se o slot do ETB rodou nos últimos 200 ms, e o timeout de runtime continua ~0,8 s.
- Decisões dos testes da fase 3:
  - VE 254: o teto do bake é a própria célula quando ela já passa de `kLtftAccumVeMax` (200). `fuel_trim.cpp` devolve false de propósito, limpa as estatísticas da célula e não desenrola o STFT. O plano pedia `CHECK` true. O teste ficou `CHECK_FALSE`: VE permanece 254, o contador de commit não anda e o STFT não muda. A produção não foi alterada para devolver true.
  - STFT: uma amostra rica (λ 800 contra alvo 1000) a 39,0 °C permanece no grampo 250. A 60,0 °C desce de 250. O limiar da malha é `clt > 400` (×10).
  - Inibição: bobina em LOW descarta o DWELL pendente. Bobina já em carga termina o dwell e a centelha sai na hora programada. O stall com a bobina em HIGH e a fila já esvaziada não chama o fim do evento, põe o pino em LOW e incrementa o contador de clear.
  - `limp_gating_update`: rev limit, flood, óleo, rail, lambda, ETB, boost, duty e sem sync no Hall. O clear do ETB não segura se o report vier em seguida. Óleo fora de faixa corta combustível e centelha. Rev limit, rail, flood, lambda, boost, duty, ETB e falta de sync cortam só combustível, fora o `spark_skip`.

### Produto e firmware

- **Higiene de código monorepo** (2026-07): secrets untracked + gitignore;
  untrack graphify/ardustim vendor; `WERROR` host+dual firmware; `ci-local` +
  lint-includes phase A/B; split host tests (1113 PASS); `vehicle_inputs`
  (ban ENGINE→APP); `hal/out_pins` BSRR inline; `nvm_boot`; ui_protocol TUs;
  `etb_pwm_*`. Documentado em `docs/qualidade_e_testes.md`.
- **Diagrama elétrico completo** (2026-06-19): power supply (buck 5V wide-Vin +
  LDO 3.3V + TVS SMBJ24CA), condicionamento de sinais (divisores, filtros RC,
  proteção TVS por canal), atuadores externos (ETB/EWG H-bridge, WBO2, flex fuel),
  conector ECU genérico (~47 pinos com VVT), star grounding (PGND/SGND/Shield).
  CAN transceiver integrado no TLE8888 (sem PHY externo). Correções ultrareview:
  divider flex fuel (3.3kΩ), ADC range NTC (0–3V), reguladores automotivos.
- **Flex fuel sensor** (2026-06-18): PB5/EXTI5, 50–150Hz → 0–100% etanol, duty
  → temperatura combustível. Stoich AFR auto-ajustado (14.7→9.8 conforme E%).
- **TLE8888 VRS conditioner** (2026-06-18): CKP reluctor entra direto no TLE8888
  VRS_IN → saída digital VRS_OUT → PA0/TIM5. Sem comparador externo.
- **Electronic Wastegate (EWG)** (2026-06-18): motor DC + H-bridge externo (PA6 PWM
  10kHz + PA7/PB4 DIR), PID de posição (2ms), cascata boost PI (20ms) → position PID.
  Calibração EWG gains + pos min/max em NVM page 0.
- **Remoção IACV** (2026-06-18): idle controlado exclusivamente pela ETB. PA6 liberado
  para EWG PWM.
- **TLE8888 integração completa** (2026-06-18): SPI2 driver (config/diag, 3.9MHz,
  timeout 500µs), INJ low-side (OC 10A), IGN push-pull (OC 6A), VRS conditioner,
  per-channel fault bitmap na telemetria + dashboard, watchdog 100ms.
- **Closed-loop fuel OEM-lite** (2026-07): STFT PI + delay, LTFT mult/add NVM,
  LEARN→VE manual (APPLY), nearest cell, gates (enable/post-start/min RPM),
  authority/rates calibráveis, DTCs P017x-lite, flush adaptativo rate-limited.
  Ver `docs/closed_loop_fuel.md`. Stack WP0–WP5 + LEARN cal (WP6).
- **WBO2 CAN ID configurável** (2026-06-17): offset 138 page 0, default 0x180.
  Idle RPM vs CLT movido para seção IDLE no dashboard.
- **Dashboard full English** (2026-06-17): tradução completa, Pedal Map Send/Burn/
  Reload pattern, Reload button restaurado em params groups.
- **DBW puro + boost por marcha** (2026-06-16): dashboard estilo MoTeC M1, subgrupos
  de parâmetros.
- **Trim combustível/ignição por cilindro** (2026-06-16): janela dente CMP.
- **Idle ETB + dirigibilidade** (2026-06-16): marcha lenta via ETB, floor + integrador
  I, warmup CLT table.
- **Unidades reais no dashboard** (2026-06-16): sem ×10/×100/µs nos valores exibidos.
- **Redistribuição eixo MAP** (2026-06-16): 9 linhas vácuo (20–110kPa) + 7 boost.
- **HIL 21/21 — math da ECU validada ponta-a-ponta** (2026-06-15): VE interpolado vivo,
  avanço e PW de injeção batem com o mirror Python em 800/1500/3000 RPM. Bench-mode
  (cmd USB `B`) força MAP/TPS/CLT/IAT e limpa SENSOR_FAULT. Script HIL corrigido
  (tabelas CLT/IAT corr 8× int16, PW usa VE interpolado).
- Clock 250 MHz real + USB CDC estável (cadeia de 6 bugs de regs.h, 2026-06-11)
- Comando `A` com ZLP; IWDG de produção (boot 10s → runtime 100ms)
- Dashboard de calibração (`tools/openems_dash/`): telemetria WS 30 Hz, editores
  VE/Spark/Lambda, parâmetros 1D/2D, config do motor, calibração de sensores
  com captura ao vivo, datalog CSV + relatório de tuning Markdown
- Eixo RPM máx 8000 (16 pontos redistribuídos)
- Página 7 (dwell 2D) acessível via protocolo; calibração de sensores ligada
  aos drivers (antes inalcançável)
