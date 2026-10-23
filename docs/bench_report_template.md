# Relatório de bancada — WeAct H562 VGT6

Copie este arquivo para `docs/bench_reports/AAAA-MM-DD.md` e preencha durante o ensaio. Roteiro e critérios: `docs/bench_test_manual.md`. Não ligue potência (sem bobina, injetor ou motor). Deixe em branco o que não foi medido, em vez de estimar.

## Identificação

| Campo | Valor |
|---|---|
| Data / responsável | |
| Commit do firmware (`git rev-parse --short HEAD`) | |
| Imagem (`openems-vgt6.bin`, sha256 curto) | |
| Chip: marcação da revisão (última linha) | |
| Chip: `REV_ID` em `DBGMCU_IDCODE` (CubeProgrammer) | |
| Rev X confirmada? (se Rev A ou Z: parar e avisar) | ☐ sim ☐ não |
| Instrumento (osciloscópio / analisador), banda e taxa | |
| Gerador (ESP32, commit de `tools/esp32_combined` e saída de `STATUS`) | |
| Porta usada | ☐ USB CDC ☐ UART PA9/PA10 |

## Resumo

| Teste | Resultado | Capturas |
|---|---|---|
| T0 Boot seguro | ☐ passa ☐ falha ☐ não feito | |
| T1 Comunicação | | |
| T2 Sincronismo e RPM | | |
| T3 Ângulo da faísca | | |
| T4 Largura do pulso de injeção | | |
| T5 CMP: sequencial e faísca perdida | | |
| T6 Perda de CKP | | |
| T7 Carga e orçamento do loop | | |
| T8 Teste de saídas | | |
| T9 Reset com o motor girando | | |
| Gate Fase 2: T3 e T4 medidos e sem falhas | ☐ sim ☐ não | |

## Verificações novas desta rodada (correções de 2026-10-21, nunca medidas em silício)

| Item | Como medir | Esperado | Medido |
|---|---|---|---|
| Baud da UART (USART1 em PCLK2) | Analisador em PA9, 115200 8N1 (só se usar a UART) | 115200 ±2 % | |
| Bit rate do CAN (FDCAN em PLL1Q) | Osciloscópio em PB9 com transceptor, ou analisador CAN | 500 kbps, 2 µs por bit | |
| `lambda` no TunerStudio com λ = 1,000 | Frame WBO2 com 1000 mili-λ | 1,000 no gauge | |
| Falha do WBO2 no barramento | Sem frame 0x180 por 1 s; ler 0x401 byte 6 | bit 1 = 1 | |
| Escrita nas páginas 0x0A e 0x0C | `w` pelo envelope | erro (0x84) | |

## T0 — Boot seguro (10 ciclos, gerador a 3000 RPM)

| Ciclo | Borda em IGN1/INJ1 no boot? | Reset espontâneo? | RPM volta em poucos s? |
|---|---|---|---|
| 1 a 10 | | | |

## T1 — Comunicação (`bench_check.py --count 20`)

Linhas OK: ___ / 20. Erros de CRC ou tamanho: ___. Snapshot de 86 bytes: ☐ sim.

## T2 — Sincronismo e RPM (30 s por degrau)

| RPM do gerador | RPM reportado | Erro % (≤ 1) | `tooth` (µs) esperado | `tooth` medido | Estado final | late | drop | clamp |
|---|---|---|---|---|---|---|---|---|
| 200 | | | 5000,0 | | | | | |
| 500 | | | 2000,0 | | | | | |
| 700 | | | 1428,6 | | | | | |
| 1000 | | | 1000,0 | | | | | |
| 2000 | | | 500,0 | | | | | |
| 3000 | | | 333,3 | | | | | |
| 5000 | | | 200,0 | | | | | |
| 7000 | | | 142,9 | | | | | |
| 8500 | | | 117,6 | | | | | |
| Descida 8500 → 200 mantém `FULL`? | | | | | | | | |

## T3 — Ângulo da faísca (modo lâmpada em 10,0°; passa com erro ≤ 0,1°)

Ponto esperado em µs após/antes do dente 0: `graus × T ÷ 6`, com T = 60 ÷ (RPM × 60) s.

| RPM | Saída | Faísca esperada | Tempo esperado (µs) | Tempo medido (µs) | Erro (°) | Dwell (µs) | Dwell estável? |
|---|---|---|---|---|---|---|---|
| 1000 | IGN1 | 10° antes do dente 0 | 1666,7 | | | | |
| 1000 | IGN3 | 170° depois | 28333 | | | | |
| 1000 | IGN4 | 10° antes | 1666,7 | | | | |
| 1000 | IGN2 | 170° depois | 28333 | | | | |
| 3000 | IGN1 | 10° antes | 555,6 | | | | |
| 3000 | IGN3 | 170° depois | 9444 | | | | |
| 3000 | IGN4 | 10° antes | 555,6 | | | | |
| 3000 | IGN2 | 170° depois | 9444 | | | | |
| 6000 | IGN1 | 10° antes | 277,8 | | | | |
| 6000 | IGN3 | 170° depois | 4722 | | | | |
| 6000 | IGN4 | 10° antes | 277,8 | | | | |
| 6000 | IGN2 | 170° depois | 4722 | | | | |

IGN1 e IGN4 trocados? ☐ não ☐ sim (anexar CKP + CMP + IGN1). Modo lâmpada desligado ao final? ☐

## T4 — Largura do pulso de injeção

**Parte A (RPM 0, teste de saídas; passa com ±2 µs):**

| Injetor | Pino | Pedido (µs) | Medido (µs) | Erro (µs) |
|---|---|---|---|---|
| INJ1 | PE0 | 1000 / 5000 / 20000 | | |
| INJ2 | PE2 | 1000 / 5000 / 20000 | | |
| INJ3 | PE4 | 1000 / 5000 / 20000 | | |
| INJ4 | PE6 | 1000 / 5000 / 20000 | | |

**Parte B (3000 RPM, MAP 60, `FULL` + `seq`; passa com ±0,1 ms):**

| Injetor | `pw` reportado (ms) | Largura medida (ms) | Ordem 1-3-4-2 e uma abertura por 720°? |
|---|---|---|---|
| INJ1 a INJ4 | | | |

## T5 — CMP (2000 RPM, `cmp 2`)

Faísca perdida sem CMP (bit 11 = 0): ☐. Voltas até semissequencial: ___ (esperado 60 no modo bancada). RPM e `FULL` mantidos: ☐. `seq` volta após 2 confirmações: ☐.

## T6 — Perda de CKP (3000 RPM)

Tempo até todas as saídas em LOW: ___ ms. Alguma bobina presa em dwell? ☐ não ☐ sim. RPM cai a 0: ☐. Volta por `WAIT_GAP` a `FULL` sem reset: ☐.

## T7 — Carga (8500 RPM, 10 min)

| Contador | Início | Fim | Variação (esperado 0) |
|---|---|---|---|
| `loop2_last` (µs) | | | |
| `loop2_max` (µs, esperado bem abaixo de 2000) | | | |
| `late_event_count` | | | |
| `cycle_schedule_drop_count` | | | |
| `calibration_clamp_count` | | | |
| Resets durante o teste | | | |

## T8 — Teste de saídas (RPM 0)

| Saída | Pulso só no pino certo? | Dwell máx. (≤ 10000 µs) |
|---|---|---|
| INJ1 a INJ4, IGN1 a IGN4 | | |

Sem keepalive por 5 s o teste aborta e as saídas voltam a LOW? ☐

## T9 — Reset com o motor girando (3000 RPM, NRST × 10)

Pulso fora de hora nas saídas? ☐ não ☐ sim. Sincronismo volta toda vez? ☐. Status bit 15 cai (modo bancada é só RAM)? ☐

## Fase 2 — itens além de T0 a T9

| Item | Resultado |
|---|---|
| Sem reset loop nem hard fault após gravar o `.bin` | |
| Latência e jitter das saídas INJ/IGN (BSRR, TIM5_CH3), valor e método | |
| CKP/CMP sintético de 200 a 8500 rpm com perda e retomada de sync (`tools/diag/sync_loss.py`) | |
| USB CDC: enumera, não bloqueia o caminho crítico, RX ≥ envelope (`tools/hil_test/usb_stress.py`) | |
| IWDG: reset em ~0,8 s com laço travado e sem reset com o laço normal | |
| Flash: nenhuma gravação com o motor girando (observar `late`/`drop` ao salvar) | |
| Datalog SD (VGT6): cartão SDHC identificado, `sdlog.py format` feito antes | |
| Datalog SD: 10 min a 100 Hz, `sdlog.py decode` sem lacunas (0 blocos perdidos) e `records_dropped` = 0 | |
| Datalog SD a 1 kHz (`L` com taxa 3): `datalogDrops` = 0 em 10 min e `loop2_max` sem aumento | |
| Datalog SD: `loop2_max` com o log ativo igual ao sem cartão (sem aumento mensurável) | |
| Datalog SD: tirar a energia durante a gravação e religar; o log continua em nova sessão | |
| Taper do ETB (`PROFILE TAPER` no ESP32, APP em 0): a lâmina (`etb_target`) não cai abaixo de `etb_idle_min` até o fim do taper e não há salto na reentrada em marcha lenta; `SLEW` usado e se o sync se manteve | |
| Malha de lambda (WBO2): degrau de combustível de ±10%; o total (STFT + LTFT) segue sem ultrapassagem nem oscilação, o STFT volta perto de 0 e o LTFT guarda a correção (`stft_ki_x1000` = 10) | |
| Malha de lambda: PW abaixo de `ltft_add_pw_threshold_us`: mesma convergência, correção em µs no LTFT aditivo | |

## Falhas e anotações

| # | Teste | O que aconteceu | Captura / log | Ação |
|---|---|---|---|---|
| | | | | |

## Conclusão

☐ Gate da Fase 2 cumprido (T3 e T4 medidos e sem falhas). ☐ Não cumprido: ver Falhas.
Ao fechar, atualize `docs/ROADMAP.md` (Fase 2) e avance para a Fase 3.
