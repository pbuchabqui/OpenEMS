# Contrato de saída: o que o motor recebe, atuador por atuador

Gerado da leitura do código (não da execução). Referências são `arquivo:linha` de `src/` e podem
derivar; confirme antes de citar. Cobertura de teste: **H** host (`make host-test`),
**P** `make precision-test` (bordas reais nos pinos), **V** `make host-test-vgt6`, **B** bancada.

Pipeline: `engine_calc_step()` (2 ms, `engine/engine_calc.cpp`) -> `limp_gating_update()` (máscaras de
corte) -> `ecu_sched_*` (fila de eventos TIM5) -> `out_pin_write()` (BSRR). Saídas ativas em nível alto;
repouso = LOW (`hal/out_pins.h`). INJ/IGN: RGT6 PA15/PB3/PC10/PC11 e PC6-PC9; VGT6 PE0/2/4/6 e PE9/11/13/15.

## 1. Injetores (INJ1-4)

| Caso | Gatilho | O motor recebe | Teste |
|---|---|---|---|
| Boot / parado | `outputs_safe_early`, `pw=0` | Nada. Só prime (dente 1-20, <= 30 ms, respeita máscara) | V, H |
| Partida (cranking) | rpm <= `crank_enter`, latch até `crank_exit` | REQ_FUEL x mult(CLT) 3,0x..1,15x, piso `crank_min_pw_us` (2,5 ms), sem VE/trim | H, P |
| HALF_SYNC cranking | `allow_half_crank_batch` | Simultâneo, 2 esguichos, sem VE/STFT/AE | H |
| Flood clear | cranking e APP >= `crank_flood_tps` | PW=0, máscara 0x0F | sem asserção no pino |
| Pós-partida | mult 1,35x -> 1,05x, 0,5-2,4 s | Fluxo x ASE; trims/AE/DFCO desligados | H |
| Marcha lenta / normal | FULL_SYNC sem corte | VE x REQ_FUEL x MAP x IAT / lambda, +trim ±50%, CLT/IAT, S-curve, dead time (Vbatt), delta-P; <= 100 ms; <= 90% do ciclo | H, P |
| DFCO | TPS fechado, CLT >= 70, rpm >= 1500 | PW=0 (sem máscara); rampa na saída | H |
| Limitador de giro | rpm >= `rev_limit` (7000) | Máscara 0x0F (combustível); retoma em limite - 200 rpm | H, P |
| Limp (sensor) | MAP/CLT/óleo em falha ou CLT >= 105 | Combustível e faísca cortados acima de 3000 rpm (histerese 20 rpm) | H (novo) |
| Falha MAP | bit MAP | **Não corta (novo)**: usa o MAP do modelo; limp corta acima de 3000 rpm | H (novo), P |
| Óleo (sensor, pressão baixa) / trilho / boost / lambda / duty / ETB (harness=1) / flood | `limp_gating.cpp` | Combustível cortado, faísca mantida; sensor de óleo e superaquecimento crítico cortam também a faísca | P (novo, nos pinos de injetor e bobina) |
| Perda de CKP em marcha | sem dentes | Todas as 8 saídas em LOW e sem nova atividade | P (novo) |
| Teste de saídas | `output_test` | Pulso <= 30 ms, ignora máscara; aborta com rpm>0 ou 5 s | H, B |

## 2. Bobinas (IGN1-4)

| Caso | O motor recebe | Teste |
|---|---|---|
| Partida | `crank_spark_deg` (8°) sem correções; dwell por tabela Vbatt x rpm | H, P |
| Normal | Tabela (rpm, MAP) + IAT + CLT + marcha lenta - anti-jerk/surge - torque - overrun; [-20°, +60°] | H, P |
| Por cilindro | + trim ±15° - knock (<= 30°); **clampado a [-20°, +60°] no escalonador (novo)** | P (knock) |
| Dwell | Tabela Vbatt x rpm; **piso 0,5 ms (novo)**, teto 10 ms; pulado se faltar < metade | H (novo), P |
| Soft-limit | Skip rotativo <= 50% na janela abaixo do limite (máscara vale por 720°); sem corte em rpm >= limite | H (máscara), P (novo, nos pinos) |
| Multi-spark | Até 3 pares extras, rpm < 1500 | H (fila) |
| Corte por limp/óleo sensor/superaquecimento | Máscara 0x0F + purga de dwell na fila | P (novo) |
| Watchdog | Dwell > 1,4x -> SPARK forçado | H, P |

## 3. ETB (borboleta)

Cadeia: pedal -> `torque_manager_update` (limp 25%, rev pullback, launch, TC, piso de partida,
integrador de marcha lenta, dashpot, slew) -> `etb_control_update` (PID ±1000) -> `etb_driver_set_motor_pwm`
(IN1 abre, IN2 fecha, PWM 10 kHz).

| Caso | O motor recebe | Teste |
|---|---|---|
| `etb_cal_valid=0` ou desabilitado | Shutdown (IN1=IN2=0, duty 0): mola devolve a lâmina | H (alvo) |
| Falha TPS (3 strikes/plausibilidade) | `ETB_FAULT`: shutdown; corte de combustível > 1500 rpm até reiniciar | H (parcial) |
| APP em falha / limp MAP-CLT | Alvo limitado a 25% | H |
| Limitador de giro | Fecha para 0 (**novo**: agora usa `g_rev_limit_active`) | H (alvo) |
| Entrar em teste de saídas | **Shutdown na entrada (novo)**; depois ±30% sob comando | H (novo) |
| Autocal na partida | Varre ±400/1023; falha mantém calibração da flash | H |
| Reset | Pinos flutuam até `etb_driver_init` (ver lacuna 11) | B |

## 4. Auxiliares

| Saída | Regra | Teste |
|---|---|---|
| Ventoinha | Liga CLT >= 95,0; desliga <= 90,0; **liga com CLT em falha (novo)** | H (novo) |
| Bomba | key-on: prime 2 s; roda enquanto rpm>0; desliga 2 s após rpm=0. **Novo: `set_key_on(true)` no boot, então prime de 2 s e depois segue o rpm** | H (só com key-on forçado) |
| A/C | Entrada PB11 (50 ms): só soma ar de marcha lenta no ETB; sem saída de embreagem | H (entrada injetada) |
| Flex | Só sobrepõe a estequiometria (AFR); sem atuador | - |
| EWG / VVT | **Não montados.** `EMS_EWG_POPULATED 0`: driver sem efeito; o boost só altera telemetria | - |

## 5. Estado seguro por pino

INJ/IGN: LOW em escrita única no boot (`out_pins_hw_init`); INJ1 em PA15 (RGT6) depende do pull-up do JTDI
(comentário em `main_stm32.cpp:221`). ETB, ventoinha, bomba e VVT só são configurados em `etb_driver_init` /
`auxiliaries_init`, depois do boot: antes disso valem os pull-downs externos. Watchdog IWDG ~0,8 s: um travamento
mantém o último PWM do ETB até o reset.

## 6. Lacunas (estado após a segunda rodada)

Corrigidas e testadas: dwell sem piso; avanço por cilindro sem clamp final; `limp_rpm_cut` sem histerese;
limitador de giro sem fechar o ETB; PWM do ETB herdado ao entrar em teste; ventoinha parada com CLT em falha;
bomba (prime de 2 s no boot e depois segue o rpm, `main_stm32.cpp`); falha de MAP não corta mais (segue com o
MAP do modelo, limp limita a 3000 rpm); Vbatt fora de 6..18 V sobe `vbatt_fault` (STATUS_SENSOR_FAULT) e usa
12 V em dwell e dead time; ETB, ventoinha e bomba em LOW logo após o reset (`etb_driver_pins_safe_early`,
`auxiliaries_pins_safe_early`); código morto removido (`limp_gating_fatal`, `phase_valid`/`sequential`, `etb_allow`).

Abertas:
- `etb_harness_present` fica em 0 por decisão do projeto: falha de TPS do ETB não corta combustível e o autocal na partida é pulado.
- Verificar no hardware o PB4 (IN2 do ETB, RGT6): reseta como NJTRST com pull-up. `etb_driver_pins_safe_early` o
  derruba, mas há uma janela entre o reset e essa chamada.
- Nunca medido em hardware: nada desta rodada foi gravado nem medido na bancada. Os testes são de host e de
  simulador (`tools/ci_local.sh` verde, inclusive os dois firmwares com `-Werror`).
- Vbatt abaixo de 6 V (queda real na partida) usa 12 V nominal em dwell e dead time, como já fazia o driver.
  Um valor entre 0 e 6 V real ficaria melhor preso em 6 V; só a leitura 0 (divisor aberto) pede o nominal.
- Falha de MAP não corta mais abaixo de 3000 rpm. Se o modelo de MAP estiver errado (antes de convergir, em
  boost), a mistura fica errada e não há bit `kProtectDis` nem corte que reponha a proteção.
- A bomba parte do boot (inclusive após reset do watchdog) e não há caminho de key-off: `set_key_on(false)` não
  tem chamadores.
- Dashboard (`tools/openems_dash/protocol.py`) ainda nomeia os bits de corte removidos (map_fault, fatal, no_sync
  de fase); eles simplesmente nunca acendem.

Achado dos testes de pino: o soft-limit por skip de faísca entregava só metade do ratio pedido (máx. ~25% em vez
de 50%), porque a máscara por volta atingia um cilindro que não disparava naquela volta. Corrigido em
`spark_skip.cpp` (máscara por ciclo de 720°); `test_spark_skip.cpp` e o cenário "spark skip" do `precision-test`
cobrem. O padrão de fábrica continua desligado (`spark_skip_window_rpm_x10 = 0`).
