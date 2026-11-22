# Sistema de ignição

> **Status:** vigente · validado 2026-11-21 (reescrito contra o código; o texto anterior descrevia a arquitetura do rusEFI)

O avanço e o dwell são calculados no passo de 2 ms do `engine_calc` e entregues ao `ecu_sched`, que arma o carregamento (DWELL) e a faísca (SPARK) das 4 bobinas por ângulo. Saídas IGN são active-high; estado seguro = LOW.

## Pipeline

1. **Avanço base** (`src/engine/ign_calc.cpp`): `get_advance_x10_prepared` interpola a tabela de avanço 20×20 (`int8`, graus inteiros) por RPM e carga, com resolução de 0,1°. O lookup preparado é o mesmo da VE e do lambda alvo.
2. **Correções** (`AdvanceCorrectionsX10`, em décimos de grau): IAT, CLT (aquecimento do catalisador), marcha lenta (`calc_idle_spark_correction_x10`), anti-jerk, retardo de torque (TC e launch, vindo do `torque_manager`) e retardo de pé levantado (`calc_overrun_spark_retard_x10`, acompanha a rampa do DFCO). O retardo de driveline (`calc_driveline_retard_x10`) entra junto do anti-jerk: vale o maior dos dois.
3. **Soma e limite**: `calc_total_advance_x10` soma e `clamp_advance_x10` limita a `kAdvanceMinX10..kAdvanceMaxX10` = −20,0°..+60,0°. Em modo luz estroboscópica, `ign_running_advance_x10` troca o valor calculado pelo fixo. Cranking usa avanço fixo.
4. **Knock**: o retardo por cilindro (`src/engine/knock.cpp`) não entra na soma; vai direto ao agendador (`ecu_sched_set_cyl_retard_x10`). Retardo persistido em NVM por cilindro; o limiar adaptativo não é persistido.
5. **Dwell**: `dwell_ms_x10_from_vbatt_rpm` = tabela por tensão da bateria × fator por RPM (Q8), com arredondamento. Tempo convertido em ticks de 16 ns do TIM5. O comando de dwell tem piso de 0,5 ms e teto `kMaxDwellTimeMs`.
6. **Agendamento** (`src/engine/ecu_sched.cpp`): o DWELL começa em `spark − dwell` e o SPARK dispara no ângulo, ambos ancorados na captura de hardware do dente (imunes à latência da ISR). Com fase confirmada pelo CMP o ciclo é sequencial (720°, uma faísca por cilindro por ciclo); sem fase, "presync" a 360° (faísca perdida). Multi-spark reaproveita a fila de eventos (48 posições); overflow nunca descarta OFF/SPARK.
7. **Cortes**: `limp_gating` (único escritor) decide as máscaras de inibição de IGN; o corte de faísca deixa a bobina carregada disparar no ângulo certo, não em tempo aleatório. Watchdog de dwell em 1,4× do comando no SysTick, independente da fila.

## Arquivos

- `src/engine/ign_calc.cpp`: tabela, correções, anti-jerk, driveline, overrun, dwell.
- `src/engine/engine_calc.cpp`: passo de 2 ms que junta avanço, dwell e injeção.
- `src/engine/ecu_sched.cpp`: fila de eventos, armamento por ângulo, watchdogs.
- `src/engine/knock.cpp`: janela, limiar e retardo por cilindro.
- `src/engine/limp_gating.cpp`: cortes de ignição.

## Anti-jerk (OpenEMS, `src/engine/ign_calc.cpp`)

Retardo de avanço no tip-in, somado em `engine_calc.cpp` à correção de driveline (vale o maior dos dois). Dispara quando `tpsdot` passa de `antijerk_tpsdot_threshold_x10` (%/s ×10; padrão 3 %/s) e usa magnitude proporcional a `tpsdot` até `antijerk_retard_deg` (retardo cheio a 100 %/s). O pico do tip-in é mantido durante a janela e o retardo cai em rampa linear até 0.

- **Janela:** byte 70 da página 0 (`antijerk_decay_x10ms`) em unidades de 10 ms (0 a 500 ms; padrão 20 = 200 ms). Antes do 2026-11-19 o byte era "ciclos" de passo de 2 ms, o que dava ~8 ms. A janela tem piso de 2 voltas do virabrequim, porque o avanço só chega ao scheduler uma vez por volta (tabela angular refeita na fronteira da volta). Páginas gravadas com o valor antigo (4) passam a valer 40 ms.
- **Rearme:** só depois que `tpsdot` cai abaixo do limiar (sem re-disparo contínuo com o pedal ainda subindo).
- **Reset:** em partida e pós-partida o estado é zerado.
- Sem efeito na bancada atual (sem combustão); limiar, retardo e janela pedem carro e datalog.
