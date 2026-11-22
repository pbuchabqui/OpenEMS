# Sistema de marcha lenta

> **Status:** vigente · validado 2026-11-21 (reescrito contra o código; o texto anterior descrevia o IAC stepper do rusEFI, removido do OpenEMS em 2026-06)

A marcha lenta é controlada pela **borboleta eletrônica (ETB)**, sem válvula IAC, mais um ajuste rápido de avanço. O ar vem do `torque_manager` (`src/engine/torque_manager.cpp`) e o avanço de `ign_calc.cpp`.

## Pipeline

1. **RPM alvo**: `auxiliaries_idle_target_rpm_x10(clt)` (`src/engine/auxiliaries.cpp`) interpola o alvo pela temperatura do líquido (motor frio gira mais alto). `engine_calc` repassa o alvo ao `torque_manager` e à correção de avanço.
2. **Fase de marcha lenta**: com o pedal fechado (`APP < etb_idle_open_pct_x10`) e sem rev limiter, launch ou TC pesado, a fase "idle" é travada com histerese de RPM: entra abaixo de `alvo + banda` e só sai acima de `alvo + 1,5 × banda`, para não alternar entre lenta e "coasting". A banda reusa `idle_spark_window_above_target_x10`.
3. **Ar em malha aberta (cranking)**: durante a partida a abertura é fixa (`crank_open_pct_x10`, preferindo `etb_idle_max_opening_x10`, nunca abaixo do mínimo).
4. **Ar em malha fechada**: integrador de ar (`g_idle_air_int_x10`, em pct×10) acumula a abertura necessária para manter o alvo, limitado entre `etb_idle_min_opening_x10` e `etb_idle_max_opening_x10`, com passos de fechar/recuperar configuráveis. O A/C (`torque_idle_set_ac_load`) soma `idle_ac_air_x10` ao piso de abertura, e a marcha engatada soma `idle_gear_air_x10`.
5. **Avanço rápido**: `calc_idle_spark_correction_x10` (`ign_calc.cpp`) corrige o avanço pelo erro de RPM (curva `idle_spark_err_corr_x10` ou ganho `idle_spark_rpm_per_deg_x10`, com zona morta e limites de retardo/avanço). Resposta rápida pelo avanço, ar lento pela ETB. Só vale com TPS e MAP baixos e RPM dentro da janela do alvo.
6. **Dashpot**: depois de soltar o pedal, a lâmina é mantida e decai exponencialmente até a abertura de lenta, suavizando o retorno e reduzindo o risco de morrer.

## Arquivos

- `src/engine/torque_manager.cpp`: integrador de ar, fase de lenta, taper de cranking, dashpot, A/C.
- `src/engine/auxiliaries.cpp`: alvo de RPM por CLT.
- `src/engine/ign_calc.cpp`: correção de avanço de marcha lenta.
- `src/engine/engine_calc.cpp`: passo de 2 ms que liga tudo.

## Notas de implementação

- **Cranking Taper**: After the crank exit the ETB floor is blended over 1 s from the
  cranking opening (`etb_idle_max`) to the idle I, then held there (never below
  `etb_idle_min`, even above the idle RPM band) until the taper ends. The end is
  `etbTaperMs` (page 0, 354) or, when 0, the end of the afterstart enrichment (CLT
  dependent, capped at 10 s). The idle I does not decay below `etb_idle_min` while the
  taper runs, so the floor no longer closes to 0 % and steps back at idle re-entry. After
  the taper the blade follows the pedal and the idle RPM band as usual. Host-tested only;
  the real RPM response needs the engine or the ETB on the bench.
