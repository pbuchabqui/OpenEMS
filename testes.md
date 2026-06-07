# OpenEMS — Auditoria de Cobertura de Testes

**Build:** `make host-test` | **Resultado atual:** 509 PASS 0 FAIL (commit d6c4d12)  
**Data:** 2026-06-06 | **Auditor:** análise estática + revisão por subagentes

---

## Metodologia

Para cada função pública (não-estática, exposta por header) verificou-se:
1. **Existe teste?** — secção `section()` que chame a função
2. **Asserções são eficazes?** — detectariam uma regressão se a lógica interna fosse alterada
3. **Casos críticos cobertos?** — caminhos de erro, clamps, limites, efeitos sobre estado global

Classificação de cada problema:
- 🔴 **Crítico** — função sem teste ou teste tautológico; regressão não detectada
- 🟡 **Médio** — teste existe mas cobre apenas o caminho feliz; risco moderado
- 🟢 **OK** — cobertura adequada para o risco da função

---

## 1. `src/engine/fuel_calc.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `get_ve` / `get_ve_prepared` | 🟢 | Ambas chamadas; prepared == unprepared verificado |
| `get_lambda_target_x1000` | 🟢 | Testado; clamp [650-1200] verificado |
| `get_lambda_target_x1000_prepared` | 🟡 | Chamada em tabela, mas asserção só verifica `>= 650` — clamp superior não testado |
| `calc_req_fuel_us` | 🟢 | Fórmula base testada |
| `default_req_fuel_us` | 🟢 | Testado indiretamente |
| `calc_base_pw_us` | 🟢 | Verificado: ve=0→0, map>300→0, overflow →100ms |
| `calc_base_pw_us_default` | 🟡 | Chamado mas usa `map_ref_bar_x100` (não baro dinâmico — isso é `calc_fuel_pw_us_default_fast`) |
| `apply_lambda_target_pw_us` | 🟢 | Lambda fora de range → pass-through testado |
| `apply_fuel_trim_pw_us` | 🟢 | Trim ±50% verificado |
| `corr_clt / corr_iat / corr_vbatt` | 🟢 | Valores tabelados verificados por ponto |
| `corr_warmup` | 🟢 | Enrichment > 256 abaixo de 70°C verificado |
| `calc_final_pw_us` | 🟢 | Aritmética verificada |
| `calc_fuel_pw_us_default_fast` | 🟡 | Testado mas **baro compensation não verificada**: teste usa baro=101 (padrão); nunca verifica que baro=70 dá PW diferente (maior) |
| `fuel_ae_set_threshold / set_taper` | 🟡 | Setters verificados indiretamente via `calc_ae_pw_us` |
| `calc_ae_pw_us` | 🟡 | Pulso inicial > 0 verificado; **taper decay não testado** (definição taper=4 mas decaimento nunca verificado) |
| `fuel_reset_adaptives` | 🟢 | STFT → 0 após reset verificado |
| `fuel_lambda_delay_reset` | 🔴 | `CHECK_TRUE(true, "no crash")` — **tautologia pura** (linha 1155) |
| `lambda_delay_ms_from_rpm_load` | 🟢 | Valores interpolados verificados por 3 pontos |
| `fuel_update_stft` | 🟡 | Direcção (pos/neg) e decay verificados; **clamp ±250 não testado**; split additive/multiplicative não testado |
| `fuel_update_stft_delayed` | 🔴 | Só verifica `range [-250,250]` (linha 1226); **lógica de delay/ring-buffer nunca exercitada** |
| `fuel_get_stft_pct_x10` | 🟢 | Consistência com retorno de `fuel_update_stft` verificada |
| `fuel_get_ltft_pct_x10` | 🟡 | Out-of-range → 0 verificado; valor após aprendizado não verificado |
| `fuel_get_ltft_add_us` | 🟡 | Idem |
| `fuel_decel_cut_update / active / reset` | 🟢 | Hysteresis entrada/saída verificada |
| `fuel_set_baro / get_baro` | 🟢 | Round-trip + clamps verificados |

**Problemas a corrigir:**  
- F1: `fuel_lambda_delay_reset` — substituir tautologia por teste real  
- F2: `fuel_update_stft_delayed` — verificar que delay bloqueia STFT até elapsed > delay_ms  
- F3: `calc_ae_pw_us` — verificar decaimento taper (taper=4 → 0 após 4 ciclos sem aceleração)  
- F4: `calc_fuel_pw_us_default_fast` baro — verificar PW(baro=70) > PW(baro=101) mesmo VE/MAP

---

## 2. `src/engine/ecu_sched.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `ECU_Hardware_Init` | 🟡 | Testado: angle_table=0, wdog=0 após init; TIM mock registers não observáveis externamente |
| `ecu_sched_commit_calibration` | 🔴 | **Nunca chamada nos testes** — somente os setters individuais são testados |
| `ecu_sched_set_advance_deg/dwell/inj_pw/soi` | 🟢 | Setters e getters verificados |
| `ecu_sched_set_presync_enable` | 🟢 | Enable/disable verificados |
| `ecu_sched_set_presync_inj_mode` | 🟡 | Setter chamado; efeito no comportamento de injeção em HALF_SYNC não verificado |
| `ecu_sched_set_presync_ign_mode` | 🟡 | Idem para ignição |
| `ecu_sched_set_ivc` | 🟢 | IVC clamp verificado (ângulo > IVC encurta PW e incrementa clamp_count) |
| `ecu_sched_ivc_clamp_count` | 🟢 | Verificado |
| `ecu_sched_dwell_watchdog` | 🟢 | Disparo após 1.4× dwell verificado; one-shot verificado |
| `ecu_sched_dwell_watchdog_count` | 🟢 | Verificado |
| `ecu_sched_reset_diagnostic_counters` | 🟡 | Existe teste na secção "dwell watchdog" (linha 2141); counters → 0 verificado mas de forma implícita |
| `ecu_sched_fire_prime_pulse` | 🔴 | **Nunca chamada nos testes** |
| `ecu_sched_set_mspark` | 🟢 | Multi-spark: eventos adicionais no ângulo table verificados |
| `ecu_sched_set_inj_inhibit_mask / get` | 🟢 | Mask bloqueia injeção: table só tem eventos para cilindros não inibidos |
| `ecu_sched_set_ign_inhibit_mask / get` | 🟢 | Idem para ignição |

**Problemas a corrigir:**  
- E1: `ecu_sched_commit_calibration` — testar que chama atomicamente os 4 parâmetros e `sanitize_runtime_calibration`  
- E2: `ecu_sched_fire_prime_pulse` — testar edge cases: pw=0 (retorna sem fazer nada), pw>30000 (clampado), pw válido (não crasha)  
- E3: `ecu_sched_reset_diagnostic_counters` — tornar explícito: acumular wdog_count, late_count → reset → verificar =0

---

## 3. `src/drv/ckp.cpp`

| Função / Comportamento | Estado | Problema |
|------------------------|--------|----------|
| `ckp_snapshot` — state | 🟢 | WAIT_GAP, HALF_SYNC, FULL_SYNC, LOSS_OF_SYNC testados |
| `ckp_snapshot` — tooth_index | 🟢 | Incremento e wrap 57→0 verificados |
| `ckp_snapshot` — phase_A | 🟢 | Toggle em cam edge válido e glitch verificados |
| `ckp_snapshot` — rpm_x10 | 🟢 | Fórmula verificada |
| `ckp_snapshot` — tooth_period_ns | 🟢 | = ticks × 16 verificado |
| `ckp_snapshot` — predicted_tooth_period_ns | 🟢 | = actual em velocidade constante verificado |
| `ckp_snapshot` — last_tim5_capture | 🟢 | > 0 após dentes verificado |
| `ckp_seed_arm / disarm` | 🟢 | Counters testados |
| `ckp_seed_confirmed_count` | 🟢 | Seed confirmado após cam válido durante probação |
| `ckp_seed_rejected_count` | 🟢 | Seed rejeitado após timeout de probação |
| `ckp_get_cmp_glitch_count` | 🟢 | Glitch contado em edge inválido |
| `ckp_stall_poll` | 🟢 | Stall quando tim5 parado; não-stall com dentes recentes |
| `ckp_tim5_ch1_isr` — LOSS_OF_SYNC por >63 dentes | 🟢 | Testado |
| `ckp_tim5_ch1_isr` — LOSS_OF_SYNC por gap precoce | 🟢 | Testado |
| `ckp_tim5_ch1_isr` — glitch < kMinToothTicks | 🟢 | Testado |
| `ckp_tim5_ch1_isr` — incoerência (períodos alternados) | 🟡 | Tentativa prévia abandonada por complexidade; incoerência genuína não testada |
| `prime_on_tooth` | 🟢 | Disparo no dente certo, one-shot verificados |
| `ckp_tim5_ch2_isr` (cam) | 🟢 | Phase toggle + glitch testados |

**Problemas a corrigir:**  
- C1: Incoerência — feed 3 dentes com período alternando 1× e 5× normal; verificar LOSS_OF_SYNC (requer ≥3 períodos inconsistentes consecutivos via `is_forward_rotation_coherent`)

---

## 4. `src/engine/knock.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `knock_init` | 🟢 | Init + threshold set/get verificados |
| `knock_set_event_threshold` | 🟡 | Setter chamado; efeito em `knock_cycle_complete` (threshold de contagem) não testado diretamente |
| `knock_set_adc_threshold / get` | 🟢 | Round-trip verificado |
| `knock_window_open / close` | 🟢 | Masking por cilindro verificado |
| `knock_adc_update` | 🟢 | Acima/abaixo do threshold testado |
| `knock_cycle_complete` | 🟢 | Retard acumula, recovery depois verificada |
| `knock_window_cycle_end` | 🟢 | Testado na fase 2 |
| `knock_get_retard_x10` | 🟢 | Clamp máximo verificado |
| `knock_save_to_nvm` | 🟡 | Só verifica que não crasha; NVM write count não incrementa no host (usa RAM shadow) |

---

## 5. `src/engine/map_estimator.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `map_estimator_init` | 🟢 | Init sem crash verificado |
| `map_estimator_update` | 🟡 | Valor estimado verificado após update; **não verifica convergência ao sensor com ganho > 0** |
| `map_get_estimated_bar_x100` | 🟢 | Consistente com update |
| `map_get_tpsdot_x10` | 🟡 | Só verifica range `[-1000,1000]`; static `s_last_time_ms=0` complica teste preciso |
| `map_is_transient` | 🟡 | Retorna bool — verificado mas valor esperado não pinado |
| `map_estimator_get_state` | 🟡 | Verificado apenas para campos struct existirem |
| `map_estimator_set_gains` | 🟢 | Setter verificado |

---

## 6. `src/engine/misfire_detect.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `misfire_init` | 🟢 | Init + mapa dente→cyl verificados |
| `misfire_reset` | 🟢 | Counters → 0 verificados |
| `misfire_get_event_count` | 🟢 | Out-of-range → 0 verificado |
| `misfire_clear_events` | 🟢 | Clear por cilindro verificado |
| `misfire_set_all_inhibit` | 🟡 | Setter chamado; **inibição de detecção nunca verificada com períodos lentos reais** |
| `misfire_on_tooth` — detecção real | 🔴 | **Nunca testada com período lento**: todos os testes usam períodos constantes → `evaluate_window` nunca dispara evento → counter permanece 0 → não detecta regressão no algoritmo de detecção |

**Problemas a corrigir:**  
- M1: `misfire_on_tooth` — alimentar FULL_SYNC com janela de dentes lentos (2× período normal) → verificar `misfire_get_event_count(cyl) > 0`  
- M2: `misfire_set_all_inhibit` — repetir com `misfire_set_all_inhibit(true)` → mesmo input → counter permanece 0

---

## 7. `src/engine/quick_crank.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `quick_crank_reset` | 🟢 | Estado limpo verificado |
| `quick_crank_update` | 🟢 | Cranking, afterstart, rpm=0 testados diretamente |
| `quick_crank_apply_pw_us` | 🟢 | Multiplicador verificado com valores concretos |
| `quick_crank_set_prime_context` | 🟢 | Via ISR path testado |
| `quick_crank_set_clt` | 🟡 | Setter chamado; efeito no multiplicador de partida a frio não verificado |
| `quick_crank_consume_prime` | 🟢 | One-shot verificado |
| `prime_on_tooth` (ISR hook) | 🟢 | Disparo no dente correto verificado |

---

## 8. `src/engine/transient_fuel.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `transient_fuel_reset` | 🟢 | Verificado |
| `transient_fuel_xtau_update` | 🟡 | Disabled→ pass-through verificado; enabled → **apenas verifica range `(0, 100ms]`**, não verifica que o modelo muda o PW de forma significativa |

---

## 9. `src/engine/xtau_autocalib.cpp`

| Função | Estado | Problema |
|--------|--------|----------|
| `xtau_autocalib_init / reset` | 🟢 | Verificado |
| `xtau_autocalib_update` não-transient | 🟢 | `returns false` verificado |
| `xtau_autocalib_update` transient | 🔴 | `CHECK_TRUE(any_update \|\| !any_update, ...)` — **tautologia pura** (linha 2585) |
| `xtau_is_learning` = true | 🔴 | **Nunca testado quando aprendizado ativo** |
| `xtau_get_state` | 🔴 | **Nunca chamada nos testes** |
| `xtau_get_current_params` | 🟡 | Verifica `x_fraction_q8 <= 255` e `tau_cycles >= 1`; valores aprendidos não verificados |
| `transient_fuel_xtau_with_autocalib` | 🟡 | Disabled→ pass-through verificado; enabled → bounds apenas |

**Problemas a corrigir:**  
- X1: `xtau_autocalib_update` — substituir tautologia; verificar `returns true` com ≥4 amostras válidas  
- X2: `xtau_is_learning` = true — verificar após atualização com learning  
- X3: `xtau_get_state` — verificar campos não-triviais após aprendizado

---

## 10. Módulos HAL

| Módulo | Estado | Observação |
|--------|--------|-----------|
| `hal/adc` | 🟢 | Todos os paths (init, read, trigger, recovery, timeout) cobertos |
| `hal/flash` (NVM) | 🟢 | Round-trip, flush, busy-poll, knock map, calibration save/load cobertos |
| `hal/timer` | 🟡 | Apenas "no crash" — registradores TIM são mocks sem comportamento observável |
| `hal/etb_driver` | 🟢 | Todos os estados e faults cobertos |
| `hal/can` | ❌ | **Fora do HOST_TEST_SRC — não compilado no host** |
| `hal/uart` | ❌ | **Fora do HOST_TEST_SRC — não compilado no host** |
| `hal/gpio` | ❌ | **Hardware puro — não testável no host** |
| `hal/usb_cdc` | ❌ | **Hardware puro — não testável no host** |
| `hal/stm32h562/system` | 🟡 | `millis/micros` testados; `system_stm32_init / iwdg_kick` não testáveis no host |

---

## Plano de Correção

### Alta Prioridade (tautologias e funções sem teste)

| ID | Função | Correção |
|----|--------|----------|
| X1 | `xtau_autocalib_update` | Substituir `any_update \|\| !any_update` por `CHECK_TRUE(any_update)` após 8 samples transient com erro=100 x1000 (dentro de [50,150]) |
| X2 | `xtau_is_learning` | Verificar `== true` após update que retorna true |
| X3 | `xtau_get_state` | Verificar campos `calibration_state >= 1` após aprendizado |
| F1 | `fuel_lambda_delay_reset` | Substituir `CHECK_TRUE(true)` por: chamar delayed com history preenchida → STFT atualiza; reset → chamar de novo → STFT não atualiza (sem history) |
| F2 | `fuel_update_stft_delayed` | Verificar que `now_ms=0, delay=200ms` → closed loop não ativado (sem sample na janela); `now_ms=300` → ativado |
| M1 | `misfire_on_tooth` detecção | Feed FULL_SYNC + dentes lentos (período=3×normal) dentro da janela de um cyl → `event_count > 0` |
| E1 | `ecu_sched_commit_calibration` | Chamar com 4 valores; verificar getters de avanço/dwell/inj_pw/soi |
| E2 | `ecu_sched_fire_prime_pulse` | pw=0 → no-op; pw=50000 → clamped (não crasha); pw=5000 → no crash |

### Média Prioridade (caminho feliz incompleto)

| ID | Função | Correção |
|----|--------|----------|
| F3 | `calc_ae_pw_us` taper | Chamar 5× sem delta TPS após pulse; verificar → 0 na 5ª chamada |
| F4 | `calc_fuel_pw_us_default_fast` baro | `fuel_set_baro_bar_x100(70)` → PW(baro=70) > PW(baro=101) com mesmo VE/MAP |
| E3 | `reset_diagnostic_counters` | Tornar explícito: wdog_count=1 após watchdog → reset → wdog_count=0 |
| M2 | `misfire_set_all_inhibit` | Verificar que inibição = true suprime detecção com mesmo input |

---

## Funções Intencionalmente Não Testadas (hardware puro)

As seguintes funções não são testáveis no host por dependerem de registradores de hardware real ou periféricos USB/CAN:
- `gpio_set_af`, `gpio_set_output_pushpull`, `gpio_write_pin`
- `usb_cdc_*`, `can0_*`, `uart0_*`
- `system_stm32_init`, `iwdg_kick`
- TIM mock writes (CCR, CCER, CR1 — estáticos privados em ecu_sched.cpp)

---

## Status Final por Commit

| Commit | PASS | FAIL | Seções |
|--------|------|------|--------|
| `5d6c787` | 323 | 0 | 10 |
| `e89e9c0` | 474 | 0 | 28 |
| `d6c4d12` | 509 | 0 | 31 |
| **Pós-correções (alvo)** | **~545** | **0** | **~38** |
