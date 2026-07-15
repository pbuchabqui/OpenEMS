# Review de branch — consolidação para `main`

**Base:** `565e319` (2026-07-04) → **HEAD:** `a0b37ba` (2026-07-14)
**Commits:** 76 (50 de `fix/cmp-false-sync-transition` + 26 de `refactor/ecu-sched-timing-safe-cleanup`)
**Revisor:** Claude (Opus 4.8) · **Review:** 2026-07-14 · **Consolidação:** 2026-07-15

> **Nota de consolidação:** a história é linear — `main` → 50 commits (fix/cmp) →
> 26 commits (ecu-sched cleanup + closed-loop fuel Fase 2). Sem conflito; o
> segundo branch parte do tip do primeiro. Este documento cobre os dois trechos:
> §1–§6 revisam os 50 commits originais; **§8 resume os 26 adicionais.**

> ⚠️ **Ruído no diff:** ~150k das ~152k linhas são geradas/vendored e ficaram
> **fora** do review de correção: `graphify-out/` (grafo de conhecimento) e
> `tools/esp32_combined/ardustim_gui/` (avrdude, FontAwesome, moment.js,
> package-lock). O código realmente revisado é o firmware `src/` (~2,3k linhas)
> e, em nível de resumo, o estimulador ESP32 e o dashboard.

---

## 1. Destaques

Apesar do nome, o branch vai muito além do fix de falso-sync do CMP. Os temas:

- **CKP/CMP** — fim do falso-sync a ruído (came desligado, PA1 flutuante), fim
  do deadlock do classificador em salto de RPM, reativação do stall-poll, e RPM
  reportado só com sincronismo (ruído de 60 Hz já não vira "60 RPM fantasma").
- **Fuel** — integrador STFT deixou de truncar a zero; acumulação LTFT por
  célula (Fase 1, só estatística); compensação de ΔP de combustível e S-curve
  do bico entram no pipeline de PW; rev-limiter reescrito estilo rusEFI.
- **Flash/NVM** — mapa de registradores NSCR/NSSR + BKSEL (Bank2), escrita em
  quad-word de 128 bits dentro de seção crítica, readback de verificação, e
  gate de layout por magic (`LTF2`) para as tabelas 20×20.
- **Tabelas 20×20** — migração 16→20 em VE, spark, lambda, LTFT e eixos, com o
  protocolo TS (envelope, page sizes) reescalado por `sizeof`/`kTableCells`.
- **Estimulador ESP32** — DAC real de MAP/TPS calibrado, slew de RPM sem restart
  do RMT, e WiFi STA + TCP 3333 para sobreviver a quedas de USB.
- **Diagnóstico** — osciloscópio CKP/CMP ('K'), contadores de perda de sync por
  caminho, estado da malha fechada no 'D', e modo de teste de saídas ('T').

**Veredito:** não encontrei nenhum **bug de correção novo** introduzido pelo
branch. Os fixes de firmware são bem fundamentados e, em geral, mais robustos
que o código que substituíram (dimensões por `sizeof`, `static_assert` de
layout, subtrações de tempo com sinal wrap-safe). Ver §4 para o único achado
latente (baixa severidade) e §5 para as mudanças de comportamento que **exigem
validação em hardware**.

---

## 2. Resumo por área

### CKP/CMP — sincronismo e decoder (`src/drv/ckp.cpp`, `ckp.h`)
- `51da5e1`, `1557ab6` — **RPM só com sync** (`rpm_if_synced`): sem HALF/FULL_SYNC
  devolve 0. Ruído periódico num CKP flutuante já não expõe RPM real, nem
  congela um "RPM fantasma" que bloqueava gates de motor-parado (burn, teste de
  saídas). O stall-poll também decai o RPM a 0 após timeout sem sync.
- `57dfa6f` — **falso stall por corrida ISR↔main**: `ckp_stall_poll` passou a
  subtração *signed* (`int32_t`) de `tim5_cnt_now − prev_capture`; se um dente
  chega entre a leitura do contador e a comparação, o elapsed fica negativo e
  não dispara stall. Era a causa original do disable de junho.
- `803816e` — **deadlock do classificador**: `consecutive_anomalies` agora
  **decai −2** em vez de zerar; num salto de RPM 2.5–3.75× o gap real caía na
  banda "normal" da média defasada e zerava o contador a cada 58 dentes, então
  o re-bootstrap nunca chegava ao threshold (CKP perfeito rejeitado p/ sempre).
- **Falso-sync do CMP** (o fix homônimo): gate de consistência de posição do
  came (`s_cmp_ref_tooth` ±3 dentes), resync por rejeições consecutivas
  (`s_cmp_reject_streak`), e fallback sequencial→wasted por ausência de CMP
  (`s_revs_since_cmp`, 6 revs em produção / 60 em bench). Perda de sync do CKP
  passou a **preservar** o estado do CMP (`cmp_confirms` cai a 1, não a 0),
  fechando só o gate exportado até uma borda de came fresca.
- `f2531a2`, `ddae403`, `f714bbf`, `cca96c6`…`3bc9715` — telemetria de bordas
  cruas + osciloscópio (rings de timestamp TIM5, comando 'K').

### Fuel — STFT / LTFT / pipeline de PW (`src/engine/fuel_calc.*`, `main_stm32.cpp`)
- `1d0dd14` — **integrador STFT** migrado de ×10 para **×1000**
  (`g_stft_integrator_x1000`): com Ki default, um erro de λ contribuía <1 unidade
  ×10/ciclo e **truncava a zero** — a integral nunca acumulava. Clamp e leitura
  reescalados por 100. Confere com a memória `stft-bench-lambda`.
- `e9e51de` — **acumulação LTFT por célula** (Fase 1): `LtftCellStats` (hits +
  somas de STFT/erro) com gate de regime (`ltft_accum_sample_valid`: closed-loop,
  Δrpm/Δtps, janelas de λ, |erro|, |STFT|). **Só estatística — sem commit na VE
  ainda.** Ver achado em §4.
- ΔP + S-curve — `apply_delta_p_compensation` (fluxo do bico ∝ √ΔP, via
  `isqrt_u32`) e `apply_injector_scurve` estão **wired** em `main_stm32.cpp:1003`,
  na ordem correta: `quick_crank → ΔP → S-curve → +dead-time` (dead-time nunca
  escala e só entra com fluxo > 0). Não é código morto.
- Rev-limiter reescrito — ver §5.

### Flash / NVM (`src/hal/flash.cpp`, `flash.h`)
- Mapa de registradores **NSCR/NSSR/NSCCR/NSKEYR único + `BKSEL`** (o antigo
  `*_CR2/SR2` estilo H7 não existe no H562; confere com a memória
  `flash-nscr-nssr-register-map-bug`).
- **Quad-word de 128 bits**: as 4 palavras de cada flash-word são escritas em
  sucessão imediata dentro de `CriticalSectionGuard` (sem poll de status nem IRQ
  no meio) — sem isso o write-buffer (WBNE/DBNE) nunca comita e o burn "sucedia"
  mas não sobrevivia ao power-cycle. Aplicado nos dois caminhos (`flash_write_words`
  e o state-machine `nvm_flush_adaptive_maps`).
- **Readback de verificação** em `nvm_save_calibration` (`memcmp` do destino).
- **Gate de layout por magic `LTF2`**: setor apagado ou layout antigo → zera e
  remarca dirty (flush regrava no layout novo). Offsets NVM derivados das
  dimensões (`kNvmOffLtft/Knock/LtftAdd/LayoutMagic/Seed`) — **verifiquei que
  não se sobrepõem** (LTFT 20×20=400B, knock 64B, add 10×10=100B, magic alinhado
  em 576, seed em 592; tudo dentro do setor).

### Tabelas 20×20 (`calibration.cpp`, `table3d.*`, `ui_protocol.cpp`, `openems.ini`)
- `f793861`, `91a96ec`, `747b0dc` — VE, spark e lambda expandidos 16×16 → 20×20;
  eixos e LTFT idem. `calibration.cpp` é migração mecânica de dados.
- Protocolo TS reescalado: `kTxSize 1024→2048`, `kEnvMaxPayload 519→807`,
  `kEnvMaxChunk 512→800`; buffers de página passaram a dimensionar por
  `sizeof()`/`kTableCells` em vez de literais (256/512). `static_assert` novo
  garante `sizeof(UiRealtimeData)==86` (a página RT cresceu para acomodar
  `map_fused`, `net_pw` e `ckpcmp_diag[16]`).

### Estimulador ESP32 (`tools/esp32_combined/esp32_combined.ino`) — *resumo*
- `3231cca`…`faaab2b` — MAP/TPS em DAC real de 8 bits + calibração (erro
  ≤1.3 kPa) + fit de 2 pontos do TPS + trim de offset. Confere com a memória
  `esp32-dac-map-tps`.
- `651241b` — arranque **simultâneo** dos loops RMT CKP/CMP (o arranque em
  sequência dava offset de fase aleatório por restart; memória `stim-rmt-cmp-drift`).
- `d62a6c8` — **slew de RPM** atualizando a RAM do RMT ao vivo, sem restart.
- `b4d6f6c`, `57daef0` — WiFi STA + TCP 3333 + diagnóstico de status no boot.

### Dashboard (`tools/openems_dash/`) — *resumo, revisão leve*
- 22 commits de UI: seleção múltipla (arraste/Shift), interpolação H/V, undo (X),
  contraste do heatmap, λ com 2 casas, gauges, navegação no grid 20×20,
  osciloscópio CKP/CMP como vista de ciclo 0-720° com trigger no gap e freeze.
- `f08cd1c` — **prime obrigatório** antes de escrever campos de página (bug
  anterior conhecido nesse arquivo; memória implícita).

### Diagnóstico / ferramentas
- `22c8218` — estado da malha fechada no 'D' (integrador, erro, contadores de
  bloqueio do STFT). `4fc4a85` — modo de teste de saídas ('T', aba TESTS).
- `5bca4ae` — ponte USB↔UART no nanoESP32-C6 p/ flashar ESP32 com USB morto.

---

## 3. Concerns levantados e **resolvidos** na revisão

Registro para não serem reabertos — todos verificados no código atual:

1. **Wrap do gate de posição do came** usa `kRealTeethPerRev` (58), não 60.
   ✅ Correto: `tooth_index` incrementa até 57 e volta a 0 (módulo 58), então a
   distância circular por 58 é a certa (`ckp.cpp:848,988`).
2. **`interpolate_xtau_2d` acessa `[xi+1][yi+1]`** numa tabela 4×4.
   ✅ Sem OOB: `table_axis_index` limita o índice a `size-2` (`table3d.cpp:22`),
   logo `xi+1 ≤ 3`.
3. **ΔP/S-curve seriam código morto.** ✅ São chamados em `main_stm32.cpp:1003-1005`.
4. **Rev-limiter soft (retardo de faísca) teria sumido.** ✅ Foi *substituído*
   de propósito (fuel-cut-only + histerese); offsets 80-85 reservados, `b565491`.
5. **`memcpy(g_page3_rt, &rt, sizeof(rt))` estouraria o buffer.** ✅ Protegido por
   `static_assert(sizeof(UiRealtimeData)==86)` == `sizeof(g_page3_rt)`.
6. **Mapeamento página→índice NVM (save vs load).** ✅ Consistente: VE 1/1,
   spark 2/2, lambda 3/3, corr 4/4; setores distintos (`kSectorCal0+page`).

---

## 4. Achado (baixa severidade, latente) — ✅ CORRIGIDO

> **Corrigido** — nos 26 commits de consolidação (§8) o acumulador foi movido para
> `fuel_trim.cpp`, que congela `hits` e somas juntos no teto (early-return em
> `hits >= 65535`), fix idêntico aplicado independentemente. **Teste de regressão
> adicionado neste branch de consolidação** (`test/mvp_bench_tests.cpp`, seção
> "LTFT accum stats") via novo hook `fuel_ltft_accum_tick_for_test`
> (`EMS_HOST_TEST` em `fuel_trim.cpp`); provado não-vacuoso. `make host-test`:
> 967 PASS / 0 FAIL.
>
> **Nota (descoberta ao escrever o teste):** o teto de 65535 é **inalcançável em
> produção** — o mesmo erro de λ que valida a amostra (`|err|≥4`) satura o
> integrador STFT acima de `kLtftAccumMaxStftX10`, e a amostra passa a ser
> rejeitada muito antes do teto. Logo o fix é puramente defensivo; o hook de teste
> injeta amostras direto no acumulador para exercitar a invariante. Descrição
> original abaixo.

**LTFT accum — média deriva se a célula saturar sem reset** ·
`src/engine/fuel_calc.cpp:769` (`fuel_ltft_accum_tick`)

`cell.hits` satura em 65535, mas `sum_stft_x10` e `sum_err_x1000` continuam a
acumular em toda amostra aceita. Como a Fase 1 é **só estatística (não há caminho
de commit/reset da célula no hot path)**, uma célula que receba amostras
indefinidamente:
- faz `mean = sum / hits` crescer com o **denominador congelado** após 65535 hits
  → `fuel_ltft_accum_cell_ready` pode virar uma célula estável para *not-ready*;
- eventualmente estoura `int32_t` na soma (UB) — na prática **inalcançável**
  (ready a 30 hits; exigiria horas contínuas no mesmo ponto de operação).

**Cenário de falha:** motor em regime perfeitamente estável numa única célula por
muitas horas com o acumulador ligado, sem nunca resetar a célula → leitura de
média incorreta (e, no extremo teórico, overflow com sinal).

**Recomendação:** quando a Fase 2 (commit na VE) for implementada, ela deve
resetar a célula (`fuel_ltft_accum_reset_cell`, já existe) ao comitar; ou
saturar as somas junto com `hits`. Não bloqueia — é higiene para a próxima fase.

---

## 5. Mudanças de comportamento — **validar em hardware**

Não são bugs, mas alteram o comportamento em motor real e o branch não tem
validação de HW (ver §6):

- **Rev-limiter rusEFI-style** (`main_stm32.cpp`): corte de **injeção 100%** ao
  atingir o hard limit, reativação abaixo de `hard − histerese`. A **ignição já
  não é cortada** pelo limitador (só em limp mode). Fim do retardo progressivo de
  faísca e do corte alternado de cilindros. Histerese com latch correto
  (`g_rev_limit_active`).
- **Multi-spark no presync/cranking** (`ecu_sched.cpp`): `emit_multispark` agora
  também dispara nos 4 canais IGN no modo presync/wasted — antes era exclusivo do
  sequencial. Aumenta a carga de dwell das bobinas em baixa rotação.
- **Modo de injeção presync default** mudou `SIMULTANEOUS → SEMI_SEQUENTIAL`, com
  auto-seleção (`g_presync_inj_auto`, ligado em `openems_init`).
- **`ckp_stall_poll` reativado** no loop principal (estava desligado desde junho
  pelos falsos stalls do wrap 16-bit do TIM3; agora com TIM5 32-bit + fix da
  corrida).
- **Máscaras de inhibit re-aplicadas** no `arm_channel` (`ecu_sched.cpp:527`) —
  tinham-se perdido na migração TIM3→TIM5 (`f42c450`); sem elas o corte
  por-cilindro (rev-limiter/limp) não fazia nada. Early-return antes do BSRR,
  timing-safe.
- **CKP ISR** passou a escrever os rings do osciloscópio a cada borda (2 escritas
  de array + índice). Trabalho novo no ISR timing-crítico (núcleo congelado) —
  desprezível, mas registrado.

---

## 6. Pendências e riscos conhecidos

- 🔴 **O fix de falso-sync do CMP é HOST-VERIFIED apenas — nada em hardware.**
  O `cmp_confirms=2` visto em bancada era **ruído**, não confirmação real. O
  pull interno é fraco e o ruído pode ser crank-correlacionado (memória
  `wasted-sequential-transition-state`). Toda a §5 precisa de bancada/motor.
- 🟡 **Estado de bench é RAM** — reenviar comandos ('B', λ simulado) após reset
  (memória `stft-bench-lambda`).
- 🟡 **`graphify-out/` polui o diff** (~150k linhas) — considerar `.gitignore`
  para o artefato gerado, se não for para versionar.
- 🟢 **TunerStudio "MS Lite" (free) recusa assinaturas custom** — bloqueio
  externo ao firmware (memória `ts-protocol-state`); a assinatura subiu para
  `OpenEMS_v1.3` neste branch.

---

## 8. Os 26 commits adicionais (`e9e51de`..`a0b37ba`)

Continuação a partir do tip de `fix/cmp-false-sync-transition`, em dois temas.
Cada um tem design/review próprio já documentado — aqui é só o resumo.

### 8.1 Closed-loop fuel OEM-lite / LEARN→VE (Fase 2)
Design autoritativo em **`docs/closed_loop_fuel.md`** (adicionado por `2874ab5`).

- `67d61cc`, `290d58e` — **LTFT Fase 2**: bake-in do mean STFT na VE (VE +=
  `mean × ltft_commit_gain_pct`), realinhando a semântica da Fase 1.
- `ded34dc` — **extrai `src/engine/fuel_trim.cpp`**: STFT/LTFT/LEARN/delay saem
  do `fuel_calc.cpp` para um módulo dedicado (o acumulador da Fase 1 migra p/ cá).
- `261b542`, `1d46de0`, `192a4ac` — **LEARN manual** com célula *nearest*
  (`table_axis_nearest_index`, igual ao highlight do dash), APPLY via comando
  `'Y'`; alinha contratos host/FW.
- `439db89`, `5382554`, `46ea040`, `28d2de4` — **gates OEM-lite** (CL enable,
  post-start, min RPM), **authority/rates** LTFT calibráveis (page0 176–190),
  **DTCs** de saturação STFT/LTFT (lean/rich), e **rate-limit** do flush NVM
  (máx. 1×/min) + `ltft_adapt_enable`.
- `f1d3e2e`, `d9370d7`, `f2265c9`, `8d823b6`, `11bb966` — **página 12 LEARN** no
  dash (visualização do acumulador, ready-bit7), enable/burn opcional, e comando
  **`'Z'`** (reset de adaptives: STFT+LEARN+LTFT shadow), que passou a zerar
  também o LTFT shadow.
- `a0b37ba` — fix: APPLY bakeia **todas** as células acumuladas, não só as ready.

### 8.2 Cleanup timing-safe do `ecu_sched`
Plano aprovado em `[[code-review-scheduling-remediation]]` (Tier 0→1→2, zero
regressão de jitter/precisão). Núcleo congelado (TIM5_CH3 + fila + BSRR) intacto.

- `ba678a5`, `69d076c`, `1a1a0f1` — cleanup sem retiming: separa os *cold angle
  builders* do hot path do dispatch; APIs públicas de diag/trace substituem os
  `__asm` peeks do protocolo (comandos 'G'/'V'/'D').
- `91c5da9` — **testes golden** de identidade (min-lead, fila, timestamps de arm):
  garantem que o cleanup não alterou a temporização.
- `c076924` — min-lead arms deixam de contar como `STATUS_SCHED_LATE`.
- `22126b5` — dwell watchdog fica armado até o pino IGN ir a LOW.

### 8.3 Protocolo / HIL
- `b430ec1` — **breaking(protocol):** remove o IVC da page0 e da realtime/sched
  (campo aposentado). Consumidores do protocolo precisam do layout novo.
- `27b137d`, `7ed5723` — HIL: lê o snapshot completo de 86 bytes da
  `UiRealtimeData`; poll de settle de RPM + rev-limit alto para testes de alta RPM.

**Cobertura de teste (consolidada):** `make host-test` = **967 PASS / 0 FAIL**,
incluindo `test_fuel_ltft_accum_commit_ve` (bake VE), a suíte LTFT
(`apply_nearest`/`authority`/`adapt_enable`), os golden do scheduler, e o novo
caso de regressão do teto do acumulador (§4).
