# Closed-loop fuel — OpenEMS (STFT / LTFT / LEARN)

Documento do firmware OpenEMS (não o modelo genérico rusEFI de `fueling_system.md`).

## Visão geral

```
WBO2 CAN (fresh)
    │
    ▼ 100 ms  (FULL_SYNC)
fuel_update_stft_delayed
    ├─ history + delay 3×3 (rpm/MAP/λtgt atrasados)
    ├─ gates: CL enable, CLT>70°C, O2, !AE, !cut, post-start
    ├─ PI → STFT global (freeze anti-windup se bloqueado)
    ├─ LTFT IIR (mult ou add por PW) se adapt_enable + RPM + MAP estável
    └─ LEARN accum (só mult) se sample_valid

2 ms PW path:
    VE bilineal + λtgt + (STFT + LTFT_mult[nearest]) + LTFT_add[nearest]
    + dead-time / X-τ / S-curve / ΔP …
```

**Módulos:** `fuel_trim.cpp` (STFT/LTFT/LEARN/delay), `fuel_calc.cpp` (PW/AE/decel).

## Célula nearest

Crédito, store e **apply** usam `table_axis_nearest_index` (igual ao highlight do dash VE).  
Não usar floor da bilineal para trims — mid-bin errava a autoridade (WP0).

## Gates (OEM-lite)

| Gate | Efeito | page0 / default |
|------|--------|-----------------|
| `closed_loop_enable` | off → freeze STFT+LTFT+LEARN | [80] = 1 |
| Post-start `closed_loop_post_start_s` | atrasa CL após CLT+O2 | [82] = 15 s |
| AE / cut / O2 stale / cold | freeze STFT (+ LTFT) | — |
| `ltft_adapt_enable` | off → só STFT | [184] = 1 |
| `ltft_adapt_min_rpm_x10` | abaixo: STFT ok, LTFT/LEARN freeze | [84] = 1200 RPM |
| MAP-dot > 8 kPa/tick | freeze LTFT/LEARN | constexpr |

## Authority / rates (page0 176–183)

| Campo | Default | Notas |
|-------|---------|--------|
| `ltft_mult_clamp_pct_x10` | 250 (±25%) | Separado de STFT clamp |
| `ltft_add_clamp_us` | 6350 | Offset µs |
| `ltft_learn_div` | 64 | IIR `cell+=(stft−cell)/div` |
| `ltft_commit_gain_pct` | 50 | Bake VE = mean×gain% |
| `ltft_max_step_x10` | 0 | 0 = sem cap de passo |

## LEARN → VE (manual)

1. Acumula hits/mean STFT/mean err em regime estável (APP + ΔRPM).  
2. Célula **ready** (thresholds page0 185–190 ou defaults) = qualidade / try_commit.  
3. Dash **APPLY** (`'Y'`) → bake em **todas** as células com hits>0 (não só ready)
   → VE RAM + desenrola LTFT cell; bulk não desenrola STFT N×.  
4. **Burn** VE: flag `ltft_apply_burn_ve` ou botão Burn do dash.  
5. Nunca auto-bake no closed-loop.

Ready wire page12: **bit7** de hits_wire = ready (fonte FW; host não reimplementa).

## Persistência

- Shadows LTFT mult/add em NVM adaptativo (Bank2, magic `LTF2`).  
- Dirty só se valor muda.  
- Flush: no máx. **1×/min** em run; **force** após `'Z'` / reset LTFT.  
- RPM seguro para qualquer write flash.

## DTCs (DiagnosticManager)

| Código | Condição (~5 s no sat) |
|--------|-------------------------|
| `STFT_LIMIT_REACHED` | STFT no clamp |
| `FUEL_TRIM_LEAN/RICH` | STFT sat + / − |
| `LTFT_LIMIT_REACHED` | LTFT mult célula apply no clamp |

Clear ~2 s fora do sat. Severidade WARNING.

## Comandos / API

| Wire | API | Acção |
|------|-----|--------|
| `'Z'` | `/api/ltft/reset` e `/api/adaptives/reset` | STFT+LEARN+LTFT shadow zero + flush ASAP |
| `'Y'` | `/api/ltft/apply-ready` | APPLY all accumulated (hits>0) → VE |
| `'B'` | `/api/bench_mode` | Liga/desliga bench CLT/IAT/λ (reseta λ→1.000 ao ligar) |
| `'L'` | `/api/bench_lambda` | Define λ simulado (u16 LE, 700-1275) — independente do enable de `'B'`, permite sequência de degraus sem re-armar o bench |
| page 10 | — | visualização LTFT mult+add |
| page 12 | LEARN tab | hits + mean STFT + ready bit |

## Smoke bancada (checklist)

1. Power-cycle BOOT0=0 → ACM / dash online.  
2. Bench λ ou WBO2: STFT move com enable=1.  
3. enable=0: STFT congela.  
4. RPM &lt; min adapt: STFT move, LEARN hits=0.  
5. Regime estável: page12 ready bit; APPLY altera VE.  
6. Z: STFT=0, hits=0, LTFT shadows 0.  
7. (Opcional) saturar STFT ~5 s → DTC STFT_LIMIT no DiagnosticManager.
8. **Degrau de λ para observar a resposta do STFT** (requer
   `closed_loop_post_start_s` já decorrido, ou zerado via write RAM de
   page0 para iteração rápida em bancada):
   1. `'B'` ON (bench CLT/IAT + arma λ simulado, reset para 1.000).
   2. Na aba Telemetria, ligar as séries `stft_pct` e `lambda_x1000` no
      gráfico deslizante (desligadas por default).
   3. `'L'` com um valor diferente de 1.000 — botão "SET λ" no dash, ao
      lado do BENCH. **Usar um degrau realista, 0.010** (ex. 1.000→1.010),
      não um erro grosseiro artificial — é a ordem de grandeza que o STFT
      corrige de facto em operação normal (ver `test_fuel_stft_convergence_time`).
   4. Observar o traço de `stft_pct` no gráfico. **Nota de resolução**: a
      página realtime reporta STFT em passos de 1% (`clamp_i8(stft/10,…)`)
      — com o degrau de 0,010λ, é uma "escada" lenta, não uma curva suave;
      esperado, não bug.
   5. Baseline quantitativo do **caminho encoder** (`stft_ki_x1000=10`, sem
      plant feedback — λ medido não reage ao trim neste bench):
      `test_fuel_stft_convergence_time` (`test/test_fuel.cpp`) mede
      **97 ticks × 100ms ≈ 9,7s** até o trim atingir 1,0% (o valor que
      cancelaria um erro constante de 1%λ) e ~2497 ticks (~250s) até
      saturar o clamp ±25% — a saturação é artefacto do clamp sem plant
      feedback, não é o número relevante para decisão de afinação. Em
      produção (`stft_ki_x1000=5`, Hall) o mesmo degrau leva ~20s a
      cancelar 1%λ — ver `test_math_stft_gains` (`test/test_math.cpp`), que
      cobre o default de produção.
      Histórico: até 2026-08-14 o termo proporcional (`p_x10 = error×Kp/100`)
      truncava a zero para qualquer erro <3,3% por dividir antes de somar
      ao integrador — a faixa que o STFT vê de facto em operação normal.
      Corrigido em `fuel_trim.cpp` (P e integrador combinam em ×1000, um só
      `/100` no fim) — este fix é universal, ambos os caminhos.
      `stft_ki_x1000` dobrado de 5→10 **só no caminho encoder**
      (`EMS_MT6835_ENCODER`, `calibration.cpp`) — mudança de afinação real,
      ainda não validada contra ruído de sensor λ real (só bancada com λ
      simulado limpo), por isso não foi estendida à produção. Qualquer
      afinação futura de Kp/Ki/clamp deve atualizar essa conta de
      propósito.

## Layout page0 (closed-loop)

| Offset | Campo |
|--------|--------|
| 80 | closed_loop_enable |
| 81 | ltft_apply_burn_ve |
| 82–83 | closed_loop_post_start_s |
| 84–85 | ltft_adapt_min_rpm_x10 |
| 175 | cal layout version (actual: 4) |
| 176–183 | LTFT authority/rates |
| 184 | ltft_adapt_enable |
| 185–190 | LEARN ready/sample thresholds |

Blobs com version &lt; actual **não** carregam 176–190 (mantêm defaults de compilação).
