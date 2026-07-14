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
2. Célula **ready** (thresholds page0 185–190 ou defaults).  
3. Dash **APPLY** (`'Y'`) → bake em VE RAM + desenrola LTFT cell; bulk não desenrola STFT N×.  
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
| `'Y'` | `/api/ltft/apply-ready` | APPLY all ready → VE |
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
