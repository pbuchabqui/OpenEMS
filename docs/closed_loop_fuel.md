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
    │     (mult: o que a célula absorve sai do integrador do STFT)
    └─ LEARN accum (só mult) se sample_valid

2 ms PW path:
    VE bilineal + λtgt + (STFT + LTFT_mult[nearest]) + LTFT_add[nearest]
    + dead-time / X-τ / S-curve / ΔP …
```

**Módulos:** `fuel_trim.cpp` (STFT/LTFT/LEARN/delay), `fuel_calc.cpp` (PW/AE/decel).

## Cascata STFT → LTFT (transferência sem salto)

O LTFT integra a saída do STFT (`cell += STFT/div`). Se o integrador do STFT não fosse descarregado, os dois integrariam o mesmo erro em série e o laço oscilaria (simulação com 10% de combustível curto e 300 ms de atraso: 46% de ultrapassagem, período de cerca de 70 s, mais de 130 s para assentar).

Por isso, no caminho multiplicativo, o que a célula absorve (variação real, depois do clamp) é subtraído do integrador do STFT. A correção total (STFT + LTFT) não muda na transferência; só a divisão entre os dois. Na mesma simulação: sem ultrapassagem, monótona, 90% em cerca de 60 s, STFT tendendo a 0. Teste: `test_fuel_stft_ltft_cascade`.

- **LEARN** acumula a correção total (STFT + LTFT da célula), que é a correção realmente necessária. O STFT sozinho perto de 0 daria bake nulo.
- **Commit**: o bake sobe a VE e o LTFT da célula desce o mesmo valor, então a total não muda. O STFT global não é desenrolado.
- **Saturação**: com o LTFT saturado no clamp a célula não drena, e o STFT volta a acumular até o seu clamp (`STFT_LIMIT_REACHED`).
- **Caminho aditivo (PW pequeno, em µs)**: a mesma transferência, convertendo os µs absorvidos em %x10 do PW atual (`Δµs × 1000 / PW`), com o resto da divisão acumulado para que 1 µs sobre um PW grande não trunque a 0.
- **Gate "ready" do LEARN**: como a média acumulada é a correção total, o limite de |média| sobe de 15% para 25% (`kLtftAccumReadyMaxMeanStftX10` = 250, igual ao clamp padrão do LTFT). A página 12 continua em int8 e satura a exibição em ±12,7%; é só exibição, os gates usam o valor completo.

### Estabilidade e ganhos do STFT

Varredura em simulação do STFT isolado (planta com combustível 10% curto, atraso de transporte e filtro de sensor de ~300 ms, `stft_kp_x100` = 3). Primeira entrada em ±1%, em segundos, e "instável" quando o erro nos últimos 30 s passa de 1%:

| `stft_ki_x1000` | atraso 0,2 s | 0,6 s | 1,0 s | 2,0 s |
|---|---|---|---|---|
| 5 (antigo) | 46 s | 46 s | 45 s | 44 s |
| **10 (padrão)** | 22 s | 22 s | 21 s | 20 s |
| 20 | 11 s | 10 s | 9 s | 8 s |
| 40 | 5 s | 4 s | 4 s | 4 s |
| 80 | 2 s | 2 s | 2 s | instável |
| 160 | 1 s | 1 s | instável | instável |
| 320 | 0 s | instável | instável | instável |

O limite de estabilidade no pior atraso (2 s) fica entre 40 e 80, então `ki` = 10 deixa cerca de 6× de margem e dobra a velocidade (assenta em ~21 s em vez de ~45 s). Com ruído de sensor de ±1%, a variação pico a pico do STFT fica em 0,2% com ki = 5 e 10 e em 0,4% com ki = 20. O termo P (`stft_kp_x100`) quase não muda o resultado. O escalonamento por atraso (`stft_gain_ref_delay_ms`) permite subir o ganho nos pontos de atraso curto.

Limite do que isto prova: a planta é um modelo (atraso mais filtro de primeira ordem). A afinação final exige motor e WBO2.

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

## Densidade IAT (`T_ref/T`)

A densidade do ar no PW base é `corr_iat_density_q8` (gás ideal, `T_ref` = 298.0 K). `iat_corr_x256` é só margem de proteção (anti-detonação em IAT alto), default neutro até ~60 °C.

Uma tabela IAT antiga em NVM (page 4) com o V-shape pré-`T_ref/T` `{272,264,256,256,264,272,280,288}` **não é carregada** — empilhava ~18 % de fuel extra a −20 °C em cima do termo físico. VE tunada contra o IAT velho fica magra acima de 25 °C em malha aberta (STFT ±25 % esconde em fechada) até retune.

## Persistência

- Shadows LTFT mult/add em NVM adaptativo (Bank2, magic `LTF3`).  
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

**Interlock.** `'Y'` e `'Z'` só executam na repetição: o 1º byte arma e responde `0x02` (`'Y'` responde `[0x02][0x00]`), o mesmo comando em até 5 s executa e consome o arme; outro comando ou o prazo vencido rearmam. Um byte solto no fio não grava a VE nem zera o aprendizado. O cliente do dash (`protocol.py`) repete sozinho depois do `confirm()`. Não há gate de estado do motor (decisão: só proteção contra byte espúrio).
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
