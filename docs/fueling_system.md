# Sistema de combustível

Speed-density em inteiros, calculado a cada 2 ms em `engine_calc_step()` (`src/engine/engine_calc.cpp`). O resultado é um pulso por abertura, em ticks do TIM5, entregue ao scheduler por `ecu_sched_commit_calibration_x10()`. A malha fechada (STFT/LTFT) está em [`closed_loop_fuel.md`](closed_loop_fuel.md); os cortes, em [`protection_system.md`](protection_system.md).

## Arquivos

| Arquivo | Papel |
|---|---|
| `engine/engine_calc.cpp` | Orquestra o passo de 2 ms: MAP fundido, política por estado de sync, pipeline do pulso, commit |
| `engine/fuel_calc.cpp` | REQ_FUEL, base speed-density, correções CLT/IAT, dead time, ΔP, curva S, 2 inclinações, AE, DFCO, proteção de duty, EOI |
| `engine/fuel_trim.cpp` | STFT, LTFT multiplicativo e aditivo, LEARN para a VE |
| `engine/map_estimator.cpp` | Fusão sensor + modelo do coletor, TPSdot |
| `engine/xtau_autocalib.cpp` | Modelo de filme de parede (X-τ) |
| `engine/quick_crank.cpp` | Partida: detecção de cranking, enriquecimento, prime, afterstart |

## Política por estado de sync

- **FULL_SYNC:** combustível de funcionamento (VE, trims, AE, X-τ).
- **HALF_SYNC em cranking:** batch simultâneo com o pulso de partida (REQ_FUEL × multiplicador por CLT, piso `crank_min_pw_us`), sem VE, STFT nem AE.
- **HALF_SYNC fora de cranking, flood clear, proteção ou sem sync:** injeção cortada. A centelha continua nos casos em que só o combustível é cortado.

## Pipeline do pulso (FULL_SYNC)

1. **Base:** `REQ_FUEL × VE × MAP / 101,325 kPa × T_ref / T_iat`, dividido pelo λ alvo e multiplicado por `1 + (STFT + LTFT)`. A pressão barométrica não entra: no mesmo MAP o cilindro recebe o mesmo ar em qualquer altitude.
2. **Correções de calibração:** CLT e IAT em Q8, limitadas a 0,25×–2,0×.
3. **LTFT aditivo:** offset em µs sobre o fluxo líquido, para pulsos pequenos.
4. **X-τ:** compensação do filme de parede com X e τ pela tabela de CLT. Fica desligado em cranking e é zerado no DFCO e sempre que a máscara de injeção está ativa (rev limit, lambda, boost, duty, ETB), porque nesses casos nada chega à parede.
5. **Limite de queda** (`fuel_pw_drop_max_us`), **AE/DE** pelo TPSdot e **rampa de retorno do DFCO**.
6. **Por abertura** (sequencial = 1, presync = 2 por ciclo): ΔP sobre o fluxo do ciclo, depois a correção de 2 inclinações e a curva S sobre o fluxo de cada abertura, e por fim o dead time, que nunca é escalado.

O laço entrega ao scheduler dois pulsos por abertura: o do modo de injeção em vigor e o do outro modo (`ecu_sched_set_inj_pw_pair`). A ISR do dente troca presync ↔ sequencial no gap, antes do próximo tick de 2 ms, e escolhe o pulso pelo modo em vigor ao armar o injetor (`inj_pw_ticks_now`). Sem isso, um injetor armado nessa janela saía com metade ou o dobro do combustível.

O pulso final é limitado a 100 ms. No scheduler, cada abertura ainda é limitada a 90 % do intervalo entre aberturas.

## Não linearidade do bico

- **2 inclinações** (`apply_injector_two_slope`): abaixo de `inj_small_pulse_break_us` o bico rende `inj_small_pulse_rate_q8 / 256` da vazão nominal. Zero em qualquer um dos dois desliga.
- **Curva S** (`apply_injector_scurve`): eficiência de entrega em função do tempo comandado.

As duas valem para uma abertura. Em presync o fluxo do ciclo é dividido em duas aberturas antes de aplicá-las: 800 µs de fluxo são duas aberturas de 400 µs, e não uma de 800 µs.

## Derivada do TPS (TPSdot) para AE e detecção de transiente

`map_estimator.cpp` mede o TPSdot entre a amostra mais nova e a mais antiga de um anel de 32 amostras a 2 ms (janela de 62 ms). Um passo de 0,1 % vale 1,6 %/s, abaixo do limiar de AE (3 %/s). A janela anterior de 4 amostras (6 ms) dava 16,7 %/s por passo, então ruído de 1 LSB no pedal disparava enriquecimento e era classificado como transiente médio.

Consequência para a calibração: depois de um degrau rápido o TPSdot fica não nulo por até 62 ms (antes 6 ms), então o adder de AE atua por mais tempo. As tabelas `ae_pw_adder_us` e `ae_de_pw_us` precisam ser reafinadas com motor.

## Autoridade do LTFT

O LTFT multiplicativo é limitado a ±25 % em RAM, mesmo que `ltft_mult_clamp_pct_x10` peça mais: a NVM guarda cada célula em int8 de 1 %, limitado a ±25. O LTFT aditivo vai até ±6350 µs (int8 de 50 µs).

## Pendências que dependem de motor

- Tabelas de AE/DE, X-τ, EOI e curva S são valores de partida, não medições.
- Escala do sensor de pressão de combustível (hoje 0–2,5 bar) e `fuel_press_nominal_bar_x1000`.
- Dead time por tensão abaixo de 9 V (a tabela fica plana).
