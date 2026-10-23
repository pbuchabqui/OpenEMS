# Controle de tração: proxy de flare

O corte de torque do TC (`torque_manager.cpp`) usa o slip nesta ordem: API externa, slip CAN (roda contra carro, `vehicle_inputs`) e, na falta dos dois, o proxy de flare de rpm descrito aqui. Tudo só vale com `tc_enable` = 1, APP ≥ `tc_app_min_x10` e rpm ≥ `tc_rpm_min_x10`.

## Por que não um limiar de rpm/s

Um limiar fixo de rpm_dot dispara em aceleração normal (1ª e 2ª marcha passam de 800 rpm/s com pedal), oscila (o corte derruba o rpm_dot e solta) e responde ao ruído da derivada de dois ticks. Sem velocidade do carro, o rpm_dot absoluto não separa patinagem de aceleração.

## Como funciona (`tc_flare_slip_x10`)

- A derivada do rpm é calculada a cada tick (2 ms) e passa por duas médias de primeira ordem em inteiro: rápida (20 ms) e lenta (150 ms).
- O sinal é o excesso `rápida − lenta`: patinagem é perda de carga, um degrau do rpm_dot. Aceleração normal sobe devagar e a média lenta a acompanha.
- Arma quando o excesso passa de `tc_rpm_dot_thresh` (rpm×10/s) e solta abaixo de metade dele. Armado, o slip sobe em rampa de 0% (metade do limiar) a 100% (1,5 vez o limiar).
- A média lenta congela enquanto o flare está armado ou em espera, para não absorvê-lo.
- Depois de soltar, o último slip é mantido por `tc_flare_hold_ms` (padrão 100 ms), o que evita o ciclo corte, queda do rpm_dot, solta.
- Subida de APP acima de `tc_flare_app_rise_x10` (padrão 20%) sobre a média de 100 ms bloqueia o proxy por 150 ms: o pisão no pedal gera rpm_dot alto legítimo.
- O slip passa pelo mesmo caminho de antes: `tc_max_reduction_pct_x10`, taxa `tc_reduction_rate_x10` e retardo de ignição proporcional.

## Calibração (page0)

| Campo | Offset | Padrão | Observação |
|---|---|---|---|
| `tc_rpm_dot_thresh` | 208 | 8000 (800 rpm/s) | Agora é o excesso sobre a média, não o valor absoluto. |
| `tc_flare_app_rise_x10` | 350 | 200 | 0 mantém o valor; limite 1000. |
| `tc_flare_hold_ms` | 352 | 100 | 0 mantém o valor; limite 1000. |

## Limites

É uma heurística: nunca será tão confiável quanto o slip CAN. Os valores padrão não foram medidos em carro. O ajuste do limiar pede datalog (SD) de uma saída em piso baixo aderência. Na bancada não há como validar.
