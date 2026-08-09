# Fork MT6835 / TIM2 encoder — objetivo e estado

Branch: `feat/mt6835-encoder`, criado a partir de `main` (58c71d9), isolado em
worktree separado (`~/openems-mt6835-encoder`) para não tocar em
`hw/v1-clean-board`, que tinha alterações não commitadas no momento da criação
deste fork.

## Objetivo

Explorar a substituição do CKP/CMP atual (Hall + roda 60-2, `TIM5` input
capture) por um encoder magnético absoluto **MT6835** (SPI + saída ABZ), lido
em **modo encoder de hardware** no `TIM2`, com disparo de eventos de
ignição/injeção em **domínio de ângulo** em vez do domínio de tempo que o
scheduler usa hoje.

Isto nasceu da avaliação de um documento externo (revisão multi-rodada de um
pseudocódigo hipotético de firmware usando MT6835) que não correspondia à
arquitetura real do OpenEMS. A avaliação completa está preservada no histórico
de conversa que originou este fork; o resumo técnico relevante está abaixo.

## Mecanismo proposto

- `TIM2` em modo encoder: `CH1`/`CH2` decodificam quadratura ABZ do MT6835
  (posição angular absoluta, contada em hardware, sem ISR por borda).
- `CH3`/`CH4` do mesmo `TIM2` ficam livres (não usados pelo modo encoder) e
  passam a gerar compare-match diretamente contra `TIM2->CNT` — ou seja,
  disparo por **ângulo alvo**, não por timestamp previsto.
- Isto elimina a conversão ângulo→tempo que hoje é feita uma vez por gap em
  `ecu_sched_angle.cpp` (aproximação que assume RPM ~constante até o próximo
  dente) — o disparo por ângulo é imune a essa aproximação porque compara
  contra posição real, não contra um tempo previsto.

## Restrição de segurança que NÃO desaparece

O dwell watchdog real do código atual (`ecu_sched_dwell_watchdog()`,
`src/engine/ecu_sched.cpp:548`, ~linha 548 em `hw/v1-clean-board` — conferir
offset neste branch) depende de `TIM5_CNT` continuar avançando **mesmo com o
motor parado**, porque o cenário que protege é exatamente esse: motor cala
com a bobina a meio da carga, sem giro nenhum para gerar novos pulsos de
ângulo. Um contador de ângulo (`TIM2` em modo encoder) **congela** nesse
cenário — não serve como base de tempo para o watchdog.

Consequência de projeto: mesmo adotando disparo em ângulo via `TIM2`, é
obrigatório manter uma referência de tempo pequena e **independente da
rotação**, só para:
- watchdog de over-dwell,
- lógica de "motor parou → forçar saídas seguras".

Candidato natural: `TIM6` — hoje só usado como trigger do ADC
(`src/hal/stm32h562/regs.h:445`), livre para ser reaproveitado como contador
de tempo leve para este propósito. Não precisa da sofisticação do `TIM5`
atual (32 bits, fila de eventos) — só precisa nunca parar de contar.

## Gates a fechar ANTES de escrever driver/firmware de verdade

Nenhum código de driver deve ser escrito enquanto estes dois pontos não
estiverem fechados — construir em cima deles agora seria basear-se em números
não confirmados (a mesma armadilha que o documento externo caiu, citando um
teto de "12 MHz" que não existe em nenhuma revisão do datasheet MagnTek):

1. **Datasheet primário do MT6835** — confirmar diretamente na fonte (não
   aceitar números de nenhuma análise de IA, incluindo esta): ABFreq real,
   footprint, pinagem, e comparação real de compatibilidade com
   MT6825/MT6816 caso a intercambialidade de footprint importe para o
   hardware.
2. **Mapeamento PA0/PA1 → `TIM2_CH1`/`CH2` (AF1) no LQFP100** — hoje só
   documentado por comentário em `src/hal/stm32h562/regs.h:203`
   ("AF1 = TIM1/TIM2  AF2 = TIM3/TIM4/TIM5"), não verificado pino a pino.
   Confirmar contra `docs/alternatefunctions.pdf` (já presente no repo) antes
   de assumir que os pinos atuais do CKP/CMP servem para isto sem mudança de
   hardware.

## Diferença conhecida entre este fork e `hw/v1-clean-board`

Este branch parte de `main`, que está **15 commits atrás** de
`hw/v1-clean-board` e não tem:

- `docs/hw/architecture_v2.md` (decisão CKP=Hall, TLE8888 → CIs dedicados
  MC33810/TPS65381A/L9960T/CJ125).
- Versões atualizadas de `src/hal/board_pinout.h`, `src/hal/out_pins.{h,cpp}`,
  `src/hal/stm32h562/regs.h`, `src/hal/tle8888.cpp`.

O núcleo do scheduler (`src/engine/ecu_sched.cpp` e afins — dispatcher `TIM5`,
fila de eventos, BSRR, dwell watchdog) é **idêntico** nos dois branches; a
divergência está isolada em HAL/pinout/hardware. Quem for portar trabalho
deste fork de volta para `hw/v1-clean-board` precisa revisar esses ficheiros
manualmente — não é um merge direto.

## Fora de escopo por agora

- Driver SPI do MT6835.
- `tim2_encoder_init()`.
- Qualquer mudança em `ecu_sched*.cpp`.

Isto só começa depois dos dois gates acima estarem fechados com fonte
primária confirmada.
