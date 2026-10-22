# Redes neurais no STM32H562: viabilidade

Estudo de 2026-10-22. Nada disto está implementado, e nada foi medido em hardware.

## Resumo

É viável com redes pequenas, quantizadas em int8, rodando fora do caminho crítico. Não entra no roadmap atual: faltam bancada (Fase 2) e dados reais de motor para treinar e validar.

## Fatos do repo

- Cortex-M33 a 250 MHz, com DSP/SIMD e FPU de precisão simples. Flash de 1 MB; RAM de 512 KB mais 128 KB de SRAM3 (`linker/stm32h562.ld`).
- `openems-vgt6.elf`: `.text` 85,7 KB, `.data` 2,8 KB, `.bss` 36,7 KB. Sobram cerca de 900 KB de Flash e 450 KB de RAM, então memória não limita.
- O build ARM usa `-mcpu=cortex-m33 -O2` sem `-mfpu` nem `-mfloat-abi`, ou seja, ponto flutuante por software. O motor é quase todo em ponto fixo; `float` só aparece em `etb_control.cpp` e `torque_manager.cpp`. Ligar a FPU exigiria mudar as flags, tratar o contexto FPU nas ISRs e revisar a latência do caminho crítico.
- Orçamento de tempo: ISR de CKP e agendamento em TIM5, tick de 2 ms (`loop2_max` ainda sem medida em bancada) e blocos de 10/20/50/100 ms em `main_stm32.cpp`.

## Custo estimado

Estimativa, não medida: um MLP int8 de 3 camadas com cerca de 1 000 MACs leva 5 a 20 µs a 250 MHz com CMSIS-NN. Uma CNN 1D pequena (janela de knock) leva de centenas de µs a poucos ms.

## Candidatos

| # | Uso | Onde rodaria | Avaliação |
|---|-----|--------------|-----------|
| 0 | Parâmetros X-τ da parede de combustível (ver seção abaixo) | bloco de 100 ms | Melhor encaixe; testar antes tabela com mais eixos. |
| 0b | STFT: feedforward aprendido e saúde da sonda (ver seção abaixo) | bloco de 100 ms | Antes, ganhos escalonados pelo atraso, sem rede. |
| 1 | Detecção de anomalia de sensor ou misfire (`misfire_detect`, `DiagnosticManager`) | bloco de 100 ms | Risco baixo: só alerta, não corta. |
| 2 | Classificação de knock (hoje: contagem acima de limiar em `knock.cpp`) | fora da ISR | Pede amostragem mais rápida e dados reais rotulados de motor. |
| 3 | Correção de VE/ignição aprendida | bloco de 100 ms ou offline | Ganho incerto frente a `table3d` + LTFT, que são auditáveis. |
| 4 | Estimativa de MAP/torque (`map_estimator`, `torque_manager`) | bloco lento | Já existe modelo físico. |

## X-τ (parede de combustível)

É o melhor encaixe. Hoje `xtau_autocalib.cpp` guarda X e τ numa tabela 4×4 (rpm × MAP), ajustada online pelo erro de lambda em transientes (12,5% por amostra), com fallback 1D por CLT. X e τ também dependem de temperatura do coletor, IAT, % de etanol, ângulo de início da injeção e derivada do TPS, que a tabela não cobre.

- A rede só estimaria X e τ (MLP pequena, 5 a 7 entradas, 2 saídas), a cada 100 ms ou quando muda o ponto de operação. O tick de 2 ms continua só com a conta física.
- A saída passa pelos limites que já existem (`learned_x_min/max`, `learned_tau_min/max`). STFT, LTFT e os cortes de lambda do `limp_gating` seguem valendo, então o pior caso é uma mistura levemente errada num transiente.
- Antes da rede, testar a tabela com mais eixos (CLT, etanol) ou grade mais fina: pode dar quase o mesmo ganho sem caixa-preta. A rede só compensa com 4 ou mais entradas relevantes.
- Treino offline, com datalog de motor real e degraus de TPS (o rótulo é o erro de lambda, com atraso de transporte da sonda). O aprendizado online atual ficaria como ajuste fino por cima.

## STFT (malha curta de lambda)

Hoje (`fuel_trim.cpp`): `fuel_update_stft` é um PI com **ganhos globais** (`stft_kp_x100` = 3, `stft_ki_x1000` = 5), clamp ±15% (`stft_clamp_pct_x10` = 150), tick de ~100 ms. O atraso de transporte vem de uma tabela 3×3 bilinear (rpm × MAP, `lambda_delay_ms_table`, 0 a 2000 ms) sobre um anel de 16 amostras de lambda (`fuel_update_stft_delayed`). O integrador congela, sem decair, quando `closed_loop_allowed` ou `post_start_elapsed` bloqueiam. O STFT alimenta o LTFT por célula e o acumulador de LEARN (bake na VE só por comando manual 'Y').

A planta é quase linear (lambda ∝ 1/combustível), com atraso e filtro de sensor que variam com rpm, carga e temperatura do escape. Os ganhos fixos precisam ser conservadores para o pior atraso (baixa rotação e carga) e ficam lentos no resto.

| Opção | O que é | Veredito |
|-------|---------|----------|
| A | Rede no lugar do PI | Não. Perde estabilidade analisável, e o PI com atraso já serve a esta planta. |
| B | Ganhos (kp, ki) escalonados pelo atraso | **Primeiro, sem rede. Implementado** (desligado por padrão): ganho ∝ ref / atraso, com o atraso da tabela 3×3, limitado a 25%–200%. Falta afinar a referência com motor e WBO2. |
| C | Preditor de Smith (modelo de atraso + filtro) | Sem rede, físico e verificável. Segunda etapa, se B não bastar. |
| D | Feedforward aprendido: a rede prevê o erro de combustível a partir de MAP, rpm, TPS', CLT, IAT e etanol; o PI corrige só o resíduo | Único uso em que a rede traz algo novo, em transientes, quando o feedback chega tarde. Sobrepõe-se ao X-τ (mesma classe de erro): tratar os dois juntos. |
| E | Atraso estimado por rede, no lugar da tabela 3×3 | Ganho marginal sobre B. |
| F | Classificador de saúde da sonda (aquecimento, envelhecimento, leitura implausível) que decide `o2_valid` | Risco baixo: só abre ou fecha a malha. Liga com o `DiagnosticManager`. |

Riscos específicos:
1. **Aprendizado duplo.** O LTFT/LEARN já absorve o erro de regime. Um feedforward somaria outra correção sobre a mesma grandeza. O STFT que alimenta o LTFT tem que ser o resíduo líquido (depois do feedforward), e o feedforward deve ter média ~0 em regime e cobrir só a dinâmica. Pesos congelados (treino offline), sem aprendizado online na ECU.
2. **Degradação segura.** Saída da rede limitada por clamp, com volta imediata aos ganhos fixos atuais e feedforward 0 se a saída for implausível, se `o2_valid` cair ou se a rede não estiver carregada.
3. **Dados.** O rótulo é a resposta da sonda a degraus conhecidos de PW: exige WBO2 real, datalog rápido (SD, no Backlog) e motor. O simulador de precisão não modela a planta de lambda de forma validável.
4. **Tempo.** Sem problema: tick de ~100 ms contra inferência de ~10 µs (estimativa).

Ordem para o STFT: B, depois C se preciso, depois D junto com o X-τ, e F quando houver datalog. A e E não valem.

## Substituir PID (ETB, EWG)

Tempo não limita: as malhas rodam nos blocos de 2 a 10 ms e uma rede de ~10 µs cabe (estimativa). O limite é verificação.

- ETB tem mola de retorno, atrito e histerese; o EWG (`ewg_control`) tem planta não linear com atraso. São os casos em que o PID mais sofre. O STFT já usa PI com atraso e dificilmente ganha de uma rede.
- Não substituir o PID: o ETB é item de segurança (aceleração não comandada), o PID tem estabilidade analisável e é ajustável em campo, e a rede não tem garantia fora da faixa treinada.
- Formas que valem: (1) PID mais feedforward aprendido, com a saída da rede limitada e o PID e o limp-home mandando; (2) modelo da planta aprendido offline só para ajustar os ganhos; (3) escalonamento de ganhos por rpm, temperatura ou tensão.
- Exige dados da planta real e o gate do ETB (autocal e PID desacoplados do motor), ou seja, depois das Fases 2 e 3.

## Fora de questão

Nenhuma rede na ISR de CKP, no agendamento de INJ/IGN, no `limp_gating` nem no ETB. Decisão de segurança tem que ser determinística e auditável.

## Como seria

- Treino fora do chip (Python), exportado em int8.
- Inferência com CMSIS-NN, ou um MLP int8 em C++ puro (cerca de 100 linhas), com pesos `const` na Flash. Combina melhor com as regras do projeto (C++17, sem STL no caminho crítico, sem alocação dinâmica) do que TFLite Micro ou X-CUBE-AI.
- Testável no host (`make host-test`) com vetores fixos.
- A saída da rede sempre passa pelos clamps de `calibration` e pelo `limp_gating`.

## Riscos

- Sem motor e sem datalog não há dados para treinar nem para medir ganho. O datalog em SD está no Backlog.
- Um modelo aprendido não passa pelo `precision-test` determinístico: exigiria teste de regressão com vetores fixos e saída limitada.
- Rodar no tick de 2 ms pode estourar o orçamento. Por isso só nos blocos lentos.

## Próximo passo, se houver interesse

Depois do datalog em SD e da medida de `loop2_max`: protótipo só no host, com um MLP int8 treinado sobre dados do simulador (`test/sim/precision_main.cpp`), sem tocar em `src/`.
