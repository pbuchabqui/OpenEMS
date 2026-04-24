# OpenEMS-v3 — Revisão Profunda do Pipeline de Funcionamento do Motor

**Data:** 2026-04-24  
**Escopo:** cranking, sync, cálculo de combustível (`req fuel`/PW), ignição (ângulo), scheduling crítico, STFT/LTFT.  
**Método:** revisão estática de código (sem suíte automatizada ativa).

---

## 1) Pipeline ponta-a-ponta (estado atual)

## Etapa A — Captura CKP/CAM e sincronismo

1. ISR CKP (`ckp_ftm3_ch0_isr`) captura período por dente usando registrador de captura de hardware.
2. Classifica evento como dente normal/gap/ruído por média histórica + janela de tolerância.
3. Avança máquina de estados de sync (`WAIT_GAP`, `HALF_SYNC`, `FULL_SYNC`, `LOSS_OF_SYNC`).
4. Em cada dente, dispara hooks para sensores/scheduling/prime pulse.

**Leitura:** arquitetura robusta para tempo real, com proteções explícitas de ruído, gap prematuro e perda de sync.

## Etapa B — Cranking e enriquecimento de partida

1. `quick_crank_update` detecta condição de cranking por RPM + sync.
2. Durante cranking aplica multiplicador de combustível por CLT e mínimo de PW.
3. Hook `prime_on_tooth` agenda prime pulse no 5º dente de cranking.
4. Pós-partida aplica afterstart com decaimento por duração dependente de CLT.

**Leitura:** estratégia de partida está estruturada e contém limites de PW.

## Etapa C — Cálculo de combustível (base + correções)

Fluxo esperado no projeto:

1. VE por lookup (RPM × carga).
2. `calc_base_pw_us(req_fuel, ve, map, map_ref)`.
3. Correções CLT/IAT + dead-time de bateria.
4. Aplicação de enriquecimento de partida/transientes.
5. Conversão para ticks do scheduler.

**Leitura:** funções de fuel têm clamps e interpolação defensiva; STFT/LTFT integrados no mesmo módulo.

## Etapa D — Cálculo de ignição

Fluxo esperado no projeto:

1. lookup de avanço base;
2. compensações térmicas/knock;
3. clamp de avanço;
4. cálculo de dwell e ângulos absolutos para scheduler.

**Leitura:** módulo de ignição possui funções e tabelas consistentes, incluindo proteção para overflow de dwell em ângulo.

## Etapa E — Scheduling angular (crítico)

1. `cycle_sched_update` publica parâmetros por cilindro com protocolo `valid=false -> escreve -> valid=true`.
2. `ecu_sched_on_tooth_hook` no caminho do dente decide e arma eventos de injeção/ignição.
3. Limites e diagnósticos: late events, drop count, clamp count.

**Leitura:** arquitetura correta para reduzir jitter entre cálculo e disparo; ainda há limitações documentadas de temporização para pulsos longos.

## Etapa F — STFT/LTFT

1. `fuel_update_stft` habilita malha fechada só em condições válidas (CLT, O2, sem AE/rev-cut).
2. Termo P+I com saturação;
3. STFT atualizado;
4. LTFT por célula (MAP/RPM) com integração lenta.

**Leitura:** lógica de controle está bem definida; persistência LTFT ainda depende de backend de armazenamento efetivo.

---

## 2) Achados críticos (inconsistências de integração)

### CRIT-1: `main_stm32.cpp` usa campos de sensores que não batem com `SensorData` atual

Exemplos observados no loop principal:

- uso de `sensors.map_kpa`, `sensors.clt_x10`, `sensors.iat_x10`, `sensors.o2_mv`, `sensors.o2_valid`;
- o struct atual expõe sufixos explícitos como `map_kpa_x10`, `clt_degc_x10`, etc.

**Impacto:** risco alto de quebra de build e/ou uso incorreto de unidades físicas.

### CRIT-2: chamada de `ecu_sched_commit_calibration(...)` no `main_stm32.cpp` não está alinhada ao contrato do módulo

A chamada no loop principal está com assinatura/ordem de argumentos incompatível com o contrato atual do scheduler angular.

**Impacto:** risco alto para build e, principalmente, para comportamento do scheduling crítico.

### CRIT-3: caminho de cálculo de ignição no `main_stm32.cpp` referencia API diferente da API presente em `ign_calc`

`main_stm32.cpp` usa função de alto nível que não corresponde às entradas públicas observadas em `ign_calc` (que fornece primitivas como lookup/clamp/combinação).

**Impacto:** integração incompleta entre camada de orquestração (main loop) e módulo de ignição.

### CRIT-4: STFT no loop principal usa sinais O2 que não batem com o modelo atual orientado a WBO2 via CAN

Há divergência entre o consumo no loop principal e o desenho atual do modelo de sensores/feedback de lambda.

**Impacto:** risco de malha fechada operar com sinais errados ou indisponíveis.

---

## 3) Riscos funcionais por etapa

- **Cranking/sync:** baixo a médio (núcleo CKP/quick_crank parece consistente).
- **Fuel base:** médio (módulo sólido, mas integração de unidades no main está inconsistente).
- **Ignition angle:** alto (orquestração no main não alinhada ao contrato do módulo).
- **Scheduling:** alto (assinatura incompatível em ponto crítico do pipeline).
- **STFT/LTFT:** médio a alto (núcleo existe, integração de fonte lambda precisa fechamento).

---

## 4) Plano de correção recomendado (ordem)

1. **Fechar contrato de dados no loop principal**
   - alinhar campos de `SensorData` e unidades (`_x10`, `_degc_x10`, etc.);
   - remover referências a campos inexistentes.

2. **Fechar contrato de ignição/scheduler**
   - padronizar sequência: cálculo avanço/dwell/PW -> commit no scheduler com assinatura única;
   - garantir consistência de ordem/unidades dos argumentos.

3. **Fechar contrato de malha lambda**
   - decidir fonte única de medição (WBO2 CAN) e adaptar `fuel_update_stft` no main.

4. **Concluir validação crítica em bancada**
   - cranking + aquisição de sync;
   - timing de ignição/injeção sob aceleração e desaceleração;
   - transientes com AE e closed-loop.

---

## 5) Conclusão

O **núcleo algorítmico** (CKP/sync, quick_crank, fuel, ign, scheduler angular e STFT/LTFT) está bem estruturado nos módulos dedicados, mas há **gaps de integração no `main_stm32.cpp`** que afetam diretamente o pipeline crítico do motor. A prioridade deve ser fechar contratos de API/unidade entre main loop e módulos antes de qualquer validação final de dirigibilidade.

