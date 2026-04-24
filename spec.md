# OpenEMS-v3 — Especificação Técnica do Projeto

**Versão do documento:** 3.0.0-draft
**Data:** 2026-04-24
**Status do projeto:** Desenvolvimento ativo (STM32H562RGT6)

---

## 1. Objetivo

Definir o escopo técnico do **OpenEMS-v3**, um sistema de gerenciamento eletrônico de motor (EMS) em tempo real para motores de 4 cilindros, com foco em:

- confiabilidade de controle de combustível e ignição;
- sincronismo angular robusto (roda 60-2);
- integração de tuning via **TunerStudio por USB CDC**;
- arquitetura em camadas com baixo acoplamento;
- build determinístico para alvo embarcado STM32H562.

---

## 2. Escopo funcional

O firmware deve prover:

1. **Decodificação CKP/CAM** com estados de sincronismo (`WAIT_GAP`, `HALF_SYNC`, `FULL_SYNC`, `LOSS_OF_SYNC`).
2. **Cálculo de combustível** baseado em VE + correções térmicas e elétricas.
3. **Cálculo de ignição** por tabela de avanço com compensações e integração ao scheduler.
4. **Scheduler angular** para eventos de injeção e ignição.
5. **Aquisição e validação de sensores** com fallback seguro em caso de falha.
6. **Comunicação CAN/CAN FD** para telemetria e integração com WBO2.
7. **Comunicação TunerStudio (USB CDC only)** para leitura/escrita de páginas de calibração e tempo real.

Fora de escopo imediato:

- suíte de testes host legada (foi removida para redesign);
- suporte ativo de tuning via UART.

---

## 3. Requisitos de plataforma

### 3.1 Hardware alvo

- MCU: **STM32H562RGT6** (ARM Cortex-M33, 250 MHz)
- Memória: flash e RAM conforme limitações do alvo
- Periféricos críticos:
  - timers para captura CKP e agendamento de ignição/injeção
  - ADC para sensores analógicos
  - CAN/FDCAN
  - USB FS (CDC-ACM)

### 3.2 Requisitos de build

- Linguagem: **C++17**
- Compilador alvo: `arm-none-eabi-g++`
- Build principal: `openems-v3/Makefile` com alvo `firmware`

---

## 4. Arquitetura

Arquitetura em 4 camadas com dependência unidirecional:

1. **APP**: protocolo TunerStudio, stack CAN e diagnósticos
2. **ENGINE**: combustível, ignição, knock, auxiliares, scheduler de ciclo
3. **DRV**: CKP/CAM e sensores
4. **HAL**: abstração de periféricos STM32H562

Princípios:

- sem dependências cíclicas entre camadas;
- encapsulamento de hardware na HAL;
- snapshot atômico para dados de sensores/CKP consumidos pelo loop.

---


## 4.1 Estrutura de repositório (visão macro)

```
OpenEMS/
├── openems-v3/                    ⭐ Projeto principal (v3)
├── openems-stm32h5/               📚 Referência (v2.2)
├── src/                           📚 REFERENCE (v1.1 Teensy base)
│   └── (Original Teensy 3.5 implementation)
└── spec.md                        📋 Especificação técnica
```

## 4.2 Política para código legado de referência

Os diretórios legados (`src/` v1.1 e `openems-stm32h5/` v2.2) permanecem no repositório por decisão técnica, com escopo **somente leitura**:

1. rastreabilidade de algoritmo (comparação de comportamento entre versões),
2. suporte a auditoria de migração durante estabilização da v3,
3. investigação de regressões em bancada/campo com base histórica.

Critério para remoção definitiva: conclusão da validação funcional da v3 + congelamento de baseline de migração em artefato externo (tag/arquivo de referência).

---

## 5. Loop principal e temporização

O loop de fundo executa tarefas periódicas com watchdog:

- **2 ms**: cálculo fuel/ignition + commit de calibração do scheduler;
- **10 ms**: atualizações de malhas auxiliares;
- **20 ms**: serviço TunerStudio via USB CDC + diagnósticos runtime;
- **50 ms**: sensores lentos e telemetria intermediária;
- **100 ms**: sensores térmicos e atualizações de trim/diagnóstico;
- **500 ms**: persistência de calibração/adaptativos.

O firmware deve manter execução não-bloqueante no loop de aplicação.

---

## 6. Especificação de sincronismo CKP/CAM

### 6.1 Roda fônica

- padrão 60-2
- 58 dentes físicos, 2 ausentes
- ângulo por posição: 6°

### 6.2 Máquina de estados

- `WAIT_GAP`: aguardando gap válido
- `HALF_SYNC`: primeiro gap confirmado, validando continuidade
- `FULL_SYNC`: sincronismo completo para scheduling sequencial
- `LOSS_OF_SYNC`: perda de consistência e retorno a estratégia de recuperação

### 6.3 Requisitos

- detecção robusta de gap em variação de RPM;
- rejeição de ruído por limites mínimos de período;
- possibilidade de fast-reacquire com seed persistido, sem bypass de validação de gap.

---

## 7. Especificação de combustível

Fluxo base:

1. leitura de carga (MAP) e rotação (RPM)
2. lookup/interpolação da tabela VE
3. cálculo de `base_pw_us`
4. aplicação de correções:
   - temperatura (CLT/IAT)
   - tensão de bateria (dead-time)
   - enriquecimento transitório (AE), quando aplicável
5. geração de `final_pw_us`

Restrições:

- limitar saída por clamps de segurança;
- comportamento previsível em sensores inválidos (fallback);
- sem alocação dinâmica.

---

## 8. Especificação de ignição

Fluxo base:

1. lookup do avanço base
2. aplicação de compensações (temperatura/knock)
3. cálculo de dwell conforme tensão
4. conversão para parâmetros de scheduling angular

Requisitos:

- garantir janela física válida para dwell/spark;
- evitar over-advance por clamp configurável;
- integração consistente com estado de sincronismo.

---

## 9. Scheduler angular

Responsável por transformar parâmetros de combustível/ignição em eventos por cilindro no domínio angular.

Requisitos:

- consistência atômica do commit de calibração;
- baixa latência no arm/disparo de eventos;
- contadores diagnósticos para:
  - eventos tardios;
  - drop por fila cheia;
  - clamps de calibração.

---

## 10. Sensores e validação

Sensores principais:

- MAP, TPS, CLT, IAT, pressão de combustível, pressão de óleo, VBATT
- WBO2 recebido por CAN para malha de lambda

Requisitos:

- range-check por sensor;
- bitmask de falhas (`fault_bits`) com política de fallback;
- snapshots atômicos para consumo pelo loop.

---

## 11. Comunicação TunerStudio (USB CDC only)

### 11.1 Transporte

- transporte oficial: **USB CDC**
- UART fora do caminho principal de tuning

### 11.2 Protocolo

- comandos de identificação/comms test
- leitura/escrita de páginas de calibração
- página de tempo real com status e diagnósticos do scheduler/sync

### 11.3 Requisitos de robustez

- parser resiliente a bytes inválidos;
- validação de limites de página/offset/comprimento;
- não corromper tabelas em escrita parcial/inválida.

---

## 12. CAN/CAN FD

- recepção de frames de sensores externos (ex.: WBO2)
- transmissão de frames diagnósticos
- arquitetura preparada para filtros avançados e roteamento por prioridade

---

## 13. Persistência (NVM)

- armazenamento de calibração e mapas adaptativos
- armazenamento de seed de sincronismo (quando aplicável)
- operações de gravação com política conservadora para desgaste de flash

---

## 14. Segurança funcional (nível de firmware)

Regras mínimas:

- watchdog ativo em toda operação normal;
- fallback seguro em falha de sensores críticos;
- limitação de RPM em modo de proteção (limp);
- reset limpo e previsível em erro irrecuperável.

---

## 15. Não funcionais

- determinismo temporal para tarefas críticas;
- clareza de telemetria e diagnósticos;
- documentação atualizada com mudanças de arquitetura;
- código legível com convenções de nomenclatura por unidade física (`_x10`, `_kpa`, `_degc`, etc.).

---

## 16. Estado atual e lacunas

### Implementado

- base arquitetural em camadas;
- pipeline principal de controle de motor;
- integração inicial USB-only no caminho TunerStudio;
- instrumentação de diagnóstico runtime relevante para sync/scheduler.

### Lacunas conhecidas

- backend USB CDC ainda em evolução (infra de endpoint/IRQ a concluir);
- suíte automatizada removida temporariamente para redesenho;
- validação completa em hardware real pendente de plano de bancada formal.

---

## 17. Critérios de aceite para próxima milestone

1. build de firmware concluindo com toolchain alvo;
2. conexão estável TunerStudio via USB CDC;
3. leitura/escrita de páginas sem corrupção de dados;
4. telemetria runtime consistente (RPM, sync, status, trims);
5. validação em bancada de:
   - cranking
   - aquisição de sincronismo
   - scheduling de ignição/injeção
   - comportamento de fallback de sensores.

---

## 18. Referências internas

- `README.md` (visão do repositório)
- `openems-v3/README.md` (visão do firmware v3)
- `openems-v3/CLAUDE.md` (guia técnico de desenvolvimento)
- `DEVELOPMENT_PLAN.md` (plano de execução)
- `openems-v3/LOGIC_REVIEW_USB_ONLY.md` (revisão lógica estática)
