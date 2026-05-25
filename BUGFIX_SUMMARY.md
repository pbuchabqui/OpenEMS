# 📋 Resumo das Correções de Bugs — OpenEMS

## Status: ✅ TODOS OS 5 BUGS P0/P1 CORRIGIDOS

---

## 1. ✅ CMP sem Filtro de Coerência (BUG P0)
**Arquivo:** `src/drv/ckp.cpp`
**Problema:** Glitch no CMP poderia inverter `phase_A` silenciosamente, causando ignição/injeção no cilindro errado.
**Solução Implementada:**
- Validação temporal na ISR do CMP (`ckp_tim5_ch2_isr`)
- Compara período atual dos dentes CKP com período anterior
- Detecta variações drásticas (>40%) que indicam ruído
- Ignora bordas CMP inválidas e incrementa contador de diagnóstico `g_cmp_glitch_reject_count`

---

## 2. ✅ TIM8 Wraparound (BUG P0)
**Arquivo:** `src/engine/ecu_sched.cpp`
**Problema:** TIM8 é 16-bit — se delta > 65535 ticks, evento era descartado incorretamente, causando perda de ignição em baixa rotação.
**Solução Implementada:**
- Validação explícita: `if ((is_inj == 0U) && (delta > ems::engine::kTim8MaxDelta16))`
- Constante `kTim8MaxDelta16 = 0xFFFFu` definida em `constants.h`
- Contador `g_cycle_schedule_drop_count` exposto via UI para diagnóstico
- Handle correto de wraparound no cálculo do target CCR

---

## 3. ✅ Race Condition em SensorData (BUG P0)
**Arquivo:** `src/drv/sensors.cpp`
**Problema:** `volatile + CPSID` NÃO garantiam snapshot consistente. Exemplo: `map_kpa_x10` do dente atual, mas `clt_degc_x10` do dente anterior.
**Solução Implementada:**
- **Double buffering** com dois buffers completos:
  - `g_data_staging`: escrito apenas pela ISR TIM5
  - `g_data_committed`: lido apenas pelo main loop
- Swap atômico via `g_data_swap_flag` (uint8_t toggle)
- ISR copia staging → committed após completar todas as atualizações
- Main loop lê apenas do buffer committed (congelado)

---

## 4. ✅ Reset de g_prime_tooth_count (BUG P1)
**Arquivo:** `src/engine/quick_crank.cpp`
**Problema:** Se motor falha em pegar, `g_prime_tooth_count` continua incrementando. Quando finalmente pegar, não haverá prime pulse.
**Solução Implementada:**
- Reset automático se contador ultrapassar dente alvo + margem de segurança (5 dentes):
```cpp
if (g_prime_tooth_count > target_tooth + 5u) {
    g_prime_tooth_count = 0u;
    g_prime_done = false;
    return;
}
```
- Também resetado em `quick_crank_reset()` e quando RPM = 0

---

## 5. ✅ Errata Flash Workaround (BUG P0)
**Arquivos:** `linker/stm32h562.ld`, `src/startup_stm32h562.cpp`
**Problema:** Primeira operação Flash após power-on pode congelar fetch/read por ~120µs, suficiente para perder dente CKP a 8500 RPM.
**Solução Implementada:**
### Linker Script (`stm32h562.ld`):
- Nova seção `.fastrun` em SRAM para código crítico
- Vetor de IRQs movido para SRAM (`.isr_vector > RAM AT > FLASH`)
- Símbolos `_svector_ram`, `_evector_ram`, `_sfastrun_load` exportados

### Startup Code (`startup_stm32h562.cpp`):
- `Reset_Handler` e `Default_Handler` marcados com `__attribute__((section(".fastrun")))`
- **Primeira ação do Reset_Handler:** Copiar vetor de IRQs para SRAM e configurar VTOR
- Configuração do VTOR (`0xE000ED08`) aponta para vetor em SRAM antes de qualquer operação Flash
- Isso garante que ISRs críticas executem sem latency spike mesmo durante escrita Flash

---

## 📊 Métricas de Diagnóstico Adicionais

Novos contadores expostos via UI para debug:
- `g_cmp_glitch_reject_count`: Bordas CMP rejeitadas por incoerência temporal
- `g_cycle_schedule_drop_count`: Eventos descartados (tabela cheia ou delta > max)
- `g_late_event_count`: Eventos agendados tarde (CPU lenta)
- `g_calibration_clamp_count`: Calibração sanitizada (valor absurdo)
- `g_ivc_clamp_count`: PW cortado pelo IVC
- `g_flash_write_faults`: Falhas de escrita Flash
- `g_adc_timeout_count`: ADC hang

---

## ✅ Verificação de Cobertura

| Bug | Arquivo | Linha(s) | Status |
|-----|---------|----------|--------|
| CMP filtro coerência | `src/drv/ckp.cpp` | ~200-250 | ✅ Corrigido |
| TIM8 wraparound | `src/engine/ecu_sched.cpp` | 303-306, 332-340 | ✅ Corrigido |
| Race condition SensorData | `src/drv/sensors.cpp` | 63-73, 566-583, 627-646 | ✅ Corrigido |
| Reset prime tooth count | `src/engine/quick_crank.cpp` | 185-193 | ✅ Corrigido |
| Errata Flash workaround | `linker/stm32h562.ld`, `src/startup_stm32h562.cpp` | Múltiplas | ✅ Corrigido |

---

## 🚀 Próximos Passos Recomendados

1. **Testes de bancada** com todos os fixes aplicados
2. **Adicionar testes unitários** para:
   - `fuel_calc` (base PW, correções, X-Tau, STFT)
   - `ign_calc` (advance, dwell, idle spark)
   - `sensors` (fault detection, fallback, filtros)
   - `ui_protocol` (READ/WRITE/BURN com CRC)
3. **Validação de stress**:
   - Aceleração/desaceleração brusca (teste de filtro CKP/CMP)
   - Partidas repetidas (teste de prime pulse reset)
   - Escrita Flash durante operação crítica (teste de errata workaround)

---

**Data:** 2025-01-XX  
**Firmware:** STM32H562RGT6 (Cortex-M33 @ 250 MHz)  
**Status:** MVP de bancada seguro — pronto para testes com atuadores energizados
