# Sistema de Alta Precisão de Timing - ESP32-S3 EFI

## Visão Geral

Este documento descreve as melhorias implementadas para alcançar máxima precisão de timing de ignição e injeção no ESP32-S3.

## Arquivos Criados

### Headers
- `firmware/s3/components/engine_control/include/high_precision_timing.h` - Sistema base de alta precisão
- `firmware/s3/components/engine_control/include/mcpwm_ignition_hp.h` - Driver de ignição otimizado
- `firmware/s3/components/engine_control/include/mcpwm_injection_hp.h` - Driver de injeção otimizado

### Implementação
- `firmware/s3/components/engine_control/src/high_precision_timing.c` - Implementação do sistema base
- `firmware/s3/components/engine_control/src/mcpwm_ignition_hp.c` - Driver de ignição HP
- `firmware/s3/components/engine_control/src/mcpwm_injection_hp.c` - Driver de injeção HP

## Melhorias Implementadas

### 1. Timer Contínuo com Compare Absoluto

**Antes:**
```c
// Timer reiniciado a cada evento
mcpwm_timer_set_period(ch->timer, period);
mcpwm_timer_start_stop(ch->timer, MCPWM_TIMER_START_STOP_FULL);
```

**Depois:**
```c
// Timer contínuo, sem reinício
mcpwm_timer_start_stop(ch->timer, MCPWM_TIMER_START_NO_STOP);
mcpwm_comparator_set_compare_value(ch->cmp_spark, absolute_spark_ticks);
```

**Benefício:** Elimina jitter de rearmar timer por evento.

### 2. Sistema de Contagem de Ciclos (CCOUNT)

```c
// Leitura direta de ciclos da CPU
uint32_t cycles = hp_get_cycle_count();

// Conversão microsegundos <-> ciclos
uint32_t cycles = hp_us_to_cycles(100.0f);  // 100us -> ciclos
float us = hp_cycles_to_us(cycles);           // ciclos -> us
```

### 3. Preditor de Fase Adaptativo

```c
phase_predictor_t predictor;

// Inicialização
hp_init_phase_predictor(&predictor, 10000.0f);  // 10ms inicial

// Atualização com medição
hp_update_phase_predictor(&predictor, measured_period, timestamp);

// Predição do próximo período
float predicted_period = hp_predict_next_period(&predictor, dt);
```

### 4. Compensação de Latência Física

```c
hardware_latency_comp_t latency;

// Inicialização
hp_init_hardware_latency(&latency);

// Cálculo de latência compensada
float coil_latency = hp_get_coil_latency(&latency, battery_voltage, temperature);
float injector_latency = hp_get_injector_latency(&latency, battery_voltage, temperature);
```

### 5. Medição de Jitter

```c
jitter_measurer_t jitter;

// Inicialização
hp_init_jitter_measurer(&jitter);

// Registro de medição
hp_record_jitter(&jitter, target_cycles, actual_cycles);

// Obtenção de estatísticas
float avg_jitter, max_jitter, min_jitter;
hp_get_jitter_stats(&jitter, &avg_jitter, &max_jitter, &min_jitter);
```

## Exemplo de Integração

```c
#include "mcpwm_ignition_hp.h"
#include "mcpwm_injection_hp.h"
#include "high_precision_timing.h"
#include "driver/mcpwm_timer.h"

void ignition_task(void *arg) {
    uint32_t counter;
    
    while (1) {
        // Obter contador atual do timer
        mcpwm_timer_get_count(timer_handle, &counter);
        
        // Calcular timing absoluto usando predição
        float predicted_period = hp_predict_next_period(&predictor, 0);
        uint32_t spark_ticks = calculate_spark_ticks(rpm, advance);
        uint32_t target = counter + (uint32_t)predicted_period;
        
        // Aplicar compensação de latência
        float compensated_timing = spark_ticks;
        mcpwm_ignition_hp_apply_latency_compensation(&compensated_timing, 
                                                      battery_voltage, temperature);
        
        // Agendar com compare absoluto
        mcpwm_ignition_hp_schedule_one_shot_absolute(
            cylinder_id,           // 1-4
            target,               // Target absoluto
            rpm,                  // RPM atual
            battery_voltage,      // Tensão
            counter               // Contador atual
        );
        
        // Aguardar próximo ciclo
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    // Inicializar drivers de alta precisão
    mcpwm_ignition_hp_init();
    mcpwm_injection_hp_init();
    
    // Criar tarefa de timing crítico no Core 0
    hp_create_critical_task(
        ignition_task,
        "ignition_hp",
        4096,
        NULL,
        0,  // Prioridade máxima
        NULL,
        0   // Core 0
    );
}
```

## Métricas de Melhoria Esperadas

| Parâmetro | Antes | Depois | Melhoria |
|-----------|-------|--------|----------|
| Precisão de Timing | ~±1-2μs | <±0.3μs | 5-7x |
| Jitter | ~5-10μs | <0.5μs | 10-20x |
| Skew entre cilíndros | Variável | <1μs | Consistência |

## Configuração de GPIO de Debug

Para medição de jitter no osciloscópio:

```c
#define JITTER_DEBUG_PIN GPIO_NUM_21

// Toggle no início do cálculo
gpio_set_level(JITTER_DEBUG_PIN, 1);

// C
calculateálculo de timing_timing();

// Toggle no disparo real
gpio_set_level(JITTER_DEBUG_PIN, 0);
```

## Compilação

Adicione ao CMakeLists.txt:

```cmake
set(COMPONENT_SRCS
    src/high_precision_timing.c
    src/mcpwm_ignition_hp.c
    src/mcpwm_injection_hp.c
)

set(COMPONENT_ADD_INCLUDEDIRS
    include
)
```

## Próximos Passos

1. **Validação**: Testar com osciloscópio para medir jitter real
2. **Calibração**: Ajustar parâmetros de latência de hardware
3. **Otimização**: Refinar filtros preditivos com dados reais
4. **Integração**: Substituir drivers originais pelos novos HP

## Referência

Para mais detalhes, consulte:
- `docs/development/PHASE_2_3_4_SUMMARY.md`
- `docs/development/IMPROVEMENTS_SUMMARY.md`
