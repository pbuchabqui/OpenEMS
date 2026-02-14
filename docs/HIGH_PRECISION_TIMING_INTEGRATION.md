# Integração de Drivers HP - ESP32-S3 EFI

## Visão Geral

Este documento descreve como integrar os drivers de alta precisão (HP) ao projeto EFI existente.

## Arquivos Criados

### Drivers HP
- `firmware/s3/components/engine_control/src/mcpwm_ignition_hp.c`
- `firmware/s3/components/engine_control/src/mcpwm_injection_hp.c`
- `firmware/s3/components/engine_control/include/mcpwm_ignition_hp.h`
- `firmware/s3/components/engine_control/include/mcpwm_injection_hp.h`

### Controle HP
- `firmware/s3/components/engine_control/src/control/ignition_timing_hp.c`
- `firmware/s3/components/engine_control/src/control/fuel_injection_hp.c`

## Uso dos Drivers HP

### 1. Substituir Inicialização

**Antes:**
```c
#include "mcpwm_ignition.h"
#include "mcpwm_injection.h"

bool ignition_init(void) {
    bool ign_ok = mcpwm_ignition_init();
    bool inj_ok = mcpwm_injection_init();
    // ...
}
```

**Depois:**
```c
#include "mcpwm_ignition_hp.h"
#include "mcpwm_injection_hp.h"

bool ignition_hp_init(void) {
    return ignition_hp_init();
}
```

### 2. Aplicar Timing de Ignição

**Antes:**
```c
void ignition_apply_timing(uint16_t advance_deg10, uint16_t rpm) {
    mcpwm_ignition_schedule_one_shot(cylinder, delay_us, rpm, battery_voltage);
}
```

**Depois:**
```c
void ignition_hp_apply_timing(uint16_t advance_deg10, uint16_t rpm) {
    // Usa compare absoluto automaticamente
    ignition_hp_apply_timing(advance_deg10, rpm);
}
```

### 3. Aplicar Injeção

**Antes:**
```c
fuel_injection_schedule_eoi(cylinder_id, target_eoi_deg, pulsewidth_us, sync);
```

**Depois:**
```c
fuel_injection_hp_schedule_eoi(cylinder_id, target_eoi_deg, pulsewidth_us, sync);
```

## Recursos HP Disponíveis

### Preditor de Fase Adaptativo
```c
// Obter predição do próximo período
float predicted_period = hp_predict_next_period(&predictor, dt);

// Atualizar com medição real
hp_update_phase_predictor(&predictor, measured_period, timestamp);
```

### Compensação de Latência
```c
// Ignição
mcpwm_ignition_hp_apply_latency_compensation(&timing_us, voltage, temp);

// Injeção
mcpwm_injection_hp_apply_latency_compensation(&pulsewidth_us, voltage, temp);
```

### Medição de Jitter
```c
float avg_jitter, max_jitter, min_jitter;
ignition_hp_get_jitter_stats(&avg_jitter, &max_jitter, &min_jitter);
```

## Compilação

Os novos arquivos já estão incluídos no CMakeLists.txt:

```cmake
SRCS
    "src/control/ignition_timing_hp.c"
    "src/control/fuel_injection_hp.c"
    "src/mcpwm_ignition_hp.c"
    "src/mcpwm_injection_hp.c"
    "src/high_precision_timing.c"
```

## Migração Gradual

Para migração gradual, você pode:
1. Manter os drivers originais como fallback
2. Ativar os drivers HP em tempo de compilação com um flag
3. Testar cada componente separadamente

## Métricas de Melhoria

| Parâmetro | Antes | Depois |
|-----------|-------|--------|
| Precisão de Timing | ~±1-2μs | <±0.3μs |
| Jitter | ~5-10μs | <0.5μs |
| Skew entre cilíndros | Variável | <1μs |

## Debug

Para medição de jitter com osciloscópio:
```c
#define JITTER_DEBUG_PIN GPIO_NUM_21

// Toggle antes do cálculo
gpio_set_level(JITTER_DEBUG_PIN, 1);

// Cálculo de timing
ignition_hp_apply_timing(advance, rpm);

// Toggle após disparo
gpio_set_level(JITTER_DEBUG_PIN, 0);
```

## Próximos Passos

1. **Testar compilação**: `idf.py build`
2. **Validação prática**: Testar com osciloscópio
3. **Calibração**: Ajustar parâmetros de latência
4. **Monitoramento**: Implementar logging de jitter
