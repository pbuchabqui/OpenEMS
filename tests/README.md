# OpenEMS Test Framework

Bem-vindo ao framework de testes automatizados do OpenEMS. Esta estrutura abrangente permite testes unitários, de integração e performance para o sistema de gerenciamento de motor.

## Estrutura de Diretórios

```
tests/
├── unit/                     # Testes unitários isolados
│   ├── sensors/              # Testes de sensores
│   │   └── test_trigger_60_2.c
│   ├── scheduler/            # Testes do agendador
│   │   └── test_event_scheduler.c
│   ├── drivers/              # Testes de drivers
│   │   └── test_mcpwm_injection.c
│   ├── control/              # Testes de controle
│   ├── utils/                # Testes de utilitários
│   └── comms/                # Testes de comunicação
├── integration/              # Testes de integração
│   └── test_core_communication.c
├── performance/              # Testes de performance
├── mocks/                   # Mocks para hardware e HAL
│   ├── mock_hal_timer.h/c
│   ├── mock_hal_gpio.h/c
│   └── mock_esp_idf.h/c
├── fixtures/                # Dados de teste
│   └── engine_test_data.h/c
├── scripts/                 # Scripts de automação
│   ├── run_tests.sh
│   └── performance_report.py
├── CMakeLists.txt          # Configuração de build
└── README.md              # Este arquivo
```

## Como Usar

### Pré-requisitos

1. **ESP-IDF v5.x** instalado e configurado
2. **Unity framework** (já configurado no ESP-IDF)
3. **Python 3** para relatórios de performance
4. **gcov/lcov** para análise de coverage (opcional)

### Configuração do Ambiente

```bash
# Exportar ESP-IDF
. $IDF_PATH/export.sh

# Navegar para o projeto
cd /home/pedro/OpenEMS

# Configurar target
idf.py set-target esp32s3
```

### Executar Testes

#### Executar Todos os Testes
```bash
./tests/scripts/run_tests.sh
```

#### Executar Apenas Testes Unitários
```bash
./tests/scripts/run_tests.sh -u
```

#### Executar Testes com Coverage
```bash
./tests/scripts/run_tests.sh --coverage
```

#### Executar Testes de Performance
```bash
./tests/scripts/run_tests.sh -p
```

#### Limpar e Rebuild
```bash
./tests/scripts/run_tests.sh -c
```

## Tipos de Testes

### 1. Testes Unitários

Testes isolados para módulos individuais:

- **trigger_60_2**: Decodificação de roda faltante 60-2
- **event_scheduler**: Agendamento baseado em ângulo
- **mcpwm_injection**: Driver de injeção de alta precisão

### 2. Testes de Integração

Testes de comunicação entre módulos:

- **core_communication**: Comunicação Core 0 ↔ Core 1 via atomic_buffer

### 3. Testes de Performance

Testes de timing crítico e performance:

- **timing_precision**: Precisão sub-microssegundo
- **high_rpm_performance**: Performance em alta rotação
- **memory_usage**: Uso de memória e recursos

## Sistema de Mocks

O framework inclui mocks completos para:

### HAL Timer
```c
// Configurar mock
mock_hal_timer_reset();
mock_hal_timer_set_auto_increment(true, 1000); // 1ms increments

// Verificar chamadas
MOCK_HAL_GPIO_ASSERT_CALL_COUNT("HAL_Time_us", expected_count);
```

### HAL GPIO
```c
// Configurar mock
mock_hal_gpio_reset();
mock_hal_gpio_set_capture_mode(true);

// Verificar estados
MOCK_HAL_GPIO_ASSERT_STATE(HAL_PIN_INJ_1, true);
```

### ESP-IDF Components
```c
// Mock para FreeRTOS, NVS, CAN, etc.
// Implementado em mock_esp_idf.h/c
```

## Dados de Teste (Fixtures)

Dados pré-configurados para cenários comuns:

```c
// Configurações de motor
extern const engine_config_test_t ENGINE_CONFIG_SMALL;
extern const engine_config_test_t ENGINE_CONFIG_MEDIUM;
extern const engine_config_test_t ENGINE_CONFIG_LARGE;

// Dados de sensores
extern const sensor_data_test_t SENSOR_DATA_IDLE;
extern const sensor_data_test_t SENSOR_DATA_CRUISE;
extern const sensor_data_test_t SENSOR_DATA_WOT;

// Dados de trigger wheel
extern const trigger_wheel_test_t TRIGGER_60_2_1000_RPM;
extern const trigger_wheel_test_t TRIGGER_60_2_3000_RPM;
extern const trigger_wheel_test_t TRIGGER_60_2_6000_RPM;
```

## Escrevendo Novos Testes

### Estrutura Básica

```c
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "unity.h"
#include "esp_err.h"
#include "modulo_a_ser_testado.h"
#include "mocks/mock_hal_timer.h"
#include "mocks/mock_hal_gpio.h"
#include "fixtures/engine_test_data.h"

// Estado de teste
static tipo_config_t test_config;
static tipo_state_t test_state;

void setUp(void) {
    // Reset mocks e estado
    mock_hal_timer_reset();
    mock_hal_gpio_reset();
    memset(&test_config, 0, sizeof(test_config));
    memset(&test_state, 0, sizeof(test_state));
}

void tearDown(void) {
    // Cleanup
}

void test_nome_do_teste(void) {
    // Arrange
    esp_err_t ret = modulo_init(&test_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Act
    ret = modulo_operacao();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // Assert
    TEST_ASSERT_TRUE(condicao_esperada);
}

// Test runner
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_nome_do_teste);
    return UNITY_END();
}
```

### Boas Práticas

1. **Setup/Teardown**: Sempre resete mocks e estado
2. **Nomenclatura**: Use `test_nomeDoModulo_descricaoDoTeste`
3. **Asserts**: Use asserts específicos (FLOAT, UINT32, etc.)
4. **Performance**: Meça timing crítico com tolerâncias
5. **Coverage**: Teste caminhos felizes e de erro

## Relatórios

### Relatório de Testes
Após execução, relatórios são gerados em `test_results/`:

```
test_results/
├── unit_tests.log          # Log dos testes unitários
├── integration_tests.log   # Log dos testes de integração
├── performance_tests.log   # Log dos testes de performance
├── test_summary.txt       # Resumo completo
├── coverage/              # Relatórios de coverage
│   └── html/index.html
└── performance/           # Relatórios de performance
    └── performance_report.html
```

### Coverage Analysis
```bash
# Gerar coverage
./tests/scripts/run_tests.sh --coverage

# Visualizar
firefox test_results/coverage/html/index.html
```

### Performance Report
```bash
# Gerar relatório de performance
./tests/scripts/run_tests.sh -p

# Visualizar
firefox test_results/performance/performance_report.html
```

## Métricas e KPIs

### Critérios de Sucesso
- **Coverage**: >90% em módulos críticos, >70% geral
- **Performance**: <1µs jitter em timing crítico
- **Confiabilidade**: <0.1% false positives
- **Velocidade**: <5min execução completa

### Métricas Monitoradas
- Tempo de execução por teste
- Uso de memória
- Precisão de timing
- Taxa de sucesso
- Coverage de código

## Integração CI/CD

O framework está preparado para integração contínua:

```yaml
# Exemplo GitHub Actions
- name: Run OpenEMS Tests
  run: |
    . $IDF_PATH/export.sh
    ./tests/scripts/run_tests.sh --coverage
```

## Troubleshooting

### Problemas Comuns

1. **Unity não encontrado**: Verifique ESP-IDF configuration
2. **Link errors**: Confirme CMakeLists.txt includes
3. **Mock não funciona**: Verifique includes e macros
4. **Timing falhando**: Ajuste tolerâncias para hardware real

### Debug de Testes

```bash
# Verbose output
idf.py test --test-filter="test_nome" --log-level=debug

# Apenas um teste
idf.py test --test-filter="test_nome_especifico"

# Verificar build
idf.py build --verbose
```

## Próximos Passos

1. **Expandir testes**: Adicionar mais módulos Core 1
2. **Hardware-in-the-loop**: Testes com hardware real
3. **Property-based testing**: Testes com dados aleatórios
4. **Fuzz testing**: Testes de robustez
5. **Regression testing**: Testes automatizados de regressão

## Contribuição

Para contribuir com novos testes:

1. Siga as boas práticas documentadas
2. Mantenha coverage >90% em código crítico
3. Adicione fixtures para dados de teste
4. Documente testes complexos
5. Verifique performance em timing crítico

---

**OpenEMS Test Framework** - Testes robustos para controle de motor de alta performance
