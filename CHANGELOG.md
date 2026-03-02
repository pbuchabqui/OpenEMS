# OpenEMS - Changelog de Correções

## 2026-03-02 - Correções Críticas de Timing e Código

### 🐛 Problemas Resolvidos

#### 1. Variáveis Globais Ausentes no Sistema Unificado de Timing
**Problema**: Falta de declarações externas para variáveis globais essenciais no `src/main.cpp`, causando erros de linkagem.
**Solução**: Adicionadas declarações externas para:
- `g_overflow_count` - Extensão de timestamp de 32 bits
- `g_ticks_per_rev` - Ticks por rotação do motor
- `g_advance_deg` - Avanço de ignição em graus
- `g_dwell_ticks` - Tempo de dwell em ticks

#### 2. Conflitos de Prescaler FTM
**Problema**: Conflito de prescaler entre FTM0 (timing) e FTM1/FTM2 (PWM).
**Solução**: Sistema unificado de timing já implementado em `ecu_sched.c` com PS=128 para FTM0.

#### 3. Casts C-style em Código C++
**Problema**: Uso de casts C-style em código C++, violando boas práticas.
**Solução**: Substituídos por `static_cast` apropriado:
- Função `nvic_enable()` - casts para manipulação de registradores NVIC
- Funções de inicialização FTM - removidos casts desnecessários

#### 4. Declaração Incompleta de Forward Declaration
**Problema**: Uso de forward declaration incompleta para `CkpSnapshot`.
**Solução**: Incluído header `drv/ckp.h` para acesso completo à estrutura.

### 🔧 Alterações Técnicas

#### src/main.cpp
```cpp
// Adicionadas declarações externas para sistema unificado de timing
extern volatile uint32_t g_overflow_count;
extern volatile uint32_t g_ticks_per_rev;
extern volatile uint32_t g_advance_deg;
extern volatile uint32_t g_dwell_ticks;

// Integração do sistema unificado de timing
const uint32_t current_timestamp = (g_overflow_count << 16) | ems::hal::ftm0_count();
ems::engine::Calculate_Sequential_Cycle(current_timestamp);
```

#### src/hal/ftm.cpp
```cpp
// Substituído C-style casts por static_cast
*ipr = (*ipr & ~(0xFFu << shift)) | (static_cast<uint32_t>(priority << 4u) << shift);
NVIC_ISER(irq / 32u) = static_cast<uint32_t>(1u << (irq % 32u));

// Removidos casts desnecessários nas funções de inicialização
SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;  // em vez de static_cast<uint32_t>(SIM_SCGC3_FTM3_MASK)
```

### 🎯 Benefícios

1. **Timing Preciso**: Sistema unificado de 32 bits para timing de motor
2. **Compensação de Aceleração**: Gap detection para rodas fônicas 60-2
3. **Código Limpo**: Eliminação de casts C-style, melhorando manutenção
4. **Compilação Limpa**: Resolução de todos os erros de linkagem

### 🧪 Testes

- ✅ Compilação bem-sucedida de `src/main.cpp` e `src/hal/ftm.cpp`
- ✅ Ausência de erros críticos de linkagem
- ✅ Apenas warnings esperados de casts de ponteiro (normais em embedded)

### 📋 Próximos Passos

1. **Testes de Hardware**: Validar timing real no microcontrolador
2. **Testes de Motor**: Verificar sincronismo e disparo de ignição
3. **Documentação**: Atualizar documentação do sistema de timing

### 🤝 Contribuidores

- Equipe OpenEMS
- Análise e correção de código: Cline (AI Assistant)

### 📄 Licença

Este projeto continua sob a licença MIT conforme especificado no repositório.