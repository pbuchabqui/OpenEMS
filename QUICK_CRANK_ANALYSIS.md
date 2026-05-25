# Análise Técnica: Quick Crank + Prime Pulse + Runtime Seed (Start-Stop)

## 📋 Resumo Executivo

O OpenEMS implementa uma estratégia sofisticada de partida rápida que combina:
1. **Prime Pulse** - Injeção de combustível pré-sincronismo para encher o coletor
2. **Quick Crank** - Enriquecimento durante cranking + afterstart decay
3. **Runtime Seed** - Memorização da posição angular para re-sincronização instantânea (Start-Stop)

Esta análise documenta o fluxo completo e identifica oportunidades de melhoria.

---

## 1. 🔄 FLUXO COMPLETO DE PARTIDA

### 1.1 Sequência Temporal Real

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ TEMPO REAL (ms)        │ ESTADO            │ AÇÃO                           │
├─────────────────────────────────────────────────────────────────────────────┤
│ t = 0                  │ Power-on          │ Boot sequence inicia           │
│ t = 0-5                │ INIT              │ Carrega runtime seed da Flash  │
│                        │                   │ → Se válida: ckp_seed_arm()    │
│ t = 5-5000             │ WAIT_SYNC         │ Aguarda FULL_SYNC (timeout 5s) │
│                        │                   │ Mas NÃO bloqueia — prossegue   │
├─────────────────────────────────────────────────────────────────────────────┤
│ t = ~100               │ CRANKING INICIAL  │ Motor começa a girar (starter) │
│                        │ SyncState: WAIT   │ CKP detecta primeiros dentes   │
│                        │                   │ g_prime_tooth_count = 0        │
│                                                                             
│ t = ~150               │ PRIME PULSE       │ Dente #3 detectado (exemplo)   │
│                        │                   │ → prime_on_tooth() na ISR      │
│                        │                   │ → g_prime_pending = true       │
│                        │                   │ → Loop 2ms consume e dispara   │
│                        │                   │ ⚠️ SEM FULL_SYNC necessário!   │
│                                                                             
│ t = ~200-400           │ CONTINUA CRANKING │ Mais injeções normais          │
│                        │                   │ Fuel mult = 2.0x-3.0x (CLT)    │
│                        │                   │ Spark = crank_spark_deg (fixo) │
│                                                                             
│ t = ~400-600           │ FIRST FIRE        │ Primeiro cilindro pega         │
│                        │ RPM > 500         │ Transição cranking → running   │
│                                                                             
│ t = ~600+              │ AFTERSTART        │ Fuel mult decai 1.35x → 1.0x   │
│                        │                   │ Duração: 500-2400ms (CLT)      │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Interação Prime Pulse ↔ Quick Crank

**Código-chave (`quick_crank.cpp`):**

```cpp
// Na ISR do CKP (prio 1) - NÃO espera FULL_SYNC
void prime_on_tooth(const CkpSnapshot& snap) noexcept {
    if (g_prime_done) { return; }
    
    // Só conta enquanto RPM indica cranking (<500 RPM típico)
    if (snap.rpm_x10 == 0u || snap.rpm_x10 >= 5000u) { return; }
    
    ++g_prime_tooth_count;
    
    // FIX P1: Reset se ultrapassar dente alvo sem disparar
    const uint8_t target_tooth = sanitized_prime_tooth();  // ex: 3
    if (g_prime_tooth_count > target_tooth + 5u) {
        g_prime_tooth_count = 0u;  // Reseta para próxima tentativa
        g_prime_done = false;
        return;
    }
    
    if (g_prime_tooth_count < target_tooth) { return; }
    
    // Calcula PW com enriquecimento por CLT
    const uint32_t mult = interp_u16(kCrankFuelMult, ..., g_prime_clt_x10);
    uint32_t pw = ((kDefaultReqFuelUs * mult) >> 8u) + g_prime_dead_time_us;
    pw = clamp(pw, 0, crank_prime_max_pw_us);  // ex: 30ms max
    
    g_prime_pw_us = pw;
    g_prime_pending = true;   // Sinaliza loop principal
    g_prime_done = true;      // Evita re-disparo
}

// No loop de 2ms - consome e dispara
if (sched_sync && !rev_cut) {
    const uint32_t prime_pw = quick_crank_consume_prime();
    if (prime_pw > 0u) {
        fuel_inject_prime(prime_pw);  // Dispara injector
    }
}
```

**Pontos Críticos:**

| Característica | Implementação | Justificativa |
|----------------|---------------|---------------|
| **Sem FULL_SYNC** | ✅ Opera em WAIT_GAP/HALF_SYNC | Prime pulse é apenas combustível, sem ignição associada |
| **Baseado em CKP** | ✅ Apenas contagem de dentes | Fase absoluta (CMP) não é necessária para encher coletor |
| **Reset automático** | ✅ Se `tooth_count > target + 5` | Previne falha em tentativas subsequentes de partida |
| **PW limitada** | ✅ Clamp em 30ms máximo | Protege injetores e evita flooding excessivo |
| **Contexto CLT** | ✅ Atualizado pelo loop 2ms | Enriquecimento correto por temperatura |

---

## 2. 🧠 RUNTIME SEED — ESTRATÉGIA START-STOP

### 2.1 Conceito Arquitetural

O **Runtime Seed** memoriza a última posição angular conhecida (tooth_index + phase_A) antes do desligamento do motor. Isso permite:

1. **Re-sincronização instantânea** no próximo start (evita esperar 2 gaps = 2 voltas)
2. **Full Sync imediato** se o motor parar na mesma posição relativa
3. **Fast reacquire** durante Start-Stop (trânsito, semáforos)

### 2.2 Fluxo de Salvamento (Engine Stop)

**Código (`main_stm32.cpp`, linhas 610-639):**

```cpp
// Monitora RPM no loop de 2ms
const uint32_t rpm = snap.rpm_x10;
if (rpm > 0u) {
    g_engine_was_running = true;
    g_zero_rpm_since_ms  = 0u;
    g_runtime_seed_saved_for_stop = false;
} else {
    // Detecta transição running → stopped
    if (g_engine_was_running && g_zero_rpm_since_ms == 0u) {
        g_zero_rpm_since_ms = now;  // Marca instante de RPM zero
    }
    
    // Salva seed após delay de estabilização (100ms)
    if (g_engine_was_running && 
        !g_runtime_seed_saved_for_stop &&
        g_zero_rpm_since_ms != 0u &&
        elapsed(now, g_zero_rpm_since_ms, kRuntimeSeedSaveDelayMs)) {
        
        // Usa snapshot do último GAP SYNC (tooth_index = 0)
        const auto seed_snap = g_last_gap_sync_snapshot;
        
        ems::hal::RuntimeSyncSeed seed = {};
        seed.flags = RUNTIME_SYNC_SEED_FLAG_VALID |
                     RUNTIME_SYNC_SEED_FLAG_FULL_SYNC |
                     (seed_snap.phase_A ? RUNTIME_SYNC_SEED_FLAG_PHASE_A : 0u);
        seed.tooth_index = seed_snap.tooth_index;  // = 0 (gap-aligned)
        seed.decoder_tag = RUNTIME_SYNC_SEED_DECODER_TAG_60_2;
        
        if (!ems::hal::nvm_save_runtime_seed(&seed)) {
            ++g_flash_write_faults;  // Diagnóstico
        }
        g_runtime_seed_saved_for_stop = true;
    }
}
```

**Características do Salvamento:**

| Parâmetro | Valor | Justificativa |
|-----------|-------|---------------|
| **Delay para salvar** | 100ms após RPM=0 | Aguarda estabilização mecânica (fim de oscilações) |
| **Posição salva** | tooth_index do último gap | Gap-aligned = referência angular absoluta |
| **Phase_A incluída** | Sim (flag separada) | Necessário para sincronismo completo (CKP+CMP) |
| **Flash Bank2** | Setor 0 (offset 512B) | Mesma região de LTFT/Knock maps |
| **Write-once** | Apenas 1 vez por shutdown | Protege endurance da Flash (1k cycles) |

### 2.3 Fluxo de Recuperação (Boot)

**Código (`main_stm32.cpp`, linhas 331-344):**

```cpp
// Durante openems_init() — antes de habilitar IRQs
{
    ems::hal::RuntimeSyncSeed seed = {};
    if (ems::hal::nvm_load_runtime_seed(&seed) &&
        ems::hal::runtime_seed_fast_reacquire_compatible_60_2(seed)) {
        
        const bool phase_a = ((seed.flags & RUNTIME_SYNC_SEED_FLAG_PHASE_A) != 0u);
        ems::drv::ckp_seed_arm(phase_a);  // Prepara máquina de estados
        
        // Janela de 2s para validar seed com primeiro gap
        g_runtime_seed_arm_window_active = true;
        g_runtime_seed_arm_window_start_ms = millis();
    }
    
    // Limpa seed da Flash (consumida)
    if (!ems::hal::nvm_clear_runtime_seed()) {
        ++g_flash_write_faults;
    }
}
```

**Validação da Seed (`runtime_seed.h`):**

```cpp
// Compatibilidade para boot (carregamento inicial)
inline bool runtime_seed_boot_compatible_60_2(const RuntimeSyncSeed& seed) noexcept {
    if ((seed.flags & RUNTIME_SYNC_SEED_FLAG_FULL_SYNC) == 0u) {
        return false;  // Precisa ter Full Sync válido
    }
    if (seed.tooth_index > RUNTIME_SYNC_SEED_MAX_TOOTH_INDEX_60_2) {
        return false;  // tooth_index deve ser <= 57 (60-2 wheel)
    }
    return (seed.decoder_tag == RUNTIME_SYNC_SEED_DECODER_TAG_60_2);
}

// Compatibilidade para FAST REACQUIRE (otimização Start-Stop)
inline bool runtime_seed_fast_reacquire_compatible_60_2(const RuntimeSyncSeed& seed) noexcept {
    if (!runtime_seed_boot_compatible_60_2(seed)) {
        return false;
    }
    // Fast-gap promotion path ancora no próximo gap detectado;
    // exige seed alinhada ao gap (tooth_index = 0)
    return (seed.tooth_index == 0u);
}
```

**Por Que `tooth_index == 0` para Fast Reacquire?**

- O **fast reacquire** é um caminho otimizado que promove diretamente para `FULL_SYNC` no primeiro gap detectado
- Se `tooth_index != 0`, o sistema precisaria contar dentes até alcançar a posição salva, perdendo a vantagem
- Salvar sempre no gap (tooth_index=0) garante que o próximo gap encontrado seja válido para promoção imediata

### 2.4 Validação da Seed (Janela de 2s)

**Código (`main_stm32.cpp`, linhas 405-411):**

```cpp
// No loop de 2ms — valida seed carregada
if (g_runtime_seed_arm_window_active) {
    if (elapsed(now, g_runtime_seed_arm_window_start_ms, kRuntimeSeedArmWindowMs)) {
        // Janela de 2s expirou sem validação
        ems::drv::ckp_seed_disarm();  // Descarta seed
        g_runtime_seed_arm_window_active = false;
    }
}
```

**Fluxo de Validação na ISR (`ckp.cpp`):**

```cpp
// Quando um gap é detectado (process_gap_event)
case SyncState::WAIT_GAP:
case SyncState::LOSS_OF_SYNC:
    if (g_state.tooth_count < kGapThresholdTooth) {
        g_state.tooth_count = 0u;
        return false;
    }
    
    if (g_seed_armed) {
        // Seed válida + gap detectado → FULL_SYNC IMEDIATO
        g_state.snap.state   = SyncState::FULL_SYNC;
        g_state.snap.phase_A = g_seed_phase_a;  // Restaura fase do CMP
        g_seed_armed         = false;
        g_seed_probation     = true;
        g_seed_probation_teeth = 0u;
    } else {
        // Sem seed → Half Sync tradicional (espera 2º gap)
        g_state.snap.state = SyncState::HALF_SYNC;
    }
    g_state.tooth_count = 0u;
    g_state.snap.tooth_index = 0u;
    return true;
```

**Período de Probation (Pós-Validação):**

```cpp
// Após validar seed, monitora próximos 70 dentes (~2 voltas)
if (g_seed_probation) {
    ++g_seed_probation_teeth;
    if (g_seed_probation_teeth >= kSeedCamConfirmMaxTeeth) {
        g_seed_confirmed_count++;  // Seed confirmada como válida
        g_seed_probation = false;
    }
}
```

---

## 3. 🎯 CENÁRIOS DE OPERAÇÃO

### 3.1 Cenário 1: Primeira Partida (Cold Start)

```
┌──────────────────────────────────────────────────────────────┐
│ CONDIÇÃO: Motor frio, primeira partida após montagem         │
├──────────────────────────────────────────────────────────────┤
│ BOOT:                                                        │
│   • nvm_load_runtime_seed() → FAIL (Flash vazia/0xFF)        │
│   • ckp_seed_arm() NÃO é chamado                             │
│   • g_runtime_seed_arm_window_active = false                 │
│                                                              │
│ CRANKING:                                                    │
│   • t=0ms: Starter aciona, CKP detecta bordas                │
│   • t=50ms: Primeiro gap detectado → HALF_SYNC               │
│   • t=150ms: Segundo gap detectado → FULL_SYNC               │
│   • t=150ms: tooth_index=0, phase_A=CMP lido                 │
│                                                              │
│ PRIME PULSE:                                                 │
│   • t=100ms: Dente #3 → prime_on_tooth()                     │
│   • t=100ms: g_prime_pending=true                            │
│   • t=102ms: Loop 2ms consume → injeta 25ms                  │
│                                                              │
│ TEMPO TOTAL PARA FULL_SYNC: ~150ms (2 gaps @ 200 RPM)        │
└──────────────────────────────────────────────────────────────┘
```

### 3.2 Cenário 2: Start-Stop (Semáforo)

```
┌──────────────────────────────────────────────────────────────┐
│ CONDIÇÃO: Motor quente, parada em semáforo (Start-Stop)      │
├──────────────────────────────────────────────────────────────┤
│ PARADA (t=0-3000ms):                                         │
│   • RPM cai de 800 → 0                                       │
│   • t=100ms após RPM=0: nvm_save_runtime_seed()              │
│     - seed.tooth_index = 0 (último gap)                      │
│     - seed.phase_A = true (exemplo)                          │
│     - seed.flags = VALID | FULL_SYNC | PHASE_A               │
│   • Flash escrita: endereço 0x08100200 (offset 512B)         │
│                                                              │
│ BOOT (t=3000ms):                                             │
│   • nvm_load_runtime_seed() → SUCCESS                        │
│   • runtime_seed_fast_reacquire_compatible_60_2() → TRUE     │
│   • ckp_seed_arm(phase_A=true)                               │
│   • g_runtime_seed_arm_window_active = true (2s)             │
│   • nvm_clear_runtime_seed()                                 │
│                                                              │
│ CRANKING ACELERADO:                                          │
│   • t=3100ms: Starter aciona                                 │
│   • t=3150ms: PRIMEIRO GAP detectado                         │
│     → g_seed_armed=true + gap → FULL_SYNC IMEDIATO!          │
│     → tooth_index=0, phase_A=true (restaurado)               │
│   • t=3150ms: Injeção sequencial já sincronizada             │
│                                                              │
│ TEMPO TOTAL PARA FULL_SYNC: ~50ms (1 gap @ 200 RPM)          │
│ ECONOMIA vs Cold Start: 100ms (33% mais rápido)              │
└──────────────────────────────────────────────────────────────┘
```

### 3.3 Cenário 3: Falha de Validação (Rotação Reversa)

```
┌──────────────────────────────────────────────────────────────┐
│ CONDIÇÃO: Seed carregada, mas motor gira reverso na partida  │
├──────────────────────────────────────────────────────────────┤
│ BOOT:                                                        │
│   • Seed carregada com sucesso, ckp_seed_arm(true)           │
│   • Janela de 2s iniciada                                    │
│                                                              │
│ CRANKING PROBLEMÁTICO:                                       │
│   • t=0ms: Starter aciona, mas pinhão engata errado          │
│   • t=0-500ms: Motor gira reverso (compressão)               │
│   • ISR CKP: Bordas detectadas, mas padrão irregular         │
│   • Gap detectado em posição inesperada                      │
│                                                              │
│ VALIDAÇÃO FALLBACK:                                          │
│   • Opção A: Janela de 2s expira sem validação               │
│     → ckp_seed_disarm()                                      │
│     → Máquina de estados reinicia em WAIT_GAP                │
│     → Aguarda 2 gaps tradicionais para FULL_SYNC             │
│                                                              │
│   • Opção B: Seed consumida mas fase inválida                │
│     → g_seed_probation_teeth não atinge 70                   │
│     → Após 70 dentes, seed considerada inválida              │
│     → g_seed_rejected_count++                                │
│                                                              │
│ RESULTADO:                                                   │
│   • Partida ocorre via caminho tradicional (mais lento)      │
│   • Seed descartada, próxima partida será cold start         │
│   • Diagnóstico: g_seed_rejected_count exposto via UI        │
└──────────────────────────────────────────────────────────────┘
```

---

## 4. 🔍 ANÁLISE DE ROBUSTEZ

### 4.1 Pontos Fortes da Implementação Atual

| Recurso | Implementação | Benefício |
|---------|---------------|-----------|
| **Prime Pulse sem FULL_SYNC** | ✅ Baseado apenas em CKP | Permite injeção antes de sincronismo completo |
| **Reset de g_prime_tooth_count** | ✅ Se `> target + 5` | Previne falha em tentativas múltiplas |
| **Seed gap-aligned** | ✅ tooth_index = 0 | Maximiza chance de fast reacquire |
| **Janela de validação** | ✅ 2s timeout | Evita seed obsoleta em parada longa |
| **Probation period** | ✅ 70 dentes (~2 voltas) | Confirma seed antes de confiar cegamente |
| **Clear após load** | ✅ nvm_clear_runtime_seed() | Evita re-uso acidental de seed antiga |
| **Diagnóstico exposto** | ✅ Contadores de seed confirm/reject | Debug em bancada |

### 4.2 Vulnerabilidades Identificadas

#### VULN-1: Seed Perdida em Rotação Reversa

**Problema:**
```cpp
// ckp.cpp — process_gap_event()
if (g_seed_armed) {
    g_state.snap.state   = FULL_SYNC;
    g_state.snap.phase_A = g_seed_phase_a;
    g_seed_armed         = false;
    // ❌ Seed consumida MESMO se rotação for reversa!
    // ❌ Não há verificação de coerência temporal
}
```

**Cenário de Falha:**
1. Seed carregada no boot (phase_A = true)
2. Motor gira reverso durante partida (compressão)
3. Gap detectado em ordem inversa
4. Seed consumida → FULL_SYNC com phase_A errado
5. Próxima partida: seed já foi limpa, cold start obrigatório

**Fix Recomendado:**
```cpp
// Adicionar validação de coerência temporal
static uint32_t g_last_valid_period_ticks = 0u;

if (g_seed_armed) {
    // Verifica se período atual é coerente com rotação normal
    const uint32_t current_period = /* período medido */;
    const uint32_t prev_period = g_last_valid_period_ticks;
    
    // Razão máxima aceitável: 0.5x (desaceleração brusca)
    if (prev_period > 0u && current_period < (prev_period / 2u)) {
        // Rotação reversa ou glitch — descarta seed
        g_seed_rejected_count++;
        g_seed_armed = false;
        g_state.snap.state = HALF_SYNC;
        return false;
    }
    
    g_state.snap.state = FULL_SYNC;
    g_state.snap.phase_A = g_seed_phase_a;
    g_seed_armed = false;
    g_last_valid_period_ticks = current_period;
}
```

#### VULN-2: Janela de 2s Muito Curta para Start-Stop Longo

**Problema:**
```cpp
static constexpr uint32_t kRuntimeSeedArmWindowMs = 2000u;  // 2s
```

**Cenário:**
- Veículo para no semáforo (seed salva)
- Motorista espera 3s (semáforo longo)
- Ao tentar partir: janela expirou, seed descartada
- Perda de benefício Start-Stop

**Fix Recomendado:**
```cpp
// Aumentar janela para 5s (cobre 95% dos semáforos urbanos)
static constexpr uint32_t kRuntimeSeedArmWindowMs = 5000u;

// OU: Janela dinâmica baseada em CLT
static constexpr uint32_t kRuntimeSeedArmWindowMs_Cold = 10000u;  // 10s se CLT < 40°C
static constexpr uint32_t kRuntimeSeedArmWindowMs_Hot = 3000u;   // 3s se CLT >= 40°C
```

#### VULN-3: Prime Pulse em LOSS_OF_SYNC

**Problema:**
```cpp
// quick_crank.cpp — prime_on_tooth()
void prime_on_tooth(const CkpSnapshot& snap) noexcept {
    // ❌ Não verifica estado de sync!
    // Opera mesmo em LOSS_OF_SYNC
    if (snap.rpm_x10 == 0u || snap.rpm_x10 >= 5000u) { return; }
    ...
}
```

**Cenário de Risco:**
- Motor em LOSS_OF_SYNC (glitch EMC, roda fônica danificada)
- Prime pulse disparado em dente aleatório
- Combustível injetado no coletor sem sincronismo
- Primeira ignição (quando FULL_SYNC retornar) pode ocorrer em cilindro errado

**Fix Recomendado:**
```cpp
void prime_on_tooth(const CkpSnapshot& snap) noexcept {
    if (g_prime_done) { return; }
    
    // Requer pelo menos HALF_SYNC (1 gap detectado)
    if (snap.state == SyncState::WAIT_GAP || 
        snap.state == SyncState::LOSS_OF_SYNC) {
        return;  // Aguarda sincronismo mínimo
    }
    
    if (snap.rpm_x10 == 0u || snap.rpm_x10 >= 5000u) { return; }
    ...
}
```

**Contraponto (segurança):**
- Prime pulse é APENAS combustível, sem ignição associada
- Combustível distribui-se homogeneamente no coletor
- Primeira ignição só ocorre após FULL_SYNC confirmado
- **Risco baixo**, mas validação adicional é defensiva

---

## 5. 📊 MÉTRICAS DE DIAGNÓSTICO EXPOSTAS

### 5.1 Contadores Runtime Seed

```cpp
// ckp.cpp — variáveis globais expostas via UI
static volatile uint32_t g_seed_loaded_count = 0u;      // Seeds carregadas no boot
static volatile uint32_t g_seed_confirmed_count = 0u;   // Seeds validadas com sucesso
static volatile uint32_t g_seed_rejected_count = 0u;    // Seeds rejeitadas (probation fail)
```

**Interpretação:**
| Métrica | Normal | Alerta | Ação |
|---------|--------|--------|------|
| `g_seed_loaded_count` | 1 por boot | >1 | Bug no clear da seed |
| `g_seed_confirmed_count` | ≈ loaded | << loaded | Problema de validação |
| `g_seed_rejected_count` | 0 | >0 | Rotação reversa ou glitch |

### 5.2 Contadores Prime Pulse

```cpp
// quick_crank.cpp
volatile uint8_t  g_prime_tooth_count = 0u;   // Dentes contados
volatile bool     g_prime_done = false;        // Disparado neste ciclo
volatile bool     g_prime_pending = false;     // Aguardando consumo
```

**Debug em Bancada:**
```
UI Command: READ_PRIME_STATUS
Response: {
  "tooth_count": 3,
  "done": true,
  "pending": false,
  "pw_us": 25000,
  "clt_x10": 250,
  "dead_time_us": 900
}
```

---

## 6. 🛠️ RECOMENDAÇÕES DE MELHORIA

### 6.1 Prioridade Alta (P1)

#### FIX-1: Validação de Coerência Temporal na Seed

**Arquivo:** `src/drv/ckp.cpp`  
**Local:** `process_gap_event()`  
**Justificativa:** Previne consumo de seed em rotação reversa

```cpp
// Adicionar antes de consumir seed
if (g_seed_armed) {
    const uint32_t current_period = /* período atual medido */;
    const uint32_t expected_min = g_state.prev_period_ticks / 3u;  // 3x desaceleração
    
    if (current_period < expected_min) {
        // Período incompatível com rotação normal — seed inválida
        g_seed_rejected_count++;
        g_seed_armed = false;
        g_state.snap.state = HALF_SYNC;
        return false;
    }
    
    // Seed válida — prossegue com FULL_SYNC
    ...
}
```

#### FIX-2: Aumentar Janela de Validação para Start-Stop

**Arquivo:** `src/main_stm32.cpp`  
**Local:** Linha 93  
**Justificativa:** Cobre semáforos longos (>2s)

```cpp
// De: 2000ms → Para: 5000ms
static constexpr uint32_t kRuntimeSeedArmWindowMs = 5000u;
```

#### FIX-3: Adicionar Validação de Sync no Prime Pulse

**Arquivo:** `src/engine/quick_crank.cpp`  
**Local:** `prime_on_tooth()`  
**Justificativa:** Defesa contra LOSS_OF_SYNC

```cpp
void prime_on_tooth(const CkpSnapshot& snap) noexcept {
    if (g_prime_done) { return; }
    
    // FIX: Requer pelo menos HALF_SYNC
    if (snap.state == SyncState::WAIT_GAP ||
        snap.state == SyncState::LOSS_OF_SYNC) {
        return;
    }
    
    // ... restante do código
}
```

### 6.2 Prioridade Média (P2)

#### ENHANCE-1: Janela Dinâmica Baseada em CLT

**Arquivo:** `src/main_stm32.cpp`

```cpp
// Substituir constante fixa por função
uint32_t get_seed_arm_window_ms(int16_t clt_x10) noexcept {
    if (clt_x10 < 400) {  // < 40°C
        return 10000u;  // 10s — motor frio, partida mais lenta
    } else if (clt_x10 < 800) {  // 40-80°C
        return 5000u;   // 5s — temperatura normal
    } else {  // >= 80°C
        return 3000u;   // 3s — motor quente, Start-Stop rápido
    }
}

// Uso no loop:
if (elapsed(now, g_runtime_seed_arm_window_start_ms,
            get_seed_arm_window_ms(sensors.clt_degc_x10))) {
    ems::drv::ckp_seed_disarm();
    g_runtime_seed_arm_window_active = false;
}
```

#### ENHANCE-2: Persistir Estatísticas de Seed em Flash

**Arquivo:** `src/hal/flash.cpp`

```cpp
struct SeedStats {
    uint32_t total_loads;
    uint32_t total_confirms;
    uint32_t total_rejects;
    uint32_t checksum;
};

// Salvar estatísticas periodicamente (a cada 100 ignitions)
void nvm_save_seed_stats(const SeedStats* stats) noexcept;
void nvm_load_seed_stats(SeedStats* stats_out) noexcept;
```

**Benefício:** Diagnóstico de longo prazo para calibração de campo.

#### ENHANCE-3: Log de Evento de Seed via UART

**Arquivo:** `src/drv/ckp.cpp`

```cpp
if (g_seed_armed && gap_detected) {
    // Log via UART para debug
    uart_printf("SEED: armed=%d phase=%d state=%d\n",
                g_seed_armed, g_seed_phase_a, g_state.snap.state);
    
    if (validation_passed) {
        uart_printf("SEED: CONFIRMED tooth=%d\n", g_state.snap.tooth_index);
    } else {
        uart_printf("SEED: REJECTED reason=%d\n", reject_reason);
    }
}
```

---

## 7. ✅ VEREDITO FINAL

### 7.1 Avaliação Geral

| Critério | Nota | Comentário |
|----------|------|------------|
| **Arquitetura** | ⭐⭐⭐⭐⭐ | Separação clara entre Prime Pulse, Quick Crank e Runtime Seed |
| **Robustez** | ⭐⭐⭐⭐ | Mecanismos de fallback presentes, mas validações podem ser reforçadas |
| **Documentação** | ⭐⭐⭐⭐⭐ | Comentários inline excepcionais, fluxos bem explicados |
| **Diagnóstico** | ⭐⭐⭐⭐ | Contadores expostos, mas poderiam ser mais granulares |
| **Eficiência** | ⭐⭐⭐⭐⭐ | Start-Stop economiza ~100ms (33% mais rápido) |

### 7.2 Status para Produção

**Pronto para MVP de Bancada:** ✅ SIM  
**Pronto para Veículo de Rua:** ⚠️ AGUARDA FIXES P1

**Bloqueadores para Produção:**
1. ✅ CMP sem filtro de coerência — **CORRIGIDO**
2. ✅ TIM8 wraparound — **CORRIGIDO**
3. ✅ Race condition SensorData — **CORRIGIDO**
4. ✅ Reset g_prime_tooth_count — **CORRIGIDO**
5. ✅ Errata Flash workaround — **CORRIGIDO**
6. 🔴 Validação de seed em rotação reversa — **PENDENTE (P1)**
7. 🔴 Janela de 2s muito curta — **PENDENTE (P1)**
8. 🔴 Prime pulse em LOSS_OF_SYNC — **PENDENTE (P2)**

### 7.3 Próximos Passos Recomendados

1. **Implementar FIX-1, FIX-2, FIX-3** (prioridade P1)
2. **Testes de bancada com cenários de falha:**
   - Rotação reversa intencional durante partida
   - Start-Stop com pausas de 1s, 3s, 5s, 10s
   - Perda de energia durante salvamento de seed
3. **Adicionar testes unitários** para:
   - `runtime_seed_fast_reacquire_compatible_60_2()`
   - `prime_on_tooth()` com diferentes estados de sync
   - Validação de janela temporal
4. **Coletar dados em veículo real** por 100 ciclos Start-Stop
5. **Revisar métricas de diagnóstico** e ajustar thresholds

---

**Documento criado:** 2025-01-XX  
**Autor:** OpenEMS Review Team  
**Revisão:** 1.0  
**Status:** Em revisão para aprovação de fixes P1
