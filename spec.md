# OpenEMS — Especificação Técnica

**Hardware alvo:** STM32H562RGT6 · Cortex-M33 · 250 MHz  
**Linguagem:** C++17 · sem STL no caminho crítico · sem alocação dinâmica  
**Build:** `make firmware` → `/tmp/openems-build/bin/openems.{hex,bin}`

---

## 1. Arquitetura em Camadas

```
┌─────────────────────────────────────────────────────────────┐
│  APP      can_stack · ui_protocol · can_filters             │
├─────────────────────────────────────────────────────────────┤
│  ENGINE   fuel_calc · ign_calc · ecu_sched · etb_control    │
│           torque_manager · knock · quick_crank · auxiliaries│
│           transient_fuel · xtau_autocalib · map_estimator   │
│           table3d · calibration · diagnostic_manager        │
├─────────────────────────────────────────────────────────────┤
│  DRV      ckp · sensors                                     │
├─────────────────────────────────────────────────────────────┤
│  HAL      system · adc · can · uart · usb_cdc · flash       │
│           etb_driver · timer · gpio · runtime_seed          │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Clock Tree e Timing

| Recurso | Base | Div/Mul | Resultado | Período |
|---|---|---|---|---|
| SYSCLK | HSE 8 MHz | PLL N=125 P=4 | **250 MHz** | 4 ns |
| HCLK | SYSCLK | ÷1 | 250 MHz | 4 ns |
| APB1/APB2 | SYSCLK | ÷2 | 125 MHz | 8 ns |
| TIM5 (CKP IC) | APB1×2 | PSC=3 (÷4) | **62,5 MHz** | 16 ns |
| TIM2/TIM8 (sched) | APB1×2 | PSC=24 (÷25) | **10 MHz** | 100 ns |
| ADC1/ADC2 | HCLK | ÷4 | 62,5 MHz | 16 ns |
| SysTick | HCLK | reload=250000 | 1 ms tick | — |
| IWDG | LSI 32 kHz | ÷32 reload=99 | **100 ms timeout** | — |

---

## 3. Mapa de Memória

### Flash (1 MB — 0x08000000)
| Região | Endereço | Conteúdo |
|---|---|---|
| Bank 1 | 0x08000000–0x080FFFFF | Firmware (.text, .rodata) |
| Bank 2 Setor 0 | 0x08100000 (8 KB) | LTFT 256 B · Knock 64 B · RuntimeSeed 32 B |
| Bank 2 Setores 1–7 | +8 KB cada | Calibração páginas 0–6 (512 B) |

### RAM (512 KB — 0x20000000)
| Seção | Conteúdo |
|---|---|
| .isr_vector | Vetor IRQ copiado da Flash (evita congelamento 120 µs pós-reset) |
| .fastrun | ISRs críticos (Default_Handler, Reset_Handler) |
| .data | Globais inicializados |
| .bss | Globais zerados |
| .stack | 8 KB (guard em `_estack_guard`) |

---

## 4. Mapa de Pinos

| Pino | Função | Timer/AF |
|---|---|---|
| PA0 | CKP input capture | TIM5_CH1 AF2 |
| PA1 | CMP (cam) input capture | TIM5_CH2 AF2 |
| PA15, PB3, PB10, PB11 | INJ1–4 output compare | TIM2_CH1–4 AF1 |
| PC6, PC7, PC8, PC9 | IGN1–4 output compare | TIM8_CH1–4 AF3 |
| PA6, PA7 | IACV / Wastegate PWM | TIM3_CH1–2 AF2 |
| PB6, PB7 | VVT Esc / Adm PWM | TIM4_CH1–2 AF2 |
| PA8 | ETB PWM (H-bridge) | TIM1_CH1 AF1 |
| PB14, PB15 | ETB DIR / EN GPIO | — |
| PA9, PA10 | UART1 TX/RX (UI) | USART1 AF7 |
| PB8, PB9 | CAN RX/TX (WBO2) | FDCAN1 AF9 |
| PA11, PA12 | USB D−/D+ | USB AF10 |
| PB12, PB13 | Fan / Fuel Pump GPIO | — |
| PA5 | Knock sensor input | ADC1_IN6 |
| PB0–PB1, PC0–PC1 | ADC1 (APP1–2, ETB_TPS1–2) | ADC1 |
| PC2–PC5 | ADC2 (CLT, IAT, FuelP, OilP) | ADC2 |

---

## 5. HAL — Hardware Abstraction Layer

### 5.1 System (`hal/stm32h562/system.h`)
```cpp
void system_stm32_init() noexcept; // PLL + SysTick + IWDG
uint32_t millis() noexcept;        // ms desde boot (atômico)
uint32_t micros() noexcept;        // µs (SysTick CVR)
void iwdg_kick() noexcept;         // feed watchdog
```

### 5.2 ADC (`hal/adc.h`)
```cpp
enum class AdcPrimaryChannel : uint8_t {
    MAP_SE10=0, MAF_V_SE11=1, TPS_SE12=2, KNOCK_SE4B=3,  // PA5 — knock
    AN1_SE6B=4, AN2_SE7B=5, AN3_SE8B=6,  AN4_SE9B=7
};
enum class AdcSecondaryChannel : uint8_t {
    CLT_SE14=0, IAT_SE15=1, FUEL_PRESS_SE5B=2, OIL_PRESS_SE6B=3
};

void    adc_init();
uint16_t adc_primary_read(AdcPrimaryChannel ch);
uint16_t adc_secondary_read(AdcSecondaryChannel ch);
void    adc_trigger_on_tooth(uint32_t tooth_period_ticks); // TIM6 ARR = period/2
bool    adc_is_recovering();
bool    adc_recovery_failed();
```
- ADC clock: 62,5 MHz · 12-bit · trigger TIM6_TRGO · GPDMA one-shot com re-arm por ISR
- Sampling: 47,5 ciclos (~0,76 µs por canal)
- **Nota:** Sensor O2/lambda (wideband) é recebido exclusivamente via CAN (FDCAN1, frame 0x180), não por ADC

### 5.3 Timers (`hal/stm32h562/timer.h`)
```cpp
void    tim5_ic_init();            // Input capture CKP/CMP
uint32_t tim5_count() noexcept;
void    tim3_pwm_init(uint32_t freq_hz);  // IACV/Wastegate
void    tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10);
void    tim4_pwm_init(uint32_t freq_hz);  // VVT
void    tim4_set_duty(uint8_t ch, uint16_t duty_pct_x10);
void    tim1_etb_pwm_init(uint32_t freq_hz); // ETB 20 kHz (namespace ems::hal)
void    tim1_etb_set_duty_x10(uint16_t duty_pct_x10);
// C-linkage wrappers chamados por etb_driver.cpp:
void    timer_etb_pwm_init(void);            // configura PA8 CH1 + PA9 CH1N com dead-time
void    timer_etb_set_duty(uint16_t duty);   // duty 0–1000 (0–100%)
```
- TIM3/TIM4: PSC=3 (÷4, 62,5 MHz base)
- TIM1 ETB: ARR=12499 @ 20 kHz, dead-time ~200 ns
- PA8 = TIM1\_CH1 (half-bridge high-side); PA9 = TIM1\_CH1N (complementar, low-side)

### 5.4 CAN (`hal/can.h`)
```cpp
struct CanFrame { uint32_t id; uint8_t dlc; uint8_t data[8]; bool extended; };
void can0_init();             // FDCAN1 500 kbps
bool can0_tx(const CanFrame&);
bool can0_rx_pop(CanFrame&);  // Pop de FIFO0
```
- Bit timing: NBRP=4 · NTSEG1=17 · NTSEG2=5 → 500 kbps
- Message RAM: TX 2 elementos · RX FIFO0 3 elementos
- Filtro RX: aceita apenas ID do WBO2 (padrão 0x180)

### 5.5 Flash/NVM (`hal/flash.h`)
```cpp
bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len);
bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len);
bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val);
int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i);
bool nvm_flush_adaptive_maps();   // Máquina de estados (não bloqueante)
bool nvm_save_runtime_seed(const RuntimeSyncSeed*);
bool nvm_load_runtime_seed(RuntimeSyncSeed*);
bool nvm_clear_runtime_seed();
```
- Bank 2 exclusivo para dados: não interfere com execução de código
- Flush adaptativo: state machine Idle→WaitErase→Program→WaitFinal, 16 words/passo

### 5.6 ETB Driver (`hal/etb_driver.h`)
```cpp
void etb_driver_init();
void etb_driver_enable(bool enable);
bool etb_driver_is_enabled();
void etb_driver_set_output_pct_x10(int16_t signed_pct_x10); // -1000 a +1000
```
- PWM TIM1_CH1 (PA8) + GPIO DIR (PB14) + EN (PB15)
- PB14=H → abertura; PB14=L → fechamento

### 5.7 UART (`hal/uart.h`)
```cpp
void uart0_init(uint32_t baud = 115200);
void uart0_poll_rx(uint16_t max_bytes);
bool uart0_rx_pop(uint8_t& byte);
bool uart0_tx_push(uint8_t byte);
void uart0_tx_poll(uint16_t max_bytes);
```
- Buffers circulares: RX 128 B · TX 256 B · polling não-bloqueante

### 5.8 USB CDC (`hal/stm32h562/usb_cdc.h`)
```cpp
void usb_cdc_init();
void usb_cdc_poll();
void usb_cdc_send_bytes(const uint8_t* data, uint16_t len);
uint16_t usb_cdc_read_bytes(uint8_t* buf, uint16_t max_len);
bool usb_cdc_dtr();
```
- VID/PID: 0x0483/0x5740 (ST compatible) · EP1 IN bulk · EP2 OUT bulk
- PMA 2 KB em 0x40016C00

### 5.9 Runtime Seed (`hal/runtime_seed.h`)
```cpp
struct RuntimeSyncSeed {
    uint16_t magic;       // 0x5343 ('SC')
    uint8_t  version;     // 1
    uint8_t  flags;       // VALID | FULL_SYNC | PHASE_A
    uint16_t tooth_index; // 0–57
    uint16_t decoder_tag; // 0x3C02 (60-2)
    uint32_t sequence;
    uint32_t crc32;
};
bool runtime_seed_fast_reacquire_compatible_60_2(const RuntimeSyncSeed&);
```
- Permite resincronização instantânea após reinicialização quente
- CRC-32 validado antes de uso

---

## 6. DRV — Drivers de Plataforma

### 6.1 CKP — Decoder Roda Fônica 60-2 (`drv/ckp.h`)

#### Estados
```
WAIT_GAP   → HALF_SYNC   (1º gap detectado)
HALF_SYNC  → FULL_SYNC   (2º gap confirmado)
FULL_SYNC  → LOSS_OF_SYNC (> 63 dentes sem gap)
LOSS_OF_SYNC → WAIT_GAP  (reset automático)
```

#### Estruturas
```cpp
struct CkpSnapshot {
    uint32_t tooth_period_ns;
    uint32_t predicted_tooth_period_ns;
    uint16_t tooth_index;     // 0–57 em FULL_SYNC
    uint32_t last_tim5_capture;
    uint32_t rpm_x10;
    SyncState state;
    bool phase_A;             // alterna a cada borda do cam sensor
};
```

#### Interface
```cpp
CkpSnapshot ckp_snapshot() noexcept;    // atômico CPSID/CPSIE + memcpy
void ckp_tim5_ch1_isr() noexcept;      // CKP ISR — prioridade NVIC 1
void ckp_tim5_ch2_isr() noexcept;      // CMP ISR — alterna phase_A
void ckp_seed_arm(bool phase_A);
void ckp_seed_disarm();
uint32_t ckp_seed_loaded_count();
uint32_t ckp_seed_confirmed_count();
uint32_t ckp_seed_rejected_count();
```

#### Algoritmo ISR (CKP)
1. Lê TIM5_CCR1 (não CNT) — timestamp preciso
2. Delta circular uint32_t → período em ticks de 16 ns
3. Rejeita < 50 ticks (anti-glitch ~800 ns)
4. Primeiros 3 dentes: aceita incondicionalmente (build histórico)
5. Detecta gap: `period × 2 > avg × 3` (ratio 1,5×)
6. Valida dente normal: `0,8×avg ≤ period ≤ 1,2×avg` (±20%)
7. Atualiza máquina de estados + snapshot
8. Dispatch hooks: `sensors_on_tooth`, `schedule_on_tooth`, `prime_on_tooth`

#### Constantes
| Constante | Valor | Descrição |
|---|---|---|
| kRealTeethPerRev | 58 | Dentes físicos por revolução |
| kTeethPositionsPerRev | 60 | Posições angulares |
| kGapThresholdTooth | 55 | Mínimo antes de aceitar novo gap |
| kMaxTeethBeforeLoss | 63 | Perda de sincronismo |
| kHistSize | 3 | Janela de média de períodos |
| kMinToothTicks | 50 | ~800 ns anti-glitch |

#### Validação de Rotação Forward (P0 BUG-12)
- Requer 3 períodos coerentes consecutivos (variação < ±25%) para aceitar seed de resincronização rápida
- Previne aceitação de seed durante rotação reversa

### 6.2 Sensors (`drv/sensors.h`)

#### Estruturas
```cpp
struct SensorData {
    uint16_t map_bar_x1000;        // 0–300,0 bar
    uint32_t maf_gps_x100;       // g/s × 100
    uint16_t tps_pct_x10;        // 0–100,0 %
    int16_t  clt_degc_x10;       // −40 a +150 °C
    int16_t  iat_degc_x10;
    uint16_t fuel_press_bar_x1000;
    uint16_t oil_press_bar_x1000;
    uint16_t vbatt_mv;           // 6000–18000 mV
    uint8_t  fault_bits;         // bit N = SensorId N
    uint16_t app1_pct_x10;       // Pedal 1
    uint16_t app2_pct_x10;       // Pedal 2
    uint16_t app_pct_x10;        // Validado (min dos dois)
    uint16_t etb_tps1_pct_x10;   // TPS borboleta 1
    uint16_t etb_tps2_pct_x10;
    uint16_t etb_tps_pct_x10;    // Média validada
    uint8_t  throttle_fault_bits;
    uint16_t an1_raw, an2_raw, an3_raw, an4_raw; // Passthrough bruto
};
```

#### Interface
```cpp
void sensors_init();
void sensors_on_tooth(const CkpSnapshot& snap); // Hook ISR — aciona ADC
void sensors_tick_50ms();                       // Pressões (fuel/oil)
void sensors_tick_100ms();                      // CLT/IAT + APP/ETB
SensorData sensors_get() noexcept;             // Lê g_data_committed (sem CPSID)

void sensors_set_tps_cal(uint16_t raw_min, uint16_t raw_max);
void sensors_set_app_cal(uint16_t a1min, uint16_t a1max, uint16_t a2min, uint16_t a2max);
void sensors_set_etb_tps_cal(uint16_t t1min, uint16_t t1max, uint16_t t2min, uint16_t t2max);
void sensors_set_plausibility(uint16_t app_max_delta, uint16_t etb_max_delta);
void sensors_set_etb_harness_present(bool present);
bool validate_sensor_values(const SensorData&);
uint8_t get_sensor_health_status();
```

#### Double Buffer (FIX-6 BUG-10)
```
g_data_staging   ← escrito por sensors_on_tooth() (ISR TIM5, prio 1)
                 ← atualizado por tick_50ms/tick_100ms (background)
g_data_committed ← snapshot atômico em tick_100ms (CPSID/CPSIE)
sensors_get()    ← lê g_data_committed (sem seção crítica necessária)
```

#### Conversões
| Sensor | Fórmula |
|---|---|
| MAP | `raw × 3000 / 4095` → bar×10 |
| TPS/APP/ETB | `(raw − min) × 1000 / (max − min)` → %×10 |
| CLT/IAT | LUT 128 pontos (−40 a +150 °C) |
| VBatt | `raw × 18000 / 4095` → mV |
| FuelP/OilP | `avg4 × 2500 / 4095` → bar×10 |

#### Ranges Padrão de Falha
| Sensor | Min raw | Max raw |
|---|---|---|
| MAP | 50 | 4095 |
| CLT | 100 | 3800 |
| IAT | 100 | 3900 |
| TPS | 50 | 4095 |
| FUEL/OIL | 50 | 4050 |

#### Throttle Fault Bits
```
bit 0: THROTTLE_FAULT_APP1       — APP1 fora de range (3 strikes)
bit 1: THROTTLE_FAULT_APP2
bit 2: THROTTLE_FAULT_APP_PLAUS  — Delta APP1/APP2 > app_max_delta
bit 3: THROTTLE_FAULT_ETB_TPS1
bit 4: THROTTLE_FAULT_ETB_TPS2
bit 5: THROTTLE_FAULT_ETB_PLAUS  — Delta TPS1/TPS2 > etb_max_delta
```

---

## 7. ENGINE — Módulos de Cálculo

### 7.1 Configuração do Motor (`engine/engine_config.h`)
| Constante | Valor | Unidade |
|---|---|---|
| kCylinderCount | 4 | — |
| kDisplacementCc | 2000 | cm³ |
| kInjectorFlowCcMin | 450 | cc/min |
| kStoichAfrX100 | 1300 | (AFR E30 = 13,0) |
| kFuelDensityMgPerCc | 755 | mg/cc |
| kAirDensityMgPerCcX1000 | 1184 | mg/cc×1000 |
| kMapRefBarX100 | 100 | bar |
| kDefaultEoiLeadDeg | 355 | ° BTDC combustão (EOI open-valve, Speeduino-style) |
| kIvcAbdcDeg | 50 | ° ABDC |
| kFiringOrder | {0,2,3,1} | — |
| `cyl_tdc_deg(cyl)` | `cyl × 180` | ° |

### 7.2 Tabelas 3D (`engine/table3d.h`)

**Eixos globais:**
- `kRpmAxisX10[16]` = {5000, 7500, 10000, …, 120000} (RPM × 10)
- `kLoadAxisBarX100[16]` = {20, 30, 40, …, 300} (bar)

**Funções:**
```cpp
Table2dLookup table3d_prepare_lookup(x_axis, y_axis, rpm_x10, map_bar_x100);
uint8_t  table3d_lookup_u8_prepared(table[16][16], lookup);   // VE
int16_t  table3d_lookup_i8_prepared(table[16][16], lookup);   // Spark
int16_t  table3d_lookup_s16_prepared(table[16][16], lookup);  // Lambda
```

Interpolação bilinear em Q8: resolve xi, yi, fx_q8, fy_q8 por busca binária.

### 7.3 Calibração (`engine/calibration.h`)

**Tabelas 3D (16×16):**
- `ve_table[16][16]` — Eficiência volumétrica (45–254 %)
- `lambda_target_table_x1000[16][16]` — Lambda alvo ×1000 (765–1050)
- `spark_table[16][16]` — Avanço base em graus (−10 a +40)

**Tabelas de correção (8 pontos):**
| Tabela | Eixo | Range |
|---|---|---|
| clt_corr_x256 | CLT ×10 | −400 a 1100 |
| iat_corr_x256 | IAT ×10 | −200 a 1200 |
| warmup_corr_x256 | CLT ×10 | −400 a 1100 |
| injector_dead_time_us | VBatt mV | 9000–16000 |
| dwell_ms_x10_table | VBatt mV | 9000–16000 |

**Tabelas AE (4 pontos):**
- ae_tpsdot_axis_x10 = {5, 20, 50, 100}
- ae_pw_adder_us = {300, 800, 1500, 2500}

**Tabelas X-tau (8 pontos):**
- xtau_x_fraction_q8 (24–77) — fração de parede em Q8
- xtau_tau_cycles (8–32) — constante de tempo em ciclos

**Lambda delay (3×3):**
- Eixo RPM: {10000, 25500, 80000}
- Eixo MAP: {10, 91, 300}
- Delay: 80–1100 ms

**Calibração ETB (offsets em page 0):**
| Offset | Campo | Bytes |
|---|---|---|
| 0–15 | APP1/APP2/TPS1/TPS2 raw_min/max | 8×2 |
| 16–25 | Deltas, limp, rate, idle | 5×2 |
| 26–27 | etb_cal_valid, etb_harness_present | 1+1 |
| 28–33 | Kp, Ki, Kd × 10 | 3×2 |

### 7.4 Cálculo de Combustível (`engine/fuel_calc.h`)

**Fórmula REQ_FUEL:**
```
REQ_FUEL_us = (disp_cc × air_mg_cc × 60_000_000)
            / (cyl × stoich_afr × inj_flow_cc_min × fuel_mg_cc × 1000)
```

**Pipeline de cálculo:**
```
VE + MAP → BASE_PW = REQ_FUEL × VE/100 × MAP/MAP_REF
         → LAMBDA_PW = BASE_PW × 1000 / lambda_target_x1000
         → TRIM_PW = LAMBDA_PW × (1 + trim_pct_x10/1000)
         → CLT_CORR = TRIM_PW × clt_x256/256
         → IAT_CORR = CLT_CORR × iat_x256/256
         → FINAL_PW = IAT_CORR + dead_time_us
```

**Interface:**
```cpp
uint8_t  get_ve(uint32_t rpm_x10, uint16_t map_bar_x100);
uint8_t  get_ve_prepared(const Table2dLookup&);
uint16_t get_lambda_target_x1000(uint32_t rpm_x10, uint16_t map_bar_x100);
uint32_t calc_fuel_pw_us_default_fast(uint8_t ve, uint16_t map_bar_x100,
    uint16_t lambda_x1000, int16_t trim_pct_x10,
    uint16_t corr_clt_x256, uint16_t corr_iat_x256, uint16_t dead_time_us);
int32_t  calc_ae_pw_us(uint16_t tps_now_x10, uint16_t tps_prev_x10,
    uint16_t dt_ms, int16_t clt_x10);

// Correções
uint16_t corr_clt(int16_t clt_x10);
uint16_t corr_iat(int16_t iat_x10);
uint16_t corr_vbatt(uint16_t vbatt_mv);       // dead time
uint16_t dwell_ms_x10_from_vbatt(uint16_t vbatt_mv);

// STFT/LTFT
int16_t fuel_update_stft_delayed(uint32_t now_ms, uint32_t rpm_x10,
    uint16_t map_bar_x100, int16_t lambda_target_x1000, int16_t lambda_measured,
    int16_t clt_x10, bool lambda_valid, bool ae_active, bool rev_cut);
int16_t fuel_get_stft_pct_x10();
int16_t fuel_get_ltft_pct_x10(uint8_t map_idx, uint8_t rpm_idx);
void    fuel_reset_adaptives();
```

**Parâmetros adaptativos:**
- STFT: Kp = 0,03/λ erro · Ki = 0,005/sample · clamp ±25 %
- LTFT: 16×16 células · range ±25 %
- Delay lambda: 80–1100 ms (tabela RPM × MAP)
- STFT habilitado: FULL_SYNC · RPM > 0 · lambda_valid · !ae_active · !rev_cut

### 7.5 Cálculo de Ignição (`engine/ign_calc.h`)
```cpp
int16_t get_advance(uint32_t rpm_x10, uint16_t load_bar_x100);
int16_t get_advance_prepared(const Table2dLookup&);
int16_t calc_total_advance(int16_t base,
    const AdvanceCorrections& corr); // clamp −10 a +40°
int16_t calc_idle_spark_correction_deg(uint32_t rpm_x10,
    uint16_t idle_target_rpm_x10, uint16_t tps_pct_x10, uint16_t map_bar_x100);
uint32_t inj_pw_us_to_scheduler_ticks(uint32_t pw_us); // × 10 (100 ns/tick)
```

**Controle de avanço em idle:**
- Ativo quando: TPS < 2,5% · MAP < 0.80 bar · RPM ∈ [500, idle+400 RPM]
- Deadband: ±50 RPM
- Ganho: 1° por 50 RPM de erro
- Limites: −8 a +12°

### 7.6 Combustível Transiente — X-tau (`engine/transient_fuel.h`)

**Modelo de Aquino:**
```
desired_q8   = fuel_pw_us << 8
evap_q8      = wall_fuel_q8 / tau_cycles
numerator_q8 = desired_q8 − evap_q8
injected_q8  = numerator_q8 × 256 / (256 − x_q8)
wall_fuel_q8 += (injected_q8 × x_q8 >> 8) − evap_q8
output_pw_us = injected_q8 >> 8
```

- `x_fraction_q8`: 24–77 (fração de parede, varia com CLT)
- `tau_cycles`: 8–32 ciclos (evaporação, varia com CLT)

```cpp
void     transient_fuel_reset() noexcept;
uint32_t transient_fuel_xtau_update(uint32_t fuel_pw_us,
    int16_t clt_x10, bool enabled) noexcept;
```

### 7.7 Autocalibração X-tau (`engine/xtau_autocalib.h`)

**Algoritmo de aprendizado:**
- Tick a cada 60 s quando: STFT ∈ [−500, +500] · RPM > 2000 · lambda_valid
- Se STFT > +20‰ → tau_delta[bucket]++ (incrementa constante de tempo)
- Se STFT < −20‰ → tau_delta[bucket]-- (decrementa)
- Clamp: tau_delta ∈ [−10, +10] por bucket CLT
- Aplicação: `xtau_tau_cycles[i] += tau_delta[i]` quando dirty

```cpp
void xtau_autocalib_init() noexcept;
void xtau_autocal_tick(uint32_t now_ms, int16_t stft_pct_x10,
    int16_t clt_x10, bool learning_ok) noexcept;
bool xtau_autocal_is_active() noexcept;
bool xtau_autocal_is_dirty() noexcept;
void xtau_autocal_apply_learned_tables() noexcept;
void xtau_autocal_clear_dirty() noexcept;
```

### 7.8 Knock (`engine/knock.h`)

**Hardware:**
- Entrada: PA5 / ADC1_IN6
- Caminho: sensor piezoelétrico → filtro passa-banda externo → PA5 → ADC1
- Detecção: software via threshold ADC (STM32H562 não possui periférico COMP)

**Parâmetros:**
| Constante | Valor |
|---|---|
| kKnockCylinders | 4 |
| kDefaultAdcThreshold | 2048 (12-bit) |
| kAdcThresholdMin | 256 |
| kAdcThresholdMax | 4000 |
| kRetardStepX10 | 20 (2,0°) |
| kRetardMaxX10 | 100 (10,0°) |
| kRecoveryStepX10 | 1 (0,1°) |
| kRecoveryDelayCycles | 10 ciclos limpos |
| kThresholdDropOnKnock | 64 |
| kThresholdRiseAfterClean | 32 (após 100 ciclos) |

```cpp
extern volatile uint16_t knock_retard_x10[4]; // por cilindro

void knock_init();
void knock_window_open(uint8_t cyl);          // ativar janela de escuta
void knock_window_close(uint8_t cyl);
void knock_window_cycle_end();                // chamado no DWELL_START (fecha cil. anterior)
void knock_adc_update(uint16_t raw_adc);      // chamado de sample_fast_channels()
void knock_cycle_complete(uint8_t cyl);       // aplicar retard/recovery
uint16_t knock_get_retard_x10(uint8_t cyl);
void knock_set_adc_threshold(uint16_t threshold); // range [256, 4000]
uint16_t knock_get_adc_threshold();
```

**Algoritmo por ciclo:**
- Janela ativa: amostragem ADC em cada dente CKP via `knock_adc_update(raw)`
- Evento knock: `raw_adc > threshold` incrementa contador da janela
- Fim do ciclo: `count > 0` → retard += 2° · threshold −= 64 (mais sensível)
- Ciclo limpo: após 10+ consecutivos → retard −= 0,1°; após 100 limpos → threshold += 32 (menos sensível)
- Persistência NVM: retardo em slot knock, threshold armazenado como `int8_t` (/32)

### 7.9 Quick Crank (`engine/quick_crank.h`)

**Enriquecimento de partida por CLT:**
| CLT | Multiplicador |
|---|---|
| −40°C | 3,00× |
| 0°C | 2,40× |
| 20°C | 2,00× |
| 70°C | 1,40× |
| 110°C | 1,15× |

**Detecção cranking:** RPM ≤ 4500 RPM (histerese saída em 7000 RPM)

**Prime pulse:** Disparado no dente-alvo (padrão: dente 3) durante cranking
- PW = REQ_FUEL × mult(CLT) + dead_time
- Consumido uma vez via `quick_crank_consume_prime()` (atômico)

**Afterstart:** 500–2400 ms pós-cranking · 1,05–1,35× · decay linear

```cpp
QuickCrankOutput quick_crank_update(uint32_t now_ms, uint32_t rpm_x10,
    bool sync_available, int16_t clt_x10, int16_t base_spark_deg) noexcept;
uint32_t quick_crank_apply_pw_us(uint32_t base_pw,
    uint16_t fuel_mult_x256, uint32_t min_pw_us) noexcept;
uint32_t quick_crank_consume_prime() noexcept; // one-shot, seção crítica
void quick_crank_set_prime_context(int16_t clt_x10, uint16_t dead_time_us);
```

### 7.10 Auxiliares (`engine/auxiliaries.h`)

**IACV (Idle Air Control):**
- Target RPM por CLT: 800–1200 RPM (mais alto quando frio)
- PID: Kp=2 · Ki=6/1000 · Kd=2,5 · integrador ±300
- PWM TIM3_CH1 @ 15 Hz

**Wastegate (Boost):**
- Target boost por RPM × TPS: 105–10.70 bar
- PID: Kp=0,08 · Ki=0,01 · integrador ±250
- Failsafe: cut se overboost > 20 bar por 500 ms
- PWM TIM3_CH2 @ 15 Hz

**VVT:**
- Intake: 180–330° × 10 · Exhaust: 60–160° × 10
- PID por atuador: Kp=1,2 · Ki=0,05
- Requer FULL_SYNC + confirmação de toggle de fase (timeout 200 ms)
- PWM TIM4_CH1/CH2 @ 15 Hz

**Fan:** CLT ≥ 95°C ON · ≤ 90°C OFF · GPIO PB12  
**Fuel Pump:** Prime 2 s no key-on · mantém enquanto RPM > 0 · GPIO PB13

```cpp
void auxiliaries_init();
void auxiliaries_set_key_on(bool key_on);
void auxiliaries_tick_10ms(); // VVT, fan, pump
void auxiliaries_tick_20ms(); // IAC, wastegate
uint16_t auxiliaries_idle_target_rpm_x10(int16_t clt_x10);
```

### 7.11 Scheduler Angular (`engine/ecu_sched.h`)

**Recursos de hardware:**
- TIM2 (32-bit): Injetores INJ1–4 via OC CH1–4 @ 10 MHz
- TIM8 (16-bit): Bobinas IGN1–4 via OC CH1–4 @ 10 MHz

**Tabela de eventos (32 slots/ciclo):**
```c
typedef struct {
    uint8_t tooth_index;    // 0–57
    uint8_t sub_frac_x256;  // fração dentro do dente (Q8)
    uint8_t channel;        // TIM channel ID
    uint8_t action;         // INJ_ON / INJ_OFF / DWELL_START / SPARK
    uint8_t phase_A;        // PHASE_A / PHASE_B / PHASE_ANY
    uint8_t valid;
} AngleEvent_t;
```

**Interface principal:**
```cpp
void ECU_Hardware_Init();
void ecu_sched_commit_calibration(uint32_t advance_deg,
    uint32_t dwell_ticks, uint32_t inj_pw_ticks,
    uint32_t eoi_lead_deg); // atômico (seção crítica)
    // EOI targeting: a injeção TERMINA em eoi_lead_deg (° BTDC);
    // SOI = EOI − PW°, com duty clamp a 648° (90% do ciclo).
void ecu_sched_set_ivc(uint8_t ivc_abdc_deg); // mantido p/ compat (clamp removido)
void ecu_sched_fire_prime_pulse(uint32_t pw_us);
```

**Diagnósticos:**
```cpp
extern volatile uint32_t g_late_event_count;         // evento atrasado
extern volatile uint32_t g_cycle_schedule_drop_count; // ciclo descartado
extern volatile uint32_t g_calibration_clamp_count;   // valor clamped
uint32_t ecu_sched_ivc_clamp_count();
```

**Modos pré-sincronismo:**
- INJ: SIMULTANEOUS (todos juntos) ou SEMI_SEQUENTIAL (pares A/B)
- IGN: WASTED_SPARK (pares 1-4, 2-3)

### 7.12 ETB Control (`engine/etb_control.h`)
```cpp
struct EtbControlState { bool active; int16_t output_pct_x10; int16_t position_error_x10; };

bool etb_control_init();   // valida calibração, retorna false se inválida
void etb_control_reset();
EtbControlState etb_control_update(uint16_t target_pct_x10,
    uint16_t measured_pct_x10, bool enable_request, uint16_t period_ms);
```

**PID com anti-windup:**
- Kp, Ki, Kd configuráveis por NVM (etb_kp_x10, etb_ki_x10, etb_kd_x10)
- Integrador: clamp ±2000 · anti-windup condicional na saturação
- D-term: derivada filtrada (`filtered_deriv`)

### 7.13 Torque Manager (`engine/torque_manager.h`)
```cpp
struct TorqueOutput {
    uint16_t etb_target_pct_x10;
    uint16_t etb_max_rate_pct_per_s;
    uint8_t  limp_reason;         // bitmask
    bool     etb_enable_request;
};

bool        torque_manager_init();
void        torque_manager_reset();
TorqueOutput torque_manager_update(const CkpSnapshot& snap,
    const SensorData& sensors, bool key_on,
    bool map_clt_limp, bool rev_cut,
    uint16_t idle_target_rpm_x10, uint16_t period_ms);
```

**Razões de limp (bitmask):**
```
bit 0: TORQUE_LIMP_MAP_CLT    — MAP/CLT fault
bit 1: TORQUE_LIMP_APP_FAULT  — APP plausibility
bit 2: TORQUE_LIMP_ETB_FAULT  — ETB TPS fault
bit 3: TORQUE_LIMP_NO_CALIB   — calibração inválida
bit 4: TORQUE_LIMP_REV_CUT    — corte de rotação
```

### 7.14 Map Estimator (`engine/map_estimator.h`)
```cpp
void    map_estimator_init();
uint16_t map_estimator_update(uint16_t map_sensor_bar_x100, uint16_t tps_pct_x10,
    uint16_t dt_ms, uint32_t rpm_x10); // retorna MAP filtrado
uint16_t map_get_estimated_bar_x100();
int16_t  map_get_tpsdot_x10();    // derivada TPS (%/s × 10)
bool     map_is_transient();
```

**Filtro complementar:**
- Estado estável (|TPSdot| < 5 %/s): 78% sensor + 22% modelo
- Transiente leve (5–15 %/s): 50/50
- Transiente pesado (> 30 %/s): 25% sensor + 75% modelo

### 7.15 Diagnostic Manager (`engine/diagnostic_manager.h`)

**Severidades:** INFO → WARNING → ERROR → CRITICAL  
**Estados de recovery:** IDLE → DETECTED → RECOVERING → RECOVERED  

```cpp
// Classe estática
DiagnosticManager::init();
DiagnosticManager::report_fault(DiagnosticCode code,
    FaultSeverity severity, uint16_t p1, uint16_t p2);
DiagnosticManager::is_fault_active(DiagnosticCode code);
DiagnosticManager::get_highest_severity();
DiagnosticManager::is_system_ready(); // sem falhas CRITICAL
```

**Códigos principais:**
| Código | Descrição |
|---|---|
| P0100–P0199 | Sensores (range, plausibilidade) |
| P0300–P0303 | Misfire por cilindro |
| P0171/P0172 | STFT lean/rich |
| P0500–P0599 | Elétrico (VBatt, ADC, Flash) |
| P0200 | Engine protection (temp, overspeed, oil) |

---

## 8. APP — Camada de Aplicação

### 8.1 CAN Stack (`app/can_stack.h`)

**TX Frames:**
| ID | Cadência | Payload (8 bytes) |
|---|---|---|
| 0x400 | 10 ms | RPM(2LE) · MAP_kpa(1) · TPS_pct(1) · CLT+40(1) · advance+40(1) · PW_x10(1) · StatusBits[7:0](1) |
| 0x401 | 100 ms | FuelP_kpa(2LE) · OilP_kpa(2LE) · IAT+40(1) · STFT+100(1) · StatusBits[15:8](1) · VVT_ex_pct(1) |
| 0x402 | 500 ms | FuelAccum_ul(4LE) · FuelDelta_ul(2LE) · reservado(2) |

**RX — WBO2 (padrão ID 0x180):**
- `data[0:1]` = lambda × 1000 (little-endian)
- `data[2]` = status byte do sensor
- Timeout: 500 ms → fallback λ = 1,050 (WBO2_SAFE_LAMBDA_MILLI)

```cpp
void     can_stack_init(uint16_t wbo2_rx_id = 0x180u);
void     can_stack_process(uint32_t now_ms, const CkpSnapshot&,
    const SensorData&, int8_t advance_deg, uint8_t pw_ms_x10,
    int8_t stft_pct, uint8_t vvt_in_pct, uint8_t vvt_ex_pct,
    uint16_t status_bits);
uint16_t can_stack_lambda_milli_safe(uint32_t now_ms); // fallback 1050
bool     can_stack_wbo2_fresh(uint32_t now_ms);         // < 500 ms
bool     can_stack_wbo2_fault();
```

### 8.2 UI Protocol (`app/ui_protocol.h`)

**Transporte:** UART1 115200 baud · USB CDC (paralelo)

**Páginas:**
| ID | Tamanho | Conteúdo |
|---|---|---|
| 0x00 | 512 B | Config geral (byte 0 = IVC ABDC) + ETB cal (offset 0–33) + X-tau cal |
| 0x01 | 256 B | Tabela VE (16×16 uint8) |
| 0x02 | 256 B | Tabela Spark (16×16 int8) |
| 0x03 | 64 B | Realtime (somente leitura) |
| 0x04 | 512 B | Tabela lambda_target ×1000 (16×16 int16) |
| 0x05 | 256 B | Curvas de correção (CLT, IAT, warmup, dwell, AE, idle spark) |
| 0x06 | 80 B | X-tau (xtau_x_fraction, xtau_tau, AE rate, quick crank) |

**Página 3 — Realtime (64 B):**
```
[0:1]   RPM
[2]     MAP bar
[3]     TPS %
[4]     CLT + 40°C
[5]     IAT + 40°C
[6]     lambda_milli / 4
[7]     PW × 10
[8]     advance + 40°
[9]     VE
[10]    STFT + 100
[11:12] StatusBits
[13:]   Diagnósticos do scheduler (late_events, drops, clamps, seeds)
```

**Protocolo:**
```
Comandos IDLE:
  'Q' / 'H' → "OpenEMS_v1.1"
  'S'        → "OpenEMS_fw_1.1"
  'F'        → "001"
  'C'        → 0x00 0xAA (teste de comms)
  'A' / 'O'  → página realtime (64 B)
  'r'        → leitura parcial: 5 bytes args (page, off_lo, off_hi, len_lo, len_hi)
  'w'        → escrita + flash: 5 bytes args + len bytes dados
  'x'        → escrita somente RAM
  'b'        → burn (flush para Flash): 1 byte page ID
  'd'        → dirty mask (1 byte bitmask de páginas modificadas)
```

**Status bits:**
```
bit 0: SYNC_FULL       bit 5: XTAU_LEARN
bit 1: PHASE_A         bit 6: SCHED_LATE
bit 2: SENSOR_FAULT    bit 7: SCHED_DROP
bit 3: LIMP_MODE       bit 8: SCHED_CLAMP
bit 4: ETB_LIMP        bit 9: WBO2_FAULT
```

---

## 9. Loop Principal (`src/main_stm32.cpp`)

### Sequência de Inicialização
1. `system_stm32_init()` — PLL 250 MHz · SysTick 1 ms · IWDG 100 ms
2. `tim5_ic_init()` — CKP input capture
3. `ECU_Hardware_Init()` — TIM2/TIM8 output compare
4. `adc_init()` — ADC1/ADC2 via TIM6
5. `can0_init()` + `uart0_init(115200)` + `usb_cdc_init()`
6. NVM: calibração (páginas 0, 4, 5) + mapas adaptativos + runtime seed
7. `sensors_init()` + `map_estimator_init()` + `xtau_autocalib_init()`
8. `etb_control_init()` + `torque_manager_init()`
9. `fuel_reset_adaptives()` + `auxiliaries_init()` + `knock_init()` + `quick_crank_reset()`
10. `ui_init()` + `can_stack_init()`
11. NVIC: TIM5 prio 1 (máxima) · SysTick prio 11
12. Aguarda FULL_SYNC ou timeout 5 s

### Slots Temporais do Loop
| Slot | Cadência | Função |
|---|---|---|
| **2ms fuel/ign** | 2 ms | CKP snapshot · Cálculo VE/lambda/PW/advance · X-tau · Quick crank · ecu_sched_commit · Prime pulse |
| **2ms ETB** | 2 ms | torque_manager_update · etb_control_update · sensors (APP/ETB) |
| **10ms** | 10 ms | auxiliaries_tick_10ms (VVT, fan, pump) |
| **20ms** | 20 ms | UI service · CAN TX · auxiliaries_tick_20ms |
| **50ms** | 50 ms | sensors_tick_50ms (fuel/oil pressure) |
| **100ms** | 100 ms | sensors_tick_100ms · STFT update · X-tau autocal tick · Runtime seed save |
| **500ms** | 500 ms | Flush calibração (mín. 300 s intervalo) · nvm_flush_adaptive_maps |

### Fluxo de Dados no Slot de 2ms
```
CKP snapshot
    ↓ rpm_x10, map_bar_x100, sched_sync, full_sync
table3d_prepare_lookup(rpm_x10, map_bar_x100)
    ↓ fuel_lookup (xi, yi, fx, fy)
get_ve_prepared(fuel_lookup)        → ve
get_lambda_target_x1000_prepared()  → lambda_target_x1000
fuel_get_stft_pct_x10()             → stft
fuel_get_ltft_pct_x10()             → ltft
calc_ae_pw_us(tps_now, tps_prev)    → ae_pw_us
calc_fuel_pw_us_default_fast(...)   → final_pw_us_base
transient_fuel_xtau_update(...)     → xtau_pw_us
get_advance_prepared(fuel_lookup)   → base_advance_deg
knock_get_retard_x10()              → knock_retard
calc_idle_spark_correction_deg()    → idle_corr
calc_total_advance(base, corr)      → advance_deg
quick_crank_update(...)             → qc.fuel_mult, qc.spark_deg
quick_crank_apply_pw_us(...)        → final_pw_us
    ↓
ecu_sched_commit_calibration(advance, dwell_ticks, inj_pw_ticks, eoi_lead)
```

### Limp Mode e Rev Cut
- **Limp ativo:** `sensors.fault_bits & (MAP_BIT | CLT_BIT) != 0`
- **Rev cut:** limp_active && RPM > 30.000 × 10 → PW = 0
- **ETB limp:** throttle_fault_bits → `torque_manager` limita target a 25%

---

## 10. Startup STM32H562 (`src/startup_stm32h562.cpp`)

1. Copia vetor IRQ Flash → SRAM (evita freeze 120 µs em operação Flash)
2. Copia `.data` (Flash → RAM)
3. Zera `.bss`
4. `__libc_init_array()` — construtores globais
5. `main()`

**Default_Handler:** SYSRESETREQ via SCB_AIRCR antes de loop infinito  
**Vetor:** 120 entradas (Cortex-M33 + STM32H562 IRQs 0–69)

---

## 11. Build System

**Toolchain:** `arm-none-eabi-g++` · C++17 · `-O2 -fno-exceptions -fno-rtti`

**Targets:**
```bash
make firmware   # → /tmp/openems-build/bin/openems.{hex,bin,elf}
make clean      # remove /tmp/openems-build
```

**Defines:**
- `TARGET_STM32H562` — firmware real
- `EMS_HOST_TEST` — build host para testes (sem hardware)

---

## 12. NVM — Layout de Persistência

| Slot | Endereço Flash | Conteúdo | Tamanho |
|---|---|---|---|
| Setor 0 | 0x08100000 | LTFT 256B (16×16 int8, off.0) + Knock 64B (8×8 int8, off.256) + LTFT_add 64B (8×8 int8, off.320) + RuntimeSeed 32B (off.512) | 8 KB |
| Setor 1 | 0x08102000 | Calibração página 0 (config + ETB + IVC) | 512 B |
| Setor 2 | 0x08104000 | Calibração página 1 (VE table) | 512 B |
| Setor 3 | 0x08106000 | Calibração página 2 (Spark table) | 512 B |
| Setor 4 | 0x08108000 | Calibração página 3 (reservada) | 512 B |
| Setor 5 | 0x0810A000 | Calibração página 4 (correções) | 512 B |
| Setor 6 | 0x0810C000 | Calibração página 5 (X-tau) | 512 B |
| Setor 7 | 0x0810E000 | Calibração página 6 (lambda target) | 512 B |

**Regras de escrita:**
- Calibração: mínimo 300 s entre salvamentos
- Mapas adaptativos: flush diferido se RPM > kFlashWriteSafeRpmX10
- Runtime seed: salvo 100 ms após RPM cair para zero · apagado no próximo boot

---

## 13. Convenções de Unidades

| Sufixo | Significado | Exemplo |
|---|---|---|
| `_x10` | × 10 (deci) | `rpm_x10 = 30000` → 3000 RPM |
| `_x100` | × 100 (centi) | `maf_gps_x100` |
| `_x1000` | × 1000 (milli) | `lambda_x1000 = 1000` → λ 1,000 |
| `_x256` / `_q8` | Q8 fixo (÷ 256) | `corr_clt_x256 = 288` → 1,125 |
| `_pct_x10` | % × 10 | `tps_pct_x10 = 500` → 50,0 % |
| `_p40` | + 40 offset (wire encoding) | `clt_p40 = 65` → 25°C |
| `_ms_x10` | ms × 10 | `dwell_ms_x10 = 30` → 3,0 ms |
| `_deg` | graus inteiros | `advance_deg = 20` → 20° BTDC |
| `_mv` | milivolts | `vbatt_mv = 14200` → 14,2 V |
