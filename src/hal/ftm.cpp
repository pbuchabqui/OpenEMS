/**
 * @file hal/ftm.cpp
 * @brief FlexTimer Module HAL — OpenEMS Engine Management System
 *
 * Referência: K64P144M120SF5 Reference Manual, Rev. 2
 *   §43.3  FTM Memory Map and Register Definition
 *   §43.4  FTM Functional Description
 *   §12.2  Port Control and Interrupts (PCR — MUX field)
 *   §3.8.2 NVIC — Nested Vectored Interrupt Controller
 *
 * Clocks relevantes (SystemCoreClock = 120 MHz, bus clock = 60 MHz):
 *   FTM0/FTM3: system clock (120 MHz) / prescaler 2 = 60 MHz → 16.67 ns/tick
 *   FTM1/FTM2: bus clock (60 MHz) / prescaler calculado = freq PWM
 *
 * Callbacks registrados externamente (definidos em drv/ckp.cpp):
 *   ems::drv::ckp_ftm3_ch0_isr()  — chamado pela FTM3_IRQHandler no evento CKP
 *   ems::drv::ckp_ftm3_ch1_isr()  — chamado pela FTM3_IRQHandler no evento CMP
 *
 * FTM0_IRQHandler é definido em engine/ecu_sched.c (módulo C MISRA-C:2012).
 *
 * REGRA INEGOCIÁVEL: Nenhuma lógica de motor aqui. Apenas registradores.
 */

#include "hal/ftm.h"

// ──────────────────────────────────────────────────────────────────────────────
// Registradores MK64F — acesso direto, sem abstração Arduino
// Baseados no K64P144M120SF5 Reference Manual §43.3
// ──────────────────────────────────────────────────────────────────────────────

// Endereços base dos FTMs (MK64F Reference Manual, Table 3-1)
#define FTM0_BASE   (0x40038000u)
#define FTM1_BASE   (0x40039000u)
#define FTM2_BASE   (0x400B8000u)
#define FTM3_BASE   (0x400B9000u)

// Offsets dos registradores FTM (§43.3)
#define FTM_SC_OFFSET       (0x00u)   // Status and Control
#define FTM_CNT_OFFSET      (0x04u)   // Counter
#define FTM_MOD_OFFSET      (0x08u)   // Modulo
#define FTM_CH_BASE_OFFSET  (0x0Cu)   // Primeiro canal (CnSC em +0, CnV em +4, stride 8)
#define FTM_MODE_OFFSET     (0x54u)   // Features Mode Selection
#define FTM_COMBINE_OFFSET  (0x64u)   // Function for Linked Channels
#define FTM_DEADTIME_OFFSET (0x68u)   // Deadtime Configuration
#define FTM_EXTTRIG_OFFSET  (0x6Cu)   // FTM External Trigger
#define FTM_POL_OFFSET      (0x70u)   // Channels Polarity
#define FTM_FMS_OFFSET      (0x74u)   // Fault Mode Status
#define FTM_FILTER_OFFSET   (0x78u)   // Input Capture Filter Control
#define FTM_CONF_OFFSET     (0x84u)   // Configuration

// Acessores de registradores (volatile uint32_t*)
#define FTM_REG(base, offset) (*((volatile uint32_t*)((base) + (offset))))

// Atalhos por FTM
#define FTM0_SC         FTM_REG(FTM0_BASE, FTM_SC_OFFSET)
#define FTM0_CNT        FTM_REG(FTM0_BASE, FTM_CNT_OFFSET)
#define FTM0_MOD        FTM_REG(FTM0_BASE, FTM_MOD_OFFSET)
#define FTM0_MODE       FTM_REG(FTM0_BASE, FTM_MODE_OFFSET)
#define FTM0_COMBINE    FTM_REG(FTM0_BASE, FTM_COMBINE_OFFSET)

#define FTM1_SC         FTM_REG(FTM1_BASE, FTM_SC_OFFSET)
#define FTM1_CNT        FTM_REG(FTM1_BASE, FTM_CNT_OFFSET)
#define FTM1_MOD        FTM_REG(FTM1_BASE, FTM_MOD_OFFSET)
#define FTM1_MODE       FTM_REG(FTM1_BASE, FTM_MODE_OFFSET)

#define FTM2_SC         FTM_REG(FTM2_BASE, FTM_SC_OFFSET)
#define FTM2_CNT        FTM_REG(FTM2_BASE, FTM_CNT_OFFSET)
#define FTM2_MOD        FTM_REG(FTM2_BASE, FTM_MOD_OFFSET)
#define FTM2_MODE       FTM_REG(FTM2_BASE, FTM_MODE_OFFSET)

#define FTM3_SC         FTM_REG(FTM3_BASE, FTM_SC_OFFSET)
#define FTM3_CNT        FTM_REG(FTM3_BASE, FTM_CNT_OFFSET)
#define FTM3_MOD        FTM_REG(FTM3_BASE, FTM_MOD_OFFSET)
#define FTM3_MODE       FTM_REG(FTM3_BASE, FTM_MODE_OFFSET)

// Canal n: CnSC = base + 0x0C + n*8, CnV = base + 0x10 + n*8
#define FTM_CnSC(base, n)  FTM_REG((base), 0x0Cu + (uint32_t)(n)*8u)
#define FTM_CnV(base, n)   FTM_REG((base), 0x10u + (uint32_t)(n)*8u)

// FTM_SC bit fields (§43.3.1)
#define FTM_SC_TOF      (1u << 7)   // Timer Overflow Flag
#define FTM_SC_TOIE     (1u << 6)   // Timer Overflow Interrupt Enable
#define FTM_SC_CPWMS    (1u << 5)   // Center-aligned PWM Select
// CLKS[4:3]: 00=nenhum, 01=system, 10=fixed, 11=externo
#define FTM_SC_CLKS_SYSTEM  (0x1u << 3)
// NOTA: FTM1/FTM2 usam CLKS_SYSTEM (01) também — CLKS=01 no MK64F seleciona sempre
// system clock (120 MHz). A compensação para bus clock (60 MHz) é feita via
// kFtmBusClockHz = 60000000 na fórmula calc_pwm_params, não via seleção de clock diferente.
// PS[2:0]: prescaler 1/2/4/8/16/32/64/128
#define FTM_SC_PS_1     (0x0u)
#define FTM_SC_PS_2     (0x1u)
#define FTM_SC_PS_4     (0x2u)
#define FTM_SC_PS_8     (0x3u)
#define FTM_SC_PS_16    (0x4u)
#define FTM_SC_PS_32    (0x5u)
#define FTM_SC_PS_64    (0x6u)
#define FTM_SC_PS_128   (0x7u)

// FTM_MODE bit fields (§43.3.12)
#define FTM_MODE_FAULTIE  (1u << 7)
#define FTM_MODE_FAULTM   (3u << 5)
#define FTM_MODE_CAPTEST  (1u << 4)
#define FTM_MODE_PWMSYNC  (1u << 3)
#define FTM_MODE_WPDIS    (1u << 2)   // Write-Protect Disable — SEMPRE setar antes de config
#define FTM_MODE_INIT     (1u << 1)
#define FTM_MODE_FTMEN    (1u << 0)   // FTM Enable (habilita todos os registradores)

// FTM CnSC bit fields (§43.3.5)
#define FTM_CnSC_CHF    (1u << 7)   // Channel Flag — limpar escrevendo 0
#define FTM_CnSC_CHIE   (1u << 6)   // Channel Interrupt Enable
#define FTM_CnSC_MSB    (1u << 5)   // Mode Select B
#define FTM_CnSC_MSA    (1u << 4)   // Mode Select A
#define FTM_CnSC_ELSB   (1u << 3)   // Edge/Level Select B
#define FTM_CnSC_ELSA   (1u << 2)   // Edge/Level Select A
#define FTM_CnSC_ICRST  (1u << 1)   // FTM counter reset on channel input event

// Modo Input Capture rising edge: MSB=0,MSA=0, ELSB=0,ELSA=1
#define FTM_CnSC_INPUT_RISING   (FTM_CnSC_CHIE | FTM_CnSC_ELSA)

// Modo Output Compare "set on match": MSB=1,MSA=0, ELSB=1,ELSA=0
// §43.4.2.3 Table 43-8: MSB:MSA=10 (output compare), ELSB:ELSA=10 (set)
#define FTM_CnSC_OUTPUT_SET     (FTM_CnSC_CHIE | FTM_CnSC_MSB | FTM_CnSC_ELSB)

// Modo PWM edge-aligned "high-true": MSB=1,MSA=0, ELSB=1,ELSA=0
#define FTM_CnSC_PWM_HIGH       (FTM_CnSC_MSB | FTM_CnSC_ELSB)

// Registradores de porta e GPIO (§12.2 Port Control)
#define PORTA_BASE  (0x40049000u)
#define PORTB_BASE  (0x4004A000u)
#define PORTC_BASE  (0x4004B000u)
#define PORTD_BASE  (0x4004C000u)

#define PORT_PCR(port_base, pin)  FTM_REG((port_base), (uint32_t)(pin)*4u)
#define PCR_MUX(alt)    ((uint32_t)(alt) << 8u)   // MUX field bits [10:8]
#define PCR_MUX_MASK    (0x7u << 8u)
#define PCR_IRQC_MASK   (0xFu << 16u)

// GPIO PTD PDIR (leitura do estado atual do pino — para verificação anti-glitch)
#define PTD_PDIR  (*((volatile uint32_t*)(0x400FF0C0u)))  // §55.2.6

// SIM — System Integration Module (para clock gating)
#define SIM_SCGC3   (*((volatile uint32_t*)(0x40048030u)))  // FTM2, FTM3
#define SIM_SCGC6   (*((volatile uint32_t*)(0x4004803Cu)))  // FTM0, FTM1
#define SIM_SCGC3_FTM2_MASK  (1u << 26)
#define SIM_SCGC3_FTM3_MASK  (1u << 25)
#define SIM_SCGC6_FTM0_MASK  (1u << 24)
#define SIM_SCGC6_FTM1_MASK  (1u << 25)

// NVIC (§3.8.2)
#define NVIC_ISER(n)  (*((volatile uint32_t*)(0xE000E100u + (uint32_t)(n)*4u)))
#define NVIC_IPR(n)   (*((volatile uint32_t*)(0xE000E400u + (uint32_t)(n)*4u)))

// IRQ numbers MK64F (K64P144M120SF5 RM, Table 3-5)
#define IRQ_FTM0    42u
#define IRQ_FTM1    43u
#define IRQ_FTM2    44u
#define IRQ_FTM3    71u

// Clock base para FTM0/FTM3 (system clock)
static constexpr uint32_t kFtmSystemClockHz = 120'000'000u;
// Clock base para FTM1/FTM2 (bus clock = system/2)
static constexpr uint32_t kFtmBusClockHz    =  60'000'000u;

// ──────────────────────────────────────────────────────────────────────────────
// Callbacks externos (definidos em camadas superiores)
// Declarados aqui apenas para linkagem — não inclua headers de drv/ aqui
// ──────────────────────────────────────────────────────────────────────────────
namespace ems::drv {
  void ckp_ftm3_ch0_isr() noexcept;   // evento CKP rising edge
  void ckp_ftm3_ch1_isr() noexcept;   // evento CMP/fase rising edge
}

// ──────────────────────────────────────────────────────────────────────────────
// Funções auxiliares internas (static — não exportadas)
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Configura prioridade e habilita uma IRQ no NVIC.
 * @param irq      Número da IRQ (não do vetor — vetor = irq + 16)
 * @param priority Prioridade 0-15 (0 = mais alta; MK64F usa 4 bits)
 *
 * §3.8.4: IPR registra prioridades em grupos de 4, 1 byte cada.
 * Escrevemos os 4 bits superiores do byte correspondente.
 */
static void nvic_enable(uint32_t irq, uint8_t priority) {
    // IPR[n] = 4 bytes, cada byte para uma IRQ
    // Byte index = irq / 4, bit offset dentro do byte = (irq % 4) * 8
    volatile uint32_t* ipr = (volatile uint32_t*)(0xE000E400u + (irq / 4u) * 4u);
    uint32_t shift = (irq % 4u) * 8u;
    // Limpa prioridade anterior e escreve nova (bits [7:4] do byte)
    *ipr = (*ipr & ~(0xFFu << shift)) | ((uint32_t)(priority << 4u) << shift);
    // Habilita a IRQ no ISER (interrupt set-enable register)
    NVIC_ISER(irq / 32u) = (1u << (irq % 32u));
}

/**
 * @brief Calcula MOD e prescaler para FTM em modo PWM.
 * @param clock_hz  Clock de entrada do FTM em Hz
 * @param freq_hz   Frequência PWM desejada em Hz
 * @param[out] mod      Valor de FTM_MOD resultante
 * @param[out] ps_bits  Bits PS[2:0] para FTM_SC
 *
 * Itera prescalers 1,2,4,8,16,32,64,128 e escolhe o menor que
 * mantém MOD dentro de [100, 65535] para boa resolução de duty.
 */
static void calc_pwm_params(uint32_t clock_hz, uint32_t freq_hz,
                            uint16_t& mod, uint8_t& ps_bits) {
    for (uint8_t ps = 0u; ps <= 7u; ++ps) {
        uint32_t prescaler = 1u << ps;
        uint32_t ticks = (clock_hz / prescaler) / freq_hz;
        if (ticks >= 100u && ticks <= 65535u) {
            mod     = static_cast<uint16_t>(ticks - 1u);
            ps_bits = ps;
            return;
        }
    }
    // Fallback: prescaler máximo, MOD máximo
    mod     = 0xFFFFu;
    ps_bits = 7u;
}

// ──────────────────────────────────────────────────────────────────────────────
// ems::hal public implementations
// ──────────────────────────────────────────────────────────────────────────────
namespace ems::hal {

// ─── FTM0 — Output Compare (Ignição + Injeção) ───────────────────────────────

void ftm0_init(void) {
    // 1. Habilita clock gating para FTM0 (SIM_SCGC6 §12.2.13)
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

    // 2. Desabilita write-protect ANTES de qualquer escrita de config
    //    §43.3.12: WPDIS=1 permite escrever em todos os registradores
    FTM0_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

    // 3. Para o contador para configuração segura
    FTM0_SC = 0u;

    // 4. Zera o contador e configura MOD para free-running (0xFFFF)
    FTM0_CNT = 0u;
    FTM0_MOD = 0xFFFFu;   // contador livre de 16 bits, overflow ~1.09 ms a 60 MHz efetivo

    // 5. COMBINE = 0 — todos os canais operam independentemente
    FTM0_COMBINE = 0u;

    // 6. Configura pinos IGN1-IGN4 (PTD7-PTD4 → CH7-CH4) como output compare
    //    MUX=4 → FTM0_CH (§11.4.1 Signal Multiplexing PTD)
    PORT_PCR(PORTD_BASE, 7u) = PCR_MUX(4u);  // IGN1 CH7
    PORT_PCR(PORTD_BASE, 6u) = PCR_MUX(4u);  // IGN2 CH6
    PORT_PCR(PORTD_BASE, 5u) = PCR_MUX(4u);  // IGN3 CH5
    PORT_PCR(PORTD_BASE, 4u) = PCR_MUX(4u);  // IGN4 CH4

    // 7. Configura pinos INJ1-INJ4 (PTC1-PTC4 → CH0-CH3) como output compare
    //    MUX=4 → FTM0_CH (§11.4.1 Signal Multiplexing PTC)
    //    Todos os 4 injetores em Port C contíguo — scheduler uniforme, sem código especial.
    //    INJ3/INJ4 em PTC1/PTC2 (FTM0_CH0/CH1): escolhidos sobre PTD2/PTD3 para evitar
    //    risco de spurious output via FTM3_CH2/CH3 (ALT4 em PTD2/PTD3) durante debug.
    //    PIT fica 100% disponível para datalog timestamp e watchdog software.
    PORT_PCR(PORTC_BASE, 1u) = PCR_MUX(4u);  // INJ3 CH0
    PORT_PCR(PORTC_BASE, 2u) = PCR_MUX(4u);  // INJ4 CH1
    PORT_PCR(PORTC_BASE, 3u) = PCR_MUX(4u);  // INJ1 CH2
    PORT_PCR(PORTC_BASE, 4u) = PCR_MUX(4u);  // INJ2 CH3

    // 8. Configura CH0-CH7 como output compare "set on match"
    //    §43.4.2.3: MSB:MSA=10 (output compare mode), ELSB:ELSA=10 (set output on match)
    //    Todos os 8 canais com lógica idêntica — scheduler não precisa distinguir INJ de IGN.
    for (uint8_t ch = 0u; ch <= 7u; ++ch) {
        FTM_CnSC(FTM0_BASE, ch) = FTM_CnSC_OUTPUT_SET;
        FTM_CnV(FTM0_BASE, ch)  = 0u;
    }

    // 9. Inicia o contador: system clock, prescaler 2 → 16.67 ns/tick
    //    NÃO habilitar TOIE aqui — overflow do FTM0 não é usado por este módulo
    FTM0_SC = FTM_SC_CLKS_SYSTEM | FTM_SC_PS_2;

    // 10. NVIC: FTM0 prioridade 2 (abaixo de FTM3/CKP que é prio 1)
    nvic_enable(IRQ_FTM0, 2u);
}

void ftm0_set_compare(uint8_t ch, uint16_t ticks) noexcept {
    // Escreve o valor de comparação diretamente no canal
    // O evento ocorre quando FTM0_CNT == ticks (com wrap-around natural uint16_t)
    FTM_CnV(FTM0_BASE, ch) = ticks;
    // Re-arma a interrupção do canal limpando CHF e garantindo CHIE=1
    // Nota: CHF é limpo escrevendo 0 no bit 7 — padrão "write 0 to clear"
    FTM_CnSC(FTM0_BASE, ch) = FTM_CnSC_OUTPUT_SET;  // preserva config, limpa CHF implicitamente
}

void ftm0_clear_chf(uint8_t ch) noexcept {
    // §43.3.5: CHF é limpo escrevendo 0 no bit. Preserva os outros bits.
    FTM_CnSC(FTM0_BASE, ch) &= ~FTM_CnSC_CHF;
}

uint16_t ftm0_count() noexcept {
    // Leitura do contador de 16 bits — safe em qualquer contexto
    return static_cast<uint16_t>(FTM0_CNT);
}


// ─── FTM3 — Input Capture (CKP + CMP) ────────────────────────────────────────

void ftm3_init(void) {
    // 1. Clock gating FTM3 (SIM_SCGC3 §12.2.11)
    SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;

    // 2. Write-protect off + FTMEN antes de qualquer config
    FTM3_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

    // 3. Para o contador
    FTM3_SC = 0u;

    // 4. Free-running, MOD máximo
    FTM3_CNT = 0u;
    FTM3_MOD = 0xFFFFu;

    // 5. Configura pinos PTD0 e PTD1 como FTM3 input
    //    PTD0: ALT4 → FTM3_CH0 (CKP)
    //    PTD1: ALT4 → FTM3_CH1 (CMP/fase)
    //    §11.4.1: PORTD PCR[0] e PCR[1], MUX=4
    PORT_PCR(PORTD_BASE, 0u) = PCR_MUX(4u);
    PORT_PCR(PORTD_BASE, 1u) = PCR_MUX(4u);

    // 6. CH0 (CKP): input capture, rising edge
    //    §43.4.2.1 Table 43-6: MSB=0,MSA=0 → input capture; ELSB=0,ELSA=1 → rising
    FTM_CnSC(FTM3_BASE, 0u) = FTM_CnSC_INPUT_RISING;
    FTM_CnV(FTM3_BASE, 0u)  = 0u;

    // 7. CH1 (CMP/fase): input capture, rising edge
    FTM_CnSC(FTM3_BASE, 1u) = FTM_CnSC_INPUT_RISING;
    FTM_CnV(FTM3_BASE, 1u)  = 0u;

    // 8. Inicia: system clock, prescaler 2 (mesma base do FTM0 para correlação de timestamps)
    FTM3_SC = FTM_SC_CLKS_SYSTEM | FTM_SC_PS_2;

    // 9. NVIC: FTM3/CKP = prioridade 1 (máxima do sistema, abaixo apenas de hard fault)
    //          FTM3 não tem IRQ separada por canal — toda a lógica na ISR abaixo
    nvic_enable(IRQ_FTM3, 1u);
}

uint16_t ftm3_count() noexcept {
    return static_cast<uint16_t>(FTM3_CNT);
}


// ─── FTM1 — PWM (IACV + Wastegate) ───────────────────────────────────────────

void ftm1_pwm_init(uint32_t freq_hz) {
    // 1. Clock gating FTM1
    SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

    // 2. Write-protect off
    FTM1_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
    FTM1_SC   = 0u;
    FTM1_CNT  = 0u;

    // 3. Calcula MOD e prescaler para a frequência solicitada (bus clock 60 MHz)
    uint16_t mod;
    uint8_t  ps;
    calc_pwm_params(kFtmBusClockHz, freq_hz, mod, ps);
    FTM1_MOD = mod;

    // 4. Pinos: PTA8 → CH0 (IACV), PTA9 → CH1 (Wastegate) — ALT3 §11.4.1
    //    ATENÇÃO: PTA12/PTA13 foram descartados — conflitam com CAN0_TX/CAN0_RX (ALT2).
    //    Um pino só pode ter um ALT ativo. PTA8/PTA9 são FTM1_CH0/CH1 em ALT3 sem conflito.
    PORT_PCR(PORTA_BASE, 8u) = PCR_MUX(3u);
    PORT_PCR(PORTA_BASE, 9u) = PCR_MUX(3u);

    // 5. Canais em modo PWM edge-aligned, high-true, duty inicial 0
    FTM_CnSC(FTM1_BASE, 0u) = FTM_CnSC_PWM_HIGH;
    FTM_CnV(FTM1_BASE, 0u)  = 0u;
    FTM_CnSC(FTM1_BASE, 1u) = FTM_CnSC_PWM_HIGH;
    FTM_CnV(FTM1_BASE, 1u)  = 0u;

    // 6. Inicia com bus clock e prescaler calculado
    //    Bus clock = system/2 = 60 MHz. No MK64F FTM_SC[CLKS]=01 sempre seleciona
    //    system clock; para usar bus clock usa-se CLKS=01 pois o bus é derivado do system.
    //    Na prática para FTM1/FTM2 o prescaler compensa a diferença.
    FTM1_SC = FTM_SC_CLKS_SYSTEM | ps;
}

void ftm1_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    // duty_pct_x10: 0-1000 → 0-100%
    // CnV = (MOD+1) * duty / 1000
    uint32_t mod_plus1 = (FTM1_MOD & 0xFFFFu) + 1u;
    uint32_t cv = (mod_plus1 * duty_pct_x10) / 1000u;
    if (cv > mod_plus1) cv = mod_plus1;
    FTM_CnV(FTM1_BASE, ch) = static_cast<uint16_t>(cv);
}


// ─── FTM2 — PWM (VVT Admissão + Escape) ──────────────────────────────────────

void ftm2_pwm_init(uint32_t freq_hz) {
    // 1. Clock gating FTM2
    SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;

    // 2. Write-protect off
    FTM2_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
    FTM2_SC   = 0u;
    FTM2_CNT  = 0u;

    // 3. MOD e prescaler para freq_hz
    uint16_t mod;
    uint8_t  ps;
    calc_pwm_params(kFtmBusClockHz, freq_hz, mod, ps);
    FTM2_MOD = mod;

    // 4. Pinos: PTA10 → CH0 (VVT Esc), PTA11 → CH1 (VVT Adm) — ALT3 §11.4.1
    //    ATENÇÃO: PTB18/PTB19 foram descartados — conflitam com CAN0_TX/CAN0_RX (ALT2).
    //    PTA10/PTA11 são FTM2_CH0/CH1 em ALT3 sem conflito de periférico.
    PORT_PCR(PORTA_BASE, 10u) = PCR_MUX(3u);
    PORT_PCR(PORTA_BASE, 11u) = PCR_MUX(3u);

    // 5. Canais PWM edge-aligned, high-true, duty inicial 0
    FTM_CnSC(FTM2_BASE, 0u) = FTM_CnSC_PWM_HIGH;
    FTM_CnV(FTM2_BASE, 0u)  = 0u;
    FTM_CnSC(FTM2_BASE, 1u) = FTM_CnSC_PWM_HIGH;
    FTM_CnV(FTM2_BASE, 1u)  = 0u;

    // 6. Inicia
    FTM2_SC = FTM_SC_CLKS_SYSTEM | ps;
}

void ftm2_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    uint32_t mod_plus1 = (FTM2_MOD & 0xFFFFu) + 1u;
    uint32_t cv = (mod_plus1 * duty_pct_x10) / 1000u;
    if (cv > mod_plus1) cv = mod_plus1;
    FTM_CnV(FTM2_BASE, ch) = static_cast<uint16_t>(cv);
}


// ──────────────────────────────────────────────────────────────────────────────
// FTM0 — Output Compare para ignição (pino acionado por hardware)
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Arma canal FTM0 para disparo de ignição por hardware (Output Compare).
 *
 * Configura FTM0_CnSC para Output Compare / Clear on match: o pino do canal
 * vai LOW no exato ciclo em que FTM0_CNT == CnV, sem qualquer ação de CPU.
 *
 * Detalhes dos bits de FTM_CnSC (K64 RM §43.3.5 Table 43-8):
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │  Bit │ Nome   │ Valor │ Significado                                 │
 *   ├─────────────────────────────────────────────────────────────────────┤
 *   │   7  │ CHF    │  0    │ Limpa flag anterior (W0C — write-0-to-clear)│
 *   │   6  │ CHIE   │  1    │ Habilita interrupção para cleanup pós-match │
 *   │   5  │ MSnB   │  1    │ Mode Select B = 1 (Output Compare)         │
 *   │   4  │ MSnA   │  0    │ Mode Select A = 0 (Output Compare)         │
 *   │   3  │ ELSnB  │  0    │ Edge/Level B = 0 (Clear on match)          │
 *   │   2  │ ELSnA  │  1    │ Edge/Level A = 1 (Clear on match)          │
 *   │   1  │  —     │  0    │ Reservado                                  │
 *   │   0  │ DMA    │  0    │ Sem DMA request                            │
 *   └─────────────────────────────────────────────────────────────────────┘
 *   CnSC = 0x64 = (CHIE | MSnB | ELSnA)
 *
 * Convenção de polaridade do pino de ignição:
 *   HIGH → bobina carregando (dwell)
 *   LOW  → corte de corrente → faísca (disparo)
 *
 * Eliminação de jitter:
 *   Comparação FTM0_CNT == CnV é testada pelo periférico a cada ciclo de
 *   clock do FTM (16,67 ns). A transição acontece nesse mesmo ciclo,
 *   independente de IRQ pendentes, pipeline do Cortex-M4 ou DMA.
 *   Jitter residual: 0 ciclos de FTM = 0 ns (hardware determinístico).
 */
void ftm0_arm_ignition(uint8_t ch, uint16_t ticks) noexcept {
#if !defined(EMS_HOST_TEST)
    // 1. Configura canal para Output Compare / Clear on match + CHIE
    //    Escrever CHF=0 aqui limpa flag residual de evento anterior (W0C).
    //    MSnB=1, ELSnA=1, CHIE=1 → CnSC = FTM_CnSC_CHIE | FTM_CnSC_MSB | FTM_CnSC_ELSA
    FTM_CnSC(FTM0_BASE, ch) = FTM_CnSC_CHIE | FTM_CnSC_MSB | FTM_CnSC_ELSA;

    // 2. Programa o valor alvo do comparador.
    //    O FTM0 dispara quando CNT (16-bit, free-running) == CnV.
    //    Aritmética de wrap: se ticks < FTM0_CNT atual, o evento ocorre na
    //    próxima volta do contador (≈1,09 ms após o wrap — max 1 overflow).
    FTM_CnV(FTM0_BASE, ch) = ticks;
#else
    // Em ambiente host, simula a programação via mock do HAL.
    ems::hal::ftm0_set_compare(ch, ticks);
#endif
}


// ──────────────────────────────────────────────────────────────────────────────
// ISR Handlers
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief ISR do FTM3 — eventos de input capture CKP e CMP.
 *
 * FLUXO POR CANAL:
 *   1. Verifica CHF para determinar qual canal gerou a interrupção
 *   2. Lê o valor capturado (FTM_CnV) — este é o timestamp do evento
 *   3. Verifica estado atual do pino via PTD_PDIR (anti-glitch, RusEFI #1488)
 *   4. Limpa CHF (write 0 no bit 7)
 *   5. Despacha para callback de camada superior
 *
 * REGRA: Nenhuma lógica de motor aqui. Apenas despacho.
 * REGRA: Nenhum Serial.print(), nenhuma alocação, nenhum float.
 */
extern "C" void FTM3_IRQHandler(void) {
    // ── CH0: CKP (PTD0) ──
    if (FTM_CnSC(FTM3_BASE, 0u) & FTM_CnSC_CHF) {
        // Anti-glitch: confirma que o pino está em nível ALTO (rising edge real)
        // PTD_PDIR bit 0 = estado atual de PTD0
        if (PTD_PDIR & (1u << 0u)) {
            ems::drv::ckp_ftm3_ch0_isr();
        }
        // Limpa CHF independentemente (mesmo em borda espúria, deve limpar para não re-entrar)
        FTM_CnSC(FTM3_BASE, 0u) &= ~FTM_CnSC_CHF;
    }

    // ── CH1: CMP/fase (PTD1) ──
    if (FTM_CnSC(FTM3_BASE, 1u) & FTM_CnSC_CHF) {
        // Anti-glitch: confirma PTD1 alto
        if (PTD_PDIR & (1u << 1u)) {
            ems::drv::ckp_ftm3_ch1_isr();
        }
        FTM_CnSC(FTM3_BASE, 1u) &= ~FTM_CnSC_CHF;
    }
}

// FTM0_IRQHandler is defined in src/engine/ecu_sched.c (C module).
// That handler manages the 32-bit timer extension (g_overflow_count) and
// hardware output-compare scheduling. Removing the C++ definition here
// avoids a multiple-definition linker error.

} // namespace ems::hal
