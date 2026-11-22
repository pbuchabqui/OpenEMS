#pragma once
#include <cstdint>

// Perfil das seções críticas (diagnóstico de bancada). -DEMS_CS_PROF=1 mede, com o contador de ciclos do DWT
// (250 MHz = 4 ns), quanto tempo cada CriticalSectionGuard mascara as interrupções. Desligado (padrão) o guard é
// idêntico ao de antes: nenhum membro, nenhuma instrução a mais no caminho do TIM5.
#ifndef EMS_CS_PROF
#define EMS_CS_PROF 0
#endif

namespace ems::hal {

// Dois baldes: 0 = laço principal (thread), 1 = outras exceções (SysTick e demais). Os guards dentro da ISR do TIM5
// não entram: ali o TIM5 já está rodando, e o que interessa é quanto ele fica BLOQUEADO por outro contexto.
// Só conta o guard mais externo (PRIMASK era 0 na entrada); os aninhados já estão dentro dele.
// max_cycles: maior tempo mascarado; max_pc: onde esse guard foi construído (endereço, para addr2line);
// count: quantas seções foram medidas. critical_enter()/critical_exit() (não RAII) não são medidos.
inline volatile uint32_t g_cs_prof_max_cycles[2] = {0U, 0U};
inline volatile uint32_t g_cs_prof_max_pc[2] = {0U, 0U};
inline volatile uint32_t g_cs_prof_count[2] = {0U, 0U};

constexpr bool cs_prof_enabled() noexcept { return EMS_CS_PROF != 0; }

inline void cs_prof_reset() noexcept {
    for (uint8_t i = 0U; i < 2U; ++i) {
        g_cs_prof_max_cycles[i] = 0U; g_cs_prof_max_pc[i] = 0U; g_cs_prof_count[i] = 0U;
    }
}

#if EMS_CS_PROF && (defined(__arm__) || defined(__thumb__))
// Registradores do ARMv8-M (Cortex-M33): DEMCR.TRCENA (bit 24) liga o DWT; DWT_CTRL.CYCCNTENA (bit 0) liga o contador.
inline volatile uint32_t& cs_prof_reg(uint32_t addr) noexcept { return *reinterpret_cast<volatile uint32_t*>(addr); }
constexpr uint32_t kCsProfTim5Ipsr = 16U + 48U;   // IPSR = 16 + IRQn (IRQ_TIM5 = 48, hal/stm32h562/regs.h)
#endif

inline void cs_prof_init() noexcept {
#if EMS_CS_PROF && (defined(__arm__) || defined(__thumb__))
    cs_prof_reg(0xE000EDFCU) |= (1U << 24U);   // DEMCR
    cs_prof_reg(0xE0001004U) = 0U;             // DWT_CYCCNT
    cs_prof_reg(0xE0001000U) |= 1U;            // DWT_CTRL
    cs_prof_reset();
#endif
}

// Par enter/exit para os pontos que não cabem num escopo RAII. Nest-safe e seguro dentro de ISR:
// guarda o PRIMASK anterior no primeiro enter (com PRIMASK=1 só um contexto roda, então o
// contador não corre) e só restaura no último exit. O `cpsie i` cru reabilitava interrupções
// mesmo quando o chamador já as tinha mascarado (ou dentro de uma ISR, permitindo preempção).
inline volatile uint32_t g_critical_depth = 0U;
inline volatile uint32_t g_critical_saved_primask = 0U;

inline void critical_enter() noexcept {
#if defined(__arm__) || defined(__thumb__)
    uint32_t pm;
    asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(pm) :: "memory");
    if (g_critical_depth == 0U) { g_critical_saved_primask = pm; }
    g_critical_depth = g_critical_depth + 1U;
#endif
}

inline void critical_exit() noexcept {
#if defined(__arm__) || defined(__thumb__)
    g_critical_depth = g_critical_depth - 1U;
    if (g_critical_depth == 0U) {
        asm volatile("msr primask, %0" :: "r"(g_critical_saved_primask) : "memory");
    }
#endif
}

/**
 * @brief RAII critical section: masks IRQs, restores the PREVIOUS PRIMASK.
 *
 * Nest-safe: an inner guard (e.g. arm_channel called from a main-loop guard,
 * or from the TIM5 ISR) never re-enables interrupts its caller had masked.
 */
class CriticalSectionGuard {
public:
    CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
        asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask_) :: "memory");
#if EMS_CS_PROF
        asm volatile("mov %0, pc" : "=r"(pc_));
        t0_ = cs_prof_reg(0xE0001004U);
#endif
#endif
    }

    ~CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
#if EMS_CS_PROF
        const uint32_t dt = cs_prof_reg(0xE0001004U) - t0_;
        uint32_t ipsr;
        asm volatile("mrs %0, ipsr" : "=r"(ipsr));
        const bool measure = (primask_ == 0U) && (ipsr != kCsProfTim5Ipsr);
        const uint8_t b = (ipsr == 0U) ? 0U : 1U;
#endif
        asm volatile("msr primask, %0" :: "r"(primask_) : "memory");
#if EMS_CS_PROF
        // Depois de liberar o PRIMASK: o registro não entra na conta. Pode ser interrompido por outro guard de
        // exceção; é diagnóstico, e na pior hipótese perde um máximo.
        if (measure) {
            g_cs_prof_count[b] = g_cs_prof_count[b] + 1U;
            if (dt > g_cs_prof_max_cycles[b]) { g_cs_prof_max_cycles[b] = dt; g_cs_prof_max_pc[b] = pc_; }
        }
#endif
#endif
    }

    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard(CriticalSectionGuard&&) = delete;
    CriticalSectionGuard& operator=(CriticalSectionGuard&&) = delete;

private:
    uint32_t primask_ = 0U;
#if EMS_CS_PROF
    uint32_t t0_ = 0U;
    uint32_t pc_ = 0U;
#endif
};

} // namespace ems::hal
