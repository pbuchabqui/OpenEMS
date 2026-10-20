#pragma once
#include <cstdint>

namespace ems::hal {

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
#endif
    }

    ~CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
        asm volatile("msr primask, %0" :: "r"(primask_) : "memory");
#endif
    }

    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard(CriticalSectionGuard&&) = delete;
    CriticalSectionGuard& operator=(CriticalSectionGuard&&) = delete;

private:
    uint32_t primask_ = 0U;
};

} // namespace ems::hal
