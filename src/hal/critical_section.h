#pragma once
#include <cstdint>

namespace ems::hal {

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
