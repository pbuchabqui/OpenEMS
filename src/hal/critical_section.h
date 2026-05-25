#pragma once
#include <cstdint>

namespace ems::hal {

/**
 * @brief RAII wrapper for critical sections (interrupt disable/enable)
 * 
 * Ensures interrupts are always re-enabled when leaving scope,
 * even on early returns or exceptions.
 * 
 * Usage:
 * @code
 * void some_function() {
 *     CriticalSectionGuard guard;
 *     
 *     if (condition) {
 *         return;  // Interrupts automatically re-enabled
 *     }
 *     
 *     // ... critical section code
 * }  // guard destructor runs here, re-enabling interrupts
 * @endcode
 * 
 * @note Non-copyable and non-movable by design
 * @note Only affects IRQ interrupts, not FIQ
 */
class CriticalSectionGuard {
public:
    CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
        // Disable IRQ interrupts (preserve FIQ)
        asm volatile("cpsid i" ::: "memory");
#endif
    }

    ~CriticalSectionGuard() noexcept {
#if defined(__arm__) || defined(__thumb__)
        // Re-enable IRQ interrupts
        asm volatile("cpsie i" ::: "memory");
#endif
    }

    // Non-copyable, non-movable
    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard(CriticalSectionGuard&&) = delete;
    CriticalSectionGuard& operator=(CriticalSectionGuard&&) = delete;

private:
#if !defined(__arm__) && !defined(__thumb__)
    // Host build - track state for testing
    bool interrupts_were_enabled_ = true;
#endif
};

} // namespace ems::hal
