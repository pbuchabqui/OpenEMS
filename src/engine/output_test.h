#pragma once

#include <cstdint>

// Teste de saídas em bancada (estilo TunerStudio "Test Modes"): exercita
// individualmente INJ1-4, IGN1-4, bomba, ventoinha, VVT×2, ETB e EWG com o
// motor parado. Enquanto activo, os controladores de malha fechada (ETB PID,
// EWG PID, VVT, bomba/ventoinha automáticos) ficam suspensos. O modo aborta
// sozinho se RPM > 0 for detectado ou se o keepalive (5 s) expirar — sempre
// restaurando o estado seguro de todas as saídas.

namespace ems::engine {

// Motivos de aborto reportados por output_test_status().
inline constexpr uint8_t kOutputTestAbortNone    = 0u;
inline constexpr uint8_t kOutputTestAbortRpm     = 1u;
inline constexpr uint8_t kOutputTestAbortTimeout = 2u;

// O relógio interno é alimentado por output_test_poll() (slot de 2 ms) —
// enter/keepalive usam o último now_ms visto pelo poll.
bool output_test_enter() noexcept;   // false se rpm_x10 != 0
void output_test_exit() noexcept;    // restore-safe + desactiva
bool output_test_active() noexcept;
void output_test_keepalive() noexcept;
// Chamar do slot de 2 ms: aborto por RPM e por timeout de keepalive.
void output_test_poll(uint32_t now_ms, uint32_t rpm_x10) noexcept;

// Comandos — devolvem false (=> NAK) se inactivo, args inválidos ou busy.
bool output_test_fire_injector(uint8_t cyl, uint16_t pw_us) noexcept;
bool output_test_fire_coil(uint8_t cyl, uint16_t dwell_us) noexcept;
bool output_test_set_pump(bool on) noexcept;
bool output_test_set_fan(bool on) noexcept;
bool output_test_set_vvt(uint8_t ch, uint16_t duty_pct_x10) noexcept;  // 0=esc 1=adm
bool output_test_set_etb(int16_t pwm) noexcept;   // clamp ±307 (~30% de 1023)
bool output_test_set_ewg(int16_t pwm) noexcept;   // clamp ±300 (30% de 1000)

// out[4] = {active, abort_reason, keepalive_restante_s, busy}
void output_test_status(uint8_t out[4]) noexcept;

#if defined(EMS_HOST_TEST)
void output_test_test_reset() noexcept;
#endif

}  // namespace ems::engine
