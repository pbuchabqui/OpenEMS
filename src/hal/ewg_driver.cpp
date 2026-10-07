#include "hal/ewg_driver.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"
#include "hal/timer.h"
#include "hal/adc.h"

namespace {

// EWG H-bridge: PA7 = IN1 (open), PD3 = IN2 (close), PB10 = TIM2_CH3 PWM.
// EWG PWM TIM2_CH3/PB10 — INJ3 já não usa PB10 (PC10). WeAct: PB10 pode não
// sair no header. PA6/TIM3 = ETB PWM. PD3 (DIR IN2) só packages c/ GPIOD.
constexpr uint8_t kIn1Pin = 7u;   // PA7
constexpr uint8_t kIn2Pin = 3u;   // PD3

// O EWG está DIFERIDO na placa de interface v1 (só footprint, sem estágio de
// potência montado — ver docs/hw/interface_board_v1.md). Enquanto assim for, o
// driver não deve reclamar pino nenhum.
//
// Não é cosmético. Com o EWG diferido, `ewg_driver_read_position_raw()` devolve
// 0 fixo (PC3 passou a VBATT), portanto o PID de posição vê erro = demanda − 0.
// Assim que a demanda sobe, o integrador satura e o driver passa a CONDUZIR
// PA7/PD3 e a pôr PWM a fundo em PB10 — três pinos accionados a sério para um
// estágio que não existe. Guardar aqui evita accionar hardware não montado.
//
// Ao repor o EWG (v2): pôr a 1, devolver-lhe um canal de ADC próprio para a
// realimentação de posição, e rever o par DIR (PD3 só existe em packages com
// GPIOD).
#define EMS_EWG_POPULATED 0

}  // namespace

namespace ems::hal {

bool ewg_driver_init() noexcept {
#if !EMS_EWG_POPULATED
    return false;
#else
    // PA7 = IN1 (GPIO output)
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (kIn1Pin * 2u))) | (1u << (kIn1Pin * 2u));
    // PD3 = IN2 (GPIO output)
    GPIOD_MODER = (GPIOD_MODER & ~(3u << (kIn2Pin * 2u))) | (1u << (kIn2Pin * 2u));

    // TIM2_CH3 (PB10) PWM @ 10 kHz for EWG motor
    tim2_pwm_init(10000u);

    ewg_driver_shutdown();
    return true;
#endif
}

void ewg_driver_set_motor_pwm(int16_t pwm) noexcept {
#if !EMS_EWG_POPULATED
    // Estágio não montado: não tocar em PA7/PD3/PB10 (ver nota no topo).
    static_cast<void>(pwm);
    return;
#else
    if (pwm >  1000) { pwm =  1000; }
    if (pwm < -1000) { pwm = -1000; }

    const uint16_t duty = static_cast<uint16_t>((pwm >= 0) ? pwm : -pwm);

    if (pwm > 0) {
        GPIOA_BSRR = (1u << kIn1Pin);                  // IN1=1 (open)
        GPIOD_BSRR = (1u << (kIn2Pin + 16u));          // IN2=0
    } else if (pwm < 0) {
        GPIOA_BSRR = (1u << (kIn1Pin + 16u));          // IN1=0
        GPIOD_BSRR = (1u << kIn2Pin);                  // IN2=1 (close)
    } else {
        GPIOA_BSRR = (1u << (kIn1Pin + 16u));          // IN1=0
        GPIOD_BSRR = (1u << (kIn2Pin + 16u));          // IN2=0 (brake)
    }
    tim2_set_duty(duty);
#endif
}

// EWG diferido na placa de interface v1: PC3/INP13 passou a ser VBATT (bloco 7), de
// modo que já não existe canal de realimentação de posição. Devolver o raw de VBATT
// aqui alimentaria o PID do EWG com a tensão da bateria — daí o retorno fixo em 0,
// que `ewg_read_position_pct_x10()` traduz em 0% e mantém o laço inerte enquanto a
// saída do EWG não estiver populada. Ao repor o EWG (v2), atribuir-lhe um canal
// próprio e restaurar a leitura.
uint16_t ewg_driver_read_position_raw() noexcept {
    return 0u;
}

void ewg_driver_shutdown() noexcept {
#if !EMS_EWG_POPULATED
    return;
#else
    GPIOA_BSRR = (1u << (kIn1Pin + 16u));
    GPIOD_BSRR = (1u << (kIn2Pin + 16u));
    tim2_set_duty(0u);
#endif
}

}  // namespace ems::hal

#else  // host test stub

namespace ems::hal {
bool ewg_driver_init() noexcept { return true; }
void ewg_driver_set_motor_pwm(int16_t) noexcept {}
uint16_t ewg_driver_read_position_raw() noexcept { return 2048u; }
void ewg_driver_shutdown() noexcept {}
}

#endif
