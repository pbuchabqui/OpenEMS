/**
 * @file hal/sd.cpp
 * @brief Cartão SD pelo SDMMC1 (STM32H562VGT6): 4 bits, polling, IDMA de buffer único.
 *
 * Pinos (AF12): PC8..PC11 = D0..D3, PC12 = CK, PD2 = CMD. Na RGT6 PC8..PC11 são
 * IGN3/4 e INJ3/4, então este arquivo só entra na VGT6 (INJ/IGN na GPIOE).
 *
 * Clock do kernel: PLL1Q = 62,5 MHz (o mesmo do FDCAN; SDMMC1SEL reset = PLL1Q).
 * SDMMC_CK = 62,5 MHz / (2 * CLKDIV): 395 kHz na identificação, 15,6 MHz nos dados
 * (modo padrão, <= 25 MHz). Sem o PLL ligado o cartão fica ABSENT.
 *
 * Só SDHC/SDXC (v2.00+, endereçamento por bloco). Sem interrupções; sd_poll()
 * avança no máximo um passo por chamada. NÃO validado em hardware.
 */

#include "hal/sd.h"

#include "hal/board_pinout.h"

#if EMS_BOARD_IS_VGT6 && !defined(EMS_HOST_TEST)

#include "hal/regs.h"
#include "hal/system.h"

namespace ems::hal {

namespace {

constexpr uint32_t kKernelHz = 62500000u;
constexpr uint32_t kClkdivInit = 79u;
constexpr uint32_t kClkdivData = 2u;
static_assert(kKernelHz / (2u * kClkdivInit) <= 400000u, "identificação do SD acima de 400 kHz");
static_assert(kKernelHz / (2u * kClkdivData) <= 25000000u, "modo padrão do SD acima de 25 MHz");

constexpr uint32_t kGpioAfSdmmc = GPIO_AF12;
constexpr uint32_t kCmdTimeoutMs = 100u;
constexpr uint32_t kInitTimeoutMs = 2000u;
constexpr uint32_t kAcmd41TimeoutMs = 1000u;
constexpr uint32_t kOpTimeoutMs = 3000u;
constexpr uint32_t kRetryAbsentMs = 1000u;
constexpr uint32_t kDataTimeoutClocks = 0x00FFFFFFu;   // ~1 s a 15,6 MHz
constexpr uint32_t kOcrErrorBits = 0xFDFFE008u;        // R1: bits de erro do cartão
constexpr uint32_t kStaDataErr = SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT |
                                 SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR | SDMMC_STA_IDMATE;

enum class Wait : uint8_t { NONE, SHORT, LONG };

enum class Step : uint8_t {
    POWER_WAIT, CMD0, CMD8, CMD55_41, ACMD41, CMD2, CMD3, CMD7, CMD55_6, ACMD6,
};

enum class OpStep : uint8_t { IDLE, CMD, DATA, STATUS };

SdState g_state = SdState::ABSENT;
Step g_step = Step::POWER_WAIT;
uint32_t g_step_ms = 0u;
uint32_t g_init_ms = 0u;
uint32_t g_last_try_ms = 0u;
uint32_t g_rca = 0u;
uint32_t g_resp = 0u;
bool g_cmd_pending = false;
Wait g_cmd_wait = Wait::NONE;
uint32_t g_cmd_ms = 0u;
bool g_enabled = false;

SdOpStatus g_op_status = SdOpStatus::IDLE;
OpStep g_op_step = OpStep::IDLE;
bool g_op_write = false;
uint32_t g_op_ms = 0u;

bool pll_is_sysclk() noexcept {
    return (RCC_CFGR1 & (7u << 3)) == RCC_CFGR1_SWS_PLL1;
}

void pins_init() noexcept {
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 8u, kGpioAfSdmmc);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 9u, kGpioAfSdmmc);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 10u, kGpioAfSdmmc);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 11u, kGpioAfSdmmc);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 12u, kGpioAfSdmmc);
    gpio_set_af(&GPIOD_MODER, &GPIOD_AFRL, &GPIOD_AFRH, &GPIOD_OSPEEDR, 2u, kGpioAfSdmmc);
    // Pull-up em D0..D3 (PC8..PC11) e CMD (PD2), como no exemplo da WeAct; CK sem pull.
    GPIOC_PUPDR = (GPIOC_PUPDR & ~((3u << 16) | (3u << 18) | (3u << 20) | (3u << 22))) |
                  (1u << 16) | (1u << 18) | (1u << 20) | (1u << 22);
    GPIOD_PUPDR = (GPIOD_PUPDR & ~(3u << 4)) | (1u << 4);
}

void cmd_issue(uint8_t idx, uint32_t arg, Wait wait, bool with_data = false) noexcept {
    SDMMC1_ICR = SDMMC_ICR_ALL;
    SDMMC1_ARG = arg;
    uint32_t cmd = static_cast<uint32_t>(idx & 0x3Fu) | SDMMC_CMD_CPSMEN;
    if (wait == Wait::SHORT) { cmd |= SDMMC_CMD_WAITRESP_SHORT; }
    if (wait == Wait::LONG)  { cmd |= SDMMC_CMD_WAITRESP_LONG; }
    if (with_data)           { cmd |= SDMMC_CMD_CMDTRANS; }
    g_cmd_wait = wait;
    g_cmd_ms = millis();
    g_cmd_pending = true;
    SDMMC1_CMD = cmd;
}

// 0 = pendente, 1 = ok, -1 = erro/timeout. ignore_crc: R3 (ACMD41) não tem CRC válido.
int cmd_result(bool ignore_crc) noexcept {
    if (!g_cmd_pending) {
        return -1;
    }
    const uint32_t sta = SDMMC1_STA;
    int r = 0;
    if (sta & SDMMC_STA_CTIMEOUT) {
        r = -1;
    } else if (g_cmd_wait == Wait::NONE) {
        if (sta & SDMMC_STA_CMDSENT) { r = 1; }
    } else if (sta & SDMMC_STA_CMDREND) {
        r = 1;
    } else if (sta & SDMMC_STA_CCRCFAIL) {
        r = ignore_crc ? 1 : -1;
    }
    if (r == 0 && (millis() - g_cmd_ms) > kCmdTimeoutMs) {
        r = -1;
    }
    if (r != 0) {
        g_cmd_pending = false;
        g_resp = SDMMC1_RESP1;
        SDMMC1_ICR = SDMMC_ICR_CMD;   // o DATAEND de uma transferência em curso não pode ser apagado aqui
    }
    return r;
}

void init_restart() noexcept {
    SDMMC1_POWER = 0u;
    SDMMC1_CLKCR = kClkdivInit;                      // 1 bit, sem flow control
    SDMMC1_DTIMER = kDataTimeoutClocks;
    SDMMC1_IDMACTRL = 0u;
    SDMMC1_POWER = SDMMC_POWER_PWRCTRL_ON;
    g_state = SdState::INIT;
    g_step = Step::POWER_WAIT;
    g_step_ms = millis();
    g_init_ms = g_step_ms;
    g_cmd_pending = false;
    g_op_status = SdOpStatus::IDLE;
    g_op_step = OpStep::IDLE;
}

void init_fail(bool absent) noexcept {
    SDMMC1_POWER = 0u;
    g_last_try_ms = millis();
    g_state = absent ? SdState::ABSENT : SdState::FAULT;
}

void init_step() noexcept {
    if ((millis() - g_init_ms) > kInitTimeoutMs) {
        init_fail(false);
        return;
    }
    int r = 0;
    switch (g_step) {
        case Step::POWER_WAIT:
            if ((millis() - g_step_ms) >= 2u) {      // >= 74 clocks de CK a 395 kHz
                cmd_issue(0u, 0u, Wait::NONE);
                g_step = Step::CMD0;
            }
            break;
        case Step::CMD0:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) { cmd_issue(8u, 0x1AAu, Wait::SHORT); g_step = Step::CMD8; }
            break;
        case Step::CMD8:
            r = cmd_result(false);
            if (r < 0) { init_fail(true); }                     // sem resposta: sem cartão
            if (r > 0) {
                if ((g_resp & 0xFFFu) != 0x1AAu) { init_fail(false); break; }
                g_step_ms = millis();                           // início do prazo do ACMD41
                cmd_issue(55u, 0u, Wait::SHORT);
                g_step = Step::CMD55_41;
            }
            break;
        case Step::CMD55_41:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) { cmd_issue(41u, 0x40FF8000u, Wait::SHORT); g_step = Step::ACMD41; }
            break;
        case Step::ACMD41:
            r = cmd_result(true);
            if (r < 0) { init_fail(false); }
            if (r > 0) {
                if (g_resp & (1u << 31)) {                      // pronto
                    if (!(g_resp & (1u << 30))) { init_fail(false); break; }   // SDSC: sem suporte
                    cmd_issue(2u, 0u, Wait::LONG);
                    g_step = Step::CMD2;
                } else if ((millis() - g_step_ms) > kAcmd41TimeoutMs) {
                    init_fail(false);
                } else {
                    cmd_issue(55u, 0u, Wait::SHORT);
                    g_step = Step::CMD55_41;
                }
            }
            break;
        case Step::CMD2:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) { cmd_issue(3u, 0u, Wait::SHORT); g_step = Step::CMD3; }
            break;
        case Step::CMD3:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) {
                g_rca = g_resp >> 16u;
                cmd_issue(7u, g_rca << 16u, Wait::SHORT);
                g_step = Step::CMD7;
            }
            break;
        case Step::CMD7:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) { cmd_issue(55u, g_rca << 16u, Wait::SHORT); g_step = Step::CMD55_6; }
            break;
        case Step::CMD55_6:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) { cmd_issue(6u, 2u, Wait::SHORT); g_step = Step::ACMD6; }   // 4 bits
            break;
        case Step::ACMD6:
            r = cmd_result(false);
            if (r < 0) { init_fail(false); }
            if (r > 0) {
                SDMMC1_CLKCR = kClkdivData | SDMMC_CLKCR_WIDBUS_4 | SDMMC_CLKCR_HWFC_EN;
                g_state = SdState::READY;
            }
            break;
    }
}

void op_finish(bool ok) noexcept {
    SDMMC1_IDMACTRL = 0u;
    SDMMC1_DCTRL = 0u;
    SDMMC1_ICR = SDMMC_ICR_ALL;
    g_cmd_pending = false;
    g_op_step = OpStep::IDLE;
    g_op_status = ok ? SdOpStatus::OK : SdOpStatus::ERROR;
}

bool op_start(uint32_t lba, uint32_t addr, bool write) noexcept {
    if (g_state != SdState::READY || g_op_status != SdOpStatus::IDLE) {
        return false;
    }
    g_op_write = write;
    g_op_ms = millis();
    SDMMC1_ICR = SDMMC_ICR_ALL;
    SDMMC1_IDMABASER = addr;
    SDMMC1_IDMACTRL = SDMMC_IDMACTRL_IDMAEN;
    SDMMC1_DTIMER = kDataTimeoutClocks;
    SDMMC1_DLEN = kSdBlockBytes;
    SDMMC1_DCTRL = SDMMC_DCTRL_DBLOCKSIZE_512 | (write ? 0u : SDMMC_DCTRL_DTDIR);
    cmd_issue(write ? 24u : 17u, lba, Wait::SHORT, /*with_data=*/true);
    g_op_status = SdOpStatus::BUSY;
    g_op_step = OpStep::CMD;
    return true;
}

void op_step() noexcept {
    if (g_op_status != SdOpStatus::BUSY) {
        return;
    }
    if ((millis() - g_op_ms) > kOpTimeoutMs) {
        op_finish(false);
        return;
    }
    switch (g_op_step) {
        case OpStep::CMD: {
            const int r = cmd_result(false);
            if (r < 0) { op_finish(false); }
            if (r > 0) {
                if (g_resp & kOcrErrorBits) { op_finish(false); }
                else { g_op_step = OpStep::DATA; }
            }
            break;
        }
        case OpStep::DATA: {
            const uint32_t sta = SDMMC1_STA;
            if (sta & kStaDataErr) {
                op_finish(false);
            } else if (sta & SDMMC_STA_DATAEND) {
                if (g_op_write) {
                    SDMMC1_ICR = SDMMC_ICR_ALL;
                    cmd_issue(13u, g_rca << 16u, Wait::SHORT);   // espera o cartão sair de "programando"
                    g_op_step = OpStep::STATUS;
                } else {
                    op_finish(true);
                }
            }
            break;
        }
        case OpStep::STATUS: {
            const int r = cmd_result(false);
            if (r < 0) {
                op_finish(false);
            } else if (r > 0) {
                const uint32_t state = (g_resp >> 9u) & 0xFu;   // 4 = transfer, 7 = programando
                if (state == 4u) {
                    op_finish(true);
                } else {
                    cmd_issue(13u, g_rca << 16u, Wait::SHORT);
                }
            }
            break;
        }
        case OpStep::IDLE:
            break;
    }
}

}  // namespace

void sd_init() noexcept {
    g_enabled = pll_is_sysclk();    // o kernel clock vem do PLL1Q: sem PLL não há cartão
    g_state = SdState::ABSENT;
    if (!g_enabled) {
        return;
    }
    RCC_AHB4ENR |= RCC_AHB4ENR_SDMMC1EN;
    (void)RCC_AHB4ENR;              // leitura de volta: o clock já chegou antes do 1º acesso
    pins_init();
    init_restart();
}

void sd_poll() noexcept {
    switch (g_state) {
        case SdState::ABSENT:
            if (g_enabled && (millis() - g_last_try_ms) >= kRetryAbsentMs) {
                init_restart();
            }
            break;
        case SdState::INIT:
            init_step();
            break;
        case SdState::READY:
            op_step();
            break;
        case SdState::FAULT:
            break;
    }
}

SdState sd_state() noexcept { return g_state; }

bool sd_read_start(uint32_t lba, uint8_t* dst) noexcept {
    return op_start(lba, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(dst)), false);
}

bool sd_write_start(uint32_t lba, const uint8_t* src) noexcept {
    return op_start(lba, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(src)), true);
}

SdOpStatus sd_op_status() noexcept { return g_op_status; }

void sd_op_ack() noexcept {
    if (g_op_status == SdOpStatus::OK || g_op_status == SdOpStatus::ERROR) {
        g_op_status = SdOpStatus::IDLE;
    }
}

}  // namespace ems::hal

#endif  // EMS_BOARD_IS_VGT6 && !EMS_HOST_TEST
