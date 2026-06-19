/**
 * @file hal/sdmmc.cpp
 * @brief SDMMC1 bare-metal driver (1-bit mode) for STM32H562VGT6.
 *
 * Pins: PC8=D0(AF12), PC12=CLK(AF12), PD2=CMD(AF12)
 * Uses FIFO polling for block writes (512 bytes).
 */

#ifndef EMS_HOST_TEST

#include "hal/sdmmc.h"
#include "hal/regs.h"

namespace {

static volatile bool     g_card_ready = false;
static volatile uint32_t g_rca        = 0u;
static volatile uint32_t g_err_count  = 0u;

static constexpr uint32_t kCmdTimeout   = 100000u;
static constexpr uint32_t kDataTimeout  = 500000u;
static constexpr uint32_t kBlockSize    = 512u;

// SDMMC1 STA flags
static constexpr uint32_t STA_CCRCFAIL = (1u << 0);
static constexpr uint32_t STA_DCRCFAIL = (1u << 1);
static constexpr uint32_t STA_CTIMEOUT = (1u << 2);
static constexpr uint32_t STA_DTIMEOUT = (1u << 3);
static constexpr uint32_t STA_CMDREND  = (1u << 6);
static constexpr uint32_t STA_CMDSENT  = (1u << 7);
static constexpr uint32_t STA_DATAEND  = (1u << 8);
static constexpr uint32_t STA_TXFIFOE  = (1u << 18);
static constexpr uint32_t STA_DPSMACT  = (1u << 12);
static constexpr uint32_t STA_TXUNDERR = (1u << 4);

static bool send_cmd(uint32_t cmd_idx, uint32_t arg, bool wait_resp) noexcept {
    SDMMC1_ICR = 0x1FE00FFFu;
    SDMMC1_ARG = arg;
    uint32_t cmd = (cmd_idx & 0x3Fu) | SDMMC1_CMD_CPSMEN;
    if (wait_resp) { cmd |= SDMMC1_CMD_WAITRESP_SHORT; }
    SDMMC1_CMD = cmd;

    uint32_t mask = wait_resp ? (STA_CMDREND | STA_CTIMEOUT | STA_CCRCFAIL)
                              : (STA_CMDSENT | STA_CTIMEOUT);
    for (uint32_t i = 0u; i < kCmdTimeout; ++i) {
        uint32_t sta = SDMMC1_STA;
        if (sta & STA_CTIMEOUT) { ++g_err_count; return false; }
        if (sta & mask) { return true; }
    }
    ++g_err_count;
    return false;
}

static bool send_acmd(uint32_t cmd_idx, uint32_t arg) noexcept {
    if (!send_cmd(55u, g_rca << 16u, true)) { return false; }
    return send_cmd(cmd_idx, arg, true);
}

}  // namespace

namespace ems::hal {

bool sdmmc_init() noexcept {
    g_card_ready = false;
    g_rca = 0u;

    // Enable SDMMC1 clock
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_SDMMC1EN;

    // GPIO: PC8=D0(AF12), PC12=CLK(AF12), PD2=CMD(AF12)
    volatile uint32_t* gpioc_moder   = reinterpret_cast<volatile uint32_t*>(GPIOC_BASE + GPIO_MODER_OFF);
    volatile uint32_t* gpioc_ospeedr = reinterpret_cast<volatile uint32_t*>(GPIOC_BASE + GPIO_OSPEEDR_OFF);
    volatile uint32_t* gpioc_afrh    = reinterpret_cast<volatile uint32_t*>(GPIOC_BASE + GPIO_AFRH_OFF);
    volatile uint32_t* gpioc_afrl    = reinterpret_cast<volatile uint32_t*>(GPIOC_BASE + GPIO_AFRL_OFF);
    (void)gpioc_afrl;

    // PC8: AF mode (10b), AF12
    *gpioc_moder = (*gpioc_moder & ~(3u << 16u)) | (2u << 16u);
    *gpioc_ospeedr |= (3u << 16u);  // very high speed
    *gpioc_afrh = (*gpioc_afrh & ~(0xFu << 0u)) | (GPIO_AF12 << 0u);  // PC8 AF12

    // PC12: AF mode, AF12
    *gpioc_moder = (*gpioc_moder & ~(3u << 24u)) | (2u << 24u);
    *gpioc_ospeedr |= (3u << 24u);
    *gpioc_afrh = (*gpioc_afrh & ~(0xFu << 16u)) | (GPIO_AF12 << 16u);  // PC12 AF12

    // PD2: AF mode, AF12
    GPIOD_MODER = (GPIOD_MODER & ~(3u << 4u)) | (2u << 4u);
    GPIOD_OSPEEDR |= (3u << 4u);
    GPIOD_AFRL = (GPIOD_AFRL & ~(0xFu << 8u)) | (GPIO_AF12 << 8u);  // PD2 AF12

    // Power on SDMMC1
    SDMMC1_POWER = SDMMC1_POWER_PWRCTRL_ON;
    for (volatile uint32_t i = 0u; i < 10000u; ++i) {}

    // Clock: 400 kHz for init (SDMMC kernel clock assumed ~48 MHz)
    // CLKDIV = 48MHz / (2 * 400kHz) = 60
    SDMMC1_CLKCR = 60u | SDMMC1_CLKCR_HWFC_EN;

    // CMD0: GO_IDLE_STATE
    if (!send_cmd(0u, 0u, false)) { return false; }

    // CMD8: SEND_IF_COND (voltage check, pattern 0xAA)
    if (!send_cmd(8u, 0x1AAu, true)) { return false; }
    uint32_t r7 = SDMMC1_RESP1;
    if ((r7 & 0xFFFu) != 0x1AAu) { ++g_err_count; return false; }

    // ACMD41: SD_SEND_OP_COND (HCS=1, 3.2-3.3V)
    for (uint32_t retry = 0u; retry < 1000u; ++retry) {
        if (!send_acmd(41u, 0x40FF8000u)) { return false; }
        uint32_t ocr = SDMMC1_RESP1;
        if (ocr & (1u << 31)) {
            // Card ready
            break;
        }
        if (retry == 999u) { ++g_err_count; return false; }
        for (volatile uint32_t d = 0u; d < 5000u; ++d) {}
    }

    // CMD2: ALL_SEND_CID
    SDMMC1_ICR = 0x1FE00FFFu;
    SDMMC1_ARG = 0u;
    SDMMC1_CMD = 2u | SDMMC1_CMD_CPSMEN | SDMMC1_CMD_WAITRESP_LONG;
    for (uint32_t i = 0u; i < kCmdTimeout; ++i) {
        if (SDMMC1_STA & (STA_CMDREND | STA_CTIMEOUT | STA_CCRCFAIL)) { break; }
    }
    if (SDMMC1_STA & STA_CTIMEOUT) { ++g_err_count; return false; }

    // CMD3: SEND_RELATIVE_ADDR
    if (!send_cmd(3u, 0u, true)) { return false; }
    g_rca = (SDMMC1_RESP1 >> 16u) & 0xFFFFu;

    // CMD7: SELECT_CARD
    if (!send_cmd(7u, g_rca << 16u, true)) { return false; }

    // Switch to full speed: ~24 MHz (CLKDIV=1)
    SDMMC1_CLKCR = 1u | SDMMC1_CLKCR_HWFC_EN;

    g_card_ready = true;
    return true;
}

bool sdmmc_write_block(uint32_t lba, const uint8_t* data) noexcept {
    if (!g_card_ready || data == nullptr) { return false; }

    // Set block length (CMD16) — SDHC uses fixed 512B, but send anyway
    send_cmd(16u, kBlockSize, true);

    // Configure data path
    SDMMC1_DTIMER = 0x0FFFFFFFu;
    SDMMC1_DLEN   = kBlockSize;
    SDMMC1_DCTRL  = SDMMC1_DCTRL_DBLOCKSIZE_512
                  | SDMMC1_DCTRL_DTEN;

    // CMD24: WRITE_SINGLE_BLOCK (SDHC: byte address = lba for SDHC)
    if (!send_cmd(24u, lba, true)) { return false; }

    // Write 512 bytes via FIFO
    const uint32_t* src = reinterpret_cast<const uint32_t*>(data);
    uint32_t words_remaining = kBlockSize / 4u;

    while (words_remaining > 0u) {
        uint32_t sta = SDMMC1_STA;
        if (sta & (STA_DTIMEOUT | STA_DCRCFAIL | STA_TXUNDERR)) {
            ++g_err_count;
            return false;
        }
        if (sta & STA_TXFIFOE) {
            uint32_t count = (words_remaining > 8u) ? 8u : words_remaining;
            for (uint32_t i = 0u; i < count; ++i) {
                SDMMC1_FIFO = *src++;
            }
            words_remaining -= count;
        }
    }

    // Wait for data transfer to complete
    for (uint32_t i = 0u; i < kDataTimeout; ++i) {
        uint32_t sta = SDMMC1_STA;
        if (sta & STA_DATAEND) { break; }
        if (sta & (STA_DTIMEOUT | STA_DCRCFAIL | STA_TXUNDERR)) {
            ++g_err_count;
            return false;
        }
        if (i == kDataTimeout - 1u) { ++g_err_count; return false; }
    }

    SDMMC1_ICR = 0x1FE00FFFu;
    return true;
}

bool sdmmc_card_present() noexcept {
    return g_card_ready;
}

uint32_t sdmmc_error_count() noexcept {
    return g_err_count;
}

}  // namespace ems::hal

#else  // EMS_HOST_TEST

#include "hal/sdmmc.h"

namespace ems::hal {

static bool     g_mock_card_present = false;
static uint32_t g_mock_err_count    = 0u;

bool     sdmmc_init() noexcept          { return true; }
bool     sdmmc_write_block(uint32_t, const uint8_t*) noexcept { return g_mock_card_present; }
bool     sdmmc_card_present() noexcept  { return g_mock_card_present; }
uint32_t sdmmc_error_count() noexcept   { return g_mock_err_count; }

}  // namespace ems::hal

#endif  // EMS_HOST_TEST
