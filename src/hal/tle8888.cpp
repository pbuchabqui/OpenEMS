#include "hal/tle8888.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"

namespace {

// TLE8888 SPI: 16-bit frame, CPOL=0 CPHA=1, max 5 MHz
// Frame: [15]=R/W (1=read), [14:8]=addr, [7:0]=data

constexpr uint16_t TLE_READ  = 0x8000u;
constexpr uint16_t TLE_WRITE = 0x0000u;

// TLE8888 registers (datasheet rev 2.1)
constexpr uint8_t REG_OPMODE      = 0x01u;  // operation mode
constexpr uint8_t REG_INCONFIG0   = 0x04u;  // INJ ch0-1 mode
constexpr uint8_t REG_INCONFIG1   = 0x05u;  // INJ ch2-3 mode
constexpr uint8_t REG_IGNCONFIG   = 0x08u;  // IGN ch0-3 mode
constexpr uint8_t REG_OC_THRESH   = 0x0Au;  // overcurrent threshold
constexpr uint8_t REG_SLEW_RATE   = 0x0Bu;  // output slew rate
constexpr uint8_t REG_CHIP_ID     = 0x7Eu;
constexpr uint8_t REG_OP_STAT     = 0x34u;  // operation status
constexpr uint8_t REG_DIAG_OUT0   = 0x38u;  // diag INJ ch0-1 (2 bits each)
constexpr uint8_t REG_DIAG_OUT1   = 0x39u;  // diag INJ ch2-3
constexpr uint8_t REG_DIAG_OUT2   = 0x3Au;  // diag IGN ch0-1
constexpr uint8_t REG_DIAG_OUT3   = 0x3Bu;  // diag IGN ch2-3
constexpr uint8_t REG_WD_CTRL     = 0x56u;  // watchdog control
constexpr uint8_t REG_WD_TRIG     = 0x57u;  // watchdog trigger

constexpr uint16_t EXPECTED_CHIP_ID = 0x88u;

// INCONFIG: 2 bits per channel — 00=off, 01=low-side, 10=high-side, 11=push-pull
constexpr uint8_t CHMODE_LOW_SIDE  = 0x01u;
// IGNCONFIG: 2 bits per channel — 00=off, 01=direct, 10=push-pull
constexpr uint8_t IGNMODE_PUSH_PULL = 0x02u;
// OC_THRESH: [3:0]=INJ bank threshold, [7:4]=IGN bank threshold
// 0x5 ≈ 10A for INJ, 0x3 ≈ 6A for IGN
constexpr uint8_t OC_INJ_10A = 0x05u;
constexpr uint8_t OC_IGN_6A  = 0x03u;
// SLEW_RATE: [3:0]=INJ (1=fast), [7:4]=IGN (1=fast)
constexpr uint8_t SLEW_FAST = 0x11u;

// Channel fault codes: 2 bits per channel
// 00=OK, 01=open-load, 10=short-to-GND, 11=short-to-VBAT
constexpr uint8_t CH_FAULT_MASK = 0x03u;

volatile uint16_t g_fault_count = 0u;
volatile bool g_tle_ok = false;
volatile bool g_tle_configured = false;
// Per-channel fault status: [0-3]=INJ, [4-7]=IGN
// Each byte: 0=OK, 1=open-load, 2=short-GND, 3=short-VBAT
volatile uint8_t g_channel_faults[8] = {};

inline void cs_low()  noexcept { GPIOB_BSRR = (1u << (12u + 16u)); }  // reset PB12
inline void cs_high() noexcept { GPIOB_BSRR = (1u << 12u); }          // set PB12

constexpr uint32_t SPI_TIMEOUT = 50000u;  // ~500µs at 250MHz

uint16_t spi2_txrx(uint16_t tx) noexcept {
    uint32_t tries = SPI_TIMEOUT;
    while (!(SPI2_SR & SPI_SR_TXP) && --tries) {}
    if (!tries) { return 0xFFFFu; }
    SPI2_TXDR = tx;

    SPI2_CR1 |= SPI_CR1_CSTART;

    tries = SPI_TIMEOUT;
    while (!(SPI2_SR & SPI_SR_RXP) && --tries) {}
    if (!tries) { SPI2_IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC; return 0xFFFFu; }
    const uint16_t rx = static_cast<uint16_t>(SPI2_RXDR);

    tries = SPI_TIMEOUT;
    while (!(SPI2_SR & SPI_SR_EOT) && --tries) {}
    SPI2_IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;

    return rx;
}

uint8_t tle_read(uint8_t addr) noexcept {
    const uint16_t cmd = TLE_READ | (static_cast<uint16_t>(addr) << 8u);
    cs_low();
    spi2_txrx(cmd);      // send command, response is from previous
    cs_high();
    // Second transaction to clock out the response
    cs_low();
    const uint16_t resp = spi2_txrx(TLE_READ);  // dummy read
    cs_high();
    return static_cast<uint8_t>(resp & 0xFFu);
}

void tle_write(uint8_t addr, uint8_t data) noexcept {
    const uint16_t cmd = TLE_WRITE | (static_cast<uint16_t>(addr) << 8u) | data;
    cs_low();
    spi2_txrx(cmd);
    cs_high();
}

bool write_verify(uint8_t addr, uint8_t data) noexcept {
    tle_write(addr, data);
    return tle_read(addr) == data;
}

bool configure_channels() noexcept {
    // INJ ch0-3: low-side switch (2 bits each, packed 2 per register)
    // INCONFIG0: ch0[1:0]=01, ch1[3:2]=01 → 0x05
    // INCONFIG1: ch2[1:0]=01, ch3[3:2]=01 → 0x05
    if (!write_verify(REG_INCONFIG0, (CHMODE_LOW_SIDE << 2u) | CHMODE_LOW_SIDE)) return false;
    if (!write_verify(REG_INCONFIG1, (CHMODE_LOW_SIDE << 2u) | CHMODE_LOW_SIDE)) return false;

    // IGN ch0-3: push-pull (2 bits each, packed 4 per register)
    // IGNCONFIG: ch0[1:0]=10, ch1[3:2]=10, ch2[5:4]=10, ch3[7:6]=10 → 0xAA
    const uint8_t ign_all_pp = (IGNMODE_PUSH_PULL << 6u) | (IGNMODE_PUSH_PULL << 4u)
                              | (IGNMODE_PUSH_PULL << 2u) | IGNMODE_PUSH_PULL;
    if (!write_verify(REG_IGNCONFIG, ign_all_pp)) return false;

    // Overcurrent: INJ 10A, IGN 6A
    if (!write_verify(REG_OC_THRESH, (OC_IGN_6A << 4u) | OC_INJ_10A)) return false;

    // Slew rate: fast for both banks
    if (!write_verify(REG_SLEW_RATE, SLEW_FAST)) return false;

    // Normal operation mode
    tle_write(REG_OPMODE, 0x01u);

    return true;
}

void decode_diag_pair(uint8_t reg_val, uint8_t base_ch) noexcept {
    g_channel_faults[base_ch]     = reg_val & CH_FAULT_MASK;
    g_channel_faults[base_ch + 1] = (reg_val >> 2u) & CH_FAULT_MASK;
}

}  // namespace

namespace ems::hal {

void tle8888_init() noexcept {
    // 1. Clock enable SPI2
    RCC_APB1LENR |= RCC_APB1LENR_SPI2EN;
    for (volatile int i = 0; i < 4; ++i) {}  // wait clock propagation

    // 2. GPIO: PB12=CS (output), PB13=SCK (AF5), PB14=MISO (AF5), PB15=MOSI (AF5)
    gpio_set_output(&GPIOB_MODER, &GPIOB_OSPEEDR, 12u);
    cs_high();  // CS idle high

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 13u, GPIO_AF5);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 14u, GPIO_AF5);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 15u, GPIO_AF5);

    // 3. SPI2 config: master, 16-bit, CPOL=0 CPHA=1, software CS
    SPI2_CR1 = 0u;  // disable first
    // CFG1: 16-bit data, prescaler = 32 → 125MHz/32 ≈ 3.9 MHz (< 5 MHz max)
    SPI2_CFG1 = SPI_CFG1_DSIZE_16BIT | (4u << 28u);  // MBR=100b → /32
    // CFG2: master, full-duplex, software SS management, CPHA=1
    SPI2_CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_CPHA
              | SPI_CFG2_COMM_FULLDUPLEX;
    // TSIZE=1 for single-frame transfers
    SPI2_CR2 = 1u;
    // Enable
    SPI2_CR1 = SPI_CR1_SPE;

    // 4. Handshake: read chip ID
    const uint8_t id = tle_read(REG_CHIP_ID);
    g_tle_ok = (id == EXPECTED_CHIP_ID);
    if (!g_tle_ok) {
        ++g_fault_count;
        return;
    }

    // 5. Configure channels: INJ low-side, IGN push-pull, OC thresholds, slew
    g_tle_configured = configure_channels();
    if (!g_tle_configured) {
        ++g_fault_count;
    }
}

void tle8888_poll_diag() noexcept {
    if (!g_tle_ok) {
        const uint8_t id = tle_read(REG_CHIP_ID);
        g_tle_ok = (id == EXPECTED_CHIP_ID);
        if (!g_tle_ok) {
            ++g_fault_count;
            return;
        }
        if (!g_tle_configured) {
            g_tle_configured = configure_channels();
        }
    }

    // Per-channel diagnostics: INJ ch0-3 (regs 0x38-0x39), IGN ch0-3 (0x3A-0x3B)
    decode_diag_pair(tle_read(REG_DIAG_OUT0), 0);  // INJ 0-1
    decode_diag_pair(tle_read(REG_DIAG_OUT1), 2);  // INJ 2-3
    decode_diag_pair(tle_read(REG_DIAG_OUT2), 4);  // IGN 0-1
    decode_diag_pair(tle_read(REG_DIAG_OUT3), 6);  // IGN 2-3

    bool any_fault = false;
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (g_channel_faults[i] != 0u) { any_fault = true; break; }
    }
    if (any_fault) { ++g_fault_count; }

    tle_write(REG_WD_TRIG, 0x01u);
}

bool tle8888_ok() noexcept {
    return g_tle_ok;
}

uint16_t tle8888_fault_count() noexcept {
    return g_fault_count;
}

uint8_t tle8888_channel_fault(uint8_t ch) noexcept {
    return (ch < 8u) ? g_channel_faults[ch] : 0u;
}

uint8_t tle8888_fault_bitmap() noexcept {
    uint8_t bm = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (g_channel_faults[i] != 0u) { bm |= (1u << i); }
    }
    return bm;
}

}  // namespace ems::hal

#else  // host test stub

namespace ems::hal {
void tle8888_init() noexcept {}
void tle8888_poll_diag() noexcept {}
bool tle8888_ok() noexcept { return true; }
uint16_t tle8888_fault_count() noexcept { return 0u; }
uint8_t tle8888_channel_fault(uint8_t) noexcept { return 0u; }
uint8_t tle8888_fault_bitmap() noexcept { return 0u; }
}

#endif
