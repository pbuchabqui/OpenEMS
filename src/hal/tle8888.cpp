#include "hal/tle8888.h"

#ifdef TARGET_STM32H562
#include "hal/stm32h562/regs.h"

namespace {

// TLE8888 SPI: 16-bit frame, CPOL=0 CPHA=1, max 5 MHz
// Frame: [15]=R/W (1=read), [14:8]=addr, [7:0]=data

constexpr uint16_t TLE_READ  = 0x8000u;
constexpr uint16_t TLE_WRITE = 0x0000u;

// TLE8888 registers
constexpr uint8_t REG_CHIP_ID     = 0x7Eu;
constexpr uint8_t REG_OP_STAT     = 0x34u;  // operation status
constexpr uint8_t REG_DIAG0       = 0x38u;  // diagnostic output 0
constexpr uint8_t REG_DIAG1       = 0x39u;  // diagnostic output 1
constexpr uint8_t REG_WD_CTRL     = 0x56u;  // watchdog control
constexpr uint8_t REG_WD_TRIG     = 0x57u;  // watchdog trigger

constexpr uint16_t EXPECTED_CHIP_ID = 0x88u;

volatile uint16_t g_fault_count = 0u;
volatile bool g_tle_ok = false;

inline void cs_low()  noexcept { GPIOB_BSRR = (1u << (12u + 16u)); }  // reset PB12
inline void cs_high() noexcept { GPIOB_BSRR = (1u << 12u); }          // set PB12

uint16_t spi2_txrx(uint16_t tx) noexcept {
    // Wait TXP (TX buffer ready)
    while (!(SPI2_SR & SPI_SR_TXP)) {}
    SPI2_TXDR = tx;

    // Start transfer
    SPI2_CR1 |= SPI_CR1_CSTART;

    // Wait RXP (data received)
    while (!(SPI2_SR & SPI_SR_RXP)) {}
    const uint16_t rx = static_cast<uint16_t>(SPI2_RXDR);

    // Wait EOT and clear flags
    while (!(SPI2_SR & SPI_SR_EOT)) {}
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
    }
}

void tle8888_poll_diag() noexcept {
    if (!g_tle_ok) {
        // Retry handshake
        const uint8_t id = tle_read(REG_CHIP_ID);
        g_tle_ok = (id == EXPECTED_CHIP_ID);
        if (!g_tle_ok) {
            ++g_fault_count;
            return;
        }
    }

    // Read diagnostic registers
    const uint8_t diag0 = tle_read(REG_DIAG0);
    const uint8_t diag1 = tle_read(REG_DIAG1);
    if (diag0 != 0u || diag1 != 0u) {
        ++g_fault_count;
    }

    // Watchdog trigger (keeps TLE8888 alive)
    tle_write(REG_WD_TRIG, 0x01u);
}

bool tle8888_ok() noexcept {
    return g_tle_ok;
}

uint16_t tle8888_fault_count() noexcept {
    return g_fault_count;
}

}  // namespace ems::hal

#else  // host test stub

namespace ems::hal {
void tle8888_init() noexcept {}
void tle8888_poll_diag() noexcept {}
bool tle8888_ok() noexcept { return true; }
uint16_t tle8888_fault_count() noexcept { return 0u; }
}

#endif
