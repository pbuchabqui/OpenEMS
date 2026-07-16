#include "test/ui_helpers.h"
#include "app/ui_protocol.h"
#include "hal/crc32.h"
#include <cstring>
using namespace ems::app;
using namespace ems::hal;

void ui_feed(const uint8_t* data, uint16_t n) {
    for (uint16_t i = 0u; i < n; ++i) { ems::app::ui_rx_byte(data[i]); }
    ems::app::ui_process();
}

uint16_t ui_drain(uint8_t* out, uint16_t max) {
    uint16_t n = 0u;
    uint8_t b = 0u;
    while (n < max && ems::app::ui_tx_pop(b)) { out[n++] = b; }
    return n;
}


uint16_t env_frame(uint8_t* out, const uint8_t* payload, uint16_t n) {
    out[0] = static_cast<uint8_t>(n >> 8u);
    out[1] = static_cast<uint8_t>(n & 0xFFu);
    memcpy(out + 2, payload, n);
    const uint32_t crc = ems::hal::crc32_calc(payload, n);
    out[2u + n] = static_cast<uint8_t>(crc >> 24u);
    out[3u + n] = static_cast<uint8_t>(crc >> 16u);
    out[4u + n] = static_cast<uint8_t>(crc >> 8u);
    out[5u + n] = static_cast<uint8_t>(crc & 0xFFu);
    return static_cast<uint16_t>(n + 6u);
}

EnvResp env_txn(const uint8_t* payload, uint16_t n) {
    EnvResp r = {};
    static uint8_t frame[1024] = {};
    const uint16_t fl = env_frame(frame, payload, n);
    ui_feed(frame, fl);

    static uint8_t buf[1024] = {};
    memset(buf, 0, sizeof(buf));
    const uint16_t rn = ui_drain(buf, sizeof(buf));
    if (rn < 7u) { return r; }
    const uint16_t psize = static_cast<uint16_t>((buf[0] << 8u) | buf[1]);
    if (static_cast<uint16_t>(psize + 6u) != rn) { return r; }
    r.frame_ok = true;
    r.code = buf[2];
    r.len = static_cast<uint16_t>(psize - 1u);
    memcpy(r.data, buf + 3, r.len);
    const uint32_t rx_crc = (static_cast<uint32_t>(buf[2u + psize]) << 24u) |
                            (static_cast<uint32_t>(buf[3u + psize]) << 16u) |
                            (static_cast<uint32_t>(buf[4u + psize]) << 8u) |
                             static_cast<uint32_t>(buf[5u + psize]);
    r.crc_ok = (rx_crc == ems::hal::crc32_calc(buf + 2, psize));
    return r;
}

