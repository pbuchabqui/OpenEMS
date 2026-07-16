#pragma once
#include <cstdint>

void ui_feed(const uint8_t* data, uint16_t n);
uint16_t ui_drain(uint8_t* out, uint16_t max);
uint16_t env_frame(uint8_t* out, const uint8_t* payload, uint16_t n);

struct EnvResp {
    bool frame_ok;
    bool crc_ok;
    uint8_t code;
    uint16_t len;
    uint8_t data[1024];
};

EnvResp env_txn(const uint8_t* payload, uint16_t n);
