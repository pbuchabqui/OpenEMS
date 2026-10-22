#include "hal/sd.h"

#if defined(EMS_HOST_TEST)

#include <cstring>

namespace ems::hal {

namespace {

uint8_t g_card[kSdSimBlocks][kSdBlockBytes];
bool g_present = false;
bool g_ready = false;
bool g_fail_ops = false;
uint32_t g_latency_polls = 0u;
uint32_t g_write_count = 0u;

SdOpStatus g_status = SdOpStatus::IDLE;
uint32_t g_polls_left = 0u;
bool g_op_is_write = false;
uint32_t g_op_lba = 0u;
uint8_t* g_op_dst = nullptr;
const uint8_t* g_op_src = nullptr;

void finish_op() noexcept {
    if (g_fail_ops || g_op_lba >= kSdSimBlocks) {
        g_status = SdOpStatus::ERROR;
        return;
    }
    if (g_op_is_write) {
        std::memcpy(g_card[g_op_lba], g_op_src, kSdBlockBytes);
        ++g_write_count;
    } else {
        std::memcpy(g_op_dst, g_card[g_op_lba], kSdBlockBytes);
    }
    g_status = SdOpStatus::OK;
}

bool start_op(uint32_t lba) noexcept {
    if (!g_ready || g_status != SdOpStatus::IDLE) {
        return false;
    }
    g_op_lba = lba;
    g_polls_left = g_latency_polls;
    g_status = SdOpStatus::BUSY;
    if (g_polls_left == 0u) {
        finish_op();
    }
    return true;
}

}  // namespace

void sd_sim_reset() noexcept {
    std::memset(g_card, 0, sizeof(g_card));
    g_present = true;
    g_ready = true;
    g_fail_ops = false;
    g_latency_polls = 0u;
    g_write_count = 0u;
    g_status = SdOpStatus::IDLE;
}

void sd_sim_set_present(bool present) noexcept {
    g_present = present;
    g_ready = present;
    g_status = SdOpStatus::IDLE;
}

void sd_sim_set_op_latency_polls(uint32_t polls) noexcept { g_latency_polls = polls; }
void sd_sim_set_fail_ops(bool fail) noexcept { g_fail_ops = fail; }
uint32_t sd_sim_write_count() noexcept { return g_write_count; }

uint8_t* sd_sim_block(uint32_t lba) noexcept {
    return (lba < kSdSimBlocks) ? g_card[lba] : nullptr;
}

void sd_init() noexcept {
    g_ready = g_present;
    g_status = SdOpStatus::IDLE;
}

void sd_poll() noexcept {
    if (g_status == SdOpStatus::BUSY) {
        if (g_polls_left > 0u) {
            --g_polls_left;
        }
        if (g_polls_left == 0u) {
            finish_op();
        }
    }
}

SdState sd_state() noexcept { return g_ready ? SdState::READY : SdState::ABSENT; }

bool sd_read_start(uint32_t lba, uint8_t* dst) noexcept {
    g_op_is_write = false;
    g_op_dst = dst;
    return start_op(lba);
}

bool sd_write_start(uint32_t lba, const uint8_t* src) noexcept {
    g_op_is_write = true;
    g_op_src = src;
    return start_op(lba);
}

SdOpStatus sd_op_status() noexcept { return g_status; }

void sd_op_ack() noexcept {
    if (g_status == SdOpStatus::OK || g_status == SdOpStatus::ERROR) {
        g_status = SdOpStatus::IDLE;
    }
}

}  // namespace ems::hal

#endif  // EMS_HOST_TEST
