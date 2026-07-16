#include "app/ui_protocol.h"
#include "app/ui_protocol_internal.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app/can_stack.h"
#include "app/can_rx_map.h"
#include "hal/tle8888.h"
#include "hal/flex_fuel.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/calibration.h"
#include "engine/ecu_sched.h"
#include "engine/etb_control.h"
#include "engine/fuel_calc.h"
#include "engine/torque_manager.h"
#include "engine/map_estimator.h"
#include "app/status_bits.h"
#include "engine/ign_calc.h"
#include "engine/math_utils.h"
#include "engine/output_test.h"
#include "hal/timer.h"
#include "engine/constants.h"
#include "engine/table3d.h"
#include "hal/crc32.h"
#include "hal/flash.h"
#include "engine/engine_config.h"

namespace ems::app::ui_detail {

// ── Envelope TS: resposta e dispatcher ───────────────────────────────────────

uint16_t tx_free() noexcept {
    return static_cast<uint16_t>((kTxSize - 1u) -
                                 ((g_tx_head - g_tx_tail) & kTxMask));
}

void env_send_response(uint8_t code, const uint8_t* data, uint16_t len) noexcept {
    const uint16_t psize = static_cast<uint16_t>(len + 1u);
    // Sem espaço para o frame inteiro → não envia nada (frame parcial seria
    // corrupção; o TunerStudio faz timeout e re-tenta).
    if (tx_free() < static_cast<uint16_t>(psize + 6u)) {
        return;
    }
    tx_push(static_cast<uint8_t>(psize >> 8u));
    tx_push(static_cast<uint8_t>(psize & 0xFFu));
    uint32_t crc = 0xFFFFFFFFu;
    crc = ems::hal::crc32_update(crc, code);
    tx_push(code);
    for (uint16_t i = 0u; i < len; ++i) {
        crc = ems::hal::crc32_update(crc, data[i]);
        tx_push(data[i]);
    }
    crc = ~crc;
    tx_push(static_cast<uint8_t>(crc >> 24u));
    tx_push(static_cast<uint8_t>(crc >> 16u));
    tx_push(static_cast<uint8_t>(crc >> 8u));
    tx_push(static_cast<uint8_t>(crc & 0xFFu));
}

// Chunk write (RAM-only; burn é explícito via 'b'). `a` aponta para
// page + off(u16 LE) + len(u16 LE) + dados; data_n = bytes de dados no frame.
// Retorna código TS, ou -1 se o header declarado não corresponder a data_n
// (permite ao caller tentar a forma com canId à frente).
int env_try_write(const uint8_t* a, uint16_t data_n) noexcept {
    const uint8_t page = normalize_page_id(a[0]);
    const uint16_t off = static_cast<uint16_t>(a[1] | (static_cast<uint16_t>(a[2]) << 8u));
    const uint16_t len = static_cast<uint16_t>(a[3] | (static_cast<uint16_t>(a[4]) << 8u));
    if (len != data_n) {
        return -1;
    }
    if (!bounds_ok(page, off, len) || page == 0x03u || len > kEnvMaxChunk) {
        return kTsRcRangeErr;
    }
    uint8_t* ptr = page_ptr(page);
    if (ptr == nullptr) {
        return kTsRcRangeErr;
    }
    if (len != 0u) {
        std::memcpy(ptr + off, a + 5u, len);
        if (!sync_table_from_page(page)) {
            sync_page_from_table(page);
            return kTsRcRangeErr;
        }
        mark_page_dirty(page);
    }
    return kTsRcOk;
}

void env_dispatch(const uint8_t* p, uint16_t n) noexcept {
    const uint8_t cmd = p[0];

    if (cmd == static_cast<uint8_t>('Q') || cmd == static_cast<uint8_t>('H')) {
        env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kSignature),
                          static_cast<uint16_t>(sizeof(kSignature) - 1u));
        return;
    }
    if (cmd == static_cast<uint8_t>('S')) {
        env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kFwVersion),
                          static_cast<uint16_t>(sizeof(kFwVersion) - 1u));
        return;
    }
    if (cmd == static_cast<uint8_t>('F')) {
        env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kProtocolVersion),
                          static_cast<uint16_t>(sizeof(kProtocolVersion) - 1u));
        return;
    }
    if (cmd == static_cast<uint8_t>('C')) {
        const uint8_t magic = kCommsTestMagic;
        env_send_response(kTsRcOk, &magic, 1u);
        return;
    }
    if (cmd == static_cast<uint8_t>('A') || cmd == static_cast<uint8_t>('O')) {
        update_realtime_page();
        env_send_response(kTsRcOk, g_page3_rt,
                          static_cast<uint16_t>(sizeof(g_page3_rt)));
        return;
    }
    if (cmd == static_cast<uint8_t>('d')) {
        const uint8_t mask[2] = {
            static_cast<uint8_t>(g_dirty_page_mask & 0xFFu),
            static_cast<uint8_t>(g_dirty_page_mask >> 8u),
        };
        env_send_response(kTsRcOk, mask, 2u);
        return;
    }
    if (cmd == static_cast<uint8_t>('r')) {
        // 'r' [canId] page off(u16 LE) len(u16 LE) → 6 ou 7 bytes no payload
        if (n != 6u && n != 7u) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        const uint8_t* a = (n == 7u) ? (p + 2u) : (p + 1u);
        const uint8_t page = normalize_page_id(a[0]);
        const uint16_t off = static_cast<uint16_t>(a[1] | (static_cast<uint16_t>(a[2]) << 8u));
        const uint16_t len = static_cast<uint16_t>(a[3] | (static_cast<uint16_t>(a[4]) << 8u));
        // page=0x0F: pseudo-página "signature" — convenção real do TunerStudio/
        // Speeduino (comms.cpp: "cmd == 0x0f → Request for signature"), usada
        // pelo Comm Manager para validar o controlador após a conexão real
        // (distinta do probe leve 'Q' cru da fase de deteção/wizard).
        if (page == 0x0Fu) {
            env_send_response(kTsRcOk, reinterpret_cast<const uint8_t*>(kSignature),
                              static_cast<uint16_t>(sizeof(kSignature) - 1u));
            return;
        }
        if (!bounds_ok(page, off, len) || len > kEnvMaxChunk) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        if (page == 0x03u) {
            update_realtime_page();
        } else {
            sync_page_from_table(page);
        }
        const uint8_t* ptr = page_ptr(page);
        if (ptr == nullptr) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        env_send_response(kTsRcOk, ptr + off, len);
        return;
    }
    if (cmd == static_cast<uint8_t>('w') || cmd == static_cast<uint8_t>('x')) {
        int rc = (n >= 6u) ? env_try_write(p + 1u, static_cast<uint16_t>(n - 6u)) : -1;
        if (rc < 0 && n >= 7u) {
            rc = env_try_write(p + 2u, static_cast<uint16_t>(n - 7u));  // forma com canId
        }
        env_send_response((rc < 0) ? kTsRcRangeErr : static_cast<uint8_t>(rc), nullptr, 0u);
        return;
    }
    if (cmd == static_cast<uint8_t>('b')) {
        // 'b' [canId] page → 2 ou 3 bytes no payload
        if (n != 2u && n != 3u) {
            env_send_response(kTsRcRangeErr, nullptr, 0u);
            return;
        }
        const uint8_t page = normalize_page_id(p[n - 1u]);
        if (!burn_rpm_safe()) {
            env_send_response(kTsRcBusyErr, nullptr, 0u);
            return;
        }
        env_send_response(burn_page_to_flash(page) ? kTsRcOk : kTsRcRangeErr,
                          nullptr, 0u);
        return;
    }
    env_send_response(kTsRcUnknown, nullptr, 0u);
}


}  // namespace ems::app::ui_detail
