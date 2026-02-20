/**
 * @file atomic_buffer.h
 * @brief Lock-free double buffer for Core 0 → Core 1 data exchange
 *
 * Core 0 (producer, ISR/time-critical) writes sensor + engine state.
 * Core 1 (consumer, FreeRTOS task) reads a consistent snapshot.
 *
 * Uses a seqlock-like protocol:
 *   - Writer increments sequence before and after writing.
 *   - Reader spins if sequence is odd (write in progress) or changes.
 *   - No mutex, no blocking, no FreeRTOS calls in writer path.
 *
 * Usage (Core 0, ISR):
 *   atomic_buf_write(&g_engine_buf, &new_state, sizeof(new_state));
 *
 * Usage (Core 1, task):
 *   engine_state_t snap;
 *   atomic_buf_read(&g_engine_buf, &snap, sizeof(snap));
 */

#ifndef ATOMIC_BUFFER_H
#define ATOMIC_BUFFER_H

#include <stdint.h>
#include <string.h>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint32_t sequence;   // Even = stable, Odd = being written
    uint8_t           data[256];  // Payload — must fit engine state struct
} atomic_buf_t;

/**
 * @brief Write new data (Core 0 / ISR safe, no blocking)
 * H9 fix: Added proper memory barriers using __sync_synchronize() for
 * cross-core data exchange on ESP32-S3 dual-core system.
 */
IRAM_ATTR static inline void atomic_buf_write(atomic_buf_t *buf,
                                               const void   *src,
                                               size_t        len) {
    buf->sequence++;                          // Mark write in progress (odd)
    __sync_synchronize();                    // H9 fix: memory barrier before data write
    memcpy(buf->data, src, len);
    __sync_synchronize();                    // H9 fix: memory barrier after data write
    buf->sequence++;                          // Mark write done (even)
}

/**
 * @brief Read a consistent snapshot (Core 1, may spin briefly)
 * H9 fix: Added proper memory barriers using __sync_synchronize() for
 * cross-core data exchange on ESP32-S3 dual-core system.
 */
static inline void atomic_buf_read(atomic_buf_t *buf,
                                    void         *dst,
                                    size_t        len) {
    uint32_t seq0, seq1;
    do {
        seq0 = buf->sequence;
        if (seq0 & 1U) continue;             // Write in progress — retry
        __sync_synchronize();                // H9 fix: memory barrier before data read
        memcpy(dst, buf->data, len);
        __sync_synchronize();                // H9 fix: memory barrier after data read
        seq1 = buf->sequence;
    } while (seq0 != seq1);
}

// Compile-time check that engine_state fits in buffer
// Include engine_state header where you declare the buffer.
#define ATOMIC_BUF_ASSERT_SIZE(type) \
    _Static_assert(sizeof(type) <= sizeof(((atomic_buf_t*)0)->data), \
                   #type " too large for atomic_buf_t")

#ifdef __cplusplus
}
#endif

#endif // ATOMIC_BUFFER_H
