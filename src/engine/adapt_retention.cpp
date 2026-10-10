#include "engine/adapt_retention.h"

#include "engine/misfire_detect.h"
#include "engine/torque_manager.h"
#include "hal/flash.h"

namespace ems::engine {

bool adapt_retention_restore() noexcept {
    ems::hal::AdaptRecord rec{};
    if (!ems::hal::nvm_load_adapt(&rec)) { return false; }
    if ((rec.flags & ems::hal::kAdaptFlagIdleValid) != 0u) {
        torque_idle_learned_restore(rec.idle_learned_x10);
    }
    for (uint8_t c = 0u; c < 4u; ++c) {
        misfire_set_total(c, rec.misfire_total[c]);
    }
    return true;
}

void adapt_retention_save() noexcept {
    ems::hal::AdaptRecord rec{};
    int16_t ofs = 0;
    if (torque_idle_learned_get(&ofs)) {
        rec.flags = ems::hal::kAdaptFlagIdleValid;
        rec.idle_learned_x10 = ofs;
    }
    for (uint8_t c = 0u; c < 4u; ++c) {
        rec.misfire_total[c] = misfire_get_total(c);
    }
    (void)ems::hal::nvm_save_adapt(&rec);
}

}  // namespace ems::engine
