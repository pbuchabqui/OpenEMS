#include "engine/transient_fuel.h"
#include "engine/xtau_autocalib.h"

namespace ems::engine {

void transient_fuel_reset() noexcept {
    // O filme de parede vive em xtau_autocalib (g_wall_state).
    xtau_wall_fuel_reset();
}

}  // namespace ems::engine
