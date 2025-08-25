#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {

inline int32_t process_nop_q15(int32_t x1_q15,
                               int32_t /*x2_q15*/,
                               int32_t /*p_q15*/) {
    return x1_q15; // passthrough modulator
}

} // namespace cc_dsp




