#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {

// Soft limiter: y = x / (1 + |x|) in Q15
inline int32_t soft_limit_q15_digital(int32_t x_q15) {
    int32_t ax = x_q15 >= 0 ? x_q15 : -x_q15;
    int32_t denom = Q15_ONE + ax; // Q15
    if (denom == 0) return 0;
    int64_t num = static_cast<int64_t>(x_q15) << 15;
    return static_cast<int32_t>(num / denom);
}

// Digital ring modulation in Q15 (from Warps):
// ring = 4 * x1 * x2 * (1 + 8 * parameter)
// return ring / (1 + |ring|)
inline int32_t process_digital_ring_q15(int32_t x1_q15,
                                        int32_t x2_q15,
                                        int32_t parameter_q15) {
    // prod = x1*x2 in Q30, then *4 -> Q32
    int64_t prod_q32 = (static_cast<int64_t>(x1_q15) * x2_q15) << 2;
    // gain = 1 + 8*parameter (Q15)
    int32_t gain_q15 = Q15_ONE + (parameter_q15 << 3);
    if (gain_q15 < 0) gain_q15 = 0; // safety
    // scale: Q32 * Q15 >> 32 -> Q15
    int32_t ring_q15 = static_cast<int32_t>((prod_q32 * static_cast<int64_t>(gain_q15)) >> 32);
    return soft_limit_q15_digital(ring_q15);
}

} // namespace cc_dsp


