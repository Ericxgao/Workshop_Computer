#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {

constexpr int32_t Q15_0P7 = 22938; // round(0.7 * 32768)

// XOR modulation in Q15 mirroring Warps' ALGORITHM_XOR
// mod = (int16_t(x1) ^ int16_t(x2))  // bitwise XOR on 16-bit signed
// sum = 0.7 * (x1 + x2)
// out = sum + (mod - sum) * parameter
inline int32_t process_xor_q15(int32_t x1_q15,
                               int32_t x2_q15,
                               int32_t parameter_q15) {
    // Clamp to 16-bit signed and XOR
    int32_t c1 = x1_q15;
    if (c1 > Q15_MAX) c1 = Q15_MAX; else if (c1 < Q15_MIN) c1 = Q15_MIN;
    int32_t c2 = x2_q15;
    if (c2 > Q15_MAX) c2 = Q15_MAX; else if (c2 < Q15_MIN) c2 = Q15_MIN;
    int16_t s1 = static_cast<int16_t>(c1);
    int16_t s2 = static_cast<int16_t>(c2);
    int16_t xr = s1 ^ s2;
    int32_t mod_q15 = static_cast<int32_t>(xr); // already Q15 scale

    // Sum term scaled by 0.7
    int32_t sum_q15 = x1_q15 + x2_q15;
    int32_t sum_scaled = mul_q15(sum_q15, Q15_0P7);

    // Linear interpolation between sum and mod by parameter
    int32_t diff = mod_q15 - sum_scaled;
    int32_t mix = sum_scaled + mul_q15(diff, parameter_q15);
    return mix;
}

} // namespace cc_dsp


