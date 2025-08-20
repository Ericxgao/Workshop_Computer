#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {

// Approximation of diode non-linearity (Q1.15) from Warps paper model:
// dead = max(0, |x| - 2/3)
// y = K * (2*dead)^2 * sign(x)
// where K ~= 0.043247658

constexpr int32_t Q15_TWO_THIRDS = 21845;        // ~0.6666667 in Q15
constexpr int32_t Q15_DIODE_COEFF = 1418;        // ~0.043247658 in Q15

inline int32_t saturate_q15_shift_left(int32_t x, int shift) {
    int64_t v = static_cast<int64_t>(x) << shift;
    if (v > Q15_MAX) return Q15_MAX;
    if (v < Q15_MIN) return Q15_MIN;
    return static_cast<int32_t>(v);
}

inline int32_t diode_q15(int32_t x_q15) {
    int32_t sign = (x_q15 >= 0) ? 1 : -1;
    int32_t ax = x_q15 >= 0 ? x_q15 : -x_q15;
    // dead0 = max(0, |x|-2/3)
    int32_t dead0 = ax - Q15_TWO_THIRDS;
    if (dead0 < 0) dead0 = 0;
    // dead1 = 2*dead0
    int32_t dead1 = dead0 << 1;
    // dead2 = (dead1)^2 in Q15
    int32_t dead2 = mul_q15(dead1, dead1);
    // y = coeff * dead2 * sign
    int32_t y = mul_q15(Q15_DIODE_COEFF, dead2);
    return sign > 0 ? y : -y;
}

// Simple soft limiter: y = x / (1 + |x|)
inline int32_t soft_limit_q15(int32_t x_q15) {
    int32_t ax = x_q15 >= 0 ? x_q15 : -x_q15;
    int32_t denom = Q15_ONE + ax; // Q15
    // Compute (x << 15) / denom to maintain Q15 scale
    int64_t num = static_cast<int64_t>(x_q15) << 15;
    if (denom == 0) return 0;
    int32_t y = static_cast<int32_t>(num / denom);
    return y;
}

// Analog ring modulation (Q1.15):
// carrier *= 2;
// ring = diode(mod + carrier) + diode(mod - carrier);
// ring *= (4.0 + parameter * 24.0);
// return soft_limit(ring);
inline int32_t process_analog_ring_q15(int32_t mod_q15,
                                       int32_t car_q15,
                                       int32_t parameter_q15) {
    // carrier *= 2 with saturation
    int32_t car2 = saturate_q15_shift_left(car_q15, 1);

    // Sum and diff
    int32_t sum = mod_q15 + car2;
    if (sum > Q15_MAX) sum = Q15_MAX;
    if (sum < Q15_MIN) sum = Q15_MIN;
    int32_t diff = mod_q15 - car2;
    if (diff > Q15_MAX) diff = Q15_MAX;
    if (diff < Q15_MIN) diff = Q15_MIN;

    int32_t ring = diode_q15(sum) + diode_q15(diff);
    // Gain = 4 + 24 * parameter
    int32_t gain_q15 = (4 * Q15_ONE) + (parameter_q15 * 24);
    // ring * gain (Q15)
    int32_t ring_amp = static_cast<int32_t>((static_cast<int64_t>(ring) * gain_q15) >> 15);
    // Soft limit
    return soft_limit_q15(ring_amp);
}

} // namespace cc_dsp


