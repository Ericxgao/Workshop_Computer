#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {


inline int32_t abs_q15(int32_t x) { return x >= 0 ? x : -x; }
inline int32_t min_q15(int32_t a, int32_t b) { return a < b ? a : b; }
inline int32_t max_q15(int32_t a, int32_t b) { return a > b ? a : b; }

// Comparator cross-modulation as in Warps' ALGORITHM_COMPARATOR
// Selects between 4 modes with smooth interpolation controlled by parameter p (Q15):
// 0: direct      -> min(mod, car)
// 1: threshold   -> (car > 0.05 ? car : mod)
// 2: window      -> arg with larger |.| (i.e., choose mod or car by magnitude)
// 3: window_2    -> +|mod| if |mod|>|car| else -|car|
inline int32_t process_comparator_q15(int32_t mod_q15,
                                      int32_t car_q15,
                                      int32_t parameter_q15) {
    // Precompute sequence entries
    int32_t direct = min_q15(mod_q15, car_q15);
    int32_t threshold = (car_q15 > Q15_0P05) ? car_q15 : mod_q15;
    int32_t am = abs_q15(mod_q15);
    int32_t ac = abs_q15(car_q15);
    int32_t window = (am > ac) ? mod_q15 : car_q15;
    int32_t window2 = (am > ac) ? am : -ac; // signed magnitude choice

    // x in [0,3) with Q15 frac for interpolation
    int32_t x_q15 = parameter_q15 * 3; // Q15
    int idx = x_q15 >> 15;             // 0..2
    if (idx < 0) idx = 0; else if (idx > 2) idx = 2;
    int32_t frac = x_q15 & 0x7FFF;     // Q15 fractional

    int32_t seq[4] = { direct, threshold, window, window2 };
    int32_t a = seq[idx];
    int32_t b = seq[idx + 1];
    int32_t diff = b - a;
    int32_t interp = a + mul_q15(diff, frac);
    return interp;
}

} // namespace cc_dsp


