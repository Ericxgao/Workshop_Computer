#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {


inline int32_t abs_q15_c8(int32_t x) { return x >= 0 ? x : -x; }
inline int32_t min_q15_c8(int32_t a, int32_t b) { return a < b ? a : b; }

// Comparator8 morph as in Warps: piecewise pair (y1,y2) over 7 segments, then interpolate
inline int32_t process_comparator8_q15(int32_t mod_q15,
                                       int32_t car_q15,
                                       int32_t parameter_q15) {
    int32_t am = abs_q15_c8(mod_q15);
    int32_t ac = abs_q15_c8(car_q15);

    // map parameter in [0,1) to x in [0,7)
    int32_t x_q15 = parameter_q15 * 7; // Q15
    int idx = x_q15 >> 15;             // 0..6
    if (idx < 0) idx = 0; else if (idx > 6) idx = 6;
    int32_t frac = x_q15 - (idx << 15); // Q15 fractional

    int32_t y1, y2;
    switch (idx) {
        case 0:
            y1 = mod_q15 + car_q15; // may saturate naturally in downstream
            if (y1 > Q15_MAX) y1 = Q15_MAX; else if (y1 < Q15_MIN) y1 = Q15_MIN;
            y2 = min_q15_c8(mod_q15, car_q15);
            break;
        case 1:
            y1 = min_q15_c8(mod_q15, car_q15);
            // (arg with larger |.|) * 2 - 1
            y2 = (am > ac ? am : ac);
            // scale 2*x - 1
            y2 = (y2 << 1) - Q15_ONE;
            if (y2 > Q15_MAX) y2 = Q15_MAX; else if (y2 < Q15_MIN) y2 = Q15_MIN;
            break;
        case 2:
            // y1: same as previous y2
            y1 = (am > ac ? am : ac);
            y1 = (y1 << 1) - Q15_ONE;
            if (y1 > Q15_MAX) y1 = Q15_MAX; else if (y1 < Q15_MIN) y1 = Q15_MIN;
            // y2: mod < car ? -car : mod
            y2 = (mod_q15 < car_q15) ? -car_q15 : mod_q15;
            break;
        case 3:
            // y1: mod < car ? -car : mod
            y1 = (mod_q15 < car_q15) ? -car_q15 : mod_q15;
            // y2: arg with larger |.|
            y2 = (am > ac) ? mod_q15 : car_q15;
            break;
        case 4:
            // y1: window by magnitude
            y1 = (am > ac) ? mod_q15 : car_q15;
            // y2: +|mod| if |mod|>|car| else -|car|
            y2 = (am > ac) ? am : -ac;
            break;
        case 5:
            // y1: as above
            y1 = (am > ac) ? am : -ac;
            // y2: threshold on carrier
            y2 = (car_q15 > Q15_0P05) ? car_q15 : mod_q15;
            break;
        default: // 6
            // y1: threshold on carrier
            y1 = (car_q15 > Q15_0P05) ? car_q15 : mod_q15;
            // y2: carrier > 0.05 ? carrier : -|mod|
            y2 = (car_q15 > Q15_0P05) ? car_q15 : -am;
            break;
    }

    int32_t diff = y2 - y1;
    int32_t out = y1 + mul_q15(diff, frac);
    return out;
}

} // namespace cc_dsp


