#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {

// Q1.15 fixed-point helpers and fold algorithm

constexpr int32_t Q15_OFFSET_0P02 = 655;     // 0.02 in Q15

inline int32_t fold_reflect_q15(int32_t x) {
    while (x > Q15_MAX) {
        x = (Q15_MAX << 1) - x;
    }
    while (x < Q15_MIN) {
        x = (Q15_MIN << 1) - x;
    }
    return x;
}

// conversions now provided by fixedpoint_int.h

// Algorithms we can select between via the main knob
enum class Algorithm : uint8_t {
    Fold = 0,
    Count
};

// Two-parameter fold (p1, p2), adapted from Warps' ALGORITHM_FOLD
inline int32_t process_fold_q15(int32_t x1_q15, int32_t x2_q15,
                                int32_t p1_q15, int32_t p2_q15) {
    // sum = x1 + x2 + 0.25 * x1 * x2
    int32_t sum = x1_q15 + x2_q15;
    int32_t prod_quarter_q15 = static_cast<int32_t>((static_cast<int64_t>(x1_q15) * x2_q15) >> 17);
    sum += prod_quarter_q15;

    // gain = 0.02 + p1
    int32_t gain_q15 = p1_q15 + Q15_OFFSET_0P02;
    if (gain_q15 < 0) gain_q15 = 0;
    int32_t driven = static_cast<int32_t>((static_cast<int64_t>(sum) * gain_q15) >> 15);

    // add offset
    int32_t driven_off = driven + p2_q15;

    // reflection folding as LUT approximation
    return fold_reflect_q15(driven_off);
}

inline int32_t process_algorithm_q15(Algorithm algo,
                                     int32_t x1_q15, int32_t x2_q15,
                                     int32_t p1_q15, int32_t p2_q15) {
    switch (algo) {
        case Algorithm::Fold:
        default:
            return process_fold_q15(x1_q15, x2_q15, p1_q15, p2_q15);
    }
}

} // namespace cc_dsp


