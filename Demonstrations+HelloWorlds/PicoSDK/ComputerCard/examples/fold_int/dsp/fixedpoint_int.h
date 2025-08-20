#pragma once
#include <cstdint>

namespace cc_dsp {

// Q1.15 fixed-point constants
constexpr int32_t Q15_ONE = 32768;   // 1.0 in Q15
constexpr int32_t Q15_MAX = 32767;   // max positive in Q15
constexpr int32_t Q15_MIN = -32768;  // min negative in Q15
constexpr int32_t Q15_0P05 = 1638;   // ~0.05 in Q15
constexpr int32_t Q15_0P8  = 26214;  // ~0.8 in Q15

// Multiply two Q1.15 values with 64-bit intermediate
inline int32_t mul_q15(int32_t a_q15, int32_t b_q15) {
    int64_t t = static_cast<int64_t>(a_q15) * static_cast<int64_t>(b_q15);
    return static_cast<int32_t>(t >> 15);
}

// Convert signed 12-bit audio sample (-2048..2047) to Q1.15 by left shift
inline int32_t audio12_to_q15(int16_t s12) {
    return static_cast<int32_t>(s12) << 4;
}

// Convert Q1.15 to signed 12-bit audio with rounding and clipping
inline int16_t q15_to_audio12(int32_t q15) {
    int32_t v = (q15 + (q15 >= 0 ? 8 : -8)) >> 4;
    if (v < -2048) v = -2048;
    if (v >  2047) v =  2047;
    return static_cast<int16_t>(v);
}

// Convert 12-bit unipolar knob (0..4095) to Q1.15 ~[0..1)
inline int32_t knob_to_q15(int32_t knob12) {
    return knob12 << 3;
}

} // namespace cc_dsp


