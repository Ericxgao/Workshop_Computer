#pragma once
#include <cstdint>
#include "fixedpoint_int.h"
#include <cmath>

namespace cc_dsp {

// Simple Q15 frequency shifter approximation:
// - Quadrature carrier from a 512-point sine LUT and phase accumulator
// - Hilbert-like 90° shift of modulator via two 1st-order all-pass sections
// - Upper/lower sideband crossfade by p2
// p1 maps to shift frequency (bipolar around 0.5)

static inline int16_t sine_lut512(uint16_t index) {
    static int16_t lut[512];
    static bool inited = false;
    if (!inited) {
        for (int i = 0; i < 512; ++i) {
            double angle = (2.0 * M_PI * i) / 512.0;
            lut[i] = static_cast<int16_t>(std::lround(32000.0 * std::sin(angle)));
        }
        inited = true;
    }
    return lut[index & 511];
}

static inline int16_t sin_from_phase(uint32_t phase32) {
    // 32-bit phase to 9-bit index with linear interpolation
    uint32_t index = phase32 >> 23; // 9-bit index
    uint32_t frac16 = (phase32 & 0x7FFFFF) >> 7; // 16-bit frac
    int32_t s1 = sine_lut512(static_cast<uint16_t>(index));
    int32_t s2 = sine_lut512(static_cast<uint16_t>(index + 1));
    int32_t y = (s2 * static_cast<int32_t>(frac16) + s1 * static_cast<int32_t>(65536 - frac16)) >> 16;
    if (y > 32767) y = 32767; else if (y < -32768) y = -32768;
    return static_cast<int16_t>(y);
}

struct FreqShifterState {
    // All-pass states for rough Hilbert transform
    int32_t ap1_x1 = 0, ap1_y1 = 0;
    int32_t ap2_x1 = 0, ap2_y1 = 0;
    // Oscillator phase 0..1024*256
    uint32_t phase = 0;
};

inline int32_t allpass_q15(int32_t x_q15, int32_t a_q15, int32_t &x1, int32_t &y1) {
    // y = -a*x + x1 + a*y1
    int32_t ax = mul_q15(a_q15, x_q15);
    int32_t ay1 = mul_q15(a_q15, y1);
    int32_t y = -ax + x1 + ay1;
    x1 = x_q15;
    y1 = y;
    if (y > Q15_MAX) y = Q15_MAX; else if (y < Q15_MIN) y = Q15_MIN;
    return y;
}

inline int32_t process_freq_shifter_q15(FreqShifterState &st,
                                        int32_t x1_q15, // input 1
                                        int32_t x2_q15, // input 2
                                        int32_t p1_q15, // frequency control 0..1
                                        int32_t p2_q15  // up/down crossfade 0..1
                                        ) {
    // Choose active input
    int32_t ax1 = x1_q15 >= 0 ? x1_q15 : -x1_q15;
    int32_t ax2 = x2_q15 >= 0 ? x2_q15 : -x2_q15;
    int32_t mod_src = (ax1 >= ax2) ? x1_q15 : x2_q15;

    // Map p1 to 0..max_hz with cubic for resolution
    const uint32_t max_hz = 4000;
    int32_t p1_sq = mul_q15(p1_q15, p1_q15);
    int32_t p1_cu = mul_q15(p1_sq, p1_q15);
    uint32_t freq_hz = static_cast<uint32_t>((static_cast<int64_t>(max_hz) * p1_cu) >> 15);
    // phase increment per sample for 1024*256 table steps at 48kHz
    const uint32_t sample_rate = 48000;
    int32_t inc = static_cast<int32_t>((static_cast<int64_t>(freq_hz) * (1024u << 8)) / sample_rate);
    st.phase = static_cast<uint32_t>(static_cast<int32_t>(st.phase) + inc);
    int16_t sin_q15 = sin_from_phase(st.phase);
    int16_t cos_q15 = sin_from_phase(st.phase + 0x40000000u); // +90 degrees

    // Hilbert-like for modulator: two cascaded all-pass with slightly different a
    // Coefficients ~0.5 and ~0.2 in Q15
    const int32_t a1 = 16384; // 0.5
    const int32_t a2 = 6554;  // 0.2
    int32_t ap1 = allpass_q15(mod_src, a1, st.ap1_x1, st.ap1_y1);
    int32_t ap2 = allpass_q15(ap1, a2, st.ap2_x1, st.ap2_y1);
    int32_t mod_i = mod_src;     // in-phase approx
    int32_t mod_q = ap2;        // quadrature approx

    // Multiply with quadrature carrier
    int32_t a = mul_q15(cos_q15, mod_i);
    int32_t b = mul_q15(sin_q15, mod_q);
    int32_t up = a - b;
    int32_t down = a + b;

    // Crossfade between up/down with p2
    if (p2_q15 < 0) p2_q15 = 0; else if (p2_q15 > Q15_ONE) p2_q15 = Q15_ONE;
    int32_t one_minus = Q15_ONE - p2_q15;
    int32_t y = static_cast<int32_t>((static_cast<int64_t>(up) * one_minus + static_cast<int64_t>(down) * p2_q15) >> 15);
    if (y > Q15_MAX) y = Q15_MAX; else if (y < Q15_MIN) y = Q15_MIN;
    return y;
}

} // namespace cc_dsp


