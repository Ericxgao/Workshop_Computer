#pragma once
#include <cstdint>
#include "fixedpoint_int.h"

namespace cc_dsp {

// Chebyshev-based waveshaper adapted from Warps' ALGORITHM_CHEBYSCHEV
// Two-parameter form:
// x = x1 + x2
// x *= (p2 * 2)
// x clipped to [-1, 1]
// n = p1 * degree (degree = 16)
// Compute T_k(x) recursively with k = floor(n) and T_{k+1}(x), then
// out = T_k + (T_{k+1} - T_k) * frac(n)
// out /= p2; out *= 0.5

inline int32_t process_chebyshev_q15(int32_t x1_q15,
                                     int32_t x2_q15,
                                     int32_t p1_q15,
                                     int32_t p2_q15) {
    // x = x1 + x2
    int32_t x = x1_q15 + x2_q15;
    if (x > Q15_MAX) x = Q15_MAX; else if (x < Q15_MIN) x = Q15_MIN;

    // Map p2 (bipolar Q15 from knob) to unipolar [0..1]
    // p2_uni = (p2 + 1) / 2, then clamp to [eps..1]
    int32_t p2_mag = (p2_q15 + Q15_ONE) >> 1;
    // avoid zero to prevent divide-by-zero and silent region
    const int32_t eps = 64; // ~0.002
    if (p2_mag < eps) p2_mag = eps;
    if (p2_mag > Q15_ONE) p2_mag = Q15_ONE;

    // x *= (p2 * 2)
    int32_t gain_q15 = p2_mag << 1; // up to ~2.0
    if (gain_q15 > (Q15_ONE * 2 - 1)) gain_q15 = (Q15_ONE * 2 - 1);
    x = static_cast<int32_t>((static_cast<int64_t>(x) * gain_q15) >> 15);

    // clip to [-1, 1]
    if (x > Q15_MAX) x = Q15_MAX; else if (x < Q15_MIN) x = Q15_MIN;

    // Degree and n decomposition
    constexpr int degree = 16;
    // n_q15 in Q15 representing n in [0, degree]
    int32_t n_q15 = p1_q15 << 4; // *16
    int n_int = n_q15 >> 15;     // integer part
    if (n_int < 0) n_int = 0; else if (n_int > degree) n_int = degree;
    int32_t n_frac_q15 = n_q15 & 0x7FFF; // fractional in Q15

    // Initialize T1 and T2
    int32_t Tn1 = x; // T1(x) = x
    // T2(x) = 2x^2 - 1
    int32_t x_squared_q15 = mul_q15(x, x);
    int32_t Tn = (x_squared_q15 << 1) - Q15_ONE;
    if (Tn > Q15_MAX) Tn = Q15_MAX; else if (Tn < Q15_MIN) Tn = Q15_MIN;

    // Reduce n by 1 while accumulating recursion so that at exit:
    // Tn1 = T_k, Tn = T_{k+1}, and n_frac_q15 is fractional part
    int steps = n_int - 1;
    while (steps-- > 0) {
        int32_t temp = Tn;
        // T_{k+1} = 2*x*T_k - T_{k-1}
        int32_t two_x_Tn = static_cast<int32_t>((static_cast<int64_t>(mul_q15(x, Tn)) << 1));
        int32_t Tnext = two_x_Tn - Tn1;
        // clamp
        if (Tnext > Q15_MAX) Tnext = Q15_MAX; else if (Tnext < Q15_MIN) Tnext = Q15_MIN;
        Tn1 = temp;
        Tn = Tnext;
    }

    // If n_int == 0: morph between T1 and T2 using n_frac; our init already matches that
    // Interpolate: out = Tn1 + (Tn - Tn1) * frac
    int32_t diff = Tn - Tn1;
    int32_t out = Tn1 + mul_q15(diff, n_frac_q15);

    // out /= p2_mag, then *= 0.5
    out = static_cast<int32_t>((static_cast<int64_t>(out) << 15) / p2_mag);
    out >>= 1; // *0.5

    // final clamp
    if (out > Q15_MAX) out = Q15_MAX; else if (out < Q15_MIN) out = Q15_MIN;
    return out;
}

} // namespace cc_dsp


