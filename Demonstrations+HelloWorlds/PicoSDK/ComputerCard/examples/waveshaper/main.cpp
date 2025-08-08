#include "ComputerCard.h"
#include <cmath>

// Multi-mode waveshaper (simplified)
// Modes (selected by Main knob):
//   0: Bitcrush + Downsample  (X=bits 4..12, Y=rate 1..32)
//   1: Chebyshev Tn           (X=order 1..8, Y=amount 0..1)
//   2: Wavefold (triangle)    (X=threshold, Y=pre-gain 1x..2x into tanh)
// Audio: In 1 -> shaped -> Out 1 & 2

class Waveshaper : public ComputerCard
{
    static constexpr int NUM_MODES = 3;

    // Bitcrush/downsample state
    int16_t heldSample = 0;
    int downsampleCounter = 0;

    // Chebyshev order smoothing (prevents audible stepping without crossfading)
    int chebOrder = 1;         // current effective order used
    int chebSlewCounter = 0;   // countdown to next step toward target

    // Fixed-point helpers (Q12)
    static constexpr int Q = 12;
    static constexpr int ONE_Q = (1 << Q);

    static inline int16_t clamp12(int32_t v)
    {
        if (v < -2048) return -2048;
        if (v > 2047) return 2047;
        return (int16_t)v;
    }

    static inline int32_t iabs32(int32_t v) { return v >= 0 ? v : -v; }

    static inline int32_t mul_q12(int32_t a, int32_t b)
    {
        return (int32_t)((int64_t)a * (int64_t)b >> Q);
    }

    static inline int32_t mix_wet(int32_t dry, int32_t wet, int32_t wet_q12)
    {
        // out = (1-wet)*dry + wet*wet
        int32_t inv = ONE_Q - wet_q12;
        return (int32_t)(((int64_t)inv * dry + (int64_t)wet_q12 * wet) >> Q);
    }

public:
    virtual void ProcessSample()
    {
        // Controls
        int32_t main_raw = KnobVal(Knob::Main); // 0..4095
        int32_t x_raw = KnobVal(Knob::X);       // 0..4095
        int32_t y_raw = KnobVal(Knob::Y);       // 0..4095

        // Mode from Main knob
        int mode = (main_raw * NUM_MODES) >> 12; // 0..NUM_MODES-1
        if (mode < 0) mode = 0;
        if (mode >= NUM_MODES) mode = NUM_MODES - 1;

        // Audio input
        int32_t x = AudioIn1(); // -2048..2047
        int32_t y = 0;          // output accumulator (int domain)

        // Helpers: map knobs
        auto map_thresh = [](int32_t k, int32_t minT, int32_t maxT) -> int32_t {
            if (minT < 1) minT = 1;
            if (maxT < minT) maxT = minT;
            return minT + ((k * (maxT - minT)) >> 12);
        };
        auto map_wet_q = [](int32_t k) -> int32_t { return (k <= 0) ? 0 : (k >= 4095 ? ONE_Q : (k << 0)); };
        auto map_gain_q = [](int32_t k, int32_t min_q12, int32_t max_q12) -> int32_t {
            return min_q12 + (int32_t)(((int64_t)k * (max_q12 - min_q12)) >> 12);
        };

        switch (mode)
        {
        case 2: // Wavefold (triangle) with pre-gain into tanh limiter
        {
            // Allow much lower threshold for more folds; invert X so higher X => smaller T (more folding)
            int32_t T = map_thresh(4095 - x_raw, 16, 2048);
            // Pre-gain from Y: 1.0x .. 2.0x (Q12)
            int32_t g_q = map_gain_q(y_raw, ONE_Q, 2 * ONE_Q);
            // z = (g * x) / 2048 in Q12
            int32_t z_q = (int32_t)(((int64_t)g_q * x) >> 11);
            // tanh approx: tanh(z) ~ z*(27+z^2)/(27+9 z^2), limit |z| <= 2.5
            int32_t z_abs = iabs32(z_q);
            int32_t z_lim = (int32_t)(2.5f * ONE_Q);
            if (z_abs > z_lim) z_q = (z_q >= 0) ? z_lim : -z_lim;
            int32_t z2_q = mul_q12(z_q, z_q);
            int32_t num_q = (int32_t)((((int64_t)z_q) * ((27 * ONE_Q) + z2_q)) >> Q); // Q12
            int32_t den_q = (27 * ONE_Q) + (9 * z2_q);
            int32_t tanh_q = (den_q != 0) ? (int32_t)((((int64_t)num_q) << Q) / den_q) : z_q; // Q12
            int32_t a = clamp12((int32_t)(((int64_t)tanh_q * 2047) >> Q));
            // Lower safety clamp so threshold can be very small
            if (T < 16) T = 16;
            while (a > T || a < -T)
            {
                if (a > T)      a = 2 * T - a;
                else if (a < -T) a = -2 * T - a;
            }
            // Automatic level compensation: scale folded range [-T, T] back to ~full-scale [-2047, 2047]
            // Keep T >= 16 above, so division is safe and bounded
            int32_t compensated = (int32_t)(((int64_t)a * 2047) / T);
            y = clamp12(compensated);
            break;
        }
        case 1: // Chebyshev Tn (order slewed for continuity)
        {
            // Target order from X: 1..32
            int n_target = 1 + ((x_raw * 32) >> 12);
            if (n_target < 1) n_target = 1;
            if (n_target > 32) n_target = 32;

            // Slew the actual order toward target without crossfading
            int diff = n_target - chebOrder;
            if (diff != 0)
            {
                if (chebSlewCounter <= 0)
                {
                    chebOrder += (diff > 0) ? 1 : -1;
                    // Step every ~64 samples; larger diffs speed up slightly
                    int step = 64 - ((diff > 0 ? diff : -diff) > 8 ? 16 : 0);
                    if (step < 16) step = 16; // clamp minimum
                    chebSlewCounter = step;
                }
                else
                {
                    chebSlewCounter--;
                }
            }

            int n = chebOrder;
            int32_t a_q = map_wet_q(y_raw);  // 0..4096
            // u in Q15
            int32_t u_q15 = (x << 15) / 2048; // -32768..32767
            int32_t Tn_2 = (1 << 15);         // 1.0 in Q15
            int32_t Tn_1 = u_q15;
            int32_t Tn = (n == 1) ? Tn_1 : Tn_2;
            for (int k = 2; k <= n; ++k)
            {
                // Tn = 2*u*Tn_1 - Tn_2
                int32_t two_u_T1 = (int32_t)((((int64_t)2 * u_q15) * Tn_1) >> 15);
                Tn = two_u_T1 - Tn_2;
                Tn_2 = Tn_1;
                Tn_1 = Tn;
            }
            // Scale to 12-bit and apply amplitude
            int32_t base12 = (int32_t)(((int64_t)Tn * 2047) >> 15);
            int32_t out = (int32_t)(((int64_t)a_q * base12) >> Q);
            y = clamp12(out);
            break;
        }
        case 0: // Bitcrush + downsample
        {
            // Invert X so higher X => fewer bits (more crushing). Expand to 1..12 bits.
            int bits = 1 + (((4095 - x_raw) * 11) >> 12); // 1..12
            if (bits < 1) bits = 1;  // safety
            if (bits > 12) bits = 12; // safety
            int shift = 12 - bits;
            int factor = 1 + ((y_raw * 31) >> 12); // 1..32

            if (downsampleCounter <= 0)
            {
                int32_t q = (x >> shift) << shift; // quantize
                heldSample = clamp12(q);
                downsampleCounter = factor;
            }
            else
            {
                downsampleCounter--;
            }
            y = heldSample;
            break;
        }
        default:
            y = x; // passthrough
            break;
        }

        // Outputs
        AudioOut1((int16_t)y);
        AudioOut2((int16_t)y);

        // LEDs: indicate mode and parameter levels
        for (int i = 0; i < 6; ++i) LedOff(i);
        LedOn(mode % 6, true);                       // mode indicator
        LedBrightness(1, (uint16_t)x_raw);           // X amount
        LedBrightness(3, (uint16_t)y_raw);           // Y amount
        int32_t mag = iabs32(x);
        if (mag > 2048) mag = 2048;
        LedBrightness(5, (uint16_t)((mag * 4095) / 2048)); // input magnitude
    }
};

int main()
{
    set_sys_clock_khz(200000, true);
    Waveshaper ws;
    ws.EnableNormalisationProbe();
    ws.Run();
}


