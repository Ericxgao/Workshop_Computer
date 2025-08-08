// Integer (Q15) State Variable Filter (Chamberlin form)
// Signals are Q15 internally; inputs/outputs are 12-bit signed integers (-2048..2047)
// Coefficients f and q are updated at control rate (floating point allowed there); per-sample path is int32-only.

#pragma once

#include <cstdint>
#include <cmath>

class StateVariableFilterInt {
public:
    enum class Mode { Lowpass, Bandpass, Highpass, Notch };

    StateVariableFilterInt() { setSampleRate(48000.0f); setMode(Mode::Lowpass); setCutoffHz(800.0f); setQ(2.0f); reset(); }

    void setSampleRate(float fs)
    {
        if (fs <= 0.0f) fs = 48000.0f;
        sampleRate = fs;
        updateCoeffs();
    }

    void setMode(Mode m) { mode = m; }

    void setCutoffHz(float fc)
    {
        if (fc < 5.0f) fc = 5.0f;
        if (fc > 10000.0f) fc = 10000.0f;
        cutoffHz = fc;
        updateCoeffs();
    }

    void setQ(float q)
    {
        // Chamberlin 'q' term is damping ~ 1/Q (smaller => more resonance)
        if (q < 0.3f) q = 0.3f;
        if (q > 12.0f) q = 12.0f;
        float q_ch = 1.0f / q;           // 0.083..3.33 -> clamp below
        if (q_ch < 0.01f) q_ch = 0.01f;
        if (q_ch > 1.98f) q_ch = 1.98f;
        q_ch_q15 = static_cast<int32_t>(q_ch * 32768.0f + 0.5f);
    }

    void setResonance01(float r)
    {
        if (r < 0.0f) r = 0.0f;
        if (r > 1.0f) r = 1.0f;
        // Map 0..1 to Q in [0.5 .. 8], then q_ch = 1/Q
        float Q = 0.5f + r * 7.5f;
        float q_ch = 1.0f / Q;
        if (q_ch < 0.01f) q_ch = 0.01f;
        if (q_ch > 1.98f) q_ch = 1.98f;
        q_ch_q15 = static_cast<int32_t>(q_ch * 32768.0f + 0.5f);
    }

    // Convert frequency (Hz) to Q15 coefficient for audio-rate modulation
    // Avoids floating-point trigonometry in the audio callback
    static int32_t frequencyToFQ15(float freq_hz, float sample_rate = 48000.0f)
    {
        const float pi = 3.14159265358979323846f;
        float f = 2.0f * std::sinf(pi * freq_hz / sample_rate);
        if (!std::isfinite(f) || f < 0.0f) f = 0.0f;
        if (f > 1.98f) f = 1.98f; // keep below 2 for stability
        int32_t f_q15 = static_cast<int32_t>(f * 32768.0f + 0.5f);
        if (f_q15 < 0) f_q15 = 0;
        if (f_q15 > 65534) f_q15 = 65534; // ~just below 2.0 in Q15
        return f_q15;
    }

    void reset()
    {
        low_q15 = 0;
        band_q15 = 0;
    }

    // Process one 12-bit sample
    inline int16_t process(int16_t x12)
    {
        // Convert input to Q15
        int32_t x = static_cast<int32_t>(x12) << 4; // -2048..2047 -> ~-32768..32752

        // Chamberlin SVF (integer):
        // low += f * band
        // high = x - low - q * band
        // band += f * high
        // notch = high + low
        // pick mode output

        // low += (f * band)
        int32_t f_band = static_cast<int32_t>((static_cast<int64_t>(f_q15) * band_q15) >> 15);
        low_q15 = sat_q15(low_q15 + f_band);

        // high = x - low - q * band
        int32_t q_band = static_cast<int32_t>((static_cast<int64_t>(q_ch_q15) * band_q15) >> 15);
        int32_t high_q15 = sat_q15(x - low_q15 - q_band);

        // band += f * high
        int32_t f_high = static_cast<int32_t>((static_cast<int64_t>(f_q15) * high_q15) >> 15);
        band_q15 = sat_q15(band_q15 + f_high);

        int32_t out_q15 = 0;
        switch (mode)
        {
        case Mode::Lowpass:  out_q15 = low_q15;               break;
        case Mode::Bandpass: out_q15 = band_q15;              break;
        case Mode::Highpass: out_q15 = high_q15;              break;
        case Mode::Notch:    out_q15 = sat_q15(high_q15 + low_q15); break;
        }

        // Back to 12-bit
        int32_t y12 = out_q15 >> 4; // Q15 -> 12-bit
        if (y12 < -2048) y12 = -2048;
        if (y12 > 2047) y12 = 2047;
        return static_cast<int16_t>(y12);
    }

    // Fast version with pre-calculated f coefficient for audio-rate modulation
    // f_mod_q15: modulated f coefficient in Q15 format (0..~65534)
    inline int16_t processWithFMod(int16_t x12, int32_t f_mod_q15)
    {
        // Convert input to Q15
        int32_t x = static_cast<int32_t>(x12) << 4; // -2048..2047 -> ~-32768..32752

        // Clamp f_mod_q15 to safe range
        if (f_mod_q15 < 0) f_mod_q15 = 0;
        if (f_mod_q15 > 65534) f_mod_q15 = 65534;

        // Chamberlin SVF with modulated f coefficient
        // low += f_mod * band
        int32_t f_band = static_cast<int32_t>((static_cast<int64_t>(f_mod_q15) * band_q15) >> 15);
        low_q15 = sat_q15(low_q15 + f_band);

        // high = x - low - q * band
        int32_t q_band = static_cast<int32_t>((static_cast<int64_t>(q_ch_q15) * band_q15) >> 15);
        int32_t high_q15 = sat_q15(x - low_q15 - q_band);

        // band += f_mod * high
        int32_t f_high = static_cast<int32_t>((static_cast<int64_t>(f_mod_q15) * high_q15) >> 15);
        band_q15 = sat_q15(band_q15 + f_high);

        int32_t out_q15 = 0;
        switch (mode)
        {
        case Mode::Lowpass:  out_q15 = low_q15;               break;
        case Mode::Bandpass: out_q15 = band_q15;              break;
        case Mode::Highpass: out_q15 = high_q15;              break;
        case Mode::Notch:    out_q15 = sat_q15(high_q15 + low_q15); break;
        }

        // Back to 12-bit
        int32_t y12 = out_q15 >> 4; // Q15 -> 12-bit
        if (y12 < -2048) y12 = -2048;
        if (y12 > 2047) y12 = 2047;
        return static_cast<int16_t>(y12);
    }

    // Dual-input version (matches Teensy AudioFilterStateVariable behavior)
    // Automatically mixes input1 and input2, then processes through filter
    inline int16_t process(int16_t input1, int16_t input2)
    {
        // Mix the inputs (Teensy behavior: multiple inputs are automatically summed)
        int32_t mixed = static_cast<int32_t>(input1) + static_cast<int32_t>(input2);
        
        // Clamp to prevent overflow
        if (mixed > 2047) mixed = 2047;
        if (mixed < -2048) mixed = -2048;
        
        return process(static_cast<int16_t>(mixed));
    }

private:
    static inline int32_t sat_q15(int32_t v)
    {
        if (v < -32768) return -32768;
        if (v > 32767)  return 32767;
        return v;
    }

    void updateCoeffs()
    {
        // Compute f = 2*sin(pi*fc/fs) in float at control rate, then convert to Q15
        const float pi = 3.14159265358979323846f;
        float f = 2.0f * std::sinf(pi * cutoffHz / sampleRate);
        if (!std::isfinite(f) || f < 0.0f) f = 0.0f;
        if (f > 1.98f) f = 1.98f; // keep below 2 for stability
        f_q15 = static_cast<int32_t>(f * 32768.0f + 0.5f);
        if (f_q15 < 0) f_q15 = 0;
        if (f_q15 > 65534) f_q15 = 65534; // ~just below 2.0 in Q15
    }

    // Control parameters
    float sampleRate = 48000.0f;
    float cutoffHz = 800.0f;
    Mode mode = Mode::Lowpass;

    // Coefficients/state (Q15)
    int32_t f_q15 = 0;     // 0..~65534 (~0..2 in Q15)
    int32_t q_ch_q15 = 0;  // 0..~1.98 in Q15
    int32_t low_q15 = 0;
    int32_t band_q15 = 0;
};


