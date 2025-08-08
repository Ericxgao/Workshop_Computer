#pragma once

#include <cstdint>
#include <cmath>
#include "../dsp/WaveformOsc.hpp"
#include "../dsp/Waveshaper.hpp"

class CrossModRingSine {
public:
    CrossModRingSine() {
        // Initialize oscillators to match original:
        // sine_fm1.frequency(1100); sine_fm1.amplitude(1);
        // sine_fm2.frequency(1367); sine_fm2.amplitude(1);
        
        osc1_.setSampleRate(48000.0f);
        osc2_.setSampleRate(48000.0f);
        
        osc1_.setShape(WaveformOscillator::Shape::Sine);
        osc2_.setShape(WaveformOscillator::Shape::Sine);
        
        // amplitude(1) = full amplitude
        osc1_.setAmplitudeQ12(4095);
        osc2_.setAmplitudeQ12(4095);
        
        // Set initial frequencies as per original
        osc1_.setFrequencyHz(1100.0f);
        osc2_.setFrequencyHz(1367.0f);
        
        // Initialize previous outputs for cross-modulation
        prev_osc1_out_ = 0;
        prev_osc2_out_ = 0;

        // Initialize chaos shaper (soft saturation) using tanh curve
        // 1025-point table (2^10 + 1) over [-1, 1]
        static float tanhTable[1025];
        static bool tableInit = false;
        if (!tableInit) {
            for (int i = 0; i < 1025; ++i) {
                float x = (static_cast<float>(i) / 1024.0f) * 2.0f - 1.0f; // -1..1
                // gentle drive, normalized to [-1,1]
                float y = std::tanh(2.0f * x) / std::tanh(2.0f);
                tanhTable[i] = y;
            }
            tableInit = true;
        }
        waveshaper_.shape(tanhTable, 1025);
    }
    
    int32_t process(int32_t k1_4095, int32_t k2_4095) {
        // Map knobs to frequencies using original's approach:
        // sine_fm1.frequency(100+(pitch1*8000));  where pitch1 = pow(knob_1, 2)
        // sine_fm2.frequency(60+(pitch2*3000));   where pitch2 = pow(knob_2, 2)
        
        // Convert knobs to 0.0-1.0 range
        float k1_01 = static_cast<float>(k1_4095) / 4095.0f;
        float k2_01 = static_cast<float>(k2_4095) / 4095.0f;
        
        // Apply quadratic response: pitch = knob^2
        float pitch1 = k1_01 * k1_01;
        float pitch2 = k2_01 * k2_01;
        
        // Calculate frequencies
        float freq1 = 100.0f + (pitch1 * 8000.0f);
        float freq2 = 60.0f + (pitch2 * 3000.0f);
        
        // Update base frequencies
        osc1_.setFrequencyHz(freq1);
        osc2_.setFrequencyHz(freq2);

        // Chaos control from k2: squared for sensitivity
        float chaos = k2_01 * k2_01; // 0..1

        // FM depth: up to ~1 octave of deviation scaled by chaos
        // depth_hz = base * (2^1 - 1) * chaos = base * 1.0 * chaos
        float depth1_hz = freq1 * chaos;
        float depth2_hz = freq2 * chaos;
        if (depth1_hz < 0.0f) depth1_hz = 0.0f;
        if (depth2_hz < 0.0f) depth2_hz = 0.0f;
        fmScale1_q16_16_ = static_cast<int32_t>(depth1_hz * 65536.0f + 0.5f);
        fmScale2_q16_16_ = static_cast<int32_t>(depth2_hz * 65536.0f + 0.5f);
        // Cap FM to keep increment positive (80% of base)
        maxFm1_q16_16_ = static_cast<int32_t>((0.8f * freq1) * 65536.0f + 0.5f);
        maxFm2_q16_16_ = static_cast<int32_t>((0.8f * freq2) * 65536.0f + 0.5f);
        if (maxFm1_q16_16_ < 0) maxFm1_q16_16_ = 0; if (maxFm2_q16_16_ < 0) maxFm2_q16_16_ = 0;

        // DC FM component from chaos (pushes into instability)
        int32_t chaos_dc_q16_16 = static_cast<int32_t>(chaos * 65536.0f + 0.5f); // 0..65536

        // Previous outputs normalized to Q16.16 (-1..1)
        int32_t nrm1_q16_16 = static_cast<int32_t>(prev_osc1_out_) * 32;
        int32_t nrm2_q16_16 = static_cast<int32_t>(prev_osc2_out_) * 32;

        // Ring feedback from previous sample, normalized to Q16.16
        int32_t ring_prev_12 = (static_cast<int32_t>(prev_osc1_out_) * static_cast<int32_t>(prev_osc2_out_)) >> 11; // -2048..2047
        int32_t ring_prev_q16_16 = ring_prev_12 * 32; // -65536..65535
        int32_t ring_amt_q16_16 = static_cast<int32_t>((0.6f * chaos) * 65536.0f + 0.5f);
        int32_t ring_contrib_q16_16 = static_cast<int32_t>((static_cast<int64_t>(ring_prev_q16_16) * ring_amt_q16_16) >> 16);

        // Noise contribution in Q16.16 (LFSR-based), scaled by chaos
        int16_t noise12 = nextNoise12_(); // -2048..2047
        int32_t noise_q16_16 = static_cast<int32_t>(noise12) * 32; // -65536..65535
        int32_t noise_amt_q16_16 = static_cast<int32_t>((0.2f * chaos) * 65536.0f + 0.5f);
        int32_t noise_contrib_q16_16 = static_cast<int32_t>((static_cast<int64_t>(noise_q16_16) * noise_amt_q16_16) >> 16);

        // Build FM inputs for each oscillator (cross-mod + DC + ring + noise)
        int32_t fm_in1_norm_q16_16 = nrm2_q16_16 + chaos_dc_q16_16 + ring_contrib_q16_16 + noise_contrib_q16_16;
        int32_t fm_in2_norm_q16_16 = nrm1_q16_16 + chaos_dc_q16_16 + ring_contrib_q16_16 + noise_contrib_q16_16;
        // Clamp normalized FM to [-1, +1] range to avoid extreme indices
        if (fm_in1_norm_q16_16 < -65536) fm_in1_norm_q16_16 = -65536;
        if (fm_in1_norm_q16_16 >  65536) fm_in1_norm_q16_16 =  65536;
        if (fm_in2_norm_q16_16 < -65536) fm_in2_norm_q16_16 = -65536;
        if (fm_in2_norm_q16_16 >  65536) fm_in2_norm_q16_16 =  65536;

        // Convert normalized FM to Hz using scales
        int32_t fm1_hz_q16_16 = static_cast<int32_t>((static_cast<int64_t>(fm_in1_norm_q16_16) * fmScale1_q16_16_) >> 16);
        int32_t fm2_hz_q16_16 = static_cast<int32_t>((static_cast<int64_t>(fm_in2_norm_q16_16) * fmScale2_q16_16_) >> 16);
        // Clamp FM to keep positive increment
        int32_t cap1 = maxFm1_q16_16_;
        int32_t cap2 = maxFm2_q16_16_;
        if (fm1_hz_q16_16 < -cap1) fm1_hz_q16_16 = -cap1;
        if (fm1_hz_q16_16 >  cap1) fm1_hz_q16_16 =  cap1;
        if (fm2_hz_q16_16 < -cap2) fm2_hz_q16_16 = -cap2;
        if (fm2_hz_q16_16 >  cap2) fm2_hz_q16_16 =  cap2;
        
        // Cross-modulation: each oscillator's output modulates the other's frequency
        // Convert previous outputs to FM format (Q16.16 Hz)
        // Scale the previous outputs for FM (similar to CrossModRingSquare)
        int32_t fm1_q16_16 = static_cast<int32_t>(prev_osc2_out_) * 32; // osc2 modulates osc1
        int32_t fm2_q16_16 = static_cast<int32_t>(prev_osc1_out_) * 32; // osc1 modulates osc2
        
        // Generate oscillator outputs with cross-modulation
        int16_t osc1_out = osc1_.nextSample(fm1_hz_q16_16);
        int16_t osc2_out = osc2_.nextSample(fm2_hz_q16_16);
        
        // Store for next sample's cross-modulation
        prev_osc1_out_ = osc1_out;
        prev_osc2_out_ = osc2_out;
        
        // Ring modulation: multiply the two oscillator outputs
        // Both signals are in range -2048 to 2047, so product needs scaling
        int32_t ring_mod = (static_cast<int32_t>(osc1_out) * static_cast<int32_t>(osc2_out)) >> 11; // Divide by 2048

        // Soft saturation via waveshaper (convert 12-bit -> 16-bit, shape, back)
        int16_t ring16 = static_cast<int16_t>(ring_mod << 4); // -32768..32752 approx
        int16_t shaped16 = waveshaper_.process(ring16);
        int32_t shaped12 = static_cast<int32_t>(shaped16) >> 4;

        // Final clamp to 12-bit output range
        if (shaped12 > 2047) shaped12 = 2047;
        if (shaped12 < -2048) shaped12 = -2048;
        return shaped12;
    }

private:
    // Signal Flow (matching Teensy patch cords):
    // sine_fm1 ←→ sine_fm2 (cross-modulation via FM inputs)
    //    ↓         ↓
    //    ring modulator → output
    
    WaveformOscillator osc1_;    // sine_fm1 equivalent  
    WaveformOscillator osc2_;    // sine_fm2 equivalent
    Waveshaper waveshaper_;
    
    // Previous outputs for cross-modulation feedback
    int16_t prev_osc1_out_;
    int16_t prev_osc2_out_;

    // FM scaling and caps (Q16.16 Hz)
    int32_t fmScale1_q16_16_ = 0;
    int32_t fmScale2_q16_16_ = 0;
    int32_t maxFm1_q16_16_ = 0;
    int32_t maxFm2_q16_16_ = 0;

    // Simple LFSR for noise
    uint16_t noiseLFSR_ = 1;

    inline int16_t nextNoise12_() {
        // 16-bit Galois LFSR
        if (noiseLFSR_ == 0) noiseLFSR_ = 1;
        bool lsb = noiseLFSR_ & 1u;
        noiseLFSR_ >>= 1;
        if (lsb) noiseLFSR_ ^= 0xB400u; // taps
        int32_t v = static_cast<int32_t>(noiseLFSR_ & 0x0FFF); // 0..4095
        return static_cast<int16_t>(v - 2048); // -2048..2047
    }
};
