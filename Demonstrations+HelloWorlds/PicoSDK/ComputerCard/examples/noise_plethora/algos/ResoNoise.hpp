// ResoNoise algorithm scaffold.
// Composes reusable DSP primitives (currently just WhiteNoise).

#pragma once

#include <cstdint>
#include "dsp/WhiteNoise.hpp"
#include "dsp/WaveformOsc.hpp"
#include "dsp/StateVariableFilterInt.hpp"
#include "dsp/Wavefolder.hpp"

class ResoNoiseAlgo {
public:
    ResoNoiseAlgo()
    {
        // Default LFO and Filter settings
        lfo.setSampleRate(48000.0f);
        lfo.setShape(WaveformOscillator::Shape::Sine);
        lfo.setFrequencyHz(0.5f);        // default, updated by X
        lfo.setAmplitudeQ12(4000);       // conservative to avoid over-driving cutoff mapping
        fmSine.setSampleRate(48000.0f);
        fmSine.setShape(WaveformOscillator::Shape::Sine);
        fmSine.setAmplitudeQ12(4095);
        modSquare.setSampleRate(48000.0f);
        modSquare.setShape(WaveformOscillator::Shape::Square);
        modSquare.setAmplitudeQ12(4095);

        svfi.setSampleRate(48000.0f);
        svfi.setMode(StateVariableFilterInt::Mode::Lowpass);
        svfi.setCutoffHz(8000.0f);           // match sine_fm1.frequency(1100) initial setting
        svfi.setQ(9.0f);                     // match P_resonoise filter1.resonance(3)
    }

    void reset(uint32_t seed) { noise.init(seed != 0 ? seed : 0x1u); }

    // Generate one sample. Controls map to P_resonoise style:
    // - x_q12: 0..4095 => primary "pitch" control (drives LFO and sine FM rate) 
    // - y_q12: 0..4095 => bias for wavefolder (mapped to DC amplitude) and resonance
    // NOTE: Noise Plethora inverts knob readings! We must match this behavior.
    inline int16_t nextSample(uint16_t x_q12, uint16_t y_q12)
    {
        // Rarely reseed noise to vary texture with X
        seedAccumulator += static_cast<uint32_t>(x_q12);
        if ((reseedCounter++ & 0x0FFFu) == 0)
        {
            noise.init(baseSeed ^ seedAccumulator);
        }
        // Synthesize base noise voice (full amplitude)
        int16_t n = noise.nextSample(4095);

        // Control-rate updates (reduce float work): update every 128 samples
        if ((paramUpdateCounter++ & 0x7F) == 0)
        {
            // Map X to pitch in [0..1]^2, then to Hz ranges
            // CRITICAL: Noise Plethora inverts knob readings: knob_1 = 1.0 - adc/1023.0
            float x01_raw = static_cast<float>(x_q12) * (1.0f / 4095.0f); // 0..1
            float x01 = 1.0f - x01_raw; // INVERT to match Noise Plethora behavior!
            float pitch = x01 * x01;
            float modHz = 20.0f + pitch * 7777.0f;
            float sineHz = 20.0f + pitch * 10000.0f;
            lfo.setFrequencyHz(modHz);
            fmSine.setFrequencyHz(sineHz);
            modSquare.setFrequencyHz(modHz);

            // Precompute FM depth in Q16.16: 25% of sine Hz
            sineHz_q16_16 = static_cast<int32_t>(sineHz * 65536.0f + (sineHz >= 0 ? 0.5f : -0.5f));
            fmDepth_q16_16 = sineHz_q16_16; // 25%
        }

        // FM the sine with the square modulator
        int16_t m = modSquare.nextSample();             // -2048..2047
        // fm_hz_q16_16 = (m/2048) * fmDepth_q16_16
        int64_t fm_tmp = static_cast<int64_t>(m) * static_cast<int64_t>(fmDepth_q16_16);
        int32_t fm_q16_16 = static_cast<int32_t>(fm_tmp >> 11); // divide by 2048 (>>11)
        int16_t sine = fmSine.nextSample(fm_q16_16);

        // Build wavefolder input: sine FM plus DC from Y (match P_resonoise: dc amplitude = 0.03 + 0.2*y)
        // Map Y (0..4095) to DC amplitude range (0.03..0.23) and convert to int16_t
        // CRITICAL: Noise Plethora inverts knob readings: knob_2 = 1.0 - adc/1023.0  
        float y_norm_raw = static_cast<float>(y_q12) / 4095.0f; // 0..1
        float y_norm = 1.0f - y_norm_raw; // INVERT to match Noise Plethora behavior!
        float dc_amplitude = y_norm * 0.2f + 0.03f; // 0.03..0.23
        int16_t dc_value = static_cast<int16_t>(dc_amplitude * 32767.0f);
        int16_t folded = folder.process(sine, dc_value);

        // Route through filter exactly like Teensy P_resonoise:
        // patchCord2(noise1, 0, filter1, 0)        -> noise to filter input 0
        // patchCord5(wavefolder1, 0, filter1, 1)   -> wavefolder to filter input 1
        // filter1 output is the final result
        // NOTE: P_resonoise uses STATIC filter frequency - no dynamic frequency modulation!
        return svfi.process(n, folded) * 1.8f;
    }

    void setBaseSeed(uint32_t seed) { baseSeed = seed != 0 ? seed : 0x1u; }

private:
    WhiteNoise noise;
    WaveformOscillator lfo; // reserved for future modulation
    WaveformOscillator fmSine;
    WaveformOscillator modSquare;
    StateVariableFilterInt svfi;
    Wavefolder folder;
    uint32_t paramUpdateCounter = 0;
    // Control-rate cached params (Q16.16 where noted)
    int32_t sineHz_q16_16 = static_cast<int32_t>(20.0f * 65536.0f);
    int32_t fmDepth_q16_16 = sineHz_q16_16 >> 2;
    uint32_t baseSeed = 0xA5A5F00Du;
    uint32_t seedAccumulator = 0;
    uint32_t reseedCounter = 0;
};


