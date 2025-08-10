// Fixed-point Freeverb (Q15) for 16-bit I/O and block processing
//
// - Input/output samples are expected as int16 in the ComputerCard audio range
//   of -2048..2047. Internally we convert to Q15 by left-shifting 4 bits and
//   use 32-bit accumulators for MACs. Output converts back by right-shifting 4.
// - Coefficients and internal delay lines are Q15 (int16_t). This keeps memory
//   footprint small and fast on RP2040 while preserving reasonable precision.
// - Delay lengths are scaled for 48 kHz from Freeverb's 44.1 kHz tunings, with
//   a stereo spread applied to the right channel.
//
// Usage:
//   #include "dsp/freeverb16.hpp"
//   dsp::Freeverb16 rv;
//   rv.init(48000);
//   rv.setRoomSizeQ15(dsp::q15FromFloat(0.7f));
//   rv.setDampingQ15(dsp::q15FromFloat(0.5f));
//   rv.setWetQ15(dsp::q15FromFloat(0.33f));
//   rv.setDryQ15(dsp::q15FromFloat(1.0f));
//   rv.setWidthQ15(dsp::q15FromFloat(1.0f));
//   rv.setFreeze(false);
//   rv.processBlock(inL, inR, outL, outR, numSamples);

#pragma once

#include <cstdint>
#include <cstring>

namespace dsp {

// Helpers for Q15 fixed-point
inline int16_t q15FromFloat(float x) {
  if (x > 0.999969f) x = 0.999969f; // ~ (1 - 2^-15)
  if (x < -1.0f) x = -1.0f;
  int32_t v = (int32_t)(x * 32768.0f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

inline int16_t q15Saturate(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

inline int16_t q15Mul(int16_t a, int16_t b) {
  // (a * b) >> 15 with rounding
  int32_t prod = (int32_t)a * (int32_t)b; // Q30
  prod += (1 << 14);
  return (int16_t)(prod >> 15);
}

// Freeverb16: integer-optimized version of the classic Freeverb
class Freeverb16 {
public:
  Freeverb16() { resetState(); }

  void init(int sampleRate) {
    // Scale base tunings from 44.1kHz → target sampleRate
    // Classic Freeverb lengths (44.1kHz):
    // Combs:   1116,1188,1277,1356,1422,1491,1557,1617
    // Allpass:  556, 441, 341, 225
    const int baseComb[8]   = {1116,1188,1277,1356,1422,1491,1557,1617};
    const int baseAllp[4]   = { 556, 441, 341, 225};
    const int spread = 23; // stereo spread for right channel

    // Integer scaling with rounding
    auto scaleLen = [&](int base) -> int {
      // length_scaled = round(base * sampleRate / 44100)
      return (int)((int64_t)base * sampleRate + 22050) / 44100;
    };

    for (int i = 0; i < kNumCombs; ++i) {
      combLenL[i] = scaleLen(baseComb[i]);
      combLenR[i] = scaleLen(baseComb[i]) + spread;
      // Clamp to buffers
      if (combLenL[i] < 1) combLenL[i] = 1;
      if (combLenR[i] < 1) combLenR[i] = 1;
      if (combLenL[i] > kCombMax) combLenL[i] = kCombMax;
      if (combLenR[i] > kCombMax) combLenR[i] = kCombMax;
    }
    for (int i = 0; i < kNumAllp; ++i) {
      allpLenL[i] = scaleLen(baseAllp[i]);
      allpLenR[i] = scaleLen(baseAllp[i]) + spread;
      if (allpLenL[i] < 1) allpLenL[i] = 1;
      if (allpLenR[i] < 1) allpLenR[i] = 1;
      if (allpLenL[i] > kAllpMax) allpLenL[i] = kAllpMax;
      if (allpLenR[i] > kAllpMax) allpLenR[i] = kAllpMax;
    }

    resetState();

    // Defaults close to original Freeverb
    setRoomSizeQ15(q15FromFloat(0.5f));  // 0..1
    setDampingQ15(q15FromFloat(0.5f));   // 0..1
    setWetQ15(q15FromFloat(1.0f/3.0f));  // 0..1
    setDryQ15(q15FromFloat(1.0f));       // 0..1
    setWidthQ15(q15FromFloat(1.0f));     // 0..1
    setFreeze(false);
  }

  void resetState() {
    std::memset(combBufL, 0, sizeof(combBufL));
    std::memset(combBufR, 0, sizeof(combBufR));
    std::memset(allpBufL, 0, sizeof(allpBufL));
    std::memset(allpBufR, 0, sizeof(allpBufR));
    std::memset(combIdxL, 0, sizeof(combIdxL));
    std::memset(combIdxR, 0, sizeof(combIdxR));
    std::memset(allpIdxL, 0, sizeof(allpIdxL));
    std::memset(allpIdxR, 0, sizeof(allpIdxR));
    std::memset(filterStoreL, 0, sizeof(filterStoreL));
    std::memset(filterStoreR, 0, sizeof(filterStoreR));

    wetQ15 = q15FromFloat(1.0f/3.0f);
    dryQ15 = q15FromFloat(1.0f);
    widthQ15 = q15FromFloat(1.0f);
    updateWetMix();

    // Default gains/mapping
    inputGainQ15 = q15FromFloat(0.10f); // higher default for 16-bit device scale
    setRoomSizeQ15(q15FromFloat(0.5f));
    setDampingQ15(q15FromFloat(0.5f));
    freeze = false;
  }

  // Setters in Q15 domain (preferred to avoid float at runtime)
  void setRoomSizeQ15(int16_t valQ15) {
    // room = 0.28 + 0.7 * val
    // Q15: roomQ15 = 0.28 + 0.7*val
    const int16_t scaleRoomQ15 = q15FromFloat(0.7f);
    const int16_t offsetRoomQ15 = q15FromFloat(0.28f);
    int16_t scaled = q15Mul(valQ15, scaleRoomQ15);
    roomSizeQ15 = q15Saturate((int32_t)scaled + offsetRoomQ15);
    updateFeedback();
  }

  void setDampingQ15(int16_t valQ15) {
    // damp = 0..0.4 * val
    const int16_t scaleDampQ15 = q15FromFloat(0.4f);
    dampQ15 = q15Mul(valQ15, scaleDampQ15);
    updateDamp();
  }

  void setWetQ15(int16_t valQ15) {
    wetQ15 = valQ15;
    updateWetMix();
  }

  void setDryQ15(int16_t valQ15) {
    dryQ15 = valQ15;
  }

  void setWidthQ15(int16_t valQ15) {
    widthQ15 = valQ15;
    updateWetMix();
  }

  void setFreeze(bool enabled) {
    freeze = enabled;
    if (freeze) {
      // In freeze mode: infinite decay, no damping, no input
      feedbackQ15 = q15FromFloat(1.0f);
      damp1Q15 = 0;
      damp2Q15 = q15FromFloat(1.0f);
      inputGainQ15 = 0;
    } else {
      inputGainQ15 = q15FromFloat(0.10f);
      updateFeedback();
      updateDamp();
    }
  }

  void setInputGainQ15(int16_t gQ15) { inputGainQ15 = gQ15; }
  void setInputGain(float g) { setInputGainQ15(q15FromFloat(g)); }

  // Optional float setters (called rarely; avoid in audio-rate)
  void setRoomSize(float v) { setRoomSizeQ15(q15FromFloat(v)); }
  void setDamping(float v)  { setDampingQ15(q15FromFloat(v)); }
  void setWet(float v)      { setWetQ15(q15FromFloat(v)); }
  void setDry(float v)      { setDryQ15(q15FromFloat(v)); }
  void setWidth(float v)    { setWidthQ15(q15FromFloat(v)); }

  // Block processing, int16 I/O in ComputerCard audio units (-2048..2047)
  void processBlock(const int16_t* inL, const int16_t* inR,
                    int16_t* outL, int16_t* outR,
                    int numSamples) {
    for (int n = 0; n < numSamples; ++n) {
      // Convert to Q15 internal
      int16_t xL = (int16_t)(inL ? (inL[n] << 4) : 0);
      int16_t xR = (int16_t)(inR ? (inR[n] << 4) : xL);

      int16_t xinL = q15Mul(xL, inputGainQ15);
      int16_t xinR = q15Mul(xR, inputGainQ15);

      // Sum combs per channel
      int32_t accL = 0;
      int32_t accR = 0;
      for (int i = 0; i < kNumCombs; ++i) {
        accL += combProcess(xinL, combBufL[i], combIdxL[i], combLenL[i], filterStoreL[i]);
        accR += combProcess(xinR, combBufR[i], combIdxR[i], combLenR[i], filterStoreR[i]);
      }
      // Normalize comb sum down (8 combs). Original Freeverb averages by 8.
      int16_t yL = (int16_t)((accL + 4) >> 3); // divide by 8 with rounding
      int16_t yR = (int16_t)((accR + 4) >> 3);

      // Allpass cascade
      for (int i = 0; i < kNumAllp; ++i) yL = allpassProcess(yL, allpBufL[i], allpIdxL[i], allpLenL[i]);
      for (int i = 0; i < kNumAllp; ++i) yR = allpassProcess(yR, allpBufR[i], allpIdxR[i], allpLenR[i]);

      // Stereo mix with width
      // outL = yL*wet1 + yR*wet2 + xL*dry
      // outR = yR*wet1 + yL*wet2 + xR*dry
      int32_t mixL = (int32_t)q15Mul(yL, wet1Q15) + (int32_t)q15Mul(yR, wet2Q15) + (int32_t)q15Mul(xL, dryQ15);
      int32_t mixR = (int32_t)q15Mul(yR, wet1Q15) + (int32_t)q15Mul(yL, wet2Q15) + (int32_t)q15Mul(xR, dryQ15);

      // Saturate to Q15 then back to device range (-2048..2047)
      int16_t outQ15L = q15Saturate(mixL);
      int16_t outQ15R = q15Saturate(mixR);
      outL[n] = (int16_t)(outQ15L >> 4);
      outR[n] = (int16_t)(outQ15R >> 4);
    }
  }

private:
  static constexpr int kNumCombs = 8;
  static constexpr int kNumAllp  = 4;

  // Buffer maxima (after 48k scaling + spread). Choose conservative caps.
  static constexpr int kCombMax = 2048; // max per-comb delay samples
  static constexpr int kAllpMax = 640;  // max per-allpass delay samples

  // Delay lines per channel
  int16_t combBufL[kNumCombs][kCombMax];
  int16_t combBufR[kNumCombs][kCombMax];
  int16_t allpBufL[kNumAllp][kAllpMax];
  int16_t allpBufR[kNumAllp][kAllpMax];

  // Current write indices
  int combIdxL[kNumCombs];
  int combIdxR[kNumCombs];
  int allpIdxL[kNumAllp];
  int allpIdxR[kNumAllp];

  // Active lengths per filter
  int combLenL[kNumCombs];
  int combLenR[kNumCombs];
  int allpLenL[kNumAllp];
  int allpLenR[kNumAllp];

  // Comb filter internal one-pole states per channel
  int16_t filterStoreL[kNumCombs];
  int16_t filterStoreR[kNumCombs];

  // Parameters (Q15)
  int16_t inputGainQ15;   // ~0.015
  int16_t roomSizeQ15;    // mapped 0.28..0.98
  int16_t dampQ15;        // mapped 0..0.4
  int16_t feedbackQ15;    // = roomSize (or 1.0 in freeze)
  int16_t damp1Q15;       // = damp
  int16_t damp2Q15;       // = 1 - damp
  int16_t wetQ15;         // overall wet
  int16_t dryQ15;         // overall dry
  int16_t widthQ15;       // 0..1
  int16_t wet1Q15;        // derived from wet & width
  int16_t wet2Q15;        // derived from wet & width
  bool freeze;

  void updateFeedback() { feedbackQ15 = roomSizeQ15; }
  void updateDamp() {
    damp1Q15 = dampQ15;
    int32_t oneQ15 = 32767; // ~1.0 in Q15
    damp2Q15 = (int16_t)(oneQ15 - damp1Q15);
  }
  void updateWetMix() {
    // wet1 = wet * (width/2 + 0.5)
    // wet2 = wet * ((1 - width)/2)
    const int16_t halfQ15 = 16384; // 0.5
    int16_t w_over_2 = (int16_t)((((int32_t)widthQ15 + 1) >> 1)); // width/2
    int16_t one_minus_w_over_2 = (int16_t)(halfQ15 - w_over_2);
    int32_t sum = (int32_t)w_over_2 + halfQ15; // (w/2 + 0.5)
    if (sum > 32767) sum = 32767;
    wet1Q15 = q15Mul(wetQ15, (int16_t)sum);      // wet*(w/2 + 0.5)
    wet2Q15 = q15Mul(wetQ15, one_minus_w_over_2);// wet*((1-w)/2)
  }

  // Single comb step (Q15). Returns output sample in Q15.
  inline int16_t combProcess(int16_t inputQ15,
                             int16_t* buf, int& idx, int len,
                             int16_t& filtStateQ15) {
    // Read delayed value
    int16_t y = buf[idx];

    // Lowpass in feedback path: filt = y*(1 - damp) + filt*damp
    // filtState = y + (filtState - y)*damp  (algebraically equivalent)
    int16_t diff = (int16_t)(filtStateQ15 - y);
    int16_t delta = q15Mul(diff, damp1Q15);
    int16_t newFilt = (int16_t)(y + delta);
    filtStateQ15 = newFilt;

    // New buffer write: input + filt*feedback
    int16_t fb = q15Mul(newFilt, feedbackQ15);
    int16_t writeVal = (int16_t)q15Saturate((int32_t)inputQ15 + fb);
    buf[idx] = writeVal;

    // Advance circular index
    idx++;
    if (idx >= len) idx = 0;

    return y;
  }

  // Single allpass step (Q15). Returns output sample in Q15.
  inline int16_t allpassProcess(int16_t inputQ15,
                                int16_t* buf, int& idx, int len) {
    // Allpass feedback coefficient ~0.5 (Q15)
    static constexpr int16_t kApFeedbackQ15 = 16384; // 0.5

    int16_t bufout = buf[idx];
    int16_t acc = (int16_t)q15Saturate((int32_t)inputQ15 + q15Mul(bufout, kApFeedbackQ15));
    buf[idx] = acc;
    // Classic Freeverb form: output = bufout - feedback * acc
    int16_t output = (int16_t)q15Saturate((int32_t)bufout - q15Mul(acc, kApFeedbackQ15));

    idx++;
    if (idx >= len) idx = 0;
    return output;
  }
};

} // namespace dsp


