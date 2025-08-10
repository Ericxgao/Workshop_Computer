// Block-based passthrough example
//
// Collects audio input into blocks and processes the block in one go,
// introducing a fixed one-block latency. Within ProcessBlock you can safely
// iterate over all samples in the block.

#include "ComputerCard.h"
#include <cstdint>
#include "dsp/freeverb16.hpp"

class BlockPassthrough : public ComputerCard {
  static constexpr int kBlockSize = 32; // Change to taste (e.g., 16/32/64)

  int16_t inputBufferL[kBlockSize];
  int16_t inputBufferR[kBlockSize];
  int16_t outputBufferL[kBlockSize];
  int16_t outputBufferR[kBlockSize];

  int currentIndex;
  bool haveOutputBlock;
  
  dsp::Freeverb16 reverb;

public:
  BlockPassthrough() : currentIndex(0), haveOutputBlock(false) {
    reverb.init(48000);
    reverb.setRoomSizeQ15(dsp::q15FromFloat(0.5f));
    reverb.setDampingQ15(dsp::q15FromFloat(0.5f));
    reverb.setWetQ15(dsp::q15FromFloat(1.0f/3.0f));
    reverb.setDryQ15(dsp::q15FromFloat(1.0f));
    reverb.setWidthQ15(dsp::q15FromFloat(1.0f));
  }

  // Called once per 48kHz sample on core 0
  void __not_in_flash_func(ProcessSample)() override {
    // 1) Capture current input sample into the current block slot
    inputBufferL[currentIndex] = AudioIn1();
    inputBufferR[currentIndex] = AudioIn2();

    // 2) While waiting for the first processed block, pass input through
    //    sample-by-sample. After that, output from the precomputed block.
    int16_t yL = haveOutputBlock ? outputBufferL[currentIndex]
                                 : inputBufferL[currentIndex];
    int16_t yR = haveOutputBlock ? outputBufferR[currentIndex]
                                 : inputBufferR[currentIndex];

    // Audio out
    AudioOut1(yL);
    AudioOut2(yR);

    // Maintain original passthrough behavior for CV and Pulse each sample
    CVOut1(CVIn1());
    CVOut2(CVIn2());
    PulseOut1(PulseIn1());
    PulseOut2(PulseIn2());

    // Simple UI like the original passthrough
    int s = SwitchVal();
    LedOn(4, s == Switch::Down);
    LedOn(2, s == Switch::Middle);
    LedOn(0, s == Switch::Up);
    LedBrightness(1, KnobVal(Knob::Main));
    LedBrightness(3, KnobVal(Knob::X));
    LedBrightness(5, KnobVal(Knob::Y));

    // 3) Advance write/playback index. If we've reached the end of a block,
    //    process the just-filled input block to produce the next output block.
    currentIndex++;
    if (currentIndex >= kBlockSize) {
      ProcessBlock(inputBufferL, inputBufferR, outputBufferL, outputBufferR, kBlockSize);
      currentIndex = 0;
      haveOutputBlock = true; // From now on, we play from the processed buffer
    }
  }

private:
  // Block processing: iterate safely through all samples in the block
  // Reverb using fixed-point Freeverb16.
  void __not_in_flash_func(ProcessBlock)(const int16_t* inL,
                                         const int16_t* inR,
                                         int16_t* outL,
                                         int16_t* outR,
                                         int numSamples) {
    // Map knobs (0..4095) to Q15 (0..1). Keep wet scaled to 0..0.5 for headroom.
    auto knobToQ15 = [](int32_t k) -> int16_t {
      if (k < 0) k = 0; if (k > 4095) k = 4095;
      int32_t v = (k * 32767 + 2047) / 4095; // rounded scale
      return (int16_t)v;
    };

    int16_t roomQ15 = knobToQ15(KnobVal(Knob::Main));
    int16_t dampQ15 = knobToQ15(KnobVal(Knob::X));
    int16_t wetQ15  = (int16_t)((KnobVal(Knob::Y) * 16384 + 2047) / 4095); // 0..0.5

    reverb.setInputGainQ15(dsp::q15FromFloat(0.5f));
    reverb.setRoomSizeQ15(roomQ15);
    reverb.setDampingQ15(dampQ15);
    reverb.setWetQ15(wetQ15);
    reverb.setDryQ15(0);   // full dry
    reverb.setWidthQ15(32767); // full width
    reverb.setFreeze(false);

    reverb.processBlock(inL, inR, outL, outR, numSamples);
  }
};

int main() {
  set_sys_clock_khz(225000, true);

  BlockPassthrough app;
  app.Run();
}


