#include "ComputerCard.h"
#include "pico/multicore.h"
#include <cstdint>

/*

Core-synchronised pipelined DSP example
---------------------------------------

Goal: demonstrate a one-sample-latency pipeline using the RP2040 multicore FIFO
so that per-sample compute can be split across two cores, effectively doubling
available DSP time (two cores each have ~20us at 48kHz → ~40us aggregate).

Scheme:
- Core 0 (audio thread):
  - On each 48kHz ProcessSample, it sends current input sample + parameter to core 1
    via the multicore FIFO, and outputs the result received from the previous sample
    (if available), giving a fixed one-sample latency.
  - It never blocks; it checks FIFO flags to avoid stalling the 48kHz callback.

- Core 1 (DSP worker):
  - Blocks waiting for work, performs the heavy per-sample processing, then returns
    the result to core 0 via the FIFO.

User interface:
- Audio In 1 → processed to Audio Out 1 and 2.
- Knob Main controls filter coefficient (smoothing amount).
- LEDs 0 and 1 briefly indicate underrun/overrun events.

*/

class CoreSync : public ComputerCard {
  // Previous output to use if no new result available this sample
  volatile int16_t previousOutput;
  // Simple event counters
  volatile uint32_t fifoUnderrunCount;
  volatile uint32_t fifoOverrunCount;

public:
  CoreSync() {
    previousOutput = 0;
    fifoUnderrunCount = 0;
    fifoOverrunCount = 0;

    // Ensure FIFOs are empty/clean before launching
    multicore_fifo_drain();
    // Launch worker on core 1
    multicore_launch_core1(core1Entry);
  }

  // Static trampoline for core 1
  static void core1Entry() { ((CoreSync*)ThisPtr())->WorkerCore(); }

  // Pack sample (int16) and parameter (uint16) into one word
  static inline uint32_t packSample(int16_t sample, uint16_t param) {
    return (uint32_t(param) << 16) | (uint16_t(sample));
  }

  // Unpack helpers
  static inline int16_t unpackSample(uint32_t word) { return int16_t(word & 0xFFFFu); }
  static inline uint16_t unpackParam(uint32_t word) { return uint16_t(word >> 16); }

  // Heavy-ish per-sample processing (core 1)
  // Integer multi-stage one-pole smoothing to simulate meaningful DSP cost.
  static inline int16_t processSample(int16_t x, uint16_t param, int32_t& z1, int32_t& z2) {
    // Map 0..4095 → alpha in Q12 (0..4095). Use a minimum to avoid denormal-like behavior.
    int32_t alpha = (int32_t)param; // Q12
    if (alpha < 64) alpha = 64;     // minimum smoothing

    // Convert input to Q12 domain
    int32_t x_q12 = ((int32_t)x) << 12; // Q12

    // Stage 1: z1 += alpha*(x - z1) in Q12
    int32_t delta1 = ((x_q12 - z1) * alpha) >> 12; // Q12
    z1 += delta1;                                  // Q12

    // Stage 2: z2 += alpha*(z1 - z2) in Q12
    int32_t delta2 = ((z1 - z2) * alpha) >> 12;    // Q12
    z2 += delta2;                                  // Q12

    // Back to 12-bit signed range with clipping to -2048..2047
    int32_t y = z2 >> 12; // back to integer sample domain
    if (y < -2048) y = -2048;
    if (y >  2047) y =  2047;
    return (int16_t)y;
  }

  // Core 1 worker loop: block for input, process, push result
  void WorkerCore() {
    // Persistent filter states in Q12
    int32_t z1 = 0;
    int32_t z2 = 0;

    while (true) {
      uint32_t inWord = multicore_fifo_pop_blocking();
      int16_t x = unpackSample(inWord);
      uint16_t param = unpackParam(inWord);

      int16_t y = processSample(x, param, z1, z2);

      // Return result to core 0 (blocks until space available)
      multicore_fifo_push_blocking((uint32_t)(uint16_t)y);
    }
  }

  // 48kHz callback on core 0
  void __not_in_flash_func(ProcessSample)() override {
    // Set LED 0 to indicate core 0 is running
    LedOn(2, true);

    // 1) Try to receive previous result without blocking
    // If not ready, reuse previousOutput and count underrun
    int16_t outputNow = previousOutput;
    if (multicore_fifo_rvalid()) {
      // Safe to pop; will be immediate
      uint32_t word = multicore_fifo_pop_blocking();
      outputNow = (int16_t)(word & 0xFFFFu);
      previousOutput = outputNow;
    } else {
      // No result ready this sample
      fifoUnderrunCount++;
      // Briefly indicate underrun on LED 0
      LedOn(0, true);
    }

    // 2) Prepare current input and parameter and try to push without blocking
    int16_t x = AudioIn1();
    uint16_t param = (uint16_t)KnobVal(Knob::Main); // 0..4095
    uint32_t sendWord = packSample(x, param);

    if (multicore_fifo_wready()) {
      multicore_fifo_push_blocking(sendWord);
    } else {
      // FIFO full — worker is falling behind; skip to avoid blocking audio
      fifoOverrunCount++;
      // Briefly indicate overrun on LED 1
      LedOn(1, true);
    }

    // 3) Output the result from previous sample
    AudioOut1(outputNow);
    AudioOut2(outputNow);

    // Decay the debug LEDs quickly
    // (Turn off if counts have not increased recently)
    static uint32_t lastUnderrun = 0;
    static uint32_t lastOverrun = 0;
    if (lastUnderrun == fifoUnderrunCount) LedOff(0); else lastUnderrun = fifoUnderrunCount;
    if (lastOverrun == fifoOverrunCount) LedOff(1); else lastOverrun = fifoOverrunCount;
  }
};

int main() {
  CoreSync cs;
  cs.Run();
}


