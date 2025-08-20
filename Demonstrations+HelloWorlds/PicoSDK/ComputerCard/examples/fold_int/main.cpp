#include "ComputerCard.h"
#include <cstdint>
#include "dsp/wavefolder_int.h"

// Integer wavefolder inspired by Warps' ALGORITHM_FOLD, rewritten for RP2040
// - No floating point in the audio callback
// - Uses Q1.15 fixed-point internally
// Two-parameter behavior (warps/dsp/modulator.cc):
//   sum = x1 + x2 + 0.25 * x1 * x2;
//   sum *= (0.02 + p1);
//   sum += p2;
//   y = LUT_bipolar_fold(sum)  [here approximated with integer reflection folding]

namespace {

using namespace cc_dsp;

} // namespace

class FoldInt : public ComputerCard {
public:
    virtual void ProcessSample() {
        // Read audio inputs (-2048..2047)
        int16_t a1_12 = AudioIn1();
        int16_t a2_12 = AudioIn2();

        // Convert to Q15
        int32_t x1_q15 = audio12_to_q15(a1_12);
        int32_t x2_q15 = audio12_to_q15(a2_12);

        // Map p1 <- Knob X (0..4095) to Q15 (0..~1.0)
        int32_t p1_q15 = knob_to_q15(KnobVal(Knob::X));
        // Map p2 <- Knob Y to bipolar Q15 using center at 2048
        int32_t p2_12 = static_cast<int32_t>(KnobVal(Knob::Y)) - 2048;
        int32_t p2_q15 = p2_12 << 4;

        // Select algorithm with Main knob using evenly-split ranges
        // Number of algos comes from Algorithm::Count
        uint32_t main_knob = KnobVal(Knob::Main); // 0..4095
        const int num_algos = static_cast<int>(Algorithm::Count);
        const uint32_t segment = 4096u / static_cast<uint32_t>(num_algos ? num_algos : 1);
        int algo_index = segment ? static_cast<int>(main_knob / segment) : 0;
        if (algo_index >= num_algos) algo_index = num_algos - 1;
        Algorithm algo = static_cast<Algorithm>(algo_index);

        // Apply selected algorithm
        int32_t y_q15 = process_algorithm_q15(algo, x1_q15, x2_q15, p1_q15, p2_q15);

        // Convert back to 12-bit and output on both channels
        int16_t y12 = q15_to_audio12(y_q15);
        AudioOut1(y12);
        AudioOut2(y12);

        // Visual: show p1 (X) on LED 1, |p2| (Y centered) on LED 3
        LedBrightness(1, static_cast<uint16_t>(KnobVal(Knob::X)));
        uint16_t p2_abs = static_cast<uint16_t>(p2_12 >= 0 ? p2_12 : -p2_12);
        LedBrightness(3, p2_abs);
    }
};

int main() {
    set_sys_clock_khz(225000, true);
    FoldInt app;
    app.Run();
}


