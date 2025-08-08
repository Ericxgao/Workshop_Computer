#include "ComputerCard.h"
#include "algos/ResoNoise.hpp"
#include "algos/RadioOhNo.hpp"
#include "algos/CrossModRingSquare.hpp"
#include "algos/CrossModRingSine.hpp"
#include "algos/ClusterSaw.hpp"
#include "algos/Atari.hpp"
#include "algos/Basurilla.hpp"
#include "algos/ArrayOnTheRocks.hpp"
#include "algos/PwCluster.hpp"

// Noise synthesis algorithms with CV control.
// - Main knob: algorithm selection (7 algorithms: ResoNoise, RadioOhNo, 
//   CrossModRingSquare, CrossModRingSine, ClusterSaw, ExistencesPain, Atari)
// - CV1 input: X parameter control for selected algorithm  
// - CV2 input: Y parameter control for selected algorithm

class NoiseDemo : public ComputerCard
{
public:
    NoiseDemo()
        : sampleHoldCounter(0), sampleHoldPeriod(8), heldSample(0), bitReductionShift(4) {}
    virtual void ProcessSample()
    {
        // Read controls
        int32_t main_knob_value_0_to_4095 = KnobVal(Knob::Main);
        
        // Read CV inputs (-2048 to 2047)
        int16_t cv1_raw = CVIn1();
        int16_t cv2_raw = CVIn2();
        
        // Sum CV with X/Y knobs (int range; individual algos may clamp)
        uint16_t kX = static_cast<uint16_t>(cv1_raw + KnobVal(Knob::X));
        uint16_t kY = static_cast<uint16_t>(cv2_raw + KnobVal(Knob::Y));

        // Also allow CV offset of Main knob, with wrap-around (0..4095)
        // Positive CV beyond max wraps back around
        int32_t kMain_wrapped = (main_knob_value_0_to_4095 + static_cast<int32_t>(AudioIn1())) % 4096;
        if (kMain_wrapped < 0) kMain_wrapped += 4096;

        // Switch algorithms via Main knob position (after CV wrap)  
        // Order: ResoNoise, RadioOhNo, CrossModRingSquare, CrossModRingSine, ClusterSaw, Basurilla, PwCluster, ArrayOnTheRocks, Atari
        int16_t s = 0;
        if (kMain_wrapped < 586)
        {
            s = reso.nextSample(kX, kY);
        }
        else if (kMain_wrapped < 1171)
        {
            s = radio.nextSample(kX, kY);
        }
        else if (kMain_wrapped < 1756)
        {
            int32_t sample = xmodring.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain_wrapped < 2341)
        {
            int32_t sample = xmodringsine.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain_wrapped < 2926)
        {
            int32_t sample = clustersaw.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain_wrapped < 3511)
        {
            int32_t sample = basurilla.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain_wrapped < 3600)
        {
            int32_t sample = pwcluster.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain_wrapped < 3800)
        {
            int32_t sample = arrayrocks.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else
        {
            int32_t sample = atari.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }

        int32_t vca_0_to_4095 = Connected(Input::Audio2) ? (AudioIn2() + 2048) : 4095;
        if (vca_0_to_4095 < 0) vca_0_to_4095 = 0;
        if (vca_0_to_4095 > 4095) vca_0_to_4095 = 4095;
        s = static_cast<int16_t>((static_cast<int32_t>(s) * vca_0_to_4095) >> 12);

        // Engage bit/sample rate reducer when the Z switch is Up
        if (SwitchVal() == Switch::Up)
        {
            // Sample rate reduction via sample-and-hold
            if (sampleHoldCounter == 0)
            {
                heldSample = s;
            }
            s = heldSample;
            sampleHoldCounter++;
            if (sampleHoldCounter >= sampleHoldPeriod) sampleHoldCounter = 0;

            // Bit-depth reduction: from 12 bits down to (12 - bitReductionShift) bits
            int32_t su = static_cast<int32_t>(s) + 2048; // map to 0..4095
            if (su < 0) su = 0; else if (su > 4095) su = 4095;
            su = (su >> bitReductionShift) << bitReductionShift;
            s = static_cast<int16_t>(su - 2048);
        }

        // On a rising edge at PulseIn1, sample-and-hold current audio sample 's' to CV Out 1
        if (PulseIn1RisingEdge())
        {
            CVOut1(s);
        }

        AudioOut1(s);
        AudioOut2(s);

        // Minimal visual feedback
        for (int i = 0; i < 6; ++i) LedOff(i);
        LedOn(0, true);
        // Match LEDs 1,3,5 to Main knob and CV inputs respectively
		LedBrightness(1, static_cast<uint16_t>(kMain_wrapped));  // Main knob after CV offset
		LedBrightness(3, kX);                   // CV1 input (X parameter)
		LedBrightness(5, kY);                   // CV2 input (Y parameter)
    }

private:
    ResoNoiseAlgo reso;
    RadioOhNoAlgo radio;
    CrossModRingSquare xmodring;
    CrossModRingSine xmodringsine;
    ClusterSaw clustersaw;
    Basurilla basurilla;
    PwCluster pwcluster;
    ArrayOnTheRocks arrayrocks;
    Atari atari;

    // Crusher state
    int sampleHoldCounter;
    int sampleHoldPeriod;      // e.g., 8 -> 48k/8 = 6kHz effective
    int16_t heldSample;
    uint8_t bitReductionShift; // 4 -> 12-4 = 8-bit effective
};

int main()
{
	set_sys_clock_khz(225000, true);
    NoiseDemo demo;
    // Enable jack-detection (normalisation probe) so Connected/Disconnected works
    demo.EnableNormalisationProbe();
    demo.Run();
}

