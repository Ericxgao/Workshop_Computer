#include "ComputerCard.h"
#include "algos/ResoNoise.hpp"
#include "algos/RadioOhNo.hpp"
#include "algos/CrossModRingSquare.hpp"
#include "algos/CrossModRingSine.hpp"
#include "algos/ClusterSaw.hpp"

// Noise synthesis algorithms with CV control.
// - Main knob: algorithm selection (6 algorithms: ResoNoise, RadioOhNo, 
//   CrossModRingSquare, CrossModRingSine, ClusterSaw, ExistencesPain)
// - CV1 input: X parameter control for selected algorithm  
// - CV2 input: Y parameter control for selected algorithm

class NoiseDemo : public ComputerCard
{
public:
    virtual void ProcessSample()
    {
        // Read controls
        uint16_t kMain = KnobVal(Knob::Main); // 0..4095 - algorithm selection
        
        // Read CV inputs (-2048 to 2047) and convert to knob range (0-4095)
        int16_t cv1_raw = CVIn1();
        int16_t cv2_raw = CVIn2();
        
        // Convert CV inputs from (-2048 to 2047) to (0 to 4095)
        uint16_t kX = (cv1_raw + KnobVal(Knob::X)); // CV1 controls X parameter
        uint16_t kY = (cv2_raw + KnobVal(Knob::Y)); // CV2 controls Y parameter

        // Switch algorithms via Main knob position  
        // 0..682: ResoNoise; 683..1365: RadioOhNo; 1366..2048: CrossModRingSquare; 
        // 2049..2731: CrossModRingSine; 2732..3413: ClusterSaw; 3414..4095: ExistencesPain
        int16_t s = 0;
        if (kMain < 683)
        {
            s = reso.nextSample(kX, kY);
        }
        else if (kMain < 1366)
        {
            s = radio.nextSample(kX, kY);
        }
        else if (kMain < 2049)
        {
            int32_t sample = xmodring.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain < 2732)
        {
            int32_t sample = xmodringsine.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }
        else if (kMain < 3414)
        {
            int32_t sample = clustersaw.process(kX, kY);
            s = static_cast<int16_t>(sample);
        }

        AudioOut1(s);
        AudioOut2(s);

        // Minimal visual feedback
        for (int i = 0; i < 6; ++i) LedOff(i);
        LedOn(0, true);
        // Match LEDs 1,3,5 to Main knob and CV inputs respectively
		LedBrightness(1, KnobVal(Knob::Main));  // Main knob (algorithm selection)
		LedBrightness(3, kX);                   // CV1 input (X parameter)
		LedBrightness(5, kY);                   // CV2 input (Y parameter)
    }

private:
    ResoNoiseAlgo reso;
    RadioOhNoAlgo radio;
    CrossModRingSquare xmodring;
    CrossModRingSine xmodringsine;
    ClusterSaw clustersaw;
};

int main()
{
	set_sys_clock_khz(220000, true);
    NoiseDemo demo;
    demo.Run();
}

