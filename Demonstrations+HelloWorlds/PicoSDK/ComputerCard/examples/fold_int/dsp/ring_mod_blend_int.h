#pragma once
#include <cstdint>
#include "fixedpoint_int.h"
#include "ring_mod_analog_int.h"
#include "ring_mod_digital_int.h"

namespace cc_dsp {

// Blend between analog and digital ring modulation like Warps' ALGORITHM_RING_MODULATION
// y_analog = analog(x1, x2, p2)
// y_digital = digital(x1, x2, p2)
// y = y_digital + (y_analog - y_digital) * p1
inline int32_t process_ring_blend_q15(int32_t x1_q15,
                                      int32_t x2_q15,
                                      int32_t p1_q15,
                                      int32_t p2_q15) {
    int32_t ya = process_analog_ring_q15(x1_q15, x2_q15, p2_q15);
    int32_t yd = process_digital_ring_q15(x1_q15, x2_q15, p2_q15);
    int32_t diff = ya - yd; // Q15
    int32_t mix = yd + mul_q15(diff, p1_q15);
    return mix;
}

} // namespace cc_dsp


