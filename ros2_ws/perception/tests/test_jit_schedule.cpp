/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_jit_schedule.cpp
 * Brief: Unit test for the JIT schedule calibration that freezes the submit delay.
 *
 * Description:
 * What it checks. After a warm-up that must not bias it, the calibration collects prepare and
 * device-pair times and freezes the delay D = pair median - prepare P99 (clipped at 0). Asking for
 * D before it is frozen, adding samples after it is frozen, and NaN samples all throw.
 */

#include "common/jit_schedule.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

int main() {
    perception::JitScheduleCalibration calibration;
    bool threw = false;
    try { (void)calibration.delay_ms(); } catch (const std::logic_error&) { threw = true; }
    assert(threw);
    for (std::size_t i = 0; i < calibration.warmup_count; ++i) {
        calibration.add(1000.0, 2000.0);  // Warmup must not bias D.
    }
    for (std::size_t i = 0; i < calibration.sample_count; ++i) {
        calibration.add(12.0, 50.0);
        assert(calibration.frozen() == (i + 1U == calibration.sample_count));
    }
    assert(calibration.pair_median_ms() == 50.0);
    assert(calibration.prepare_p99_ms() == 12.0);
    assert(calibration.delay_ms() == 36.0);
    threw = false;
    try { calibration.add(12.0, 50.0); } catch (const std::logic_error&) { threw = true; }
    assert(threw);
    perception::JitScheduleCalibration clipped;
    for (std::size_t i = 0; i < clipped.warmup_count + clipped.sample_count; ++i) {
        clipped.add(55.0, 50.0);
    }
    assert(clipped.delay_ms() == 0.0);
    perception::JitScheduleCalibration invalid;
    threw = false;
    try { invalid.add(std::numeric_limits<double>::quiet_NaN(), 50.0); }
    catch (const std::logic_error&) { threw = true; }
    assert(threw);
}
