/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_cadence.cpp
 * Brief: R4 exact boundary, silence, exclusion and finite-window negative tests of the Objects cadence.
 *
 * Description:
 * What it checks. The cadence ledger decides infer_gap and infer_rate_low from publication
 * intervals (19 section 6.3). Cases cover exactly 10 s of unique good intervals, proportion and
 * absolute-threshold reasons kept apart (one allowed 80 ms interval keeps gap active but not
 * rate), atomic admission at the 1 percent boundary, a duplicate source that cannot hide an open
 * gap, paused and excluded periods, nearest-rank percentiles, and that declaring after a violation
 * cannot erase its history.
 */

#include "output/rns_cadence.hpp"
#include <cassert>
#include <iostream>
#include <stdexcept>
using namespace perception::rns;
namespace {
void reason_recovery() {
    ObjectsCadence c(0, 1024);
    c.observe(30000, 30000);
    c.observe(30120, 30120);
    auto s = c.snapshot(30120);
    assert(s.observed_ms == 30120 && s.infer_gap && s.infer_rate_low);
    assert(s.infer_gap_since_ms == 30120 && s.infer_rate_low_since_ms == 30120);
    for (int i = 1; i < 250; ++i) {
        const auto now = 30120 + i * 40;
        c.observe(now, now); s = c.snapshot(now);
        assert(s.infer_gap && s.infer_rate_low);
    }
    // Exactly 10 seconds of unique good intervals, NOT 20 seconds.
    c.observe(40120, 40120); s = c.snapshot(40120);
    assert(s.full_window && s.pass && !s.infer_gap && !s.infer_rate_low);
    assert(!s.infer_gap_since_ms && !s.infer_rate_low_since_ms);
    assert(s.historical_max_gap_ms == 120 && s.max_gap_ms == 40);

    // Proportion and absolute threshold are different reasons. One allowed
    // 80ms interval in a full window keeps gap active, but must not keep rate.
    ObjectsCadence rare(0, 1024);
    rare.observe(30000, 30000); rare.observe(30120, 30120);
    std::int64_t t = 30120;
    for (int i = 1; i <= 249; ++i) {
        t += (i == 100 ? 80 : 40); rare.observe(t, t);
    }
    assert(t == 40120);
    s = rare.snapshot(t);
    assert(s.full_window && s.tier1 && s.tier2 && s.over50 == 1);
    assert(s.infer_gap && !s.infer_rate_low && s.historical_max_gap_ms == 120);

    // Admission must be atomic wrt closed-gap ratio. Before the new normal
    // gap is appended, pruning leaves 2/199; the actual observed window is
    // always 2/200 = 1%. Never latch from that implementation-only midpoint.
    ObjectsCadence atomic(0, 1024);
    t = 30000; atomic.observe(t, t);
    for (int i = 1; i <= 500; ++i) {
        t += (i == 400 || i == 450) ? 20 : ((i == 401 || i == 451) ? 80 : 50);
        atomic.observe(t, t); s = atomic.snapshot(t);
        assert(s.tier2 && !s.infer_rate_low);
    }
    assert(s.closed_gaps == 200 && s.over50 == 2 && s.infer_gap);

    // Duplicate source cannot hide an open gap or supply recovery evidence.
    ObjectsCadence dup(0, 1024);
    dup.observe(30000, 30000);
    assert(dup.observe(30000, 30120) == CadenceEvent::Duplicate);
    s = dup.snapshot(30120);
    assert(s.infer_gap && s.infer_rate_low && s.open_gap_ms == 120 && s.unique_results == 1);
    for (t = 30160; t <= 40120; t += 40) dup.observe(30000, t);
    s = dup.snapshot(40120);
    assert(s.infer_gap && s.infer_rate_low && s.open_gap_ms == 10120);

    ObjectsCadence paused(0, 1024);
    paused.observe(30000, 30000); paused.observe(30120, 30120);
    paused.declare_exclusion(Exclusion::Reconnecting, 30120);
    s = paused.snapshot(50120);
    assert(s.excluded && s.infer_gap && s.infer_rate_low);
    paused.declare_exclusion(Exclusion::None, 50120);
    s = paused.snapshot(60120);
    assert(s.awaiting_new_segment && s.infer_gap && s.infer_rate_low);
    paused.observe(60121, 60121);
    for (int i = 1; i < 250; ++i) {
        t = 60121 + i * 40; paused.observe(t, t);
    }
    s = paused.snapshot(70120);
    assert(!s.full_window && s.infer_gap && s.infer_rate_low);
    paused.observe(70121, 70121); s = paused.snapshot(70121);
    assert(s.full_window && !s.infer_gap && !s.infer_rate_low);

    ObjectsCadence threshold(0, 1024);
    threshold.observe(30000, 30000);
    s = threshold.snapshot(30050); assert(!s.infer_gap && !s.infer_rate_low);
    s = threshold.snapshot(30051); assert(s.infer_gap && !s.infer_rate_low);
    s = threshold.snapshot(30100); assert(s.infer_gap && !s.infer_rate_low);
    s = threshold.snapshot(30101); assert(s.infer_gap && s.infer_rate_low);
    assert(!threshold.snapshot(30100).valid);
    threshold.observe(30140, 30140);
    for (int i = 1; i <= 250; ++i) threshold.observe(30140 + i * 40, 30140 + i * 40);
    s = threshold.snapshot(40140);
    assert(!s.valid && s.infer_gap && s.infer_rate_low); // invalid never clears
    std::cout << "RNS reason A19-RATE-1 120ms/10s, rare allowed gap, duplicates, exclusion, boundary and invalid hold PASS\n";
}
}
int main() {
    reason_recovery();
    ObjectsCadence c(0, 1024);
    assert(!c.snapshot(29999).pass && c.snapshot(29999).excluded);
    std::int64_t t = 30000;
    assert(c.observe(t - 400, t) == CadenceEvent::New);
    // An age of 400ms has no effect on a 40ms publication cadence.
    for (int i = 0; i < 250; ++i) { t += 40; assert(c.observe(t - 400, t) == CadenceEvent::New); }
    auto s = c.snapshot(t);
    assert(s.full_window && s.pass && s.max_gap_ms == 40 && s.over50 == 0);
    assert(s.closed_gaps == 250 && *s.p99_with_open_ms == 40);
    assert(c.observe(t - 400, t + 30) == CadenceEvent::Duplicate);
    assert(c.observe(t - 401, t + 40) == CadenceEvent::OutOfOrder);
    s = c.snapshot(t + 101);
    assert(!s.pass && s.open_gap_ms == 101 && s.duplicates == 1 && s.out_of_order == 1);
    assert(s.historical_max_gap_ms == 101);
    assert(!c.snapshot(t - 1).valid);  // No implicit epoch reset washing failure away.

    ObjectsCadence ratio(0, 1024); t = 30000;
    ratio.observe(t, t);
    for (int i = 0; i < 99; ++i) { t += 40; ratio.observe(t, t); }
    t += 80; ratio.observe(t, t);
    s = ratio.snapshot(t);
    assert(s.closed_gaps == 100 && s.over50 == 1 && s.over50_fraction == .01 && s.tier2);
    assert(s.p99_with_open_ms == 40 && !s.full_window && !s.pass);
    // 101 samples include open=0; nearest-rank is 100th, still 40.
    ObjectsCadence ratio_fail(0, 1024); t = 30000; ratio_fail.observe(t, t);
    for (int i = 0; i < 98; ++i) { t += 40; ratio_fail.observe(t, t); }
    t += 80; ratio_fail.observe(t, t);
    s = ratio_fail.snapshot(t); assert(s.closed_gaps == 99 && !s.tier2);

    ObjectsCadence boundary(0, 1024);
    boundary.observe(30000, 30000); boundary.observe(30100, 30100);
    assert(boundary.snapshot(30100).tier1);
    boundary.observe(30201, 30201); assert(!boundary.snapshot(30201).tier1);
    s = boundary.snapshot(40201);
    assert(s.closed_gaps == 0 && s.open_gap_ms == 10000 && !s.tier1 && !s.pass);

    ObjectsCadence exclusions(0, 128);
    exclusions.observe(30000, 30000);
    assert(exclusions.declare_exclusion(Exclusion::Reconnecting, 30040));
    assert(exclusions.observe(32000, 32000) == CadenceEvent::Excluded);
    assert(exclusions.snapshot(35000).excluded);
    assert(exclusions.declare_exclusion(Exclusion::None, 35000));
    assert(exclusions.observe(32000, 35001) == CadenceEvent::Duplicate);
    s = exclusions.snapshot(36000);
    assert(s.awaiting_new_segment && !s.pass && s.declared_excluded_ms == 4960);
    exclusions.observe(36010, 36010); s = exclusions.snapshot(36010);
    assert(!s.awaiting_new_segment && s.closed_gaps == 0 && s.historical_max_gap_ms == 40);
    // Declaring after a violation cannot erase its history.
    exclusions.declare_exclusion(Exclusion::Reconfiguring, 36200);
    assert(exclusions.snapshot(36200).historical_max_gap_ms == 190);

    ObjectsCadence empty(0, 4);
    s = empty.snapshot(30101); assert(!s.pass && s.open_gap_ms == 101);
    ObjectsCadence capacity(0, 2);
    capacity.observe(30000,30000); capacity.observe(30001,30001); capacity.observe(30002,30002);
    assert(capacity.observe(30003,30003) == CadenceEvent::Invalid && !capacity.snapshot(30003).valid);
    bool rejected = false;
    try { ObjectsCadence invalid(0, static_cast<std::size_t>(-1)); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    std::cout << "RNS cadence boundary, no-refresh, open-gap, exclusion checks PASS\n";
}
