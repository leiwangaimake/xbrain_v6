/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_formal_window.cpp
 * Brief: Unit tests for the offline formal observation window over Objects results.
 *
 * Description:
 * What it checks. The window selects a fixed cohort of successfully published results for an
 * offline cadence observation and never controls scheduling. Cases cover start, phase and a queued
 * result, no reselection within the first 900 results, exact binding with sticky failure,
 * malformed and uncommitted selections, and that a duration cut never replaces or weakens an
 * identity (a native put crossing the cut after selection stays as observed).
 */

#include "rns_formal_window.hpp"
#include <iostream>
#include <limits>
#include <utility>

namespace {
using rns_smoke_support::FormalWindow;
using yolo::JitResultPhase;
std::size_t checks=0;
void check(bool value,int line) {
    ++checks;
    if(!value) throw std::runtime_error("formal window test line "+std::to_string(line));
}
#define CHECK(value) check(static_cast<bool>(value),__LINE__)
template<class F> void rejects(F&& f) {
    bool rejected=false;
    try { std::forward<F>(f)(); } catch(const std::exception&) { rejected=true; }
    CHECK(rejected);
}
perception::native_depth::SourceId source(std::uint64_t sequence=1) {
    return {7,sequence,sequence+10,sequence+20,sequence*1000+1,sequence*1000+2,1000000000};
}
FormalWindow window() { return {3,1000,30000,900}; }
constexpr auto measuring=JitResultPhase::Measuring;
constexpr auto calibrating=JitResultPhase::Calibrating;
constexpr double delay=17.123456789012345;
constexpr std::int64_t boundary=31000000000LL;
void start(FormalWindow& w) {
    const auto selected=w.select(measuring,delay,source(),3,boundary,0);
    CHECK(selected.formal && selected.index==1 && !selected.after);
    w.commit_successful_put();
}
void phase_and_queued_result() {
    yolo::DetectionResult result;
    CHECK(result.jit_phase==JitResultPhase::Unavailable && result.jit_frozen_delay_ms==-1.0);
    for(const auto& pair:{std::pair{"CALIBRATION_WARMUP",JitResultPhase::CalibrationWarmup},
                         std::pair{"CALIBRATING",calibrating},std::pair{"MEASURING",measuring}}) {
        CHECK(yolo::jit_result_phase_from_name(pair.first)==pair.second);
        CHECK(std::string(rns_smoke_support::jit_phase_name(pair.second))==pair.first);
    }
    CHECK(yolo::jit_result_phase_from_name("unknown")==JitResultPhase::Unavailable);
    CHECK(yolo::jit_result_phase_from_name("measuring")==JitResultPhase::Unavailable);
    auto w=window();
    // Reproduce the old seam: this result was enqueued by CALIBRATING, then
    // the global owner froze before the callback. The selector cannot read it.
    result.jit_phase=yolo::jit_result_phase_from_name("CALIBRATING");
    result.jit_frozen_delay_ms=-1.0;
    result.native_depth_source=source(420);
    const bool later_global_freeze=true;
    CHECK(later_global_freeze);
    const auto queued=w.select(result.jit_phase,result.jit_frozen_delay_ms,
        result.native_depth_source,3,boundary+1000000,0);
    CHECK(!queued.formal && !queued.after && queued.index==0 && !w.bound());
    w.commit_successful_put();
    result.jit_phase=yolo::jit_result_phase_from_name("MEASURING");
    result.jit_frozen_delay_ms=delay;
    result.native_depth_source=source(421);
    const auto first=w.select(result.jit_phase,result.jit_frozen_delay_ms,
        result.native_depth_source,3,boundary+2000000,0);
    CHECK(first.formal && first.index==1);
    CHECK(w.clock_epoch()==3 && w.stream_id()==7 && w.delay_ms()==delay);
    w.commit_successful_put();
}
void first900_no_reselection() {
    auto w=window();
    const auto early=w.select(measuring,delay,source(),3,boundary-1,0);
    CHECK(!early.formal && early.index==0 && !w.bound());
    w.commit_successful_put();
    for(std::size_t completed=0;completed<900;++completed) {
        const auto selected=w.select(measuring,delay,source(completed+2),3,
            boundary+static_cast<std::int64_t>(completed)*50000000,completed);
        CHECK(selected.formal && !selected.after && selected.index==completed+1);
        CHECK(w.delay_ms()==delay && w.stream_id()==7 && w.clock_epoch()==3);
        w.commit_successful_put();
    }
    const auto after=w.select(measuring,delay,source(902),3,boundary+45000000000LL,900);
    CHECK(!after.formal && after.after && after.index==0);
    w.commit_successful_put();
}
void exact_binding_and_sticky_failure() {
    for(int mismatch=0;mismatch<5;++mismatch) {
        auto w=window(); start(w);
        auto changed_source=source(2);
        auto phase=measuring;
        double changed_delay=delay;
        std::uint64_t epoch=3;
        auto before=boundary+50000000;
        if(mismatch==0) { phase=calibrating;changed_delay=-1.0; }
        if(mismatch==1) { changed_delay=std::nextafter(delay,std::numeric_limits<double>::infinity());
            CHECK(changed_delay!=delay && std::llround(changed_delay*1000)==std::llround(delay*1000)); }
        if(mismatch==2) changed_source.stream_id=8;
        if(mismatch==3) epoch=4;
        if(mismatch==4) before=boundary-1;
        rejects([&]{w.select(phase,changed_delay,changed_source,epoch,before,1);});
        // Restoring the expected fields may not replace the rejected member.
        rejects([&]{w.select(measuring,delay,source(3),3,boundary+100000000,1);});
    }
}
void malformed_and_uncommitted() {
    for(const double invalid:{-1.0,std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::max()}) {
        auto w=window();
        rejects([&]{w.select(measuring,invalid,source(),3,boundary,0);});
    }
    for(const auto phase:{JitResultPhase::Unavailable,static_cast<JitResultPhase>(255)}) {
        auto w=window();
        rejects([&]{w.select(phase,-1,source(),3,boundary,0);});
    }
    { auto w=window();rejects([&]{w.select(calibrating,0,source(),3,boundary,0);}); }
    { auto w=window();auto invalid=source();invalid.stream_id=0;
      rejects([&]{w.select(measuring,delay,invalid,3,boundary,0);}); }
    { auto w=window();auto invalid=source();invalid.sequence=0;
      rejects([&]{w.select(measuring,delay,invalid,3,boundary,0);}); }
    { auto w=window();rejects([&]{w.select(measuring,delay,source(),3,-1,0);}); }
    { auto w=window();rejects([&]{w.select(measuring,delay,source(),3,boundary,1);}); }
    { auto w=window();rejects([&]{w.select(measuring,delay,source(),3,boundary,901);}); }
    { auto w=window();rejects([&]{w.commit_successful_put();}); }
    { auto w=window();
      CHECK(w.select(measuring,delay,source(),3,boundary,0).formal);
      // A failed native put does not call commit_successful_put. It cannot be
      // retried with a different result to manufacture a full 900 cohort.
      rejects([&]{w.select(measuring,delay,source(2),3,boundary+50000000,0);});
      rejects([&]{w.commit_successful_put();}); }
    rejects([]{FormalWindow invalid(0,1000,30000,900);});
    rejects([]{FormalWindow invalid(3,-1,30000,900);});
    rejects([]{FormalWindow invalid(3,1000,-1,900);});
    rejects([]{FormalWindow invalid(3,1000,30000,0);});
}
void duration_cut_does_not_replace_or_weaken_identity() {
    auto w=window();start(w);
    // Native put crosses the time cut AFTER selecting an eligible result.
    const auto crossing=w.select(measuring,delay,source(2),3,boundary+50000000,1);
    CHECK(crossing.formal && crossing.index==2);
    w.commit_successful_put(false);
    const auto after=w.select(measuring,delay,source(3),3,boundary+100000000,1);
    CHECK(after.formal && after.index==2);
    w.commit_successful_put(false);
    // Even cleanup rows must retain the bound phase/source/D identity.
    rejects([&]{w.select(calibrating,-1.0,source(4),3,boundary+150000000,1);});
    rejects([&]{w.select(measuring,delay,source(5),3,boundary+200000000,1);});
}
}
int main() {
    try {
        phase_and_queued_result();first900_no_reselection();
        exact_binding_and_sticky_failure();malformed_and_uncommitted();duration_cut_does_not_replace_or_weaken_identity();
        std::cout<<"{\"status\":\"PASS\",\"groups\":5,\"checks\":"<<checks
            <<",\"synthetic_only\":true,\"device\":false,\"model_execution\":false,\"network\":false}\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
