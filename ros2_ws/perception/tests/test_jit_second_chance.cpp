/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_jit_second_chance.cpp
 * Brief: Unit tests for the JIT second-chance pop of a fresher frame.
 *
 * Description:
 * What it checks. When the first pop finds no fresher frame, one bounded second chance may take a
 * frame that arrives during the wait. The tests cover configuration, that ineligible branches do
 * nothing, a frame arriving during the wait with its source identity preserved, the queue staying
 * empty with no third chance, stop cases, and identity and failure cases.
 */

/* Production helper + real queue/source binding; synthetic frames, no backend. */
#include "common/jit_second_chance.hpp"
#include "common/inference_source_binding.hpp"
#include "common/threadsafe_queue.hpp"
#include <iostream>
#include <utility>

namespace {
using perception::JitSecondChance;
using perception::JitSecondChanceGate;
std::size_t checks=0;
void check(bool value,int line) {
    ++checks;
    if(!value) throw std::runtime_error("second chance test line "+std::to_string(line));
}
#define CHECK(value) check(static_cast<bool>(value),__LINE__)
template<class F> void rejects(F&& action) {
    bool rejected=false;
    try { std::forward<F>(action)(); } catch(const std::exception&) { rejected=true; }
    CHECK(rejected);
}
JitSecondChanceGate eligible() { return {true,true,true,false,true,false,false,true,true,10,100}; }
yolo::FrameData frame(std::uint64_t id=101) {
    yolo::FrameData f;f.frame_id=id;
    f.native_depth_source={3,id,1000+id,2000+id,3000000+id,4000000+id,5000000+static_cast<std::int64_t>(id)};
    f.pipeline_timing.frame_push_begin_ns=6000000+static_cast<std::int64_t>(id);
    return f;
}
bool same_source(const perception::native_depth::SourceId& a,const perception::native_depth::SourceId& b) {
    return a.stream_id==b.stream_id && a.sequence==b.sequence &&
        a.color_index==b.color_index && a.depth_index==b.depth_index &&
        a.color_device_us==b.color_device_us && a.depth_device_us==b.depth_device_us &&
        a.host_ready_mono_ns==b.host_ready_mono_ns;
}
struct Harness {
    yolo::ThreadsafeQueue<yolo::FrameData> queue{1};
    yolo::FrameData scratch;
    perception::InferenceSourceBinding binding;
    bool running=true,stop_after_pop=false,stop_in_prepare=false,throw_pop=false,throw_prepare=false;
    int running_reads=0,clock_reads=0,pops=0,prepares=0;
    std::uint64_t prepared_id=0;
    bool run(JitSecondChance& chance,JitSecondChanceGate gate=eligible()) {
        return chance.attempt(gate,scratch,
            [&]{++running_reads;return running;},
            [&]{return std::int64_t{7000000}+clock_reads++;},
            [&](yolo::FrameData& candidate) {
                ++pops;
                if(throw_pop) throw std::runtime_error("injected pop failure");
                const bool success=queue.try_pop(candidate);
                if(success && stop_after_pop) running=false;
                return success;
            },
            [&](yolo::FrameData&& candidate) {
                ++prepares;
                if(throw_prepare) throw std::runtime_error("injected prepare failure");
                // Same admission rule as the production prepare callback.
                if(candidate.frame_id<=gate.current_frame_id) throw std::logic_error("stale admission");
                binding.bind(candidate);prepared_id=candidate.frame_id;
                if(stop_in_prepare) running=false;
            });
    }
};
void untouched(const JitSecondChance& chance) {
    const auto& t=chance.trace();
    CHECK(!t.attempted && !t.succeeded && !t.stopped && !t.discarded_on_stop);
    CHECK(t.begin_ns==-1 && t.end_ns==-1 && t.candidate_frame_id==0 && t.candidate_push_begin_ns==-1);
}
void configuration() {
    CHECK(!perception::parse_jit_second_chance_enabled(nullptr));
    CHECK(!perception::parse_jit_second_chance_enabled("0"));
    CHECK(perception::parse_jit_second_chance_enabled("1"));
    for(const auto* value:{"","true","false","2","01"," 1","1 "})
        rejects([&]{perception::parse_jit_second_chance_enabled(value);});
    perception::validate_jit_second_chance_config(true,true,true,true,false,1);
    for(int invalid=0;invalid<6;++invalid) {
        const bool jit=invalid!=0,async=invalid!=1,profile=invalid!=2,fresh=invalid==3;
        const std::size_t capacity=invalid==4?0:invalid==5?2:1;
        rejects([&]{perception::validate_jit_second_chance_config(true,jit,async,profile,fresh,capacity);});
        perception::validate_jit_second_chance_config(false,jit,async,profile,fresh,capacity);
    }
    rejects([]{JitSecondChance invalid(0,0);});
}
void ineligible_branches_do_nothing() {
    for(int which=0;which<9;++which) {
        auto gate=eligible();
        if(which==0)gate.enabled=false;
        if(which==1)gate.measuring=false; // Both calibration phases are outside this path.
        if(which==2)gate.async_submit=false;
        if(which==3)gate.fresh_bind=true;
        if(which==4)gate.initial_attempted=false;
        if(which==5)gate.initial_succeeded=true;
        if(which==6)gate.initial_stopped=true;
        if(which==7)gate.current_ready=false;
        if(which==8)gate.next_slot_empty=false;
        Harness h;h.queue.push(frame());JitSecondChance chance(10,100);
        CHECK(!h.run(chance,gate));untouched(chance);
        CHECK(h.pops==0 && h.prepares==0 && h.running_reads==0 && h.clock_reads==0);
        CHECK(h.queue.size()==1);
    }
}
void arrives_during_wait_and_preserves_source() {
    Harness h;auto gate=eligible();
    CHECK(!h.queue.try_pop(h.scratch)); // Original ordinary try_pop really empty.
    auto candidate=frame();const auto expected=candidate.native_depth_source;
    const auto original_push=candidate.pipeline_timing.frame_push_begin_ns;
    // Simulated wait_ready completion: a complete FrameData has since arrived.
    h.queue.push(std::move(candidate));JitSecondChance chance(10,100);
    CHECK(h.run(chance,gate));
    CHECK(!gate.initial_succeeded && gate.initial_attempted); // No initial trace rewrite.
    CHECK(h.pops==1 && h.prepares==1 && h.clock_reads==2 && h.running_reads==2);
    CHECK(h.queue.empty() && h.prepared_id==101);
    const auto& t=chance.trace();
    CHECK(t.attempted && t.succeeded && !t.stopped && !t.discarded_on_stop);
    CHECK(t.begin_ns==7000000 && t.end_ns==7000001);
    CHECK(t.candidate_frame_id==101 && t.candidate_push_begin_ns==original_push);
    CHECK(same_source(h.binding.source(),expected));
    // Use the actual production binding to finish current and successor
    // independently; preparation itself emits no inference completion.
    perception::InferenceSourceBinding current;
    auto current_input=frame(100);const auto expected_current=current_input.native_depth_source;
    current.bind(current_input);
    std::int64_t observed=9000000;
    perception::rns::SourceJournal journal(perception::rns::SourceKind::InferenceComplete,8,
        {&observed,+[](void* p) noexcept {return (*static_cast<std::int64_t*>(p))++;}});
    CHECK(journal.published()==0);
    yolo::DetectionResult current_result;current_result.frame_id=100;
    current.complete(current_result,true,&journal);
    CHECK(same_source(current_result.native_depth_source,expected_current));
    yolo::DetectionResult next_result;next_result.frame_id=101;
    h.binding.complete(next_result,true,&journal);
    CHECK(same_source(next_result.native_depth_source,expected));
    CHECK(current_result.inference_completed && next_result.inference_completed);
    CHECK(journal.valid() && journal.published()==2);
    rejects([&]{h.run(chance,gate);});CHECK(h.pops==1 && h.prepares==1);
}
void remains_empty_and_no_third_chance() {
    Harness h;JitSecondChance chance(10,100);
    CHECK(!h.run(chance));
    CHECK(h.pops==1 && h.prepares==0 && h.clock_reads==2 && h.running_reads==1);
    const auto& t=chance.trace();
    CHECK(t.attempted && !t.succeeded && !t.stopped);
    CHECK(t.candidate_frame_id==0 && t.candidate_push_begin_ns==-1);
    h.queue.push(frame());
    rejects([&]{h.run(chance);}); // Late arrival does not cause a third pop.
    CHECK(h.pops==1 && h.queue.size()==1);
}
void stop_cases() {
    { Harness h;h.running=false;h.queue.push(frame());JitSecondChance chance(10,100);
      CHECK(!h.run(chance));const auto& t=chance.trace();
      CHECK(t.stopped && !t.attempted && !t.succeeded && !t.discarded_on_stop);
      CHECK(t.begin_ns==-1 && t.end_ns==-1 && h.pops==0 && h.prepares==0 && h.queue.size()==1); }
    { Harness h;h.stop_after_pop=true;auto candidate=frame();
      candidate.pipeline_timing.frame_push_begin_ns=0; // Preserve unavailable metadata, never invent now.
      h.queue.push(std::move(candidate));JitSecondChance chance(10,100);
      CHECK(!h.run(chance));const auto& t=chance.trace();
      CHECK(t.attempted && t.succeeded && t.stopped && t.discarded_on_stop);
      CHECK(t.candidate_frame_id==101 && t.candidate_push_begin_ns==0);
      CHECK(h.pops==1 && h.prepares==0 && h.queue.empty());
      CHECK(h.binding.source().stream_id==0); }
    { Harness h;h.stop_in_prepare=true;h.queue.push(frame());JitSecondChance chance(10,100);
      CHECK(h.run(chance));
      // Existing production submit_next has a fresh running check after prepare.
      const bool submit_next=h.running && h.prepared_id!=0;
      CHECK(!submit_next && h.prepares==1 && !chance.trace().discarded_on_stop); }
}
void identity_and_failure_cases() {
    for(int wrong=0;wrong<2;++wrong) {
        Harness h;h.queue.push(frame());JitSecondChance chance(10,100);auto gate=eligible();
        if(wrong==0)++gate.current_generation;else ++gate.current_frame_id;
        rejects([&]{h.run(chance,gate);});CHECK(h.pops==0 && h.prepares==0 && h.queue.size()==1);
    }
    { Harness h;h.throw_pop=true;h.queue.push(frame());JitSecondChance chance(10,100);
      rejects([&]{h.run(chance);});CHECK(h.pops==1 && h.prepares==0 && chance.trace().attempted);
      rejects([&]{h.run(chance);});CHECK(h.pops==1); }
    { Harness h;h.throw_prepare=true;h.queue.push(frame());JitSecondChance chance(10,100);
      rejects([&]{h.run(chance);});CHECK(h.pops==1 && h.prepares==1 && chance.trace().succeeded);
      CHECK(h.binding.source().stream_id==0);rejects([&]{h.run(chance);});CHECK(h.pops==1); }
    { Harness h;h.queue.push(frame(100));JitSecondChance chance(10,100);
      rejects([&]{h.run(chance);});CHECK(h.prepared_id==0 && h.binding.source().stream_id==0); }
}
}
int main() {
    try {
        configuration();ineligible_branches_do_nothing();arrives_during_wait_and_preserves_source();
        remains_empty_and_no_third_chance();stop_cases();identity_and_failure_cases();
        std::cout<<"{\"status\":\"PASS\",\"groups\":6,\"checks\":"<<checks
            <<",\"production_helper\":true,\"synthetic_only\":true,\"device\":false,\"model_execution\":false,\"network\":false}\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
