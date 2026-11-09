/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_source_journal.cpp
 * Brief: Prepared source event transfer tests; synthetic clocks, no devices or network.
 *
 * Description:
 * What it checks. The source journal hands prepared native-source events from the capture side to
 * their consumers. Cases cover configuration, value copies and stage rules, ordering including
 * equal nanoseconds, overflow, negative inputs, a paused clock, re-entrancy, counter saturation,
 * concurrent handoff and allocation freedom.
 */

#include "output/rns_source_journal.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <thread>
#ifdef NDEBUG
#error "Source journal tests require enabled assertions"
#endif
static std::atomic<std::size_t> ordinary_news{0}, aligned_news{0};
void* operator new(std::size_t n) {
    ++ordinary_news; if (void* p = std::malloc(n ? n : 1)) return p; throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    try { return ::operator new(n); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept { return ::operator new(n, std::nothrow); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t alignment) {
    ++aligned_news; void* p = nullptr;
    if (posix_memalign(&p, static_cast<std::size_t>(alignment), n ? n : 1) == 0) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n, std::align_val_t a) { return ::operator new(n, a); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    try { return ::operator new(n, a); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return ::operator new(n, a, std::nothrow);
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }



namespace perception::rns {
struct SourceJournalTestAccess {
    static void positions(SourceJournal& j,std::uint64_t value){j.head_=value;j.tail_=value;}
    static void generation(SourceJournal& j,std::uint64_t value){j.generation_=value;}
    static void counters(SourceJournal& j,std::uint64_t overflow,std::uint64_t faults) {
        j.overflow_=overflow;j.faults_=faults;
    }
};
}
using namespace perception::rns;
namespace nd=perception::native_depth;
namespace {
using Steady=std::chrono::steady_clock;
std::size_t groups=0;
void pass(const char* name){++groups;std::cout<<"PASS "<<name<<'\n';}
nd::SourceId source(std::uint64_t seq=1) {
    nd::SourceId s;s.stream_id=7;s.sequence=seq;
    s.color_index=seq-1;s.depth_index=seq-1;s.color_device_us=seq+30;s.depth_device_us=seq+40;
    s.host_ready_mono_ns=100;return s;
}
DepthHealthObservation observation(std::uint64_t seq=1) {
    DepthHealthObservation d;d.code=DepthHealthCode::Ready;d.source=source(seq);
    d.roi_pixels=128000;d.invalid_pixels=static_cast<std::size_t>(seq%7);
    d.invalid_ratio=static_cast<double>(d.invalid_pixels)/d.roi_pixels;
    return d;
}
bool same(const nd::SourceId& a,const nd::SourceId& b) {
    return a.stream_id==b.stream_id&&a.sequence==b.sequence&&a.color_index==b.color_index&&
        a.depth_index==b.depth_index&&a.color_device_us==b.color_device_us&&
        a.depth_device_us==b.depth_device_us&&a.host_ready_mono_ns==b.host_ready_mono_ns;
}
bool same(const DepthHealthObservation& a,const DepthHealthObservation& b) {
    return a.code==b.code&&same(a.source,b.source)&&a.roi_kind==b.roi_kind&&
        a.roi_pixels==b.roi_pixels&&a.invalid_pixels==b.invalid_pixels&&a.invalid_ratio==b.invalid_ratio;
}
bool same(const SourceEvent& a,const SourceEvent& b) {
    return a.kind==b.kind&&same(a.source,b.source)&&a.observed_ns==b.observed_ns&&same(a.depth,b.depth);
}
struct Clock {
    std::int64_t now=100;std::uint64_t calls=0;bool always_odd=true;SourceJournal* journal=nullptr;
    static std::int64_t read(void* p) noexcept {
        auto& c=*static_cast<Clock*>(p);++c.calls;
        if(c.journal)c.always_odd=c.always_odd&&((c.journal->generation()&1U)!=0);
        return c.now;
    }
    SourceClock binding(){return {this,read};}
};
template<class Fn> void no_allocation(Fn fn) {
    const auto normal=ordinary_news.load(),aligned=aligned_news.load();fn();
    assert(ordinary_news==normal&&aligned_news==aligned);
}
void configuration() {
    Clock c;
    for(const auto kind:{SourceKind::DepthArrival,SourceKind::InferenceComplete,SourceKind::DepthProcessed}) {
        SourceJournal j(kind,1,c.binding());assert(j.valid()&&j.kind()==kind);
        assert(j.generation()==0&&j.published()==0&&j.consumed()==0);
    }
    SourceJournal no_clock(SourceKind::DepthArrival,1,{}),empty(SourceKind::DepthArrival,0,c.binding()),
        bad_kind(static_cast<SourceKind>(99),1,c.binding());
    for(auto* p:{&no_clock,&empty,&bad_kind}) {
        assert(!p->valid()&&!p->record(source())&&p->generation()==0);
        SourceEvent e;assert(!p->peek(e)&&!p->pop(e));
    }
    SourceClock stateless{nullptr,[](void*)noexcept->std::int64_t{return 100;}};
    SourceJournal j(SourceKind::DepthArrival,1,stateless);assert(j.valid()&&j.record(source()));
    assert(c.calls==0);
    pass("explicit_clock_only_no_fabricated_default_kind_capacity_checked");
}
void value_copy_and_stage_rules() {
    Clock c;SourceJournal arrival(SourceKind::DepthArrival,1,c.binding());c.journal=&arrival;
    auto s=source();s.color_index=0;s.color_device_us=0;
    assert(arrival.record(s));s.sequence=99;
    SourceEvent e;assert(arrival.peek(e)&&e.source.sequence==1&&e.source.color_device_us==0);
    assert(e.depth.code==DepthHealthCode::InvalidFrame&&!e.depth.invalid_ratio&&e.observed_ns==100);
    const auto first=e;assert(arrival.pop(e)&&same(first,e)&&!arrival.pop(e));
    assert(c.calls==1&&c.always_odd&&arrival.generation()==2);
    Clock p;SourceJournal processed(SourceKind::DepthProcessed,3,p.binding());p.journal=&processed;
    auto d=observation();const auto original=d;
    assert(processed.record(d));d.source.sequence=123;d.invalid_pixels=120000;
    assert(processed.pop(e)&&e.kind==SourceKind::DepthProcessed&&same(e.source,original.source)&&same(e.depth,original));
    // The journal does not silently repair evidence. Downstream must check
    // ratio correctness/provenance; missing or inconsistent values survive.
    d=observation(2);d.invalid_ratio=.75;assert(processed.record(d));
    assert(processed.pop(e)&&e.depth.invalid_ratio==.75);
    d=observation(3);d.invalid_ratio.reset();assert(processed.record(d));
    assert(processed.pop(e)&&!e.depth.invalid_ratio&&processed.valid());
    assert(p.always_odd);
    pass("raw_arrival_without_rgb_and_processed_owned_evidence_not_recomputed");
}
void ordering_and_equal_nanoseconds() {
    Clock c;SourceJournal j(SourceKind::InferenceComplete,5,c.binding());c.journal=&j;
    assert(j.record(source(1)));
    auto tied=source(4);tied.depth_device_us=source(1).depth_device_us;
    assert(j.record(tied)); // Same observed/host/device ns(us), distinct index/sequence.
    auto new_stream=source(1);new_stream.stream_id=2; // Numeric ID may decrease.
    assert(j.record(new_stream));
    SourceEvent e;const auto limit=j.published();assert(limit==3);
    std::size_t count=0;
    while(j.consumed()<limit){assert(j.pop(e)&&e.observed_ns==100);++count;}
    assert(count==3&&j.valid()&&j.generation()==6&&c.always_odd);
    const auto unchanged=e;assert(!j.peek(e)&&same(e,unchanged)&&!j.pop(e)&&same(e,unchanged));
    const auto before=j.generation();(void)j.valid();(void)j.overflow_count();(void)j.fault_count();
    assert(j.generation()==before&&c.calls==3);
    pass("same_ns_unique_sources_sequence_gaps_and_non_numeric_stream_transition");
}
void overflow() {
    Clock c;SourceJournal j(SourceKind::DepthArrival,1,c.binding());c.journal=&j;
    assert(j.record(source()));assert(!j.record(source(2)));
    assert(!j.valid()&&j.overflow_count()==1&&j.fault_count()==0&&j.generation()==4&&c.always_odd);
    assert(j.published()==1&&j.consumed()==0&&c.calls==2);
    assert(!j.record(source(3))&&c.calls==2); // Fault cannot masquerade as a repaired empty run.
    SourceEvent e;assert(j.pop(e)&&e.source.sequence==1&&!j.pop(e)&&j.consumed()==1);
    pass("queue_full_sticky_overflow_no_drop_overwrite_or_fake_zero_sample");
}
void negatives() {
    for(unsigned n=0;n<10;++n) {
        Clock c;SourceJournal j(SourceKind::InferenceComplete,2,c.binding());c.journal=&j;
        auto s=source();
        switch(n){
        case 0:s.stream_id=0;break;
        case 1:s.sequence=0;break;
        case 2:s.host_ready_mono_ns=0;break;
        case 3:s.host_ready_mono_ns=-1;break;
        case 4:s.depth_device_us=0;break;
        case 5:s.color_device_us=0;break;
        case 6:c.now=99;break;
        case 7:c.now=-1;break;
        case 8:c.now=0;break;
        default:break;
        }
        assert(n==9?!j.record(observation()):!j.record(s)); // Wrong overload also samples under odd.
        assert(!j.valid()&&j.generation()==2&&j.fault_count()==1&&j.published()==0);
        assert(c.calls==1&&c.always_odd);
    }
    for(unsigned n=0;n<6;++n) {
        Clock c;SourceJournal j(SourceKind::DepthArrival,2,c.binding());c.journal=&j;
        auto first=source();first.host_ready_mono_ns=90;assert(j.record(first));
        auto next=source(2);next.host_ready_mono_ns=90;
        switch(n) {
        case 0:next=first;break;
        case 1:next.sequence=first.sequence;break;
        case 2:next.depth_index=first.depth_index;break;
        case 3:next.depth_device_us=first.depth_device_us-1;break;
        case 4:next.host_ready_mono_ns=89;break;
        default:c.now=99;break;
        }
        assert(!j.record(next)&&!j.valid()&&j.published()==1&&j.generation()==4&&j.fault_count()==1);
        SourceEvent e;assert(j.pop(e)&&same(e.source,first)&&!j.pop(e));
    }
    for(unsigned n=0;n<5;++n) {
        Clock c;SourceJournal j(SourceKind::DepthProcessed,1,c.binding());c.journal=&j;
        auto d=observation();
        switch(n) {
        case 0:d.code=DepthHealthCode::InvalidFrame;break;
        case 1:d.roi_pixels=0;break;
        case 2:d.invalid_pixels=d.roi_pixels+1;break;
        case 3:d.source.color_device_us=0;break;
        default:break;
        }
        assert(n==4?!j.record(source()):!j.record(d));
        assert(!j.valid()&&j.generation()==2&&j.fault_count()==1&&j.published()==0&&c.always_odd);
    }
    pass("twentyone_source_order_clock_stage_and_count_negative_controls_even_exit");
}
struct PausedClock {
    std::atomic<bool> entered{false},release{false},failed{false};
    SourceJournal* journal=nullptr;
    static std::int64_t read(void* ptr)noexcept {
        auto& c=*static_cast<PausedClock*>(ptr);
        if(!c.journal||(c.journal->generation()&1U)==0)c.failed=true;
        c.entered.store(true,std::memory_order_release);
        const auto deadline=Steady::now()+std::chrono::seconds(5);
        while(!c.release.load(std::memory_order_acquire)){
            if(Steady::now()>=deadline){c.failed=true;return -1;}
            std::this_thread::yield();
        }
        return 200;
    }
};
void paused_clock() {
    PausedClock c;SourceJournal j(SourceKind::DepthArrival,1,{&c,PausedClock::read});c.journal=&j;
    bool accepted=false;std::thread producer([&]{accepted=j.record(source());});
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    while(!c.entered.load(std::memory_order_acquire)){
        if(Steady::now()>=deadline){c.failed=true;break;}
        std::this_thread::yield();
    }
    SourceEvent e;
    const bool hidden=j.valid()&&j.generation()==1&&j.published()==0&&!j.peek(e)&&!j.pop(e);
    c.release.store(true,std::memory_order_release);producer.join();
    assert(hidden&&accepted&&!c.failed&&j.valid()&&j.generation()==2);
    assert(j.pop(e)&&e.observed_ns==200&&same(e.source,source()));
    pass("callback_pause_exposes_odd_before_time_sample_no_early_commit");
}
void reentrancy() {
    struct Context {
        SourceJournal* j=nullptr;bool inner_result=true;std::uint32_t calls=0;
        static std::int64_t read(void* p)noexcept {
            auto& c=*static_cast<Context*>(p);++c.calls;c.inner_result=c.j->record(source(2));return 100;
        }
    } c;
    SourceJournal j(SourceKind::DepthArrival,1,{&c,Context::read});c.j=&j;
    assert(!j.record(source())&&!c.inner_result&&c.calls==1);
    assert(!j.valid()&&j.fault_count()==1&&j.generation()==2&&j.published()==0);
    pass("same_producer_callback_reentry_fault_outer_record_restores_even");
}
void saturation() {
    constexpr auto max=std::numeric_limits<std::uint64_t>::max();
    {
        Clock c;SourceJournal j(SourceKind::DepthArrival,1,c.binding());
        SourceJournalTestAccess::positions(j,max-1);assert(j.record(source())&&j.published()==max);
        SourceEvent e;assert(j.pop(e)&&j.consumed()==max&&!j.pop(e));
        assert(!j.record(source(2))&&!j.valid()&&j.generation()==4&&j.published()==max);
    }
    {
        Clock c;SourceJournal j(SourceKind::DepthArrival,1,c.binding());
        SourceJournalTestAccess::generation(j,max-3);assert(j.record(source()));
        assert(j.generation()==max-1&&!j.record(source(2))&&!j.valid());
        assert(j.generation()==max-1&&c.calls==1);
    }
    {
        Clock c;SourceJournal j(SourceKind::DepthArrival,1,c.binding());assert(j.record(source()));
        SourceJournalTestAccess::counters(j,max,0);assert(!j.record(source(2))&&!j.valid()&&j.overflow_count()==max);
    }
    {
        Clock c;SourceJournal j(SourceKind::DepthArrival,1,c.binding());
        SourceJournalTestAccess::counters(j,0,max);c.now=-1;
        assert(!j.record(source())&&!j.valid()&&j.fault_count()==max&&j.generation()==2);
    }
    pass("head_generation_and_both_counters_saturate_without_wraparound");
}
void concurrent_handoff() {
    constexpr std::uint64_t count=50000,capacity=64;
    Clock clock;SourceJournal j(SourceKind::DepthProcessed,capacity,clock.binding());clock.journal=&j;
    std::atomic<unsigned> ready{0};std::atomic<bool> start{false},failed{false};
    std::uint64_t received=0;
    std::thread producer([&]{
        ++ready;while(!start.load(std::memory_order_acquire))std::this_thread::yield();
        const auto deadline=Steady::now()+std::chrono::seconds(10);
        for(std::uint64_t i=1;i<=count&&!failed.load();++i){
            // Test driver paces before record; the journal itself never waits
            // for space and would latch overflow on a full attempted record.
            while(j.published()-j.consumed()>=capacity){
                if(failed.load()||Steady::now()>=deadline){failed=true;return;}
                std::this_thread::yield();
            }
            clock.now=static_cast<std::int64_t>(i)+1000;auto d=observation(i);
            d.source.host_ready_mono_ns=static_cast<std::int64_t>(i)+500;
            if(!j.record(d)){failed=true;return;}
            d.source.sequence=0;d.invalid_ratio.reset(); // No retained caller view.
        }
    });
    std::thread consumer([&]{
        ++ready;while(!start.load(std::memory_order_acquire))std::this_thread::yield();
        const auto deadline=Steady::now()+std::chrono::seconds(10);
        for(std::uint64_t i=1;i<=count&&!failed.load();++i){
            SourceEvent e,p;
            while(!j.peek(p)){
                if(failed.load()||Steady::now()>=deadline){failed=true;return;}
                std::this_thread::yield();
            }
            auto d=observation(i);d.source.host_ready_mono_ns=static_cast<std::int64_t>(i)+500;
            if(!j.pop(e)||!same(e,p)||e.kind!=SourceKind::DepthProcessed||
                e.observed_ns!=static_cast<std::int64_t>(i)+1000||
                !same(e.source,d.source)||!same(e.depth,d)){failed=true;return;}
            ++received;
        }
    });
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    while(ready.load()!=2){
        if(Steady::now()>=deadline){failed=true;break;}
        std::this_thread::yield();
    }
    start.store(true,std::memory_order_release);producer.join();consumer.join();
    assert(!failed&&j.valid()&&received==count&&j.published()==count&&j.consumed()==count);
    assert(j.generation()==count*2&&clock.calls==count&&clock.always_odd);
    assert(j.overflow_count()==0&&j.fault_count()==0);
    pass("two_real_threads_50000_complete_owned_events_fixed_ring_and_clock_sequence");
}
void allocation_freedom() {
    Clock a,b,c;SourceJournal arrival(SourceKind::DepthArrival,1,a.binding()),
        inference(SourceKind::InferenceComplete,1,b.binding()),processed(SourceKind::DepthProcessed,1,c.binding());
    a.journal=&arrival;b.journal=&inference;c.journal=&processed;
    no_allocation([&] {
        SourceEvent e;
        for(std::uint64_t i=1;i<=2000;++i){
            assert(arrival.record(source(i))&&arrival.peek(e)&&arrival.pop(e)&&!arrival.pop(e));
            assert(inference.record(source(i))&&inference.peek(e)&&inference.pop(e));
            assert(processed.record(observation(i))&&processed.peek(e)&&processed.pop(e));
            (void)processed.valid();(void)processed.generation();(void)processed.published();
            (void)processed.consumed();(void)processed.overflow_count();(void)processed.fault_count();
        }
        assert(arrival.record(source(2001))&&!arrival.record(source(2002)));
        assert(!arrival.valid()&&arrival.overflow_count()==1);
        assert(!inference.record(observation(2001))&&!inference.valid());
    });
    pass("three_record_paths_consumer_and_failure_paths_no_cpp_heap_allocations");
}
}
int main(){
    configuration();value_copy_and_stage_rules();ordering_and_equal_nanoseconds();overflow();negatives();
    paused_clock();reentrancy();saturation();concurrent_handoff();allocation_freedom();
    std::cout<<"SourceJournal PASS: "<<groups<<" groups; 50000 concurrent events; no devices/network.\n";
}

