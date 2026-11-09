/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_publication_journal.cpp
 * Brief: SPSC publication journal tests: synthetic audits, no network, device or clock acceptance.
 *
 * Description:
 * What it checks. The journal carries publication audits from the publishing thread to the Status
 * owner. Cases cover configuration, ordering and raw completion, reservation and cancellation, a
 * paused producer, illegal lifecycle calls, exhaustion, concurrent handoff and allocation freedom.
 */

#include "output/rns_publication_journal.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <thread>
#ifdef NDEBUG
#error "Publication journal tests require enabled assertions"
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
struct PublicationJournalTestAccess {
    static void positions(PublicationJournal& j,std::uint64_t value) {
        j.head_.store(value);j.tail_.store(value);
    }
    static void generation(PublicationJournal& j,std::uint64_t value) {j.generation_.store(value);}
    static void counters(PublicationJournal& j,std::uint64_t admission,std::uint64_t protocol) {
        j.admission_rejected_.store(admission);j.protocol_faults_.store(protocol);
    }
};
}
using namespace perception::rns;
namespace {
using Journal=PublicationJournal;
using Event=Journal::Event;
using Kind=Event::Kind;
using Steady=std::chrono::steady_clock;
std::size_t groups=0;
void pass(const char* name){++groups;std::cout<<"PASS "<<name<<'\n';}
bool same(const PublishAudit& a,const PublishAudit& b) {
    return a.attempted==b.attempted&&a.threw==b.threw&&a.clock_valid==b.clock_valid&&
        a.seq==b.seq&&a.capture_ms==b.capture_ms&&a.put_before_ms==b.put_before_ms&&
        a.put_return_ms==b.put_return_ms&&a.packet_bytes==b.packet_bytes&&
        a.receipt.outcome==b.receipt.outcome&&a.receipt.native_code==b.receipt.native_code&&
        a.cadence_event==b.cadence_event;
}
bool same(const Event& a,const Event& b) {
    return a.kind==b.kind&&a.topic==b.topic&&same(a.audit,b.audit);
}
PublishAudit attempt(std::uint64_t seq=1) {
    PublishAudit a;a.attempted=true;a.seq=seq;
    a.capture_ms=static_cast<Millis>(seq)-1;a.put_before_ms=static_cast<Millis>(seq)+10;
    a.packet_bytes=256+seq%10;return a;
}
PublishAudit complete(PublishAudit a) {
    a.put_return_ms=a.put_before_ms+1;a.clock_valid=true;
    if(a.seq%3==0)a.receipt={PutOutcome::Rejected,-12};
    else if(a.seq%3==1)a.receipt={PutOutcome::Accepted,0};
    else {a.receipt={PutOutcome::Unknown,-17};a.threw=true;}
    if(a.seq%7==0) {a.clock_valid=false;a.put_return_ms=-1;}
    a.cadence_event=CadenceEvent::New;return a;
}
void begin(Journal& j,const PublishAudit& a) {
    assert(j.reserve());j.begin_attempt();j.record_attempt(a);assert(j.valid());
}
void end(Journal& j,const PublishAudit& a) {
    j.begin_completion();j.record_completion(complete(a));assert(j.valid());
}
template<class Fn> void no_allocation(Fn fn) {
    const auto ordinary=ordinary_news.load(),aligned=aligned_news.load();
    fn();
    assert(ordinary_news.load()==ordinary&&aligned_news.load()==aligned);
}
void configuration() {
    Journal p(Topic::Profile,2),o(Topic::Objects,3);
    assert(p.valid()&&p.topic()==Topic::Profile&&o.valid()&&o.topic()==Topic::Objects);
    for(const auto topic:{Topic::Status,static_cast<Topic>(99)}) {
        Journal invalid(topic,2);assert(!invalid.valid()&&!invalid.reserve());
        Event e;assert(!invalid.peek(e)&&!invalid.pop(e));
    }
    for(const auto capacity:{std::size_t{0},std::size_t{1}}) {
        Journal invalid(Topic::Objects,capacity);assert(!invalid.valid()&&!invalid.reserve());
        Event e;assert(!invalid.peek(e)&&!invalid.pop(e));
    }
    pass("configuration_only_profile_objects_capacity_two_or_more");
}
void ordering_and_raw_completion() {
    Journal j(Topic::Objects,2);const auto a=attempt(7),c=complete(a);
    Event e;e.kind=Kind::Completion;e.audit.seq=900;const auto untouched=e;
    assert(!j.peek(e)&&same(e,untouched)&&!j.pop(e)&&same(e,untouched));
    assert(j.reserve()&&j.generation()==0&&j.published()==0);
    j.begin_attempt();assert(j.generation()==1&&!j.peek(e));
    j.record_attempt(a);assert(j.generation()==2&&j.published()==1&&j.consumed()==0);
    assert(j.peek(e)&&e.kind==Kind::Attempt&&e.topic==Topic::Objects&&same(e.audit,a));
    const auto again=e;assert(j.peek(e)&&same(e,again)&&j.consumed()==0);
    // Save a finite drain head; a later Completion does not extend that batch.
    const auto captured_head=j.published();
    assert(j.pop(e)&&j.consumed()==captured_head);
    j.begin_completion();assert(j.generation()==3&&!j.peek(e));
    j.record_completion(c);assert(j.generation()==4&&j.published()==2);
    assert(j.consumed()==captured_head); // Caller stops draining at its saved head.
    assert(j.pop(e)&&e.kind==Kind::Completion&&same(e.audit,c));
    assert(!e.audit.clock_valid&&e.audit.put_return_ms==-1);
    assert(j.consumed()==2&&!j.pop(e)&&j.valid());
    pass("attempt_visible_pending_completion_keeps_real_bad_clock_and_exact_identity");
}
void reservation() {
    Journal j(Topic::Profile,3);Event e;
    const auto a=attempt();begin(j,a);end(j,a);assert(j.published()==2);
    assert(!j.reserve()&&j.valid()&&j.generation()==4&&j.counters().admission_rejected==1);
    assert(j.pop(e)); // Exactly two free slots: a complete publication fits.
    const auto b=attempt(2);assert(j.reserve());j.begin_attempt();j.record_attempt(b);
    assert(j.pop(e)&&e.kind==Kind::Completion&&same(e.audit,complete(a)));
    assert(j.pop(e)&&e.kind==Kind::Attempt&&same(e.audit,b));
    // Consumer progress is optional; the pending completion slot was reserved.
    j.begin_completion();j.record_completion(complete(b));assert(j.valid());
    assert(j.pop(e)&&same(e.audit,complete(b))&&!j.pop(e));
    assert(j.published()==4&&j.consumed()==4);
    Journal exact(Topic::Objects,2);begin(exact,a);end(exact,a);
    assert(!exact.reserve()&&exact.valid());
    assert(exact.pop(e));assert(!exact.reserve()&&exact.valid()); // One slot is insufficient.
    assert(exact.pop(e));assert(exact.reserve());exact.cancel();
    pass("reserve_two_slots_full_rejection_and_ring_reuse_without_partial_admission");
}
void cancellation() {
    Journal j(Topic::Profile,2);Event e;
    assert(j.reserve());j.cancel();assert(j.valid()&&j.generation()==0&&j.published()==0);
    assert(j.reserve());j.begin_attempt();assert(j.generation()==1);
    j.cancel();assert(j.valid()&&j.generation()==2&&!j.peek(e)&&j.published()==0);
    const auto a=attempt();begin(j,a);end(j,a);
    assert(j.generation()==6&&j.published()==2&&j.counters().protocol_faults==0);
    pass("preflight_prepare_and_failed_timestamp_cancel_release_reservation_no_events");
}
void pending_producer_pause() {
    Journal j(Topic::Objects,2);
    std::atomic<bool> pending{false},release{false},failed{false};
    std::thread producer([&] {
        const auto a=attempt();
        if(!j.reserve()){failed=true;return;}
        j.begin_attempt();j.record_attempt(a);pending.store(true,std::memory_order_release);
        const auto deadline=Steady::now()+std::chrono::seconds(5);
        while(!release.load(std::memory_order_acquire)) {
            if(Steady::now()>=deadline){failed=true;return;}
            std::this_thread::yield();
        }
        j.begin_completion();j.record_completion(complete(a));
    });
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    while(!pending.load(std::memory_order_acquire)&&!failed.load()) {
        if(Steady::now()>=deadline){failed=true;break;}
        std::this_thread::yield();
    }
    Event e;
    const bool observed=!failed.load()&&j.valid()&&j.generation()==2&&j.published()==1&&
        j.pop(e)&&e.kind==Kind::Attempt&&same(e.audit,attempt())&&!j.peek(e);
    release.store(true,std::memory_order_release);producer.join();
    assert(observed&&!failed&&j.valid()&&j.generation()==4&&j.published()==2);
    assert(j.pop(e)&&e.kind==Kind::Completion&&same(e.audit,complete(attempt())));
    assert(!j.pop(e));
    pass("two_threads_simulated_pending_put_keeps_even_generation_and_attempt_consumable");
}
void illegal_lifecycle() {
    for(unsigned variant=0;variant<17;++variant) {
        Journal j(Topic::Profile,4);const auto a=attempt();
        switch(variant) {
        case 0:j.begin_attempt();break;
        case 1:j.record_attempt(a);break;
        case 2:j.begin_completion();break;
        case 3:j.record_completion(complete(a));break;
        case 4:j.cancel();break;
        case 5:assert(j.reserve());assert(!j.reserve());break;
        case 6:assert(j.reserve());j.begin_attempt();j.begin_attempt();break;
        case 7:{
            assert(j.reserve());j.begin_attempt();auto invalid=a;invalid.seq=0;j.record_attempt(invalid);
            assert(j.generation()==1&&j.published()==0);break;
        }
        case 8:begin(j,a);j.record_attempt(a);break;
        case 9:begin(j,a);j.cancel();break;
        case 10:begin(j,a);j.record_completion(complete(a));break;
        case 11:begin(j,a);end(j,a);j.record_completion(complete(a));break;
        case 12:{
            begin(j,a);j.begin_completion();auto invalid=complete(a);++invalid.seq;
            j.record_completion(invalid);break;
        }
        case 13:{
            begin(j,a);j.begin_completion();auto invalid=complete(a);++invalid.packet_bytes;
            j.record_completion(invalid);break;
        }
        case 14:begin(j,a);j.begin_completion();j.begin_completion();break;
        case 15:{
            assert(j.reserve());j.begin_attempt();auto invalid=a;invalid.capture_ms=-1;
            j.record_attempt(invalid);break;
        }
        default:{
            assert(j.reserve());j.begin_attempt();auto invalid=a;invalid.put_before_ms=invalid.capture_ms-1;
            j.record_attempt(invalid);break;
        }
        }
        assert(!j.valid()&&j.counters().protocol_faults==1&&!j.reserve());
        const auto committed=j.published();
        Event e;std::uint64_t drained=0;
        while(j.pop(e)) {
            assert(e.audit.seq==1&&e.topic==Topic::Profile);
            ++drained;
        }
        assert(drained==committed&&j.consumed()==committed);
    }
    pass("seventeen_illegal_lifecycles_sticky_fault_without_fabricated_events");
}
void exhaustion() {
    constexpr auto max=std::numeric_limits<std::uint64_t>::max();
    {
        Journal j(Topic::Profile,2);PublicationJournalTestAccess::positions(j,max-2);
        const auto a=attempt();begin(j,a);end(j,a);assert(j.published()==max);
        Event e;assert(j.pop(e)&&j.pop(e)&&j.consumed()==max&&!j.pop(e));
        assert(!j.reserve()&&!j.valid()&&j.published()==max&&j.consumed()==max);
    }
    {
        Journal j(Topic::Profile,2);PublicationJournalTestAccess::generation(j,max-5);
        const auto a=attempt();begin(j,a);end(j,a);assert(j.generation()==max-1);
        Event e;assert(j.pop(e)&&j.pop(e));
        assert(!j.reserve()&&!j.valid()&&j.generation()==max-1);
    }
    {
        Journal j(Topic::Profile,2);const auto a=attempt();begin(j,a);end(j,a);
        PublicationJournalTestAccess::counters(j,max,0);
        assert(!j.reserve()&&!j.valid()&&j.counters().admission_rejected==max);
    }
    {
        Journal j(Topic::Profile,2);PublicationJournalTestAccess::counters(j,0,max);
        j.begin_completion();assert(!j.valid()&&j.counters().protocol_faults==max&&!j.reserve());
    }
    pass("positions_generation_and_counters_exhaust_without_unsigned_wrap");
}
void concurrent_handoff() {
    constexpr std::uint64_t count=50000;
    Journal j(Topic::Objects,64);
    std::atomic<bool> start{false},failed{false};
    std::atomic<unsigned> ready{0};
    std::uint64_t received=0;
    std::thread producer([&] {
        ++ready;while(!start.load(std::memory_order_acquire))std::this_thread::yield();
        const auto deadline=Steady::now()+std::chrono::seconds(10);
        for(std::uint64_t seq=1;seq<=count&&!failed.load();++seq) {
            while(!j.reserve()) {
                if(!j.valid()||failed.load()||Steady::now()>=deadline){failed=true;return;}
                std::this_thread::yield();
            }
            const auto a=attempt(seq);
            j.begin_attempt();j.record_attempt(a);j.begin_completion();j.record_completion(complete(a));
            if(!j.valid()){failed=true;return;}
        }
    });
    std::thread consumer([&] {
        ++ready;while(!start.load(std::memory_order_acquire))std::this_thread::yield();
        const auto deadline=Steady::now()+std::chrono::seconds(10);
        for(std::uint64_t i=0;i<count*2&&!failed.load();++i) {
            Event p,e;
            while(!j.peek(p)) {
                if(failed.load()||Steady::now()>=deadline){failed=true;return;}
                std::this_thread::yield();
            }
            const auto a=attempt(i/2+1);
            const Event expected{i%2?Kind::Completion:Kind::Attempt,Topic::Objects,i%2?complete(a):a};
            if(!same(p,expected)||!j.pop(e)||!same(p,e)){failed=true;return;}
            ++received;
        }
    });
    const auto deadline=Steady::now()+std::chrono::seconds(5);
    while(ready.load()!=2) {
        if(Steady::now()>=deadline){failed=true;break;}
        std::this_thread::yield();
    }
    start.store(true,std::memory_order_release);producer.join();consumer.join();
    assert(!failed&&j.valid()&&received==count*2);
    assert(j.published()==count*2&&j.consumed()==count*2&&j.generation()==count*4);
    assert(j.counters().protocol_faults==0);
    pass("two_real_threads_100000_exact_events_acquire_release_ring_handoff");
}
void allocation_freedom() {
    Journal j(Topic::Objects,2);
    no_allocation([&] {
        Event e;
        for(std::uint64_t i=1;i<=2000;++i) {
            assert(j.reserve());j.cancel();
            assert(j.reserve());j.begin_attempt();j.cancel();
            const auto a=attempt(i);
            begin(j,a);end(j,a);
            assert(!j.reserve()&&j.valid()); // Includes the bounded full-queue counter path.
            assert(j.peek(e)&&j.pop(e)&&j.pop(e)&&!j.peek(e));
            (void)j.counters();(void)j.generation();(void)j.published();(void)j.consumed();
        }
        j.begin_completion();assert(!j.valid()&&!j.reserve()); // Fault path also allocates nothing.
    });
    pass("all_hot_paths_no_ordinary_or_aligned_cpp_allocations");
}
}
int main() {
    configuration();ordering_and_raw_completion();reservation();cancellation();pending_producer_pause();
    illegal_lifecycle();exhaustion();concurrent_handoff();allocation_freedom();
    std::cout<<"PublicationJournal PASS: "<<groups<<" groups; 100000 concurrent events; no devices/network.\n";
}
