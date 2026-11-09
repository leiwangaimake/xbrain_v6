/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_health_pending.cpp
 * Brief: Unit tests for pending publications and late completions in the health window.
 *
 * Description:
 * What it checks. A put may complete after the window it belongs to. The tests keep the ledger
 * invariant across pending and independent topics, window boundaries and late completion, attempt
 * validation and association, malformed and conflicting completions, clock-fault settlement,
 * capacity and sequence limits, and a long split ledger without allocation. Rate samples go
 * through the SourceId interface via rns_health_feed.hpp.
 */

/* Synthetic split-publication health ledger. No threads, SDK, devices or puts. */
#include "output/rns_health_window.hpp"
#include "rns_health_feed.hpp"
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>

namespace {
std::atomic<std::size_t> allocations{0};
std::size_t groups=0, checks=0;
void* alloc(std::size_t n) {
    if (auto* p=std::malloc(n ? n : 1)) { ++allocations; return p; }
    throw std::bad_alloc();
}
void* aligned(std::size_t n, std::size_t a) {
    void* p=nullptr;
    if (posix_memalign(&p,a,n ? n : 1)) throw std::bad_alloc();
    ++allocations;
    return p;
}
[[noreturn]] void fail(const char* expr,int line) {
    std::cerr << "health pending CHECK " << line << ": " << expr << '\n'; std::abort();
}
#define CHECK(x) do { ++checks; if (!(x)) fail(#x,__LINE__); } while(false)
}
void* operator new(std::size_t n) { return alloc(n); }
void* operator new[](std::size_t n) { return alloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p,std::size_t) noexcept { std::free(p); }
void operator delete[](void* p,std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n,std::align_val_t a) { return aligned(n,static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n,std::align_val_t a) { return aligned(n,static_cast<std::size_t>(a)); }
void operator delete(void* p,std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p,std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p,std::size_t,std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p,std::size_t,std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::rns;
using Code=HealthEventCode;
PublishAudit audit(std::uint64_t seq,Millis capture,Millis before,Millis after,
        PutOutcome outcome=PutOutcome::Accepted) {
    PublishAudit a;
    a.seq=seq; a.capture_ms=capture; a.put_before_ms=before; a.put_return_ms=after;
    a.attempted=true; a.clock_valid=true; a.receipt={outcome,17}; a.packet_bytes=128;
    return a;
}
void invariant(const HealthPutCounts& c) {
    CHECK(c.attempted==c.accepted+c.rejected+c.unknown+c.pending);
}
void pending_and_independent_topics() {
    ++groups;
    HealthWindow w(0,8);
    auto s=w.snapshot(0);
    CHECK(s.valid && !s.objects.ever_observed && !s.objects.open_gap_ms);
    CHECK(w.observe_publication_attempt(Topic::Objects,1,80,100,100)==Code::Accepted);
    s=w.snapshot(150);
    CHECK(s.valid && s.objects.total.attempted==1 && s.objects.total.pending==1);
    CHECK(s.objects.window.pending==1 && !s.objects.window.unknown);
    CHECK(s.objects.latency_p50_ms==20 && s.objects.latency_p99_ms==20 && s.objects.open_gap_ms==50);
    CHECK(w.observe_publication_attempt(Topic::Profile,1,80,100,150)==Code::Accepted);
    CHECK(test_feed::depth(w,80,100,150)==Code::Accepted);
    CHECK(test_feed::inference(w,80,140,150)==Code::Accepted);
    s=w.snapshot(200);
    CHECK(s.profile.total.pending==1 && s.objects.total.pending==1);
    CHECK(s.depth.total_frames==1 && s.inference.total_frames==1);
    CHECK(w.resolve_publication(Topic::Objects,audit(1,80,100,220),220)==Code::Accepted);
    s=w.snapshot(220);
    CHECK(s.objects.total.accepted==1 && s.objects.total.pending==0 && s.profile.total.pending==1);
    CHECK(s.objects.total.attempted==1 && s.objects.open_gap_ms==120 && s.objects.latency_p99_ms==20);
    invariant(s.objects.total); invariant(s.profile.total);
}
void window_boundary_and_late_completion() {
    ++groups;
    for (const Millis end : {10099,10100,20100}) {
        HealthWindow w(0,1);
        CHECK(w.observe_publication_attempt(Topic::Objects,9,80,100,100)==Code::Accepted);
        const auto before=w.snapshot(end);
        CHECK(before.valid && before.objects.total.pending==1);
        CHECK(before.objects.window.pending==static_cast<std::uint64_t>(end<10100));
        CHECK(before.objects.open_gap_ms==end-100);
        CHECK(w.resolve_publication(Topic::Objects,audit(9,80,100,end),end)==Code::Accepted);
        const auto after=w.snapshot(end);
        CHECK(after.valid && after.objects.total.pending==0 && after.objects.total.accepted==1);
        CHECK(after.objects.window.accepted==static_cast<std::uint64_t>(end<10100));
        CHECK(after.objects.window.attempted==before.objects.window.attempted);
        CHECK(after.objects.open_gap_ms==before.objects.open_gap_ms &&
              after.objects.gap_p99_ms==before.objects.gap_p99_ms);
        CHECK(after.objects.latency_p99_ms.has_value()==(end<10100));
        CHECK(w.resolve_publication(Topic::Objects,audit(9,80,100,end),end)==Code::Duplicate);
        invariant(after.objects.total); invariant(after.objects.window);
    }
}
void attempt_validation_and_association() {
    ++groups;
    HealthWindow w(0,8);
    CHECK(w.observe_publication_attempt(Topic::Objects,1,80,100,100)==Code::Accepted);
    CHECK(w.observe_publication_attempt(Topic::Objects,1,80,100,110)==Code::Duplicate);
    CHECK(w.snapshot(110).valid);
    CHECK(w.observe_publication_attempt(Topic::Objects,2,90,110,110)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Objects,1,81,100,110)==Code::Invalid);
    auto s=w.snapshot(110);
    CHECK(s.objects.total.attempted==1 && s.objects.total.pending==1);
    CHECK(w.resolve_publication(Topic::Objects,audit(1,80,100,120),120)==Code::Accepted);
    CHECK(w.observe_publication_attempt(Topic::Objects,2,90,130,130)==Code::Accepted);
    // A replay of the immediately previous completion cannot settle the new call.
    CHECK(w.resolve_publication(Topic::Objects,audit(1,80,100,120),130)==Code::Duplicate);
    CHECK(w.snapshot(130).objects.total.pending==1);
    CHECK(w.resolve_publication(Topic::Objects,audit(2,90,130,140),140)==Code::Accepted);
    CHECK(w.resolve_publication(Topic::Objects,audit(1,80,100,120),140)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Objects,2,100,150,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Objects,0,100,150,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Status,3,100,150,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(static_cast<Topic>(42),3,100,150,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Profile,1,-1,150,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Profile,1,151,150,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Profile,1,100,151,150)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Profile,1,0,0,10000)==Code::Invalid); // Expired, never revived.
    s=w.snapshot(10000);
    CHECK(s.objects.total.attempted==2 && s.objects.total.accepted==2 && s.profile.total.attempted==0);
}
void malformed_and_conflicting_completions() {
    ++groups;
    for (unsigned i=0;i<15;++i) {
        HealthWindow w(0,2);
        CHECK(w.observe_publication_attempt(Topic::Objects,7,80,100,100)==Code::Accepted);
        auto a=audit(7,80,100,120);
        auto topic=Topic::Objects;
        switch(i) {
            case 0: a.seq=0; break;
            case 1: a.seq=8; break;
            case 2: a.capture_ms=81; break;
            case 3: a.put_before_ms=101; break;
            case 4: topic=Topic::Profile; break;
            case 5: topic=Topic::Status; break;
            case 6: a.attempted=false; break;
            case 7: a.receipt.outcome=static_cast<PutOutcome>(42); break;
            case 8: a.put_return_ms=121; break;
            case 9: a.put_return_ms=99; break;
            case 10: a.threw=true; break;
            case 11: a.capture_ms=-1; break;
            case 12: a.capture_ms=101; break;
            case 13: a.put_before_ms=121; break;
            default: topic=static_cast<Topic>(42); break;
        }
        CHECK(w.resolve_publication(topic,a,120)==Code::Invalid);
        auto s=w.snapshot(120);
        CHECK(!s.valid && s.objects.total.pending==1 && s.objects.total.attempted==1);
        CHECK(w.resolve_publication(Topic::Objects,audit(7,80,100,120),120)==Code::Accepted);
        CHECK(w.snapshot(120).objects.total.accepted==1);
    }
    for (unsigned i=0;i<7;++i) {
        HealthWindow w(0,2);
        CHECK(w.observe_publication_attempt(Topic::Objects,1,80,100,100)==Code::Accepted);
        auto a=audit(1,80,100,120,PutOutcome::Unknown);
        CHECK(w.resolve_publication(Topic::Objects,a,120)==Code::Accepted);
        CHECK(w.resolve_publication(Topic::Objects,a,120)==Code::Duplicate);
        CHECK(w.snapshot(120).valid);
        switch(i) {
            case 0: a.receipt.outcome=PutOutcome::Accepted; break;
            case 1: a.receipt.native_code=18; break;
            case 2: a.put_return_ms=121; break;
            case 3: a.clock_valid=false; break;
            case 4: a.threw=true; break;
            case 5: a.packet_bytes=256; break;
            default: a.capture_ms=81; break;
        }
        CHECK(w.resolve_publication(Topic::Objects,a,121)==Code::Invalid);
        const auto s=w.snapshot(121);
        CHECK(!s.valid && s.objects.total.unknown==1 && s.objects.total.accepted==0 && s.objects.total.pending==0);
    }
}
void clock_fault_settlement_and_clock_failure() {
    ++groups;
    HealthWindow w(0,4);
    CHECK(w.observe_publication_attempt(Topic::Objects,1,80,100,100)==Code::Accepted);
    auto a=audit(1,80,100,-1); a.clock_valid=false;
    CHECK(w.resolve_publication(Topic::Objects,a,110)==Code::Accepted);
    auto s=w.snapshot(110);
    CHECK(!s.valid && s.diagnostics.clock_faults==1 && s.diagnostics.invalid==0);
    CHECK(s.objects.total.attempted==1 && s.objects.total.accepted==1 && s.objects.total.pending==0);
    CHECK(s.objects.window.accepted==1 && s.objects.latency_p99_ms==20);
    CHECK(w.resolve_publication(Topic::Objects,a,110)==Code::Duplicate);
    CHECK(w.snapshot(110).diagnostics.clock_faults==1);
    CHECK(w.observe_publication_attempt(Topic::Objects,2,90,120,120)==Code::Accepted);
    auto b=audit(2,90,120,130,PutOutcome::Unknown); b.threw=true;
    CHECK(w.resolve_publication(Topic::Objects,b,119)==Code::Invalid);
    CHECK(w.snapshot(120).objects.total.pending==1);
    CHECK(w.resolve_publication(Topic::Objects,b,130)==Code::Accepted);
    s=w.snapshot(130);
    CHECK(s.objects.total.unknown==1 && s.objects.total.accepted==1 && !s.objects.total.pending);
}
void capacity_and_sequence_limits() {
    ++groups;
    HealthWindow w(0,1);
    CHECK(w.observe_publication_attempt(Topic::Objects,1,80,100,100)==Code::Accepted);
    CHECK(w.resolve_publication(Topic::Objects,audit(1,80,100,100),100)==Code::Accepted);
    CHECK(w.observe_publication_attempt(Topic::Objects,2,90,110,110)==Code::CapacityExceeded);
    auto s=w.snapshot(110);
    CHECK(s.objects.window.attempted==1 && s.objects.total.attempted==1 && !s.objects.total.pending);
    CHECK(s.objects.latency_p99_ms==20);
    CHECK(w.observe_publication_attempt(Topic::Objects,2,10100,10100,10100)==Code::Accepted);
    s=w.snapshot(20100);
    CHECK(!s.objects.window.attempted && s.objects.total.pending==1);
    CHECK(w.observe_publication_attempt(Topic::Objects,3,20100,20100,20100)==Code::Invalid);
    CHECK(w.resolve_publication(Topic::Objects,audit(2,10100,10100,20100),20100)==Code::Accepted);
    CHECK(w.observe_publication_attempt(Topic::Objects,std::numeric_limits<std::uint64_t>::max(),20101,20101,20101)==Code::Accepted);
    CHECK(w.resolve_publication(Topic::Objects,audit(std::numeric_limits<std::uint64_t>::max(),20101,20101,20101),20101)==Code::Accepted);
    CHECK(w.observe_publication_attempt(Topic::Objects,0,20102,20102,20102)==Code::Invalid);
    CHECK(w.observe_publication_attempt(Topic::Objects,3,20102,20102,20102)==Code::Invalid);
    s=w.snapshot(20102);
    CHECK(s.objects.total.attempted==3 && s.objects.total.accepted==3 && !s.objects.total.pending);
    invariant(s.objects.total);
}
void split_ledger_and_no_allocation() {
    ++groups;
    HealthWindow split(0,256);
    const auto before_alloc=allocations.load();
    for (Millis i=0;i<1000;++i) {
        const auto outcome=i%3==0 ? PutOutcome::Accepted : (i%3==1 ? PutOutcome::Rejected : PutOutcome::Unknown);
        const Millis before=100+i*51;
        for (const auto topic : {Topic::Profile,Topic::Objects}) {
            const auto a=audit(static_cast<std::uint64_t>(i+1),before-20,before,before+2,outcome);
            CHECK(split.observe_publication_attempt(topic,a.seq,a.capture_ms,before,before+2)==Code::Accepted);
            CHECK(split.resolve_publication(topic,a,before+2)==Code::Accepted);
        }
        CHECK(test_feed::depth(split,before-20,before,before+2)==Code::Accepted);
        CHECK(test_feed::inference(split,before-20,before+1,before+2)==Code::Accepted);
        const auto s=split.snapshot(before+2);
        // Events 51 ms apart, window (now-10000, now]: (i-j)*51+2 < 10000 keeps the last 197.
        const auto n=static_cast<std::uint64_t>(i+1);
        const auto in_window=std::min<std::uint64_t>(n,197);
        CHECK(s.valid && s.now_ms==before+2 && s.full_window==(before+2>=10000));
        CHECK(s.depth.total_frames==n && s.inference.total_frames==n);
        CHECK(s.depth.window_frames==in_window && s.inference.window_frames==in_window);
        for (const auto* p : {&s.profile,&s.objects}) {
            invariant(p->window); invariant(p->total);
            CHECK(p->ever_observed && p->total.attempted==n && !p->total.pending);
            CHECK(p->window.attempted==in_window && !p->window.pending);
            CHECK(p->latency_p50_ms==20 && p->latency_p99_ms==20 && p->open_gap_ms==2);
            CHECK(p->gap_max_ms==(i ? 51 : 2) && p->closed_gap_samples==(i<197 ? n-1 : in_window));
        }
    }
    CHECK(allocations.load()==before_alloc);
}
} // namespace
int main() {
    pending_and_independent_topics();
    window_boundary_and_late_completion();
    attempt_validation_and_association();
    malformed_and_conflicting_completions();
    clock_fault_settlement_and_clock_failure();
    capacity_and_sequence_limits();
    split_ledger_and_no_allocation();
    std::cout << "RNS health pending PASS groups=" << groups << " checks=" << checks
              << " [inflight, expiry, association, conflicts, clock, capacity, split-ledger+no-allocation]"
              << "; synthetic-only no device/actual-publish\n";
}
