/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_declaration_handoff.cpp
 * Brief: Objects declaration -> certified cut -> independent Status.
 *
 * Description:
 * Injected only; no camera/network, production lifecycle owner or 20 Hz acceptance evidence.
 */

#include "output/rns_publication_handoff.hpp"
#include "output/rns_health_fields.hpp"
#include "output/rns_status_heartbeat.hpp"
#include <array>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>

static std::atomic<std::size_t> allocations{0};
void* operator new(std::size_t n) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    if (auto* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace perception::rns;
namespace {
std::size_t checks = 0, groups = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << __FILE__ << ':' << __LINE__ \
    << ": " #x "\n"; std::abort(); } } while (false)
struct Clock : PublishClock {
    Millis now = 0;
    Millis mono_millis() noexcept override { return now; }
    Millis wall_millis() noexcept override { return 1700000000000LL; }
};
struct Sink final : RnsTransport {
    std::array<char, 16384> packet{};
    std::size_t calls = 0;
    PutReceipt put(std::string_view, std::string_view p, PublishQos) override {
        CHECK(p.size() < packet.size());
        std::memcpy(packet.data(), p.data(), p.size()); packet[p.size()] = 0;
        ++calls; return {PutOutcome::Accepted, 0};
    }
};
Objects body(Millis capture) { Objects o; o.t_capture_mono_ms = capture; return o; }
struct Rig {
    Clock clock;
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublicationJournal p{Topic::Profile, 64}, o{Topic::Objects, 64};
    HealthWindow health{0, 64}; ObjectsCadence mirror{0, 64}, owner{0, 64};
    DepthHealthWindow depth{0, 64};
    Sink sink, status_sink;
    PublisherChannel objects{Topic::Objects, envelope, clock, sink, 4096, 8192, &owner, &o};
    PublisherChannel status{Topic::Status, envelope, clock, status_sink, 4096, 8192};
    PublicationHealthHandoff h{0, p, o, health, mirror};
    PublicationCut drain(Millis now) { clock.now = now; return h.drain(clock); }
    void put(Millis capture, Millis now) { clock.now = now; CHECK(objects.publish(body(capture))); }
    void declare(Exclusion e, Millis now) {
        clock.now = now; const auto r = objects.declare_exclusion(e);
        CHECK(r && r.declaration == e && r.declared_ms == now);
    }
};
void late_declaration_cannot_heal_history() {
    Rig r; r.put(29960, 30000);
    CHECK(r.drain(30001).code == HandoffCode::Ready);
    r.declare(Exclusion::Reconnecting, 30120);
    const auto partial = r.drain(30120);
    CHECK(partial.code == HandoffCode::Ready && partial.declarations == 0);
    CHECK(r.mirror.snapshot(30119).declared_exclusion == Exclusion::None);
    auto cut = r.drain(30121);
    CHECK(cut.declarations == 1 && cut.events[1] == 1);
    auto c = r.mirror.snapshot(cut.observed_ms);
    CHECK(c.excluded && c.declared_exclusion == Exclusion::Reconnecting);
    CHECK(c.infer_gap && c.infer_rate_low && c.historical_max_gap_ms == 120);
    CHECK(r.health.snapshot(cut.observed_ms).objects.open_gap_ms == 120);
    r.declare(Exclusion::Reconfiguring, 30200);
    r.declare(Exclusion::Reconfiguring, 30220); // Repeated declaration cannot reset duration.
    r.declare(Exclusion::None, 31000);
    cut = r.drain(31001); CHECK(cut.declarations == 3);
    c = r.mirror.snapshot(cut.observed_ms);
    const auto local = r.owner.snapshot(cut.observed_ms);
    CHECK(c.declared_excluded_ms == 880 && local.declared_excluded_ms == 880);
    CHECK(!c.excluded && c.declared_exclusion == Exclusion::None && c.awaiting_new_segment);
    CHECK(c.historical_max_gap_ms == 120 && local.historical_max_gap_ms == 120);
    CHECK(c.infer_gap && c.infer_rate_low); // Declaration never erases latched reasons.
    const auto before = r.objects.counters().attempted;
    r.clock.now = 31010;
    CHECK(r.objects.publish(body(29960)).error == PublishError::Duplicate);
    CHECK(r.drain(31011).events[1] == 0);
    CHECK(r.mirror.snapshot(31010).awaiting_new_segment);
    CHECK(r.health.snapshot(31010).objects.open_gap_ms == 1010);
    r.put(30980, 31020); cut = r.drain(31021);
    c = r.mirror.snapshot(cut.observed_ms);
    CHECK(!c.awaiting_new_segment && c.open_gap_ms == 0 && c.closed_gaps == 0);
    const auto w = r.health.snapshot(cut.observed_ms);
    CHECK(w.objects.total.attempted == 2 && w.objects.total.accepted == 2);
    CHECK(w.objects.gap_max_ms == 1020 && r.objects.counters().attempted == before + 1);
    ++groups;
}
void same_millisecond_fifo() {
    Rig before; before.declare(Exclusion::Reconnecting, 31000); before.put(30900, 31000);
    CHECK(before.drain(31001).events[1] == 3);
    auto b = before.mirror.snapshot(31000);
    CHECK(b.unique_results == 0 && b.excluded_results == 1);
    Rig after; after.put(30900, 31000); after.declare(Exclusion::Reconnecting, 31000);
    CHECK(after.drain(31001).events[1] == 3);
    auto a = after.mirror.snapshot(31000);
    CHECK(a.unique_results == 1 && a.excluded_results == 0);
    CHECK(before.health.snapshot(31000).objects.total.attempted == 1);
    CHECK(after.health.snapshot(31000).objects.total.attempted == 1);
    ++groups;
}
struct NestedClock : Clock {
    PublicationHealthHandoff* h = nullptr; Clock consumer;
    HandoffCode seen = HandoffCode::Invalid;
    Millis mono_millis() noexcept override {
        if (h) seen = h->drain(consumer).code;
        return now;
    }
};
struct ChangingClock : Clock {
    PublisherChannel* objects = nullptr; bool once = true;
    Millis mono_millis() noexcept override {
        if (once) { once = false; CHECK(objects->declare_exclusion(Exclusion::Reconnecting)); }
        return now;
    }
};
void declarations_participate_in_certificate() {
    Rig r; NestedClock producer; producer.now = 31000; producer.consumer.now = 31001; producer.h = &r.h;
    ObjectsCadence owner{0, 64};
    PublisherChannel objects{Topic::Objects, r.envelope, producer, r.sink, 4096, 8192, &owner, &r.o};
    CHECK(objects.declare_exclusion(Exclusion::Reconnecting));
    CHECK(producer.seen == HandoffCode::Busy && r.h.last_cut() == -1 && r.o.consumed() == 0);
    CHECK(r.health.snapshot(0).objects.total.attempted == 0);
    CHECK(r.drain(31001).declarations == 1);
    Rig changed; changed.clock.now = 31000;
    ChangingClock consumer; consumer.now = 31001; consumer.objects = &changed.objects;
    CHECK(changed.h.drain(consumer).code == HandoffCode::Busy);
    CHECK(changed.h.last_cut() == -1 && changed.o.consumed() == 0);
    CHECK(changed.health.snapshot(0).objects.total.attempted == 0);
    CHECK(changed.h.drain(consumer).declarations == 1);
    ++groups;
}
void invalid_declaration_times() {
    Rig late; CHECK(late.drain(31001).code == HandoffCode::Ready);
    // Deliberately violate producer clock discipline through the lower-level
    // journal: a prior request time may NOT be backdated into a published cut.
    CHECK(late.o.begin_declaration()); CHECK(late.o.record_declaration(Exclusion::Reconnecting, 31000));
    CHECK(late.drain(31002).code == HandoffCode::Invalid && late.o.consumed() == 0);
    CHECK(late.mirror.snapshot(31001).declared_exclusion == Exclusion::None);
    Rig future; CHECK(future.o.begin_declaration());
    CHECK(future.o.record_declaration(Exclusion::Reconfiguring, 31002));
    CHECK(future.drain(31001).code == HandoffCode::Invalid && future.o.consumed() == 0);
    CHECK(future.health.snapshot(31000).objects.total.attempted == 0);
    ++groups;
}
void pending_completion_then_declaration() {
    Rig r;
    PublishAudit a; a.attempted = true; a.seq = 1; a.capture_ms = 29960;
    a.put_before_ms = 30000; a.packet_bytes = 100;
    CHECK(r.o.reserve()); r.o.begin_attempt(); r.o.record_attempt(a);
    CHECK(r.drain(30501).code == HandoffCode::Ready);
    CHECK(r.health.snapshot(30500).objects.total.pending == 1);
    // Inject a long put's actual return, then apply a declaration in the SAME
    // millisecond. The completion consumes its reserved slot before control.
    a.put_return_ms = 31000; a.clock_valid = true; a.receipt = {PutOutcome::Accepted, 0};
    r.o.begin_completion(); r.o.record_completion(a);
    CHECK(r.o.begin_declaration()); CHECK(r.o.record_declaration(Exclusion::Reconnecting, 31000));
    const auto cut = r.drain(31001);
    CHECK(cut.code == HandoffCode::Ready && cut.events[1] == 2 && cut.declarations == 1);
    const auto w = r.health.snapshot(cut.observed_ms); const auto c = r.mirror.snapshot(cut.observed_ms);
    CHECK(w.objects.total.attempted == 1 && w.objects.total.pending == 0 && w.objects.total.accepted == 1);
    CHECK(w.objects.total.unknown == 0 && w.objects.latency_p99_ms == 40 && w.objects.open_gap_ms == 1000);
    CHECK(c.declared_exclusion == Exclusion::Reconnecting && c.historical_max_gap_ms == 1000);
    CHECK(c.infer_gap && c.infer_rate_low);
    ++groups;
}
void source_rate_identity_is_not_wire_identity() {
    Rig r;
    perception::native_depth::SourceId source{1, 1, 0, 0, 33333, 33333, 29960000000LL};
    CHECK(r.health.observe_inference(source, 29990, 29990) == HealthEventCode::Accepted);
    ++source.sequence; ++source.depth_index; ++source.color_index;
    source.depth_device_us += 33333; source.color_device_us += 33333;
    CHECK(r.health.observe_inference(source, 29991, 29991) == HealthEventCode::Accepted);
    r.put(29960, 30000);
    r.declare(Exclusion::Reconnecting, 30001); r.declare(Exclusion::None, 30002);
    r.clock.now = 30003; CHECK(r.objects.publish(body(29960)).error == PublishError::Duplicate);
    const auto cut = r.drain(30004); CHECK(cut.code == HandoffCode::Ready);
    const auto w = r.health.snapshot(cut.observed_ms);
    CHECK(w.inference.total_frames == 2 && w.objects.total.attempted == 1);
    CHECK(r.mirror.snapshot(cut.observed_ms).awaiting_new_segment && r.sink.calls == 1);
    ++groups;
}
void status_reasons_and_packets(bool emit) {
    Rig r; r.put(29960, 30000);
    HealthFieldContext context; context.extrinsic_calibrated = false; context.ground_seg_level = 0;
    const std::array<Exclusion, 4> states{Exclusion::None, Exclusion::Reconnecting,
        Exclusion::Reconfiguring, Exclusion::None};
    const std::array<const char*, 4> names{"ordinary", "reconnecting", "reconfiguring", "cleared"};
    for (std::size_t i = 0; i < states.size(); ++i) {
        const Millis time = 30000 + static_cast<Millis>(i) * 1000;
        if (i) r.declare(states[i], time);
        const auto cut = r.drain(time + 1); CHECK(cut.code == HandoffCode::Ready);
        const auto w = r.health.snapshot(time); const auto c = r.mirror.snapshot(time);
        const auto d = r.depth.snapshot(time);
        Status status; status.degraded_reasons = {"roi_fallback", "no_traversable_seg"};
        if (c.infer_gap || c.max_gap_ms > 50) status.degraded_reasons.emplace_back("infer_gap");
        if (c.infer_rate_low || c.max_gap_ms > 100 || c.over50_fraction > .01)
            status.degraded_reasons.emplace_back("infer_rate_low");
        if (states[i] != Exclusion::None) {
            status.fps_infer = 123; // Failure must leave the caller's DTO intact.
            CHECK(apply_health_fields(w, c, time, d, context, status) == HealthFieldsCode::MissingReason);
            CHECK(status.fps_infer == 123);
            status.degraded_reasons.emplace_back(states[i] == Exclusion::Reconnecting ? "reconfiguring" : "reconnecting");
            CHECK(apply_health_fields(w, c, time, d, context, status) == HealthFieldsCode::MissingReason);
            status.degraded_reasons.back() = names[i];
        }
        CHECK(apply_health_fields(w, c, time, d, context, status) == HealthFieldsCode::Ready);
        CHECK(status.infer_gap_ms_max == static_cast<double>(time - 30000));
        CHECK(status.latency_ms_p99 == 40 && status.fps_infer == 0 && !status.invalid_pixel_ratio);
        auto bad = c; bad.declared_exclusion = static_cast<Exclusion>(91);
        CHECK(apply_health_fields(w, bad, time, d, context, status) == HealthFieldsCode::MissingCadence);
        if (states[i] != Exclusion::None) {
            bad = c; bad.excluded = false;
            CHECK(apply_health_fields(w, bad, time, d, context, status) == HealthFieldsCode::MissingCadence);
        }
        status.degraded_reasons.emplace_back(states[i] == Exclusion::Reconnecting ? "reconfiguring" : "reconnecting");
        CHECK(apply_health_fields(w, c, time, d, context, status) == HealthFieldsCode::MissingReason);
        status.degraded_reasons.pop_back();
        const auto published = r.status.publish_status_snapshot(status, time, 10, time + 1);
        CHECK(published && published.audit.put_before_ms == time + 1);
        CHECK(r.health.snapshot(time).objects.total.attempted == 1);
        if (emit) std::cout << "{\"case\":\"" << names[i] << "\",\"key\":\"" << r.status.key()
            << "\",\"packet\":" << r.status_sink.packet.data() << ",\"cut_ms\":" << time
            << ",\"declarations_in_cut\":" << cut.declarations
            << ",\"objects_attempted\":" << w.objects.total.attempted
            << ",\"scope\":\"injected_clocks_no_network_no_camera\"}\n";
    }
    CHECK(r.status_sink.calls == 4 && r.sink.calls == 1);
    // Warmup alone must not demand or manufacture a reconnect reason.
    Rig warmup; CHECK(warmup.drain(1001).code == HandoffCode::Ready);
    Status s; s.degraded_reasons = {"roi_fallback", "no_traversable_seg"};
    const auto c = warmup.mirror.snapshot(1000); CHECK(c.excluded && c.declared_exclusion == Exclusion::None);
    CHECK(apply_health_fields(warmup.health.snapshot(1000), c, 1000,
        warmup.depth.snapshot(1000), context, s) == HealthFieldsCode::Ready);
    s.degraded_reasons.emplace_back("reconnecting");
    CHECK(apply_health_fields(warmup.health.snapshot(1000), c, 1000,
        warmup.depth.snapshot(1000), context, s) == HealthFieldsCode::MissingReason);
    ++groups;
}
void allocation_free() {
    Rig r;
    const auto before = allocations.load();
    for (std::size_t i = 0; i < 1000; ++i) {
        const Millis now = 31000 + static_cast<Millis>(i) * 11000;
        r.declare(Exclusion::Reconnecting, now);
        r.put(now - 20, now);
        r.declare(Exclusion::None, now + 1);
        r.put(now - 19, now + 1);
        const auto cut = r.drain(now + 2);
        CHECK(cut.code == HandoffCode::Ready && cut.declarations == 2 && cut.events[1] == 6);
        CHECK(r.health.snapshot(cut.observed_ms).objects.total.attempted == 2 * (i + 1));
        CHECK(r.mirror.snapshot(cut.observed_ms).valid);
    }
    CHECK(allocations.load() == before);
    ++groups;
}
}
int main(int argc, char** argv) {
    const bool emit = argc == 2 && std::string_view(argv[1]) == "--emit-fixtures";
    if (emit) { status_reasons_and_packets(true); return 0; }
    late_declaration_cannot_heal_history(); same_millisecond_fifo(); declarations_participate_in_certificate();
    invalid_declaration_times(); pending_completion_then_declaration(); source_rate_identity_is_not_wire_identity();
    status_reasons_and_packets(false); allocation_free();
    std::cout << "PASS declaration_handoff groups=" << groups << " checks=" << checks
        << " hot_cycles=1000 cpp_allocations=0 injected_only\n";
}
