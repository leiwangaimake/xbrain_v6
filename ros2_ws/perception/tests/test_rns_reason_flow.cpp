/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_reason_flow.cpp
 * Brief: Host-only flow: actual injected puts to cadence reasons to the journal and Status.
 *
 * Description:
 * What it checks. Real puts through an injected transport drive the cadence ledger; the resulting
 * degraded reasons pass through the reason edges into the journal and appear in Status, end to
 * end, without a network.
 */

#include "output/rns_health_fields.hpp"
#include "output/rns_reason_edges.hpp"
#include "output/rns_status_heartbeat.hpp"
#include "rns_health_feed.hpp"
#include "strict_json.hpp"
#include <array>
#include <cassert>
#include <cstring>
#include <iostream>
using namespace perception::rns;
namespace nd = perception::native_depth;
namespace {
struct Clock final : PublishClock {
    Millis now = 0;
    Millis wall_millis() noexcept override { return 1700000000000; }
    Millis mono_millis() noexcept override { return now; }
};
struct Sink final : RnsTransport {
    Clock& clock;
    std::array<char, 8192> packet{};
    explicit Sink(Clock& c) : clock(c) {}
    PutReceipt put(std::string_view, std::string_view p, PublishQos) override {
        assert(p.size() < packet.size());
        std::memcpy(packet.data(), p.data(), p.size()); packet[p.size()] = 0;
        // A19-ENC-1 on the event wire text this route produces: strict JSON, no inf / nan token.
        assert(test_json::validate(p).ok && test_json::first_non_finite_token(p) == std::string_view::npos);
        clock.now += 3;
        return {PutOutcome::Accepted, 0};
    }
};
std::uint64_t bits(const CadenceSnapshot& c) {
    return (c.infer_gap ? 1U : 0U) | (c.infer_rate_low ? 2U : 0U);
}
struct Flow {
    ObjectsCadence cadence{0, 1024};
    HealthWindow health{0, 1024};
    ReasonEdges edges{0, 2, 8}; // local bits0/1: gap/rate, not a GEN schema
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    Clock clock;
    Sink sink{clock};
    PublisherChannel objects{Topic::Objects, envelope, clock, sink, 4096, 8192, &cadence};
    PublisherChannel status_channel{Topic::Status, envelope, clock, sink, 4096, 8192};
    StatusHeartbeat heartbeat{status_channel, 29123, 10};
    nd::Mailbox mailbox{8}; nd::Frame frame{8};
    DepthHealthWindow depth_window{0, 64};
    std::uint64_t stream = mailbox.begin_stream(), serial = 0;
    std::array<Status, 4> prepared;
    explicit Flow() {
        for (std::size_t i = 0; i < prepared.size(); ++i) {
            prepared[i].degraded_reasons = {"roi_fallback", "no_traversable_seg"};
            if (i & 1) prepared[i].degraded_reasons.emplace_back("infer_gap");
            if (i & 2) prepared[i].degraded_reasons.emplace_back("infer_rate_low");
        }
    }
    void put(Millis t, bool emit_status, bool fixtures) {
        assert(test_feed::depth(health, t - 20, t - 15, t - 15) == HealthEventCode::Accepted);
        assert(test_feed::inference(health, t - 20, t - 5, t - 5) == HealthEventCode::Accepted);
        clock.now = t;
        Objects body; body.t_capture_mono_ms = t - 20;
        const auto p = objects.publish(body);
        assert(p && p.audit.attempted && p.audit.clock_valid);
        assert(test_feed::publish(health, Topic::Objects, p.audit, clock.now) == HealthEventCode::Accepted);
        const auto c = cadence.snapshot(clock.now);
        assert(c.valid && c.observed_ms == clock.now);
        const auto e = edges.update(bits(c), clock.now);
        assert(e.code != ReasonEdgeCode::Invalid && e.code != ReasonEdgeCode::Overflow);
        if (!emit_status) return;
        const auto now = clock.now;
        const std::array<std::uint16_t, 8> pixels{0, 0, 0, 0, 0, 100, 100, 100};
        ++serial;
        nd::View v;
        v.metadata.source = {stream, serial, serial, serial, serial * 1000, serial * 1000, (t - 15) * 1000000};
        v.metadata.intrinsics = {1, 1, 0, 0, 2, 4};
        v.metadata.distortion.reported = true; v.metadata.unit_m = .001F;
        v.pixels = pixels.data(); v.bytes = sizeof(pixels); v.pre_d2c = true;
        assert(mailbox.try_offer(v) == nd::Offer::Accepted && mailbox.try_take(frame) == nd::Take::Taken);
        const auto depth = observe_depth_health(frame, {});
        HealthFieldContext x;
        x.extrinsic_calibrated = false; x.ground_seg_level = 2;
        x.expected_depth_source = frame.metadata().source;
        assert(depth_window.observe(depth, now, now) == DepthWindowCode::Accepted);
        const auto depth_report = depth_window.snapshot(now);
        auto& status = prepared[bits(c)];
        const auto w = health.snapshot(now);
        assert(apply_health_fields(w, c, now, depth_report, x, status) == HealthFieldsCode::Ready);
        const auto r = heartbeat.poll(now, &status, now);
        assert(r.code == HeartbeatCode::Published);
        const auto state = t == 30120 ? "fault" : (t == 39120 ? "still_fault" : "recovered");
        if (t == 40120) assert(!c.infer_gap && !c.infer_rate_low && c.max_gap_ms == 40);
        else assert(c.infer_gap && c.infer_rate_low && c.max_gap_ms == 120);
        if (fixtures) std::cout << "{\"case\":\"" << state << "\",\"packet\":" << sink.packet.data()
            << ",\"before\":" << now << ",\"after\":" << clock.now
            << ",\"reason_bits\":" << bits(c) << ",\"observed_ms\":" << c.observed_ms
            << ",\"source_capture_ms\":" << body.t_capture_mono_ms
            << ",\"historical_max_gap_ms\":" << c.historical_max_gap_ms
            << ",\"queued_edges\":" << edges.queued() << "}\n";
    }
};
void run(bool fixtures) {
    Flow f;
    f.put(30000, false, fixtures);
    f.put(30120, true, fixtures);
    for (int i = 1; i <= 250; ++i) {
        const Millis now = 30120 + i * 40;
        f.put(now, now == 39120 || now == 40120, fixtures);
    }
    assert(f.edges.valid() && f.edges.counters().enqueued_edges == 2 && f.edges.queued() == 2);
    ReasonEdge first, second, none;
    assert(f.edges.pop(first) && f.edges.pop(second) && !f.edges.pop(none));
    assert(first.severity == ReasonEdgeSeverity::Warn && first.raw_added == 3 && first.eligible_added == 3);
    assert(second.severity == ReasonEdgeSeverity::Info && second.raw_removed == 3 && second.eligible_removed == 3);
    assert(first.observed_ms == 30123 && second.observed_ms == 40123);
    assert(first.active_after == 3 && second.active_after == 0);
    assert(f.heartbeat.counters().put_attempts == 3);
    if (fixtures) {
        std::cout << "{\"case\":\"local_edges\",\"warn_at\":" << first.observed_ms
            << ",\"info_at\":" << second.observed_ms << ",\"warn_added\":" << first.eligible_added
            << ",\"info_removed\":" << second.eligible_removed
            << ",\"enqueued\":2,\"GEN_puts\":0,\"remote_received\":null}\n";
    } else std::cout << "PASS actual_injected_252_objects_puts reason_edges_2 status_puts_3 A19_recovery_10s_not_20s NO_GEN_or_device\n";
}
}
int main(int argc, char** argv) { run(argc == 2 && std::string_view(argv[1]) == "--emit-fixtures"); }
