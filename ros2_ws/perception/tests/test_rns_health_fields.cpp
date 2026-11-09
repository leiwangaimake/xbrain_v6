/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_health_fields.cpp
 * Brief: Unit tests for the Status health fields, from an injected native raster to the actual put.
 *
 * Description:
 * What it checks. A synthetic native depth raster and injected depth/inference/publication
 * observations feed the health windows; the computed fields are written into Status and sent
 * through a real PublisherChannel with an injected transport, so a field that is computed but
 * never serialised fails. It also covers rejected inputs and that the fields stay bound to the
 * source they were computed from, and that a completion which was never published does not count
 * as Objects rate (inference FPS is not objects Hz).
 * A19-NULL-3 (19 section 14). With no depth frame processed in the 1 s heartbeat window the packet
 * that reaches the transport carries "invalid_pixel_ratio":null (not the previous value, not 0), a
 * quantile without a sample is null in the same packet, and the 1 Hz beat goes on for 12 s without
 * depth, both after a frame was seen and when none was ever seen: the newest Status never gets
 * older than the consumer's 3 s timeout (T-53).
 * Scope. Injected clock and transport only; no device, network or calibration claim. The loop of
 * the Status thread in the service is not linked here; the last group repeats its steps by hand.
 */

#include "output/rns_health_fields.hpp"
#include "output/rns_status_heartbeat.hpp"
#include "rns_health_feed.hpp"
#include "strict_json.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdlib>
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
    PutOutcome outcome = PutOutcome::Accepted;
    int put_count = 0;
    explicit Sink(Clock& c) : clock(c) {}
    PutReceipt put(std::string_view, std::string_view p, PublishQos) override {
        assert(p.size() < packet.size());
        std::memcpy(packet.data(), p.data(), p.size()); packet[p.size()] = 0;
        // A19-ENC-1 on the Status wire text this route produces: strict JSON, no inf / nan token.
        assert(test_json::validate(p).ok && test_json::first_non_finite_token(p) == std::string_view::npos);
        clock.now += 3; ++put_count;
        return {outcome, 0};
    }
};
// A19-NULL-3 is judged on the bytes the consumer receives. These read the last packet the Sink
// kept, so a field that is null in the Status struct but encoded as a number still fails.
constexpr const char* kNullRatio = "\"invalid_pixel_ratio\":null";
bool wire_has(const Sink& sink, const char* text) { return std::strstr(sink.packet.data(), text) != nullptr; }
// Integer after a field name; the publisher pads its fixed-width stamp slot with spaces.
Millis wire_millis(const Sink& sink, const char* field) {
    const char* at = std::strstr(sink.packet.data(), field);
    assert(at);
    return std::strtoll(at + std::strlen(field), nullptr, 10);
}
struct Fixture {
    HealthWindow health{0, 512};
    ObjectsCadence cadence{0, 512};
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    Clock clock;
    Sink sink{clock};
    PublisherChannel objects{Topic::Objects, envelope, clock, sink, 4096, 8192, &cadence};
    PublisherChannel status_channel{Topic::Status, envelope, clock, sink, 4096, 8192};
    StatusHeartbeat heartbeat{status_channel, 39100, 10};
    nd::Mailbox mailbox{8}; nd::Frame frame{8};
    DepthHealthObservation depth;
    DepthHealthWindow depth_window{0, 64};
    HealthFieldContext context;
    Status status;
    // fed = false leaves every window empty: no depth frame, inference or Objects put was ever
    // observed (A19-NULL-3, the run before the first frame).
    explicit Fixture(bool fed = true) {
        if (!fed) return;
        for (int i = 0; i < 250; ++i) {
            const Millis t = 30100 + i * 40;
            assert(test_feed::depth(health, t - 30, t - 20, t - 20) == HealthEventCode::Accepted);
            assert(test_feed::inference(health, t - 30, t - 10, t - 10) == HealthEventCode::Accepted);
            clock.now = t;
            sink.outcome = i % 3 == 0 ? PutOutcome::Rejected :
                (i % 3 == 1 ? PutOutcome::Unknown : PutOutcome::Accepted);
            Objects o; o.t_capture_mono_ms = t - 30;
            const auto p = objects.publish(o);
            assert(p.audit.attempted && p.audit.clock_valid);
            assert(test_feed::publish(health, Topic::Objects, p.audit, clock.now) == HealthEventCode::Accepted);
        }
        // A completion that has NOT been published: FPS must not be objects Hz.
        assert(test_feed::depth(health, 40080, 40085, 40085) == HealthEventCode::Accepted);
        assert(test_feed::inference(health, 40080, 40090, 40090) == HealthEventCode::Accepted);
        const auto stream = mailbox.begin_stream();
        const std::array<std::uint16_t, 8> pixels{0, 0, 0, 0, 100, 0, 100, 100};
        nd::View v;
        v.metadata.source = {stream, 1, 1, 1, 1000, 1000, 40090000000LL};
        v.metadata.intrinsics = {1, 1, 0, 0, 2, 4}; v.metadata.unit_m = .001F;
        v.metadata.distortion.reported = true;
        v.pixels = pixels.data(); v.bytes = sizeof(pixels); v.pre_d2c = true;
        assert(mailbox.try_offer(v) == nd::Offer::Accepted);
        assert(mailbox.try_take(frame) == nd::Take::Taken);
        depth = observe_depth_health(frame, {});
        assert(depth.code == DepthHealthCode::Ready && depth.roi_pixels == 4 && depth.invalid_pixels == 1);
        context.extrinsic_calibrated = false;
        // Explicit injected existing-level input; no product enum invented.
        context.ground_seg_level = 2;
        context.expected_depth_source = frame.metadata().source;
        assert(depth_window.observe(depth, 40090, 40090) == DepthWindowCode::Accepted);
        status.degraded_reasons = {"roi_fallback", "no_traversable_seg"};
    }
};
void rejection_and_binding() {
    Fixture f;
    const auto w = f.health.snapshot(40100);
    const auto c = f.cadence.snapshot(40100);
    const auto depth = f.depth_window.snapshot(40100);
    assert(w.depth.fps == 25 && w.inference.fps == 25 && w.objects.window.attempted == 249);
    assert(w.objects.window.accepted + w.objects.window.rejected + w.objects.window.unknown == 249);
    assert(*w.objects.latency_p50_ms == 30 && *w.objects.latency_p99_ms == 30);
    assert(apply_health_fields(w, c, 40100, depth, f.context, f.status) == HealthFieldsCode::Ready);
    assert(f.status.fps_infer == 25 && f.status.latency_ms_p99 == 30 &&
        f.status.invalid_pixel_ratio == .25 && !f.status.traversable_seg_available);
    const auto before = f.status;
    auto bad_w = w; bad_w.objects.latency_p99_ms.reset();
    assert(apply_health_fields(bad_w, c, 40100, depth, f.context, f.status) == HealthFieldsCode::MissingLatency);
    bad_w = w; bad_w.depth.ever_observed = false;
    assert(apply_health_fields(bad_w, c, 40100, depth, f.context, f.status) == HealthFieldsCode::MissingRates);
    bad_w = w; bad_w.valid = false;
    assert(apply_health_fields(bad_w, c, 40100, depth, f.context, f.status) == HealthFieldsCode::InvalidWindow);
    auto bad_c = c; bad_c.excluded = true;
    assert(apply_health_fields(w, bad_c, 40100, depth, f.context, f.status) == HealthFieldsCode::Ready);
    bad_c.valid = false;
    assert(apply_health_fields(w, bad_c, 40100, depth, f.context, f.status) == HealthFieldsCode::MissingCadence);
    assert(apply_health_fields(w, c, 40099, depth, f.context, f.status) == HealthFieldsCode::MissingCadence);
    auto bad_d = depth; bad_d.invalid_ratio.reset();
    assert(apply_health_fields(w, c, 40100, bad_d, f.context, f.status) == HealthFieldsCode::InvalidDepth);
    auto x = f.context; ++x.expected_depth_source.depth_index;
    assert(apply_health_fields(w, c, 40100, depth, x, f.status) == HealthFieldsCode::InvalidDepth);
    bad_d = depth; --bad_d.now_ms;
    assert(apply_health_fields(w, c, 40100, bad_d, f.context, f.status) == HealthFieldsCode::InvalidDepth);
    x = f.context; x.extrinsic_calibrated = true;
    assert(apply_health_fields(w, c, 40100, depth, x, f.status) == HealthFieldsCode::MissingContext);
    x = f.context; x.ground_seg_level.reset();
    assert(apply_health_fields(w, c, 40100, depth, x, f.status) == HealthFieldsCode::MissingContext);
    x = f.context; x.last_valid_seg_update_ms = 40101;
    assert(apply_health_fields(w, c, 40100, depth, x, f.status) == HealthFieldsCode::InvalidSegmentationTime);
    assert(f.status.fps_infer == before.fps_infer && f.status.invalid_pixel_ratio == before.invalid_pixel_ratio &&
           f.status.degraded_reasons == before.degraded_reasons);
    f.status.degraded_reasons.clear();
    assert(apply_health_fields(w, c, 40100, depth, f.context, f.status) == HealthFieldsCode::MissingReason);
    f.status.degraded_reasons = {"roi_fallback"};
    x = f.context; x.last_valid_seg_update_ms = 30101;
    assert(apply_health_fields(w, c, 40100, depth, x, f.status) == HealthFieldsCode::Ready);
    assert(f.status.traversable_seg_available);
    x.last_valid_seg_update_ms = 30100;
    assert(apply_health_fields(w, c, 40100, depth, x, f.status) == HealthFieldsCode::MissingReason);
}
// A19-NULL-3 injection 1: depth was flowing (one frame at 40090 ms, ratio .25), then nothing.
void packets(bool emit) {
    Fixture f;
    for (const auto now : {40100, 41100}) {
        const auto w = f.health.snapshot(now);
        const auto c = f.cadence.snapshot(now);
        const auto depth = f.depth_window.snapshot(now);
        if (now == 41100) {
            assert(apply_health_fields(w, c, now, depth, f.context, f.status) == HealthFieldsCode::MissingReason);
            // External reason owner supplies the observed fault, not a faster
            // receiver substitution or a fabricated zero-gap health sample.
            f.status.degraded_reasons = {"roi_fallback", "no_traversable_seg", "infer_gap", "infer_rate_low"};
        }
        assert(apply_health_fields(w, c, now, depth, f.context, f.status) == HealthFieldsCode::Ready);
        assert(f.status.invalid_pixel_ratio.has_value() == (now == 40100));
        f.clock.now = now; f.sink.outcome = PutOutcome::Accepted;
        const auto r = f.heartbeat.poll(now, &f.status, w.now_ms);
        assert(r.code == HeartbeatCode::Published && r.publication.audit.put_return_ms == now + 3);
        assert(f.status.t_publish_mono_ms == -1);
        // A19-NULL-3: the packet of the beat without a depth frame says null, the one with a frame does not.
        assert(wire_has(f.sink, kNullRatio) == (now == 41100));
        if (emit) std::cout << "{\"case\":\"" << (now == 40100 ? "computed_active" : "computed_silent_open_gap")
            << "\",\"packet\":" << f.sink.packet.data()
            << ",\"depth_count\":" << w.depth.window_frames << ",\"infer_count\":" << w.inference.window_frames
            << ",\"objects_attempted\":" << w.objects.window.attempted
            << ",\"accepted\":" << w.objects.window.accepted << ",\"rejected\":" << w.objects.window.rejected
            << ",\"unknown\":" << w.objects.window.unknown << ",\"roi\":" << depth.roi_pixels
            << ",\"invalid\":" << depth.invalid_pixels << ",\"depth_sample_ms\":40090"
            << ",\"open_gap\":" << c.open_gap_ms << ",\"before\":" << now << ",\"after\":" << now + 3 << "}\n";
    }
    const auto w = f.health.snapshot(42100); const auto c = f.cadence.snapshot(42100);
    const auto depth = f.depth_window.snapshot(42100);
    assert(apply_health_fields(w, c, 42100, depth, f.context, f.status) == HealthFieldsCode::Ready);
    assert(!f.status.invalid_pixel_ratio);
    f.clock.now = 42100;
    assert(f.heartbeat.poll(42100, &f.status, 42100).code == HeartbeatCode::Published);
    // A19-NULL-3: still null one beat later (the .25 of 40090 ms never comes back).
    assert(wire_millis(f.sink, "\"t_publish_mono_ms\":") == 42100 && wire_has(f.sink, kNullRatio));
}
// A19-NULL-3 injection 2: startup with nothing observed, one all-invalid frame, then no depth again.
void r6_packets(bool emit) {
    HealthWindow health{0, 64}; DepthHealthWindow depth_window{0, 64};
    ObjectsCadence cadence{0, 64}; Clock clock; Sink sink{clock};
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublisherChannel objects{Topic::Objects, envelope, clock, sink, 4096, 8192, &cadence};
    PublisherChannel status_channel{Topic::Status, envelope, clock, sink, 4096, 8192};
    StatusHeartbeat heartbeat{status_channel, 0, 10};
    Status status; status.degraded_reasons = {"roi_fallback", "no_traversable_seg"};
    HealthFieldContext context; context.extrinsic_calibrated = false; context.ground_seg_level = 0;
    nd::Mailbox mailbox{8}; nd::Frame frame{8};
    constexpr const char* names[] = {"status_startup_unknown", "status_observed_all_invalid",
        "status_depth_window_empty", "status_empty_latency_open_gap"};
    const std::array<Millis, 4> times{1000, 2000, 3000, 13000};
    for (std::size_t i = 0; i < times.size(); ++i) {
        const auto now = times[i];
        if (i == 1) {
            std::array<std::uint16_t, 8> pixels{}; nd::View v;
            v.metadata.source = {mailbox.begin_stream(), 1, 1, 1, 100, 100, 1900000000};
            v.metadata.intrinsics = {1, 1, 0, 0, 2, 4}; v.metadata.distortion.reported = true;
            v.metadata.unit_m = .001F; v.pixels = pixels.data(); v.bytes = sizeof(pixels); v.pre_d2c = true;
            assert(mailbox.try_offer(v) == nd::Offer::Accepted && mailbox.try_take(frame) == nd::Take::Taken);
            const auto observation = observe_depth_health(frame, {});
            assert(observation.invalid_ratio == 1 && observation.roi_pixels == 4);
            context.expected_depth_source = observation.source;
            assert(depth_window.observe(observation, 1900, 1900) == DepthWindowCode::Accepted);
            assert(test_feed::depth(health, 1880, 1900, 1900) == HealthEventCode::Accepted);
            assert(test_feed::inference(health, 1900, 1940, 1940) == HealthEventCode::Accepted);
            clock.now = 1950; Objects o; o.t_capture_mono_ms = 1900;
            const auto p = objects.publish(o); assert(p && p.audit.attempted);
            assert(test_feed::publish(health, Topic::Objects, p.audit, clock.now) == HealthEventCode::Accepted);
        }
        const auto w = health.snapshot(now); const auto d = depth_window.snapshot(now);
        const auto c = cadence.snapshot(now);
        assert(c.excluded); // warmup is NOT a reason to hide measured health or suppress status
        assert(apply_health_fields(w, c, now, d, context, status) == HealthFieldsCode::Ready);
        assert(status.invalid_pixel_ratio.has_value() == (i == 1));
        if (i == 1) assert(status.invalid_pixel_ratio == 1);
        if (i == 0) assert(!status.latency_ms_p50 && !status.latency_ms_p99 &&
            !status.infer_gap_ms_p99 && !status.infer_gap_ms_max && status.fps_depth == 0);
        if (i == 2) assert(status.latency_ms_p50 == 50 && status.infer_gap_ms_max == 1050);
        if (i == 3) assert(!status.latency_ms_p50 && !status.latency_ms_p99 &&
            status.infer_gap_ms_max == 11050 && status.infer_gap_ms_p99 == 11050);
        clock.now = now;
        const auto result = heartbeat.poll(now, &status, now);
        assert(result.code == HeartbeatCode::Published && result.publication.audit.attempted);
        // A19-NULL-3 on the wire. Ratio: null without a depth frame in the last 1 s (i = 0, 2, 3),
        // the measured 1 at i = 1. Latency: null without an Objects put in the last 10 s (i = 0, 3).
        // Gap: null only before the first Objects put (i = 0); afterwards the open gap is a number.
        assert(wire_has(sink, kNullRatio) == (i != 1));
        assert(wire_has(sink, "\"latency_ms_p50\":null") == (i == 0 || i == 3) &&
            wire_has(sink, "\"latency_ms_p99\":null") == (i == 0 || i == 3));
        assert(wire_has(sink, "\"infer_gap_ms_p99\":null") == (i == 0) &&
            wire_has(sink, "\"infer_gap_ms_max\":null") == (i == 0));
        if (emit) {
            std::cout << "{\"case\":\"" << names[i] << "\",\"key\":\"" << status_channel.key()
                << "\",\"packet\":" << sink.packet.data() << ",\"observed_ms\":" << now
                << ",\"processed_frames\":" << d.processed_frames << ",\"roi\":" << d.roi_pixels
                << ",\"invalid\":" << d.invalid_pixels << ",\"objects_attempted\":" << w.objects.window.attempted
                << ",\"closed_gap_samples\":" << w.objects.closed_gap_samples
                << ",\"before\":" << result.publication.audit.put_before_ms
                << ",\"after\":" << result.publication.audit.put_return_ms << "}\n";
        }
    }
    assert(heartbeat.counters().put_attempts == 4);
}
// A19-NULL-3, heartbeat half ("the beat stops => T-53 red"). T-53 is the consumer's Status timeout
// of 3 s (11 section 1.6, indexed in 3.1B.5): at every control tick it takes
// age = now - t_publish_mono_ms of the newest Status it accepted (a strictly newer stamp), so a
// beat that stops is seen without waiting for the next one. Two injections, the two ways a
// heartbeat window comes to hold no processed depth frame (11 section 3.1B.3):
// depth_seen: the stream is cut. The last processed depth frame is at 40090 ms and none follows
// for 12 s. That is longer than the 10 s statistics window, so the latency quantiles lose their
// last sample on the way and the beat has to survive that too.
// !depth_seen: nothing observed yet. No depth frame, inference or Objects put ever, same 13 beats;
// the ratio and all four quantiles are null in every packet.
// The loop repeats the steps of the Status thread every 10 ms (snapshots at one instant, reasons
// from the cadence gates, no snapshot when the fields are not ready) and plays that consumer on
// the bytes the transport received.
void heartbeat_without_depth(bool depth_seen) {
    constexpr Millis kLastDepthMs = 40090, kLastObjectsPutMs = 40060, kFirstBeatMs = 40100;
    // Contract numbers, on purpose not the constants of the code under test: 1 Hz beat, 1 s
    // heartbeat window of the ratio, 10 s statistics window (19 section 5 and 5.1), T-53 = 3 s.
    constexpr Millis kPeriodMs = 1000, kRatioWindowMs = 1000, kStatisticsWindowMs = 10000;
    constexpr Millis kTickMs = 10, kStatusTimeoutMs = 3000;
    constexpr int kSilentBeats = 12;
    constexpr Millis kEndMs = kFirstBeatMs + kSilentBeats * kPeriodMs;
    Fixture f{depth_seen};
    // The fixture's context verdicts; before the first frame there is no depth source to expect.
    f.context.extrinsic_calibrated = false; f.context.ground_seg_level = 2;
    f.sink.outcome = PutOutcome::Accepted;
    Millis newest_ms = -1, worst_age_ms = 0;
    int beats = 0, null_beats = 0;
    for (Millis now = kFirstBeatMs; now <= kEndMs; now += kTickMs) {
        // The consumer's tick: the age it sees before this tick's packet (if any) arrives.
        if (newest_ms >= 0) worst_age_ms = std::max(worst_age_ms, now - newest_ms);
        const auto w = f.health.snapshot(now);
        const auto c = f.cadence.snapshot(now);
        const auto depth = f.depth_window.snapshot(now);
        f.status.degraded_reasons = {"roi_fallback", "no_traversable_seg"};
        if (c.infer_gap || c.max_gap_ms > 50) f.status.degraded_reasons.emplace_back("infer_gap");
        if (c.infer_rate_low || c.max_gap_ms > 100 || c.over50_fraction > .01)
            f.status.degraded_reasons.emplace_back("infer_rate_low");
        const bool ready = apply_health_fields(w, c, now, depth, f.context, f.status) == HealthFieldsCode::Ready;
        f.clock.now = now;
        const int sent = f.sink.put_count;
        const auto r = f.heartbeat.poll(now, ready ? &f.status : nullptr, now);
        if (f.sink.put_count == sent) continue;
        // One Status packet reached the transport: stamped at the put, newer than the last one,
        // and on the 1 s grid (beat k exactly at first + k * 1000 ms, none late, none skipped).
        const Millis stamp = wire_millis(f.sink, "\"t_publish_mono_ms\":");
        assert(r.code == HeartbeatCode::Published && stamp == r.publication.audit.put_before_ms);
        assert(stamp > newest_ms && stamp == kFirstBeatMs + beats * kPeriodMs);
        // Ratio on the wire: null exactly when the 1 s window holds no processed depth frame.
        // After the cut that is every beat but the first (not the .25 of 40090 ms kept, not 0,
        // not 1); before the first frame it is every beat.
        const bool no_depth = !depth_seen || now - kLastDepthMs >= kRatioWindowMs;
        assert(wire_has(f.sink, kNullRatio) == no_depth);
        // Latency: null exactly when the 10 s window holds no Objects put (the last one, if any,
        // was at 40060 ms: Objects stopped with the depth).
        const bool no_latency = !depth_seen || now - kLastObjectsPutMs >= kStatisticsWindowMs;
        assert(wire_has(f.sink, "\"latency_ms_p50\":null") == no_latency &&
            wire_has(f.sink, "\"latency_ms_p99\":null") == no_latency);
        // Gap: null only while no Objects was ever put. After a put the open gap now - last put
        // goes on counting into the maximum even with an empty window (19 section 5.1: the
        // outage is not hidden); the closed gaps of this fixture are 40 ms, so it is the maximum.
        assert(wire_has(f.sink, "\"infer_gap_ms_p99\":null") == !depth_seen &&
            wire_has(f.sink, "\"infer_gap_ms_max\":null") == !depth_seen);
        if (depth_seen) assert(wire_millis(f.sink, "\"infer_gap_ms_max\":") == now - kLastObjectsPutMs);
        newest_ms = stamp; ++beats; null_beats += no_depth;
    }
    // T-53: a Status arrived, and at no tick of the 12 s without depth was the newest one older than 3 s.
    assert(beats > 0 && worst_age_ms <= kStatusTimeoutMs);
    // Stronger than the timeout: every 1 s slot got its beat (largest age = one period), and every
    // beat without a depth frame in its window (12 after the cut, all 13 before the first frame)
    // carried the null ratio.
    assert(beats == kSilentBeats + 1 && worst_age_ms == kPeriodMs);
    assert(null_beats == (depth_seen ? kSilentBeats : kSilentBeats + 1));
    const auto counters = f.heartbeat.counters();
    assert(counters.put_attempts == static_cast<std::uint64_t>(beats) && counters.accepted == counters.put_attempts &&
        counters.missed_deadlines == 0 && counters.snapshot_rejections == 0);
}
}
int main(int argc, char** argv) {
    const bool emit = argc == 2 && std::string_view(argv[1]) == "--emit-fixtures";
    const bool r6_emit = argc == 2 && std::string_view(argv[1]) == "--emit-r6-status-fixtures";
    if (r6_emit) { r6_packets(true); return 0; }
    rejection_and_binding(); packets(emit);
    r6_packets(false);
    heartbeat_without_depth(true); heartbeat_without_depth(false);
    if (!emit) std::cout << "PASS health_fields native_ROI source_binding availability segmentation_boundary"
        " computed_window_to_heartbeat A19-NULL-3_no_depth_null_on_wire A19-NULL-3_heartbeat_without_depth_T-53"
        " A19-NULL-3_heartbeat_before_first_depth_T-53\n";
}
