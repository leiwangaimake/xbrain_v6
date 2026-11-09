/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_profile_thread.hpp
 * Brief: The Profile thread's owners on a host, for tests that run the service's real Profile loop.
 *
 * Description:
 * What it provides. ProfileOwners builds what the service's first-frame setup builds for the
 * steady loop (service/profile_steady_loop.hpp), over the synthetic camera of
 * test_rns_native_profile.cpp (128 px focal length, a flat floor 1 m below the camera): the
 * geometry, the grid, a T owner on the rectified reference colour projection, an S projector, the
 * T and S slots, the T ready flag, the reason state, the counters, the timing handoff, and a
 * Profile channel whose transport records every put. The clocks are real: the capture clock and
 * the publish clock both read CLOCK_MONOTONIC, as in the service. offer() hands one frame of the
 * floor to the mailbox with host ready time = now and capture time = now minus a lag;
 * light_floor() keeps 1/16 of its pixels, for machines where a full-floor step is too slow.
 * RecordingTransport keeps, per put, the put bracket, t_capture_mono_ms, t_seg_mono_ms (-1 for
 * null) and the number of bins whose src has bit 0 (the T backing), in a fixed array: a put
 * allocates nothing, so a test may count the Profile thread's heap calls with it in place.
 * What it is not. No camera, network, Zenoh, model or calibration. The T margin is zero (test
 * branch, notes/DECISIONS.md question 10), so a fresh mask does back FREE bins with bit 0.
 */
#pragma once

#include "output/rns_capture_estimate.hpp"
#include "output/rns_native_put_audit.hpp"
#include "output/rns_system_clock.hpp"
#include "service/profile_steady_loop.hpp"
#include "service/profile_timing_handoff.hpp"
#include "service/semantic_slot.hpp"
#include "service/t_mask_slot.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <time.h>
#include <vector>

namespace perception::test_profile {

namespace geo = perception::geometry;
namespace nd = perception::native_depth;
namespace rns = perception::rns;
namespace svc = perception::service;

// CLOCK_MONOTONIC in ns, the service's clock; -1 on error.
inline std::int64_t mono_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
inline std::int64_t mono_now(const void*) noexcept { return mono_ns(); }

// The synthetic camera and geometry options of test_rns_native_profile.cpp.
inline geo::ProjectionBinding binding() {
    geo::ProjectionBinding b;
    b.intrinsics = {128, 128, 320, -128, 640, 400};
    b.distortion.reported = true;
    b.distortion.model = 0;
    b.unit_m = 1.0F / 1024;
    b.R_base_camera = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    b.t_base_camera = {0, 0, 1};
    return b;
}
inline geo::GeometryOptions options() {
    geo::GeometryOptions o;
    o.prior = {{0, 0, 1}, 0};
    o.fit.inlier_distance_m = .015; o.fit.min_inlier_fraction = .8;
    o.fit.max_inlier_rms_m = .005; o.fit.min_xy_stddev_m = .01;
    o.fit.min_up_dot = .5; o.fit.max_prior_angle_rad = .3;
    o.fit.min_inliers = 100; o.fit.hypotheses = 32; o.fit.seed = 0x243f6a8885a308d3ULL;
    o.fit_near_min_m = .7; o.fit_near_max_m = 2.9; o.fit_prior_band_m = .25;
    o.range_max_m = 6; o.blind_near_lo_m = .01; o.h_tol_m = .125;
    o.z_pass_m = .5; o.eps_neg = .1; o.w_h_m = .1;
    o.angle_min_rad = -155.0 / 128; o.angle_step_rad = 1.0 / 128; o.n_bins = 311;
    return o;
}
inline std::vector<geo::Vec3> rays(const geo::ProjectionBinding& b) {
    std::vector<geo::Vec3> r(geo::kGeometryPixels);
    for (std::size_t y = 0; y < geo::kNativeHeight; ++y)
        for (std::size_t x = 0; x < geo::kNativeWidth; ++x)
            r[y * geo::kNativeWidth + x] = {(x - b.intrinsics.cx) / b.intrinsics.fx,
                                            (y - b.intrinsics.cy) / b.intrinsics.fy, 1};
    return r;
}

// One put as the wire text shows it. capture_ms / seg_ms: -1 for null, -2 when not found.
struct PutRecord {
    std::int64_t put_begin_ns = -1, put_end_ns = -1;
    long long capture_ms = -2, seg_ms = -2;
    int bit0_bins = 0;
};

// The integer after `key` in `text`: -1 for null, -2 when the key or a number is missing.
inline long long wire_integer(std::string_view text, std::string_view key) noexcept {
    const auto at = text.find(key);
    if (at == std::string_view::npos) return -2;
    std::size_t i = at + key.size();
    if (text.substr(i, 4) == "null") return -1;
    long long value = 0;
    bool any = false;
    for (; i < text.size() && text[i] >= '0' && text[i] <= '9'; ++i) {
        value = value * 10 + (text[i] - '0');
        any = true;
    }
    return any ? value : -2;
}
// How many entries of the "src" array have bit 0 set.
inline int wire_bit0_bins(std::string_view text) noexcept {
    constexpr std::string_view key = "\"src\":[";
    auto i = text.find(key);
    if (i == std::string_view::npos) return -1;
    int bins = 0, value = 0;
    for (i += key.size(); i < text.size() && text[i] != ']'; ++i) {
        if (text[i] == ',') { bins += value & 1; value = 0; }
        else if (text[i] >= '0' && text[i] <= '9') value = value * 10 + (text[i] - '0');
    }
    return bins + (value & 1);
}

// The Profile publisher's transport. Records each put (fixed array; the overflow is counted) and
// fills a native put audit with the put bracket, as ZenohRtTransport does. Only the publishing
// thread writes; size() may be read by another thread (acquire), records only after a join.
class RecordingTransport final : public rns::RnsTransport {
public:
    static constexpr std::size_t kCapacity = 8192;
    rns::PutReceipt put(std::string_view, std::string_view bytes, rns::PublishQos) noexcept override {
        PutRecord r;
        r.put_begin_ns = mono_ns();
        r.capture_ms = wire_integer(bytes, "\"t_capture_mono_ms\":");
        r.seg_ms = wire_integer(bytes, "\"t_seg_mono_ms\":");
        r.bit0_bins = wire_bit0_bins(bytes);
        r.put_end_ns = mono_ns();
        audit_ = {};
        audit_.bytes = bytes.size();
        audit_.native_called = true;
        audit_.clock_valid = r.put_begin_ns > 0 && r.put_end_ns >= r.put_begin_ns;
        audit_.put_begin_ns = r.put_begin_ns;
        audit_.put_end_ns = r.put_end_ns;
        const auto n = size_.load(std::memory_order_relaxed);
        if (n < records_.size()) {
            records_[n] = r;
            size_.store(n + 1, std::memory_order_release);
        } else {
            ++overflow_;
        }
        return {rns::PutOutcome::Accepted, 0};
    }
    std::size_t size() const noexcept { return size_.load(std::memory_order_acquire); }
    std::size_t overflow() const noexcept { return overflow_; }
    const PutRecord& operator[](std::size_t i) const noexcept { return records_[i]; }
    // The PutAuditReader of the steady loop (service/profile_steady_loop.hpp).
    static rns::NativePutAudit read_audit(const void* self) noexcept {
        return static_cast<const RecordingTransport*>(self)->audit_;
    }

private:
    std::array<PutRecord, kCapacity> records_{};
    std::atomic<std::size_t> size_{0};
    std::size_t overflow_ = 0;
    rns::NativePutAudit audit_;
};

// The two reason bits the steady loop maps the tick's reason names to (any two distinct bits).
inline constexpr std::uint64_t kGroundFitFallback = std::uint64_t{1} << 4;
inline constexpr std::uint64_t kGroundFreeWithdrawn = std::uint64_t{1} << 5;

// Everything run_profile_steady() needs, over the flat floor; see the file header.
struct ProfileOwners {
    std::int64_t t_max_age_ns;
    geo::ProjectionBinding b = binding();
    std::vector<geo::Vec3> optical = rays(b);
    geo::NativeGeometry geometry;
    geo::NormalGrid grid;
    geo::RectifiedColor color;
    geo::ColorProjection projection;
    geo::NormalTraversability t;
    rns::SemanticOptions semantic_options;
    rns::SemanticProjector semantic;
    nd::Mailbox mailbox{geo::kGeometryPixels};
    svc::TMaskSlot t_slot;
    svc::SemanticSlot s_slot;
    std::atomic<bool> t_ready{false};
    svc::ProfileReasonState reasons{kGroundFitFallback | kGroundFreeWithdrawn};
    svc::ProfileCounters counters;
    // Nobody takes from it unless a test does: at stop the loop logs what waits, then when=stop.
    svc::ProfileTimingHandoff timing_handoff;
    rns::SystemPublishClock publish_clock;
    rns::ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    RecordingTransport transport;
    rns::PublisherChannel channel{rns::Topic::Profile, envelope, publish_clock, transport, 131072, 262144};
    int clock_context = 0;
    rns::EstimatedCaptureClock capture_clock{{1, "profile thread host test"}, &clock_context, 1, mono_now};
    std::vector<std::uint16_t> raster = std::vector<std::uint16_t>(geo::kGeometryPixels);
    std::vector<std::uint8_t> mask = std::vector<std::uint8_t>(std::size_t{1280} * 800, 1);
    std::uint64_t stream = 0, sequence = 0;

    // max_age_ns: seg_max_age_ms; stale_ns / guard_ns: objects_stale_ms and the publish guard.
    ProfileOwners(std::int64_t max_age_ns, std::int64_t stale_ns, std::int64_t guard_ns)
        : t_max_age_ns(max_age_ns),
          geometry(b, optical.data(), optical.size(), options()),
          grid(geometry, {.25, .9, .4, .02}),
          color([] {
              geo::RectifiedColor c;
              c.R_color_base = {0, -1, 0, 0, 0, -1, 1, 0, 0};
              c.t_color_base = {.125, 1.25, .25};
              c.fx = 200; c.fy = 160; c.cx = 640; c.cy = 100;
              return c;
          }()),
          projection{1280, 800, svc::kTProjectionRevision, &color, geo::project_rectified_color},
          t(geometry, grid, projection, geo::TOptions{.8, 0, 0, 0, max_age_ns}),
          semantic_options{0, 0, 0, stale_ns, svc::kSemanticBaseRevision, guard_ns},
          semantic(geometry, semantic_options, rns::SemanticDepthPolicy::NormalOrFitFailed) {
        for (std::size_t i = 0; i < raster.size(); ++i)
            raster[i] = static_cast<std::uint16_t>(std::llround(1.0 / optical[i].y / b.unit_m));
        stream = mailbox.begin_stream();
    }
    ProfileOwners(const ProfileOwners&) = delete;
    ProfileOwners& operator=(const ProfileOwners&) = delete;

    // A lighter floor for the real-time tests: only the lower half of the image (rows 200-399, the
    // nearer floor) and every 8th column keep their depth, the rest reads 0 (no measurement), so
    // 1/16 of the pixels. The fit stays normal, the grid has ground evidence, a fresh mask still
    // backs FREE bins with bit 0 and S runs as before, at about a sixth of the step's cost
    // (development Mac, -O2, fresh mask: p50 2.9 ms instead of 18.4 ms; the 1.8 took about 51 ms
    // with the full floor, 2026-10-15).
    void light_floor() {
        for (std::size_t y = 0; y < geo::kNativeHeight; ++y)
            for (std::size_t x = 0; x < geo::kNativeWidth; ++x)
                if (y < kLightFirstRow || x % kLightColumnStep) raster[y * geo::kNativeWidth + x] = 0;
    }
    static constexpr std::size_t kLightFirstRow = 200, kLightColumnStep = 8;

    // The inputs the service hands to run_profile_steady(), with the T and S owners.
    svc::ProfileSteadyInputs inputs() {
        return {mailbox, geometry, grid, rns::HealthRoiView{}, false, nullptr, capture_clock.native_clock(), channel,
                {&transport, &RecordingTransport::read_audit}, &semantic, &s_slot, semantic_options, &t, &t_slot,
                &t_ready, t_max_age_ns, reasons, kGroundFitFallback, kGroundFreeWithdrawn, timing_handoff};
    }
    // One frame of the floor: host ready = now_ns, capture = now_ns - lag_ns. The stamp the slow
    // line would give its results for this frame goes to `stamp`.
    nd::Offer offer(std::int64_t now_ns, std::int64_t lag_ns, geo::CaptureStamp& stamp) {
        nd::View v;
        v.metadata.intrinsics = b.intrinsics;
        v.metadata.distortion = b.distortion;
        v.metadata.unit_m = b.unit_m;
        auto& s = v.metadata.source;
        s.stream_id = stream;
        s.sequence = ++sequence;
        s.color_index = sequence * 3;
        s.depth_index = sequence * 5 + 1;
        s.color_device_us = sequence * 1000;
        s.depth_device_us = sequence * 1000 + 1;
        s.host_ready_mono_ns = now_ns;
        s.capture_mono_ns = now_ns - lag_ns;
        v.pre_d2c = true;
        v.pixels = raster.data();
        v.bytes = raster.size() * sizeof(raster[0]);
        stamp = {s, 1, s.capture_mono_ns};
        return mailbox.try_offer(v);
    }
};

}  // namespace perception::test_profile
