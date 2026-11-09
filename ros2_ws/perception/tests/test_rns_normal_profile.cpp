/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_normal_profile.cpp
 * Brief: Synthetic native depth -> normal geometry/grid/T -> r4 Profile -> injected local put.
 *
 * Description:
 * No camera, network, calibration certification, Q4 or navigation. Also holds the A19-NEG-1
 * pit-scene golden of 19 section 14 (test_a19_neg1_pit_scene).
 */

#include "output/rns_normal_profile.hpp"
#include "strict_json.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
namespace json = perception::rns::test_json;
std::atomic<std::size_t> allocations{0};
std::size_t geometry_calls = 0, grid_calls = 0, t_calls = 0, build_calls = 0, publish_calls = 0, strict_packets = 0;
void* allocate(std::size_t n) {
    void* p = std::malloc(n ? n : 1);
    if (!p) throw std::bad_alloc();
    ++allocations;
    return p;
}
void* allocate_aligned(std::size_t n, std::size_t alignment) {
    void* p = nullptr;
    if (posix_memalign(&p, alignment, n ? n : 1)) throw std::bad_alloc();
    ++allocations;
    return p;
}
[[noreturn]] void fail(const char* expression, int line) {
    std::cerr << "normal profile CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
}
#define CHECK(x) do { if (!(x)) fail(#x, __LINE__); } while (false)
}
void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) { return allocate_aligned(n, static_cast<std::size_t>(a)); }
void* operator new[](std::size_t n, std::align_val_t a) { return allocate_aligned(n, static_cast<std::size_t>(a)); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace perception::geometry;
using namespace perception::rns;
namespace nd = perception::native_depth;
constexpr double pi = 3.14159265358979323846;
std::size_t at(std::size_t x, std::size_t y) { return y * kNativeWidth + x; }
bool close(double a, double b, double tolerance = 1e-6) { return std::abs(a - b) <= tolerance; }
constexpr double kSceneRangeM = 8;   // level ground farther than this gives no return in a scene
bool empty(const Bin& b) {
    return !b.d_free && !b.d_block && !b.h_block && !b.slope_deg &&
        b.src == 0 && b.conf == 0 && b.terrain == 0;
}
ProjectionBinding binding() {
    ProjectionBinding b;
    b.intrinsics = {128, 128, 320, -128, 640, 400};
    b.distortion.reported = true; b.distortion.model = 0;
    b.unit_m = 1.0F / 1024.0F;
    b.R_base_camera = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    b.t_base_camera = {0, 0, 1};
    return b;
}
// The camera of the A19-PROF-2/3/4 scenes: 1 m above the ground like binding(), but looking
// ahead. The optical axis is level, the rows above row 160 look up (a bar overhead is in view),
// and the level ground is seen from 1.34 m outwards; binding() looks steeply down and sees no
// ground beyond 2.7 m and nothing above its own height.
ProjectionBinding forward_binding() {
    auto b = binding();
    b.intrinsics = {320, 320, 320, 160, 640, 400};
    return b;
}
GeometryOptions make_geometry_options() {
    GeometryOptions o;
    o.prior = {{0, 0, 1}, 0};
    o.fit.inlier_distance_m = .015; o.fit.min_inlier_fraction = .8;
    o.fit.max_inlier_rms_m = .005; o.fit.min_xy_stddev_m = .01;
    o.fit.min_up_dot = .5; o.fit.max_prior_angle_rad = .3;
    o.fit.min_inliers = 100; o.fit.hypotheses = 32; o.fit.seed = 0x243f6a8885a308d3ULL;
    o.fit_near_min_m = .7; o.fit_near_max_m = 2.9; o.fit_prior_band_m = .25;
    o.range_max_m = 6; o.blind_near_lo_m = .01; o.h_tol_m = .125;
    o.z_pass_m = .5; o.eps_neg = .1; o.w_h_m = .1;
    // Shared geometry/wire angular grid: both endpoints and step exactly FP32.
    o.angle_min_rad = -155.0 / 128; o.angle_step_rad = 1.0 / 128; o.n_bins = 311;
    return o;
}
std::vector<Vec3> rays(const ProjectionBinding& b) {
    std::vector<Vec3> out(kGeometryPixels);
    for (std::size_t y = 0; y < kNativeHeight; ++y)
        for (std::size_t x = 0; x < kNativeWidth; ++x)
            out[at(x, y)] = {(static_cast<double>(x) - b.intrinsics.cx) / b.intrinsics.fx,
                             (static_cast<double>(y) - b.intrinsics.cy) / b.intrinsics.fy, 1};
    return out;
}
RectifiedColor color_binding() {
    RectifiedColor c;
    c.R_color_base = {0, -1, 0, 0, 0, -1, 1, 0, 0};
    c.t_color_base = {.125, 1.25, .25};
    c.fx = 200; c.fy = 160; c.cx = 640; c.cy = 100;
    return c;
}
TOptions t_options() { return {.8, 0, 0, 0, 50000000}; }
struct Clock final : PublishClock {
    Millis mono = 100000, wall = 1700000000123;
    Millis wall_millis() noexcept override { return wall; }
    Millis mono_millis() noexcept override { return mono; }
};
struct Sink final : RnsTransport {
    explicit Sink(Clock& c) : clock(c) {}
    Clock& clock;
    std::array<char, 262144> packet{};
    std::array<char, 128> key{};
    std::size_t size = 0, calls = 0;
    PutOutcome outcome = PutOutcome::Accepted;
    PublishQos qos{Priority::DataHigh, true, false};
    // A19-ENC-1 on the wire text of the real route (geometry -> grid -> builder -> envelope ->
    // channel), for every packet that reaches this Sink -- from this file and from the tests that
    // include it (test_rns_semantic_profile, test_rns_withdrawn_evidence): the strict JSON parse
    // and the inf / nan scan of tests/strict_json.hpp. The validator allocates and put() is
    // called inside the callers' allocation guards, so its own allocations are taken out of the
    // count again (these tests are single-threaded); what the product code allocates still counts.
    void strict(std::string_view text) {
        const auto before = allocations.load();
        const auto verdict = json::validate(text);
        const bool finite = json::first_non_finite_token(text) == std::string_view::npos;
        if (!verdict.ok) std::cerr << "strict parse stops at byte " << verdict.offset << " (" << verdict.reason << ")\n";
        allocations.store(before);
        CHECK(verdict.ok);
        CHECK(finite);
        ++strict_packets;
    }
    PutReceipt put(std::string_view k, std::string_view p, PublishQos q) override {
        CHECK(k.size() < key.size() && p.size() < packet.size());
        ++calls; size = p.size(); qos = q;
        std::memcpy(packet.data(), p.data(), p.size()); packet[p.size()] = 0;
        std::memcpy(key.data(), k.data(), k.size()); key[k.size()] = 0;
        ++clock.mono;
        strict(p);
        return {outcome, outcome == PutOutcome::Rejected ? 17 : 0};
    }
};
struct Fixture {
    ProjectionBinding projection_binding;
    GeometryOptions geometry_options = make_geometry_options();
    GridOptions grid_options{.25, .9, .4, .02};
    std::vector<Vec3> optical_rays = rays(projection_binding);
    std::vector<std::uint16_t> raster = std::vector<std::uint16_t>(kGeometryPixels);
    nd::Mailbox mailbox{kGeometryPixels};
    nd::Frame frame{kGeometryPixels};
    NativeGeometry geometry{projection_binding, optical_rays.data(), optical_rays.size(), geometry_options};
    NormalGrid grid{geometry, grid_options};
    RectifiedColor color = color_binding();
    ColorProjection projection{1280, 800, 77, &color, project_rectified_color};
    std::vector<std::uint8_t> mask_pixels = std::vector<std::uint8_t>(1280U * 800U, 1);
    NormalTraversability t;
    NormalProfileBuilder builder;
    CaptureStamp capture;
    std::uint64_t stream = 0, sequence = 0, tick = 0;
    Clock clock;
    Sink sink{clock};
    ProcessEnvelope envelope{"robot_1", "abcdef01", 5000};
    PublisherChannel channel{Topic::Profile, envelope, clock, sink, 131072, 262144};

    explicit Fixture(TOptions options = t_options(), GeometryOptions go = make_geometry_options(),
                     ProjectionBinding pb = binding())
        : projection_binding(pb), geometry_options(go), t(geometry, grid, projection, options), builder(grid, t) {
        stream = mailbox.begin_stream();
        full_plane();
        CHECK(builder.body() == nullptr && builder.report().status == NormalProfileStatus::NotBuilt);
        CHECK(builder.storage_bytes() == geometry_options.n_bins * sizeof(Bin));
        CHECK(channel.valid());
    }
    // Horizontal range of the point at depth z on a ray: R turns optical +z into base +x and
    // optical +x into base -y.
    static double range_at(const Vec3& ray, double z) { return z * std::hypot(1.0, ray.x); }
    std::uint16_t units(double z) const { return static_cast<std::uint16_t>(std::llround(z / projection_binding.unit_m)); }
    void full_plane() {
        std::size_t without_return = 0;
        for (std::size_t i = 0; i < raster.size(); ++i) {
            // R turns optical +y into base -z; camera height = 1 metre. A ray at or above the
            // horizon, or one that meets the ground beyond kSceneRangeM, has no return.
            const auto& ray = optical_rays[i];
            const bool seen = ray.y > 0 && range_at(ray, 1.0 / ray.y) <= kSceneRangeM;
            raster[i] = seen ? units(1.0 / ray.y) : 0;
            without_return += raster[i] == 0;
        }
        // binding() looks down everywhere: every pixel of its level ground has a return.
        CHECK(projection_binding.intrinsics.cy > 0 || without_return == 0);
    }
    void exact_ground_support() {
        std::fill(raster.begin(), raster.end(), 0);
        for (std::size_t x = 0; x < kNativeWidth; ++x) {
            raster[at(x, 0)] = 1024;
            raster[at(x, 128)] = 512;
            raster[at(x, 384)] = 256;
        }
    }
    void advance(bool run_t = true, bool has_mask = false) {
        nd::View view;
        view.metadata.intrinsics = projection_binding.intrinsics;
        view.metadata.distortion = projection_binding.distortion;
        view.metadata.unit_m = projection_binding.unit_m;
        auto& s = view.metadata.source;
        s.stream_id = stream; s.sequence = ++sequence;
        s.color_index = sequence * 3; s.depth_index = sequence * 5 + 1;
        s.color_device_us = sequence * 1000; s.depth_device_us = sequence * 1000 + 1;
        s.host_ready_mono_ns = static_cast<std::int64_t>(sequence * 1000000ULL);
        view.pre_d2c = true; view.pixels = raster.data(); view.bytes = raster.size() * sizeof(raster[0]);
        const auto before = allocations.load();
        CHECK(mailbox.try_offer(view) == nd::Offer::Accepted);
        CHECK(mailbox.try_take(frame) == nd::Take::Taken);
        geometry.process(frame, mailbox); ++geometry_calls;
        grid.process(mailbox); ++grid_calls;
        CHECK(allocations.load() == before);
        capture = {frame.metadata().source, 42,
                   1000000000LL + static_cast<std::int64_t>(++tick * 10000000ULL)};
        if (run_t) {
            auto mask = raw_mask();
            process_t(has_mask ? &mask : nullptr);
        }
    }
    RawTMask raw_mask() const {
        return {capture, MaskMaterial::RawTraversable, projection.revision, 1280, 800,
                1280, mask_pixels.size(), mask_pixels.data()};
    }
    void process_t(const RawTMask* mask) {
        const auto before = allocations.load();
        t.process(mailbox, capture, mask); ++t_calls;
        CHECK(allocations.load() == before);
    }
    NormalProfileStatus build(const CaptureStamp& stamp) {
        const auto before = allocations.load();
        const auto status = builder.build(mailbox, stamp).status; ++build_calls;
        CHECK(allocations.load() == before);
        return status;
    }
    NormalProfileStatus build() { return build(capture); }
    NormalProfilePut publish() {
        const auto before = allocations.load();
        auto result = builder.publish(mailbox, channel); ++publish_calls;
        CHECK(allocations.load() == before);
        return result;
    }
    void expect_no_dispatch() {
        const auto calls = sink.calls;
        CHECK(!publish().dispatched && sink.calls == calls && builder.body() == nullptr);
    }
    void restart() { mailbox.end_stream(); stream = mailbox.begin_stream(); sequence = 0; }
};

void check_body(const Fixture& f, bool fresh) {
    const auto* p = f.builder.body(); CHECK(p);
    CHECK(p->bins.size() == f.geometry_options.n_bins && !p->extrinsic_calibrated);
    CHECK(p->t_publish_mono_ms == -1 && p->t_capture_mono_ms == f.capture.capture_mono_ns / 1000000);
    CHECK(p->t_capture_mono_ms != f.capture.source.host_ready_mono_ns / 1000000);
    CHECK(p->t_seg_mono_ms.has_value() == fresh);
    CHECK(p->range_max_m <= f.geometry_options.range_max_m);
    CHECK(p->z_pass_m <= f.geometry_options.z_pass_m);
    CHECK(p->angle_min_rad == f.geometry_options.angle_min_rad &&
          p->angle_step_rad == f.geometry_options.angle_step_rad);
    CHECK(f.grid.result().blind_m && p->blind_near_m && *p->blind_near_m >= *f.grid.result().blind_m);
    std::size_t geom = 0, ts = 0, blocked = 0;
    for (std::size_t i = 0; i < p->bins.size(); ++i) {
        const auto& b = p->bins[i]; const auto& e = f.grid.bins()[i]; const auto& s = f.t.prefixes()[i];
        CHECK((b.src & kG) == ((e.valid_returns || e.valid_pierce_rays) ? kG : 0));
        const auto conf = e.expected_rays ? std::min(e.valid_returns, e.expected_rays) * 255 / e.expected_rays : 0;
        CHECK(b.conf == conf && b.terrain == 0 && !(b.src & kS));
        if (b.d_free) CHECK(s.free_end_m && *b.d_free <= *s.free_end_m && *b.d_free >= *p->blind_near_m);
        if (b.d_block) CHECK(s.block_m && *b.d_block <= *s.block_m);
        if (b.d_free && b.d_block) CHECK(*b.d_free < *b.d_block);
        if (b.src & kT) CHECK(fresh && s.t_backed_prefix && b.d_free && *b.d_free > *p->blind_near_m);
        if (!b.d_free || *b.d_free <= *p->blind_near_m) CHECK(!(b.src & kT) && !b.slope_deg);
        if (b.h_block) {
            CHECK(s.block_height_m && close(*b.h_block, *s.block_height_m));
            CHECK((b.src & kNegative) ? *b.h_block <= 0 : *b.h_block > 0);
        }
        geom += (b.src & kG) != 0; ts += (b.src & kT) != 0; blocked += b.d_block.has_value();
    }
    CHECK(f.builder.report().geometry_bins == geom && f.builder.report().t_bins == ts &&
          f.builder.report().blocked_bins == blocked);
}
void print_fixture(const char* name, const Fixture& f, const NormalProfilePut& put) {
    CHECK(put.dispatched && put.publish);
    std::cout << "{\"case\":\"" << name << "\",\"key\":\"" << f.sink.key.data()
              << "\",\"packet\":" << f.sink.packet.data() << ",\"capture\":" << put.publish.audit.capture_ms
              << ",\"before\":" << put.publish.audit.put_before_ms
              << ",\"after\":" << put.publish.audit.put_return_ms << "}\n";
}

// Without T the Profile is geometry-only (FREE beyond the blind edge) and is put once on the real-
// time profile key with the capture time; with fresh T it carries t_seg; the same frame is not
// built twice.
void test_normal_missing_and_fresh(bool fixtures) {
    Fixture f;
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
    CHECK(f.builder.report().t_status == TStatus::Missing && f.builder.report().t_bins == 0);
    CHECK(std::any_of(f.builder.body()->bins.begin(), f.builder.body()->bins.end(),
                     [&](const Bin& b) { return b.d_free && *b.d_free > *f.builder.body()->blind_near_m; }));
    auto put = f.publish(); CHECK(put.dispatched && put.publish && f.sink.calls == 1);
    CHECK(put.publish.audit.capture_ms == f.capture.capture_mono_ns / 1000000);
    CHECK(f.sink.qos.priority == Priority::RealTime && f.sink.qos.drop && f.sink.qos.express && !f.sink.qos.reliable);
    CHECK(std::string_view(f.sink.key.data()) == "xbrain/robot_1/rt/perception/profile");
    if (fixtures) print_fixture("normal_no_t", f, put);
    f.expect_no_dispatch();
    f.advance(true, true); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, true);
    CHECK(f.builder.report().t_status == TStatus::Fresh && f.builder.report().t_bins > 0);
    CHECK(f.builder.body()->t_seg_mono_ms == f.capture.capture_mono_ns / 1000000);
    put = f.publish(); CHECK(put.dispatched && put.publish);
    if (fixtures) print_fixture("normal_fresh_true", f, put);
    CHECK(f.build() == NormalProfileStatus::InactiveOrStale); f.expect_no_dispatch();
}

// A geometry-only builder never calls T, allocates nothing, and encodes the same bytes as a
// T-configured builder whose T is missing; a foreign mailbox or a failed frame builds nothing.
void test_explicit_geometry_only() {
    Fixture f;
    NormalProfileBuilder only_g(f.grid);
    f.advance(false); // No call into T, even though the comparison fixture owns it.
    auto before = allocations.load();
    CHECK(only_g.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready);
    CHECK(allocations.load() == before);
    CHECK(only_g.report().t_status == TStatus::Missing && only_g.report().t_bins == 0);
    CHECK(!only_g.body()->t_seg_mono_ms);
    f.process_t(nullptr);
    CHECK(f.build() == NormalProfileStatus::Ready);
    std::array<char, 131072> a{}, b{};
    const auto ea = encode_at_publish(*only_g.body(), 2000, a.data(), a.size());
    const auto eb = encode_at_publish(*f.builder.body(), 2000, b.data(), b.size());
    CHECK(ea && eb && ea.size == eb.size && std::memcmp(a.data(), b.data(), ea.size) == 0);
    nd::Mailbox other{kGeometryPixels};
    CHECK(other.begin_stream() == f.stream);
    CHECK(!only_g.publish(other, f.channel).dispatched && only_g.body() == nullptr);
    f.advance(false);
    CHECK(only_g.build(other, f.capture).status == NormalProfileStatus::InvalidIdentity);
    CHECK(only_g.body() == nullptr && f.sink.calls == 0);
    CHECK(only_g.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready);
    CHECK(only_g.publish(f.mailbox, f.channel).publish);
    f.advance(false);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    f.advance(false);
    CHECK(only_g.build(f.mailbox, f.capture).status != NormalProfileStatus::Ready);
    CHECK(!only_g.publish(f.mailbox, f.channel).dispatched);

    // The existing configured-T mode still cannot use a foreign mailbox.
    f.full_plane(); f.advance();
    CHECK(f.builder.build(other, f.capture).status == NormalProfileStatus::InvalidIdentity);
    CHECK(f.build() == NormalProfileStatus::Ready);
    CHECK(!f.builder.publish(other, f.channel).dispatched);
}

// With no ground in the Profile's domain the bin has no FREE, slope or blind edge but keeps the
// measured obstacle (also with T configured); only a strictly nearer fresh S takes the boundary;
// publishing stays bound to the current owners and generation.
void test_no_ground_reference_geometry_only() {
    auto go = make_geometry_options(); go.angle_min_rad = 0; go.n_bins = 1;
    Fixture f(t_options(), go);
    NormalProfileBuilder only_g(f.grid);
    f.exact_ground_support();
    // Other columns support the actual fit, but the only in-domain return is
    // an obstacle: no nearest ground exists in this Profile's angular domain.
    for (std::size_t y = 0; y < kNativeHeight; ++y) f.raster[at(320, y)] = 0;
    f.raster[at(320, 128)] = 384;
    f.advance(false);
    CHECK(f.geometry.result().has_evidence() && f.geometry.result().fit.inliers >= 100);
    CHECK(!f.geometry.result().nearest_ground_m);
    CHECK(f.grid.result().status == GridStatus::NoGroundReference && !f.grid.result().blind_m);
    const auto build = [&] {
        const auto before = allocations.load();
        const auto status = only_g.build(f.mailbox, f.capture).status; ++build_calls;
        CHECK(allocations.load() == before);
        return status;
    };
    CHECK(build() == NormalProfileStatus::Ready && only_g.report().no_ground_reference);
    CHECK(only_g.report().t_status == TStatus::Missing && only_g.report().semantic_status == SemanticStatus::Missing);
    CHECK(only_g.report().geometry_bins == 1 && only_g.report().blocked_bins == 1);
    CHECK(only_g.report().t_bins == 0 && only_g.report().semantic_bins == 0);
    const auto* p = only_g.body(); CHECK(p && !p->blind_near_m && !p->t_seg_mono_ms);
    CHECK(!p->extrinsic_calibrated && p->bins.size() == 1);
    const auto& b = p->bins[0];
    CHECK(b.src == kG && b.d_block == .375F && b.h_block && close(*b.h_block, .25));
    CHECK(!b.d_free && !b.slope_deg && b.terrain == 0);
    CHECK(b.conf == std::min(f.grid.bins()[0].valid_returns, f.grid.bins()[0].expected_rays) *
          255 / f.grid.bins()[0].expected_rays);
    auto before = allocations.load();
    const auto put = only_g.publish(f.mailbox, f.channel); ++publish_calls;
    CHECK(allocations.load() == before && put.dispatched && put.publish && f.sink.calls == 1);
    CHECK(std::string_view(f.sink.packet.data()).find("\"blind_near_m\":null") != std::string_view::npos);
    CHECK(std::string_view(f.sink.packet.data()).find("\"d_free\":[null]") != std::string_view::npos);
    CHECK(!only_g.publish(f.mailbox, f.channel).dispatched && f.sink.calls == 1);
    CHECK(build() == NormalProfileStatus::InactiveOrStale && !only_g.report().no_ground_reference);

    // Reply batch 6, B6-2 (19 A19-NULL-1 / A19-FIT-3): a configured T or S owner
    // does NOT stop a legal no-ground frame. FREE, the T backing and the slope
    // are withdrawn; the measured obstacle stays; an independent S boundary is
    // kept and wins only when it is strictly nearer.
    f.advance();
    CHECK(f.build() == NormalProfileStatus::Ready && f.builder.report().no_ground_reference);
    {
        const auto* tp = f.builder.body();
        CHECK(tp && !tp->blind_near_m && !tp->bins[0].d_free && !(tp->bins[0].src & kT) && !tp->bins[0].slope_deg);
        CHECK(tp->bins[0].d_block == .375F && tp->bins[0].h_block);   // the measured obstacle survives
        CHECK(f.builder.report().t_bins == 0 && f.builder.report().semantic_bins == 0);
    }
    SemanticProjector semantic(f.geometry, {0, 0, 0, 50000000, 1});
    // S missing: still publishable, still geometry-only
    CHECK(semantic.process(f.mailbox, f.capture, f.capture.capture_mono_ns, nullptr).status == SemanticStatus::Missing);
    NormalProfileBuilder with_s(f.grid, f.t, semantic);
    CHECK(with_s.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready);
    CHECK(with_s.body() && with_s.body()->bins[0].d_block == .375F && !(with_s.body()->bins[0].src & kS));
    // S fresh and strictly nearer than the measured obstacle: S owns the boundary
    f.advance();
    SemanticObject object;
    object.track_id = 3; object.class_id = 1; object.class_name = "door";
    object.vertices = 4;
    object.footprint[0] = {.20F, -.05F}; object.footprint[1] = {.30F, -.05F};
    object.footprint[2] = {.30F, .05F}; object.footprint[3] = {.20F, .05F};
    SemanticFrame view{f.capture, 1, SemanticMaterial::MeasuredBaseFootprint, &object, 1};
    CHECK(semantic.process(f.mailbox, f.capture, f.capture.capture_mono_ns, &view).status == SemanticStatus::Fresh);
    CHECK(with_s.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready);
    {
        const auto* sp = with_s.body();
        CHECK(sp && sp->bins[0].d_block && *sp->bins[0].d_block < .375F);
        CHECK((sp->bins[0].src & kS) && (sp->bins[0].src & kG) && !sp->bins[0].h_block);
        CHECK(!sp->bins[0].d_free && !(sp->bins[0].src & kT) && !sp->bins[0].slope_deg);
        CHECK(with_s.report().semantic_bins == 1);
    }
    // S fresh but farther than the measured obstacle: geometry keeps the boundary
    f.advance();
    object.footprint[0] = {.80F, -.05F}; object.footprint[1] = {.90F, -.05F};
    object.footprint[2] = {.90F, .05F}; object.footprint[3] = {.80F, .05F};
    view = {f.capture, 1, SemanticMaterial::MeasuredBaseFootprint, &object, 1};
    CHECK(semantic.process(f.mailbox, f.capture, f.capture.capture_mono_ns, &view).status == SemanticStatus::Fresh);
    CHECK(with_s.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready);
    {
        const auto* sp = with_s.body();
        CHECK(sp && sp->bins[0].d_block == .375F && sp->bins[0].h_block && !(sp->bins[0].src & kS));
        CHECK(with_s.report().semantic_bins == 0);
    }
    // the S snapshot expires: its old boundary is NOT carried over
    const auto stale_view = view;
    for (int i = 0; i < 6; ++i) f.advance();
    CHECK(semantic.process(f.mailbox, f.capture, f.capture.capture_mono_ns, &stale_view).status == SemanticStatus::Stale);
    CHECK(with_s.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready);
    CHECK(with_s.body() && !(with_s.body()->bins[0].src & kS) && with_s.report().semantic_bins == 0);
    CHECK(f.sink.calls == 1);   // nothing published in this block

    // Publishing remains bound to the current owners and generation.
    CHECK(build() == NormalProfileStatus::Ready);
    nd::Mailbox other{kGeometryPixels}; CHECK(other.begin_stream() == f.stream);
    CHECK(!only_g.publish(other, f.channel).dispatched && !only_g.body());
    f.advance(false); CHECK(build() == NormalProfileStatus::Ready);
    f.grid.process(f.mailbox); ++grid_calls; // replay invalidates grid evidence
    CHECK(!only_g.publish(f.mailbox, f.channel).dispatched && !only_g.body());
    f.advance(false); CHECK(build() == NormalProfileStatus::Ready);
    f.mailbox.end_stream();
    CHECK(!only_g.publish(f.mailbox, f.channel).dispatched && !only_g.body());
    f.restart(); f.full_plane(); f.advance(false);
    CHECK(f.grid.result().status == GridStatus::NormalEvidence);
    CHECK(build() == NormalProfileStatus::Ready && !only_g.report().no_ground_reference);
    CHECK(only_g.body()->blind_near_m && only_g.publish(f.mailbox, f.channel).publish);
}

// B6-1: the depth-independent azimuth path and its fallback. The fast path is
// an algebraic factoring that is only valid with NO lateral offset in
// base <- camera; with any lateral offset the general path must run, and both
// must produce the evidence the Profile is built from.
void test_fast_azimuth_and_fallback() {
    {
        Fixture f;                                  // t = (0, 0, 1): no lateral offset
        CHECK(f.geometry.fast_azimuth());
        f.advance(false);
        CHECK(f.geometry.result().status == GeometryStatus::NormalEvidence);
        // every valid pixel's cached bin equals the reference bin of its own point
        std::size_t checked = 0;
        for (std::size_t k = 0; k < f.geometry.pixels().size(); k += 997) {
            const auto& e = f.geometry.pixels()[k];
            if (e.classification != SampleClass::Ground) continue;
            Vec3 p;
            CHECK(f.geometry.ground_point(k, p));
            CHECK(e.bin == f.geometry.reference_azimuth_bin(p));
            CHECK(std::abs(e.range_m - std::hypot(p.x, p.y)) <= 1e-12);
            ++checked;
        }
        CHECK(checked > 0);
    }
    {
        auto b = binding();
        b.t_base_camera = {0.05, 0, 1};             // lateral offset: fast path must be OFF
        Fixture f(t_options(), make_geometry_options());
        // the fixture owns its own geometry, so build a separate one on the same rays
        auto rays_copy = rays(b);
        NativeGeometry general(b, rays_copy.data(), rays_copy.size(), make_geometry_options());
        CHECK(!general.fast_azimuth());
        // and it still produces evidence on a real frame
        f.advance(false);
        const auto& r = general.process(f.frame, f.mailbox);
        CHECK(r.status == GeometryStatus::NormalEvidence || r.status == GeometryStatus::FitFailed);
        CHECK(r.visited_pixels == kGeometryPixels);
    }
}

// Ground exactly at range_max is still a no-ground-reference frame: blind edge at that range, no
// FREE and no block.
void test_range_edge_ground_reference() {
    auto go = make_geometry_options(); go.angle_min_rad = 0; go.n_bins = 1;
    go.range_max_m = 1; go.fit_near_min_m = .3; go.fit_near_max_m = 1;
    Fixture f(t_options(), go);
    NormalProfileBuilder only_g(f.grid);
    f.exact_ground_support();
    for (std::size_t y = 0; y < kNativeHeight; ++y) f.raster[at(320, y)] = 0;
    f.raster[at(320, 0)] = 1024; // real ground exactly at the configured range edge
    f.advance(false);
    CHECK(f.geometry.result().has_evidence() && f.geometry.result().nearest_ground_m == 1);
    CHECK(f.grid.result().status == GridStatus::NoGroundReference && f.grid.result().blind_m == 1);
    CHECK(f.grid.result().cells_per_bin == 0);
    const auto before = allocations.load();
    CHECK(only_g.build(f.mailbox, f.capture).status == NormalProfileStatus::Ready); ++build_calls;
    CHECK(allocations.load() == before && only_g.report().no_ground_reference);
    const auto* p = only_g.body(); CHECK(p && p->blind_near_m == 1 && !p->t_seg_mono_ms);
    CHECK(p->bins[0].src == kG && !p->bins[0].d_free && !p->bins[0].d_block &&
          !p->bins[0].h_block && !p->bins[0].slope_deg);
    CHECK(only_g.publish(f.mailbox, f.channel).publish); ++publish_calls;
}

// Fresh T with an all-false mask keeps every FREE at the blind edge while t_seg is set; with a
// positive jitter margin and near support missing, FREE also stays at the blind edge.
void test_fresh_false_and_positive_margin(bool fixtures) {
    Fixture f;
    std::fill(f.mask_pixels.begin(), f.mask_pixels.end(), 0);
    f.advance(true, true); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, true);
    CHECK(f.builder.report().t_bins == 0 && f.builder.body()->t_seg_mono_ms);
    bool original_advanced = false;
    for (std::size_t i = 0; i < f.grid.bins().size(); ++i) {
        const auto& b = f.builder.body()->bins[i];
        original_advanced |= f.grid.bins()[i].traversal.passed_cells > 0;
        CHECK(!b.d_free || *b.d_free <= *f.builder.body()->blind_near_m);
    }
    CHECK(original_advanced && !f.builder.report().near_support_missing);
    auto put = f.publish(); CHECK(put.dispatched && put.publish);
    if (fixtures) print_fixture("normal_fresh_false", f, put);
    auto options = t_options(); options.jitter_m = .02;
    Fixture margin(options);
    margin.raster[kGeometryPixels - 1] = 187;
    margin.advance(true, true); CHECK(margin.build() == NormalProfileStatus::Ready); check_body(margin, true);
    CHECK(margin.builder.report().near_support_missing && margin.builder.report().t_bins == 0);
    CHECK(margin.builder.report().blocked_bins > 0);
    for (const auto& b : margin.builder.body()->bins)
        CHECK(!b.d_free || *b.d_free <= *margin.builder.body()->blind_near_m);
    put = margin.publish(); CHECK(put.dispatched && put.publish);
    if (fixtures) print_fixture("normal_positive_margin", margin, put);
}

// A raised obstacle and a below-plane drop land in their own bins as G and G|negative blocks with
// their measured heights.
void test_positive_and_negative_association(bool fixtures) {
    Fixture f;
    const auto positive = kGeometryPixels - 1, negative = at(320, 128);
    f.raster[positive] = 187; f.raster[negative] = 584;
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
    const auto& p = f.geometry.pixels()[positive]; const auto& n = f.geometry.pixels()[negative];
    CHECK(p.classification == SampleClass::Obstacle && n.classification == SampleClass::BelowPlane);
    CHECK(p.bin != n.bin && n.pierce_bin == n.bin);
    const auto& bp = f.builder.body()->bins[static_cast<std::size_t>(p.bin)];
    const auto& bn = f.builder.body()->bins[static_cast<std::size_t>(n.bin)];
    CHECK(bp.src == kG && bp.d_block && bp.h_block && *bp.h_block > 0);
    CHECK(*bp.d_block <= p.range_m && close(*bp.h_block, p.height_m));
    CHECK(bn.src == (kG | kNegative) && bn.d_block && bn.h_block && *bn.h_block < 0);
    CHECK(close(*bn.h_block, n.height_m));
    CHECK(f.builder.report().blocked_bins >= 2);
    auto put = f.publish(); CHECK(put.dispatched && put.publish);
    if (fixtures) print_fixture("normal_positive_negative", f, put);
}

// A19-NEG-1 golden (19 section 14; 11 section 3.1B.1 bit3): a pit in flat ground, as a depth scene
// through geometry, grid and builder, asserted on the Profile body. In the centre columns every ray
// that would meet the ground at or beyond the pit edge (0.80 m) lands on the pit floor 0.30 m lower:
// farther than expected (pierce) and below the plane. The nearest in-pit return is at 1.04 m, past
// the cell that holds the edge, so only the pierce criterion can put the boundary in that cell.
// Expected in every pit bin: d_block at the near edge of the cell that holds the pit edge, d_free
// one cell earlier, bit3 (NEG) and bit1 (G), h_block the depth measured in the pit (-0.3 m: every
// pit bin has in-pit returns, 19 section 3.4 and the 11 pit row), and with a fresh mask bit0 too
// (the 0b1011 of the 11 pit row); no other bin has bit3. With blind 0.2431640625 and dr 0.25 that
// is d_free 0.4931640625 / d_block 0.7431640625. The frame must also be put: a negative
// boundary whose distance comes from pierce and whose height from returns one cell farther.
// Paired mutant: the pierce criterion switched off. The in-pit points still set bit3, so the bit
// alone stays green, but the boundary moves out to their cell (0.9931640625) and d_free stops one
// cell farther (0.7431640625); the two distance checks turn that red.
void test_a19_neg1_pit_scene() {
    constexpr double kPitEdgeM = .8, kPitDepthM = .3;
    constexpr std::size_t kPitColumnLo = 288, kPitColumnHi = 352;   // ray.x in [-0.25, 0.25]
    Fixture f;
    for (std::size_t y = 0; y < kNativeHeight; ++y)
        for (std::size_t x = kPitColumnLo; x <= kPitColumnHi; ++x) {
            const auto& ray = f.optical_rays[at(x, y)];
            // Horizontal range at which this ray would have met the level ground.
            if (std::hypot(1.0, ray.x) / ray.y < kPitEdgeM) continue;
            const double z = (1.0 + kPitDepthM) / ray.y;
            f.raster[at(x, y)] = static_cast<std::uint16_t>(std::llround(z / f.projection_binding.unit_m));
        }
    for (const bool fresh : {false, true}) {
        f.advance(true, fresh); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, fresh);
        const auto* body = f.builder.body();
        const double blind = *f.grid.result().blind_m, dr = f.grid_options.dr_m;
        // The cells start at the nearest ground return of the scene: bottom row, centre column, where
        // ray.x = 0 and the range equals the depth. So the expected distances come from the scene.
        CHECK(close(blind, f.raster[at(320, kNativeHeight - 1)] * static_cast<double>(f.projection_binding.unit_m)));
        std::size_t edge_cell = 0;   // the cell [blind + k*dr, blind + (k+1)*dr) that holds the pit edge
        while (blind + static_cast<double>(edge_cell + 1) * dr <= kPitEdgeM) ++edge_cell;
        CHECK(edge_cell == 2);       // so one whole cell of FREE is left before the pit
        const double block_m = blind + static_cast<double>(edge_cell) * dr;
        const double free_m = blind + static_cast<double>(edge_cell - 1) * dr;
        // The bins of the pit columns. The top row is inside the pit in each of them; image +x is
        // base -y, so the highest column has the lowest bin.
        const int low = f.geometry.pixels()[at(kPitColumnHi, 0)].bin;
        const int high = f.geometry.pixels()[at(kPitColumnLo, 0)].bin;
        CHECK(low >= 0 && low <= high);
        const auto first = static_cast<std::size_t>(low), last = static_cast<std::size_t>(high);
        const std::uint8_t pit_src = fresh ? (kT | kG | kNegative) : (kG | kNegative);
        std::size_t pit_bins = 0;
        for (std::size_t i = 0; i < body->bins.size(); ++i) {
            const auto& b = body->bins[i];
            if (i < first || i > last) { CHECK(!(b.src & kNegative)); continue; }   // flat ground: no pit
            ++pit_bins;
            CHECK(b.src == pit_src);
            CHECK(b.d_free && b.d_block && *b.d_free < *b.d_block);
            // FREE ends one cell before the cell with the pit edge (rounded down on the wire) ...
            CHECK(*b.d_free <= free_m && close(*b.d_free, free_m));
            // ... and the boundary is the near edge of that cell, not the first in-pit return.
            CHECK(*b.d_block <= block_m && close(*b.d_block, block_m));
            // The edge comes from pierce alone, the height from the in-pit returns of the same bin
            // (in a later cell): the lowest of them, the pit depth within the 1 mm depth unit.
            CHECK(b.h_block && *b.h_block <= -kPitDepthM + 0.002 && *b.h_block >= -kPitDepthM - 0.002);
        }
        CHECK(pit_bins > 0);
        const auto put = f.publish(); CHECK(put.dispatched && put.publish);
    }
}

// The bins of the interior of a block of columns: the bins of its two edge columns, read from a
// pixel that is ground in both (row `y`), with one bin left out on either side (an edge bin also
// holds columns outside the block).
std::pair<std::size_t, std::size_t> interior_bins(const Fixture& f, std::size_t column_lo, std::size_t column_hi, std::size_t y) {
    const int low = f.geometry.pixels()[at(column_hi, y)].bin, high = f.geometry.pixels()[at(column_lo, y)].bin;
    CHECK(low >= 0 && high - low >= 4);   // image +x is base -y: the highest column has the lowest bin
    return {static_cast<std::size_t>(low) + 1, static_cast<std::size_t>(high) - 1};
}

// A19-PROF-2 golden (19 section 14): a "glass hole" in level ground, as a depth scene through
// geometry, grid and builder, asserted on the Profile body. In the centre columns the depth is
// invalid where the ground lies between 2.5 m and 3.5 m (no return at all: neither an obstacle nor
// a pit), and valid ground again behind it. Expected in every hole bin: FREE stops before the
// hole -- at the far edge of the last cell that is still fully seen, which is the cell before the
// one the hole starts in -- and there is NO boundary: d_block and h_block null, bit1 only. FREE
// must not reach the ground behind the hole either. Outside the hole the same ground is FREE
// well past the hole's range, so the stop is the hole's doing.
// The contract's mutant: invalid depth filled with range_max. Here the hole's rays then carry a
// return far beyond where the ground should be and the hole reads as a drop: the src check turns
// red. Its sibling, invalid depth filled in as ground (any depth completion before the traversal),
// makes the hole passable: FREE runs through it and the distance check turns red.
void test_a19_prof2_glass_hole() {
    constexpr double kHoleNearM = 2.5, kHoleFarM = 3.5;
    constexpr std::size_t kLo = 256, kHi = 384;   // ray.x in [-0.2, 0.2]
    Fixture f(t_options(), make_geometry_options(), forward_binding());
    std::size_t holes = 0;
    for (std::size_t y = 0; y < kNativeHeight; ++y)
        for (std::size_t x = kLo; x <= kHi; ++x) {
            const auto& ray = f.optical_rays[at(x, y)];
            if (ray.y <= 0) continue;
            const double range = Fixture::range_at(ray, 1.0 / ray.y);
            if (range >= kHoleNearM && range < kHoleFarM) { f.raster[at(x, y)] = 0; ++holes; }
        }
    CHECK(holes > 1000);
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
    const auto* body = f.builder.body();
    const double blind = *f.grid.result().blind_m, dr = f.grid_options.dr_m;
    std::size_t hole_cell = 0;   // the cell [blind + k*dr, blind + (k+1)*dr) the hole starts in
    while (blind + static_cast<double>(hole_cell + 1) * dr <= kHoleNearM) ++hole_cell;
    CHECK(hole_cell >= 2);       // whole cells of FREE lie before the hole
    const double free_m = blind + static_cast<double>(hole_cell) * dr;   // far edge of the cell before it
    CHECK(free_m <= kHoleNearM && free_m > kHoleNearM - dr);
    const auto [first, last] = interior_bins(f, kLo, kHi, kNativeHeight - 1);
    std::size_t outside = 0;
    for (std::size_t i = 0; i < body->bins.size(); ++i) {
        const auto& b = body->bins[i];
        if (i >= first && i <= last) {
            CHECK(b.src == kG);
            // FREE ends before the hole (rounded down on the wire), within one cell of it ...
            CHECK(b.d_free && *b.d_free <= free_m && close(*b.d_free, free_m));
            // ... and nothing is reported as a boundary: the hole is unknown, not an obstacle.
            CHECK(!b.d_block && !b.h_block && !(b.src & kNegative));
        } else if (b.d_free && (i + 8 < first || i > last + 8) && (b.src & kG) && !b.d_block && *b.d_free > kHoleFarM) {
            ++outside;   // the same ground next to the hole is FREE past the hole's far edge
        }
    }
    CHECK(last - first >= 20 && outside >= 20);
    const auto put = f.publish(); CHECK(put.dispatched && put.publish);
}

// A19-PROF-3 golden: a thin pole, ONE pixel column wide, 1.5 m tall, 4.0 m ahead, on level ground,
// in front of a wall 7 m away (beyond range_max, so the wall itself is outside the domain).
// Its bin must carry the boundary at the pole: d_block in the grid cell that holds 4.0 m (at most
// 4.0, less than one cell short of it), h_block the pole's tallest return below z_pass, FREE
// ending before it; the bins a few steps to either side stay unblocked. The input is the
// full-resolution raster (PROF-3: no downsampling anywhere in the pipeline).
// The contract's mutant: a 2x2 mean before the traversal. The pole's column is averaged with what
// is beside it -- the ground and the wall behind -- so its returns land at 4.8 m and beyond and
// the boundary at 4.0 is gone.
// Why the wall: every pixel beside the pole then has a return FARTHER than the pole. Without it
// the scene has "no return" (code 0) next to ground 8 m away around row 200, and a 2x2 mean of
// {4.0, 0, 4.0, 7.8} m is 3.95 m: the mutant put an obstacle back within one cell of the pole
// by accident and this test stayed green (seen on 2026-10-14 when the mutant was first run).
void test_a19_prof3_thin_pole() {
    constexpr double kPoleRangeM = 4.0, kPoleHeightM = 1.5, kWallDepthM = 7.0;
    constexpr std::size_t kColumn = 320;   // ray.x = 0: the range of a return is its depth
    Fixture f(t_options(), make_geometry_options(), forward_binding());
    CHECK(kWallDepthM > f.geometry_options.range_max_m);
    for (std::size_t i = 0; i < f.raster.size(); ++i)   // the wall hides the sky and the ground behind it
        if (f.raster[i] == 0 || f.raster[i] > f.units(kWallDepthM)) f.raster[i] = f.units(kWallDepthM);
    std::size_t pole_pixels = 0;
    for (std::size_t y = 0; y < kNativeHeight; ++y) {
        const auto& ray = f.optical_rays[at(kColumn, y)];
        const double height = 1.0 - kPoleRangeM * ray.y;   // of the ray where it crosses the pole's range
        if (height < 0 || height > kPoleHeightM) continue;
        f.raster[at(kColumn, y)] = f.units(kPoleRangeM); ++pole_pixels;
    }
    CHECK(pole_pixels > 50);
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
    const auto* body = f.builder.body();
    // Row 220 sees the pole 0.25 m above the ground: below z_pass, so an obstacle return (the
    // part of the pole above z_pass is classified Overhead and takes no part in the boundary).
    const auto& pole = f.geometry.pixels()[at(kColumn, 220)];
    const int bin = pole.bin;
    CHECK(bin >= 8 && pole.classification == SampleClass::Obstacle);
    const auto& b = body->bins[static_cast<std::size_t>(bin)];
    CHECK((b.src & kG) && !(b.src & kNegative));
    CHECK(b.d_block && *b.d_block <= kPoleRangeM && *b.d_block > kPoleRangeM - f.grid_options.dr_m);
    // The boundary's height is that of the pole's tallest return below z_pass (one row is about 1 cm here).
    CHECK(b.h_block && *b.h_block > f.geometry_options.z_pass_m - 0.03 && *b.h_block <= f.geometry_options.z_pass_m);
    CHECK(b.d_free && *b.d_free < *b.d_block);
    for (const int offset : {-6, -5, -4, 4, 5, 6}) {   // level ground a few bins away: no boundary
        const auto& side = body->bins[static_cast<std::size_t>(bin + offset)];
        CHECK((side.src & kG) && !side.d_block && side.d_free && *side.d_free > kPoleRangeM);
    }
    const auto put = f.publish(); CHECK(put.dispatched && put.publish);
}

// A19-PROF-4 golden PAIR: a horizontal bar 3.0 m ahead, 10 cm thick, across the centre columns,
// over level ground; z_pass is 1.5 m.
//   overhead (bar at 1.8 m, above z_pass): it is NOT a boundary. Its returns are classified
//     Overhead, the bins under it have no d_block and stay FREE past the bar.
//   lowered (the same bar at 0.5 m, below z_pass): it MUST be the boundary: d_block in the grid
//     cell that holds 3.0 m, h_block 0.5 m, FREE ending before it.
// The pair is the point. The contract's mutants: no overhead filter -- the first half goes red
// (the bar at 1.8 m becomes a boundary); every height filtered -- the second half goes red (the
// bar at 0.5 m is passed under). Either half alone would stay green under one of them.
void test_a19_prof4_bar_pair() {
    constexpr double kBarForwardM = 3.0, kBarHalfThicknessM = .05;
    constexpr std::size_t kLo = 240, kHi = 400;   // ray.x in [-0.25, 0.25]
    auto options = make_geometry_options();
    options.z_pass_m = 1.5;
    for (const double bar_height : {1.8, 0.5}) {
        const bool overhead = bar_height > options.z_pass_m;
        Fixture f(t_options(), options, forward_binding());
        std::size_t bar_pixels = 0;
        for (std::size_t y = 0; y < kNativeHeight; ++y)
            for (std::size_t x = kLo; x <= kHi; ++x) {
                const double height = 1.0 - kBarForwardM * f.optical_rays[at(x, y)].y;   // the bar is at depth 3.0
                if (std::abs(height - bar_height) > kBarHalfThicknessM) continue;
                f.raster[at(x, y)] = f.units(kBarForwardM); ++bar_pixels;
            }
        CHECK(bar_pixels > 500);
        f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
        const auto* body = f.builder.body();
        const auto [first, last] = interior_bins(f, kLo, kHi, kNativeHeight - 1);
        for (std::size_t i = first; i <= last; ++i) {
            const auto& b = body->bins[i];
            CHECK((b.src & kG) && !(b.src & kNegative));
            if (overhead) {
                // Passed under: no boundary, and FREE goes on behind the bar.
                CHECK(!b.d_block && !b.h_block);
                CHECK(b.d_free && *b.d_free > kBarForwardM + 2 * f.grid_options.dr_m);
            } else {
                // The bar is 3.0 m ahead; along an oblique bin its range is a little more.
                const double range = kBarForwardM * std::hypot(1.0, .25);
                CHECK(b.d_block && *b.d_block <= range && *b.d_block > kBarForwardM - f.grid_options.dr_m);
                CHECK(b.h_block && *b.h_block >= bar_height - kBarHalfThicknessM - 0.01 &&
                      *b.h_block <= bar_height + kBarHalfThicknessM + 0.01);
                CHECK(b.d_free && *b.d_free < *b.d_block);
            }
        }
        CHECK(last - first >= 20);
        // The reason behind the body, after it: the bar's returns are in the class the pair needs.
        std::size_t classified = 0;
        for (std::size_t y = 0; y < kNativeHeight; ++y) {
            const auto& pixel = f.geometry.pixels()[at(320, y)];
            if (std::abs(1.0 - kBarForwardM * f.optical_rays[at(320, y)].y - bar_height) > kBarHalfThicknessM) continue;
            CHECK(pixel.classification == (overhead ? SampleClass::Overhead : SampleClass::Obstacle));
            ++classified;
        }
        CHECK(classified >= 5);
        const auto put = f.publish(); CHECK(put.dispatched && put.publish);
    }
}

// A19-TIME-1 (19 section 14): one Profile is ONE frame -- every array of a body comes from the
// same depth frame. The generator gives each frame a marker that is the same in every bin and
// different in every frame (the contract's "a different colour per frame"): a wall across the
// whole view whose HEIGHT is the frame's colour; its distance changes with it. The colours are
// 7 cm apart and are read back within 3 cm; the distances are 0.6 m apart, more than the two
// cells FREE may end before a wall; and the frames do not come in order of distance. A bin that
// carries another frame's value -- the previous one or any other -- then cannot pass as this
// frame's. The walls stand beyond the region the ground plane is fitted in (2.9 m), so the fit
// sees level ground only. Asserted per frame, on the built body:
//   every bin with a boundary has THIS frame's colour in h_block and a d_block on this wall;
//   every bin of the centre columns has that boundary and FREE short of it, and in at least
//   eight of ten of them FREE ends within two cells of it.
// The contract's mutant: the per-bin post-processing reads the previous frame's evidence.
void test_a19_time1_one_frame_per_profile() {
    constexpr std::array<int, 5> kOrder{0, 3, 1, 4, 2};
    constexpr std::size_t kLo = 288, kHi = 352;   // ray.x in [-0.1, 0.1]: the wall's range is within 0.5% of its depth
    Fixture f(t_options(), make_geometry_options(), forward_binding());
    for (const int k : kOrder) {
        const double wall_m = 3.0 + 0.6 * k, colour_m = 0.18 + 0.07 * k;
        CHECK(colour_m > f.geometry_options.h_tol_m + 0.03 && colour_m < f.geometry_options.z_pass_m);
        CHECK(wall_m > f.geometry_options.fit_near_max_m && wall_m * 1.00499 < f.geometry_options.range_max_m);
        f.full_plane();
        std::size_t wall_pixels = 0;
        for (std::size_t i = 0; i < f.raster.size(); ++i) {
            const double height = 1.0 - wall_m * f.optical_rays[i].y;   // of the ray at the wall's depth
            if (height < 0 || height > colour_m) continue;               // ground before it, or over its top
            f.raster[i] = f.units(wall_m); ++wall_pixels;
        }
        CHECK(wall_pixels > 3000);
        f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
        const auto* body = f.builder.body();
        const auto [first, last] = interior_bins(f, kLo, kHi, kNativeHeight - 1);   // the bottom row is ground in every frame
        CHECK(last - first >= 16);
        std::size_t marked = 0, free_at_wall = 0;
        for (std::size_t i = 0; i < body->bins.size(); ++i) {
            const auto& b = body->bins[i];
            const bool centre = i >= first && i <= last;
            if (centre) CHECK((b.src & kG) && b.d_block && b.h_block && b.d_free);
            if (!b.d_block) continue;
            ++marked;
            // This frame's colour, in every bin that has a boundary at all (one row is at most 1.7 cm).
            CHECK(b.h_block && *b.h_block > colour_m - 0.03 && *b.h_block <= colour_m + 0.005);
            // On this frame's wall: a plane at depth wall_m, so its range along a bin is wall_m or
            // more, up to wall_m * sqrt(2) at the edge of the 90-degree view (where range_max cuts it off).
            CHECK(*b.d_block > wall_m - 0.005 && *b.d_block <= wall_m * 1.4143 + 0.005);
            CHECK(b.d_free && *b.d_free < *b.d_block);
            if (centre) {
                CHECK(*b.d_block <= wall_m * 1.00499);   // hypot(1, 0.1)
                free_at_wall += *b.d_free > *b.d_block - 2 * f.grid_options.dr_m - 0.005;
            }
        }
        CHECK(marked >= 100);   // the nearest wall is inside range_max over the whole view (200 bins), the farthest over 115
        // FREE reaches this frame's wall in the centre bins. Not asserted for every single bin:
        // FREE is coverage-gated, 5 m away a cell of a bin holds some ten rays, and a bin where one
        // of them is off its expected cell ends FREE earlier (2 of 23 bins with the wall at 5.4 m).
        CHECK(free_at_wall * 10 >= (last - first + 1) * 8);
        const auto put = f.publish(); CHECK(put.dispatched && put.publish);
    }
}

// A mask older than the age limit gives stale T and a geometry-only Profile; a mask with a
// mismatched stamp, clock epoch, depth index, material or projection revision builds nothing.
void test_age_and_malformed_masks() {
    Fixture f;
    f.advance(false); auto mask = f.raw_mask();
    mask.stamp.capture_mono_ns -= 50000001;
    f.process_t(&mask); CHECK(f.t.report().status == TStatus::Stale);
    CHECK(f.build() == NormalProfileStatus::Ready); check_body(f, false);
    CHECK(!f.builder.body()->t_seg_mono_ms && f.builder.report().t_bins == 0);
    CHECK(f.publish().publish);
    for (int mutation = 0; mutation < 5; ++mutation) {
        f.advance(false); mask = f.raw_mask();
        switch (mutation) {
        case 0: ++mask.stamp.capture_mono_ns; break;
        case 1: ++mask.stamp.clock_epoch; break;
        case 2: ++mask.stamp.source.depth_index; break;
        case 3: mask.material = MaskMaterial::FusedOrImageEroded; break;
        case 4: ++mask.projection_revision; break;
        }
        f.process_t(&mask);
        CHECK(f.t.report().status != TStatus::Fresh && f.t.report().status != TStatus::Missing &&
              f.t.report().status != TStatus::Stale);
        CHECK(f.build() != NormalProfileStatus::Ready); f.expect_no_dispatch();
    }
}

// Any mismatch in the capture identity refuses the build; T must come from the same frame as
// geometry; a new clock epoch on the same stream is an invalid identity, not a quiet rebase.
void test_full_identity_and_old_t() {
    Fixture f;
    for (int mutation = 0; mutation < 10; ++mutation) {
        f.advance(true, true); auto bad = f.capture;
        switch (mutation) {
        case 0: ++bad.source.stream_id; break;
        case 1: ++bad.source.sequence; break;
        case 2: ++bad.source.color_index; break;
        case 3: ++bad.source.depth_index; break;
        case 4: ++bad.source.color_device_us; break;
        case 5: ++bad.source.depth_device_us; break;
        case 6: ++bad.source.host_ready_mono_ns; break;
        case 7: ++bad.clock_epoch; break;
        case 8: bad.clock_epoch = 0; break;
        case 9: ++bad.capture_mono_ns; break;
        }
        CHECK(f.build(bad) != NormalProfileStatus::Ready); f.expect_no_dispatch();
    }
    f.advance(true, true); CHECK(f.build() == NormalProfileStatus::Ready);
    f.advance(false); // Geometric owners have a newer frame; T still old.
    CHECK(f.build() != NormalProfileStatus::Ready); f.expect_no_dispatch();
    auto mask = f.raw_mask(); f.process_t(&mask);
    CHECK(f.build() == NormalProfileStatus::Ready);
    CHECK(f.publish().publish);
    f.advance(false);
    ++f.capture.clock_epoch; // New clock epoch on the same active stream is NOT a quiet rebase.
    mask = f.raw_mask(); f.process_t(&mask);
    CHECK(f.build() == NormalProfileStatus::InvalidIdentity); f.expect_no_dispatch();
}

// A built Profile is not put if any upstream snapshot changed after the build or the stream ended
// or restarted; a failed fit builds nothing; good frames recover.
void test_upstream_change_stop_and_restart() {
    Fixture f;
    // Successful build never licenses a put after any upstream snapshot changed.
    for (int mutation = 0; mutation < 4; ++mutation) {
        f.advance(true, true); CHECK(f.build() == NormalProfileStatus::Ready);
        if (mutation == 0) { f.geometry.process(f.frame, f.mailbox); ++geometry_calls; }
        if (mutation == 1) { f.grid.process(f.mailbox); ++grid_calls; }
        if (mutation == 2) f.process_t(nullptr); // same-frame replay clears its report
        if (mutation == 3) f.advance(false);
        f.expect_no_dispatch();
    }
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready);
    f.mailbox.end_stream(); f.expect_no_dispatch();
    f.restart(); f.advance(); CHECK(f.build() == NormalProfileStatus::Ready);
    f.restart(); f.expect_no_dispatch();
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready); CHECK(f.publish().publish);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    f.advance(); CHECK(f.geometry.result().status == GeometryStatus::FitFailed);
    CHECK(f.build() != NormalProfileStatus::Ready); f.expect_no_dispatch();
    f.full_plane(); f.advance(); CHECK(f.build() == NormalProfileStatus::Ready);
    CHECK(f.publish().publish);
}

// Owners from another chain are refused; each frame is put at most once, even after a rejected,
// unknown or capacity-failed put; the header never claims a higher overhead threshold than used,
// and unrepresentable angle cells or thresholds are refused.
void test_owners_and_once_only_puts() {
    Fixture f;
    NormalGrid other_grid(f.geometry, f.grid_options);
    bool rejected = false;
    try { NormalProfileBuilder bad(other_grid, f.t); }
    catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready);
    CHECK(f.publish().publish); f.expect_no_dispatch();
    for (const auto outcome : {PutOutcome::Rejected, PutOutcome::Unknown}) {
        f.advance(); CHECK(f.build() == NormalProfileStatus::Ready);
        f.sink.outcome = outcome;
        const auto result = f.publish(); CHECK(result.dispatched && !result.publish);
        CHECK(result.publish.error == (outcome == PutOutcome::Rejected ? PublishError::PutRejected : PublishError::PutUnknown));
        f.expect_no_dispatch();
        CHECK(f.build() == NormalProfileStatus::InactiveOrStale); f.expect_no_dispatch();
    }
    // Even a transport preflight failure cannot be retried as a new frame.
    f.advance(); CHECK(f.build() == NormalProfileStatus::Ready);
    PublisherChannel too_small(Topic::Profile, f.envelope, f.clock, f.sink, 2, 2);
    const auto calls = f.sink.calls;
    const auto result = f.builder.publish(f.mailbox, too_small); ++publish_calls;
    CHECK(result.dispatched && result.publish.error == PublishError::Capacity && f.sink.calls == calls);
    f.expect_no_dispatch();
    // The header may not claim an overhead threshold above that actually used
    // for point filtering. This value rounds upward with a plain float cast.
    auto go = make_geometry_options(); go.z_pass_m = static_cast<double>(.75F) - 1e-9;
    CHECK(static_cast<double>(static_cast<float>(go.z_pass_m)) > go.z_pass_m);
    Fixture threshold(t_options(), go);
    threshold.advance(); CHECK(threshold.build() == NormalProfileStatus::Ready);
    check_body(threshold, false);
    CHECK(threshold.builder.body()->z_pass_m < .75F);
    // Nonrepresentable angular cells cannot silently move at serialization.
    go = make_geometry_options(); go.angle_min_rad = -.1; go.angle_step_rad = .1; go.n_bins = 3;
    rejected = false;
    try { Fixture bad(t_options(), go); }
    catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
    go = make_geometry_options(); go.z_pass_m = 1e300;
    rejected = false;
    try { Fixture bad(t_options(), go); }
    catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
}

// One bin's evidence maps exactly to the wire (confidence, source, slope, heights); float rounding
// never closes the gap between FREE and the block and drops FREE once the rounded blind edge
// overtakes it; inconsistent inputs give an empty bin.
void test_pure_mapping_and_precision() {
    GridBinEvidence e; e.valid_returns = 1; e.expected_rays = 3;
    BinWalkResult r; r.valid = true; r.stop = StopReason::Range;
    r.free_end_m = 2; r.passed_cells = 2; r.slope_rad = pi / 12;
    Bin out;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out));
    CHECK(out.src == kG && out.conf == 85 && out.terrain == 0 && out.d_free == 2 && out.slope_deg == 15);
    e.expected_rays = 0;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out) && out.conf == 0);
    e.valid_returns = 99; e.expected_rays = 1;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out) && out.conf == 255);
    e.valid_returns = 0; e.valid_pierce_rays = 3; e.expected_rays = 3;
    r = {}; r.valid = true; r.stop = StopReason::Boundary; r.boundary = BoundaryKind::Negative;
    r.block_m = 1.5;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out));
    CHECK(out.src == (kG | kNegative) && out.conf == 0 && out.d_block == 1.5F && !out.h_block);
    r.block_height_m = -.25;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out) && out.h_block == -.25F);
    r.boundary = BoundaryKind::Positive; r.block_height_m = .25;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out));
    CHECK(out.src == kG && out.h_block == .25F && !out.d_free);
    e = {}; r = {}; r.valid = true; r.stop = StopReason::NoSamples;
    CHECK(normal_profile_bin(e, r, .5, 6, false, out) && empty(out));

    // Distinct FP64 geometric boundaries both round to FP32 1.0. The wire
    // must keep an UNKNOWN gap, never clamp the FREE endpoint to BLOCKED.
    e.valid_returns = e.expected_rays = 1;
    r = {}; r.valid = true; r.stop = StopReason::Boundary; r.boundary = BoundaryKind::Positive;
    r.free_end_m = 1.0 + 1e-8; r.block_m = 1.0 + 2e-8; r.block_height_m = .2;
    r.passed_cells = 2; r.t_backed_prefix = true; r.slope_rad = .001;
    CHECK(static_cast<float>(*r.free_end_m) == static_cast<float>(*r.block_m));
    CHECK(normal_profile_bin(e, r, .5, 6, true, out));
    CHECK(out.d_free && out.d_block && *out.d_free < *out.d_block);
    CHECK(*out.d_free <= *r.free_end_m && *out.d_block <= *r.block_m && (out.src & kT));
    CHECK(out.slope_deg == 1);
    // Now the FP32 outward blind overtakes the representable safe FREE value.
    CHECK(normal_profile_bin(e, r, 1.0 + 5e-9, 6, true, out));
    CHECK(!out.d_free && !(out.src & kT) && !out.slope_deg && out.d_block && out.h_block);

    for (int mutation = 0; mutation < 11; ++mutation) {
        auto bad_e = e; auto bad_r = r; double blind = .5, maximum = 6;
        switch (mutation) {
        case 0: bad_e.valid_returns = kGeometryPixels + 1; break;
        case 1: bad_e.expected_rays = kGeometryPixels + 1; break;
        case 2: bad_r.block_height_m = -.2; break;
        case 3: bad_r.block_m = *bad_r.free_end_m; break;
        case 4: bad_r.slope_rad = std::numeric_limits<double>::quiet_NaN(); break;
        case 5: bad_r.boundary = BoundaryKind::None; break;
        case 6: bad_r.valid = false; break;
        case 7: bad_r.passed_cells = kMaxRadialCells + 1; break;
        case 8: bad_e.valid_returns = 0; break;
        case 9: blind = std::numeric_limits<double>::quiet_NaN(); break;
        case 10: maximum = 7; break;
        }
        out.d_free = 3; out.src = 15; out.terrain = 4;
        CHECK(!normal_profile_bin(bad_e, bad_r, blind, maximum, true, out) && empty(out));
    }
    r.t_backed_prefix = false;
    CHECK(!normal_profile_bin(e, r, .5, 6, true, out) && empty(out));
}

// A no-ground bin maps to the measured positive block only; any FREE, slope, T or negative-block
// claim is refused and empties the bin.
void test_no_ground_pure_mapping() {
    GridBinEvidence e; e.valid_returns = 1; e.expected_rays = 3;
    BinWalkResult r; r.valid = true; r.stop = StopReason::NoGroundReference;
    r.boundary = BoundaryKind::Positive; r.block_m = 1.0 + 2e-8; r.block_height_m = .25;
    Bin out;
    CHECK(no_ground_profile_bin(e, r, 6, out));
    CHECK(out.src == kG && out.conf == 85 && out.d_block == 1 && out.h_block == .25F);
    CHECK(!out.d_free && !out.slope_deg && out.terrain == 0);
    for (int mutation = 0; mutation < 15; ++mutation) {
        auto bad_e = e; auto bad_r = r; double maximum = 6;
        switch (mutation) {
        case 0: bad_r.free_end_m = 0; break; // even zero-length FREE is forbidden
        case 1: bad_r.passed_cells = 1; break;
        case 2: bad_r.t_backed_prefix = true; break;
        case 3: bad_r.slope_rad = 0; break;
        case 4: bad_r.boundary = BoundaryKind::Negative; break;
        case 5: bad_r.block_height_m = -.1; break;
        case 6: bad_r.block_height_m.reset(); break;
        case 7: bad_r.block_m.reset(); break;
        case 8: bad_r.stop = StopReason::Range; break;
        case 9: bad_r.valid = false; break;
        case 10: bad_e.valid_returns = kGeometryPixels + 1; break;
        case 11: bad_e.valid_pierce_rays = kGeometryPixels + 1; break;
        case 12: bad_e.expected_rays = kGeometryPixels + 1; break;
        case 13: maximum = std::numeric_limits<double>::quiet_NaN(); break;
        case 14: bad_e.valid_returns = 0; bad_e.valid_pierce_rays = 1; break;
        }
        out.d_free = 3; out.src = 15; out.terrain = 4;
        CHECK(!no_ground_profile_bin(bad_e, bad_r, maximum, out) && empty(out));
    }
    e = {}; r = {}; r.valid = true; r.stop = StopReason::NoSamples;
    CHECK(no_ground_profile_bin(e, r, 6, out) && empty(out));
    e.valid_pierce_rays = 1; r.stop = StopReason::NoGroundReference;
    CHECK(no_ground_profile_bin(e, r, 6, out) && out.src == kG && !out.d_block && !out.h_block);
}
}

int main(int argc, char** argv) {
    const bool fixtures = argc == 2 && std::string_view(argv[1]) == "--fixtures";
    test_normal_missing_and_fresh(fixtures);
    test_fresh_false_and_positive_margin(fixtures);
    test_positive_and_negative_association(fixtures);
    if (fixtures) return 0;
    test_age_and_malformed_masks(); test_full_identity_and_old_t();
    test_upstream_change_stop_and_restart(); test_owners_and_once_only_puts();
    test_pure_mapping_and_precision();
    test_explicit_geometry_only();
    test_no_ground_reference_geometry_only(); test_range_edge_ground_reference(); test_no_ground_pure_mapping();
    test_fast_azimuth_and_fallback();
    test_a19_neg1_pit_scene();
    test_a19_prof2_glass_hole(); test_a19_prof3_thin_pole(); test_a19_prof4_bar_pair();
    test_a19_time1_one_frame_per_profile();
    std::cout << "normal Profile host integration 18/18 groups PASS (incl. the A19-NEG-1, A19-PROF-2, A19-PROF-3 and "
                 "A19-PROF-4 scene goldens and the A19-TIME-1 frame-marker sequence); geometry="
              << geometry_calls
              << " grid=" << grid_calls << " T=" << t_calls << " build=" << build_calls
              << " publish=" << publish_calls << " strict-JSON packets=" << strict_packets
              << "; measured hot paths ordinary/aligned C++ new delta=0; synthetic, NOT deployment\n";
}
