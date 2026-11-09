/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_withdrawn_evidence.cpp
 * Brief: Unit tests for the fit-failed (withdrawn) Profile evidence and its Profile/Status publication.
 *
 * Description:
 * What it checks. When the ground fit fails, 19 section 3.2A withdraws FREE and keeps only
 * independent evidence. The tests cover failed versus normal frames; absent, stale and excluded
 * semantic (S) input; guards such as the explicit geometry-only overload not bypassing an already
 * configured S; invalid sources; raw depth support and its independence from S; mixed provenance;
 * the withdrawn Status put; the three serialized parser fixtures; rejection and ownership;
 * upstream lifecycle (a failed rebuild invalidates the built Profile); freshness, clock and the
 * commit instant.
 * Trap. The default is closed: nothing is published from evidence that was not rebuilt for this
 * frame.
 */

/* Reuse only the sealed synthetic native-depth fixture; no SDK/network. */
#define main normal_profile_fixture_unused_main
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#include "test_rns_normal_profile.cpp"
#pragma GCC diagnostic pop
#undef main
#include "output/rns_withdrawn_evidence.hpp"
#include "output/rns_withdrawn_profile.hpp"
#include "output/rns_status_heartbeat.hpp"

namespace {
std::size_t semantic_count = 0, withdrawn_count = 0, withdrawn_profile_builds = 0, withdrawn_profile_puts = 0;
struct W {
    Fixture f;
    SemanticOptions options{0, 0, 0, 50000000, 77};
    SemanticProjector s{f.geometry, options, SemanticDepthPolicy::NormalOrFitFailed};
    WithdrawnSemanticEvidence w{s};
    SemanticObject object;
    SemanticFrame frame;
    std::int64_t now = -1;
    W() {
        object.track_id = 5; object.class_id = 8; object.class_name = "barrier";
        object.plane_basis = SemanticPlaneBasis::IndependentMeasurement;
        object.vertices = 4;
        object.footprint[0] = {.6F, -.08F}; object.footprint[1] = {.7F, -.08F};
        object.footprint[2] = {.7F, .08F}; object.footprint[3] = {.6F, .08F};
    }
    void advance(bool no_depth = true) {
        if (no_depth) std::fill(f.raster.begin(), f.raster.end(), 0);
        else f.full_plane();
        f.advance(); now = f.capture.capture_mono_ns + 1000000;
        frame = {f.capture, 77, SemanticMaterial::MeasuredBaseFootprint, &object, 1};
    }
    void advance_sparse() {
        std::fill(f.raster.begin(), f.raster.end(), 0);
        // Too few samples for a plane, but real native measurements remain.
        f.raster[at(320, 64)] = 1024;
        f.raster[at(321, 64)] = 1024;
        f.advance(); now = f.capture.capture_mono_ns + 1000000;
        frame = {f.capture, 77, SemanticMaterial::MeasuredBaseFootprint, &object, 1};
    }
    SemanticStatus process(const SemanticFrame* input) {
        const auto before = allocations.load();
        const auto r = s.process(f.mailbox, f.capture, now, input).status;
        ++semantic_count; CHECK(allocations.load() == before); return r;
    }
    WithdrawnStatus build() {
        const auto before = allocations.load();
        const auto r = w.build(f.mailbox, now).status;
        ++withdrawn_count; CHECK(allocations.load() == before); return r;
    }
    void check(bool present) {
        const auto before = allocations.load();
        const auto* bins = w.bins(f.mailbox, now);
        CHECK(allocations.load() == before && bins);
        CHECK(bins->size() == f.geometry_options.n_bins);
        std::size_t blocks = 0, raw_bins = 0;
        std::uint64_t raw_samples = 0;
        for (std::size_t i = 0; i < bins->size(); ++i) {
            const auto& b = (*bins)[i];
            CHECK(!b.d_free && !b.h_block && !b.slope_deg && b.conf == 0 && b.terrain == 0);
            CHECK((b.src & (kT | kNegative)) == 0);
            const auto raw = f.geometry.raw_sample_counts()[i];
            CHECK(((b.src & kG) != 0) == (raw > 0));
            raw_bins += raw > 0; raw_samples += raw;
            if (b.d_block) {
                ++blocks; CHECK((b.src & kS) != 0 && s.distances()[i]);
                CHECK(*b.d_block >= 0 && *b.d_block <= *s.distances()[i]);
            } else CHECK(b.src == (raw ? kG : 0));
        }
        CHECK((blocks > 0) == present && blocks == w.report().semantic_bins);
        CHECK(raw_bins == w.report().raw_sample_bins && raw_samples == w.report().raw_samples);
        CHECK(w.report().free_withdrawn);
        CHECK(w.report().reasons[0] == "ground_fit_fallback");
        CHECK(w.report().reasons[1] == "ground_free_withdrawn");
        // No complete Profile is manufactured from these partial bin facts.
        CHECK(f.build() == NormalProfileStatus::GeometryUnavailable);
        f.expect_no_dispatch();
    }
};
void failed_and_normal() {
    W x; x.advance();
    CHECK(x.f.geometry.result().status == GeometryStatus::FitFailed);
    CHECK(x.f.geometry.result().valid_depth_pixels == 0);
    CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(true);
    CHECK(x.build() == WithdrawnStatus::InvalidSource && !x.w.bins(x.f.mailbox, x.now));
    x.advance(false); CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.build() == WithdrawnStatus::InvalidSource && !x.w.bins(x.f.mailbox, x.now));
    x.advance(); CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(true);
}
void absent_stale_and_excluded() {
    W x; x.advance(); const auto old = x.frame;
    CHECK(x.process(nullptr) == SemanticStatus::Missing);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(false);
    for (int i = 0; i < 6; ++i) x.advance();
    CHECK(x.process(&old) == SemanticStatus::Stale);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(false);
    x.advance(); x.object.class_name = "traversable_area";
    CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.s.report().excluded_t_objects == 1);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(false);
}
void guards() {
    W x; x.advance();
    // The explicit G-only overload must not bypass an already configured S
    // owner, even on a real FitFailed frame with a valid capture stamp.
    CHECK(x.w.build(x.f.mailbox, x.f.capture, x.now).status == WithdrawnStatus::InvalidSource);
    CHECK(!x.w.bins(x.f.mailbox, x.now));
    // Geometry from A cannot be consumed via B merely because both allocated
    // generation 1. A rejected first call must not bind the projector to B.
    nd::Mailbox first_wrong{kGeometryPixels}; first_wrong.begin_stream();
    CHECK(x.s.process(first_wrong, x.f.capture, x.now, &x.frame).status == SemanticStatus::InvalidIdentity);
    CHECK(!x.s.bound_to(first_wrong) && !x.s.bound_to(x.f.mailbox));
    CHECK(x.w.build(first_wrong, x.now).status == WithdrawnStatus::InvalidSource);
    CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    // Two mailbox objects can share numeric stream 1. The FIRST build must
    // reject the wrong owner, not bind to it based only on a stream number.
    nd::Mailbox other{kGeometryPixels}; other.begin_stream();
    CHECK(!x.s.bound_to(other) && x.s.bound_to(x.f.mailbox));
    CHECK(x.w.build(other, x.now).status == WithdrawnStatus::InvalidSource);
    CHECK(!x.w.bins(other, x.now));
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(x.w.bins(x.f.mailbox, x.f.capture.capture_mono_ns + 50000000));
    CHECK(!x.w.bins(x.f.mailbox, x.f.capture.capture_mono_ns + 50000001));
    CHECK(!x.w.bins(x.f.mailbox, x.now - 1));
    CHECK(!x.w.bins(x.f.mailbox, -1));
    CHECK(!x.w.bins(other, x.now));
    x.process(nullptr); // rejected reprocess must invalidate previously built evidence
    CHECK(!x.w.bins(x.f.mailbox, x.now));
    x.advance(); CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    x.f.mailbox.end_stream(); CHECK(!x.w.bins(x.f.mailbox, x.now));
    x.f.restart(); x.advance();
    CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(true);
    WithdrawnProfileBuilder ready(x.w);
    CHECK(ready.build(x.f.mailbox, x.now).status == WithdrawnProfileStatus::Ready && ready.body());
    CHECK(x.w.build(x.f.mailbox, x.f.capture, x.now).status == WithdrawnStatus::InvalidSource);
    CHECK(!x.w.bins(x.f.mailbox, x.now));
    CHECK(!ready.publish(x.f.mailbox, x.f.channel, x.now).dispatched && !ready.body());
    x.advance(); CHECK(!x.w.bins(x.f.mailbox, x.now)); // new geometry without process S
}
void invalid_sources() {
    W x;
    for (int variant = 0; variant < 3; ++variant) {
        x.advance(); auto bad = x.frame;
        if (variant == 0) ++bad.base_revision;
        if (variant == 1) bad.stamp.capture_mono_ns = x.now + 1;
        if (variant == 2) bad.material = SemanticMaterial::ImageBox;
        if (variant == 3) ++bad.stamp.source.depth_device_us;
        CHECK(x.process(&bad) != SemanticStatus::Fresh);
        CHECK(x.build() != WithdrawnStatus::ReadyEvidenceOnly);
        CHECK(!x.w.bins(x.f.mailbox, x.now));
    }
    x.f.projection_binding.unit_m *= 2; x.advance();
    CHECK(x.f.geometry.result().status == GeometryStatus::ProfileMismatch);
    CHECK(x.process(&x.frame) == SemanticStatus::InvalidInput);
    CHECK(x.build() == WithdrawnStatus::InvalidSource);
}
void normal_owner_guard() {
    W x; NormalProfileBuilder normal{x.f.grid, x.f.t, x.s};
    x.advance(false); CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    nd::Mailbox other{kGeometryPixels}; other.begin_stream();
    CHECK(normal.build(other, x.f.capture).status == NormalProfileStatus::InvalidIdentity);
    CHECK(!normal.body());
    CHECK(normal.build(x.f.mailbox, x.f.capture).status == NormalProfileStatus::Ready);
    CHECK(!normal.publish(other, x.f.channel, x.now).dispatched);
    CHECK(!normal.body() && x.f.sink.calls == 0);
}
void default_is_closed() {
    W x; SemanticProjector legacy{x.f.geometry, x.options};
    WithdrawnSemanticEvidence closed{legacy}; x.advance();
    CHECK(legacy.process(x.f.mailbox, x.f.capture, x.now, &x.frame).status == SemanticStatus::InvalidInput);
    CHECK(closed.build(x.f.mailbox, x.now).status == WithdrawnStatus::InvalidSemantic);
    CHECK(!closed.bins(x.f.mailbox, x.now));
    bool threw = false;
    try { SemanticProjector bad{x.f.geometry, x.options, static_cast<SemanticDepthPolicy>(9)}; }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
}
void raw_support_and_independence() {
    W x;
    for (const auto basis : {SemanticPlaneBasis::Unknown, SemanticPlaneBasis::GroundPlaneDependent,
                             SemanticPlaneBasis::IndependentMeasurement}) {
        x.advance_sparse(); x.object.plane_basis = basis;
        CHECK(x.f.geometry.result().status == GeometryStatus::FitFailed);
        CHECK(x.f.geometry.result().has_raw_sample_evidence());
        CHECK(!x.f.geometry.result().has_evidence());
        CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
        const bool independent = basis == SemanticPlaneBasis::IndependentMeasurement;
        CHECK(x.s.report().excluded_plane_basis_objects == (independent ? 0U : 1U));
        CHECK(x.s.report().projected_objects == (independent ? 1U : 0U));
        CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(independent);
        CHECK(x.w.report().raw_samples == 2 && x.w.report().raw_sample_bins > 0);
    }
    x.advance_sparse(); CHECK(x.process(nullptr) == SemanticStatus::Missing);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(false);
    CHECK(x.w.report().raw_samples == 2);
    x.advance(); CHECK(x.process(nullptr) == SemanticStatus::Missing);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(false);
    CHECK(x.w.report().raw_samples == 0 && x.w.report().raw_sample_bins == 0);
}
void mixed_provenance() {
    W x; x.advance_sparse();
    std::array<SemanticObject, 3> objects{x.object, x.object, x.object};
    objects[1].track_id = 6; objects[1].plane_basis = SemanticPlaneBasis::GroundPlaneDependent;
    objects[2].track_id = 7; objects[2].plane_basis = SemanticPlaneBasis::Unknown;
    // Excluded nearer boxes must not shorten the independent object's boundary.
    for (std::size_t k = 1; k < objects.size(); ++k)
        for (std::size_t v = 0; v < objects[k].vertices; ++v) objects[k].footprint[v].x *= .5F;
    x.frame.objects = objects.data(); x.frame.count = objects.size();
    CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.s.report().object_count == 3 && x.s.report().projected_objects == 1);
    CHECK(x.s.report().excluded_plane_basis_objects == 2);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(true);
    for (const auto& d : x.s.distances()) if (d) CHECK(*d >= .59);
    x.advance_sparse(); x.object.plane_basis = static_cast<SemanticPlaneBasis>(99);
    CHECK(x.process(&x.frame) == SemanticStatus::InvalidInput);
    CHECK(x.build() == WithdrawnStatus::InvalidSemantic);
    CHECK(!x.w.bins(x.f.mailbox, x.now));
}
void withdrawn_status_put(bool emit) {
    W x; x.advance_sparse();
    CHECK(x.process(&x.frame) == SemanticStatus::Fresh);
    CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly); x.check(true);
    // Explicit synthetic metrics snapshot: this tests reason/packet wiring,
    // NOT real health provenance or the pending missing-status-value policy.
    Status status;
    status.fps_depth = 30; status.fps_infer = 25;
    status.latency_ms_p50 = 40; status.latency_ms_p99 = 60;
    status.infer_gap_ms_p99 = 40; status.infer_gap_ms_max = 45;
    status.invalid_pixel_ratio = static_cast<double>(kGeometryPixels - 2) / kGeometryPixels;
    status.ground_seg_level = 0; status.extrinsic_calibrated = true;
    for (const auto reason : x.w.report().reasons) status.degraded_reasons.emplace_back(reason);
    PublisherChannel channel{Topic::Status, x.f.envelope, x.f.clock, x.f.sink, 4096, 8192};
    const auto observed = x.now / 1000000;
    StatusHeartbeat heartbeat{channel, observed - 1000, 10};
    x.f.clock.mono = observed;
    CHECK(x.w.bins(x.f.mailbox, x.now));
    const auto before = allocations.load();
    const auto result = heartbeat.poll(observed, &status, observed);
    CHECK(allocations.load() == before && result.code == HeartbeatCode::Published);
    CHECK(x.f.sink.calls == 1 && heartbeat.counters().put_attempts == 1);
    const std::string_view packet{x.f.sink.packet.data(), x.f.sink.size};
    CHECK(packet.find("ground_fit_fallback") != std::string_view::npos);
    CHECK(packet.find("ground_free_withdrawn") != std::string_view::npos);
    CHECK(packet.find("\"d_free\":") == std::string_view::npos);
    if (emit) {
        std::cout << "{\"case\":\"withdrawn_dual_reason_status\",\"packet\":" << packet
                  << ",\"profile_puts\":0,\"status_puts\":1,\"raw_samples\":" << x.w.report().raw_samples
                  << ",\"scope\":\"synthetic metrics, injected local put; no complete withdrawn Profile\"}\n";
    }
}
struct P {
    W x;
    WithdrawnProfileBuilder p{x.w};
    WithdrawnProfileStatus build() {
        const auto before = allocations.load();
        const auto status = p.build(x.f.mailbox, x.now).status;
        ++withdrawn_profile_builds; CHECK(allocations.load() == before); return status;
    }
    WithdrawnProfilePut publish(std::int64_t now) {
        x.f.clock.mono = now / 1000000;
        const auto before = allocations.load();
        const auto result = p.publish(x.f.mailbox, x.f.channel, now);
        ++withdrawn_profile_puts; CHECK(allocations.load() == before); return result;
    }
    void ready(bool sparse = true, bool semantic = true) {
        if (sparse) x.advance_sparse(); else x.advance();
        CHECK(x.process(semantic ? &x.frame : nullptr) == (semantic ? SemanticStatus::Fresh : SemanticStatus::Missing));
        CHECK(x.build() == WithdrawnStatus::ReadyEvidenceOnly);
        CHECK(build() == WithdrawnProfileStatus::Ready);
    }
    void check() {
        const auto* body = p.body(); const auto* bins = x.w.bins(x.f.mailbox, x.now);
        CHECK(body && bins && &x.w.geometry() == &x.f.geometry);
        CHECK(!body->blind_near_m && !body->t_seg_mono_ms && !body->extrinsic_calibrated);
        CHECK(body->t_capture_mono_ms == x.f.capture.capture_mono_ns / 1000000);
        CHECK(body->t_publish_mono_ms == -1 && body->range_max_m <= x.f.geometry_options.range_max_m);
        CHECK(body->z_pass_m <= x.f.geometry_options.z_pass_m);
        CHECK(body->angle_min_rad == x.f.geometry_options.angle_min_rad &&
              body->angle_step_rad == x.f.geometry_options.angle_step_rad);
        CHECK(body->bins.size() == bins->size() && p.storage_bytes() == bins->size() * sizeof(Bin));
        for (std::size_t i = 0; i < bins->size(); ++i) {
            const auto& b = body->bins[i]; const auto& source = (*bins)[i];
            CHECK(!b.d_free && !b.h_block && !b.slope_deg && !b.conf && !b.terrain);
            CHECK(!(b.src & (kT | kNegative)) && b.src == source.src && b.d_block == source.d_block);
        }
        CHECK(p.report().evidence.raw_samples == x.w.report().raw_samples);
        CHECK(p.report().evidence.free_withdrawn && p.report().evidence.reasons == x.w.report().reasons);
    }
    void no_put() {
        const auto calls = x.f.sink.calls;
        CHECK(!publish(x.now).dispatched && x.f.sink.calls == calls && !p.body());
    }
};
void profile_fixtures(bool emit) {
    // These are exactly the three serialized parser fixtures. Additional
    // all-invalid+S / expired-S coverage lives in profile_current_stale_semantic.
    for (int variant = 0; variant < 3; ++variant) {
        P a; CHECK(!a.p.body() && a.p.report().status == WithdrawnProfileStatus::NotBuilt);
        a.ready(variant != 0, variant == 2); a.check();
        CHECK(a.x.w.report().raw_samples == (variant ? 2U : 0U));
        CHECK((a.x.w.report().semantic_bins > 0) == (variant == 2));
        const auto result = a.publish(a.x.now);
        CHECK(result.dispatched && result.publish && a.x.f.sink.calls == 1);
        CHECK(a.x.f.sink.qos.priority == Priority::RealTime && !a.x.f.sink.qos.reliable &&
              a.x.f.sink.qos.express && a.x.f.sink.qos.drop);
        const std::string_view packet{a.x.f.sink.packet.data(), a.x.f.sink.size};
        CHECK(packet.find("\"blind_near_m\":null") != std::string_view::npos);
        CHECK(packet.find("\"t_seg_mono_ms\":null") != std::string_view::npos);
        if (emit) {
            const char* name = variant == 0 ? "withdrawn_empty" :
                variant == 1 ? "withdrawn_raw" : "withdrawn_raw_independent_s";
            std::cout << "{\"case\":\"" << name << "\",\"key\":\"" << a.x.f.sink.key.data()
                      << "\",\"packet\":" << packet << ",\"capture\":" << result.publish.audit.capture_ms
                      << ",\"before\":" << result.publish.audit.put_before_ms
                      << ",\"after\":" << result.publish.audit.put_return_ms
                      << ",\"profile_puts\":1,\"raw_samples\":" << a.x.w.report().raw_samples
                      << ",\"raw_sample_bins\":" << a.x.w.report().raw_sample_bins
                      << ",\"semantic_bins\":" << a.x.w.report().semantic_bins
                      << ",\"free_withdrawn\":true,\"source_sequence\":" << a.x.f.capture.source.sequence
                      << ",\"clock_epoch\":" << a.x.f.capture.clock_epoch << "}\n";
        }
        a.no_put(); // no duplicate even after the successful first put
        CHECK(a.build() != WithdrawnProfileStatus::Ready); a.no_put();
    }
}
void profile_reject_and_owner() {
    P a; a.ready();
    a.x.f.sink.outcome = PutOutcome::Rejected;
    const auto result = a.publish(a.x.now);
    CHECK(result.dispatched && result.publish.error == PublishError::PutRejected && a.x.f.sink.calls == 1);
    a.x.f.sink.outcome = PutOutcome::Accepted; a.no_put();
    CHECK(a.build() != WithdrawnProfileStatus::Ready); a.no_put();
    a.ready(); a.x.f.sink.outcome = PutOutcome::Unknown;
    const auto unknown = a.publish(a.x.now);
    CHECK(unknown.dispatched && unknown.publish.error == PublishError::PutUnknown && a.x.f.sink.calls == 2);
    a.x.f.sink.outcome = PutOutcome::Accepted; a.no_put();
    CHECK(a.build() != WithdrawnProfileStatus::Ready); a.no_put();
    a.ready();
    nd::Mailbox other{kGeometryPixels}; other.begin_stream();
    const auto before = allocations.load();
    CHECK(!a.p.publish(other, a.x.f.channel, a.x.now).dispatched && !a.p.body());
    CHECK(allocations.load() == before && a.x.f.sink.calls == 2);
    a.no_put();
    P first; first.x.advance_sparse();
    CHECK(first.x.process(nullptr) == SemanticStatus::Missing);
    CHECK(first.x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(first.p.build(other, first.x.now).status == WithdrawnProfileStatus::InvalidSource);
    CHECK(first.build() == WithdrawnProfileStatus::Ready); first.check();
}
void profile_upstream_and_lifecycle() {
    P a; a.ready();
    // Even a failed rebuild of evidence invalidates the built Profile.
    CHECK(a.x.build() == WithdrawnStatus::InvalidSource); a.no_put();
    a.ready();
    a.x.process(nullptr); // same-frame semantic reprocessing invalidates its revision
    a.no_put();
    a.ready(); a.x.advance_sparse();
    CHECK(a.x.process(&a.x.frame) == SemanticStatus::Fresh);
    CHECK(a.x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    a.no_put(); // a successful evidence rebuild for the next source is also not the snapshot
    CHECK(a.build() == WithdrawnProfileStatus::Ready);
    CHECK(a.publish(a.x.now).publish);
    a.ready(); a.x.advance_sparse(); // new native geometry without new S/evidence
    a.no_put();
    CHECK(a.x.process(&a.x.frame) == SemanticStatus::Fresh);
    CHECK(a.x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(a.build() == WithdrawnProfileStatus::Ready);
    a.x.f.mailbox.end_stream(); a.no_put();
    a.x.f.restart(); a.ready(); a.check();
    CHECK(a.publish(a.x.now).publish);
}
void profile_freshness_and_clock() {
    P a; a.ready();
    CHECK(a.publish(a.x.f.capture.capture_mono_ns + 50000000).publish);
    a.ready();
    const auto calls = a.x.f.sink.calls;
    CHECK(!a.publish(a.x.f.capture.capture_mono_ns + 50000001).dispatched);
    CHECK(!a.p.body() && a.x.f.sink.calls == calls); a.no_put();
    a.ready(); CHECK(!a.publish(a.x.now - 1).dispatched); a.no_put();
    a.ready(); CHECK(!a.publish(-1).dispatched); a.no_put();
    a.ready();
    a.x.advance_sparse(); ++a.x.f.capture.clock_epoch; a.x.frame.stamp = a.x.f.capture;
    CHECK(a.x.process(&a.x.frame) == SemanticStatus::InvalidIdentity);
    CHECK(a.x.build() != WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(a.build() != WithdrawnProfileStatus::Ready); a.no_put();
    // A new stream may explicitly change the clock epoch, but timestamps must
    // still remain increasing within this process publisher's MONOTONIC clock.
    a.x.f.restart(); a.x.advance_sparse(); ++a.x.f.capture.clock_epoch; a.x.frame.stamp = a.x.f.capture;
    CHECK(a.x.process(&a.x.frame) == SemanticStatus::Fresh);
    CHECK(a.x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(a.build() == WithdrawnProfileStatus::Ready); a.check();
    CHECK(a.publish(a.x.now).publish);
}
// Reply batch 7, B7-1: a withdrawn body with S bins keeps its S lifetime at the
// ACTUAL commit instant too. The caller's entry time is still valid, the
// channel's own clock at put-before is not: nothing may reach the transport.
void profile_commit_instant() {
    const std::int64_t kMaxAge = 50000000;
    {
        P a; a.ready(false, true);
        CHECK(a.p.report().evidence.semantic_bins > 0);
        const auto t_obj = a.x.frame.stamp.capture_mono_ns;
        a.x.f.clock.mono = (t_obj + kMaxAge + 10000000) / 1000000;   // commit: 60 ms old
        const auto calls = a.x.f.sink.calls;
        const auto r = a.p.publish(a.x.f.mailbox, a.x.f.channel, t_obj + 40000000);  // entry: 40 ms old
        ++withdrawn_profile_puts;
        CHECK(r.dispatched && !r.publish && r.publish.error == PublishError::SnapshotInvalid);
        CHECK(!r.publish.audit.attempted && a.x.f.sink.calls == calls);
    }
    {   // the same body with a valid commit instant is put
        P a; a.ready(false, true);
        const auto t_obj = a.x.frame.stamp.capture_mono_ns;
        a.x.f.clock.mono = (t_obj + 40000000) / 1000000;
        const auto r = a.p.publish(a.x.f.mailbox, a.x.f.channel, t_obj + 40000000);
        ++withdrawn_profile_puts;
        CHECK(r.dispatched && r.publish);
    }
    {   // raw support only (no S bins): no lifetime, no guard
        P a; a.ready(true, false);
        CHECK(a.p.report().evidence.semantic_bins == 0);
        a.x.f.clock.mono = a.x.f.capture.capture_mono_ns / 1000000 + 1000;
        const auto r = a.p.publish(a.x.f.mailbox, a.x.f.channel, a.x.now);
        ++withdrawn_profile_puts;
        CHECK(r.dispatched && r.publish);
    }
}
void profile_current_stale_semantic() {
    P a; a.ready(false, true); a.check(); const auto old = a.x.frame;
    CHECK(a.p.report().evidence.raw_samples == 0 && a.p.report().evidence.semantic_bins > 0);
    CHECK(a.publish(a.x.now).publish);
    for (int i = 0; i < 6; ++i) a.x.advance_sparse();
    CHECK(a.x.process(&old) == SemanticStatus::Stale);
    CHECK(a.x.build() == WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(a.build() == WithdrawnProfileStatus::Ready); a.check();
    CHECK(a.p.report().evidence.semantic_status == SemanticStatus::Stale);
    CHECK(a.p.report().evidence.raw_samples == 2 && a.p.report().evidence.semantic_bins == 0);
    CHECK(a.publish(a.x.now).publish); // actual current raw support, no expired S or invented FREE
}
void profile_normal_withdrawal_normal() {
    P a;
    const auto normal = [&] {
        a.x.advance(false);
        CHECK(a.x.f.build() == NormalProfileStatus::Ready);
        CHECK(a.x.f.builder.body()->blind_near_m);
        CHECK(std::any_of(a.x.f.builder.body()->bins.begin(), a.x.f.builder.body()->bins.end(),
                          [](const Bin& bin) { return bin.d_free.has_value(); }));
        a.x.f.clock.mono = a.x.now / 1000000;
        CHECK(a.x.f.publish().publish);
    };
    normal();
    a.ready(); a.check(); CHECK(a.publish(a.x.now).publish);
    normal();
    CHECK(a.x.f.sink.calls == 3);
    a.no_put(); // the old withdrawn snapshot cannot be replayed after normal recovers
}
void profile_provenance_and_invalid_geometry() {
    P a;
    for (const auto basis : {SemanticPlaneBasis::Unknown, SemanticPlaneBasis::GroundPlaneDependent,
                             SemanticPlaneBasis::IndependentMeasurement}) {
        a.x.object.plane_basis = basis; a.ready(); a.check();
        CHECK((a.p.report().evidence.semantic_bins > 0) == (basis == SemanticPlaneBasis::IndependentMeasurement));
        CHECK(a.publish(a.x.now).publish);
    }
    a.x.advance(false); CHECK(a.x.f.geometry.result().has_evidence());
    CHECK(a.x.process(&a.x.frame) == SemanticStatus::Fresh);
    CHECK(a.x.build() == WithdrawnStatus::InvalidSource);
    CHECK(a.build() != WithdrawnProfileStatus::Ready); a.no_put();
    a.x.f.projection_binding.unit_m *= 2; a.x.advance();
    CHECK(a.x.f.geometry.result().status == GeometryStatus::ProfileMismatch);
    CHECK(a.x.process(&a.x.frame) == SemanticStatus::InvalidInput);
    CHECK(a.x.build() != WithdrawnStatus::ReadyEvidenceOnly);
    CHECK(a.build() != WithdrawnProfileStatus::Ready); a.no_put();
}
void profile_metadata() {
    auto b = binding(); auto optical_rays = rays(b); auto go = make_geometry_options();
    go.angle_min_rad = -.1; go.angle_step_rad = .1; go.n_bins = 3;
    NativeGeometry geometry{b, optical_rays.data(), optical_rays.size(), go};
    SemanticProjector s{geometry, {0, 0, 0, 50000000, 77}, SemanticDepthPolicy::NormalOrFitFailed};
    WithdrawnSemanticEvidence evidence{s}; bool threw = false;
    try { WithdrawnProfileBuilder invalid{evidence}; }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw); // cannot canonicalize the angle grid only at the wire boundary
}
} // namespace
int main(int argc, char** argv) {
    const bool emit = argc == 2 && std::string_view(argv[1]) == "--emit-status-fixture";
    const bool profile_emit = argc == 2 && std::string_view(argv[1]) == "--emit-profile-fixture";
    failed_and_normal(); absent_stale_and_excluded(); guards(); invalid_sources(); default_is_closed(); normal_owner_guard();
    raw_support_and_independence(); mixed_provenance(); withdrawn_status_put(emit);
    profile_fixtures(profile_emit); profile_reject_and_owner(); profile_upstream_and_lifecycle();
    profile_freshness_and_clock(); profile_current_stale_semantic(); profile_normal_withdrawal_normal();
    profile_commit_instant();
    profile_provenance_and_invalid_geometry(); profile_metadata();
    if (!emit && !profile_emit) std::cout << "withdrawn_evidence 9/9 PASS semantic_calls=" << semantic_count
              << " build_calls=" << withdrawn_count << " new_delta=0; old_status_profile_puts=0\n"
              << "withdrawn_profile 9/9 PASS build_calls=" << withdrawn_profile_builds
              << " put_calls=" << withdrawn_profile_puts << " new_delta=0; INJECTED_R6_PUT_ONLY\n";
}
