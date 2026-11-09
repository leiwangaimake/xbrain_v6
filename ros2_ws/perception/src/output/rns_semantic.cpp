/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_semantic.cpp
 * Brief: SemanticProjector: snapshot identity/age checks, per-object inflation and bin union.
 *
 * Description:
 * What problem this solves. Implements the S-channel projector declared in rns_semantic.hpp
 * (19 section 3.4A). For each depth frame it checks that the depth frame and the Objects
 * snapshot are in order and on one clock, that the snapshot is not older than objects_stale_ms
 * at the moment of use, then inflates every footprint by the motion margin and keeps, per bin,
 * the nearest covered distance.
 *
 * Why so many identity checks. A snapshot from another stream, a reordered snapshot, or one
 * built with another base transform would inject a boundary at a wrong place, and a wrong
 * semantic block is invisible downstream (it looks like a real obstacle). Every such case
 * produces an explicit status with no distances instead.
 *
 * Numerics. The margin must never be rounded down: the farthest-vertex radius and the margin are
 * computed with outward rounding (farthest_upper here, t_margin_upper_bound in traversability).
 */
#include "output/rns_semantic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
constexpr std::size_t kClassNameBytes = 128; // storage support bound, not a class policy
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Full identity equality (every field), as in rns_fused_objects.
bool same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
bool same(const geometry::CaptureStamp& a, const geometry::CaptureStamp& b) noexcept {
    return same(a.source, b.source) && a.clock_epoch == b.clock_epoch &&
        a.capture_mono_ns == b.capture_mono_ns;
}
// Mailbox identities increase in all device indices/timestamps, not merely in
// the application sequence. Equal sequence is the exact same RGB-D observation.
bool older_or_same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    if (a.stream_id != b.stream_id || a.sequence == 0 || a.sequence > b.sequence ||
        a.host_ready_mono_ns < 0) return false;
    if (a.sequence == b.sequence) return same(a, b);
    return a.color_device_us > 0 && a.depth_device_us > 0 &&
        a.color_index < b.color_index && a.depth_index < b.depth_index &&
        a.color_device_us < b.color_device_us && a.depth_device_us < b.depth_device_us &&
        a.host_ready_mono_ns <= b.host_ready_mono_ns;
}
bool nonnegative(double v) noexcept { return std::isfinite(v) && v >= 0; }
// Same strict UTF-8 rule as the JSON encoder, plus the slot's storage bound.
bool class_name(std::string_view text) noexcept {
    if (text.empty() || text.size() > kClassNameBytes || text.data() == nullptr) return false;
    std::size_t i = 0;
    while (i < text.size()) {
        const auto c = static_cast<unsigned char>(text[i++]);
        // Byte rules as in rns_contract utf8(): lead byte, continuations, range.
        if (c < 0x80) continue;
        unsigned tail = 0, value = 0, minimum = 0;
        if (c >= 0xc2 && c <= 0xdf) { tail = 1; value = c & 0x1f; minimum = 0x80; }
        else if (c >= 0xe0 && c <= 0xef) { tail = 2; value = c & 0x0f; minimum = 0x800; }
        else if (c >= 0xf0 && c <= 0xf4) { tail = 3; value = c & 7; minimum = 0x10000; }
        else return false;
        if (tail > text.size() - i) return false;
        for (unsigned n = 0; n < tail; ++n) {
            const auto next = static_cast<unsigned char>(text[i++]);
            if ((next & 0xc0) != 0x80) return false;
            value = (value << 6) | (next & 0x3f);
        }
        if (value < minimum || value > 0x10ffff || (value >= 0xd800 && value <= 0xdfff)) return false;
    }
    return true;
}
// Upper bound of the farthest vertex distance from the base_link origin; it scales
// the rotational part of the margin (omega_max * r). +inf for a vertex outside the
// coordinate bound, which the caller rejects.
double farthest_upper(const SemanticObject& o) noexcept {
    double far = 0;
    for (std::size_t i = 0; i < o.vertices; ++i) {
        const double x = o.footprint[i].x, y = o.footprint[i].y;
        if (!std::isfinite(x) || !std::isfinite(y) ||
            std::abs(x) > kSemanticCoordinateBound || std::abs(y) > kSemanticCoordinateBound)
            return kInfinity;
        // float32 coordinate squares are exact in binary64 at this bound.
        // Round their sum outward, then check the square-root residual: do not
        // let a nearest-rounded radius reduce the angular-motion envelope.
        const double sum = x*x + y*y;
        if (sum == 0) continue;
        const double upper = std::nextafter(sum, kInfinity);
        double r = std::sqrt(upper);
        if (std::fma(-r, r, upper) > 0) r = std::nextafter(r, kInfinity);
        if (!std::isfinite(r) || std::fma(-r, r, upper) > 0) return kInfinity;
        far = std::max(far, r);
    }
    return far;
}
} // namespace

SemanticProjector::SemanticProjector(const geometry::NativeGeometry& geometry,
        const SemanticOptions& options, SemanticDepthPolicy policy)
        : geometry_(geometry), options_(options), depth_policy_(policy) {
    const auto& o = geometry.options();
    // The Profile's own bin layout, so S bin b and Profile bin b are the same angle.
    layout_ = {o.angle_min_rad, o.angle_step_rad, o.range_max_m, o.n_bins};
    // Configuration errors are startup errors (throw), not per-frame statuses:
    // the same bounds as the Profile contract (range <= 6 m, <= kMaxBins bins),
    // a non-negative envelope and an explicit age bound and base revision.
    if ((policy != SemanticDepthPolicy::NormalOnly && policy != SemanticDepthPolicy::NormalOrFitFailed) ||
        !nonnegative(options.v_max_mps) || !nonnegative(options.omega_max_rps) ||
        !nonnegative(options.jitter_m) || options.jitter_m > kSemanticCoordinateBound ||
        options.max_age_ns < 0 || options.publish_guard_ns < 0 ||
        options.publish_guard_ns > options.max_age_ns || options.base_revision == 0 || o.n_bins == 0 ||
        o.n_bins > kMaxBins || !std::isfinite(o.angle_min_rad) ||
        !std::isfinite(o.angle_step_rad) || o.angle_step_rad <= 0 ||
        !std::isfinite(o.range_max_m) || o.range_max_m <= 0 || o.range_max_m > 6)
        throw std::invalid_argument("semantic projection requires bounded geometry and explicit age/base binding");
    // The only allocations; process() reuses them.
    distances_.resize(o.n_bins);
    scratch_.resize(o.n_bins);
    clear(SemanticStatus::NotProcessed);
}
// Depth usable for S: normal geometry evidence, or (NormalOrFitFailed) a fully
// visited frame whose ground fit failed with no plane at all but that still has
// raw samples; A19-FIT-3 keeps an independent S block in that frame.
bool SemanticProjector::usable_depth() const noexcept {
    const auto& r = geometry_.result();
    return r.has_evidence() || (depth_policy_ == SemanticDepthPolicy::NormalOrFitFailed &&
        r.status == geometry::GeometryStatus::FitFailed && r.visited_pixels == geometry::kGeometryPixels &&
        r.fit.status != geometry::FitStatus::Pass && !r.fit.plane && r.has_raw_sample_evidence());
}
void SemanticProjector::clear(SemanticStatus status) noexcept {
    std::fill(distances_.begin(), distances_.end(), std::nullopt);
    std::fill(scratch_.begin(), scratch_.end(), std::nullopt);
    report_ = {};
    report_.status = status;
    report_.revision = revision_;
}
const SemanticReport& SemanticProjector::process(const native_depth::Mailbox& mailbox,
        const geometry::CaptureStamp& capture, std::int64_t now,
        const SemanticFrame* frame) noexcept {
    // Revision exhaustion is terminal rather than allowing an old builder
    // snapshot to become equal again after wraparound.
    if (revision_ == std::numeric_limits<std::uint64_t>::max()) {
        clear(SemanticStatus::InvalidInput); return report_;
    }
    ++revision_;
    clear(SemanticStatus::NotProcessed);
    const auto active = mailbox.active_stream();
    // Every early exit: no distances, and if the stream changed meanwhile the
    // result says so instead of blaming the input.
    const auto finish = [&](SemanticStatus status) -> const SemanticReport& {
        clear(mailbox.active_stream() == active ? status : SemanticStatus::InactiveOrStale);
        report_.depth = capture;
        report_.checked_at_ns = now;
        return report_;
    };
    // A numeric stream id is local to one Mailbox, not globally unique. Verify
    // the geometry owner before binding this projector on its FIRST call.
    // Preserve the existing wrong-owner result after an established binding.
    if (mailbox_ != nullptr && mailbox_ != &mailbox) return finish(SemanticStatus::InactiveOrStale);
    if (!geometry_.bound_to(mailbox)) return finish(SemanticStatus::InvalidIdentity);
    if (mailbox_ == nullptr) mailbox_ = &mailbox;
    const auto& source = capture.source;
    // Depth order: the active stream, and strictly newer than the last frame.
    if (mailbox_ != &mailbox || active == 0 || source.stream_id != active ||
        source.stream_id < last_.stream_id || source.sequence == 0 ||
        (source.stream_id == last_.stream_id && source.sequence <= last_.sequence))
        return finish(SemanticStatus::InactiveOrStale);
    // The geometry must be of this very frame, on one clock, with capture <= now
    // and time moving forward.
    if (!same(geometry_.result().source, source) || capture.clock_epoch == 0 ||
        capture.capture_mono_ns < 0 || now < 0 || now < capture.capture_mono_ns ||
        (source.stream_id == last_.stream_id && clock_epoch_ != capture.clock_epoch) ||
        capture.capture_mono_ns <= last_capture_ns_ || now < last_check_ns_)
        return finish(SemanticStatus::InvalidIdentity);
    // Accept the depth identity before the S checks: a frame without usable S
    // is still consumed, so it cannot be offered again.
    last_ = source;
    clock_epoch_ = capture.clock_epoch;
    last_capture_ns_ = capture.capture_mono_ns;
    last_check_ns_ = now;
    // Without usable depth there is no geometric profile to inject into.
    if (!usable_depth()) return finish(SemanticStatus::InvalidInput);
    if (frame == nullptr) return finish(SemanticStatus::Missing);
    // Snapshot identity: same clock, same base transform, and an observation not
    // newer than this depth frame on the same stream.
    const auto& stamp = frame->stamp;
    if (stamp.clock_epoch != capture.clock_epoch || stamp.capture_mono_ns < 0 ||
        frame->base_revision != options_.base_revision || !older_or_same(stamp.source, source))
        return finish(SemanticStatus::InvalidIdentity);
    if (stamp.capture_mono_ns > capture.capture_mono_ns) return finish(SemanticStatus::Future);
    // Same observation as the depth frame: the stamps must match exactly.
    if (stamp.source.sequence == source.sequence && !same(stamp, capture))
        return finish(SemanticStatus::InvalidIdentity);
    // Snapshot order: the same snapshot again (latest-wins slot not yet
    // replaced) must be identical; a new one must be strictly newer.
    if (last_objects_ && last_objects_->source.stream_id == source.stream_id) {
        if (stamp.source.sequence == last_objects_->source.sequence) {
            if (!same(stamp, *last_objects_)) return finish(SemanticStatus::InvalidIdentity);
        } else if (!older_or_same(last_objects_->source, stamp.source) ||
                   stamp.capture_mono_ns <= last_objects_->capture_mono_ns) {
            return finish(SemanticStatus::InvalidIdentity);
        }
    }
    // Both timestamps are nonnegative and ordered, so these differences cannot
    // overflow int64. Freshness is checked at consumption, not just at capture.
    const auto age_at_capture = capture.capture_mono_ns - stamp.capture_mono_ns;
    const auto age_at_check = now - stamp.capture_mono_ns;
    // The publish guard (see the header): a snapshot this close to the bound would pass here
    // and fail the recheck at publish, which refuses the whole body.
    if (age_at_check > options_.max_age_ns - options_.publish_guard_ns) {
        finish(SemanticStatus::Stale);
        if (report_.status == SemanticStatus::Stale) {
            // The source header is valid even though its evidence has expired.
            // Retain its audit identity and order watermark, never distances.
            last_objects_ = stamp;
            report_.objects = stamp;
            report_.age_at_capture_ns = age_at_capture;
            report_.age_at_check_ns = age_at_check;
            report_.stale_by_guard = age_at_check <= options_.max_age_ns;
        }
        return report_;
    }
    // Only measured base_link footprints are semantic evidence (see header).
    if (frame->material != SemanticMaterial::MeasuredBaseFootprint || frame->count > kMaxObjects ||
        (frame->count != 0 && frame->objects == nullptr))
        return finish(SemanticStatus::InvalidInput);

    // Margin per object uses the age at capture (t_capture - t_obj), the gap the
    // robot may have moved through between the two observations. The formula is T's
    // (t_margin_upper_bound); tau = 1 only satisfies its option check, the margin ignores it.
    const geometry::TOptions margin_options{1, options_.v_max_mps, options_.omega_max_rps,
                                          options_.jitter_m, options_.max_age_ns};
    const bool fit_failed = geometry_.result().status == geometry::GeometryStatus::FitFailed;
    for (std::size_t i = 0; i < frame->count; ++i) {
        const auto& object = frame->objects[i];
        if (object.track_id < 0 || object.class_id < 0 || !class_name(object.class_name) ||
            object.vertices < 3 || object.vertices > kSemanticVertices ||
            (object.plane_basis != SemanticPlaneBasis::Unknown &&
             object.plane_basis != SemanticPlaneBasis::IndependentMeasurement &&
             object.plane_basis != SemanticPlaneBasis::GroundPlaneDependent))
            return finish(SemanticStatus::InvalidInput);
        for (std::size_t j = 0; j < i; ++j)
            if (frame->objects[j].track_id == object.track_id) return finish(SemanticStatus::InvalidInput);
        // Inflate with one uniform margin sized for the farthest vertex, then
        // project into scratch_ (closed-sector coverage, see header).
        const double far = farthest_upper(object);
        const double margin = geometry::t_margin_upper_bound(margin_options, age_at_capture, far);
        if (!std::isfinite(far) || !std::isfinite(margin) || margin > kSemanticCoordinateBound ||
            !project_semantic_footprint(object.footprint.data(), object.vertices, layout_, margin,
                                      scratch_.data(), scratch_.size()))
            return finish(SemanticStatus::InvalidInput);
        // The traversable class is T evidence, never a semantic block (11 v2.1).
        if (object.class_name == "traversable_area") { ++report_.excluded_t_objects; continue; }
        // Exclusion is per object: a mixed frame may retain independent S while
        // dropping another footprint derived from the failed ground plane.
        // Still validate all input geometry above; a corrupt object is not
        // silently converted into a healthy empty semantic observation.
        if (fit_failed && object.plane_basis != SemanticPlaneBasis::IndependentMeasurement) {
            ++report_.excluded_plane_basis_objects; continue;
        }
        ++report_.projected_objects;
        report_.maximum_margin_m = std::max(report_.maximum_margin_m, margin);
        // Union over objects: the nearest semantic boundary per bin wins.
        for (std::size_t b = 0; b < distances_.size(); ++b) {
            if (scratch_[b] && (!distances_[b] || *scratch_[b] < *distances_[b])) distances_[b] = scratch_[b];
        }
    }
    // Re-check after the loop: the capture thread may have switched streams
    // meanwhile, and a result for a retired stream must not be published.
    if (mailbox.active_stream() != active || !same(geometry_.result().source, source) ||
        !usable_depth()) return finish(SemanticStatus::InactiveOrStale);
    last_objects_ = stamp;
    report_.status = SemanticStatus::Fresh;
    report_.depth = capture;
    report_.objects = stamp;
    report_.checked_at_ns = now;
    report_.age_at_capture_ns = age_at_capture;
    report_.age_at_check_ns = age_at_check;
    report_.object_count = frame->count;
    for (const auto& distance : distances_) report_.touched_bins += distance.has_value();
    return report_;
}
// Checked again whenever a Profile builder uses the result, because time has
// passed since process(): the depth binding must still hold and a Fresh snapshot
// must still be within max age.
bool SemanticProjector::usable_at(std::int64_t now) const noexcept {
    if (mailbox_ == nullptr || report_.checked_at_ns < 0 || now < report_.checked_at_ns ||
        mailbox_->active_stream() == 0 || mailbox_->active_stream() != report_.depth.source.stream_id ||
        !usable_depth() || !same(geometry_.result().source, report_.depth.source)) return false;
    // No S is a valid, current answer for this depth frame.
    if (report_.status == SemanticStatus::Missing || report_.status == SemanticStatus::Stale) return true;
    if (report_.status != SemanticStatus::Fresh || !report_.objects ||
        report_.objects->capture_mono_ns < 0 || report_.objects->capture_mono_ns > now) return false;
    return now - report_.objects->capture_mono_ns <= options_.max_age_ns;
}
// Both per-bin buffers, for the startup memory log.
std::size_t SemanticProjector::storage_bytes() const noexcept {
    return (distances_.capacity() + scratch_.capacity()) * sizeof(std::optional<double>);
}
} // namespace perception::rns
