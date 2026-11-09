/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_normal_profile.cpp
 * Brief: Per-bin wire conversion and the NormalProfileBuilder build/publish cycle.
 *
 * Description:
 * What problem this solves. Implements rns_normal_profile.hpp. The per-bin functions turn one
 * bin's geometric evidence and walk result into a wire Bin under the 11 section 3.1B.1 src
 * rules; the builder runs them for all bins of one depth frame and publishes the body once.
 *
 * Why every input is re-validated. The walk results come from other owners; a result the wire
 * rules cannot express (FREE past the block, bit0 without FREE, a height on a semantic block)
 * would either be rejected by RNS as a whole or, worse, be accepted with a wrong meaning. Such a
 * bin fails the whole frame here (InvalidEvidence) instead of being "repaired".
 *
 * Why current() is checked twice. The owners are serial but live on after build(); between
 * build and publish the S snapshot may expire or the stream may change. publish() re-checks,
 * and a Profile carrying S bins is additionally guarded at the transport's put-before instant.
 */
#include "output/rns_normal_profile.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// Full identity equality (every field), as in rns_fused_objects: two frames with the same
// sequence but any other differing field are different frames (e.g. after a stream restart).
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
bool same_optional(const std::optional<geometry::CaptureStamp>& a,
                  const std::optional<geometry::CaptureStamp>& b) noexcept {
    return a.has_value() == b.has_value() && (!a || same(*a, *b));
}
// float32 wire rounding: down() never increases a value (less FREE, nearer
// block), up() never decreases it (blind range outward).
float down(double x) noexcept {
    float f = static_cast<float>(x);
    if (static_cast<double>(f) > x) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return f;
}
float up(double x) noexcept {
    float f = static_cast<float>(x);
    if (static_cast<double>(f) < x) f = std::nextafter(f, std::numeric_limits<float>::infinity());
    return f;
}
// Finite and within [0, maximum].
bool range(double x, double maximum) noexcept {
    return std::isfinite(x) && x >= 0 && x <= maximum;
}
// A height must be finite and representable as float32 (or absent).
bool optional_height(const std::optional<double>& h) noexcept {
    return !h || (std::isfinite(*h) && std::abs(*h) <= std::numeric_limits<float>::max());
}
// Acceptable T states for a frame that has no ground reference: the owner
// cannot compute anything without one, and that is not a T fault (B6-2).
bool no_ground_t(geometry::TStatus status) noexcept {
    return status == geometry::TStatus::GeometryUnavailable;
}
// T states a normal frame accepts: a fresh sample, or an ordinary absence (no mask yet, or one
// too old), both of which fall back to geometry-only for this frame.
bool normal_t(geometry::TStatus status) noexcept {
    using geometry::TStatus;
    // Malformed masks/unknown clocks are not treated as an absent T sample by
    // this adapter. Ordinary missing or expired masks explicitly permit G-only.
    return status == TStatus::Fresh || status == TStatus::Missing || status == TStatus::Stale;
}
} // namespace

bool normal_profile_bin(const geometry::GridBinEvidence& e,
        const geometry::BinWalkResult& r, double blind, double maximum,
        bool fresh_t, Bin& output, bool semantic_selected) noexcept {
    output = {};
    using geometry::BoundaryKind;
    const bool has_g = e.valid_returns > 0 || e.valid_pierce_rays > 0;
    // Reject any walk the wire rules could not express faithfully: counts and
    // distances in bounds, FREE before the block, FREE beyond blind iff cells
    // passed, T backing only with a fresh T and cells (and a fresh T backs every
    // passed prefix), slope only over >= 2 cells and below 90 degrees, and with
    // no geometry sample (bit1 = 0) nothing but an S block.
    if (!std::isfinite(maximum) || maximum <= 0 || maximum > 6 || !range(blind, maximum) ||
        !r.valid || e.valid_returns > geometry::kGeometryPixels ||
        e.valid_pierce_rays > geometry::kGeometryPixels || e.expected_rays > geometry::kGeometryPixels ||
        r.passed_cells > geometry::kMaxRadialCells ||
        (r.free_end_m && (!range(*r.free_end_m, maximum) || *r.free_end_m < blind)) ||
        (r.block_m && !range(*r.block_m, maximum)) || !optional_height(r.block_height_m) ||
        (r.free_end_m && r.block_m && *r.free_end_m >= *r.block_m) ||
        (!r.free_end_m && (r.passed_cells || r.slope_rad || r.t_backed_prefix)) ||
        (r.free_end_m && ((*r.free_end_m > blind) != (r.passed_cells > 0))) ||
        (r.t_backed_prefix && (!fresh_t || r.passed_cells == 0)) ||
        (fresh_t && r.passed_cells > 0 && !r.t_backed_prefix) ||
        (r.slope_rad && (!std::isfinite(*r.slope_rad) || *r.slope_rad < 0 ||
                         *r.slope_rad > 1.5707963267948966 || r.passed_cells < 2)) ||
        (!has_g && (r.free_end_m || (!semantic_selected && r.block_m) ||
                    r.block_height_m || r.t_backed_prefix))) return false;
    // Boundary kind must agree with block and height: an S block is positive and
    // heightless; none has neither; positive has a height > 0; negative has a
    // height <= 0 or none.
    if (semantic_selected) {
        if (!r.block_m || r.block_height_m || r.boundary != BoundaryKind::Positive) return false;
    } else if (r.boundary == BoundaryKind::None) {
        if (r.block_m || r.block_height_m) return false;
    } else if (r.boundary == BoundaryKind::Positive) {
        if (!r.block_m || !r.block_height_m || *r.block_height_m <= 0) return false;
    } else if (r.boundary == BoundaryKind::Negative) {
        if (!r.block_m || (r.block_height_m && *r.block_height_m > 0)) return false;
    } else return false;

    // If rounding pushes blind past the range the bin cannot be expressed.
    const float blind_wire = up(blind), range_wire = down(maximum);
    if (blind_wire > range_wire) return false;
    // bit1 (G) whenever the bin has any geometric sample; conf = share of the expected prior-plane
    // rays that returned, 0..255.
    Bin b;
    b.src = has_g ? kG : 0;
    // Integer arithmetic implements saturating floor; bounded 256000*255.
    if (e.expected_rays) b.conf = static_cast<std::uint8_t>(
        std::min(e.valid_returns, e.expected_rays) * 255 / e.expected_rays);
    // Block rounded nearer; source bit S (semantic), NEG (negative) or none
    // (geometric positive block, implied by bit2 = bit3 = 0).
    if (r.block_m) {
        b.d_block = down(*r.block_m);
        if (semantic_selected) b.src |= kS;
        else if (r.boundary == BoundaryKind::Negative) b.src |= kNegative;
        if (r.block_height_m) b.h_block = static_cast<float>(*r.block_height_m);
    }
    if (r.free_end_m && r.passed_cells == 0 && *r.free_end_m == blind) {
        // A TRUE empty verified interval (no cell passed, the walk stopped at
        // the blind range itself, with geometry evidence -- free_end is only
        // accepted above with a sample). 11 section 3.1B.1: "d_free ==
        // blind_near_m" means exactly this, and it must not turn into null
        // (unobserved) merely because blind is not float32-representable
        // (reply batch 8, B8-3). The wire value is the SAME float as
        // blind_near_m: no FREE is claimed, bit0 stays clear, no slope. If that
        // value cannot stay strictly before the boundary, fail closed (null):
        // the boundary is never moved and no tolerance is added.
        if (!b.d_block || blind_wire < *b.d_block) b.d_free = blind_wire;
    } else if (r.free_end_m) {
        // FREE rounded down and kept strictly before the block. bit0 and slope
        // only when FREE extends beyond blind; slope in ceil degrees, capped at 90.
        float free = down(*r.free_end_m);
        if (b.d_block && free >= *b.d_block)
            free = std::nextafter(*b.d_block, -std::numeric_limits<float>::infinity());
        if (free >= blind_wire) {
            b.d_free = free;
            if (free > blind_wire) {
                if (r.t_backed_prefix) b.src |= kT;
                if (r.slope_rad) b.slope_deg = static_cast<int>(std::min(90.0,
                    std::ceil(*r.slope_rad * (180.0 / 3.14159265358979323846))));
            }
        }
    }
    output = b;
    return true;
}

bool no_ground_profile_bin(const geometry::GridBinEvidence& e,
        const geometry::BinWalkResult& r, double maximum, Bin& output,
        bool semantic_selected) noexcept {
    output = {};
    using geometry::BoundaryKind;
    const bool has_g = e.valid_returns > 0 || e.valid_pierce_rays > 0;
    // No FREE of any kind, and the walk must have stopped for the missing ground
    // reference (or, without samples, for lack of samples).
    if (!std::isfinite(maximum) || maximum <= 0 || maximum > 6 ||
        !r.valid || e.valid_returns > geometry::kGeometryPixels ||
        e.valid_pierce_rays > geometry::kGeometryPixels || e.expected_rays > geometry::kGeometryPixels ||
        r.free_end_m || r.passed_cells || r.t_backed_prefix || r.slope_rad ||
        r.stop != (has_g ? geometry::StopReason::NoGroundReference : geometry::StopReason::NoSamples) ||
        (r.block_m && !range(*r.block_m, maximum)) || !optional_height(r.block_height_m)) return false;
    if (semantic_selected) {
        // An S boundary needs no ground reference and no geometry sample: this
        // is the "semantic sees it, geometry is blind" case the S channel
        // exists for. It never carries a height and never creates FREE.
        if (!r.block_m || r.block_height_m || r.boundary != BoundaryKind::Positive) return false;
    } else if (r.boundary == BoundaryKind::None) {
        if (r.block_m || r.block_height_m) return false;
    } else if (r.boundary == BoundaryKind::Positive) {
        // An expected-plane pierce ray alone is not a measured positive return.
        if (!e.valid_returns || !r.block_m || !r.block_height_m || *r.block_height_m <= 0) return false;
    } else return false;
    // Same bit1/conf/block rules as normal_profile_bin; never FREE, T or NEG.
    Bin b;
    b.src = has_g ? kG : 0;
    if (e.expected_rays) b.conf = static_cast<std::uint8_t>(
        std::min(e.valid_returns, e.expected_rays) * 255 / e.expected_rays);
    if (r.block_m) {
        b.d_block = down(*r.block_m);
        if (semantic_selected) b.src |= kS;
        else b.h_block = static_cast<float>(*r.block_height_m);
    }
    output = b;
    return true;
}

bool profile_bin_with_semantic(const geometry::GridBinEvidence& evidence,
        const geometry::BinWalkResult& prefix, const geometry::CellEvidence* cells, std::size_t cell_count,
        std::optional<double> blind_m, double dr_m, double range_m, bool fresh_t,
        std::optional<double> semantic_m, Bin& output) noexcept {
    output = {};
    // S can only make a bin MORE restrictive: restrict_semantic_prefix picks the nearer of the
    // geometric and the semantic boundary and cuts FREE before it; it never extends FREE.
    geometry::BinWalkResult walk = prefix;
    bool selected_s = false;
    if (semantic_m) {
        // No ground reference means no cells and no measured blind to truncate
        // against; the restriction then only chooses the nearer boundary.
        if (!restrict_semantic_prefix(prefix, blind_m ? cells : nullptr, blind_m ? cell_count : 0,
                blind_m ? *blind_m : 0.0, dr_m, *semantic_m, walk, selected_s)) return false;
    }
    // A known blind range means a ground reference: the full conversion.
    return blind_m ? normal_profile_bin(evidence, walk, *blind_m, range_m, fresh_t, output, selected_s)
                   : no_ground_profile_bin(evidence, walk, range_m, output, selected_s);
}

NormalProfileBuilder::NormalProfileBuilder(const geometry::NormalGrid& grid) : grid_(grid) {
    // Wire metadata must be exactly representable: RNS rebuilds every bin angle
    // from angle_min/step, so they must round-trip through float32 unchanged;
    // z_pass and range are rounded down. A violation is a startup error.
    const auto& o = grid_.geometry().options();
    if (!std::isfinite(o.z_pass_m) || o.z_pass_m <= 0 ||
        o.z_pass_m > std::numeric_limits<float>::max())
        throw std::invalid_argument("normal Profile z_pass exceeds float32 range");
    const float step = static_cast<float>(o.angle_step_rad), z = down(o.z_pass_m);
    if (o.n_bins == 0 || o.n_bins > kMaxBins || !std::isfinite(step) || step <= 0 ||
        !std::isfinite(z) || z <= 0 || !std::isfinite(static_cast<float>(o.angle_min_rad)) ||
        static_cast<double>(step) != o.angle_step_rad ||
        static_cast<double>(static_cast<float>(o.angle_min_rad)) != o.angle_min_rad ||
        down(o.range_max_m) <= 0) throw std::invalid_argument("normal Profile wire metadata is not representable");
    body_.bins.resize(o.n_bins);
    clear(NormalProfileStatus::NotBuilt);
}
NormalProfileBuilder::NormalProfileBuilder(const geometry::NormalGrid& grid,
        const geometry::NormalTraversability& traversability) : NormalProfileBuilder(grid) {
    // Every owner must describe this very geometry and grid.
    if (&traversability.grid() != &grid_ || &traversability.geometry() != &grid_.geometry())
        throw std::invalid_argument("normal Profile requires exact geometry/grid/T owners");
    t_ = &traversability;
}
NormalProfileBuilder::NormalProfileBuilder(const geometry::NormalGrid& grid,
        const geometry::NormalTraversability& t, const SemanticProjector& semantic)
        : NormalProfileBuilder(grid, t) {
    if (&semantic.geometry() != &grid_.geometry())
        throw std::invalid_argument("normal Profile requires exact semantic geometry owner");
    semantic_ = &semantic;
}
NormalProfileBuilder::NormalProfileBuilder(const geometry::NormalGrid& grid,
        const SemanticProjector& semantic) : NormalProfileBuilder(grid, &semantic) {}
NormalProfileBuilder::NormalProfileBuilder(const geometry::NormalGrid& grid,
        const SemanticProjector* semantic) : NormalProfileBuilder(grid) {
    // nullptr: no S configured, exactly the geometry-only builder.
    if (!semantic) return;
    if (&semantic->geometry() != &grid_.geometry())
        throw std::invalid_argument("normal Profile requires exact semantic geometry owner");
    semantic_ = semantic;
}
NormalProfileBuilder::NormalProfileBuilder(const geometry::NormalGrid& grid,
        const geometry::NormalTraversability* t, const SemanticProjector* semantic)
        : NormalProfileBuilder(grid, semantic) {
    // nullptr: no T configured, exactly the (grid, S*) builder.
    if (!t) return;
    if (&t->grid() != &grid_ || &t->geometry() != &grid_.geometry())
        throw std::invalid_argument("normal Profile requires exact geometry/grid/T owners");
    t_ = t;
}
// Reset to an empty body; body() then returns nullptr until the next Ready build.
void NormalProfileBuilder::clear(NormalProfileStatus status) noexcept {
    std::fill(body_.bins.begin(), body_.bins.end(), Bin{});
    body_.t_capture_mono_ms = body_.t_publish_mono_ms = -1;
    body_.t_seg_mono_ms.reset();
    body_.extrinsic_calibrated = false;
    body_.z_pass_m = body_.angle_min_rad = body_.angle_step_rad = body_.range_max_m = 0;
    body_.blind_near_m.reset();
    report_ = {};
    report_.status = status;
    t_snapshot_ = {};
    semantic_revision_ = 0;
    dispatched_ = false;
}
// True while everything the body was built from is unchanged: the T report
// snapshot, the S revision/depth/status (still usable), and geometry and grid
// results of this very frame on the active stream. Checked at the end of build()
// and again in publish().
bool NormalProfileBuilder::current(const native_depth::Mailbox& mailbox) const noexcept {
    const auto& c = report_.capture;
    const bool same_t = !t_ || (same(t_->report().depth, c) &&
        t_->report().status == t_snapshot_.status &&
        (normal_t(t_->report().status) ||
         (report_.no_ground_reference && no_ground_t(t_->report().status))) &&
        same_optional(t_->report().segmentation, t_snapshot_.segmentation) &&
        t_->report().age_ns == t_snapshot_.age_ns &&
        t_->report().near_support_missing == t_snapshot_.near_support_missing);
    const bool same_s = !semantic_ || (semantic_->bound_to(mailbox) && semantic_->report().revision == semantic_revision_ &&
        same(semantic_->report().depth, c) &&
        semantic_->report().status == report_.semantic_status &&
        semantic_->usable_at(semantic_->report().checked_at_ns));
    return same_s && same_t && grid_.geometry().bound_to(mailbox) &&
        mailbox.active_stream() != 0 && mailbox.active_stream() == c.source.stream_id &&
        grid_.geometry().result().has_evidence() &&
        grid_.result().status == (report_.no_ground_reference
            ? geometry::GridStatus::NoGroundReference : geometry::GridStatus::NormalEvidence) &&
        same(grid_.geometry().result().source, c.source) && same(grid_.result().source, c.source);
}
const NormalProfileReport& NormalProfileBuilder::build(const native_depth::Mailbox& mailbox,
        const geometry::CaptureStamp& capture) noexcept {
    clear(NormalProfileStatus::NotBuilt);
    const auto reject = [&](NormalProfileStatus status) -> const NormalProfileReport& {
        clear(status); return report_;
    };
    // Order: the active stream, strictly newer than the last built frame.
    const auto& s = capture.source;
    const auto active = mailbox.active_stream();
    if (active == 0 || s.stream_id != active || s.stream_id < last_.stream_id || s.sequence == 0 ||
        (s.stream_id == last_.stream_id && s.sequence <= last_.sequence))
        return reject(NormalProfileStatus::InactiveOrStale);
    // Identity: every owner result must be of this very capture.
    if (!grid_.geometry().bound_to(mailbox) || capture.clock_epoch == 0 || capture.capture_mono_ns < 0 ||
        (s.stream_id == last_.stream_id && epoch_ != capture.clock_epoch) ||
        !same(grid_.geometry().result().source, s) || !same(grid_.result().source, s) ||
        (t_ && !same(t_->report().depth, capture)) || (semantic_ && !semantic_->bound_to(mailbox)))
        return reject(NormalProfileStatus::InvalidIdentity);
    // The wire carries milliseconds; they must strictly increase per body.
    const Millis capture_ms = capture.capture_mono_ns / 1000000;
    if (capture_ms <= last_capture_ms_) return reject(NormalProfileStatus::InactiveOrStale);
    // Consume the identity even if the build fails below: a frame is built at most once, so a
    // retry can never publish an older frame after a newer one.
    last_ = s; epoch_ = capture.clock_epoch; last_capture_ms_ = capture_ms;
    if (!grid_.geometry().result().has_evidence()) return reject(NormalProfileStatus::GeometryUnavailable);
    const auto& gr = grid_.result();
    const bool no_ground = gr.status == geometry::GridStatus::NoGroundReference;
    // 19 A19-NULL-1 / A19-FIT-3 (reply batch 6, B6-2): a frame with no ground
    // reference is still published. FREE, the T backing and the slope are
    // withdrawn -- bit0 is never set without a verifiable FREE -- while measured
    // positive obstacles and an independent S boundary are kept.
    // Grid consistency: normal evidence needs a measured blind range inside the
    // range; a no-ground frame has no cells and at most the range-edge blind.
    const auto& o = grid_.geometry().options();
    if ((!no_ground && (gr.status != geometry::GridStatus::NormalEvidence || !gr.blind_m)) ||
        (gr.blind_m && (!range(*gr.blind_m, o.range_max_m) || up(*gr.blind_m) > down(o.range_max_m))) ||
        (no_ground && (gr.cells_per_bin != 0 || (gr.blind_m && *gr.blind_m != o.range_max_m))) ||
        // Without a ground reference the T owner reports GeometryUnavailable:
        // that is the same missing reference, not a T fault, and it must not
        // stop the frame (reply batch 6, B6-2). Malformed T states still do.
        (t_ && !normal_t(t_->report().status) && !(no_ground && no_ground_t(t_->report().status))))
        return reject(NormalProfileStatus::InvalidEvidence);
    // Without a T owner, behave exactly as "T sample missing" (G-only).
    geometry::TReport missing_t;
    missing_t.depth = capture;
    missing_t.status = geometry::TStatus::Missing;
    const auto& tr = t_ ? t_->report() : missing_t;
    const bool fresh_sample = tr.status == geometry::TStatus::Fresh;
    // A no-ground frame has no verifiable FREE, so no T backing is written to
    // the wire (bit0 clear, t_seg null) even if a fresh mask exists.
    const bool fresh = fresh_sample && !no_ground;
    // A fresh T sample must carry a consistent segmentation stamp on this clock
    // and stream, not after the depth frame; any other state carries none.
    if (fresh_sample) {
        if (!tr.segmentation || tr.age_ns < 0 ||
            tr.segmentation->clock_epoch != capture.clock_epoch ||
            tr.segmentation->source.stream_id != s.stream_id ||
            tr.segmentation->capture_mono_ns < 0 ||
            tr.segmentation->capture_mono_ns > capture.capture_mono_ns ||
            tr.age_ns != capture.capture_mono_ns - tr.segmentation->capture_mono_ns)
            return reject(NormalProfileStatus::InvalidEvidence);
    } else if (tr.segmentation || tr.age_ns != -1) return reject(NormalProfileStatus::InvalidEvidence);
    // Every owner must deliver exactly one result per wire bin.
    if (grid_.bins().size() != body_.bins.size() || (t_ && t_->prefixes().size() != body_.bins.size()))
        return reject(NormalProfileStatus::InvalidEvidence);
    // S must have processed this very depth frame and still be usable; its
    // revision is recorded so publish() can tell it is the same result.
    if (semantic_) {
        const auto& sr = semantic_->report();
        if (!same(sr.depth, capture) || sr.revision == 0 ||
            !semantic_->usable_at(sr.checked_at_ns) ||
            semantic_->distances().size() != body_.bins.size())
            return reject(NormalProfileStatus::InvalidEvidence);
        semantic_revision_ = sr.revision;
        report_.semantic_status = sr.status;
    }
    // Bin by bin: the walk (T-restricted when T is present), optionally cut by S, converted to
    // the wire. One inexpressible bin rejects the whole frame (see the file header).
    for (std::size_t i = 0; i < body_.bins.size(); ++i) {
        // Without a ground reference there is no FREE to back, so the T prefix
        // adds nothing over the geometry traversal and bit0 must stay clear.
        const geometry::BinWalkResult& prefix =
            (t_ && !no_ground) ? t_->prefixes()[i] : grid_.bins()[i].traversal;
        // Only a Fresh S result contributes distances.
        const std::optional<double> semantic_m =
            (semantic_ && semantic_->report().status == SemanticStatus::Fresh) ? semantic_->distances()[i]
                                                                               : std::nullopt;
        if (!profile_bin_with_semantic(grid_.bins()[i], prefix,
                no_ground ? nullptr : grid_.cells().data() + i * grid_.cell_capacity_per_bin(),
                no_ground ? 0 : gr.cells_per_bin,
                no_ground ? std::nullopt : gr.blind_m, grid_.options().dr_m, o.range_max_m, fresh,
                semantic_m, body_.bins[i]))
            return reject(NormalProfileStatus::InvalidEvidence);
        // Per-source bin counts for the report and the owner's diagnostics.
        report_.geometry_bins += (body_.bins[i].src & kG) != 0;
        report_.t_bins += (body_.bins[i].src & kT) != 0;
        report_.semantic_bins += (body_.bins[i].src & kS) != 0;
        report_.blocked_bins += body_.bins[i].d_block.has_value();
    }
    // Header fields, rounded the safe way; blind_near_m stays null without a
    // ground reference (unless the grid measured the range edge), t_seg only
    // with a fresh T.
    body_.t_capture_mono_ms = capture_ms;
    body_.extrinsic_calibrated = calibrated_;
    body_.z_pass_m = down(o.z_pass_m);
    body_.angle_min_rad = static_cast<float>(o.angle_min_rad);
    body_.angle_step_rad = static_cast<float>(o.angle_step_rad);
    body_.range_max_m = down(o.range_max_m);
    if (gr.blind_m) body_.blind_near_m = up(*gr.blind_m);
    if (fresh) body_.t_seg_mono_ms = tr.segmentation->capture_mono_ns / 1000000;
    report_.capture = capture;
    report_.t_status = tr.status;
    report_.near_support_missing = tr.near_support_missing;
    report_.no_ground_reference = no_ground;
    t_snapshot_ = tr;
    // The owners may have moved on while the bins were converted.
    if (!current(mailbox)) return reject(NormalProfileStatus::InactiveOrStale);
    report_.status = NormalProfileStatus::Ready;
    return report_;
}
// Borrowed pointer, only for a Ready build.
const Profile* NormalProfileBuilder::body() const noexcept {
    return report_.status == NormalProfileStatus::Ready ? &body_ : nullptr;
}
NormalProfilePut NormalProfileBuilder::publish(const native_depth::Mailbox& mailbox,
        PublisherChannel& channel, std::int64_t now_mono_ns) noexcept {
    // Nothing built: report the build status, dispatch nothing.
    NormalProfilePut r;
    r.status = report_.status;
    if (r.status != NormalProfileStatus::Ready) return r;
    // At most once, and only while the inputs are still current and S usable now.
    if (dispatched_ || !current(mailbox) || (semantic_ && !semantic_->usable_at(now_mono_ns))) {
        clear(NormalProfileStatus::InactiveOrStale);
        r.status = report_.status; return r;
    }
    // Marked before the put: a transport rejection does not allow a retry of
    // this body (the next depth frame brings a newer one).
    dispatched_ = true;
    r.dispatched = true;
    // A body with S bins depends on a snapshot with a lifetime. Its age is
    // rechecked once more at the actual commit instant (put-before sample), not
    // only against the caller's now: serialization and envelope assembly happen
    // between the two. Millisecond rounding is made conservative: the object
    // stamp and the lifetime are both floored, so the recheck is never looser
    // than the projector's own gate. A body without S bins does not depend on
    // the snapshot and is published unguarded.
    if (semantic_ && report_.semantic_bins > 0 && semantic_->report().objects) {
        const Millis observed = semantic_->report().objects->capture_mono_ns / 1000000;
        const Millis max_age = semantic_->max_age_ns() / 1000000;
        r.publish = channel.publish_guarded(body_, observed, max_age, observed);
    } else {
        r.publish = channel.publish(body_);
    }
    return r;
}
std::size_t NormalProfileBuilder::storage_bytes() const noexcept {
    return body_.bins.capacity() * sizeof(Bin);
}
} // namespace perception::rns
