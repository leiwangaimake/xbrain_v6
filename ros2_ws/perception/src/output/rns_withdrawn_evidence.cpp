/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_withdrawn_evidence.cpp
 * Brief: Per-bin evidence for a fit-failed frame: raw-sample bit and independent S blocks only.
 *
 * Description:
 * Called by NativeProfileConsumer on the Profile thread when geometry reports FitFailed
 * (rns_native_profile.cpp), after the S owner (if any) has processed the same frame.
 *
 * Order of work in build_current(): clear everything; check that the frame is new, belongs to the
 * active stream, has a consistent capture stamp and really is a complete fit-failed geometry
 * result; commit the watermarks (so the frame can never be offered twice); fill kG from raw
 * sample counts and kS/d_block from fresh S distances; finally re-run the same check that
 * bins() will run at read time. Any failure clears all bins.
 *
 * Why the S distance is rounded down. The wire field is float32 and the S owner computes in
 * double. Rounding to nearest could move a block up to half a float ulp farther than measured;
 * stepping down one ulp when needed keeps the published block never farther than the measured
 * one, which is the safe direction for a stop distance.
 */
#include "output/rns_withdrawn_evidence.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>

namespace perception::rns {
namespace {
// Full source identity (stream, sequence, SDK indices and device times, host ready time).
bool same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
// Same frame, same clock domain and same capture time.
bool same(const geometry::CaptureStamp& a, const geometry::CaptureStamp& b) noexcept {
    return same(a.source, b.source) && a.clock_epoch == b.clock_epoch &&
        a.capture_mono_ns == b.capture_mono_ns;
}
// A genuine fit failure of a COMPLETE frame: every pixel visited, no plane produced and
// the raw sample counts valid. A partially processed frame is not evidence of anything.
bool fit_failed(const geometry::GeometryResult& r) noexcept {
    return r.status == geometry::GeometryStatus::FitFailed &&
        r.visited_pixels == geometry::kGeometryPixels &&
        r.fit.status != geometry::FitStatus::Pass && !r.fit.plane && r.has_raw_sample_evidence();
}
} // namespace
// Bin count follows the S owner's distance table, or the geometry options in G-only
// mode, so bins line up one to one with the Profile bins.
WithdrawnSemanticEvidence::WithdrawnSemanticEvidence(const SemanticProjector& s)
    : geometry_(s.geometry()), semantic_(&s), bins_(s.distances().size()) {}
WithdrawnSemanticEvidence::WithdrawnSemanticEvidence(const geometry::NativeGeometry& g)
    : geometry_(g), bins_(g.options().n_bins) {}
WithdrawnSemanticEvidence::WithdrawnSemanticEvidence(const geometry::NativeGeometry& g,
        const SemanticProjector* s)
    : geometry_(g), semantic_(s), bins_(s ? s->distances().size() : g.options().n_bins) {
    // Two geometry owners would mean two calibrations for one frame.
    if (s && &s->geometry() != &g)
        throw std::invalid_argument("withdrawn evidence requires the projector's own geometry owner");
}
void WithdrawnSemanticEvidence::clear(WithdrawnStatus status) noexcept {
    // Bin{} is the empty bin: d_free/d_block/h_block/slope null; src, conf and
    // terrain 0. That is exactly the withdrawn shape for bins without evidence.
    std::fill(bins_.begin(), bins_.end(), Bin{});
    report_ = {}; report_.status = status; revision_ = 0;
}
bool WithdrawnSemanticEvidence::current(const native_depth::Mailbox& m, std::int64_t now) const noexcept {
    // With an S owner: still bound to this mailbox, still describing this frame, not
    // re-processed since (revision), and still within its age limit at 'now'.
    const bool same_s = !semantic_ || (semantic_->bound_to(m) &&
        same(semantic_->report().depth, report_.capture) &&
        semantic_->report().revision == revision_ && semantic_->usable_at(now));
    // Always: same mailbox, time not before the build, the frame's stream still active,
    // and geometry still holding this frame's fit-failed result.
    return mailbox_ == &m && geometry_.bound_to(m) && same_s && now >= checked_at_ns_ && m.active_stream() != 0 &&
        m.active_stream() == report_.capture.source.stream_id &&
        fit_failed(geometry_.result()) && same(geometry_.result().source, report_.capture.source);
}
const WithdrawnReport& WithdrawnSemanticEvidence::build(const native_depth::Mailbox& m,
                                                       std::int64_t now) noexcept {
    // Wrong overload for a G-only instance: refuse rather than invent a stamp.
    if (!semantic_) { clear(WithdrawnStatus::InvalidSource); return report_; }
    return build_current(m, semantic_->report().depth, now);
}
const WithdrawnReport& WithdrawnSemanticEvidence::build(const native_depth::Mailbox& m,
        const geometry::CaptureStamp& capture, std::int64_t now) noexcept {
    // A configured S owner may not be bypassed with a caller-supplied stamp.
    if (semantic_) { clear(WithdrawnStatus::InvalidSource); return report_; }
    return build_current(m, capture, now);
}
const WithdrawnReport& WithdrawnSemanticEvidence::build_current(const native_depth::Mailbox& m,
        const geometry::CaptureStamp& c, std::int64_t now) noexcept {
    // Previous evidence is dropped before anything else, success or not.
    clear(WithdrawnStatus::NotBuilt);
    // Geometry and S owner must both read this mailbox; otherwise their frames
    // cannot be the same frame.
    if (!geometry_.bound_to(m) || (semantic_ && !semantic_->bound_to(m))) {
        clear(WithdrawnStatus::InvalidSource); return report_;
    }
    // The first build binds this object to one mailbox for good.
    if (!mailbox_) mailbox_ = &m;
    const auto& s = c.source;
    // Every rejection clears all bins and sets the reason status.
    const auto reject = [&](WithdrawnStatus status) -> const WithdrawnReport& {
        clear(status); return report_;
    };
    // Frame admission: active stream; stream/sequence strictly newer; capture time
    // strictly newer and not after 'now'; in G-only mode also capture <= host ready
    // <= now (with S, the stamp is the one the consumer verified and handed to the
    // S owner); same epoch within a stream; and geometry holds exactly this frame
    // as a complete fit failure.
    if (mailbox_ != &m || m.active_stream() == 0 || s.stream_id != m.active_stream() ||
        s.stream_id < last_.stream_id || s.sequence == 0 ||
        (s.stream_id == last_.stream_id && s.sequence <= last_.sequence) ||
        c.clock_epoch == 0 || c.capture_mono_ns <= last_capture_ns_ || now < c.capture_mono_ns ||
        (!semantic_ && (c.capture_mono_ns > s.host_ready_mono_ns || s.host_ready_mono_ns > now)) ||
        now < checked_at_ns_ || (s.stream_id == last_.stream_id && c.clock_epoch != epoch_) ||
        !same(geometry_.result().source, s) || !fit_failed(geometry_.result()))
        return reject(WithdrawnStatus::InvalidSource);
    // A failed downstream build cannot be retried as new evidence for this frame.
    // (Watermarks are committed before the bins are filled, on purpose.)
    last_ = s; epoch_ = c.clock_epoch; last_capture_ns_ = c.capture_mono_ns; checked_at_ns_ = now;
    const auto& raw = geometry_.raw_sample_counts();
    // Bin tables must line up, and a configured S owner must be usable now with at
    // least one processed revision (Missing/Stale count as usable: they add no S).
    if (bins_.size() != raw.size() || (semantic_ && (!semantic_->usable_at(now) ||
        bins_.size() != semantic_->distances().size() || semantic_->report().revision == 0)))
        return reject(WithdrawnStatus::InvalidSemantic);
    // One pass over the bins. An invalid S value aborts the whole frame (rather than
    // dropping one bin), because it means the S table itself cannot be trusted.
    for (std::size_t i = 0; i < bins_.size(); ++i) {
        // r5 bit1 records actual valid samples, not confirmed ground. All FREE,
        // height, negative and traversability fields remain at empty defaults.
        // (A19-FIT-3: "src bit1 by samples", d_free null, bit0 = 0.)
        if (raw[i]) {
            bins_[i].src = kG;
            // Counters only feed the report (logs/tests), not the wire.
            ++report_.raw_sample_bins;
            report_.raw_samples += raw[i];
        }
        // S block for this bin, if the S owner has one (nullopt = no semantic target).
        const auto d = semantic_ ? semantic_->distances()[i] : std::nullopt;
        if (!d) continue;
        // A distance from a non-fresh S state, or outside [0, range_max], is corrupt.
        if (semantic_->report().status != SemanticStatus::Fresh || !std::isfinite(*d) || *d < 0 ||
            *d > geometry_.options().range_max_m)
            return reject(WithdrawnStatus::InvalidSemantic);
        // Round toward zero distance (see the file header).
        float f = static_cast<float>(*d);
        if (static_cast<double>(f) > *d) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
        // Unreachable after the range check above (d <= range_max_m); kept so that a
        // future change of that check can never publish inf or a negative block.
        if (!std::isfinite(f) || f < 0) return reject(WithdrawnStatus::InvalidSemantic);
        // kS (bit2) with the block distance; h_block stays null (no plane to measure on).
        bins_[i].d_block = f; bins_[i].src |= kS; ++report_.semantic_bins;
    }
    // Record what the evidence is bound to, then prove it is still current.
    report_.capture = c;
    // G-only mode reports Missing: there is no S owner at all in this deployment.
    report_.semantic_status = semantic_ ? semantic_->report().status : SemanticStatus::Missing;
    // The S revision this evidence was built from; bins() refuses any later one.
    revision_ = semantic_ ? semantic_->report().revision : 0;
    if (!current(m, now)) return reject(WithdrawnStatus::InvalidSource);
    // Both reasons together, as 11 section 3.1B.3 v2.2 requires for a withdrawn frame.
    report_.free_withdrawn = true;
    report_.reasons = {"ground_fit_fallback", "ground_free_withdrawn"};
    report_.status = WithdrawnStatus::ReadyEvidenceOnly;
    return report_;
}
// Read-time revalidation: evidence is handed out only while it still describes the
// current frame and a still-usable S state.
const std::vector<Bin>* WithdrawnSemanticEvidence::bins(const native_depth::Mailbox& m,
                                                      std::int64_t now) const noexcept {
    return report_.status == WithdrawnStatus::ReadyEvidenceOnly && current(m, now) ? &bins_ : nullptr;
}
// capacity(), not size(): the allocation actually held, for the memory budget.
std::size_t WithdrawnSemanticEvidence::storage_bytes() const noexcept { return bins_.capacity() * sizeof(Bin); }
} // namespace perception::rns
