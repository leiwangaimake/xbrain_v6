/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_depth_health.cpp
 * Brief: One-pass ROI scan per depth frame and a 1 s pixel-weighted ring window.
 *
 * Description:
 * observe_depth_health() runs once per processed native depth frame (Profile thread) and costs one
 * pass over the raster with no allocation. DepthHealthWindow runs on the Status thread and keeps
 * the last second of per-frame counts in a fixed ring.
 *
 * Why pixel-weighted. The window ratio is SUM invalid / SUM ROI pixels, not the mean of frame
 * ratios: every pixel-second counts once, whatever the frame rate.
 *
 * Why so many rejections latch. A health number that silently skipped bad input would look
 * better than the sensor is. Malformed observations, clock rollback and ring overflow latch the
 * window invalid for good; Status then refuses to publish the field (rns_health_fields.cpp).
 * Only a replayed frame (same identity) or an older frame is skipped and counted, since both are
 * handoff artefacts and not measurements.
 *
 * Data path in the service. Profile thread: observe_depth_health() on the frame it owns, then
 * SourceJournal (kind DepthProcessed) records the observation with its processing time. Status
 * thread: PublicationHealthHandoff::drain() moves journal entries into this window with the cut
 * time (sampled now - 1 ms) as ingest_ms, then snapshot() at the same cut feeds
 * apply_health_fields(). The handoff is stricter than this window: it treats any result other
 * than Accepted, Duplicate included, as a handoff fault and stops certifying cuts.
 * The ring holds 2048 entries in the service; at 30 fps a 1 s window needs about 30.
 *
 * Tests. tests/test_rns_depth_health.cpp (per-frame scan, ROI rules, window weighting, null
 * versus 1.0, latching) and tests/test_rns_health_fields.cpp (consumption by Status).
 */
#include "output/rns_depth_health.hpp"
#include "output/rns_envelope.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace perception::rns {
namespace {
// The two counting loops of observe_depth_health() (2026-10-13). They count, so any order gives
// the same numbers; on aarch64 sixteen pixels are compared at a time and the rest of the raster,
// like every other platform, goes through the plain loop below.

// Zero codes (no measurement) among pixels[0, count).
std::size_t count_zero_codes(const std::uint16_t* pixels, std::size_t count) noexcept {
    std::size_t zeros = 0, i = 0;
#if defined(__aarch64__)
    for (; i + 16 <= count; i += 16) {
        // One byte per pixel, 0xFF for a zero code; the top bits of the sixteen bytes add up to <= 16.
        const uint8x16_t zero = vcombine_u8(vmovn_u16(vceqzq_u16(vld1q_u16(pixels + i))),
                                            vmovn_u16(vceqzq_u16(vld1q_u16(pixels + i + 8))));
        zeros += vaddvq_u8(vshrq_n_u8(zero, 7));
    }
#endif
    for (; i < count; ++i) zeros += pixels[i] == 0 ? 1U : 0U;
    return zeros;
}

// Under a mask: pixels whose mask byte is not 0 (roi), the zero codes among them (invalid), and
// whether any mask byte is above 1 (the mask is not binary).
struct MaskedCounts {
    std::size_t roi = 0, invalid = 0;
    bool not_binary = false;
};
MaskedCounts count_masked(const std::uint16_t* pixels, const std::uint8_t* mask, std::size_t count) noexcept {
    MaskedCounts out;
    std::size_t i = 0;
#if defined(__aarch64__)
    const uint8x16_t one = vdupq_n_u8(1);
    uint8x16_t above_one = vdupq_n_u8(0);
    for (; i + 16 <= count; i += 16) {
        const uint8x16_t m = vld1q_u8(mask + i);
        above_one = vorrq_u8(above_one, vcgtq_u8(m, one));
        const uint8x16_t in = vtstq_u8(m, m);  // 0xFF where the mask byte is not 0
        const uint8x16_t zero = vcombine_u8(vmovn_u16(vceqzq_u16(vld1q_u16(pixels + i))),
                                            vmovn_u16(vceqzq_u16(vld1q_u16(pixels + i + 8))));
        out.roi += vaddvq_u8(vshrq_n_u8(in, 7));
        out.invalid += vaddvq_u8(vshrq_n_u8(vandq_u8(in, zero), 7));
    }
    out.not_binary = vmaxvq_u8(above_one) != 0;
#endif
    for (; i < count; ++i) {
        const std::uint8_t m = mask[i];
        if (m > 1) out.not_binary = true;
        if (m != 0) {
            ++out.roi;
            if (pixels[i] == 0) ++out.invalid;
        }
    }
    return out;
}
}  // namespace

DepthHealthObservation observe_depth_health(const native_depth::Frame& frame,
                                         HealthRoiView roi) noexcept {
    // Starts as InvalidFrame with zero counts; every early return keeps that shape.
    DepthHealthObservation out;
    const auto& m = frame.metadata();
    // Identity and ROI kind are filled even on failure, so a rejection can be traced.
    out.source = m.source;
    out.roi_kind = roi.kind;
    const auto w = m.intrinsics.width, h = m.intrinsics.height;
    // Frame sanity: geometry matches the pixel count, buffer present, a real source
    // identity and a usable depth unit. Otherwise InvalidFrame with no counts.
    if (!w || !h || w > native_depth::kMaxPixels || h > native_depth::kMaxPixels ||
        // declared size must equal the actual pixel count (no overread past the buffer)
        static_cast<std::uint64_t>(w) * h != frame.pixel_count() ||
        !frame.pixel_count() || frame.pixel_count() > native_depth::kMaxPixels ||
        frame.pixel_count() > frame.capacity_pixels() || !frame.pixels() ||
        // stream_id 0 = unavailable and sequences start at 1, so a zero means no real source
        !m.source.stream_id || !m.source.sequence || m.source.host_ready_mono_ns < 0 ||
        !std::isfinite(m.unit_m) || m.unit_m <= 0) return out;
    const bool fallback = roi.kind == HealthRoiKind::LowerHalfUncalibrated;
    out.code = DepthHealthCode::InvalidRoi;
    // The fallback must not come with a mask (a mask means someone believes the
    // camera is calibrated); a calibrated ROI must be a mask of exactly this raster.
    if (fallback) {
        if (roi.mask || roi.mask_pixels || roi.width || roi.height) return out;
    } else if (roi.kind != HealthRoiKind::CalibratedStaticMask || !roi.mask ||
               // (an unknown kind value is rejected here as well)
               roi.mask_pixels != frame.pixel_count() || roi.width != w || roi.height != h) {
        return out;
    }
    // Single pass, no allocation. Native depth code 0 = no measurement (same rule as native
    // geometry).
    if (fallback) {
        // Lower half = rows [h/2, h), i.e. the pixels from index w * (h/2) to the end (the raster
        // is row-major and its size was checked above), the same pixels as "row i / w >= h / 2".
        const std::size_t first = static_cast<std::size_t>(w) * (h / 2U);
        out.roi_pixels = frame.pixel_count() - first;
        out.invalid_pixels = count_zero_codes(frame.pixels() + first, out.roi_pixels);
    } else {
        const MaskedCounts counts = count_masked(frame.pixels(), roi.mask, frame.pixel_count());
        // A calibrated mask is strictly binary; any other value means a wrong buffer. No partial
        // counts then: an invalid mask is not a smaller measured ROI.
        if (counts.not_binary) return out;
        out.roi_pixels = counts.roi;
        out.invalid_pixels = counts.invalid;
    }
    // An empty ROI has no ratio; reporting 0 or 1 would invent a measurement.
    if (!out.roi_pixels) { out.code = DepthHealthCode::EmptyRoi; return out; }
    // The window recomputes this same expression and requires bit equality.
    out.invalid_ratio = static_cast<double>(out.invalid_pixels) / out.roi_pixels;
    out.code = DepthHealthCode::Ready;
    return out;
}

namespace {
constexpr std::int64_t max_health_time = ProcessEnvelope::kMaxMonoMs;
// Configuration errors throw at construction (startup), never at run time.
std::size_t checked_window(std::int64_t startup, std::size_t capacity) {
    if (startup < 0 || startup > max_health_time || !capacity || capacity > DepthHealthWindow::kMaxCapacity)
        throw std::invalid_argument("invalid depth health window configuration");
    return capacity;
}
// Full identity: a frame with the same stream/sequence but different SDK data is a
// different frame claiming a used identity, which is corruption, not a replay.
bool same_source(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
}
// last_clock_ starts at startup, so any time before process start is a rollback.
DepthHealthWindow::DepthHealthWindow(std::int64_t startup, std::size_t capacity)
    : events_(checked_window(startup, capacity)), startup_(startup), last_clock_(startup) {}
// Counts the rejection; Invalid and CapacityExceeded also latch the window invalid.
// A counter at its maximum latches too, so a count can never wrap to look clean.
DepthWindowCode DepthHealthWindow::reject(DepthWindowCode code) noexcept {
    std::uint64_t* count = nullptr;
    // Duplicate and OutOfOrder are skipped samples: counted, window stays valid.
    if (code == DepthWindowCode::Duplicate) count = &duplicates_;
    else if (code == DepthWindowCode::OutOfOrder) count = &out_of_order_;
    else {
        valid_ = false;
        count = code == DepthWindowCode::CapacityExceeded ? &overflow_ : &invalid_;
    }
    // Saturate instead of wrapping: a wrapped counter would read as "no rejections".
    if (*count == std::numeric_limits<std::uint64_t>::max()) valid_ = false;
    else ++*count;
    return code;
}
// Every observe/snapshot time must be nondecreasing; a rollback latches invalid.
bool DepthHealthWindow::clock(std::int64_t now) noexcept {
    if (now < last_clock_ || now < 0 || now > max_health_time) {
        (void)reject(DepthWindowCode::Invalid); return false;
    }
    // Equal times are allowed: several drains can fall in one millisecond.
    last_clock_ = now; return true;
}
// Drop entries processed at or before now - 1000 ms: the window is (now-1000, now].
void DepthHealthWindow::prune(std::int64_t now) noexcept {
    // Entries are in processing-time order (observe enforces it), so pruning can
    // stop at the first entry that is still inside the window.
    while (size_ && now - events_[begin_].processed_ms >= kWindowMs) {
        begin_ = (begin_ + 1) % events_.size(); --size_;
    }
}
DepthWindowCode DepthHealthWindow::observe(const DepthHealthObservation& o,
        std::int64_t processed, std::int64_t ingest) noexcept {
    // Advance the window to the drain time first, so capacity reflects live entries.
    // clock() has already counted and latched the rollback, hence no second reject().
    if (!clock(ingest)) return DepthWindowCode::Invalid;
    prune(ingest);
    const auto& s = o.source;
    // Re-verify the observation itself (the journal crossed a thread boundary) and its
    // times: startup <= processed, host ready <= processed <= ingest.
    if (o.code != DepthHealthCode::Ready || !o.roi_pixels ||
        // counts: non-empty ROI no larger than one raster, invalid part of it
        o.roi_pixels > native_depth::kMaxPixels || o.invalid_pixels > o.roi_pixels ||
        // ratio present, finite and equal to the counts (same expression, bit exact)
        !o.invalid_ratio || !std::isfinite(*o.invalid_ratio) ||
        *o.invalid_ratio != static_cast<double>(o.invalid_pixels) / o.roi_pixels ||
        // a known ROI kind (an out-of-range enum value is memory damage)
        (o.roi_kind != HealthRoiKind::LowerHalfUncalibrated && o.roi_kind != HealthRoiKind::CalibratedStaticMask) ||
        // a real source identity, and the time order above
        !s.stream_id || !s.sequence || s.host_ready_mono_ns < 0 || processed < startup_ ||
        processed > ingest || s.host_ready_mono_ns / 1000000 > processed)
        return reject(DepthWindowCode::Invalid);
    // Same identity as the last accepted frame: a replay if identical, else corrupt.
    if (ever_ && s.stream_id == last_source_.stream_id && s.sequence == last_source_.sequence)
        return reject(same_source(s, last_source_) ? DepthWindowCode::Duplicate : DepthWindowCode::Invalid);
    // An older stream or sequence arriving late is skipped, not merged into the window.
    if (ever_ && (s.stream_id < last_source_.stream_id ||
        (s.stream_id == last_source_.stream_id && s.sequence < last_source_.sequence)))
        return reject(DepthWindowCode::OutOfOrder);
    // Newer identity but processing or host-ready time going backwards, or a sample
    // already outside the window when it arrives: the handoff is broken, so latch
    // rather than guess. (The Status thread drains several times a second, so a
    // healthy sample is always well inside the 1 s window on arrival.)
    if (processed < last_processed_ || ingest - processed >= kWindowMs ||
        (ever_ && s.host_ready_mono_ns <= last_source_.host_ready_mono_ns))
        return reject(DepthWindowCode::Invalid);
    // Full ring: dropping the oldest would silently shorten the window, so latch.
    if (size_ == events_.size()) return reject(DepthWindowCode::CapacityExceeded);
    // Append at the ring tail; ever_ stays true for the life of the window.
    events_[(begin_ + size_) % events_.size()] = {o, processed}; ++size_;
    last_source_ = s; last_processed_ = processed; ever_ = true;
    return DepthWindowCode::Accepted;
}
DepthHealthWindowReport DepthHealthWindow::snapshot(std::int64_t now) noexcept {
    DepthHealthWindowReport out;
    // now_ms lets Status check this snapshot was cut at its own instant.
    out.now_ms = now; out.ever_observed = ever_;
    // Snapshot also advances the window clock; a rollback here latches invalid too.
    const bool good_clock = clock(now);
    if (good_clock) prune(now);
    // The latched state and all counters are reported even when invalid, for the log.
    out.valid = valid_;
    out.duplicates = duplicates_; out.out_of_order = out_of_order_;
    out.invalid = invalid_; out.overflow = overflow_;
    // With a bad clock the counts are not trustworthy; return the (invalid) header only.
    if (!good_clock) return out;
    out.processed_frames = size_;
    for (std::size_t i = 0; i < size_; ++i) {
        const auto& o = events_[(begin_ + i) % events_.size()].observation;
        // A calibration change inside one second would mix two ROIs in one ratio.
        if (out.roi_kind && *out.roi_kind != o.roi_kind) out.valid = false;
        // Ring order is arrival order, so the last entry visited is the latest frame.
        out.roi_kind = o.roi_kind; out.latest_source = o.source;
        // uint64 sums: 1000000 entries x kMaxPixels cannot overflow.
        out.roi_pixels += o.roi_pixels; out.invalid_pixels += o.invalid_pixels;
    }
    // No processed frame -> no ratio (null on the wire); all invalid -> exactly 1.0.
    if (out.roi_pixels) out.invalid_ratio = static_cast<double>(out.invalid_pixels) / out.roi_pixels;
    return out;
}
}  // namespace perception::rns
