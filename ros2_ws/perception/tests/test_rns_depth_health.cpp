/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_depth_health.cpp
 * Brief: Depth health tests on a real bounded Mailbox to Frame transfer; no SDK, network or calibration claim.
 *
 * Description:
 * What it checks. Depth-valid ratios are computed on the native frame. Cases cover the initial
 * state and source identity, the lower half rather than the full image when uncalibrated, odd and
 * single rows, a calibrated static mask, ratio extremes and an empty ROI, invalid calibrated
 * views, no partial counts from invalid binaries, that an uncalibrated build cannot silently use a
 * mask, allocation stress and the heartbeat window with its failures. The scan itself (sixteen
 * pixels at a time on aarch64 since 2026-10-13) is held against the earlier pixel-by-pixel loop,
 * kept here as the reference, over many raster sizes, fill patterns and masks.
 */

#include "output/rns_depth_health.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#ifdef NDEBUG
#error "Depth health tests require enabled assertions"
#endif

static std::atomic<std::size_t> ordinary_news{0}, aligned_news{0};
void* operator new(std::size_t n) {
    ++ordinary_news;
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    try { return ::operator new(n); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
    return ::operator new(n, std::nothrow);
}
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) {
    ++aligned_news;
    void* p = nullptr;
    if (posix_memalign(&p, static_cast<std::size_t>(a), n ? n : 1) == 0) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n, std::align_val_t a) { return ::operator new(n, a); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    try { return ::operator new(n, a); } catch (...) { return nullptr; }
}
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return ::operator new(n, a, std::nothrow);
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }

using namespace perception::rns;
namespace nd = perception::native_depth;
namespace {
std::size_t groups = 0, observations = 0;
void pass(const char* name) { ++groups; std::cout << "PASS " << name << '\n'; }
bool same(const nd::SourceId& a, const nd::SourceId& b) {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
DepthHealthObservation checked(const nd::Frame& frame, HealthRoiView roi = {}) {
    const auto ordinary_before = ordinary_news.load(), aligned_before = aligned_news.load();
    const auto result = observe_depth_health(frame, roi);
    assert(ordinary_news == ordinary_before && aligned_news == aligned_before);
    ++observations;
    return result;
}
struct Fixture {
    std::uint32_t width, height;
    std::vector<std::uint16_t> raster;
    std::vector<std::uint8_t> mask;
    nd::Mailbox mailbox;
    nd::Frame frame;
    nd::View view;
    Fixture(std::uint32_t w, std::uint32_t h)
        : width(w), height(h), raster(static_cast<std::size_t>(w)*h, 1000),
          mask(raster.size(), 0), mailbox(raster.size()), frame(raster.size()) {
        auto& m = view.metadata;
        m.source = {mailbox.begin_stream(), 1, 7, 12, 100, 102, 2000000};
        m.intrinsics = {200, 201, .25F, .5F, w, h};
        m.distortion.reported = true;
        m.unit_m = .001F;
        view.pixels = raster.data(); view.bytes = raster.size()*sizeof(raster[0]);
        view.pre_d2c = true;
    }
    void transfer() {
        const auto ordinary_before = ordinary_news.load(), aligned_before = aligned_news.load();
        assert(mailbox.try_offer(view) == nd::Offer::Accepted);
        assert(mailbox.try_take(frame) == nd::Take::Taken);
        assert(ordinary_news == ordinary_before && aligned_news == aligned_before);
        assert(frame.pixel_count() == raster.size());
        assert(same(frame.metadata().source, view.metadata.source));
        auto& s = view.metadata.source;
        ++s.sequence; ++s.color_index; ++s.depth_index;
        s.color_device_us += 100; s.depth_device_us += 100; s.host_ready_mono_ns += 1000000;
    }
    HealthRoiView calibrated() const {
        return {HealthRoiKind::CalibratedStaticMask, mask.data(), mask.size(), width, height};
    }
};
void expect_empty(const DepthHealthObservation& o, DepthHealthCode code) {
    assert(o.code == code && !o.invalid_ratio && o.roi_pixels == 0 && o.invalid_pixels == 0);
}
void initial_and_source_identity() {
    Fixture f(4, 4);
    auto r = checked(f.frame);
    expect_empty(r, DepthHealthCode::InvalidFrame);
    assert(r.source.stream_id == 0 && r.source.sequence == 0);
    f.transfer();
    r = checked(f.frame);
    assert(r.code == DepthHealthCode::Ready && r.invalid_ratio && *r.invalid_ratio == 0);
    assert(same(r.source, f.frame.metadata().source));
    assert(r.roi_kind == HealthRoiKind::LowerHalfUncalibrated);
    assert(r.roi_pixels == 8 && r.invalid_pixels == 0);
    const auto first = r;
    f.raster[8] = 0; f.transfer();
    r = checked(f.frame);
    assert(r.source.sequence == first.source.sequence + 1);
    assert(r.invalid_pixels == 1 && *r.invalid_ratio == .125);
    assert(first.invalid_pixels == 0 && *first.invalid_ratio == 0); // returned value owns its facts
    pass("empty_frame_then_real_transfer_and_all_source_fields");
}
void lower_half_not_full_image() {
    Fixture f(4, 4);
    std::fill(f.raster.begin(), f.raster.begin() + 8, 0);
    f.raster[8] = 0; f.raster[15] = 0; f.transfer();
    auto r = checked(f.frame);
    assert(r.code == DepthHealthCode::Ready && r.roi_pixels == 8 && r.invalid_pixels == 2);
    assert(r.invalid_ratio && *r.invalid_ratio == .25 && *r.invalid_ratio != 10.0/16);
    std::fill(f.raster.begin(), f.raster.begin() + 8, 1000); f.transfer();
    r = checked(f.frame);
    assert(r.roi_pixels == 8 && r.invalid_pixels == 2 && *r.invalid_ratio == .25);
    pass("uncalibrated_lower_half_not_whole_image");
}
void odd_and_single_rows() {
    Fixture odd(3, 3);
    odd.raster = {1000, 1000, 1000, 0, 0, 0, 1000, 0, 1000};
    odd.view.pixels = odd.raster.data(); odd.transfer();
    const auto r = checked(odd.frame);
    assert(r.code == DepthHealthCode::Ready && r.roi_pixels == 6 && r.invalid_pixels == 4);
    assert(r.invalid_ratio && *r.invalid_ratio == 4.0/6); // includes middle row
    Fixture one(3, 1); one.raster[1] = 0; one.transfer();
    const auto one_r = checked(one.frame);
    assert(one_r.roi_pixels == 3 && one_r.invalid_pixels == 1 && *one_r.invalid_ratio == 1.0/3);
    pass("odd_height_middle_row_and_height_one");
}
void calibrated_static_mask() {
    Fixture f(4, 3);
    f.raster = {0, 1000, 0, 1000, 0, 0, 1000, 1000, 0, 1000, 1000, 1000};
    f.view.pixels = f.raster.data();
    f.mask = {1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0};
    f.transfer();
    auto r = checked(f.frame, f.calibrated());
    assert(r.code == DepthHealthCode::Ready && r.roi_kind == HealthRoiKind::CalibratedStaticMask);
    assert(r.roi_pixels == 4 && r.invalid_pixels == 3 && *r.invalid_ratio == .75);
    assert(same(r.source, f.frame.metadata().source));
    const auto fallback = checked(f.frame);
    assert(fallback.roi_pixels == 8 && fallback.invalid_pixels == 3 && *fallback.invalid_ratio == .375);
    // A new borrowed ROI is observed, not retained or replaced with fallback.
    std::fill(f.mask.begin(), f.mask.end(), 0); f.mask[1] = 1;
    r = checked(f.frame, f.calibrated());
    assert(r.roi_pixels == 1 && r.invalid_pixels == 0 && *r.invalid_ratio == 0);
    pass("explicit_calibrated_mask_not_dynamic_ground_or_fallback");
}
void ratio_extremes_and_empty_roi() {
    Fixture f(5, 4);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    std::fill(f.mask.begin(), f.mask.end(), 1); f.transfer();
    auto r = checked(f.frame);
    assert(r.code == DepthHealthCode::Ready && r.roi_pixels == 10 && r.invalid_pixels == 10);
    assert(r.invalid_ratio && *r.invalid_ratio == 1);
    r = checked(f.frame, f.calibrated());
    assert(r.roi_pixels == 20 && r.invalid_pixels == 20 && *r.invalid_ratio == 1);
    std::fill(f.mask.begin(), f.mask.end(), 0);
    r = checked(f.frame, f.calibrated());
    expect_empty(r, DepthHealthCode::EmptyRoi);
    assert(same(r.source, f.frame.metadata().source));
    std::fill(f.raster.begin(), f.raster.end(), std::numeric_limits<std::uint16_t>::max());
    std::fill(f.mask.begin(), f.mask.end(), 1); f.transfer();
    r = checked(f.frame, f.calibrated());
    assert(r.roi_pixels == 20 && r.invalid_pixels == 0 && *r.invalid_ratio == 0);
    pass("all_invalid_one_empty_roi_null_and_nonzero_depth_no_extra_cutoff");
}
void invalid_calibrated_views() {
    Fixture f(4, 4); f.raster[0] = 0; f.transfer();
    std::fill(f.mask.begin(), f.mask.end(), 1);
    for (int variant = 0; variant < 8; ++variant) {
        auto roi = f.calibrated();
        switch (variant) {
            case 0: roi.mask = nullptr; break;
            case 1: roi.mask_pixels = 0; break;
            case 2: --roi.mask_pixels; break;
            case 3: ++roi.mask_pixels; break;
            case 4: roi.width = 0; break;
            case 5: ++roi.width; break;
            case 6: --roi.height; break;
            case 7: roi.kind = static_cast<HealthRoiKind>(77); break;
        }
        const auto r = checked(f.frame, roi);
        expect_empty(r, DepthHealthCode::InvalidRoi);
        assert(same(r.source, f.frame.metadata().source));
    }
    pass("calibrated_mask_presence_size_dimensions_kind_rejected");
}
void invalid_binary_no_partial_counts() {
    Fixture f(4, 4);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    std::fill(f.mask.begin(), f.mask.end(), 1); f.transfer();
    for (std::size_t at : {std::size_t{0}, std::size_t{8}, std::size_t{15}}) {
        f.mask[at] = 2;
        expect_empty(checked(f.frame, f.calibrated()), DepthHealthCode::InvalidRoi);
        f.mask[at] = 1;
    }
    f.mask.back() = 255;
    expect_empty(checked(f.frame, f.calibrated()), DepthHealthCode::InvalidRoi);
    f.mask.back() = 1;
    const auto restored = checked(f.frame, f.calibrated());
    assert(restored.code == DepthHealthCode::Ready && restored.roi_pixels == 16 &&
           restored.invalid_pixels == 16 && *restored.invalid_ratio == 1);
    pass("nonbinary_at_start_middle_end_clears_partial_counts");
}
void uncalibrated_cannot_silently_use_mask() {
    Fixture f(4, 4); f.transfer();
    for (int variant = 0; variant < 4; ++variant) {
        HealthRoiView roi;
        if (variant == 0) roi.mask = f.mask.data();
        if (variant == 1) roi.mask_pixels = 1;
        if (variant == 2) roi.width = f.width;
        if (variant == 3) roi.height = f.height;
        expect_empty(checked(f.frame, roi), DepthHealthCode::InvalidRoi);
    }
    auto wrong_flag = f.calibrated(); wrong_flag.kind = HealthRoiKind::LowerHalfUncalibrated;
    expect_empty(checked(f.frame, wrong_flag), DepthHealthCode::InvalidRoi);
    pass("uncalibrated_rejects_stale_supplied_roi_metadata");
}
void allocation_stress() {
    const auto ordinary_before = ordinary_news.load(), aligned_before = aligned_news.load();
    void* a = ::operator new(1); ::operator delete(a);
    void* b = ::operator new(64, std::align_val_t{64}); ::operator delete(b, std::align_val_t{64});
    assert(ordinary_news == ordinary_before + 1 && aligned_news == aligned_before + 1);
    Fixture f(4, 4); f.raster[8] = 0; f.mask[8] = 1; f.transfer();
    for (int i = 0; i < 10000; ++i) {
        const auto r = checked(f.frame, i % 2 ? f.calibrated() : HealthRoiView{});
        assert(r.code == DepthHealthCode::Ready && r.invalid_pixels == 1);
    }
    pass("ordinary_aligned_allocation_hooks_and_10000_observations");
}
DepthHealthObservation window_sample(std::uint64_t seq, std::int64_t ready_ms,
                                    std::size_t roi, std::size_t invalid) {
    DepthHealthObservation o;
    o.code = DepthHealthCode::Ready; o.source = {1, seq, seq, seq, seq, seq, ready_ms * 1000000};
    o.roi_pixels = roi; o.invalid_pixels = invalid;
    o.invalid_ratio = static_cast<double>(invalid) / roi; return o;
}
void heartbeat_window() {
    DepthHealthWindow w{0, 4};
    const auto a = window_sample(1, 9, 1, 1), b = window_sample(2, 19, 9, 0);
    const auto ordinary = ordinary_news.load(), aligned = aligned_news.load();
    auto s = w.snapshot(0);
    assert(s.valid && !s.ever_observed && !s.invalid_ratio && !s.processed_frames);
    assert(w.observe(a, 10, 10) == DepthWindowCode::Accepted);
    s = w.snapshot(10); assert(s.invalid_ratio == 1 && s.processed_frames == 1);
    assert(w.observe(b, 20, 20) == DepthWindowCode::Accepted);
    s = w.snapshot(1009);
    assert(s.valid && s.processed_frames == 2 && s.invalid_ratio == .1);
    assert(s.roi_pixels == 10 && s.invalid_pixels == 1); // not mean(1,0)=.5
    const auto repeat = w.snapshot(1009);
    assert(repeat.processed_frames == 2 && repeat.invalid_ratio == s.invalid_ratio);
    s = w.snapshot(1010); assert(s.processed_frames == 1 && s.invalid_ratio == 0);
    s = w.snapshot(1020);
    assert(s.valid && s.ever_observed && !s.invalid_ratio && !s.roi_kind && !s.latest_source);
    assert(s.processed_frames == 0 && s.roi_pixels == 0 && s.invalid_pixels == 0);
    assert(w.observe(b, 20, 1020) == DepthWindowCode::Duplicate); // no resurrection
    assert(w.snapshot(1020).valid);
    assert(ordinary_news == ordinary && aligned_news == aligned);
    pass("one_second_processed_window_weighted_roi_boundary_no_stale_replay_zero_allocations");
}
void heartbeat_window_failures() {
    const auto a = window_sample(2, 9, 4, 1);
    DepthHealthWindow w{0, 2};
    assert(w.observe(a, 10, 10) == DepthWindowCode::Accepted);
    auto older = window_sample(1, 1, 4, 1);
    assert(w.observe(older, 2, 10) == DepthWindowCode::OutOfOrder);
    assert(w.snapshot(10).valid);
    auto forged = a; ++forged.source.depth_index;
    assert(w.observe(forged, 10, 10) == DepthWindowCode::Invalid);
    assert(!w.snapshot(10).valid);
    for (int mode = 0; mode < 5; ++mode) {
        DepthHealthWindow bad{0, 1}; auto o = a;
        if (mode == 0) o.invalid_ratio = .9;
        if (mode == 1) o.code = DepthHealthCode::EmptyRoi;
        if (mode == 2) o.roi_pixels = 0;
        if (mode == 3) o.source.host_ready_mono_ns = 11000000;
        if (mode == 4) o.roi_kind = static_cast<HealthRoiKind>(7);
        assert(bad.observe(o, 10, 10) == DepthWindowCode::Invalid);
        assert(!bad.snapshot(10).valid);
    }
    DepthHealthWindow full{0, 1};
    assert(full.observe(a, 10, 10) == DepthWindowCode::Accepted);
    assert(full.observe(window_sample(3, 19, 4, 0), 20, 20) == DepthWindowCode::CapacityExceeded);
    assert(!full.snapshot(20).valid && full.snapshot(20).overflow == 1);
    DepthHealthWindow expired{0, 1};
    assert(expired.observe(a, 10, 1010) == DepthWindowCode::Invalid);
    DepthHealthWindow rollback{0, 1}; assert(rollback.snapshot(10).valid);
    assert(!rollback.snapshot(9).valid);
    DepthHealthWindow mixed{0, 2}; assert(mixed.observe(a, 10, 10) == DepthWindowCode::Accepted);
    auto other = window_sample(3, 19, 8, 4); other.roi_kind = HealthRoiKind::CalibratedStaticMask;
    assert(mixed.observe(other, 20, 20) == DepthWindowCode::Accepted);
    assert(!mixed.snapshot(20).valid); // no coherent calibration/ROI context
    bool threw = false; try { DepthHealthWindow invalid{-1, 1}; } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    pass("window_bad_identity_timing_roi_capacity_and_calibration_are_not_empty_measurements");
}
} // namespace

// The scan of observe_depth_health() as it was before 2026-10-13: one pixel at a time, the lower
// half decided per pixel by its row. Only the counts and the code; the frame checks before the
// scan are not repeated here (every case below passes them).
struct ReferenceCounts {
    DepthHealthCode code = DepthHealthCode::InvalidRoi;
    std::size_t roi_pixels = 0, invalid_pixels = 0;
    std::optional<double> invalid_ratio;
};
ReferenceCounts reference_scan(const nd::Frame& frame, HealthRoiView roi) {
    ReferenceCounts out;
    const auto w = frame.metadata().intrinsics.width, h = frame.metadata().intrinsics.height;
    const bool fallback = roi.kind == HealthRoiKind::LowerHalfUncalibrated;
    for (std::size_t i = 0; i < frame.pixel_count(); ++i) {
        if (!fallback && roi.mask[i] > 1) {
            out.roi_pixels = out.invalid_pixels = 0;
            return out;
        }
        if (fallback ? i / w >= h / 2U : roi.mask[i] != 0) {
            ++out.roi_pixels;
            if (!frame.pixels()[i]) ++out.invalid_pixels;
        }
    }
    if (!out.roi_pixels) { out.code = DepthHealthCode::EmptyRoi; return out; }
    out.invalid_ratio = static_cast<double>(out.invalid_pixels) / out.roi_pixels;
    out.code = DepthHealthCode::Ready;
    return out;
}
void same_as_reference(const Fixture& f, HealthRoiView roi) {
    const auto got = checked(f.frame, roi);
    const auto want = reference_scan(f.frame, roi);
    assert(got.code == want.code && got.roi_kind == roi.kind);
    assert(got.roi_pixels == want.roi_pixels && got.invalid_pixels == want.invalid_pixels);
    assert(got.invalid_ratio.has_value() == want.invalid_ratio.has_value());
    // The window recomputes the ratio and requires bit equality, so compare it exactly.
    if (got.invalid_ratio) assert(*got.invalid_ratio == *want.invalid_ratio);
    assert(same(got.source, f.frame.metadata().source));
}
// Raster sizes around the sixteen-pixel block (widths and pixel counts that are below, at and
// above multiples of 16, odd heights, single rows and columns) and the native 640 x 400; zero
// codes placed at random, at the very first and last pixel, at the first and last pixel of the
// lower half and around every block edge; masks random, all ones, all zeros, one pixel, and with
// a byte above 1 at the first, the last, a block-edge and a tail position.
std::size_t scan_cases = 0;
void scan_matches_reference_loop() {
    std::mt19937 rng(20261013U);
    const std::array<std::array<std::uint32_t, 2>, 22> sizes{{
        {1, 1}, {1, 2}, {2, 1}, {3, 3}, {15, 1}, {16, 1}, {17, 1}, {1, 33}, {5, 7}, {8, 4}, {16, 2}, {16, 3},
        {17, 3}, {31, 5}, {32, 2}, {33, 2}, {47, 3}, {48, 5}, {63, 9}, {65, 4}, {640, 400}, {641, 3}}};
    for (const auto& size : sizes) {
        const std::uint32_t w = size[0], h = size[1];
        const std::size_t n = static_cast<std::size_t>(w) * h;
        const std::size_t half = static_cast<std::size_t>(w) * (h / 2U);
        Fixture f(w, h);
        const int rounds = n > 100000 ? 6 : 40;
        for (int round = 0; round < rounds; ++round) {
            // Depth codes: a zero density per round, then zeros forced at the places listed above.
            const unsigned per_mille = std::array<unsigned, 5>{0, 3, 50, 500, 1000}[static_cast<std::size_t>(round) % 5];
            for (auto& code : f.raster) code = rng() % 1000U < per_mille ? 0 : static_cast<std::uint16_t>(1 + rng() % 65535U);
            if (round % 4 == 1) {
                for (const std::size_t at : {std::size_t{0}, n - 1, half, half ? half - 1 : 0}) f.raster[at] = 0;
                for (std::size_t edge = 16; edge < n; edge += 16) {
                    if (rng() % 3U == 0) f.raster[edge - 1] = 0;
                    if (rng() % 3U == 0) f.raster[edge] = 0;
                }
                for (std::size_t tail = n - n % 16; tail < n; ++tail) f.raster[tail] = 0;
            }
            f.transfer();
            same_as_reference(f, {});
            ++scan_cases;
            // Masks on the same frame.
            for (int kind = 0; kind < 9; ++kind) {
                for (auto& m : f.mask) m = static_cast<std::uint8_t>(rng() % 2U);
                if (kind == 1) std::fill(f.mask.begin(), f.mask.end(), 1);
                if (kind == 2) std::fill(f.mask.begin(), f.mask.end(), 0);
                if (kind == 3) { std::fill(f.mask.begin(), f.mask.end(), 0); f.mask[rng() % n] = 1; }
                // Not binary: one byte above 1, where the block scan and the plain loop meet.
                if (kind == 4) f.mask[0] = 2;
                if (kind == 5) f.mask[n - 1] = 255;
                if (kind == 6) f.mask[n >= 16 ? (n / 16) * 16 - 1 : n - 1] = 3;
                if (kind == 7) f.mask[rng() % n] = static_cast<std::uint8_t>(2 + rng() % 254U);
                // The byte above 1 lies on a pixel the mask would otherwise leave out.
                if (kind == 8) { std::fill(f.mask.begin(), f.mask.end(), 0); f.mask[n / 2] = 128; }
                same_as_reference(f, f.calibrated());
                ++scan_cases;
            }
        }
    }
    assert(scan_cases > 5000U);
    pass("scan_matches_the_pixel_by_pixel_reference_loop");
}
int main() {
    scan_matches_reference_loop();
    initial_and_source_identity(); lower_half_not_full_image(); odd_and_single_rows();
    calibrated_static_mask(); ratio_extremes_and_empty_roi(); invalid_calibrated_views();
    invalid_binary_no_partial_counts(); uncalibrated_cannot_silently_use_mask(); allocation_stress();
    heartbeat_window(); heartbeat_window_failures();
    std::cout << "rns_depth_health: " << groups << " groups PASS; " << observations
              << " observations (" << scan_cases << " scans against the reference loop), "
              << "ordinary/aligned hot allocations = 0/0\n";
}
