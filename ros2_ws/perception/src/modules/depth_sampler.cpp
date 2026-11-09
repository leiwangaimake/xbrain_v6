/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: depth_sampler.cpp
 * Brief: Implements depth_sampler.hpp: band selection, back-projection and the depth hysteresis.
 *
 * Description:
 * Three implementations of the same sample, all bit-identical in their outputs:
 *   1. Fast scan (production: codesort plus PERCEPTION_DEPTH_SCAN_FAST=1, no profile): two passes
 *      over the region, the first counts depth codes in a histogram, the second keeps only the
 *      selected band; the u/v medians come from per-column and per-row counts.
 *   2. Reference path with code sort: stores every valid pixel, orders depths from the code
 *      histogram. Used when a profile or a sorted snapshot is requested.
 *   3. Reference path with std::sort over floats, the original implementation.
 * test_depth_codesort holds code sort against std::sort; test_depth_scan_fast_identity holds the
 * fast scan against both.
 */

#include "modules/depth_sampler.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <time.h>
#include <utility>
#include <vector>

#include "common/logger.hpp"

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace perception {
namespace {

// Fills an optional DepthSampleProfile: next() closes the running stage and opens the given one,
// the destructor closes the last stage and the totals. Does nothing (no clock reads) without a
// profile.
class DepthSampleProfileScope {
public:
    explicit DepthSampleProfileScope(DepthSampleProfile* profile) noexcept
        : profile_(profile) {
        if (!profile_) return; // The disabled path performs no clock reads.
        *profile_ = DepthSampleProfile{};
        profile_->wall_clock_valid = read_clock(CLOCK_MONOTONIC, profile_->start_ns);
        last_ns_ = profile_->start_ns;
#if defined(__linux__) && defined(CLOCK_THREAD_CPUTIME_ID)
        profile_->cpu_clock_valid = read_clock(CLOCK_THREAD_CPUTIME_ID, cpu_start_ns_);
#endif
    }

    ~DepthSampleProfileScope() noexcept {
        if (!profile_) return;
#if defined(__linux__) && defined(CLOCK_THREAD_CPUTIME_ID)
        if (profile_->cpu_clock_valid) {
            std::uint64_t cpu_end_ns = 0;
            if (read_clock(CLOCK_THREAD_CPUTIME_ID, cpu_end_ns) && cpu_end_ns >= cpu_start_ns_) {
                profile_->thread_cpu_ns = cpu_end_ns - cpu_start_ns_;
            } else {
                profile_->cpu_clock_valid = false;
                profile_->thread_cpu_ns = 0;
            }
        }
#endif
        if (profile_->wall_clock_valid) {
            std::uint64_t end_ns = 0;
            if (read_clock(CLOCK_MONOTONIC, end_ns) && end_ns >= last_ns_) {
                profile_->*stage_ += end_ns - last_ns_;
                profile_->end_ns = end_ns;
                profile_->total_wall_ns = end_ns - profile_->start_ns;
                return;
            }
        }
        profile_->wall_clock_valid = false;
        profile_->start_ns = profile_->end_ns = profile_->total_wall_ns = 0;
        profile_->setup_ns = profile_->scan_ns = profile_->depth_copy_ns = 0;
        profile_->sort_ns = profile_->cluster_selection_ns = profile_->select_pixels_ns = 0;
        profile_->median_u_ns = profile_->median_v_ns = profile_->statistics_ns = profile_->finalize_ns = 0;
    }

    DepthSampleProfileScope(const DepthSampleProfileScope&) = delete;
    DepthSampleProfileScope& operator=(const DepthSampleProfileScope&) = delete;

    void next(std::uint64_t DepthSampleProfile::*stage) noexcept {
        if (!profile_ || !profile_->wall_clock_valid) return;
        std::uint64_t now = 0;
        if (!read_clock(CLOCK_MONOTONIC, now) || now < last_ns_) {
            profile_->wall_clock_valid = false;
            return;
        }
        profile_->*stage_ += now - last_ns_;
        last_ns_ = now;
        stage_ = stage;
    }

private:
    static bool read_clock(clockid_t clock, std::uint64_t& ns) noexcept {
        timespec value{};
        if (clock_gettime(clock, &value) != 0 || value.tv_sec < 0 ||
            value.tv_nsec < 0 || value.tv_nsec >= 1000000000L) {
            ns = 0;
            return false;
        }
        ns = static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
             static_cast<std::uint64_t>(value.tv_nsec);
        return true;
    }

    DepthSampleProfile* profile_;
    std::uint64_t last_ns_ = 0;
#if defined(__linux__) && defined(CLOCK_THREAD_CPUTIME_ID)
    std::uint64_t cpu_start_ns_ = 0;
#endif
    std::uint64_t DepthSampleProfile::*stage_ = &DepthSampleProfile::setup_ns;
};

float code_depth_meters(std::uint16_t code, float scale) {
    return static_cast<float>(code) * scale;
}

float depth_meters(const cv::Mat& depth, int y, int x, float scale) {
    if (depth.type() == CV_16UC1) {
        return code_depth_meters(depth.at<std::uint16_t>(y, x), scale);
    }
    if (depth.type() == CV_32FC1) {
        return depth.at<float>(y, x) * scale;
    }
    throw std::invalid_argument("aligned depth must be CV_16UC1 or CV_32FC1");
}

float median_in_place(std::vector<float>& values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

struct PixelDepth {
    float depth_m = 0.0F;
    float u = 0.0F;
    float v = 0.0F;
};

struct DepthSamplerScratch {
    std::vector<PixelDepth> pixels;
    std::vector<float> sorted_depths;
    std::vector<float> selected_us;
    std::vector<float> selected_vs;
    std::vector<float> selected_depths;

    void clear() {
        pixels.clear();
        sorted_depths.clear();
        selected_us.clear();
        selected_vs.clear();
        selected_depths.clear();
    }
};

#if defined(__aarch64__)
// The fast scan reads a row sixteen pixels at a time (2026-10-13). Which of the sixteen a test
// keeps is returned as four bits per pixel (0xF kept, 0 not), pixel 0 in the lowest nibble; the
// kept pixels are then taken in increasing order, so everything that depends on the scan order
// (the selected-pixel list, the selected depths and their float sum) is what the pixel-by-pixel
// loop produces.
inline std::uint64_t pixel_nibbles(uint8x16_t bytes) {
    return vget_lane_u64(vreinterpret_u64_u8(vshrn_n_u16(vreinterpretq_u16_u8(bytes), 4)), 0);
}
// 0xFF where the mask byte is non-zero; without a mask row every pixel is in the region.
inline uint8x16_t region_bytes(const std::uint8_t* mask) {
    if (mask == nullptr) return vdupq_n_u8(0xFF);
    const uint8x16_t m = vld1q_u8(mask);
    return vtstq_u8(m, m);
}
// 0xFF where first <= code <= last.
inline uint8x16_t code_in_bytes(const std::uint16_t* depth, uint16x8_t first, uint16x8_t last) {
    const uint16x8_t lo = vld1q_u16(depth), hi = vld1q_u16(depth + 8);
    return vcombine_u8(vmovn_u16(vandq_u16(vcgeq_u16(lo, first), vcleq_u16(lo, last))),
                       vmovn_u16(vandq_u16(vcgeq_u16(hi, first), vcleq_u16(hi, last))));
}
inline int first_kept_pixel(std::uint64_t nibbles) { return __builtin_ctzll(nibbles) >> 2; }
inline std::uint64_t without_pixel(std::uint64_t nibbles, int pixel) { return nibbles & ~(0xFULL << (4 * pixel)); }
constexpr std::uint64_t kAllSixteen = ~0ULL;
#endif

// Samples that took the fast scan (depth_scan_fast_entries(), a test witness).
std::atomic<std::uint64_t> fast_scan_entries{0};

// Read once per process. The service sets it from the runtime switch depth_scan_fast
// (production: "1").
bool depth_scan_fast_enabled() {
    static const bool enabled = []() {
        const char* value = std::getenv("PERCEPTION_DEPTH_SCAN_FAST");
        return value != nullptr && std::string(value) == "1";
    }();
    return enabled;
}

struct DepthSelection {
    float depth_m = 0.0F;
    float lower_m = 0.0F;
    float upper_m = 0.0F;
    std::size_t required_count = 0U;
    bool used_fallback = false;
};

bool same_float_bits(float lhs, float rhs) {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "32-bit float required");
    std::uint32_t lhs_bits = 0;
    std::uint32_t rhs_bits = 0;
    std::memcpy(&lhs_bits, &lhs, sizeof(lhs_bits));
    std::memcpy(&rhs_bits, &rhs, sizeof(rhs_bits));
    return lhs_bits == rhs_bits;
}

void require_shadow_match(bool matches, const std::string& field) {
    if (!matches) {
        throw std::runtime_error("DepthSampler sort shadow mismatch: " + field);
    }
}

// Shadow mode: every field of the two samples, and the two sorted arrays, must be bit-equal.
void verify_sort_shadow(
    const DepthSample& expected,
    const DepthSample& actual,
    const std::vector<float>& expected_sorted,
    const std::vector<float>& actual_sorted) {
    require_shadow_match(expected_sorted.size() == actual_sorted.size(), "sorted_depths.size");
    for (std::size_t i = 0; i < expected_sorted.size(); ++i) {
        if (!same_float_bits(expected_sorted[i], actual_sorted[i])) {
            throw std::runtime_error(
                "DepthSampler sort shadow mismatch: sorted_depths[" + std::to_string(i) + "]");
        }
    }
    require_shadow_match(same_float_bits(expected.depth_m, actual.depth_m), "depth_m");
    for (std::size_t i = 0; i < expected.centroid_3d.size(); ++i) {
        require_shadow_match(
            same_float_bits(expected.centroid_3d[i], actual.centroid_3d[i]),
            "centroid_3d[" + std::to_string(i) + "]");
    }
    require_shadow_match(expected.sample_region_pixel_count == actual.sample_region_pixel_count,
                       "sample_region_pixel_count");
    require_shadow_match(expected.valid_pixel_count == actual.valid_pixel_count, "valid_pixel_count");
    require_shadow_match(expected.selected_pixel_count == actual.selected_pixel_count,
                       "selected_pixel_count");
    require_shadow_match(expected.required_cluster_pixel_count == actual.required_cluster_pixel_count,
                       "required_cluster_pixel_count");
    require_shadow_match(same_float_bits(expected.valid_fraction, actual.valid_fraction), "valid_fraction");
    require_shadow_match(same_float_bits(expected.selected_fraction, actual.selected_fraction),
                       "selected_fraction");
    require_shadow_match(same_float_bits(expected.selected_min_m, actual.selected_min_m), "selected_min_m");
    require_shadow_match(same_float_bits(expected.selected_median_m, actual.selected_median_m),
                       "selected_median_m");
    require_shadow_match(same_float_bits(expected.selected_max_m, actual.selected_max_m), "selected_max_m");
    require_shadow_match(same_float_bits(expected.selected_stddev_m, actual.selected_stddev_m),
                       "selected_stddev_m");
    require_shadow_match(expected.sample_source == actual.sample_source, "sample_source");
    require_shadow_match(expected.selection_status == actual.selection_status, "selection_status");
    require_shadow_match(expected.selection_fallback == actual.selection_fallback, "selection_fallback");
    require_shadow_match(expected.valid == actual.valid, "valid");
}

// The depth at index floor(percentile x (n - 1)) of the sorted values, with a band of
// +-band_width_m / 2 around it; the band decides which pixels feed the centroid and statistics.
DepthSelection percentile_selection(const std::vector<float>& sorted, float percentile, float band_width_m) {
    const std::size_t index = std::min(
        sorted.size() - 1U,
        static_cast<std::size_t>(std::floor(percentile * static_cast<float>(sorted.size() - 1U))));
    const float selected = sorted[index];
    const float half_band = band_width_m * 0.5F;
    return {selected, selected - half_band, selected + half_band, 0U, false};
}

// Walks windows [d, d + cluster_width_m] from the nearest depth outward and takes the first one
// that holds max(cluster_min_pixels, ceil(cluster_min_fraction x n)) pixels. Isolated near noise
// cannot reach that count, and background seen through a hollow object (a chair) loses because
// the nearer surface qualifies first. The output is the window's
// nearest_cluster_output_percentile quantile, not its minimum. If no window qualifies, the
// percentile selection is used and flagged as a fallback.
DepthSelection nearest_cluster_selection(
    const std::vector<float>& sorted,
    const DepthSampler::Params& params) {
    const std::size_t fraction_count = static_cast<std::size_t>(
        std::ceil(params.cluster_min_fraction * static_cast<float>(sorted.size())));
    const std::size_t required =
        std::max(static_cast<std::size_t>(params.cluster_min_pixels), fraction_count);

    for (std::size_t begin = 0; begin < sorted.size(); ++begin) {
        const auto end = std::upper_bound(
            sorted.begin() + static_cast<std::ptrdiff_t>(begin),
            sorted.end(),
            sorted[begin] + params.cluster_width_m);
        const std::size_t count = static_cast<std::size_t>(
            std::distance(sorted.begin() + static_cast<std::ptrdiff_t>(begin), end));
        if (count >= required) {
            const std::size_t output_offset = std::min(
                count - 1U,
                static_cast<std::size_t>(std::floor(
                    params.nearest_cluster_output_percentile *
                    static_cast<float>(count - 1U))));
            return {
                sorted[begin + output_offset],
                sorted[begin],
                *(end - 1),
                required,
                false};
        }
    }
    DepthSelection fallback = percentile_selection(sorted, params.percentile, params.cluster_width_m);
    fallback.required_count = required;
    fallback.used_fallback = true;
    return fallback;
}

}  // namespace

std::uint64_t depth_scan_fast_entries() noexcept { return fast_scan_entries.load(std::memory_order_relaxed); }

DepthCodeInterval depth_codes_in(float scale, float lo, float hi) {
    DepthCodeInterval out;
    if (std::isnan(lo) || std::isnan(hi) || !std::isfinite(scale) || !(scale > 0.0F)) return out;
    const auto metres = [scale](std::uint32_t code) { return static_cast<float>(code) * scale; };
    std::uint32_t a = 0U, b = 65536U;  // first code with metres >= lo
    while (a < b) {
        const std::uint32_t m = a + (b - a) / 2U;
        if (metres(m) >= lo) b = m; else a = m + 1U;
    }
    const std::uint32_t first = a;
    a = 0U;
    b = 65536U;  // first code with metres > hi
    while (a < b) {
        const std::uint32_t m = a + (b - a) / 2U;
        if (metres(m) > hi) b = m; else a = m + 1U;
    }
    if (a == 0U || first > a - 1U) return out;
    out.first = first;
    out.last = a - 1U;
    return out;
}

DepthSampler::DepthSampler(const Params& params) {
    set_params(params);
}

// An unknown mode falls back to nearest_cluster with a warning; numeric values are clamped, not
// rejected.
void DepthSampler::set_params(const Params& params) {
    params_ = params;
    if (params_.sample_mode != "nearest_cluster" && params_.sample_mode != "percentile" &&
        params_.sample_mode != "median") {
        LOG_WARN_M("DepthSampler", "unknown sample_mode=" << params_.sample_mode
                                                             << "; using nearest_cluster");
        params_.sample_mode = "nearest_cluster";
    }
    params_.percentile = std::clamp(params_.percentile, 0.01F, 0.50F);
    params_.cluster_width_m = std::clamp(params_.cluster_width_m, 0.02F, 1.0F);
    params_.cluster_min_fraction = std::clamp(params_.cluster_min_fraction, 0.001F, 1.0F);
    params_.cluster_min_pixels = std::max(1, params_.cluster_min_pixels);
    params_.nearest_cluster_output_percentile =
        std::clamp(params_.nearest_cluster_output_percentile, 0.01F, 0.50F);
}

DepthSampler::Params DepthSampler::params() const {
    return params_;
}

// Dispatch on sort_impl; only Shadow does extra work (both implementations, then the bit check).
DepthSample DepthSampler::sample(
    const cv::Mat& aligned_depth,
    const yolo::DepthIntrinsics& intrinsics,
    float depth_scale,
    const yolo::Detection& detection,
    DepthSampleProfile* profile,
    std::vector<float>* sorted_snapshot,
    std::vector<std::uint32_t>* selected_indices) const {
    if (params_.sort_impl != SortImplementation::Shadow) {
        return sample_impl(aligned_depth, intrinsics, depth_scale, detection,
                          params_.sort_impl, profile, sorted_snapshot, selected_indices);
    }

    // Each complete call releases its scratch references before the next call.
    // Owning snapshots survive that scratch reuse; Shadow is diagnostic only.
    DepthSampleProfile std_profile;
    DepthSampleProfile code_profile;
    std::vector<float> std_sorted;
    std::vector<float> code_sorted;
    const DepthSample std_sample = sample_impl(
        aligned_depth, intrinsics, depth_scale, detection,
        SortImplementation::StdSort, &std_profile, &std_sorted, selected_indices);
    const DepthSample code_sample = sample_impl(
        aligned_depth, intrinsics, depth_scale, detection,
        SortImplementation::CodeSort, &code_profile, &code_sorted, nullptr);
    verify_sort_shadow(std_sample, code_sample, std_sorted, code_sorted);
    if (profile) {
        *profile = std_profile;
        profile->sort_shadow_checked = true;
    }
    if (sorted_snapshot) *sorted_snapshot = std::move(std_sorted);
    return std_sample;
}

// One sample, in order: check the inputs; region = box clipped to the image, read through the
// box-local mask, or without a mask the central half of the box (a quarter trimmed per side);
// keep valid depths; order them (code histogram or std::sort); select the band (mode); take the
// u/v medians of the band's pixels and back-project; fill the band statistics. The fast scan
// below and the reference path after it produce the same bits.
DepthSample DepthSampler::sample_impl(
    const cv::Mat& aligned_depth,
    const yolo::DepthIntrinsics& intrinsics,
    float depth_scale,
    const yolo::Detection& detection,
    SortImplementation sort_impl,
    DepthSampleProfile* profile,
    std::vector<float>* sorted_snapshot,
    std::vector<std::uint32_t>* selected_indices) const {
    DepthSampleProfileScope timing(profile);
    if (sorted_snapshot) sorted_snapshot->clear();
    if (selected_indices) selected_indices->clear();
    // Row-major pixel index into aligned_depth, the form ObjectGeometryBuilder reads.
    const auto pixel_index = [&aligned_depth](int x, int y) {
        return static_cast<std::uint32_t>(y) * static_cast<std::uint32_t>(aligned_depth.cols) +
               static_cast<std::uint32_t>(x);
    };

    const bool use_code_sort = sort_impl == SortImplementation::CodeSort &&
        aligned_depth.type() == CV_16UC1 && std::isfinite(depth_scale) && depth_scale > 0.0F;
    std::vector<std::uint32_t>* code_counts = nullptr;
    std::uint32_t min_code = 65536U;
    std::uint32_t max_code = 0U;
    // The fast scan (below) keeps its own exactly-cleared histogram and never reads this one.
    const bool use_fast_scan = depth_scan_fast_enabled() && use_code_sort && profile == nullptr &&
        sorted_snapshot == nullptr;
    if (use_code_sort && !use_fast_scan) {
        // Lazily allocated per calling thread/lane. Reset during setup, before
        // every early return; ineligible/StdSort calls never access these bins.
        // Only the reference code-sort path reads it, so the fast scan skips this 256 KB
        // clear (P2, 2026-10-04: 14-87 us per sample on 1.8, bit-identical outputs).
        thread_local std::vector<std::uint32_t> histogram(65536U);
        std::fill(histogram.begin(), histogram.end(), 0U);
        code_counts = &histogram;
    }
    DepthSample output;
    if (aligned_depth.empty() || intrinsics.fx <= 0.0F || intrinsics.fy <= 0.0F || depth_scale <= 0.0F) {
        LOG_WARN_M("DepthSampler", "invalid depth image/intrinsics/scale; skipping sample");
        return output;
    }
    if (aligned_depth.type() != CV_16UC1 && aligned_depth.type() != CV_32FC1) {
        LOG_ERROR_M("DepthSampler", "unsupported depth type=" << aligned_depth.type());
        return output;
    }

    cv::Rect bounds(0, 0, aligned_depth.cols, aligned_depth.rows);
    cv::Rect roi = detection.bbox & bounds;
    if (roi.empty()) {
        output.selection_status = "bbox_outside";
        LOG_WARN_M("DepthSampler", "detection bbox outside depth image");
        return output;
    }
    const bool use_mask = !detection.mask.empty() && detection.mask.size() == detection.bbox.size();
    output.sample_source = use_mask ? "mask" : "bbox_center";
    if (!use_mask) {
        const int margin_x = roi.width / 4;
        const int margin_y = roi.height / 4;
        roi.x += margin_x;
        roi.y += margin_y;
        roi.width -= 2 * margin_x;
        roi.height -= 2 * margin_y;
    }

    // Production samples on two lanes (sup_process and depth_lane2, see
    // FixedTwoLaneDepthSampler). Per-thread scratch keeps DepthSampler safe across threads
    // while avoiding several heap allocations for every tracked object and frame.
    thread_local DepthSamplerScratch scratch;
    scratch.clear();
    if (use_fast_scan) {
        fast_scan_entries.fetch_add(1, std::memory_order_relaxed);
        // Same pixels, same scan order, same code*scale float arithmetic and the
        // same code histogram as the legacy path, without storing every valid
        // pixel: pass 1 counts, pass 2 collects only the selected band in the
        // identical row-major order. Both passes test codes against intervals
        // (depth_codes_in) instead of converting each pixel to metres. Histogram bins used by the previous call on
        // this lane are cleared exactly, instead of all 65536 bins.
        thread_local std::vector<std::uint32_t> fast_histogram(65536U, 0U);
        thread_local std::uint32_t fast_min = 65536U, fast_max = 0U;
        if (fast_min <= fast_max) {
            std::fill(fast_histogram.begin() + fast_min, fast_histogram.begin() + fast_max + 1U, 0U);
        }
        fast_min = 65536U;
        fast_max = 0U;
        // Valid depth (finite, within [MIN_DEPTH_M, MAX_DEPTH_M]) as a code interval: a finite
        // MAX_DEPTH_M also excludes +inf, and the product is never NaN for this scale.
        const DepthCodeInterval valid_codes = depth_codes_in(depth_scale, MIN_DEPTH_M, MAX_DEPTH_M);
        const int x0 = roi.x, x1 = roi.x + roi.width;
        std::size_t valid_count = 0U;
        for (int y = roi.y; y < roi.y + roi.height; ++y) {
            const std::uint16_t* depth_row = aligned_depth.ptr<std::uint16_t>(y);
            const std::uint8_t* mask_row = use_mask ? detection.mask.ptr<std::uint8_t>(y - detection.bbox.y) : nullptr;
            int x = x0;
#if defined(__aarch64__)
            // Sixteen pixels at a time while a full block fits; an empty interval (first > last)
            // keeps nothing and is left to the loop below.
            if (valid_codes.first <= valid_codes.last) {
                const uint16x8_t first = vdupq_n_u16(static_cast<std::uint16_t>(valid_codes.first));
                const uint16x8_t last = vdupq_n_u16(static_cast<std::uint16_t>(valid_codes.last));
                for (; x + 16 <= x1; x += 16) {
                    const uint8x16_t region = region_bytes(mask_row ? mask_row + (x - detection.bbox.x) : nullptr);
                    const std::uint64_t in_region = pixel_nibbles(region);
                    if (in_region == 0U) continue;
                    output.sample_region_pixel_count += __builtin_popcountll(in_region) >> 2;
                    std::uint64_t keep = pixel_nibbles(vandq_u8(region, code_in_bytes(depth_row + x, first, last)));
                    valid_count += static_cast<std::size_t>(__builtin_popcountll(keep) >> 2);
                    while (keep != 0U) {
                        const int pixel = first_kept_pixel(keep);
                        keep = without_pixel(keep, pixel);
                        const std::uint32_t code = depth_row[x + pixel];
                        ++fast_histogram[code];
                        fast_min = std::min<std::uint32_t>(fast_min, code);
                        fast_max = std::max<std::uint32_t>(fast_max, code);
                    }
                }
            }
#endif
            for (; x < x1; ++x) {
                if (mask_row && mask_row[x - detection.bbox.x] == 0U) continue;
                ++output.sample_region_pixel_count;
                const std::uint32_t code = depth_row[x];
                if (code < valid_codes.first || code > valid_codes.last) continue;
                ++valid_count;
                ++fast_histogram[code];
                fast_min = std::min<std::uint32_t>(fast_min, code);
                fast_max = std::max<std::uint32_t>(fast_max, code);
            }
        }
        output.valid_pixel_count = static_cast<int>(valid_count);
        if (output.sample_region_pixel_count > 0) {
            output.valid_fraction =
                static_cast<float>(output.valid_pixel_count) /
                static_cast<float>(output.sample_region_pixel_count);
        }
        if (valid_count == 0U) {
            output.selection_status = "no_valid_depth";
            LOG_DEBUG_M("DepthSampler", "no valid depth pixels for detection");
            return output;
        }
        auto& sorted_depths = scratch.sorted_depths;
        sorted_depths.resize(valid_count);
        {
            auto output_depth = sorted_depths.begin();
            for (std::uint32_t code = fast_min; code <= fast_max; ++code) {
                const std::uint32_t count = fast_histogram[code];
                if (count == 0U) continue;
                output_depth = std::fill_n(output_depth, count,
                    code_depth_meters(static_cast<std::uint16_t>(code), depth_scale));
            }
        }
        DepthSelection selection;
        if (params_.sample_mode == "median") {
            const float selected = sorted_depths[sorted_depths.size() / 2U];
            selection = {selected, sorted_depths.front(), sorted_depths.back(), 0U, false};
            output.selection_status = "median";
        } else if (params_.sample_mode == "percentile") {
            selection = percentile_selection(sorted_depths, params_.percentile, params_.cluster_width_m);
            output.selection_status = "percentile";
        } else {
            selection = nearest_cluster_selection(sorted_depths, params_);
            output.selection_status =
                selection.used_fallback ? "percentile_fallback" : "nearest_cluster";
        }
        const auto selected_sorted_begin =
            std::lower_bound(sorted_depths.begin(), sorted_depths.end(), selection.lower_m);
        const auto selected_sorted_end =
            std::upper_bound(selected_sorted_begin, sorted_depths.end(), selection.upper_m);
        const std::size_t selected_sorted_count = static_cast<std::size_t>(
            std::distance(selected_sorted_begin, selected_sorted_end));
        if (selected_sorted_count == 0U) {
            output.selection_status = "selection_empty";
            LOG_ERROR_M("DepthSampler", "depth selection unexpectedly produced zero pixels");
            return output;
        }
        // The selected pixels' u and v are whole pixel coordinates inside the roi, so their
        // medians (the element at index n/2 after ordering, as median_in_place) are read
        // from per-column and per-row counts instead of ordering two n-element arrays
        // (optimization round 2, 2026-10-04): the same values, one pass. The depths are
        // still kept in scan order for the order-dependent sum and variance below.
        thread_local std::vector<std::uint32_t> column_counts, row_counts;
        column_counts.assign(static_cast<std::size_t>(roi.width), 0U);
        row_counts.assign(static_cast<std::size_t>(roi.height), 0U);
        // The second pass writes through pointers: it keeps a subset of the valid pixels, so
        // valid_count entries (plus one block of slack for a sixteen-wide store) always suffice.
        // The depths go to a per-lane buffer that only grows (nothing reads beyond what this call
        // wrote); the caller's list is cut to what was written after the pass.
        thread_local std::vector<float> selected_depth_buffer;
        if (selected_depth_buffer.size() < valid_count + 16U) selected_depth_buffer.resize(valid_count + 16U);
        if (selected_indices) selected_indices->resize(valid_count + 16U);
        float* const selected_depths = selected_depth_buffer.data();
        float* depth_out = selected_depths;
        std::uint32_t* index_out = selected_indices ? selected_indices->data() : nullptr;
        // Valid and inside [lower_m, upper_m]: the intersection of the two code intervals.
        const DepthCodeInterval band = depth_codes_in(depth_scale, selection.lower_m, selection.upper_m);
        const std::uint32_t keep_first = std::max(band.first, valid_codes.first);
        const std::uint32_t keep_last = std::min(band.last, valid_codes.last);
        for (int y = roi.y; y < roi.y + roi.height; ++y) {
            const std::uint16_t* depth_row = aligned_depth.ptr<std::uint16_t>(y);
            const std::uint8_t* mask_row = use_mask ? detection.mask.ptr<std::uint8_t>(y - detection.bbox.y) : nullptr;
            const std::uint32_t row_index = pixel_index(0, y);
            std::uint32_t row_kept = 0U;
            int x = x0;
#if defined(__aarch64__)
            if (keep_first <= keep_last) {
                const uint16x8_t first = vdupq_n_u16(static_cast<std::uint16_t>(keep_first));
                const uint16x8_t last = vdupq_n_u16(static_cast<std::uint16_t>(keep_last));
                const float32x4_t scale = vdupq_n_f32(depth_scale);
                const uint32x4_t one = vdupq_n_u32(1U);
                const std::uint32_t lane_values[4] = {0U, 1U, 2U, 3U};
                const uint32x4_t lanes = vld1q_u32(lane_values);
                for (; x + 16 <= x1; x += 16) {
                    const uint8x16_t region = region_bytes(mask_row ? mask_row + (x - detection.bbox.x) : nullptr);
                    std::uint64_t keep = pixel_nibbles(vandq_u8(region, code_in_bytes(depth_row + x, first, last)));
                    if (keep == 0U) continue;
                    if (keep == kAllSixteen) {
                        // All sixteen kept: the same float(code) * scale per pixel, a single
                        // IEEE multiplication either way, written in pixel order.
                        const uint16x8_t lo = vld1q_u16(depth_row + x), hi = vld1q_u16(depth_row + x + 8);
                        vst1q_f32(depth_out, vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_low_u16(lo))), scale));
                        vst1q_f32(depth_out + 4, vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(lo))), scale));
                        vst1q_f32(depth_out + 8, vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_low_u16(hi))), scale));
                        vst1q_f32(depth_out + 12, vmulq_f32(vcvtq_f32_u32(vmovl_u16(vget_high_u16(hi))), scale));
                        depth_out += 16;
                        std::uint32_t* columns = column_counts.data() + (x - x0);
                        for (int k = 0; k < 16; k += 4) vst1q_u32(columns + k, vaddq_u32(vld1q_u32(columns + k), one));
                        if (index_out) {
                            const uint32x4_t base = vaddq_u32(vdupq_n_u32(row_index + static_cast<std::uint32_t>(x)), lanes);
                            vst1q_u32(index_out, base);
                            vst1q_u32(index_out + 4, vaddq_u32(base, vdupq_n_u32(4U)));
                            vst1q_u32(index_out + 8, vaddq_u32(base, vdupq_n_u32(8U)));
                            vst1q_u32(index_out + 12, vaddq_u32(base, vdupq_n_u32(12U)));
                            index_out += 16;
                        }
                        row_kept += 16U;
                        continue;
                    }
                    while (keep != 0U) {
                        const int pixel = first_kept_pixel(keep);
                        keep = without_pixel(keep, pixel);
                        ++column_counts[static_cast<std::size_t>(x + pixel - x0)];
                        ++row_kept;
                        *depth_out++ = code_depth_meters(depth_row[x + pixel], depth_scale);
                        if (index_out) *index_out++ = row_index + static_cast<std::uint32_t>(x + pixel);
                    }
                }
            }
#endif
            for (; x < x1; ++x) {
                if (mask_row && mask_row[x - detection.bbox.x] == 0U) continue;
                const std::uint32_t code = depth_row[x];
                if (code < keep_first || code > keep_last) continue;
                ++column_counts[static_cast<std::size_t>(x - x0)];
                ++row_kept;
                *depth_out++ = code_depth_meters(static_cast<std::uint16_t>(code), depth_scale);
                if (index_out) *index_out++ = row_index + static_cast<std::uint32_t>(x);
            }
            row_counts[static_cast<std::size_t>(y - roi.y)] += row_kept;
        }
        const std::size_t selected_count = static_cast<std::size_t>(depth_out - selected_depths);
        if (selected_indices) selected_indices->resize(selected_count);
        if (selected_count == 0U) {
            output.selection_status = "selection_empty";
            LOG_ERROR_M("DepthSampler", "depth selection unexpectedly produced zero pixels");
            return output;
        }
        const float depth_m = selection.depth_m;
        // First coordinate whose running count passes index n/2 (counts sum to n).
        const auto median_coordinate = [selected_count](const std::vector<std::uint32_t>& counts, int origin) {
            const std::size_t rank = selected_count / 2U;
            std::size_t seen = 0U;
            for (std::size_t i = 0; i < counts.size(); ++i) {
                seen += counts[i];
                if (seen > rank) return static_cast<float>(origin + static_cast<int>(i));
            }
            return static_cast<float>(origin + static_cast<int>(counts.size()) - 1);
        };
        const float u = median_coordinate(column_counts, x0);
        const float v = median_coordinate(row_counts, roi.y);
        float selected_sum = 0.0F;
        for (std::size_t i = 0; i < selected_count; ++i) selected_sum += selected_depths[i];
        const float selected_mean = selected_sum / static_cast<float>(selected_count);
        float selected_squared_error_sum = 0.0F;
        for (std::size_t i = 0; i < selected_count; ++i) {
            const float error = selected_depths[i] - selected_mean;
            selected_squared_error_sum += error * error;
        }
        output.depth_m = depth_m;
        output.centroid_3d = {
            (u - intrinsics.cx) * depth_m / intrinsics.fx,
            (v - intrinsics.cy) * depth_m / intrinsics.fy,
            depth_m};
        output.selected_pixel_count = static_cast<int>(selected_count);
        output.required_cluster_pixel_count = static_cast<int>(selection.required_count);
        output.selected_fraction =
            static_cast<float>(selected_count) / static_cast<float>(valid_count);
        output.selected_min_m = *selected_sorted_begin;
        output.selected_median_m =
            *(selected_sorted_begin + static_cast<std::ptrdiff_t>(selected_sorted_count / 2U));
        output.selected_max_m = *(selected_sorted_end - 1);
        output.selected_stddev_m =
            std::sqrt(selected_squared_error_sum / static_cast<float>(selected_count));
        output.selection_fallback = selection.used_fallback;
        output.valid = true;
        return output;
    }
    // Reference path: keeps every valid pixel (depth, u, v), then sorts. Taken whenever the fast
    // scan is not: fast scan off, std::sort, float depth, a profile or a sorted snapshot.
    auto& pixels = scratch.pixels;
    pixels.reserve(static_cast<std::size_t>(roi.area()));
    if (profile) {
        profile->roi_pixels = static_cast<std::uint64_t>(roi.width) *
                              static_cast<std::uint64_t>(roi.height);
    }
    timing.next(&DepthSampleProfile::scan_ns);
    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            const int mask_y = y - detection.bbox.y;
            const int mask_x = x - detection.bbox.x;
            if (use_mask && detection.mask.at<std::uint8_t>(mask_y, mask_x) == 0U) {
                continue;
            }
            ++output.sample_region_pixel_count;
            const float depth_m = depth_meters(aligned_depth, y, x, depth_scale);
            if (!std::isfinite(depth_m) || depth_m < MIN_DEPTH_M || depth_m > MAX_DEPTH_M) {
                continue;
            }
            pixels.push_back({depth_m, static_cast<float>(x), static_cast<float>(y)});
            if (code_counts) {
                const std::uint32_t code = aligned_depth.at<std::uint16_t>(y, x);
                ++(*code_counts)[code];
                min_code = std::min(min_code, code);
                max_code = std::max(max_code, code);
            }
        }
    }
    if (profile) {
        profile->region_pixels = static_cast<std::uint64_t>(output.sample_region_pixel_count);
        profile->valid_pixels = static_cast<std::uint64_t>(pixels.size());
    }
    output.valid_pixel_count = static_cast<int>(pixels.size());
    if (output.sample_region_pixel_count > 0) {
        output.valid_fraction =
            static_cast<float>(output.valid_pixel_count) /
            static_cast<float>(output.sample_region_pixel_count);
    }
    if (pixels.empty()) {
        output.selection_status = "no_valid_depth";
        LOG_DEBUG_M("DepthSampler", "no valid depth pixels for detection");
        return output;
    }

    timing.next(&DepthSampleProfile::depth_copy_ns);
    auto& sorted_depths = scratch.sorted_depths;
    if (use_code_sort) {
        sorted_depths.resize(pixels.size());
    } else {
        // Preserve the independent original float-copy and std::sort path.
        sorted_depths.reserve(pixels.size());
        for (const auto& pixel : pixels) {
            sorted_depths.push_back(pixel.depth_m);
        }
    }
    timing.next(&DepthSampleProfile::sort_ns);
    if (use_code_sort) {
        auto output_depth = sorted_depths.begin();
        // The wider index must advance past raw code 65535 without wrapping.
        for (std::uint32_t code = min_code; code <= max_code; ++code) {
            const std::uint32_t count = (*code_counts)[code];
            if (count == 0U) continue;
            output_depth = std::fill_n(
                output_depth, count,
                code_depth_meters(static_cast<std::uint16_t>(code), depth_scale));
        }
        if (profile) profile->code_sort_used = true;
    } else {
        std::sort(sorted_depths.begin(), sorted_depths.end());
    }
    if (sorted_snapshot) *sorted_snapshot = sorted_depths;

    timing.next(&DepthSampleProfile::cluster_selection_ns);
    DepthSelection selection;
    if (params_.sample_mode == "median") {
        const float selected = sorted_depths[sorted_depths.size() / 2U];
        selection = {selected, sorted_depths.front(), sorted_depths.back(), 0U, false};
        output.selection_status = "median";
    } else if (params_.sample_mode == "percentile") {
        selection = percentile_selection(sorted_depths, params_.percentile, params_.cluster_width_m);
        output.selection_status = "percentile";
    } else {
        selection = nearest_cluster_selection(sorted_depths, params_);
        output.selection_status =
            selection.used_fallback ? "percentile_fallback" : "nearest_cluster";
    }

    const auto selected_sorted_begin =
        std::lower_bound(sorted_depths.begin(), sorted_depths.end(), selection.lower_m);
    const auto selected_sorted_end =
        std::upper_bound(selected_sorted_begin, sorted_depths.end(), selection.upper_m);
    const std::size_t selected_sorted_count = static_cast<std::size_t>(
        std::distance(selected_sorted_begin, selected_sorted_end));
    if (selected_sorted_count == 0U) {
        output.selection_status = "selection_empty";
        LOG_ERROR_M("DepthSampler", "depth selection unexpectedly produced zero pixels");
        return output;
    }

    timing.next(&DepthSampleProfile::select_pixels_ns);
    auto& selected_us = scratch.selected_us;
    auto& selected_vs = scratch.selected_vs;
    auto& selected_depths = scratch.selected_depths;
    selected_us.reserve(selected_sorted_count);
    selected_vs.reserve(selected_sorted_count);
    selected_depths.reserve(selected_sorted_count);
    if (selected_indices) selected_indices->reserve(selected_sorted_count);
    for (const auto& pixel : pixels) {
        if (pixel.depth_m >= selection.lower_m && pixel.depth_m <= selection.upper_m) {
            selected_us.push_back(pixel.u);
            selected_vs.push_back(pixel.v);
            selected_depths.push_back(pixel.depth_m);
            // u and v hold whole pixel coordinates, so the conversion back is exact.
            if (selected_indices) {
                selected_indices->push_back(pixel_index(static_cast<int>(pixel.u), static_cast<int>(pixel.v)));
            }
        }
    }
    if (selected_depths.empty()) {
        output.selection_status = "selection_empty";
        LOG_ERROR_M("DepthSampler", "depth selection unexpectedly produced zero pixels");
        return output;
    }
    const std::size_t selected_count = selected_depths.size();
    const float depth_m = selection.depth_m;
    if (profile) profile->selected_pixels = static_cast<std::uint64_t>(selected_count);
    timing.next(&DepthSampleProfile::median_u_ns);
    const float u = median_in_place(selected_us);
    timing.next(&DepthSampleProfile::median_v_ns);
    const float v = median_in_place(selected_vs);
    timing.next(&DepthSampleProfile::statistics_ns);
    float selected_sum = 0.0F;
    for (const float value : selected_depths) {
        selected_sum += value;
    }
    const float selected_mean = selected_sum / static_cast<float>(selected_count);
    float selected_squared_error_sum = 0.0F;
    for (const float value : selected_depths) {
        const float error = value - selected_mean;
        selected_squared_error_sum += error * error;
    }
    timing.next(&DepthSampleProfile::finalize_ns);
    output.depth_m = depth_m;
    output.centroid_3d = {
        (u - intrinsics.cx) * depth_m / intrinsics.fx,
        (v - intrinsics.cy) * depth_m / intrinsics.fy,
        depth_m};
    output.selected_pixel_count = static_cast<int>(selected_count);
    output.required_cluster_pixel_count = static_cast<int>(selection.required_count);
    output.selected_fraction =
        static_cast<float>(selected_count) / static_cast<float>(pixels.size());
    output.selected_min_m = *selected_sorted_begin;
    output.selected_median_m =
        *(selected_sorted_begin + static_cast<std::ptrdiff_t>(selected_sorted_count / 2U));
    output.selected_max_m = *(selected_sorted_end - 1);
    output.selected_stddev_m =
        std::sqrt(selected_squared_error_sum / static_cast<float>(selected_count));
    output.selection_fallback = selection.used_fallback;
    output.valid = true;
    LOG_DEBUG_M("DepthSampler", "sampled mode=" << params_.sample_mode << " depth_m=" << output.depth_m
                                                   << " valid_pixels=" << output.valid_pixel_count
                                                   << " valid_fraction=" << output.valid_fraction
                                                   << " selected_pixels=" << output.selected_pixel_count
                                                   << " selected_fraction=" << output.selected_fraction
                                                   << " required_cluster_pixels="
                                                   << output.required_cluster_pixel_count
                                                   << " sample_source=" << output.sample_source
                                                   << " selection_status=" << output.selection_status
                                                   << " selection_fallback=" << output.selection_fallback);
    return output;
}

TrackDepthHysteresis::TrackDepthHysteresis(const Params& params) {
    set_params(params);
}

void TrackDepthHysteresis::set_params(const Params& params) {
    params_ = params;
    params_.jump_threshold_m = std::clamp(params_.jump_threshold_m, 0.01F, 5.0F);
    params_.candidate_tolerance_m =
        std::clamp(params_.candidate_tolerance_m, 0.01F, 5.0F);
    params_.confirmation_frames = std::max(1, params_.confirmation_frames);
    reset();
}

TrackDepthHysteresis::Params TrackDepthHysteresis::params() const {
    return params_;
}

DepthSample TrackDepthHysteresis::update(int track_id, const DepthSample& raw_sample) {
    if (!raw_sample.valid) {
        const auto state_it = states_.find(track_id);
        if (state_it != states_.end()) {
            state_it->second.candidate_frames = 0;
        }
        return raw_sample;
    }

    const auto [state_it, inserted] = states_.try_emplace(track_id);
    State& state = state_it->second;
    if (inserted || params_.confirmation_frames <= 1) {
        state.accepted = raw_sample;
        state.candidate_frames = 0;
        return raw_sample;
    }

    // Within jump_threshold_m of the accepted depth: a normal change, accepted at once.
    if (std::fabs(raw_sample.depth_m - state.accepted.depth_m) <=
        params_.jump_threshold_m) {
        state.accepted = raw_sample;
        state.candidate_frames = 0;
        return raw_sample;
    }

    // A jump. It counts only while each new sample stays within candidate_tolerance_m of the
    // previous candidate (the band follows the latest candidate); otherwise counting restarts.
    // The accepted depth is published until the count reaches confirmation_frames.
    if (state.candidate_frames == 0 ||
        std::fabs(raw_sample.depth_m - state.candidate.depth_m) >
            params_.candidate_tolerance_m) {
        state.candidate = raw_sample;
        state.candidate_frames = 1;
        return state.accepted;
    }

    state.candidate = raw_sample;
    ++state.candidate_frames;
    if (state.candidate_frames >= params_.confirmation_frames) {
        state.accepted = state.candidate;
        state.candidate_frames = 0;
        return state.accepted;
    }
    return state.accepted;
}

void TrackDepthHysteresis::retain(const std::unordered_set<int>& active_track_ids) {
    for (auto state_it = states_.begin(); state_it != states_.end();) {
        if (active_track_ids.find(state_it->first) == active_track_ids.end()) {
            state_it = states_.erase(state_it);
        } else {
            ++state_it;
        }
    }
}

void TrackDepthHysteresis::reset() {
    states_.clear();
}

}  // namespace perception
