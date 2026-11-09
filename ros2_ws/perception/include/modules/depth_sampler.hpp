/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: depth_sampler.hpp
 * Brief: One depth and camera-frame centroid per detection, from the depth registered onto RGB.
 *
 * Description:
 * What problem this solves. A tracked object needs one distance and one 3D point. Reading the
 * depth under the mask (or the central half of the box when there is no mask) gives many values
 * with holes, background and edge bleeding mixed in; DepthSampler picks a band of them and
 * reports its statistics.
 * How. Valid pixels (0.3-10 m) are sorted; the default mode, nearest_cluster, takes the nearest
 * depth window of cluster_width_m that holds enough pixels and outputs a low quantile of it
 * (median and percentile are the other modes). The centroid back-projects the median pixel of
 * the band through the pinhole intrinsics. TrackDepthHysteresis then holds a track's depth
 * across single-frame jumps. The depth is the Gemini 338Le's, registered onto the 1280x800 RGB
 * image (SW-D2C); the type itself is camera-neutral.
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/core.hpp>

#include "common/types.hpp"

namespace perception {

// Result of one sample. depth_m is the selected quantile of the band, centroid_3d the band's
// median pixel back-projected at depth_m (camera optical frame, metres); valid is false on every
// early return, and selection_status says which.
struct DepthSample {
    float depth_m = 0.0F;
    std::array<float, 3> centroid_3d = {0.0F, 0.0F, 0.0F};
    // Number of mask/ROI pixels considered, then valid pixels before and
    // after depth-band selection.
    int sample_region_pixel_count = 0;
    int valid_pixel_count = 0;
    int selected_pixel_count = 0;
    int required_cluster_pixel_count = 0;
    float valid_fraction = 0.0F;
    float selected_fraction = 0.0F;
    float selected_min_m = 0.0F;
    float selected_median_m = 0.0F;
    float selected_max_m = 0.0F;
    float selected_stddev_m = 0.0F;
    // Diagnostic-only provenance. These fields never change the selected
    // depth and let field analysis distinguish sparse masks from bad depth.
    std::string sample_source = "unavailable";
    std::string selection_status = "invalid_input";
    // True only when nearest_cluster found no sufficiently supported band and
    // had to use the configured percentile fallback.
    bool selection_fallback = false;
    bool valid = false;
};

// Opt-in diagnostic only. Wall stages are mutually exclusive and sum exactly
// to total_wall_ns. Unvisited stages remain zero, including on an early return.
// Thread CPU time uses Linux CLOCK_THREAD_CPUTIME_ID for this sample's caller;
// false/zero means unavailable, not zero work. Wall minus CPU is not a causal
// measure of contention. Invalid wall-clock readings clear all wall fields.
struct DepthSampleProfile {
    std::uint64_t start_ns = 0;
    std::uint64_t end_ns = 0;
    std::uint64_t total_wall_ns = 0;
    std::uint64_t thread_cpu_ns = 0;
    std::uint64_t setup_ns = 0;
    std::uint64_t scan_ns = 0;
    std::uint64_t depth_copy_ns = 0;
    std::uint64_t sort_ns = 0;
    std::uint64_t cluster_selection_ns = 0;
    std::uint64_t select_pixels_ns = 0;
    std::uint64_t median_u_ns = 0;
    std::uint64_t median_v_ns = 0;
    std::uint64_t statistics_ns = 0;
    std::uint64_t finalize_ns = 0;
    // roi is the final scanned rectangle, after the bbox-center fallback.
    // roi >= region (mask accepted) >= valid (depth accepted) >= selected.
    std::uint64_t roi_pixels = 0;
    std::uint64_t region_pixels = 0;
    std::uint64_t valid_pixels = 0;
    std::uint64_t selected_pixels = 0;
    bool wall_clock_valid = false;
    bool cpu_clock_valid = false;
    bool code_sort_used = false;
    bool sort_shadow_checked = false;
};

class DepthSampler {
public:
    // How the valid depths are ordered. StdSort sorts floats; CodeSort counts the raw 16-bit
    // codes in a histogram (same order, bit-identical result; production sets codesort through
    // the runtime section); Shadow runs both and throws on any difference (diagnostic).
    enum class SortImplementation { StdSort, CodeSort, Shadow };

    // Defaults below; the service sets them from the depth section of the configuration, and
    // set_params() clamps each one to a sane range.
    struct Params {
        // nearest_cluster (default), percentile or median.
        std::string sample_mode = "nearest_cluster";
        // Quantile used by the percentile mode and by the nearest_cluster fallback.
        float percentile = 0.20F;
        // Width of a cluster window, and of the band around a percentile pick.
        float cluster_width_m = 0.12F;
        // A cluster needs max(cluster_min_pixels, cluster_min_fraction x valid pixels) pixels.
        float cluster_min_fraction = 0.08F;
        int cluster_min_pixels = 8;
        // Robust near-surface quantile inside a cluster that already passed
        // the density gate. This is intentionally above the raw minimum.
        float nearest_cluster_output_percentile = 0.10F;
        SortImplementation sort_impl = SortImplementation::StdSort;
    };

    DepthSampler() = default;
    explicit DepthSampler(const Params& params);

    void set_params(const Params& params);
    Params params() const;

    /**
     * @brief Samples mask pixels, or a central bbox region when mask is empty.
     * sorted_snapshot, when supplied, owns a copy for diagnostics/tests only;
     * its copy cost is included in the profile, so it is not a performance mode.
     * Shadow runs both implementations and returns the StdSort sample/profile.
     * Safe to call concurrently on one sampler: params_ is only read and every scratch buffer
     * is per thread (production samples on two lanes, see FixedTwoLaneDepthSampler).
     * selected_indices, when supplied (2026-10-06), receives the selected band's pixels as
     * row-major indices into aligned_depth (y * cols + x), in scan order; it is cleared first and
     * left empty when the sample is not valid. ObjectGeometryBuilder then reuses this list
     * instead of scanning the region again; the caller owns it (never per-thread scratch).
     */
    DepthSample sample(
        const cv::Mat& aligned_depth,
        const yolo::DepthIntrinsics& intrinsics,
        float depth_scale,
        const yolo::Detection& detection,
        DepthSampleProfile* profile = nullptr,
        std::vector<float>* sorted_snapshot = nullptr,
        std::vector<std::uint32_t>* selected_indices = nullptr) const;

private:
    DepthSample sample_impl(
        const cv::Mat& aligned_depth,
        const yolo::DepthIntrinsics& intrinsics,
        float depth_scale,
        const yolo::Detection& detection,
        SortImplementation sort_impl,
        DepthSampleProfile* profile,
        std::vector<float>* sorted_snapshot,
        std::vector<std::uint32_t>* selected_indices) const;

    // Depths outside [MIN_DEPTH_M, MAX_DEPTH_M] (and non-finite ones) are not valid pixels.
    static constexpr float MIN_DEPTH_M = 0.3F;
    static constexpr float MAX_DEPTH_M = 10.0F;
    Params params_;
};

/**
 * @brief The 16-bit depth codes whose depth in metres, float(code) * scale, lies in [lo, hi].
 *
 * For a positive finite scale that depth is non-decreasing in the code (exact integer-to-float
 * conversion, monotone IEEE rounding; an overflow to +inf still orders last), so the codes form
 * one interval [first, last], found by two binary searches. first > last: no code (also for a NaN
 * bound or an unusable scale). The fast depth scan compares codes against it instead of
 * converting every pixel (2026-10-06); the selected pixels are the same.
 */
struct DepthCodeInterval {
    std::uint32_t first = 1;
    std::uint32_t last = 0;
};
DepthCodeInterval depth_codes_in(float scale, float lo, float hi);

/**
 * @brief Holds a track's last accepted depth across isolated large cluster jumps.
 *
 * Normal frame-to-frame depth changes pass through immediately. A jump larger
 * than jump_threshold_m must remain in the same candidate band for
 * confirmation_frames before it replaces the accepted sample.
 * Per track: the first valid sample is accepted at once; while a jump is unconfirmed, update()
 * returns the accepted (older) sample; an invalid sample passes through unchanged and restarts
 * the count. retain() forgets tracks that are gone. Updated only on the sup_process thread.
 * confirmation_frames counts update() calls with valid samples (depth updates of the track), not
 * camera frames. Production sets 6, chosen 2026-07-28 from a D435 occlusion replay (GPT 3.1).
 */
class TrackDepthHysteresis {
public:
    struct Params {
        float jump_threshold_m = 0.20F;
        float candidate_tolerance_m = 0.12F;
        int confirmation_frames = 2;
    };

    TrackDepthHysteresis() = default;
    explicit TrackDepthHysteresis(const Params& params);

    void set_params(const Params& params);
    Params params() const;
    DepthSample update(int track_id, const DepthSample& raw_sample);
    void retain(const std::unordered_set<int>& active_track_ids);
    void reset();

private:
    struct State {
        DepthSample accepted;
        DepthSample candidate;
        int candidate_frames = 0;
    };

    Params params_;
    std::unordered_map<int, State> states_;
};

// How many DepthSampler::sample() calls of this process took the fast scan (runtime switch
// depth_scan_fast "1", code sort, no profile). A witness for tests: a test of the fast scan can
// show that its calls really went there instead of passing on the reference path. One relaxed
// atomic increment per sample; the service does not read it.
std::uint64_t depth_scan_fast_entries() noexcept;

}  // namespace perception
