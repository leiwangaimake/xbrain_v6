/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: bytetrack.hpp
 * Brief: ByteTrack-style multi-object tracker interface.
 *
 * Description:
 * What problem this solves. The detector reports every frame on its own; Objects needs one
 * track_id per physical object, first/last observation times and a semantic status that does
 * not flip on one bad frame (19 section 4.1, field mapping of track_id / stable_frames /
 * first_seen_mono_ms / last_seen_mono_ms / semantic_status). The tracker associates detections
 * across frames by box overlap in two rounds (ByteTrack: confident detections first, then the
 * rest against the tracks still unmatched), keeps an unmatched track for a short buffer so a
 * briefly missed object keeps its id, and publishes confirmed tracks only.
 *
 * Boundaries. Image plane only: no depth, no 3-D motion (track_velocity), no base_link geometry
 * (rns_object_geometry). Returned Track values are copies with their own masks, so callers never
 * share state with the tracker.
 *
 * Pitfalls.
 *   - Three kinds of count, easy to mix up. AGES (track_buffer_frames, max_age as the coasting
 *     limit, the ttl_frames of the output states) are differences of capture frame ids
 *     (result.frame_id): inference skips camera frames, so a track ages by every skipped frame.
 *     HIT COUNTS (consecutive_hits against min_hits / low_confidence_min_hits, the semantic
 *     streak, the semantic grace counter) count tracker updates. stable_frames is a SCORE:
 *     stable_delta() per hit, minus one per miss.
 *   - The timed update() takes the capture-monotonic time and throws on a regression BEFORE any
 *     state changes; a stream restart needs a new tracker.
 *   - Duplicate suppression may hand an older track_id to the kept publication copy. Internal
 *     tracks keep their own ids, so the public id of an object can change back when the
 *     duplicate pair ends.
 *   - first_seen_frame / last_seen_frame are int; the Objects adapter refuses frame ids above
 *     INT_MAX rather than compare wrapped values.
 */

#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "common/config.hpp"
#include "common/types.hpp"

namespace perception {

/**
 * @brief V1 ByteTrack-style tracker using two-round greedy IoU association.
 */
class ByteTracker {
public:
    // Ages are capture frame id differences, hit counts are tracker updates (see the file
    // header); confidences are detector scores in [0, 1]. The constructor clamps every value
    // into a usable range.
    struct Params {
        // Detections above this join the first association round, the rest only the second.
        float track_thresh = 0.5F;
        // How long an unmatched track stays re-associable. Raised to at least max_age and every
        // per-class max age, so a track is never forgotten while it may still be published.
        int track_buffer_frames = 30;
        // Minimum association score; the score is max(IoU, overlap / smaller box area).
        float match_thresh = 0.8F;
        // Consecutive hits that confirm a normal track; also the semantic confirmation streak.
        int min_hits = 3;
        // How long (frame ids) a confirmed track is still published while coasting. The same
        // number also caps the semantic grace counter, which counts updates, not frame ids.
        int max_age = 5;
        // Weight of the new detection box in the published, smoothed bbox_2d.
        float bbox_ema_alpha = 0.4F;
        // Per-class overrides of max_age and bbox_ema_alpha. Weak (low_confidence_candidate)
        // tracks always use the generic values: their class label is only a proxy.
        std::map<std::string, int> per_class_max_age;
        std::map<std::string, float> per_class_bbox_ema_alpha;
        // Consecutive hits that confirm a weak (generic obstacle) track.
        int low_confidence_min_hits = 5;
        // Minimum confidence of a normal detection to count toward semantic confirmation.
        float semantic_confirmation_threshold = 0.30F;
    };

    /** @brief Creates a tracker with explicit tracking parameters. */
    explicit ByteTracker(const Params& params);

    /** @brief Creates a tracker from runtime configuration values. */
    explicit ByteTracker(const yolo::Config& config);

    /**
     * @brief Updates tracker state with one frame of detections.
     *
     * The returned tracks are value copies. Each returned mask is cloned so
     * downstream modules can own it safely across pipeline boundaries.
     */
    std::vector<Track> update(const std::vector<yolo::Detection>& detections, std::uint64_t frame_id);

    // The service path. Explicit capture-monotonic time. Negative or regressing supplied time
    // throws std::invalid_argument BEFORE any tracker state is modified.
    // Equal milliseconds are valid; frame identity must still be checked by
    // the caller. Stream restart requires a new tracker (no implicit reset).
    std::vector<Track> update(const std::vector<yolo::Detection>& detections,
                              std::uint64_t frame_id, std::int64_t capture_mono_ms);

private:
    // (track_id, first_seen_frame): unique for the tracker's lifetime, unlike a published
    // track_id, which duplicate suppression may replace on the output copy.
    using TrackKey = std::pair<int, int>;
    // Two TrackKeys in sorted order, so (a, b) and (b, a) name the same pair.
    using PartialNestedPairKey = std::pair<TrackKey, TrackKey>;

    struct Match {
        std::size_t track_index = 0;
        std::size_t detection_index = 0;
        float iou = 0.0F;
    };

    // Published-box hold of one public identity: when duplicate members alternate between a
    // large and a nested box mode, a switch is held for one opportunity before it is accepted.
    struct OutputBboxModeState {
        cv::Rect accepted_bbox;
        cv::Rect pending_bbox;
        std::uint64_t last_output_frame = 0;
        bool last_low_confidence_tier = false;
        int ttl_frames = 0;
    };

    // Evidence that two same-class tracks are one object seen in two detector box modes: the
    // small weak-tier and large normal-tier roles swap between them (reciprocal swaps). Once
    // the swaps reach required_swaps the pair is treated as one object for publication.
    struct PartialNestedPairState {
        TrackKey last_small_weak_key;
        TrackKey last_large_normal_key;
        std::uint64_t last_qualifying_frame = 0;
        std::uint64_t first_reciprocal_swap_frame = 0;
        int ttl_frames = 0;
        int reciprocal_swaps = 0;
        int required_swaps = 1;
        bool equivalent = false;
    };

    static float iou(const cv::Rect& a, const cv::Rect& b);
    static int stable_delta(const std::string& class_name, float confidence);
    static std::uint64_t now_ms();

    std::vector<Track> update_impl(const std::vector<yolo::Detection>& detections,
                                 std::uint64_t frame_id,
                                 std::optional<std::int64_t> capture_mono_ms);
    Track make_track(int track_id, const yolo::Detection& detection, std::uint64_t frame_id,
                    std::optional<std::int64_t> capture_mono_ms) const;
    static Track clone_for_output(const Track& track);
    int output_max_age(const Track& track) const;

    std::vector<Match> greedy_match(
        const std::vector<Track>& tracks,
        const std::vector<std::size_t>& track_indices,
        const std::vector<yolo::Detection>& detections,
        const std::vector<std::size_t>& detection_indices) const;
    void apply_detection(Track& track, const yolo::Detection& detection, std::uint64_t frame_id,
                        std::optional<std::int64_t> capture_mono_ms) const;
    void push_output_track(const Track& track, std::vector<Track>& output) const;
    void suppress_duplicate_output_tracks(std::vector<Track>& output, std::uint64_t frame_id);
    void prune_expired_lost_tracks(std::uint64_t frame_id);
    static std::vector<std::size_t> matched_detection_positions(
        const std::vector<Match>& matches,
        const std::vector<std::size_t>& detection_indices);

    Params params_;
    int next_track_id_ = 1;
    std::uint64_t current_frame_ = 0;
    // Tracks matched in the last update, and unmatched ones still inside the buffer.
    std::vector<Track> active_tracks_;
    std::vector<Track> lost_tracks_;
    // Immutable admission provenance used only by publication-layer duplicate
    // suppression. It deliberately stays out of the public Track payload.
    std::set<TrackKey> born_weak_track_keys_;
    std::map<PartialNestedPairKey, PartialNestedPairState>
        partial_nested_pair_states_;
    std::map<TrackKey, OutputBboxModeState> output_bbox_mode_states_;
    std::optional<std::int64_t> last_capture_mono_ms_;
};

}  // namespace perception
