/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: bytetrack.cpp
 * Brief: ByteTrack-style multi-object tracker implementation.
 *
 * Description:
 * What problem this solves. Implements bytetrack.hpp. One update() is one inference frame:
 *   1. forget lost tracks older than track_buffer_frames;
 *   2. round one matches every live track against the confident detections, round two matches
 *      the tracks still free against the remaining detections (greedy, see greedy_match);
 *   3. matched tracks take the detection (apply_detection); unmatched ones coast in the lost
 *      buffer and stay published for at most their output max age;
 *   4. unmatched detections start new tracks;
 *   5. duplicates among the PUBLISHED copies are suppressed (suppress_duplicate_output_tracks).
 * Only confirmed tracks are published: min_hits consecutive hits for a normal track,
 * low_confidence_min_hits for a weak (generic obstacle) one.
 *
 * Why duplicates are handled on the output, not inside the tracker. A partially occluded
 * object can briefly look like two boxes; merging internal tracks would destroy the identity it
 * needs to recover. So internal tracks stay separate and only the published copies are merged.
 *
 * Pitfalls. A frame id regression (a new stream fed to an old tracker) wraps the unsigned
 * frame differences: every lost track is pruned and every unmatched track is dropped. Hence one
 * tracker per stream (bytetrack.hpp).
 */

#include "modules/bytetrack.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace perception {

namespace {

// Confidence bands of stable_delta().
constexpr float HIGH_CONFIDENCE = 0.85F;
constexpr float MID_CONFIDENCE = 0.65F;
// Published-copy duplicate rule: IoU above this, or one box at least this much inside the other.
constexpr float OUTPUT_DUPLICATE_IOU_THRESHOLD = 0.45F;
constexpr float OUTPUT_DUPLICATE_CONTAINMENT_THRESHOLD = 0.90F;
// Box-mode geometry: a "large nested switch" is a new published box much smaller or larger than
// the accepted one and mostly inside it; two modes are "similar" above the IoU bound.
constexpr float OUTPUT_BBOX_MODE_SWITCH_MAX_AREA_RATIO = 0.70F;
constexpr float OUTPUT_BBOX_MODE_SWITCH_MIN_CONTAINMENT = 0.90F;
constexpr float OUTPUT_PARTIAL_NESTED_MAX_AREA_RATIO = 0.55F;
constexpr float OUTPUT_PARTIAL_NESTED_MIN_CONTAINMENT = 0.75F;
constexpr float OUTPUT_BBOX_MODE_SIMILAR_MIN_IOU = 0.65F;
// Keep this aligned with the existing output-duplicate IoU boundary. The
// additional area-ratio and centre-displacement checks are what prevent a
// contained or spatially shifted normal object from stealing a weak track.
constexpr float WEAK_TO_NORMAL_PROMOTION_MIN_IOU = OUTPUT_DUPLICATE_IOU_THRESHOLD;
constexpr float WEAK_TO_NORMAL_PROMOTION_MIN_AREA_RATIO = 0.50F;
constexpr float WEAK_TO_NORMAL_PROMOTION_MAX_CENTER_DISTANCE_RATIO = 0.20F;

bool is_vehicle_class(const std::string& class_name) {
    return class_name == "vehicle" || class_name == "car" || class_name == "truck" || class_name == "bus" ||
           class_name == "motorcycle" || class_name == "bicycle" || class_name == "train";
}

bool contains_index(const std::vector<std::size_t>& indices, std::size_t value) {
    return std::find(indices.begin(), indices.end(), value) != indices.end();
}

// Containment: 1 when the smaller box lies entirely inside the larger one.
float intersection_over_smaller_area(const cv::Rect& a, const cv::Rect& b) {
    const int smaller_area = std::min(a.area(), b.area());
    if (smaller_area <= 0) {
        return 0.0F;
    }
    return static_cast<float>((a & b).area()) / static_cast<float>(smaller_area);
}

float smaller_to_larger_area_ratio(const cv::Rect& a, const cv::Rect& b) {
    const int smaller_area = std::min(a.area(), b.area());
    const int larger_area = std::max(a.area(), b.area());
    if (smaller_area <= 0 || larger_area <= 0) {
        return 0.0F;
    }
    return static_cast<float>(smaller_area) / static_cast<float>(larger_area);
}

float center_distance_over_smaller_diagonal(const cv::Rect& a, const cv::Rect& b) {
    const float a_diagonal =
        std::hypot(static_cast<float>(a.width), static_cast<float>(a.height));
    const float b_diagonal =
        std::hypot(static_cast<float>(b.width), static_cast<float>(b.height));
    const float smaller_diagonal = std::min(a_diagonal, b_diagonal);
    if (smaller_diagonal <= 0.0F) {
        return std::numeric_limits<float>::infinity();
    }
    const float a_center_x = static_cast<float>(a.x) + 0.5F * static_cast<float>(a.width);
    const float a_center_y = static_cast<float>(a.y) + 0.5F * static_cast<float>(a.height);
    const float b_center_x = static_cast<float>(b.x) + 0.5F * static_cast<float>(b.width);
    const float b_center_y = static_cast<float>(b.y) + 0.5F * static_cast<float>(b.height);
    return std::hypot(a_center_x - b_center_x, a_center_y - b_center_y) /
           smaller_diagonal;
}

// A weak box and a normal box describe the same extent: enough overlap, similar size and close
// centres. Overlap alone is not enough, because a small normal object inside a large weak
// obstacle overlaps it completely.
bool weak_normal_geometry_compatible(
    const cv::Rect& weak_bbox,
    const cv::Rect& normal_bbox,
    float iou_score) {
    return iou_score >= WEAK_TO_NORMAL_PROMOTION_MIN_IOU &&
           smaller_to_larger_area_ratio(weak_bbox, normal_bbox) >=
               WEAK_TO_NORMAL_PROMOTION_MIN_AREA_RATIO &&
           center_distance_over_smaller_diagonal(weak_bbox, normal_bbox) <=
               WEAK_TO_NORMAL_PROMOTION_MAX_CENTER_DISTANCE_RATIO;
}

bool is_partial_nested_output_pair(const cv::Rect& lhs, const cv::Rect& rhs) {
    return smaller_to_larger_area_ratio(lhs, rhs) <=
               OUTPUT_PARTIAL_NESTED_MAX_AREA_RATIO &&
           intersection_over_smaller_area(lhs, rhs) >=
               OUTPUT_PARTIAL_NESTED_MIN_CONTAINMENT;
}

bool is_large_nested_output_bbox_switch(const cv::Rect& accepted, const cv::Rect& proposed) {
    const float area_ratio = smaller_to_larger_area_ratio(accepted, proposed);
    const float containment = intersection_over_smaller_area(accepted, proposed);
    return (area_ratio < OUTPUT_BBOX_MODE_SWITCH_MAX_AREA_RATIO &&
            containment >= OUTPUT_BBOX_MODE_SWITCH_MIN_CONTAINMENT) ||
           (area_ratio <= OUTPUT_PARTIAL_NESTED_MAX_AREA_RATIO &&
            containment >= OUTPUT_PARTIAL_NESTED_MIN_CONTAINMENT);
}

bool output_bbox_modes_similar(const cv::Rect& lhs, const cv::Rect& rhs) {
    const int intersection_area = (lhs & rhs).area();
    const int union_area = lhs.area() + rhs.area() - intersection_area;
    if (intersection_area <= 0 || union_area <= 0) {
        return false;
    }
    return static_cast<float>(intersection_area) / static_cast<float>(union_area) >=
           OUTPUT_BBOX_MODE_SIMILAR_MIN_IOU;
}

// True when more than ttl_frames have passed since last_output_frame, and also when the frame id
// went backwards: a regression ends the state instead of wrapping into a huge gap.
bool frame_gap_exceeds_ttl(
    std::uint64_t frame_id,
    std::uint64_t last_output_frame,
    int ttl_frames) {
    if (frame_id < last_output_frame) {
        return true;
    }
    return frame_id - last_output_frame >
           static_cast<std::uint64_t>(std::max(0, ttl_frames));
}

std::vector<std::size_t> unmatched_indices(std::size_t count, const std::vector<std::size_t>& matched_indices) {
    std::vector<std::size_t> result;
    result.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (!contains_index(matched_indices, i)) {
            result.push_back(i);
        }
    }
    return result;
}

}  // namespace

// Clamp rather than reject: a negative frame count becomes 0 and an EMA weight is kept in
// [0.01, 1] (a weight of 0 would freeze the published box).
ByteTracker::ByteTracker(const Params& params) : params_(params) {
    params_.track_buffer_frames = std::max(0, params_.track_buffer_frames);
    params_.min_hits = std::max(1, params_.min_hits);
    params_.low_confidence_min_hits = std::max(1, params_.low_confidence_min_hits);
    params_.max_age = std::max(0, params_.max_age);
    params_.track_buffer_frames = std::max(params_.track_buffer_frames, params_.max_age);
    for (auto& [unused_class_name, max_age] : params_.per_class_max_age) {
        (void)unused_class_name;
        max_age = std::max(0, max_age);
        params_.track_buffer_frames = std::max(params_.track_buffer_frames, max_age);
    }
    params_.bbox_ema_alpha = std::clamp(params_.bbox_ema_alpha, 0.01F, 1.0F);
    for (auto& [unused_class_name, alpha] : params_.per_class_bbox_ema_alpha) {
        (void)unused_class_name;
        alpha = std::clamp(alpha, 0.01F, 1.0F);
    }
}

// The semantic confirmation threshold is the detector's own confidence threshold.
ByteTracker::ByteTracker(const yolo::Config& config)
    : ByteTracker(Params{config.track_threshold(),
                         config.track_buffer_frames(),
                         config.match_threshold(),
                         config.track_min_hits(),
                         config.track_max_age(),
                         config.track_bbox_ema_alpha(),
                         config.track_per_class_max_ages(),
                         config.track_per_class_bbox_ema_alphas(),
                         config.track_low_confidence_min_hits(),
                         config.conf_threshold()}) {}

// Untimed legacy entry: the tracks carry no observation times (see update_impl).
std::vector<Track> ByteTracker::update(const std::vector<yolo::Detection>& detections, std::uint64_t frame_id) {
    return update_impl(detections, frame_id, std::nullopt);
}

std::vector<Track> ByteTracker::update(const std::vector<yolo::Detection>& detections,
                                     std::uint64_t frame_id, std::int64_t capture_mono_ms) {
    if (capture_mono_ms < 0 ||
        (last_capture_mono_ms_ && capture_mono_ms < *last_capture_mono_ms_)) {
        throw std::invalid_argument("ByteTracker capture monotonic time is negative or regressed");
    }
    auto result = update_impl(detections, frame_id, capture_mono_ms);
    last_capture_mono_ms_ = capture_mono_ms;
    return result;
}

std::vector<Track> ByteTracker::update_impl(const std::vector<yolo::Detection>& detections,
        std::uint64_t frame_id, std::optional<std::int64_t> capture_mono_ms) {
    current_frame_ = frame_id;
    prune_expired_lost_tracks(frame_id);

    // Every live track, active or lost, takes part in the association.
    std::vector<Track> candidates;
    candidates.reserve(active_tracks_.size() + lost_tracks_.size());
    candidates.insert(candidates.end(), active_tracks_.begin(), active_tracks_.end());
    candidates.insert(candidates.end(), lost_tracks_.begin(), lost_tracks_.end());
    for (auto& track : candidates) {
        track.current_observation_low_confidence_candidate = false;
        // An untimed frame cannot refresh or retrospectively reconstruct an
        // observation history. Legacy callers continue to receive old fields.
        if (!capture_mono_ms) track.rns_observation_times.reset();
    }

    // ByteTrack split: confident detections first, the rest only against leftover tracks.
    std::vector<std::size_t> high_detections;
    std::vector<std::size_t> low_detections;
    high_detections.reserve(detections.size());
    low_detections.reserve(detections.size());
    for (std::size_t i = 0; i < detections.size(); ++i) {
        if (detections[i].confidence > params_.track_thresh) {
            high_detections.push_back(i);
        } else {
            low_detections.push_back(i);
        }
    }

    const std::vector<std::size_t> all_track_indices = unmatched_indices(candidates.size(), {});
    const std::vector<Match> first_matches = greedy_match(candidates, all_track_indices, detections, high_detections);

    std::vector<std::size_t> matched_tracks;
    matched_tracks.reserve(first_matches.size());
    for (const auto& match : first_matches) {
        apply_detection(candidates[match.track_index], detections[match.detection_index], frame_id, capture_mono_ms);
        matched_tracks.push_back(match.track_index);
    }

    const std::vector<std::size_t> remaining_tracks = unmatched_indices(candidates.size(), matched_tracks);
    const std::vector<Match> second_matches = greedy_match(candidates, remaining_tracks, detections, low_detections);
    for (const auto& match : second_matches) {
        apply_detection(candidates[match.track_index], detections[match.detection_index], frame_id, capture_mono_ms);
        matched_tracks.push_back(match.track_index);
    }

    std::vector<std::size_t> first_matched_high_positions =
        matched_detection_positions(first_matches, high_detections);
    std::vector<std::size_t> second_matched_low_positions =
        matched_detection_positions(second_matches, low_detections);

    active_tracks_.clear();
    lost_tracks_.clear();

    // Rebuild the two lists. A matched track is active; an unmatched one decays and coasts.
    std::vector<Track> output;
    output.reserve(candidates.size() + high_detections.size());
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        Track track = std::move(candidates[i]);
        if (contains_index(matched_tracks, i)) {
            if (track.confirmed) {
                push_output_track(track, output);
            }
            active_tracks_.push_back(std::move(track));
            continue;
        }

        // A miss takes one point off the stable_frames score and breaks the hit streak.
        track.stable_frames = std::max(0, track.stable_frames - 1);
        track.consecutive_hits = 0;
        const auto missed_frames = frame_id - static_cast<std::uint64_t>(track.last_seen_frame);
        const int max_output_age = output_max_age(track);
        const bool short_coast =
            missed_frames <= static_cast<std::uint64_t>(max_output_age);
        if (track.semantic_status == "confirmed" && short_coast &&
            track.semantic_low_quality_frames < max_output_age) {
            ++track.semantic_low_quality_frames;
        } else {
            // Before first confirmation, continuity is strict: even one coast
            // clears the streak. Confirmed semantics tolerate only the same
            // bounded low-quality budget used by the existing output age.
            track.semantic_status = "proxy";
            track.semantic_evidence_class_id = -1;
            track.semantic_evidence_class_name.clear();
            track.semantic_consecutive_hits = 0;
            track.semantic_low_quality_frames = 0;
        }
        if (missed_frames <= static_cast<std::uint64_t>(params_.track_buffer_frames)) {
            if (track.low_confidence_candidate &&
                missed_frames > static_cast<std::uint64_t>(max_output_age)) {
                // Keep the buffered identity for later reassociation, but a
                // weak obstacle that outlived its generic display age must earn
                // temporal confirmation again before it can be published.
                track.confirmed = false;
            }
            if (track.confirmed && missed_frames <= static_cast<std::uint64_t>(max_output_age)) {
                push_output_track(track, output);
            }
            lost_tracks_.push_back(std::move(track));
        }
    }

    // Unmatched confident detections start new tracks. Weak-born tracks are remembered by key:
    // that provenance decides how much evidence a duplicate pair needs (two swaps, not one).
    const std::vector<std::size_t> unmatched_high_positions =
        unmatched_indices(high_detections.size(), first_matched_high_positions);
    for (const auto position : unmatched_high_positions) {
        const yolo::Detection& detection = detections[high_detections[position]];
        Track track = make_track(next_track_id_++, detection, frame_id, capture_mono_ms);
        if (detection.low_confidence_candidate) {
            born_weak_track_keys_.emplace(track.track_id, track.first_seen_frame);
        }
        if (track.confirmed) {
            push_output_track(track, output);
        }
        active_tracks_.push_back(std::move(track));
    }

    // Unmatched lower-score detections start tracks too; in the original ByteTrack they only
    // extend existing tracks. Two different "low" notions meet here and must not be confused:
    // confidence <= track_thresh only picks the association round, while
    // low_confidence_candidate (the weak-obstacle path) picks the stricter
    // low_confidence_min_hits and never counts as semantic evidence. A lower-score NORMAL
    // detection is confirmed by min_hits like any other and may become semantically confirmed.
    // Why low scores may start tracks is not recorded; dropping this branch would change what is
    // published and needs its own evaluation.
    const std::vector<std::size_t> unmatched_low_positions =
        unmatched_indices(low_detections.size(), second_matched_low_positions);
    for (const auto position : unmatched_low_positions) {
        const yolo::Detection& detection = detections[low_detections[position]];
        Track track = make_track(next_track_id_++, detection, frame_id, capture_mono_ms);
        if (detection.low_confidence_candidate) {
            born_weak_track_keys_.emplace(track.track_id, track.first_seen_frame);
        }
        if (track.confirmed) {
            push_output_track(track, output);
        }
        active_tracks_.push_back(std::move(track));
    }

    // Provenance of tracks that left both lists is dropped, so the set stays bounded.
    const auto is_live_track_key = [this](const std::pair<int, int>& key) {
        const auto matches_key = [&key](const Track& track) {
            return track.track_id == key.first &&
                   track.first_seen_frame == key.second;
        };
        return std::any_of(active_tracks_.begin(), active_tracks_.end(), matches_key) ||
               std::any_of(lost_tracks_.begin(), lost_tracks_.end(), matches_key);
    };
    for (auto provenance_it = born_weak_track_keys_.begin();
         provenance_it != born_weak_track_keys_.end();) {
        if (!is_live_track_key(*provenance_it)) {
            provenance_it = born_weak_track_keys_.erase(provenance_it);
        } else {
            ++provenance_it;
        }
    }

    // Detector NMS cannot remove a duplicate that survives as a coasting track
    // while another track is matched in the current frame. Apply the same
    // conservative IoU/containment rule to the published tracks only. Internal
    // tracker state remains intact, so a partially occluded real object can
    // recover without losing its identity.
    suppress_duplicate_output_tracks(output, frame_id);
    return output;
}

float ByteTracker::iou(const cv::Rect& a, const cv::Rect& b) {
    const cv::Rect intersection = a & b;
    const int intersection_area = intersection.area();
    if (intersection_area <= 0) {
        return 0.0F;
    }

    const int union_area = a.area() + b.area() - intersection_area;
    if (union_area <= 0) {
        return 0.0F;
    }
    return static_cast<float>(intersection_area) / static_cast<float>(union_area);
}

// What one observation adds to stable_frames (a miss takes one away). Confident person, vehicle
// and marker observations count more, so those classes pass the velocity estimator's
// min_stable_frames gate sooner; every other class counts one per hit.
int ByteTracker::stable_delta(const std::string& class_name, float confidence) {
    if (class_name == "person") {
        return confidence > HIGH_CONFIDENCE ? 3 : (confidence >= MID_CONFIDENCE ? 2 : 1);
    }
    if (is_vehicle_class(class_name)) {
        return confidence > HIGH_CONFIDENCE ? 2 : (confidence >= MID_CONFIDENCE ? 2 : 1);
    }
    if (class_name == "marker") {
        return confidence > HIGH_CONFIDENCE ? 2 : 1;
    }
    return 1;
}

std::uint64_t ByteTracker::now_ms() {
    // LINT-ALLOW(clock): feeds ONLY the legacy first_seen_ms/last_seen_ms diagnostic fields. The r6 wire takes
    // first_seen_mono_ms/last_seen_mono_ms from rns_observation_times (monotonic) and the adapter refuses any other value.
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

// A new track takes its first detection at once; (track_id, first_seen_frame) keys it for life.
Track ByteTracker::make_track(int track_id, const yolo::Detection& detection, std::uint64_t frame_id,
                            std::optional<std::int64_t> capture_mono_ms) const {
    Track track;
    track.track_id = track_id;
    track.first_seen_frame = static_cast<int>(frame_id);
    track.first_seen_ms = now_ms();
    apply_detection(track, detection, frame_id, capture_mono_ms);
    return track;
}

// The published copy gets its own mask: a coasting track publishes its stored mask again next
// frame, so a caller writing into a shared mask would corrupt the tracker's copy.
Track ByteTracker::clone_for_output(const Track& track) {
    Track output = track;
    output.mask = track.mask.clone();
    return output;
}

// How long a coasting track stays published. Weak tracks always use the generic max_age,
// whatever proxy label they carry.
int ByteTracker::output_max_age(const Track& track) const {
    const auto class_max_age = track.low_confidence_candidate
                                   ? params_.per_class_max_age.end()
                                   : params_.per_class_max_age.find(track.class_name);
    return class_max_age == params_.per_class_max_age.end()
               ? params_.max_age
               : class_max_age->second;
}

// One association round. Each admissible (track, detection) pair is scored by max(IoU,
// containment) against the track's latest raw detector box (raw_bbox_2d is the association
// geometry; bbox_2d is smoothed and lags). Admissible: the same class; a weak track and a weak
// detection of any class; or a weak track and a normal detection that pass the weak/normal
// geometry gate (a promotion). Pairs are then taken greedily by priority (same-class normal 3,
// same-class weak 2, cross-class 1) and score, each track and detection at most once.
std::vector<ByteTracker::Match> ByteTracker::greedy_match(
    const std::vector<Track>& tracks,
    const std::vector<std::size_t>& track_indices,
    const std::vector<yolo::Detection>& detections,
    const std::vector<std::size_t>& detection_indices) const {
    struct Candidate {
        std::size_t track_index = 0;
        std::size_t detection_index = 0;
        float score = 0.0F;
        int priority = 0;
    };

    std::vector<Candidate> candidates;
    for (const auto track_index : track_indices) {
        for (const auto detection_index : detection_indices) {
            const bool same_class =
                tracks[track_index].class_id == detections[detection_index].class_id;
            const cv::Rect& association_bbox = tracks[track_index].raw_bbox_2d.empty()
                                                   ? tracks[track_index].bbox_2d
                                                   : tracks[track_index].raw_bbox_2d;
            const cv::Rect& detection_bbox = detections[detection_index].bbox;
            const float iou_score = iou(association_bbox, detection_bbox);
            const float score = std::max(
                iou_score,
                intersection_over_smaller_area(association_bbox, detection_bbox));
            const bool weak_to_normal_transition =
                tracks[track_index].low_confidence_candidate &&
                !detections[detection_index].low_confidence_candidate;
            const bool weak_to_normal_geometry_compatible =
                !weak_to_normal_transition ||
                weak_normal_geometry_compatible(association_bbox, detection_bbox, iou_score);
            if (!weak_to_normal_geometry_compatible) {
                // A proxy label can coincidentally equal a nearby normal
                // object's class. Apply the promotion geometry gate even in
                // that same-class case, otherwise a contained small normal box
                // can shrink or erase the larger physical-obstacle track.
                continue;
            }
            const bool generic_weak_match =
                tracks[track_index].low_confidence_candidate &&
                detections[detection_index].low_confidence_candidate;
            const bool weak_to_normal_promotion =
                weak_to_normal_transition && weak_to_normal_geometry_compatible;
            if (!same_class && !generic_weak_match && !weak_to_normal_promotion) {
                continue;
            }
            if (score >= params_.match_thresh) {
                const int priority = same_class
                                         ? (tracks[track_index].low_confidence_candidate ? 2 : 3)
                                         : 1;
                candidates.push_back({track_index, detection_index, score, priority});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.priority != rhs.priority) {
            return lhs.priority > rhs.priority;
        }
        return lhs.score > rhs.score;
    });

    std::vector<std::size_t> used_tracks;
    std::vector<std::size_t> used_detections;
    std::vector<Match> matches;
    for (const auto& candidate : candidates) {
        if (contains_index(used_tracks, candidate.track_index) ||
            contains_index(used_detections, candidate.detection_index)) {
            continue;
        }
        used_tracks.push_back(candidate.track_index);
        used_detections.push_back(candidate.detection_index);
        matches.push_back({candidate.track_index, candidate.detection_index, candidate.score});
    }
    return matches;
}

void ByteTracker::apply_detection(Track& track, const yolo::Detection& detection, std::uint64_t frame_id,
                                 std::optional<std::int64_t> capture_mono_ms) const {
    const bool initializing = track.class_id < 0;
    // Observation times (19 section 4.1): first and last seen on the capture-monotonic clock.
    // An untimed update clears them instead of mixing an unknown time into a timed history.
    if (!capture_mono_ms) {
        track.rns_observation_times.reset();
    } else if (initializing) {
        track.rns_observation_times = RnsObservationTimes{*capture_mono_ms, *capture_mono_ms};
    } else if (track.rns_observation_times) {
        track.rns_observation_times->last_seen_mono_ms = *capture_mono_ms;
    }
    // A weak track matched by a weak detection of another class keeps its first proxy label.
    const bool same_class = track.class_id == detection.class_id;
    const bool weak_cross_class =
        !initializing && !same_class && track.low_confidence_candidate &&
        detection.low_confidence_candidate;
    if (!weak_cross_class) {
        track.class_id = detection.class_id;
        track.class_name = detection.class_name;
    }
    if (initializing) {
        track.low_confidence_candidate = detection.low_confidence_candidate;
    } else if (track.low_confidence_candidate && !detection.low_confidence_candidate) {
        // A strong normal observation promotes a weak track. Cross-class
        // promotion is admitted only by greedy_match's stricter geometry gate.
        // Never downgrade an ordinary track after later weak observations.
        track.low_confidence_candidate = false;
    }
    track.current_observation_low_confidence_candidate =
        detection.low_confidence_candidate;
    track.confidence = detection.confidence;
    track.mask = detection.mask.clone();
    if (track.bbox_2d.empty()) {
        track.bbox_2d = detection.bbox;
    } else {
        const auto class_alpha = track.low_confidence_candidate
                                     ? params_.per_class_bbox_ema_alpha.end()
                                     : params_.per_class_bbox_ema_alpha.find(track.class_name);
        const float alpha = class_alpha == params_.per_class_bbox_ema_alpha.end()
                                ? params_.bbox_ema_alpha
                                : class_alpha->second;
        const auto smooth = [alpha](int previous, int current) {
            return cvRound((1.0F - alpha) * static_cast<float>(previous) + alpha * static_cast<float>(current));
        };
        track.bbox_2d = cv::Rect(
            smooth(track.bbox_2d.x, detection.bbox.x),
            smooth(track.bbox_2d.y, detection.bbox.y),
            std::max(1, smooth(track.bbox_2d.width, detection.bbox.width)),
            std::max(1, smooth(track.bbox_2d.height, detection.bbox.height)));
    }
    // The raw box drives the next association; bbox_2d above is the smoothed published box.
    track.raw_bbox_2d = detection.bbox;
    track.last_seen_frame = static_cast<int>(frame_id);
    track.last_seen_ms = now_ms();
    track.stable_frames += stable_delta(track.class_name, detection.confidence);
    track.consecutive_hits += 1;
    // Semantic status ("confirmed" / "proxy", 19 section 4.1). A class is confirmed after
    // min_hits consecutive confident normal observations of it. A confirmed class survives up to
    // output_max_age weaker same-class updates (matched on weak evidence, or coasting), and a
    // confident one refills that budget. A different observed class drops the status to proxy
    // at once and starts a new streak, which cannot confirm on the update of the change.
    const bool has_observed_class =
        detection.class_id >= 0 && !detection.class_name.empty();
    const bool had_semantic_class =
        track.semantic_evidence_class_id >= 0 &&
        !track.semantic_evidence_class_name.empty();
    const bool same_semantic_class =
        has_observed_class && had_semantic_class &&
        track.semantic_evidence_class_id == detection.class_id &&
        track.semantic_evidence_class_name == detection.class_name;
    const bool high_confidence_semantic_evidence =
        has_observed_class && !detection.low_confidence_candidate &&
        detection.confidence >= params_.semantic_confirmation_threshold;
    const bool semantic_was_confirmed = track.semantic_status == "confirmed";

    if (semantic_was_confirmed && same_semantic_class) {
        if (high_confidence_semantic_evidence) {
            track.semantic_consecutive_hits =
                std::max(params_.min_hits, track.semantic_consecutive_hits);
            track.semantic_low_quality_frames = 0;
        } else if (track.semantic_low_quality_frames < output_max_age(track)) {
            // Same-class weak/low evidence consumes a bounded grace frame; a
            // later qualifying observation replenishes the entire budget.
            ++track.semantic_low_quality_frames;
        } else {
            track.semantic_status = "proxy";
            track.semantic_consecutive_hits = 0;
            track.semantic_low_quality_frames = 0;
        }
    } else if (!same_semantic_class) {
        const bool changed_semantic_class =
            had_semantic_class && has_observed_class;
        if (has_observed_class) {
            track.semantic_evidence_class_id = detection.class_id;
            track.semantic_evidence_class_name = detection.class_name;
        } else {
            track.semantic_evidence_class_id = -1;
            track.semantic_evidence_class_name.clear();
        }
        track.semantic_consecutive_hits =
            high_confidence_semantic_evidence ? 1 : 0;
        track.semantic_low_quality_frames = 0;
        // A different observed class immediately invalidates the old semantic
        // confirmation. Its own high-confidence streak starts at one, but
        // cannot be confirmed on the class-change frame.
        track.semantic_status =
            !changed_semantic_class &&
                    track.semantic_consecutive_hits >= params_.min_hits
                ? "confirmed"
                : "proxy";
    } else {
        if (high_confidence_semantic_evidence) {
            track.semantic_consecutive_hits =
                std::min(params_.min_hits, track.semantic_consecutive_hits + 1);
        } else {
            // Before confirmation, low-confidence evidence breaks the required
            // consecutive positive-observation streak.
            track.semantic_consecutive_hits = 0;
        }
        track.semantic_low_quality_frames = 0;
        track.semantic_status =
            track.semantic_consecutive_hits >= params_.min_hits
                ? "confirmed"
                : "proxy";
    }
    // Publication confirmation is sticky, except that a weak track which coasted past its
    // output age must earn it again (update_impl).
    const int required_hits =
        track.low_confidence_candidate ? params_.low_confidence_min_hits : params_.min_hits;
    track.confirmed = track.confirmed || track.consecutive_hits >= required_hits;
}

void ByteTracker::push_output_track(const Track& track, std::vector<Track>& output) const {
    output.push_back(clone_for_output(track));
}

// Runs on the published copies only (see the file header), in four steps:
//   1. expire box-hold states and pair states (a member died, or no evidence within the TTL);
//   2. find same-class pairs seen this frame in which a small weak-tier box sits inside a large
//      normal-tier box, and count reciprocal role swaps; enough swaps make the pair equivalent;
//   3. walk the copies in preference order (fresh, normal-tier, non-weak, confident, stable,
//      large, low id) and fold each duplicate into the first kept copy it duplicates, handing
//      the oldest compatible identity to the kept copy;
//   4. for duplicate members, hold a large nested box-mode switch for one opportunity.
void ByteTracker::suppress_duplicate_output_tracks(std::vector<Track>& output, std::uint64_t frame_id) {
    for (auto state_it = output_bbox_mode_states_.begin();
         state_it != output_bbox_mode_states_.end();) {
        if (frame_gap_exceeds_ttl(
                frame_id,
                state_it->second.last_output_frame,
                state_it->second.ttl_frames)) {
            state_it = output_bbox_mode_states_.erase(state_it);
        } else {
            ++state_it;
        }
    }

    const auto track_key = [](const Track& track) {
        return TrackKey{track.track_id, track.first_seen_frame};
    };
    const auto pair_key = [](TrackKey lhs, TrackKey rhs) {
        if (rhs < lhs) {
            std::swap(lhs, rhs);
        }
        return PartialNestedPairKey{lhs, rhs};
    };
    const auto live_track_key = [this, &track_key](const TrackKey& key) {
        const auto matches_key = [&track_key, &key](const Track& track) {
            return track_key(track) == key;
        };
        return std::any_of(active_tracks_.begin(), active_tracks_.end(), matches_key) ||
               std::any_of(lost_tracks_.begin(), lost_tracks_.end(), matches_key);
    };

    for (auto state_it = partial_nested_pair_states_.begin();
         state_it != partial_nested_pair_states_.end();) {
        const bool member_died =
            !live_track_key(state_it->first.first) ||
            !live_track_key(state_it->first.second);
        const bool evidence_expired = frame_gap_exceeds_ttl(
            frame_id,
            state_it->second.last_qualifying_frame,
            state_it->second.ttl_frames);
        if (member_died || evidence_expired) {
            state_it = partial_nested_pair_states_.erase(state_it);
        } else {
            ++state_it;
        }
    }

    // Freeze immutable internal keys before any publication-copy identity
    // handoff. Pair evidence must never depend on a canonical public ID.
    std::vector<TrackKey> internal_track_keys;
    internal_track_keys.reserve(output.size());
    for (const Track& track : output) {
        internal_track_keys.push_back(track_key(track));
    }

    const auto classify_qualifying_partial_pair =
        [this, frame_id, &internal_track_keys](
            const std::vector<Track>& tracks,
            std::size_t lhs_index,
            std::size_t rhs_index,
            TrackKey& small_weak_key,
            TrackKey& large_normal_key,
            int& ttl_frames,
            int& required_swaps) {
            // Qualifying pair: same class, both observed this frame, both normal tracks, raw
            // boxes nested, and exactly one current observation weak-tier, namely the small one.
            // A weak-born member raises the evidence to two swaps within up to twice the TTL.
            const Track& lhs = tracks[lhs_index];
            const Track& rhs = tracks[rhs_index];
            const bool same_class =
                lhs.class_id == rhs.class_id &&
                lhs.class_name == rhs.class_name;
            if (!same_class ||
                lhs.last_seen_frame != static_cast<int>(frame_id) ||
                rhs.last_seen_frame != static_cast<int>(frame_id) ||
                lhs.low_confidence_candidate ||
                rhs.low_confidence_candidate ||
                lhs.raw_bbox_2d.empty() ||
                rhs.raw_bbox_2d.empty() ||
                lhs.current_observation_low_confidence_candidate ==
                    rhs.current_observation_low_confidence_candidate ||
                !is_partial_nested_output_pair(lhs.raw_bbox_2d, rhs.raw_bbox_2d)) {
                return false;
            }

            const bool lhs_is_small =
                lhs.raw_bbox_2d.area() < rhs.raw_bbox_2d.area();
            const Track& small = lhs_is_small ? lhs : rhs;
            const Track& large = lhs_is_small ? rhs : lhs;
            if (!small.current_observation_low_confidence_candidate ||
                large.current_observation_low_confidence_candidate) {
                return false;
            }

            small_weak_key =
                internal_track_keys[lhs_is_small ? lhs_index : rhs_index];
            large_normal_key =
                internal_track_keys[lhs_is_small ? rhs_index : lhs_index];
            const int base_ttl_frames =
                std::max(0, std::min(output_max_age(lhs), output_max_age(rhs)));
            const bool has_born_weak_provenance =
                born_weak_track_keys_.count(internal_track_keys[lhs_index]) != 0U ||
                born_weak_track_keys_.count(internal_track_keys[rhs_index]) != 0U;
            if (has_born_weak_provenance) {
                const auto doubled_base_ttl =
                    2LL * static_cast<long long>(base_ttl_frames);
                ttl_frames = static_cast<int>(std::min(
                    static_cast<long long>(params_.track_buffer_frames),
                    doubled_base_ttl));
                required_swaps = 2;
            } else {
                ttl_frames = base_ttl_frames;
                required_swaps = 1;
            }
            return true;
        };

    std::set<PartialNestedPairKey> qualifying_partial_pairs;
    std::set<PartialNestedPairKey> equivalent_partial_pairs;
    for (std::size_t lhs_index = 0; lhs_index < output.size(); ++lhs_index) {
        for (std::size_t rhs_index = lhs_index + 1U;
             rhs_index < output.size();
             ++rhs_index) {
            TrackKey small_weak_key;
            TrackKey large_normal_key;
            int ttl_frames = 0;
            int required_swaps = 1;
            if (!classify_qualifying_partial_pair(
                    output,
                    lhs_index,
                    rhs_index,
                    small_weak_key,
                    large_normal_key,
                    ttl_frames,
                    required_swaps)) {
                continue;
            }

            const PartialNestedPairKey key =
                pair_key(internal_track_keys[lhs_index], internal_track_keys[rhs_index]);
            qualifying_partial_pairs.insert(key);
            auto [state_it, inserted] = partial_nested_pair_states_.emplace(
                key,
                PartialNestedPairState{
                    small_weak_key,
                    large_normal_key,
                    frame_id,
                    0,
                    std::max(0, ttl_frames),
                    0,
                    std::max(1, required_swaps),
                    false});
            PartialNestedPairState& state = state_it->second;
            if (!inserted) {
                const bool reciprocal_role_swap =
                    state.last_small_weak_key == large_normal_key &&
                    state.last_large_normal_key == small_weak_key;
                state.required_swaps =
                    std::max(state.required_swaps, std::max(1, required_swaps));
                state.ttl_frames = std::max(0, ttl_frames);
                if (!state.equivalent && reciprocal_role_swap) {
                    if (state.reciprocal_swaps == 0) {
                        state.reciprocal_swaps = 1;
                        state.first_reciprocal_swap_frame = frame_id;
                    } else if (
                        frame_gap_exceeds_ttl(
                            frame_id,
                            state.first_reciprocal_swap_frame,
                            state.ttl_frames)) {
                        // A same-role run must not extend the two-swap window.
                        // This transition becomes swap one of a fresh cycle.
                        state.reciprocal_swaps = 1;
                        state.first_reciprocal_swap_frame = frame_id;
                    } else {
                        ++state.reciprocal_swaps;
                    }
                }
                state.last_small_weak_key = small_weak_key;
                state.last_large_normal_key = large_normal_key;
                state.last_qualifying_frame = frame_id;
                state.equivalent =
                    state.reciprocal_swaps >= state.required_swaps;
            }
            if (state.equivalent) {
                equivalent_partial_pairs.insert(key);
            }
        }
    }

    const auto is_preferred = [frame_id](const Track& lhs, const Track& rhs) {
        const bool lhs_current = lhs.last_seen_frame == static_cast<int>(frame_id);
        const bool rhs_current = rhs.last_seen_frame == static_cast<int>(frame_id);
        if (lhs_current != rhs_current) {
            return lhs_current;
        }
        if (lhs_current &&
            lhs.current_observation_low_confidence_candidate !=
                rhs.current_observation_low_confidence_candidate) {
            return !lhs.current_observation_low_confidence_candidate;
        }
        if (lhs.low_confidence_candidate != rhs.low_confidence_candidate) {
            return !lhs.low_confidence_candidate;
        }
        if (lhs.confidence != rhs.confidence) {
            return lhs.confidence > rhs.confidence;
        }
        if (lhs.stable_frames != rhs.stable_frames) {
            return lhs.stable_frames > rhs.stable_frames;
        }
        if (lhs.bbox_2d.area() != rhs.bbox_2d.area()) {
            return lhs.bbox_2d.area() > rhs.bbox_2d.area();
        }
        return lhs.track_id < rhs.track_id;
    };

    std::vector<std::size_t> priority_order;
    priority_order.reserve(output.size());
    for (std::size_t i = 0; i < output.size(); ++i) {
        priority_order.push_back(i);
    }
    std::stable_sort(
        priority_order.begin(),
        priority_order.end(),
        [&output, &is_preferred](std::size_t lhs, std::size_t rhs) {
            return is_preferred(output[lhs], output[rhs]);
        });

    // Duplicate admission must use immutable publication representatives.
    // Temporal merging may later replace a kept public bbox with another
    // member's conservative bbox, and identity handoff may replace its public
    // ID. Neither mutation is allowed to manufacture a new legacy edge.
    const std::vector<Track> frozen_publication_tracks = output;
    std::vector<bool> suppressed(output.size(), false);
    std::vector<bool> duplicate_cluster_member(output.size(), false);
    std::vector<bool> reset_bbox_mode_state(output.size(), false);
    std::vector<std::set<TrackKey>> duplicate_cluster_track_keys(output.size());
    std::vector<cv::Rect> duplicate_cluster_conservative_bboxes(output.size());
    for (std::size_t i = 0; i < output.size(); ++i) {
        duplicate_cluster_track_keys[i].insert(internal_track_keys[i]);
        duplicate_cluster_conservative_bboxes[i] = output[i].bbox_2d;
    }
    const auto has_direct_current_temporal_edge =
        [&pair_key, &qualifying_partial_pairs, &equivalent_partial_pairs](
            const std::set<TrackKey>& lhs_cluster,
            const std::set<TrackKey>& rhs_cluster) {
            for (const TrackKey& lhs_key : lhs_cluster) {
                for (const TrackKey& rhs_key : rhs_cluster) {
                    const PartialNestedPairKey key =
                        pair_key(lhs_key, rhs_key);
                    if (qualifying_partial_pairs.count(key) != 0U &&
                        equivalent_partial_pairs.count(key) != 0U) {
                        return true;
                    }
                }
            }
            return false;
        };
    std::vector<std::size_t> kept_indices;
    kept_indices.reserve(output.size());
    for (const std::size_t candidate_index : priority_order) {
        std::size_t duplicate_kept_index = output.size();
        bool duplicate_same_class = false;
        bool duplicate_partial_equivalent = false;
        cv::Rect duplicate_conservative_bbox;
        for (const std::size_t kept_index : kept_indices) {
            const bool temporal_equivalent_partial_pair =
                has_direct_current_temporal_edge(
                    duplicate_cluster_track_keys[candidate_index],
                    duplicate_cluster_track_keys[kept_index]);

            const Track& frozen_candidate =
                frozen_publication_tracks[candidate_index];
            const Track& frozen_kept =
                frozen_publication_tracks[kept_index];
            const bool same_class =
                frozen_candidate.class_id == frozen_kept.class_id &&
                frozen_candidate.class_name == frozen_kept.class_name;
            // Only a track that still belongs to the generic weak tier may
            // make a cross-class pair eligible for legacy duplicate
            // suppression.  An established normal track is not generic just
            // because its current same-class observation fell below the
            // normal admission threshold for one frame.  Treating that
            // transient observation flag as identity provenance can erase a
            // distinct foreground/background object whenever their 2-D
            // projections overlap.
            const bool generic_weak_overlap =
                frozen_candidate.low_confidence_candidate ||
                frozen_kept.low_confidence_candidate;
            bool legacy_duplicate_eligible =
                same_class || generic_weak_overlap;
            const bool candidate_weak_tier =
                frozen_candidate.low_confidence_candidate ||
                frozen_candidate.current_observation_low_confidence_candidate;
            const bool kept_weak_tier =
                frozen_kept.low_confidence_candidate ||
                frozen_kept.current_observation_low_confidence_candidate;
            const float output_iou =
                iou(frozen_candidate.bbox_2d, frozen_kept.bbox_2d);
            if (legacy_duplicate_eligible &&
                candidate_weak_tier != kept_weak_tier &&
                !weak_normal_geometry_compatible(
                    frozen_candidate.bbox_2d,
                    frozen_kept.bbox_2d,
                    output_iou)) {
                // An independent small normal object may legitimately sit
                // inside a larger generic weak obstacle. Do not let weak-aware
                // containment suppression erase either physical obstacle.
                legacy_duplicate_eligible = false;
            }
            const bool legacy_duplicate =
                legacy_duplicate_eligible &&
                (
                output_iou > OUTPUT_DUPLICATE_IOU_THRESHOLD ||
                intersection_over_smaller_area(
                    frozen_candidate.bbox_2d, frozen_kept.bbox_2d) >=
                    OUTPUT_DUPLICATE_CONTAINMENT_THRESHOLD);
            const bool duplicate =
                legacy_duplicate || temporal_equivalent_partial_pair;
            if (duplicate) {
                duplicate_kept_index = kept_index;
                duplicate_same_class = same_class;
                duplicate_partial_equivalent =
                    temporal_equivalent_partial_pair;
                if (duplicate_partial_equivalent) {
                    duplicate_conservative_bbox =
                        duplicate_cluster_conservative_bboxes[candidate_index].area() >
                                duplicate_cluster_conservative_bboxes[kept_index].area()
                            ? duplicate_cluster_conservative_bboxes[candidate_index]
                            : duplicate_cluster_conservative_bboxes[kept_index];
                }
                break;
            }
        }
        if (duplicate_kept_index != output.size()) {
            // Keep geometry/mask/confidence from the freshest preferred track,
            // but retain the oldest identity in the duplicate cluster. Without
            // this handoff, two alternating detector box modes publish a
            // different track_id every few frames even though only one box is
            // visible, which resets downstream velocity state and creates false
            // motion for a static object.
            // This is deliberately a publication-copy handoff, not an internal
            // track alias. Continuity is therefore bounded by the lifetime of
            // the older internal track in the existing identity buffer.
            Track& kept = output[duplicate_kept_index];
            const Track& duplicate = output[candidate_index];
            const bool duplicate_has_older_identity =
                duplicate.first_seen_frame < kept.first_seen_frame ||
                (duplicate.first_seen_frame == kept.first_seen_frame &&
                 duplicate.track_id < kept.track_id);
            const bool compatible_identity_handoff =
                kept.low_confidence_candidate == duplicate.low_confidence_candidate &&
                (duplicate_same_class || kept.low_confidence_candidate);
            if (duplicate_has_older_identity && compatible_identity_handoff) {
                kept.track_id = duplicate.track_id;
                kept.first_seen_frame = duplicate.first_seen_frame;
                kept.first_seen_ms = duplicate.first_seen_ms;
                if (kept.rns_observation_times && duplicate.rns_observation_times) {
                    // First observation follows the canonical old identity;
                    // last observation stays with the retained fresh geometry.
                    kept.rns_observation_times->first_seen_mono_ms =
                        duplicate.rns_observation_times->first_seen_mono_ms;
                } else {
                    kept.rns_observation_times.reset();
                }
                // The public ID now denotes the older identity, so its semantic
                // evidence state must move with that ID as one atomic handoff.
                kept.semantic_status = duplicate.semantic_status;
                kept.semantic_evidence_class_id =
                    duplicate.semantic_evidence_class_id;
                kept.semantic_evidence_class_name =
                    duplicate.semantic_evidence_class_name;
                kept.semantic_consecutive_hits =
                    duplicate.semantic_consecutive_hits;
                kept.semantic_low_quality_frames =
                    duplicate.semantic_low_quality_frames;
            }
            if (duplicate_partial_equivalent) {
                // Priority selection retains raw geometry, mask, confidence and
                // observation time from the current-normal (large raw) member.
                // Only the public smoothed box is conservative across the pair.
                kept.bbox_2d = duplicate_conservative_bbox;
                reset_bbox_mode_state[duplicate_kept_index] = true;
            }
            duplicate_cluster_track_keys[duplicate_kept_index].insert(
                duplicate_cluster_track_keys[candidate_index].begin(),
                duplicate_cluster_track_keys[candidate_index].end());
            if (duplicate_cluster_conservative_bboxes[candidate_index].area() >
                duplicate_cluster_conservative_bboxes[duplicate_kept_index].area()) {
                duplicate_cluster_conservative_bboxes[duplicate_kept_index] =
                    duplicate_cluster_conservative_bboxes[candidate_index];
            }
            kept.stable_frames = std::max(kept.stable_frames, duplicate.stable_frames);
            kept.consecutive_hits = std::max(kept.consecutive_hits, duplicate.consecutive_hits);
            duplicate_cluster_member[duplicate_kept_index] = true;
            duplicate_cluster_member[candidate_index] = true;
            suppressed[candidate_index] = true;
        } else {
            kept_indices.push_back(candidate_index);
        }
    }

    std::size_t next = 0U;
    for (std::size_t i = 0; i < output.size(); ++i) {
        if (suppressed[i]) {
            continue;
        }
        if (next != i) {
            output[next] = std::move(output[i]);
            duplicate_cluster_member[next] = duplicate_cluster_member[i];
            reset_bbox_mode_state[next] = reset_bbox_mode_state[i];
        }
        ++next;
    }
    output.resize(next);
    duplicate_cluster_member.resize(next);
    reset_bbox_mode_state.resize(next);

    for (std::size_t i = 0; i < output.size(); ++i) {
        Track& track = output[i];
        const auto canonical_key =
            std::make_pair(track.track_id, track.first_seen_frame);
        if (reset_bbox_mode_state[i]) {
            // Every fully qualifying proven pair starts this output frame from
            // its conservative larger smoothed box. A non-qualifying frame may
            // have published both member modes, so never revive either smaller
            // canonical history when suppression resumes.
            output_bbox_mode_states_.erase(canonical_key);
        }
        const cv::Rect proposed_bbox = track.bbox_2d;
        const bool low_confidence_tier =
            track.low_confidence_candidate ||
            track.current_observation_low_confidence_candidate;
        const int ttl_frames = output_max_age(track);
        auto state_it = output_bbox_mode_states_.find(canonical_key);
        if (state_it == output_bbox_mode_states_.end()) {
            output_bbox_mode_states_.emplace(
                canonical_key,
                OutputBboxModeState{
                    proposed_bbox,
                    cv::Rect(),
                    frame_id,
                    low_confidence_tier,
                    ttl_frames});
            continue;
        }

        OutputBboxModeState& state = state_it->second;
        const bool tier_changed =
            state.last_low_confidence_tier != low_confidence_tier;
        const bool consecutive_output =
            frame_id > state.last_output_frame &&
            frame_id - state.last_output_frame == 1U;
        const bool large_nested_switch =
            is_large_nested_output_bbox_switch(state.accepted_bbox, proposed_bbox);
        const auto accept_proposed_bbox = [&state, &proposed_bbox]() {
            state.accepted_bbox = proposed_bbox;
            state.pending_bbox = cv::Rect();
        };

        if (!duplicate_cluster_member[i] || tier_changed ||
            state.accepted_bbox.empty() || !large_nested_switch) {
            // Non-duplicate output, confidence-tier transitions, ordinary
            // jitter/translation and clearly disjoint geometry are direct
            // observations. They must never inherit duplicate-mode latency.
            accept_proposed_bbox();
        } else if (
            !state.pending_bbox.empty() &&
            output_bbox_modes_similar(proposed_bbox, state.accepted_bbox)) {
            // The detector returned to the accepted mode before confirming the
            // alternate mode. Follow the current accepted-mode jitter and
            // cancel the pending switch.
            accept_proposed_bbox();
        } else if (state.pending_bbox.empty() || !consecutive_output) {
            // First observation of a substantially different nested mode: hold
            // the previously accepted public bbox for exactly one opportunity
            // to confirm it. Raw geometry, mask and all identity/semantic
            // fields remain the current output evidence.
            state.pending_bbox = proposed_bbox;
            track.bbox_2d = state.accepted_bbox;
        } else if (output_bbox_modes_similar(proposed_bbox, state.pending_bbox)) {
            // The alternate mode persisted with only ordinary detector jitter.
            // The second hit takes over without another held frame.
            accept_proposed_bbox();
        } else {
            // Even if a second consecutive jump falls outside the similarity
            // tolerance, rebase now so a changing detector mode can never
            // freeze output for more than one additional frame.
            accept_proposed_bbox();
        }

        state.last_output_frame = frame_id;
        state.last_low_confidence_tier = low_confidence_tier;
        state.ttl_frames = ttl_frames;
    }
}

// Frame ids only increase within one tracker; the file header says what a regression does.
void ByteTracker::prune_expired_lost_tracks(std::uint64_t frame_id) {
    lost_tracks_.erase(
        std::remove_if(
            lost_tracks_.begin(),
            lost_tracks_.end(),
            [this, frame_id](const Track& track) {
                return frame_id - static_cast<std::uint64_t>(track.last_seen_frame) >
                       static_cast<std::uint64_t>(params_.track_buffer_frames);
            }),
        lost_tracks_.end());
}

// Positions, in the round's own index list (high or low), of the detections that round matched.
std::vector<std::size_t> ByteTracker::matched_detection_positions(
    const std::vector<Match>& matches,
    const std::vector<std::size_t>& detection_indices) {
    std::vector<std::size_t> positions;
    positions.reserve(matches.size());
    for (const auto& match : matches) {
        const auto iter = std::find(detection_indices.begin(), detection_indices.end(), match.detection_index);
        if (iter != detection_indices.end()) {
            positions.push_back(static_cast<std::size_t>(std::distance(detection_indices.begin(), iter)));
        }
    }
    return positions;
}

}  // namespace perception
