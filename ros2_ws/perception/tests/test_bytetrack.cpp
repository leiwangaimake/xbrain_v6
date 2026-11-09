/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_bytetrack.cpp
 * Brief: Unit tests for ByteTrack association metadata.
 *
 * Description:
 *   1. Verifies first_seen_ms and last_seen_ms are populated by the tracker.
 *   2. Verifies last_seen_ms is refreshed on matched detections.
 */

#include "modules/bytetrack.hpp"
#include "../src/supervisor/weak_obstacle_policy.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace {

yolo::Detection make_detection() {
    yolo::Detection det;
    det.bbox = cv::Rect(10, 10, 20, 20);
    det.confidence = 0.9F;
    det.class_id = 0;
    det.class_name = "person";
    det.mask = cv::Mat::ones(20, 20, CV_8UC1);
    return det;
}

yolo::Detection make_detection(
    const cv::Rect& bbox,
    int class_id,
    const char* class_name,
    float confidence,
    bool low_confidence_candidate = false) {
    yolo::Detection det = make_detection();
    det.bbox = bbox;
    det.confidence = confidence;
    det.class_id = class_id;
    det.class_name = class_name;
    det.mask = cv::Mat::ones(bbox.size(), CV_8UC1);
    det.low_confidence_candidate = low_confidence_candidate;
    return det;
}

cv::Rect ema_rect(
    const cv::Rect& previous,
    const cv::Rect& current,
    float alpha) {
    const auto blend = [alpha](int previous_value, int current_value) {
        return cvRound(
            (1.0F - alpha) * static_cast<float>(previous_value) +
            alpha * static_cast<float>(current_value));
    };
    return cv::Rect(
        blend(previous.x, current.x),
        blend(previous.y, current.y),
        blend(previous.width, current.width),
        blend(previous.height, current.height));
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// A new track gets first_seen_ms and last_seen_ms; a matched update keeps the id and
// first_seen_ms.
void test_seen_times_populated_and_updated() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.5F, 1, 5, 0.4F, {}, {}});

    const auto first = tracker.update({make_detection()}, 100);
    require(first.size() == 1U, "first update did not create a track");
    require(first.front().first_seen_ms > 0U, "first_seen_ms was not populated");
    require(first.front().last_seen_ms >= first.front().first_seen_ms, "last_seen_ms precedes first_seen_ms");

    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const auto second = tracker.update({make_detection()}, 101);
    require(second.size() == 1U, "second update did not keep the track");
    require(second.front().track_id == first.front().track_id, "track id changed across matched update");
    require(second.front().first_seen_ms == first.front().first_seen_ms, "first_seen_ms changed after match");
    require(second.front().last_seen_ms >= first.front().last_seen_ms, "last_seen_ms was not refreshed");
}

// A track is output only after min_hits consecutive hits (a miss resets the streak); unconfirmed
// tracks never coast, confirmed ones coast up to max_age.
void test_confirmation_and_coast_hysteresis() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.5F, 3, 5, 0.4F, {}, {}});

    require(tracker.update({make_detection()}, 1).empty(), "track appeared before min_hits");
    require(tracker.update({}, 2).empty(), "unconfirmed track coasted");
    require(tracker.update({make_detection()}, 3).empty(), "hit streak was not reset by a miss");
    require(tracker.update({make_detection()}, 4).empty(), "track appeared on the second consecutive hit");
    const auto confirmed = tracker.update({make_detection()}, 5);
    require(confirmed.size() == 1U, "track did not appear at min_hits");
    const int track_id = confirmed.front().track_id;

    for (std::uint64_t frame_id = 6; frame_id <= 10; ++frame_id) {
        const auto coasted = tracker.update({}, frame_id);
        require(coasted.size() == 1U, "confirmed track vanished inside max_age");
        require(coasted.front().track_id == track_id, "coast changed the track id");
    }
    require(tracker.update({}, 11).empty(), "track survived beyond max_age");
}

// The published bbox is EMA-smoothed while raw_bbox and the mask keep the current detection
// geometry.
void test_output_bbox_ema_keeps_raw_detection_geometry() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.5F, 1, 5, 0.4F, {}, {}});
    const auto first = tracker.update({make_detection()}, 1);
    require(first.size() == 1U, "EMA test did not create track");

    yolo::Detection shifted = make_detection();
    shifted.bbox.x = 14;
    shifted.mask = cv::Mat::ones(shifted.bbox.size(), CV_8UC1);
    const auto second = tracker.update({shifted}, 2);
    require(second.size() == 1U, "EMA test lost track");
    require(second.front().bbox_2d.x == 12, "published bbox did not use configured EMA");
    require(second.front().raw_bbox_2d.x == 14, "raw bbox was not preserved for mask/depth");
    require(second.front().mask.size() == shifted.bbox.size(), "current detection mask geometry changed");
}

// A nested same-class detection updates the existing track; no duplicate track and no new id.
void test_nested_same_class_detection_keeps_track_id() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.5F, 1, 5, 0.4F, {}, {}});
    const auto outer = tracker.update({make_detection(cv::Rect(10, 10, 100, 100), 62, "tv", 0.9F)}, 1);
    require(outer.size() == 1U, "nested test did not create the outer track");

    const auto inner = tracker.update({make_detection(cv::Rect(35, 35, 50, 50), 62, "tv", 0.4F)}, 2);
    require(inner.size() == 1U, "nested same-class detection created a duplicate track");
    require(inner.front().track_id == outer.front().track_id, "nested same-class detection changed track id");
}

// A detection of a different class never reuses an existing track.
void test_different_class_cannot_steal_track() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.5F, 1, 5, 0.4F, {}, {}});
    const auto person = tracker.update({make_detection(cv::Rect(10, 10, 80, 80), 0, "person", 0.9F)}, 1);
    require(person.size() == 1U, "class guard test did not create the first track");

    const auto next = tracker.update({make_detection(cv::Rect(10, 10, 80, 80), 2, "car", 0.9F)}, 2);
    require(next.size() == 2U, "different-class detection incorrectly reused an existing track");
    require(next[0].track_id != next[1].track_id, "different classes shared a track id");
}

// Past max_age the stale box is no longer published, but track_buffer_frames lets the same id
// resume.
void test_track_buffer_preserves_identity_without_displaying_stale_box() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 5, 0.5F, 1, 2, 0.4F, {}, {}});
    const auto first = tracker.update({make_detection()}, 1);
    require(first.size() == 1U, "buffer test did not create a track");
    const int first_id = first.front().track_id;

    require(tracker.update({}, 2).size() == 1U, "track vanished before max_age");
    require(tracker.update({}, 3).size() == 1U, "track vanished at max_age");
    require(tracker.update({}, 4).empty(), "stale box remained visible beyond max_age");

    const auto resumed = tracker.update({make_detection()}, 5);
    require(resumed.size() == 1U, "buffered track did not resume");
    require(resumed.front().track_id == first_id, "track_buffer_frames did not preserve identity");

    for (std::uint64_t frame_id = 6; frame_id <= 11; ++frame_id) {
        (void)tracker.update({}, frame_id);
    }
    const auto expired = tracker.update({make_detection()}, 12);
    require(expired.size() == 1U, "expired detection did not create a new track");
    require(expired.front().track_id != first_id, "track survived beyond track_buffer_frames");
}

// B8-4: the latest-wins result handoff can skip results. A skipped result is
// not a miss: hits accumulate over the results the tracker does see, while
// ageing follows the source frame_id gap (capture time), not the number of
// results processed.
void test_skipped_results_age_by_source_frame_gap() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.5F, 3, 5, 0.4F, {}, {}});
    require(tracker.update({make_detection()}, 1).empty(), "track appeared before min_hits");
    require(tracker.update({make_detection()}, 3).empty(), "track appeared after two processed results");
    const auto confirmed = tracker.update({make_detection()}, 4);
    require(confirmed.size() == 1U, "a skipped result reset the hit streak");
    const int track_id = confirmed.front().track_id;
    for (const std::uint64_t frame_id : {6U, 7U, 9U, 10U}) {
        const auto seen = tracker.update({make_detection()}, frame_id);
        require(seen.size() == 1U && seen.front().track_id == track_id, "skipped results changed the track");
    }

    // Gone after frame 10: only two empty results arrive inside max_age = 5 source frames.
    auto coast = tracker.update({}, 12);
    require(coast.size() == 1U && coast.front().track_id == track_id, "coast vanished inside max_age");
    coast = tracker.update({}, 15);
    require(coast.size() == 1U && coast.front().track_id == track_id, "coast vanished at max_age");
    require(tracker.update({}, 16).empty(), "skipped results let a stale box outlive max_age");

    // Identity is kept for track_buffer_frames source frames, not processed results.
    const auto resumed = tracker.update({make_detection()}, 40);
    require(resumed.size() == 1U && resumed.front().track_id == track_id,
            "identity lost inside track_buffer_frames");
    require(tracker.update({}, 50).empty(), "resumed track was not ageing again");
    require(tracker.update({make_detection()}, 71).empty(), "expired identity was revived as confirmed");
    require(tracker.update({make_detection()}, 72).empty(), "new track appeared before min_hits");
    const auto fresh = tracker.update({make_detection()}, 73);
    require(fresh.size() == 1U && fresh.front().track_id != track_id,
            "identity survived beyond track_buffer_frames source frames");
}

// per_class_max_age lengthens coasting only for its class (suitcase), never for person.
void test_per_class_max_age_only_extends_configured_class() {
    perception::ByteTracker::Params params{0.5F, 30, 0.5F, 1, 5, 0.4F, {}, {}};
    params.per_class_max_age = {{"suitcase", 9}};
    perception::ByteTracker suitcase_tracker(params);
    const auto suitcase = suitcase_tracker.update(
        {make_detection(cv::Rect(10, 10, 40, 60), 28, "suitcase", 0.8F)}, 1);
    require(suitcase.size() == 1U, "per-class age test did not create suitcase");
    for (std::uint64_t frame_id = 2; frame_id <= 10; ++frame_id) {
        require(
            suitcase_tracker.update({}, frame_id).size() == 1U,
            "suitcase vanished inside per-class max age");
    }
    require(
        suitcase_tracker.update({}, 11).empty(),
        "suitcase survived beyond per-class max age");

    perception::ByteTracker person_tracker(params);
    require(person_tracker.update({make_detection()}, 1).size() == 1U, "person track missing");
    for (std::uint64_t frame_id = 2; frame_id <= 6; ++frame_id) {
        require(
            person_tracker.update({}, frame_id).size() == 1U,
            "person vanished inside global max age");
    }
    require(
        person_tracker.update({}, 7).empty(),
        "per-class suitcase age leaked into person tracking");
}

// per_class_bbox_ema_alpha smooths only its class; raw bboxes stay unsmoothed.
void test_per_class_bbox_ema_only_smooths_configured_class() {
    perception::ByteTracker::Params params{0.5F, 30, 0.25F, 1, 5, 0.4F, {}, {{"suitcase", 0.05F}}};

    perception::ByteTracker suitcase_tracker(params);
    require(
        suitcase_tracker
                .update({make_detection(cv::Rect(10, 10, 60, 60), 28, "suitcase", 0.9F)}, 1)
                .size() == 1U,
        "per-class bbox EMA test did not create suitcase");
    const auto suitcase = suitcase_tracker.update(
        {make_detection(cv::Rect(30, 10, 60, 60), 28, "suitcase", 0.9F)}, 2);
    require(suitcase.size() == 1U, "per-class bbox EMA test lost suitcase");
    require(suitcase.front().bbox_2d.x == 11, "suitcase did not use per-class bbox EMA");
    require(suitcase.front().raw_bbox_2d.x == 30, "suitcase raw bbox was not preserved");

    perception::ByteTracker person_tracker(params);
    require(
        person_tracker
                .update({make_detection(cv::Rect(10, 10, 60, 60), 0, "person", 0.9F)}, 1)
                .size() == 1U,
        "per-class bbox EMA test did not create person");
    const auto person = person_tracker.update(
        {make_detection(cv::Rect(30, 10, 60, 60), 0, "person", 0.9F)}, 2);
    require(person.size() == 1U, "per-class bbox EMA test lost person");
    require(person.front().bbox_2d.x == 18, "per-class suitcase EMA leaked into person");
}

// Output suppression removes only nested same-class duplicates, keeping the stronger one; distinct
// objects and other classes stay.
void test_published_tracks_suppress_only_nested_same_class_duplicates() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});

    const auto nested = tracker.update(
        {
            make_detection(cv::Rect(10, 10, 200, 300), 0, "person", 0.9F),
            make_detection(cv::Rect(30, 30, 160, 260), 0, "person", 0.7F),
        },
        1);
    require(nested.size() == 1U, "nested same-class output tracks were not suppressed");
    require(nested.front().confidence == 0.9F, "duplicate suppression kept the weaker track");

    perception::ByteTracker distinct_tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});
    const auto distinct = distinct_tracker.update(
        {
            make_detection(cv::Rect(10, 10, 120, 280), 0, "person", 0.9F),
            make_detection(cv::Rect(150, 10, 120, 280), 0, "person", 0.8F),
        },
        1);
    require(distinct.size() == 2U, "distinct same-class people were incorrectly suppressed");

    perception::ByteTracker different_class_tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});
    const auto different_classes = different_class_tracker.update(
        {
            make_detection(cv::Rect(10, 10, 200, 300), 0, "person", 0.9F),
            make_detection(cv::Rect(30, 30, 160, 260), 56, "chair", 0.7F),
        },
        1);
    require(
        different_classes.size() == 2U,
        "same-class duplicate suppression leaked across semantic classes");
}

// A suppressed nested duplicate hands its current geometry to the oldest id; the first nested mode
// switch is held for confirmation.
void test_duplicate_output_hands_current_geometry_to_oldest_identity() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});

    const cv::Rect older_geometry(10, 10, 200, 200);
    const cv::Rect newer_geometry(40, 40, 120, 120);
    const auto first = tracker.update(
        {
            make_detection(older_geometry, 28, "suitcase", 0.9F),
            make_detection(newer_geometry, 28, "suitcase", 0.8F),
        },
        1);
    require(first.size() == 1U, "identity handoff setup did not suppress nested output");
    const int oldest_track_id = first.front().track_id;

    const auto second = tracker.update(
        {make_detection(newer_geometry, 28, "suitcase", 0.9F)},
        2);
    require(second.size() == 1U, "identity handoff emitted duplicate tracks");
    require(
        second.front().track_id == oldest_track_id,
        "current duplicate geometry changed the published canonical track id");
    require(
        second.front().bbox_2d == older_geometry,
        "first nested geometry mode switch was not held for confirmation");
    require(
        second.front().raw_bbox_2d == newer_geometry &&
            second.front().mask.size() == newer_geometry.size() &&
            second.front().last_seen_frame == 2,
        "bbox hold replaced current raw geometry, mask, or observation frame");

    const auto confirmed_newer = tracker.update(
        {make_detection(newer_geometry, 28, "suitcase", 0.9F)},
        3);
    require(confirmed_newer.size() == 1U, "confirmed geometry handoff emitted duplicates");
    require(
        confirmed_newer.front().track_id == oldest_track_id,
        "confirmed geometry handoff changed canonical track id");
    require(
        confirmed_newer.front().bbox_2d == newer_geometry,
        "persistent nested geometry did not take over on its second frame");
}

// Partially nested same-class tracks are suppressed only after a reciprocal role swap inside the
// TTL, keeping the oldest id and the current raw geometry.
void test_ordinary_partial_nested_modes_keep_oldest_identity_and_current_raw_geometry() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});

    const cv::Rect initial_large(100, 100, 400, 400);
    const cv::Rect initial_small(100, 80, 100, 100);
    const cv::Rect reciprocal_large(50, 50, 300, 300);
    const cv::Rect reciprocal_small(250, 270, 100, 100);

    const auto primed = tracker.update(
        {
            make_detection(initial_large, 56, "chair", 0.95F, false),
            make_detection(initial_small, 56, "chair", 0.85F, false),
        },
        1);
    require(
        primed.size() == 2U,
        "temporal partial-nested setup erased ordinary tracks");
    const int oldest_track_id =
        std::min(primed[0].track_id, primed[1].track_id);

    const auto first_role = tracker.update(
        {
            make_detection(initial_large, 56, "chair", 0.95F, false),
            make_detection(initial_small, 56, "chair", 0.20F, true),
        },
        2);
    require(
        first_role.size() == 2U,
        "first qualifying partial pair was suppressed without temporal evidence");

    const auto repeated_role = tracker.update(
        {
            make_detection(initial_large, 56, "chair", 0.95F, false),
            make_detection(initial_small, 56, "chair", 0.20F, true),
        },
        3);
    require(
        repeated_role.size() == 2U,
        "repeated same-role partial evidence activated suppression");

    // The new large raw box overlaps the previous small track more strongly,
    // while the new small raw box overlaps the previous large track. This
    // creates the reciprocal internal TrackKey role swap seen in formal_003.
    const auto reciprocal_role = tracker.update(
        {
            make_detection(reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(reciprocal_small, 56, "chair", 0.20F, true),
        },
        4);
    require(
        reciprocal_role.size() == 1U,
        "reciprocal role swap inside TTL did not activate suppression");
    require(
        reciprocal_role.front().track_id == oldest_track_id,
        "activated publication handoff did not retain the oldest buffered id");
    require(
        reciprocal_role.front().bbox_2d == reciprocal_large &&
            reciprocal_role.front().raw_bbox_2d == reciprocal_large &&
            reciprocal_role.front().mask.size() == reciprocal_large.size() &&
            reciprocal_role.front().confidence == 0.95F &&
            reciprocal_role.front().last_seen_frame == 4 &&
            !reciprocal_role.front().current_observation_low_confidence_candidate,
        "activated pair did not preserve current-normal geometry and evidence");
}

// Recorded partial-nesting evidence does not disable the legacy 90% containment suppression.
void test_partial_evidence_does_not_disable_legacy_containment() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});

    const cv::Rect large_geometry(100, 100, 400, 400);
    const cv::Rect partial_small_geometry(100, 80, 100, 100);
    const cv::Rect contained_small_geometry(100, 100, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(
                            large_geometry, 56, "chair", 0.95F, false),
                        make_detection(
                            partial_small_geometry, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 2U,
        "legacy-after-evidence setup lost ordinary partial objects");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            large_geometry, 56, "chair", 0.95F, false),
                        make_detection(
                            partial_small_geometry, 56, "chair", 0.20F, true),
                    },
                    2)
                .size() == 2U,
        "first partial observation activated temporal suppression");

    const auto contained_same_tier = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(
                contained_small_geometry, 56, "chair", 0.85F, false),
        },
        3);
    require(
        contained_same_tier.size() == 1U,
        "recorded partial evidence disabled legacy 90% containment suppression");
}

// The first non-equivalent partial evidence still goes through the legacy duplicate rule.
void test_first_partial_evidence_still_uses_legacy_duplicate_rule() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});

    const cv::Rect large_geometry(100, 100, 200, 200);
    // This starts at about 79% containment, below both legacy duplicate
    // thresholds, while keeping enough IoU to retain the same internal track.
    const cv::Rect partial_small_geometry(129, 70, 142, 142);
    const cv::Rect contained_small_geometry(129, 129, 142, 142);

    require(
        tracker
                .update(
                    {
                        make_detection(
                            large_geometry, 56, "chair", 0.95F, false),
                        make_detection(
                            partial_small_geometry, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 2U,
        "first-evidence legacy setup lost ordinary partial objects");

    const auto first_qualifying = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(
                contained_small_geometry, 56, "chair", 0.20F, true),
        },
        2);
    require(
        first_qualifying.size() == 1U,
        "first non-equivalent partial evidence disabled legacy duplicate suppression");
}

// Two independent normal-born objects at the formal overlap ratio stay two outputs.
void test_born_normal_independent_formal_ratio_remains_two_outputs() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});

    // The exact same 2D ratio can represent a real small chair projected over
    // a larger chair. With both current observations in the normal tier, 2D
    // geometry alone is insufficient evidence to merge them.
    const auto output = tracker.update(
        {
            make_detection(cv::Rect(954, 205, 524, 490), 56, "chair", 0.95F),
            make_detection(cv::Rect(1015, 172, 282, 182), 56, "chair", 0.85F),
        },
        1);
    require(
        output.size() == 2U,
        "born-normal independent formal-ratio objects were suppressed");
}

// A current weak observation never pairs with, and so never suppresses, a coasting ordinary track.
void test_partial_nested_does_not_pair_current_weak_with_coasting_ordinary() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect large_geometry(954, 205, 524, 490);
    const cv::Rect small_geometry(1015, 172, 282, 182);

    const auto first = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(small_geometry, 56, "chair", 0.85F, false),
        },
        1);
    require(first.size() == 2U, "coasting partial-nested setup lost an object");

    const auto output = tracker.update(
        {make_detection(small_geometry, 56, "chair", 0.20F, true)},
        2);
    require(
        output.size() == 2U,
        "current weak observation suppressed a coasting ordinary track");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [](const perception::Track& track) {
                return track.last_seen_frame == 2 &&
                       track.current_observation_low_confidence_candidate;
            }),
        "coasting regression lost the current weak observation");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [](const perception::Track& track) {
                return track.last_seen_frame == 1 &&
                       !track.current_observation_low_confidence_candidate;
            }),
        "coasting regression lost the independent stale track");
}

// A born-weak track promoted to the ordinary tier stays independent of a partially nested
// obstacle.
void test_promoted_born_weak_partial_nested_remains_independent() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect large_geometry(954, 205, 524, 490);
    const cv::Rect small_geometry(1015, 172, 282, 182);

    const auto born = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(small_geometry, 56, "chair", 0.20F, true),
        },
        1);
    require(
        born.size() == 2U,
        "born-weak provenance setup suppressed an independent obstacle");
    const auto born_weak = std::find_if(
        born.begin(),
        born.end(),
        [](const perception::Track& track) {
            return track.low_confidence_candidate;
        });
    require(
        born_weak != born.end(),
        "born-weak provenance setup did not create a weak track");
    const int born_weak_track_id = born_weak->track_id;

    const auto promoted = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(small_geometry, 56, "chair", 0.95F, false),
        },
        2);
    require(
        promoted.size() == 2U,
        "born-weak promotion suppressed an independent obstacle");
    const auto promoted_track = std::find_if(
        promoted.begin(),
        promoted.end(),
        [born_weak_track_id](const perception::Track& track) {
            return track.track_id == born_weak_track_id;
        });
    require(
        promoted_track != promoted.end() &&
            !promoted_track->low_confidence_candidate &&
            !promoted_track->current_observation_low_confidence_candidate,
        "born-weak track did not promote to the ordinary current tier");

    const auto current_weak = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(small_geometry, 56, "chair", 0.20F, true),
        },
        3);
    require(
        current_weak.size() == 2U,
        "promoted born-weak provenance was forgotten by partial suppression");
    require(
        std::any_of(
            current_weak.begin(),
            current_weak.end(),
            [born_weak_track_id](const perception::Track& track) {
                return track.track_id == born_weak_track_id &&
                       !track.low_confidence_candidate &&
                       track.current_observation_low_confidence_candidate;
            }),
        "promoted born-weak obstacle did not retain its independent current observation");
}

// For a promoted born-weak pair one reciprocal swap does not activate suppression; two inside the
// TTL do.
void test_promoted_born_weak_requires_two_reciprocal_swaps() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect first_large(100, 100, 200, 200);
    const cv::Rect first_small(70, 129, 142, 142);
    const cv::Rect reciprocal_large(70, 100, 200, 200);
    const cv::Rect reciprocal_small(158, 129, 142, 142);

    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_small, 56, "chair", 0.20F, true),
                    },
                    1)
                .size() == 2U,
        "promoted two-swap setup lost a born-weak obstacle");
    const auto promoted = tracker.update(
        {
            make_detection(first_large, 56, "chair", 0.95F, false),
            make_detection(first_small, 56, "chair", 0.95F, false),
        },
        2);
    require(
        promoted.size() == 2U &&
            std::all_of(
                promoted.begin(),
                promoted.end(),
                [](const perception::Track& track) {
                    return !track.low_confidence_candidate;
                }),
        "born-weak member did not promote before temporal evidence");

    for (std::uint64_t frame_id = 3; frame_id <= 5; ++frame_id) {
        require(
            tracker
                    .update(
                        {
                            make_detection(
                                first_large,
                                56,
                                "chair",
                                0.95F,
                                false),
                            make_detection(
                                first_small,
                                56,
                                "chair",
                                0.20F,
                                true),
                        },
                        frame_id)
                    .size() == 2U,
            "fixed-role promoted evidence activated suppression");
    }

    const auto first_swap = tracker.update(
        {
            make_detection(
                reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(
                reciprocal_small, 56, "chair", 0.20F, true),
        },
        6);
    require(
        first_swap.size() == 2U,
        "one promoted reciprocal swap activated suppression");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            reciprocal_large,
                            56,
                            "chair",
                            0.95F,
                            false),
                        make_detection(
                            reciprocal_small,
                            56,
                            "chair",
                            0.20F,
                            true),
                    },
                    7)
                .size() == 2U,
        "same-role refresh completed a promoted two-swap cycle");

    const auto second_swap = tracker.update(
        {
            make_detection(first_large, 56, "chair", 0.95F, false),
            make_detection(first_small, 56, "chair", 0.20F, true),
        },
        8);
    require(
        second_swap.size() == 1U,
        "two promoted reciprocal swaps inside TTL did not activate suppression");
}

// A second promoted swap outside the first swap's TTL must not reuse that stale evidence.
void test_promoted_born_weak_two_swap_window_cannot_be_refreshed() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 2, 1.0F, {}, {}, 1});
    const cv::Rect first_large(100, 100, 200, 200);
    const cv::Rect first_small(70, 129, 142, 142);
    const cv::Rect reciprocal_large(70, 100, 200, 200);
    const cv::Rect reciprocal_small(158, 129, 142, 142);

    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_small, 56, "chair", 0.20F, true),
                    },
                    1)
                .size() == 2U,
        "promoted TTL setup lost a born-weak obstacle");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_small, 56, "chair", 0.95F, false),
                    },
                    2)
                .size() == 2U,
        "promoted TTL setup failed promotion");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_small, 56, "chair", 0.20F, true),
                    },
                    3)
                .size() == 2U,
        "promoted TTL baseline role activated suppression");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            reciprocal_large,
                            56,
                            "chair",
                            0.95F,
                            false),
                        make_detection(
                            reciprocal_small,
                            56,
                            "chair",
                            0.20F,
                            true),
                    },
                    4)
                .size() == 2U,
        "promoted TTL first swap activated suppression");

    // The promoted evidence TTL is min(30, 2 * 2) = 4 frames. Repeating the
    // post-swap role refreshes track liveness, but must not move frame 4's
    // first-swap deadline.
    for (std::uint64_t frame_id = 5; frame_id <= 9; ++frame_id) {
        require(
            tracker
                    .update(
                        {
                            make_detection(
                                reciprocal_large,
                                56,
                                "chair",
                                0.95F,
                                false),
                            make_detection(
                                reciprocal_small,
                                56,
                                "chair",
                                0.20F,
                                true),
                        },
                        frame_id)
                    .size() == 2U,
            "same-role frames extended or completed the promoted swap window");
    }

    const auto expired_second_swap = tracker.update(
        {
            make_detection(first_large, 56, "chair", 0.95F, false),
            make_detection(first_small, 56, "chair", 0.20F, true),
        },
        10);
    require(
        expired_second_swap.size() == 2U,
        "second promoted swap outside the first-swap TTL reused stale evidence");

    const auto fresh_second_swap = tracker.update(
        {
            make_detection(
                reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(
                reciprocal_small, 56, "chair", 0.20F, true),
        },
        11);
    require(
        fresh_second_swap.size() == 1U,
        "expired transition did not restart as swap one of a fresh cycle");
}

// With a low EMA alpha, partial nesting is judged on current raw geometry, never on stale smoothed
// boxes.
void test_partial_nested_requires_current_raw_geometry_with_low_alpha() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 0.05F, {}, {}, 1});
    const cv::Rect large_geometry(954, 205, 524, 490);
    const cv::Rect initial_small_geometry(1015, 172, 282, 182);
    const cv::Rect shifted_small_raw_geometry(1015, 72, 282, 182);

    const auto first = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(initial_small_geometry, 56, "chair", 0.85F, false),
        },
        1);
    require(
        first.size() == 2U,
        "low-alpha raw-geometry setup lost an independent object");

    // The 0.05 EMA leaves the published small bbox near y=167, where IOSA with
    // the large bbox remains above 0.75. The current raw box is at y=72 and has
    // IOSA only about 0.27, so historical smoothing must not trigger a merge.
    const auto output = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(shifted_small_raw_geometry, 56, "chair", 0.20F, true),
        },
        2);
    require(
        output.size() == 2U,
        "stale low-alpha EMA geometry suppressed raw-independent objects");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [shifted_small_raw_geometry](const perception::Track& track) {
                return track.raw_bbox_2d == shifted_small_raw_geometry &&
                       track.bbox_2d.y > shifted_small_raw_geometry.y;
            }),
        "low-alpha regression did not preserve distinct raw and EMA geometry");
}

// A reciprocal swap after the pair TTL does not activate suppression.
void test_partial_nested_reciprocal_swap_after_ttl_does_not_activate() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 2, 1.0F, {}, {}, 1});
    const cv::Rect initial_large(100, 100, 400, 400);
    const cv::Rect initial_small(100, 80, 100, 100);
    const cv::Rect reciprocal_large(50, 50, 300, 300);
    const cv::Rect reciprocal_small(250, 270, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 2U,
        "TTL setup lost ordinary tracks");
    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.20F, true),
                    },
                    2)
                .size() == 2U,
        "TTL setup suppressed first role evidence");

    for (std::uint64_t frame_id = 3; frame_id <= 5; ++frame_id) {
        require(
            tracker
                    .update(
                        {
                            make_detection(initial_large, 56, "chair", 0.95F, false),
                            make_detection(initial_small, 56, "chair", 0.85F, false),
                        },
                        frame_id)
                    .size() == 2U,
            "non-qualifying TTL gap suppressed ordinary tracks");
    }

    const auto expired_reciprocal = tracker.update(
        {
            make_detection(reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(reciprocal_small, 56, "chair", 0.20F, true),
        },
        6);
    require(
        expired_reciprocal.size() == 2U,
        "reciprocal swap beyond pair TTL reused expired evidence");
}

// A tier-only swap without a small/large reversal does not activate suppression nor taint later
// same-role evidence.
void test_partial_nested_tier_only_swap_does_not_activate() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect large_geometry(100, 100, 400, 400);
    const cv::Rect small_geometry(100, 80, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(large_geometry, 56, "chair", 0.95F, false),
                        make_detection(small_geometry, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 2U,
        "tier-only setup lost ordinary tracks");
    require(
        tracker
                .update(
                    {
                        make_detection(large_geometry, 56, "chair", 0.95F, false),
                        make_detection(small_geometry, 56, "chair", 0.20F, true),
                    },
                    2)
                .size() == 2U,
        "tier-only setup suppressed first role evidence");

    const auto tier_only_swap = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.20F, true),
            make_detection(small_geometry, 56, "chair", 0.95F, false),
        },
        3);
    require(
        tier_only_swap.size() == 2U,
        "tier-only swap without small/large TrackKey reversal activated suppression");

    const auto original_role_again = tracker.update(
        {
            make_detection(large_geometry, 56, "chair", 0.95F, false),
            make_detection(small_geometry, 56, "chair", 0.20F, true),
        },
        4);
    require(
        original_role_again.size() == 2U,
        "tier-only frame contaminated later same-role evidence");
}

// On activation the larger smoothed pair bbox is published while the current-normal raw evidence
// is kept.
void test_partial_nested_low_alpha_activation_uses_larger_public_bbox() {
    constexpr float alpha = 0.05F;
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, alpha, {}, {}, 1});
    const cv::Rect initial_large(100, 100, 400, 400);
    const cv::Rect initial_small(100, 80, 100, 100);
    const cv::Rect reciprocal_large(50, 50, 300, 300);
    const cv::Rect reciprocal_small(250, 270, 100, 100);
    const auto smooth = [](
                            const cv::Rect& previous,
                            const cv::Rect& current) {
        const auto one = [](int previous_value, int current_value) {
            return cvRound(
                (1.0F - alpha) * static_cast<float>(previous_value) +
                alpha * static_cast<float>(current_value));
        };
        return cv::Rect(
            one(previous.x, current.x),
            one(previous.y, current.y),
            one(previous.width, current.width),
            one(previous.height, current.height));
    };

    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 2U,
        "low-alpha activation setup lost ordinary tracks");
    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.20F, true),
                    },
                    2)
                .size() == 2U,
        "low-alpha activation setup suppressed first evidence");

    const auto activated = tracker.update(
        {
            make_detection(reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(reciprocal_small, 56, "chair", 0.20F, true),
        },
        3);
    require(
        activated.size() == 1U,
        "low-alpha reciprocal swap did not activate");
    const cv::Rect weak_member_smoothed =
        smooth(initial_large, reciprocal_small);
    const cv::Rect normal_member_smoothed =
        smooth(initial_small, reciprocal_large);
    const cv::Rect expected_public_bbox =
        weak_member_smoothed.area() > normal_member_smoothed.area()
            ? weak_member_smoothed
            : normal_member_smoothed;
    require(
        activated.front().bbox_2d == expected_public_bbox,
        "activation did not choose the larger pair-member smoothed bbox");
    require(
        activated.front().raw_bbox_2d == reciprocal_large &&
            activated.front().mask.size() == reciprocal_large.size() &&
            activated.front().confidence == 0.95F &&
            activated.front().last_seen_frame == 3 &&
            !activated.front().current_observation_low_confidence_candidate,
        "larger public bbox replaced current-normal raw evidence");
}

// A non-qualifying frame (coasting or same tier) publishes both tracks again; complete evidence
// resumes suppression.
void test_partial_nested_invalid_frame_restores_both_outputs() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect initial_large(100, 100, 400, 400);
    const cv::Rect initial_small(100, 80, 100, 100);
    const cv::Rect reciprocal_large(50, 50, 300, 300);
    const cv::Rect reciprocal_small(250, 270, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 2U,
        "invalid-frame setup lost ordinary tracks");
    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.20F, true),
                    },
                    2)
                .size() == 2U,
        "invalid-frame setup suppressed first evidence");
    require(
        tracker
                .update(
                    {
                        make_detection(reciprocal_large, 56, "chair", 0.95F, false),
                        make_detection(reciprocal_small, 56, "chair", 0.20F, true),
                    },
                    3)
                .size() == 1U,
        "invalid-frame setup did not activate equivalence");

    const auto coasting = tracker.update(
        {
            make_detection(reciprocal_small, 56, "chair", 0.20F, true),
        },
        4);
    require(
        coasting.size() == 2U,
        "equivalent pair suppressed a non-qualifying coasting frame");
    require(
        std::any_of(
            coasting.begin(),
            coasting.end(),
            [](const perception::Track& track) {
                return track.last_seen_frame == 3;
            }),
        "coasting recovery lost the stale current-normal member");

    const auto same_tier = tracker.update(
        {
            make_detection(reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(reciprocal_small, 56, "chair", 0.85F, false),
        },
        5);
    require(
        same_tier.size() == 2U,
        "equivalent pair suppressed a non-qualifying same-tier frame");

    const auto qualifying_again = tracker.update(
        {
            make_detection(reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(reciprocal_small, 56, "chair", 0.20F, true),
        },
        6);
    require(
        qualifying_again.size() == 1U,
        "equivalent pair did not resume suppression on complete evidence");
    require(
        qualifying_again.front().bbox_2d == reciprocal_large,
        "resumed equivalent suppression revived a smaller canonical bbox");
    require(
        qualifying_again.front().raw_bbox_2d == reciprocal_large &&
            qualifying_again.front().mask.size() == reciprocal_large.size() &&
            qualifying_again.front().confidence == 0.95F &&
            qualifying_again.front().last_seen_frame == 6 &&
            !qualifying_again.front().current_observation_low_confidence_candidate,
        "resumed suppression replaced current-normal raw evidence");
}

// An equivalent pair neither suppresses nor merges a third track.
void test_partial_nested_three_track_keys_do_not_pollute_pairs() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect initial_large(100, 100, 400, 400);
    const cv::Rect initial_small(100, 80, 100, 100);
    const cv::Rect third_initial(300, 50, 100, 100);
    const cv::Rect reciprocal_large(50, 50, 300, 300);
    const cv::Rect reciprocal_small(250, 270, 100, 100);
    const cv::Rect third_small(260, 40, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.85F, false),
                        make_detection(third_initial, 56, "chair", 0.80F, false),
                    },
                    1)
                .size() == 3U,
        "three-track setup suppressed an independent track");
    require(
        tracker
                .update(
                    {
                        make_detection(initial_large, 56, "chair", 0.95F, false),
                        make_detection(initial_small, 56, "chair", 0.20F, true),
                        make_detection(third_initial, 56, "chair", 0.80F, false),
                    },
                    2)
                .size() == 3U,
        "three-track setup suppressed first pair evidence");

    const auto activated_one_pair = tracker.update(
        {
            make_detection(reciprocal_large, 56, "chair", 0.95F, false),
            make_detection(reciprocal_small, 56, "chair", 0.20F, true),
            make_detection(third_small, 56, "chair", 0.20F, true),
        },
        3);
    require(
        activated_one_pair.size() == 2U,
        "one equivalent PairKey suppressed or merged the third track");
    require(
        std::any_of(
            activated_one_pair.begin(),
            activated_one_pair.end(),
            [reciprocal_large](const perception::Track& track) {
                return track.raw_bbox_2d == reciprocal_large &&
                       !track.current_observation_low_confidence_candidate;
            }),
        "three-track activation lost the current-normal pair output");
    require(
        std::any_of(
            activated_one_pair.begin(),
            activated_one_pair.end(),
            [third_small](const perception::Track& track) {
                return track.raw_bbox_2d == third_small &&
                       track.current_observation_low_confidence_candidate;
            }),
        "equivalent PairKey contaminated the third track's first evidence");
}

// Current direct AB and BC edges converge into one cluster with the preferred current-normal
// evidence and the largest bbox.
void test_partial_nested_current_direct_three_track_cluster_converges() {
    constexpr float alpha = 0.05F;
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, alpha, {}, {}, 1});
    const cv::Rect first_a_large(0, 100, 200, 200);
    const cv::Rect first_b_small(120, 150, 100, 100);
    const cv::Rect first_c_large(140, 100, 200, 200);
    const cv::Rect reciprocal_b_large(80, 100, 200, 200);
    const cv::Rect reciprocal_a_small(60, 150, 100, 100);
    const cv::Rect reciprocal_c_small(200, 150, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_a_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_b_small, 56, "chair", 0.90F, false),
                        make_detection(
                            first_c_large, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 3U,
        "direct-cluster setup lost an ordinary track");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_a_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_b_small, 56, "chair", 0.20F, true),
                        make_detection(
                            first_c_large, 56, "chair", 0.90F, false),
                    },
                    2)
                .size() == 3U,
        "direct-cluster first AB/BC evidence suppressed a track");

    const auto converged = tracker.update(
        {
            make_detection(
                reciprocal_b_large, 56, "chair", 0.99F, false),
            make_detection(
                reciprocal_a_small, 56, "chair", 0.20F, true),
            make_detection(
                reciprocal_c_small, 56, "chair", 0.15F, true),
        },
        3);
    require(
        converged.size() == 1U,
        "current direct AB and BC temporal edges did not converge one cluster");
    require(
        converged.front().raw_bbox_2d == reciprocal_b_large &&
            converged.front().confidence == 0.99F &&
            !converged.front().current_observation_low_confidence_candidate,
        "three-track convergence replaced the preferred current-normal evidence");
    require(
        converged.front().bbox_2d ==
            ema_rect(first_a_large, reciprocal_a_small, alpha),
        "three-track convergence did not retain the largest conservative cluster bbox");
}

// A historical cluster edge alone cannot suppress a third track without a current qualifying edge.
void test_partial_nested_historical_cluster_edge_cannot_suppress_third_track() {
    constexpr float alpha = 0.05F;
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, alpha, {}, {}, 1});
    const cv::Rect first_a_large(0, 100, 200, 200);
    const cv::Rect first_b_small(120, 150, 100, 100);
    const cv::Rect first_c_large(140, 100, 200, 200);
    const cv::Rect reciprocal_b_large(80, 100, 200, 200);
    const cv::Rect reciprocal_a_small(60, 150, 100, 100);
    const cv::Rect reciprocal_c_small(200, 150, 100, 100);
    const cv::Rect shifted_c_raw(240, 150, 100, 100);

    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_a_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_b_small, 56, "chair", 0.90F, false),
                        make_detection(
                            first_c_large, 56, "chair", 0.85F, false),
                    },
                    1)
                .size() == 3U,
        "historical-edge setup lost an ordinary track");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            first_a_large, 56, "chair", 0.95F, false),
                        make_detection(
                            first_b_small, 56, "chair", 0.20F, true),
                        make_detection(
                            first_c_large, 56, "chair", 0.90F, false),
                    },
                    2)
                .size() == 3U,
        "historical-edge setup suppressed first evidence");
    require(
        tracker
                .update(
                    {
                        make_detection(
                            reciprocal_b_large,
                            56,
                            "chair",
                            0.99F,
                            false),
                        make_detection(
                            reciprocal_a_small,
                            56,
                            "chair",
                            0.20F,
                            true),
                        make_detection(
                            reciprocal_c_small,
                            56,
                            "chair",
                            0.15F,
                            true),
                    },
                    3)
                .size() == 1U,
        "historical-edge setup did not prove both direct pairs");

    const auto retained_third = tracker.update(
        {
            make_detection(
                reciprocal_b_large, 56, "chair", 0.99F, false),
            make_detection(
                reciprocal_a_small, 56, "chair", 0.20F, true),
            make_detection(shifted_c_raw, 56, "chair", 0.15F, true),
        },
        4);
    require(
        retained_third.size() == 2U,
        "historical BC equivalence suppressed C without a current qualifying edge");
    const auto merged_ab = std::find_if(
        retained_third.begin(),
        retained_third.end(),
        [reciprocal_b_large](const perception::Track& track) {
            return track.raw_bbox_2d == reciprocal_b_large;
        });
    const auto current_c = std::find_if(
        retained_third.begin(),
        retained_third.end(),
        [shifted_c_raw](const perception::Track& track) {
            return track.raw_bbox_2d == shifted_c_raw;
        });
    require(
        merged_ab != retained_third.end() &&
            merged_ab->bbox_2d ==
                ema_rect(
                    ema_rect(first_a_large, reciprocal_a_small, alpha),
                    reciprocal_a_small,
                    alpha),
        "AB temporal merge did not exercise conservative bbox enlargement");
    require(
        current_c != retained_third.end() &&
            (merged_ab->bbox_2d & current_c->bbox_2d).area() > 0,
        "retained C did not overlap the enlarged AB bbox in the regression");
}

// The legacy containment check uses frozen cluster-member metadata, so mixed enlarged boxes create
// no false edge.
void test_partial_nested_legacy_uses_frozen_cluster_member_metadata() {
    constexpr float alpha = 0.05F;
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, alpha, {}, {}, 1});
    const cv::Rect first_a_small(120, 150, 100, 100);
    const cv::Rect first_b_large(0, 100, 200, 200);
    const cv::Rect c_geometry(10, 140, 80, 100);
    const cv::Rect reciprocal_a_large(80, 100, 200, 200);
    const cv::Rect reciprocal_b_small(60, 150, 100, 100);

    (void)tracker.update(
        {
            make_detection(
                first_a_small, 56, "chair", 0.95F, false),
            make_detection(
                first_b_large, 56, "chair", 0.90F, false),
            make_detection(c_geometry, 56, "chair", 0.85F, false),
        },
        1);
    (void)tracker.update(
        {
            make_detection(
                first_a_small, 56, "chair", 0.20F, true),
            make_detection(
                first_b_large, 56, "chair", 0.95F, false),
            make_detection(c_geometry, 56, "chair", 0.20F, true),
        },
        2);

    // A becomes the current-normal root and B the current-weak member.
    // Low-alpha history makes B's publication bbox the conservative maximum.
    // Coasting normal-born C overlaps only B's frozen bbox. Direct C-B legacy
    // admission must reject that weak/normal geometry, while C-A is disjoint.
    const auto output = tracker.update(
        {
            make_detection(
                reciprocal_a_large, 56, "chair", 0.99F, false),
            make_detection(
                reciprocal_b_small, 56, "chair", 0.20F, true),
        },
        3);
    require(
        output.size() == 2U,
        "mixed conservative bbox and root metadata created a false legacy edge");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [
                first_b_large,
                reciprocal_a_large,
                reciprocal_b_small](
                const perception::Track& track) {
                return track.raw_bbox_2d == reciprocal_a_large &&
                       track.bbox_2d ==
                           ema_rect(
                               first_b_large,
                               reciprocal_b_small,
                               alpha);
            }),
        "frozen-metadata regression did not exercise conservative enlargement");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [c_geometry](const perception::Track& track) {
                return track.raw_bbox_2d == c_geometry &&
                       track.last_seen_frame == 2;
            }),
        "frozen-metadata regression lost the coasting normal-born C track");
}

// Similar-sized, partly overlapping same-class objects stay two outputs.
void test_similar_area_partial_overlap_remains_two_outputs() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});

    // Area ratio is 0.60, IOSA is 0.80 and IoU is about 0.43. These may be two
    // nearby same-class objects, not a small/large detector mode pair.
    const auto output = tracker.update(
        {
            make_detection(cv::Rect(10, 32, 200, 200), 56, "chair", 0.95F),
            make_detection(cv::Rect(30, 0, 150, 160), 56, "chair", 0.85F),
        },
        1);
    require(
        output.size() == 2U,
        "similar-area partially overlapping same-class objects were suppressed");
}

// A nested duplicate's public bbox mode changes only after two consecutive hits; identity, raw
// geometry, mask and last_seen are unchanged.
void test_duplicate_bbox_mode_alternation_needs_two_consecutive_hits() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});

    const cv::Rect outer_geometry(10, 10, 200, 200);
    const cv::Rect inner_geometry(55, 55, 100, 100);
    const cv::Rect inner_jitter(57, 56, 102, 98);
    const auto first = tracker.update(
        {
            make_detection(outer_geometry, 28, "suitcase", 0.95F),
            make_detection(inner_geometry, 28, "suitcase", 0.85F),
        },
        1);
    require(first.size() == 1U, "bbox mode setup did not suppress nested duplicates");
    require(first.front().bbox_2d == outer_geometry, "bbox mode setup chose the wrong mode");
    const int canonical_track_id = first.front().track_id;
    const int canonical_first_seen_frame = first.front().first_seen_frame;

    const auto first_inner = tracker.update(
        {make_detection(inner_geometry, 28, "suitcase", 0.95F)},
        2);
    require(first_inner.size() == 1U, "first inner mode emitted duplicates");
    require(
        first_inner.front().bbox_2d == outer_geometry,
        "one-frame nested mode switch changed the public bbox");
    require(
        first_inner.front().track_id == canonical_track_id &&
            first_inner.front().first_seen_frame == canonical_first_seen_frame,
        "bbox mode hold changed canonical identity");
    require(
        first_inner.front().raw_bbox_2d == inner_geometry &&
            first_inner.front().mask.size() == inner_geometry.size() &&
            first_inner.front().last_seen_frame == 2,
        "bbox mode hold replaced current raw geometry, mask, or last_seen");
    require(
        first_inner.front().semantic_status == "confirmed" &&
            first_inner.front().semantic_evidence_class_id == 28 &&
            first_inner.front().semantic_evidence_class_name == "suitcase",
        "bbox mode hold changed semantic rev3 evidence");

    const auto returned_outer = tracker.update(
        {make_detection(outer_geometry, 28, "suitcase", 0.95F)},
        3);
    require(returned_outer.size() == 1U, "return to outer mode emitted duplicates");
    require(
        returned_outer.front().bbox_2d == outer_geometry,
        "A/B alternation published the unconfirmed inner mode");

    const auto second_inner = tracker.update(
        {make_detection(inner_geometry, 28, "suitcase", 0.95F)},
        4);
    require(second_inner.size() == 1U, "second inner proposal emitted duplicates");
    require(
        second_inner.front().bbox_2d == outer_geometry,
        "alternating inner proposal bypassed two-hit confirmation");

    const auto persistent_inner = tracker.update(
        {make_detection(inner_jitter, 28, "suitcase", 0.95F)},
        5);
    require(persistent_inner.size() == 1U, "persistent inner mode emitted duplicates");
    require(
        persistent_inner.front().bbox_2d == inner_jitter,
        "slightly jittered pending mode did not take over on its second hit");
    require(
        persistent_inner.front().raw_bbox_2d == inner_jitter &&
            persistent_inner.front().mask.size() == inner_jitter.size(),
        "pending mode takeover changed current raw geometry or mask");
}

// A changing nested mode holds the public bbox for at most one frame.
void test_changing_nested_mode_cannot_freeze_beyond_one_frame() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});
    const cv::Rect outer_geometry(10, 10, 200, 200);
    const cv::Rect first_inner_mode(55, 55, 100, 100);
    const cv::Rect second_inner_mode(75, 75, 60, 60);

    require(
        tracker
                .update(
                    {
                        make_detection(outer_geometry, 28, "suitcase", 0.95F),
                        make_detection(first_inner_mode, 28, "suitcase", 0.85F),
                    },
                    1)
                .size() == 1U,
        "changing-mode setup emitted duplicates");
    const auto first_jump = tracker.update(
        {make_detection(first_inner_mode, 28, "suitcase", 0.95F)},
        2);
    require(
        first_jump.size() == 1U && first_jump.front().bbox_2d == outer_geometry,
        "changing-mode setup did not hold the first jump");

    const auto second_jump = tracker.update(
        {make_detection(second_inner_mode, 28, "suitcase", 0.95F)},
        3);
    require(second_jump.size() == 1U, "second changing mode emitted duplicates");
    require(
        second_jump.front().bbox_2d == second_inner_mode,
        "non-similar sustained jumps froze the public bbox beyond one frame");
}

// A single translated track inherits no duplicate-mode latency.
void test_single_track_translation_has_no_mode_hold() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});
    const cv::Rect original_geometry(10, 10, 100, 100);
    const cv::Rect translated_geometry(35, 10, 100, 100);

    require(
        tracker
                .update(
                    {make_detection(original_geometry, 56, "chair", 0.9F)},
                    1)
                .size() == 1U,
        "single-track translation setup did not create a track");
    const auto translated = tracker.update(
        {make_detection(translated_geometry, 56, "chair", 0.9F)},
        2);
    require(translated.size() == 1U, "single translated track disappeared");
    require(
        translated.front().bbox_2d == translated_geometry,
        "single-track same-size translation inherited duplicate-mode latency");
}

// A translated duplicate cluster stays suppressed, keeps its id and moves at once.
void test_duplicate_cluster_translation_has_no_mode_hold() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}});
    const cv::Rect outer_geometry(10, 10, 200, 200);
    const cv::Rect inner_geometry(55, 55, 100, 100);
    const cv::Rect translated_outer(40, 25, 200, 200);
    const cv::Rect translated_inner(85, 70, 100, 100);

    const auto first = tracker.update(
        {
            make_detection(outer_geometry, 62, "tv", 0.95F),
            make_detection(inner_geometry, 62, "tv", 0.85F),
        },
        1);
    require(first.size() == 1U, "duplicate translation setup emitted duplicates");
    const int canonical_track_id = first.front().track_id;

    const auto translated = tracker.update(
        {
            make_detection(translated_outer, 62, "tv", 0.95F),
            make_detection(translated_inner, 62, "tv", 0.85F),
        },
        2);
    require(translated.size() == 1U, "translated duplicate cluster was not suppressed");
    require(
        translated.front().track_id == canonical_track_id,
        "translated duplicate cluster changed canonical identity");
    require(
        translated.front().bbox_2d == translated_outer &&
            translated.front().raw_bbox_2d == translated_outer,
        "same-size duplicate cluster translation was frozen at its old position");
}

// Tier promotion of a duplicate is immediate: no bbox-mode latency, no identity change, no
// duplicate output.
void test_duplicate_bbox_tier_promotion_is_immediate() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect outer_geometry(10, 10, 100, 100);
    const cv::Rect inner_geometry(20, 20, 80, 80);
    const auto weak_duplicates = tracker.update(
        {
            make_detection(inner_geometry, 28, "suitcase", 0.20F, true),
            make_detection(outer_geometry, 56, "chair", 0.40F, true),
        },
        1);
    require(weak_duplicates.size() == 1U, "tier promotion setup emitted duplicates");
    require(
        weak_duplicates.front().bbox_2d == outer_geometry &&
            weak_duplicates.front().low_confidence_candidate,
        "tier promotion setup did not cache the weak outer mode");
    const int canonical_track_id = weak_duplicates.front().track_id;

    const auto promoted = tracker.update(
        {make_detection(inner_geometry, 28, "suitcase", 0.90F, false)},
        2);
    require(promoted.size() == 1U, "tier promotion emitted duplicate outputs");
    require(
        promoted.front().track_id == canonical_track_id,
        "tier promotion changed canonical identity");
    require(
        !promoted.front().low_confidence_candidate &&
            !promoted.front().current_observation_low_confidence_candidate,
        "tier promotion remained in the weak tier");
    require(
        promoted.front().bbox_2d == inner_geometry,
        "tier promotion inherited duplicate bbox mode latency");
}

// Expired bbox-mode state cannot revive old geometry after a gap; the identity buffer still keeps
// the id.
void test_expired_duplicate_bbox_mode_state_cannot_revive_old_geometry() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 10, 0.25F, 1, 1, 1.0F, {}, {}});
    const cv::Rect outer_geometry(10, 10, 200, 200);
    const cv::Rect inner_geometry(55, 55, 100, 100);
    const auto first = tracker.update(
        {
            make_detection(outer_geometry, 28, "suitcase", 0.95F),
            make_detection(inner_geometry, 28, "suitcase", 0.85F),
        },
        1);
    require(first.size() == 1U, "bbox TTL setup emitted duplicates");
    const int canonical_track_id = first.front().track_id;

    const auto pending = tracker.update(
        {make_detection(inner_geometry, 28, "suitcase", 0.95F)},
        2);
    require(
        pending.size() == 1U && pending.front().bbox_2d == outer_geometry,
        "bbox TTL setup did not create a held pending mode");
    require(tracker.update({}, 4).empty(), "bbox TTL gap still published stale tracks");

    const auto reappeared = tracker.update(
        {
            make_detection(inner_geometry, 28, "suitcase", 0.95F),
            make_detection(outer_geometry, 28, "suitcase", 0.85F),
        },
        5);
    require(reappeared.size() == 1U, "bbox TTL reappearance emitted duplicates");
    require(
        reappeared.front().track_id == canonical_track_id,
        "identity buffer did not preserve canonical id across bbox TTL gap");
    require(
        reappeared.front().bbox_2d == inner_geometry,
        "expired bbox mode state revived old accepted geometry");
}

// The duplicate identity handoff carries the semantic confirmation state to the older id.
void test_duplicate_identity_handoff_carries_semantic_state() {
    const perception::ByteTracker::Params params{
        0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F};
    perception::ByteTracker tracker(params);
    const cv::Rect older_geometry(0, 0, 100, 100);
    const cv::Rect newer_geometry(30, 0, 100, 100);
    const auto older_high = make_detection(
        older_geometry, 28, "suitcase", 0.90F, false);
    const auto newer_high = make_detection(
        newer_geometry, 28, "suitcase", 0.90F, false);
    const auto newer_low = make_detection(
        newer_geometry, 28, "suitcase", 0.20F, false);

    require(
        tracker.update({older_high, newer_high}, 1).empty(),
        "semantic handoff setup published first-hit duplicates");
    require(
        tracker.update({older_high, newer_high}, 2).empty(),
        "semantic handoff setup published second-hit duplicates");
    const auto older_confirmed = tracker.update({older_high}, 3);
    require(older_confirmed.size() == 1U, "older identity did not confirm on third high hit");
    require(
        older_confirmed.front().semantic_status == "confirmed",
        "older identity semantic setup was not confirmed");
    const int oldest_track_id = older_confirmed.front().track_id;

    require(
        tracker.update({newer_low}, 4).size() == 1U,
        "older identity did not coast during newer proxy buildup");
    require(
        tracker.update({newer_low}, 5).size() == 1U,
        "older identity vanished before semantic handoff");
    const auto handed_off = tracker.update({newer_low}, 6);
    require(handed_off.size() == 1U, "semantic identity handoff emitted duplicate outputs");
    require(
        handed_off.front().track_id == oldest_track_id,
        "semantic identity handoff did not retain oldest id");
    require(
        handed_off.front().bbox_2d == newer_geometry &&
            handed_off.front().last_seen_frame == 6,
        "semantic identity handoff did not retain current geometry/observation");
    require(
        handed_off.front().semantic_status == "confirmed",
        "oldest id carried the newer track's proxy semantic state");
    require(
        handed_off.front().semantic_consecutive_hits == 3,
        "oldest id lost its confirmed semantic evidence during handoff");
    require(
        handed_off.front().semantic_evidence_class_id == 28 &&
            handed_off.front().semantic_evidence_class_name == "suitcase",
        "oldest id lost its semantic evidence class during handoff");
    require(
        handed_off.front().semantic_low_quality_frames == 3,
        "oldest id lost its semantic grace age during handoff");

    perception::ByteTracker proxy_identity_tracker(params);
    const auto older_low = make_detection(
        older_geometry, 28, "suitcase", 0.20F, false);
    require(
        proxy_identity_tracker.update({older_low, newer_low}, 1).empty(),
        "proxy handoff setup published first-hit duplicates");
    require(
        proxy_identity_tracker.update({older_low, newer_low}, 2).empty(),
        "proxy handoff setup published second-hit duplicates");
    const auto older_proxy = proxy_identity_tracker.update({older_low}, 3);
    require(older_proxy.size() == 1U, "older proxy identity was not admitted");
    require(
        older_proxy.front().semantic_status == "proxy",
        "older low-confidence identity unexpectedly confirmed semantics");
    const int oldest_proxy_id = older_proxy.front().track_id;

    require(
        proxy_identity_tracker.update({newer_high}, 4).size() == 1U,
        "older proxy identity did not coast during newer confirmation");
    require(
        proxy_identity_tracker.update({newer_high}, 5).size() == 1U,
        "older proxy identity vanished before confirmed handoff");
    const auto proxy_handed_off =
        proxy_identity_tracker.update({newer_high}, 6);
    require(
        proxy_handed_off.size() == 1U,
        "proxy semantic identity handoff emitted duplicate outputs");
    require(
        proxy_handed_off.front().track_id == oldest_proxy_id,
        "proxy semantic identity handoff did not retain oldest id");
    require(
        proxy_handed_off.front().bbox_2d == newer_geometry &&
            proxy_handed_off.front().last_seen_frame == 6,
        "proxy semantic identity handoff did not retain current geometry");
    require(
        proxy_handed_off.front().semantic_status == "proxy",
        "oldest proxy id borrowed the newer track's confirmed semantics");
    require(
        proxy_handed_off.front().semantic_consecutive_hits == 0,
        "oldest proxy id borrowed the newer track's semantic streak");
    require(
        proxy_handed_off.front().semantic_evidence_class_id == -1 &&
            proxy_handed_off.front().semantic_evidence_class_name.empty(),
        "oldest proxy id borrowed the newer track's semantic evidence class");
}

// Weak tracks associate across classes, confirm at their own min hits and keep the weak marker,
// tier and proxy label.
void test_weak_track_associates_across_classes_and_uses_delayed_confirmation() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 3});
    const cv::Rect obstacle_bbox(40, 40, 100, 140);

    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
                1)
            .empty(),
        "weak track appeared before low_confidence_min_hits");
    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 56, "chair", 0.19F, true)},
                2)
            .empty(),
        "weak cross-class track appeared on the second hit");

    const auto confirmed = tracker.update(
        {make_detection(obstacle_bbox, 39, "bottle", 0.18F, true)},
        3);
    require(confirmed.size() == 1U, "weak track was not confirmed at its dedicated min hits");
    require(confirmed.front().low_confidence_candidate, "confirmed weak track lost its marker");
    require(
        confirmed.front().current_observation_low_confidence_candidate,
        "weak observation tier was not copied to the track output");
    require(confirmed.front().class_id == 28, "weak proxy label changed after cross-class association");
    require(confirmed.front().class_name == "suitcase", "weak proxy name changed after cross-class association");
    const int weak_track_id = confirmed.front().track_id;

    const auto continued = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.21F, true)},
        4);
    require(continued.size() == 1U, "confirmed weak track disappeared after class oscillation");
    require(continued.front().track_id == weak_track_id, "weak cross-class association changed track id");
    require(continued.front().class_id == 28, "weak track did not retain its first proxy class");
    require(
        continued.front().current_observation_low_confidence_candidate,
        "continued weak observation lost current tier");

    const auto coasted = tracker.update({}, 5);
    require(coasted.size() == 1U, "confirmed weak track did not coast");
    require(
        !coasted.front().current_observation_low_confidence_candidate,
        "coasting track incorrectly claimed a current weak observation");
}

// A same-class normal hit promotes a weak track in place: marker cleared, current tier ordinary,
// same track.
void test_weak_track_promotes_on_same_class_normal_hit() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 2, 5, 1.0F, {}, {}, 5});
    const cv::Rect obstacle_bbox(60, 60, 90, 120);

    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
                1)
            .empty(),
        "weak promotion setup published too early");
    const auto promoted = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.80F, false)},
        2);
    require(promoted.size() == 1U, "same-class normal hit did not promote weak track");
    require(!promoted.front().low_confidence_candidate, "promoted weak track kept weak marker");
    require(
        !promoted.front().current_observation_low_confidence_candidate,
        "normal promotion retained current weak tier");
    require(promoted.front().track_id == 1, "promotion created a replacement track");
}

// Semantic confirmation needs consecutive current same-class hits above the global threshold.
void test_semantic_confirmation_requires_current_consecutive_class_evidence() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F});
    const cv::Rect obstacle_bbox(60, 60, 90, 120);

    const auto weak = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.08F, true)},
        1);
    require(weak.size() == 1U, "weak semantic setup did not preserve obstacle output");
    require(weak.front().semantic_status == "proxy", "weak hit confirmed a semantic label");
    const int track_id = weak.front().track_id;

    const auto promoted_below_threshold = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.118F, false)},
        2);
    require(
        promoted_below_threshold.size() == 1U,
        "0.118 weak-to-normal promotion lost obstacle output");
    require(
        promoted_below_threshold.front().track_id == track_id,
        "0.118 weak-to-normal promotion changed id");
    require(
        !promoted_below_threshold.front().low_confidence_candidate,
        "0.118 normal-tier observation did not promote weak track");
    require(
        promoted_below_threshold.front().class_id == 56 &&
            promoted_below_threshold.front().class_name == "chair",
        "0.118 normal-tier observation did not update the proxy class");
    require(
        promoted_below_threshold.front().semantic_status == "proxy",
        "single 0.118 weak-to-normal promotion confirmed semantics");
    require(
        promoted_below_threshold.front().semantic_consecutive_hits == 0,
        "single 0.118 promotion counted below the global inference threshold");

    const auto one_high_hit = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        3);
    require(one_high_hit.size() == 1U, "one high semantic hit lost obstacle output");
    require(one_high_hit.front().track_id == track_id, "one high semantic hit changed id");
    require(
        one_high_hit.front().semantic_status == "proxy",
        "one high hit confirmed a weak track's semantic label");
    require(
        one_high_hit.front().semantic_consecutive_hits == 1,
        "first high semantic hit did not start a one-frame streak");

    const auto coasted = tracker.update({}, 4);
    require(coasted.size() == 1U, "semantic coast test lost admitted obstacle");
    require(coasted.front().track_id == track_id, "semantic coast changed id");
    require(coasted.front().semantic_status == "proxy", "coast retained semantic confirmation");
    require(
        coasted.front().semantic_consecutive_hits == 0,
        "coast did not reset the semantic hit streak");

    const auto first_sustained = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        5);
    require(first_sustained.size() == 1U, "first sustained semantic hit lost output");
    require(first_sustained.front().semantic_status == "proxy", "first sustained hit confirmed semantics");

    const auto second_sustained = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        6);
    require(second_sustained.size() == 1U, "second sustained semantic hit lost output");
    require(second_sustained.front().semantic_status == "proxy", "second sustained hit confirmed semantics");

    const auto third_sustained = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        7);
    require(third_sustained.size() == 1U, "third sustained semantic hit lost output");
    require(
        third_sustained.front().semantic_status == "confirmed",
        "three same-class hits above inference threshold did not confirm semantics");
    require(
        third_sustained.front().semantic_consecutive_hits == 3,
        "semantic confirmation did not use tracking min_hits");

    const auto same_class_low = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.299F, false)},
        8);
    require(same_class_low.size() == 1U, "same-class low hit lost confirmed obstacle");
    require(
        same_class_low.front().semantic_status == "confirmed",
        "same-class low hit erased prior semantic confirmation");
    require(
        same_class_low.front().semantic_consecutive_hits == 3,
        "same-class low hit erased confirmed semantic evidence");
    require(
        same_class_low.front().semantic_low_quality_frames == 1,
        "same-class low hit did not consume one grace frame");

    const auto same_class_weak = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.08F, true)},
        9);
    require(same_class_weak.size() == 1U, "same-class weak hit lost confirmed obstacle");
    require(
        same_class_weak.front().semantic_status == "confirmed",
        "same-class weak hit erased prior semantic confirmation");
    require(
        same_class_weak.front().semantic_consecutive_hits == 3,
        "same-class weak hit erased confirmed semantic evidence");
    require(
        same_class_weak.front().semantic_low_quality_frames == 2,
        "same-class weak hit did not consume a second grace frame");

    perception::ByteTracker threshold_tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F});
    const auto threshold_weak = threshold_tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.08F, true)},
        1);
    require(threshold_weak.size() == 1U, "threshold-equality setup lost weak obstacle");
    const auto first_threshold_equal = threshold_tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.30F, false)},
        2);
    require(first_threshold_equal.size() == 1U, "first threshold-equal hit lost obstacle output");
    require(
        first_threshold_equal.front().semantic_status == "proxy",
        "first threshold-equal hit confirmed semantics");

    const auto second_threshold_equal = threshold_tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.30F, false)},
        3);
    require(second_threshold_equal.size() == 1U, "second threshold-equal hit lost obstacle output");
    require(
        second_threshold_equal.front().semantic_status == "proxy",
        "second threshold-equal hit confirmed semantics");

    const auto third_threshold_equal = threshold_tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.30F, false)},
        4);
    require(third_threshold_equal.size() == 1U, "third threshold-equal hit lost obstacle output");
    require(
        third_threshold_equal.front().semantic_status == "confirmed",
        "inference threshold equality was not accepted after min_hits");
}

// A low-confidence promotion stays proxy until a qualifying high-confidence streak.
void test_low_confidence_promotion_remains_proxy_without_qualifying_streak() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F});
    const cv::Rect obstacle_bbox(65, 65, 90, 120);

    const auto weak = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.08F, true)},
        1);
    require(weak.size() == 1U, "low-promotion setup lost weak obstacle");
    const int track_id = weak.front().track_id;

    const auto promoted = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.118F, false)},
        2);
    require(promoted.size() == 1U, "0.118 promotion lost obstacle output");
    require(promoted.front().track_id == track_id, "0.118 promotion changed track id");
    require(promoted.front().semantic_status == "proxy", "0.118 promotion confirmed semantics");
    require(
        promoted.front().semantic_consecutive_hits == 0,
        "0.118 promotion counted as global-threshold evidence");

    const auto first_high = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.31F, false)},
        3);
    require(first_high.size() == 1U, "first isolated high hit lost obstacle output");
    require(first_high.front().semantic_status == "proxy", "first isolated high hit confirmed semantics");
    require(first_high.front().semantic_consecutive_hits == 1, "first isolated high hit did not count once");

    const auto second_high = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.31F, false)},
        4);
    require(second_high.size() == 1U, "second isolated high hit lost obstacle output");
    require(second_high.front().semantic_status == "proxy", "two high hits confirmed semantics");
    require(second_high.front().semantic_consecutive_hits == 2, "second high hit did not count twice");

    const auto coasted = tracker.update({}, 5);
    require(coasted.size() == 1U, "unconfirmed low-promotion track vanished on coast");
    require(coasted.front().semantic_status == "proxy", "unconfirmed coast confirmed semantics");
    require(
        coasted.front().semantic_consecutive_hits == 0,
        "unconfirmed coast did not clear the two-hit streak");

    const auto low = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.12F, false)},
        6);
    require(low.size() == 1U, "post-coast low hit lost obstacle output");
    require(low.front().semantic_status == "proxy", "post-coast low hit confirmed semantics");
    require(low.front().semantic_consecutive_hits == 0, "post-coast low hit restarted semantic streak");
}

// A hit of a different class breaks the semantic streak.
void test_different_class_breaks_semantic_streak() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F});
    const cv::Rect obstacle_bbox(70, 70, 90, 120);

    const auto weak = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.08F, true)},
        1);
    require(weak.size() == 1U, "different-class streak setup lost weak obstacle");
    const int admitted_track_id = weak.front().track_id;

    const auto first_chair = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        2);
    require(first_chair.size() == 1U, "first high chair hit lost admitted obstacle");
    require(first_chair.front().track_id == admitted_track_id, "first chair hit changed track id");
    require(first_chair.front().semantic_status == "proxy", "first chair hit confirmed semantics");
    require(
        first_chair.front().semantic_consecutive_hits == 1,
        "first chair hit did not start semantic streak");

    const auto second_chair = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        3);
    require(second_chair.size() == 1U, "second high chair hit lost admitted obstacle");
    require(second_chair.front().track_id == admitted_track_id, "second chair hit changed track id");
    require(second_chair.front().semantic_status == "proxy", "two chair hits confirmed semantics");
    require(
        second_chair.front().semantic_consecutive_hits == 2,
        "second chair hit did not extend semantic streak");

    const auto different_class = tracker.update(
        {make_detection(obstacle_bbox, 39, "bottle", 0.90F, false)},
        4);
    require(
        different_class.size() == 1U,
        "different-class hit changed existing track admission behavior");
    require(
        different_class.front().track_id == admitted_track_id,
        "different-class hit replaced the admitted track id");
    require(
        different_class.front().class_id == 56 &&
            different_class.front().class_name == "chair",
        "different-class hit stole the existing track before its own min_hits");
    require(
        different_class.front().last_seen_frame == 3,
        "different-class hit was incorrectly treated as same-track evidence");
    require(
        different_class.front().semantic_status == "proxy",
        "different-class hit completed the existing semantic streak");
    require(
        different_class.front().semantic_consecutive_hits == 0,
        "different-class hit did not reset the existing semantic streak");
}

// A confirmed class survives a bounded number of low-quality frames, and a new confirmation
// refreshes that grace.
void test_confirmed_semantic_low_quality_grace_is_bounded_and_refreshable() {
    const perception::ByteTracker::Params params{
        0.5F,
        30,
        0.25F,
        3,
        5,
        1.0F,
        {{"chair", 2}},
        {},
        1,
        0.30F};
    const cv::Rect obstacle_bbox(75, 75, 100, 130);
    const auto high = make_detection(
        obstacle_bbox, 56, "chair", 0.90F, false);
    const auto low = make_detection(
        obstacle_bbox, 56, "chair", 0.12F, false);
    const auto weak = make_detection(
        obstacle_bbox, 56, "chair", 0.08F, true);

    perception::ByteTracker expiring_tracker(params);
    require(expiring_tracker.update({high}, 1).empty(), "grace setup published first high hit");
    require(expiring_tracker.update({high}, 2).empty(), "grace setup published second high hit");
    const auto confirmed = expiring_tracker.update({high}, 3);
    require(confirmed.size() == 1U, "grace setup did not confirm chair");
    require(confirmed.front().semantic_status == "confirmed", "grace setup semantics not confirmed");
    require(confirmed.front().semantic_low_quality_frames == 0, "new confirmation consumed grace");

    const auto first_low = expiring_tracker.update({low}, 4);
    require(first_low.size() == 1U, "first low grace frame lost track");
    require(first_low.front().semantic_status == "confirmed", "first low grace frame downgraded semantics");
    require(first_low.front().semantic_low_quality_frames == 1, "first low grace frame not counted");

    const auto second_weak = expiring_tracker.update({weak}, 5);
    require(second_weak.size() == 1U, "second weak grace frame lost track");
    require(second_weak.front().semantic_status == "confirmed", "second grace frame downgraded semantics");
    require(second_weak.front().semantic_low_quality_frames == 2, "second grace frame not counted");

    const auto expired = expiring_tracker.update({low}, 6);
    require(expired.size() == 1U, "expired semantic grace lost admitted obstacle");
    require(expired.front().semantic_status == "proxy", "semantic grace did not expire");
    require(expired.front().semantic_consecutive_hits == 0, "expired grace retained positive streak");
    require(expired.front().semantic_low_quality_frames == 0, "expired grace counter did not reset");

    const auto reconfirm_one = expiring_tracker.update({high}, 7);
    require(reconfirm_one.size() == 1U, "first reconfirmation hit lost obstacle");
    require(reconfirm_one.front().semantic_status == "proxy", "one hit reconfirmed expired semantics");
    require(reconfirm_one.front().semantic_consecutive_hits == 1, "first reconfirmation hit not counted");
    const auto reconfirm_two = expiring_tracker.update({high}, 8);
    require(reconfirm_two.size() == 1U, "second reconfirmation hit lost obstacle");
    require(reconfirm_two.front().semantic_status == "proxy", "two hits reconfirmed expired semantics");
    const auto reconfirm_three = expiring_tracker.update({high}, 9);
    require(reconfirm_three.size() == 1U, "third reconfirmation hit lost obstacle");
    require(
        reconfirm_three.front().semantic_status == "confirmed",
        "expired semantics did not require and accept three fresh high hits");

    perception::ByteTracker refresh_tracker(params);
    require(refresh_tracker.update({high}, 1).empty(), "refresh setup published first high hit");
    require(refresh_tracker.update({high}, 2).empty(), "refresh setup published second high hit");
    const auto refresh_confirmed = refresh_tracker.update({high}, 3);
    require(refresh_confirmed.size() == 1U, "refresh setup did not publish third high hit");
    require(
        refresh_confirmed.front().semantic_status == "confirmed",
        "refresh setup did not confirm semantics");
    const auto low_before_refresh = refresh_tracker.update({low}, 4);
    require(low_before_refresh.size() == 1U, "pre-refresh low frame lost track");
    require(
        low_before_refresh.front().semantic_low_quality_frames == 1,
        "pre-refresh low frame not counted");
    const auto refreshed = refresh_tracker.update({high}, 5);
    require(refreshed.size() == 1U, "qualifying refresh frame lost track");
    require(refreshed.front().semantic_status == "confirmed", "qualifying evidence lost confirmation");
    require(
        refreshed.front().semantic_low_quality_frames == 0,
        "qualifying evidence did not replenish semantic grace");
    const auto weak_after_refresh = refresh_tracker.update({weak}, 6);
    require(weak_after_refresh.size() == 1U, "weak frame after refresh lost track");
    require(
        weak_after_refresh.front().semantic_status == "confirmed" &&
            weak_after_refresh.front().semantic_low_quality_frames == 1,
        "weak frame after refresh did not start a fresh grace budget");

    perception::ByteTracker mixed_tracker(
        perception::ByteTracker::Params{
            0.5F,
            30,
            0.25F,
            3,
            5,
            1.0F,
            {{"chair", 3}},
            {},
            1,
            0.30F});
    require(mixed_tracker.update({high}, 1).empty(), "mixed setup published first high hit");
    require(mixed_tracker.update({high}, 2).empty(), "mixed setup published second high hit");
    require(
        mixed_tracker.update({high}, 3).front().semantic_status == "confirmed",
        "mixed setup did not confirm semantics");
    const auto mixed_low = mixed_tracker.update({low}, 4);
    require(
        mixed_low.front().semantic_low_quality_frames == 1,
        "mixed low frame did not consume shared grace");
    const auto mixed_coast = mixed_tracker.update({}, 5);
    require(
        mixed_coast.front().semantic_status == "confirmed" &&
            mixed_coast.front().semantic_low_quality_frames == 2,
        "mixed coast did not consume shared grace");
    const auto mixed_weak = mixed_tracker.update({weak}, 6);
    require(
        mixed_weak.front().semantic_status == "confirmed" &&
            mixed_weak.front().semantic_low_quality_frames == 3,
        "mixed weak frame did not consume shared grace");
    const auto mixed_expired = mixed_tracker.update({low}, 7);
    require(
        mixed_expired.front().semantic_status == "proxy" &&
            mixed_expired.front().semantic_low_quality_frames == 0,
        "mixed low/weak/coast observations did not share one bounded grace budget");
}

// A confirmed semantic track stays published while it coasts inside the grace.
void test_confirmed_semantic_track_persists_on_short_coast() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F});
    const cv::Rect obstacle_bbox(80, 80, 100, 130);
    const auto chair_detection = make_detection(
        obstacle_bbox, 56, "chair", 0.90F, false);

    require(
        tracker.update({chair_detection}, 1).empty(),
        "confirmed-coast setup published first hit");
    require(
        tracker.update({chair_detection}, 2).empty(),
        "confirmed-coast setup published second hit");
    const auto confirmed = tracker.update({chair_detection}, 3);
    require(confirmed.size() == 1U, "third chair hit did not publish confirmed track");
    require(
        confirmed.front().semantic_status == "confirmed",
        "third chair hit did not confirm semantics");
    require(
        confirmed.front().semantic_consecutive_hits == 3,
        "confirmed semantic track did not reach min_hits");
    const int track_id = confirmed.front().track_id;

    for (std::uint64_t frame_id = 4; frame_id <= 8; ++frame_id) {
        const auto coasted = tracker.update({}, frame_id);
        require(coasted.size() == 1U, "confirmed semantic track vanished inside coast grace");
        require(coasted.front().track_id == track_id, "coast changed confirmed semantic track id");
        require(coasted.front().semantic_status == "confirmed", "short coast erased confirmed semantics");
        require(
            coasted.front().semantic_consecutive_hits == 3,
            "short coast erased confirmed semantic evidence");
        require(
            coasted.front().semantic_low_quality_frames ==
                static_cast<int>(frame_id - 3),
            "short coast consumed the wrong grace count");
    }

    require(
        tracker.update({}, 9).empty(),
        "track remained visible beyond existing output max age");
    const auto reacquired = tracker.update({chair_detection}, 10);
    require(reacquired.size() == 1U, "buffered track was not reacquired after coast expiry");
    require(reacquired.front().track_id == track_id, "identity buffer changed id after coast expiry");
    require(
        reacquired.front().semantic_status == "proxy",
        "expired coast grace retained semantic confirmation");
    require(
        reacquired.front().semantic_consecutive_hits == 1,
        "expired coast grace did not require a fresh semantic streak");
}

// A different-class observation downgrades a confirmed track to proxy without losing the track or
// its id.
void test_confirmed_semantic_track_downgrades_on_different_class_observation() {
    const auto run_case = [](float confidence, int expected_streak) {
        perception::ByteTracker tracker(
            perception::ByteTracker::Params{
                0.5F, 30, 0.25F, 3, 5, 1.0F, {}, {}, 1, 0.30F});
        const cv::Rect obstacle_bbox(90, 90, 100, 130);
        const auto chair_detection = make_detection(
            obstacle_bbox, 56, "chair", 0.90F, false);

        require(
            tracker.update({chair_detection}, 1).empty(),
            "different-class setup published first chair hit");
        require(
            tracker.update({chair_detection}, 2).empty(),
            "different-class setup published second chair hit");
        const auto confirmed = tracker.update({chair_detection}, 3);
        require(confirmed.size() == 1U, "different-class setup did not confirm chair");
        require(
            confirmed.front().semantic_status == "confirmed",
            "different-class setup chair semantics were not confirmed");
        const int track_id = confirmed.front().track_id;

        // ByteTrack association is intentionally unchanged and keys on class_id.
        // Hold that association key stable while changing the semantic class
        // name, so this test isolates the semantic contradiction transition.
        const auto changed = tracker.update(
            {make_detection(obstacle_bbox, 56, "bottle", confidence, false)},
            4);
        require(changed.size() == 1U, "different semantic class lost admitted track");
        require(changed.front().track_id == track_id, "different semantic class changed track id");
        require(
            changed.front().class_name == "bottle",
            "different semantic class was not applied to track output");
        require(
            changed.front().semantic_status == "proxy",
            "different semantic class retained old confirmation");
        require(
            changed.front().semantic_consecutive_hits == expected_streak,
            "different semantic class restarted at the wrong streak");
        require(
            changed.front().semantic_low_quality_frames == 0,
            "different semantic class inherited old grace");
    };

    run_case(0.90F, 1);
    run_case(0.118F, 0);
}

// An ordinary track accepts a same-class weak observation without being downgraded, and records
// the current weak tier.
void test_ordinary_track_records_current_weak_observation_tier() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 3});
    const cv::Rect obstacle_bbox(80, 80, 100, 120);
    const auto normal = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.80F, false)},
        1);
    require(normal.size() == 1U, "ordinary-tier setup did not create track");
    const int track_id = normal.front().track_id;

    const auto weak_observation = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
        2);
    require(weak_observation.size() == 1U, "ordinary track did not accept same-class weak observation");
    require(weak_observation.front().track_id == track_id, "same-class weak observation changed id");
    require(
        !weak_observation.front().low_confidence_candidate,
        "ordinary track was downgraded by a weak observation");
    require(
        weak_observation.front().current_observation_low_confidence_candidate,
        "current weak tier was lost on an ordinary track");
}

// One weak observation does not let an established track suppress a different normal class.
void test_established_current_weak_track_does_not_suppress_different_normal_class() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 3});
    const cv::Rect shared_projection(100, 100, 180, 220);

    const auto established = tracker.update(
        {
            make_detection(
                shared_projection, 56, "chair", 0.80F, false),
            make_detection(
                shared_projection, 89, "heavy_machinery", 0.85F, false),
        },
        1);
    require(
        established.size() == 2U,
        "different normal classes sharing a projection were suppressed at setup");

    const auto current_weak = tracker.update(
        {
            make_detection(
                shared_projection, 56, "chair", 0.08F, true),
            make_detection(
                shared_projection, 89, "heavy_machinery", 0.85F, false),
        },
        2);
    require(
        current_weak.size() == 2U,
        "one weak observation made an established normal track suppress a different class");
    require(
        std::count_if(
            current_weak.begin(),
            current_weak.end(),
            [](const perception::Track& track) {
                return track.low_confidence_candidate;
            }) == 0,
        "established normal track was incorrectly converted to generic weak identity");
    require(
        std::count_if(
            current_weak.begin(),
            current_weak.end(),
            [](const perception::Track& track) {
                return track.current_observation_low_confidence_candidate;
            }) == 1,
        "current weak observation provenance was not preserved for downstream depth gating");
}

// Between duplicates the current weak observation wins over a stale normal one, and stays under
// the weak depth gate.
void test_current_weak_output_beats_stale_normal_duplicate() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect obstacle_bbox(100, 100, 120, 140);
    const auto normal = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.80F, false)},
        1);
    require(normal.size() == 1U, "current-vs-stale setup did not create normal track");
    const int stale_normal_id = normal.front().track_id;

    const auto current_weak = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
        2);
    require(current_weak.size() == 1U, "duplicate suppression did not select one output");
    require(
        current_weak.front().track_id != stale_normal_id,
        "stale normal identity replaced the current weak observation");
    require(
        current_weak.front().last_seen_frame == 2,
        "stale normal geometry beat current weak geometry");
    require(
        current_weak.front().current_observation_low_confidence_candidate,
        "selected current weak output would bypass supervisor depth gate");
}

// Strong normal geometry promotes a weak track across classes: same id, the normal class, ordinary
// tier.
void test_weak_track_promotes_across_class_on_strong_normal_geometry() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect obstacle_bbox(120, 120, 100, 140);
    const auto weak = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
        1);
    require(weak.size() == 1U, "cross-class promotion setup did not create weak track");
    const int weak_track_id = weak.front().track_id;

    const auto promoted = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.90F, false)},
        2);
    require(promoted.size() == 1U, "strong normal geometry did not promote weak track");
    require(promoted.front().track_id == weak_track_id, "cross-class promotion changed track id");
    require(promoted.front().class_id == 56, "cross-class promotion kept proxy class");
    require(promoted.front().class_name == "chair", "cross-class promotion kept proxy name");
    require(!promoted.front().low_confidence_candidate, "cross-class promotion remained weak");
    require(
        !promoted.front().current_observation_low_confidence_candidate,
        "normal cross-class promotion retained weak current tier");
}

// A small normal detection contained in a weak track does not promote it; both obstacles stay.
void test_weak_track_rejects_contained_small_normal_promotion() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect weak_bbox(100, 100, 200, 200);
    const auto weak = tracker.update(
        {make_detection(weak_bbox, 28, "suitcase", 0.20F, true)},
        1);
    require(weak.size() == 1U, "contained-promotion setup did not create weak track");
    const int weak_track_id = weak.front().track_id;

    const cv::Rect contained_normal_bbox(160, 160, 60, 60);
    const auto output = tracker.update(
        {make_detection(contained_normal_bbox, 56, "chair", 0.90F, false)},
        2);
    require(output.size() == 2U, "independent contained normal or weak obstacle was suppressed");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [weak_track_id, weak_bbox](const perception::Track& track) {
                return track.track_id == weak_track_id && track.bbox_2d == weak_bbox &&
                       track.low_confidence_candidate;
            }),
        "contained normal erased or promoted the large weak obstacle");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [weak_track_id, contained_normal_bbox](const perception::Track& track) {
                return track.track_id != weak_track_id &&
                       track.bbox_2d == contained_normal_bbox &&
                       !track.low_confidence_candidate;
            }),
        "contained normal did not retain an independent current track");
}

// A small contained same-class detection does not promote a weak track either; both stay.
void test_weak_track_rejects_contained_small_same_class_promotion() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect weak_bbox(100, 100, 200, 200);
    const auto weak = tracker.update(
        {make_detection(weak_bbox, 56, "chair", 0.20F, true)},
        1);
    require(weak.size() == 1U, "same-class contained setup did not create weak track");
    const int weak_track_id = weak.front().track_id;

    const cv::Rect contained_normal_bbox(160, 160, 60, 60);
    const auto output = tracker.update(
        {make_detection(contained_normal_bbox, 56, "chair", 0.90F, false)},
        2);
    require(output.size() == 2U, "same-class contained normal or weak obstacle was suppressed");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [weak_track_id, weak_bbox](const perception::Track& track) {
                return track.track_id == weak_track_id && track.bbox_2d == weak_bbox &&
                       track.low_confidence_candidate;
            }),
        "same-class contained normal erased or promoted the large weak obstacle");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [weak_track_id, contained_normal_bbox](const perception::Track& track) {
                return track.track_id != weak_track_id &&
                       track.bbox_2d == contained_normal_bbox &&
                       !track.low_confidence_candidate;
            }),
        "same-class contained normal did not retain an independent current track");
}

// A distant cross-class detection does not promote a weak track.
void test_weak_track_rejects_distant_cross_class_promotion() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect weak_bbox(100, 100, 200, 200);
    const auto weak = tracker.update(
        {make_detection(weak_bbox, 28, "suitcase", 0.20F, true)},
        1);
    require(weak.size() == 1U, "distant-promotion setup did not create weak track");
    const int weak_track_id = weak.front().track_id;

    // IoU is still above 0.50, but a 60 px center displacement is too large
    // for a one-frame cross-class semantic promotion of this box.
    const cv::Rect distant_normal_bbox(160, 100, 200, 200);
    const auto output = tracker.update(
        {make_detection(distant_normal_bbox, 56, "chair", 0.90F, false)},
        2);
    require(output.size() == 2U, "distant normal or weak obstacle was suppressed");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [weak_track_id](const perception::Track& track) {
                return track.track_id == weak_track_id && track.low_confidence_candidate;
            }),
        "distant normal incorrectly promoted or erased the weak track");
    require(
        std::any_of(
            output.begin(),
            output.end(),
            [weak_track_id, distant_normal_bbox](const perception::Track& track) {
                return track.track_id != weak_track_id &&
                       track.bbox_2d == distant_normal_bbox &&
                       !track.low_confidence_candidate;
            }),
        "distant normal did not retain an independent current track");
}

// A normal same-class association takes precedence over a weak track's cross-class promotion.
void test_normal_same_class_association_beats_weak_cross_class_promotion() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 30, 0.25F, 1, 5, 1.0F, {}, {}, 1});
    const cv::Rect normal_bbox(0, 0, 100, 100);
    const cv::Rect weak_bbox(20, 20, 100, 100);
    const auto first = tracker.update(
        {
            make_detection(normal_bbox, 56, "chair", 0.90F, false),
            make_detection(weak_bbox, 28, "suitcase", 0.20F, true),
        },
        1);
    require(first.size() == 1U, "association-priority setup emitted duplicates");
    const int normal_track_id = first.front().track_id;

    const auto second = tracker.update(
        {make_detection(weak_bbox, 56, "chair", 0.90F, false)},
        2);
    require(second.size() == 1U, "association-priority update emitted duplicates");
    require(
        second.front().track_id == normal_track_id,
        "weak cross-class promotion stole a normal same-class detection");
}

// Weak tracks use the generic EMA and max_age, not the overrides of their proxy class.
void test_weak_track_uses_generic_ema_and_max_age() {
    perception::ByteTracker::Params params{
        0.5F,
        30,
        0.25F,
        1,
        2,
        0.4F,
        {{"suitcase", 9}},
        {{"suitcase", 0.05F}},
        1};
    perception::ByteTracker tracker(params);

    require(
        tracker
                .update(
                    {make_detection(cv::Rect(10, 10, 60, 60), 28, "suitcase", 0.20F, true)},
                    1)
                .size() == 1U,
        "generic weak policy setup did not create track");
    const auto shifted = tracker.update(
        {make_detection(cv::Rect(30, 10, 60, 60), 56, "chair", 0.20F, true)},
        2);
    require(shifted.size() == 1U, "generic weak policy lost cross-class track");
    require(shifted.front().bbox_2d.x == 18, "weak track used proxy-class EMA");

    require(tracker.update({}, 3).size() == 1U, "weak track vanished before generic max age");
    require(tracker.update({}, 4).size() == 1U, "weak track vanished at generic max age");
    require(tracker.update({}, 5).empty(), "weak track used proxy-class max age");
}

// An expired weak track can reconfirm with the same id inside the identity buffer.
void test_expired_weak_track_reconfirms_inside_identity_buffer() {
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{0.5F, 10, 0.25F, 1, 1, 1.0F, {}, {}, 3});
    const cv::Rect obstacle_bbox(140, 140, 80, 100);

    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
                1)
            .empty(),
        "weak re-confirmation setup published first hit");
    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
                2)
            .empty(),
        "weak re-confirmation setup published second hit");
    const auto confirmed = tracker.update(
        {make_detection(obstacle_bbox, 28, "suitcase", 0.20F, true)},
        3);
    require(confirmed.size() == 1U, "weak re-confirmation setup did not confirm");
    const int buffered_track_id = confirmed.front().track_id;

    require(tracker.update({}, 4).size() == 1U, "weak track vanished inside generic max age");
    require(tracker.update({}, 5).empty(), "weak track survived beyond generic max age");
    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 56, "chair", 0.20F, true)},
                6)
            .empty(),
        "buffered weak identity republished on first return hit");
    require(
        tracker
            .update(
                {make_detection(obstacle_bbox, 39, "bottle", 0.20F, true)},
                7)
            .empty(),
        "buffered weak identity republished on second return hit");
    const auto reconfirmed = tracker.update(
        {make_detection(obstacle_bbox, 56, "chair", 0.20F, true)},
        8);
    require(reconfirmed.size() == 1U, "buffered weak identity did not reconfirm");
    require(
        reconfirmed.front().track_id == buffered_track_id,
        "weak identity buffer did not preserve id during reconfirmation");
}

// The weak publication gate uses this frame's raw depth: invalid, fallback, sparse, beyond 6 m or
// held-unreliable samples block a weak observation; normal observations pass.
void test_weak_depth_publication_policy_uses_current_raw_evidence() {
    using perception::detail::should_publish_observation;
    require(
        should_publish_observation(false, false, true, 0, 0.0F, false),
        "normal observation was incorrectly gated by weak depth policy");
    require(
        !should_publish_observation(true, false, false, 8, 1.0F, true),
        "invalid current raw depth admitted weak observation");
    require(
        !should_publish_observation(true, true, true, 8, 1.0F, true),
        "fallback current raw depth admitted weak observation");
    require(
        !should_publish_observation(true, true, false, 4, 1.0F, true),
        "sparse current raw depth admitted weak observation");
    require(
        !should_publish_observation(true, true, false, 8, 6.1F, true),
        "out-of-range current raw depth admitted weak observation");
    require(
        !should_publish_observation(true, true, false, 8, 1.0F, false),
        "held unreliable stabilized depth admitted weak observation");
    require(
        should_publish_observation(true, true, false, 8, 1.0F, true),
        "good current raw depth rejected weak observation");

    perception::Track coasting_weak_track;
    coasting_weak_track.low_confidence_candidate = true;
    coasting_weak_track.current_observation_low_confidence_candidate = false;
    require(
        !should_publish_observation(
            coasting_weak_track.low_confidence_candidate ||
                coasting_weak_track.current_observation_low_confidence_candidate,
            false,
            false,
            0,
            0.0F,
            false),
        "coasting born-weak track bypassed raw depth gate");
}

// A mask of the wrong type does not change the tracker output.
void test_invalid_mask_type_keeps_tracker_output() {
    // The tracker only copies detection masks, so a mask that is not CV_8UC1
    // must not change association or the published track list.
    perception::ByteTracker tracker(
        perception::ByteTracker::Params{
            0.5F, 30, 0.5F, 1, 5, 1.0F, {}, {}});
    yolo::Detection invalid_mask =
        make_detection(cv::Rect(10, 10, 20, 20), 39, "bottle", 0.9F);
    invalid_mask.mask =
        cv::Mat::ones(invalid_mask.bbox.size(), CV_32F);
    const auto output = tracker.update({invalid_mask}, 1);

    require(
        output.size() == 1U,
        "invalid mask type changed normal tracker output");
}

}  // namespace

int main() {
    test_seen_times_populated_and_updated();
    test_confirmation_and_coast_hysteresis();
    test_output_bbox_ema_keeps_raw_detection_geometry();
    test_nested_same_class_detection_keeps_track_id();
    test_different_class_cannot_steal_track();
    test_track_buffer_preserves_identity_without_displaying_stale_box();
    test_skipped_results_age_by_source_frame_gap();
    test_per_class_max_age_only_extends_configured_class();
    test_per_class_bbox_ema_only_smooths_configured_class();
    test_published_tracks_suppress_only_nested_same_class_duplicates();
    test_duplicate_output_hands_current_geometry_to_oldest_identity();
    test_ordinary_partial_nested_modes_keep_oldest_identity_and_current_raw_geometry();
    test_partial_evidence_does_not_disable_legacy_containment();
    test_first_partial_evidence_still_uses_legacy_duplicate_rule();
    test_born_normal_independent_formal_ratio_remains_two_outputs();
    test_partial_nested_does_not_pair_current_weak_with_coasting_ordinary();
    test_promoted_born_weak_partial_nested_remains_independent();
    test_promoted_born_weak_requires_two_reciprocal_swaps();
    test_promoted_born_weak_two_swap_window_cannot_be_refreshed();
    test_partial_nested_requires_current_raw_geometry_with_low_alpha();
    test_partial_nested_reciprocal_swap_after_ttl_does_not_activate();
    test_partial_nested_tier_only_swap_does_not_activate();
    test_partial_nested_low_alpha_activation_uses_larger_public_bbox();
    test_partial_nested_invalid_frame_restores_both_outputs();
    test_partial_nested_three_track_keys_do_not_pollute_pairs();
    test_partial_nested_current_direct_three_track_cluster_converges();
    test_partial_nested_historical_cluster_edge_cannot_suppress_third_track();
    test_partial_nested_legacy_uses_frozen_cluster_member_metadata();
    test_similar_area_partial_overlap_remains_two_outputs();
    test_duplicate_bbox_mode_alternation_needs_two_consecutive_hits();
    test_changing_nested_mode_cannot_freeze_beyond_one_frame();
    test_single_track_translation_has_no_mode_hold();
    test_duplicate_cluster_translation_has_no_mode_hold();
    test_duplicate_bbox_tier_promotion_is_immediate();
    test_expired_duplicate_bbox_mode_state_cannot_revive_old_geometry();
    test_duplicate_identity_handoff_carries_semantic_state();
    test_weak_track_associates_across_classes_and_uses_delayed_confirmation();
    test_weak_track_promotes_on_same_class_normal_hit();
    test_semantic_confirmation_requires_current_consecutive_class_evidence();
    test_low_confidence_promotion_remains_proxy_without_qualifying_streak();
    test_different_class_breaks_semantic_streak();
    test_confirmed_semantic_low_quality_grace_is_bounded_and_refreshable();
    test_confirmed_semantic_track_persists_on_short_coast();
    test_confirmed_semantic_track_downgrades_on_different_class_observation();
    test_ordinary_track_records_current_weak_observation_tier();
    test_established_current_weak_track_does_not_suppress_different_normal_class();
    test_current_weak_output_beats_stale_normal_duplicate();
    test_weak_track_promotes_across_class_on_strong_normal_geometry();
    test_weak_track_rejects_contained_small_normal_promotion();
    test_weak_track_rejects_contained_small_same_class_promotion();
    test_weak_track_rejects_distant_cross_class_promotion();
    test_normal_same_class_association_beats_weak_cross_class_promotion();
    test_weak_track_uses_generic_ema_and_max_age();
    test_expired_weak_track_reconfirms_inside_identity_buffer();
    test_weak_depth_publication_policy_uses_current_raw_evidence();
    test_invalid_mask_type_keeps_tracker_output();
    std::cout << "test_bytetrack passed\n";
    return 0;
}
