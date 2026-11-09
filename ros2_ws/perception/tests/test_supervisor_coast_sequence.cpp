/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_supervisor_coast_sequence.cpp
 * Brief: Sequence test: not building a coasting track's geometry leaves every Objects body unchanged.
 *
 * Description:
 * What problem this solves. A track that was not observed in a frame (coasting) is never published,
 * so the service's geometry hook no longer builds its geometry (2026-10-13). A single-frame test
 * shows the adapter does not read that geometry; it cannot show that nothing carried from frame to
 * frame (the track's depth hysteresis, its velocity window, its identity and the order of the list)
 * depends on it. This test shows that over a sequence.
 *
 * How. Independent Supervisors (own tracker, hysteresis, velocity estimator, adapter, geometry
 * builder) are fed the same frames through the real process_loop(), from a test queue:
 *   reference  the hook builds the geometry of every track, coasting or not (the behaviour before);
 *   current    the service's hook as it is (coasting tracks skipped).
 * Both use the service's own measure_object_geometry(); the reference only presents a coasting
 * track to it as observed. Every published Objects body, its encoded wire bytes and the adapter's
 * report must be identical, frame by frame.
 *
 * The sequence (30 Hz capture time; tracker, depth and velocity parameters of the delivered
 * fixture tests/service/fixtures/perception.dev_candR.yaml): three objects are observed until
 * their velocity is estimated. Then
 *   - a static person and a static chair stop being detected; with "depth events" the depth under
 *     their last box jumps by more than the hysteresis threshold, becomes invalid and comes back;
 *   - they are detected again;
 *   - a moving car is not detected for three frames while the depth under its box stays as it was,
 *     so its velocity window lives through the coasting frames;
 *   - the person stays away longer than its published coasting age and than the velocity
 *     estimator keeps its state.
 * The pair reference / current is run twice: with the depth events and without them (steady
 * depth). The two scenes differ in what the coasting frames do to the kept state: a depth jump is
 * held by the hysteresis and a held or invalid sample clears the track's velocity state, while with
 * steady depth the velocity window of a coasting track stays alive and is used when the track is
 * detected again. That the two scenes publish different bodies from the first re-detection on is
 * checked too: it shows the coasting frames do reach later bodies through the kept state.
 *
 * What it guards and what not. It guards the geometry hook: a change that makes anything published
 * depend on a coasting track's geometry. Both arms share the Supervisor, so a change of how the
 * Supervisor itself treats coasting tracks is seen only by the absolute expectations in
 * check_scenario() (identities, counts, and the frames at which each track has a valid velocity
 * again), not by the comparison. It shows nothing about timing, the counters of the audit line or
 * inputs outside this sequence.
 *
 * Differences from the service, on purpose. init() is not run; prepare() sets what process_loop()
 * reads the way init() does, except: a FIFO test queue holding the whole sequence instead of the
 * latest-wins result slot (so no frame is ever skipped), no odom buffer (camera-frame velocity, as
 * in production while TF is refused), and the sort implementation fixed to the production value
 * instead of being read from the environment. Runtime switches are not read from the fixture: the
 * depth scan runs as in production (fast scan, PERCEPTION_DEPTH_SCAN_FAST=1) in this process and
 * once more on the reference scan in a child process; the two-lane sampler is run off, fixed and
 * shared. No camera, model, SDK or network.
 *
 * Also here, because it needs the same real loop: A19-NULL-4 (run_null4_case). A fourth object is
 * detected in every frame and has no depth under its box for 40 frames; every body of those frames
 * must keep it, with the four geometry fields and velocity_xy null on the wire.
 */

#include "supervisor/supervisor.hpp"

#include "common/logger.hpp"
#include "modules/rgb_depth_registration.hpp"
#include "output/rns_contract.hpp"
#include "output/rns_object_geometry.hpp"
#include "service/object_geometry_hook.hpp"
#include "strict_json.hpp"

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace perception {
struct SupervisorRnsJoinTestAccess {
    // The per-frame state of process_loop(), set from the configuration as init() sets it; the
    // capture, encoder and inference modules are not created (nothing here starts a thread).
    static void prepare(Supervisor& s, const yolo::Config& config, std::size_t queue_capacity) {
        s.resolved_config_ = config;
        s.config_ = &s.resolved_config_;
        s.rns_objects_ = std::make_unique<rns::FusedObjectsAdapter>(s.rns_context_->clock_epoch);
        s.rns_objects_->set_extrinsic_calibrated(s.rns_context_->extrinsic_calibrated);
        s.detection_queue_ = std::make_shared<yolo::ThreadsafeQueue<yolo::DetectionResult>>(
            queue_capacity, yolo::QueuePolicy::kDropOldestFifo);
        s.tracker_ = std::make_unique<ByteTracker>(*s.config_);
        s.depth_sampler_.set_params(DepthSampler::Params{
            s.config_->depth_sample_mode(), s.config_->depth_percentile(), s.config_->depth_cluster_width_m(),
            s.config_->depth_cluster_min_fraction(), s.config_->depth_cluster_min_pixels(),
            s.config_->depth_nearest_cluster_output_percentile()});
        auto params = s.depth_sampler_.params();
        params.sort_impl = DepthSampler::SortImplementation::CodeSort;  // production (depth_sort_impl)
        s.depth_sampler_.set_params(params);
        s.per_class_depth_samplers_.clear();
        for (const auto& [class_name, sample_mode] : s.config_->depth_per_class_sample_modes()) {
            auto class_params = params;
            class_params.sample_mode = sample_mode;
            s.per_class_depth_samplers_.emplace(class_name, DepthSampler(class_params));
        }
        s.depth_hysteresis_.set_params(TrackDepthHysteresis::Params{
            s.config_->depth_jump_threshold_m(), s.config_->depth_jump_candidate_tolerance_m(),
            s.config_->depth_jump_confirmation_frames()});
        // Camera-frame velocity only: no odom buffer, as in production while TF is refused.
        auto& v = s.lidar_velocity_estimator_;
        v.set_motion_decision_axes(MotionDecisionAxes::XZ);
        v.set_lateral_image_evidence_gate_enabled(true);
        v.set_camera_z_uncertainty_gate_config(CameraZUncertaintyGateConfig{
            s.config_->velocity_camera_z_uncertainty_gate_enabled(),
            s.config_->velocity_camera_z_uncertainty_stddev_multiplier(),
            s.config_->velocity_camera_z_uncertainty_max_threshold_m()});
        v.set_dt_max_s(s.config_->velocity_dt_max_s());
        v.set_centroid_ema_alpha(s.config_->velocity_centroid_ema_alpha());
        v.set_per_class_centroid_ema_alphas(s.config_->velocity_per_class_centroid_ema_alphas());
        v.set_velocity_window_s(s.config_->velocity_window_s());
        v.set_motion_confirmation_windows(s.config_->velocity_motion_confirmation_windows());
        v.set_static_displacement_m(s.config_->velocity_static_displacement_m());
        v.set_generic_static_displacement_m(s.config_->velocity_generic_static_displacement_m());
        v.set_per_class_static_displacements_m(s.config_->velocity_per_class_static_displacements_m());
        v.set_gating_config(VelocityGatingConfig{s.config_->velocity_gating_min_stable_frames(),
                                                 s.config_->velocity_gating_min_depth_confidence(),
                                                 s.config_->velocity_gating_skip_fusion_quality()});
    }
    static void push(Supervisor& s, yolo::DetectionResult result) { s.detection_queue_->push(std::move(result)); }
    // Runs the real loop on the calling thread until stop() (called from the Objects callback of
    // the last frame) or a failed frame.
    static void run(Supervisor& s) {
        s.running_.store(true);
        s.process_loop();
        s.running_.store(false);
    }
    static void stop(Supervisor& s) { s.running_.store(false); }
};
}  // namespace perception

namespace {
using namespace perception;
using namespace perception::rns;
using perception::service::ObjectGeometryShared;
using perception::service::measure_object_geometry;

std::size_t checks = 0;
void check(bool ok, int line) {
    ++checks;
    if (!ok) throw std::runtime_error("coast sequence line " + std::to_string(line));
}
#define CHECK(x) check(static_cast<bool>(x), __LINE__)

// ---- the scene ----
constexpr int kWidth = 1280, kHeight = 800;
const yolo::DepthIntrinsics kColour{690.0F, 690.0F, 640.0F, 400.0F, kWidth, kHeight};
constexpr float kDepthScale = 0.001F;
constexpr std::uint64_t kEpoch = 7;
constexpr std::int64_t kT0Ns = 10000000000LL, kFrameNs = 33333333LL, kLinkNs = 20000000LL;
constexpr int kFrames = 90;
constexpr std::uint64_t kFrameIdBase = 100;

// Detected in [1, kFrames] except the listed gaps (inclusive frame numbers).
struct Gap { int first, last; };
struct SceneObject {
    const char* class_name;
    int class_id;
    float confidence;
    cv::Rect box;          // at frame 1
    int dx_per_frame;      // lateral motion of the box, pixels
    std::uint16_t depth;   // codes (mm)
    std::vector<Gap> gaps;
};
// person: generic coasting age 5; chair: per-class coasting age 15; car: moving, generic age 5.
const std::vector<SceneObject>& scene() {
    static const std::vector<SceneObject> objects{
        {"person", 0, 0.90F, cv::Rect(150, 250, 120, 300), 0, 2000, {{31, 34}, {50, 70}}},
        {"chair", 56, 0.80F, cv::Rect(500, 350, 200, 200), 0, 3000, {{31, 40}}},
        {"car", 2, 0.85F, cv::Rect(800, 300, 110, 220), 4, 2500, {{45, 47}}},
    };
    return objects;
}
// A19-NULL-4: a fourth object for the "detected, but not locatable" sequence. It is detected in
// every frame; in [kNoDepthFirst, kNoDepthLast] the depth camera returns nothing under its box.
// Its box lies below the car's row and right of the chair, so it overlaps no other box at any
// frame. Only run_arm() calls that pass it see it; the coasting comparison above does not.
constexpr int kNoDepthFirst = 21, kNoDepthLast = 60;
const SceneObject kNoDepthObject{"bench", 13, 0.75F, cv::Rect(1000, 560, 160, 120), 0, 3500, {}};
bool detected(const SceneObject& o, int frame) {
    for (const auto& gap : o.gaps) if (frame >= gap.first && frame <= gap.last) return false;
    return true;
}
// What the depth camera sees in an object's (last) box while the object is not detected:
// the depth events of the coasting phases. 0 = invalid pixels.
std::uint16_t depth_while_undetected(std::size_t object, int frame, bool depth_events) {
    const auto& o = scene()[object];
    if (!depth_events) return o.depth;
    if (object == 0) {  // person
        if (frame == 31 || frame == 32) return 2500;  // jump above the hysteresis threshold
        if (frame == 33) return 0;                    // invalid
        return o.depth;                               // back (frame 34 and the long gap)
    }
    if (object == 1) {  // chair
        if (frame >= 31 && frame <= 37) return 3600;  // jump, held long enough to be confirmed
        if (frame == 38) return 0;                    // invalid
        return o.depth;                               // back: a jump again, against the confirmed 3.6 m
    }
    return o.depth;
}
cv::Rect box_at(const SceneObject& o, int frame) {
    return cv::Rect(o.box.x + o.dx_per_frame * (frame - 1), o.box.y, o.box.width, o.box.height);
}
// An ellipse inside the box (1 = object), so the mask matters to the sampler and the builder.
cv::Mat mask_of(const cv::Rect& box) {
    cv::Mat mask(box.height, box.width, CV_8UC1, cv::Scalar(0));
    const double a = box.width / 2.0, b = box.height / 2.0;
    for (int y = 0; y < box.height; ++y) {
        auto* row = mask.ptr<std::uint8_t>(y);
        for (int x = 0; x < box.width; ++x) {
            const double u = (x + 0.5 - a) / a, w = (y + 0.5 - b) / b;
            row[x] = u * u + w * w <= 1.0 ? 255 : 0;
        }
    }
    return mask;
}

struct Raster {
    std::shared_ptr<const RgbDepthRegistration> registration;
    cv::Mat colour;  // one shared image: the loop only keeps references to it
};

yolo::DetectionResult make_frame(int frame, const Raster& raster, std::uint64_t stream, bool depth_events,
                                 const SceneObject* no_depth_object = nullptr) {
    yolo::DetectionResult result;
    result.frame_id = kFrameIdBase + static_cast<std::uint64_t>(frame);
    result.inference_completed = true;
    const auto k = static_cast<std::uint64_t>(frame);
    result.native_depth_source = {stream, k, k + 10, k + 20, k * 33333 + 1, k * 33333 + 2, kT0Ns + frame * kFrameNs};
    result.stamp_sec = 1000.0 + frame / 30.0;
    result.frame = raster.colour;
    result.depth_intrinsics = kColour;
    result.depth_scale = kDepthScale;
    result.depth_registration = raster.registration;
    // A wall 4.5 m away, then each object's box at its depth with a few millimetres of relief.
    result.depth = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(4500));
    for (std::size_t i = 0; i < scene().size(); ++i) {
        const auto& o = scene()[i];
        const bool seen = detected(o, frame);
        // While undetected the box stays where it was last detected (the track's raw box).
        int last_seen = frame;
        while (last_seen > 1 && !detected(o, last_seen)) --last_seen;
        const cv::Rect box = box_at(o, last_seen);
        const std::uint16_t base = seen ? o.depth : depth_while_undetected(i, frame, depth_events);
        for (int y = box.y; y < box.y + box.height; ++y) {
            auto* row = result.depth.ptr<std::uint16_t>(y);
            for (int x = box.x; x < box.x + box.width; ++x)
                row[x] = base == 0 ? 0 : static_cast<std::uint16_t>(base + (x * 7 + y * 13) % 5);
        }
        if (!seen) continue;
        yolo::Detection detection;
        detection.bbox = box;
        detection.confidence = o.confidence;
        detection.class_id = o.class_id;
        detection.class_name = o.class_name;
        detection.mask = mask_of(box);
        result.detections.push_back(std::move(detection));
    }
    if (no_depth_object != nullptr) {
        // Detected in this frame whatever the depth is; no depth at all under its box in the interval.
        const auto& o = *no_depth_object;
        const cv::Rect box = box_at(o, frame);
        const bool no_depth = frame >= kNoDepthFirst && frame <= kNoDepthLast;
        for (int y = box.y; y < box.y + box.height; ++y) {
            auto* row = result.depth.ptr<std::uint16_t>(y);
            for (int x = box.x; x < box.x + box.width; ++x)
                row[x] = no_depth ? 0 : static_cast<std::uint16_t>(o.depth + (x * 7 + y * 13) % 5);
        }
        yolo::Detection detection;
        detection.bbox = box;
        detection.confidence = o.confidence;
        detection.class_id = o.class_id;
        detection.class_name = o.class_name;
        detection.mask = mask_of(box);
        result.detections.push_back(std::move(detection));
    }
    return result;
}

// ---- one arm: an independent Supervisor with its own geometry capability ----
struct Published {
    std::uint64_t frame_id = 0;
    Objects body;
    std::string wire;
    FusedObjectsReport report;
};
enum class Hook { BuildEveryTrack, Service };
struct ArmResult {
    std::vector<Published> frames;
    std::uint64_t coast_skipped = 0, coast_skipped_points = 0, build_calls = 0, located = 0, not_ready = 0,
        raster_mismatch = 0, mask_unusable = 0, selected_list_rejected = 0, float_points = 0;
    bool fatal = false;
};

std::int64_t journal_clock(void*) noexcept { return kT0Ns; }

void make_ready(ObjectGeometryShared& shared, const Raster& raster, double float_margin_m) {
    OpticalRayTable rays;
    rays.width = static_cast<std::uint32_t>(raster.registration->width());
    rays.height = static_cast<std::uint32_t>(raster.registration->height());
    rays.x = raster.registration->ray_x();
    rays.y = raster.registration->ray_y();
    // Camera 0.8 m high, level, looking along base +x (optical z forward, x right, y down).
    rns::RigidTransform base_from_colour;
    base_from_colour.R = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    base_from_colour.t = {0, 0, 0.8};
    ObjectGeometryOptions options{5, 0.01, 0.02};  // the service's kObjectGeometryOptions
    options.float_margin_m = float_margin_m;
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.builder = std::make_unique<ObjectGeometryBuilder>(std::move(rays), base_from_colour, options);
    shared.colour = kColour;
    shared.registration = raster.registration;
    for (std::size_t i = 0; i < 9; ++i) shared.base_from_colour_R[i] = static_cast<float>(base_from_colour.R[i]);
    shared.ready.store(true);
}

std::size_t strict_failures = 0;   // Objects texts that failed the strict parse or the inf / nan scan

std::string encode_wire(const Objects& body) {
    Objects stamped = body;
    stamped.t_publish_mono_ms = body.t_capture_mono_ms + 1;  // the publisher's stamp; fixed here
    std::vector<char> buffer(262144);
    const auto encoded = encode(stamped, buffer.data(), buffer.size());
    CHECK(encoded);
    std::string wire(buffer.data(), encoded.size);
    // A19-ENC-1 on the Objects text of every body the real loop publishes here: strict JSON and
    // no inf / nan token (tests/strict_json.hpp). Counted, not thrown: this runs inside the
    // Objects callback of the loop, where an exception only looks like a failed frame; run_arm()
    // checks the count after the loop.
    if (!test_json::validate(wire).ok || test_json::first_non_finite_token(wire) != std::string_view::npos)
        ++strict_failures;
    return wire;
}

ArmResult run_arm(const yolo::Config& config, const Raster& raster, Hook hook, double float_margin_m,
                  bool depth_events, const SceneObject* no_depth_object = nullptr) {
    ArmResult out;
    auto shared = std::make_shared<ObjectGeometryShared>();
    make_ready(*shared, raster, float_margin_m);
    Supervisor supervisor;
    Supervisor* self = &supervisor;
    SupervisorRnsContext context;
    context.native_mailbox = std::make_shared<native_depth::Mailbox>(64);
    const std::uint64_t stream = context.native_mailbox->begin_stream();
    context.depth_arrivals = std::make_shared<SourceJournal>(SourceKind::DepthArrival, 8, SourceClock{nullptr, &journal_clock});
    context.inference_completions =
        std::make_shared<SourceJournal>(SourceKind::InferenceComplete, 8, SourceClock{nullptr, &journal_clock});
    context.clock_epoch = kEpoch;
    context.extrinsic_calibrated = true;
    context.callback_owner = shared;
    // Later than every frame's host-ready time and never going back.
    context.now_ns = [] { return kT0Ns + (kFrames + 10) * kFrameNs; };
    context.capture_stamp = [](const native_depth::SourceId& source, geometry::CaptureStamp& stamp) {
        stamp.source = source;
        stamp.clock_epoch = kEpoch;
        stamp.capture_mono_ns = source.host_ready_mono_ns - kLinkNs;
        return true;
    };
    context.objects_ready = [&out, self](const yolo::DetectionResult& result, const geometry::CaptureStamp&,
                                         const Objects& body, const FusedObjectsReport& report) {
        out.frames.push_back({result.frame_id, body, encode_wire(body), report});
        if (result.frame_id == kFrameIdBase + kFrames) SupervisorRnsJoinTestAccess::stop(*self);
        return true;
    };
    context.object_geometry = [shared, hook](const yolo::DetectionResult& result, const cv::Rect& bbox,
                                             const cv::Mat& mask, float lo, float hi,
                                             const std::vector<std::uint32_t>* selected, FusedDetection& detection) {
        if (hook == Hook::Service) {
            measure_object_geometry(*shared, result, bbox, mask, lo, hi, selected, detection);
            return;
        }
        // Before 2026-10-13: every track built. Present the track as observed for this one call.
        const int last_seen = detection.last_seen_frame;
        detection.last_seen_frame = static_cast<int>(result.frame_id);
        measure_object_geometry(*shared, result, bbox, mask, lo, hi, selected, detection);
        detection.last_seen_frame = last_seen;
    };
    CHECK(context.valid());
    CHECK(supervisor.configure_rns(std::move(context)));
    SupervisorRnsJoinTestAccess::prepare(supervisor, config, kFrames + 1);
    for (int frame = 1; frame <= kFrames; ++frame)
        SupervisorRnsJoinTestAccess::push(supervisor, make_frame(frame, raster, stream, depth_events, no_depth_object));
    SupervisorRnsJoinTestAccess::run(supervisor);
    CHECK(strict_failures == 0);   // A19-ENC-1: every Objects text of this arm is strict JSON without inf / nan
    out.fatal = supervisor.fatal_failure_detected();
    out.coast_skipped = shared->coast_skipped.load();
    out.coast_skipped_points = shared->coast_skipped_points.load();
    out.build_calls = shared->build_calls.load();
    out.located = shared->located.load();
    out.not_ready = shared->not_ready.load();
    out.raster_mismatch = shared->raster_mismatch.load();
    out.mask_unusable = shared->mask_unusable.load();
    out.selected_list_rejected = shared->selected_list_rejected.load();
    out.float_points = shared->float_points.load();
    return out;
}

// ---- comparisons (floats by bit pattern: the values must be the same numbers, not close ones) ----
template <typename T>
bool same_bits(const T& a, const T& b) { return std::memcmp(&a, &b, sizeof(T)) == 0; }
bool same_object(const Object& a, const Object& b) {
    if (a.track_id != b.track_id || a.class_id != b.class_id || a.class_name != b.class_name ||
        !same_bits(a.confidence, b.confidence) || a.semantic_status != b.semantic_status ||
        a.velocity_frame != b.velocity_frame || a.velocity_status != b.velocity_status ||
        a.depth_quality != b.depth_quality || a.velocity_valid != b.velocity_valid ||
        a.stable_frames != b.stable_frames || a.first_seen_mono_ms != b.first_seen_mono_ms ||
        a.last_seen_mono_ms != b.last_seen_mono_ms || a.geometry.has_value() != b.geometry.has_value() ||
        a.velocity_xy.has_value() != b.velocity_xy.has_value())
        return false;
    if (a.velocity_xy && (!same_bits((*a.velocity_xy)[0], (*b.velocity_xy)[0]) ||
                          !same_bits((*a.velocity_xy)[1], (*b.velocity_xy)[1])))
        return false;
    if (a.geometry) {
        const auto& g = *a.geometry;
        const auto& h = *b.geometry;
        if (!same_bits(g.z_min, h.z_min) || !same_bits(g.z_max, h.z_max) ||
            g.footprint_xy.size() != h.footprint_xy.size())
            return false;
        for (std::size_t i = 0; i < g.footprint_xy.size(); ++i)
            if (!same_bits(g.footprint_xy[i].x, h.footprint_xy[i].x) ||
                !same_bits(g.footprint_xy[i].y, h.footprint_xy[i].y))
                return false;
    }
    return true;
}
bool same_published(const Published& a, const Published& b) {
    if (a.frame_id != b.frame_id || a.wire != b.wire ||
        a.body.t_capture_mono_ms != b.body.t_capture_mono_ms ||
        a.body.t_publish_mono_ms != b.body.t_publish_mono_ms ||
        a.body.extrinsic_calibrated != b.body.extrinsic_calibrated ||
        a.body.objects.size() != b.body.objects.size())
        return false;
    for (std::size_t i = 0; i < a.body.objects.size(); ++i)  // same order, not only the same set
        if (!same_object(a.body.objects[i], b.body.objects[i])) return false;
    const auto& r = a.report;
    const auto& q = b.report;
    return r.status == q.status && r.frame_id == q.frame_id && r.input_fused == q.input_fused &&
           r.tracks == q.tracks && r.coast_filtered == q.coast_filtered && r.objects == q.objects &&
           r.located == q.located && r.unlocatable == q.unlocatable && r.velocity_numeric == q.velocity_numeric &&
           r.velocity_unconfirmed_numeric == q.velocity_unconfirmed_numeric &&
           r.velocity_gating_mapped == q.velocity_gating_mapped &&
           r.velocity_gated_track_unstable == q.velocity_gated_track_unstable &&
           r.velocity_gated_low_depth_confidence == q.velocity_gated_low_depth_confidence &&
           r.velocity_gated_depth_unreliable == q.velocity_gated_depth_unreliable && r.detail == q.detail;
}
std::size_t first_difference(const ArmResult& a, const ArmResult& b) {
    const std::size_t n = std::min(a.frames.size(), b.frames.size());
    for (std::size_t i = 0; i < n; ++i)
        if (!same_published(a.frames[i], b.frames[i])) return i;
    return a.frames.size() == b.frames.size() ? std::numeric_limits<std::size_t>::max() : n;
}
constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

const Object* find_class(const Published& p, const std::string& class_name) {
    for (const auto& o : p.body.objects) if (o.class_name == class_name) return &o;
    return nullptr;
}
const Published& at(const ArmResult& arm, int frame) { return arm.frames.at(static_cast<std::size_t>(frame - 1)); }

// One line per frame for a failed run or PERCEPTION_TEST_VERBOSE=1: what was published.
void print_sequence(const char* label, const ArmResult& arm) {
    for (const auto& p : arm.frames) {
        std::cout << label << " frame=" << (p.frame_id - kFrameIdBase) << " tracks=" << p.report.tracks
                  << " fused=" << p.report.input_fused << " coast=" << p.report.coast_filtered << " objects=";
        for (const auto& o : p.body.objects) {
            std::cout << o.class_name << "#" << o.track_id << ":" << o.semantic_status << ":" << o.velocity_status
                      << (o.velocity_valid ? ":valid" : "") << ":s" << o.stable_frames
                      << (o.geometry ? "" : ":unlocatable");
            if (o.velocity_xy) std::cout << ":v(" << (*o.velocity_xy)[0] << "," << (*o.velocity_xy)[1] << ")";
            std::cout << " ";
        }
        std::cout << "\n";
    }
}

// First frame from `from` on at which the object of this class is published with a valid velocity;
// 0 if there is none.
int first_valid_velocity(const ArmResult& arm, const std::string& class_name, int from) {
    for (int frame = from; frame <= kFrames; ++frame) {
        const Object* o = find_class(at(arm, frame), class_name);
        if (o && o->velocity_valid) return frame;
    }
    return 0;
}
// The frames at which each track has a valid velocity again after it was detected again: the
// absolute outcome of what the coasting frames did to the kept state.
struct VelocityReturns {
    int person_after_35, chair_after_41, car_after_48, person_after_71;
    bool operator==(const VelocityReturns& o) const {
        return person_after_35 == o.person_after_35 && chair_after_41 == o.chair_after_41 &&
               car_after_48 == o.car_after_48 && person_after_71 == o.person_after_71;
    }
};
VelocityReturns velocity_returns(const ArmResult& arm) {
    return {first_valid_velocity(arm, "person", 35), first_valid_velocity(arm, "chair", 41),
            first_valid_velocity(arm, "car", 48), first_valid_velocity(arm, "person", 71)};
}
// The expected frames, with the reason for each (velocity window 0.30 s = 9 frame periods):
//   depth events   person 44: detected again at 35 with its velocity state cleared, one full window;
//                  chair 54: detected again at 41 while the depth jump back to 3.0 m is still
//                  held against the 3.6 m confirmed during coasting (accepted at 44), then a window;
//   steady depth   person 35, chair 41: the window lived through the coasting frames;
//   both scenes    car 48: steady depth under its box, valid at once; person 81: after the long
//                  gap its state was forgotten and the track has to become stable again first.
const VelocityReturns kEventReturns{44, 54, 48, 81};
const VelocityReturns kSteadyReturns{35, 41, 48, 81};

// The scenario is what the file header says it is, in absolute terms.
void check_scenario(const ArmResult& arm) {
    CHECK(!arm.fatal && arm.frames.size() == static_cast<std::size_t>(kFrames));
    // Every frame published, in order.
    for (int frame = 1; frame <= kFrames; ++frame) CHECK(at(arm, frame).frame_id == kFrameIdBase + frame);
    // Before the first gap all three are published, each with a velocity estimate.
    const auto& before = at(arm, 30);
    CHECK(before.body.objects.size() == 3 && before.report.coast_filtered == 0);
    const Object* person = find_class(before, "person");
    const Object* chair = find_class(before, "chair");
    const Object* car = find_class(before, "car");
    CHECK(person && chair && car);
    CHECK(person->geometry && chair->geometry && car->geometry);
    CHECK(person->velocity_status == "static" && person->velocity_valid);
    CHECK(chair->velocity_status == "static" && chair->velocity_valid);
    CHECK(car->velocity_status == "moving" && car->velocity_valid);
    const auto person_id = person->track_id, chair_id = chair->track_id, car_id = car->track_id;
    CHECK(person_id != chair_id && chair_id != car_id && person_id != car_id);
    // Frames 31-34: person and chair coast (tracked, not published). 35-40: only the chair.
    for (int frame = 31; frame <= 40; ++frame) {
        const auto& p = at(arm, frame);
        const std::size_t coasting = frame <= 34 ? 2 : 1;
        CHECK(p.report.tracks == 3 && p.report.coast_filtered == coasting);
        CHECK(p.body.objects.size() == 3 - coasting);
        const Object* moving = find_class(p, "car");
        CHECK(moving && moving->track_id == car_id);
        CHECK(!find_class(p, "chair") && (frame <= 34) == !find_class(p, "person"));
    }
    // Detected again: the same identities, first-seen times unchanged.
    const Object* person_back = find_class(at(arm, 35), "person");
    CHECK(person_back);
    CHECK(person_back->track_id == person_id && person_back->first_seen_mono_ms == person->first_seen_mono_ms);
    const Object* chair_back = find_class(at(arm, 41), "chair");
    CHECK(chair_back);
    CHECK(chair_back->track_id == chair_id && chair_back->first_seen_mono_ms == chair->first_seen_mono_ms);
    // Frames 45-47: the car coasts, the other two are published.
    for (int frame = 45; frame <= 47; ++frame) {
        const auto& p = at(arm, frame);
        CHECK(p.report.tracks == 3 && p.report.coast_filtered == 1 && p.body.objects.size() == 2);
        CHECK(!find_class(p, "car") && find_class(p, "person") && find_class(p, "chair"));
    }
    // The long gap of the person (50-70): published coasting age 5, then the track leaves the list
    // but keeps its identity for the detection of frame 71. By then its last velocity sample
    // (frame 54) is older than dt_max (0.5 s) and forgotten: the estimate starts anew.
    for (int frame = 50; frame <= 70; ++frame) {
        const auto& p = at(arm, frame);
        CHECK(!find_class(p, "person") && p.body.objects.size() == 2);
        CHECK(p.report.tracks == (frame <= 54 ? 3U : 2U) && p.report.coast_filtered == (frame <= 54 ? 1U : 0U));
    }
    const Object* person_late = find_class(at(arm, 71), "person");
    CHECK(person_late);
    CHECK(person_late->track_id == person_id && person_late->first_seen_mono_ms == person->first_seen_mono_ms);
    CHECK(person_late->velocity_status == "warming_up" && !person_late->velocity_valid && !person_late->velocity_xy);
    // At the end everything is observed and estimated again.
    const auto& last = at(arm, kFrames);
    CHECK(last.body.objects.size() == 3 && last.report.coast_filtered == 0 && last.report.located == 3);
    for (const auto& o : last.body.objects) CHECK(o.velocity_valid && o.semantic_status == "confirmed");
    // Outside its own gap the car is published in every frame from its confirmation on, always
    // the same identity.
    for (int frame = 3; frame <= kFrames; ++frame) {
        if (frame >= 45 && frame <= 47) continue;
        const Object* o = find_class(at(arm, frame), "car");
        CHECK(o && o->track_id == car_id && o->geometry);
    }
}

// The pipeline configuration as the service hands it to yolo::Config (service_config.cpp): the
// delivered fixture's perception.pipeline without runtime / media / ros, plus an hevc block. The two
// sealed dual-model contract files are the ones of this source tree (the fixture names a path of
// another machine).
yolo::Config load_pipeline_config(const std::filesystem::path& root) {
    const auto fixture = root / "tests/service/fixtures/perception.dev_candR.yaml";
    const YAML::Node pipeline = YAML::LoadFile(fixture.string())["perception"]["pipeline"];
    CHECK(pipeline && pipeline.IsMap());
    YAML::Node doc(YAML::NodeType::Map);
    for (const auto& item : pipeline) {
        const auto key = item.first.as<std::string>();
        if (key != "runtime" && key != "media" && key != "ros") doc[key] = YAML::Clone(item.second);
    }
    doc["dual_model"]["runtime_config_path"] = (root / "config/dual_model_runtime_config.json").string();
    doc["dual_model"]["class_schema_path"] = (root / "config/class_schema_94.json").string();
    YAML::Node hevc(YAML::NodeType::Map);
    hevc["enable"] = true;
    hevc["sink"] = "med2_loopback";
    hevc["rtsp_listen"] = "rtsp://127.0.0.1:8555/rgb";
    hevc["width"] = kWidth;
    hevc["height"] = kHeight;
    hevc["fps"] = 30;
    hevc["bitrate"] = 4000000;
    hevc["codec"] = "nvv4l2h265enc";
    hevc["gop"] = 30;
    doc["hevc"] = hevc;
    yolo::Config config;
    CHECK(config.load_from_node(doc, fixture.string()));
    return config;
}

std::string returns_text(const VelocityReturns& r) {
    return std::to_string(r.person_after_35) + "/" + std::to_string(r.chair_after_41) + "/" +
           std::to_string(r.car_after_48) + "/" + std::to_string(r.person_after_71);
}

void run_case(const yolo::Config& config, const Raster& raster, const char* parallel, double float_margin_m) {
    if (parallel == nullptr) unsetenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    else CHECK(setenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE", parallel, 1) == 0);
    const std::string label = std::string("parallel=") + (parallel ? parallel : "off") +
                              " float_margin=" + std::to_string(float_margin_m);
    // With the depth events, and with steady depth: the pair reference / current each time.
    const ArmResult reference = run_arm(config, raster, Hook::BuildEveryTrack, float_margin_m, true);
    const ArmResult current = run_arm(config, raster, Hook::Service, float_margin_m, true);
    const ArmResult steady_reference = run_arm(config, raster, Hook::BuildEveryTrack, float_margin_m, false);
    const ArmResult steady = run_arm(config, raster, Hook::Service, float_margin_m, false);
    const char* verbose = std::getenv("PERCEPTION_TEST_VERBOSE");
    const std::size_t different = first_difference(reference, current);
    const std::size_t steady_different = first_difference(steady_reference, steady);
    if (different != kNone || steady_different != kNone || (verbose != nullptr && std::string(verbose) == "1")) {
        print_sequence("reference", reference);
        print_sequence("current", current);
        print_sequence("steady_reference", steady_reference);
        print_sequence("steady", steady);
    }
    const auto shown = [](std::size_t index) { return index == kNone ? -1 : static_cast<long>(index) + 1; };
    std::cout << "CASE " << label << " frames=" << current.frames.size()
              << " events: builds=" << reference.build_calls << "/" << current.build_calls
              << " coast_skipped=" << current.coast_skipped << " first_difference=" << shown(different)
              << " velocity_returns=" << returns_text(velocity_returns(current))
              << " | steady: builds=" << steady_reference.build_calls << "/" << steady.build_calls
              << " coast_skipped=" << steady.coast_skipped << " first_difference=" << shown(steady_different)
              << " velocity_returns=" << returns_text(velocity_returns(steady)) << "\n";
    // 1. The property: identical published sequence, body by body, in both scenes.
    CHECK(different == kNone);
    CHECK(steady_different == kNone);
    // 2. The arms of a pair really differ in the work done, by exactly the coasting tracks, and
    //    the point path asked for is the one that ran.
    for (const auto* pair : {&reference, &steady_reference}) CHECK(pair->coast_skipped == 0);
    CHECK(current.coast_skipped_points > 0 && steady.coast_skipped_points > 0);
    CHECK(reference.build_calls == current.build_calls + current.coast_skipped);
    CHECK(steady_reference.build_calls == steady.build_calls + steady.coast_skipped);
    for (const auto* arm : {&reference, &current, &steady_reference, &steady}) {
        CHECK(arm->not_ready == 0 && arm->raster_mismatch == 0 && arm->mask_unusable == 0);
        CHECK(arm->selected_list_rejected == 0);
        CHECK(float_margin_m > 0 ? arm->float_points == arm->build_calls : arm->float_points == 0);
    }
    // Coasting tracks with a valid raw sample, by the scene. With the depth events: person 31, 32,
    // 34 and 50-54 (after 54 the track is no longer in the list); chair 31-37, 39, 40 (33 and 38
    // are the invalid frames: no sample, the hook is not called); car 45-47. With steady depth
    // every coasting frame has a sample: person 31-34 and 50-54, chair 31-40, car 45-47.
    CHECK(current.coast_skipped == 3 + 5 + 9 + 3);
    CHECK(steady.coast_skipped == 4 + 5 + 10 + 3);
    // 3. The scenario happened as described, in all four arms.
    for (const auto* arm : {&reference, &current, &steady_reference, &steady}) check_scenario(*arm);
    // 4. What the coasting frames did to the kept state, in absolute terms.
    //    With the depth events: the jump is held by the hysteresis, and a held sample (like an
    //    invalid one) clears the track's velocity state, so person and chair come back
    //    "warming_up" and need a full window again. With steady depth their windows lived through
    //    the coasting frames: valid at once. The car's depth is steady in both scenes.
    const Object* person_events = find_class(at(current, 35), "person");
    const Object* person_steady = find_class(at(steady, 35), "person");
    const Object* chair_events = find_class(at(current, 41), "chair");
    const Object* chair_steady = find_class(at(steady, 41), "chair");
    const Object* car_events = find_class(at(current, 48), "car");
    const Object* car_steady = find_class(at(steady, 48), "car");
    CHECK(person_events && person_steady && chair_events && chair_steady && car_events && car_steady);
    CHECK(person_events->velocity_status == "warming_up" && person_steady->velocity_status == "static");
    CHECK(chair_events->velocity_status == "warming_up" && chair_steady->velocity_status == "static");
    CHECK(car_events->velocity_valid && car_steady->velocity_valid);
    CHECK(velocity_returns(reference) == velocity_returns(current));
    CHECK(velocity_returns(steady_reference) == velocity_returns(steady));
    CHECK(velocity_returns(current) == kEventReturns);
    CHECK(velocity_returns(steady) == kSteadyReturns);
    // 5. The two scenes publish different bodies from the first re-detection on (frame 35), not
    //    before: the coasting frames reach later bodies through the kept state.
    const std::size_t scene_difference = first_difference(current, steady);
    CHECK(scene_difference != kNone && scene_difference + 1 == 35);
}

// The wire text of the object of this class in one published body: from its class_name to the end
// of its JSON object.
std::string wire_entry(const Published& p, const std::string& class_name) {
    const auto begin = p.wire.find("\"class_name\":\"" + class_name + "\"");
    CHECK(begin != std::string::npos);
    const auto end = p.wire.find('}', begin);
    CHECK(end != std::string::npos);
    return p.wire.substr(begin, end - begin);
}

// A19-NULL-4 (19 section 14; 11 section 3.1B.2) in the real loop: an object that is detected
// while the depth under it is unusable stays in Objects, with footprint_xy, z_min, z_max, r_near
// and velocity_xy all null. The same Supervisor, tracker, sampler, hysteresis, velocity estimator,
// adapter, geometry hook and encoder as the rest of this file; the fourth object of the scene
// (kNoDepthObject) is detected in all 90 frames and has no depth under its box in frames 21-60.
// Asserted on every published body of that interval, on the body and on the wire text; before and
// after the interval the object is located; and the three other objects are published exactly as
// in the same scene without the fourth one (same bits), so keeping an unlocatable object in the
// list costs the others nothing.
// The contract's mutants: the object is dropped from the list (the consumer would see an empty
// scene and drive at full speed), or only r_near is nulled (the consumer refuses the packet).
void run_null4_case(const yolo::Config& config, const Raster& raster) {
    unsetenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    const ArmResult base = run_arm(config, raster, Hook::Service, 0.0, false);
    const ArmResult arm = run_arm(config, raster, Hook::Service, 0.0, false, &kNoDepthObject);
    const char* verbose = std::getenv("PERCEPTION_TEST_VERBOSE");
    if (verbose != nullptr && std::string(verbose) == "1") print_sequence("null4", arm);
    CHECK(!arm.fatal && !base.fatal);
    CHECK(arm.frames.size() == static_cast<std::size_t>(kFrames) && base.frames.size() == arm.frames.size());
    const std::string name = kNoDepthObject.class_name;
    // Located while it has depth, just before the interval.
    const Object* before = find_class(at(arm, kNoDepthFirst - 1), name);
    CHECK(before && before->geometry);
    const auto id = before->track_id;
    std::size_t unlocatable_frames = 0;
    for (int frame = kNoDepthFirst; frame <= kNoDepthLast; ++frame) {
        const auto& p = at(arm, frame);
        const Object* o = find_class(p, name);
        CHECK(o);                                   // kept in the list, not dropped
        CHECK(o->track_id == id);                   // the same object as before, not a new track
        CHECK(!o->geometry && !o->velocity_xy && !o->velocity_valid);
        CHECK(p.report.unlocatable == 1 && p.report.located + 1 == p.report.objects);
        CHECK(p.body.objects.size() == p.report.objects);
        // On the wire: the four geometry fields and velocity_xy are null together.
        const std::string entry = wire_entry(p, name);
        CHECK(entry.find(",\"footprint_xy\":null,\"z_min\":null,\"z_max\":null,\"r_near\":null,\"velocity_xy\":null,") != std::string::npos);
        CHECK(entry.find("\"velocity_valid\":false") != std::string::npos);
        ++unlocatable_frames;
    }
    CHECK(unlocatable_frames == static_cast<std::size_t>(kNoDepthLast - kNoDepthFirst + 1));
    // With depth again: the same object, located again by the end.
    const Object* after = find_class(at(arm, kFrames), name);
    CHECK(after && after->track_id == id && after->geometry);
    CHECK(at(arm, kFrames).report.unlocatable == 0);
    // The other three objects: bit for bit what they are without the fourth one, in every frame.
    for (int frame = 1; frame <= kFrames; ++frame) {
        for (const char* other : {"person", "chair", "car"}) {
            const Object* a = find_class(at(base, frame), other);
            const Object* b = find_class(at(arm, frame), other);
            CHECK((a != nullptr) == (b != nullptr));
            if (a != nullptr) CHECK(same_object(*a, *b));
        }
        CHECK(at(arm, frame).body.objects.size() >= at(base, frame).body.objects.size());
    }
    std::cout << "CASE A19-NULL-4 detected without depth: frames " << kNoDepthFirst << "-" << kNoDepthLast
              << " published unlocatable (" << unlocatable_frames << " bodies), located before and after; "
              << "the other objects unchanged\n";
}

// One whole run of all cases in this process, on the depth scan the environment selects.
void run_all(const std::filesystem::path& root) {
    const yolo::Config config = load_pipeline_config(root);
    CHECK(config.camera_driver() == "orbbec_gemini338le");
    CHECK(config.camera_output_width() == kWidth && config.camera_output_height() == kHeight);
    Raster raster;
    raster.registration = std::make_shared<const RgbDepthRegistration>(
        kWidth, kHeight, kColour.fx, kColour.fy, kColour.cx, kColour.cy,
        StandardDistortion{-0.03, 0.03, -0.01, 0, 0});
    raster.colour = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    // Serial sampling and the two-lane sampler in both splits (fixed "1", shared "2"); exact and
    // float geometry points.
    for (const char* parallel : {static_cast<const char*>(nullptr), "1", "2"})
        for (const double float_margin_m : {0.0, 0.001}) run_case(config, raster, parallel, float_margin_m);
    unsetenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    run_null4_case(config, raster);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        yolo::Logger::instance().set_level(yolo::LogLevel::ERROR);
        const auto root = std::filesystem::absolute(std::filesystem::path(__FILE__)).parent_path().parent_path();
        // The depth scan is chosen once per process (PERCEPTION_DEPTH_SCAN_FAST). This process
        // runs the production one, the fast scan; then a child process repeats every case on the
        // reference scan. The witness counter shows which one really ran.
        const bool child = argc > 1 && std::string(argv[1]) == "--reference-scan";
        CHECK(setenv("PERCEPTION_DEPTH_SCAN_FAST", child ? "0" : "1", 1) == 0);
        run_all(root);
        const std::uint64_t fast_samples = depth_scan_fast_entries();
        CHECK(child ? fast_samples == 0 : fast_samples > 0);
        if (!child) {
            // No thread of ours is alive here: every Supervisor and its sampling helper are gone.
            std::cout.flush();
            const pid_t pid = fork();
            CHECK(pid >= 0);
            if (pid == 0) {
                execl(argv[0], argv[0], "--reference-scan", static_cast<char*>(nullptr));
                _exit(127);
            }
            int status = 0;
            CHECK(waitpid(pid, &status, 0) == pid);
            CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
        }
        std::cout << "{\"status\":\"PASS\",\"checks\":" << checks << ",\"depth_scan\":\""
                  << (child ? "reference" : "fast") << "\",\"fast_scan_samples\":" << fast_samples
                  << ",\"real_process_loop\":true,\"real_hook\":true,\"device\":false,"
                     "\"model_execution\":false,\"network\":false}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
