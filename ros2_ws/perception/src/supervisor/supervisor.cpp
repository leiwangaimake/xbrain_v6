/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: supervisor.cpp
 * Brief: R6 perception pipeline supervisor (capture, inference, tracking, fusion, Objects).
 *
 * Description:
 *   1. Starts and stops the pipeline modules in a controlled order: RGB-D capture (with the
 *      native depth tap for the Profile mailbox), MED-2 HEVC encoder, inference, tracker.
 *   2. process_loop (sup_process thread): per inference result, checks the R6 frame identity
 *      and capture stamp, tracks, samples depth, fuses, builds the measured object geometry
 *      and hands the Objects body to the service (SupervisorRnsContext::objects_ready).
 *   3. heartbeat_loop (sup_heartbeat thread): stops the service when the Gemini hard contract
 *      no longer holds (RGB-D capture, inference or the HEVC encoder no longer running, or the
 *      encoder failed).
 *   4. handle_capture_command: the evidence-capture command, on the ROS executor thread.
 * Since the level-2 trim it runs only with R6 wiring (configure_rns before init); the legacy
 * V5 outputs (Zenoh publisher, health monitor, visualization, RTSP forward, PTZ) are gone.
 */

#include "common/thread_name.hpp"
#include "supervisor/supervisor.hpp"

#include "weak_obstacle_policy.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <future>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/2d.hpp>
#endif

#include "common/logger.hpp"
#include "common/thread_sched_probe.hpp"
#include "common/time_utils.hpp"
#include "common/timer.hpp"
#include "modules/rgbd_capture_factory.hpp"
#ifdef HAS_ORBBEC
#include "modules/orbbec_capture.hpp"
#include "modules/rgb_depth_registration.hpp"
#endif
#include "supervisor/fixed_two_lane_depth_sampler.hpp"

namespace perception {

namespace {

constexpr const char* MODULE_NAME = "Supervisor";
constexpr std::size_t ENCODE_QUEUE_SIZE = 3;
constexpr auto QUEUE_POP_TIMEOUT = std::chrono::milliseconds(100);
constexpr auto HEARTBEAT_INTERVAL = std::chrono::seconds(1);
constexpr auto THREAD_JOIN_TIMEOUT = std::chrono::milliseconds(500);
constexpr auto HEARTBEAT_SLEEP_SLICE = std::chrono::milliseconds(100);

std::string resolve_relative_to_config(const std::string& config_path, const std::string& path) {
    if (path.empty() || path.front() == '/') {
        return path;
    }

    const std::string::size_type pos = config_path.rfind('/');
    if (pos == std::string::npos) {
        return path;
    }

    return config_path.substr(0, pos + 1) + path;
}

std::string format_stamp(double stamp_sec) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(6);
    out << stamp_sec;
    return out.str();
}

// --- T11 capture worker helpers (V5 evidence-capture command, see supervisor.hpp) ---
// Minimal string-value extractor for the small, controlled capture command
// schema. perception builds/parses JSON by hand (no nlohmann dep), so this only
// handles "key":"value" with simple escaping. Returns "" if the key is absent.
std::string json_string_field(const std::string& json, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    std::size_t pos = json.find(needle);
    if (pos == std::string::npos) {
        return "";
    }
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) {
        return "";
    }
    ++pos;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) {
        ++pos;
    }
    if (pos >= json.size() || json[pos] != '"') {
        return "";  // not a string value (null / number) -> treat as absent
    }
    ++pos;
    std::string value;
    while (pos < json.size() && json[pos] != '"') {
        if (json[pos] == '\\' && pos + 1 < json.size()) {
            ++pos;  // unescape one char (best-effort)
        }
        value.push_back(json[pos]);
        ++pos;
    }
    return value;
}

std::string json_escape(const std::string& input) {
    std::string out;
    out.reserve(input.size());
    for (const char ch : input) {
        if (ch == '"' || ch == '\\') {
            out.push_back('\\');
        }
        out.push_back(ch);
    }
    return out;
}

std::string to_iso8601_utc(double sec) {
    const auto whole = static_cast<std::time_t>(sec);
    std::tm tm_utc{};
    gmtime_r(&whole, &tm_utc);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_utc);
    const int ms = static_cast<int>((sec - std::floor(sec)) * 1000.0);
    std::ostringstream out;
    out << buf << "." << std::setw(3) << std::setfill('0') << std::max(0, std::min(999, ms)) << "Z";
    return out.str();
}

// Evidence filename timestamp suffix YY-MM-DD_HH-MM-SS (V5 capture rule).
// Uses LOCAL time (robot wall clock) on purpose -- the field tech matches the
// on-site clock, not UTC (the JSON receipt keeps captured_at in UTC separately).
// R6 sets no TZ of its own (the systemd drop-in has none), so localtime_r follows
// the host's time zone (/etc/localtime); V5's init_timezone does not exist here. Hyphen-only
// (no colon) so the name stays portable when evidence is scp/rsync'd to a
// Windows/SMB/exFAT target, where ':' is an illegal filename char.
std::string local_timestamp_for_filename(double sec) {
    const auto whole = static_cast<std::time_t>(sec);
    std::tm tm_local{};
    localtime_r(&whole, &tm_local);
    char buf[24] = {0};
    std::strftime(buf, sizeof(buf), "%y-%m-%d_%H-%M-%S", &tm_local);
    return std::string(buf);
}

// Parse "YYYY-MM-DD HH:MM:SS[.mmm]" LOCAL wall-clock time (the form xbrain emits in
// image_time_hint) to epoch seconds; returns <0 on failure
// so the caller falls back to the newest buffered frame.
double parse_image_time_hint_to_epoch(const std::string& hint) {
    if (hint.empty()) {
        return -1.0;
    }
    std::tm tm_local{};
    std::istringstream ss(hint);
    ss >> std::get_time(&tm_local, "%Y-%m-%d %H:%M:%S");
    if (ss.fail()) {
        return -1.0;
    }
    tm_local.tm_isdst = -1;  // let mktime resolve DST for the local zone
    const std::time_t secs = std::mktime(&tm_local);
    if (secs == static_cast<std::time_t>(-1)) {
        return -1.0;
    }
    double epoch = static_cast<double>(secs);
    const std::size_t dot = hint.find('.');
    if (dot != std::string::npos) {
        epoch += std::atof(hint.c_str() + dot);  // ".123" -> 0.123
    }
    return epoch;
}

std::string build_capture_done(
    const std::string& event_uuid,
    bool success,
    const std::string& file_path,
    std::size_t bytes,
    int width,
    int height,
    double captured_at_sec,
    float frame_age_ms,
    float encoding_ms,
    const std::string& error_message,
    const std::vector<std::string>& pruned_paths = {}) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"event_uuid\":\"" << json_escape(event_uuid) << "\",";
    out << "\"success\":" << (success ? "true" : "false") << ",";
    out << "\"file_path\":\"" << json_escape(file_path) << "\",";
    out << "\"image_size_bytes\":" << bytes << ",";
    out << "\"image_w\":" << width << ",";
    out << "\"image_h\":" << height << ",";
    out << "\"captured_at\":\"" << to_iso8601_utc(captured_at_sec) << "\",";
    out << "\"frame_age_ms\":" << frame_age_ms << ",";
    out << "\"encoding_ms\":" << encoding_ms << ",";
    if (error_message.empty()) {
        out << "\"error_message\":null";
    } else {
        out << "\"error_message\":\"" << json_escape(error_message) << "\"";
    }
    // Evidence files the LRU quota just deleted; xbrain NULLs their image_path.
    out << ",\"pruned_paths\":[";
    for (std::size_t i = 0; i < pruned_paths.size(); ++i) {
        out << (i != 0U ? "," : "") << "\"" << json_escape(pruned_paths[i]) << "\"";
    }
    out << "]";
    out << "}";
    return out.str();
}

// Atomic JPEG write: <final>.tmp -> fsync -> rename. Returns "" on success or the
// error token "write_failed" (no separate token for quota problems yet).
std::string write_atomic_jpeg(
    const std::string& final_path,
    const std::vector<unsigned char>& bytes) {
    std::error_code ec;
    const std::filesystem::path final_fs(final_path);
    if (final_fs.has_parent_path()) {
        std::filesystem::create_directories(final_fs.parent_path(), ec);
    }
    const std::string tmp_path = final_path + ".tmp";
    const int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        return "write_failed";
    }
    std::size_t written = 0;
    while (written < bytes.size()) {
        const ssize_t n = ::write(fd, bytes.data() + written, bytes.size() - written);
        if (n <= 0) {
            ::close(fd);
            return "write_failed";
        }
        written += static_cast<std::size_t>(n);
    }
    // fsync is required: a power loss after rename but before flush would lose the
    // evidence (the capture is not fire-and-forget).
    ::fsync(fd);
    ::close(fd);
    std::filesystem::rename(tmp_path, final_path, ec);
    if (ec) {
        return "write_failed";
    }
    return "";
}

// B4-2: with depth registered onto the (distorted) RGB image the sampler's
// pinhole back-projection of the median RGB pixel is off by the lens
// distortion. Recover that pixel from the pinhole centroid (float round-off
// ~1e-4 px) and back-project it through the registration's undistortion.
// Without a registration the sample is returned unchanged.
DepthSample undistort_centroid(DepthSample sample, const yolo::DepthIntrinsics& k,
                              const RgbDepthRegistration* registration) {
    if (!registration || !sample.valid) return sample;
    if (!undistort_pinhole_point(*registration, k.fx, k.fy, k.cx, k.cy, sample.centroid_3d[0], sample.centroid_3d[1],
                               sample.centroid_3d[2])) {
        throw std::runtime_error("DEPTH_CENTROID_UNDISTORT_FAILED");
    }
    return sample;
}

}  // namespace

Supervisor::Supervisor() = default;

Supervisor::~Supervisor() {
    stop();
}

bool Supervisor::configure_rns(rns::SupervisorRnsContext context) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (init_attempted_ || initialized_ || running_.load() || rns_context_ ||
        !context.valid()) return false;
    rns_context_ = std::move(context);
    return true;
}

// Builds the module graph once: R6 wiring checks (Gemini 338Le, SW-D2C, depth 640x400, output
// 1280x800, dual model, frame-sync SEI build), the three queues (capture->inference,
// capture->encoder, the inference->Supervisor result slot), tracker and depth samplers, the
// capture, encoder and inference modules, then the velocity estimators. Nothing runs until
// start().
bool Supervisor::init(const yolo::Config& config) {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    // The Supervisor serves only the R6 service: its sole outputs are the R6
    // typed callbacks, so it refuses to start without that wiring.
    if (!rns_context_) {
        LOG_ERROR_M(MODULE_NAME, "Supervisor requires R6 wiring (configureRns) before init");
        return false;
    }
    if (init_attempted_) {
        LOG_ERROR_M(MODULE_NAME, "R6 owner cannot be reinitialized; recreate its source/track lifecycle");
        return false;
    }
    if (running_.load()) {
        LOG_WARN_M(MODULE_NAME, "init ignored while supervisor is running");
        return false;
    }
    init_attempted_ = true;
    stopped_ = false;

    // A failed re-initialization must never leave start() able to consume a
    // partially replaced module graph.
    initialized_ = false;
    fatal_failure_detected_.store(false);

    resolved_config_ = config;
    resolved_config_.set_model_path(resolve_relative_to_config(config.path(), config.model_path()));
    resolved_config_.set_trt_engine_path(resolve_relative_to_config(config.path(), config.trt_engine_path()));
    config_ = &resolved_config_;
    if (rns_context_) {
        if (!rns_context_->valid() || config_->camera_driver() != "orbbec_gemini338le" ||
            config_->camera_alignment_mode() != "sw_d2c" || config_->camera_depth_width() != 640 ||
            config_->camera_depth_height() != 400 || config_->camera_output_width() != 1280 ||
            config_->camera_output_height() != 800 || !config_->dual_model_enabled()) {
            LOG_ERROR_M(MODULE_NAME, "R6 requires explicit valid native800 SW-D2C dual-model owner wiring");
            return false;
        }
        rns_objects_ = std::make_unique<rns::FusedObjectsAdapter>(rns_context_->clock_epoch);
        rns_objects_->set_extrinsic_calibrated(rns_context_->extrinsic_calibrated);
    }

#ifndef PERCEPTION_FRAME_SYNC_SEI
    if (config_->camera_driver() == "orbbec_gemini338le") {
        LOG_ERROR_M(
            MODULE_NAME,
            "Gemini hard contract requires a production build with frame-sync SEI enabled");
        return false;
    }
#endif

    // T11 capture: ring buffer depth + evidence-dir LRU quota from config.
    frame_buffer_capacity_ = static_cast<std::size_t>(std::max(1, config.capture_frame_buffer_size()));
    evidence_quota_bytes_ =
        static_cast<std::uint64_t>(std::max(0, config.evidence_quota_mb())) * 1024ULL * 1024ULL;

    const std::size_t frame_queue_size = std::max<std::size_t>(1, config_->frame_queue_size());
    const std::size_t detection_queue_size = std::max<std::size_t>(1, config_->result_queue_size());
    frame_queue_ = std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(frame_queue_size);
    encode_queue_ = std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(ENCODE_QUEUE_SIZE);
    // 19 P19-2: one result slot is a latest-wins pointer swap, never a queue.
    const yolo::QueuePolicy result_policy = detection_queue_size == 1
        ? yolo::QueuePolicy::kLatestWinsSwap : yolo::QueuePolicy::kDropOldestFifo;
    detection_queue_ =
        std::make_shared<yolo::ThreadsafeQueue<yolo::DetectionResult>>(detection_queue_size, result_policy);
    detection_queue_->set_discard_observer([this](const yolo::DetectionResult& r, yolo::QueueDiscard why) {
        result_ledger_.on_discarded(r.frame_id, why == yolo::QueueDiscard::kCleared);
    });
    LOG_INFO_M(MODULE_NAME, "RESULT_HANDOFF_CONFIG policy="
        << (result_policy == yolo::QueuePolicy::kLatestWinsSwap ? "latest_wins_swap" : "drop_oldest_fifo")
        << " capacity=" << detection_queue_size << " input_capacity=" << frame_queue_size);

    odom_buffer_ = std::make_unique<OdomBuffer>(*config_);
    tracker_ = std::make_unique<ByteTracker>(*config_);
    depth_sampler_.set_params(DepthSampler::Params{
        config_->depth_sample_mode(),
        config_->depth_percentile(),
        config_->depth_cluster_width_m(),
        config_->depth_cluster_min_fraction(),
        config_->depth_cluster_min_pixels(),
        config_->depth_nearest_cluster_output_percentile()});
    // Read the sort implementation once, before the workers and per-class samplers start (no
    // environment access per sample). Production sets codesort through the runtime switch
    // depth_sort_impl; unset means stdsort. The "probe" wording in the log line is historical.
    // The non-Gemini branch is unreachable under R6 (the driver is checked above).
    const char* depth_sort_value = std::getenv("PERCEPTION_DEPTH_SORT_IMPL");
    const std::string depth_sort_name =
        depth_sort_value == nullptr ? "stdsort" : depth_sort_value;
    if (config_->camera_driver() == "orbbec_gemini338le") {
        auto params = depth_sampler_.params();
        if (depth_sort_name == "stdsort") {
            params.sort_impl = DepthSampler::SortImplementation::StdSort;
        } else if (depth_sort_name == "codesort") {
            params.sort_impl = DepthSampler::SortImplementation::CodeSort;
        } else if (depth_sort_name == "shadow") {
            params.sort_impl = DepthSampler::SortImplementation::Shadow;
        } else {
            LOG_ERROR_M(MODULE_NAME,
                "invalid PERCEPTION_DEPTH_SORT_IMPL=" << depth_sort_name);
            return false;
        }
        depth_sampler_.set_params(params);
        LOG_INFO_M(MODULE_NAME, "DEPTH_SORT_CONFIG implementation=" << depth_sort_name
            << " scope=performance_probe_only default=stdsort");
    } else if (depth_sort_value != nullptr && depth_sort_name != "stdsort") {
        LOG_WARN_M(MODULE_NAME, "ignoring PERCEPTION_DEPTH_SORT_IMPL for non-Gemini camera");
    }
    per_class_depth_samplers_.clear();
    const DepthSampler::Params default_depth_params = depth_sampler_.params();
    for (const auto& [class_name, sample_mode] : config_->depth_per_class_sample_modes()) {
        DepthSampler::Params class_params = default_depth_params;
        class_params.sample_mode = sample_mode;
        per_class_depth_samplers_.emplace(class_name, DepthSampler(class_params));
        LOG_INFO_M(
            MODULE_NAME,
            "depth sample override class=" << class_name << " mode=" << sample_mode);
    }
    depth_hysteresis_.set_params(TrackDepthHysteresis::Params{
        config_->depth_jump_threshold_m(),
        config_->depth_jump_candidate_tolerance_m(),
        config_->depth_jump_confirmation_frames()});
    // P3 camera-to-base placeholder reuses configured extrinsics; recalibrate on robot.
    // It is the config's base<-lidar extrinsic; the R6 service refuses that retired section, so it
    // is always the identity. It only feeds the odom path, which is idle while TF is refused
    // (tf_enable, B4-1). Before that path is enabled this must come from the base<-colour chain
    // (19 section 4.2).
    extrinsic_camera_to_base_ = extrinsic_camera_to_base_from_config(*config_);
    std::string capture_factory_error;
#ifdef PERCEPTION_TESTING
    if (!use_injected_rgbd_capture_) {
        rgbd_capture_ = create_rgbd_capture(
            *config_, frame_queue_, encode_queue_, capture_factory_error,
            rns_context_->native_mailbox, rns_context_->depth_arrivals.get());
    } else if (!rgbd_capture_) {
        capture_factory_error = "injected RGB-D capture is null";
    }
#else
    rgbd_capture_ = create_rgbd_capture(
        *config_, frame_queue_, encode_queue_, capture_factory_error,
        rns_context_->native_mailbox, rns_context_->depth_arrivals.get());
#endif
    if (!rgbd_capture_) {
        LOG_ERROR_M(MODULE_NAME, "RGB-D capture initialization failed: " << capture_factory_error);
        return false;
    }
    hevc_encoder_ = std::make_unique<HevcEncoder>(*config_, encode_queue_);
    inference_ = std::make_unique<yolo::InferenceModule>(*config_, frame_queue_, detection_queue_,
        rns_context_->inference_completions.get(),
        yolo::InferenceModule::ModelLoadPolicy::ExistingEnginesOnly);

    odom_velocity_estimator_.set_motion_decision_axes(MotionDecisionAxes::XY);
    lidar_velocity_estimator_.set_motion_decision_axes(MotionDecisionAxes::XZ);
    lidar_velocity_estimator_.set_lateral_image_evidence_gate_enabled(true);
    lidar_velocity_estimator_.set_camera_z_uncertainty_gate_config(
        CameraZUncertaintyGateConfig{
            config_->velocity_camera_z_uncertainty_gate_enabled(),
            config_->velocity_camera_z_uncertainty_stddev_multiplier(),
            config_->velocity_camera_z_uncertainty_max_threshold_m()});
    odom_velocity_estimator_.set_dt_max_s(config_->velocity_dt_max_s());
    lidar_velocity_estimator_.set_dt_max_s(config_->velocity_dt_max_s());
    odom_velocity_estimator_.set_centroid_ema_alpha(config_->velocity_centroid_ema_alpha());
    lidar_velocity_estimator_.set_centroid_ema_alpha(config_->velocity_centroid_ema_alpha());
    odom_velocity_estimator_.set_per_class_centroid_ema_alphas(
        config_->velocity_per_class_centroid_ema_alphas());
    lidar_velocity_estimator_.set_per_class_centroid_ema_alphas(
        config_->velocity_per_class_centroid_ema_alphas());
    odom_velocity_estimator_.set_velocity_window_s(config_->velocity_window_s());
    lidar_velocity_estimator_.set_velocity_window_s(config_->velocity_window_s());
    odom_velocity_estimator_.set_motion_confirmation_windows(
        config_->velocity_motion_confirmation_windows());
    lidar_velocity_estimator_.set_motion_confirmation_windows(
        config_->velocity_motion_confirmation_windows());
    odom_velocity_estimator_.set_static_displacement_m(config_->velocity_static_displacement_m());
    lidar_velocity_estimator_.set_static_displacement_m(config_->velocity_static_displacement_m());
    odom_velocity_estimator_.set_generic_static_displacement_m(
        config_->velocity_generic_static_displacement_m());
    lidar_velocity_estimator_.set_generic_static_displacement_m(
        config_->velocity_generic_static_displacement_m());
    odom_velocity_estimator_.set_per_class_static_displacements_m(
        config_->velocity_per_class_static_displacements_m());
    lidar_velocity_estimator_.set_per_class_static_displacements_m(
        config_->velocity_per_class_static_displacements_m());
    const VelocityGatingConfig velocity_gating{
        config_->velocity_gating_min_stable_frames(),
        config_->velocity_gating_min_depth_confidence(),
        config_->velocity_gating_skip_fusion_quality()};
    odom_velocity_estimator_.set_gating_config(velocity_gating);
    lidar_velocity_estimator_.set_gating_config(velocity_gating);
    has_last_velocity_frame_ = false;
    last_velocity_frame_has_odom_ = false;

    LOG_INFO_M(MODULE_NAME, "R6 typed final-output callback enabled; legacy ROS/Zenoh metadata disabled");

    LOG_INFO_M(MODULE_NAME,
               "initialized with frame_queue=" << frame_queue_size
                                                 << ", detection_queue=" << detection_queue_size
                                                 << " (drop-oldest)");
    initialized_ = true;
    return true;
}

// Starts consumers before the producer: inference, then the HEVC encoder, then RGB-D capture,
// so no frame is captured before something can take it; then the process and heartbeat
// threads. Any module failure stops everything already started.
bool Supervisor::start() {
    std::unique_lock<std::mutex> lock(lifecycle_mutex_);
    if (rns_context_ && rns_start_attempted_) {
        LOG_ERROR_M(MODULE_NAME, "R6 owner start/restart already consumed");
        return false;
    }
    if (running_.exchange(true)) {
        return true;
    }

    if (!initialized_ || !config_ || !frame_queue_ || !detection_queue_ ||
        !rgbd_capture_ || !inference_) {
        running_.store(false);
        LOG_ERROR_M(MODULE_NAME, "start called before init");
        return false;
    }
    stopped_ = false;
    if (rns_context_) {
        if (!rns_context_->valid() || !rns_objects_ || !rns_objects_->valid()) {
            running_.store(false);
            LOG_ERROR_M(MODULE_NAME, "R6 source owner invalid before start");
            return false;
        }
        rns_start_attempted_ = true;
    }

    {
        std::lock_guard<std::mutex> lock(run_mutex_);
        shutdown_requested_ = false;
    }

    bool ok = true;
    ok = ok && start_module(inference_.get(), "InferenceModule");
    if (config_->hevc_enabled() && hevc_encoder_ && !hevc_encoder_->start()) {
        if (config_->camera_driver() == "orbbec_gemini338le") {
            LOG_ERROR_M(
                MODULE_NAME,
                "HevcEncoder unavailable; Gemini hard contract requires configured native video output");
            ok = false;
        } else {
            // Preserve the accepted D435 degradation behavior exactly.
            // (Unreachable under R6: init() admits only the Gemini 338Le.)
            LOG_WARN_M(
                MODULE_NAME,
                "HevcEncoder unavailable; D435 inference continues without video output");
        }
    }
    const std::string capture_module_name = rgbd_capture_->name();
    ok = ok && start_module(rgbd_capture_.get(), capture_module_name.c_str());
    if (!ok) {
        LOG_ERROR_M(MODULE_NAME, "module startup failed; stopping supervisor");
        lock.unlock();
        stop();
        return false;
    }

    process_thread_ = std::thread(&Supervisor::process_loop, this);
    heartbeat_thread_ = std::thread(&Supervisor::heartbeat_loop, this);
    LOG_INFO_M(MODULE_NAME, "started");
    return true;
}

// Order: wake run(), clear the queues (wakes blocked pops), join the process and heartbeat
// threads, stop inference, clear the result slot again (inference may have pushed until its
// stop), then stop capture and the encoder. Idempotent; a second call only wakes run().
void Supervisor::stop() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!running_.exchange(false)) {
        request_shutdown();
        return;
    }

    request_shutdown();
    if (frame_queue_) {
        frame_queue_->clear();
    }
    if (encode_queue_) {
        encode_queue_->clear();
    }
    if (detection_queue_) {
        detection_queue_->clear();
    }

    wait_for_thread_exit(process_thread_, "process_thread", THREAD_JOIN_TIMEOUT);
    wait_for_thread_exit(heartbeat_thread_, "heartbeat_thread", THREAD_JOIN_TIMEOUT);

    if (inference_) {
        inference_->stop();
    }
    // The first clear only woke the consumer; inference may have pushed until
    // its stop. Discard (and account by frame_id) what nobody will consume.
    if (detection_queue_) {
        detection_queue_->clear();
    }
    if (rgbd_capture_) {
        rgbd_capture_->stop();
    }
    if (hevc_encoder_) {
        hevc_encoder_->stop();
    }
    odom_velocity_estimator_.reset();
    lidar_velocity_estimator_.reset();
    depth_hysteresis_.reset();
    has_last_velocity_frame_ = false;
    last_velocity_frame_has_odom_ = false;
    last_publish_ms_ = 0.0F;
    stopped_ = true;

    LOG_INFO_M(MODULE_NAME, "stopped");
}

void Supervisor::run() {
    std::unique_lock<std::mutex> lock(run_mutex_);
    run_cv_.wait(lock, [this]() { return shutdown_requested_; });
}

void Supervisor::request_shutdown() {
    {
        std::lock_guard<std::mutex> lock(run_mutex_);
        shutdown_requested_ = true;
    }
    run_cv_.notify_all();
}

bool Supervisor::shutdown_requested() const {
    std::lock_guard<std::mutex> lock(run_mutex_);
    return shutdown_requested_;
}

bool Supervisor::fatal_failure_detected() const {
    return fatal_failure_detected_.load();
}

namespace {
template <typename T>
QueueAccounting queue_accounting(const std::shared_ptr<yolo::ThreadsafeQueue<T>>& q) {
    QueueAccounting out;
    if (!q) return out;
    out.pushed = q->pushed_count();
    out.popped = q->popped_count();
    out.dropped = q->dropped_count();
    out.cleared = q->cleared_count();
    out.pending = q->size();
    return out;
}
}  // namespace

ResultHandoffSnapshot Supervisor::drain_result_handoff() {
    ResultHandoffSnapshot out;
    if (!frame_queue_ || !detection_queue_) return out;
    out.available = true;
    out.result_latest_wins = detection_queue_->policy() == yolo::QueuePolicy::kLatestWinsSwap;
    out.ledger = result_ledger_.drain();
    out.input = queue_accounting(frame_queue_);
    out.result = queue_accounting(detection_queue_);
    return out;
}

PipelineRuntimeAudit Supervisor::runtime_audit() const {
    PipelineRuntimeAudit out;
    std::unique_lock<std::mutex> lock(lifecycle_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return out;
    if (!stopped_ || running_.load() || process_thread_.joinable() || heartbeat_thread_.joinable()) return out;
    out.available = true;
    out.camera_running = rgbd_capture_ && rgbd_capture_->is_running();
    out.inference_running = inference_ && inference_->is_running();
    out.input_dropped = frame_queue_ ? frame_queue_->dropped_count() : 0;
    out.result_dropped = detection_queue_ ? detection_queue_->dropped_count() : 0;
    out.encode_dropped = encode_queue_ ? encode_queue_->dropped_count() : 0;
    out.encoded_frames = hevc_encoder_ ? hevc_encoder_->encoded_frames() : 0;
    out.rns_prepared = rns_prepared_; out.rns_callback_attempts = rns_callback_attempts_;
    out.rns_callback_successes = rns_callback_successes_;
    if (rns_context_) {
        out.depth_arrivals = rns_context_->depth_arrivals->published();
        out.inference_completions = rns_context_->inference_completions->published();
    }
#ifdef HAS_ORBBEC
    if (const auto* camera = dynamic_cast<const OrbbecCapture*>(rgbd_capture_.get())) {
        out.sdk_stop_available = true;
        out.sdk_stop_attempts = camera->sdk_stop_attempts();
        out.sdk_stop_successes = camera->sdk_stop_successes();
        out.sdk_stop_failures = camera->sdk_stop_failures();
    }
#endif
    return out;
}
bool Supervisor::jit_calibration_complete() const noexcept {
    return inference_ && inference_->jit_calibration_complete();
}
std::int64_t Supervisor::jit_frozen_delay_us() const noexcept {
    return inference_ ? inference_->jit_frozen_delay_us() : -1;
}

std::string Supervisor::handle_capture_command(const std::string& cmd_json) {
    // T11 (V5 capture command). Picks the buffered frame nearest image_time_hint
    // (newest if hint empty/unparsable) and LRU-prunes the evidence dir to the quota
    // after writing. Still CPU/cv::Mat (no GPU ring buffer / libjpeg-turbo) -- fine
    // at 10Hz; a standalone perception.evidence_pruned event topic is deferred,
    // pruned paths ride back on capture_done.pruned_paths instead.
    const std::string event_uuid = json_string_field(cmd_json, "event_uuid");
    std::string save_dir = json_string_field(cmd_json, "save_dir");
    if (save_dir.empty()) {
        save_dir = "data/evidence/";
    }
    if (save_dir.back() != '/') {
        save_dir.push_back('/');
    }

    const std::string image_time_hint = json_string_field(cmd_json, "image_time_hint");
    double frame_stamp_sec = 0.0;
    const cv::Mat frame = select_capture_frame(image_time_hint, frame_stamp_sec);

    const double captured_at_sec = system_now_sec();
    if (frame.empty()) {
        LOG_WARN_M(MODULE_NAME, "capture command but no decoded frame yet, event_uuid=" << event_uuid);
        return build_capture_done(event_uuid, false, "", 0U, 0, 0, captured_at_sec, 0.0F, 0.0F, "no_frame_available");
    }

    yolo::Timer encode_timer;
    encode_timer.start();
    std::vector<unsigned char> jpeg;
    bool encoded = false;
    try {
        const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 75};
        encoded = cv::imencode(".jpg", frame, jpeg, params);
    } catch (const std::exception& ex) {
        LOG_ERROR_M(MODULE_NAME, "capture jpeg encode failed: " << ex.what());
        encoded = false;
    }
    encode_timer.stop();
    const auto encoding_ms = static_cast<float>(encode_timer.elapsed_ms());
    if (!encoded || jpeg.empty()) {
        return build_capture_done(
            event_uuid, false, "", 0U, frame.cols, frame.rows, captured_at_sec, 0.0F, encoding_ms, "encode_failed");
    }

    // V5 capture rule: name is <event_uuid>_<YY-MM-DD_HH-MM-SS>.jpg. The
    // local-time suffix de-duplicates even when event_uuid is empty (fallback
    // "capture") so repeated unkeyed captures no longer overwrite each other.
    // event_uuid SHOULD be non-empty (xbrain passes events.id-trace_id[:8]).
    const std::string name_base = event_uuid.empty() ? "capture" : event_uuid;
    const std::string final_path =
        save_dir + name_base + "_" + local_timestamp_for_filename(captured_at_sec) + ".jpg";
    const std::string write_err = write_atomic_jpeg(final_path, jpeg);
    const float frame_age_ms =
        frame_stamp_sec > 0.0 ? static_cast<float>((captured_at_sec - frame_stamp_sec) * 1000.0) : 0.0F;
    if (!write_err.empty()) {
        return build_capture_done(
            event_uuid, false, "", jpeg.size(), frame.cols, frame.rows, captured_at_sec, frame_age_ms, encoding_ms,
            write_err);
    }

    // Evidence-dir LRU: prune oldest jpgs back under the quota AFTER this write (so
    // the frame we just saved isn't the one deleted). Deleted paths ride back on
    // capture_done so xbrain can NULL the matching events.image_path.
    const std::vector<std::string> pruned = enforce_evidence_quota(save_dir);
    LOG_INFO_M(
        MODULE_NAME,
        "capture saved event_uuid=" << event_uuid << " path=" << final_path << " bytes=" << jpeg.size()
                                    << " age_ms=" << frame_age_ms << " encode_ms=" << encoding_ms
                                    << " pruned=" << pruned.size());
    return build_capture_done(
        event_uuid, true, final_path, jpeg.size(), frame.cols, frame.rows, captured_at_sec, frame_age_ms, encoding_ms,
        "", pruned);
}

// No maximum matching error and no age limit: a hint outside the ring selects its nearest end
// (oldest or newest frame) and the receipt still reports success. frame_age_ms in the receipt is
// the command time minus the chosen frame's stamp, not the hint matching error.
cv::Mat Supervisor::select_capture_frame(const std::string& image_time_hint, double& out_stamp_sec) {
    std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
    if (frame_buffer_.empty()) {
        out_stamp_sec = 0.0;
        return cv::Mat{};
    }
    const double hint_epoch = parse_image_time_hint_to_epoch(image_time_hint);
    if (hint_epoch < 0.0) {
        // no / unparsable hint -> newest frame (the previous behaviour)
        out_stamp_sec = frame_buffer_.back().stamp_sec;
        return frame_buffer_.back().frame;
    }
    // pick the buffered frame whose stamp is closest to the hint
    const FrameBufferEntry* best = &frame_buffer_.back();
    double best_diff = std::fabs(best->stamp_sec - hint_epoch);
    for (const auto& entry : frame_buffer_) {
        const double diff = std::fabs(entry.stamp_sec - hint_epoch);
        if (diff < best_diff) {
            best_diff = diff;
            best = &entry;
        }
    }
    out_stamp_sec = best->stamp_sec;
    return best->frame;
}

std::vector<std::string> Supervisor::enforce_evidence_quota(const std::string& save_dir) const {
    std::vector<std::string> pruned;
    if (evidence_quota_bytes_ == 0U) {
        return pruned;  // quota disabled
    }
    namespace fs = std::filesystem;
    std::error_code ec;
    struct Item {
        fs::path path;
        std::uintmax_t size;
        fs::file_time_type mtime;
    };
    std::vector<Item> items;
    std::uintmax_t total = 0;
    for (const auto& entry : fs::directory_iterator(save_dir, ec)) {
        if (ec) {
            break;
        }
        std::error_code item_ec;
        if (!entry.is_regular_file(item_ec) || entry.path().extension() != ".jpg") {
            continue;
        }
        const std::uintmax_t sz = entry.file_size(item_ec);
        if (item_ec) {
            continue;
        }
        const fs::file_time_type mtime = fs::last_write_time(entry.path(), item_ec);
        if (item_ec) {
            continue;  // unknown mtime -> can't rank it for LRU safely; skip (keep it)
        }
        items.push_back({entry.path(), sz, mtime});
        total += sz;
    }
    if (total <= evidence_quota_bytes_) {
        return pruned;
    }
    // oldest-first so we evict least-recently-written evidence
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.mtime < b.mtime; });
    for (const auto& item : items) {
        if (total <= evidence_quota_bytes_) {
            break;
        }
        std::error_code rm_ec;
        if (fs::remove(item.path, rm_ec) && !rm_ec) {
            total -= item.size;
            pruned.push_back(item.path.string());
        }
    }
    if (!pruned.empty()) {
        LOG_WARN_M(
            MODULE_NAME,
            "evidence quota exceeded; LRU-pruned " << pruned.size() << " oldest jpg(s) from " << save_dir);
    }
    return pruned;
}

// The Objects loop (sup_process thread). Per inference result taken from the latest-wins slot:
//   1. R6 identity checks before anything mutates state: context valid, inference completed,
//      the source stream is still the active one, a capture stamp exists, and the adapter
//      prepares the frame. Any failure stops the service (fatal), naming the check.
//   2. Ring-buffer the RGB frame for capture commands; run ByteTrack on the capture time.
//   3. Depth per track: raw sample (two lanes when switched on and >= 2 tracks, else serially),
//      distortion-corrected centroid, measured object geometry from the raw band, then the
//      depth hysteresis; weak-tier tracks are published only with good depth evidence.
//   4. Odom transform (only with an odom pose; idle while TF is refused) and velocity.
//   5. Build the Objects body and hand it to the service (objects_ready). A failed build or
//      callback is fatal too: R6 never skips a frame silently.
// Diagnostic logs (DUAL_MODEL_DIAG every 30 frames; SUP_FRAME and DEPTH_PROFILE_* when their
// switches are on) stay out of the published body.
void Supervisor::process_loop() {
    set_current_thread_name("sup_process");
    std::uint64_t dual_diagnostic_updates = 0;
    const bool parallel_depth_sample_on =
        parallel_depth_sample_enabled(config_->camera_driver());
    // "2" (2026-10-08): the shared split of FixedTwoLaneDepthSampler, same outputs as "1".
    const bool parallel_depth_sample_shared = parallel_depth_sample_on && parallel_depth_sample_split_shared();
    // Diagnostic "alt" (2026-10-08): fixed split on even frame ids, shared on odd ones, so both
    // splits meet the same scene in one run; outputs are the same either way.
    const bool parallel_depth_sample_alternate = parallel_depth_sample_on && parallel_depth_sample_split_alternate();
    const char* depth_profile_value = std::getenv("PERCEPTION_DEPTH_PROFILE");
    const bool depth_sort_shadow_enabled =
        depth_sampler_.params().sort_impl == DepthSampler::SortImplementation::Shadow;
    const bool depth_profile_enabled =
        ((depth_profile_value != nullptr && std::string(depth_profile_value) == "1") ||
         depth_sort_shadow_enabled) &&
        config_->camera_driver() == "orbbec_gemini338le";
    LOG_INFO_M(MODULE_NAME, "DEPTH_PROFILE_CONFIG enabled=" << depth_profile_enabled
                << " parallel=" << parallel_depth_sample_on
                << " scope=diagnostic_only");
    const char* parallel_depth_sample_value =
        std::getenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    if (parallel_depth_sample_value != nullptr &&
        std::string(parallel_depth_sample_value) != "0" &&
        std::string(parallel_depth_sample_value) != "" &&
        std::string(parallel_depth_sample_value) != "1" &&
        std::string(parallel_depth_sample_value) != "2" &&
        std::string(parallel_depth_sample_value) != "alt") {
        LOG_WARN_M(
            MODULE_NAME,
            "ignoring invalid PERCEPTION_PARALLEL_DEPTH_SAMPLE="
                << parallel_depth_sample_value << "; exact value 1, 2 or alt is required");
    }
    if (parallel_depth_sample_value != nullptr &&
        (std::string(parallel_depth_sample_value) == "1" || std::string(parallel_depth_sample_value) == "2" ||
         std::string(parallel_depth_sample_value) == "alt") &&
        config_->camera_driver() != "orbbec_gemini338le") {
        LOG_WARN_M(
            MODULE_NAME,
            "ignoring PERCEPTION_PARALLEL_DEPTH_SAMPLE for camera.driver="
                << config_->camera_driver()
                << "; canary is Gemini 338Le only");
    }

    std::unique_ptr<internal::FixedTwoLaneDepthSampler> parallel_depth_sampler;
    if (parallel_depth_sample_on) {
        try {
            parallel_depth_sampler = std::make_unique<internal::FixedTwoLaneDepthSampler>(
                parallel_depth_sample_shared ? internal::TwoLaneSplit::Shared : internal::TwoLaneSplit::Fixed);
            LOG_WARN_M(
                MODULE_NAME,
                "fixed two-lane raw depth sampling canary enabled split="
                    << (parallel_depth_sample_alternate ? "alt" : parallel_depth_sample_shared ? "shared" : "fixed"));
        } catch (const std::exception& ex) {
            fatal_failure_detected_.store(true);
            LOG_ERROR_M(
                MODULE_NAME,
                "failed to start fixed depth-sampling helper: " << ex.what());
            request_shutdown();
            return;
        }
    }
    const char* sup_frame_log_value = std::getenv("PERCEPTION_SUP_FRAME_LOG");
    const bool sup_frame_log = sup_frame_log_value != nullptr && std::string(sup_frame_log_value) == "1";
    const auto thread_cpu_ns = []() -> std::int64_t {
        timespec ts{};
        if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return -1;
        return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    };
    // SUP_FRAME only: this thread's run-queue wait, timeslices, page faults and context switches,
    // read at the same points of a frame as the CPU clock. Made here, on the processing thread;
    // with the log off there is no probe and sched_now() reads nothing.
    std::unique_ptr<ThreadSchedProbe> sched_probe;
    if (sup_frame_log) sched_probe = std::make_unique<ThreadSchedProbe>();
    const auto sched_now = [&sched_probe]() { return sched_probe ? sched_probe->read() : ThreadSchedCounters{}; };
    LOG_INFO_M(MODULE_NAME, "SUP_FRAME_LOG_CONFIG enabled=" << static_cast<int>(sup_frame_log)
                                << " schedstat=" << static_cast<int>(sched_probe && sched_probe->schedstat_open()));
    std::vector<internal::RawDepthSampleTask> parallel_depth_tasks;
    std::vector<DepthSample> parallel_raw_samples;
    // Each sample's selected pixels, handed to the object geometry so it does not scan the
    // region again (2026-10-06); kept across frames so their storage is reused.
    std::vector<std::vector<std::uint32_t>> parallel_selected_pixels;
    std::vector<std::uint32_t> serial_selected_pixels;
    // SUP_FRAME only: one byte per depth pixel, marked while a frame's selected-pixel lists are
    // counted (bit 0: selected by any track, bit 1: by a track observed in this frame).
    std::vector<std::uint8_t> overlap_marks;
    std::vector<std::exception_ptr> parallel_raw_exceptions;
    std::vector<DepthSampleProfile> depth_profiles;

    while (running_.load()) {
        yolo::DetectionResult result;
        if (!detection_queue_->pop(result, QUEUE_POP_TIMEOUT)) {
            continue;
        }
        result_ledger_.on_popped(result.frame_id);
        if (result.pipeline_timing.valid) {
            result.pipeline_timing.supervisor_pop_end_ns = monotonic_now_ns();
        }
        const std::int64_t sup_cpu_begin_ns = sup_frame_log ? thread_cpu_ns() : -1;
        const std::int64_t sup_wall_begin_ns = sup_frame_log ? monotonic_now_ns() : -1;
        const ThreadSchedCounters sup_sched_begin = sched_now();
        // SUP_FRAME only: the wall clock again once the scheduler counters are read, so that the
        // gap between "wall begin" and "counters begin" is on the line (the two are not one instant).
        const std::int64_t sup_probe_begin_done_ns = sup_frame_log ? monotonic_now_ns() : 0;

        try {
            geometry::CaptureStamp rns_capture;
            if (rns_context_) {
                // Original raw identity is carried in-memory through both JIT
                // slots. Validate before this frame can mutate the sole tracker.
                // Same checks in the same order as before; the message now names the
                // one that rejected the frame (a fatal stop must say why).
                const auto& source = result.native_depth_source;
                std::string why;
                if (!rns_context_->valid()) {
                    why = "context_invalid";
                } else if (!result.inference_completed) {
                    why = "inference_incomplete";
                } else if (const auto active = rns_context_->native_mailbox->active_stream();
                           active != source.stream_id) {
                    why = "stream_mismatch active_stream=" + std::to_string(active);
                } else if (!rns_context_->capture_stamp(source, rns_capture)) {
                    why = "capture_stamp_unavailable";
                } else if (const auto status = rns_objects_->prepare(result, rns_capture, rns_context_->now_ns());
                           status != rns::FusedObjectsStatus::Prepared) {
                    why = "prepare_status=" + std::to_string(static_cast<int>(status));
                }
                if (!why.empty())
                    throw std::runtime_error("R6_SOURCE_OR_CAPTURE_REJECTED_BEFORE_TRACKER reason=" + why +
                        " frame_id=" + std::to_string(result.frame_id) +
                        " source_stream=" + std::to_string(source.stream_id) +
                        " source_sequence=" + std::to_string(source.sequence) +
                        " source_host_ready_ns=" + std::to_string(source.host_ready_mono_ns));
                ++rns_prepared_;
            }
            ++dual_diagnostic_updates;
            if (dual_diagnostic_updates == 1U ||
                (dual_diagnostic_updates % 30U) == 0U) {
                float max_conflict_iou = 0.0F;
                for (const auto& conflict : result.cross_model_conflicts) {
                    max_conflict_iou =
                        std::max(max_conflict_iou, conflict.box_iou);
                }
                LOG_INFO_M(
                    MODULE_NAME,
                    "DUAL_MODEL_DIAG frame_id=" << result.frame_id
                                                 << " tracker_detections="
                                                 << result.detections.size()
                                                 << " map_detections="
                                                 << result.map_detections.size()
                                                 << " conflict_count="
                                                 << result.cross_model_conflicts.size()
                                                 << " max_conflict_iou="
                                                 << max_conflict_iou);
            }
            // T11 capture: append to the frame ring buffer (shared ref, O(1) --
            // cv::Mat is ref-counted, no deep copy) so a capture command can pick
            // the frame nearest its image_time_hint. Trim to capacity (drop oldest).
            if (!result.frame.empty()) {
                std::lock_guard<std::mutex> frame_lock(frame_buffer_mutex_);
                frame_buffer_.push_back(
                    {result.frame, result.stamp_sec > 0.0 ? result.stamp_sec : system_now_sec()});
                while (frame_buffer_.size() > frame_buffer_capacity_) {
                    frame_buffer_.pop_front();
                }
            }

            yolo::Timer tracker_timer;
            tracker_timer.start();
            const std::vector<Track> tracks =
                tracker_->update(result.detections, result.frame_id, rns_capture.capture_mono_ns / 1000000);
            tracker_timer.stop();
            const std::int64_t sup_cpu_tracker_end_ns = sup_frame_log ? thread_cpu_ns() : -1;
            const std::int64_t sup_tracker_end_ns = sup_frame_log ? monotonic_now_ns() : 0;
            const ThreadSchedCounters sup_sched_tracker_end = sched_now();

            const double frame_stamp_sec = result.stamp_sec > 0.0 ? result.stamp_sec : system_now_sec();
            yolo::Timer fusion_core_timer;
            fusion_core_timer.start();
            // SUP_FRAME only: where the older fusion_core timer starts and stops on the wall clock.
            // It starts after the stamps and the counter read above and stops before the ones
            // below, so it is a shorter interval than batch + loop; both ends are logged.
            const std::int64_t sup_fusion_timer_start_ns = sup_frame_log ? monotonic_now_ns() : 0;
            std::vector<FusedDetection> fused;
            fused.reserve(tracks.size());
            std::unordered_set<int> active_depth_track_ids;
            active_depth_track_ids.reserve(tracks.size());
            double depth_sample_ms = 0.0;
            // Two-lane batch timings of this frame for SUP_FRAME (null: no batch this frame).
            const internal::TwoLaneBatchStats* lane_stats = nullptr;
            std::uint64_t depth_batch_start_ns = 0;
            std::uint64_t depth_batch_end_ns = 0;
            if (depth_profile_enabled) {
                depth_profiles.clear();
                depth_profiles.resize(tracks.size());
            }

            // Two-lane path (production: parallel_depth_sample "1", frames with two or more
            // tracks; formerly called the canary): build immutable raw-sample inputs in original
            // track order, execute only DepthSampler::sample() on two fixed lanes,
            // then consume the index-aligned results below on this thread.
            const bool use_parallel_depth_batch =
                parallel_depth_sampler != nullptr && tracks.size() >= 2U;
            // The selected-pixel lists are only collected when the geometry hook will read them.
            const bool keep_selected_pixels = rns_context_ && rns_context_->object_geometry;
            if (use_parallel_depth_batch) {
                parallel_depth_tasks.clear();
                parallel_depth_tasks.reserve(tracks.size());
                for (const auto& track : tracks) {
                    const cv::Rect& raw_bbox =
                        track.raw_bbox_2d.empty() ? track.bbox_2d : track.raw_bbox_2d;
                    yolo::Detection sample_detection;
                    sample_detection.bbox = raw_bbox;
                    sample_detection.confidence = track.confidence;
                    sample_detection.class_id = track.class_id;
                    sample_detection.class_name = track.class_name;
                    sample_detection.mask = track.mask;
                    const bool use_generic_weak_policy =
                        track.low_confidence_candidate ||
                        track.current_observation_low_confidence_candidate;
                    const auto class_sampler = use_generic_weak_policy
                                                   ? per_class_depth_samplers_.end()
                                                   : per_class_depth_samplers_.find(track.class_name);
                    const DepthSampler* sampler =
                        class_sampler == per_class_depth_samplers_.end()
                            ? &depth_sampler_
                            : &class_sampler->second;
                    parallel_depth_tasks.push_back(
                        {sampler, std::move(sample_detection)});
                }
                parallel_raw_samples.clear();
                parallel_raw_samples.resize(parallel_depth_tasks.size());
                parallel_raw_exceptions.clear();
                parallel_raw_exceptions.resize(parallel_depth_tasks.size());
                if (keep_selected_pixels) parallel_selected_pixels.resize(parallel_depth_tasks.size());
                if (parallel_depth_sample_alternate)
                    parallel_depth_sampler->set_split(result.frame_id % 2 != 0 ? internal::TwoLaneSplit::Shared
                                                                               : internal::TwoLaneSplit::Fixed);
                const auto parallel_sample_start = std::chrono::steady_clock::now();
                parallel_depth_sampler->sample(
                    parallel_depth_tasks,
                    result.depth,
                    result.depth_intrinsics,
                    result.depth_scale,
                    parallel_raw_samples,
                    parallel_raw_exceptions,
                    depth_profile_enabled ? &depth_profiles : nullptr,
                    keep_selected_pixels ? &parallel_selected_pixels : nullptr);
                const auto parallel_sample_end = std::chrono::steady_clock::now();
                lane_stats = &parallel_depth_sampler->last_stats();
                if (depth_profile_enabled) {
                    depth_batch_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        parallel_sample_start.time_since_epoch()).count();
                    depth_batch_end_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        parallel_sample_end.time_since_epoch()).count();
                }
                depth_sample_ms += std::chrono::duration_cast<std::chrono::microseconds>(
                                       parallel_sample_end - parallel_sample_start)
                                       .count() /
                    1000.0;
            }

            // SUP_FRAME only: the fusion core is the batch (sampling inputs and the two-lane call
            // above) and then this per-track loop; inside the loop the mask stage and the object
            // geometry hook are timed per track and summed. A timed stage reads the wall and the
            // thread CPU clock at both ends (four reads per stage and track), only with the log on.
            const std::int64_t sup_cpu_loop_begin_ns = sup_frame_log ? thread_cpu_ns() : -1;
            const std::int64_t sup_loop_begin_ns = sup_frame_log ? monotonic_now_ns() : 0;
            struct StageSum {
                std::int64_t wall_ns = 0, cpu_ns = 0;
                std::size_t calls = 0;
                std::uint64_t units = 0;
                bool cpu_valid = true;
            };
            StageSum mask_stage, geometry_stage;
            // Overlap of the tracks' selected pixels (2026-10-13, diagnostic): how many of the pixels
            // handed to the geometry hook in this frame are distinct, over all tracks and over the
            // tracks observed in this frame (a coasting track's geometry is not built). The marking
            // is a stage of its own, so the geometry stage and rest_* keep their meaning; it does
            // add to the thread CPU of a frame while the log is on.
            StageSum overlap_stage;
            std::uint64_t overlap_distinct = 0, overlap_observed_px = 0, overlap_observed_distinct = 0;
            std::size_t overlap_observed_tracks = 0, overlap_without_list = 0;
            std::int64_t stage_wall_ns = 0, stage_cpu_ns = -1;
            const auto stage_begin = [&]() {
                if (!sup_frame_log) return;
                stage_wall_ns = monotonic_now_ns();
                stage_cpu_ns = thread_cpu_ns();
            };
            const auto stage_end = [&](StageSum& sum, std::uint64_t units) {
                if (!sup_frame_log) return;
                const std::int64_t cpu_ns = thread_cpu_ns();
                sum.wall_ns += monotonic_now_ns() - stage_wall_ns;
                if (cpu_ns < 0 || stage_cpu_ns < 0) sum.cpu_valid = false;
                else sum.cpu_ns += cpu_ns - stage_cpu_ns;
                ++sum.calls;
                sum.units += units;
            };
            if (sup_frame_log && keep_selected_pixels) {
                stage_begin();
                overlap_marks.assign(result.depth.total(), 0);
                stage_end(overlap_stage, 0);
            }
            for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
                const auto& track = tracks[track_index];
                active_depth_track_ids.insert(track.track_id);
                const cv::Rect& raw_bbox = track.raw_bbox_2d.empty() ? track.bbox_2d : track.raw_bbox_2d;
                FusedDetection detection;
                detection.track_id = track.track_id;
                detection.class_id = track.class_id;
                detection.class_name = track.class_name;
                detection.confidence = track.confidence;
                detection.semantic_status = track.semantic_status;
                detection.bbox_2d = track.bbox_2d;
                detection.raw_bbox_2d = raw_bbox;
                detection.first_seen_frame = track.first_seen_frame;
                detection.last_seen_frame = track.last_seen_frame;
                detection.first_seen_ms = track.first_seen_ms;
                detection.last_seen_ms = track.last_seen_ms;
                detection.stable_frames = track.stable_frames;
                if (!track.mask.empty()) {
                    stage_begin();
                    detection.mask_pixel_count = cv::countNonZero(track.mask);
                    std::vector<std::vector<cv::Point>> contours;
                    // No copy: since OpenCV 3.2 findContours does not modify its input (the
                    // target has 4.5.4), so the track mask can be read in place.
                    cv::findContours(track.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                    if (!contours.empty()) {
                        const auto largest = std::max_element(
                            contours.begin(), contours.end(), [](const auto& lhs, const auto& rhs) {
                                return cv::contourArea(lhs) < cv::contourArea(rhs);
                            });
                        constexpr std::size_t MAX_POLYGON_POINTS = 64U;
                        const std::size_t point_count = std::min(MAX_POLYGON_POINTS, largest->size());
                        detection.mask_polygon_2d.reserve(point_count);
                        for (std::size_t i = 0; i < point_count; ++i) {
                            const cv::Point& local = (*largest)[i * largest->size() / point_count];
                            detection.mask_polygon_2d.emplace_back(
                                local.x + raw_bbox.x, local.y + raw_bbox.y);
                        }
                    }
                    stage_end(mask_stage, track.mask.total());
                }
                // Only a track whose identity was born from the generic weak
                // tier carries an arbitrary proxy label. A normal semantic
                // track with one weak observation keeps its class velocity
                // policy, while still taking the stricter weak depth gate.
                std::chrono::steady_clock::time_point depth_sample_start;
                const DepthSample raw_sample = undistort_centroid([&]() {
                    if (use_parallel_depth_batch) {
                        depth_sample_start = std::chrono::steady_clock::now();
                        // Sampling is side-effect free, but hysteresis is not.
                        // Re-throw at the original track index so every earlier
                        // successful track is committed exactly as in the
                        // serial loop before this frame reaches its first error.
                        if (parallel_raw_exceptions[track_index]) {
                            std::rethrow_exception(
                                parallel_raw_exceptions[track_index]);
                        }
                        detection.use_generic_velocity_policy =
                            track.low_confidence_candidate;
                        return std::move(parallel_raw_samples[track_index]);
                    }
                    // Serial path (parallel sampling off, or fewer than two tracks in this
                    // frame): preserve the original input
                    // construction and per-class sampler lookup here only.
                    yolo::Detection sample_detection;
                    sample_detection.bbox = raw_bbox;
                    sample_detection.confidence = track.confidence;
                    sample_detection.class_id = track.class_id;
                    sample_detection.class_name = track.class_name;
                    sample_detection.mask = track.mask;
                    depth_sample_start = std::chrono::steady_clock::now();
                    const bool use_generic_weak_policy =
                        track.low_confidence_candidate ||
                        track.current_observation_low_confidence_candidate;
                    detection.use_generic_velocity_policy =
                        track.low_confidence_candidate;
                    const auto class_sampler = use_generic_weak_policy
                                                   ? per_class_depth_samplers_.end()
                                                   : per_class_depth_samplers_.find(track.class_name);
                    const DepthSampler& sampler =
                        class_sampler == per_class_depth_samplers_.end()
                            ? depth_sampler_
                            : class_sampler->second;
                    return sampler.sample(
                        result.depth,
                        result.depth_intrinsics,
                        result.depth_scale,
                        sample_detection,
                        depth_profile_enabled ? &depth_profiles[track_index] : nullptr,
                        nullptr,
                        keep_selected_pixels ? &serial_selected_pixels : nullptr);
                }(), result.depth_intrinsics, result.depth_registration.get());
                if (depth_sort_shadow_enabled &&
                    !depth_profiles[track_index].sort_shadow_checked) {
                    throw std::logic_error("DEPTH_SORT_SHADOW_UNCHECKED_TASK");
                }
                if (rns_context_ && rns_context_->object_geometry && raw_sample.valid) {
                    // Same region and point band as this frame's raw sample; the
                    // hysteresis-held published depth is NOT used for geometry.
                    const std::vector<std::uint32_t>* selected_pixels =
                        use_parallel_depth_batch ? &parallel_selected_pixels[track_index] : &serial_selected_pixels;
                    stage_begin();
                    rns_context_->object_geometry(result, raw_bbox, track.mask, raw_sample.selected_min_m,
                                                  raw_sample.selected_max_m, selected_pixels, detection);
                    stage_end(geometry_stage, selected_pixels->size());
                    if (sup_frame_log) {
                        stage_begin();
                        const bool observed = rns::observed_in_frame(track.last_seen_frame, result.frame_id);
                        if (selected_pixels->empty()) ++overlap_without_list;
                        if (observed) { ++overlap_observed_tracks; overlap_observed_px += selected_pixels->size(); }
                        const std::uint8_t observed_bit = observed ? 2 : 0;
                        for (const std::uint32_t pixel : *selected_pixels) {
                            if (pixel >= overlap_marks.size()) continue;
                            std::uint8_t& mark = overlap_marks[pixel];
                            if ((mark & 1) == 0) ++overlap_distinct;
                            if (observed_bit != 0 && (mark & 2) == 0) ++overlap_observed_distinct;
                            mark = static_cast<std::uint8_t>(mark | 1 | observed_bit);
                        }
                        stage_end(overlap_stage, selected_pixels->size());
                    }
                }
                const DepthSample sample =
                    depth_hysteresis_.update(track.track_id, raw_sample);
                depth_sample_ms += std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now() -
                                       depth_sample_start)
                                       .count() /
                    1000.0;
                // Only the selected band contributes to the published depth.
                // Keep total valid pixels separately so field diagnostics can
                // distinguish weak clusters from a generally sparse mask.
                detection.fused_points_count = sample.selected_pixel_count;
                detection.depth_sample_region_pixel_count = sample.sample_region_pixel_count;
                detection.depth_valid_pixel_count = sample.valid_pixel_count;
                detection.depth_required_cluster_pixel_count = sample.required_cluster_pixel_count;
                detection.depth_valid_fraction = sample.valid_fraction;
                detection.depth_selected_fraction = sample.selected_fraction;
                detection.depth_sample_source = sample.sample_source;
                detection.depth_selection_status = sample.selection_status;
                detection.depth_selection_fallback = sample.selection_fallback;
                detection.depth_raw_valid = raw_sample.valid;
                detection.depth_raw_m = raw_sample.depth_m;
                detection.depth_raw_sample_region_pixel_count =
                    raw_sample.sample_region_pixel_count;
                detection.depth_raw_valid_pixel_count = raw_sample.valid_pixel_count;
                detection.depth_raw_selected_pixel_count = raw_sample.selected_pixel_count;
                detection.depth_raw_required_cluster_pixel_count =
                    raw_sample.required_cluster_pixel_count;
                detection.depth_raw_valid_fraction = raw_sample.valid_fraction;
                detection.depth_raw_selected_fraction = raw_sample.selected_fraction;
                detection.depth_raw_selected_stddev_m = raw_sample.selected_stddev_m;
                detection.depth_raw_sample_source = raw_sample.sample_source;
                detection.depth_raw_selection_status = raw_sample.selection_status;
                detection.depth_raw_selection_fallback = raw_sample.selection_fallback;
                detection.depth_hysteresis_held =
                    raw_sample.valid && sample.valid &&
                    std::fabs(raw_sample.depth_m - sample.depth_m) > 1e-5F;
                detection.depth_m = sample.depth_m;
                detection.depth_confidence = sample.valid ? 1.0F : 0.0F;
                detection.centroid_3d[0] = sample.centroid_3d[0];
                detection.centroid_3d[1] = sample.centroid_3d[1];
                detection.centroid_3d[2] = sample.centroid_3d[2];
                detection.depth_stat.min = sample.selected_min_m;
                detection.depth_stat.median = sample.selected_median_m;
                detection.depth_stat.max = sample.selected_max_m;
                detection.depth_stat.stddev = sample.selected_stddev_m;
                // The same test as detail::has_good_raw_depth_evidence (>= 5 selected pixels,
                // <= 6 m; origin there), applied here to the hysteresis-held sample. The two
                // results may differ on purpose: while a jump is unconfirmed the held sample is
                // the older one, with its own statistics. A weak observation needs both to pass.
                detection.fusion_quality =
                    sample.valid && !sample.selection_fallback && sample.selected_pixel_count >= 5 &&
                            sample.depth_m <= 6.0F
                        ? "good"
                        : "depth_unreliable";
                if (detail::should_publish_observation(
                        track.low_confidence_candidate ||
                            track.current_observation_low_confidence_candidate,
                        raw_sample.valid,
                        raw_sample.selection_fallback,
                        raw_sample.selected_pixel_count,
                        raw_sample.depth_m,
                        detection.fusion_quality == "good")) {
                    fused.push_back(std::move(detection));
                }
            }
            depth_hysteresis_.retain(active_depth_track_ids);
            fusion_core_timer.stop();
            const std::int64_t sup_fusion_timer_stop_ns = sup_frame_log ? monotonic_now_ns() : 0;
            const std::int64_t sup_cpu_fusion_end_ns = sup_frame_log ? thread_cpu_ns() : -1;
            const std::int64_t sup_fusion_end_ns = sup_frame_log ? monotonic_now_ns() : 0;
            const ThreadSchedCounters sup_sched_fusion_end = sched_now();

            yolo::Timer odom_timer;
            odom_timer.start();
            OdomPose odom_pose;
            float odom_diff_ms = 0.0F;
            const bool has_odom =
                odom_buffer_ && odom_buffer_->get_nearest(frame_stamp_sec, odom_pose, odom_diff_ms);
            if (has_odom) {
                transform_detections_to_odom(fused, odom_pose, extrinsic_camera_to_base_);
            }
            odom_timer.stop();

            yolo::Timer velocity_timer;
            velocity_timer.start();
            update_velocity(fused, rns_capture, has_odom);
            velocity_timer.stop();

            PerceptionFrame frame = build_frame(
                result,
                fused,
                0.0F,
                has_odom ? "odom" : "camera",
                frame_stamp_sec,
                0.0,
                static_cast<float>(tracker_timer.elapsed_ms()),
                static_cast<float>(fusion_core_timer.elapsed_ms()),
                static_cast<float>(depth_sample_ms),
                static_cast<float>(odom_timer.elapsed_ms()),
                static_cast<float>(velocity_timer.elapsed_ms()),
                last_publish_ms_);
            LOG_DEBUG_M(
                MODULE_NAME,
                "DEPTH_SAMPLE image_stamp_sec=" << format_stamp(result.stamp_sec)
                                              << " depth_scale=" << result.depth_scale
                                              << " frame_queue_depth=" << frame.frame_queue_depth
                                              << " detection_queue_depth=" << frame.detection_queue_depth
                                              << " inference_time_ms=" << result.inference_time_ms
                                              << " good_detection_count="
                                              << std::count_if(
                                                     fused.begin(),
                                                     fused.end(),
                                                     [](const FusedDetection& det) {
                                                         return det.fusion_quality == "good";
                                                     })
                                              << " detection_count=" << fused.size()
                                              << " depth_size=" << result.depth.cols << "x" << result.depth.rows);
            yolo::Timer publish_timer;
            publish_timer.start();
            // SUP_FRAME only: where the publish step goes (body build vs. the service callback).
            const std::int64_t sup_cpu_publish_begin_ns = sup_frame_log ? thread_cpu_ns() : -1;
            const std::int64_t sup_publish_begin_ns = sup_frame_log ? monotonic_now_ns() : 0;
            const ThreadSchedCounters sup_sched_publish_begin = sched_now();
            std::int64_t sup_build_begin_ns = 0, sup_build_end_ns = 0, sup_callback_end_ns = 0;
            if (!apply_gemini_runtime_geometry(result, frame)) {
                throw std::runtime_error("R6_FINAL_FRAME_GEOMETRY_REJECTED");
            }
            finalize_prepublish_timing(result, frame);
            // Isolated timing probe: host API call bounds, not DDS/Zenoh delivery.
            // Keep these side-channel observations out of the published payload.
            const auto publish_call_begin_ns = depth_profile_enabled ? monotonic_now_ns() : 0;
            bool detections_published = false;
            if (rns_context_) {
                if (!rns_context_->valid() || rns_context_->native_mailbox->active_stream() !=
                        result.native_depth_source.stream_id)
                    throw std::runtime_error("R6_SOURCE_RETIRED_BEFORE_FINAL_OUTPUT");
                if (sup_frame_log) sup_build_begin_ns = monotonic_now_ns();
                const auto report = rns_objects_->build(result, rns_capture, fused, tracks);
                if (sup_frame_log) sup_build_end_ns = monotonic_now_ns();
                if (report.status != rns::FusedObjectsStatus::Ready)
                    throw std::runtime_error("R6_FINAL_FUSED_CONTRACT_REJECTED status=" +
                                             std::to_string(static_cast<int>(report.status)) + " detail=" + report.detail);
                ++rns_callback_attempts_;
                detections_published = rns_context_->objects_ready(
                    result, rns_capture, rns_objects_->last_body(), report);
                if (!detections_published) throw std::runtime_error("R6_FINAL_CALLBACK_FAILED_NO_RETRY");
                ++rns_callback_successes_;
                if (sup_frame_log) sup_callback_end_ns = monotonic_now_ns();
            }
            const auto publish_call_end_ns = depth_profile_enabled ? monotonic_now_ns() : 0;
            if (depth_sort_shadow_enabled && !detections_published) {
                throw std::runtime_error("DEPTH_SORT_SHADOW_PUBLICATION_FAILED");
            }
            publish_timer.stop();
            last_publish_ms_ = static_cast<float>(publish_timer.elapsed_ms());
            if (sup_frame_log) {
                const auto wall_end_ns = monotonic_now_ns();
                const auto cpu_end_ns = thread_cpu_ns();
                const ThreadSchedCounters sched_end = sched_now();
                const std::int64_t probe_end_done_ns = monotonic_now_ns();
                // Four consecutive spans cover the frame on this thread: head (pop to the tracker's end:
                // source checks, capture ring, tracker), fusion core (depth sampling, geometry,
                // hysteresis), between (odom, velocity, frame build, debug log) and publish (body build
                // and the service callback). *_ms is the span's wall time, *_cpu_ms this thread's CPU in
                // it; wall minus CPU is waiting (for a CPU, for depth_lane, for a lock). A CPU clock
                // that failed to read shows as -1, never as a guess.
                const auto cpu_span_ms = [](std::int64_t a, std::int64_t b) {
                    return a < 0 || b < 0 ? -1.0 : (b - a) / 1.0e6;
                };
                std::ostringstream line;
                line << "SUP_FRAME frame_id=" << result.frame_id << " tracks=" << tracks.size()
                    << " fused=" << fused.size() << " tracker_ms=" << tracker_timer.elapsed_ms()
                    << " fusion_core_ms=" << fusion_core_timer.elapsed_ms() << " depth_sample_ms=" << depth_sample_ms
                    << " velocity_ms=" << velocity_timer.elapsed_ms() << " publish_ms=" << publish_timer.elapsed_ms()
                    << " prepublish_ms=" << (sup_build_begin_ns > 0 ? (sup_build_begin_ns - sup_publish_begin_ns) / 1.0e6 : -1.0)
                    << " objects_build_ms=" << (sup_build_end_ns > 0 ? (sup_build_end_ns - sup_build_begin_ns) / 1.0e6 : -1.0)
                    << " callback_ms=" << (sup_callback_end_ns > 0 ? (sup_callback_end_ns - sup_build_end_ns) / 1.0e6 : -1.0)
                    << " head_ms=" << (sup_tracker_end_ns - sup_wall_begin_ns) / 1.0e6
                    << " between_ms=" << (sup_publish_begin_ns - sup_fusion_end_ns) / 1.0e6
                    << " head_cpu_ms=" << cpu_span_ms(sup_cpu_begin_ns, sup_cpu_tracker_end_ns)
                    << " fusion_core_cpu_ms=" << cpu_span_ms(sup_cpu_tracker_end_ns, sup_cpu_fusion_end_ns)
                    << " between_cpu_ms=" << cpu_span_ms(sup_cpu_fusion_end_ns, sup_cpu_publish_begin_ns)
                    << " publish_cpu_ms=" << cpu_span_ms(sup_cpu_publish_begin_ns, cpu_end_ns)
                    << " wall_ms=" << (wall_end_ns - sup_wall_begin_ns) / 1.0e6
                    << " thread_cpu_ms=" << cpu_span_ms(sup_cpu_begin_ns, cpu_end_ns);
                // Two-lane batch (2026-10-08): items per lane (caller/helper), whether the helper
                // picked the batch up, when (ms after the batch start), when the caller finished
                // its own items and when the batch was joined; "-" without a batch this frame.
                if (lane_stats) {
                    line << " lane_split=" << (lane_stats->split == internal::TwoLaneSplit::Shared ? "shared" : "fixed")
                         << " lane_items=" << lane_stats->caller_items << "/" << lane_stats->helper_items
                         << " lane_claimed=" << lane_stats->helper_claimed
                         << " lane_helper_start_ms=" << (lane_stats->helper_start_ns >= 0 ? lane_stats->helper_start_ns / 1.0e6 : -1.0)
                         << " lane_caller_done_ms=" << lane_stats->caller_done_ns / 1.0e6
                         << " lane_join_ms=" << lane_stats->join_ns / 1.0e6;
                } else {
                    line << " lane_items=-";
                }
                line << " sup_begin_ns=" << sup_wall_begin_ns << " sup_end_ns=" << wall_end_ns;
                // Fusion core split (2026-10-11): batch = the sampling inputs and the two-lane call,
                // loop = the per-track loop after it (batch + loop = fusion core). Inside the loop
                // mask_* (mask area, contour, polygon) and geom_* (the object geometry hook) are
                // sums over the tracks, rest_* is what remains (sample hand-over, undistortion,
                // hysteresis, field copies). *_n: timed tracks; mask_px: mask pixels scanned;
                // geom_px: selected pixels handed to the hook. geom_uniq_px: distinct pixels among
                // them; geom_obs_*: the same three counts for the tracks observed in this frame;
                // geom_nolist_n: hook calls without a list; ovl_*: the cost of counting these.
                const auto stage_cpu_ms = [](const StageSum& sum) { return sum.cpu_valid ? sum.cpu_ns / 1.0e6 : -1.0; };
                const double loop_cpu_ms = cpu_span_ms(sup_cpu_loop_begin_ns, sup_cpu_fusion_end_ns);
                const bool rest_cpu_valid = loop_cpu_ms >= 0 && mask_stage.cpu_valid && geometry_stage.cpu_valid &&
                                            overlap_stage.cpu_valid;
                line << " batch_ms=" << (sup_loop_begin_ns - sup_tracker_end_ns) / 1.0e6
                     << " batch_cpu_ms=" << cpu_span_ms(sup_cpu_tracker_end_ns, sup_cpu_loop_begin_ns)
                     << " loop_ms=" << (sup_fusion_end_ns - sup_loop_begin_ns) / 1.0e6
                     << " loop_cpu_ms=" << loop_cpu_ms
                     << " mask_ms=" << mask_stage.wall_ns / 1.0e6 << " mask_cpu_ms=" << stage_cpu_ms(mask_stage)
                     << " mask_n=" << mask_stage.calls << " mask_px=" << mask_stage.units
                     << " geom_ms=" << geometry_stage.wall_ns / 1.0e6 << " geom_cpu_ms=" << stage_cpu_ms(geometry_stage)
                     << " geom_n=" << geometry_stage.calls << " geom_px=" << geometry_stage.units
                     << " geom_uniq_px=" << overlap_distinct << " geom_obs_n=" << overlap_observed_tracks
                     << " geom_obs_px=" << overlap_observed_px << " geom_obs_uniq_px=" << overlap_observed_distinct
                     << " geom_nolist_n=" << overlap_without_list
                     << " ovl_ms=" << overlap_stage.wall_ns / 1.0e6 << " ovl_cpu_ms=" << stage_cpu_ms(overlap_stage)
                     << " rest_ms=" << (sup_fusion_end_ns - sup_loop_begin_ns - mask_stage.wall_ns - geometry_stage.wall_ns -
                                        overlap_stage.wall_ns) / 1.0e6
                     << " rest_cpu_ms="
                     << (rest_cpu_valid ? loop_cpu_ms - (mask_stage.cpu_ns + geometry_stage.cpu_ns + overlap_stage.cpu_ns) / 1.0e6
                                        : -1.0);
                // The kernel's account of the same frame on this thread (thread_sched_probe.hpp):
                // sched_run_ms on a CPU, sched_wait_ms runnable but waiting for one, sched_slices
                // timeslices run, and that wait inside each of the four spans; page faults and
                // context switches (nivcsw: taken off the CPU while still runnable). -1: not readable.
                const auto wait_ms = [](const ThreadSchedCounters& a, const ThreadSchedCounters& b) {
                    return sched_span_ms(a.sched_valid, a.wait_ns, b.sched_valid, b.wait_ns);
                };
                const auto& sched_begin = sup_sched_begin;
                line << " sched_run_ms=" << sched_span_ms(sched_begin.sched_valid, sched_begin.run_ns, sched_end.sched_valid, sched_end.run_ns)
                     << " sched_wait_ms=" << wait_ms(sched_begin, sched_end)
                     << " sched_slices=" << sched_span_count(sched_begin.sched_valid, sched_begin.slices, sched_end.sched_valid, sched_end.slices)
                     << " head_wait_ms=" << wait_ms(sched_begin, sup_sched_tracker_end)
                     << " fusion_core_wait_ms=" << wait_ms(sup_sched_tracker_end, sup_sched_fusion_end)
                     << " between_wait_ms=" << wait_ms(sup_sched_fusion_end, sup_sched_publish_begin)
                     << " publish_wait_ms=" << wait_ms(sup_sched_publish_begin, sched_end)
                     << " minflt=" << usage_span_count(sched_begin.usage_valid, sched_begin.minor_faults, sched_end.usage_valid, sched_end.minor_faults)
                     << " majflt=" << usage_span_count(sched_begin.usage_valid, sched_begin.major_faults, sched_end.usage_valid, sched_end.major_faults)
                     << " nvcsw=" << usage_span_count(sched_begin.usage_valid, sched_begin.voluntary_switches, sched_end.usage_valid, sched_end.voluntary_switches)
                     << " nivcsw=" << usage_span_count(sched_begin.usage_valid, sched_begin.involuntary_switches, sched_end.usage_valid, sched_end.involuntary_switches);
                // Which intervals are which (2026-10-11 night; the first diagnostic window failed
                // its own check that batch + loop equals fusion_core_ms, because they are not the
                // same interval). Every split on this line that is meant to add up uses ONE set of
                // stamps: head_ms + batch_ms + loop_ms + between_ms + publish_span_ms = wall_ms, and
                // fusion_span_ms = batch_ms + loop_ms by construction. fusion_core_ms is the older
                // timer, an interval inside that span: fusion_lead_ms and fusion_tail_ms are the
                // two pieces of the span outside it (stamps, the counter read, clock reads, and
                // any time the thread was off the CPU just then). publish_ms is an older timer
                // too, but it is NOT inside publish_span_ms: it starts before the span's first
                // stamp and stops before its last one, so the two overlap and neither bounds the
                // other. The scheduler counters are read just AFTER the wall stamps at both ends:
                // probe_gap_begin_ms is in wall_ms but not in the sched_* deltas, probe_gap_end_ms
                // is in the sched_* deltas but not in wall_ms. The probe is several separate
                // reads, not one snapshot, so wall_ms - sched_run_ms - sched_wait_ms is a residual
                // without a hard bound: it can be negative by more than the end gap (2026-10-12:
                // -0.025 ms on a frame whose end gap was 0.004 ms). It is not a measurement of
                // sleeping.
                line << " fusion_span_ms=" << (sup_fusion_end_ns - sup_tracker_end_ns) / 1.0e6
                     << " fusion_lead_ms=" << (sup_fusion_timer_start_ns - sup_tracker_end_ns) / 1.0e6
                     << " fusion_tail_ms=" << (sup_fusion_end_ns - sup_fusion_timer_stop_ns) / 1.0e6
                     << " publish_span_ms=" << (wall_end_ns - sup_publish_begin_ns) / 1.0e6
                     << " probe_gap_begin_ms=" << (sup_probe_begin_done_ns - sup_wall_begin_ns) / 1.0e6
                     << " probe_gap_end_ms=" << (probe_end_done_ns - wall_end_ns) / 1.0e6;
                LOG_INFO_M(MODULE_NAME, line.str());
            }
            // Diagnostic-only data stays out of the published interface. Emit
            // after publication, on this thread and only after the lane join.
            // The logging cost can still delay the next frame: compare the
            // profile off/on/off windows rather than assuming zero overhead.
            if (depth_profile_enabled) {
                const auto emit_start_ns = monotonic_now_ns();
                std::uint32_t depth_scale_bits = 0;
                static_assert(sizeof(depth_scale_bits) == sizeof(result.depth_scale));
                std::memcpy(&depth_scale_bits, &result.depth_scale, sizeof(depth_scale_bits));
                for (std::size_t i = 0; i < depth_profiles.size(); ++i) {
                    const auto& p = depth_profiles[i];
                    LOG_INFO_M(MODULE_NAME,
                        "DEPTH_PROFILE_TASK frame_id=" << result.frame_id
                        << " track_index=" << i << " track_id=" << tracks[i].track_id
                        << " class_id=" << tracks[i].class_id
                        << " lane=" << (use_parallel_depth_batch ? i % 2U : 0U)
                        << " parallel=" << use_parallel_depth_batch
                        << " start_ns=" << p.start_ns << " end_ns=" << p.end_ns
                        << " total_wall_ns=" << p.total_wall_ns
                        << " thread_cpu_ns=" << p.thread_cpu_ns
                        << " wall_clock_valid=" << p.wall_clock_valid
                        << " cpu_clock_valid=" << p.cpu_clock_valid
                        << " code_sort_used=" << p.code_sort_used
                        << " sort_shadow_checked=" << p.sort_shadow_checked
                        << " depth_type=" << result.depth.type()
                        << " depth_scale_bits=" << depth_scale_bits
                        << " roi_pixels=" << p.roi_pixels << " region_pixels=" << p.region_pixels
                        << " valid_pixels=" << p.valid_pixels << " selected_pixels=" << p.selected_pixels
                        << " setup_ns=" << p.setup_ns << " scan_ns=" << p.scan_ns
                        << " depth_copy_ns=" << p.depth_copy_ns << " sort_ns=" << p.sort_ns
                        << " cluster_selection_ns=" << p.cluster_selection_ns
                        << " select_pixels_ns=" << p.select_pixels_ns
                        << " median_u_ns=" << p.median_u_ns << " median_v_ns=" << p.median_v_ns
                        << " statistics_ns=" << p.statistics_ns << " finalize_ns=" << p.finalize_ns);
                }
                const auto emit_end_ns = monotonic_now_ns();
                LOG_INFO_M(MODULE_NAME,
                    "DEPTH_PROFILE_BATCH frame_id=" << result.frame_id
                    << " task_count=" << depth_profiles.size()
                    << " parallel=" << use_parallel_depth_batch
                    << " batch_start_ns=" << depth_batch_start_ns
                    << " batch_end_ns=" << depth_batch_end_ns
                    << " emit_start_ns=" << emit_start_ns << " emit_end_ns=" << emit_end_ns
                    << " publish_call_begin_ns=" << publish_call_begin_ns
                    << " publish_call_end_ns=" << publish_call_end_ns
                    << " rns_callback=" << static_cast<int>(rns_context_.has_value())
                    << " publish_success=" << static_cast<int>(detections_published));
            }
        } catch (const std::exception& ex) {
            LOG_ERROR_M(MODULE_NAME, "processLoop failed: " << ex.what());
            // R6 never skips a frame silently: a failed frame is fatal, and a
            // failed correctness comparison can never pass as a clean window.
            if (rns_objects_) rns_objects_->abort();
            fatal_failure_detected_.store(true);
            request_shutdown();
            return;
        }
    }
}

// Time base (B1, GPT answer 1.2, 2026-10-04): the velocity windows run on the frame's
// validated capture-monotonic stamp, the same one the tracker takes, converted from ns to
// seconds without truncation. Before, they ran on result.stamp_sec, the WALL clock sampled when
// the frame reached the host: a wall-clock correction then stretched, shrank or restarted a
// window (19 section 10: every timestamp, timeout and age is CLOCK_MONOTONIC). The capture stamp
// is still host receipt minus a fixed link estimate, so transport jitter remains; only the clock
// changed. The odom pose lookup keeps the wall stamp: TF is refused today and its time mapping is
// part of the pending 19 section 4.2 work.
void Supervisor::update_velocity(std::vector<FusedDetection>& fused,
                                 const geometry::CaptureStamp& capture, bool has_odom) {
    const double velocity_stamp_sec = static_cast<double>(capture.capture_mono_ns) * 1e-9;
    // A switch between camera and odom axes restarts the estimator being switched to: its old
    // windows were built in the other frame.
    const bool switched_velocity_frame =
        has_last_velocity_frame_ && (has_odom != last_velocity_frame_has_odom_);
    if (switched_velocity_frame) {
        if (has_odom) {
            odom_velocity_estimator_.reset();
        } else {
            lidar_velocity_estimator_.reset();
        }
        LOG_WARN_M(
            MODULE_NAME,
            "velocity frame switched to " << (has_odom ? "odom" : "camera")
                                           << "; warm-starting estimator");
    }

    if (has_odom) {
        odom_velocity_estimator_.update(fused, velocity_stamp_sec);
    } else {
        lidar_velocity_estimator_.update(fused, velocity_stamp_sec);
    }
    // The adapter rotates only Camera velocities into base_link (rns_fused_objects).
    for (auto& det : fused) {
        det.velocity_frame = has_odom ? FusedDetection::VelocityFrame::Odom : FusedDetection::VelocityFrame::Camera;
    }
    has_last_velocity_frame_ = true;
    last_velocity_frame_has_odom_ = has_odom;
}

bool Supervisor::apply_gemini_runtime_geometry(
    const yolo::DetectionResult& result,
    PerceptionFrame& frame) {
    if (!config_ || config_->camera_driver() != "orbbec_gemini338le") {
        return true;
    }
    const bool valid = validate_gemini_runtime_geometry(result);
    if (!valid) {
        fatal_failure_detected_.store(true);
        LOG_ERROR_M(
            MODULE_NAME,
            "Gemini runtime intrinsics/frame contract invalid frame_id="
                << result.frame_id << " rgb=" << result.frame.cols << "x"
                << result.frame.rows << " depth=" << result.depth.cols << "x"
                << result.depth.rows << " intrinsics="
                << result.depth_intrinsics.width << "x"
                << result.depth_intrinsics.height);
        request_shutdown();
        return false;
    }
    frame.camera_intrinsics = result.depth_intrinsics;
    frame.camera_intrinsics_valid = true;
    return true;
}

// Checks the hard contract once per second; sleeps in 100 ms slices so stop() is seen quickly.
void Supervisor::heartbeat_loop() {
    set_current_thread_name("sup_heartbeat");
    while (running_.load()) {
        if (!gemini_runtime_healthy()) {
            LOG_ERROR_M(
                MODULE_NAME,
                "Gemini hard-contract runtime stopped: RGB-D capture or HEVC encoder failed");
            fatal_failure_detected_.store(true);
            request_shutdown();
            return;
        }

        auto slept = std::chrono::steady_clock::duration::zero();
        while (running_.load() && slept < HEARTBEAT_INTERVAL) {
            const auto remaining = HEARTBEAT_INTERVAL - slept;
            const auto slice = std::min<std::chrono::steady_clock::duration>(
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(HEARTBEAT_SLEEP_SLICE),
                remaining);
            std::this_thread::sleep_for(slice);
            slept += slice;
        }
    }
}

bool Supervisor::validate_gemini_runtime_geometry(
    const yolo::DetectionResult& result) const {
    return config_ && std::isfinite(result.depth_intrinsics.fx) &&
           std::isfinite(result.depth_intrinsics.fy) &&
           std::isfinite(result.depth_intrinsics.cx) &&
           std::isfinite(result.depth_intrinsics.cy) &&
           result.depth_intrinsics.fx > 0.0F &&
           result.depth_intrinsics.fy > 0.0F &&
           result.depth_intrinsics.cx >= 0.0F &&
           result.depth_intrinsics.cy >= 0.0F &&
           result.depth_intrinsics.width > 0 &&
           result.depth_intrinsics.height > 0 &&
           result.depth_intrinsics.cx <
               static_cast<float>(result.depth_intrinsics.width) &&
           result.depth_intrinsics.cy <
               static_cast<float>(result.depth_intrinsics.height) &&
           !result.frame.empty() && !result.depth.empty() &&
           result.depth_intrinsics.width == result.frame.cols &&
           result.depth_intrinsics.height == result.frame.rows &&
           result.depth_intrinsics.width == result.depth.cols &&
           result.depth_intrinsics.height == result.depth.rows &&
           result.depth_intrinsics.width == config_->camera_output_width() &&
           result.depth_intrinsics.height == config_->camera_output_height();
}

bool Supervisor::gemini_runtime_healthy() const {
    if (!config_ || config_->camera_driver() != "orbbec_gemini338le") {
        return true;
    }
    return rgbd_capture_ && rgbd_capture_->is_running() && inference_ &&
           inference_->is_running() && hevc_encoder_ &&
           hevc_encoder_->is_running() && !hevc_encoder_->failed();
}

bool Supervisor::wait_for_thread_exit(
    std::thread& thread,
    const char* thread_name,
    std::chrono::milliseconds timeout,
    JoinLauncher launcher) {
    if (!thread.joinable()) {
        return true;
    }

    std::future<void> join_future;
    try {
        join_future = launcher ? launcher(thread) : std::async(std::launch::async, [&thread]() {
            thread.join();
        });
    } catch (...) {
        // Failure to create the joining helper must not leave an owned worker
        // joinable while stop() has already cleared running_. The existing
        // timeout path waits too; direct join adds no new watchdog promise.
        if (thread.joinable()) thread.join();
        LOG_WARN_M(MODULE_NAME, thread_name << " async join creation failed; joined directly");
        return false;
    }
    if (join_future.wait_for(timeout) == std::future_status::ready) {
        join_future.get();
        return true;
    }

    LOG_WARN_M(MODULE_NAME, thread_name << " did not exit within " << timeout.count() << "ms; waiting");
    join_future.get();
    return false;
}

bool Supervisor::start_module(yolo::ModuleBase* module, const char* name) {
    if (!module) {
        LOG_ERROR_M(MODULE_NAME, "missing module " << name);
        return false;
    }

    const bool ok = module->start();
    LOG_INFO_M(MODULE_NAME, name << (ok ? " started" : " failed to start"));
    return ok;
}

void Supervisor::finalize_prepublish_timing(
    yolo::DetectionResult& result, PerceptionFrame& frame) {
    if (frame.pipeline_timing.valid) {
        frame.pipeline_timing.prepublish_ns = monotonic_now_ns();
        frame.pipeline_timing.valid = are_positive_nondecreasing(
            {frame.pipeline_timing.capture_ready_ns,
             frame.pipeline_timing.frame_push_begin_ns,
             frame.pipeline_timing.inference_pop_end_ns,
             frame.pipeline_timing.result_push_begin_ns,
             frame.pipeline_timing.supervisor_pop_end_ns,
             frame.pipeline_timing.prepublish_ns});
    }
    // build_frame made a value copy. The R6 callback consumes result, so the stamped ledger
    // is copied back (the legacy callback that consumed frame was removed in the level-2 trim).
    result.pipeline_timing = frame.pipeline_timing;
}

PerceptionFrame Supervisor::build_frame(
    const yolo::DetectionResult& result,
    const std::vector<FusedDetection>& fused,
    float sync_diff_ms,
    const std::string& output_frame,
    double publish_stamp_sec,
    double publish_lidar_stamp_sec,
    float tracker_ms,
    float fusion_core_ms,
    float depth_sample_ms,
    float odom_ms,
    float velocity_ms,
    float publish_ms) const {
    PerceptionFrame frame;
    frame.frame_id = result.frame_id;
    frame.frame = output_frame;
    frame.stamp_sec = publish_stamp_sec;
    frame.video_source = "rgb";
    frame.camera_metadata = result.camera_metadata;
    frame.lidar_stamp_sec = publish_lidar_stamp_sec;
    frame.lidar_sync_diff_ms = sync_diff_ms;
    frame.inference_ms = static_cast<float>(result.inference_time_ms);
    frame.capture_ms = static_cast<float>(result.capture_time_ms);
    frame.depth_sample_ms = depth_sample_ms;
    frame.tracker_ms = tracker_ms;
    frame.fusion_core_ms = fusion_core_ms;
    frame.odom_ms = odom_ms;
    frame.velocity_ms = velocity_ms;
    frame.publish_ms = publish_ms;
    frame.fusion_ms = tracker_ms + fusion_core_ms + odom_ms + velocity_ms;
    if (frame_queue_) {
        frame.frame_queue_depth = frame_queue_->size();
        frame.frame_queue_dropped = frame_queue_->dropped_count();
    }
    if (detection_queue_) {
        frame.detection_queue_depth = detection_queue_->size();
        frame.detection_queue_dropped = detection_queue_->dropped_count();
    }
    frame.detections = fused;
    frame.pipeline_timing = result.pipeline_timing;
    return frame;
}

double Supervisor::steady_now_sec() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now).count()) / 1000000.0;
}

double Supervisor::system_now_sec() {
    // LINT-ALLOW(clock): wall stamps for the capture-done receipt and the fallback frame stamp only.
    // No Objects field, timeout or contract age is derived from it. Precisely: captured_at and
    // frame_age_ms of the capture-done receipt (the latter a difference of two wall stamps, a V5
    // diagnostic), plus the frame stamp used when a result carries none.
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now).count()) / 1000000.0;
}

bool Supervisor::parallel_depth_sample_enabled(const std::string& camera_driver) {
    const char* value = std::getenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    return value != nullptr && (std::string(value) == "1" || std::string(value) == "2" || std::string(value) == "alt") &&
        camera_driver == "orbbec_gemini338le";
}

bool Supervisor::parallel_depth_sample_split_alternate() {
    const char* value = std::getenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    return value != nullptr && std::string(value) == "alt";
}

bool Supervisor::parallel_depth_sample_split_shared() {
    const char* value = std::getenv("PERCEPTION_PARALLEL_DEPTH_SAMPLE");
    return value != nullptr && std::string(value) == "2";
}

}  // namespace perception
