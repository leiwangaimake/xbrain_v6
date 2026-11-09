/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_capture.cpp
 * Brief: Orbbec Gemini 338Le RGB and fail-closed aligned depth capture.
 *
 * Description:
 * What it does. start() opens exactly the configured camera (one device, matching serial and
 * model), selects the exact colour (MJPG) and depth (Y16) profiles, validates a warm-up frameset,
 * the intrinsics and the depth scale, builds the RGB<-depth registration when the measured
 * configuration matches (B4-2), then starts the cap_loop thread. Anything unexpected refuses
 * the start. Production: SW-D2C, native depth 640x400, colour and output 1280x800 at 30 fps.
 * Per frameset, cap_loop: records the raw depth arrival (health); checks the raw RGB-D
 * contract; offers the native depth to the Profile mailbox (before any alignment); aligns depth
 * to colour in software; decodes MJPG to BGR; re-checks shape, scale, intrinsics, the
 * colour/depth device-timestamp skew (<= 5 ms) and that device timestamps and indices only
 * increase; copies colour and registers depth onto the RGB image; then hands the frame to the
 * encoder queue and the inference queue (order per inference_handoff_first, production 0:
 * encoder first).
 * Failure policy. Five consecutive timeouts or incomplete/failed framesets, five seconds without a
 * capture time after a device stream's first second (a deadline on host time, capture_time_state.hpp),
 * or any contract change, end cap_loop; the Supervisor heartbeat then sees the capture stopped and stops the
 * service. There is no silent reconnect.
 * Cost (1.8, 2026-10-04 logs): about 30 ms of the 33 ms frame period, mostly MJPG decode
 * (17-19 ms under heavy load, about 12.5 ms wall in the 3-core windows of 2026-10-05) and
 * software alignment (about 8 ms). CameraStats / CameraTiming lines every 150 frames report
 * counts and stage times.
 * MJPG decode (runtime switch gemini_mjpeg_decode): sdk is the SDK's software decode; nvv4l2 is
 * the hardware decode of HwMjpegDecoder, the production value since the integration lead approved
 * the hardware decoder on 2026-10-08 (issues.md 3.5 A; pixel consistency is our acceptance, power
 * and thermal margin to be watched). The nvv4l2 start checks the
 * hardware decode of the warm-up frame against the SDK decode of the same bytes and refuses to
 * start outside the accepted limits.
 */

#include "common/thread_name.hpp"
#include "modules/orbbec_capture.hpp"
#include <cstdlib>
#include "modules/hw_mjpeg_decoder.hpp"
#include "modules/orbbec_frame_output.hpp"
#include "modules/orbbec_inference_handoff.hpp"
#include "modules/orbbec_native_depth.hpp"
#include "modules/rgb_depth_registration.hpp"
#include "output/rns_source_journal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/h/TypeHelper.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/logger.hpp"
#include "common/camera_identity.hpp"
#include "common/capture_affinity_control.hpp"
#include "common/capture_clock.hpp"
#include "common/capture_time_state.hpp"
#include "common/sdk_stream_thread_placement.hpp"
#include "common/time_utils.hpp"

namespace perception {

namespace {

constexpr unsigned int FRAME_WAIT_TIMEOUT_MS = 1000U;
constexpr int MAX_CONSECUTIVE_TIMEOUTS = 5;
constexpr int MAX_CONSECUTIVE_INCOMPLETE_FRAMES = 5;
constexpr std::uint64_t CAMERA_STATS_INTERVAL_FRAMES = 150U;
// Threads the camera SDK creates on the caller's thread while the pipeline starts: per video
// stream one sensor queue thread and the RTP receive and assemble threads; two streams (color,
// depth). Measured on the Gemini 338Le over Ethernet, SDK reported as 2.9.3 (2026-10-13, three
// starts); before 2.9.3 the SDK started two per stream. The placement self-check accepts
// exactly this number of threads born during the start, so an SDK that starts another number
// fails the check and does not pass as a silent change.
constexpr std::size_t SDK_START_THREADS = 3U * 2U;
constexpr std::uint64_t MAX_COLOR_DEPTH_SKEW_US = 5000U;
// CaptureClockOptions::max_bracket_ns: how far apart the two CLOCK_MONOTONIC readings around the
// CLOCK_MONOTONIC_RAW one may lie (measured 2026-10-14: median 0.1 us, largest 0.45 us).
constexpr std::int64_t CAPTURE_CLOCK_MAX_BRACKET_NS = CaptureClockOptions{}.max_bracket_ns;
constexpr double MILLIMETRES_TO_METRES = 0.001;
// OpenCV 4.10 defaults to all eight Orin cores. In the full dual-model chain,
// capture resize and inference preprocess then contend for the same global
// worker pool and reduce the canonical video path below its 30 FPS contract.
// The measured 1/2/4/8 matrix selects two as the best video/inference balance.
// That is the default; production sets 1 through the runtime switch gemini_opencv_threads
// (PERCEPTION_GEMINI_OPENCV_THREADS, read in start()).
constexpr int GEMINI_OPENCV_THREADS = 2;

// What the capture clock needs from the depth frame beyond its timestamp. Both read SDK
// metadata, which may be absent or throw: absent is a value here, never an exception.
// Half the exposure: the frame timestamp marks the end of the exposure, the sensor timestamp
// its middle (measured 2026-10-13: 15054 us at the 30108 us exposure of the service's profile).
std::int64_t depth_half_exposure_us(const std::shared_ptr<ob::DepthFrame>& depth) noexcept {
    try {
        if (!depth->hasMetadata(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP)) return -1;
        const auto frame_us = static_cast<std::int64_t>(depth->getTimeStampUs());
        return frame_us - depth->getMetadataValue(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP);
    } catch (...) {
        return -1;
    }
}
// The SDK's mapping of the frame timestamp to the host clock (CLOCK_MONOTONIC_RAW with the
// clock type start() sets); 0 or an exception: the SDK has none for this frame.
std::optional<std::uint64_t> depth_global_us(const std::shared_ptr<ob::DepthFrame>& depth) noexcept {
    try {
        const std::uint64_t value = depth->getGlobalTimeStampUs();
        return value ? std::optional<std::uint64_t>(value) : std::nullopt;
    } catch (...) {
        return std::nullopt;
    }
}
const char* capture_stamp_status_name(CaptureStampStatus status) noexcept {
    switch (status) {
        case CaptureStampStatus::Stamped: return "stamped";
        case CaptureStampStatus::NoTrustedTime: return "no_trusted_time";
        case CaptureStampStatus::Duplicate: return "duplicate";
        case CaptureStampStatus::NotLater: return "not_later";
        case CaptureStampStatus::BracketTooWide: return "bracket_too_wide";
        case CaptureStampStatus::InvalidInput: return "invalid_input";
        case CaptureStampStatus::Settling: return "settling";
    }
    return "unknown";
}

std::string ob_error_message(const ob::Error& error) {
    return std::string(error.what()) + " [function=" + error.getFunction() +
           ", args=" + error.getArgs() + ", status=" +
           std::to_string(static_cast<int>(error.getStatus())) + ", type=" +
           std::to_string(static_cast<int>(error.getExceptionType())) + "]";
}

bool finite_positive(float value) {
    return std::isfinite(value) && value > 0.0F;
}

bool same_intrinsics(
    const OBCameraIntrinsic& left,
    const OBCameraIntrinsic& right) {
    constexpr float tolerance = 1.0e-5F;
    return left.width == right.width && left.height == right.height &&
           std::fabs(left.fx - right.fx) <= tolerance &&
           std::fabs(left.fy - right.fy) <= tolerance &&
           std::fabs(left.cx - right.cx) <= tolerance &&
           std::fabs(left.cy - right.cy) <= tolerance;
}

double percentile(std::vector<double> values, double quantile) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const auto rank = static_cast<std::size_t>(
        std::ceil(quantile * static_cast<double>(values.size())));
    const auto index = std::min(
        values.size() - 1U,
        rank == 0U ? 0U : rank - 1U);
    return values[index];
}

std::shared_ptr<ob::VideoStreamProfile> find_exact_profile(
    const std::shared_ptr<ob::StreamProfileList>& profiles,
    int width,
    int height,
    int fps,
    OBFormat format) {
    for (std::uint32_t index = 0; index < profiles->getCount(); ++index) {
        auto profile = profiles->getProfile(index)->as<ob::VideoStreamProfile>();
        if (profile->getWidth() == static_cast<std::uint32_t>(width) &&
            profile->getHeight() == static_cast<std::uint32_t>(height) &&
            profile->getFps() == static_cast<std::uint32_t>(fps) &&
            profile->getFormat() == format) {
            return profile;
        }
    }
    return nullptr;
}

std::uint64_t absolute_difference(std::uint64_t left, std::uint64_t right) {
    return left > right ? left - right : right - left;
}

std::string diagnostic_depth_work_mode(const std::shared_ptr<ob::Device>& device) {
    try {
        if (!device->isPropertySupported(
                OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ)) {
            return "unsupported";
        }
        const auto mode = device->getCurrentDepthWorkMode();
        if (mode.name[0] == '\0') {
            return "empty";
        }
        return std::string(mode.name) + "(tag=" +
               std::to_string(static_cast<int>(mode.tag)) + ")";
    } catch (const ob::Error&) {
        return "read_error";
    } catch (const std::exception&) {
        return "read_error";
    }
}

std::string diagnostic_current_preset_name(const std::shared_ptr<ob::Device>& device) {
    try {
        const char* preset_name = device->getCurrentPresetName();
        if (preset_name == nullptr || preset_name[0] == '\0') {
            return "empty";
        }
        return preset_name;
    } catch (const ob::Error&) {
        return "unsupported_or_read_error";
    } catch (const std::exception&) {
        return "unsupported_or_read_error";
    }
}

std::string diagnostic_int_property(
    const std::shared_ptr<ob::Device>& device,
    OBPropertyID property_id) {
    try {
        if (!device->isPropertySupported(property_id, OB_PERMISSION_READ)) {
            return "unsupported";
        }
        return std::to_string(device->getIntProperty(property_id));
    } catch (const ob::Error&) {
        return "read_error";
    } catch (const std::exception&) {
        return "read_error";
    }
}

std::string diagnostic_float_property(
    const std::shared_ptr<ob::Device>& device,
    OBPropertyID property_id) {
    try {
        if (!device->isPropertySupported(property_id, OB_PERMISSION_READ)) {
            return "unsupported";
        }
        return std::to_string(device->getFloatProperty(property_id));
    } catch (const ob::Error&) {
        return "read_error";
    } catch (const std::exception&) {
        return "read_error";
    }
}

std::string diagnostic_bool_property(
    const std::shared_ptr<ob::Device>& device,
    OBPropertyID property_id) {
    try {
        if (!device->isPropertySupported(property_id, OB_PERMISSION_READ)) {
            return "unsupported";
        }
        return device->getBoolProperty(property_id) ? "true" : "false";
    } catch (const ob::Error&) {
        return "read_error";
    } catch (const std::exception&) {
        return "read_error";
    }
}

}  // namespace

struct OrbbecCapture::Impl {
    // stream_rtsp: every frame, shared immutable frame (no clone).
    bool media_stream = false;
    std::shared_ptr<ob::Context> context;
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Config> pipeline_config;
    std::shared_ptr<ob::Align> software_aligner;
    std::shared_ptr<ob::FormatConvertFilter> format_converter;
    // Runtime switch gemini_mjpeg_decode=nvv4l2 (approved, issues.md 3.5 A): hardware MJPG decode
    // replaces format_converter in the loop; null means the SDK's software decode.
    bool hw_mjpeg_decode = false;
    std::unique_ptr<HwMjpegDecoder> hw_decoder;
    // RGB <- aligned depth registration (B4-2); null outside the measured scope.
    std::shared_ptr<const perception::RgbDepthRegistration> registration;
    OBCameraIntrinsic aligned_depth_intrinsics{};
    yolo::DepthIntrinsics output_depth_intrinsics{};
    float depth_scale = 0.0F;
    bool pipeline_started = false;
    std::uint64_t total_timeouts = 0;
    std::uint64_t total_incomplete = 0;
    std::uint64_t total_alignment_failures = 0;
    std::uint64_t timestamp_regressions = 0;
    std::uint64_t color_depth_skew_us_max = 0;
    std::uint64_t previous_color_timestamp_us = 0;
    std::uint64_t previous_depth_timestamp_us = 0;
    std::uint64_t previous_color_index = 0;
    std::uint64_t previous_depth_index = 0;
    int previous_opencv_threads = 0;
    bool opencv_threads_limited = false;
    std::uint64_t native_stream_id = 0, native_sequence = 0;
    bool inference_handoff_first = false;
    CaptureAffinityMode capture_affinity_mode = CaptureAffinityMode::Disabled;
    // Runtime switch capture_sdk_threads_cpu (sdk_stream_thread_placement.hpp); null when off.
    std::unique_ptr<SdkStreamThreadPlacement> sdk_thread_placement;
    // Capture time of every frameset (capture_clock.hpp, runtime switch capture_min_path_us).
    // Present exactly when the R6 native tap is: a frameset the clock does not stamp is handed
    // neither to the Profile nor to the inference (capture_time_state.hpp).
    std::unique_ptr<CaptureClock> capture_clock;
    std::int64_t capture_min_path_us = 0;

    std::shared_ptr<ob::FrameSet> align_frameset(
        const std::shared_ptr<ob::FrameSet>& frameset) const {
        if (!software_aligner) {
            return frameset;
        }
        auto aligned = software_aligner->process(frameset);
        return aligned ? aligned->as<ob::FrameSet>() : nullptr;
    }
};

OrbbecCapture::OrbbecCapture(
    const yolo::Config& config,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> output_queue,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> encode_queue,
    std::shared_ptr<native_depth::Mailbox> native_depth_output,
    rns::SourceJournal* health_arrivals)
    : RgbdCapture("OrbbecCapture"),
      config_(config),
      output_queue_(std::move(output_queue)),
      encode_queue_(std::move(encode_queue)),
      native_depth_output_(std::move(native_depth_output)),
      health_arrivals_(health_arrivals),
      impl_(std::make_unique<Impl>()) {}

OrbbecCapture::~OrbbecCapture() {
    stop();
}

// The exact configuration this driver accepts: Gemini 338Le with a serial, MJPG colour, Y16
// depth, depth-to-colour alignment. The R6 native tap and health hooks need SW-D2C at 640x400.
bool OrbbecCapture::config_is_valid() const {
    const std::string alignment_mode = config_.camera_alignment_mode();
    // Current health integration is explicitly the Gemini native-depth path;
    // no synthetic stream IDs for HW-D2C or another camera driver.
    if (health_arrivals_ && (!native_depth_output_ || !health_arrivals_->valid() ||
        health_arrivals_->kind() != rns::SourceKind::DepthArrival)) return false;
    // Never relabel hardware-aligned pixels as pre-D2C samples or silently
    // switch the caller's camera mode. This output is explicitly opt-in.
    if (native_depth_output_ &&
        (alignment_mode != "sw_d2c" || config_.camera_depth_width() != 640 ||
         config_.camera_depth_height() != 400)) return false;
    return config_.camera_driver() == "orbbec_gemini338le" &&
           !config_.camera_serial().empty() &&
           config_.camera_color_width() > 0 &&
           config_.camera_color_height() > 0 &&
           config_.camera_color_fps() > 0 &&
           config_.camera_color_device_format() == "mjpg" &&
           config_.camera_depth_width() > 0 &&
           config_.camera_depth_height() > 0 &&
           config_.camera_depth_fps() > 0 &&
           config_.camera_depth_device_format() == "y16" &&
           (alignment_mode == "hw_d2c" || alignment_mode == "sw_d2c") &&
           config_.camera_align_to_color() &&
           config_.camera_output_width() > 0 &&
           config_.camera_output_height() > 0 &&
           config_.camera_output_fps() > 0;
}

// See the file header. Every failure path joins the worker, ends the native stream, stops the
// SDK pipeline and restores the OpenCV thread count before returning false.
bool OrbbecCapture::start() {
    if (running_.load()) {
        return true;
    }
    if (!output_queue_) {
        LOG_ERROR_M(name_, "cannot start without output queue");
        return false;
    }
    if (!config_is_valid()) {
        LOG_ERROR_M(name_, "invalid or incomplete Gemini 338Le camera configuration");
        return false;
    }
    if (worker_.joinable() || impl_->pipeline_started) {
        stop();
        impl_ = std::make_unique<Impl>();
    }
    // A normal stop leaves the SDK pipeline object allocated. Reset all
    // per-stream timestamp/stat state before every new start so a device clock
    // epoch change cannot be compared against the previous session.
    if (!worker_.joinable() && !impl_->pipeline_started && impl_->pipeline) {
        impl_ = std::make_unique<Impl>();
    }

    try {
        // Parse once before any SDK/device access. Invalid opt-ins fail closed.
        impl_->inference_handoff_first = orbbec_capture_detail::inference_handoff_first(
            std::getenv("PERCEPTION_INFERENCE_HANDOFF_FIRST"));
        impl_->capture_affinity_mode = parse_capture_affinity_control(
            std::getenv("PERCEPTION_CAPTURE_AFFINITY_PIN2"));
        const int sdk_threads_cpu = parse_sdk_stream_threads_cpu(
            std::getenv("PERCEPTION_CAPTURE_SDK_THREADS_CPU"));
        impl_->sdk_thread_placement.reset();
        if (sdk_threads_cpu >= 0) {
            impl_->sdk_thread_placement =
                std::make_unique<SdkStreamThreadPlacement>(sdk_threads_cpu, SDK_START_THREADS);
        }
        publish_sdk_thread_placement(SdkThreadPlacementState::Disabled);
        impl_->capture_clock.reset();
        if (native_depth_output_) {
            CaptureClockOptions clock_options;
            clock_options.min_path_us = parse_capture_min_path_us(std::getenv("PERCEPTION_CAPTURE_MIN_PATH_US"));
            impl_->capture_min_path_us = clock_options.min_path_us;
            impl_->capture_clock = std::make_unique<CaptureClock>(clock_options);
            if (!impl_->capture_clock->valid()) throw std::runtime_error("capture clock options are not valid");
            LOG_INFO_M(name_, "CAPTURE_CLOCK_CONFIG min_path_us=" << clock_options.min_path_us
                << " guard_us=" << clock_options.guard_us
                << " stream_settle_ms=" << clock_options.stream_settle_ns / 1000000
                << " envelope_rise_ppm=" << clock_options.envelope_rise_ppm
                << " sdk_clock_type=monotonic source=camera_frame_timestamp");
        }
        if (const char* decode = std::getenv("PERCEPTION_GEMINI_MJPEG_DECODE")) {
            const std::string text(decode);
            if (text != "sdk" && text != "nvv4l2") {
                throw std::runtime_error("PERCEPTION_GEMINI_MJPEG_DECODE must be exactly sdk or nvv4l2");
            }
            impl_->hw_mjpeg_decode = text == "nvv4l2";
        }
        impl_->media_stream = config_.hevc_stream_sink();
        LOG_INFO_M(name_, "MEDIA_HANDOFF_CONFIG stream_rtsp=" << static_cast<int>(impl_->media_stream)
            << " shared_frame=" << static_cast<int>(impl_->media_stream) << " every_frame=1");
        LOG_INFO_M(name_, "CAPTURE_HANDOFF_CONFIG inference_first="
            << static_cast<int>(impl_->inference_handoff_first)
            << " capture_time_scope=sdk_return_to_inference_handoff_ready"
            << " camera_processing_scope=sdk_return_to_selected_handoff_work_complete");
        impl_->context = std::make_shared<ob::Context>();
        if (impl_->capture_clock) {
            // The SDK's system and global frame timestamps on CLOCK_MONOTONIC_RAW instead of the
            // wall clock (host-side setting of this process; measured 2026-10-14): a step of the
            // wall clock cannot reach a capture time. Refused, not assumed, when it does not take.
            impl_->context->setTimestampClockType(OB_CLOCK_TYPE_MONOTONIC);
            if (impl_->context->getTimestampClockType() != OB_CLOCK_TYPE_MONOTONIC)
                throw std::runtime_error("the camera SDK did not take the monotonic timestamp clock type");
        }
        auto devices = impl_->context->queryDeviceList();
        // PSC-3 / A19-BOOT-2 (camera_identity.hpp): exactly one camera is listed and it carries
        // the configured serial, or the start is refused before any device is opened.
        std::vector<std::string> listed_serials;
        for (std::uint32_t index = 0; index < devices->getCount(); ++index) {
            const char* const serial = devices->getSerialNumber(index);
            listed_serials.emplace_back(serial ? serial : "");
        }
        const auto identity = check_camera_identity(config_.camera_serial(), listed_serials);
        if (!identity.ok) throw std::runtime_error("PSC-3 " + identity.reason);

        impl_->device = devices->getDeviceBySN(config_.camera_serial().c_str());
        auto device_info = impl_->device->getDeviceInfo();
        const std::string model = device_info->getName();
        if (model.find("Gemini 338Le") == std::string::npos) {
            throw std::runtime_error("unexpected Orbbec model: " + model);
        }
        // Read all SDK-owned identity strings before the worker starts. Any
        // getter failure is then cleaned up without racing a live capture loop.
        const std::string device_serial = device_info->getSerialNumber();
        const std::string firmware_version = device_info->getFirmwareVersion();
        const std::string hardware_version = device_info->getHardwareVersion();
        const std::string connection_type = device_info->getConnectionType();
        const std::string device_uid = device_info->getUid();
        const std::string device_ip = device_info->getIpAddress();
        impl_->pipeline = std::make_shared<ob::Pipeline>(impl_->device);

        auto color_profile = find_exact_profile(
            impl_->pipeline->getStreamProfileList(OB_SENSOR_COLOR),
            config_.camera_color_width(),
            config_.camera_color_height(),
            config_.camera_color_fps(),
            OB_FORMAT_MJPG);
        if (!color_profile) {
            throw std::runtime_error("configured Gemini color profile is unavailable");
        }
        const bool software_d2c = config_.camera_alignment_mode() == "sw_d2c";
        auto depth_profiles = software_d2c
            ? impl_->pipeline->getStreamProfileList(OB_SENSOR_DEPTH)
            : impl_->pipeline->getD2CDepthProfileList(color_profile, ALIGN_D2C_HW_MODE);
        auto depth_profile = find_exact_profile(
            depth_profiles,
            config_.camera_depth_width(),
            config_.camera_depth_height(),
            config_.camera_depth_fps(),
            OB_FORMAT_Y16);
        if (!depth_profile) {
            throw std::runtime_error(
                "configured Gemini " + config_.camera_alignment_mode() +
                " depth profile is unavailable");
        }

        impl_->pipeline_config = std::make_shared<ob::Config>();
        impl_->pipeline_config->enableStream(color_profile);
        impl_->pipeline_config->enableStream(depth_profile);
        impl_->pipeline_config->setAlignMode(
            software_d2c ? ALIGN_DISABLE : ALIGN_D2C_HW_MODE);
        impl_->pipeline_config->setFrameAggregateOutputMode(
            OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        if (software_d2c) {
            impl_->software_aligner = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            impl_->software_aligner->setMatchTargetResolution(true);
        }
        impl_->pipeline->enableFrameSync();
        {
            // The SDK creates its stream threads on this thread during the call. With the
            // placement switched on, this thread is narrowed to the target CPU for the call
            // only; the guard gives it its own set back and runs the self-check, also when the
            // call throws. A failed step is logged and raises a Status reason, and the capture
            // starts all the same -- unless this thread did not get its own set back (below).
            SdkStreamThreadPlacement* const placement = impl_->sdk_thread_placement.get();
            const auto placement_log = [this](const SdkThreadPlacementRecord& record) {
                if (record.success) LOG_INFO_M(name_, record);
                else LOG_ERROR_M(name_, record);
            };
            SdkThreadPlacementEndGuard placement_end(placement, placement_log);
            if (placement) {
                (void)emit_sdk_thread_placement(placement->begin(), placement_log);
                // An earlier start of this process left this thread narrowed: no pipeline is
                // started from it either.
                if (!placement->needs_end() && placement->caller_restore_unconfirmed()) {
                    throw std::runtime_error(
                        "SDK stream thread placement: an earlier start left the calling thread narrowed; not started");
                }
            }
            impl_->pipeline->start(impl_->pipeline_config);
        }
        impl_->pipeline_started = true;
        if (impl_->sdk_thread_placement) {
            publish_sdk_thread_placement(impl_->sdk_thread_placement->placed() ? SdkThreadPlacementState::Placed
                                                                               : SdkThreadPlacementState::Fault);
            // This thread is still narrowed to the one CPU, or that cannot be ruled out: the
            // capture loop created below, and every thread the caller creates after this
            // returns (the supervisor's process and heartbeat threads), would inherit it. The
            // start ends here, before the capture loop exists; the cleanup below stops the
            // pipeline, the end record above is the evidence. Threads that were started before
            // the camera (inference, the owners) are not affected. Nothing else is done about
            // this thread's set: the OS refused it.
            if (impl_->sdk_thread_placement->caller_restore_unconfirmed()) {
                throw std::runtime_error(
                    "SDK stream thread placement: the calling thread's own CPU set is not confirmed restored; "
                    "the capture loop is not created");
            }
        }

        // Validate the exact runtime output before exposing the capture as ready.
        auto warmup_input = impl_->pipeline->waitForFrameset(FRAME_WAIT_TIMEOUT_MS);
        if (!warmup_input || !warmup_input->getColorFrame() ||
            !warmup_input->getDepthFrame()) {
            throw std::runtime_error("Gemini warm-up frameset is incomplete");
        }
        if (software_d2c) {
            auto raw_color = warmup_input->getColorFrame();
            auto raw_depth = warmup_input->getDepthFrame();
            const std::size_t expected_raw_depth_bytes =
                static_cast<std::size_t>(raw_depth->getWidth()) *
                static_cast<std::size_t>(raw_depth->getHeight()) *
                sizeof(std::uint16_t);
            if (raw_color->getWidth() !=
                    static_cast<std::uint32_t>(config_.camera_color_width()) ||
                raw_color->getHeight() !=
                    static_cast<std::uint32_t>(config_.camera_color_height()) ||
                raw_color->getFormat() != OB_FORMAT_MJPG ||
                raw_depth->getWidth() !=
                    static_cast<std::uint32_t>(config_.camera_depth_width()) ||
                raw_depth->getHeight() !=
                    static_cast<std::uint32_t>(config_.camera_depth_height()) ||
                raw_depth->getFormat() != OB_FORMAT_Y16 ||
                !raw_color->getData() || !raw_depth->getData() ||
                raw_depth->getDataSize() != expected_raw_depth_bytes ||
                raw_color->getTimeStampUs() == 0 ||
                raw_depth->getTimeStampUs() == 0 ||
                absolute_difference(
                    raw_color->getTimeStampUs(), raw_depth->getTimeStampUs()) >
                    MAX_COLOR_DEPTH_SKEW_US) {
                throw std::runtime_error("Gemini SW-D2C raw warm-up contract mismatch");
            }
        }
        auto warmup = impl_->align_frameset(warmup_input);
        if (!warmup || !warmup->getDepthFrame()) {
            throw std::runtime_error("Gemini aligned warm-up frameset is incomplete");
        }
        // SW alignment must not replace or transform the authoritative MJPG
        // color frame. Only the aligned depth output joins that raw color frame.
        auto warm_color = software_d2c
            ? warmup_input->getColorFrame()
            : warmup->getColorFrame();
        auto warm_depth = warmup->getDepthFrame();
        if (!warm_color) {
            throw std::runtime_error("Gemini warm-up color frame is incomplete");
        }
        if (warm_color->getWidth() !=
                static_cast<std::uint32_t>(config_.camera_color_width()) ||
            warm_color->getHeight() !=
                static_cast<std::uint32_t>(config_.camera_color_height()) ||
            warm_depth->getWidth() != warm_color->getWidth() ||
            warm_depth->getHeight() != warm_color->getHeight() ||
            warm_color->getFormat() != OB_FORMAT_MJPG ||
            warm_depth->getFormat() != OB_FORMAT_Y16) {
            throw std::runtime_error("Gemini runtime frame shape/format violates RGB-D contract");
        }
        const std::size_t expected_depth_bytes =
            static_cast<std::size_t>(warm_depth->getWidth()) *
            static_cast<std::size_t>(warm_depth->getHeight()) * sizeof(std::uint16_t);
        if (!warm_depth->getData() ||
            warm_depth->getDataSize() != expected_depth_bytes) {
            throw std::runtime_error("Gemini aligned-depth buffer size violates RGB-D contract");
        }

        impl_->aligned_depth_intrinsics =
            warm_depth->getStreamProfile()->as<ob::VideoStreamProfile>()->getIntrinsic();
        if (!finite_positive(impl_->aligned_depth_intrinsics.fx) ||
            !finite_positive(impl_->aligned_depth_intrinsics.fy) ||
            !std::isfinite(impl_->aligned_depth_intrinsics.cx) ||
            !std::isfinite(impl_->aligned_depth_intrinsics.cy) ||
            impl_->aligned_depth_intrinsics.cx < 0.0F ||
            impl_->aligned_depth_intrinsics.cy < 0.0F ||
            impl_->aligned_depth_intrinsics.cx >=
                static_cast<float>(warm_depth->getWidth()) ||
            impl_->aligned_depth_intrinsics.cy >=
                static_cast<float>(warm_depth->getHeight()) ||
            impl_->aligned_depth_intrinsics.width !=
                static_cast<std::int16_t>(warm_depth->getWidth()) ||
            impl_->aligned_depth_intrinsics.height !=
                static_cast<std::int16_t>(warm_depth->getHeight())) {
            throw std::runtime_error("Gemini reported invalid aligned-depth intrinsics");
        }
        if (software_d2c) {
            const OBCameraIntrinsic color_intrinsics =
                warm_color->getStreamProfile()
                    ->as<ob::VideoStreamProfile>()
                    ->getIntrinsic();
            if (!same_intrinsics(color_intrinsics, impl_->aligned_depth_intrinsics)) {
                LOG_ERROR_M(
                    name_,
                    "Gemini SW-D2C runtime intrinsics mismatch color="
                        << color_intrinsics.fx << "," << color_intrinsics.fy << ","
                        << color_intrinsics.cx << "," << color_intrinsics.cy << ","
                        << color_intrinsics.width << "x" << color_intrinsics.height
                        << " aligned_depth=" << impl_->aligned_depth_intrinsics.fx << ","
                        << impl_->aligned_depth_intrinsics.fy << ","
                        << impl_->aligned_depth_intrinsics.cx << ","
                        << impl_->aligned_depth_intrinsics.cy << ","
                        << impl_->aligned_depth_intrinsics.width << "x"
                        << impl_->aligned_depth_intrinsics.height);
                throw std::runtime_error(
                    "Gemini SW-D2C aligned depth does not carry the runtime color intrinsics");
            }
        }
        const float depth_scale_mm = warm_depth->getValueScale();
        impl_->depth_scale =
            static_cast<float>(depth_scale_mm * MILLIMETRES_TO_METRES);
        if (!finite_positive(impl_->depth_scale)) {
            throw std::runtime_error("Gemini reported a non-positive depth scale");
        }

        const float output_scale_x = static_cast<float>(config_.camera_output_width()) /
                                     static_cast<float>(warm_depth->getWidth());
        const float output_scale_y = static_cast<float>(config_.camera_output_height()) /
                                     static_cast<float>(warm_depth->getHeight());
        if (!finite_positive(output_scale_x) || !finite_positive(output_scale_y) ||
            std::fabs(output_scale_x - output_scale_y) > 1.0e-6F ||
            config_.camera_output_fps() != config_.camera_color_fps()) {
            throw std::runtime_error(
                "Gemini software output must be a uniform-scale transform at the native FPS");
        }
        impl_->output_depth_intrinsics.fx =
            impl_->aligned_depth_intrinsics.fx * output_scale_x;
        impl_->output_depth_intrinsics.fy =
            impl_->aligned_depth_intrinsics.fy * output_scale_y;
        // cv::resize maps pixel centres, so the principal point must use the
        // same half-pixel transform. A plain cx*scale introduces a systematic
        // RGB/depth ray offset in the canonical 1080p coordinate system.
        // (1080p was an earlier output size; R6 outputs the 1280x800 colour raster itself, scale 1.)
        impl_->output_depth_intrinsics.cx =
            (impl_->aligned_depth_intrinsics.cx + 0.5F) * output_scale_x - 0.5F;
        impl_->output_depth_intrinsics.cy =
            (impl_->aligned_depth_intrinsics.cy + 0.5F) * output_scale_y - 0.5F;
        impl_->output_depth_intrinsics.width = config_.camera_output_width();
        impl_->output_depth_intrinsics.height = config_.camera_output_height();
        if (!finite_positive(impl_->output_depth_intrinsics.fx) ||
            !finite_positive(impl_->output_depth_intrinsics.fy) ||
            impl_->output_depth_intrinsics.cx < 0.0F ||
            impl_->output_depth_intrinsics.cy < 0.0F ||
            impl_->output_depth_intrinsics.cx >=
                static_cast<float>(impl_->output_depth_intrinsics.width) ||
            impl_->output_depth_intrinsics.cy >=
                static_cast<float>(impl_->output_depth_intrinsics.height)) {
            throw std::runtime_error("Gemini transformed output intrinsics are invalid");
        }
        impl_->format_converter = std::make_shared<ob::FormatConvertFilter>();
        if (impl_->hw_mjpeg_decode) {
            // Start-up self-check on the warm-up frame: the hardware decode must agree with the
            // SDK's software decode of the same bytes within the accepted limits (user,
            // 2026-10-05: mean <= 1.5 levels, pixels over 8 levels <= 0.1 %), else no start.
            const int w = static_cast<int>(warm_color->getWidth());
            const int h = static_cast<int>(warm_color->getHeight());
            auto decoder = std::make_unique<HwMjpegDecoder>();
            std::string error;
            if (!decoder->open(w, h, error)) {
                throw std::runtime_error("Gemini hardware MJPG decode unavailable: " + error);
            }
            impl_->format_converter->setFormatConvertType(FORMAT_MJPG_TO_BGR);
            const auto sdk_bgr = impl_->format_converter->process(warm_color);
            if (!sdk_bgr || sdk_bgr->getFormat() != OB_FORMAT_BGR ||
                sdk_bgr->getDataSize() != static_cast<std::uint32_t>(w * h * 3)) {
                throw std::runtime_error("Gemini warm-up MJPG-to-BGR conversion failed");
            }
            cv::Mat hw_bgr;
            HwMjpegDecoder::Timing timing;
            if (!decoder->decode(static_cast<const std::uint8_t*>(warm_color->getData()),
                                 warm_color->getDataSize(), hw_bgr, error, &timing)) {
                throw std::runtime_error("Gemini hardware MJPG decode failed on the warm-up frame: " + error);
            }
            const auto agreement = compare_bgr(hw_bgr, cv::Mat(h, w, CV_8UC3, sdk_bgr->getData()));
            LOG_INFO_M(name_, "GEMINI_MJPEG_DECODE mode=nvv4l2 selfcheck_mean_abs=" << agreement.mean_abs
                << " selfcheck_over8_share=" << agreement.over8_share << " selfcheck_max_abs=" << agreement.max_abs
                << " limit_mean_abs=" << kHwDecodeMeanAbsLimit << " limit_over8_share=" << kHwDecodeOver8Limit
                << " hw_ms=" << timing.hw_us / 1000.0 << " convert_ms=" << timing.convert_us / 1000.0);
            if (agreement.mean_abs > kHwDecodeMeanAbsLimit || agreement.over8_share > kHwDecodeOver8Limit) {
                throw std::runtime_error("Gemini hardware MJPG decode disagrees with the SDK decode of the warm-up frame");
            }
            impl_->hw_decoder = std::move(decoder);
        } else {
            LOG_INFO_M(name_, "GEMINI_MJPEG_DECODE mode=sdk");
        }

        if (software_d2c) {
            // Session calibration for base_link object geometry: the aligned depth
            // is on the colour raster, so colour intrinsics/distortion deproject it
            // and the native depth->colour extrinsic links it to the depth camera.
            const auto raw_color_profile =
                warmup_input->getColorFrame()->getStreamProfile()->as<ob::VideoStreamProfile>();
            const auto raw_depth_profile =
                warmup_input->getDepthFrame()->getStreamProfile()->as<ob::VideoStreamProfile>();
            const OBCameraIntrinsic ci = raw_color_profile->getIntrinsic();
            const OBCameraDistortion cd = raw_color_profile->getDistortion();
            const OBExtrinsic ex = raw_depth_profile->getExtrinsicTo(raw_color_profile);
            yolo::RgbdCalibration calibration;
            calibration.colour = {ci.fx, ci.fy, ci.cx, ci.cy, static_cast<int>(ci.width), static_cast<int>(ci.height)};
            calibration.colour_distortion_model = static_cast<int>(cd.model);
            calibration.colour_distortion = {cd.k1, cd.k2, cd.k3, cd.k4, cd.k5, cd.k6, cd.p1, cd.p2};
            for (std::size_t i = 0; i < 9; ++i) calibration.depth_to_colour_R[i] = ex.rot[i];
            for (std::size_t i = 0; i < 3; ++i) calibration.depth_to_colour_t_m[i] = ex.trans[i] * 0.001F;  // SDK mm
            calibration.aligned_width = static_cast<int>(warm_depth->getWidth());
            calibration.aligned_height = static_cast<int>(warm_depth->getHeight());
            calibration.output_width = config_.camera_output_width();
            calibration.output_height = config_.camera_output_height();
            calibration.device_name = model;
            calibration.device_serial = device_serial;
            calibration.firmware = firmware_version;
            calibration.sdk_version = std::to_string(ob::Version::getMajor()) + "." +
                std::to_string(ob::Version::getMinor()) + "." + std::to_string(ob::Version::getPatch());
            // Registration gate = the configuration measured on .23 (alignprobe v2
            // and b42 checkerboard): SDK 2.9.3 software aligner, Gemini 338Le
            // firmware 1.8.17, colour 1280x800 MJPG with BROWN_CONRADY_K6 and
            // k4..k6 = 0, aligned == colour == published raster. Anything else is
            // an unverified configuration: no registration (object geometry then
            // degrades), never an assumed model.
            std::string gate;
            if (calibration.sdk_version != "2.9.3") gate = "sdk " + calibration.sdk_version + " not measured";
            else if (model.find("Gemini 338Le") == std::string::npos) gate = "model " + model + " not measured";
            else if (firmware_version != "1.8.17") gate = "firmware " + firmware_version + " not measured";
            else if (cd.model != OB_DISTORTION_BROWN_CONRADY_K6 || cd.k4 != 0 || cd.k5 != 0 || cd.k6 != 0)
                gate = "colour distortion model " + std::to_string(static_cast<int>(cd.model)) + " with k4..k6 not measured";
            else if (ci.width != 1280 || ci.height != 800 || warmup_input->getColorFrame()->getFormat() != OB_FORMAT_MJPG ||
                     calibration.aligned_width != ci.width || calibration.aligned_height != ci.height ||
                     calibration.output_width != ci.width || calibration.output_height != ci.height)
                gate = "raster/format combination not measured";
            if (gate.empty()) {
                try {
                    const auto build_begin = std::chrono::steady_clock::now();
                    auto registration = std::make_shared<const perception::RgbDepthRegistration>(
                        static_cast<int>(ci.width), static_cast<int>(ci.height), ci.fx, ci.fy, ci.cx, ci.cy,
                        perception::StandardDistortion{cd.k1, cd.k2, cd.k3, cd.p1, cd.p2});
                    const double build_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_begin).count();
                    calibration.registration = registration;
                    calibration.registration_status = "active";
                    impl_->registration = registration;
                    LOG_INFO_M(name_, "[RgbDepthRegistration] status=active max_shift_px=" << registration->max_shift_px()
                        << " outside_pixels=" << registration->outside_pixels() << " max_forward_residual_px="
                        << registration->max_forward_residual_px() << " build_ms=" << build_ms << " device=\"" << model
                        << "\" serial=" << device_serial << " firmware=" << firmware_version << " sdk=" << calibration.sdk_version);
                } catch (const std::invalid_argument& e) {
                    gate = std::string("model not invertible: ") + e.what();
                }
            }
            if (!gate.empty()) {
                calibration.registration_status = "unavailable: " + gate;
                LOG_WARN_M(name_, "[RgbDepthRegistration] status=unavailable reason=\"" << gate
                    << "\" depth stays on the SDK aligned raster; object geometry degrades");
            }
            {
                std::lock_guard<std::mutex> lock(calibration_mutex_);
                calibration_ = calibration;
            }
            LOG_INFO_M(name_, "[RgbdCalibration] colour_fx=" << ci.fx << " colour_fy=" << ci.fy << " colour_cx=" << ci.cx
                << " colour_cy=" << ci.cy << " colour=" << ci.width << "x" << ci.height << " distortion_model="
                << static_cast<int>(cd.model) << " k1=" << cd.k1 << " k2=" << cd.k2 << " k3=" << cd.k3 << " k4=" << cd.k4
                << " k5=" << cd.k5 << " k6=" << cd.k6 << " p1=" << cd.p1 << " p2=" << cd.p2
                << " depth_to_colour_R=[" << ex.rot[0] << "," << ex.rot[1] << "," << ex.rot[2] << "," << ex.rot[3] << ","
                << ex.rot[4] << "," << ex.rot[5] << "," << ex.rot[6] << "," << ex.rot[7] << "," << ex.rot[8]
                << "] depth_to_colour_t_mm=[" << ex.trans[0] << "," << ex.trans[1] << "," << ex.trans[2] << "]");
        }

        // Capture the device-owned depth state without changing it. These
        // values are diagnostic rather than startup requirements: firmware may
        // expose only a subset, so unsupported/read failures must remain visible
        // without silently changing capture behaviour or blocking the start.
        const std::string depth_work_mode = diagnostic_depth_work_mode(impl_->device);
        const std::string device_preset = diagnostic_current_preset_name(impl_->device);
        const std::string depth_precision_level = diagnostic_int_property(
            impl_->device, OB_PROP_DEPTH_PRECISION_LEVEL_INT);
        const std::string depth_unit_flexible = diagnostic_float_property(
            impl_->device, OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT);
        const std::string hardware_disparity_to_depth = diagnostic_bool_property(
            impl_->device, OB_PROP_DISPARITY_TO_DEPTH_BOOL);
        const std::string sdk_disparity_to_depth = diagnostic_bool_property(
            impl_->device, OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL);
        const std::string disparity_range_config = diagnostic_int_property(
            impl_->device, OB_PROP_DISP_SEARCH_RANGE_MODE_INT);
        const std::string disparity_range_current = diagnostic_int_property(
            impl_->device, OB_PROP_CURRENT_DISP_SEARCH_RANGE_MODE_INT);
        const std::string disparity_offset_config = diagnostic_int_property(
            impl_->device, OB_PROP_DISP_SEARCH_OFFSET_INT);
        const std::string disparity_offset_current = diagnostic_int_property(
            impl_->device, OB_PROP_CURRENT_DISP_SEARCH_OFFSET_INT);
        const std::string hardware_distortion = diagnostic_bool_property(
            impl_->device, OB_PROP_HARDWARE_DISTORTION_SWITCH_BOOL);
        const std::string d2c_preprocess = diagnostic_bool_property(
            impl_->device, OB_PROP_D2C_PREPROCESS_BOOL);
        const std::string hardware_depth_align = diagnostic_bool_property(
            impl_->device, OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL);
        const std::string hardware_depth_align_mode = diagnostic_int_property(
            impl_->device, OB_PROP_DEPTH_ALIGN_HARDWARE_MODE_INT);

        LOG_INFO_M(
            name_,
            "[CameraIdentity] driver=orbbec_gemini338le model=" << model
                << " serial=" << device_serial
                << " firmware=" << firmware_version
                << " hardware=" << hardware_version
                << " sdk_version=" << ob::Version::getMajor() << "."
                << ob::Version::getMinor() << "." << ob::Version::getPatch()
                << " transport=" << connection_type
                << " uid=" << device_uid
                << " ip=" << device_ip);
        LOG_INFO_M(
            name_,
            "[CameraProfile] native_color=" << config_.camera_color_width() << "x"
                << config_.camera_color_height() << "@" << config_.camera_color_fps()
                << " format=" << config_.camera_color_device_format()
                << " depth_input=" << config_.camera_depth_width() << "x"
                << config_.camera_depth_height() << "@" << config_.camera_depth_fps()
                << " format=" << config_.camera_depth_device_format()
                << " alignment_mode=" << config_.camera_alignment_mode()
                << " aligned_native_depth=" << impl_->aligned_depth_intrinsics.width << "x"
                << impl_->aligned_depth_intrinsics.height
                << " canonical_output=" << config_.camera_output_width() << "x"
                << config_.camera_output_height() << "@" << config_.camera_output_fps()
                << " transform=uniform_scale_" << output_scale_x
                << " native_1080p=false align_to_color=true depth_scale_m="
                << impl_->depth_scale
                << " output_fx=" << impl_->output_depth_intrinsics.fx
                << " output_fy=" << impl_->output_depth_intrinsics.fy
                << " output_cx=" << impl_->output_depth_intrinsics.cx
                << " output_cy=" << impl_->output_depth_intrinsics.cy);
        LOG_INFO_M(
            name_,
            "[DepthDeviceState] read_only=true work_mode=" << depth_work_mode
                << " device_preset=" << device_preset
                << " precision_level=" << depth_precision_level
                << " depth_unit_flexible=" << depth_unit_flexible
                << " frame_value_scale_m=" << impl_->depth_scale
                << " hw_disparity_to_depth=" << hardware_disparity_to_depth
                << " sdk_disparity_to_depth=" << sdk_disparity_to_depth
                << " disparity_range_config=" << disparity_range_config
                << " disparity_range_current=" << disparity_range_current
                << " disparity_offset_config=" << disparity_offset_config
                << " disparity_offset_current=" << disparity_offset_current
                << " hardware_distortion=" << hardware_distortion
                << " d2c_preprocess=" << d2c_preprocess
                << " hardware_depth_align=" << hardware_depth_align
                << " hardware_depth_align_mode=" << hardware_depth_align_mode);
        LOG_INFO_M(
            name_,
            "RGB exposure/gain metadata remains diagnostic-unavailable in M2; "
                << "the SDK documents model-dependent exposure units");

        impl_->previous_opencv_threads = cv::getNumThreads();
        // Optional override for the capture-lifecycle OpenCV pool size (default unchanged: 2).
        // Only "1" or "2" are accepted; the enforcement check below is kept for either value.
        int gemini_opencv_threads = GEMINI_OPENCV_THREADS;
        if (const char* override_threads = std::getenv("PERCEPTION_GEMINI_OPENCV_THREADS")) {
            const std::string text(override_threads);
            if (text != "1" && text != "2") {
                throw std::runtime_error("PERCEPTION_GEMINI_OPENCV_THREADS must be exactly 1 or 2");
            }
            gemini_opencv_threads = text == "1" ? 1 : 2;
        }
        cv::setNumThreads(gemini_opencv_threads);
        impl_->opencv_threads_limited = true;
        const int effective_opencv_threads = cv::getNumThreads();
        if (effective_opencv_threads != gemini_opencv_threads) {
            throw std::runtime_error(
                "failed to enforce Gemini OpenCV thread limit expected=" +
                std::to_string(gemini_opencv_threads) + " actual=" +
                std::to_string(effective_opencv_threads));
        }
        LOG_INFO_M(
            name_,
            "[OpenCvThreads] previous=" << impl_->previous_opencv_threads
                << " effective=" << effective_opencv_threads
                << " default=" << GEMINI_OPENCV_THREADS
                << " scope=gemini_capture_lifecycle");

        frame_counter_.store(0);
        if (native_depth_output_) {
            impl_->native_stream_id = native_depth_output_->begin_stream();
            impl_->native_sequence = 0;
        }
        running_.store(true);
        update_heartbeat();
        worker_ = std::thread(&OrbbecCapture::capture_loop, this);
        LOG_INFO_M(name_, "started");
        return true;
    } catch (const ob::Error& error) {
        LOG_ERROR_M(name_, "failed to start Gemini pipeline: " << ob_error_message(error));
    } catch (const std::exception& error) {
        LOG_ERROR_M(name_, "failed to start Gemini pipeline: " << error.what());
    }

    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    // No capture loop checks the placement now. The fault stays published only when this
    // thread did not get its own set back: that is still true of the process.
    if (impl_->sdk_thread_placement && !impl_->sdk_thread_placement->caller_restore_unconfirmed()) {
        publish_sdk_thread_placement(SdkThreadPlacementState::Disabled);
    }
    if (native_depth_output_) native_depth_output_->end_stream();
    if (impl_->pipeline_started && impl_->pipeline) {
        try {
            ++sdk_stop_attempts_;
            impl_->pipeline->stop();
            ++sdk_stop_successes_;
        } catch (const std::exception& error) {
            ++sdk_stop_failures_;
            LOG_WARN_M(name_, "pipeline cleanup after start failure failed: " << error.what());
        } catch (...) {
            ++sdk_stop_failures_;
            throw;
        }
        impl_->pipeline_started = false;
    }
    if (impl_->opencv_threads_limited) {
        cv::setNumThreads(impl_->previous_opencv_threads);
        impl_->opencv_threads_limited = false;
    }
    return false;
}

std::optional<yolo::RgbdCalibration> OrbbecCapture::rgbd_calibration() const {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    return calibration_;
}

// Joins cap_loop first (it reads impl_), then stops the SDK pipeline and restores the OpenCV
// thread count. Safe to call twice.
void OrbbecCapture::stop() {
    {
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        calibration_.reset();
    }
    const bool was_running = running_.exchange(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    // The capture loop reads impl_->registration and the decoder: release them only after the join.
    if (impl_) impl_->registration.reset();
    if (impl_ && impl_->hw_decoder) impl_->hw_decoder.reset();
    if (native_depth_output_) native_depth_output_->end_stream();

    const bool had_pipeline = impl_ && impl_->pipeline_started;
    if (had_pipeline) {
        try {
            ++sdk_stop_attempts_;
            impl_->pipeline->stop();
            ++sdk_stop_successes_;
        } catch (const ob::Error& error) {
            ++sdk_stop_failures_;
            LOG_WARN_M(name_, "pipeline stop failed: " << ob_error_message(error));
        } catch (const std::exception& error) {
            ++sdk_stop_failures_;
            LOG_WARN_M(name_, "pipeline stop failed: " << error.what());
        } catch (...) {
            ++sdk_stop_failures_;
            throw;
        }
        impl_->pipeline_started = false;
    }
    if (impl_ && impl_->opencv_threads_limited) {
        const int restore_threads = impl_->previous_opencv_threads;
        cv::setNumThreads(restore_threads);
        impl_->opencv_threads_limited = false;
        LOG_INFO_M(name_, "[OpenCvThreads] restored=" << cv::getNumThreads());
    }
    if (was_running || had_pipeline) {
        LOG_INFO_M(name_, "stopped");
    }
}

// The capture thread (cap_loop); the file header lists the per-frame steps and the exits.
void OrbbecCapture::capture_loop() {
    set_current_thread_name("cap_loop");
    CaptureAffinityControl capture_affinity(impl_->capture_affinity_mode);
    const auto affinity_log = [&](const CaptureAffinityRecord& record) {
        LOG_INFO_M(name_, record);
    };
    // Restoration and its logging are noexcept and affect this worker only.
    // Disabled (environment absent) never reads, sets, or logs affinity.
    CaptureAffinityRestoreGuard affinity_restore(capture_affinity, affinity_log);
    // Placement of the SDK's stream threads (nothing when switched off): checked once a second
    // on every turn of this loop, frame or no frame. Once the loop is left nobody checks any
    // more, so Status stops reporting on it.
    SdkThreadPlacementWatch sdk_thread_watch(impl_->sdk_thread_placement.get());
    const auto sdk_thread_log = [this](const SdkThreadPlacementRecord& record) {
        if (record.success) LOG_INFO_M(name_, record);
        else LOG_ERROR_M(name_, record);
    };
    struct SdkThreadStateReset {
        bool active;
        ~SdkThreadStateReset() noexcept {
            if (active) publish_sdk_thread_placement(SdkThreadPlacementState::Disabled);
        }
    } sdk_thread_state_reset{impl_->sdk_thread_placement != nullptr};
    const bool profile_enabled = inference_profile_enabled();
    int consecutive_timeouts = 0;
    int consecutive_incomplete_frames = 0;
    // Framesets in a row the capture clock did not stamp (log lines and the 5 s limit).
    CaptureTimeWithheldRun capture_withheld;
    std::int64_t capture_latency_us_last = -1;
    bool capture_alarm_logged = false;
    std::uint64_t timing_frames = 0;
    std::uint64_t interarrival_samples = 0;
    double wait_us_sum = 0.0;
    double interarrival_us_sum = 0.0;
    double software_align_us_sum = 0.0;
    double software_align_us_max = 0.0;
    std::vector<double> software_align_us_samples;
    software_align_us_samples.reserve(CAMERA_STATS_INTERVAL_FRAMES);
    double mjpg_to_bgr_us_sum = 0.0;
    double hw_decode_us_sum = 0.0, ycc_convert_us_sum = 0.0;
    double color_resize_us_sum = 0.0;
    double depth_resize_us_sum = 0.0;
    double encode_copy_us_sum = 0.0;
    double processing_us_sum = 0.0;
    double processing_us_max = 0.0;
    std::chrono::steady_clock::time_point previous_capture_steady{};

    const auto elapsed_us = [](const auto& begin, const auto& end) {
        return static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) /
            1000.0;
    };

    try {
        while (running_.load()) {
            const auto wait_begin = std::chrono::steady_clock::now();
            sdk_thread_watch.tick(
                std::chrono::duration_cast<std::chrono::nanoseconds>(wait_begin.time_since_epoch()).count(),
                sdk_thread_log);
            auto input_frameset = impl_->pipeline->waitForFrameset(FRAME_WAIT_TIMEOUT_MS);
            const auto wait_end = std::chrono::steady_clock::now();
            if (!input_frameset) {
                ++consecutive_timeouts;
                ++impl_->total_timeouts;
                LOG_WARN_M(
                    name_,
                    "frame wait timed out (" << consecutive_timeouts << "/"
                                              << MAX_CONSECUTIVE_TIMEOUTS << ")");
                update_heartbeat();
                // The capture time limit is a deadline on host time, also while no frame comes.
                if (impl_->capture_clock) {
                    const std::int64_t now_ns = monotonic_now_ns();
                    if (capture_withheld.overdue(now_ns)) {
                        LOG_ERROR_M(name_, "CAPTURE_TIME_LOST framesets=" << capture_withheld.framesets()
                            << " for_ms=" << (now_ns - capture_withheld.limit_from_ns()) / 1000000
                            << " during frame wait timeouts; no capture time for "
                            << kCaptureTimeWithheldLimitNs / 1000000000 << " s, capture loop exiting");
                        break;
                    }
                }
                if (consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
                    LOG_ERROR_M(name_, "too many consecutive Gemini frame timeouts; capture loop exiting");
                    break;
                }
                continue;
            }
            consecutive_timeouts = 0;
            const auto capture_steady = wait_end;
            // LINT-ALLOW(clock): wall stamp (frame_data.stamp_sec) for media/SEI, recording alignment and the
            // evidence-capture frame ring (19 section 10, anchor "墙钟只用于日志展示与录包对齐"). Contract times and
            // the velocity windows come from the native source id and the capture estimate, never from this value.
            // One more reader remains: the odom TF lookup, refused today (tf_enable) and part of the pending
            // 19 section 4.2 work.
            const auto capture_wall = std::chrono::system_clock::now();
            // The host-ready sample. With the capture clock it is read as MONOTONIC, RAW,
            // MONOTONIC: the clock needs both host clocks at one instant, and a thread switch
            // between the readings widens the bracket; then they are read again (three tries; a
            // bracket that is still too wide makes the clock refuse the frameset).
            std::int64_t capture_ready_ns = 0, pickup_raw_ns = -1, pickup_bracket_ns = -1;
            if (profile_enabled || native_depth_output_) {
                capture_ready_ns = monotonic_now_ns();
                for (int attempt = 0; impl_->capture_clock && attempt < 3; ++attempt) {
                    if (attempt) capture_ready_ns = monotonic_now_ns();
                    pickup_raw_ns = monotonic_raw_now_ns();
                    pickup_bracket_ns = monotonic_now_ns() - capture_ready_ns;
                    if (pickup_bracket_ns <= CAPTURE_CLOCK_MAX_BRACKET_NS) break;
                }
            }
            native_depth::SourceId native_source;
            std::uint64_t health_source_sequence = 0;
            if (health_arrivals_ && impl_->native_stream_id != 0) {
                // An SDK depth can exist without colour, with invalid layout,
                // or despite later alignment/tap failure. Count its local
                // observation independently, before any such rejection.
                const auto depth = input_frameset->getDepthFrame();
                if (depth) {
                    if (impl_->native_sequence == std::numeric_limits<std::uint64_t>::max()) {
                        LOG_ERROR_M(name_, "raw depth health sequence exhausted");
                        health_arrivals_->record(native_depth::SourceId{}); // sticky identity fault
                        break;
                    }
                    health_source_sequence = ++impl_->native_sequence;
                    const native_depth::SourceId arrived{impl_->native_stream_id,
                        health_source_sequence, 0, depth->getIndex(), 0,
                        depth->getTimeStampUs(), capture_ready_ns};
                    // observed_ns is sampled inside record(), AFTER metadata
                    // access. It is a local raw-depth observation, not the
                    // exact SDK-return instant or an exposure timestamp.
                    health_arrivals_->record(arrived);
                }
            } else if (health_arrivals_) {
                // Lost native lifecycle identity cannot masquerade as an
                // observed zero-depth window while capture itself continues.
                health_arrivals_->record(native_depth::SourceId{});
            }
            wait_us_sum += elapsed_us(wait_begin, wait_end);
            if (previous_capture_steady.time_since_epoch().count() != 0) {
                interarrival_us_sum += elapsed_us(previous_capture_steady, capture_steady);
                ++interarrival_samples;
            }
            previous_capture_steady = capture_steady;
            std::uint64_t source_color_timestamp_us = 0;
            std::uint64_t source_depth_timestamp_us = 0;
            std::uint64_t source_color_index = 0;
            std::uint64_t source_depth_index = 0;
            if (impl_->software_aligner) {
                auto raw_color = input_frameset->getColorFrame();
                auto raw_depth = input_frameset->getDepthFrame();
                if (!raw_color || !raw_depth) {
                    ++consecutive_incomplete_frames;
                    ++impl_->total_incomplete;
                    LOG_WARN_M(
                        name_,
                        "incomplete SW-D2C input color=" << static_cast<bool>(raw_color)
                            << " depth=" << static_cast<bool>(raw_depth));
                    if (consecutive_incomplete_frames >=
                        MAX_CONSECUTIVE_INCOMPLETE_FRAMES) {
                        LOG_ERROR_M(
                            name_,
                            "too many consecutive incomplete SW-D2C inputs; "
                            "capture loop exiting");
                        break;
                    }
                    continue;
                }
                const std::size_t expected_raw_depth_bytes =
                    static_cast<std::size_t>(raw_depth->getWidth()) *
                    static_cast<std::size_t>(raw_depth->getHeight()) *
                    sizeof(std::uint16_t);
                if (raw_color->getWidth() !=
                        static_cast<std::uint32_t>(config_.camera_color_width()) ||
                    raw_color->getHeight() !=
                        static_cast<std::uint32_t>(config_.camera_color_height()) ||
                    raw_color->getFormat() != OB_FORMAT_MJPG ||
                    raw_depth->getWidth() !=
                        static_cast<std::uint32_t>(config_.camera_depth_width()) ||
                    raw_depth->getHeight() !=
                        static_cast<std::uint32_t>(config_.camera_depth_height()) ||
                    raw_depth->getFormat() != OB_FORMAT_Y16 ||
                    !raw_depth->getData() ||
                    raw_depth->getDataSize() != expected_raw_depth_bytes) {
                    LOG_ERROR_M(name_, "Gemini SW-D2C raw input contract changed");
                    break;
                }
                source_color_timestamp_us = raw_color->getTimeStampUs();
                source_depth_timestamp_us = raw_depth->getTimeStampUs();
                source_color_index = raw_color->getIndex();
                source_depth_index = raw_depth->getIndex();
                // Capture time of this frameset, before anything is handed on. Nothing above this
                // point hands the frameset to a consumer (only its raw-depth arrival is counted),
                // and it must stay so: a frameset without a capture time reaches no consumer.
                // Timestamps the checks further down reject (zero, not increasing) are left to
                // those checks: they end the loop, and nothing is offered for such a frameset.
                CaptureStampResult stamp;
                const bool timestamps_increase =
                    source_color_timestamp_us != 0 && source_depth_timestamp_us != 0 &&
                    (impl_->previous_color_timestamp_us == 0 ||
                     source_color_timestamp_us > impl_->previous_color_timestamp_us) &&
                    (impl_->previous_depth_timestamp_us == 0 ||
                     source_depth_timestamp_us > impl_->previous_depth_timestamp_us);
                if (impl_->capture_clock && timestamps_increase) {
                    CaptureClockSample sample;
                    sample.pickup_raw_ns = pickup_raw_ns;
                    sample.pickup_mono_ns = capture_ready_ns;
                    sample.bracket_ns = pickup_bracket_ns;
                    sample.device_us = source_depth_timestamp_us;
                    sample.half_exposure_us = depth_half_exposure_us(raw_depth);
                    sample.global_us = depth_global_us(raw_depth);
                    stamp = impl_->capture_clock->step(sample);
                    auto& counters = capture_time_counters();
                    if (stamp.status != CaptureStampStatus::Stamped) {
                        // No capture time: the frameset is not handed to the Profile or to the
                        // inference (and not encoded), as if the camera had not delivered it. The
                        // raw-depth arrival above stays counted; the pause in the publications
                        // counts in the Objects cadence like any other.
                        counters.withheld.fetch_add(1, std::memory_order_relaxed);
                        if (stamp.status == CaptureStampStatus::Settling)
                            counters.settling.fetch_add(1, std::memory_order_relaxed);
                        counters.withheld_now.store(true, std::memory_order_relaxed);
                        // The withheld frameset still uses up a capture frame id: the tracker ages
                        // lost tracks by frame id differences (bytetrack.hpp, AGES), so withheld
                        // framesets age them like any camera frame inference skips (GPT 16.1).
                        frame_counter_.fetch_add(1);
                        // The first second of a device stream is exempt as a time interval, not
                        // only for framesets whose status is Settling (in_stream_settle()).
                        const auto run = capture_withheld.withheld(
                            capture_ready_ns, impl_->capture_clock->in_stream_settle(sample.pickup_raw_ns));
                        if (run == CaptureTimeWithheldRun::Step::Started) {
                            LOG_WARN_M(name_, "CAPTURE_TIME_WITHHELD status=" << capture_stamp_status_name(stamp.status)
                                << " device_stream=" << stamp.device_stream
                                << " global_usable=" << static_cast<int>(stamp.global_usable)
                                << " envelope_anchored=" << static_cast<int>(stamp.envelope_anchored)
                                << " depth_index=" << source_depth_index
                                << " (framesets are not handed on until one is stamped)");
                        } else if (run == CaptureTimeWithheldRun::Step::TooLong) {
                            // Framesets keep coming, none gets a capture time: a capture failure
                            // like five frame timeouts in a row (capture_time_state.hpp).
                            LOG_ERROR_M(name_, "CAPTURE_TIME_LOST framesets=" << capture_withheld.framesets()
                                << " for_ms=" << (capture_ready_ns - capture_withheld.limit_from_ns()) / 1000000
                                << " last_status=" << capture_stamp_status_name(stamp.status)
                                << " global_usable=" << static_cast<int>(stamp.global_usable)
                                << " envelope_anchored=" << static_cast<int>(stamp.envelope_anchored)
                                << "; no capture time for " << kCaptureTimeWithheldLimitNs / 1000000000
                                << " s, capture loop exiting");
                            break;
                        }
                        impl_->previous_color_timestamp_us = source_color_timestamp_us;
                        impl_->previous_depth_timestamp_us = source_depth_timestamp_us;
                        impl_->previous_color_index = source_color_index;
                        impl_->previous_depth_index = source_depth_index;
                        update_heartbeat();
                        continue;
                    }
                    const auto run = capture_withheld.stamped(capture_ready_ns);
                    if (run == CaptureTimeWithheldRun::Step::TooLong) {
                        // Stamped, but only after the deadline: it does not clear the run.
                        LOG_ERROR_M(name_, "CAPTURE_TIME_LOST framesets=" << capture_withheld.framesets()
                            << " for_ms=" << (capture_ready_ns - capture_withheld.limit_from_ns()) / 1000000
                            << " (a frameset was stamped again, after the deadline); no capture time for "
                            << kCaptureTimeWithheldLimitNs / 1000000000 << " s, capture loop exiting");
                        break;
                    }
                    counters.stamped.fetch_add(1, std::memory_order_relaxed);
                    if (stamp.estimated) counters.estimated.fetch_add(1, std::memory_order_relaxed);
                    counters.estimated_now.store(stamp.estimated, std::memory_order_relaxed);
                    counters.withheld_now.store(false, std::memory_order_relaxed);
                    if (run == CaptureTimeWithheldRun::Step::Resumed) {
                        LOG_INFO_M(name_, "CAPTURE_TIME_RESUMED after_framesets=" << capture_withheld.ended_framesets()
                            << " after_ms=" << (capture_ready_ns - capture_withheld.since_ns()) / 1000000
                            << " source=" << (stamp.source == CaptureStampSource::Global ? "global" : "envelope")
                            << " estimated=" << static_cast<int>(stamp.estimated)
                            << " latency_us=" << stamp.latency_ns / 1000);
                    }
                    capture_latency_us_last = stamp.latency_ns / 1000;
                }
                if (native_depth_output_ && impl_->native_stream_id != 0 &&
                    stamp.status == CaptureStampStatus::Stamped) {
                    if (!health_source_sequence &&
                        impl_->native_sequence == std::numeric_limits<std::uint64_t>::max()) {
                        LOG_ERROR_M(name_, "native depth source sequence exhausted; tap disabled until restart");
                        native_depth_output_->end_stream();
                        impl_->native_stream_id = 0;
                    } else {
                        const native_depth::TapContext context{
                            true,
                            static_cast<std::uint32_t>(config_.camera_depth_width()),
                            static_cast<std::uint32_t>(config_.camera_depth_height()),
                            MAX_COLOR_DEPTH_SKEW_US, impl_->native_stream_id,
                            health_source_sequence ? health_source_sequence : ++impl_->native_sequence,
                            capture_ready_ns, stamp.capture_mono_ns, stamp.estimated};
                        // Offer BEFORE align/decode/resize; no SDK pointer survives
                        // the call. Consumers must still observe stream invalidation.
                        const auto tap = native_depth::offer_orbbec(
                            *native_depth_output_, raw_color, raw_depth, context);
                        if (tap.error != native_depth::TapError::None ||
                            (tap.outcome != native_depth::Offer::Accepted &&
                             tap.outcome != native_depth::Offer::Replaced &&
                             tap.outcome != native_depth::Offer::Busy)) {
                            LOG_ERROR_M(name_, "native depth rejected error="
                                << static_cast<int>(tap.error) << " outcome=" << static_cast<int>(tap.outcome)
                                << "; tap disabled until restart");
                            // A metadata/tap fault invalidates only this optional
                            // output, not otherwise-valid legacy RGB-D. Existing
                            // alignment/timestamp checks below remain authoritative.
                            native_depth_output_->end_stream();
                            impl_->native_stream_id = 0;
                        } else {
                            // Busy is counted, not retried. The aligned frame still
                            // identifies this source even when the tap missed it.
                            native_source = tap.source;
                        }
                    }
                }
            }

            auto frameset = input_frameset;
            if (impl_->software_aligner) {
                const auto software_align_begin = std::chrono::steady_clock::now();
                frameset = impl_->align_frameset(input_frameset);
                const auto software_align_end = std::chrono::steady_clock::now();
                if (!frameset) {
                    ++consecutive_incomplete_frames;
                    ++impl_->total_alignment_failures;
                    LOG_WARN_M(name_, "Gemini SW-D2C returned no frameset");
                    if (consecutive_incomplete_frames >=
                        MAX_CONSECUTIVE_INCOMPLETE_FRAMES) {
                        LOG_ERROR_M(
                            name_,
                            "too many consecutive Gemini SW-D2C failures; "
                            "capture loop exiting");
                        break;
                    }
                    continue;
                }
                const double software_align_us =
                    elapsed_us(software_align_begin, software_align_end);
                software_align_us_sum += software_align_us;
                software_align_us_max = std::max(software_align_us_max, software_align_us);
                software_align_us_samples.push_back(software_align_us);
            }
            auto color = impl_->software_aligner
                ? input_frameset->getColorFrame()
                : frameset->getColorFrame();
            auto depth = frameset->getDepthFrame();
            if (!color || !depth) {
                ++consecutive_incomplete_frames;
                ++impl_->total_incomplete;
                LOG_WARN_M(
                    name_,
                    "incomplete frameset color=" << static_cast<bool>(color)
                                                 << " depth=" << static_cast<bool>(depth));
                if (consecutive_incomplete_frames >= MAX_CONSECUTIVE_INCOMPLETE_FRAMES) {
                    LOG_ERROR_M(name_, "too many consecutive incomplete Gemini frames; capture loop exiting");
                    break;
                }
                continue;
            }
            consecutive_incomplete_frames = 0;
            if (!impl_->software_aligner) {
                source_color_timestamp_us = color->getTimeStampUs();
                source_depth_timestamp_us = depth->getTimeStampUs();
            }

            if (color->getWidth() !=
                    static_cast<std::uint32_t>(config_.camera_color_width()) ||
                color->getHeight() !=
                    static_cast<std::uint32_t>(config_.camera_color_height()) ||
                depth->getWidth() != color->getWidth() ||
                depth->getHeight() != color->getHeight() ||
                color->getFormat() != OB_FORMAT_MJPG ||
                depth->getFormat() != OB_FORMAT_Y16) {
                LOG_ERROR_M(name_, "Gemini runtime frame shape/format changed; capture loop exiting");
                break;
            }
            const std::size_t expected_depth_bytes =
                static_cast<std::size_t>(depth->getWidth()) *
                static_cast<std::size_t>(depth->getHeight()) * sizeof(std::uint16_t);
            if (!depth->getData() || depth->getDataSize() != expected_depth_bytes) {
                LOG_ERROR_M(name_, "Gemini aligned-depth buffer size changed; capture loop exiting");
                break;
            }

            const auto mjpg_begin = std::chrono::steady_clock::now();
            // The frozen input contract rejects all other colour formats above. The hardware
            // path decodes into a fresh image per frame (it becomes the frame's own buffer).
            std::shared_ptr<ob::Frame> bgr;
            cv::Mat hw_bgr;
            HwMjpegDecoder::Timing hw_timing;
            if (impl_->hw_decoder) {
                std::string error;
                if (!impl_->hw_decoder->decode(static_cast<const std::uint8_t*>(color->getData()),
                                               color->getDataSize(), hw_bgr, error, &hw_timing)) {
                    LOG_ERROR_M(name_, "Gemini hardware MJPG decode failed: " << error);
                    break;
                }
            } else {
                impl_->format_converter->setFormatConvertType(FORMAT_MJPG_TO_BGR);
                bgr = impl_->format_converter->process(color);
            }
            const auto bgr_end = std::chrono::steady_clock::now();
            if (!impl_->hw_decoder && (!bgr || bgr->getFormat() != OB_FORMAT_BGR ||
                bgr->getDataSize() != color->getWidth() * color->getHeight() * 3U)) {
                LOG_ERROR_M(name_, "Gemini MJPG-to-BGR conversion failed");
                break;
            }

            const float scale = static_cast<float>(
                depth->getValueScale() * MILLIMETRES_TO_METRES);
            if (!finite_positive(scale) || std::fabs(scale - impl_->depth_scale) > 1.0e-9F) {
                LOG_ERROR_M(name_, "Gemini depth scale changed or became invalid");
                break;
            }
            if (impl_->software_aligner) {
                const auto frame_intrinsics =
                    depth->getStreamProfile()->as<ob::VideoStreamProfile>()->getIntrinsic();
                if (!same_intrinsics(frame_intrinsics, impl_->aligned_depth_intrinsics)) {
                    LOG_ERROR_M(
                        name_,
                        "Gemini SW-D2C aligned-depth intrinsics changed; capture loop exiting");
                    break;
                }
            }

            const std::uint64_t color_depth_skew_us =
                absolute_difference(source_color_timestamp_us, source_depth_timestamp_us);
            if (source_color_timestamp_us == 0 || source_depth_timestamp_us == 0) {
                LOG_ERROR_M(name_, "Gemini returned a zero device timestamp; capture loop exiting");
                break;
            }
            if (color_depth_skew_us > MAX_COLOR_DEPTH_SKEW_US) {
                LOG_ERROR_M(
                    name_,
                    "Gemini color/depth device timestamp skew=" << color_depth_skew_us
                        << "us exceeds " << MAX_COLOR_DEPTH_SKEW_US << "us; capture loop exiting");
                break;
            }
            if ((impl_->previous_color_timestamp_us != 0 &&
                 source_color_timestamp_us <= impl_->previous_color_timestamp_us) ||
                (impl_->previous_depth_timestamp_us != 0 &&
                 source_depth_timestamp_us <= impl_->previous_depth_timestamp_us)) {
                ++impl_->timestamp_regressions;
                LOG_ERROR_M(name_, "Gemini device timestamp regressed; capture loop exiting");
                break;
            }
            if (impl_->software_aligner &&
                ((impl_->previous_color_index != 0 &&
                  source_color_index <= impl_->previous_color_index) ||
                 (impl_->previous_depth_index != 0 &&
                  source_depth_index <= impl_->previous_depth_index))) {
                LOG_ERROR_M(name_, "Gemini SW-D2C source frame index regressed");
                break;
            }
            impl_->previous_color_timestamp_us = source_color_timestamp_us;
            impl_->previous_depth_timestamp_us = source_depth_timestamp_us;
            if (impl_->software_aligner) {
                impl_->previous_color_index = source_color_index;
                impl_->previous_depth_index = source_depth_index;
            }
            impl_->color_depth_skew_us_max = std::max(
                impl_->color_depth_skew_us_max,
                color_depth_skew_us);

            cv::Mat color_view(
                static_cast<int>(color->getHeight()),
                static_cast<int>(color->getWidth()),
                CV_8UC3,
                impl_->hw_decoder ? static_cast<void*>(hw_bgr.data) : bgr->getData());
            cv::Mat depth_view(
                static_cast<int>(depth->getHeight()),
                static_cast<int>(depth->getWidth()),
                CV_16UC1,
                depth->getData());

            yolo::FrameData frame_data;
            const cv::Size output_size(
                config_.camera_output_width(), config_.camera_output_height());
            const auto color_resize_begin = std::chrono::steady_clock::now();
            // The hardware decode already owns a fresh buffer: at the output size it is used as is.
            frame_data.frame = impl_->hw_decoder && hw_bgr.size() == output_size
                ? hw_bgr
                : orbbec_capture_detail::copy_or_resize_owned(color_view, output_size, cv::INTER_LINEAR);
            const auto color_resize_end = std::chrono::steady_clock::now();
            // Never interpolate raw depth values: it would fabricate ranges at
            // object edges and across invalid (zero) pixels. NEAREST_EXACT uses
            // the same pixel-centre convention as the intrinsics transform.
            if (impl_->registration && depth_view.isContinuous() && depth_view.cols == impl_->registration->width() &&
                depth_view.rows == impl_->registration->height() && output_size == depth_view.size()) {
                // Registered onto the RGB image: nearest aligned value per RGB
                // pixel, 0 where the pinhole aligned raster has no counterpart.
                frame_data.depth.create(depth_view.rows, depth_view.cols, CV_16UC1);
                impl_->registration->apply(depth_view.ptr<std::uint16_t>(), frame_data.depth.ptr<std::uint16_t>());
                frame_data.depth_registration = impl_->registration;
            } else {
                frame_data.depth = orbbec_capture_detail::copy_or_resize_owned(
                    depth_view,
                    output_size,
                    cv::INTER_NEAREST_EXACT);
            }
            const auto depth_resize_end = std::chrono::steady_clock::now();
            frame_data.timestamp = capture_steady;
            frame_data.stamp_sec =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    capture_wall.time_since_epoch()).count();
            frame_data.frame_id = frame_counter_.fetch_add(1);
            frame_data.native_depth_source = native_source;
            if (profile_enabled) {
                frame_data.pipeline_timing.capture_epoch_ns =
                    system_time_ns(capture_wall);
                frame_data.pipeline_timing.capture_ready_ns = capture_ready_ns;
            }
            frame_data.depth_intrinsics = impl_->output_depth_intrinsics;
            frame_data.depth_scale = impl_->depth_scale;

            // Exposure metadata units are explicitly model-dependent in SDK
            // v2.9.3. Keep the established valid flags false until the 338Le
            // unit is independently calibrated; never publish a guessed value.

            if (frame_data.frame.empty() || frame_data.depth.empty()) {
                LOG_ERROR_M(name_, "failed to copy Gemini color/depth frame");
                break;
            }

            const std::uint64_t frame_id = frame_data.frame_id;
            // Freeze identity before any move. B can be consumed/destroyed
            // synchronously while the capture thread still copies for HEVC.
            auto encode_copy_begin = capture_steady;
            auto encode_copy_end = capture_steady;
            auto inference_handoff_ready = capture_steady;
            const auto encode_stage = [&](const yolo::FrameData& encoding_source) {
                encode_copy_begin = std::chrono::steady_clock::now();
                yolo::FrameData encode_frame;
                // stream_rtsp shares the owned, immutable capture frame (no consumer
                // mutates pixels after handoff); the legacy branch keeps its clone.
                encode_frame.frame = impl_->media_stream ? encoding_source.frame : encoding_source.frame.clone();
                encode_frame.timestamp = encoding_source.timestamp;
                encode_frame.stamp_sec = encoding_source.stamp_sec;
                encode_frame.frame_id = encoding_source.frame_id;
                encode_frame.native_depth_source = encoding_source.native_depth_source;
                encode_frame.camera_metadata = encoding_source.camera_metadata;
                if (!encode_frame.frame.empty()) {
                    encode_queue_->push(std::move(encode_frame));
                } else {
                    LOG_WARN_M(name_, "failed to copy Gemini RGB frame for HEVC branch");
                }
                encode_copy_end = std::chrono::steady_clock::now();
            };
            const auto inference_stage = [&](yolo::FrameData& complete_frame) {
                // This field reports only work already finished BEFORE handoff.
                // A retains its historical inclusion of encoding; B excludes
                // later encoding. Never backfill future time in a queued object.
                inference_handoff_ready = std::chrono::steady_clock::now();
                complete_frame.capture_time_ms =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        inference_handoff_ready - capture_steady).count() / 1000.0;
                if (profile_enabled) {
                    complete_frame.pipeline_timing.frame_push_begin_ns = monotonic_now_ns();
                }
                output_queue_->push(std::move(complete_frame));
            };
            // B may already have handed off inference before encode fails. That
            // partial order is irreversible; the exception reaches the outer
            // exception/stop behavior unchanged. No rollback, swallowed error or retry.
            orbbec_capture_detail::handoff_complete_rgbd(
                impl_->inference_handoff_first, static_cast<bool>(encode_queue_),
                frame_data, encode_stage, inference_stage);
            // A ends at its historical pre-inference handoff-ready point; B
            // ends after the encoding stage. Thus CameraTiming covers the
            // complete selected capture work, not a rewritten frame timestamp.
            const auto processing_end = std::max(inference_handoff_ready, encode_copy_end);
            update_heartbeat();
            if (impl_->capture_clock) {
                // The watchdogs compare the two candidates; they never change a stamp. One line
                // when any of them comes on, one when all are off again.
                const auto& h = impl_->capture_clock->health();
                const bool alarm = h.fast_alarm || h.slow_alarm || h.incomparable;
                if (alarm != capture_alarm_logged) {
                    capture_alarm_logged = alarm;
                    LOG_WARN_M(name_, "CAPTURE_CLOCK_WATCH " << (alarm ? "raised" : "cleared")
                        << " fast_alarm=" << static_cast<int>(h.fast_alarm)
                        << " slow_alarm=" << static_cast<int>(h.slow_alarm)
                        << " incomparable=" << static_cast<int>(h.incomparable)
                        << " global_minus_envelope_fast_median_us=" << h.fast_median_us
                        << " slow_median_us=" << h.slow_median_us);
                }
            }

            ++timing_frames;
            mjpg_to_bgr_us_sum += elapsed_us(mjpg_begin, bgr_end);
            hw_decode_us_sum += hw_timing.hw_us;
            ycc_convert_us_sum += hw_timing.convert_us;
            color_resize_us_sum += elapsed_us(color_resize_begin, color_resize_end);
            depth_resize_us_sum += elapsed_us(color_resize_end, depth_resize_end);
            encode_copy_us_sum += elapsed_us(encode_copy_begin, encode_copy_end);
            const double processing_us = elapsed_us(capture_steady, processing_end);
            processing_us_sum += processing_us;
            processing_us_max = std::max(processing_us_max, processing_us);

            if (frame_id > 0 && frame_id % CAMERA_STATS_INTERVAL_FRAMES == 0U) {
                LOG_INFO_M(
                    name_,
                    "[CameraStats] frames=" << (frame_id + 1)
                        << " timeout=" << impl_->total_timeouts
                        << " incomplete=" << impl_->total_incomplete
                        << " alignment_failure=" << impl_->total_alignment_failures
                        << " timestamp_regress=" << impl_->timestamp_regressions
                        << " color_depth_skew_us_max=" << impl_->color_depth_skew_us_max
                        << " inference_queue_drop=" << output_queue_->dropped_count()
                        << " encode_queue_drop="
                        << (encode_queue_ ? encode_queue_->dropped_count() : 0)
                        << " media_stream=" << static_cast<int>(impl_->media_stream));
                if (impl_->capture_clock) {
                    const auto& h = impl_->capture_clock->health();
                    LOG_INFO_M(
                        name_,
                        "[CaptureClock] stamped_global=" << h.stamped_global
                            << " stamped_envelope=" << h.stamped_envelope
                            << " estimated=" << capture_time_counters().estimated.load(std::memory_order_relaxed)
                            << " settling=" << h.settling
                            << " no_trusted_time=" << h.no_trusted_time
                            << " not_later=" << h.not_later
                            << " duplicates=" << h.duplicates
                            << " bracket_too_wide=" << h.bracket_too_wide
                            << " invalid_input=" << h.invalid_input
                            << " device_streams=" << h.device_streams
                            << " anchors=" << h.anchors
                            << " fast_alarm=" << static_cast<int>(h.fast_alarm)
                            << " slow_alarm=" << static_cast<int>(h.slow_alarm)
                            << " incomparable=" << static_cast<int>(h.incomparable)
                            << " global_minus_envelope_fast_median_us=" << h.fast_median_us
                            << " slow_median_us=" << h.slow_median_us
                            << " slow_locked_us=" << (h.slow_locked ? h.slow_locked_us : 0)
                            << " last_latency_us=" << capture_latency_us_last
                            << " min_path_us=" << impl_->capture_min_path_us);
                }
                const double timing_divisor = static_cast<double>(timing_frames);
                const double interarrival_divisor =
                    static_cast<double>(std::max<std::uint64_t>(interarrival_samples, 1U));
                LOG_INFO_M(
                    name_,
                    "[CameraTiming] samples=" << timing_frames
                        << " wait_avg_ms=" << wait_us_sum / timing_divisor / 1000.0
                        << " interarrival_avg_ms="
                        << interarrival_us_sum / interarrival_divisor / 1000.0
                        << " software_align_avg_ms="
                        << software_align_us_sum / timing_divisor / 1000.0
                        << " software_align_p50_ms="
                        << percentile(software_align_us_samples, 0.50) / 1000.0
                        << " software_align_p95_ms="
                        << percentile(software_align_us_samples, 0.95) / 1000.0
                        << " software_align_p99_ms="
                        << percentile(software_align_us_samples, 0.99) / 1000.0
                        << " software_align_max_ms=" << software_align_us_max / 1000.0
                        << " mjpg_to_bgr_avg_ms="
                        << mjpg_to_bgr_us_sum / timing_divisor / 1000.0
                        << " mjpg_decode=" << (impl_->hw_decoder ? "nvv4l2" : "sdk")
                        << " hw_decode_avg_ms=" << hw_decode_us_sum / timing_divisor / 1000.0
                        << " ycc_convert_avg_ms=" << ycc_convert_us_sum / timing_divisor / 1000.0
                        << " color_resize_avg_ms="
                        << color_resize_us_sum / timing_divisor / 1000.0
                        << " depth_resize_avg_ms="
                        << depth_resize_us_sum / timing_divisor / 1000.0
                        << " encode_copy_avg_ms="
                        << encode_copy_us_sum / timing_divisor / 1000.0
                        << " processing_avg_ms="
                        << processing_us_sum / timing_divisor / 1000.0
                        << " processing_max_ms=" << processing_us_max / 1000.0);
                timing_frames = 0;
                interarrival_samples = 0;
                wait_us_sum = 0.0;
                interarrival_us_sum = 0.0;
                software_align_us_sum = 0.0;
                software_align_us_max = 0.0;
                software_align_us_samples.clear();
                mjpg_to_bgr_us_sum = 0.0;
                hw_decode_us_sum = 0.0;
                ycc_convert_us_sum = 0.0;
                color_resize_us_sum = 0.0;
                depth_resize_us_sum = 0.0;
                encode_copy_us_sum = 0.0;
                processing_us_sum = 0.0;
                processing_us_max = 0.0;
            }
            if (capture_affinity.needs_apply()) {
                // The first full decoder/copy/both-queue handoff ran with the
                // common inherited mask. Change only THIS capture task, after
                // its complete frame and unchanged timing records exist.
                const auto& affinity = capture_affinity.apply(frame_id);
                const bool logged = emit_capture_affinity(affinity, affinity_log);
                if (!affinity.success || !logged) {
                    // No retry or unpinned continuation; the RAII guard restores
                    // an applied mask even if the verification/readback failed.
                    // Leaving the loop ends the native depth stream, so say why at
                    // ERROR: otherwise the first consumer only sees a stream
                    // mismatch (r7diag1_B1: initial_mask_not_2_3 on 2-3+7).
                    LOG_ERROR_M(name_, "capture loop stopped by the capture affinity control reason="
                        << (logged ? affinity.reason : "log_emission_failed")
                        << " (PERCEPTION_CAPTURE_AFFINITY_PIN2)");
                    break;
                }
                // This thread is pinned and the SDK placement has ended: the other threads may be
                // narrowed from now on (profile_thread_placement.hpp).
                if (affinity.mode == CaptureAffinityMode::Pin2)
                    capture_pinned_tid().store(affinity.after.tid, std::memory_order_release);
            }
        }
    } catch (const ob::Error& error) {
        if (running_.load()) {
            LOG_ERROR_M(name_, "Gemini capture failed: " << ob_error_message(error));
            // Metadata may have failed before a raw depth could be identified.
            // Do not certify that partial observation as a complete zero count.
            if (health_arrivals_) health_arrivals_->record(native_depth::SourceId{});
        }
    } catch (const std::exception& error) {
        if (running_.load()) {
            LOG_ERROR_M(name_, "Gemini capture failed: " << error.what());
            if (health_arrivals_) health_arrivals_->record(native_depth::SourceId{});
        }
    }

    if (native_depth_output_) native_depth_output_->end_stream();
    running_.store(false);
    update_heartbeat();
}

}  // namespace perception
