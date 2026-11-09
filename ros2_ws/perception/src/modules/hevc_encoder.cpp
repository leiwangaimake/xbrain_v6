/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: hevc_encoder.cpp
 * Brief: Feeds timestamped camera BGR frames to Jetson's GStreamer NVENC (R6: MED-2 preview).
 *
 * Description:
 * R6 path (med2_loopback): hevc_worker pops a frame, selects it by capture time for the
 * preview rate, drops it with a count when appsrc already holds 3 buffers (backpressure),
 * converts the centre crop to BGRx once, stamps a monotonic PTS and the SEI metadata, and
 * pushes it: appsrc -> nvvidconv (NV12) -> nvv4l2h264enc -> SEI probe -> parse -> RTP ->
 * udpsink 127.0.0.1:56000. The RTSP server (hevc_rtsp thread) relays that RTP over TCP with a
 * 500 ms bounded queue. stop() drains with EOS and logs STREAM_PIPELINE_ACCOUNTING, where
 * received == pushed + drops + rate_skipped + out_of_order_skipped must hold.
 * launch_pipeline()'s file branch and push_frame() serve the V5 file_rtsp sink only (MP4
 * plus RTSP); the R6 service never configures it.
 */

#include "common/thread_name.hpp"
#include "common/bgrx_convert.hpp"
#include "modules/hevc_encoder.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-client.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp-server/rtsp-session.h>
#include <gst/rtsp/gstrtspurl.h>
#include <opencv2/core.hpp>

#include "common/logger.hpp"
#ifdef PERCEPTION_FRAME_SYNC_SEI
#include "common/frame_sync_gst.hpp"
#endif

namespace perception {

namespace {

constexpr auto QUEUE_TIMEOUT = std::chrono::milliseconds(100);
constexpr GstClockTime PIPELINE_TIMEOUT = 5 * GST_SECOND;
constexpr int RTP_LOOPBACK_PORT = 56000;
constexpr guint RTSP_MAX_SESSIONS = 4;
// Match the target GStreamer 1.20 default. A five-second timeout was too
// aggressive for FFmpeg 8 clients: one delayed keepalive response could expire
// an otherwise active TCP session and freeze only that viewer.
constexpr guint RTSP_SESSION_TIMEOUT_SECONDS = 60;
constexpr auto ENCODER_STATS_INTERVAL = std::chrono::seconds(5);

void init_gstreamer() {
    static std::once_flag once;
    std::call_once(once, [] { gst_init(nullptr, nullptr); });
}

void configure_rtsp_session(
    GstRTSPClient*, GstRTSPSession* session, gpointer) {
    gst_rtsp_session_set_timeout(session, RTSP_SESSION_TIMEOUT_SECONDS);
}

void configure_rtsp_client(
    GstRTSPServer*, GstRTSPClient* client, gpointer) {
    g_object_set(client, "drop-backlog", TRUE, nullptr);
    g_signal_connect(client, "new-session", G_CALLBACK(configure_rtsp_session), nullptr);
}

gboolean cleanup_expired_rtsp_sessions(
    GstRTSPSessionPool* pool, gpointer) {
    gst_rtsp_session_pool_cleanup(pool);
    return G_SOURCE_CONTINUE;
}

GstRTSPFilterResult count_rtsp_client(
    GstRTSPServer*, GstRTSPClient*, gpointer user_data) {
    auto* count = static_cast<int*>(user_data);
    ++(*count);
    return GST_RTSP_FILTER_KEEP;
}

#ifdef PERCEPTION_FRAME_SYNC_SEI
GstPadProbeReturn count_buffer_probe(
    GstPad*, GstPadProbeInfo* info, gpointer user_data) {
    if (info && user_data &&
        (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) != 0U) {
        static_cast<std::atomic<std::uint64_t>*>(user_data)->fetch_add(
            1U, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}
#endif

}  // namespace

GstPadProbeReturn count_med2_probe(
    GstPad*, GstPadProbeInfo* info, gpointer user_data) {
    if (info && user_data &&
        (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) != 0U) {
        static_cast<std::atomic<std::uint64_t>*>(user_data)->fetch_add(
            1U, std::memory_order_relaxed);
    }
    return GST_PAD_PROBE_OK;
}

// Explicit bound instead of appsrc leaky mode: a frame is never dropped
// silently after a successful push; above this level it is counted instead.
constexpr guint64 MED2_APPSRC_MAX_LEVEL = 3;
constexpr std::uint64_t RELAY_QUEUE_LIMIT_NS = 500000000ULL;   // RTSP relay backlog bound (B8-6)

// MED-2 codecs (19 section 13.1): H.264 is the contract default; H.265 remains
// only to reproduce the earlier diagnostic runs (reply batch 8, B8-6).
bool med2_codec(const std::string& codec) {
    return codec == "nvv4l2h264enc" || codec == "nvv4l2h265enc";
}

struct HevcEncoder::Impl {
    bool med2 = false;
    bool avc = false;              // true: H.264 (AVC) carrier and RTP payload
    int crop_top = 0;              // rows removed above the preview (centre crop)
    std::int64_t select_gap_ns = 0;  // 0: every frame; else min capture gap between selected frames
    bool has_selected = false;
    std::chrono::steady_clock::time_point last_selected{};
    std::atomic<std::uint64_t> med2_rate_skipped{0};   // intentional preview frame selection
    // Frames whose capture time is not after the previous received frame (the
    // capture completes RGB-D pairs out of order now and then). With a reduced
    // preview rate they are skipped here; at the full rate they are encoded as
    // before (PTS kept monotonic). Counted either way, never as rate_skipped.
    // In R6 one capture thread pushes in capture order and 102 recorded 1.8 runs counted 0
    // (2026-10-04), so this is a guard that keeps the PTS from going backwards.
    std::atomic<std::uint64_t> med2_out_of_order{0}, med2_out_of_order_skipped{0};
    bool has_received = false;
    std::chrono::steady_clock::time_point last_received{};
    // RTSP relay (send side, B8-6): RTP packets into / out of the relay queue of
    // the shared RTSP media. in - out = dropped by the bounded queue plus packets
    // still queued at teardown (not separable here).
    std::atomic<std::uint64_t> relay[3]{};   // [0] rtp in, [1] rtp out, [2] configured media
    std::atomic<std::uint64_t> med2_received{0};
    std::atomic<std::uint64_t> med2_pushed{0};
    std::atomic<std::uint64_t> med2_backpressure_drops{0};
    std::atomic<std::uint64_t> med2_appsrc_out{0};
    std::atomic<std::uint64_t> med2_encoder_out{0};
    std::atomic<bool> med2_eos_drained{false};
    GstPad* med2_appsrc_pad = nullptr;
    GstPad* med2_encoder_pad = nullptr;
    gulong med2_appsrc_probe = 0;
    gulong med2_encoder_probe = 0;
    GstElement* pipeline = nullptr;
    GstAppSrc* appsrc = nullptr;
    GstBus* bus = nullptr;
    GMainContext* rtsp_context = nullptr;
    GMainLoop* rtsp_loop = nullptr;
    GstRTSPServer* rtsp_server = nullptr;
    GSource* rtsp_source = nullptr;
    GSource* rtsp_cleanup_source = nullptr;
    std::thread rtsp_thread;
    std::chrono::steady_clock::time_point first_stamp{};
    GstClockTime last_pts = GST_CLOCK_TIME_NONE;
#ifdef PERCEPTION_FRAME_SYNC_SEI
    std::unique_ptr<frame_sync::GstSeiInjector> frame_sync_injector;
    GstPad* frame_sync_pad = nullptr;
    gulong frame_sync_probe_id = 0;
    GstPad* appsrc_output_pad = nullptr;
    gulong appsrc_output_probe_id = 0;
    GstPad* nvenc_input_pad = nullptr;
    gulong nvenc_input_probe_id = 0;
    std::atomic<std::uint64_t> appsrc_output_buffers{0};
    std::atomic<std::uint64_t> nvenc_input_buffers{0};
#endif
};

HevcEncoder::HevcEncoder(
    const yolo::Config& config,
    std::shared_ptr<yolo::ThreadsafeQueue<yolo::FrameData>> input_queue)
    : yolo::ModuleBase("HevcEncoder"),
      config_(config),
      input_queue_(std::move(input_queue)),
      impl_(std::make_unique<Impl>()) {}

HevcEncoder::~HevcEncoder() {
    stop();
}

bool HevcEncoder::config_is_valid() const {
    if (config_.hevc_stream_sink()) {
        return input_queue_ && config_.hevc_enabled() && config_.hevc_width() > 0 &&
               config_.hevc_height() > 0 && config_.hevc_fps() > 0 && config_.hevc_bitrate() > 0 &&
               config_.hevc_gop() > 0 && med2_codec(config_.hevc_codec());
    }
    return input_queue_ && config_.hevc_enabled() && config_.hevc_sink() == "file_rtsp" &&
           config_.hevc_width() > 0 && config_.hevc_height() > 0 && config_.hevc_fps() > 0 &&
           config_.hevc_bitrate() > 0 && config_.hevc_gop() > 0 &&
           config_.hevc_codec() == "nvv4l2h265enc" && !config_.hevc_file_path().empty();
}

// Builds and starts the R6 MED-2 pipeline (see the file header). Every pad the accounting and
// the SEI injection need must exist, otherwise the start fails.
bool HevcEncoder::launch_stream_pipeline() {
    init_gstreamer();
    impl_->med2 = true;
    impl_->avc = config_.hevc_codec() == "nvv4l2h264enc";
    // Independent preview (19 section 13.1): the analysis frame (camera output)
    // is never modified. The preview is a centre crop of it to the configured
    // size (config validation: same width, even height difference) and, when
    // its rate is below the camera rate, a capture-time frame selection.
    impl_->crop_top = (config_.camera_output_height() - config_.hevc_height()) / 2;
    impl_->select_gap_ns = config_.hevc_fps() < config_.camera_output_fps()
        ? 1000000000LL / config_.hevc_fps() - 1000000000LL / (2LL * config_.camera_output_fps())
        : 0;
    impl_->has_selected = false;
    // H.264 (contract default): CBR, IDR and I-frame every GOP, no B frames,
    // lowest-latency hardware preset; there is no plugin property literally
    // named "zerolatency" (properties recorded in evidence/med2_h264).
    // H.265 keeps the earlier diagnostic settings byte for byte.
    const std::string encoder = impl_->avc
        ? "nvv4l2h264enc name=hardware_encoder control-rate=1 bitrate=" + std::to_string(config_.hevc_bitrate()) +
              " iframeinterval=" + std::to_string(config_.hevc_gop()) + " idrinterval=" +
              std::to_string(config_.hevc_gop()) +
              " num-B-Frames=0 preset-level=1 maxperf-enable=true insert-vui=true insert-sps-pps=true ! "
        : "nvv4l2h265enc name=hardware_encoder bitrate=" + std::to_string(config_.hevc_bitrate()) +
              " iframeinterval=" + std::to_string(config_.hevc_gop()) +
              " num-B-Frames=0 preset-level=1 maxperf-enable=true insert-vui=true "
              "insert-sps-pps=true ! ";
    const std::string parse_pay = impl_->avc
        ? "h264parse config-interval=-1 ! rtph264pay pt=96 config-interval=1 ! "
        : "h265parse config-interval=-1 ! rtph265pay pt=96 config-interval=1 ! ";
    // Same encoder settings and RTP branch as file_rtsp; no tee/MP4 branch.
    // The worker converts the shared immutable BGR frame once to BGRx (the
    // exact bytes videoconvert produced), so NVENC receives identical input.
    const std::string caps = "video/x-raw,format=BGRx,width=" +
        std::to_string(config_.hevc_width()) + ",height=" +
        std::to_string(config_.hevc_height()) + ",framerate=" +
        std::to_string(config_.hevc_fps()) + "/1";
    const std::string description =
        "appsrc name=source is-live=true format=time block=false max-buffers=0 caps=\"" + caps +
        "\" ! queue max-size-buffers=2 leaky=no ! nvvidconv ! "
        "video/x-raw(memory:NVMM),format=NV12 ! " + encoder +
#ifdef PERCEPTION_FRAME_SYNC_SEI
        "identity name=frame_sync_point ! " +
#endif
        parse_pay + "udpsink "
        "host=127.0.0.1 port=" + std::to_string(RTP_LOOPBACK_PORT) + " sync=false async=false";
    GError* error = nullptr;
    impl_->pipeline = gst_parse_launch(description.c_str(), &error);
    if (!impl_->pipeline || error) {
        LOG_ERROR_M(name_, "stream GStreamer pipeline parse failed: "
                           << (error ? error->message : "unknown error"));
        if (error) g_error_free(error);
        stop_pipeline();
        return false;
    }
    GstElement* source = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "source");
    GstElement* hardware_encoder = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "hardware_encoder");
    if (!source || !hardware_encoder) {
        if (source) gst_object_unref(source);
        if (hardware_encoder) gst_object_unref(hardware_encoder);
        LOG_ERROR_M(name_, "stream pipeline is missing appsrc or NVENC");
        stop_pipeline();
        return false;
    }
    impl_->appsrc = GST_APP_SRC(source);
    impl_->med2_appsrc_pad = gst_element_get_static_pad(source, "src");
    impl_->med2_encoder_pad = gst_element_get_static_pad(hardware_encoder, "src");
#ifdef PERCEPTION_FRAME_SYNC_SEI
    impl_->nvenc_input_pad = gst_element_get_static_pad(hardware_encoder, "sink");
#endif
    gst_object_unref(hardware_encoder);
    if (!impl_->med2_appsrc_pad || !impl_->med2_encoder_pad) {
        LOG_ERROR_M(name_, "cannot get stream accounting pads");
        stop_pipeline();
        return false;
    }
    impl_->med2_appsrc_probe = gst_pad_add_probe(impl_->med2_appsrc_pad, GST_PAD_PROBE_TYPE_BUFFER,
        &count_med2_probe, &impl_->med2_appsrc_out, nullptr);
    impl_->med2_encoder_probe = gst_pad_add_probe(impl_->med2_encoder_pad, GST_PAD_PROBE_TYPE_BUFFER,
        &count_med2_probe, &impl_->med2_encoder_out, nullptr);
    if (impl_->med2_appsrc_probe == 0 || impl_->med2_encoder_probe == 0) {
        LOG_ERROR_M(name_, "cannot attach stream accounting probes");
        stop_pipeline();
        return false;
    }
#ifdef PERCEPTION_FRAME_SYNC_SEI
    if (!impl_->nvenc_input_pad) {
        LOG_ERROR_M(name_, "cannot get stream NVENC input pad");
        stop_pipeline();
        return false;
    }
    impl_->nvenc_input_probe_id = gst_pad_add_probe(impl_->nvenc_input_pad, GST_PAD_PROBE_TYPE_BUFFER,
        &count_buffer_probe, &impl_->nvenc_input_buffers, nullptr);
    GstElement* frame_sync_point = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "frame_sync_point");
    if (!frame_sync_point || impl_->nvenc_input_probe_id == 0) {
        if (frame_sync_point) gst_object_unref(frame_sync_point);
        LOG_ERROR_M(name_, "stream pipeline is missing frame_sync_point or NVENC probe");
        stop_pipeline();
        return false;
    }
    impl_->frame_sync_pad = gst_element_get_static_pad(frame_sync_point, "sink");
    gst_object_unref(frame_sync_point);
    if (!impl_->frame_sync_pad) {
        LOG_ERROR_M(name_, "cannot get stream frame_sync_point sink pad");
        stop_pipeline();
        return false;
    }
    impl_->frame_sync_injector = std::make_unique<frame_sync::GstSeiInjector>(
        120, impl_->avc ? frame_sync::Codec::Avc : frame_sync::Codec::Hevc);
    impl_->frame_sync_probe_id = gst_pad_add_probe(impl_->frame_sync_pad, GST_PAD_PROBE_TYPE_BUFFER,
        &frame_sync::GstSeiInjector::pad_probe, impl_->frame_sync_injector.get(), nullptr);
    if (impl_->frame_sync_probe_id == 0) {
        LOG_ERROR_M(name_, "cannot attach stream frame-sync SEI probe");
        stop_pipeline();
        return false;
    }
#endif
    impl_->bus = gst_element_get_bus(impl_->pipeline);
    if (gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        LOG_ERROR_M(name_, "stream GStreamer pipeline failed to enter PLAYING");
        stop_pipeline();
        return false;
    }
    LOG_INFO_M(name_, "STREAM_PIPELINE_STARTED codec=" << (impl_->avc ? "h264" : "h265") << " "
        << config_.hevc_width() << "x" << config_.hevc_height() << "@" << config_.hevc_fps()
        << " bitrate=" << config_.hevc_bitrate() << " control_rate=" << (impl_->avc ? "cbr" : "default")
        << " gop=" << config_.hevc_gop() << " idr=" << (impl_->avc ? config_.hevc_gop() : 0)
        << " b_frames=0 formal_med2=" << static_cast<int>(impl_->avc)
        << " source=" << config_.camera_output_width() << "x" << config_.camera_output_height() << "@"
        << config_.camera_output_fps() << " crop_top=" << impl_->crop_top
        << " select_min_gap_ms=" << impl_->select_gap_ns / 1000000.0 << " rtsp=" << config_.hevc_rtsp_listen()
#ifdef PERCEPTION_FRAME_SYNC_SEI
        << " frame_sync_sei=1"
#else
        << " frame_sync_sei=0"
#endif
        << " mp4=0 cpu_videoconvert=0 appsrc_leaky=none explicit_drop_level=" << MED2_APPSRC_MAX_LEVEL);
    return true;
}

// One frame on hevc_worker: validate, order and rate selection, backpressure, BGRx crop, PTS and
// SEI metadata, push. Returns false only on a hard error; the worker then marks the encoder failed.
bool HevcEncoder::push_stream_frame(const yolo::FrameData& frame) {
    const int width = config_.hevc_width();
    const int height = config_.hevc_height();
    // The analysis frame is the camera output; the preview is its centre crop.
    if (frame.frame.empty() || frame.frame.type() != CV_8UC3 || frame.frame.cols != width ||
        frame.frame.rows != height + 2 * impl_->crop_top) {
        LOG_ERROR_M(name_, "stream rejecting invalid RGB frame id=" << frame.frame_id);
        return false;
    }
    impl_->med2_received.fetch_add(1U, std::memory_order_relaxed);
    const bool out_of_order = impl_->has_received && frame.timestamp <= impl_->last_received;
    if (!out_of_order) {
        impl_->has_received = true;
        impl_->last_received = frame.timestamp;
    } else {
        impl_->med2_out_of_order.fetch_add(1U, std::memory_order_relaxed);
        if (impl_->select_gap_ns > 0) {
            impl_->med2_out_of_order_skipped.fetch_add(1U, std::memory_order_relaxed);
            return true;
        }
    }
    // Intentional preview rate (e.g. 30 -> 15): select by capture time, never
    // counted as a drop. Backpressure drops below are a separate count.
    if (impl_->select_gap_ns > 0) {
        if (impl_->has_selected && std::chrono::duration_cast<std::chrono::nanoseconds>(
                frame.timestamp - impl_->last_selected).count() < impl_->select_gap_ns) {
            impl_->med2_rate_skipped.fetch_add(1U, std::memory_order_relaxed);
            return true;
        }
        impl_->has_selected = true;
        impl_->last_selected = frame.timestamp;
    }
    if (gst_app_src_get_current_level_buffers(impl_->appsrc) >= MED2_APPSRC_MAX_LEVEL) {
        impl_->med2_backpressure_drops.fetch_add(1U, std::memory_order_relaxed);
        return true;
    }
    const std::size_t pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, pixels * 4U, nullptr);
    GstMapInfo map;
    if (!buffer || !gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        if (buffer) gst_buffer_unref(buffer);
        LOG_ERROR_M(name_, "stream buffer allocation failed id=" << frame.frame_id);
        return false;
    }
    for (int y = 0; y < height; ++y) {
        convert_bgr_row_to_bgrx(frame.frame.ptr<std::uint8_t>(y + impl_->crop_top),
            map.data + static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 4U, width);
    }
    gst_buffer_unmap(buffer, &map);
    if (encoded_frames_.load() == 0 && impl_->med2_pushed.load(std::memory_order_relaxed) == 0U) {
        impl_->first_stamp = frame.timestamp;
        LOG_INFO_M(name_, "SYNC_MAP frame_id=" << frame.frame_id << " capture_epoch=" << std::fixed
                              << std::setprecision(6) << frame.stamp_sec << " pts_ns=0");
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        frame.timestamp - impl_->first_stamp).count();
    GstClockTime pts = elapsed > 0 ? static_cast<GstClockTime>(elapsed) : 0;
    if (impl_->last_pts != GST_CLOCK_TIME_NONE && pts <= impl_->last_pts) pts = impl_->last_pts + 1;
    impl_->last_pts = pts;
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, config_.hevc_fps());
#ifdef PERCEPTION_FRAME_SYNC_SEI
    frame_sync::Metadata sync_metadata;
    sync_metadata.frame_id = frame.frame_id;
    sync_metadata.encoder_pts_ns = pts;
    const double capture_stamp_ns = frame.stamp_sec * 1.0e9;
    sync_metadata.capture_stamp_valid = std::isfinite(capture_stamp_ns) && capture_stamp_ns > 0.0 &&
        capture_stamp_ns <= static_cast<double>(std::numeric_limits<std::uint64_t>::max());
    sync_metadata.capture_stamp_ns = sync_metadata.capture_stamp_valid
        ? static_cast<std::uint64_t>(capture_stamp_ns + 0.5) : 0U;
    if (!impl_->frame_sync_injector->remember(pts, sync_metadata)) {
        gst_buffer_unref(buffer);
        LOG_ERROR_M(name_, "cannot register stream frame-sync metadata frame_id=" << frame.frame_id);
        return false;
    }
#endif
    const GstFlowReturn flow = gst_app_src_push_buffer(impl_->appsrc, buffer);
    if (flow != GST_FLOW_OK) {
#ifdef PERCEPTION_FRAME_SYNC_SEI
        impl_->frame_sync_injector->forget(pts);
#endif
        LOG_ERROR_M(name_, "stream appsrc rejected frame id=" << frame.frame_id << " flow=" << flow);
        return false;
    }
    impl_->med2_pushed.fetch_add(1U, std::memory_order_relaxed);
    GstMessage* message = gst_bus_pop_filtered(impl_->bus, GST_MESSAGE_ERROR);
    if (message) {
        GError* error = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        LOG_ERROR_M(name_, "stream pipeline error: " << (error ? error->message : "unknown"));
        if (error) g_error_free(error);
        g_free(debug);
        gst_message_unref(message);
        return false;
    }
    return true;
}

// The R6 stream sinks go to launch_stream_pipeline(); the rest is the V5 file_rtsp pipeline.
bool HevcEncoder::launch_pipeline() {
    if (config_.hevc_stream_sink()) {
        return launch_stream_pipeline();
    }
    const std::filesystem::path output_path(config_.hevc_file_path());
    std::error_code fs_error;
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path(), fs_error);
    }
    if (fs_error) {
        LOG_ERROR_M(name_, "cannot create output directory: " << fs_error.message());
        return false;
    }

    init_gstreamer();
    const std::string caps = "video/x-raw,format=BGR,width=" +
        std::to_string(config_.hevc_width()) + ",height=" +
        std::to_string(config_.hevc_height()) + ",framerate=" +
        std::to_string(config_.hevc_fps()) + "/1";
    const std::string description =
        "appsrc name=source is-live=true format=time block=false max-buffers=3 "
        "leaky-type=downstream caps=\"" + caps + "\" ! queue max-size-buffers=2 "
        "leaky=no ! videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! "
        "video/x-raw(memory:NVMM),format=NV12 ! "
        "nvv4l2h265enc name=hardware_encoder bitrate=" +
        std::to_string(config_.hevc_bitrate()) + " iframeinterval=" +
        std::to_string(config_.hevc_gop()) +
        " num-B-Frames=0 preset-level=1 maxperf-enable=true insert-vui=true "
        "insert-sps-pps=true ! "
#ifdef PERCEPTION_FRAME_SYNC_SEI
        "identity name=frame_sync_point ! "
#endif
        "tee name=encoded "
        "encoded. ! queue ! h265parse ! qtmux ! filesink name=output sync=false "
        "encoded. ! queue max-size-buffers=2 leaky=downstream ! h265parse "
        "config-interval=-1 ! rtph265pay pt=96 config-interval=1 ! udpsink "
        "host=127.0.0.1 port=" + std::to_string(RTP_LOOPBACK_PORT) +
        " sync=false async=false";

    GError* error = nullptr;
    impl_->pipeline = gst_parse_launch(description.c_str(), &error);
    if (!impl_->pipeline || error) {
        LOG_ERROR_M(name_, "GStreamer pipeline parse failed: "
                           << (error ? error->message : "unknown error"));
        if (error) g_error_free(error);
        stop_pipeline();
        return false;
    }
    GstElement* source = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "source");
    GstElement* output = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "output");
    GstElement* hardware_encoder =
        gst_bin_get_by_name(GST_BIN(impl_->pipeline), "hardware_encoder");
    if (!source || !output || !hardware_encoder) {
        if (source) gst_object_unref(source);
        if (output) gst_object_unref(output);
        if (hardware_encoder) gst_object_unref(hardware_encoder);
        LOG_ERROR_M(
            name_,
            "GStreamer pipeline is missing appsrc, NVENC, or filesink");
        stop_pipeline();
        return false;
    }
    impl_->appsrc = GST_APP_SRC(source);
    g_object_set(output, "location", config_.hevc_file_path().c_str(), nullptr);
    gst_object_unref(output);
#ifdef PERCEPTION_FRAME_SYNC_SEI
    impl_->appsrc_output_pad = gst_element_get_static_pad(source, "src");
    impl_->nvenc_input_pad = gst_element_get_static_pad(hardware_encoder, "sink");
    gst_object_unref(hardware_encoder);
    if (!impl_->appsrc_output_pad || !impl_->nvenc_input_pad) {
        LOG_ERROR_M(name_, "cannot get HEVC accounting probe pads");
        stop_pipeline();
        return false;
    }
    impl_->appsrc_output_probe_id = gst_pad_add_probe(
        impl_->appsrc_output_pad,
        GST_PAD_PROBE_TYPE_BUFFER,
        &count_buffer_probe,
        &impl_->appsrc_output_buffers,
        nullptr);
    impl_->nvenc_input_probe_id = gst_pad_add_probe(
        impl_->nvenc_input_pad,
        GST_PAD_PROBE_TYPE_BUFFER,
        &count_buffer_probe,
        &impl_->nvenc_input_buffers,
        nullptr);
    if (impl_->appsrc_output_probe_id == 0 || impl_->nvenc_input_probe_id == 0) {
        LOG_ERROR_M(name_, "cannot attach HEVC accounting probes");
        stop_pipeline();
        return false;
    }
    GstElement* frame_sync_point =
        gst_bin_get_by_name(GST_BIN(impl_->pipeline), "frame_sync_point");
    if (!frame_sync_point) {
        LOG_ERROR_M(name_, "GStreamer pipeline is missing frame_sync_point");
        stop_pipeline();
        return false;
    }
    impl_->frame_sync_pad =
        gst_element_get_static_pad(frame_sync_point, "sink");
    gst_object_unref(frame_sync_point);
    if (!impl_->frame_sync_pad) {
        LOG_ERROR_M(name_, "cannot get frame_sync_point sink pad");
        stop_pipeline();
        return false;
    }
    impl_->frame_sync_injector =
        std::make_unique<frame_sync::GstSeiInjector>(120);
    impl_->frame_sync_probe_id = gst_pad_add_probe(
        impl_->frame_sync_pad,
        GST_PAD_PROBE_TYPE_BUFFER,
        &frame_sync::GstSeiInjector::pad_probe,
        impl_->frame_sync_injector.get(),
        nullptr);
    if (impl_->frame_sync_probe_id == 0) {
        LOG_ERROR_M(name_, "cannot attach HEVC frame-sync SEI probe");
        stop_pipeline();
        return false;
    }
#endif
    impl_->bus = gst_element_get_bus(impl_->pipeline);
    if (gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        LOG_ERROR_M(name_, "GStreamer pipeline failed to enter PLAYING");
        stop_pipeline();
        return false;
    }
#ifdef PERCEPTION_FRAME_SYNC_SEI
    LOG_INFO_M(name_, "GStreamer NVENC file+RTP pipeline started output="
                       << config_.hevc_file_path() << " " << config_.hevc_width() << "x"
                       << config_.hevc_height() << "@" << config_.hevc_fps()
                       << " b_frames=0 preset=ultrafast maxperf=true insert_vui=true"
                       << " frame_sync_sei=true schema=1");
#else
    gst_object_unref(hardware_encoder);
    LOG_INFO_M(name_, "GStreamer NVENC file+RTP pipeline started output="
                       << config_.hevc_file_path() << " " << config_.hevc_width() << "x"
                       << config_.hevc_height() << "@" << config_.hevc_fps()
                       << " b_frames=0 preset=ultrafast maxperf=true insert_vui=true"
                       << " frame_sync_sei=false");
#endif
    return true;
}

namespace {
// Attaches send-side accounting to the relay queue of each configured RTSP
// media (created in the RTSP server thread when the first client connects).
void configure_rtsp_media(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
    auto* counters = static_cast<std::atomic<std::uint64_t>*>(user_data);   // [in, out, media]
    GstElement* element = gst_rtsp_media_get_element(media);
    if (!element) return;
    GstElement* queue = gst_bin_get_by_name(GST_BIN(element), "relay_queue");
    if (queue) {
        GstPad* sink = gst_element_get_static_pad(queue, "sink");
        GstPad* src = gst_element_get_static_pad(queue, "src");
        if (sink) { gst_pad_add_probe(sink, GST_PAD_PROBE_TYPE_BUFFER, &count_med2_probe, &counters[0], nullptr); gst_object_unref(sink); }
        if (src) { gst_pad_add_probe(src, GST_PAD_PROBE_TYPE_BUFFER, &count_med2_probe, &counters[1], nullptr); gst_object_unref(src); }
        counters[2].fetch_add(1U, std::memory_order_relaxed);
        gst_object_unref(queue);
    }
    gst_object_unref(element);
}
}  // namespace

// RTSP relay on its own GLib main loop thread (hevc_rtsp): TCP only, one shared media, at most
// 4 sessions, 60 s session timeout, a 500 ms bounded relay queue.
bool HevcEncoder::start_rtsp_server() {
    GstRTSPUrl* url = nullptr;
    if (gst_rtsp_url_parse(config_.hevc_rtsp_listen().c_str(), &url) != GST_RTSP_OK ||
        !url || !url->host || !url->abspath || url->abspath[0] == '\0') {
        if (url) gst_rtsp_url_free(url);
        LOG_ERROR_M(name_, "invalid RTSP listen URL: " << config_.hevc_rtsp_listen());
        return false;
    }
    const std::string service = std::to_string(url->port == 0 ? 8555 : url->port);
    impl_->rtsp_context = g_main_context_new();
    impl_->rtsp_loop = g_main_loop_new(impl_->rtsp_context, FALSE);
    impl_->rtsp_server = gst_rtsp_server_new();
    g_object_set(impl_->rtsp_server, "address", url->host,
                 "service", service.c_str(), nullptr);
    g_signal_connect(
        impl_->rtsp_server, "client-connected", G_CALLBACK(configure_rtsp_client), nullptr);

    GstRTSPSessionPool* pool = gst_rtsp_server_get_session_pool(impl_->rtsp_server);
    gst_rtsp_session_pool_set_max_sessions(pool, RTSP_MAX_SESSIONS);
    impl_->rtsp_cleanup_source = gst_rtsp_session_pool_create_watch(pool);
    if (impl_->rtsp_cleanup_source) {
        // create_watch() only schedules cleanup notifications.  It does not
        // remove expired sessions unless the source has a pool callback.
        g_source_set_callback(
            impl_->rtsp_cleanup_source,
            G_SOURCE_FUNC(cleanup_expired_rtsp_sessions),
            nullptr,
            nullptr);
    }
    if (!impl_->rtsp_cleanup_source ||
        g_source_attach(impl_->rtsp_cleanup_source, impl_->rtsp_context) == 0) {
        g_object_unref(pool);
        gst_rtsp_url_free(url);
        LOG_ERROR_M(name_, "failed to attach RTSP session cleanup watch");
        stop_rtsp_server();
        return false;
    }
    g_object_unref(pool);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_protocols(factory, GST_RTSP_LOWER_TRANS_TCP);
    const bool avc = impl_->med2 && impl_->avc;
    // Relay queue bounded by TIME, not by a packet count (B8-6): a key-frame
    // access unit arrives as a burst of ~60+ RTP packets, and a 60-packet leaky
    // queue dropped its FIRST packets (SPS/PPS and the frame-sync SEI) although
    // the client was keeping up. 500 ms still bounds the backlog of a stalled
    // client (oldest dropped first); drops are accounted (relay_in/relay_out).
    const std::string launch =
        "( udpsrc address=127.0.0.1 port=" + std::to_string(RTP_LOOPBACK_PORT) +
        " buffer-size=524288 caps=\"application/x-rtp,media=video,clock-rate=90000,"
        "encoding-name=" + std::string(avc ? "H264" : "H265") + ",payload=96\" ! queue name=relay_queue "
        "max-size-buffers=0 max-size-bytes=0 max-size-time=" + std::to_string(RELAY_QUEUE_LIMIT_NS) +
        " leaky=downstream ! " + std::string(avc ? "rtph264depay ! h264parse ! rtph264pay" :
                                                  "rtph265depay ! h265parse ! rtph265pay") +
        " name=pay0 pt=96 config-interval=1 )";
    gst_rtsp_media_factory_set_launch(factory, launch.c_str());
    for (auto& counter : impl_->relay) counter.store(0);
    g_signal_connect(factory, "media-configure", G_CALLBACK(configure_rtsp_media), impl_->relay);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(impl_->rtsp_server);
    gst_rtsp_mount_points_add_factory(mounts, url->abspath, factory);
    g_object_unref(mounts);
    const guint source_id = gst_rtsp_server_attach(impl_->rtsp_server, impl_->rtsp_context);
    if (source_id == 0) {
        gst_rtsp_url_free(url);
        LOG_ERROR_M(name_, "failed to bind RTSP listener at " << config_.hevc_rtsp_listen());
        stop_rtsp_server();
        return false;
    }
    GSource* source = g_main_context_find_source_by_id(impl_->rtsp_context, source_id);
    impl_->rtsp_source = source ? g_source_ref(source) : nullptr;
    impl_->rtsp_thread = std::thread([this] {
        perception::set_current_thread_name("hevc_rtsp");
        g_main_context_push_thread_default(impl_->rtsp_context);
        g_main_loop_run(impl_->rtsp_loop);
        g_main_context_pop_thread_default(impl_->rtsp_context);
    });
    LOG_INFO_M(name_, "RTSP listening at " << config_.hevc_rtsp_listen()
                       << " max_sessions=" << RTSP_MAX_SESSIONS
                       << " session_timeout_s=" << RTSP_SESSION_TIMEOUT_SECONDS
                       << " transport=tcp drop_backlog=true");
    gst_rtsp_url_free(url);
    return true;
}

bool HevcEncoder::push_frame(const yolo::FrameData& frame) {
    if (frame.frame.empty() || frame.frame.type() != CV_8UC3 ||
        frame.frame.cols != config_.hevc_width() || frame.frame.rows != config_.hevc_height()) {
        LOG_ERROR_M(name_, "rejecting invalid RGB frame id=" << frame.frame_id);
        return false;
    }

    const cv::Mat contiguous = frame.frame.isContinuous() ? frame.frame : frame.frame.clone();
    const std::size_t byte_count = contiguous.total() * contiguous.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, byte_count, nullptr);
    if (!buffer || gst_buffer_fill(buffer, 0, contiguous.data, byte_count) != byte_count) {
        if (buffer) gst_buffer_unref(buffer);
        LOG_ERROR_M(name_, "failed to allocate GStreamer buffer for frame id=" << frame.frame_id);
        return false;
    }

    if (encoded_frames_.load() == 0) {
        impl_->first_stamp = frame.timestamp;
        LOG_INFO_M(
            name_,
            "SYNC_MAP frame_id=" << frame.frame_id << " capture_epoch=" << std::fixed
                                  << std::setprecision(6) << frame.stamp_sec << " pts_ns=0");
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        frame.timestamp - impl_->first_stamp).count();
    GstClockTime pts = elapsed > 0 ? static_cast<GstClockTime>(elapsed) : 0;
    if (impl_->last_pts != GST_CLOCK_TIME_NONE && pts <= impl_->last_pts) {
        pts = impl_->last_pts + 1;
    }
    impl_->last_pts = pts;
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, config_.hevc_fps());

#ifdef PERCEPTION_FRAME_SYNC_SEI
    frame_sync::Metadata sync_metadata;
    sync_metadata.frame_id = frame.frame_id;
    sync_metadata.encoder_pts_ns = pts;
    const double capture_stamp_ns = frame.stamp_sec * 1.0e9;
    sync_metadata.capture_stamp_valid =
        std::isfinite(capture_stamp_ns) && capture_stamp_ns > 0.0 &&
        capture_stamp_ns <=
            static_cast<double>(std::numeric_limits<std::uint64_t>::max());
    sync_metadata.capture_stamp_ns =
        sync_metadata.capture_stamp_valid
            ? static_cast<std::uint64_t>(capture_stamp_ns + 0.5)
            : 0U;
    if (!impl_->frame_sync_injector->remember(pts, sync_metadata)) {
        gst_buffer_unref(buffer);
        LOG_ERROR_M(
            name_,
            "cannot register HEVC frame-sync metadata frame_id="
                << frame.frame_id << " pts_ns=" << pts);
        return false;
    }
#endif

    const GstFlowReturn flow = gst_app_src_push_buffer(impl_->appsrc, buffer);
    if (flow != GST_FLOW_OK) {
#ifdef PERCEPTION_FRAME_SYNC_SEI
        impl_->frame_sync_injector->forget(pts);
#endif
        LOG_ERROR_M(name_, "GStreamer rejected frame id=" << frame.frame_id << " flow=" << flow);
        return false;
    }
    GstMessage* message = gst_bus_pop_filtered(impl_->bus, GST_MESSAGE_ERROR);
    if (message) {
        GError* error = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        LOG_ERROR_M(name_, "GStreamer pipeline error: " << (error ? error->message : "unknown"));
        if (error) g_error_free(error);
        g_free(debug);
        gst_message_unref(message);
        return false;
    }
    return true;
}

// Pipeline first, then the RTSP relay, then the worker. An invalid configuration returns false;
// a pipeline or relay failure also sets failed().
bool HevcEncoder::start() {
    if (running_.load()) {
        return true;
    }
    if (!config_is_valid()) {
        LOG_ERROR_M(name_, "invalid HEVC file-sink configuration or input queue");
        return false;
    }
    if (worker_.joinable() || impl_->pipeline) {
        stop();
        impl_ = std::make_unique<Impl>();
    }
    encoded_frames_.store(0);
    failed_.store(false);
    {
        std::lock_guard<std::mutex> lock(frame_sync_stats_mutex_);
        frame_sync_stats_ = FrameSyncStats{};
    }
    if (!launch_pipeline()) {
        failed_.store(true);
        return false;
    }
    if (!start_rtsp_server()) {
        failed_.store(true);
        stop_pipeline();
        return false;
    }
    // c16 (PERCEPTION_MEDIA_NICE preview-thread niceness) was rejected and is not
    // part of the production encoder; its history remains in git (c93c962).
    running_.store(true);
    update_heartbeat();
    worker_ = std::thread(&HevcEncoder::worker_loop, this);
    return true;
}

// Pops frames until stop. Every 5 s: ENCODER_STATS, and the SEI failure counters (injection
// failures, unmatched, duplicate, expired) must all be zero, otherwise the worker ends with
// failed() set.
void HevcEncoder::worker_loop() {
    perception::set_current_thread_name("hevc_worker");
    auto stats_start = std::chrono::steady_clock::now();
    std::uint64_t stats_start_frames = encoded_frames_.load();
    while (running_.load()) {
        yolo::FrameData frame;
        if (!input_queue_->pop(frame, QUEUE_TIMEOUT)) {
            continue;
        }
        if (!(impl_->med2 ? push_stream_frame(frame) : push_frame(frame))) {
            failed_.store(true);
            break;
        }
        const std::uint64_t submitted_frames = encoded_frames_.fetch_add(1) + 1;
        const auto now = std::chrono::steady_clock::now();
        const double stats_elapsed = std::chrono::duration<double>(now - stats_start).count();
        if (stats_elapsed >= std::chrono::duration<double>(ENCODER_STATS_INTERVAL).count()) {
            const double fps = static_cast<double>(submitted_frames - stats_start_frames) / stats_elapsed;
#ifdef PERCEPTION_FRAME_SYNC_SEI
            const auto frame_sync_stats =
                impl_->frame_sync_injector->stats();
            {
                std::lock_guard<std::mutex> lock(frame_sync_stats_mutex_);
                frame_sync_stats_ = FrameSyncStats{
                    frame_sync_stats.remembered,
                    frame_sync_stats.injected,
                    frame_sync_stats.unmatched_buffers,
                    frame_sync_stats.duplicate_pts,
                    frame_sync_stats.expired,
                    frame_sync_stats.injection_failures,
                    frame_sync_stats.pending};
            }
            if (frame_sync_stats.injection_failures != 0U ||
                frame_sync_stats.unmatched_buffers != 0U ||
                frame_sync_stats.duplicate_pts != 0U ||
                frame_sync_stats.expired != 0U) {
                failed_.store(true);
                LOG_ERROR_M(
                    name_,
                    "HEVC frame-sync SEI contract failed injection_failures="
                        << frame_sync_stats.injection_failures
                        << " unmatched_buffers="
                        << frame_sync_stats.unmatched_buffers
                        << " duplicate_pts=" << frame_sync_stats.duplicate_pts
                        << " expired=" << frame_sync_stats.expired);
                break;
            }
            LOG_INFO_M(name_, "ENCODER_STATS submitted_frames=" << submitted_frames
                               << " fps=" << std::fixed << std::setprecision(2) << fps
                               << " frame_sync_injected="
                               << frame_sync_stats.injected
                               << " frame_sync_pending="
                               << frame_sync_stats.pending
                               << " frame_sync_failures="
                               << frame_sync_stats.injection_failures);
#else
            LOG_INFO_M(name_, "ENCODER_STATS submitted_frames=" << submitted_frames
                               << " fps=" << std::fixed << std::setprecision(2) << fps);
#endif
            stats_start = now;
            stats_start_frames = submitted_frames;
        }
        update_heartbeat();
    }
    running_.store(false);
    LOG_INFO_M(name_, "encoder worker stopped submitted_frames=" << encoded_frames_.load());
}

void HevcEncoder::stop_rtsp_server() {
    if (!impl_) return;
    if (impl_->rtsp_loop) g_main_loop_quit(impl_->rtsp_loop);
    if (impl_->rtsp_thread.joinable()) impl_->rtsp_thread.join();
    if (impl_->rtsp_server) {
        const auto in = impl_->relay[0].load(), out = impl_->relay[1].load();
        LOG_INFO_M(name_, "RTSP_RELAY_ACCOUNTING media=" << impl_->relay[2].load() << " rtp_in=" << in
            << " rtp_out=" << out << " not_forwarded=" << (in >= out ? in - out : 0)
            << " (= dropped by the bounded relay queue + still queued at teardown)"
            << " queue_limit_ms=" << RELAY_QUEUE_LIMIT_NS / 1000000);
    }
    if (impl_->rtsp_cleanup_source) {
        g_source_destroy(impl_->rtsp_cleanup_source);
        g_source_unref(impl_->rtsp_cleanup_source);
    }
    if (impl_->rtsp_source) {
        g_source_destroy(impl_->rtsp_source);
        g_source_unref(impl_->rtsp_source);
    }
    if (impl_->rtsp_server) g_object_unref(impl_->rtsp_server);
    if (impl_->rtsp_loop) g_main_loop_unref(impl_->rtsp_loop);
    if (impl_->rtsp_context) g_main_context_unref(impl_->rtsp_context);
    impl_->rtsp_source = nullptr;
    impl_->rtsp_cleanup_source = nullptr;
    impl_->rtsp_server = nullptr;
    impl_->rtsp_loop = nullptr;
    impl_->rtsp_context = nullptr;
}

// EOS drain (bounded by 5 s), pipeline to NULL, then the final accounting line. A missing EOS or
// an incomplete account marks the encoder failed.
void HevcEncoder::stop_pipeline() {
    if (!impl_) return;
    const bool had_pipeline = impl_->pipeline != nullptr;
    if (impl_->appsrc && impl_->bus) {
        gst_app_src_end_of_stream(impl_->appsrc);
        GstMessage* message = gst_bus_timed_pop_filtered(
            impl_->bus, PIPELINE_TIMEOUT,
            static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (!message) {
            failed_.store(true);
            LOG_WARN_M(name_, "timed out waiting for GStreamer EOS; MP4 may be incomplete");
        } else if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
            GError* error = nullptr;
            gchar* debug = nullptr;
            gst_message_parse_error(message, &error, &debug);
            failed_.store(true);
            LOG_WARN_M(name_, "GStreamer shutdown error: "
                               << (error ? error->message : "unknown"));
            if (error) g_error_free(error);
            g_free(debug);
            gst_message_unref(message);
        } else {
            gst_message_unref(message);
            if (impl_->med2) {
                impl_->med2_eos_drained.store(true);
                LOG_INFO_M(name_, "GStreamer EOS received; stream drained");
            } else {
                LOG_INFO_M(name_, "GStreamer EOS received; MP4 finalized");
            }
        }
    }
    if (impl_->pipeline) gst_element_set_state(impl_->pipeline, GST_STATE_NULL);
    if (impl_->med2 && had_pipeline) {
        const auto received = impl_->med2_received.load();
        const auto pushed = impl_->med2_pushed.load();
        const auto drops = impl_->med2_backpressure_drops.load();
        const auto rate_skipped = impl_->med2_rate_skipped.load();
        const auto ooo = impl_->med2_out_of_order.load(), ooo_skipped = impl_->med2_out_of_order_skipped.load();
        const auto appsrc_out = impl_->med2_appsrc_out.load();
        const auto encoder_out = impl_->med2_encoder_out.load();
#ifdef PERCEPTION_FRAME_SYNC_SEI
        const auto nvenc_in = impl_->nvenc_input_buffers.load();
        const auto sei = impl_->frame_sync_injector ? impl_->frame_sync_injector->stats()
                                                    : frame_sync::GstSeiInjector::Stats{};
        const bool sei_ok = impl_->frame_sync_injector && sei.remembered == pushed &&
            sei.injected == pushed && sei.pending == 0U && sei.injection_failures == 0U &&
            sei.unmatched_buffers == 0U && nvenc_in == pushed;
#else
        const std::uint64_t nvenc_in = 0;
        const bool sei_ok = true;
#endif
        const bool complete = impl_->med2_eos_drained.load() && received == pushed + drops + rate_skipped + ooo_skipped &&
            appsrc_out == pushed && encoder_out == appsrc_out && sei_ok;
        if (impl_->appsrc && !complete) failed_.store(true);
        LOG_INFO_M(name_, "STREAM_PIPELINE_ACCOUNTING received=" << received << " pushed=" << pushed
            << " backpressure_drops=" << drops << " rate_skipped=" << rate_skipped << " out_of_order=" << ooo
            << " out_of_order_skipped=" << ooo_skipped << " appsrc_out=" << appsrc_out
            << " nvenc_in=" << nvenc_in << " encoder_out=" << encoder_out
#ifdef PERCEPTION_FRAME_SYNC_SEI
            << " sei_remembered=" << sei.remembered << " sei_injected=" << sei.injected
            << " sei_pending=" << sei.pending
#endif
            << " eos_drained=" << static_cast<int>(impl_->med2_eos_drained.load())
            << " complete=" << static_cast<int>(complete));
        if (impl_->med2_appsrc_pad && impl_->med2_appsrc_probe) gst_pad_remove_probe(impl_->med2_appsrc_pad, impl_->med2_appsrc_probe);
        if (impl_->med2_encoder_pad && impl_->med2_encoder_probe) gst_pad_remove_probe(impl_->med2_encoder_pad, impl_->med2_encoder_probe);
        if (impl_->med2_appsrc_pad) gst_object_unref(impl_->med2_appsrc_pad);
        if (impl_->med2_encoder_pad) gst_object_unref(impl_->med2_encoder_pad);
        impl_->med2_appsrc_pad = impl_->med2_encoder_pad = nullptr;
        impl_->med2_appsrc_probe = impl_->med2_encoder_probe = 0;
    }
#ifdef PERCEPTION_FRAME_SYNC_SEI
    if (impl_->frame_sync_injector) {
        const auto stats = impl_->frame_sync_injector->stats();
        LOG_INFO_M(
            name_,
            "HEVC_PIPELINE_ACCOUNTING remembered=" << stats.remembered
                << " appsrc_out="
                << (impl_->med2 ? impl_->med2_appsrc_out.load(std::memory_order_relaxed)
                                : impl_->appsrc_output_buffers.load(std::memory_order_relaxed))
                << " nvenc_in="
                << impl_->nvenc_input_buffers.load(std::memory_order_relaxed)
                << " injected=" << stats.injected
                << " pending=" << stats.pending);
        std::lock_guard<std::mutex> lock(frame_sync_stats_mutex_);
        frame_sync_stats_ = FrameSyncStats{
            stats.remembered,
            stats.injected,
            stats.unmatched_buffers,
            stats.duplicate_pts,
            stats.expired,
            stats.injection_failures,
            stats.pending};
    }
    if (impl_->frame_sync_pad && impl_->frame_sync_probe_id != 0) {
        gst_pad_remove_probe(
            impl_->frame_sync_pad,
            impl_->frame_sync_probe_id);
    }
    if (impl_->frame_sync_pad) {
        gst_object_unref(impl_->frame_sync_pad);
    }
    impl_->frame_sync_probe_id = 0;
    impl_->frame_sync_pad = nullptr;
    impl_->frame_sync_injector.reset();
    if (impl_->appsrc_output_pad && impl_->appsrc_output_probe_id != 0) {
        gst_pad_remove_probe(
            impl_->appsrc_output_pad,
            impl_->appsrc_output_probe_id);
    }
    if (impl_->nvenc_input_pad && impl_->nvenc_input_probe_id != 0) {
        gst_pad_remove_probe(
            impl_->nvenc_input_pad,
            impl_->nvenc_input_probe_id);
    }
    if (impl_->appsrc_output_pad) {
        gst_object_unref(impl_->appsrc_output_pad);
    }
    if (impl_->nvenc_input_pad) {
        gst_object_unref(impl_->nvenc_input_pad);
    }
    impl_->appsrc_output_probe_id = 0;
    impl_->nvenc_input_probe_id = 0;
    impl_->appsrc_output_pad = nullptr;
    impl_->nvenc_input_pad = nullptr;
#endif
    if (impl_->bus) gst_object_unref(impl_->bus);
    if (impl_->appsrc) gst_object_unref(impl_->appsrc);
    if (impl_->pipeline) gst_object_unref(impl_->pipeline);
    impl_->bus = nullptr;
    impl_->appsrc = nullptr;
    impl_->pipeline = nullptr;
}

// Stops the worker (clearing the queue wakes it), then the pipeline, then the RTSP relay.
void HevcEncoder::stop() {
    const bool was_running = running_.exchange(false);
    if (input_queue_) {
        if (impl_ && impl_->med2) {
            LOG_INFO_M(name_, "STREAM_QUEUE_AT_STOP pending_not_received=" << input_queue_->size()
                << " queue_capacity_drops=" << input_queue_->dropped_count());
        }
        input_queue_->clear();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    stop_pipeline();
    stop_rtsp_server();
    if (was_running) {
        LOG_INFO_M(name_, "stopped submitted_frames=" << encoded_frames_.load());
    }
}

std::uint64_t HevcEncoder::encoded_frames() const {
    return encoded_frames_.load();
}

bool HevcEncoder::failed() const {
    return failed_.load();
}

int HevcEncoder::rtsp_client_count() const {
    if (!impl_ || !impl_->rtsp_server) {
        return 0;
    }
    int count = 0;
    gst_rtsp_server_client_filter(impl_->rtsp_server, count_rtsp_client, &count);
    return count;
}

HevcEncoder::MediaAccounting HevcEncoder::media_accounting() const {
    MediaAccounting result;
    if (!impl_) return result;
    result.med2 = impl_->med2;
    result.received = impl_->med2_received.load();
    result.pushed = impl_->med2_pushed.load();
    result.backpressure_drops = impl_->med2_backpressure_drops.load();
    result.rate_skipped = impl_->med2_rate_skipped.load();
    result.out_of_order = impl_->med2_out_of_order.load();
    result.out_of_order_skipped = impl_->med2_out_of_order_skipped.load();
    result.appsrc_out = impl_->med2_appsrc_out.load();
    result.encoder_out = impl_->med2_encoder_out.load();
    result.eos_drained = impl_->med2_eos_drained.load();
    return result;
}

HevcEncoder::FrameSyncStats HevcEncoder::frame_sync_stats() const {
    std::lock_guard<std::mutex> lock(frame_sync_stats_mutex_);
    return frame_sync_stats_;
}

}  // namespace perception
