/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: hw_mjpeg_decoder.cpp
 * Brief: GStreamer nvv4l2decoder pipeline for one MJPG raster, one JPEG in, one BGR image out.
 *
 * Description:
 * The capture thread pushes one complete JPEG and waits for its decoded planes, so a frame never
 * overtakes another and nothing is queued between calls. The output caps are checked on every
 * frame (Y42B, the opened size) and the buffer must be exactly the planar 4:2:2 size: the layout
 * is then unambiguous for a width that is a multiple of 8, so no video-library stride query is
 * needed. Any error (no sample within the timeout, a bus error, other caps, other size) is
 * returned to the caller, which stops the capture as for a failed software conversion.
 */

#include "modules/hw_mjpeg_decoder.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <mutex>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include "modules/jpeg_ycc.hpp"

namespace perception {
namespace {

bool init_gstreamer(std::string& error) {
    static std::once_flag once;
    static bool ok = false;
    static std::string failure;
    std::call_once(once, [] {
        GError* err = nullptr;
        ok = gst_init_check(nullptr, nullptr, &err) == TRUE;
        if (!ok) failure = err ? err->message : "gst_init_check failed";
        if (err) g_error_free(err);
    });
    if (!ok) error = failure;
    return ok;
}

double elapsed_us(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

}  // namespace

struct HwMjpegDecoder::Impl {
    GstElement* pipeline = nullptr;
    GstAppSrc* src = nullptr;
    GstAppSink* sink = nullptr;
    int width = 0, height = 0;
    GstClockTime pts = 0;

    // The first error message waiting on the pipeline bus, if any.
    std::string bus_error() const {
        std::string text;
        GstBus* bus = gst_element_get_bus(pipeline);
        if (!bus) return text;
        if (GstMessage* m = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR)) {
            GError* err = nullptr;
            gchar* debug = nullptr;
            gst_message_parse_error(m, &err, &debug);
            text = err ? err->message : "unknown pipeline error";
            if (err) g_error_free(err);
            g_free(debug);
            gst_message_unref(m);
        }
        gst_object_unref(bus);
        return text;
    }
};

HwMjpegDecoder::HwMjpegDecoder() : impl_(std::make_unique<Impl>()) {}

HwMjpegDecoder::~HwMjpegDecoder() { close(); }

bool HwMjpegDecoder::is_open() const { return impl_->pipeline != nullptr; }

void HwMjpegDecoder::close() {
    if (impl_->pipeline) gst_element_set_state(impl_->pipeline, GST_STATE_NULL);
    if (impl_->src) gst_object_unref(impl_->src);
    if (impl_->sink) gst_object_unref(impl_->sink);
    if (impl_->pipeline) gst_object_unref(impl_->pipeline);
    *impl_ = Impl{};
}

bool HwMjpegDecoder::open(int width, int height, std::string& error) {
    close();
    if (width < 4 || width % 8 != 0 || height < 1) {
        error = "raster must have a width that is a multiple of 8 and a positive height";
        return false;
    }
    if (!init_gstreamer(error)) return false;
    const std::string size = "width=" + std::to_string(width) + ",height=" + std::to_string(height);
    const std::string description =
        "appsrc name=src is-live=false format=time block=true caps=image/jpeg," + size +
        ",framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=Y42B," + size +
        " ! appsink name=sink sync=false max-buffers=2 drop=false";
    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(description.c_str(), &err);
    if (!pipeline || err) {
        error = std::string("pipeline: ") + (err ? err->message : "gst_parse_launch failed");
        if (err) g_error_free(err);
        if (pipeline) gst_object_unref(pipeline);
        return false;
    }
    impl_->pipeline = pipeline;
    impl_->src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline), "src"));
    impl_->sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
    impl_->width = width;
    impl_->height = height;
    if (!impl_->src || !impl_->sink) {
        error = "pipeline elements src/sink missing";
        close();
        return false;
    }
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        error = "pipeline refused PLAYING: " + impl_->bus_error();
        close();
        return false;
    }
    return true;
}

bool HwMjpegDecoder::decode(const std::uint8_t* jpeg, std::size_t size, cv::Mat& bgr, std::string& error,
                            Timing* timing, int timeout_ms) {
    if (!is_open()) {
        error = "decoder not open";
        return false;
    }
    if (!jpeg || size == 0) {
        error = "empty JPEG";
        return false;
    }
    const auto begin = std::chrono::steady_clock::now();
    GstBuffer* in = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!in || gst_buffer_fill(in, 0, jpeg, size) != size) {
        if (in) gst_buffer_unref(in);
        error = "input buffer allocation failed";
        return false;
    }
    constexpr GstClockTime kFrame = GST_SECOND / 30;
    GST_BUFFER_PTS(in) = impl_->pts;
    GST_BUFFER_DURATION(in) = kFrame;
    impl_->pts += kFrame;
    if (gst_app_src_push_buffer(impl_->src, in) != GST_FLOW_OK) {  // takes ownership of in
        error = "push refused: " + impl_->bus_error();
        return false;
    }
    GstSample* sample = gst_app_sink_try_pull_sample(
        impl_->sink, static_cast<GstClockTime>(timeout_ms) * GST_MSECOND);
    const auto decoded = std::chrono::steady_clock::now();
    if (!sample) {
        const auto bus = impl_->bus_error();
        error = "no decoded frame within " + std::to_string(timeout_ms) + " ms" +
                (bus.empty() ? "" : ": " + bus);
        return false;
    }
    bool ok = false;
    GstCaps* sample_caps = gst_sample_get_caps(sample);
    const GstStructure* caps = sample_caps ? gst_caps_get_structure(sample_caps, 0) : nullptr;
    const gchar* format = caps ? gst_structure_get_string(caps, "format") : nullptr;
    int w = 0, h = 0;
    GstMapInfo map;
    GstBuffer* out = gst_sample_get_buffer(sample);
    const std::size_t plane =
        static_cast<std::size_t>(impl_->width) * static_cast<std::size_t>(impl_->height);
    if (!format || std::string(format) != "Y42B" || !gst_structure_get_int(caps, "width", &w) ||
        !gst_structure_get_int(caps, "height", &h) || w != impl_->width || h != impl_->height) {
        error = "decoded caps differ from Y42B " + std::to_string(impl_->width) + "x" +
                std::to_string(impl_->height);
    } else if (!out || !gst_buffer_map(out, &map, GST_MAP_READ)) {
        error = "decoded buffer not readable";
    } else {
        if (map.size != 2 * plane) {
            error = "decoded buffer has " + std::to_string(map.size) + " bytes, expected " +
                    std::to_string(2 * plane);
        } else {
            if (bgr.rows != impl_->height || bgr.cols != impl_->width || bgr.type() != CV_8UC3)
                bgr.create(impl_->height, impl_->width, CV_8UC3);
            const std::uint8_t* y = map.data;
            const std::uint8_t* cb = y + plane;
            const std::uint8_t* cr = cb + plane / 2;
            jfif_ycc422_to_bgr(y, impl_->width, cb, cr, impl_->width / 2, impl_->width, impl_->height,
                               bgr.data, static_cast<int>(bgr.step));
            ok = true;
        }
        gst_buffer_unmap(out, &map);
    }
    gst_sample_unref(sample);
    if (timing) {
        timing->hw_us = elapsed_us(begin, decoded);
        timing->convert_us = elapsed_us(decoded, std::chrono::steady_clock::now());
    }
    return ok;
}

DecodeAgreement compare_bgr(const cv::Mat& a, const cv::Mat& b) {
    DecodeAgreement r;
    if (a.size() != b.size() || a.type() != CV_8UC3 || b.type() != CV_8UC3 || a.empty()) {
        r.mean_abs = r.over8_share = 1e9;
        r.max_abs = 255;
        return r;
    }
    std::uint64_t sum = 0, over8 = 0;
    for (int row = 0; row < a.rows; ++row) {
        const std::uint8_t* p = a.ptr<std::uint8_t>(row);
        const std::uint8_t* q = b.ptr<std::uint8_t>(row);
        for (int x = 0; x < a.cols; ++x) {
            int largest = 0;
            for (int c = 0; c < 3; ++c) {
                const int d = std::abs(int(p[3 * x + c]) - int(q[3 * x + c]));
                sum += static_cast<std::uint64_t>(d);
                largest = std::max(largest, d);
            }
            over8 += largest > 8 ? 1U : 0U;
            r.max_abs = std::max(r.max_abs, largest);
        }
    }
    const double pixels = static_cast<double>(a.rows) * a.cols;
    r.mean_abs = static_cast<double>(sum) / (3.0 * pixels);
    r.over8_share = static_cast<double>(over8) / pixels;
    return r;
}

}  // namespace perception
