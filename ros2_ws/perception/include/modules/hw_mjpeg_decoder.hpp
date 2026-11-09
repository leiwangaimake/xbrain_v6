/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: hw_mjpeg_decoder.hpp
 * Brief: Hardware MJPG decode for the Gemini colour stream (GStreamer nvv4l2decoder), BGR out.
 *
 * Description:
 * Behind the runtime switch gemini_mjpeg_decode. The integration lead approved the hardware
 * decoder as a resource on 2026-10-08 (issues.md 3.5 A, nvv4l2decoder from the whitelist), so the
 * production value is nvv4l2; sdk keeps the SDK software decode.
 * Pipeline: appsrc (image/jpeg) -> nvv4l2decoder mjpeg=1 -> nvvidconv -> Y42B in system memory ->
 * appsink. Only the plugins of the checked whitelist are used (nvvideo4linux2, nvvidconv, app).
 * nvvidconv only copies the planes; the colour conversion is jfif_ycc422_to_bgr, because
 * nvvidconv's own conversion treats JPEG's full range as limited range.
 * Cost on 1.8 (probe, 2026-10-05): about 6 ms wall and 2.6 ms CPU per 1280x800 frame for the
 * GStreamer part, plus about 4.5 ms for the conversion. The SDK's software decode measured about
 * 12.5 ms wall in the 3-core service windows.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace perception {

class HwMjpegDecoder {
public:
    struct Timing {
        double hw_us = 0.0;       // push of the JPEG to arrival of the decoded planes (wall)
        double convert_us = 0.0;  // YCbCr 4:2:2 to BGR on the calling thread (wall)
    };

    HwMjpegDecoder();
    ~HwMjpegDecoder();
    HwMjpegDecoder(const HwMjpegDecoder&) = delete;
    HwMjpegDecoder& operator=(const HwMjpegDecoder&) = delete;

    // Builds and starts the pipeline for one raster size (width even, >= 4). Returns false with a
    // reason on any failure; the object is then closed and may be opened again.
    bool open(int width, int height, std::string& error);
    // Decodes one complete JPEG into bgr (allocated as CV_8UC3 width x height when its size or
    // type differs, so a fresh cv::Mat gets its own buffer). Blocks until the decoded planes arrive,
    // at most timeout_ms. On failure returns false with a reason and leaves bgr unspecified.
    bool decode(const std::uint8_t* jpeg, std::size_t size, cv::Mat& bgr, std::string& error,
                Timing* timing = nullptr, int timeout_ms = 500);
    void close();
    bool is_open() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Decoded-image agreement used by the start-up self-check and the tests: mean absolute difference
// per channel sample, and the share of pixels whose largest channel difference exceeds 8 levels.
// The accepted limits (user, 2026-10-05) are mean <= 1.5 and share <= 0.001.
struct DecodeAgreement {
    double mean_abs = 0.0;
    double over8_share = 0.0;
    int max_abs = 0;
};
DecodeAgreement compare_bgr(const cv::Mat& a, const cv::Mat& b);
constexpr double kHwDecodeMeanAbsLimit = 1.5;
constexpr double kHwDecodeOver8Limit = 0.001;

}  // namespace perception
