/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: jpeg_ycc.hpp
 * Brief: Planar 4:2:2 JPEG YCbCr to BGR, computed the way libjpeg does it.
 *
 * Description:
 * The hardware MJPG decoder (nvv4l2decoder) returns the decoded YCbCr planes. Its own colour
 * conversion (nvvidconv) treats JPEG's full-range samples as limited range, about 8 levels off
 * (PSNR 27.9 dB against the SDK decode, measured on 1.8 on 2026-10-05), so the conversion is done
 * here instead.
 * The colour matrix uses libjpeg's integer tables (jdcolor.c: 16-bit fixed point, rounded). The
 * chroma is widened with libjpeg's h2v1 "fancy" triangle filter (jdsample.c), so the only
 * remaining difference to a software decode is the decoder's own inverse DCT: about 1 level on
 * 20 camera frames.
 */

#pragma once

#include <cstdint>

namespace perception {

// Converts one frame of full-range (JFIF) YCbCr with horizontally halved chroma (4:2:2) to
// interleaved BGR. width must be even and at least 4; height at least 1. Strides are in bytes and
// must cover a row (y and bgr: width and 3 * width; cb and cr: width / 2). Throws
// std::invalid_argument on a bad size, stride or null pointer. Reentrant: the chroma rows are
// widened into thread-local buffers.
void jfif_ycc422_to_bgr(const std::uint8_t* y, int y_stride, const std::uint8_t* cb,
                        const std::uint8_t* cr, int c_stride, int width, int height,
                        std::uint8_t* bgr, int bgr_stride);

}  // namespace perception
