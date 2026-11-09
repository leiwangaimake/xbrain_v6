/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: post_dump.hpp
 * Brief: Diagnostic-only raw postprocess tensor dump for offline bit-exact regression.
 *
 * Description:
 * Format: 8-byte magic "PDUMP1\0\0", uint64 JSON length, JSON, then raw float32
 * arrays in the order declared by the JSON "arrays" list. Output summaries use
 * FNV-1a 64 over mask bytes and exact float bit patterns.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "common/types.hpp"

namespace perception::post_dump {

inline std::uint64_t fnv1a(const unsigned char* data, std::size_t size, std::uint64_t h = 14695981039346656037ULL) noexcept {
    for (std::size_t i = 0; i < size; ++i) h = (h ^ data[i]) * 1099511628211ULL;
    return h;
}

inline std::uint64_t mat_hash(const cv::Mat& m) {
    std::uint64_t h = 14695981039346656037ULL;
    if (m.empty()) return 0;
    for (int y = 0; y < m.rows; ++y) h = fnv1a(m.ptr<unsigned char>(y), static_cast<std::size_t>(m.cols) * m.elemSize(), h);
    return h;
}

inline std::uint32_t float_bits(float v) noexcept { std::uint32_t b; std::memcpy(&b, &v, sizeof(b)); return b; }

inline std::string summarize(const std::vector<yolo::Detection>& detections) {
    std::ostringstream out;
    out << "count " << detections.size() << "\n";
    for (const auto& d : detections) {
        out << "det source=" << static_cast<int>(d.source) << " class=" << d.class_id << " orig=" << d.original_class_id
            << " conf_bits=" << std::hex << float_bits(d.confidence) << std::dec
            << " bbox=" << d.bbox.x << "," << d.bbox.y << "," << d.bbox.width << "," << d.bbox.height
            << " low=" << static_cast<int>(d.low_confidence_candidate)
            << " mask=" << d.mask.cols << "x" << d.mask.rows << "t" << d.mask.type() << " mask_hash=" << std::hex << mat_hash(d.mask)
            << " full=" << std::dec << d.full_image_mask.cols << "x" << d.full_image_mask.rows << " full_hash=" << std::hex
            << mat_hash(d.full_image_mask) << std::dec << " name=" << d.class_name << "\n";
    }
    return out.str();
}

}  // namespace perception::post_dump
