/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: int8_plan.cpp
 * Brief: INT8 build plan for a DLA engine: cache parsing, file identity, range and FP16 assignment.
 *
 * Description:
 * See int8_plan.hpp. The rules mirror the Orin experiment that passed all eight COCO80 acceptance
 * lines (2026-10-09, st1009a cal_stem_g0504): ranges for the network input and every
 * floating-point layer output, in network order; the FP16-listed layers' outputs left without a
 * range. Unlike the experiment's scratch builder, which skipped a missing name or scale silently,
 * every gap here refuses the build.
 */
#include "modules/int8_plan.hpp"

#include "common/config.hpp"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace yolo {

namespace {

constexpr const char* kCacheHeaderPrefix = "TRT-100300-";
constexpr const char* kCacheHeaderKind = "Calibration";  // MinMaxCalibration, EntropyCalibration2, ...

[[noreturn]] void refuse(const std::string& message) {
    throw std::runtime_error("INT8 plan refused: " + message);
}

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) refuse("cannot read " + path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool is_sha256_hex(const std::string& value) {
    if (value.size() != 64U) return false;
    for (const char c : value) {
        if (!std::isxdigit(static_cast<unsigned char>(c)) || std::isupper(static_cast<unsigned char>(c))) return false;
    }
    return true;
}

// Lines of a text file without the line ends ("\r\n" accepted).
std::vector<std::string> split_lines(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

std::string trim(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

}  // namespace

Int8RangeCache parse_int8_range_cache(const std::string& text) {
    const std::vector<std::string> lines = split_lines(text);
    if (lines.empty() || lines.front().empty()) refuse("calibration cache is empty or has no header line");
    Int8RangeCache cache;
    cache.header = lines.front();
    const std::size_t prefix_size = std::strlen(kCacheHeaderPrefix);
    const std::size_t kind = cache.header.find(kCacheHeaderKind, prefix_size);
    if (cache.header.rfind(kCacheHeaderPrefix, 0) != 0 || kind == std::string::npos || kind == prefix_size) {
        refuse("calibration cache header \"" + cache.header + "\" is not a TensorRT 10.3 cache (TRT-100300-...Calibration)");
    }
    for (std::size_t i = 1; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        if (line.empty()) continue;
        // The scale is the last field; a tensor name may itself contain ": ".
        const std::size_t colon = line.rfind(": ");
        const std::string name = colon == std::string::npos ? std::string() : line.substr(0, colon);
        const std::string hex = colon == std::string::npos ? std::string() : line.substr(colon + 2);
        bool hex_ok = hex.size() == 8U;
        for (const char c : hex) hex_ok = hex_ok && std::isxdigit(static_cast<unsigned char>(c));
        if (name.empty() || !hex_ok) refuse("calibration cache line " + std::to_string(i + 1) + " is not \"<name>: <8 hex digits>\"");
        const auto bits = static_cast<std::uint32_t>(std::stoul(hex, nullptr, 16));
        float scale = 0.0F;
        std::memcpy(&scale, &bits, sizeof(scale));
        if (!std::isfinite(scale) || !(scale > 0.0F) || !std::isfinite(int8_dynamic_range(scale))) {
            refuse("calibration cache scale of \"" + name + "\" is not finite and positive with a finite range");
        }
        if (!cache.scales.emplace(name, scale).second) refuse("calibration cache names \"" + name + "\" twice");
    }
    if (cache.scales.empty()) refuse("calibration cache has no tensor scale");
    return cache;
}

std::vector<std::string> parse_fp16_layer_list(const std::string& text) {
    std::vector<std::string> names;
    std::set<std::string> seen;
    for (const std::string& raw : split_lines(text)) {
        const std::string name = trim(raw);
        if (name.empty() || name[0] == '#') continue;
        if (name.rfind("halfq ", 0) == 0) refuse("FP16 layer list line \"" + name + "\": halfq is not supported");
        if (!seen.insert(name).second) refuse("FP16 layer list names \"" + name + "\" twice");
        names.push_back(name);
    }
    if (names.empty()) refuse("FP16 layer list has no layer name");
    return names;
}

float int8_dynamic_range(float scale) { return scale * 127.0F; }

Int8Plan load_int8_plan(const std::string& ranges_path, const std::string& ranges_sha256,
                        const std::string& fp16_layers_path, const std::string& fp16_layers_sha256) {
    if (ranges_path.empty()) refuse("int8 needs a ranges file (calibration cache)");
    if (!is_sha256_hex(ranges_sha256)) refuse("int8_ranges_sha256 must be 64 lowercase hex digits");
    if (!fp16_layers_path.empty() && !is_sha256_hex(fp16_layers_sha256)) {
        refuse("int8_fp16_layers_sha256 must be 64 lowercase hex digits");
    }
    if (fp16_layers_path.empty() && !fp16_layers_sha256.empty()) refuse("int8_fp16_layers_sha256 given without int8_fp16_layers");
    Int8Plan plan;
    plan.ranges_path = ranges_path;
    plan.ranges_sha256 = ranges_sha256;
    plan.fp16_layers_path = fp16_layers_path;
    plan.fp16_layers_sha256 = fp16_layers_sha256;
    const std::string ranges_text = read_file(ranges_path);
    const std::string ranges_actual = sha256_hex(ranges_text);
    if (ranges_actual != ranges_sha256) refuse("SHA-256 of " + ranges_path + " is " + ranges_actual + ", configured " + ranges_sha256);
    plan.cache = parse_int8_range_cache(ranges_text);
    if (!fp16_layers_path.empty()) {
        const std::string list_text = read_file(fp16_layers_path);
        const std::string list_actual = sha256_hex(list_text);
        if (list_actual != fp16_layers_sha256) {
            refuse("SHA-256 of " + fp16_layers_path + " is " + list_actual + ", configured " + fp16_layers_sha256);
        }
        plan.fp16_layers = parse_fp16_layer_list(list_text);
    }
    return plan;
}

Int8Assignment plan_int8_assignment(const std::vector<std::string>& float_inputs,
                                    const std::vector<Int8LayerView>& layers, const Int8Plan& plan) {
    Int8Assignment out;
    std::unordered_map<std::string, std::vector<std::size_t>> by_name;
    for (std::size_t i = 0; i < layers.size(); ++i) by_name[layers[i].name].push_back(i);

    std::set<std::string> unranged;
    for (const std::string& name : plan.fp16_layers) {
        const auto it = by_name.find(name);
        if (it == by_name.end()) refuse("FP16 layer \"" + name + "\" is not a layer of the network");
        if (it->second.size() != 1U) refuse("FP16 layer name \"" + name + "\" matches several layers");
        const std::size_t index = it->second.front();
        out.fp16_layers.push_back(index);
        for (const auto& output : layers[index].float_outputs) {
            unranged.insert(output.tensor);
            if (!output.network_output) out.half_outputs.push_back(output.tensor);
        }
    }
    out.unranged_outputs = unranged.size();

    std::set<std::string> network_tensors;
    const auto assign = [&](const std::string& tensor) {
        network_tensors.insert(tensor);
        if (unranged.count(tensor) != 0U) return;
        const auto it = plan.cache.scales.find(tensor);
        if (it == plan.cache.scales.end()) refuse("no INT8 range for tensor \"" + tensor + "\" (not in the calibration cache)");
        out.ranges.emplace_back(tensor, int8_dynamic_range(it->second));
    };
    for (const std::string& tensor : float_inputs) assign(tensor);
    for (const auto& layer : layers) {
        for (const auto& output : layer.float_outputs) assign(output.tensor);
    }
    for (const auto& entry : plan.cache.scales) out.unused_cache_entries += network_tensors.count(entry.first) == 0U ? 1U : 0U;
    return out;
}

}  // namespace yolo
