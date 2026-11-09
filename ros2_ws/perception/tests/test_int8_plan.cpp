/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_int8_plan.cpp
 * Brief: Unit tests for the INT8 build plan (int8_plan): cache and list parsing, file identity,
 *        range and FP16 assignment, and every refusal.
 *
 * Description:
 * What it checks.
 * - The cache scale is the float whose bits are the hex digits, most significant first:
 *   3c010204 is 1/127 (the input range 1.0 of the 2026-10-09 COCO80 cache), and the range is
 *   scale * 127. MinMax and Entropy2 headers of TensorRT 10.3 are accepted; "\r\n" line ends
 *   too.
 * - The refusals: empty cache, other TensorRT version, header without a calibrator kind, a
 *   malformed line, a repeated name, a zero, negative, infinite or NaN scale, no scale at all;
 *   a list with a repeated or no name; a SHA-256 mismatch or a malformed configured SHA; a
 *   list SHA without a list.
 * - The assignment: ranges for the network input and every output in network order, none for
 *   the FP16 layers' outputs (all of them, also for a layer with two outputs), FP16 output type
 *   for those outputs except network outputs, unused cache names counted; refusal of an unknown
 *   or ambiguous FP16 layer name and of a tensor without a scale, each by its message.
 */

#include "common/config.hpp"
#include "modules/int8_plan.hpp"

#include <unistd.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int failures = 0;
#define CHECK(cond) do { if (!(cond)) { ++failures; std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #cond); } } while (0)

// True when f refuses with the INT8 plan prefix and, if given, a message containing `why`.
bool refuses(const std::function<void()>& f, const char* why = nullptr) {
    try {
        f();
    } catch (const std::runtime_error& e) {
        return std::strncmp(e.what(), "INT8 plan refused: ", 19) == 0 &&
               (why == nullptr || std::strstr(e.what(), why) != nullptr);
    }
    return false;
}

std::uint32_t float_bits(float f) {
    std::uint32_t b = 0;
    std::memcpy(&b, &f, sizeof(b));
    return b;
}

const char* kCache = "TRT-100300-MinMaxCalibration\nimages: 3c010204\n/a: 3f4b4cdc\noutput0: 40a53d74\n";

void test_cache_parse() {
    const auto c = yolo::parse_int8_range_cache(kCache);
    CHECK(c.header == "TRT-100300-MinMaxCalibration");
    CHECK(c.scales.size() == 3U);
    CHECK(float_bits(c.scales.at("images")) == 0x3c010204U);
    CHECK(float_bits(c.scales.at("/a")) == 0x3f4b4cdcU);
    CHECK(std::fabs(yolo::int8_dynamic_range(c.scales.at("images")) - 1.0F) < 1e-6F);
    CHECK(std::fabs(yolo::int8_dynamic_range(c.scales.at("output0")) - 655.7964F) < 1e-3F);
    // Entropy2 header, CRLF line ends, a name containing ": ", uppercase hex.
    const auto e = yolo::parse_int8_range_cache("TRT-100300-EntropyCalibration2\r\nx: y: 3C010204\r\n");
    CHECK(e.scales.size() == 1U && e.scales.count("x: y") == 1U);
    CHECK(float_bits(e.scales.at("x: y")) == 0x3c010204U);
    // A blank line between entries is skipped.
    const auto b = yolo::parse_int8_range_cache("TRT-100300-MinMaxCalibration\na: 3c010204\n\nb: 3c010204\n");
    CHECK(b.scales.size() == 2U);
}

void test_cache_refusals() {
    const std::vector<std::string> bad = {
        "",
        "\nimages: 3c010204\n",
        "TRT-100200-MinMaxCalibration\nimages: 3c010204\n",  // TensorRT 10.2
        "TRT-100300-MinMax\nimages: 3c010204\n",             // no calibrator kind
        "TRT-100300-MinMaxCalibration\n",                    // no scale
        "TRT-100300-MinMaxCalibration\nimages 3c010204\n",   // no ": "
        "TRT-100300-MinMaxCalibration\nimages: 3c01020\n",   // 7 digits
        "TRT-100300-MinMaxCalibration\nimages: 3c010204a\n", // 9 digits
        "TRT-100300-MinMaxCalibration\nimages: 3c01020g\n",
        "TRT-100300-MinMaxCalibration\n: 3c010204\n",        // empty name
        "TRT-100300-MinMaxCalibration\nimages: 3c010204\nimages: 3c010204\n",
        "TRT-100300-MinMaxCalibration\nimages: 00000000\n",  // zero
        "TRT-100300-MinMaxCalibration\nimages: bf800000\n",  // -1
        "TRT-100300-MinMaxCalibration\nimages: 7f800000\n",  // inf
        "TRT-100300-MinMaxCalibration\nimages: 7fc00000\n",  // NaN
        "TRT-100300-MinMaxCalibration\nimages: 7f000000\n",  // finite scale, infinite range
        "TRT-100300-Calibration\nimages: 3c010204\n",        // empty calibrator kind
    };
    for (const auto& text : bad) CHECK(refuses([&] { (void)yolo::parse_int8_range_cache(text); }));
}

void test_list_parse() {
    const auto l = yolo::parse_fp16_layer_list("# head tail\n\n  /model.23/Sigmoid  \n/model.0/conv/Conv\r\n");
    CHECK(l.size() == 2U && l[0] == "/model.23/Sigmoid" && l[1] == "/model.0/conv/Conv");
    CHECK(refuses([] { (void)yolo::parse_fp16_layer_list("/a\n/b\n/a\n"); }));
    CHECK(refuses([] { (void)yolo::parse_fp16_layer_list("# only a comment\n\n"); }));
    CHECK(refuses([] { (void)yolo::parse_fp16_layer_list("/a\nhalfq /b\n"); }, "halfq"));
}

yolo::Int8Plan plan_with(const std::string& cache, const std::vector<std::string>& fp16) {
    yolo::Int8Plan p;
    p.cache = yolo::parse_int8_range_cache(cache);
    p.fp16_layers = fp16;
    return p;
}

void test_assignment() {
    const std::vector<std::string> inputs = {"images"};
    std::vector<yolo::Int8LayerView> layers(3);
    layers[0].name = "/conv0";
    layers[0].float_outputs = {{"t0", false}};
    layers[1].name = "/head";
    layers[1].float_outputs = {{"t1", false}};
    layers[2].name = "/out";
    layers[2].float_outputs = {{"output0", true}};
    const std::string cache = "TRT-100300-MinMaxCalibration\nimages: 3c010204\nt0: 3f4b4cdc\noutput0: 40a53d74\nzz: 3c010204\n";
    const auto a = yolo::plan_int8_assignment(inputs, layers, plan_with(cache, {"/head"}));
    CHECK(a.ranges.size() == 3U);
    CHECK(a.ranges[0].first == "images" && a.ranges[1].first == "t0" && a.ranges[2].first == "output0");
    CHECK(std::fabs(a.ranges[0].second - 1.0F) < 1e-6F);
    CHECK(a.fp16_layers.size() == 1U && a.fp16_layers[0] == 1U);
    CHECK(a.half_outputs.size() == 1U && a.half_outputs[0] == "t1");
    CHECK(a.unranged_outputs == 1U);
    CHECK(a.unused_cache_entries == 1U);  // zz
    // No FP16 list: every float tensor needs a scale, t1 has none.
    CHECK(refuses([&] { (void)yolo::plan_int8_assignment(inputs, layers, plan_with(cache, {})); }));
    // Unknown FP16 layer name.
    CHECK(refuses([&] { (void)yolo::plan_int8_assignment(inputs, layers, plan_with(cache, {"/head", "/nope"})); },
                  "is not a layer"));
    // Missing scale for a tensor that needs one.
    const std::string short_cache = "TRT-100300-MinMaxCalibration\nimages: 3c010204\noutput0: 40a53d74\n";
    CHECK(refuses([&] { (void)yolo::plan_int8_assignment(inputs, layers, plan_with(short_cache, {"/head"})); },
                  "no INT8 range for tensor \"t0\""));
    // An FP16 name that matches two layers is ambiguous (refused even though every tensor has a
    // scale); the same duplicate unlisted is fine.
    auto dup = layers;
    dup.push_back(dup[1]);
    dup.back().float_outputs = {{"t2", false}};
    const std::string cache_t12 = "TRT-100300-MinMaxCalibration\nimages: 3c010204\nt0: 3f4b4cdc\nt1: 3f4b4cdc\n"
                                  "t2: 3f4b4cdc\noutput0: 40a53d74\n";
    CHECK(refuses([&] { (void)yolo::plan_int8_assignment(inputs, dup, plan_with(cache_t12, {"/head"})); },
                  "matches several layers"));
    // An FP16 layer that produces a network output: unranged, but its output type is not touched.
    const auto d = yolo::plan_int8_assignment(inputs, dup, plan_with(cache_t12, {"/out"}));
    CHECK(d.ranges.size() == 4U && d.fp16_layers.size() == 1U && d.fp16_layers[0] == 2U);
    CHECK(d.half_outputs.empty() && d.unranged_outputs == 1U);
    // An FP16 layer with two float outputs: both unranged, both FP16.
    auto split = layers;
    split[1].float_outputs = {{"t1a", false}, {"t1b", false}};
    const std::string cache_split = "TRT-100300-MinMaxCalibration\nimages: 3c010204\nt0: 3f4b4cdc\noutput0: 40a53d74\n";
    const auto m = yolo::plan_int8_assignment(inputs, split, plan_with(cache_split, {"/head"}));
    CHECK(m.ranges.size() == 3U && m.unranged_outputs == 2U);
    CHECK(m.half_outputs.size() == 2U && m.half_outputs[0] == "t1a" && m.half_outputs[1] == "t1b");
}

void test_load() {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / ("int8_plan_test_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    const std::string ranges = (dir / "ranges.cache").string(), list = (dir / "fp16.txt").string();
    const std::string list_text = "# kept FP16\n/head\n";
    std::ofstream(ranges, std::ios::binary) << kCache;
    std::ofstream(list, std::ios::binary) << list_text;
    const std::string rs = yolo::sha256_hex(kCache), ls = yolo::sha256_hex(list_text);
    const auto p = yolo::load_int8_plan(ranges, rs, list, ls);
    CHECK(p.cache.scales.size() == 3U && p.fp16_layers.size() == 1U && p.fp16_layers[0] == "/head");
    const auto q = yolo::load_int8_plan(ranges, rs, "", "");  // no FP16 list
    CHECK(q.fp16_layers.empty());
    std::string wrong = rs;
    wrong[0] = wrong[0] == '0' ? '1' : '0';
    CHECK(refuses([&] { (void)yolo::load_int8_plan(ranges, wrong, list, ls); }));      // ranges SHA
    CHECK(refuses([&] { (void)yolo::load_int8_plan(ranges, rs, list, wrong); }));      // list SHA
    CHECK(refuses([&] { (void)yolo::load_int8_plan(ranges, "ABC", list, ls); }));      // malformed SHA
    std::string upper = rs;
    for (auto& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    CHECK(rs == upper || refuses([&] { (void)yolo::load_int8_plan(ranges, upper, "", ""); }));  // uppercase SHA
    CHECK(refuses([&] { (void)yolo::load_int8_plan("", rs, "", ""); }));                // no ranges file
    CHECK(refuses([&] { (void)yolo::load_int8_plan(ranges, rs, "", ls); }));            // list SHA without list
    CHECK(refuses([&] { (void)yolo::load_int8_plan((dir / "absent").string(), rs, "", ""); }));
    fs::remove_all(dir);
}

}  // namespace

int main() {
    test_cache_parse();
    test_cache_refusals();
    test_list_parse();
    test_assignment();
    test_load();
    if (failures != 0) {
        std::fprintf(stderr, "test_int8_plan: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("test_int8_plan: all checks passed\n");
    return 0;
}
