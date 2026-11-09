/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_psc4_golden.cpp
 * Brief: Host test: PSC-4 golden assets (19 section 11 / 13.2, reply batch 7 B7-3).
 *
 * Description:
 * Canonical output digest, manifest parse/write, identity and output checks,
 * with a negative case for every field. No SDK, OpenCV or TensorRT.
 */

#include "service/psc4_golden.hpp"

#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace perception::service::psc4;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

std::array<std::uint8_t, 12> kMaskA{{0, 255, 255, 0, 0, 255, 255, 255, 0, 0, 0, 255}};   // 3x4
std::array<std::uint8_t, 6> kMaskB{{255, 0, 255, 0, 255, 0}};                             // 2x3

CanonMat mat(const std::uint8_t* d, int rows, int cols, std::size_t stride = 0) {
    CanonMat m;
    m.rows = rows; m.cols = cols; m.type = 0; m.data = d;
    m.row_bytes = static_cast<std::size_t>(cols);
    m.stride = stride ? stride : m.row_bytes;
    return m;
}
CanonDetection det(int source, int cls, float conf, const char* name, const std::uint8_t* mask, int r, int c) {
    CanonDetection d;
    d.x = 10 * cls; d.y = 5; d.w = 40; d.h = 30; d.confidence = conf;
    d.class_id = cls; d.original_class_id = cls; d.source = source; d.class_name = name;
    d.mask = mat(mask, r, c);
    return d;
}
std::vector<CanonDetection> scene() {
    return {det(kSourceCoco80, 0, 0.91F, "person", kMaskA.data(), 3, 4),
            det(kSourceActive12, 7, 0.66F, "barrier", kMaskB.data(), 2, 3),
            det(kSourceCoco80, 56, 0.52F, "chair", kMaskB.data(), 2, 3)};
}
Manifest sealed(const PartitionOutput& out, const std::string& partition) {
    Manifest m;
    m.rule = std::string(kRule); m.partition = partition;
    m.engine_file = partition == "coco80" ? "yolo11n-seg-dla0-fp16.engine" : "active12.engine";
    m.engine_sha256 = std::string(64, 'a'); m.engine_bytes = 7938628;
    m.input_file = "input.png"; m.input_sha256 = std::string(64, 'b'); m.input_pixels_sha256 = std::string(64, 'c');
    m.input_rows = 800; m.input_cols = 1280;
    m.pipeline_digest = std::string(64, 'd'); m.runtime_digest = std::string(64, 'e');
    m.expected_detections = out.detections; m.expected_map_detections = out.map_detections;
    m.expected_sha256 = out.sha256; m.timeout_ms = 20000; m.sealed_repeats = 20;
    m.build_env = {"UNKNOWN", "no build record found for this engine"};
    m.dual_runtime_sha256 = std::string(64, '5'); m.dual_schema_sha256 = std::string(64, '6');
    return m;
}
Observed observed_for(const Manifest& m) {
    Observed o;
    o.engine_file = m.engine_file; o.engine_sha256 = m.engine_sha256; o.engine_bytes = m.engine_bytes;
    o.input_sha256 = m.input_sha256; o.input_pixels_sha256 = m.input_pixels_sha256;
    o.input_rows = m.input_rows; o.input_cols = m.input_cols;
    o.pipeline_digest = m.pipeline_digest; o.runtime_digest = m.runtime_digest;
    o.dual_runtime_sha256 = m.dual_runtime_sha256; o.dual_schema_sha256 = m.dual_schema_sha256;
    return o;
}
bool throws(const std::string& text, const char* expect) {
    try { (void)parse_manifest(text); }
    catch (const std::runtime_error& e) { return std::string(e.what()).find(expect) != std::string::npos; }
    return false;
}
std::string replace_once(std::string s, const std::string& a, const std::string& b) {
    const auto at = s.find(a);
    if (at == std::string::npos) return s + "<<NOT FOUND " + a + ">>";
    return s.replace(at, a.size(), b);
}
}  // namespace

int main() {
    const auto dets = scene();
    const std::vector<CanonDetection> none;
    std::array<std::uint8_t, 4> safe{{1, 0, 0, 1}};
    const auto safe_mask = mat(safe.data(), 2, 2);

    // ---- canonical digest ----
    const auto coco = digest_partition(kSourceCoco80, dets, none, &safe_mask);
    const auto act = digest_partition(kSourceActive12, dets, none, &safe_mask);
    CHECK(coco.detections == 2 && act.detections == 1 && coco.map_detections == 0);
    CHECK(coco.sha256.size() == 64 && coco.sha256 != act.sha256);
    CHECK(digest_partition(kSourceCoco80, dets, none, &safe_mask).sha256 == coco.sha256);   // deterministic
    // the traversable mask belongs to the active12 partition only
    std::array<std::uint8_t, 4> safe2{{1, 0, 1, 1}};
    const auto safe_mask2 = mat(safe2.data(), 2, 2);
    CHECK(digest_partition(kSourceCoco80, dets, none, &safe_mask2).sha256 == coco.sha256);
    CHECK(digest_partition(kSourceActive12, dets, none, &safe_mask2).sha256 != act.sha256);
    // every field of a detection is covered; the other partition is untouched
    const auto mutate = [&](auto change) {
        auto d = scene(); change(d[0]);
        return std::make_pair(digest_partition(kSourceCoco80, d, none, &safe_mask).sha256,
                              digest_partition(kSourceActive12, d, none, &safe_mask).sha256);
    };
    const auto conf_bit = mutate([](CanonDetection& d) {
        std::uint32_t b; std::memcpy(&b, &d.confidence, 4); b ^= 1U; std::memcpy(&d.confidence, &b, 4); });
    CHECK(conf_bit.first != coco.sha256 && conf_bit.second == act.sha256);   // one ULP of confidence
    CHECK(mutate([](CanonDetection& d) { d.x += 1; }).first != coco.sha256);
    CHECK(mutate([](CanonDetection& d) { d.h += 1; }).first != coco.sha256);
    CHECK(mutate([](CanonDetection& d) { d.class_id += 1; }).first != coco.sha256);
    CHECK(mutate([](CanonDetection& d) { d.original_class_id += 1; }).first != coco.sha256);
    CHECK(mutate([](CanonDetection& d) { d.class_name = "persoN"; }).first != coco.sha256);
    std::array<std::uint8_t, 12> mask_flip = kMaskA; mask_flip[5] ^= 0xFF;
    CHECK(mutate([&](CanonDetection& d) { d.mask.data = mask_flip.data(); }).first != coco.sha256);
    CHECK(mutate([](CanonDetection& d) { d.mask.rows = 2; }).first != coco.sha256);
    CHECK(mutate([&](CanonDetection& d) { d.full_image_mask = mat(kMaskB.data(), 2, 3); }).first != coco.sha256);
    // order matters (the module's output order is part of the result)
    auto swapped = scene(); std::swap(swapped[0], swapped[2]);
    CHECK(digest_partition(kSourceCoco80, swapped, none, &safe_mask).sha256 != coco.sha256);
    // a padded (non-continuous) mask hashes its used bytes only: same as continuous
    std::array<std::uint8_t, 16> padded{};
    for (int r = 0; r < 3; ++r) std::memcpy(padded.data() + 5 * r, kMaskA.data() + 4 * r, 4);
    auto strided = scene(); strided[0].mask = mat(padded.data(), 3, 4, 5);
    CHECK(digest_partition(kSourceCoco80, strided, none, &safe_mask).sha256 == coco.sha256);
    // map detections are counted and hashed separately from detections
    const auto moved = digest_partition(kSourceCoco80, {dets[0]}, {dets[2]}, &safe_mask);
    CHECK(moved.detections == 1 && moved.map_detections == 1 && moved.sha256 != coco.sha256);

    // ---- manifest round trip ----
    const auto m = sealed(coco, "coco80");
    const auto text = write_manifest(m);
    const auto back = parse_manifest(text);
    CHECK(write_manifest(back) == text && back.expected_sha256 == coco.sha256 && back.build_env.status == "UNKNOWN");

    // ---- manifest negatives: every one refuses, naming the field ----
    CHECK(throws("", "not an object") || throws("", "missing"));
    CHECK(throws("[1, 2]", "not an object"));
    CHECK(text.find("\"version\": 2") != std::string::npos);
    // a v1 manifest (before B8-1) is refused with a re-seal message, never upgraded
    CHECK(throws(replace_once(text, "\"version\": 2", "\"version\": 1"), "re-seal required"));
    CHECK(throws(replace_once(text, "\"version\": 2", "\"version\": 3"), "unknown version"));
    // consumed_inputs (B8-1): present, both-or-neither, lowercase SHA or null
    const std::string ci_key = "  \"consumed_inputs\": {\"dual_class_schema\": \"" + std::string(64, '6') +
                               "\", \"dual_runtime_config\": \"" + std::string(64, '5') + "\"},\n";
    CHECK(text.find(ci_key) != std::string::npos);
    CHECK(throws(replace_once(text, ci_key, ""), "missing consumed_inputs"));
    CHECK(throws(replace_once(text, "\"dual_runtime_config\": \"" + std::string(64, '5') + "\"", "\"dual_runtime_config\": null"),
                 "both null or both set"));
    CHECK(throws(replace_once(text, "\"dual_runtime_config\": \"" + std::string(64, '5'), "\"dual_runtime_config\": \"" + std::string(64, 'Z')),
                 "consumed_inputs.dual_runtime_config"));
    CHECK(throws(replace_once(text, "\"dual_class_schema\": \"", "\"extra\": 1, \"dual_class_schema\": \""), "exactly"));
    // sealed without the dual model (the service no longer can: dual_model.enabled
    // is required and true): both null round-trips, and is compared as "not consumed"
    auto single = m; single.dual_runtime_sha256.clear(); single.dual_schema_sha256.clear();
    const auto single_text = write_manifest(single);
    CHECK(single_text.find("\"consumed_inputs\": {\"dual_class_schema\": null, \"dual_runtime_config\": null}") != std::string::npos);
    const auto single_back = parse_manifest(single_text);
    CHECK(single_back.dual_runtime_sha256.empty() && single_back.dual_schema_sha256.empty());
    CHECK(throws(replace_once(text, "psc4-canon-v1", "psc4-canon-v0"), "unknown comparison rule"));
    CHECK(throws(replace_once(text, "\"partition\": \"coco80\"", "\"partition\": \"coco\""), "unknown partition"));
    CHECK(throws(replace_once(text, "\"engine_sha256\": \"" + std::string(64, 'a'), "\"engine_sha256\": \"" + std::string(64, 'A')), "engine_sha256"));
    CHECK(throws(replace_once(text, "\"input_sha256\": \"" + std::string(64, 'b'), "\"input_sha256\": \"" + std::string(63, 'b')), "input_sha256"));
    CHECK(throws(replace_once(text, "\"input_file\": \"input.png\"", "\"input_file\": \"../input.png\""), "input_file"));
    CHECK(throws(replace_once(text, "\"input_file\": \"input.png\"", "\"input_file\": \"x/input.png\""), "input_file"));
    CHECK(throws(replace_once(text, "\"sealed_repeats\": 20", "\"sealed_repeats\": 1"), "sealed_repeats"));
    CHECK(throws(replace_once(text, "\"timeout_ms\": 20000", "\"timeout_ms\": 0"), "positive"));
    CHECK(throws(replace_once(text, "\"timeout_ms\": 20000", "\"timeout_ms\": -5"), "timeout_ms"));
    CHECK(throws(replace_once(text, "\"status\": \"UNKNOWN\"", "\"status\": \"TODAY\""), "build_env.status"));
    CHECK(throws(replace_once(text, "\"expected_detections\": 2", "\"expected_detections\": 0"), "empty"));
    CHECK(throws(replace_once(text, "  \"runtime_digest\": \"" + std::string(64, 'e') + "\",\n", ""), "missing runtime_digest"));
    CHECK(throws(replace_once(text, "\"engine_bytes\": 7938628", "\"engine_bytes\": \"many\""), "engine_bytes"));

    // ---- identity: each mismatch is reported by name ----
    const auto obs = observed_for(m);
    CHECK(check_identity(m, obs, "coco80").empty());
    CHECK(check_identity(m, obs, "active12").find("partition") == 0);
    auto o2 = obs; o2.engine_sha256 = std::string(64, 'f');
    CHECK(check_identity(m, o2, "coco80").find("engine_sha256") == 0);   // wrong engine
    o2 = obs; o2.engine_file = "other.engine";
    CHECK(check_identity(m, o2, "coco80").find("engine_file") == 0);
    o2 = obs; o2.engine_bytes -= 1;
    CHECK(check_identity(m, o2, "coco80").find("engine_bytes") == 0);
    o2 = obs; o2.input_sha256 = std::string(64, '0');
    CHECK(check_identity(m, o2, "coco80").find("input_sha256") == 0);     // corrupted input asset
    o2 = obs; o2.input_pixels_sha256 = std::string(64, '0');
    CHECK(check_identity(m, o2, "coco80").find("input_pixels_sha256") == 0);
    o2 = obs; o2.input_cols = 1920;
    CHECK(check_identity(m, o2, "coco80").find("input size") == 0);
    o2 = obs; o2.pipeline_digest = std::string(64, '1');
    CHECK(check_identity(m, o2, "coco80").find("pipeline_digest") == 0);  // configuration changed
    o2 = obs; o2.runtime_digest = std::string(64, '1');
    CHECK(check_identity(m, o2, "coco80").find("runtime_digest") == 0);
    // B8-1: same path, different content of either dual-model JSON
    o2 = obs; o2.dual_runtime_sha256 = std::string(64, '7');
    CHECK(check_identity(m, o2, "coco80").find("consumed_inputs.dual_runtime_config") == 0);
    o2 = obs; o2.dual_schema_sha256 = std::string(64, '7');
    CHECK(check_identity(m, o2, "coco80").find("consumed_inputs.dual_class_schema") == 0);
    // sealed with the dual model, started without it (and the reverse): refused
    o2 = obs; o2.dual_runtime_sha256.clear(); o2.dual_schema_sha256.clear();
    CHECK(check_identity(m, o2, "coco80").find("consumed_inputs.dual_runtime_config") == 0);
    auto single_obs = observed_for(single);
    CHECK(check_identity(single, single_obs, "coco80").empty());
    single_obs.dual_runtime_sha256 = std::string(64, '5'); single_obs.dual_schema_sha256 = std::string(64, '6');
    CHECK(check_identity(single, single_obs, "coco80").find("consumed_inputs") == 0);

    // ---- output: counts and digest ----
    CHECK(check_output(m, coco).empty());
    CHECK(check_output(m, conf_bit.first == coco.sha256 ? coco : PartitionOutput{2, 0, conf_bit.first}).find("digest") != std::string::npos);
    CHECK(check_output(m, PartitionOutput{1, 0, coco.sha256}).find("counts") == 0);
    CHECK(check_output(m, PartitionOutput{2, 1, coco.sha256}).find("counts") == 0);
    // a tampered expected value (A19-BOOT-3) is a mismatch, never a pass
    auto tampered = m; tampered.expected_sha256[0] = tampered.expected_sha256[0] == 'a' ? 'b' : 'a';
    CHECK(!check_output(tampered, coco).empty());

    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks
              << ",\"failures\":" << failures << ",\"scope\":\"psc4_golden_host\",\"sdk\":false,\"engine\":false}"
              << std::endl;
    return failures ? 1 : 0;
}
