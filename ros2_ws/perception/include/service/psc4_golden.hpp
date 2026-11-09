/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: psc4_golden.hpp
 * Brief: PSC-4 golden frame (19 section 11 PSC-4, section 13.2).
 *
 * Description:
 * The sealed assets, their manifest, the canonical output digest and the checks.
 * Layout (reply batch 7, B7-3; a perception-side convention, NOT a contract
 * key): every required engine has a sibling directory
 *
 *     <engine path>.psc4/manifest.json
 *     <engine path>.psc4/input.png          fixed, lossless, real input
 *
 * The manifest binds, independently per engine: the engine bytes (SHA-256 and
 * size), the input file and its decoded pixels, the resolved pipeline section
 * and runtime switches the output depends on, the content of the dual-model
 * JSON files the configuration actually consumed (manifest v2, reply batch 8
 * B8-1; the format still allows both null, "not read", but the service always
 * reads both since dual_model.enabled is required and true, so a null pair
 * never matches), the comparison rule version, the
 * sealed expected output digest and counts, and the build environment record
 * (RECORDED_AT_BUILD / RECOVERED_BY_ASSOCIATION / UNKNOWN -- never today's
 * environment written as the build environment).
 *
 * The expected value is produced ONCE by the seal tool, which requires every
 * repeat to agree and every partition to be non-empty. The service never
 * derives an expected value at startup, never rebuilds an engine, and refuses
 * to start (exit 78, PSC-4) on a missing or malformed asset, any identity
 * mismatch, a failed or timed-out golden run, or a different output.
 *
 * Canonical output ("psc4-canon-v1"): per model partition (the detection's
 * source), in the module's output order, fixed-width little-endian fields and
 * row-major mask bytes without padding:
 *   u32 count; per detection: i32 x, y, w, h; u32 confidence bits; i32 class_id,
 *   original_class_id, source; u32 name length + bytes; mask; full_image_mask
 *   mask := i32 rows, cols, type; rows*row_bytes bytes
 * then the map detections the same way, and for the active12 partition one
 * more mask: the retired slot of the former safe traversable mask. The service
 * always fills it with the empty matrix (i32 0, 0, 0) since level-3 trim; it
 * stays so psc4-canon-v1 digests remain comparable. SHA-256 over those bytes.
 * This covers every field the module hands downstream; it is not a claim that
 * every raw tensor element of the engines is covered.
 *
 * Pure: no SDK, OpenCV, TensorRT or device access (the OpenCV adapter lives in
 * psc4_run.hpp, used only by device code).
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace perception::service::psc4 {

inline constexpr std::string_view kRule = "psc4-canon-v1";
// v2 (reply batch 8, B8-1): adds consumed_inputs. A v1 manifest is REFUSED
// with a re-seal message; it is never upgraded in place.
inline constexpr int kManifestVersion = 2;
inline constexpr int kSourceCoco80 = 0;    // yolo::DetectionSource::coco80
inline constexpr int kSourceActive12 = 1;  // yolo::DetectionSource::active12

inline std::string asset_dir(const std::string& engine_path) { return engine_path + ".psc4"; }
std::string_view partition_name(int source) noexcept;   // "coco80" / "active12" / ""

struct CanonMat {
    int rows = 0, cols = 0, type = 0;
    const std::uint8_t* data = nullptr;
    std::size_t row_bytes = 0;   // bytes of one row actually used
    std::size_t stride = 0;      // bytes between row starts (>= row_bytes)
};
struct CanonDetection {
    int x = 0, y = 0, w = 0, h = 0;
    float confidence = 0.0F;
    int class_id = -1, original_class_id = -1, source = -1;
    std::string_view class_name;
    CanonMat mask, full_image_mask;
};
struct PartitionOutput {
    std::size_t detections = 0, map_detections = 0;
    std::string sha256;
};
// Digest of ONE partition: only the detections whose source matches are
// included; the retired mask slot is part of the active12 partition only
// (nullptr hashes as the empty matrix).
PartitionOutput digest_partition(int source, const std::vector<CanonDetection>& detections,
                                const std::vector<CanonDetection>& map_detections,
                                const CanonMat* retired_mask_slot);

struct BuildEnv {
    std::string status;   // RECORDED_AT_BUILD | RECOVERED_BY_ASSOCIATION | UNKNOWN
    std::string detail;   // where the record came from, or why it is unknown
};
struct Manifest {
    int version = kManifestVersion;
    std::string rule;
    std::string partition;                 // "coco80" | "active12"
    std::string engine_file;               // basename of the engine this dir belongs to
    std::string engine_sha256;
    std::uint64_t engine_bytes = 0;
    std::string input_file;                // basename inside the .psc4 directory
    std::string input_sha256;              // raw file bytes
    std::string input_pixels_sha256;       // decoded BGR bytes
    int input_rows = 0, input_cols = 0;
    std::string pipeline_digest;           // resolved pipeline section, canonical dump
    std::string runtime_digest;            // resolved runtime switches
    // Content SHA-256 of the dual-model JSON files as parsed; empty = not
    // consumed (only a manifest sealed without the dual model). Both or neither.
    std::string dual_runtime_sha256, dual_schema_sha256;
    std::uint64_t expected_detections = 0, expected_map_detections = 0;
    std::string expected_sha256;
    std::uint32_t timeout_ms = 0;          // bound on the golden inference wait
    std::uint32_t sealed_repeats = 0;      // all agreed at seal time
    BuildEnv build_env;
};
// Strict: every key present with the right type, version and rule known,
// SHA fields 64 lowercase hex, counts / sizes positive where required.
// Throws std::runtime_error naming the offending key.
Manifest parse_manifest(const std::string& json_text);
std::string write_manifest(const Manifest&);   // canonical JSON, sorted keys

struct Observed {
    std::string engine_file, engine_sha256;
    std::uint64_t engine_bytes = 0;
    std::string input_sha256, input_pixels_sha256;
    int input_rows = 0, input_cols = 0;
    std::string pipeline_digest, runtime_digest;
    std::string dual_runtime_sha256, dual_schema_sha256;   // empty = not consumed
};
// Empty string = pass; otherwise the first failing check, for the refusal log.
std::string check_identity(const Manifest&, const Observed&, std::string_view expected_partition);
std::string check_output(const Manifest&, const PartitionOutput&);

}  // namespace perception::service::psc4
