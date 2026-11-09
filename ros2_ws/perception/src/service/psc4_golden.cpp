/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: psc4_golden.cpp
 * Brief: Canonical output digest, strict manifest parse/write and the PSC-4 checks.
 *
 * Description:
 * What problem this solves. Implements psc4_golden.hpp (the canonical byte layout is specified
 * there). Everything here is platform-independent byte handling: fixed-width little-endian
 * fields so the same output gives the same digest on any host, and a manifest parser that
 * refuses anything it does not fully understand, because a half-read manifest could let a
 * changed engine pass.
 */
#include "service/psc4_golden.hpp"

#include "common/config.hpp"   // yolo::sha256Hex

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace perception::service::psc4 {
namespace {

// Little-endian regardless of the host byte order.
void put_u32(std::string& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xFFU));
}
void put_i32(std::string& out, int v) { put_u32(out, static_cast<std::uint32_t>(v)); }

// rows, cols, type, then the used bytes of each row (stride padding skipped).
void put_mat(std::string& out, const CanonMat& m) {
    put_i32(out, m.rows);
    put_i32(out, m.cols);
    put_i32(out, m.type);
    if (m.rows <= 0 || m.row_bytes == 0 || m.data == nullptr) return;
    for (int r = 0; r < m.rows; ++r)
        out.append(reinterpret_cast<const char*>(m.data + static_cast<std::size_t>(r) * m.stride), m.row_bytes);
}

// Count, then every detection of this partition in output order; the float
// confidence is hashed by its exact bits.
void put_detections(std::string& out, int source, const std::vector<CanonDetection>& dets, std::size_t& count) {
    count = 0;
    for (const auto& d : dets) count += d.source == source;
    put_u32(out, static_cast<std::uint32_t>(count));
    for (const auto& d : dets) {
        if (d.source != source) continue;
        put_i32(out, d.x); put_i32(out, d.y); put_i32(out, d.w); put_i32(out, d.h);
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(d.confidence), "float32 expected");
        std::memcpy(&bits, &d.confidence, sizeof(bits));
        put_u32(out, bits);
        put_i32(out, d.class_id); put_i32(out, d.original_class_id); put_i32(out, d.source);
        put_u32(out, static_cast<std::uint32_t>(d.class_name.size()));
        out.append(d.class_name.data(), d.class_name.size());
        put_mat(out, d.mask);
        put_mat(out, d.full_image_mask);
    }
}

// Exactly 64 lowercase hex digits: the form the sealing tool writes. Upper case or a short value
// is refused rather than normalised, so two spellings of one hash cannot both pass.
bool is_sha(const std::string& s) {
    return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

// Manifest field readers: a missing, null or mistyped field throws with its key.
const YAML::Node need(const YAML::Node& root, const char* key) {
    const auto node = root[key];
    if (!node || node.IsNull()) throw std::runtime_error(std::string("manifest: missing ") + key);
    return node;
}
std::string need_string(const YAML::Node& root, const char* key) {
    const auto n = need(root, key);
    if (!n.IsScalar()) throw std::runtime_error(std::string("manifest: ") + key + " is not a string");
    return n.as<std::string>();
}
std::uint64_t need_unsigned(const YAML::Node& root, const char* key) {
    const auto n = need(root, key);
    if (!n.IsScalar()) throw std::runtime_error(std::string("manifest: ") + key + " is not a number");
    // Digits only: yaml-cpp would otherwise accept forms like "+5", "0x10" or "1e3".
    const auto text = n.as<std::string>();
    if (text.empty() || !std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; }))
        throw std::runtime_error(std::string("manifest: ") + key + " is not an unsigned integer");
    return n.as<std::uint64_t>();
}
std::string need_sha(const YAML::Node& root, const char* key) {
    auto s = need_string(root, key);
    if (!is_sha(s)) throw std::runtime_error(std::string("manifest: ") + key + " is not a lowercase SHA-256");
    return s;
}
// JSON string literal for the manifest writer (quote, backslash and control characters escaped).
std::string json_string(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out + "\"";
}
}  // namespace

std::string_view partition_name(int source) noexcept {
    if (source == kSourceCoco80) return "coco80";
    if (source == kSourceActive12) return "active12";
    return {};
}

PartitionOutput digest_partition(int source, const std::vector<CanonDetection>& detections,
                                const std::vector<CanonDetection>& map_detections,
                                const CanonMat* retired_mask_slot) {
    // The rule name and partition prefix the bytes, so a digest of another rule
    // or partition can never match.
    std::string bytes;
    bytes.append(kRule.data(), kRule.size());
    put_i32(bytes, source);
    PartitionOutput out;
    put_detections(bytes, source, detections, out.detections);
    put_detections(bytes, source, map_detections, out.map_detections);
    // Active12 also carries the retired safe-mask slot: it must stay empty, and hashing it means
    // a revived slot would change the digest instead of passing unnoticed.
    if (source == kSourceActive12) {
        const CanonMat empty{};
        put_mat(bytes, retired_mask_slot ? *retired_mask_slot : empty);
    }
    out.sha256 = yolo::sha256_hex(bytes);
    return out;
}

Manifest parse_manifest(const std::string& json_text) {
    YAML::Node root;
    try { root = YAML::Load(json_text); }
    catch (const std::exception& e) { throw std::runtime_error(std::string("manifest: not parseable: ") + e.what()); }
    if (!root.IsMap()) throw std::runtime_error("manifest: not an object");
    // Every field is required and type-checked; an unknown version, rule or partition is refused
    // instead of being read with today's meaning.
    Manifest m;
    m.version = static_cast<int>(need_unsigned(root, "version"));
    if (m.version == 1)
        throw std::runtime_error("manifest: version 1 predates the consumed-input binding (reply batch 8, B8-1); re-seal required");
    if (m.version != kManifestVersion) throw std::runtime_error("manifest: unknown version");
    m.rule = need_string(root, "rule");
    if (m.rule != kRule) throw std::runtime_error("manifest: unknown comparison rule " + m.rule);
    m.partition = need_string(root, "partition");
    if (m.partition != "coco80" && m.partition != "active12")
        throw std::runtime_error("manifest: unknown partition " + m.partition);
    // Engine identity: file name, content hash and size (the size gives a fast, readable first
    // mismatch when a different engine is configured).
    m.engine_file = need_string(root, "engine_file");
    m.engine_sha256 = need_sha(root, "engine_sha256");
    m.engine_bytes = need_unsigned(root, "engine_bytes");
    // A plain file name only: the input must live inside the asset directory.
    m.input_file = need_string(root, "input_file");
    if (m.input_file.empty() || m.input_file.find('/') != std::string::npos || m.input_file[0] == '.')
        throw std::runtime_error("manifest: input_file must be a plain file name inside the asset directory");
    // Input identity twice: the file bytes, and the decoded pixels (a decoder change would alter
    // the network input while the file stays the same).
    m.input_sha256 = need_sha(root, "input_sha256");
    m.input_pixels_sha256 = need_sha(root, "input_pixels_sha256");
    m.input_rows = static_cast<int>(need_unsigned(root, "input_rows"));
    m.input_cols = static_cast<int>(need_unsigned(root, "input_cols"));
    // The inference configuration and the runtime switches the seal ran with: the same engine and
    // input under other thresholds or switches is not the sealed computation.
    m.pipeline_digest = need_sha(root, "pipeline_digest");
    m.runtime_digest = need_sha(root, "runtime_digest");
    m.expected_detections = need_unsigned(root, "expected_detections");
    m.expected_map_detections = need_unsigned(root, "expected_map_detections");
    m.expected_sha256 = need_sha(root, "expected_sha256");
    m.timeout_ms = static_cast<std::uint32_t>(need_unsigned(root, "timeout_ms"));
    m.sealed_repeats = static_cast<std::uint32_t>(need_unsigned(root, "sealed_repeats"));
    // v2: the dual-model JSON files the configuration actually read (both or
    // neither).
    const auto inputs = need(root, "consumed_inputs");
    if (!inputs.IsMap() || inputs.size() != 2)
        throw std::runtime_error("manifest: consumed_inputs must hold exactly dual_runtime_config and dual_class_schema");
    const auto optional_sha = [&](const char* key) -> std::string {
        const auto n = inputs[key];
        if (!n) throw std::runtime_error(std::string("manifest: consumed_inputs missing ") + key);
        if (n.IsNull()) return {};
        const auto v = n.IsScalar() ? n.as<std::string>() : std::string();
        if (!is_sha(v)) throw std::runtime_error(std::string("manifest: consumed_inputs.") + key + " is not null or a lowercase SHA-256");
        return v;
    };
    m.dual_runtime_sha256 = optional_sha("dual_runtime_config");
    m.dual_schema_sha256 = optional_sha("dual_class_schema");
    if (m.dual_runtime_sha256.empty() != m.dual_schema_sha256.empty())
        throw std::runtime_error("manifest: consumed_inputs must be both null or both set");
    const auto env = need(root, "build_env");
    if (!env.IsMap()) throw std::runtime_error("manifest: build_env is not an object");
    m.build_env.status = need_string(env, "status");
    m.build_env.detail = need_string(env, "detail");
    if (m.build_env.status != "RECORDED_AT_BUILD" && m.build_env.status != "RECOVERED_BY_ASSOCIATION" &&
        m.build_env.status != "UNKNOWN")
        throw std::runtime_error("manifest: unknown build_env.status " + m.build_env.status);
    if (m.engine_bytes == 0 || m.input_rows <= 0 || m.input_cols <= 0 || m.timeout_ms == 0)
        throw std::runtime_error("manifest: engine_bytes, input size and timeout must be positive");
    // The seal must have been repeatable at least twice.
    if (m.sealed_repeats < 2) throw std::runtime_error("manifest: sealed_repeats must be >= 2 (repeatability)");
    // An empty golden output has almost no discriminating power (reply B7-3).
    if (m.expected_detections + m.expected_map_detections == 0)
        throw std::runtime_error("manifest: the sealed output of this partition is empty");
    return m;
}

// Canonical JSON with sorted keys, so re-sealing the same facts gives the same file.
std::string write_manifest(const Manifest& m) {
    std::ostringstream o;
    o << "{\n"
      << "  \"build_env\": {\"detail\": " << json_string(m.build_env.detail)
      << ", \"status\": " << json_string(m.build_env.status) << "},\n"
      << "  \"consumed_inputs\": {\"dual_class_schema\": "
      << (m.dual_schema_sha256.empty() ? std::string("null") : json_string(m.dual_schema_sha256))
      << ", \"dual_runtime_config\": "
      << (m.dual_runtime_sha256.empty() ? std::string("null") : json_string(m.dual_runtime_sha256)) << "},\n"
      << "  \"engine_bytes\": " << m.engine_bytes << ",\n"
      << "  \"engine_file\": " << json_string(m.engine_file) << ",\n"
      << "  \"engine_sha256\": " << json_string(m.engine_sha256) << ",\n"
      << "  \"expected_detections\": " << m.expected_detections << ",\n"
      << "  \"expected_map_detections\": " << m.expected_map_detections << ",\n"
      << "  \"expected_sha256\": " << json_string(m.expected_sha256) << ",\n"
      << "  \"input_cols\": " << m.input_cols << ",\n"
      << "  \"input_file\": " << json_string(m.input_file) << ",\n"
      << "  \"input_pixels_sha256\": " << json_string(m.input_pixels_sha256) << ",\n"
      << "  \"input_rows\": " << m.input_rows << ",\n"
      << "  \"input_sha256\": " << json_string(m.input_sha256) << ",\n"
      << "  \"partition\": " << json_string(m.partition) << ",\n"
      << "  \"pipeline_digest\": " << json_string(m.pipeline_digest) << ",\n"
      << "  \"rule\": " << json_string(m.rule) << ",\n"
      << "  \"runtime_digest\": " << json_string(m.runtime_digest) << ",\n"
      << "  \"sealed_repeats\": " << m.sealed_repeats << ",\n"
      << "  \"timeout_ms\": " << m.timeout_ms << ",\n"
      << "  \"version\": " << m.version << "\n"
      << "}\n";
    return o.str();
}

// First mismatch wins, in the order an operator would investigate: which
// engine, its bytes, the input, then the configuration digests.
std::string check_identity(const Manifest& m, const Observed& o, std::string_view expected_partition) {
    if (m.partition != expected_partition) return "partition (manifest " + m.partition + ")";
    if (m.engine_file != o.engine_file) return "engine_file (manifest " + m.engine_file + ", configured " + o.engine_file + ")";
    if (m.engine_sha256 != o.engine_sha256) return "engine_sha256 (the engine is not the sealed one)";
    if (m.engine_bytes != o.engine_bytes) return "engine_bytes";
    if (m.input_sha256 != o.input_sha256) return "input_sha256 (input file changed or corrupted)";
    if (m.input_pixels_sha256 != o.input_pixels_sha256) return "input_pixels_sha256 (decoded input differs)";
    if (m.input_rows != o.input_rows || m.input_cols != o.input_cols) return "input size";
    if (m.pipeline_digest != o.pipeline_digest) return "pipeline_digest (inference configuration changed since sealing)";
    if (m.runtime_digest != o.runtime_digest) return "runtime_digest (runtime switches changed since sealing)";
    if (m.dual_runtime_sha256 != o.dual_runtime_sha256)
        return "consumed_inputs.dual_runtime_config (dual-model runtime JSON content differs from the sealed one)";
    if (m.dual_schema_sha256 != o.dual_schema_sha256)
        return "consumed_inputs.dual_class_schema (class schema JSON content differs from the sealed one)";
    return {};
}

// Counts first (more readable), then the full digest.
std::string check_output(const Manifest& m, const PartitionOutput& out) {
    if (out.detections != m.expected_detections || out.map_detections != m.expected_map_detections)
        return "counts (" + std::to_string(out.detections) + "/" + std::to_string(out.map_detections) +
               " vs sealed " + std::to_string(m.expected_detections) + "/" +
               std::to_string(m.expected_map_detections) + ")";
    if (out.sha256 != m.expected_sha256) return "digest (the output differs from the sealed value)";
    return {};
}

}  // namespace perception::service::psc4
