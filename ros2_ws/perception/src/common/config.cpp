/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: config.cpp
 * Brief: YAML configuration loader implementation.
 *
 * Description:
 *   1. Loads detector configuration from disk using yaml-cpp.
 *   2. Provides typed getters with defaults for missing keys; dual_model.enabled has no
 *      default (required, must be true).
 * Three kinds of reads, all in cache_values():
 *   - Gemini camera and hevc blocks (require_orbbec_*): every key required, complete tuples
 *     only; anything else throws and the load fails.
 *   - The sealed dual-model contract (read_dual_model_config): the two JSON files are read once,
 *     every policy value is compared with its sealed value and both files must match their
 *     SHA-256 (B8-1).
 *   - Everything else (read_value and friends): missing or null -> built-in default; a present
 *     value that does not convert refuses the load with the dotted key path (R1, 2026-10-04;
 *     before, yaml-cpp's as<T>(fallback) silently returned the default). inference.per_class_conf
 *     values are strict too; the other per-class maps skip an invalid entry with a WARN. PSC-2
 *     checks only the pipeline section names.
 */

#include "common/config.hpp"

#include "common/logger.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace yolo {

namespace {

constexpr const char* CONFIG_MODULE = "Config";
constexpr std::array<const char*, 80> DEFAULT_MODEL_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"};

// A missing key, or one written as null, keeps the built-in default.
bool absent(const YAML::Node& value) {
    return !value || value.IsNull();
}

// A present value must convert to T. yaml-cpp's as<T>(default) would silently return the default
// for a value such as "0.2m" or "six"; this refuses the load instead and names the key (R1,
// 2026-10-04: an optional key is optional, a mistyped one is not).
template <typename T>
T convert_or_refuse(const YAML::Node& value, const std::string& path) {
    try {
        return value.as<T>();
    } catch (const YAML::Exception&) {
        throw std::invalid_argument(
            path + ": " + (value.IsScalar() ? "value '" + value.Scalar() + "'" : std::string("a non-scalar value")) +
            " does not convert to the expected type");
    }
}

// section.key, or default_value when the key is missing or null; a present value must convert.
template <typename T>
T read_value(const YAML::Node& node, const char* section, const char* key, const T& default_value) {
    if (!node || !node[section] || absent(node[section][key])) {
        return default_value;
    }
    return convert_or_refuse<T>(node[section][key], std::string(section) + "." + key);
}

std::string read_value(
    const YAML::Node& node,
    const char* section,
    const char* key,
    const std::string& default_value) {
    if (!node || !node[section] || absent(node[section][key])) {
        return default_value;
    }
    return convert_or_refuse<std::string>(node[section][key], std::string(section) + "." + key);
}

// Replaces each ${NAME} with the environment value. An unset or empty name yields an empty
// result for the whole value (with a warning), never a partly expanded string.
std::string expand_env_vars(const std::string& value, const char* section, const char* key) {
    std::string expanded;
    std::size_t search_pos = 0U;

    while (search_pos < value.size()) {
        const std::size_t marker_pos = value.find("${", search_pos);
        if (marker_pos == std::string::npos) {
            expanded.append(value, search_pos, std::string::npos);
            break;
        }

        expanded.append(value, search_pos, marker_pos - search_pos);
        const std::size_t close_pos = value.find('}', marker_pos + 2U);
        if (close_pos == std::string::npos) {
            expanded.append(value, marker_pos, std::string::npos);
            break;
        }

        const std::string env_name = value.substr(marker_pos + 2U, close_pos - marker_pos - 2U);
        const char* env_value = std::getenv(env_name.c_str());
        if (env_name.empty() || env_value == nullptr) {
            LOG_WARN_M(
                CONFIG_MODULE,
                "environment variable " << env_name << " is not set for " << section << "." << key
                                        << "; using empty value");
            return "";
        }

        expanded += env_value;
        search_pos = close_pos + 1U;
    }

    return expanded;
}

std::string read_string_value(
    const YAML::Node& node,
    const char* section,
    const char* key,
    const std::string& default_value) {
    return expand_env_vars(read_value(node, section, key, default_value), section, key);
}

std::string read_camera_driver(const YAML::Node& node) {
    const YAML::Node camera = node["camera"];
    if (!camera.IsDefined()) {
        return "d435";
    }

    const YAML::Node driver = camera["driver"];
    if (!driver.IsDefined()) {
        return "d435";
    }
    if (!driver.IsScalar()) {
        throw std::invalid_argument("camera.driver must be a scalar");
    }

    return expand_env_vars(driver.as<std::string>(), "camera", "driver");
}

[[noreturn]] void invalid_orbbec_config(const std::string& message) {
    throw std::invalid_argument("invalid camera config for orbbec_gemini338le: " + message);
}

YAML::Node require_orbbec_child(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    if (!parent || !parent[key]) {
        invalid_orbbec_config("missing " + location + "." + key);
    }
    return parent[key];
}

YAML::Node require_orbbec_map(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    const YAML::Node child = require_orbbec_child(parent, key, location);
    if (!child.IsMap()) {
        invalid_orbbec_config(location + "." + key + " must be an object");
    }
    return child;
}

template <typename T>
T require_orbbec_scalar(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    const YAML::Node child = require_orbbec_child(parent, key, location);
    if (!child.IsScalar()) {
        invalid_orbbec_config(location + "." + key + " must be a scalar");
    }
    try {
        return child.as<T>();
    } catch (const YAML::Exception&) {
        invalid_orbbec_config(location + "." + key + " has an invalid scalar value");
    }
}

std::array<float, 3> read_array3(
    const YAML::Node& node,
    const char* section,
    const char* key,
    const std::array<float, 3>& default_value) {
    if (!node || !node[section] || absent(node[section][key])) {
        return default_value;
    }
    const YAML::Node value = node[section][key];
    const std::string path = std::string(section) + "." + key;
    if (!value.IsSequence() || value.size() != 3U) {
        throw std::invalid_argument(path + ": expected a sequence of 3 numbers");
    }
    return {
        convert_or_refuse<float>(value[0], path + "[0]"),
        convert_or_refuse<float>(value[1], path + "[1]"),
        convert_or_refuse<float>(value[2], path + "[2]")};
}

std::array<float, 4> read_array4(
    const YAML::Node& node,
    const char* section,
    const char* key,
    const std::array<float, 4>& default_value) {
    if (!node || !node[section] || absent(node[section][key])) {
        return default_value;
    }
    const YAML::Node value = node[section][key];
    const std::string path = std::string(section) + "." + key;
    if (!value.IsSequence() || value.size() != 4U) {
        throw std::invalid_argument(path + ": expected a sequence of 4 numbers");
    }
    return {
        convert_or_refuse<float>(value[0], path + "[0]"),
        convert_or_refuse<float>(value[1], path + "[1]"),
        convert_or_refuse<float>(value[2], path + "[2]"),
        convert_or_refuse<float>(value[3], path + "[3]")};
}

std::vector<std::string> read_string_vector(
    const YAML::Node& node,
    const char* section,
    const char* subsection,
    const char* key,
    const std::vector<std::string>& default_value) {
    if (!node || !node[section] || !node[section][subsection] || absent(node[section][subsection][key])) {
        return default_value;
    }
    const YAML::Node value = node[section][subsection][key];
    const std::string path = std::string(section) + "." + subsection + "." + key;
    if (!value.IsSequence()) {
        throw std::invalid_argument(path + ": expected a sequence of strings");
    }

    std::vector<std::string> output;
    output.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        output.push_back(convert_or_refuse<std::string>(value[i], path + "[" + std::to_string(i) + "]"));
    }
    return output;
}

std::vector<std::string> read_string_vector(
    const YAML::Node& node,
    const char* section,
    const char* key,
    const std::vector<std::string>& default_value) {
    if (!node || !node[section] || absent(node[section][key])) {
        return default_value;
    }
    const YAML::Node value = node[section][key];
    const std::string path = std::string(section) + "." + key;
    if (!value.IsSequence()) {
        throw std::invalid_argument(path + ": expected a sequence of strings");
    }

    std::vector<std::string> output;
    output.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        output.push_back(convert_or_refuse<std::string>(value[i], path + "[" + std::to_string(i) + "]"));
    }
    return output;
}

template <typename T>
T read_nested_value(
    const YAML::Node& node,
    const char* section,
    const char* subsection,
    const char* key,
    const T& default_value) {
    if (!node || !node[section] || !node[section][subsection] || absent(node[section][subsection][key])) {
        return default_value;
    }
    return convert_or_refuse<T>(
        node[section][subsection][key], std::string(section) + "." + subsection + "." + key);
}

std::vector<std::string> default_model_classes() {
    std::vector<std::string> classes;
    classes.reserve(DEFAULT_MODEL_CLASSES.size());
    for (const char* class_name : DEFAULT_MODEL_CLASSES) {
        classes.emplace_back(class_name);
    }
    return classes;
}

int class_id_from_name(const std::string& class_name) {
    for (std::size_t i = 0; i < DEFAULT_MODEL_CLASSES.size(); ++i) {
        if (class_name == DEFAULT_MODEL_CLASSES[i]) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::map<int, float> read_per_class_conf(const YAML::Node& node) {
    std::map<int, float> thresholds;
    if (!node || !node["inference"] || !node["inference"]["per_class_conf"] ||
        !node["inference"]["per_class_conf"].IsMap()) {
        return thresholds;
    }

    const YAML::Node values = node["inference"]["per_class_conf"];
    for (const auto& item : values) {
        const std::string key = item.first.as<std::string>("");
        int class_id = -1;
        try {
            std::size_t consumed = 0U;
            const int parsed = std::stoi(key, &consumed);
            if (consumed == key.size()) {
                class_id = parsed;
            }
        } catch (const std::exception&) {
            class_id = -1;
        }
        if (class_id < 0) {
            class_id = class_id_from_name(key);
        }
        if (class_id < 0 || class_id >= static_cast<int>(DEFAULT_MODEL_CLASSES.size())) {
            LOG_WARN_M(CONFIG_MODULE, "ignoring unknown inference.per_class_conf class key=" << key);
            continue;
        }
        // A mistyped threshold refuses the load (R1); it used to become 0.25 silently.
        thresholds[class_id] = convert_or_refuse<float>(item.second, "inference.per_class_conf." + key);
    }
    return thresholds;
}

bool is_depth_sample_mode(const std::string& mode) {
    return mode == "nearest_cluster" || mode == "percentile" || mode == "median";
}

std::map<std::string, std::string> read_per_class_depth_sample_modes(const YAML::Node& node) {
    std::map<std::string, std::string> modes;
    if (!node || !node["depth"] || !node["depth"]["per_class_sample_mode"] ||
        !node["depth"]["per_class_sample_mode"].IsMap()) {
        return modes;
    }

    for (const auto& item : node["depth"]["per_class_sample_mode"]) {
        const std::string class_name = item.first.as<std::string>("");
        const std::string mode = item.second.as<std::string>("");
        if (class_name.empty() || !is_depth_sample_mode(mode)) {
            LOG_WARN_M(
                CONFIG_MODULE,
                "ignoring invalid depth.per_class_sample_mode entry class="
                    << class_name << " mode=" << mode);
            continue;
        }
        modes[class_name] = mode;
    }
    return modes;
}

std::map<std::string, int> read_per_class_max_ages(const YAML::Node& node) {
    std::map<std::string, int> ages;
    if (!node || !node["tracking"] || !node["tracking"]["per_class_max_age"] ||
        !node["tracking"]["per_class_max_age"].IsMap()) {
        return ages;
    }

    for (const auto& item : node["tracking"]["per_class_max_age"]) {
        const std::string class_name = item.first.as<std::string>("");
        const int age = item.second.as<int>(-1);
        if (class_name.empty() || age < 0) {
            LOG_WARN_M(
                CONFIG_MODULE,
                "ignoring invalid tracking.per_class_max_age entry class="
                    << class_name << " age=" << age);
            continue;
        }
        ages[class_name] = age;
    }
    return ages;
}

std::map<std::string, float> read_per_class_bbox_ema_alphas(const YAML::Node& node) {
    std::map<std::string, float> alphas;
    if (!node || !node["tracking"] || !node["tracking"]["per_class_bbox_ema_alpha"] ||
        !node["tracking"]["per_class_bbox_ema_alpha"].IsMap()) {
        return alphas;
    }

    for (const auto& item : node["tracking"]["per_class_bbox_ema_alpha"]) {
        const std::string class_name = item.first.as<std::string>("");
        const float alpha = item.second.as<float>(-1.0F);
        if (class_name.empty() || alpha < 0.01F || alpha > 1.0F) {
            LOG_WARN_M(
                CONFIG_MODULE,
                "ignoring invalid tracking.per_class_bbox_ema_alpha entry class="
                    << class_name << " alpha=" << alpha);
            continue;
        }
        alphas[class_name] = alpha;
    }
    return alphas;
}

std::map<std::string, float> read_per_class_centroid_ema_alphas(const YAML::Node& node) {
    std::map<std::string, float> alphas;
    if (!node || !node["velocity"] || !node["velocity"]["per_class_centroid_ema_alpha"] ||
        !node["velocity"]["per_class_centroid_ema_alpha"].IsMap()) {
        return alphas;
    }

    for (const auto& item : node["velocity"]["per_class_centroid_ema_alpha"]) {
        const std::string class_name = item.first.as<std::string>("");
        const float alpha = item.second.as<float>(-1.0F);
        if (class_name.empty() || alpha < 0.01F || alpha > 1.0F) {
            LOG_WARN_M(
                CONFIG_MODULE,
                "ignoring invalid velocity.per_class_centroid_ema_alpha entry class="
                    << class_name << " alpha=" << alpha);
            continue;
        }
        alphas[class_name] = alpha;
    }
    return alphas;
}

std::map<std::string, float> read_per_class_static_displacements(const YAML::Node& node) {
    std::map<std::string, float> displacements;
    if (!node || !node["velocity"] ||
        !node["velocity"]["per_class_static_displacement_m"] ||
        !node["velocity"]["per_class_static_displacement_m"].IsMap()) {
        return displacements;
    }

    for (const auto& item : node["velocity"]["per_class_static_displacement_m"]) {
        const std::string class_name = item.first.as<std::string>("");
        const float displacement_m = item.second.as<float>(-1.0F);
        if (class_name.empty() || displacement_m < 0.0F) {
            LOG_WARN_M(
                CONFIG_MODULE,
                "ignoring invalid velocity.per_class_static_displacement_m entry class="
                    << class_name << " displacement_m=" << displacement_m);
            continue;
        }
        displacements[class_name] = displacement_m;
    }
    return displacements;
}

// SHA-256 of the sealed dual-model JSON files (dual_model_runtime_config.json, class_schema_94.json).
constexpr const char* SEALED_RUNTIME_SHA256 =
    "dd6f857d79492973b0c2910ed11430587922c58c9f00da900521f3ea9fdc0aca";
constexpr const char* SEALED_CLASS_SCHEMA_SHA256 =
    "c00cdc6ef6d72101e60a405a54cf404ed12af54b458dfaf29dd2ed15a1ba8505";

// Self-contained SHA-256 (FIPS 180-4), so the sealed-file check needs no crypto library.
class Sha256 {
public:
    void update(const std::uint8_t* data, std::size_t size) {
        bit_count_ += static_cast<std::uint64_t>(size) * 8U;
        while (size > 0U) {
            const std::size_t copy_size = std::min(size, buffer_.size() - buffer_size_);
            std::copy(data, data + copy_size, buffer_.begin() + buffer_size_);
            buffer_size_ += copy_size;
            data += copy_size;
            size -= copy_size;
            if (buffer_size_ == buffer_.size()) {
                transform(buffer_.data());
                buffer_size_ = 0U;
            }
        }
    }

    std::string final_hex() {
        buffer_[buffer_size_++] = 0x80U;
        if (buffer_size_ > 56U) {
            std::fill(buffer_.begin() + buffer_size_, buffer_.end(), 0U);
            transform(buffer_.data());
            buffer_size_ = 0U;
        }
        std::fill(buffer_.begin() + buffer_size_, buffer_.begin() + 56U, 0U);
        for (std::size_t index = 0U; index < 8U; ++index) {
            buffer_[63U - index] =
                static_cast<std::uint8_t>((bit_count_ >> (index * 8U)) & 0xffU);
        }
        transform(buffer_.data());

        std::ostringstream output;
        output << std::hex << std::setfill('0');
        for (const std::uint32_t word : state_) {
            output << std::setw(8) << word;
        }
        return output.str();
    }

private:
    static std::uint32_t rotate_right(std::uint32_t value, unsigned int count) {
        return (value >> count) | (value << (32U - count));
    }

    void transform(const std::uint8_t* block) {
        static constexpr std::array<std::uint32_t, 64> round_constants = {
            0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
            0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
            0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
            0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
            0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
            0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
            0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
            0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
            0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
            0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
            0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
            0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
            0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
            0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
            0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
            0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

        std::array<std::uint32_t, 64> schedule{};
        for (std::size_t index = 0U; index < 16U; ++index) {
            const std::size_t offset = index * 4U;
            schedule[index] =
                (static_cast<std::uint32_t>(block[offset]) << 24U) |
                (static_cast<std::uint32_t>(block[offset + 1U]) << 16U) |
                (static_cast<std::uint32_t>(block[offset + 2U]) << 8U) |
                static_cast<std::uint32_t>(block[offset + 3U]);
        }
        for (std::size_t index = 16U; index < schedule.size(); ++index) {
            const std::uint32_t sigma0 =
                rotate_right(schedule[index - 15U], 7U) ^
                rotate_right(schedule[index - 15U], 18U) ^
                (schedule[index - 15U] >> 3U);
            const std::uint32_t sigma1 =
                rotate_right(schedule[index - 2U], 17U) ^
                rotate_right(schedule[index - 2U], 19U) ^
                (schedule[index - 2U] >> 10U);
            schedule[index] =
                schedule[index - 16U] + sigma0 + schedule[index - 7U] + sigma1;
        }

        std::uint32_t a = state_[0];
        std::uint32_t b = state_[1];
        std::uint32_t c = state_[2];
        std::uint32_t d = state_[3];
        std::uint32_t e = state_[4];
        std::uint32_t f = state_[5];
        std::uint32_t g = state_[6];
        std::uint32_t h = state_[7];
        for (std::size_t index = 0U; index < schedule.size(); ++index) {
            const std::uint32_t sum1 =
                rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
            const std::uint32_t choose = (e & f) ^ ((~e) & g);
            const std::uint32_t temp1 =
                h + sum1 + choose + round_constants[index] + schedule[index];
            const std::uint32_t sum0 =
                rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
            const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t temp2 = sum0 + majority;
            h = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }
        state_[0] += a;
        state_[1] += b;
        state_[2] += c;
        state_[3] += d;
        state_[4] += e;
        state_[5] += f;
        state_[6] += g;
        state_[7] += h;
    }

    std::array<std::uint32_t, 8> state_ = {
        0x6a09e667U,
        0xbb67ae85U,
        0x3c6ef372U,
        0xa54ff53aU,
        0x510e527fU,
        0x9b05688cU,
        0x1f83d9abU,
        0x5be0cd19U};
    std::array<std::uint8_t, 64> buffer_{};
    std::uint64_t bit_count_ = 0U;
    std::size_t buffer_size_ = 0U;
};

// Reads a whole file ONCE. The dual-model JSON files are parsed from these
// bytes and hashed from these same bytes, so the content that is compared
// with the sealed digest is exactly the content that is used (reply batch 8,
// B8-1): no second open between the check and the use.
std::string read_file_bytes(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.good()) {
        throw std::runtime_error("cannot open file: " + path);
    }
    std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (file.bad()) {
        throw std::runtime_error("failed while reading file: " + path);
    }
    return bytes;
}

std::string bytes_sha256(const std::string& bytes) {
    Sha256 digest;
    digest.update(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
    return digest.final_hex();
}

[[noreturn]] void invalid_dual_config(const std::string& message) {
    throw std::runtime_error("invalid dual-model sealed config: " + message);
}

YAML::Node require_child(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    if (!parent || !parent[key]) {
        invalid_dual_config("missing " + location + "." + key);
    }
    return parent[key];
}

YAML::Node require_map_child(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    const YAML::Node child = require_child(parent, key, location);
    if (!child.IsMap()) {
        invalid_dual_config(location + "." + key + " must be an object");
    }
    return child;
}

template <typename T>
T require_scalar(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    const YAML::Node child = require_child(parent, key, location);
    if (!child.IsScalar()) {
        invalid_dual_config(location + "." + key + " must be a scalar");
    }
    return child.as<T>();
}

int parse_indexed_key(const YAML::Node& key_node, const std::string& location) {
    const std::string key = key_node.as<std::string>("");
    try {
        std::size_t consumed = 0U;
        const int parsed = std::stoi(key, &consumed);
        if (consumed == key.size() && parsed >= 0) {
            return parsed;
        }
    } catch (const std::exception&) {
        // Report one fail-closed error below.
    }
    invalid_dual_config(location + " contains non-index key=" + key);
}

std::vector<int> require_int_sequence(
    const YAML::Node& parent,
    const char* key,
    const std::string& location) {
    const YAML::Node value = require_child(parent, key, location);
    if (!value.IsSequence()) {
        invalid_dual_config(location + "." + key + " must be an array");
    }
    std::vector<int> output;
    output.reserve(value.size());
    for (const auto& item : value) {
        output.push_back(item.as<int>());
    }
    return output;
}

std::vector<int> require_indexed_int_map(
    const YAML::Node& parent,
    const char* key,
    std::size_t expected_size,
    const std::string& location) {
    const YAML::Node values = require_map_child(parent, key, location);
    if (values.size() != expected_size) {
        invalid_dual_config(
            location + "." + key + " must contain exactly " +
            std::to_string(expected_size) + " entries");
    }
    std::vector<int> output(expected_size, -1);
    std::vector<bool> seen(expected_size, false);
    for (const auto& item : values) {
        const int index = parse_indexed_key(item.first, location + "." + key);
        if (index < 0 || static_cast<std::size_t>(index) >= expected_size ||
            seen[static_cast<std::size_t>(index)]) {
            invalid_dual_config(location + "." + key + " has invalid or duplicate index");
        }
        output[static_cast<std::size_t>(index)] = item.second.as<int>();
        seen[static_cast<std::size_t>(index)] = true;
    }
    return output;
}

std::vector<std::string> require_indexed_string_map(
    const YAML::Node& parent,
    const char* key,
    std::size_t expected_size,
    const std::string& location) {
    const YAML::Node values = require_map_child(parent, key, location);
    if (values.size() != expected_size) {
        invalid_dual_config(
            location + "." + key + " must contain exactly " +
            std::to_string(expected_size) + " entries");
    }
    std::vector<std::string> output(expected_size);
    std::vector<bool> seen(expected_size, false);
    for (const auto& item : values) {
        const int index = parse_indexed_key(item.first, location + "." + key);
        if (index < 0 || static_cast<std::size_t>(index) >= expected_size ||
            seen[static_cast<std::size_t>(index)]) {
            invalid_dual_config(location + "." + key + " has invalid or duplicate index");
        }
        output[static_cast<std::size_t>(index)] = item.second.as<std::string>();
        if (output[static_cast<std::size_t>(index)].empty()) {
            invalid_dual_config(location + "." + key + " contains an empty class name");
        }
        seen[static_cast<std::size_t>(index)] = true;
    }
    return output;
}

bool nearly_equal(float lhs, float rhs) {
    return std::fabs(lhs - rhs) <= 1.0e-6F;
}

void require_exact_float(
    float actual,
    float expected,
    const std::string& location) {
    if (!nearly_equal(actual, expected)) {
        invalid_dual_config(
            location + " changed from sealed value " + std::to_string(expected));
    }
}

void require_unique_ids(
    const std::vector<int>& ids,
    int minimum,
    int maximum,
    const std::string& location) {
    std::set<int> unique;
    for (const int id : ids) {
        if (id < minimum || id > maximum || !unique.insert(id).second) {
            invalid_dual_config(location + " contains an out-of-range or duplicate id");
        }
    }
}

std::string resolve_config_relative_path(
    const std::string& config_path,
    const std::string& value) {
    if (value.empty()) {
        return "";
    }
    std::filesystem::path candidate(value);
    if (candidate.is_absolute()) {
        return candidate.lexically_normal().string();
    }

    std::filesystem::path source_path(config_path);
    if (source_path.is_relative()) {
        source_path = std::filesystem::absolute(source_path);
    }
    return (source_path.parent_path() / candidate).lexically_normal().string();
}

bool is_lower_sha256_hex(const std::string& value) {
    if (value.size() != 64U) return false;
    for (const char c : value) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

// INT8 build inputs of one model block (inference or dual_model); see Config::Int8BuildConfig.
// Only the presence and form of the keys are checked here; the engine build reads the files and
// checks their SHA-256 (modules/int8_plan.cpp). Paths are taken as given in both blocks (the
// formal service requires them absolute, PSC-2). INT8 is accepted only on the DLA, the one path
// verified on Orin (2026-10-09).
Config::Int8BuildConfig read_int8_build(
    const YAML::Node& root,
    const char* section,
    bool enabled,
    bool use_dla) {
    Config::Int8BuildConfig out;
    out.enabled = enabled;
    out.ranges_path = read_string_value(root, section, "int8_ranges", "");
    out.ranges_sha256 = read_string_value(root, section, "int8_ranges_sha256", "");
    out.fp16_layers_path = read_string_value(root, section, "int8_fp16_layers", "");
    out.fp16_layers_sha256 = read_string_value(root, section, "int8_fp16_layers_sha256", "");
    const std::string where = std::string(section) + ".";
    if (!enabled) {
        if (!out.ranges_path.empty() || !out.ranges_sha256.empty() || !out.fp16_layers_path.empty() ||
            !out.fp16_layers_sha256.empty()) {
            throw std::runtime_error(where + "int8 is false but an int8_* key is set");
        }
        return out;
    }
    if (!use_dla) {
        throw std::runtime_error(where + "int8 is supported only with the DLA (use_dla true)");
    }
    if (out.ranges_path.empty() || !is_lower_sha256_hex(out.ranges_sha256)) {
        throw std::runtime_error(
            where + "int8 needs int8_ranges and int8_ranges_sha256 (64 lowercase hex digits)");
    }
    if (out.fp16_layers_path.empty() != out.fp16_layers_sha256.empty() ||
        (!out.fp16_layers_sha256.empty() && !is_lower_sha256_hex(out.fp16_layers_sha256))) {
        throw std::runtime_error(
            where + "int8_fp16_layers and int8_fp16_layers_sha256 go together (64 lowercase hex digits)");
    }
    return out;
}

Config::DualModelConfig read_dual_model_config(
    const YAML::Node& root,
    const std::string& config_path) {
    Config::DualModelConfig output;
    // The R6 pipeline runs COCO80 and Active12 together, so dual_model.enabled
    // is a required YAML bool and must be true; there is no default. A missing,
    // null, non-bool or false value is refused here, before any path is
    // resolved, any environment variable is expanded or any sealed file is read.
    const YAML::Node section = root["dual_model"];
    bool enabled = false;
    if (!section || !section.IsMap() || !section["enabled"] || !section["enabled"].IsScalar() ||
        !YAML::convert<bool>::decode(section["enabled"], enabled)) {
        invalid_dual_config("dual_model.enabled is required and must be a YAML bool");
    }
    if (!enabled) {
        invalid_dual_config("dual_model.enabled must be true (COCO80 and Active12 run together)");
    }
    output.enabled = true;
    output.runtime_config_path = resolve_config_relative_path(
        config_path,
        read_string_value(
            root,
            "dual_model",
            "runtime_config_path",
            "dual_model_runtime_config.json"));
    output.class_schema_path = resolve_config_relative_path(
        config_path,
        read_string_value(root, "dual_model", "class_schema_path", "class_schema_94.json"));
    output.active_model_path = resolve_config_relative_path(
        config_path,
        read_string_value(
            root,
            "dual_model",
            "active_model_path",
            "../models/active12_yolo11n_seg_896_best.onnx"));
    output.active_engine_path = resolve_config_relative_path(
        config_path,
        read_string_value(
            root,
            "dual_model",
            "active_engine_path",
            "../models/active12_yolo11n_seg_896_best-dla1-fp16.engine"));
    output.use_dla = read_value<bool>(root, "dual_model", "use_dla", true);
    output.dla_core = read_value<int>(root, "dual_model", "dla_core", 1);
    output.use_fp16 = read_value<bool>(root, "dual_model", "use_fp16", true);
    output.allow_gpu_fallback =
        read_value<bool>(root, "dual_model", "allow_gpu_fallback", true);
    output.workspace_mb = read_value<int>(root, "dual_model", "workspace_mb", 2048);
    output.int8 = read_int8_build(
        root, "dual_model", read_value<bool>(root, "dual_model", "int8", false), output.use_dla);

    if (output.runtime_config_path.empty() || output.class_schema_path.empty() ||
        output.active_model_path.empty() || output.active_engine_path.empty()) {
        invalid_dual_config("enabled deployment paths must not be empty");
    }
    if (output.dla_core < 0 || output.dla_core > 1) {
        invalid_dual_config("dual_model.dla_core must be 0 or 1");
    }
    if (output.workspace_mb <= 0) {
        invalid_dual_config("dual_model.workspace_mb must be positive");
    }
    // dual_model.input_shape (optional) names the deployed input: the sealed square, or the
    // sealed 896 x 608 window of the square letterbox (rows 160..767; postprocess.cpp). A
    // missing or null key keeps the square; any other shape or form is refused.
    int deployed_input_height = 0;
    if (const YAML::Node deployed = section["input_shape"]; deployed.IsDefined() && !deployed.IsNull()) {
        std::vector<int> shape;
        if (deployed.IsSequence()) {
            for (const auto& item : deployed) {
                int value = 0;
                if (!YAML::convert<int>::decode(item, value)) {  // also a null, a list or a map
                    shape.clear();
                    break;
                }
                shape.push_back(value);
            }
        }
        if (shape != std::vector<int>({1, 3, 896, 896}) && shape != std::vector<int>({1, 3, 608, 896})) {
            invalid_dual_config("dual_model.input_shape must be [1,3,896,896] or [1,3,608,896]");
        }
        deployed_input_height = shape[2];
    }

    const std::string runtime_bytes = read_file_bytes(output.runtime_config_path);
    const std::string schema_bytes = read_file_bytes(output.class_schema_path);
    const YAML::Node runtime = YAML::Load(runtime_bytes);
    const YAML::Node schema = YAML::Load(schema_bytes);
    if (!runtime.IsMap() || !schema.IsMap()) {
        invalid_dual_config("runtime config and class schema must be JSON objects");
    }

    const YAML::Node branches = require_map_child(runtime, "branches", "runtime");
    const YAML::Node active12 = require_map_child(branches, "active12", "runtime.branches");
    const std::vector<int> input_shape =
        require_int_sequence(active12, "input_shape", "runtime.branches.active12");
    if (input_shape != std::vector<int>({1, 3, 896, 896})) {
        invalid_dual_config("runtime.branches.active12.input_shape must be [1,3,896,896]");
    }
    output.input_size = input_shape[2];
    output.input_height = deployed_input_height != 0 ? deployed_input_height : output.input_size;
    output.conf_threshold =
        require_scalar<float>(active12, "confidence_threshold", "runtime.branches.active12");
    output.nms_iou_threshold =
        require_scalar<float>(active12, "nms_iou_threshold", "runtime.branches.active12");
    output.mask_threshold =
        require_scalar<float>(active12, "mask_probability_threshold", "runtime.branches.active12");
    const float mask_logit_threshold =
        require_scalar<float>(active12, "mask_logit_threshold", "runtime.branches.active12");
    const bool class_aware_nms =
        require_scalar<bool>(active12, "class_aware_nms", "runtime.branches.active12");
    output.max_detections =
        require_scalar<int>(active12, "max_detections", "runtime.branches.active12");
    require_exact_float(
        output.conf_threshold, 0.25F, "runtime.branches.active12.confidence_threshold");
    require_exact_float(
        output.nms_iou_threshold, 0.70F, "runtime.branches.active12.nms_iou_threshold");
    require_exact_float(
        output.mask_threshold, 0.50F, "runtime.branches.active12.mask_probability_threshold");
    require_exact_float(
        mask_logit_threshold, 0.0F, "runtime.branches.active12.mask_logit_threshold");
    if (!class_aware_nms || output.max_detections != 300) {
        invalid_dual_config("active12 NMS mode or max_detections changed from sealed values");
    }

    output.local_class_names =
        require_indexed_string_map(runtime, "active12_local_classes", 12U, "runtime");
    output.local_to_global =
        require_indexed_int_map(runtime, "active12_to_global_id", 12U, "runtime");
    const std::vector<int> expected_mapping =
        {80, 81, 82, 83, 84, 85, 86, 89, 90, 91, 92, 93};
    if (output.local_to_global != expected_mapping) {
        invalid_dual_config("active12 mapping changed or references reserved ids 87/88");
    }
    require_unique_ids(output.local_to_global, 80, 93, "runtime.active12_to_global_id");

    output.reserved_ids =
        require_int_sequence(runtime, "reserved_inactive_global_ids", "runtime");
    if (output.reserved_ids != std::vector<int>({87, 88})) {
        invalid_dual_config("reserved inactive global ids must be exactly [87,88]");
    }

    const YAML::Node merge =
        require_map_child(runtime, "cross_model_merge", "runtime");
    if (require_scalar<bool>(merge, "generic_cross_model_nms", "runtime.cross_model_merge")) {
        invalid_dual_config("generic cross-model NMS must remain disabled");
    }
    const YAML::Node conflict = require_map_child(
        merge, "vehicle_heavy_conflict", "runtime.cross_model_merge");
    output.coco_vehicle_ids = require_int_sequence(
        conflict, "coco_vehicle_ids", "runtime.cross_model_merge.vehicle_heavy_conflict");
    output.heavy_id = require_scalar<int>(
        conflict, "active_global_id", "runtime.cross_model_merge.vehicle_heavy_conflict");
    output.conflict_iou = require_scalar<float>(
        conflict, "box_iou_threshold", "runtime.cross_model_merge.vehicle_heavy_conflict");
    output.conflict_mode = require_scalar<std::string>(
        conflict, "mode", "runtime.cross_model_merge.vehicle_heavy_conflict");
    const bool automatic_suppression = require_scalar<bool>(
        conflict,
        "automatic_suppression_enabled",
        "runtime.cross_model_merge.vehicle_heavy_conflict");
    if (output.coco_vehicle_ids != std::vector<int>({2, 5, 7}) ||
        output.heavy_id != 89 || output.conflict_mode != "audit_only" ||
        automatic_suppression) {
        invalid_dual_config("vehicle/heavy conflict policy must remain audit_only");
    }
    require_exact_float(
        output.conflict_iou,
        0.30F,
        "runtime.cross_model_merge.vehicle_heavy_conflict.box_iou_threshold");

    const YAML::Node tracking = require_map_child(runtime, "tracking", "runtime");
    output.tracker_excluded_ids =
        require_int_sequence(tracking, "excluded_global_ids", "runtime.tracking");
    if (output.tracker_excluded_ids != std::vector<int>({92})) {
        invalid_dual_config("tracker exclusion must contain only traversable class 92");
    }

    // The sealed safe_traversable section has no runtime reader since the safe
    // traversable mask builder left the pipeline (level-3 trim). It is still
    // required and validated exactly as before: the sealed JSON must not change
    // by one byte, the 92/93 identity is what plan 2f (T channel, raw class 92)
    // relies on, and 2f ground erosion may reuse erosion_pixels.
    const YAML::Node safe = require_map_child(runtime, "safe_traversable", "runtime");
    output.safe_traversable.traversable_global_id =
        require_scalar<int>(safe, "traversable_global_id", "runtime.safe_traversable");
    output.safe_traversable.pit_global_id =
        require_scalar<int>(safe, "pit_global_id", "runtime.safe_traversable");
    output.safe_traversable.active_obstacle_global_ids = require_int_sequence(
        safe, "active_obstacle_global_ids", "runtime.safe_traversable");
    output.safe_traversable.coco_navigation_obstacle_ids = require_int_sequence(
        safe, "coco_navigation_obstacle_ids", "runtime.safe_traversable");
    output.safe_traversable.erosion_pixels =
        require_scalar<int>(safe, "erosion_pixels", "runtime.safe_traversable");
    output.safe_traversable.erosion_kernel =
        require_scalar<std::string>(safe, "erosion_kernel", "runtime.safe_traversable");
    const std::vector<int> expected_active_obstacles =
        {80, 81, 82, 83, 84, 85, 86, 89, 90, 91, 93};
    if (output.safe_traversable.traversable_global_id != 92 ||
        output.safe_traversable.pit_global_id != 93 ||
        output.safe_traversable.active_obstacle_global_ids != expected_active_obstacles ||
        output.safe_traversable.erosion_pixels != 5 ||
        output.safe_traversable.erosion_kernel.empty()) {
        invalid_dual_config("safe-traversable contract changed from sealed values");
    }
    require_unique_ids(
        output.safe_traversable.coco_navigation_obstacle_ids,
        0,
        79,
        "runtime.safe_traversable.coco_navigation_obstacle_ids");

    if (require_scalar<int>(schema, "nc", "schema") != 94) {
        invalid_dual_config("class schema nc must be 94");
    }
    const std::vector<std::string> global_names =
        require_indexed_string_map(schema, "names", 94U, "schema");
    output.global_class_names = global_names;
    const std::vector<int> old_output_ids =
        require_int_sequence(schema, "old_output_ids", "schema");
    std::vector<int> expected_old_ids(80U);
    for (std::size_t index = 0U; index < expected_old_ids.size(); ++index) {
        expected_old_ids[index] = static_cast<int>(index);
    }
    if (old_output_ids != expected_old_ids) {
        invalid_dual_config("schema.old_output_ids must remain 0..79");
    }
    const std::vector<int> schema_mapping =
        require_indexed_int_map(schema, "active_output_mapping", 12U, "schema");
    const std::vector<int> schema_reserved =
        require_int_sequence(schema, "inactive_reserved_ids", "schema");
    if (schema_mapping != output.local_to_global ||
        schema_reserved != output.reserved_ids) {
        invalid_dual_config("runtime/schema mapping or reserved-id mismatch");
    }
    for (std::size_t local_id = 0U; local_id < output.local_class_names.size(); ++local_id) {
        const int global_id = output.local_to_global[local_id];
        if (global_names[static_cast<std::size_t>(global_id)] !=
            output.local_class_names[local_id]) {
            invalid_dual_config("runtime/schema class-name mapping mismatch");
        }
    }

    output.runtime_config_sha256 = bytes_sha256(runtime_bytes);
    output.class_schema_sha256 = bytes_sha256(schema_bytes);
    if (output.runtime_config_sha256 != SEALED_RUNTIME_SHA256) {
        invalid_dual_config(
            "runtime JSON SHA-256 mismatch: expected " +
            std::string(SEALED_RUNTIME_SHA256) + ", got " +
            output.runtime_config_sha256);
    }
    if (output.class_schema_sha256 != SEALED_CLASS_SCHEMA_SHA256) {
        invalid_dual_config(
            "class schema SHA-256 mismatch: expected " +
            std::string(SEALED_CLASS_SCHEMA_SHA256) + ", got " +
            output.class_schema_sha256);
    }

    return output;
}

}  // namespace

std::string sha256_hex(const std::string& bytes) {
    Sha256 digest;
    digest.update(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
    return digest.final_hex();
}

bool Config::load(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& exception) {
        return load_failed(path, exception.what());
    }
    return load_from_node(root, path);
}

bool Config::load_from_node(const YAML::Node& root, const std::string& origin_path) {
    try {
        path_ = origin_path;
        node_ = YAML::Clone(root);
        cache_values();
        return true;
    } catch (const std::exception& exception) {
        return load_failed(origin_path, exception.what());
    }
}

bool Config::load_failed(const std::string& path, const char* what) {
    {
        path_ = path;
        LOG_ERROR_M(CONFIG_MODULE, "failed to load " << path << ": " << what);
        // A failed reload must never leave a previously enabled supplemental
        // branch armed with stale paths or stale sealed policy.
        cached_.dual_model = DualModelConfig{};
        // Likewise, a rejected camera profile must not leave a previously valid
        // driver selection or a partially parsed Gemini profile usable.
        cached_.camera_driver.clear();
        cached_.camera_serial.clear();
        cached_.camera_color_device_format.clear();
        cached_.camera_depth_device_format.clear();
        cached_.camera_alignment_mode.clear();
        cached_.camera_color_width = 0;
        cached_.camera_color_height = 0;
        cached_.camera_color_fps = 0;
        cached_.camera_depth_width = 0;
        cached_.camera_depth_height = 0;
        cached_.camera_depth_fps = 0;
        cached_.camera_align_to_color = false;
        cached_.camera_output_width = 0;
        cached_.camera_output_height = 0;
        cached_.camera_output_fps = 0;
        node_ = YAML::Node();
        return false;
    }
}

std::string Config::path() const { return path_; }

// Reads every value once into cached_; throws (the load then fails) on any strict-part error.
// Order matters where a read depends on an earlier one: camera_driver selects the strict Gemini
// blocks (which then override the generic hevc reads), and generic_static_displacement_m
// defaults to static_displacement_m.
void Config::cache_values() {
    cached_.rtsp_url = read_string_value(node_, "rtsp", "url", "");  // no R6 reader
    cached_.camera_driver = read_camera_driver(node_);
    // A Config instance may be reloaded. Clear driver-specific data before
    // parsing so a later D435 config cannot expose stale Gemini values.
    cached_.camera_serial.clear();
    cached_.camera_color_device_format.clear();
    cached_.camera_depth_device_format.clear();
    cached_.camera_alignment_mode.clear();
    cached_.camera_color_width = 0;
    cached_.camera_color_height = 0;
    cached_.camera_color_fps = 0;
    cached_.camera_depth_width = 0;
    cached_.camera_depth_height = 0;
    cached_.camera_depth_fps = 0;
    cached_.camera_align_to_color = true;
    cached_.camera_output_width = 0;
    cached_.camera_output_height = 0;
    cached_.camera_output_fps = 0;
    if (cached_.camera_driver == "orbbec_gemini338le") {
        const YAML::Node camera = node_["camera"];
        if (!camera.IsMap()) {
            invalid_orbbec_config("camera must be an object");
        }

        cached_.camera_serial = expand_env_vars(
            require_orbbec_scalar<std::string>(camera, "serial", "camera"),
            "camera",
            "serial");
        if (cached_.camera_serial.find_first_not_of(" \t\r\n") == std::string::npos) {
            invalid_orbbec_config("camera.serial must be non-empty");
        }

        const YAML::Node color = require_orbbec_map(camera, "color", "camera");
        cached_.camera_color_width =
            require_orbbec_scalar<int>(color, "width", "camera.color");
        cached_.camera_color_height =
            require_orbbec_scalar<int>(color, "height", "camera.color");
        cached_.camera_color_fps =
            require_orbbec_scalar<int>(color, "fps", "camera.color");
        cached_.camera_color_device_format = require_orbbec_scalar<std::string>(
            color, "device_format", "camera.color");

        const YAML::Node depth = require_orbbec_map(camera, "depth", "camera");
        cached_.camera_depth_width =
            require_orbbec_scalar<int>(depth, "width", "camera.depth");
        cached_.camera_depth_height =
            require_orbbec_scalar<int>(depth, "height", "camera.depth");
        cached_.camera_depth_fps =
            require_orbbec_scalar<int>(depth, "fps", "camera.depth");
        cached_.camera_depth_device_format = require_orbbec_scalar<std::string>(
            depth, "device_format", "camera.depth");
        cached_.camera_alignment_mode = "hw_d2c";
        const YAML::Node alignment_mode = camera["alignment_mode"];
        if (alignment_mode.IsDefined()) {
            cached_.camera_alignment_mode =
                require_orbbec_scalar<std::string>(camera, "alignment_mode", "camera");
        }
        cached_.camera_align_to_color =
            require_orbbec_scalar<bool>(camera, "align_to_color", "camera");

        const YAML::Node output = require_orbbec_map(camera, "output", "camera");
        cached_.camera_output_width =
            require_orbbec_scalar<int>(output, "width", "camera.output");
        cached_.camera_output_height =
            require_orbbec_scalar<int>(output, "height", "camera.output");
        cached_.camera_output_fps =
            require_orbbec_scalar<int>(output, "fps", "camera.output");

        if (cached_.camera_color_fps != 30 || cached_.camera_color_device_format != "mjpg") {
            invalid_orbbec_config("camera.color must use 30 fps and device_format=mjpg");
        }
        const bool color_profile_1280x720 =
            cached_.camera_color_width == 1280 && cached_.camera_color_height == 720;
        const bool color_profile_1280x800 =
            cached_.camera_color_width == 1280 && cached_.camera_color_height == 800;
        const bool depth_profile_848x530 =
            cached_.camera_depth_width == 848 && cached_.camera_depth_height == 530;
        const bool depth_profile_640x400 =
            cached_.camera_depth_width == 640 && cached_.camera_depth_height == 400;
        const bool depth_profile_1280x800 =
            cached_.camera_depth_width == 1280 && cached_.camera_depth_height == 800;
        const bool hw_d2c = cached_.camera_alignment_mode == "hw_d2c";
        const bool sw_d2c = cached_.camera_alignment_mode == "sw_d2c";
        if (!hw_d2c && !sw_d2c) {
            invalid_orbbec_config(
                "camera.alignment_mode must be hw_d2c or sw_d2c");
        }
        if (cached_.camera_depth_fps != 30 ||
            cached_.camera_depth_device_format != "y16") {
            invalid_orbbec_config("camera.depth must use 30 fps and device_format=y16");
        }
        if (!cached_.camera_align_to_color) {
            invalid_orbbec_config("camera.align_to_color must be true");
        }
        if (cached_.camera_output_fps != 30) {
            invalid_orbbec_config("camera.output must use 30 fps");
        }
        const bool output_profile_1920x1080 =
            cached_.camera_output_width == 1920 && cached_.camera_output_height == 1080;
        const bool output_profile_1280x800 =
            cached_.camera_output_width == 1280 && cached_.camera_output_height == 800;
        // Validate complete tuples: allowing each dimension independently would
        // also enable unverified cross-combinations of the legacy and native routes.
        const bool legacy_tuple = color_profile_1280x720 && output_profile_1920x1080 &&
            ((hw_d2c && (depth_profile_848x530 || depth_profile_640x400)) ||
             (sw_d2c && depth_profile_1280x800));
        const bool native_800_tuple = color_profile_1280x800 && depth_profile_640x400 &&
            sw_d2c && output_profile_1280x800;
        if (!legacy_tuple && !native_800_tuple) {
            invalid_orbbec_config(
                "unsupported color/depth/alignment/output tuple; expected "
                "1280x720/(848x530 or 640x400)/hw_d2c/1920x1080, "
                "1280x720/1280x800/sw_d2c/1920x1080, or "
                "1280x800/640x400/sw_d2c/1280x800");
        }
    }
    // V5 D435 keys: read for compatibility; nothing in R6 uses them (the R6 service refuses a d435
    // section in perception.pipeline).
    cached_.d435_enable = read_value<bool>(node_, "d435", "enable", true);
    cached_.d435_color_width = read_value<int>(node_, "d435", "color_width", 1920);
    cached_.d435_color_height = read_value<int>(node_, "d435", "color_height", 1080);
    cached_.d435_color_fps = read_value<int>(node_, "d435", "color_fps", 30);
    cached_.d435_depth_width = read_value<int>(node_, "d435", "depth_width", 1280);
    cached_.d435_depth_height = read_value<int>(node_, "d435", "depth_height", 720);
    cached_.d435_depth_fps = read_value<int>(node_, "d435", "depth_fps", 30);
    cached_.d435_align_to_color = read_value<bool>(node_, "d435", "align_to_color", true);
    cached_.d435_power_line_frequency =
        read_value<int>(node_, "d435", "power_line_frequency", -1);
    // hevc: in R6 always the block service_config.cpp synthesizes (sink med2_loopback).
    cached_.hevc_enable = read_value<bool>(node_, "hevc", "enable", true);
    cached_.hevc_sink = read_string_value(node_, "hevc", "sink", "file_rtsp");
    cached_.hevc_rtsp_listen =
        read_string_value(node_, "hevc", "rtsp_listen", "rtsp://0.0.0.0:8555/rgb");
    cached_.hevc_file_path = read_string_value(node_, "hevc", "file_path", "data/p4_rgb.mp4");
    cached_.hevc_width = read_value<int>(node_, "hevc", "width", 1920);
    cached_.hevc_height = read_value<int>(node_, "hevc", "height", 1080);
    cached_.hevc_fps = read_value<int>(node_, "hevc", "fps", 30);
    cached_.hevc_bitrate = read_value<int>(node_, "hevc", "bitrate", 4000000);
    cached_.hevc_codec = read_string_value(node_, "hevc", "codec", "nvv4l2h265enc");
    cached_.hevc_gop = read_value<int>(node_, "hevc", "gop", 30);
    if (cached_.camera_driver == "orbbec_gemini338le") {
        const YAML::Node hevc = node_["hevc"];
        if (!hevc.IsMap()) {
            invalid_orbbec_config("hevc must be an object");
        }
        // Generic optional config keeps its historical defaults, but a Gemini
        // hard-contract candidate must not turn malformed YAML into 1080p30 by
        // silently falling back to those same defaults.
        cached_.hevc_width = require_orbbec_scalar<int>(hevc, "width", "hevc");
        cached_.hevc_height = require_orbbec_scalar<int>(hevc, "height", "hevc");
        cached_.hevc_fps = require_orbbec_scalar<int>(hevc, "fps", "hevc");
        cached_.hevc_enable = require_orbbec_scalar<bool>(hevc, "enable", "hevc");
        cached_.hevc_sink = require_orbbec_scalar<std::string>(hevc, "sink", "hevc");
        cached_.hevc_rtsp_listen =
            require_orbbec_scalar<std::string>(hevc, "rtsp_listen", "hevc");
        cached_.hevc_bitrate = require_orbbec_scalar<int>(hevc, "bitrate", "hevc");
        cached_.hevc_codec = require_orbbec_scalar<std::string>(hevc, "codec", "hevc");
        cached_.hevc_gop = require_orbbec_scalar<int>(hevc, "gop", "hevc");
        if (cached_.hevc_sink == "med2_loopback") {
            // Formal R6 service preview branch (19 section 13.1 MED-2): an
            // independent preview of the camera output, bound to loopback only;
            // port/size/fps/bitrate/GOP come from perception.med2. Implemented
            // adaptation (reply batch 8, B8-6): centre crop to the same width and
            // a smaller-or-equal height (even difference), and a frame rate that
            // divides the camera rate (capture-time selection). No scaling.
            if (cached_.hevc_width != cached_.camera_output_width ||
                cached_.hevc_height <= 0 || cached_.hevc_height > cached_.camera_output_height ||
                (cached_.camera_output_height - cached_.hevc_height) % 2 != 0 ||
                cached_.hevc_fps <= 0 || cached_.hevc_fps > cached_.camera_output_fps ||
                cached_.camera_output_fps % cached_.hevc_fps != 0) {
                invalid_orbbec_config("hevc preview must be a centre crop of camera.output (same width, even height "
                                    "difference) at a rate dividing the camera rate");
            }
            const std::string loopback = "rtsp://127.0.0.1:";
            bool listen_ok = cached_.hevc_rtsp_listen.rfind(loopback, 0) == 0;
            if (listen_ok) {
                const auto rest = cached_.hevc_rtsp_listen.substr(loopback.size());
                const auto slash = rest.find('/');
                const auto port = rest.substr(0, slash);
                listen_ok = slash != std::string::npos && slash + 1 < rest.size() && !port.empty() &&
                    port.size() <= 5 && port[0] != '0' &&
                    std::all_of(port.begin(), port.end(), [](unsigned char c) { return c >= '0' && c <= '9'; }) &&
                    std::stol(port) <= 65535;
            }
            if (!cached_.hevc_enable || !listen_ok || cached_.hevc_bitrate <= 0 ||
                (cached_.hevc_codec != "nvv4l2h264enc" && cached_.hevc_codec != "nvv4l2h265enc") ||
                cached_.hevc_gop <= 0) {
                invalid_orbbec_config(
                    "hevc med2_loopback must stream at rtsp://127.0.0.1:<port>/<path> using "
                    "nvv4l2h264enc (19 section 13.1) or nvv4l2h265enc (diagnostic) with positive bitrate and GOP");
            }
        } else if (cached_.hevc_sink == "stream_rtsp") {
            // Same video stream as file_rtsp (HEVC 4 Mbit/s GOP 30, camera output
            // size and fps, RTSP 8555/rgb, frame-sync SEI) without the forensic
            // MP4 file branch. Exact values only.
            if (cached_.hevc_width != cached_.camera_output_width ||
                cached_.hevc_height != cached_.camera_output_height ||
                cached_.hevc_fps != cached_.camera_output_fps) {
                invalid_orbbec_config("hevc dimensions and fps must match camera.output");
            }
            if (!cached_.hevc_enable || cached_.hevc_rtsp_listen != "rtsp://0.0.0.0:8555/rgb" ||
                cached_.hevc_bitrate != 4000000 ||
                cached_.hevc_codec != "nvv4l2h265enc" || cached_.hevc_gop != 30) {
                invalid_orbbec_config(
                    "hevc stream_rtsp must stream at rtsp://0.0.0.0:8555/rgb using "
                    "nvv4l2h265enc at 4000000 bit/s with GOP 30");
            }
        } else {
            if (cached_.hevc_width != cached_.camera_output_width ||
                cached_.hevc_height != cached_.camera_output_height ||
                cached_.hevc_fps != cached_.camera_output_fps) {
                invalid_orbbec_config("hevc dimensions and fps must match camera.output");
            }
            if (!cached_.hevc_enable || cached_.hevc_sink != "file_rtsp" ||
                cached_.hevc_rtsp_listen != "rtsp://0.0.0.0:8555/rgb" ||
                cached_.hevc_bitrate != 4000000 ||
                cached_.hevc_codec != "nvv4l2h265enc" || cached_.hevc_gop != 30) {
                invalid_orbbec_config(
                    "hevc must enable file_rtsp at rtsp://0.0.0.0:8555/rgb using "
                    "nvv4l2h265enc at 4000000 bit/s with GOP 30");
            }
        }
    }
    cached_.model_path =
        read_string_value(node_, "inference", "model_path", "../models/yolo11n-seg.onnx");
    cached_.use_dla = read_value<bool>(node_, "inference", "use_dla", true);
    cached_.dla_core = read_value<int>(node_, "inference", "dla_core", 0);
    cached_.trt_engine_path = read_string_value(
        node_, "inference", "trt_engine_path", "../models/yolo11n-seg-dla0-fp16.engine");
    cached_.trt_workspace_mb = read_value<int>(node_, "inference", "trt_workspace_mb", 1024);
    cached_.use_int8 = read_value<bool>(node_, "inference", "int8", false);
    // INT8 build inputs; paths as given, like model_path and trt_engine_path.
    cached_.int8_build = read_int8_build(node_, "inference", cached_.use_int8, cached_.use_dla);
    // From here on: optional keys with built-in defaults; a mistyped value refuses the load.
    cached_.model_version = read_string_value(node_, "inference", "model_version", "unknown");
    cached_.model_classes = read_string_vector(node_, "inference", "model_classes", default_model_classes());
    cached_.conf_threshold = read_value<float>(node_, "inference", "conf_threshold", 0.25F);
    cached_.per_class_conf = read_per_class_conf(node_);
    cached_.low_confidence_floor =
        read_value<float>(node_, "inference", "low_confidence_floor", 0.0F);
    cached_.low_confidence_max_candidates =
        read_value<int>(node_, "inference", "low_confidence_max_candidates", 8);
    cached_.iou_threshold = read_value<float>(node_, "inference", "iou_threshold", 0.45F);
    cached_.mask_threshold = read_value<float>(node_, "inference", "mask_threshold", 0.5F);
    cached_.input_size = read_value<int>(node_, "inference", "input_size", 640);
    cached_.use_fp16 = read_value<bool>(node_, "inference", "use_fp16", false);
    cached_.timeout_ms = read_value<int>(node_, "inference", "timeout_ms", 500);
    cached_.dual_model = read_dual_model_config(node_, path_);
    cached_.frame_queue_size = read_value<std::size_t>(node_, "queue", "frame_queue_size", 5);
    cached_.result_queue_size = read_value<std::size_t>(node_, "queue", "result_queue_size", 10);
    cached_.log_level = read_string_value(node_, "log", "level", "INFO");
    // V5 camera.intrinsics and pipeline_delay_ms: no R6 reader (R6 uses the SDK intrinsics).
    cached_.camera_intrinsics.fx =
        read_nested_value<float>(node_, "camera", "intrinsics", "fx", cached_.camera_intrinsics.fx);
    cached_.camera_intrinsics.fy =
        read_nested_value<float>(node_, "camera", "intrinsics", "fy", cached_.camera_intrinsics.fy);
    cached_.camera_intrinsics.cx =
        read_nested_value<float>(node_, "camera", "intrinsics", "cx", cached_.camera_intrinsics.cx);
    cached_.camera_intrinsics.cy =
        read_nested_value<float>(node_, "camera", "intrinsics", "cy", cached_.camera_intrinsics.cy);
    cached_.camera_intrinsics.width_px =
        read_nested_value<int>(node_, "camera", "intrinsics", "width_px", cached_.camera_intrinsics.width_px);
    cached_.camera_intrinsics.height_px =
        read_nested_value<int>(node_, "camera", "intrinsics", "height_px", cached_.camera_intrinsics.height_px);
    cached_.camera_intrinsics.calibration_date = read_nested_value<std::string>(
        node_, "camera", "intrinsics", "calibration_date", cached_.camera_intrinsics.calibration_date);
    cached_.camera_intrinsics.reprojection_rms_px = read_nested_value<float>(
        node_, "camera", "intrinsics", "reprojection_rms_px", cached_.camera_intrinsics.reprojection_rms_px);
    cached_.camera_pipeline_delay_ms = read_value<int>(node_, "camera", "pipeline_delay_ms", 0);
    cached_.track_threshold = read_value<float>(node_, "tracking", "track_thresh", 0.5F);
    cached_.track_buffer_frames = read_value<int>(node_, "tracking", "track_buffer_frames", 30);
    cached_.match_threshold = read_value<float>(node_, "tracking", "match_thresh", 0.8F);
    cached_.track_min_hits = read_value<int>(node_, "tracking", "min_hits", 3);
    cached_.track_low_confidence_min_hits =
        read_value<int>(node_, "tracking", "low_confidence_min_hits", 5);
    cached_.track_max_age = read_value<int>(node_, "tracking", "max_age", 5);
    cached_.track_per_class_max_ages = read_per_class_max_ages(node_);
    cached_.track_bbox_ema_alpha = read_value<float>(node_, "tracking", "bbox_ema_alpha", 0.4F);
    cached_.track_per_class_bbox_ema_alphas = read_per_class_bbox_ema_alphas(node_);
    cached_.depth_sample_mode = read_string_value(node_, "depth", "sample_mode", "nearest_cluster");
    cached_.depth_per_class_sample_modes = read_per_class_depth_sample_modes(node_);
    cached_.depth_percentile = read_value<float>(node_, "depth", "percentile", 0.20F);
    cached_.depth_cluster_width_m = read_value<float>(node_, "depth", "cluster_width_m", 0.12F);
    cached_.depth_cluster_min_fraction =
        read_value<float>(node_, "depth", "cluster_min_fraction", 0.08F);
    cached_.depth_cluster_min_pixels = read_value<int>(node_, "depth", "cluster_min_pixels", 8);
    cached_.depth_nearest_cluster_output_percentile =
        read_value<float>(node_, "depth", "nearest_cluster_output_percentile", 0.10F);
    cached_.depth_jump_threshold_m =
        read_value<float>(node_, "depth", "jump_threshold_m", 0.20F);
    cached_.depth_jump_candidate_tolerance_m =
        read_value<float>(node_, "depth", "jump_candidate_tolerance_m", 0.12F);
    cached_.depth_jump_confirmation_frames =
        read_value<int>(node_, "depth", "jump_confirmation_frames", 2);
    cached_.capture_frame_buffer_size = read_value<int>(node_, "capture", "frame_buffer_size", 20);
    cached_.evidence_quota_mb = read_value<int>(node_, "capture", "evidence_quota_mb", 1800);
    cached_.odom_stale_ms = read_value<float>(node_, "odom", "stale_ms", 200.0F);
    cached_.velocity_dt_max_s = read_value<float>(node_, "velocity", "dt_max_s", 0.5F);
    cached_.velocity_centroid_ema_alpha =
        read_value<float>(node_, "velocity", "centroid_ema_alpha", 0.35F);
    cached_.velocity_per_class_centroid_ema_alphas =
        read_per_class_centroid_ema_alphas(node_);
    cached_.velocity_window_s = read_value<float>(node_, "velocity", "window_s", 0.25F);
    cached_.velocity_motion_confirmation_windows =
        read_value<int>(node_, "velocity", "motion_confirmation_windows", 1);
    cached_.velocity_static_displacement_m =
        read_value<float>(node_, "velocity", "static_displacement_m", 0.04F);
    cached_.velocity_generic_static_displacement_m = read_value<float>(
        node_,
        "velocity",
        "generic_static_displacement_m",
        cached_.velocity_static_displacement_m);
    cached_.velocity_per_class_static_displacements_m =
        read_per_class_static_displacements(node_);
    cached_.velocity_gating_min_stable_frames =
        read_nested_value<int>(node_, "velocity", "gating", "min_stable_frames", 10);
    cached_.velocity_gating_min_depth_confidence =
        read_nested_value<float>(node_, "velocity", "gating", "min_depth_confidence", 0.5F);
    cached_.velocity_gating_skip_fusion_quality = read_string_vector(
        node_, "velocity", "gating", "skip_fusion_quality", {"depth_unreliable"});
    cached_.velocity_camera_z_uncertainty_gate_enabled = read_nested_value<bool>(
        node_, "velocity", "camera_z_uncertainty_gate", "enabled", false);
    cached_.velocity_camera_z_uncertainty_stddev_multiplier = read_nested_value<float>(
        node_, "velocity", "camera_z_uncertainty_gate", "stddev_multiplier", 1.0F);
    cached_.velocity_camera_z_uncertainty_max_threshold_m = read_nested_value<float>(
        node_, "velocity", "camera_z_uncertainty_gate", "max_threshold_m", 0.06F);
    // The R6 service refuses the retired extrinsic_base_lidar section, so these stay at the defaults.
    cached_.extrinsic_base_lidar_translation =
        read_array3(node_, "extrinsic_base_lidar", "translation", {0.0F, 0.0F, 0.0F});
    cached_.extrinsic_base_lidar_rotation_xyzw =
        read_array4(node_, "extrinsic_base_lidar", "rotation_xyzw", {0.0F, 0.0F, 0.0F, 1.0F});
}

std::string Config::rtsp_url() const { return cached_.rtsp_url; }
std::string Config::camera_driver() const { return cached_.camera_driver; }
std::string Config::camera_serial() const { return cached_.camera_serial; }
int Config::camera_color_width() const { return cached_.camera_color_width; }
int Config::camera_color_height() const { return cached_.camera_color_height; }
int Config::camera_color_fps() const { return cached_.camera_color_fps; }
std::string Config::camera_color_device_format() const {
    return cached_.camera_color_device_format;
}
int Config::camera_depth_width() const { return cached_.camera_depth_width; }
int Config::camera_depth_height() const { return cached_.camera_depth_height; }
int Config::camera_depth_fps() const { return cached_.camera_depth_fps; }
std::string Config::camera_depth_device_format() const {
    return cached_.camera_depth_device_format;
}
std::string Config::camera_alignment_mode() const { return cached_.camera_alignment_mode; }
bool Config::camera_align_to_color() const { return cached_.camera_align_to_color; }
int Config::camera_output_width() const { return cached_.camera_output_width; }
int Config::camera_output_height() const { return cached_.camera_output_height; }
int Config::camera_output_fps() const { return cached_.camera_output_fps; }
bool Config::d435_enabled() const { return cached_.d435_enable; }
int Config::d435_color_width() const { return cached_.d435_color_width; }
int Config::d435_color_height() const { return cached_.d435_color_height; }
int Config::d435_color_fps() const { return cached_.d435_color_fps; }
int Config::d435_depth_width() const { return cached_.d435_depth_width; }
int Config::d435_depth_height() const { return cached_.d435_depth_height; }
int Config::d435_depth_fps() const { return cached_.d435_depth_fps; }
bool Config::d435_align_to_color() const { return cached_.d435_align_to_color; }
int Config::d435_power_line_frequency() const { return cached_.d435_power_line_frequency; }
bool Config::hevc_enabled() const { return cached_.hevc_enable; }
bool Config::hevc_stream_sink() const {
    return cached_.hevc_sink == "stream_rtsp" || cached_.hevc_sink == "med2_loopback";
}
std::string Config::hevc_sink() const { return cached_.hevc_sink; }
std::string Config::hevc_rtsp_listen() const { return cached_.hevc_rtsp_listen; }
std::string Config::hevc_file_path() const { return cached_.hevc_file_path; }
int Config::hevc_width() const { return cached_.hevc_width; }
int Config::hevc_height() const { return cached_.hevc_height; }
int Config::hevc_fps() const { return cached_.hevc_fps; }
int Config::hevc_bitrate() const { return cached_.hevc_bitrate; }
std::string Config::hevc_codec() const { return cached_.hevc_codec; }
int Config::hevc_gop() const { return cached_.hevc_gop; }
std::string Config::model_path() const { return cached_.model_path; }
void Config::set_model_path(const std::string& path) { cached_.model_path = path; }
bool Config::use_dla() const { return cached_.use_dla; }
int Config::dla_core() const { return cached_.dla_core; }
std::string Config::trt_engine_path() const { return cached_.trt_engine_path; }
void Config::set_trt_engine_path(const std::string& path) { cached_.trt_engine_path = path; }
int Config::trt_workspace_mb() const { return cached_.trt_workspace_mb; }
bool Config::use_int8() const { return cached_.use_int8; }
const Config::Int8BuildConfig& Config::int8_build() const { return cached_.int8_build; }
float Config::conf_threshold() const { return cached_.conf_threshold; }
const std::map<int, float>& Config::per_class_confidence_thresholds() const { return cached_.per_class_conf; }
float Config::low_confidence_floor() const { return cached_.low_confidence_floor; }
int Config::low_confidence_max_candidates() const { return cached_.low_confidence_max_candidates; }
std::string Config::model_version() const { return cached_.model_version; }
std::vector<std::string> Config::model_classes() const { return cached_.model_classes; }
float Config::iou_threshold() const { return cached_.iou_threshold; }
float Config::mask_threshold() const { return cached_.mask_threshold; }
int Config::input_size() const { return cached_.input_size; }
bool Config::use_fp16() const { return cached_.use_fp16; }
int Config::timeout_ms() const { return cached_.timeout_ms; }
bool Config::dual_model_enabled() const { return cached_.dual_model.enabled; }
const Config::DualModelConfig& Config::dual_model() const { return cached_.dual_model; }
std::size_t Config::frame_queue_size() const { return cached_.frame_queue_size; }
std::size_t Config::result_queue_size() const { return cached_.result_queue_size; }
std::string Config::log_level() const { return cached_.log_level; }
Config::CameraIntrinsics Config::camera_intrinsics() const { return cached_.camera_intrinsics; }
int Config::camera_pipeline_delay_ms() const { return cached_.camera_pipeline_delay_ms; }
float Config::track_threshold() const { return cached_.track_threshold; }
int Config::track_buffer_frames() const { return cached_.track_buffer_frames; }
float Config::match_threshold() const { return cached_.match_threshold; }
int Config::track_min_hits() const { return cached_.track_min_hits; }
int Config::track_low_confidence_min_hits() const { return cached_.track_low_confidence_min_hits; }
int Config::track_max_age() const { return cached_.track_max_age; }
const std::map<std::string, int>& Config::track_per_class_max_ages() const {
    return cached_.track_per_class_max_ages;
}
float Config::track_bbox_ema_alpha() const { return cached_.track_bbox_ema_alpha; }
const std::map<std::string, float>& Config::track_per_class_bbox_ema_alphas() const {
    return cached_.track_per_class_bbox_ema_alphas;
}
std::string Config::depth_sample_mode() const { return cached_.depth_sample_mode; }
const std::map<std::string, std::string>& Config::depth_per_class_sample_modes() const {
    return cached_.depth_per_class_sample_modes;
}
float Config::depth_percentile() const { return cached_.depth_percentile; }
float Config::depth_cluster_width_m() const { return cached_.depth_cluster_width_m; }
float Config::depth_cluster_min_fraction() const { return cached_.depth_cluster_min_fraction; }
int Config::depth_cluster_min_pixels() const { return cached_.depth_cluster_min_pixels; }
float Config::depth_nearest_cluster_output_percentile() const {
    return cached_.depth_nearest_cluster_output_percentile;
}
float Config::depth_jump_threshold_m() const { return cached_.depth_jump_threshold_m; }
float Config::depth_jump_candidate_tolerance_m() const {
    return cached_.depth_jump_candidate_tolerance_m;
}
int Config::depth_jump_confirmation_frames() const {
    return cached_.depth_jump_confirmation_frames;
}
int Config::capture_frame_buffer_size() const { return cached_.capture_frame_buffer_size; }
int Config::evidence_quota_mb() const { return cached_.evidence_quota_mb; }
float Config::velocity_dt_max_s() const { return cached_.velocity_dt_max_s; }
float Config::velocity_centroid_ema_alpha() const { return cached_.velocity_centroid_ema_alpha; }
const std::map<std::string, float>& Config::velocity_per_class_centroid_ema_alphas() const {
    return cached_.velocity_per_class_centroid_ema_alphas;
}
float Config::velocity_window_s() const { return cached_.velocity_window_s; }
int Config::velocity_motion_confirmation_windows() const {
    return std::max(1, cached_.velocity_motion_confirmation_windows);
}
float Config::velocity_static_displacement_m() const { return cached_.velocity_static_displacement_m; }
float Config::velocity_generic_static_displacement_m() const {
    return cached_.velocity_generic_static_displacement_m;
}
const std::map<std::string, float>& Config::velocity_per_class_static_displacements_m() const {
    return cached_.velocity_per_class_static_displacements_m;
}
int Config::velocity_gating_min_stable_frames() const { return cached_.velocity_gating_min_stable_frames; }
float Config::velocity_gating_min_depth_confidence() const {
    return cached_.velocity_gating_min_depth_confidence;
}
std::vector<std::string> Config::velocity_gating_skip_fusion_quality() const {
    return cached_.velocity_gating_skip_fusion_quality;
}
bool Config::velocity_camera_z_uncertainty_gate_enabled() const {
    return cached_.velocity_camera_z_uncertainty_gate_enabled;
}
float Config::velocity_camera_z_uncertainty_stddev_multiplier() const {
    return cached_.velocity_camera_z_uncertainty_stddev_multiplier;
}
float Config::velocity_camera_z_uncertainty_max_threshold_m() const {
    return cached_.velocity_camera_z_uncertainty_max_threshold_m;
}
float Config::odom_stale_ms() const { return cached_.odom_stale_ms; }
std::array<float, 3> Config::extrinsic_base_lidar_translation() const {
    return cached_.extrinsic_base_lidar_translation;
}
std::array<float, 4> Config::extrinsic_base_lidar_rotation_xyzw() const {
    return cached_.extrinsic_base_lidar_rotation_xyzw;
}

}  // namespace yolo
