/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_config_env.cpp
 * Brief: Unit tests for configuration environment variable expansion.
 *
 * Description:
 *   1. Verifies ${VAR} expansion for the RTSP URL.
 *   2. Verifies unset variables resolve to empty values instead of defaults.
 *   3. dual_model.enabled is a required YAML bool that must be true: every config that must
 *      load carries the bundled sealed contracts, and a false, missing or non-bool key is
 *      refused before any sealed file is read (supersedes the B8-1 "dual model off reads no
 *      files" regression; that branch was removed in the level-3 trim).
 *   4. R1: a present but mistyped value refuses the load (missing or null keeps the default).
 *   5. INT8 build keys (inference.int8_*, dual_model.int8_*): presence and form rules, paths.
 *   6. dual_model.input_shape: only the two sealed inputs load; a missing or null key is the square.
 *   The checks of the historical V5 config files under config/ live in
 *   test_config_legacy_bundles.cpp (development only; config/ is not delivered).
 */

#include "common/config.hpp"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

constexpr const char* TEST_ENV = "PERCEPTION_TEST_CAM_PASSWORD";
constexpr const char* MISSING_ENV = "PERCEPTION_TEST_CAM_PASSWORD_MISSING";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string sealed_dual_model_yaml();

std::string write_config(const std::string& path, const std::string& env_name) {
    std::ofstream file(path);
    require(file.good(), "failed to open test config");

    file << "rtsp:\n"
         << "  url: \"rtsp://admin:${" << env_name << "}@192.168.18.108:554/id=1&type=0\"\n"
         << "camera:\n"
         << "  password: \"${" << env_name << "}\"\n"
         << "  pipeline_delay_ms: 37\n"
         << sealed_dual_model_yaml();
    file.close();
    return path;
}

const std::array<const char*, 12> ACTIVE12_NAMES = {
    "tree_trunk",
    "rock",
    "dirt_pile",
    "bush",
    "pole",
    "pillar",
    "pallet",
    "heavy_machinery",
    "traffic_cone",
    "barrier",
    "traversable_area",
    "pit"};

const std::array<int, 12> ACTIVE12_MAPPING = {
    80, 81, 82, 83, 84, 85, 86, 89, 90, 91, 92, 93};

std::filesystem::path source_root() {
    std::filesystem::path source_file(__FILE__);
    if (source_file.is_relative()) {
        source_file = std::filesystem::absolute(source_file);
    }
    return source_file.parent_path().parent_path();
}

void copy_bundled_sealed_contracts(
    const std::filesystem::path& runtime_destination,
    const std::filesystem::path& schema_destination) {
    const std::filesystem::path root = source_root();
    std::filesystem::copy_file(
        root / "config/dual_model_runtime_config.json",
        runtime_destination,
        std::filesystem::copy_options::overwrite_existing);
    std::filesystem::copy_file(
        root / "config/class_schema_94.json",
        schema_destination,
        std::filesystem::copy_options::overwrite_existing);
}

// dual_model.enabled is required and must be true (level-3 trim), so every
// config below that must load carries this block: the bundled sealed contracts
// by absolute path; the other dual_model keys keep their defaults.
std::string sealed_dual_model_yaml() {
    const std::filesystem::path root = source_root();
    return "dual_model:\n"
           "  enabled: true\n"
           "  runtime_config_path: \"" + (root / "config/dual_model_runtime_config.json").string() + "\"\n"
           "  class_schema_path: \"" + (root / "config/class_schema_94.json").string() + "\"\n";
}

void write_runtime_contract(
    const std::string& path,
    const std::string& conflict_mode = "audit_only",
    const std::string& reserved_ids = "87, 88",
    int heavy_mapping = 89) {
    std::ofstream file(path);
    require(file.good(), "failed to open dual runtime contract");
    file << "{\n"
         << "  \"branches\": {\n"
         << "    \"coco80\": {\"confidence_threshold\": 0.01, \"nms_iou_threshold\": 0.99},\n"
         << "    \"active12\": {\n"
         << "      \"input_shape\": [1, 3, 896, 896],\n"
         << "      \"confidence_threshold\": 0.25,\n"
         << "      \"nms_iou_threshold\": 0.70,\n"
         << "      \"mask_probability_threshold\": 0.50,\n"
         << "      \"mask_logit_threshold\": 0.0,\n"
         << "      \"class_aware_nms\": true,\n"
         << "      \"max_detections\": 300\n"
         << "    }\n"
         << "  },\n"
         << "  \"active12_local_classes\": {\n";
    for (std::size_t index = 0U; index < ACTIVE12_NAMES.size(); ++index) {
        file << "    \"" << index << "\": \"" << ACTIVE12_NAMES[index] << "\""
             << (index + 1U == ACTIVE12_NAMES.size() ? "\n" : ",\n");
    }
    file << "  },\n"
         << "  \"active12_to_global_id\": {\n";
    for (std::size_t index = 0U; index < ACTIVE12_MAPPING.size(); ++index) {
        const int global_id = index == 7U ? heavy_mapping : ACTIVE12_MAPPING[index];
        file << "    \"" << index << "\": " << global_id
             << (index + 1U == ACTIVE12_MAPPING.size() ? "\n" : ",\n");
    }
    file << "  },\n"
         << "  \"reserved_inactive_global_ids\": [" << reserved_ids << "],\n"
         << "  \"cross_model_merge\": {\n"
         << "    \"generic_cross_model_nms\": false,\n"
         << "    \"vehicle_heavy_conflict\": {\n"
         << "      \"coco_vehicle_ids\": [2, 5, 7],\n"
         << "      \"active_global_id\": 89,\n"
         << "      \"box_iou_threshold\": 0.30,\n"
         << "      \"mode\": \"" << conflict_mode << "\",\n"
         << "      \"automatic_suppression_enabled\": false\n"
         << "    }\n"
         << "  },\n"
         << "  \"safe_traversable\": {\n"
         << "    \"traversable_global_id\": 92,\n"
         << "    \"pit_global_id\": 93,\n"
         << "    \"active_obstacle_global_ids\": "
            "[80,81,82,83,84,85,86,89,90,91,93],\n"
         << "    \"coco_navigation_obstacle_ids\": [0,1,2,3,5,7,9,10,11,12,13,15],\n"
         << "    \"erosion_pixels\": 5,\n"
         << "    \"erosion_kernel\": \"OpenCV MORPH_ELLIPSE\"\n"
         << "  },\n"
         << "  \"tracking\": {\"excluded_global_ids\": [92]}\n"
         << "}\n";
}

void write_class_schema(const std::string& path, int class_count = 94) {
    std::vector<std::string> names(94U);
    for (std::size_t index = 0U; index < names.size(); ++index) {
        names[index] = "class_" + std::to_string(index);
    }
    for (std::size_t local_id = 0U; local_id < ACTIVE12_NAMES.size(); ++local_id) {
        names[static_cast<std::size_t>(ACTIVE12_MAPPING[local_id])] =
            ACTIVE12_NAMES[local_id];
    }
    names[87] = "carton";
    names[88] = "storage_rack";

    std::ofstream file(path);
    require(file.good(), "failed to open dual class schema");
    file << "{\n"
         << "  \"nc\": " << class_count << ",\n"
         << "  \"names\": {\n";
    for (std::size_t index = 0U; index < names.size(); ++index) {
        file << "    \"" << index << "\": \"" << names[index] << "\""
             << (index + 1U == names.size() ? "\n" : ",\n");
    }
    file << "  },\n"
         << "  \"old_output_ids\": [";
    for (int id = 0; id < 80; ++id) {
        file << id << (id == 79 ? "" : ",");
    }
    file << "],\n"
         << "  \"active_output_mapping\": {\n";
    for (std::size_t index = 0U; index < ACTIVE12_MAPPING.size(); ++index) {
        file << "    \"" << index << "\": " << ACTIVE12_MAPPING[index]
             << (index + 1U == ACTIVE12_MAPPING.size() ? "\n" : ",\n");
    }
    file << "  },\n"
         << "  \"inactive_reserved_ids\": [87, 88]\n"
         << "}\n";
}

std::string write_dual_yaml(
    const std::string& path,
    const std::string& runtime_path,
    const std::string& schema_path,
    bool enabled = true) {
    std::ofstream file(path);
    require(file.good(), "failed to open dual YAML config");
    file << "inference:\n"
         << "  conf_threshold: 0.30\n"
         << "  iou_threshold: 0.45\n"
         << "  per_class_conf:\n"
         << "    person: 0.40\n"
         << "dual_model:\n"
         << "  enabled: " << (enabled ? "true" : "false") << "\n"
         << "  runtime_config_path: \"" << runtime_path << "\"\n"
         << "  class_schema_path: \"" << schema_path << "\"\n"
         << "  active_model_path: \"models/active12.onnx\"\n"
         << "  active_engine_path: \"engines/active12.engine\"\n"
         << "  use_dla: true\n"
         << "  dla_core: 1\n"
         << "  use_fp16: true\n"
         << "  allow_gpu_fallback: true\n"
         << "  workspace_mb: 3072\n";
    return path;
}

void write_inline_config(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream file(path);
    require(file.good(), "failed to open inline test config");
    file << contents;
}

std::string valid_orbbec_camera_config() {
    return std::string(
        "hevc:\n"
        "  enable: true\n"
        "  sink: file_rtsp\n"
        "  rtsp_listen: rtsp://0.0.0.0:8555/rgb\n"
        "  width: 1920\n"
        "  height: 1080\n"
        "  fps: 30\n"
        "  bitrate: 4000000\n"
        "  codec: nvv4l2h265enc\n"
        "  gop: 30\n"
        "camera:\n"
        "  driver: orbbec_gemini338le\n"
        "  serial: CHL5663000DS\n"
        "  color:\n"
        "    width: 1280\n"
        "    height: 720\n"
        "    fps: 30\n"
        "    device_format: mjpg\n"
        "  depth:\n"
        "    width: 848\n"
        "    height: 530\n"
        "    fps: 30\n"
        "    device_format: y16\n"
        "  alignment_mode: hw_d2c\n"
        "  align_to_color: true\n"
        "  output:\n"
        "    width: 1920\n"
        "    height: 1080\n"
        "    fps: 30\n") +
        sealed_dual_model_yaml();
}

std::string replace_once(
    std::string value,
    const std::string& needle,
    const std::string& replacement) {
    const std::size_t position = value.find(needle);
    require(position != std::string::npos, "test fixture replacement target missing");
    value.replace(position, needle.size(), replacement);
    return value;
}

// An environment variable in a value is expanded (the RTSP password here), and an explicit camera
// pipeline delay is read.
void test_env_expansion() {
    setenv(TEST_ENV, "s3cr3t", 1);

    yolo::Config config;
    require(config.load(write_config("/tmp/xbrain_config_env_set.yaml", TEST_ENV)), "failed to load env config");
    require(config.camera_pipeline_delay_ms() == 37, "camera pipeline delay config mismatch");
    require(
        config.rtsp_url() == "rtsp://admin:s3cr3t@192.168.18.108:554/id=1&type=0",
        "rtsp url env expansion mismatch");
}

// An unset environment variable makes the whole value empty (fail closed), not a partial URL.
void test_unset_env_becomes_empty() {
    unsetenv(MISSING_ENV);

    yolo::Config config;
    require(config.load(write_config("/tmp/xbrain_config_env_missing.yaml", MISSING_ENV)), "failed to load missing config");
    require(config.rtsp_url().empty(), "missing rtsp password env did not fail closed to empty url");
}

// An explicit camera pipeline delay is read; a missing one is 0.
void test_default_pipeline_delay_is_zero() {
    yolo::Config config;
    require(config.load(write_config("/tmp/xbrain_config_default_delay.yaml", TEST_ENV)), "failed to load delay config");
    require(config.camera_pipeline_delay_ms() == 37, "explicit camera pipeline delay mismatch");

    std::ofstream file("/tmp/xbrain_config_no_delay.yaml");
    require(file.good(), "failed to open no-delay test config");
    file << "rtsp:\n"
         << "  url: \"rtsp://example.invalid/stream\"\n"
         << sealed_dual_model_yaml();
    file.close();

    yolo::Config no_delay_config;
    require(no_delay_config.load("/tmp/xbrain_config_no_delay.yaml"), "failed to load no-delay config");
    require(no_delay_config.camera_pipeline_delay_ms() == 0, "default camera pipeline delay was not zero");
}

// Camera intrinsics, model version and classes, per-class thresholds and the weak (low-confidence)
// path are read; without them the defaults hold (COCO 80, weak path off).
void test_camera_intrinsics_and_per_class_confidence() {
    std::ofstream file("/tmp/xbrain_config_intrinsics_per_class.yaml");
    require(file.good(), "failed to open intrinsics test config");
    file << "camera:\n"
         << "  intrinsics:\n"
         << "    fx: 100.5\n"
         << "    fy: 101.5\n"
         << "    cx: 320.0\n"
         << "    cy: 240.0\n"
         << "    width_px: 640\n"
         << "    height_px: 480\n"
         << "    calibration_date: \"2026-06-12\"\n"
         << "inference:\n"
         << "  model_version: \"test-model\"\n"
         << "  model_classes: [\"person\", \"car\"]\n"
         << "  low_confidence_floor: 0.12\n"
         << "  low_confidence_max_candidates: 4\n"
         << "  per_class_conf:\n"
         << "    person: 0.60\n"
         << "    2: 0.40\n"
         << "tracking:\n"
         << "  low_confidence_min_hits: 7\n"
         << sealed_dual_model_yaml();
    file.close();

    yolo::Config config;
    require(config.load("/tmp/xbrain_config_intrinsics_per_class.yaml"), "failed to load intrinsics config");
    const yolo::Config::CameraIntrinsics intrinsics = config.camera_intrinsics();
    require(intrinsics.fx == 100.5F, "camera intrinsics fx mismatch");
    require(intrinsics.fy == 101.5F, "camera intrinsics fy mismatch");
    require(intrinsics.cx == 320.0F, "camera intrinsics cx mismatch");
    require(intrinsics.cy == 240.0F, "camera intrinsics cy mismatch");
    require(intrinsics.width_px == 640, "camera intrinsics width mismatch");
    require(intrinsics.height_px == 480, "camera intrinsics height mismatch");
    require(intrinsics.calibration_date == "2026-06-12", "camera intrinsics date mismatch");
    require(config.model_version() == "test-model", "model version mismatch");
    require(config.model_classes().size() == 2U, "model classes config mismatch");
    require(config.per_class_confidence_thresholds().at(0) == 0.60F, "person threshold mismatch");
    require(config.per_class_confidence_thresholds().at(2) == 0.40F, "class id threshold mismatch");
    require(config.low_confidence_floor() == 0.12F, "low-confidence floor mismatch");
    require(config.low_confidence_max_candidates() == 4, "low-confidence candidate cap mismatch");
    require(config.track_low_confidence_min_hits() == 7, "low-confidence min hits mismatch");

    std::ofstream default_file("/tmp/xbrain_config_intrinsics_defaults.yaml");
    require(default_file.good(), "failed to open default intrinsics test config");
    default_file << "rtsp:\n"
                 << "  url: \"rtsp://example.invalid/stream\"\n"
                 << sealed_dual_model_yaml();
    default_file.close();

    yolo::Config default_config;
    require(default_config.load("/tmp/xbrain_config_intrinsics_defaults.yaml"), "failed to load default config");
    const yolo::Config::CameraIntrinsics defaults = default_config.camera_intrinsics();
    require(defaults.fx == 1393.5F, "default fx changed");
    require(defaults.fy == 1393.5F, "default fy changed");
    require(defaults.cx == 960.0F, "default cx changed");
    require(defaults.cy == 540.0F, "default cy changed");
    require(defaults.calibration_date == "2026-XX-XX", "default calibration date changed");
    require(default_config.per_class_confidence_thresholds().empty(), "default per-class thresholds not empty");
    require(default_config.low_confidence_floor() == 0.0F, "weak path was not disabled by default");
    require(default_config.low_confidence_max_candidates() == 8, "default weak candidate cap changed");
    require(default_config.track_low_confidence_min_hits() == 5, "default weak min hits changed");
    require(default_config.model_version() == "unknown", "default model version changed");
    require(default_config.model_classes().size() == 80U, "default model classes not COCO 80");
}

// D435 settings are read, a missing camera.driver means D435, and every D435, depth and tracking
// default holds when the sections are absent.
void test_d435_config_and_defaults() {
    std::ofstream file("/tmp/xbrain_config_d435.yaml");
    require(file.good(), "failed to open D435 config test file");
    file << "d435:\n"
         << "  enable: false\n"
         << "  color_width: 1280\n"
         << "  color_height: 720\n"
         << "  color_fps: 15\n"
         << "  depth_width: 848\n"
         << "  depth_height: 480\n"
         << "  depth_fps: 15\n"
         << "  align_to_color: false\n"
         << "  power_line_frequency: 1\n"
         << sealed_dual_model_yaml();
    file.close();

    yolo::Config config;
    require(config.load("/tmp/xbrain_config_d435.yaml"), "failed to load D435 config");
    require(config.camera_driver() == "d435", "missing camera.driver did not default to D435");
    require(!config.d435_enabled(), "D435 enable mismatch");
    require(config.d435_color_width() == 1280, "D435 color width mismatch");
    require(config.d435_color_height() == 720, "D435 color height mismatch");
    require(config.d435_color_fps() == 15, "D435 color FPS mismatch");
    require(config.d435_depth_width() == 848, "D435 depth width mismatch");
    require(config.d435_depth_height() == 480, "D435 depth height mismatch");
    require(config.d435_depth_fps() == 15, "D435 depth FPS mismatch");
    require(!config.d435_align_to_color(), "D435 align flag mismatch");
    require(config.d435_power_line_frequency() == 1, "D435 power line frequency mismatch");
    require(
        config.camera_output_width() == 0 && config.camera_output_height() == 0 &&
            config.camera_output_fps() == 0,
        "D435 config without camera.output changed behavior");

    std::ofstream defaults_file("/tmp/xbrain_config_d435_defaults.yaml");
    require(defaults_file.good(), "failed to open D435 defaults test file");
    defaults_file << "log:\n  level: INFO\n" << sealed_dual_model_yaml();
    defaults_file.close();

    yolo::Config defaults;
    require(defaults.load("/tmp/xbrain_config_d435_defaults.yaml"), "failed to load D435 defaults");
    require(defaults.camera_driver() == "d435", "default camera driver changed");
    require(defaults.d435_enabled(), "default D435 enable changed");
    require(defaults.d435_color_width() == 1920, "default D435 color width changed");
    require(defaults.d435_color_height() == 1080, "default D435 color height changed");
    require(defaults.d435_color_fps() == 30, "default D435 color FPS changed");
    require(defaults.d435_depth_width() == 1280, "default D435 depth width changed");
    require(defaults.d435_depth_height() == 720, "default D435 depth height changed");
    require(defaults.d435_depth_fps() == 30, "default D435 depth FPS changed");
    require(defaults.d435_align_to_color(), "default D435 align flag changed");
    require(defaults.d435_power_line_frequency() == -1, "default D435 power line frequency changed");
    require(
        defaults.camera_output_width() == 0 && defaults.camera_output_height() == 0 &&
            defaults.camera_output_fps() == 0,
        "default D435 config unexpectedly armed camera.output");
    require(defaults.depth_cluster_min_fraction() == 0.08F, "default depth cluster fraction changed");
    require(
        defaults.depth_nearest_cluster_output_percentile() == 0.10F,
        "default nearest-cluster output percentile changed");
    require(defaults.depth_jump_threshold_m() == 0.20F, "default depth jump threshold changed");
    require(
        defaults.depth_jump_candidate_tolerance_m() == 0.12F,
        "default depth jump candidate tolerance changed");
    require(
        defaults.depth_jump_confirmation_frames() == 2,
        "default depth jump confirmation changed");
    require(defaults.depth_per_class_sample_modes().empty(), "default per-class depth modes not empty");
    require(defaults.track_per_class_max_ages().empty(), "default per-class max ages not empty");
    require(
        defaults.track_per_class_bbox_ema_alphas().empty(),
        "default per-class bbox EMA alphas not empty");
    require(
        defaults.velocity_per_class_centroid_ema_alphas().empty(),
        "default per-class centroid EMA alphas not empty");
    require(
        defaults.velocity_per_class_static_displacements_m().empty(),
        "default per-class velocity displacements not empty");
}

// camera.driver and the Orbbec camera, depth, alignment and encoder fields must have the right
// type and an accepted value; otherwise the load is refused and a failed reload leaves no camera
// armed.
void test_camera_driver_yaml_type_validation() {
    const std::filesystem::path base =
        std::filesystem::temp_directory_path() / "xbrain_camera_driver_yaml_types";
    std::filesystem::create_directories(base);

    const std::filesystem::path missing_path = base / "missing.yaml";
    write_inline_config(missing_path, "camera:\n  pipeline_delay_ms: 37\n" + sealed_dual_model_yaml());
    yolo::Config missing;
    require(missing.load(missing_path.string()), "missing camera.driver did not load");
    require(missing.camera_driver() == "d435", "missing camera.driver did not default to D435");

    const std::filesystem::path scalar_path = base / "scalar.yaml";
    write_inline_config(scalar_path, valid_orbbec_camera_config());
    yolo::Config scalar;
    require(scalar.load(scalar_path.string()), "scalar camera.driver did not load");
    require(
        scalar.camera_driver() == "orbbec_gemini338le",
        "explicit scalar camera.driver changed");
    require(scalar.camera_serial() == "CHL5663000DS", "Orbbec serial mismatch");
    require(
        scalar.camera_color_width() == 1280 && scalar.camera_color_height() == 720 &&
            scalar.camera_color_fps() == 30 && scalar.camera_color_device_format() == "mjpg",
        "Orbbec color profile mismatch");
    require(
        scalar.camera_depth_width() == 848 && scalar.camera_depth_height() == 530 &&
            scalar.camera_depth_fps() == 30 && scalar.camera_depth_device_format() == "y16",
        "Orbbec depth profile mismatch");
    require(scalar.camera_alignment_mode() == "hw_d2c", "Orbbec alignment mode mismatch");
    require(scalar.camera_align_to_color(), "Orbbec alignment flag mismatch");
    require(
        scalar.camera_output_width() == 1920 && scalar.camera_output_height() == 1080 &&
            scalar.camera_output_fps() == 30,
        "Orbbec output profile mismatch");

    const std::filesystem::path official_depth_path = base / "official_depth_640x400.yaml";
    write_inline_config(
        official_depth_path,
        replace_once(
            valid_orbbec_camera_config(),
            "  depth:\n"
            "    width: 848\n"
            "    height: 530\n"
            "    fps: 30\n"
            "    device_format: y16\n",
            "  depth:\n"
            "    width: 640\n"
            "    height: 400\n"
            "    fps: 30\n"
            "    device_format: y16\n"));
    yolo::Config official_depth;
    require(
        official_depth.load(official_depth_path.string()),
        "officially validated 640x400 Orbbec depth profile did not load");
    require(
        official_depth.camera_depth_width() == 640 &&
            official_depth.camera_depth_height() == 400 &&
            official_depth.camera_depth_fps() == 30 &&
            official_depth.camera_depth_device_format() == "y16" &&
            official_depth.camera_alignment_mode() == "hw_d2c" &&
            official_depth.camera_align_to_color(),
        "640x400 Orbbec depth profile cache mismatch");

    std::string sw_d2c_contents = replace_once(
        valid_orbbec_camera_config(),
        "  depth:\n"
        "    width: 848\n"
        "    height: 530\n"
        "    fps: 30\n"
        "    device_format: y16\n",
        "  depth:\n"
        "    width: 1280\n"
        "    height: 800\n"
        "    fps: 30\n"
        "    device_format: y16\n");
    sw_d2c_contents = replace_once(
        sw_d2c_contents,
        "  alignment_mode: hw_d2c\n",
        "  alignment_mode: sw_d2c\n");
    const std::filesystem::path sw_d2c_path = base / "depth_1280x800_sw_d2c.yaml";
    write_inline_config(sw_d2c_path, sw_d2c_contents);
    yolo::Config sw_d2c;
    require(sw_d2c.load(sw_d2c_path.string()), "1280x800 SW-D2C profile did not load");
    require(
        sw_d2c.camera_depth_width() == 1280 && sw_d2c.camera_depth_height() == 800 &&
            sw_d2c.camera_alignment_mode() == "sw_d2c",
        "1280x800 SW-D2C profile cache mismatch");

    const std::filesystem::path legacy_mode_path = base / "legacy_default_hw_d2c.yaml";
    write_inline_config(
        legacy_mode_path,
        replace_once(valid_orbbec_camera_config(), "  alignment_mode: hw_d2c\n", ""));
    yolo::Config legacy_mode;
    require(legacy_mode.load(legacy_mode_path.string()), "legacy HW-D2C config did not load");
    require(
        legacy_mode.camera_alignment_mode() == "hw_d2c",
        "missing alignment_mode did not default to HW-D2C");

    struct MalformedCase {
        const char* name;
        const char* contents;
    };
    const std::array<MalformedCase, 3> malformed = {{
        {"sequence.yaml", "camera:\n  driver: [d435]\n"},
        {"map.yaml", "camera:\n  driver: {name: d435}\n"},
        {"null.yaml", "camera:\n  driver: null\n"},
    }};
    for (const MalformedCase& test_case : malformed) {
        const std::filesystem::path path = base / test_case.name;
        write_inline_config(path, test_case.contents + sealed_dual_model_yaml());
        yolo::Config config;
        require(!config.load(path.string()), "non-scalar camera.driver did not fail closed");
    }

    struct MissingOrbbecCase {
        const char* needle;
        const char* replacement;
    };
    const std::vector<MissingOrbbecCase> required_fields = {
        {"  serial: CHL5663000DS\n", ""},
        {"  color:\n"
         "    width: 1280\n"
         "    height: 720\n"
         "    fps: 30\n"
         "    device_format: mjpg\n", ""},
        {"    width: 1280\n", ""},
        {"    height: 720\n", ""},
        {"    fps: 30\n", ""},
        {"    device_format: mjpg\n", ""},
        {"  depth:\n"
         "    width: 848\n"
         "    height: 530\n"
         "    fps: 30\n"
         "    device_format: y16\n", ""},
        {"    width: 848\n", ""},
        {"    height: 530\n", ""},
        {"    height: 530\n    fps: 30\n", "    height: 530\n"},
        {"    device_format: y16\n", ""},
        {"  align_to_color: true\n", ""},
        {"  output:\n"
         "    width: 1920\n"
         "    height: 1080\n"
         "    fps: 30\n", ""},
        {"  output:\n    width: 1920\n", "  output:\n"},
        {"    width: 1920\n    height: 1080\n", "    width: 1920\n"},
        {"    height: 1080\n    fps: 30\n", "    height: 1080\n"},
    };
    for (std::size_t index = 0U; index < required_fields.size(); ++index) {
        const std::filesystem::path path = base / ("missing_" + std::to_string(index) + ".yaml");
        write_inline_config(
            path,
            replace_once(
                valid_orbbec_camera_config(),
                required_fields[index].needle,
                required_fields[index].replacement));
        yolo::Config config;
        require(!config.load(path.string()), "missing required Orbbec field did not fail closed");
    }

    struct InvalidOrbbecCase {
        const char* name;
        const char* needle;
        const char* replacement;
    };
    const std::vector<InvalidOrbbecCase> invalid_orbbec = {
        {"empty_serial", "CHL5663000DS", "\"\""},
        {"color_width", "width: 1280", "width: 640"},
        {"color_height", "height: 720", "height: 800"},
        {"color_fps",
         "    fps: 30\n    device_format: mjpg",
         "    fps: 20\n    device_format: mjpg"},
        {"color_format", "device_format: mjpg", "device_format: rgb"},
        {"unsupported_depth_profile",
         "  depth:\n"
         "    width: 848\n"
         "    height: 530\n"
         "    fps: 30\n"
         "    device_format: y16\n",
         "  depth:\n"
         "    width: 424\n"
         "    height: 266\n"
         "    fps: 30\n"
         "    device_format: y16\n"},
        {"hw_d2c_with_1280_depth",
         "  depth:\n"
         "    width: 848\n"
         "    height: 530\n"
         "    fps: 30\n"
         "    device_format: y16\n",
         "  depth:\n"
         "    width: 1280\n"
         "    height: 800\n"
         "    fps: 30\n"
         "    device_format: y16\n"},
        {"depth_height", "height: 530", "height: 480"},
        {"depth_fps", "    fps: 30\n    device_format: y16", "    fps: 15\n    device_format: y16"},
        {"depth_format", "device_format: y16", "device_format: z16"},
        {"alignment_mode", "alignment_mode: hw_d2c", "alignment_mode: automatic"},
        {"alignment_mode_type", "alignment_mode: hw_d2c", "alignment_mode: [hw_d2c]"},
        {"alignment_mode_null", "alignment_mode: hw_d2c", "alignment_mode: null"},
        {"alignment", "align_to_color: true", "align_to_color: false"},
        {"invalid_number", "width: 1280", "width: wide"},
        {"output_width", "    width: 1920\n", "    width: 1280\n"},
        {"output_height", "    height: 1080\n", "    height: 720\n"},
        {"output_fps", "    height: 1080\n    fps: 30", "    height: 1080\n    fps: 15"},
        {"serial_type", "serial: CHL5663000DS", "serial: [CHL5663000DS]"},
        {"color_map_type",
         "  color:\n"
         "    width: 1280\n"
         "    height: 720\n"
         "    fps: 30\n"
         "    device_format: mjpg\n",
         "  color: []\n"},
        {"color_scalar_type", "width: 1280", "width: [1280]"},
        {"color_format_type", "device_format: mjpg", "device_format: {name: mjpg}"},
        {"depth_map_type",
         "  depth:\n"
         "    width: 848\n"
         "    height: 530\n"
         "    fps: 30\n"
         "    device_format: y16\n",
         "  depth: []\n"},
        {"depth_scalar_type", "height: 530", "height: {pixels: 530}"},
        {"depth_format_type", "device_format: y16", "device_format: [y16]"},
        {"alignment_type", "align_to_color: true", "align_to_color: [true]"},
        {"output_map_type",
         "  output:\n"
         "    width: 1920\n"
         "    height: 1080\n"
         "    fps: 30\n",
         "  output: []\n"},
        {"output_width_type", "    width: 1920\n", "    width: [1920]\n"},
        {"output_height_type", "    height: 1080\n", "    height: {pixels: 1080}\n"},
        {"output_fps_type", "    height: 1080\n    fps: 30", "    height: 1080\n    fps: [30]"},
    };
    for (const InvalidOrbbecCase& test_case : invalid_orbbec) {
        const std::filesystem::path path = base / (std::string(test_case.name) + ".yaml");
        write_inline_config(
            path,
            replace_once(valid_orbbec_camera_config(), test_case.needle, test_case.replacement));
        yolo::Config config;
        require(!config.load(path.string()), "invalid Orbbec profile did not fail closed");
    }

    std::string sw_with_640_contents = replace_once(
        valid_orbbec_camera_config(),
        "  depth:\n"
        "    width: 848\n"
        "    height: 530\n"
        "    fps: 30\n"
        "    device_format: y16\n",
        "  depth:\n"
        "    width: 640\n"
        "    height: 400\n"
        "    fps: 30\n"
        "    device_format: y16\n");
    sw_with_640_contents = replace_once(
        sw_with_640_contents,
        "alignment_mode: hw_d2c",
        "alignment_mode: sw_d2c");
    const std::filesystem::path sw_with_640_path = base / "sw_d2c_with_640.yaml";
    write_inline_config(sw_with_640_path, sw_with_640_contents);
    yolo::Config sw_with_640;
    require(
        !sw_with_640.load(sw_with_640_path.string()),
        "640x400 was incorrectly accepted for the isolated SW-D2C route");

    const std::filesystem::path sw_with_848_path = base / "sw_d2c_with_848.yaml";
    write_inline_config(
        sw_with_848_path,
        replace_once(
            valid_orbbec_camera_config(),
            "alignment_mode: hw_d2c",
            "alignment_mode: sw_d2c"));
    yolo::Config sw_with_848;
    require(
        !sw_with_848.load(sw_with_848_path.string()),
        "848x530 was incorrectly accepted for the isolated SW-D2C route");

    const std::filesystem::path invalid_sw_reload_path = base / "invalid_sw_reload.yaml";
    write_inline_config(
        invalid_sw_reload_path,
        replace_once(sw_d2c_contents, "alignment_mode: sw_d2c", "alignment_mode: null"));
    require(!sw_d2c.load(invalid_sw_reload_path.string()), "invalid SW-D2C reload was accepted");
    require(
        sw_d2c.camera_driver().empty() && sw_d2c.camera_serial().empty() &&
            sw_d2c.camera_alignment_mode().empty() && sw_d2c.camera_depth_width() == 0 &&
            sw_d2c.camera_depth_height() == 0 && !sw_d2c.camera_align_to_color(),
        "failed SW-D2C reload left stale camera state armed");

    const std::filesystem::path invalid_reload_path = base / "invalid_reload.yaml";
    write_inline_config(
        invalid_reload_path,
        replace_once(valid_orbbec_camera_config(), "width: 1280", "width: 640"));
    require(!scalar.load(invalid_reload_path.string()), "invalid Gemini reload was accepted");
    require(
        scalar.camera_driver().empty() && scalar.camera_serial().empty() &&
            scalar.camera_color_width() == 0 && scalar.camera_depth_width() == 0 &&
            scalar.camera_alignment_mode().empty() && !scalar.camera_align_to_color() &&
            scalar.camera_output_width() == 0 &&
            scalar.camera_output_height() == 0 && scalar.camera_output_fps() == 0,
        "failed Gemini reload left stale camera state armed");

    const std::filesystem::path mismatched_hevc_path = base / "mismatched_hevc.yaml";
    write_inline_config(
        mismatched_hevc_path,
        replace_once(valid_orbbec_camera_config(), "  width: 1920\n", "  width: 1280\n"));
    yolo::Config mismatched_hevc;
    require(
        !mismatched_hevc.load(mismatched_hevc_path.string()),
        "Gemini HEVC dimensions mismatching camera.output did not fail closed");

    const std::vector<InvalidOrbbecCase> invalid_hevc_types = {
        {"hevc_width_type", "width: 1920", "width: [1920]"},
        {"hevc_height_type", "height: 1080", "height: {pixels: 1080}"},
        {"hevc_fps_type", "fps: 30", "fps: [30]"},
        {"hevc_enable", "enable: true", "enable: false"},
        {"hevc_sink", "sink: file_rtsp", "sink: file"},
        {"hevc_rtsp_listen",
         "rtsp_listen: rtsp://0.0.0.0:8555/rgb",
         "rtsp_listen: rtsp://127.0.0.1:8555/rgb"},
        {"hevc_bitrate", "bitrate: 4000000", "bitrate: 2500000"},
        {"hevc_codec", "codec: nvv4l2h265enc", "codec: x265enc"},
        {"hevc_gop", "gop: 30", "gop: 60"},
    };
    const std::string valid_orbbec_with_hevc = valid_orbbec_camera_config();
    for (const InvalidOrbbecCase& test_case : invalid_hevc_types) {
        const std::filesystem::path path =
            base / (std::string(test_case.name) + ".yaml");
        write_inline_config(
            path,
            replace_once(valid_orbbec_with_hevc, test_case.needle, test_case.replacement));
        yolo::Config config;
        require(!config.load(path.string()), "invalid Gemini HEVC YAML type did not fail closed");
    }

    const std::vector<std::string> required_hevc_fields = {
        "  enable: true\n",
        "  sink: file_rtsp\n",
        "  rtsp_listen: rtsp://0.0.0.0:8555/rgb\n",
        "  width: 1920\n",
        "  height: 1080\n",
        "  fps: 30\n",
        "  bitrate: 4000000\n",
        "  codec: nvv4l2h265enc\n",
        "  gop: 30\n",
    };
    for (std::size_t index = 0U; index < required_hevc_fields.size(); ++index) {
        const std::filesystem::path path =
            base / ("missing_hevc_" + std::to_string(index) + ".yaml");
        write_inline_config(
            path,
            replace_once(valid_orbbec_camera_config(), required_hevc_fields[index], ""));
        yolo::Config config;
        require(!config.load(path.string()), "missing Gemini HEVC field did not fail closed");
    }
}

// Only accepted Orbbec colour/depth/alignment/output size combinations load (native 800p
// included); a refused reload leaves the camera state cleared.
void test_orbbec_native800_tuple_validation() {
    const std::filesystem::path base =
        std::filesystem::temp_directory_path() / "xbrain_orbbec_native800_tuples";
    std::filesystem::create_directories(base);

    struct Dimensions {
        int width;
        int height;
    };
    const std::array<Dimensions, 3> depths = {{{848, 530}, {640, 400}, {1280, 800}}};
    const std::array<Dimensions, 2> outputs = {{{1920, 1080}, {1280, 800}}};
    const std::array<const char*, 4> accepted_tuples = {{
        "720_848_530_hw_d2c_1920_1080",
        "720_640_400_hw_d2c_1920_1080",
        "720_1280_800_sw_d2c_1920_1080",
        "800_640_400_sw_d2c_1280_800",
    }};
    std::string native800_contents;
    // Exercise the full 24-combination matrix with matching HEVC dimensions so
    // an unrelated HEVC mismatch cannot hide an over-broad camera validator.
    for (const int color_height : {720, 800}) {
        for (const Dimensions& depth : depths) {
            for (const char* alignment : {"hw_d2c", "sw_d2c"}) {
                for (const Dimensions& output : outputs) {
                    YAML::Node candidate = YAML::Load(valid_orbbec_camera_config());
                    candidate["camera"]["color"]["height"] = color_height;
                    candidate["camera"]["depth"]["width"] = depth.width;
                    candidate["camera"]["depth"]["height"] = depth.height;
                    candidate["camera"]["alignment_mode"] = alignment;
                    candidate["camera"]["output"]["width"] = output.width;
                    candidate["camera"]["output"]["height"] = output.height;
                    candidate["hevc"]["width"] = output.width;
                    candidate["hevc"]["height"] = output.height;
                    const std::string name = std::to_string(color_height) + "_" +
                        std::to_string(depth.width) + "_" + std::to_string(depth.height) + "_" +
                        alignment + "_" + std::to_string(output.width) + "_" +
                        std::to_string(output.height);
                    bool expected_accepted = false;
                    for (const char* accepted : accepted_tuples) {
                        expected_accepted = expected_accepted || name == accepted;
                    }
                    const std::string contents = YAML::Dump(candidate);
                    const std::filesystem::path path = base / (name + ".yaml");
                    write_inline_config(path, contents);
                    yolo::Config config;
                    require(
                        config.load(path.string()) == expected_accepted,
                        ("wrong camera tuple acceptance: " + name).c_str());
                    if (!expected_accepted) {
                        continue;
                    }
                    require(
                        config.camera_driver() == "orbbec_gemini338le" &&
                            config.camera_serial() == "CHL5663000DS" &&
                            config.camera_color_width() == 1280 &&
                            config.camera_color_height() == color_height &&
                            config.camera_color_fps() == 30 &&
                            config.camera_color_device_format() == "mjpg" &&
                            config.camera_depth_width() == depth.width &&
                            config.camera_depth_height() == depth.height &&
                            config.camera_depth_fps() == 30 &&
                            config.camera_depth_device_format() == "y16" &&
                            config.camera_alignment_mode() == alignment &&
                            config.camera_align_to_color() &&
                            config.camera_output_width() == output.width &&
                            config.camera_output_height() == output.height &&
                            config.camera_output_fps() == 30 &&
                            config.hevc_width() == output.width &&
                            config.hevc_height() == output.height && config.hevc_fps() == 30,
                        ("accepted camera tuple cache mismatch: " + name).c_str());
                    if (color_height == 800) {
                        native800_contents = contents;
                    }
                }
            }
        }
    }
    require(!native800_contents.empty(), "native 800p tuple was not exercised");

    struct InvalidNativeCase {
        const char* name;
        void (*invalidate)(YAML::Node&);
    };
    const std::vector<InvalidNativeCase> invalid_native = {
        {"color_width", [](YAML::Node& n) { n["camera"]["color"]["width"] = 640; }},
        {"color_fps", [](YAML::Node& n) { n["camera"]["color"]["fps"] = 15; }},
        {"color_format", [](YAML::Node& n) { n["camera"]["color"]["device_format"] = "rgb"; }},
        {"depth_height", [](YAML::Node& n) { n["camera"]["depth"]["height"] = 480; }},
        {"depth_fps", [](YAML::Node& n) { n["camera"]["depth"]["fps"] = 15; }},
        {"depth_format", [](YAML::Node& n) { n["camera"]["depth"]["device_format"] = "z16"; }},
        {"alignment_false", [](YAML::Node& n) { n["camera"]["align_to_color"] = false; }},
        {"alignment_missing", [](YAML::Node& n) { n["camera"].remove("alignment_mode"); }},
        {"alignment_null", [](YAML::Node& n) { n["camera"]["alignment_mode"] = YAML::Null; }},
        {"output_fps", [](YAML::Node& n) {
            n["camera"]["output"]["fps"] = 15;
            n["hevc"]["fps"] = 15;
        }},
        {"output_dimensions", [](YAML::Node& n) {
            n["camera"]["output"]["height"] = 720;
            n["hevc"]["height"] = 720;
        }},
        {"hevc_width", [](YAML::Node& n) { n["hevc"]["width"] = 1920; }},
        {"hevc_height", [](YAML::Node& n) { n["hevc"]["height"] = 1080; }},
        {"hevc_fps", [](YAML::Node& n) { n["hevc"]["fps"] = 15; }},
        {"hevc_width_missing", [](YAML::Node& n) { n["hevc"].remove("width"); }},
        {"hevc_height_type", [](YAML::Node& n) { n["hevc"]["height"] = YAML::Load("[800]"); }},
        {"hevc_disabled", [](YAML::Node& n) { n["hevc"]["enable"] = false; }},
        {"hevc_codec", [](YAML::Node& n) { n["hevc"]["codec"] = "x265enc"; }},
    };
    const std::filesystem::path valid_path = base / "valid_native800.yaml";
    write_inline_config(valid_path, native800_contents);
    yolo::Config reloaded;
    for (const InvalidNativeCase& test_case : invalid_native) {
        require(reloaded.load(valid_path.string()), "valid native 800p reload failed");
        YAML::Node candidate = YAML::Load(native800_contents);
        test_case.invalidate(candidate);
        const std::filesystem::path path = base / (std::string(test_case.name) + ".yaml");
        write_inline_config(path, YAML::Dump(candidate));
        require(
            !reloaded.load(path.string()),
            (std::string("invalid native 800p reload accepted: ") + test_case.name).c_str());
        require(
            reloaded.camera_driver().empty() && reloaded.camera_serial().empty() &&
                reloaded.camera_color_device_format().empty() &&
                reloaded.camera_depth_device_format().empty() &&
                reloaded.camera_alignment_mode().empty() &&
                reloaded.camera_color_width() == 0 && reloaded.camera_color_height() == 0 &&
                reloaded.camera_color_fps() == 0 && reloaded.camera_depth_width() == 0 &&
                reloaded.camera_depth_height() == 0 && reloaded.camera_depth_fps() == 0 &&
                !reloaded.camera_align_to_color() && reloaded.camera_output_width() == 0 &&
                reloaded.camera_output_height() == 0 && reloaded.camera_output_fps() == 0,
            (std::string("failed native 800p reload left camera state armed: ") +
             test_case.name).c_str());
    }
}

YAML::Node valid_native800_med2_loopback() {
    YAML::Node n = YAML::Load(valid_orbbec_camera_config());
    n["camera"]["color"]["height"] = 800;
    n["camera"]["depth"]["width"] = 640;
    n["camera"]["depth"]["height"] = 400;
    n["camera"]["alignment_mode"] = "sw_d2c";
    n["camera"]["output"]["width"] = 1280;
    n["camera"]["output"]["height"] = 800;
    n["hevc"]["width"] = 1280;
    n["hevc"]["height"] = 800;
    n["hevc"]["sink"] = "med2_loopback";
    n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.1:18083/rgb";
    n["hevc"]["bitrate"] = 2000000;
    n["hevc"]["gop"] = 15;
    return n;
}

// The med2_loopback sink loads, load_from_node equals load(path), the 19 section 13.1 default
// preview (H.264 1280x720@15) loads, and bad stream values are refused.
void test_med2_loopback_sink_and_node_load() {
    const std::filesystem::path base =
        std::filesystem::temp_directory_path() / "xbrain_med2_loopback";
    std::filesystem::create_directories(base);
    const YAML::Node valid = valid_native800_med2_loopback();
    const std::filesystem::path valid_path = base / "valid.yaml";
    write_inline_config(valid_path, YAML::Dump(valid));
    yolo::Config from_file;
    require(from_file.load(valid_path.string()), "med2_loopback config rejected");
    require(from_file.hevc_stream_sink() && from_file.hevc_sink() == "med2_loopback" &&
                from_file.hevc_rtsp_listen() == "rtsp://127.0.0.1:18083/rgb" &&
                from_file.hevc_bitrate() == 2000000 && from_file.hevc_gop() == 15,
            "med2_loopback cache mismatch");
    yolo::Config from_node;
    require(from_node.load_from_node(valid, "/opt/xbrain_v6/data/run/resolved/perception.yaml"),
            "loadFromNode rejected the same document");
    require(from_node.path() == "/opt/xbrain_v6/data/run/resolved/perception.yaml" &&
                from_node.hevc_rtsp_listen() == from_file.hevc_rtsp_listen() &&
                from_node.camera_serial() == from_file.camera_serial() &&
                from_node.camera_output_width() == 1280 && from_node.hevc_stream_sink(),
            "loadFromNode differs from load(path)");
    YAML::Node stream = valid_native800_med2_loopback();
    stream["hevc"]["sink"] = "stream_rtsp";
    stream["hevc"]["rtsp_listen"] = "rtsp://0.0.0.0:8555/rgb";
    stream["hevc"]["bitrate"] = 4000000;
    stream["hevc"]["gop"] = 30;
    yolo::Config stream_config;
    require(stream_config.load_from_node(stream, (base / "stream.yaml").string()) &&
                stream_config.hevc_stream_sink(),
            "stream_rtsp no longer treated as a stream sink");
    // 19 section 13.1 default preview: H.264 1280x720@15 from the 1280x800@30 output
    YAML::Node contract = valid_native800_med2_loopback();
    contract["hevc"]["codec"] = "nvv4l2h264enc";
    contract["hevc"]["height"] = 720;
    contract["hevc"]["fps"] = 15;
    yolo::Config contract_config;
    require(contract_config.load_from_node(contract, (base / "contract.yaml").string()) &&
                contract_config.hevc_codec() == "nvv4l2h264enc" && contract_config.hevc_height() == 720 &&
                contract_config.hevc_fps() == 15 && contract_config.camera_output_height() == 800,
            "19 section 13.1 H.264 1280x720@15 preview rejected");
    struct Invalid { const char* name; void (*mutate)(YAML::Node&); };
    const std::vector<Invalid> invalid = {
        {"wildcard_bind", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://0.0.0.0:18083/rgb"; }},
        {"lan_bind", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://192.168.1.23:18083/rgb"; }},
        // Same length as 127.0.0.1: a fixed-width parse cannot reject these by accident.
        {"other_loopback", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.2:18083/rgb"; }},
        {"lan_same_length", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://10.1.1.23:18083/rgb"; }},
        {"port_zero", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.1:0/rgb"; }},
        {"port_leading_zero", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.1:018083/rgb"; }},
        {"port_range", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.1:65536/rgb"; }},
        {"no_path", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.1:18083"; }},
        {"empty_path", [](YAML::Node& n) { n["hevc"]["rtsp_listen"] = "rtsp://127.0.0.1:18083/"; }},
        {"bitrate_zero", [](YAML::Node& n) { n["hevc"]["bitrate"] = 0; }},
        {"gop_zero", [](YAML::Node& n) { n["hevc"]["gop"] = 0; }},
        {"codec", [](YAML::Node& n) { n["hevc"]["codec"] = "x264enc"; }},
        {"disabled", [](YAML::Node& n) { n["hevc"]["enable"] = false; }},
        // B8-6: the preview is a centre crop at a rate dividing the camera rate
        {"fps_not_dividing", [](YAML::Node& n) { n["hevc"]["fps"] = 20; }},
        {"fps_above_camera", [](YAML::Node& n) { n["hevc"]["fps"] = 60; }},
        {"crop_odd", [](YAML::Node& n) { n["hevc"]["height"] = 721; }},
        {"taller_than_camera", [](YAML::Node& n) { n["hevc"]["height"] = 900; }},
        {"width_scaling", [](YAML::Node& n) { n["hevc"]["width"] = 1920; }},
    };
    for (const auto& test_case : invalid) {
        YAML::Node candidate = valid_native800_med2_loopback();
        test_case.mutate(candidate);
        yolo::Config reloaded;
        require(reloaded.load_from_node(valid, "valid"), "valid med2_loopback reload failed");
        require(!reloaded.load_from_node(candidate, test_case.name),
                (std::string("invalid med2_loopback accepted: ") + test_case.name).c_str());
        require(reloaded.camera_driver().empty() && reloaded.camera_serial().empty(),
                (std::string("failed node load left camera armed: ") + test_case.name).c_str());
    }
}

// Tracking stability, depth sampling and velocity settings are read, including the per-class maps,
// whose invalid entries are skipped.
void test_stability_config() {
    std::ofstream file("/tmp/perception_stability_config.yaml");
    require(file.good(), "failed to open stability config");
    file << "tracking:\n"
         << "  bbox_ema_alpha: 0.42\n"
         << "  per_class_max_age:\n"
         << "    suitcase: 9\n"
         << "    person: -1\n"
         << "  per_class_bbox_ema_alpha:\n"
         << "    suitcase: 0.05\n"
         << "    chair: 1.01\n"
         << "depth:\n"
         << "  sample_mode: nearest_cluster\n"
         << "  per_class_sample_mode:\n"
         << "    suitcase: median\n"
         << "    chair: invalid_mode\n"
         << "  percentile: 0.18\n"
         << "  cluster_width_m: 0.15\n"
         << "  cluster_min_fraction: 0.04\n"
         << "  cluster_min_pixels: 11\n"
         << "  nearest_cluster_output_percentile: 0.12\n"
         << "  jump_threshold_m: 0.25\n"
         << "  jump_candidate_tolerance_m: 0.14\n"
         << "  jump_confirmation_frames: 3\n"
         << "velocity:\n"
         << "  centroid_ema_alpha: 0.31\n"
         << "  per_class_centroid_ema_alpha:\n"
         << "    chair: 0.05\n"
         << "    person: 0.0\n"
         << "  motion_confirmation_windows: 2\n"
         << "  static_displacement_m: 0.05\n"
         << "  per_class_static_displacement_m:\n"
         << "    chair: 0.10\n"
         << "    bottle: -0.01\n"
         << sealed_dual_model_yaml();
    file.close();

    yolo::Config config;
    require(config.load("/tmp/perception_stability_config.yaml"), "failed to load stability config");
    require(config.track_bbox_ema_alpha() == 0.42F, "bbox EMA config mismatch");
    require(config.track_per_class_max_ages().size() == 1U, "per-class max age count mismatch");
    require(config.track_per_class_max_ages().at("suitcase") == 9, "suitcase max age mismatch");
    require(
        config.track_per_class_bbox_ema_alphas().size() == 1U,
        "per-class bbox EMA alpha count mismatch");
    require(
        config.track_per_class_bbox_ema_alphas().at("suitcase") == 0.05F,
        "suitcase bbox EMA alpha mismatch");
    require(config.depth_sample_mode() == "nearest_cluster", "depth sample mode mismatch");
    require(config.depth_per_class_sample_modes().size() == 1U, "per-class depth mode count mismatch");
    require(
        config.depth_per_class_sample_modes().at("suitcase") == "median",
        "suitcase depth mode mismatch");
    require(config.depth_percentile() == 0.18F, "depth percentile mismatch");
    require(config.depth_cluster_width_m() == 0.15F, "depth cluster width mismatch");
    require(config.depth_cluster_min_fraction() == 0.04F, "depth cluster fraction mismatch");
    require(config.depth_cluster_min_pixels() == 11, "depth cluster pixels mismatch");
    require(
        config.depth_nearest_cluster_output_percentile() == 0.12F,
        "nearest-cluster output percentile mismatch");
    require(config.depth_jump_threshold_m() == 0.25F, "depth jump threshold mismatch");
    require(
        config.depth_jump_candidate_tolerance_m() == 0.14F,
        "depth jump candidate tolerance mismatch");
    require(config.depth_jump_confirmation_frames() == 3, "depth jump confirmation mismatch");
    require(config.velocity_centroid_ema_alpha() == 0.31F, "centroid EMA config mismatch");
    require(
        config.velocity_per_class_centroid_ema_alphas().size() == 1U,
        "per-class centroid EMA alpha count mismatch");
    require(
        config.velocity_per_class_centroid_ema_alphas().at("chair") == 0.05F,
        "chair centroid EMA alpha mismatch");
    require(
        config.velocity_motion_confirmation_windows() == 2,
        "motion confirmation windows mismatch");
    require(config.velocity_static_displacement_m() == 0.05F, "velocity deadzone config mismatch");
    require(
        config.velocity_per_class_static_displacements_m().size() == 1U,
        "per-class velocity displacement count mismatch");
    require(
        config.velocity_per_class_static_displacements_m().at("chair") == 0.10F,
        "chair velocity displacement mismatch");
}

// Without a generic-class static displacement (velocity deadzone) the global value is used; an
// explicit one overrides it.
void test_generic_static_displacement_fallback_and_explicit_override() {
    std::ofstream fallback_file("/tmp/perception_generic_velocity_fallback.yaml");
    require(fallback_file.good(), "failed to open generic velocity fallback config");
    fallback_file << "velocity:\n"
                  << "  static_displacement_m: 0.02\n"
                  << sealed_dual_model_yaml();
    fallback_file.close();

    yolo::Config fallback;
    require(
        fallback.load("/tmp/perception_generic_velocity_fallback.yaml"),
        "failed to load generic velocity fallback config");
    require(
        fallback.velocity_static_displacement_m() == 0.02F,
        "fallback test global velocity deadzone mismatch");
    require(
        fallback.velocity_generic_static_displacement_m() ==
            fallback.velocity_static_displacement_m(),
        "missing generic velocity deadzone did not fall back to the global value");

    std::ofstream explicit_file("/tmp/perception_generic_velocity_explicit.yaml");
    require(explicit_file.good(), "failed to open explicit generic velocity config");
    explicit_file << "velocity:\n"
                  << "  static_displacement_m: 0.02\n"
                  << "  generic_static_displacement_m: 0.06\n"
                  << sealed_dual_model_yaml();
    explicit_file.close();

    yolo::Config explicit_config;
    require(
        explicit_config.load("/tmp/perception_generic_velocity_explicit.yaml"),
        "failed to load explicit generic velocity config");
    require(
        explicit_config.velocity_static_displacement_m() == 0.02F,
        "explicit global velocity deadzone mismatch");
    require(
        explicit_config.velocity_generic_static_displacement_m() == 0.06F,
        "explicit generic velocity deadzone mismatch");
}

// The camera-Z uncertainty gate defaults off (multiplier 1.0, cap 0.06 m); explicit values are
// read.
void test_camera_z_uncertainty_gate_defaults_and_explicit_config() {
    std::ofstream default_file("/tmp/perception_camera_z_uncertainty_default.yaml");
    require(default_file.good(), "failed to open camera-Z default config");
    default_file << "velocity:\n"
                 << "  window_s: 0.30\n"
                 << sealed_dual_model_yaml();
    default_file.close();

    yolo::Config defaults;
    require(
        defaults.load("/tmp/perception_camera_z_uncertainty_default.yaml"),
        "failed to load camera-Z default config");
    require(
        !defaults.velocity_camera_z_uncertainty_gate_enabled(),
        "missing camera-Z uncertainty config did not default off");
    require(
        defaults.velocity_camera_z_uncertainty_stddev_multiplier() == 1.0F,
        "camera-Z uncertainty default multiplier mismatch");
    require(
        defaults.velocity_camera_z_uncertainty_max_threshold_m() == 0.06F,
        "camera-Z uncertainty default cap mismatch");

    std::ofstream explicit_file("/tmp/perception_camera_z_uncertainty_enabled.yaml");
    require(explicit_file.good(), "failed to open camera-Z explicit config");
    explicit_file << "velocity:\n"
                  << "  camera_z_uncertainty_gate:\n"
                  << "    enabled: true\n"
                  << "    stddev_multiplier: 1.0\n"
                  << "    max_threshold_m: 0.06\n"
                  << sealed_dual_model_yaml();
    explicit_file.close();

    yolo::Config explicit_config;
    require(
        explicit_config.load("/tmp/perception_camera_z_uncertainty_enabled.yaml"),
        "failed to load camera-Z explicit config");
    require(
        explicit_config.velocity_camera_z_uncertainty_gate_enabled(),
        "explicit camera-Z uncertainty gate was not enabled");
    require(
        explicit_config.velocity_camera_z_uncertainty_stddev_multiplier() == 1.0F,
        "explicit camera-Z uncertainty multiplier mismatch");
    require(
        explicit_config.velocity_camera_z_uncertainty_max_threshold_m() == 0.06F,
        "explicit camera-Z uncertainty cap mismatch");
}

// A valid dual-model config resolves paths from the YAML directory, takes every active12 value
// from the sealed runtime JSON and class schema (hashes checked) without touching the COCO
// settings, and reads no files when off.
void test_dual_model_defaults_and_sealed_contract() {
    const std::filesystem::path base("/tmp/xbrain_dual_config");
    const std::filesystem::path sealed = base / "sealed";
    std::filesystem::create_directories(sealed);
    copy_bundled_sealed_contracts(sealed / "runtime.json", sealed / "schema.json");
    const std::string config_path = write_dual_yaml(
        (base / "perception.yaml").string(),
        "sealed/runtime.json",
        "sealed/schema.json");

    yolo::Config config;
    require(config.load(config_path), "failed to load valid dual-model config");
    require(config.dual_model_enabled(), "valid dual-model config was not enabled");
    const yolo::Config::DualModelConfig& dual = config.dual_model();
    require(
        dual.runtime_config_path == (sealed / "runtime.json").lexically_normal().string(),
        "dual runtime relative path was not resolved from YAML directory");
    require(
        dual.class_schema_path == (sealed / "schema.json").lexically_normal().string(),
        "dual schema relative path was not resolved from YAML directory");
    require(
        dual.active_model_path == (base / "models/active12.onnx").lexically_normal().string(),
        "dual ONNX relative path mismatch");
    require(
        dual.active_engine_path ==
            (base / "engines/active12.engine").lexically_normal().string(),
        "dual engine relative path mismatch");
    require(dual.input_size == 896, "active12 sealed input size mismatch");
    require(dual.input_height == 896, "active12 input is not the square without dual_model.input_shape");
    require(
        dual.runtime_config_sha256 ==
            "dd6f857d79492973b0c2910ed11430587922c58c9f00da900521f3ea9fdc0aca",
        "sealed runtime SHA-256 mismatch");
    require(
        dual.class_schema_sha256 ==
            "c00cdc6ef6d72101e60a405a54cf404ed12af54b458dfaf29dd2ed15a1ba8505",
        "sealed class schema SHA-256 mismatch");
    require(dual.conf_threshold == 0.25F, "active12 sealed confidence mismatch");
    require(dual.nms_iou_threshold == 0.70F, "active12 sealed NMS mismatch");
    require(dual.mask_threshold == 0.50F, "active12 sealed mask threshold mismatch");
    require(dual.max_detections == 300, "active12 sealed max detections mismatch");
    require(dual.local_class_names.size() == 12U, "active12 class count mismatch");
    require(
        dual.local_to_global == std::vector<int>(ACTIVE12_MAPPING.begin(), ACTIVE12_MAPPING.end()),
        "active12 local/global mapping mismatch");
    require(dual.reserved_ids == std::vector<int>({87, 88}), "reserved ID contract mismatch");
    require(dual.global_class_names.size() == 94U, "global class schema size mismatch");
    require(
        dual.global_class_names[80] == "tree_trunk" &&
            dual.global_class_names[87] == "carton" &&
            dual.global_class_names[88] == "storage_rack" &&
            dual.global_class_names[93] == "pit",
        "global class schema names mismatch");
    require(dual.conflict_mode == "audit_only", "conflict mode mismatch");
    require(dual.conflict_iou == 0.30F, "conflict IoU mismatch");
    require(dual.coco_vehicle_ids == std::vector<int>({2, 5, 7}), "vehicle ID list mismatch");
    require(dual.heavy_id == 89, "heavy-machinery global ID mismatch");
    require(
        dual.tracker_excluded_ids == std::vector<int>({92}),
        "tracker exclusion mismatch");
    require(
        dual.safe_traversable.traversable_global_id == 92,
        "traversable global ID mismatch");
    require(dual.safe_traversable.pit_global_id == 93, "pit global ID mismatch");
    require(dual.safe_traversable.erosion_pixels == 5, "safe erosion mismatch");
    require(dual.workspace_mb == 3072, "dual deployment workspace mismatch");
    require(dual.use_dla && dual.dla_core == 1, "dual DLA deployment mismatch");
    require(dual.use_fp16 && dual.allow_gpu_fallback, "dual fallback deployment mismatch");

    // The sealed JSON deliberately contains different coco80 values. It must
    // never override the legacy branch's established YAML settings.
    require(config.conf_threshold() == 0.30F, "sealed JSON overwrote legacy COCO confidence");
    require(config.iou_threshold() == 0.45F, "sealed JSON overwrote legacy COCO NMS");
    require(
        config.per_class_confidence_thresholds().at(0) == 0.40F,
        "sealed JSON overwrote legacy per-class confidence");
}

// Config logs a refused load as one ERROR line; in the default synchronous
// logger mode ERROR lines go to std::cerr, so the refusal reason is read there.
bool load_capturing_errors(yolo::Config& config, const std::string& path, std::string& errors) {
    std::ostringstream captured;
    std::streambuf* const previous = std::cerr.rdbuf(captured.rdbuf());
    const bool loaded = config.load(path);
    std::cerr.rdbuf(previous);
    errors = captured.str();
    return loaded;
}

// Supersedes the B8-1 regression "dual model off reads no files" (level-3 trim,
// 2026-09-29): the disabled branch is gone. dual_model.enabled is a required
// YAML bool that must be true; false, a missing key or a non-bool value is
// refused BEFORE any sealed file is read. Negative control: the same absent
// runtime JSON with the key true is refused as a file error, so a refusal
// triggered by the missing file (a read) would be seen.
void test_dual_model_required_and_refused_before_read() {
    const char* async_log = std::getenv("PERCEPTION_ASYNC_LOG");
    require(
        async_log == nullptr || std::string(async_log) != "1",
        "run with the synchronous logger: the refusal reason is read from std::cerr");
    const std::filesystem::path base("/tmp/xbrain_dual_required");
    std::filesystem::create_directories(base);
    copy_bundled_sealed_contracts(base / "runtime.json", base / "schema.json");
    const std::filesystem::path absent = base / "absent_runtime.json";
    std::filesystem::remove(absent);
    const auto has = [](const std::string& text, const std::string& part) {
        return text.find(part) != std::string::npos;
    };
    const auto refusal = [](yolo::Config& config, const std::string& name, const std::string& path) {
        std::string errors;
        require(!load_capturing_errors(config, path, errors), (name + " was accepted").c_str());
        require(!config.dual_model_enabled(), (name + " left the dual model armed").c_str());
        require(!errors.empty(), (name + ": no refusal reason captured from std::cerr").c_str());
        return errors;
    };

    // false with the runtime JSON absent: refused for the key, not for the file.
    // The same Config first loads the enabled model, so the refusal also disarms it.
    yolo::Config reloaded;
    require(
        reloaded.load(write_dual_yaml((base / "on_present.yaml").string(), "runtime.json", "schema.json")) &&
            reloaded.dual_model_enabled(),
        "positive control: enabled dual model with the sealed files did not load");
    const std::string off_absent = refusal(
        reloaded,
        "enabled: false (runtime JSON absent)",
        write_dual_yaml((base / "off_absent.yaml").string(), absent.string(), "schema.json", false));
    require(
        has(off_absent, "dual_model.enabled must be true") && !has(off_absent, "cannot open file"),
        "disabled dual model was not refused before the sealed files were read");

    // Negative control: the same absent file with the key true is a file refusal.
    yolo::Config on_absent_config;
    const std::string on_absent = refusal(
        on_absent_config,
        "enabled: true (runtime JSON absent)",
        write_dual_yaml((base / "on_absent.yaml").string(), absent.string(), "schema.json", true));
    require(
        has(on_absent, "cannot open file: " + absent.string()) && !has(on_absent, "dual_model.enabled"),
        "negative control: the absent runtime JSON was not reported as a file error");

    // false with both sealed files present: the same refusal (file presence is irrelevant).
    yolo::Config off_present_config;
    const std::string off_present = refusal(
        off_present_config,
        "enabled: false (sealed files present)",
        write_dual_yaml((base / "off_present.yaml").string(), "runtime.json", "schema.json", false));
    require(has(off_present, "dual_model.enabled must be true"), "disabled dual model with sealed files was not refused");

    // Missing section, missing key, null or non-bool value: no default, same point.
    struct Case {
        const char* name;
        std::string block;
    };
    const std::string absent_path = "  runtime_config_path: \"" + absent.string() + "\"\n";
    const std::vector<Case> cases = {
        {"missing_section", ""},
        {"missing_key", "dual_model:\n" + absent_path},
        {"null_value", "dual_model:\n  enabled: null\n" + absent_path},
        {"non_bool_value", "dual_model:\n  enabled: 1\n" + absent_path},
        {"sequence_value", "dual_model:\n  enabled: [true]\n" + absent_path},
    };
    for (const Case& test_case : cases) {
        const std::filesystem::path path = base / (std::string(test_case.name) + ".yaml");
        write_inline_config(path, "log:\n  level: INFO\n" + test_case.block);
        yolo::Config config;
        const std::string errors = refusal(config, test_case.name, path.string());
        require(
            has(errors, "dual_model.enabled is required and must be a YAML bool") &&
                !has(errors, "cannot open file"),
            (std::string("dual_model.enabled not refused before any read: ") + test_case.name).c_str());
    }

    // Read as a YAML bool, as the service's PSC-2 check reads it: other spellings of true load.
    const std::filesystem::path spelled_path = base / "spelled_yes.yaml";
    write_inline_config(
        spelled_path,
        "dual_model:\n  enabled: yes\n  runtime_config_path: \"runtime.json\"\n"
        "  class_schema_path: \"schema.json\"\n");
    yolo::Config spelled;
    require(spelled.load(spelled_path.string()) && spelled.dual_model_enabled(), "YAML bool 'yes' was not read as true");
}

// Invalid dual-model contracts (reserved IDs missing or mapped into, non-audit conflict mode,
// unsealed bytes, wrong class count) refuse the load; a failed reload leaves the branch disabled.
void test_dual_model_invalid_contracts_fail_closed() {
    const std::filesystem::path base("/tmp/xbrain_dual_invalid");
    std::filesystem::create_directories(base);
    copy_bundled_sealed_contracts(base / "valid_runtime.json", base / "schema.json");

    write_runtime_contract(
        (base / "reserved.json").string(), "audit_only", "87");
    yolo::Config invalid_reserved;
    require(
        !invalid_reserved.load(write_dual_yaml(
            (base / "reserved.yaml").string(), "reserved.json", "schema.json")),
        "missing reserved ID 88 did not fail closed");

    write_runtime_contract(
        (base / "mapping.json").string(), "audit_only", "87, 88", 87);
    yolo::Config invalid_mapping;
    require(
        !invalid_mapping.load(write_dual_yaml(
            (base / "mapping.yaml").string(), "mapping.json", "schema.json")),
        "active mapping into reserved ID 87 did not fail closed");

    yolo::Config invalid_reload;
    require(
        invalid_reload.load(write_dual_yaml(
            (base / "reload_valid.yaml").string(), "valid_runtime.json", "schema.json")),
        "failed to establish enabled config before invalid reload");
    require(invalid_reload.dual_model_enabled(), "reload precondition was not enabled");
    require(
        !invalid_reload.load((base / "mapping.yaml").string()),
        "invalid reload unexpectedly succeeded");
    require(
        !invalid_reload.dual_model_enabled(),
        "invalid reload left stale dual-model branch enabled");

    write_runtime_contract(
        (base / "mode.json").string(), "prefer_active");
    yolo::Config invalid_mode;
    require(
        !invalid_mode.load(write_dual_yaml(
            (base / "mode.yaml").string(), "mode.json", "schema.json")),
        "non-audit conflict mode did not fail closed");

    // This generated contract has valid parsed semantics but different bytes
    // from the delivered sealed JSON. Hash pinning must still reject it.
    write_runtime_contract((base / "semantic_clone.json").string());
    yolo::Config invalid_hash;
    require(
        !invalid_hash.load(write_dual_yaml(
            (base / "hash.yaml").string(), "semantic_clone.json", "schema.json")),
        "semantically valid but non-sealed runtime bytes were accepted");

    write_class_schema((base / "wrong_count_schema.json").string(), 93);
    yolo::Config invalid_count;
    require(
        !invalid_count.load(write_dual_yaml(
            (base / "count.yaml").string(),
            "valid_runtime.json",
            "wrong_count_schema.json")),
        "wrong 94-class schema count did not fail closed");
}

// The sealed runtime JSON and class schema shipped in config/ load and give the expected active12
// mapping.
void test_bundled_sealed_contracts_load() {
    const std::filesystem::path root = source_root();
    const std::filesystem::path runtime =
        root / "config/dual_model_runtime_config.json";
    const std::filesystem::path schema = root / "config/class_schema_94.json";
    require(std::filesystem::is_regular_file(runtime), "bundled sealed runtime JSON missing");
    require(std::filesystem::is_regular_file(schema), "bundled class schema JSON missing");

    yolo::Config config;
    require(
        config.load(write_dual_yaml(
            "/tmp/xbrain_dual_bundled.yaml", runtime.string(), schema.string())),
        "bundled sealed dual-model contracts failed validation");
    require(config.dual_model_enabled(), "bundled sealed contract was not enabled");
    require(
        config.dual_model().local_to_global ==
            std::vector<int>(ACTIVE12_MAPPING.begin(), ACTIVE12_MAPPING.end()),
        "bundled sealed active12 mapping mismatch");
}

}  // namespace

// R1 (2026-10-04): a present value that does not convert refuses the load, naming the key; it
// used to become the built-in default silently. A missing key or an explicit null keeps the
// default, and a valid value is read exactly as before.
void test_mistyped_values_refuse_the_load() {
    const std::filesystem::path path = "/tmp/xbrain_config_mistyped.yaml";
    const std::vector<std::pair<std::string, std::string>> mistyped = {
        {"float", "depth:\n  jump_threshold_m: 0.2m\n"},
        {"int", "depth:\n  jump_confirmation_frames: six\n"},
        {"size_t", "queue:\n  result_queue_size: [1]\n"},
        {"bool", "inference:\n  use_fp16: maybe\n"},
        {"nested bool", "velocity:\n  camera_z_uncertainty_gate:\n    enabled: maybe\n"},
        {"nested int", "velocity:\n  gating:\n    min_stable_frames: ten\n"},
        {"string", "inference:\n  model_version: {a: 1}\n"},
        {"string vector", "velocity:\n  gating:\n    skip_fusion_quality: depth_unreliable\n"},
        {"string vector item", "inference:\n  model_classes: [person, {a: 1}]\n"},
        {"array length", "extrinsic_base_lidar:\n  translation: [0.0, 0.0]\n"},
        {"array item", "extrinsic_base_lidar:\n  rotation_xyzw: [0.0, x, 0.0, 1.0]\n"},
        {"per_class_conf", "inference:\n  per_class_conf:\n    person: high\n"},
    };
    for (const auto& [name, snippet] : mistyped) {
        write_inline_config(path, snippet + sealed_dual_model_yaml());
        yolo::Config config;
        require(!config.load(path.string()), ("mistyped " + name + " value was accepted").c_str());
    }

    write_inline_config(
        path,
        std::string("depth:\n  jump_threshold_m: null\n  cluster_width_m: 0.25\n") + sealed_dual_model_yaml());
    yolo::Config config;
    require(config.load(path.string()), "null or valid depth values were refused");
    require(config.depth_jump_threshold_m() == 0.20F, "explicit null did not keep the default");
    require(config.depth_cluster_width_m() == 0.25F, "valid depth value was not read");
    require(config.depth_jump_confirmation_frames() == 2, "missing key did not keep the default");
}

// INT8 build keys (2026-10-10): inference.int8_* for COCO80 and dual_model.int8_* for Active12.
// int8 false refuses any int8_* key; int8 true needs use_dla, the ranges file and its SHA-256, and
// the FP16 list goes with its SHA-256 (64 lowercase hex digits); paths are taken as given in both
// blocks (the formal service requires them absolute, see test_service_config).
void test_int8_build_keys() {
    const std::filesystem::path path = "/tmp/xbrain_config_int8.yaml";
    const std::string sha(64, 'a');
    const std::string upper_sha(64, 'A');
    {
        write_inline_config(path, sealed_dual_model_yaml());
        yolo::Config config;
        require(config.load(path.string()), "config without INT8 keys was refused");
        require(!config.use_int8() && !config.int8_build().enabled, "INT8 enabled by default");
        require(!config.dual_model().int8.enabled, "Active12 INT8 enabled by default");
    }
    const std::vector<std::pair<std::string, std::string>> refused = {
        {"key while int8 false", "inference:\n  int8_ranges: /x/r.cache\n"},
        {"sha while int8 false", "inference:\n  int8: false\n  int8_ranges_sha256: " + sha + "\n"},
        {"int8 without ranges", "inference:\n  int8: true\n"},
        {"ranges without sha", "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n"},
        {"short sha", "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n  int8_ranges_sha256: abc\n"},
        {"uppercase sha", "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n  int8_ranges_sha256: " + upper_sha + "\n"},
        {"list without sha", "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n  int8_ranges_sha256: " + sha +
                                 "\n  int8_fp16_layers: /x/l.txt\n"},
        {"list sha without list", "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n  int8_ranges_sha256: " + sha +
                                      "\n  int8_fp16_layers_sha256: " + sha + "\n"},
        {"short list sha", "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n  int8_ranges_sha256: " + sha +
                               "\n  int8_fp16_layers: /x/l.txt\n  int8_fp16_layers_sha256: abc\n"},
        {"int8 without the DLA", "inference:\n  use_dla: false\n  int8: true\n  int8_ranges: /x/r.cache\n"
                                     "  int8_ranges_sha256: " + sha + "\n"},
    };
    for (const auto& [name, snippet] : refused) {
        write_inline_config(path, snippet + sealed_dual_model_yaml());
        yolo::Config config;
        require(!config.load(path.string()), ("INT8 config accepted: " + name).c_str());
    }
    {
        write_inline_config(path, sealed_dual_model_yaml() + "  int8_ranges: r.cache\n");
        yolo::Config config;
        require(!config.load(path.string()), "dual_model int8_* key accepted while int8 false");
    }
    {
        write_inline_config(path, sealed_dual_model_yaml() + "  use_dla: false\n  int8: true\n  int8_ranges: /x/r.cache\n"
                                                             "  int8_ranges_sha256: " + sha + "\n");
        yolo::Config config;
        require(!config.load(path.string()), "dual_model int8 accepted without the DLA");
    }
    {
        write_inline_config(path, "inference:\n  int8: true\n  int8_ranges: /x/r.cache\n  int8_ranges_sha256: " + sha +
                                      "\n  int8_fp16_layers: rel/l.txt\n  int8_fp16_layers_sha256: " + sha + "\n" +
                                      sealed_dual_model_yaml() + "  int8: true\n  int8_ranges: a12/r.cache\n"
                                      "  int8_ranges_sha256: " + sha + "\n  int8_fp16_layers: a12/l.txt\n"
                                      "  int8_fp16_layers_sha256: " + sha + "\n");
        yolo::Config config;
        require(config.load(path.string()), "valid INT8 keys were refused");
        const auto& c = config.int8_build();
        require(config.use_int8() && c.enabled, "inference INT8 not enabled");
        require(c.ranges_path == "/x/r.cache" && c.ranges_sha256 == sha, "inference ranges not read as given");
        require(c.fp16_layers_path == "rel/l.txt" && c.fp16_layers_sha256 == sha, "inference FP16 list not as given");
        const auto& a = config.dual_model().int8;
        require(a.enabled && a.ranges_path == "a12/r.cache", "dual_model ranges not read as given");
        require(a.fp16_layers_path == "a12/l.txt" && a.fp16_layers_sha256 == sha, "dual_model FP16 list not as given");
    }
}

// dual_model.input_shape (2026-10-10): the deployed Active12 input. Only the sealed square and the
// sealed 896 x 608 window load; every other shape or form is refused with a message that names
// the key, and a missing or null key is the square.
void test_dual_model_input_shape() {
    const char* async_log = std::getenv("PERCEPTION_ASYNC_LOG");
    require(async_log == nullptr || std::string(async_log) != "1",
            "run with the synchronous logger: the refusal reason is read from std::cerr");
    const std::filesystem::path path = "/tmp/xbrain_config_input_shape.yaml";
    const std::vector<std::tuple<std::string, std::string, int>> accepted = {
        {"window, flow form", "  input_shape: [1, 3, 608, 896]\n", 608},
        {"window, block form", "  input_shape:\n    - 1\n    - 3\n    - 608\n    - 896\n", 608},
        {"square", "  input_shape: [1, 3, 896, 896]\n", 896},
        {"null", "  input_shape:\n", 896},
        {"tilde", "  input_shape: ~\n", 896},
    };
    for (const auto& [name, snippet, height] : accepted) {
        write_inline_config(path, sealed_dual_model_yaml() + snippet);
        yolo::Config config;
        require(config.load(path.string()), ("input_shape refused: " + name).c_str());
        require(config.dual_model().input_size == 896, ("input width is not 896: " + name).c_str());
        require(config.dual_model().input_height == height, ("input height not as configured: " + name).c_str());
    }
    const std::vector<std::pair<std::string, std::string>> refused = {
        {"896 x 640", "[1, 3, 640, 896]"},
        {"896 x 576", "[1, 3, 576, 896]"},
        {"height and width swapped", "[1, 3, 896, 608]"},
        {"608 x 608", "[1, 3, 608, 608]"},
        {"batch 2", "[2, 3, 608, 896]"},
        {"one channel", "[1, 1, 608, 896]"},
        {"three numbers", "[1, 3, 608]"},
        {"five numbers", "[1, 3, 608, 896, 1]"},
        {"height and width only", "[608, 896]"},
        {"empty list", "[]"},
        {"a number", "608"},
        {"a string", "\"608x896\""},
        {"a float", "[1, 3, 608.0, 896]"},
        {"a word", "[1, 3, tall, 896]"},
        {"a nested list", "[1, 3, [608], 896]"},
        {"a null item", "[1, 3, ~, 896]"},
        {"a word after the four numbers", "[1, 3, 608, 896, x]"},
        {"a null among the four numbers", "[1, ~, 3, 608, 896]"},
        {"a list among the four numbers", "[1, 3, [], 608, 896]"},
        {"a map", "{height: 608, width: 896}"},
    };
    for (const auto& [name, value] : refused) {
        write_inline_config(path, sealed_dual_model_yaml() + "  input_shape: " + value + "\n");
        yolo::Config config;
        std::string errors;
        require(!load_capturing_errors(config, path.string(), errors), ("input_shape accepted: " + name).c_str());
        require(errors.find("dual_model.input_shape must be [1,3,896,896] or [1,3,608,896]") != std::string::npos,
                ("input_shape refused for another reason: " + name + ": " + errors).c_str());
    }
}

int main() {
    test_env_expansion();
    test_unset_env_becomes_empty();
    test_default_pipeline_delay_is_zero();
    test_camera_intrinsics_and_per_class_confidence();
    test_d435_config_and_defaults();
    test_camera_driver_yaml_type_validation();
    test_orbbec_native800_tuple_validation();
    test_med2_loopback_sink_and_node_load();
    test_stability_config();
    test_generic_static_displacement_fallback_and_explicit_override();
    test_camera_z_uncertainty_gate_defaults_and_explicit_config();
    test_dual_model_defaults_and_sealed_contract();
    test_dual_model_required_and_refused_before_read();
    test_dual_model_invalid_contracts_fail_closed();
    test_bundled_sealed_contracts_load();
    test_mistyped_values_refuse_the_load();
    test_int8_build_keys();
    test_dual_model_input_shape();

    std::cout << "test_config_env passed" << std::endl;
    return 0;
}
