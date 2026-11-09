/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: config.hpp
 * Brief: YAML configuration loader for detector runtime parameters.
 *
 * Description:
 *   1. Loads default.yaml and exposes typed getters with defaults.
 *   2. Centralizes RTSP, inference, queue, and logging settings.
 * R6: the service builds this document from perception.pipeline (minus runtime/media/ros, plus a
 * synthesized hevc block, see service_config.cpp) and calls load_from_node(); no default.yaml.
 * Strict parts: the Gemini camera and hevc blocks and the sealed dual-model contract refuse bad
 * input. Everything else is optional with a built-in default when missing or null, but a present
 * value of the wrong type refuses the load (R1, 2026-10-04). Getters marked "no R6 reader" below
 * are read but nothing in the delivered service uses them (V5 leftovers).
 */

#pragma once

#include <cstddef>
#include <array>
#include <map>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace yolo {

/** @brief Lowercase SHA-256 hex digest of exactly these bytes. */
std::string sha256_hex(const std::string& bytes);

/**
 * @brief Reads and serves detector configuration from a YAML file.
 *
 * Getter methods return built-in defaults when optional keys are missing, so
 * early development can proceed with partial configuration files.
 */
class Config {
public:
    // V5 config-driven intrinsics (1920x1080 defaults); no R6 reader. R6 takes the intrinsics
    // the Orbbec SDK reports for the running stream.
    struct CameraIntrinsics {
        float fx = 1393.5F;
        float fy = 1393.5F;
        float cx = 960.0F;
        float cy = 540.0F;
        int width_px = 1920;
        int height_px = 1080;
        std::string calibration_date = "2026-XX-XX";
        float reprojection_rms_px = 1.0F;
    };

    // Sealed runtime JSON section, validated but not read at run time since the
    // safe traversable mask left the pipeline (level-3 trim); see read_dual_model_config.
    struct SafeTraversableConfig {
        int traversable_global_id = 92;
        int pit_global_id = 93;
        std::vector<int> active_obstacle_global_ids;
        std::vector<int> coco_navigation_obstacle_ids;
        int erosion_pixels = 5;
        std::string erosion_kernel;
    };

    /**
     * @brief INT8 engine build inputs of one model (2026-10-10; see modules/int8_plan.hpp).
     *
     * int8 true needs use_dla and the ranges file (a TensorRT 10.3 calibration cache) with its
     * SHA-256; the FP16 layer list is optional, its SHA-256 required when it is given. int8
     * false refuses any of these keys, so a configured file is never silently unused. Paths are
     * taken as given (the formal service requires them absolute, PSC-2). Only the engine build
     * reads the files; a running service loads the built engine.
     */
    struct Int8BuildConfig {
        bool enabled = false;
        std::string ranges_path;
        std::string ranges_sha256;
        std::string fp16_layers_path;
        std::string fp16_layers_sha256;
    };

    /**
     * @brief Active12 deployment settings plus the validated sealed merge contract.
     *
     * The YAML file controls only deployment/hardware choices. Thresholds, class
     * mappings, conflict policy, tracker exclusions, and traversable-area policy
     * are populated from the sealed runtime JSON. dual_model.enabled is required
     * and must be true, so a loaded Config always has the dual model.
     * dual_model.input_shape (optional, 2026-10-10) chooses between the two sealed inputs:
     * [1,3,896,896] (default) or [1,3,608,896]. Any other value refuses the load.
     */
    struct DualModelConfig {
        bool enabled = false;  // false only before a successful load or after a failed one
        std::string runtime_config_path;
        // SHA-256 of the exact bytes that were parsed (one read; B8-1).
        std::string runtime_config_sha256;
        std::string class_schema_path;
        std::string class_schema_sha256;
        std::string active_model_path;
        std::string active_engine_path;
        int input_size = 896;  // side of the sealed square input (the input width)
        // Rows of the deployed input: input_size, or 608 for the sealed 896 x 608 window of the
        // square letterbox (dual_model.input_shape; see active12_seg_model_spec).
        int input_height = 896;
        float conf_threshold = 0.25F;
        float nms_iou_threshold = 0.70F;
        float mask_threshold = 0.50F;
        int max_detections = 300;
        bool use_dla = true;
        int dla_core = 1;
        bool use_fp16 = true;
        bool allow_gpu_fallback = true;
        int workspace_mb = 2048;
        Int8BuildConfig int8;  // dual_model.int8 and int8_* keys, paths as given (absolute in the service)
        std::vector<std::string> local_class_names;
        std::vector<int> local_to_global;
        std::vector<int> reserved_ids;
        std::vector<std::string> global_class_names;
        std::string conflict_mode;
        float conflict_iou = 0.30F;
        std::vector<int> coco_vehicle_ids;
        int heavy_id = 89;
        std::vector<int> tracker_excluded_ids;
        SafeTraversableConfig safe_traversable;
    };

    /** @brief Loads a YAML config file from disk. */
    bool load(const std::string& path);

    /**
     * @brief Loads an already parsed YAML document (deep-copied).
     *
     * origin_path is reported by path() and is the base for any relative model
     * path, exactly as load(path) would use the file path. Same validation and
     * failure cleanup as load(); no file is read here.
     */
    bool load_from_node(const YAML::Node& root, const std::string& origin_path);

    /** @brief Returns the path of the loaded YAML config file. */
    std::string path() const;

    /** @brief Returns the configured RTSP URL. */
    // No R6 reader (V5 RTSP input; the rtsp section is retired in R6).
    std::string rtsp_url() const;

    /** @brief Returns the exact RGB-D driver selected by camera.driver. */
    std::string camera_driver() const;

    /** @brief Returns the exact device serial required by the selected RGB-D camera. */
    std::string camera_serial() const;
    int camera_color_width() const;
    int camera_color_height() const;
    int camera_color_fps() const;
    std::string camera_color_device_format() const;
    int camera_depth_width() const;
    int camera_depth_height() const;
    int camera_depth_fps() const;
    std::string camera_depth_device_format() const;
    std::string camera_alignment_mode() const;
    bool camera_align_to_color() const;
    int camera_output_width() const;
    int camera_output_height() const;
    int camera_output_fps() const;

    /** @brief Returns whether direct Intel RealSense D435 capture is enabled. */
    // The d435_* getters have no R6 reader: the D435 driver is not built or delivered.
    bool d435_enabled() const;

    int d435_color_width() const;
    int d435_color_height() const;
    int d435_color_fps() const;
    int d435_depth_width() const;
    int d435_depth_height() const;
    int d435_depth_fps() const;
    bool d435_align_to_color() const;
    int d435_power_line_frequency() const;

    /** @brief Returns whether the D435 HEVC output branch is enabled. */
    // (R6: the MED-2 preview encoder; the service synthesizes the hevc block as med2_loopback.)
    bool hevc_enabled() const;
    std::string hevc_sink() const;
    /** @brief True for the shared-frame streaming encoder path (stream_rtsp or med2_loopback). */
    bool hevc_stream_sink() const;
    std::string hevc_rtsp_listen() const;
    std::string hevc_file_path() const;
    int hevc_width() const;
    int hevc_height() const;
    int hevc_fps() const;
    int hevc_bitrate() const;
    std::string hevc_codec() const;
    int hevc_gop() const;

    /** @brief Returns the ONNX model path used when building a TensorRT engine. */
    std::string model_path() const;

    /** @brief Overrides the cached ONNX model path after config loading. */
    void set_model_path(const std::string& path);

    /** @brief Returns whether TensorRT should target DLA. */
    bool use_dla() const;

    /** @brief Returns the configured DLA core index. */
    int dla_core() const;

    /** @brief Returns the serialized TensorRT engine path. */
    std::string trt_engine_path() const;

    /** @brief Overrides the cached TensorRT engine path after config loading. */
    void set_trt_engine_path(const std::string& path);

    /** @brief Returns the TensorRT build workspace limit in MiB. */
    int trt_workspace_mb() const;

    /** @brief Returns whether INT8 engine building is requested. */
    bool use_int8() const;

    /** @brief Returns the primary (COCO80) model's INT8 build inputs (inference.int8_* keys). */
    const Int8BuildConfig& int8_build() const;

    /** @brief Returns the confidence threshold for detections. */
    float conf_threshold() const;

    /** @brief Returns class-specific confidence thresholds keyed by COCO class id. */
    const std::map<int, float>& per_class_confidence_thresholds() const;

    /** @brief Returns the disabled-by-default generic weak-candidate confidence floor. */
    float low_confidence_floor() const;

    /** @brief Returns the maximum weak candidates retained after overlap suppression. */
    int low_confidence_max_candidates() const;

    /** @brief Returns configured model version metadata. */
    // No R6 reader (V5 status metadata).
    std::string model_version() const;

    /** @brief Returns configured class labels, or COCO defaults when absent. */
    // No R6 reader: class names come from the code (COCO80) and the sealed class schema.
    std::vector<std::string> model_classes() const;

    /** @brief Returns the IoU threshold used by NMS. */
    float iou_threshold() const;

    /** @brief Returns the threshold used to binarize segmentation masks. */
    float mask_threshold() const;

    /** @brief Returns the square network input size. */
    int input_size() const;

    /** @brief Returns whether FP16 inference is enabled (O1 optimization). */
    bool use_fp16() const;

    /** @brief Returns inference queue pop and execution timeout in ms. */
    int timeout_ms() const;

    /** @brief Returns true once a load passed (the active12 branch is required and sealed-validated). */
    bool dual_model_enabled() const;

    /** @brief Returns the validated dual-model deployment and merge contract. */
    const DualModelConfig& dual_model() const;

    /** @brief Returns the maximum number of queued frames. */
    std::size_t frame_queue_size() const;

    /** @brief Returns the maximum number of queued detection results. */
    std::size_t result_queue_size() const;

    /** @brief Returns the configured log level string. */
    std::string log_level() const;

    // No R6 reader for these two (V5).
    CameraIntrinsics camera_intrinsics() const;
    int camera_pipeline_delay_ms() const;

    float track_threshold() const;
    int track_buffer_frames() const;
    float match_threshold() const;
    int track_min_hits() const;
    int track_low_confidence_min_hits() const;
    int track_max_age() const;
    const std::map<std::string, int>& track_per_class_max_ages() const;
    float track_bbox_ema_alpha() const;
    const std::map<std::string, float>& track_per_class_bbox_ema_alphas() const;

    std::string depth_sample_mode() const;
    const std::map<std::string, std::string>& depth_per_class_sample_modes() const;
    float depth_percentile() const;
    float depth_cluster_width_m() const;
    float depth_cluster_min_fraction() const;
    int depth_cluster_min_pixels() const;
    float depth_nearest_cluster_output_percentile() const;
    float depth_jump_threshold_m() const;
    float depth_jump_candidate_tolerance_m() const;
    int depth_jump_confirmation_frames() const;

    int capture_frame_buffer_size() const;
    int evidence_quota_mb() const;

    float velocity_dt_max_s() const;
    float velocity_centroid_ema_alpha() const;
    const std::map<std::string, float>& velocity_per_class_centroid_ema_alphas() const;
    float velocity_window_s() const;
    int velocity_motion_confirmation_windows() const;
    float velocity_static_displacement_m() const;
    float velocity_generic_static_displacement_m() const;
    const std::map<std::string, float>& velocity_per_class_static_displacements_m() const;
    int velocity_gating_min_stable_frames() const;
    float velocity_gating_min_depth_confidence() const;
    std::vector<std::string> velocity_gating_skip_fusion_quality() const;
    bool velocity_camera_z_uncertainty_gate_enabled() const;
    float velocity_camera_z_uncertainty_stddev_multiplier() const;
    float velocity_camera_z_uncertainty_max_threshold_m() const;

    float odom_stale_ms() const;
    // R6 refuses the extrinsic_base_lidar section (retired), so these are always the defaults
    // (zero translation, identity rotation); see Supervisor::init.
    std::array<float, 3> extrinsic_base_lidar_translation() const;
    std::array<float, 4> extrinsic_base_lidar_rotation_xyzw() const;

private:
    struct CachedValues {
        std::string rtsp_url;
        std::string hevc_sink = "file_rtsp";
        std::string hevc_rtsp_listen = "rtsp://0.0.0.0:8555/rgb";
        std::string hevc_file_path = "data/p4_rgb.mp4";
        std::string hevc_codec = "nvv4l2h265enc";
        std::string model_path;
        std::string trt_engine_path;
        std::string model_version = "unknown";
        std::string log_level;
        std::string camera_driver = "d435";
        std::string camera_serial;
        std::string camera_color_device_format;
        std::string camera_depth_device_format;
        std::string camera_alignment_mode;
        bool use_fp16 = false;
        bool use_dla = true;
        bool use_int8 = false;
        Int8BuildConfig int8_build;
        bool d435_enable = true;
        bool d435_align_to_color = true;
        bool hevc_enable = true;
        int camera_color_width = 0;
        int camera_color_height = 0;
        int camera_color_fps = 0;
        int camera_depth_width = 0;
        int camera_depth_height = 0;
        int camera_depth_fps = 0;
        bool camera_align_to_color = true;
        int camera_output_width = 0;
        int camera_output_height = 0;
        int camera_output_fps = 0;
        int d435_color_width = 1920;
        int d435_color_height = 1080;
        int d435_color_fps = 30;
        int d435_depth_width = 1280;
        int d435_depth_height = 720;
        int d435_depth_fps = 30;
        int d435_power_line_frequency = -1;
        int hevc_width = 1920;
        int hevc_height = 1080;
        int hevc_fps = 30;
        int hevc_bitrate = 4000000;
        int hevc_gop = 30;
        int input_size = 640;
        int dla_core = 0;
        int trt_workspace_mb = 1024;
        int timeout_ms = 500;
        int camera_pipeline_delay_ms = 0;
        int track_buffer_frames = 30;
        int track_min_hits = 3;
        int track_low_confidence_min_hits = 5;
        int track_max_age = 5;
        std::map<std::string, int> track_per_class_max_ages;
        float conf_threshold = 0.25F;
        float low_confidence_floor = 0.0F;
        int low_confidence_max_candidates = 8;
        float iou_threshold = 0.45F;
        float mask_threshold = 0.5F;
        float track_threshold = 0.5F;
        float match_threshold = 0.8F;
        float track_bbox_ema_alpha = 0.4F;
        std::map<std::string, float> track_per_class_bbox_ema_alphas;
        std::string depth_sample_mode = "nearest_cluster";
        std::map<std::string, std::string> depth_per_class_sample_modes;
        float depth_percentile = 0.20F;
        float depth_cluster_width_m = 0.12F;
        float depth_cluster_min_fraction = 0.08F;
        int depth_cluster_min_pixels = 8;
        float depth_nearest_cluster_output_percentile = 0.10F;
        float depth_jump_threshold_m = 0.20F;
        float depth_jump_candidate_tolerance_m = 0.12F;
        int depth_jump_confirmation_frames = 2;
        // Capture command (T11): frame ring size and evidence-dir quota in MB.
        int capture_frame_buffer_size = 20;
        int evidence_quota_mb = 1800;
        float odom_stale_ms = 200.0F;
        float velocity_dt_max_s = 0.5F;
        float velocity_centroid_ema_alpha = 0.35F;
        std::map<std::string, float> velocity_per_class_centroid_ema_alphas;
        float velocity_window_s = 0.25F;
        int velocity_motion_confirmation_windows = 1;
        float velocity_static_displacement_m = 0.04F;
        float velocity_generic_static_displacement_m = 0.04F;
        std::map<std::string, float> velocity_per_class_static_displacements_m;
        int velocity_gating_min_stable_frames = 10;
        float velocity_gating_min_depth_confidence = 0.5F;
        std::vector<std::string> velocity_gating_skip_fusion_quality = {"depth_unreliable"};
        bool velocity_camera_z_uncertainty_gate_enabled = false;
        float velocity_camera_z_uncertainty_stddev_multiplier = 1.0F;
        float velocity_camera_z_uncertainty_max_threshold_m = 0.06F;
        std::array<float, 3> extrinsic_base_lidar_translation = {0.0F, 0.0F, 0.0F};
        std::array<float, 4> extrinsic_base_lidar_rotation_xyzw = {0.0F, 0.0F, 0.0F, 1.0F};
        CameraIntrinsics camera_intrinsics;
        std::map<int, float> per_class_conf;
        std::vector<std::string> model_classes;
        DualModelConfig dual_model;
        std::size_t frame_queue_size = 5;
        std::size_t result_queue_size = 10;
    };

    void cache_values();
    bool load_failed(const std::string& path, const char* what);

    std::string path_;
    YAML::Node node_;
    CachedValues cached_;
};

}  // namespace yolo
