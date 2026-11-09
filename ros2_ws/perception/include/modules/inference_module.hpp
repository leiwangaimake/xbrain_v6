/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: inference_module.hpp
 * Brief: Dual YOLO11-seg TensorRT/DLA inference stage: captured frame in, merged detections out.
 *
 * Description:
 * What problem this solves. Every captured RGB frame has to pass two segmentation models: COCO80
 * (640x640, DLA0) and Active12 (896x896, FP16, DLA1; its class 92 is the traversable area that
 * feeds T). This stage owns both engines, their pinned host buffers and CUDA streams, the
 * letterbox preprocessing, the two postprocessors, the dual-model merge and the frame schedule,
 * and pushes one DetectionResult per frame to the Supervisor. The cadence design is 19 section 6;
 * its acceptance gates (publish intervals) are 19 section 6.3.
 *
 * Boundaries. No tracking, depth or geometry (Supervisor); no publishing; no engine build in the
 * service: the Supervisor asks for ExistingEnginesOnly, and engines are built on the machine by
 * the engine build tool (PSC-4 then checks them with a golden frame at startup).
 *
 * Pitfalls.
 *   - Two frame loops exist. The service runs infer_loop_jit (two_slot_jit = 1 in the configured
 *     perception.pipeline.runtime set): two host slots, one device pair in flight, the next
 *     submit overlapped with the current CPU postprocess. infer_loop is the serial alternative
 *     (two_slot_jit = 0).
 *   - Failure handling differs: the serial loop reloads both engines after
 *     MAX_CONSECUTIVE_FAILURES errors, the JIT loop stops on the first error (JIT_PROBE_FATAL,
 *     fail closed) and start() then refuses until the process restarts.
 *   - The PERCEPTION_* switches are read once in start(). The service sets them from the
 *     digested runtime section (service_config.cpp, switch_table) and clears the diagnostic ones;
 *     an inconsistent combination makes start() fail rather than run a half-valid mode.
 */

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common/mask_area_window.hpp"
#include "common/scene_luma_window.hpp"
#include "common/config.hpp"
#include "common/inference_source_binding.hpp"
#include "common/postprocess.hpp"
#include "common/postprocess_mask_ownership.hpp"
#include "common/preprocess.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/module_base.hpp"

namespace perception::rns { class SourceJournal; }
namespace yolo {

/**
 * @brief Runs YOLOv11n-seg inference for frames from the capture module.
 */
class InferenceModule : public ModuleBase {
public:
    enum class ModelLoadPolicy { AllowBuild, ExistingEnginesOnly };
    /** @brief Creates an inference module with input and output queues. */
    InferenceModule(
        const Config& config,
        std::shared_ptr<ThreadsafeQueue<FrameData>> input_queue,
        std::shared_ptr<ThreadsafeQueue<DetectionResult>> output_queue,
        perception::rns::SourceJournal* health_completions = nullptr,
        ModelLoadPolicy model_load_policy = ModelLoadPolicy::AllowBuild);

    /** @brief Stops the inference thread before destruction. */
    ~InferenceModule() override;

    /** @brief Loads the model and starts the inference worker thread. */
    bool start() override;

    /** @brief Requests inference shutdown and joins the worker thread. */
    void stop() override;

    // False/-1 until this run's JIT owner freezes and validates its delay.
    // Readiness is not a cadence or acceptance gate.
    bool jit_calibration_complete() const noexcept {
        return jit_calibration_complete_.load(std::memory_order_acquire);
    }
    std::int64_t jit_frozen_delay_us() const noexcept {
        return jit_frozen_delay_us_.load(std::memory_order_acquire);
    }

private:
    /** @brief Worker loop that pops frames and pushes detection results. */
    void infer_loop();
#ifdef HAS_TENSORRT
    void infer_loop_jit();
#endif

    /** @brief Loads or builds the TensorRT engine and prepares its execution context. */
    bool load_model();
    // The sealed active12 ModelSpec of dual_model.input_shape (square or 896 x 608 window).
    const ModelSpec& active12_spec() const;
    std::vector<float> preprocess_active12(const cv::Mat& image, LetterboxInfo& info) const;

#ifdef HAS_TENSORRT
    class ParallelTaskWorker;
    struct TensorRtModelContract;
    struct TensorRtModelState;

    bool initialize_tensor_rt_model(
        const TensorRtModelContract& contract,
        std::unique_ptr<TensorRtModelState>& state);
    bool build_serialized_engine(const TensorRtModelContract& contract);
    bool configure_tensor_buffers(TensorRtModelState& state);
    void ensure_pinned_input_buffers();
    void preprocess_model_inputs(
        const cv::Mat& image,
        LetterboxInfo& primary_info,
        LetterboxInfo& active12_info,
        ParallelTaskWorker* parallel_worker);
    void validate_tensor_rt_input(
        const TensorRtModelState& state,
        const float* tensor_data,
        std::size_t tensor_bytes) const;
    void submit_tensor_rt(
        TensorRtModelState& state,
        const float* tensor_data,
        std::size_t tensor_bytes);
    void copy_tensor_rt_outputs_and_wait(TensorRtModelState& state);
    void queue_tensor_rt_outputs(TensorRtModelState& state, float* detection, float* prototype);
    std::array<float, 4> tensor_rt_diagnostic_intervals(const TensorRtModelState& state) const;
    void drain_tensor_rt_no_throw(TensorRtModelState& state) noexcept;
    void execute_tensor_rt_pair(
        TensorRtModelState& primary,
        const float* primary_tensor_data,
        std::size_t primary_tensor_bytes,
        TensorRtModelState& active,
        const float* active_tensor_data,
        std::size_t active_tensor_bytes,
        int active_lead_us);
    std::vector<Detection> postprocess_primary(
        const TensorRtModelState& state,
        const LetterboxInfo& letterbox_info) const;
    std::vector<Detection> postprocess_active12(
        const TensorRtModelState& state,
        const LetterboxInfo& letterbox_info,
        std::uint64_t frame_id,
        Active12MaskProfile* deferred_profile = nullptr) const;
    void log_active12_mask_profile(const Active12MaskProfile& profile, std::uint64_t frame_id) const;
    void print_mask_summary(bool partial) const;
    void record_mask_areas(const std::vector<Detection>& coco, const std::vector<Detection>& active) const;
    void print_mask_areas(bool partial) const;
    void sample_scene_luma(const cv::Mat& frame, std::uint64_t frame_id);
    void print_scene_luma(bool partial);
    void release_tensor_rt_model(std::unique_ptr<TensorRtModelState>& state) noexcept;
    void release_tensor_rt() noexcept;
    void release_pinned_input_buffers() noexcept;
#endif

    const Config& config_;
    std::shared_ptr<ThreadsafeQueue<FrameData>> input_queue_;
    std::shared_ptr<ThreadsafeQueue<DetectionResult>> output_queue_;
    // Optional single-producer telemetry. Caller owns lifetime through stop().
    // A telemetry fault does not turn a failed inference into a completion.
    perception::rns::SourceJournal* health_completions_;
    const ModelLoadPolicy model_load_policy_;
    std::thread worker_;
    // Runtime switches, read once in start() (see the file header). schedule_probe_enabled_ /
    // schedule_probe_jit_: PERCEPTION_TWO_SLOT_JIT present / equal to 1; the jit_* flags are the
    // PERCEPTION_JIT_* switches of the JIT loop.
    bool schedule_probe_enabled_ = false;
    bool schedule_probe_jit_ = false;
    bool jit_async_submit_ = false;
    bool jit_fresh_bind_ = false;
    bool jit_second_chance_ = false;
    bool jit_empty_acquire_ = false;
    bool jit_empty_wait_ready_ = false;
    bool jit_frame_move_ = false;
    bool jit_stall_trace_ = false;  // diagnostic: caller-thread CPU around post/merge/push in JIT_FRAME
    // Minimum interval between two device submissions of the JIT loop in ms
    // (PERCEPTION_INFER_MIN_INTERVAL_MS, see jit_schedule.hpp); 0 = off, back to back.
    int infer_min_interval_ms_ = 0;
    // Diagnostic: per-instance ACTIVE_MASK_TASK and per-frame ACTIVE_MASK_PROFILE lines
    // (PERCEPTION_ACTIVE12_MASK_LOG=1; the service always unsets it). They were about three
    // quarters of the production log bytes; production keeps ACTIVE_MASK_SUMMARY instead.
    bool active12_mask_log_ = false;
    // Padding-anchor guard of the Active12 decoder (PERCEPTION_ACTIVE12_PADDING_ANCHOR_GUARD=1,
    // see Active12PaddingGuard in postprocess.hpp) and what it dropped since start(): candidates
    // and the frames that had at least one. Printed in ACTIVE_MASK_SUMMARY and once by stop().
    // Mutable like mask_summary_: the Active12 post is a const member.
    bool active12_padding_guard_ = false;
    mutable std::atomic<std::uint64_t> active12_padding_dropped_{0}, active12_padding_frames_{0};
    // Accumulated Active12 mask profile, printed as one ACTIVE_MASK_SUMMARY line every
    // kMaskSummaryFrames profiled frames and once more (partial=1) by stop(). Written by the thread
    // that runs the Active12 post (the inference caller, or the post helper when post_role_swap=1),
    // one post at a time: ParallelTaskWorker::run joins before the next frame, start() resets it
    // before the worker starts and stop() prints it after the join, so no lock.
    struct MaskProfileSummary {
        std::uint64_t frames = 0, instances = 0, bbox_pixels = 0;
        std::int64_t task_ns = 0, gemm_ns = 0, resize_ns = 0, threshold_ns = 0, batch_wall_ns = 0, max_task_ns = 0;
    };
    mutable MaskProfileSummary mask_summary_;
    // Acceptance-list mask/ROI area record (Perception完整验收清单, section 共载与场景, anchor
    // "mask/ROI面积"; user decision 2026-09-30, B9-17 (b)):
    // ROI (box) area and mask foreground per instance and branch, class 92 (T's input) apart;
    // one MASK_AREA_SUMMARY line per 300 profiled frames and a partial one at stop(). Written by
    // the thread that merges the two branches (infer loop or JIT caller), one per run; no lock.
    mutable perception::MaskAreaWindow mask_area_;
    // Scene brightness (2026-10-16, numbers only, see scene_luma_window.hpp): one grid sample a
    // second of the colour frame being prepared, one SCENE_LUMA line per 60 samples and a partial
    // one at stop(), with the inference profile on. Written from the JIT loop's prepare step, which
    // runs on the JIT caller or, for an asynchronous acquire, on its submit thread while the caller
    // waits for it (a serial hand-over, joined before the caller goes on); read at stop() after the
    // loop has ended. No lock.
    perception::SceneLumaWindow scene_luma_;
    std::string post_dump_dir_;
    int post_dump_remaining_ = 0;
    int jit_probe_log_ = 2;  // 2 full PROBE_FRAME/PROBE_LOAD, 1 compact JIT_FRAME, 0 none
    bool execution_cpu_trace_ = false;
    bool post_role_swap_ = false;
    Active12MaskImplementation active12_mask_implementation_ = Active12MaskImplementation::Serial;
    Active12MaskBinaryImplementation active12_mask_binary_implementation_ = Active12MaskBinaryImplementation::Legacy;
    perception::MaskOutputCopyMode mask_output_copy_mode_ = perception::MaskOutputCopyMode::Clone;
    std::unique_ptr<perception::PersistentTaskPair> active12_mask_lanes_;
    double jit_fixed_bind_delay_ms_ = -1.0;
    std::atomic<bool> schedule_probe_failed_{false};
    std::atomic<bool> jit_calibration_complete_{false};
    std::atomic<std::int64_t> jit_frozen_delay_us_{-1};
#ifdef HAS_TENSORRT
    std::unique_ptr<TensorRtModelState> primary_model_;
    std::unique_ptr<TensorRtModelState> active12_model_;
    float* primary_input_host_;
    std::size_t primary_input_host_bytes_;
    float* active12_input_host_;
    std::size_t active12_input_host_bytes_;
    int active_lead_us_;
#endif
    bool dual_model_enabled_;
    // Serial loop only: errors in a row before both engines are reloaded.
    int consecutive_failures_;
    static constexpr int MAX_CONSECUTIVE_FAILURES = 3;

    // One prepared frame: both input tensors (CPU path) and everything the result carries over
    // from the capture. The serial loop keeps one (filled only after the current result is
    // published); each JIT slot holds its own.
    struct PreparedInput {
        std::vector<float> tensor_data;
        std::vector<float> active12_tensor_data;
        cv::Mat frame;
        cv::Mat depth;
        DepthIntrinsics depth_intrinsics;
        std::shared_ptr<const perception::RgbDepthRegistration> depth_registration;
        float depth_scale = 0.0F;
        CameraFrameMetadata camera_metadata;
        LetterboxInfo letterbox_info;
        LetterboxInfo active12_letterbox_info;
        uint64_t frame_id = 0;
        double stamp_sec = 0.0;
        double capture_time_ms = 0.0;
        PipelineTiming pipeline_timing;
        perception::InferenceSourceBinding source_binding;
        bool valid = false;
    };
    PreparedInput next_input_;
};

}  // namespace yolo
