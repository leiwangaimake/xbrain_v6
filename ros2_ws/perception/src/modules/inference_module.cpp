/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: inference_module.cpp
 * Brief: YOLOv11n-seg TensorRT/DLA inference module implementation.
 *
 * Description:
 * Implements inference_module.hpp; read its Pitfalls first. The production path, in order:
 *   start()          read and cross-check every runtime switch, load both engines (never build),
 *                    prewarm the Active12 mask kernels, start infer_loop_jit;
 *   calibration      120 warm-up plus 300 measured frames, strictly serial (prepare, submit, wait,
 *                    publish); these frames are published like any other;
 *   steady loop      submit the current pair; sleep until submit + D; take the next frame without
 *                    waiting; wait for the current pair; publish it (both postprocessors in
 *                    parallel, merge, push) while the submit lane already submits the next pair.
 * D is the configured fixed delay (jit_bind_delay_ms) or the measured
 * max(0, pair median - prepare p99 - 2 ms): a frame taken at submit + D and prepared is ready
 * about when the device pair finishes, so the device stays busy and the frame is as fresh as
 * possible. Only one device pair is ever in flight; the two host slots only let the CPU work on
 * one frame while the device runs the other. With a minimum submit interval (infer_min_interval_ms,
 * off by default, async submit only) every submit first waits until that long after the previous
 * one; the frame it submits is the one already prepared. Each ms of the cap above the pair time
 * removes one ms of host-jitter margin under the 50 ms line, and the push of a frame also waits
 * for the next frame's wait where that is longer than the post (jit_schedule.hpp).
 *
 * Without HAS_TENSORRT the module builds but returns no detections (host tests).
 */

#include "common/thread_name.hpp"
#include "common/post_dump.hpp"
#include "modules/inference_module.hpp"
#include "modules/int8_plan.hpp"
#include "output/rns_source_hooks.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#ifdef __linux__
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "common/logger.hpp"
#include "common/jit_schedule.hpp"
#include "common/jit_second_chance.hpp"
#include "common/persistent_task_pair.hpp"
#include "common/parallel_dispatch_trace.hpp"
#include "common/post_role_dispatch.hpp"
#include "common/dual_model_merge.hpp"
#include "common/postprocess.hpp"
#include "common/preprocess.hpp"
#include "common/time_utils.hpp"
#include "common/timer.hpp"

#ifdef HAS_TENSORRT
#include <NvOnnxParser.h>
#endif

namespace yolo {

namespace {
const char* active12_mask_implementation_name(Active12MaskImplementation implementation) {
    switch (implementation) {
        case Active12MaskImplementation::Serial: return "serial";
        case Active12MaskImplementation::Parallel2: return "parallel2";
        case Active12MaskImplementation::Shadow: return "shadow";
    }
    return "invalid";
}
}  // namespace

#ifdef HAS_TENSORRT
namespace {

// Sealed tensor contracts: input [1,3,S,S], detections [1,C,N] channel-major, 32 prototype
// masks of P x P. load_model() refuses a configuration that disagrees.
constexpr int PRIMARY_INPUT_SIZE = 640;
constexpr int PRIMARY_DETECTION_CHANNELS = 116;
constexpr int PRIMARY_DETECTION_CANDIDATES = 8400;
constexpr int MASK_CHANNELS = 32;
constexpr int PRIMARY_MASK_SIZE = 160;
constexpr int ACTIVE12_INPUT_SIZE = 896;
constexpr int ACTIVE12_DETECTION_CHANNELS = 48;
constexpr int ACTIVE12_DETECTION_CANDIDATES = 16464;
constexpr int ACTIVE12_MASK_SIZE = 224;

// PROBE_LOAD fields (jit_probe_log = full only): mask raster sizes and foreground per frame.
std::string diagnostic_load_fields(const std::vector<Detection>& detections) {
    std::uint64_t roi_pixels = 0;
    std::uint64_t mask_nonzero = 0;
    std::uint64_t fullmask_pixels = 0;
    std::uint64_t class92_count = 0;
    for (const auto& detection : detections) {
        roi_pixels += detection.mask.total();
        if (!detection.mask.empty()) {
            mask_nonzero += static_cast<std::uint64_t>(cv::countNonZero(detection.mask));
        }
        fullmask_pixels += detection.full_image_mask.total();
        class92_count += detection.class_id == 92 ? 1U : 0U;
    }
    std::ostringstream output;
    output << detections.size() << " roi_pixels=" << roi_pixels
           << " mask_nonzero=" << mask_nonzero
           << " fullmask_pixels=" << fullmask_pixels
           << " class92_count=" << class92_count;
    return output.str();
}

class TensorRtLogger final : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* message) noexcept override {
        if (message == nullptr || severity == Severity::kVERBOSE) {
            return;
        }
        if (severity == Severity::kINTERNAL_ERROR || severity == Severity::kERROR) {
            LOG_ERROR_M("TensorRT", message);
        } else if (severity == Severity::kWARNING) {
            LOG_WARN_M("TensorRT", message);
        } else {
            LOG_INFO_M("TensorRT", message);
        }
    }
};

TensorRtLogger TRT_LOGGER;

// TensorRT before 10 frees its objects with destroy(), 10 with delete.
template <typename T>
void destroy_tensor_rt_object(T*& object) noexcept {
    if (object == nullptr) {
        return;
    }
#if NV_TENSORRT_MAJOR < 10
    object->destroy();
#else
    delete object;
#endif
    object = nullptr;
}

template <typename T>
struct TensorRtDeleter {
    void operator()(T* object) const noexcept {
        destroy_tensor_rt_object(object);
    }
};

template <typename T>
using TensorRtUniquePtr = std::unique_ptr<T, TensorRtDeleter<T>>;

void check_cuda(cudaError_t status, const char* operation) {
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(operation) + " failed: " + cudaGetErrorString(status));
    }
}

std::string dims_to_string(const nvinfer1::Dims& dims) {
    std::ostringstream out;
    out << "[";
    for (int i = 0; i < dims.nbDims; ++i) {
        if (i > 0) {
            out << ",";
        }
        out << dims.d[i];
    }
    out << "]";
    return out.str();
}

std::string data_type_to_string(nvinfer1::DataType type) {
    switch (type) {
        case nvinfer1::DataType::kFLOAT:
            return "FP32";
        case nvinfer1::DataType::kHALF:
            return "FP16";
        case nvinfer1::DataType::kINT8:
            return "INT8";
        case nvinfer1::DataType::kINT32:
            return "INT32";
        case nvinfer1::DataType::kBOOL:
            return "BOOL";
        default:
            return "OTHER";
    }
}

// INT8 build plan (modules/int8_plan.hpp, 2026-10-10): explicit dynamic ranges from the
// calibration cache for the network input and every floating-point layer output, the listed
// layers kept in FP16 (compute and non-network outputs, no range), precision constraints
// preferred. Any gap (file identity, unknown layer name, tensor without a scale, rejected range)
// throws, so the engine is never built with a silent FP16 fallback.
void apply_int8_plan(
    nvinfer1::INetworkDefinition& network,
    nvinfer1::IBuilderConfig& builder_config,
    const Config::Int8BuildConfig& inputs,
    const std::string& label,
    const std::string& module_name) {
    const Int8Plan plan = load_int8_plan(
        inputs.ranges_path, inputs.ranges_sha256, inputs.fp16_layers_path, inputs.fp16_layers_sha256);
    const auto is_float = [](nvinfer1::DataType type) {
        return type == nvinfer1::DataType::kFLOAT || type == nvinfer1::DataType::kHALF;
    };
    std::unordered_map<std::string, nvinfer1::ITensor*> tensors;
    std::vector<std::string> float_inputs;
    for (int i = 0; i < network.getNbInputs(); ++i) {
        nvinfer1::ITensor* tensor = network.getInput(i);
        if (tensor == nullptr || !is_float(tensor->getType())) continue;
        float_inputs.emplace_back(tensor->getName());
        tensors[tensor->getName()] = tensor;
    }
    std::vector<Int8LayerView> views(static_cast<std::size_t>(network.getNbLayers()));
    for (int l = 0; l < network.getNbLayers(); ++l) {
        nvinfer1::ILayer* layer = network.getLayer(l);
        auto& view = views[static_cast<std::size_t>(l)];
        view.name = layer->getName();
        for (int o = 0; o < layer->getNbOutputs(); ++o) {
            nvinfer1::ITensor* tensor = layer->getOutput(o);
            if (tensor == nullptr || !is_float(tensor->getType())) continue;
            view.float_outputs.push_back({tensor->getName(), tensor->isNetworkOutput()});
            tensors[tensor->getName()] = tensor;
        }
    }
    const Int8Assignment assignment = plan_int8_assignment(float_inputs, views, plan);
    // Implicit quantization (explicit dynamic ranges) is deprecated in TensorRT 10, but it is the
    // DLA INT8 path verified on Orin (2026-10-09): there is no Q/DQ in the ONNX, and a DLA network
    // accepts only an Entropy2 calibrator, which lowered the COCO80 scores.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    for (const auto& [name, range] : assignment.ranges) {
        if (!tensors.at(name)->setDynamicRange(-range, range)) {
            throw std::runtime_error("INT8 plan refused: TensorRT rejected the range of tensor \"" + name +
                                     "\" for model=" + label);
        }
    }
#pragma GCC diagnostic pop
    const std::set<std::string> half_outputs(assignment.half_outputs.begin(), assignment.half_outputs.end());
    for (const std::size_t index : assignment.fp16_layers) {
        nvinfer1::ILayer* layer = network.getLayer(static_cast<int>(index));
        layer->setPrecision(nvinfer1::DataType::kHALF);
        for (int o = 0; o < layer->getNbOutputs(); ++o) {
            nvinfer1::ITensor* tensor = layer->getOutput(o);
            if (tensor != nullptr && half_outputs.count(tensor->getName()) != 0U) {
                layer->setOutputType(o, nvinfer1::DataType::kHALF);
            }
        }
    }
    if (!assignment.fp16_layers.empty()) {
        builder_config.setFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS);
    }
    LOG_INFO_M(
        module_name,
        "INT8_PLAN model=" << label << " cache_header=" << plan.cache.header
                           << " ranges_sha256=" << plan.ranges_sha256
                           << " cache_entries=" << plan.cache.scales.size()
                           << " ranged=" << assignment.ranges.size()
                           << " fp16_layers=" << assignment.fp16_layers.size()
                           << " unranged_outputs=" << assignment.unranged_outputs
                           << " unused_cache_entries=" << assignment.unused_cache_entries
                           << " fp16_list_sha256=" << (plan.fp16_layers_sha256.empty() ? "none" : plan.fp16_layers_sha256));
}

bool has_dynamic_dimension(const nvinfer1::Dims& dims) {
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) {
            return true;
        }
    }
    return false;
}

std::vector<int64_t> shape_vector(const nvinfer1::Dims& dims) {
    std::vector<int64_t> shape;
    shape.reserve(static_cast<std::size_t>(dims.nbDims));
    for (int i = 0; i < dims.nbDims; ++i) {
        shape.push_back(dims.d[i]);
    }
    return shape;
}

bool shape_equals(const nvinfer1::Dims& dims, const std::vector<int64_t>& expected) {
    if (dims.nbDims != static_cast<int>(expected.size())) {
        return false;
    }
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] != expected[static_cast<std::size_t>(i)]) {
            return false;
        }
    }
    return true;
}

std::size_t element_count(const nvinfer1::Dims& dims) {
    std::size_t count = 1U;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) {
            throw std::runtime_error("unresolved TensorRT tensor shape " + dims_to_string(dims));
        }
        const std::size_t extent = static_cast<std::size_t>(dims.d[i]);
        if (count > std::numeric_limits<std::size_t>::max() / extent) {
            throw std::runtime_error("TensorRT tensor element count overflow");
        }
        count *= extent;
    }
    return count;
}

std::vector<char> read_binary_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("failed to open TensorRT engine: " + path);
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        throw std::runtime_error("TensorRT engine is empty: " + path);
    }
    std::vector<char> bytes(static_cast<std::size_t>(size));
    file.seekg(0, std::ios::beg);
    if (!file.read(bytes.data(), size)) {
        throw std::runtime_error("failed to read TensorRT engine: " + path);
    }
    return bytes;
}

void write_binary_file(const std::string& path, const void* data, std::size_t size) {
    const std::filesystem::path engine_path(path);
    if (!engine_path.parent_path().empty()) {
        std::filesystem::create_directories(engine_path.parent_path());
    }
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file || !file.write(static_cast<const char*>(data), static_cast<std::streamsize>(size))) {
        throw std::runtime_error("failed to write TensorRT engine: " + path);
    }
}

// The postprocessors index the outputs channel-major; a size mismatch means a different engine.
void validate_contiguous_channels(
    const float* data,
    std::size_t element_count,
    int expected_channels,
    int expected_plane) {
    const std::size_t expected_count =
        static_cast<std::size_t>(expected_channels) * static_cast<std::size_t>(expected_plane);
    if (data == nullptr || element_count != expected_count) {
        std::ostringstream error;
        error << "unexpected contiguous TensorRT output size=" << element_count
              << ", expected channels=" << expected_channels << " plane=" << expected_plane;
        throw std::runtime_error(error.str());
    }
}

}  // namespace

// What one engine must be: files, tensor shapes and DLA placement, from the configuration.
struct InferenceModule::TensorRtModelContract {
    std::string label;
    std::string onnx_path;
    std::string engine_path;
    int input_size = 0;    // input width (columns)
    int input_height = 0;  // input rows: input_size for a square model, 608 for the active12 window
    int detection_channels = 0;
    int detection_candidates = 0;
    int mask_channels = MASK_CHANNELS;
    int mask_size = 0;     // prototype width
    int mask_height = 0;   // prototype rows (input_height / 4)
    int workspace_mb = 0;
    bool use_fp16 = false;
    bool use_int8 = false;
    Config::Int8BuildConfig int8_build;  // the INT8 plan's files, read only when the engine is built
    bool use_dla = false;
    int dla_core = -1;
    bool allow_gpu_fallback = false;
};

// One loaded engine: TensorRT objects, its own CUDA stream, device buffers and pinned host outputs.
struct InferenceModule::TensorRtModelState {
    explicit TensorRtModelState(TensorRtModelContract contract_in)
        : contract(std::move(contract_in)) {}

    TensorRtModelContract contract;
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t cuda_stream = nullptr;
    void* input_device = nullptr;
    void* detection_device = nullptr;
    void* prototype_device = nullptr;
    std::size_t input_bytes = 0U;
    std::size_t detection_bytes = 0U;
    std::size_t prototype_bytes = 0U;
    std::size_t detection_elements = 0U;
    std::size_t prototype_elements = 0U;
    std::string input_tensor_name;
    std::string detection_tensor_name;
    std::string prototype_tensor_name;
    float* detection_host = nullptr;
    float* prototype_host = nullptr;
    std::vector<int64_t> detection_shape;
    std::vector<int64_t> prototype_shape;
    bool loaded = false;
    // Reused only after both current-frame D2H completions are consumed.
    // Timing is CUDA-stream-local; never subtracted from host timestamps.
    std::array<cudaEvent_t, 4> diagnostic_events{};
    int cuda_device_id = -1;
};

// The portable worker is shared with the real-thread diagnostic tests. Its
// original locking/callback/notification algorithm is unchanged when trace=null.
class InferenceModule::ParallelTaskWorker : public perception::ParallelTaskWorker {};
#endif

InferenceModule::InferenceModule(
    const Config& config,
    std::shared_ptr<ThreadsafeQueue<FrameData>> input_queue,
    std::shared_ptr<ThreadsafeQueue<DetectionResult>> output_queue,
    perception::rns::SourceJournal* health_completions,
    ModelLoadPolicy model_load_policy)
    : ModuleBase("InferenceModule"),
      config_(config),
      input_queue_(std::move(input_queue)),
      output_queue_(std::move(output_queue)),
      health_completions_(health_completions),
      model_load_policy_(model_load_policy),
      worker_()
#ifdef HAS_TENSORRT
      ,
      primary_model_(nullptr),
      active12_model_(nullptr),
      primary_input_host_(nullptr),
      primary_input_host_bytes_(0U),
      active12_input_host_(nullptr),
      active12_input_host_bytes_(0U),
      active_lead_us_(-1)
#endif
      ,
      dual_model_enabled_(config.dual_model_enabled()),
      consecutive_failures_(0) {}

InferenceModule::~InferenceModule() {
    stop();
#ifdef HAS_TENSORRT
    release_tensor_rt();
    release_pinned_input_buffers();
#endif
}

// Every invalid switch value or combination fails start(); there is no fallback mode. The checks
// below mirror the switch table of the service (service_config.cpp).
bool InferenceModule::start() {
    if (running_.load()) {
        return true;
    }
    // COCO80 and Active12 always run together; the single-model route is gone
    // (level-3 trim). A loaded Config always has the dual model
    // (dual_model.enabled is required and must be true), so this refuses only
    // a Config that was never loaded or whose load failed.
    if (!dual_model_enabled_) {
        LOG_ERROR_M(name_, "inference requires dual_model.enabled=true (COCO80 and Active12)");
        return false;
    }

    jit_calibration_complete_.store(false, std::memory_order_release);
    jit_frozen_delay_us_.store(-1, std::memory_order_release);
    if (health_completions_ && (!health_completions_->valid() ||
        health_completions_->kind() != perception::rns::SourceKind::InferenceComplete)) {
        LOG_ERROR_M(name_, "invalid optional inference health journal");
        return false;
    }

    if (schedule_probe_failed_.load()) {
        LOG_ERROR_M(name_, "JIT_PROBE_FATAL diagnostic failure is latched; restart process explicitly");
        return false;
    }
    schedule_probe_enabled_ = false;
    schedule_probe_jit_ = false;
    jit_async_submit_ = false;
    jit_fresh_bind_ = false;
    jit_second_chance_ = false;
    jit_empty_acquire_ = false;
    jit_empty_wait_ready_ = false;
    jit_frame_move_ = false;
    jit_probe_log_ = 2;
    execution_cpu_trace_ = false;
    post_role_swap_ = false;
    jit_fixed_bind_delay_ms_ = -1.0;
    active12_mask_implementation_ = Active12MaskImplementation::Serial;
    active12_mask_binary_implementation_ = Active12MaskBinaryImplementation::Legacy;
    mask_output_copy_mode_ = perception::MaskOutputCopyMode::Clone;
    try {
        mask_output_copy_mode_ = perception::parse_mask_output_copy_mode(
            std::getenv("PERCEPTION_MASK_OUTPUT_COPY"));
        execution_cpu_trace_ = perception::parse_execution_cpu_trace(
            std::getenv("PERCEPTION_EXECUTION_CPU_TRACE"));
        post_role_swap_ = perception::parse_post_role_swap(
            std::getenv("PERCEPTION_POST_ROLE_SWAP"));
    } catch (const std::invalid_argument& error) {
        LOG_ERROR_M(name_, error.what());
        return false;
    }
    if (const char* binary = std::getenv("PERCEPTION_ACTIVE12_MASK_BINARY_IMPL")) {
        try {
            active12_mask_binary_implementation_ = parse_active12_mask_binary_implementation(binary);
        } catch (const std::invalid_argument& error) {
            LOG_ERROR_M(name_, error.what());
            return false;
        }
    }
    if (const char* mode = std::getenv("PERCEPTION_ACTIVE12_MASK_IMPL")) {
        const std::string text(mode);
        if (text == "serial") active12_mask_implementation_ = Active12MaskImplementation::Serial;
        else if (text == "parallel2") active12_mask_implementation_ = Active12MaskImplementation::Parallel2;
        else if (text == "shadow") active12_mask_implementation_ = Active12MaskImplementation::Shadow;
        else {
            LOG_ERROR_M(name_, "PERCEPTION_ACTIVE12_MASK_IMPL must be serial, parallel2 or shadow");
            return false;
        }
    }
#ifndef HAS_TENSORRT
    if (active12_mask_implementation_ != Active12MaskImplementation::Serial ||
        active12_mask_binary_implementation_ != Active12MaskBinaryImplementation::Legacy) {
        LOG_ERROR_M(name_, "non-default Active12 mask lane/binary mode requires TensorRT");
        return false;
    }
#endif
    if (const char* mode = std::getenv("PERCEPTION_TWO_SLOT_JIT")) {
        const std::string text(mode);
        if (text != "0" && text != "1") {
            LOG_ERROR_M(name_, "PERCEPTION_TWO_SLOT_JIT must be exactly 0 or 1");
            return false;
        }
        schedule_probe_enabled_ = true;
        schedule_probe_jit_ = text == "1";
    }
    if (const char* async = std::getenv("PERCEPTION_JIT_ASYNC_SUBMIT")) {
        const std::string text(async);
        if (!schedule_probe_jit_ || (text != "0" && text != "1")) {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_ASYNC_SUBMIT requires JIT mode and exactly 0 or 1");
            return false;
        }
        jit_async_submit_ = text == "1";
    }
    if (const char* delay = std::getenv("PERCEPTION_JIT_BIND_DELAY_MS")) {
        try {
            if (!schedule_probe_jit_) {
                throw std::invalid_argument("fixed bind delay requires JIT mode");
            }
            jit_fixed_bind_delay_ms_ = perception::parse_jit_bind_delay_ms(
                delay, static_cast<double>(config_.timeout_ms()));
        } catch (const std::exception& error) {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_BIND_DELAY_MS invalid: " << error.what());
            return false;
        }
    }
    try {
        jit_fresh_bind_ = perception::parse_jit_fresh_bind_enabled(
            std::getenv("PERCEPTION_JIT_FRESH_BIND"));
        if (jit_fresh_bind_ &&
            (!schedule_probe_jit_ || !perception::inference_profile_enabled() ||
             jit_fixed_bind_delay_ms_ < 0.0 || !jit_async_submit_ || config_.frame_queue_size() != 1U)) {
            throw std::invalid_argument(
                "fresh bind requires JIT, profiling, fixed D, async submit and frame_queue_size=1");
        }
    } catch (const std::exception& error) {
        LOG_ERROR_M(name_, "PERCEPTION_JIT_FRESH_BIND invalid: " << error.what());
        return false;
    }
    try {
        jit_second_chance_ = perception::parse_jit_second_chance_enabled(
            std::getenv("PERCEPTION_JIT_SECOND_CHANCE"));
        perception::validate_jit_second_chance_config(jit_second_chance_, schedule_probe_jit_,
            jit_async_submit_, perception::inference_profile_enabled(), jit_fresh_bind_,
            config_.frame_queue_size());
    } catch (const std::exception& error) {
        LOG_ERROR_M(name_, "PERCEPTION_JIT_SECOND_CHANCE invalid: " << error.what());
        return false;
    }
    if (const char* move = std::getenv("PERCEPTION_JIT_FRAME_MOVE")) {
        const std::string text(move);
        if (text != "0" && text != "1") {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_FRAME_MOVE must be exactly 0 or 1");
            return false;
        }
        jit_frame_move_ = text == "1";
    }
    jit_stall_trace_ = false;
    active12_mask_log_ = false;
    mask_summary_ = MaskProfileSummary{};
    mask_area_.reset();
    if (const char* mask_log = std::getenv("PERCEPTION_ACTIVE12_MASK_LOG")) {
        const std::string text(mask_log);
        if (text != "0" && text != "1") {
            LOG_ERROR_M(name_, "PERCEPTION_ACTIVE12_MASK_LOG must be exactly 0 or 1");
            return false;
        }
        active12_mask_log_ = text == "1";
    }
    active12_padding_guard_ = false;
    active12_padding_dropped_.store(0);
    active12_padding_frames_.store(0);
    if (const char* guard = std::getenv("PERCEPTION_ACTIVE12_PADDING_ANCHOR_GUARD")) {
        const std::string text(guard);
        if (text != "0" && text != "1") {
            LOG_ERROR_M(name_, "PERCEPTION_ACTIVE12_PADDING_ANCHOR_GUARD must be exactly 0 or 1");
            return false;
        }
        active12_padding_guard_ = text == "1";
    }
    if (const char* stall = std::getenv("PERCEPTION_JIT_STALL_TRACE")) {
        const std::string text(stall);
        if (text != "0" && text != "1") {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_STALL_TRACE must be exactly 0 or 1");
            return false;
        }
        jit_stall_trace_ = text == "1";
    }
    post_dump_dir_.clear();
    post_dump_remaining_ = 0;
    if (const char* dump_dir = std::getenv("PERCEPTION_POST_DUMP_DIR")) {
        const char* frames = std::getenv("PERCEPTION_POST_DUMP_FRAMES");
        post_dump_dir_ = dump_dir;
        post_dump_remaining_ = frames ? std::atoi(frames) : 0;
        if (post_dump_dir_.empty() || post_dump_dir_[0] != '/' || post_dump_remaining_ <= 0 || post_dump_remaining_ > 200) {
            LOG_ERROR_M(name_, "PERCEPTION_POST_DUMP_DIR must be absolute and PERCEPTION_POST_DUMP_FRAMES in [1,200]");
            return false;
        }
        LOG_WARN_M(name_, "POST_DUMP_CONFIG DIAGNOSTIC_ONLY dir=" << post_dump_dir_ << " frames=" << post_dump_remaining_);
    }
    if (const char* probe_log = std::getenv("PERCEPTION_JIT_PROBE_LOG")) {
        const std::string text(probe_log);
        if (text == "full") jit_probe_log_ = 2;
        else if (text == "compact") jit_probe_log_ = 1;
        else if (text == "off") jit_probe_log_ = 0;
        else {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_PROBE_LOG must be full, compact or off");
            return false;
        }
    }
    LOG_INFO_M(name_, "CANDIDATE_OUTPUT_CONFIG jit_probe_log="
        << (jit_probe_log_ == 2 ? "full" : jit_probe_log_ == 1 ? "compact" : "off")
        << " jit_frame_move=" << static_cast<int>(jit_frame_move_)
        << " jit_stall_trace=" << static_cast<int>(jit_stall_trace_));
    if (const char* acquire = std::getenv("PERCEPTION_JIT_EMPTY_ACQUIRE")) {
        const std::string text(acquire);
        if (text != "0" && text != "1" && text != "2") {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_EMPTY_ACQUIRE must be exactly 0, 1 or 2");
            return false;
        }
        // 1: background acquire after an empty JIT pop.
        // 2: first wait (caller lane) until the in-flight pair is observed
        //    complete; a frame arriving in that interval uses the ordinary
        //    two-lane prepare and overlap submit, else background acquire.
        jit_empty_acquire_ = text == "1" || text == "2";
        jit_empty_wait_ready_ = text == "2";
        if (jit_empty_acquire_ && (!schedule_probe_jit_ || !jit_async_submit_ || jit_fresh_bind_ ||
                                   jit_second_chance_ || config_.frame_queue_size() != 1U)) {
            LOG_ERROR_M(name_, "PERCEPTION_JIT_EMPTY_ACQUIRE requires JIT async submit, frame_queue_size=1, "
                               "and excludes fresh bind and second chance");
            return false;
        }
    }
    // Only the JIT loop paces its submissions (jit_schedule.hpp); the serial loop has no cap. Its
    // serial-next path would hold every result back by the whole wait, so a cap needs async submit.
    infer_min_interval_ms_ = 0;
    try {
        infer_min_interval_ms_ = perception::parse_infer_min_interval_ms(
            std::getenv("PERCEPTION_INFER_MIN_INTERVAL_MS"));
        if (infer_min_interval_ms_ > 0 && (!schedule_probe_jit_ || !jit_async_submit_)) {
            throw std::invalid_argument("a minimum submit interval requires JIT mode and async submit");
        }
    } catch (const std::exception& error) {
        LOG_ERROR_M(name_, "PERCEPTION_INFER_MIN_INTERVAL_MS invalid: " << error.what());
        return false;
    }
    if (infer_min_interval_ms_ > 0) {
        LOG_INFO_M(name_, "INFER_PACE_CONFIG interval_ms=" << infer_min_interval_ms_
            << " slice_ms=" << perception::JIT_PACE_SLICE_NS / 1000000 << " scope=every_jit_submit");
    }
    if (schedule_probe_enabled_ &&
        (!dual_model_enabled_ || !perception::inference_profile_enabled())) {
        LOG_ERROR_M(name_, "schedule probe requires dual-model and PERCEPTION_INFERENCE_PROFILE=1");
        return false;
    }
    if (execution_cpu_trace_ && (!schedule_probe_jit_ || !perception::inference_profile_enabled())) {
        LOG_ERROR_M(name_, "PERCEPTION_EXECUTION_CPU_TRACE requires profiled diagnostic JIT");
        return false;
    }
    try {
        perception::validate_post_role_swap_config(post_role_swap_, perception::inference_profile_enabled(),
            schedule_probe_jit_, dual_model_enabled_);
    } catch (const std::invalid_argument& error) {
        LOG_ERROR_M(name_, error.what());
        return false;
    }
#ifndef HAS_TENSORRT
    if (schedule_probe_enabled_) {
        LOG_ERROR_M(name_, "schedule probe requires TensorRT");
        return false;
    }
#endif

#ifdef HAS_TENSORRT
    active_lead_us_ = -1;
    if (const char* value = std::getenv("PERCEPTION_DLA_ACTIVE_LEAD_US");
        value != nullptr && value[0] != '\0') {
        const char* const end = value + std::char_traits<char>::length(value);
        const auto parsed = std::from_chars(value, end, active_lead_us_);
        constexpr int kMaxDiagnosticLeadUs = 50000;
        if (parsed.ec != std::errc{} || parsed.ptr != end || active_lead_us_ < 0 ||
            active_lead_us_ > kMaxDiagnosticLeadUs) {
            LOG_ERROR_M(
                name_,
                "PERCEPTION_DLA_ACTIVE_LEAD_US must be an integer in [0,"
                    << kMaxDiagnosticLeadUs << "]");
            return false;
        }
    }
#endif

    if (!load_model()) {
        LOG_ERROR_M(name_, "failed to load model");
        return false;
    }
#ifdef HAS_TENSORRT
    // The Active12 mask kernels calibrate once per size; do it here, not on the first decoded frame.
    if (active12_model_ && active12_model_->prototype_shape.size() == 4U) {
        yolo::prewarm_active12_mask_kernels(static_cast<int>(active12_model_->prototype_shape[3]),
                                            static_cast<int>(active12_model_->prototype_shape[2]),
                                            config_.camera_output_width(), config_.camera_output_height());
    }
#endif

    try {
        active12_mask_lanes_.reset();
        if (active12_mask_implementation_ != Active12MaskImplementation::Serial) {
            active12_mask_lanes_ = std::make_unique<perception::PersistentTaskPair>(
                []() { perception::set_current_thread_name("mask_lane"); });
        }
    } catch (const std::exception& error) {
        LOG_ERROR_M(name_, "failed to initialize Active12 mask CPU lanes: " << error.what());
        return false;
    }
    LOG_INFO_M(name_, "ACTIVE_MASK_CONFIG impl="
        << active12_mask_implementation_name(active12_mask_implementation_)
        << " binary_impl=" << active12_mask_binary_implementation_name(active12_mask_binary_implementation_)
        << " profile=" << static_cast<int>(perception::inference_profile_enabled())
        << " opencv_threads=" << cv::getNumThreads()
        << " padding_anchor_guard=" << static_cast<int>(active12_padding_guard_));
    LOG_INFO_M(name_, "MASK_OUTPUT_COPY_CONFIG mode="
        << perception::mask_output_copy_mode_name(mask_output_copy_mode_));

    running_.store(true);
#ifdef HAS_TENSORRT
    if (schedule_probe_jit_) {
        worker_ = std::thread(&InferenceModule::infer_loop_jit, this);
    } else if (schedule_probe_enabled_ || active12_mask_implementation_ == Active12MaskImplementation::Shadow ||
               active12_mask_binary_implementation_ == Active12MaskBinaryImplementation::Shadow) {
        // The serial diagnostic also fails closed for preprocessing/queue
        // exceptions outside infer_loop's established execution-only catch.
        worker_ = std::thread([this]() {
            try {
                infer_loop();
            } catch (const std::exception& error) {
                if (primary_model_) drain_tensor_rt_no_throw(*primary_model_);
                if (active12_model_) drain_tensor_rt_no_throw(*active12_model_);
                schedule_probe_failed_.store(true);
                running_.store(false);
                LOG_ERROR_M(name_, "JIT_PROBE_FATAL mode=serial outer_error="
                    << error.what() << " reload=0");
            } catch (...) {
                if (primary_model_) drain_tensor_rt_no_throw(*primary_model_);
                if (active12_model_) drain_tensor_rt_no_throw(*active12_model_);
                schedule_probe_failed_.store(true);
                running_.store(false);
                LOG_ERROR_M(name_, "JIT_PROBE_FATAL mode=serial unknown exception reload=0");
            }
        });
    } else
#endif
    {
        worker_ = std::thread(&InferenceModule::infer_loop, this);
    }
    LOG_INFO_M(name_, "started");
    return true;
}

// Joins the loop (the JIT loop drains the device before it returns), then prints the last mask
// windows.
void InferenceModule::stop() {
    const bool was_running = running_.exchange(false);

    if (worker_.joinable()) {
        worker_.join();
    }
#ifdef HAS_TENSORRT
    // The workers are joined (the post helper belongs to the joined loop): print the last window.
    print_mask_summary(true);
    // Printed with or without the mask profile, so a guard that fired is always on record.
    LOG_INFO_M(name_, "ACTIVE12_PADDING_GUARD enabled=" << static_cast<int>(active12_padding_guard_)
        << " dropped_total=" << active12_padding_dropped_.load(std::memory_order_relaxed)
        << " frames_total=" << active12_padding_frames_.load(std::memory_order_relaxed));
    print_mask_areas(true);
    print_scene_luma(true);
#endif
    active12_mask_lanes_.reset();
    if (was_running) {
        LOG_INFO_M(name_, "stopped");
    }
}

// Serial alternative (two_slot_jit = 0): prepare, run the pair and wait, postprocess, merge,
// push; the next frame is prepared only after the current result is pushed. After
// MAX_CONSECUTIVE_FAILURES errors in a row both engines are reloaded.
void InferenceModule::infer_loop() {
    perception::set_current_thread_name("infer_loop");
    next_input_.valid = false;
    std::uint64_t probe_generation = 0;
#ifdef HAS_TENSORRT
    std::uint64_t probe_last_frame_id = 0;
    bool probe_has_frame = false;
#endif
    if (schedule_probe_enabled_) {
        LOG_WARN_M(name_, "SCHEDULE_PROBE mode=serial phase=MEASURING full_postprocess=1");
    }

    const bool profile_enabled = perception::inference_profile_enabled();
#ifdef HAS_TENSORRT
    LOG_INFO_M(
        name_,
        "dual-model launch schedule="
            << (active_lead_us_ < 0 ? "legacy_primary_first" : "active12_first")
            << " active_lead_us=" << std::max(active_lead_us_, 0));
#endif
    const auto elapsed_ms = [](const auto& started) {
        return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                   std::chrono::steady_clock::now() - started)
            .count();
    };
    struct ProfileAccumulator {
        std::uint64_t samples = 0;
        double prepare_ms = 0.0;
        double clone_ms = 0.0;
        double execute_ms = 0.0;
        double primary_post_ms = 0.0;
        double active_post_ms = 0.0;
        double post_wall_ms = 0.0;
        double merge_ms = 0.0;
        double inference_ms = 0.0;
        double queue_push_ms = 0.0;
        double work_ms = 0.0;
        double max_work_ms = 0.0;
        std::uint64_t primary_detections = 0;
        std::uint64_t active_detections = 0;
    } profile;
    double prepared_preprocess_ms = 0.0;
    double prepared_clone_ms = 0.0;
#ifdef HAS_TENSORRT
    auto parallel_worker = std::make_unique<ParallelTaskWorker>();
#endif

    while (running_.load()) {
        if (!next_input_.valid) {
            FrameData frame;
            if (!input_queue_->pop(frame, std::chrono::milliseconds(100))) {
                update_heartbeat();
                continue;
            }
            if (profile_enabled) {
                frame.pipeline_timing.inference_pop_end_ns =
                    perception::monotonic_now_ns();
            }
            const auto prepare_started = std::chrono::steady_clock::now();
#ifdef HAS_TENSORRT
            preprocess_model_inputs(
                frame.frame,
                next_input_.letterbox_info,
                next_input_.active12_letterbox_info,
                parallel_worker.get());
#else
            next_input_.tensor_data =
                preprocess(frame.frame, config_.input_size(), next_input_.letterbox_info);
            if (dual_model_enabled_) {
                next_input_.active12_tensor_data = preprocess_active12(
                    frame.frame, next_input_.active12_letterbox_info);
            }
#endif
            prepared_preprocess_ms = elapsed_ms(prepare_started);
            const auto clone_started = std::chrono::steady_clock::now();
            next_input_.frame = frame.frame.clone();
            next_input_.depth = frame.depth.clone();
            prepared_clone_ms = elapsed_ms(clone_started);
            next_input_.depth_intrinsics = frame.depth_intrinsics;
            next_input_.depth_registration = frame.depth_registration;
            next_input_.depth_scale = frame.depth_scale;
            next_input_.camera_metadata = frame.camera_metadata;
            next_input_.frame_id = frame.frame_id;
            next_input_.stamp_sec = frame.stamp_sec;
            next_input_.capture_time_ms = frame.capture_time_ms;
            next_input_.pipeline_timing = frame.pipeline_timing;
            next_input_.source_binding.bind(frame);
            next_input_.valid = true;
        }

#ifndef HAS_TENSORRT
        auto current_data = std::move(next_input_.tensor_data);
        auto current_active12_data = std::move(next_input_.active12_tensor_data);
#endif
        auto current_frame = std::move(next_input_.frame);
        auto current_depth = std::move(next_input_.depth);
        auto current_depth_intrinsics = next_input_.depth_intrinsics;
        auto current_depth_registration = next_input_.depth_registration;
        auto current_depth_scale = next_input_.depth_scale;
        auto current_camera_metadata = next_input_.camera_metadata;
        auto current_info = next_input_.letterbox_info;
        auto current_active12_info = next_input_.active12_letterbox_info;
        auto current_frame_id = next_input_.frame_id;
        auto current_stamp_sec = next_input_.stamp_sec;
        auto current_capture_time_ms = next_input_.capture_time_ms;
        auto current_pipeline_timing = next_input_.pipeline_timing;
        const auto current_native_source = next_input_.source_binding.source();
        const double current_preprocess_ms = prepared_preprocess_ms;
        const double current_clone_ms = prepared_clone_ms;
        next_input_.valid = false;

        Timer timer;
        timer.start();

        std::vector<Detection> detections;
        std::vector<Detection> map_detections;
        std::vector<CrossModelConflictAudit> cross_model_conflicts;
        double execute_ms = 0.0;
        double primary_post_ms = 0.0;
        double active_post_ms = 0.0;
        double post_wall_ms = 0.0;
        double merge_ms = 0.0;
        std::int64_t probe_submit_ns = 0;
        std::int64_t probe_ready_ns = 0;
        std::array<float, 4> probe_primary_cuda{};
        std::array<float, 4> probe_active_cuda{};
        double probe_load_log_ms = 0.0;
        std::size_t primary_detection_count = 0;
        std::size_t active_detection_count = 0;
        bool inference_succeeded = false;
        const auto infer_start = std::chrono::steady_clock::now();
        try {
#ifdef HAS_TENSORRT
            if (primary_model_ == nullptr || !primary_model_->loaded) {
                throw std::runtime_error("primary TensorRT engine is not initialized");
            }

            if (active12_model_ == nullptr || !active12_model_->loaded) {
                throw std::runtime_error("active12 TensorRT engine is not initialized");
            }
            const auto execute_started = std::chrono::steady_clock::now();
            if (schedule_probe_enabled_) {
                if (probe_has_frame && current_frame_id <= probe_last_frame_id) {
                    throw std::logic_error("serial probe frame ID is not strictly increasing");
                }
                probe_has_frame = true;
                probe_last_frame_id = current_frame_id;
                ++probe_generation;
                probe_submit_ns = perception::monotonic_now_ns();
            }
            execute_tensor_rt_pair(
                *primary_model_,
                primary_input_host_,
                primary_input_host_bytes_,
                *active12_model_,
                active12_input_host_,
                active12_input_host_bytes_,
                active_lead_us_);
            execute_ms = elapsed_ms(execute_started);
            if (schedule_probe_enabled_) {
                probe_ready_ns = perception::monotonic_now_ns();
                probe_primary_cuda = tensor_rt_diagnostic_intervals(*primary_model_);
                probe_active_cuda = tensor_rt_diagnostic_intervals(*active12_model_);
            }

            if (parallel_worker == nullptr) {
                throw std::logic_error(
                    "dual-model postprocess requires a parallel task worker");
            }
            // The two model outputs own separate host buffers and their
            // postprocessors only read immutable configuration. Reuse one
            // persistent helper thread for the independent CPU branches,
            // then preserve the established merge order below.
            const auto post_wall_started = std::chrono::steady_clock::now();
            std::vector<Detection> primary_detections;
            std::vector<Detection> active12_detections;
            parallel_worker->run(
                [this,
                 &current_info,
                 &elapsed_ms,
                 &primary_detections,
                 &primary_post_ms]() {
                    const auto started = std::chrono::steady_clock::now();
                    primary_detections =
                        postprocess_primary(*primary_model_, current_info);
                    primary_post_ms = elapsed_ms(started);
                },
                [this,
                 &current_active12_info,
                 current_frame_id,
                 &elapsed_ms,
                 &active12_detections,
                 &active_post_ms]() {
                    const auto started = std::chrono::steady_clock::now();
                    active12_detections = postprocess_active12(
                        *active12_model_, current_active12_info, current_frame_id);
                    active_post_ms = elapsed_ms(started);
                });
            active_detection_count = active12_detections.size();
            primary_detection_count = primary_detections.size();
            post_wall_ms = elapsed_ms(post_wall_started);
            if (schedule_probe_enabled_) {
                const auto load_started = std::chrono::steady_clock::now();
                LOG_INFO_M(name_, "PROBE_LOAD mode=serial phase=MEASURING frame_id="
                    << current_frame_id << " generation=" << probe_generation
                    << " branch=coco80 candidates=-1 detections="
                    << diagnostic_load_fields(primary_detections));
                LOG_INFO_M(name_, "PROBE_LOAD mode=serial phase=MEASURING frame_id="
                    << current_frame_id << " generation=" << probe_generation
                    << " branch=active12 candidates=-1 detections="
                    << diagnostic_load_fields(active12_detections));
                probe_load_log_ms = elapsed_ms(load_started);
            }

            if (perception::inference_profile_enabled()) record_mask_areas(primary_detections, active12_detections);
            const auto& dual = config_.dual_model();
            DualModelMergeConfig merge_config;
            merge_config.vehicle_heavy_conflict_iou = dual.conflict_iou;
            const auto merge_started = std::chrono::steady_clock::now();
            auto merged = merge_dual_model_detections(
                primary_detections,
                active12_detections,
                merge_config);
            merge_ms = elapsed_ms(merge_started);
            detections = std::move(merged.tracker_detections);
            map_detections = std::move(merged.map_detections);
            cross_model_conflicts = std::move(merged.conflicts);
            consecutive_failures_ = 0;
            inference_succeeded = true;
#else
            (void)current_data;
            (void)current_active12_data;
            (void)current_info;
            (void)current_active12_info;
#endif
        } catch (const std::exception& ex) {
            if (schedule_probe_enabled_ || active12_mask_implementation_ == Active12MaskImplementation::Shadow ||
                active12_mask_binary_implementation_ == Active12MaskBinaryImplementation::Shadow) {
#ifdef HAS_TENSORRT
                if (primary_model_) drain_tensor_rt_no_throw(*primary_model_);
                if (active12_model_) drain_tensor_rt_no_throw(*active12_model_);
#endif
                schedule_probe_failed_.store(true);
                running_.store(false);
                LOG_ERROR_M(name_, "JIT_PROBE_FATAL mode=serial frame_id=" << current_frame_id
                    << " error=" << ex.what() << " reload=0 publish_failed_frame=0");
                return;
            }
            ++consecutive_failures_;
            LOG_ERROR_M(name_, "inference failed: " << ex.what()
                                                    << ", consecutive_failures=" << consecutive_failures_);
#ifdef HAS_TENSORRT
            if (consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) {
                LOG_WARN_M(name_, "reloading TensorRT engine after repeated failures");
                if (load_model()) {
                    consecutive_failures_ = 0;
                }
            }
#endif
        }
        // One normal frame completion after both outputs, post and merge
        // are usable, BEFORE result queue admission or Objects publication.
        // Empty detections count; failed/non-TRT work does not.
        // record() samples MONOTONIC inside its commit boundary, independently
        // of optional profiling. Failed/reloaded work never enters record().
        perception::rns::record_inference_completion(health_completions_,
            inference_succeeded, current_native_source);
        const auto infer_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - infer_start)
                                      .count();
        if (infer_elapsed > config_.timeout_ms()) {
            LOG_WARN_M(
                name_,
                "inference slow: " << infer_elapsed << "ms > timeout " << config_.timeout_ms() << "ms");
        }

        timer.stop();

        DetectionResult result;
        result.detections = std::move(detections);
        result.map_detections = std::move(map_detections);
        result.cross_model_conflicts = std::move(cross_model_conflicts);
        result.frame = std::move(current_frame);
        result.depth = std::move(current_depth);
        result.depth_intrinsics = current_depth_intrinsics;
        result.depth_registration = current_depth_registration;
        result.depth_scale = current_depth_scale;
        result.camera_metadata = current_camera_metadata;
        result.frame_id = current_frame_id;
        result.stamp_sec = current_stamp_sec;
        result.capture_time_ms = current_capture_time_ms;
        result.inference_time_ms = timer.elapsed_ms();
        current_pipeline_timing.preprocess_ms = current_preprocess_ms;
        current_pipeline_timing.clone_ms = current_clone_ms;
        current_pipeline_timing.execute_pair_ms = execute_ms;
        current_pipeline_timing.post_wall_ms = post_wall_ms;
        current_pipeline_timing.merge_ms = merge_ms;
        current_pipeline_timing.result_push_begin_ns =
            profile_enabled ? perception::monotonic_now_ns() : 0;
        current_pipeline_timing.valid =
            profile_enabled && inference_succeeded &&
            current_pipeline_timing.capture_epoch_ns > 0 &&
            perception::are_positive_nondecreasing(
                {current_pipeline_timing.capture_ready_ns,
                 current_pipeline_timing.frame_push_begin_ns,
                 current_pipeline_timing.inference_pop_end_ns,
                 current_pipeline_timing.result_push_begin_ns});
        result.pipeline_timing = current_pipeline_timing;
        result.native_depth_source = current_native_source;
        result.inference_completed = inference_succeeded;
        const auto queue_push_started = std::chrono::steady_clock::now();
        output_queue_->push(std::move(result));
        const double queue_push_ms = elapsed_ms(queue_push_started);
        if (schedule_probe_enabled_) {
            LOG_INFO_M(name_, "PROBE_FRAME mode=serial phase=MEASURING frame_id="
                << current_frame_id << " generation=" << probe_generation
                << " slot=0 submit_ns=" << probe_submit_ns
                << " ready_seen_ns=" << probe_ready_ns
                << " push_ns=" << current_pipeline_timing.result_push_begin_ns
                << " prepare_ms=" << current_preprocess_ms
                << " clone_ms=" << current_clone_ms
                << " execute_wait_ms=" << execute_ms
                << " post_wall_ms=" << post_wall_ms << " merge_ms=" << merge_ms
                << " load_log_ms=" << probe_load_log_ms
                << " primary_cuda_h2d_ms=" << probe_primary_cuda[0]
                << " primary_cuda_exec_span_ms=" << probe_primary_cuda[1]
                << " primary_cuda_d2h_ms=" << probe_primary_cuda[2]
                << " primary_cuda_total_ms=" << probe_primary_cuda[3]
                << " active_cuda_h2d_ms=" << probe_active_cuda[0]
                << " active_cuda_exec_span_ms=" << probe_active_cuda[1]
                << " active_cuda_d2h_ms=" << probe_active_cuda[2]
                << " active_cuda_total_ms=" << probe_active_cuda[3]);
        }
        update_heartbeat();

        if (profile_enabled && inference_succeeded) {
            const double inference_ms = timer.elapsed_ms();
            const double work_ms =
                current_preprocess_ms + current_clone_ms + inference_ms + queue_push_ms;
            ++profile.samples;
            profile.prepare_ms += current_preprocess_ms;
            profile.clone_ms += current_clone_ms;
            profile.execute_ms += execute_ms;
            profile.primary_post_ms += primary_post_ms;
            profile.active_post_ms += active_post_ms;
            profile.post_wall_ms += post_wall_ms;
            profile.merge_ms += merge_ms;
            profile.inference_ms += inference_ms;
            profile.queue_push_ms += queue_push_ms;
            profile.work_ms += work_ms;
            profile.max_work_ms = std::max(profile.max_work_ms, work_ms);
            profile.primary_detections += primary_detection_count;
            profile.active_detections += active_detection_count;

            constexpr std::uint64_t kProfileWindow = 60;
            if (profile.samples == kProfileWindow) {
                const double sample_count = static_cast<double>(profile.samples);
                const double mean_work_ms = profile.work_ms / sample_count;
                const double measured_inference_ms =
                    profile.execute_ms + profile.post_wall_ms + profile.merge_ms;
                LOG_INFO_M(
                    name_,
                    "INFERENCE_PROFILE samples=" << profile.samples
                                                   << " prepare_ms="
                                                   << (profile.prepare_ms / sample_count)
                                                   << " clone_ms="
                                                   << (profile.clone_ms / sample_count)
                                                   << " execute_pair_ms="
                                                   << (profile.execute_ms / sample_count)
                                                   << " primary_post_ms="
                                                   << (profile.primary_post_ms / sample_count)
                                                   << " active_post_ms="
                                                   << (profile.active_post_ms / sample_count)
                                                   << " post_wall_ms="
                                                   << (profile.post_wall_ms / sample_count)
                                                   << " merge_ms="
                                                   << (profile.merge_ms / sample_count)
                                                   << " inference_total_ms="
                                                   << (profile.inference_ms / sample_count)
                                                   << " inference_unaccounted_ms="
                                                   << ((profile.inference_ms - measured_inference_ms) /
                                                       sample_count)
                                                   << " queue_push_ms="
                                                   << (profile.queue_push_ms / sample_count)
                                                   << " work_total_ms=" << mean_work_ms
                                                   << " work_max_ms=" << profile.max_work_ms
                                                   << " work_fps="
                                                   << (mean_work_ms > 0.0
                                                           ? 1000.0 / mean_work_ms
                                                           : 0.0)
                                                   << " primary_detections="
                                                   << (static_cast<double>(
                                                           profile.primary_detections) /
                                                       sample_count)
                                                   << " active_detections="
                                                   << (static_cast<double>(
                                                           profile.active_detections) /
                                                       sample_count));
                profile = ProfileAccumulator{};
            }
        }

        // The TensorRT enqueue and synchronize above run on this same worker
        // thread, so preparing the next frame before enqueue did not overlap
        // with GPU work; it only delayed publication of the current result.
        // Fill the one-frame prepared slot after publication instead.
        FrameData next_frame;
        if (running_.load() && input_queue_->try_pop(next_frame)) {
            if (profile_enabled) {
                next_frame.pipeline_timing.inference_pop_end_ns =
                    perception::monotonic_now_ns();
            }
            const auto prepare_started = std::chrono::steady_clock::now();
#ifdef HAS_TENSORRT
            // Both TensorRT streams were synchronized before publication, so
            // the persistent pinned buffers are no longer read by CUDA here.
            preprocess_model_inputs(
                next_frame.frame,
                next_input_.letterbox_info,
                next_input_.active12_letterbox_info,
                parallel_worker.get());
#else
            next_input_.tensor_data =
                preprocess(next_frame.frame, config_.input_size(), next_input_.letterbox_info);
            if (dual_model_enabled_) {
                next_input_.active12_tensor_data = preprocess_active12(
                    next_frame.frame, next_input_.active12_letterbox_info);
            }
#endif
            prepared_preprocess_ms = elapsed_ms(prepare_started);
            const auto clone_started = std::chrono::steady_clock::now();
            next_input_.frame = next_frame.frame.clone();
            next_input_.depth = next_frame.depth.clone();
            prepared_clone_ms = elapsed_ms(clone_started);
            next_input_.depth_intrinsics = next_frame.depth_intrinsics;
            next_input_.depth_registration = next_frame.depth_registration;
            next_input_.depth_scale = next_frame.depth_scale;
            next_input_.camera_metadata = next_frame.camera_metadata;
            next_input_.frame_id = next_frame.frame_id;
            next_input_.stamp_sec = next_frame.stamp_sec;
            next_input_.capture_time_ms = next_frame.capture_time_ms;
            next_input_.pipeline_timing = next_frame.pipeline_timing;
            next_input_.source_binding.bind(next_frame);
            next_input_.valid = true;
        }
    }
}

#ifdef HAS_TENSORRT
// The production loop; the schedule is described in the file header. Every ownership rule (slot
// phase, generation, frame id) is checked and a violation throws: the loop then drains both
// streams and stops (JIT_PROBE_FATAL) instead of publishing a doubtful frame.
void InferenceModule::infer_loop_jit() {
    perception::set_current_thread_name("jit_caller");
    enum class Phase { empty, prepared, in_flight, ready, cpu_read };
    struct FreshBindTrace {
        bool normal_next = false;
        bool initial_frame_valid = false;
        std::uint64_t initial_frame_id = 0;
        std::int64_t initial_push_begin_ns = 0;
        std::int64_t wait_start_ns = 0;
        std::int64_t wait_end_ns = 0;
        std::int64_t decision_ns = 0;
        std::int64_t deadline_ns = 0;
        std::int64_t deadline_overrun_ns = 0;
        QueueUpdateWaitInfo wait;
        const char* reason = "not_applied";
    };
    struct Slot {
        // primary input, active input, primary detection/prototype,
        // active detection/prototype. Slot 0 borrows the established buffers.
        std::array<float*, 6> host{};
        bool owns_host = false;
        PreparedInput input;
        Phase phase = Phase::empty;
        std::uint64_t generation = 0;
        double prepare_ms = 0.0;
        double clone_ms = 0.0;
        std::int64_t prepared_ns = 0;
        std::int64_t submit_ns = 0;
        std::int64_t submit_end_ns = 0;
        std::int64_t pace_wait_ns = 0;  // waited for the minimum submit interval before submit_ns
        std::int64_t wait_begin_ns = 0;
        std::int64_t ready_seen_ns = 0;
        std::array<float, 4> primary_cuda{};
        std::array<float, 4> active_cuda{};
        perception::ExecutionCpuRecord prepare_primary_cpu, prepare_active_cpu;
        perception::ExecutionCpuRecord prepare_join_cpu, clone_cpu;
        FreshBindTrace fresh_bind;
        // Empty-pop acquire: this slot was filled by the submit lane after the
        // JIT try_pop found no frame (never a replayed or older frame).
        struct AcquireTrace {
            bool used = false;       // background acquire filled this slot
            bool late_used = false;  // caller-lane wait-until-ready filled this slot
            std::int64_t late_begin_ns = -1, late_pop_end_ns = -1;
            std::int64_t start_ns = -1, pop_end_ns = -1, prepared_ns = -1;
            std::int64_t device_free_ns = -1, submit_ns = -1;
        } acquire;
        ~Slot() {
            if (owns_host) {
                for (float* pointer : host) {
                    if (pointer) cudaFreeHost(pointer);
                }
            }
        }
    };
    // Declared outside try: every exception drains the device streams before
    // these pinned buffers can be destroyed. No context ever executes two frames.
    std::array<Slot, 2> slots;
    std::unique_ptr<ParallelTaskWorker> parallel_worker;
    std::unique_ptr<perception::PersistentTaskPair> submit_worker;
    std::mutex acquire_mutex;
    std::condition_variable acquire_cv;
    bool acquire_device_free = false;
    std::atomic<bool> acquire_abort{false};
    const auto abort_acquire = [&]() noexcept {
        acquire_abort.store(true, std::memory_order_release);
        try {
            std::lock_guard<std::mutex> lock(acquire_mutex);
            acquire_cv.notify_all();
        } catch (...) {
        }
    };
    std::uint64_t generation = 0;
    std::uint64_t last_admitted_frame = 0;
    std::uint64_t last_published_generation = 0;
    bool has_admitted_frame = false;
    bool pair_in_flight = false;
    std::uint64_t device_generation = 0;
    std::uint64_t device_frame_id = 0;
    // Minimum submit interval (0 = off). last_submit_ns is the previous submit of either slot;
    // like pair_in_flight it is ordered across the two lanes by the task-pair start/join.
    const std::int64_t pace_interval_ns = static_cast<std::int64_t>(infer_min_interval_ms_) * 1000000;
    std::int64_t last_submit_ns = 0;
    struct PaceSummary {
        std::uint64_t submits = 0, paced = 0;
        std::int64_t wait_total_ns = 0, wait_max_ns = 0;
    } pace;
    // Once per run, on every exit, after the submit lane has stopped; nothing with the cap off.
    const auto log_pace_summary = [&]() {
        if (pace_interval_ns <= 0) return;
        LOG_INFO_M(name_, "INFER_PACE_SUMMARY interval_ms=" << infer_min_interval_ms_
            << " paced=" << pace.paced
            << " wait_total_ms=" << static_cast<double>(pace.wait_total_ns) / 1000000.0
            << " wait_max_ms=" << static_cast<double>(pace.wait_max_ns) / 1000000.0
            << " submits=" << pace.submits);
    };
    const auto now_ns = []() {
        const auto value = perception::monotonic_now_ns();
        if (value <= 0) throw std::runtime_error("host monotonic clock unavailable");
        return value;
    };
    const auto ms_between = [](std::int64_t begin, std::int64_t end) {
        if (begin <= 0 || end < begin) {
            throw std::logic_error("non-monotonic JIT host endpoints");
        }
        return static_cast<double>(end - begin) / 1000000.0;
    };
    const auto fresh_bind_fields = [&](const FreshBindTrace& trace, bool calibrating = false) {
        std::ostringstream fields;
        fields << " fresh_bind_enabled=" << static_cast<int>(jit_fresh_bind_)
            << " fresh_bind_normal_next=" << static_cast<int>(trace.normal_next)
            << " fresh_bind_wait_start_ns=" << trace.wait_start_ns
            << " fresh_bind_wait_end_ns=" << trace.wait_end_ns
            << " fresh_bind_deadline_ns=" << trace.deadline_ns
            << " fresh_bind_decision_ns=" << trace.decision_ns
            << " fresh_bind_initial_frame_valid=" << static_cast<int>(trace.initial_frame_valid)
            << " fresh_bind_initial_frame_id=" << trace.initial_frame_id
            << " fresh_bind_initial_push_begin_ns=" << trace.initial_push_begin_ns
            << " fresh_bind_wait_requested=" << static_cast<int>(trace.wait.wait_requested)
            << " fresh_bind_updated=" << static_cast<int>(trace.wait.updated)
            << " fresh_bind_cleared=" << static_cast<int>(trace.wait.cleared)
            << " fresh_bind_deadline_overrun_ns=" << trace.deadline_overrun_ns
            << " fresh_bind_reason=" << (!jit_fresh_bind_ ? "off" :
                trace.normal_next ? trace.reason : calibrating ? "calibration" : "reprime");
        return fields.str();
    };
    try {
        if (!primary_model_ || !active12_model_ || !primary_model_->loaded ||
            !active12_model_->loaded || !dual_model_enabled_ ||
            !schedule_probe_enabled_ || !schedule_probe_jit_) {
            throw std::logic_error("JIT requires initialized diagnostic dual models");
        }
        const int cuda_device = primary_model_->cuda_device_id;
        if (cuda_device < 0 || active12_model_->cuda_device_id != cuda_device) {
            throw std::logic_error("JIT models have different or unknown CUDA creation devices");
        }
        check_cuda(cudaSetDevice(cuda_device), "select JIT model creation CUDA device");
        parallel_worker = std::make_unique<ParallelTaskWorker>();
        if (jit_async_submit_) {
            submit_worker = std::make_unique<perception::PersistentTaskPair>([cuda_device]() {
                perception::set_current_thread_name("jit_submit");
                check_cuda(cudaSetDevice(cuda_device), "select persistent submit CUDA device");
                int actual_device = -1;
                check_cuda(cudaGetDevice(&actual_device), "verify persistent submit CUDA device");
                if (actual_device != cuda_device) {
                    throw std::runtime_error("persistent submit CUDA device mismatch");
                }
            });
        }
        slots[0].host = {primary_input_host_, active12_input_host_,
                         primary_model_->detection_host, primary_model_->prototype_host,
                         active12_model_->detection_host, active12_model_->prototype_host};
        const std::array<std::size_t, 6> byte_counts = {
            primary_input_host_bytes_, active12_input_host_bytes_,
            primary_model_->detection_bytes, primary_model_->prototype_bytes,
            active12_model_->detection_bytes, active12_model_->prototype_bytes};
        slots[1].owns_host = true;
        for (std::size_t i = 0; i < byte_counts.size(); ++i) {
            void* pointer = nullptr;
            check_cuda(cudaMallocHost(&pointer, byte_counts[i]), "allocate second JIT host slot");
            slots[1].host[i] = static_cast<float*>(pointer);
            if (!slots[0].host[i] || slots[0].host[i] == slots[1].host[i]) {
                throw std::logic_error("JIT host slots alias or are unavailable");
            }
        }
        LOG_WARN_M(name_, "SCHEDULE_PROBE mode=jit phase=CALIBRATING warmup=120 samples=300"
            << " delay_rule=max(0,host_pair_median-prepare_clone_p99-2ms)"
            << " contexts_per_model=1 device_inflight_frames_max=1 host_slots=2"
            << " d2h=immediate active_lead_us=" << active_lead_us_
            << " async_submit=" << static_cast<int>(jit_async_submit_)
            << " fixed_D=" << static_cast<int>(jit_fixed_bind_delay_ms_ >= 0.0)
            << " cuda_device=" << cuda_device);
        LOG_INFO_M(name_, "EXECUTION_CPU_TRACE_CONFIG enabled=" << static_cast<int>(execution_cpu_trace_)
            << " wall_clock=CLOCK_MONOTONIC cpu_clock=CLOCK_THREAD_CPUTIME_ID"
            << " logging=after_join wall_minus_cpu=non_running_not_scheduler_attribution"
            << " unknown_duration=-1 default=off");
        LOG_INFO_M(name_, "POST_ROLE_SWAP_CONFIG enabled=" << static_cast<int>(post_role_swap_)
            << " scope=jit_current_post helper_role=" << perception::post_helper_role(post_role_swap_)
            << " caller_role=" << perception::post_caller_role(post_role_swap_)
            << " exception_priority=active default=off prepare_submit_roles=unchanged");

        // prepare: letterbox both inputs into the slot's pinned buffers (two lanes in parallel,
        // or serially on the acquire lane) and take over the frame and depth.
        const auto prepare = [&](Slot& slot, FrameData frame, bool serial = false) {
            if (slot.phase != Phase::empty) {
                throw std::logic_error("overwrite of an owned JIT slot");
            }
            if (has_admitted_frame && frame.frame_id <= last_admitted_frame) {
                throw std::logic_error("JIT admitted frame ID is not strictly increasing");
            }
            has_admitted_frame = true;
            last_admitted_frame = frame.frame_id;
            // Before inference_pop_end_ns and the prepare timing: a sample's own cost is in the
            // SCENE_LUMA line (cost_us_*), not in prepare_ms; it falls in the pop-to-prepare gap.
            if (perception::inference_profile_enabled()) sample_scene_luma(frame.frame, frame.frame_id);
            slot.fresh_bind = FreshBindTrace{};
            slot.prepare_primary_cpu = {}; slot.prepare_active_cpu = {};
            slot.prepare_join_cpu = {}; slot.clone_cpu = {};
            slot.input.pipeline_timing = frame.pipeline_timing;
            slot.input.pipeline_timing.inference_pop_end_ns = now_ns();
            const auto started = now_ns();
            const auto prepare_active = [&]() {
                perception::ExecutionCpuScope scope(slot.prepare_active_cpu, execution_cpu_trace_);
                const auto& spec = active12_spec();
                preprocess_window_into(frame.frame, ModelInputShape{spec.input_width, spec.input_height},
                    spec.input_window_top, slot.input.active12_letterbox_info, slot.host[1],
                    active12_input_host_bytes_ / sizeof(float));
            };
            const auto prepare_primary = [&]() {
                perception::ExecutionCpuScope scope(slot.prepare_primary_cpu, execution_cpu_trace_);
                preprocess_into(frame.frame, config_.input_size(),
                    slot.input.letterbox_info, slot.host[0],
                    primary_input_host_bytes_ / sizeof(float));
            };
            if (serial) {
                // Same two calls on the same immutable image into the same
                // slot-local buffers; only the executing lane differs.
                prepare_active();
                prepare_primary();
            } else {
                parallel_worker->run(prepare_active, prepare_primary,
                    execution_cpu_trace_ ? &slot.prepare_join_cpu : nullptr);
            }
            const auto preprocessed = now_ns();
            slot.prepare_ms = ms_between(started, preprocessed);
            {
                perception::ExecutionCpuScope scope(slot.clone_cpu, execution_cpu_trace_);
                if (jit_frame_move_) {
                    // The popped FrameData exclusively owns (or shares read-only
                    // with the stream encoder) immutable pixels; moving keeps the
                    // same bytes without a second copy.
                    slot.input.frame = std::move(frame.frame);
                    slot.input.depth = std::move(frame.depth);
                } else {
                    slot.input.frame = frame.frame.clone();
                    slot.input.depth = frame.depth.clone();
                }
            }
            slot.prepared_ns = now_ns();
            slot.clone_ms = ms_between(preprocessed, slot.prepared_ns);
            slot.input.depth_intrinsics = frame.depth_intrinsics;
            slot.input.depth_registration = frame.depth_registration;
            slot.input.depth_scale = frame.depth_scale;
            slot.input.camera_metadata = frame.camera_metadata;
            slot.input.frame_id = frame.frame_id;
            slot.input.stamp_sec = frame.stamp_sec;
            slot.input.capture_time_ms = frame.capture_time_ms;
            slot.input.source_binding.bind(frame);
            slot.input.valid = true;
            slot.generation = ++generation;
            slot.phase = Phase::prepared;
        };
        // submit: H2D, enqueue and D2H of both models into the slot's buffers; the device pair
        // now belongs to this slot until wait_ready. Every submit of the loop (calibration,
        // prime, serial next, overlap and acquire lanes) goes through here, so does the cap.
        const auto submit = [&](Slot& slot) {
            if (slot.phase != Phase::prepared || !slot.input.valid || pair_in_flight) {
                throw std::logic_error("JIT submitted an unready slot or reused an in-flight context");
            }
            validate_tensor_rt_input(*primary_model_, slot.host[0], primary_input_host_bytes_);
            validate_tensor_rt_input(*active12_model_, slot.host[1], active12_input_host_bytes_);
            // Minimum submit interval: wait out what is left of it since the previous submit,
            // with the device idle. A stop ends the wait; the pair is then submitted and drained
            // like any pair in flight at a stop, so no caller sees a slot left half-submitted.
            // Who waits: the caller on calibration and prime (the frame's own wait); the submit
            // lane on the overlap path, where publish's join also holds back the push of the
            // current frame by max(0, this wait - its post wall); the acquire lane, whose wait the
            // current push does not join. The serial-next call (async submit off) is refused
            // with a cap, as it would delay the current publish by the whole wait.
            if (pace_interval_ns > 0) {
                slot.pace_wait_ns = perception::jit_pace_wait(last_submit_ns, pace_interval_ns, now_ns,
                    [](std::int64_t ns) { std::this_thread::sleep_for(std::chrono::nanoseconds(ns)); },
                    [&]() { return running_.load() && !acquire_abort.load(std::memory_order_acquire); });
                ++pace.submits;
                if (slot.pace_wait_ns > 0) {
                    ++pace.paced;
                    pace.wait_total_ns += slot.pace_wait_ns;
                    pace.wait_max_ns = std::max(pace.wait_max_ns, slot.pace_wait_ns);
                }
            }
            // Latch ownership before the first async operation, including a
            // partially failed pair. The catch below drains both streams.
            pair_in_flight = true;
            device_generation = slot.generation;
            device_frame_id = slot.input.frame_id;
            slot.phase = Phase::in_flight;
            slot.submit_ns = now_ns();
            last_submit_ns = slot.submit_ns;
            if (active_lead_us_ < 0) {
                submit_tensor_rt(*primary_model_, slot.host[0], primary_input_host_bytes_);
                submit_tensor_rt(*active12_model_, slot.host[1], active12_input_host_bytes_);
            } else {
                submit_tensor_rt(*active12_model_, slot.host[1], active12_input_host_bytes_);
                if (active_lead_us_ > 0) {
                    std::this_thread::sleep_for(std::chrono::microseconds(active_lead_us_));
                }
                submit_tensor_rt(*primary_model_, slot.host[0], primary_input_host_bytes_);
            }
            // Both output transfers are queued now, never deferred until after
            // another frame's CPU postprocess. The target is slot-local pinned
            // memory; shared TensorRtModelState host pointers are never changed.
            queue_tensor_rt_outputs(*primary_model_, slot.host[2], slot.host[3]);
            queue_tensor_rt_outputs(*active12_model_, slot.host[4], slot.host[5]);
            slot.submit_end_ns = now_ns();
        };
        const auto wait_ready = [&](Slot& slot) {
            if (slot.phase != Phase::in_flight || !pair_in_flight ||
                device_generation != slot.generation ||
                device_frame_id != slot.input.frame_id) {
                throw std::logic_error("JIT completion has wrong slot generation/frame");
            }
            slot.wait_begin_ns = now_ns();
            // Blocking-sync events avoid a high-frequency query loop. These
            // events were recorded after both output D2H copies on each stream.
            check_cuda(cudaEventSynchronize(primary_model_->diagnostic_events[3]),
                      "wait JIT primary output ready");
            check_cuda(cudaEventSynchronize(active12_model_->diagnostic_events[3]),
                      "wait JIT active output ready");
            slot.ready_seen_ns = now_ns();
            slot.primary_cuda = tensor_rt_diagnostic_intervals(*primary_model_);
            slot.active_cuda = tensor_rt_diagnostic_intervals(*active12_model_);
            pair_in_flight = false;
            slot.phase = Phase::ready;
        };
        struct NextPopTrace {
            std::int64_t begin_ns = -1, end_ns = -1;
            bool attempted = false, succeeded = false, stopped = false;
        };
        // publish: postprocess both branches, merge, push one DetectionResult and log the frame;
        // with overlap_next the submit lane submits that slot during this CPU work.
        const auto publish = [&](Slot& slot, int slot_index, const char* phase,
                                 double frozen_delay_ms, Slot* overlap_next = nullptr,
                                 const NextPopTrace* next_pop_trace = nullptr,
                                 const perception::JitSecondChanceTrace* second_pop_trace = nullptr,
                                 const std::function<void()>* after_views = nullptr) {
            const NextPopTrace unattempted_next_pop;
            const auto& next_pop = next_pop_trace ? *next_pop_trace : unattempted_next_pop;
            const perception::JitSecondChanceTrace unattempted_second_pop;
            const auto& second_pop = second_pop_trace ? *second_pop_trace : unattempted_second_pop;
            if (slot.phase != Phase::ready || !slot.input.valid ||
                slot.generation <= last_published_generation) {
                throw std::logic_error("JIT publish has stale/unfinished slot");
            }
            slot.phase = Phase::cpu_read;
            // Copy BEFORE dispatching the next submit callback. During the
            // concurrent region postprocess only reads these immutable views,
            // never a shared ModelState or the next slot's mutable state.
            TensorRtModelState primary_view(*primary_model_);
            TensorRtModelState active_view(*active12_model_);
            primary_view.detection_host = slot.host[2];
            primary_view.prototype_host = slot.host[3];
            active_view.detection_host = slot.host[4];
            active_view.prototype_host = slot.host[5];
            const bool dump_this = post_dump_remaining_ > 0 && std::strcmp(phase, "MEASURING") == 0;
            if (dump_this) {
                // DIAGNOSTIC ONLY: copies happen before the next submit may reuse
                // shared model state; this frame's slot buffers are immutable.
                std::ostringstream json;
                const auto& dual = config_.dual_model();
                json << std::setprecision(17) << "{\"frame_id\":" << slot.input.frame_id
                     << ",\"primary\":{\"det_shape\":[" << primary_view.detection_shape[0] << "," << primary_view.detection_shape[1] << "," << primary_view.detection_shape[2]
                     << "],\"proto_shape\":[" << primary_view.prototype_shape[0] << "," << primary_view.prototype_shape[1] << "," << primary_view.prototype_shape[2] << "," << primary_view.prototype_shape[3]
                     << "],\"det_elements\":" << primary_view.detection_elements << ",\"proto_elements\":" << primary_view.prototype_elements
                     << ",\"lb\":[" << slot.input.letterbox_info.scale << "," << slot.input.letterbox_info.pad_left << "," << slot.input.letterbox_info.pad_top << ","
                     << slot.input.letterbox_info.new_width << "," << slot.input.letterbox_info.new_height << "," << slot.input.letterbox_info.original_width << ","
                     << slot.input.letterbox_info.original_height << "],\"conf\":" << config_.conf_threshold() << ",\"per_class\":{";
                bool first = true;
                for (const auto& [k, v] : config_.per_class_confidence_thresholds()) { json << (first ? "" : ",") << "\"" << k << "\":" << v; first = false; }
                json << "},\"low_floor\":" << config_.low_confidence_floor() << ",\"low_max\":" << config_.low_confidence_max_candidates()
                     << ",\"iou\":" << config_.iou_threshold() << ",\"mask_thr\":" << config_.mask_threshold() << ",\"input\":" << config_.input_size()
                     << "},\"active\":{\"det_shape\":[" << active_view.detection_shape[0] << "," << active_view.detection_shape[1] << "," << active_view.detection_shape[2]
                     << "],\"proto_shape\":[" << active_view.prototype_shape[0] << "," << active_view.prototype_shape[1] << "," << active_view.prototype_shape[2] << "," << active_view.prototype_shape[3]
                     << "],\"det_elements\":" << active_view.detection_elements << ",\"proto_elements\":" << active_view.prototype_elements
                     << ",\"lb\":[" << slot.input.active12_letterbox_info.scale << "," << slot.input.active12_letterbox_info.pad_left << "," << slot.input.active12_letterbox_info.pad_top << ","
                     << slot.input.active12_letterbox_info.new_width << "," << slot.input.active12_letterbox_info.new_height << "," << slot.input.active12_letterbox_info.original_width << ","
                     << slot.input.active12_letterbox_info.original_height << "],\"conf\":" << dual.conf_threshold << ",\"iou\":" << dual.nms_iou_threshold
                     << ",\"mask_thr\":" << dual.mask_threshold << ",\"input\":" << dual.input_size
                     << ",\"input_height\":" << dual.input_height
                     << ",\"mask_impl\":" << static_cast<int>(active12_mask_implementation_) << ",\"binary_impl\":" << static_cast<int>(active12_mask_binary_implementation_)
                     << "}}";
                const std::string text = json.str();
                const std::string path = post_dump_dir_ + "/frame_" + std::to_string(slot.input.frame_id) + ".bin";
                std::ofstream out(path, std::ios::binary);
                const char magic[8] = {'P','D','U','M','P','1',0,0};
                const std::uint64_t length = text.size();
                out.write(magic, 8);
                out.write(reinterpret_cast<const char*>(&length), sizeof(length));
                out.write(text.data(), static_cast<std::streamsize>(text.size()));
                out.write(reinterpret_cast<const char*>(primary_view.detection_host), static_cast<std::streamsize>(primary_view.detection_elements * sizeof(float)));
                out.write(reinterpret_cast<const char*>(primary_view.prototype_host), static_cast<std::streamsize>(primary_view.prototype_elements * sizeof(float)));
                out.write(reinterpret_cast<const char*>(active_view.detection_host), static_cast<std::streamsize>(active_view.detection_elements * sizeof(float)));
                out.write(reinterpret_cast<const char*>(active_view.prototype_host), static_cast<std::streamsize>(active_view.prototype_elements * sizeof(float)));
                if (!out) throw std::runtime_error("post dump write failed: " + path);
            }
            // Empty-pop acquire may submit the next pair only after these
            // immutable views exist (same happens-before as the overlap path).
            if (after_views) (*after_views)();
            const auto frame_id = slot.input.frame_id;
            const auto this_generation = slot.generation;
            std::vector<Detection> primary_detections;
            std::vector<Detection> active_detections;
            double primary_post_ms = 0.0;
            double active_post_ms = 0.0;
            // The work counters of each branch's decode, read on that branch's own thread right
            // after it (yolo::PostWorkCounts; Fable question 13.2).
            yolo::PostWorkCounts primary_work, active_work;
            perception::ExecutionCpuRecord primary_post_cpu, active_post_cpu, post_join_cpu, merge_cpu;
            Active12MaskProfile deferred_mask_profile;
            std::int64_t post_begin_ns = 0;
            std::int64_t post_done_ns = 0;
            const auto caller_cpu_ns = [this]() -> std::int64_t {
                if (!jit_stall_trace_) return -1;
                timespec ts{};
                if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) return -1;
                return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
            };
            std::int64_t cpu_post_begin = -1, cpu_post_done = -1, cpu_push_begin = -1, cpu_pushed = -1;
            std::int64_t overlap_dispatch_ns = 0;
            std::int64_t overlap_join_ns = 0;
            std::uint64_t overlap_next_generation = 0;
            const auto postprocess_current = [&]() {
                cpu_post_begin = caller_cpu_ns();
                post_begin_ns = now_ns();
                perception::run_post_tasks(*parallel_worker, post_role_swap_,
                    [&]() {
                        const auto started = now_ns();
                        perception::ExecutionCpuScope scope(primary_post_cpu, execution_cpu_trace_);
                        primary_detections = postprocess_primary(primary_view, slot.input.letterbox_info);
                        primary_work = yolo::last_post_work_counts();
                        primary_post_ms = ms_between(started, now_ns());
                    },
                    [&]() {
                        const auto started = now_ns();
                        perception::ExecutionCpuScope scope(active_post_cpu, execution_cpu_trace_);
                        active_detections = postprocess_active12(active_view, slot.input.active12_letterbox_info, frame_id,
                            execution_cpu_trace_ ? &deferred_mask_profile : nullptr);
                        active_work = yolo::last_post_work_counts();
                        active_post_ms = ms_between(started, now_ns());
                    }, execution_cpu_trace_ ? &post_join_cpu : nullptr);
                post_done_ns = now_ns();
                cpu_post_done = caller_cpu_ns();
            };
            if (overlap_next) {
                if (!submit_worker || overlap_next == &slot ||
                    overlap_next->phase != Phase::prepared || pair_in_flight) {
                    throw std::logic_error("invalid next-submit/current-post overlap ownership");
                }
                overlap_next_generation = overlap_next->generation;
                overlap_dispatch_ns = now_ns();
                // Both callbacks finish before run returns OR rethrows. A next
                // submit failure cannot leak an apparently successful current
                // publication, and a post failure cannot race stream drain.
                // With a minimum submit interval the next submit includes its
                // wait, so this frame's push comes max(0, wait - post wall) later.
                submit_worker->run([&]() { submit(*overlap_next); }, postprocess_current);
                overlap_join_ns = now_ns();
                if (!pair_in_flight || overlap_next->phase != Phase::in_flight ||
                    overlap_next->generation != overlap_next_generation ||
                    device_generation != overlap_next_generation ||
                    device_frame_id != overlap_next->input.frame_id) {
                    throw std::logic_error("next-submit generation/ownership failed after join");
                }
            } else {
                postprocess_current();
            }
            if (dump_this) {
                const std::string path = post_dump_dir_ + "/frame_" + std::to_string(frame_id) + "_out.txt";
                std::ofstream out(path);
                out << "primary\n" << perception::post_dump::summarize(primary_detections)
                    << "active\n" << perception::post_dump::summarize(active_detections);
                if (!out) throw std::runtime_error("post dump output write failed: " + path);
                --post_dump_remaining_;
            }
            const auto load_begin_ns = now_ns();
            if (jit_probe_log_ == 2) {
                LOG_INFO_M(name_, "PROBE_LOAD mode=jit phase=" << phase << " frame_id="
                    << frame_id << " generation=" << this_generation
                    << " branch=coco80 candidates=-1 detections="
                    << diagnostic_load_fields(primary_detections));
                LOG_INFO_M(name_, "PROBE_LOAD mode=jit phase=" << phase << " frame_id="
                    << frame_id << " generation=" << this_generation
                    << " branch=active12 candidates=-1 detections="
                    << diagnostic_load_fields(active_detections));
            }
            if (perception::inference_profile_enabled()) record_mask_areas(primary_detections, active_detections);
            const auto& dual = config_.dual_model();
            DualModelMergeConfig merge_config;
            merge_config.vehicle_heavy_conflict_iou = dual.conflict_iou;
            const auto merge_begin_ns = now_ns();
            perception::ExecutionCpuScope merge_scope(merge_cpu, execution_cpu_trace_);
            auto merged = merge_dual_model_detections(primary_detections, active_detections, merge_config);
            merge_scope.finish();
            const auto merge_done_ns = now_ns();
            if (slot.generation != this_generation || slot.input.frame_id != frame_id ||
                slot.phase != Phase::cpu_read) {
                throw std::logic_error("JIT slot changed during CPU postprocess");
            }
            DetectionResult result;
            result.detections = std::move(merged.tracker_detections);
            result.map_detections = std::move(merged.map_detections);
            result.cross_model_conflicts = std::move(merged.conflicts);
            result.frame = std::move(slot.input.frame);
            result.depth = std::move(slot.input.depth);
            result.depth_intrinsics = slot.input.depth_intrinsics;
            result.depth_registration = slot.input.depth_registration;
            result.depth_scale = slot.input.depth_scale;
            result.camera_metadata = slot.input.camera_metadata;
            result.frame_id = frame_id;
            result.stamp_sec = slot.input.stamp_sec;
            result.capture_time_ms = slot.input.capture_time_ms;
            result.inference_time_ms = ms_between(slot.submit_ns, merge_done_ns);
            auto timing = slot.input.pipeline_timing;
            timing.preprocess_ms = slot.prepare_ms;
            timing.clone_ms = slot.clone_ms;
            // Host frame residence from submit to observed complete outputs.
            // Includes legal overlap/host observation delay, NOT pure DLA time.
            // This does not include this frame's preparation (it precedes submit).
            timing.execute_pair_ms = ms_between(slot.submit_ns, slot.ready_seen_ns);
            timing.post_wall_ms = ms_between(post_begin_ns, post_done_ns);
            timing.merge_ms = ms_between(merge_begin_ns, merge_done_ns);
            timing.result_push_begin_ns = now_ns();
            cpu_push_begin = caller_cpu_ns();
            timing.valid = timing.capture_epoch_ns > 0 &&
                perception::are_positive_nondecreasing({timing.capture_ready_ns,
                    timing.frame_push_begin_ns, timing.inference_pop_end_ns,
                    slot.prepared_ns, slot.submit_ns, slot.ready_seen_ns,
                    timing.result_push_begin_ns});
            if (!timing.valid) {
                throw std::logic_error("JIT current-frame pipeline timing identity/order invalid");
            }
            result.pipeline_timing = timing;
            result.ready_observed_mono_ns = slot.ready_seen_ns;
            // The phase/delay belong to THIS completed slot's publish call.
            // A later global freeze must not relabel a queued calibration result.
            result.jit_phase = jit_result_phase_from_name(phase);
            result.jit_frozen_delay_ms = frozen_delay_ms;
            // This owner commits the current slot only after usable outputs,
            // full post/merge and identity checks. Calibration frames also
            // complete; failed, prepared-only and drain-only slots never do.
            slot.input.source_binding.complete(result, true, health_completions_);
            output_queue_->push(std::move(result));
            const auto pushed_ns = now_ns();
            cpu_pushed = caller_cpu_ns();
            last_published_generation = this_generation;
            update_heartbeat();
            if (jit_probe_log_ == 1) {
                LOG_INFO_M(name_, "JIT_FRAME phase=" << phase << " frame_id=" << frame_id
                    << " generation=" << this_generation
                    << " next_try_pop_attempted=" << static_cast<int>(next_pop.attempted)
                    << " next_try_pop_succeeded=" << static_cast<int>(next_pop.succeeded)
                    << " next_try_pop_end_ns=" << next_pop.end_ns
                    << " acquire_used=" << static_cast<int>(slot.acquire.used)
                    << " acquire_late_used=" << static_cast<int>(slot.acquire.late_used)
                    << " prepared_ns=" << slot.prepared_ns << " submit_ns=" << slot.submit_ns
                    << " submit_end_ns=" << slot.submit_end_ns << " ready_seen_ns=" << slot.ready_seen_ns
                    << " post_begin_ns=" << post_begin_ns << " post_done_ns=" << post_done_ns
                    << " push_ns=" << timing.result_push_begin_ns << " pushed_ns=" << pushed_ns
                    << " coco_detections=" << primary_detections.size()
                    << " active_detections=" << active_detections.size()
                    << " coco_candidates=" << primary_work.candidates << " coco_selected=" << primary_work.selected
                    << " coco_mask_roi2=" << primary_work.mask_roi2 << " coco_mask_roi=" << primary_work.mask_roi
                    << " coco_mask_full=" << primary_work.mask_full
                    << " active_candidates=" << active_work.candidates << " active_selected=" << active_work.selected
                    << " caller_post_cpu_us=" << (jit_stall_trace_ ? (cpu_post_done - cpu_post_begin) / 1000 : -1)
                    << " caller_merge_cpu_us=" << (jit_stall_trace_ ? (cpu_push_begin - cpu_post_done) / 1000 : -1)
                    << " caller_push_cpu_us=" << (jit_stall_trace_ ? (cpu_pushed - cpu_push_begin) / 1000 : -1)
                    << " pace_wait_ns=" << slot.pace_wait_ns);
            }
            if (jit_probe_log_ == 2)
            LOG_INFO_M(name_, "PROBE_FRAME mode=jit phase=" << phase << " frame_id="
                << frame_id << " generation=" << this_generation << " slot=" << slot_index
                << " post_role_swap=" << static_cast<int>(post_role_swap_)
                << " post_helper_role=" << perception::post_helper_role(post_role_swap_)
                << " post_caller_role=" << perception::post_caller_role(post_role_swap_)
                << " async_submit=" << static_cast<int>(jit_async_submit_)
                << " active_lead_us=" << active_lead_us_
                << " submit_overlap=" << static_cast<int>(overlap_next != nullptr)
                << " overlap_next_generation=" << overlap_next_generation
                << " overlap_dispatch_ns=" << overlap_dispatch_ns
                << " overlap_join_ns=" << overlap_join_ns
                << " overlap_join_wait_ms=" << (overlap_next
                    ? ms_between(post_done_ns, overlap_join_ns) : 0.0)
                << " load_begin_ns=" << load_begin_ns
                << " overlap_next_submit_ns=" << (overlap_next ? overlap_next->submit_ns : 0)
                << " overlap_next_submit_end_ns=" << (overlap_next ? overlap_next->submit_end_ns : 0)
                << " next_try_pop_begin_ns=" << next_pop.begin_ns
                << " next_try_pop_end_ns=" << next_pop.end_ns
                << " next_try_pop_attempted=" << static_cast<int>(next_pop.attempted)
                << " next_try_pop_succeeded=" << static_cast<int>(next_pop.succeeded)
                << " next_try_pop_stopped=" << static_cast<int>(next_pop.stopped)
                << " second_chance_enabled=" << static_cast<int>(jit_second_chance_)
                << " second_try_pop_begin_ns=" << second_pop.begin_ns
                << " second_try_pop_end_ns=" << second_pop.end_ns
                << " second_try_pop_attempted=" << static_cast<int>(second_pop.attempted)
                << " second_try_pop_succeeded=" << static_cast<int>(second_pop.succeeded)
                << " second_try_pop_stopped=" << static_cast<int>(second_pop.stopped)
                << " second_try_pop_discarded_on_stop=" << static_cast<int>(second_pop.discarded_on_stop)
                << " second_try_pop_candidate_frame_id=" << second_pop.candidate_frame_id
                << " second_try_pop_candidate_push_begin_ns=" << second_pop.candidate_push_begin_ns
                << " prepared_ns=" << slot.prepared_ns << " submit_ns=" << slot.submit_ns
                << " submit_end_ns=" << slot.submit_end_ns
                << " wait_begin_ns=" << slot.wait_begin_ns
                << " ready_seen_ns=" << slot.ready_seen_ns
                << " post_begin_ns=" << post_begin_ns << " post_done_ns=" << post_done_ns
                << " push_ns=" << timing.result_push_begin_ns << " pushed_ns=" << pushed_ns
                << " D_ms=" << std::setprecision(12) << frozen_delay_ms
                << " prepare_ms=" << slot.prepare_ms << " clone_ms=" << slot.clone_ms
                << " prepared_age_at_submit_ms=" << ms_between(slot.prepared_ns, slot.submit_ns)
                << " execute_wait_ms=" << ms_between(slot.wait_begin_ns, slot.ready_seen_ns)
                << " submit_to_ready_host_ms=" << timing.execute_pair_ms
                << " ready_to_post_gap_ms=" << ms_between(slot.ready_seen_ns, post_begin_ns)
                << " primary_post_ms=" << primary_post_ms << " active_post_ms=" << active_post_ms
                << " post_wall_ms=" << timing.post_wall_ms << " merge_ms=" << timing.merge_ms
                << " coco_detections=" << primary_detections.size() << " active_detections=" << active_detections.size()
                << " coco_candidates=" << primary_work.candidates << " coco_selected=" << primary_work.selected
                << " coco_mask_roi2=" << primary_work.mask_roi2 << " coco_mask_roi=" << primary_work.mask_roi
                << " coco_mask_full=" << primary_work.mask_full
                << " active_candidates=" << active_work.candidates << " active_selected=" << active_work.selected
                << " load_log_ms=" << ms_between(load_begin_ns, merge_begin_ns)
                << " primary_cuda_h2d_ms=" << slot.primary_cuda[0]
                << " primary_cuda_exec_span_ms=" << slot.primary_cuda[1]
                << " primary_cuda_d2h_ms=" << slot.primary_cuda[2]
                << " primary_cuda_total_ms=" << slot.primary_cuda[3]
                << " active_cuda_h2d_ms=" << slot.active_cuda[0]
                << " active_cuda_exec_span_ms=" << slot.active_cuda[1]
                << " active_cuda_d2h_ms=" << slot.active_cuda[2]
                << " active_cuda_total_ms=" << slot.active_cuda[3]
                << fresh_bind_fields(slot.fresh_bind, frozen_delay_ms < 0.0)
                << " empty_acquire_enabled=" << static_cast<int>(jit_empty_acquire_)
                << " acquire_used=" << static_cast<int>(slot.acquire.used)
                << " acquire_late_used=" << static_cast<int>(slot.acquire.late_used)
                << " acquire_late_begin_ns=" << slot.acquire.late_begin_ns
                << " acquire_late_pop_end_ns=" << slot.acquire.late_pop_end_ns
                << " acquire_start_ns=" << slot.acquire.start_ns
                << " acquire_pop_end_ns=" << slot.acquire.pop_end_ns
                << " acquire_prepared_ns=" << slot.acquire.prepared_ns
                << " acquire_device_free_ns=" << slot.acquire.device_free_ns
                << " acquire_submit_ns=" << slot.acquire.submit_ns);
            if (execution_cpu_trace_) {
                // All prepare/post/mask callbacks have joined. These fixed
                // records do not log from either actual execution lane.
                const auto emit_cpu = [&](const char* stage, const perception::ExecutionCpuRecord& record) {
                    LOG_INFO_M(name_, "EXECUTION_CPU_STAGE frame_id=" << frame_id
                        << " generation=" << this_generation << " phase=" << phase
                        << " stage=" << stage << record);
                };
                emit_cpu("prepare_primary", slot.prepare_primary_cpu);
                emit_cpu("prepare_active", slot.prepare_active_cpu);
                emit_cpu("prepare_join_wait", slot.prepare_join_cpu);
                emit_cpu("prepare_clone", slot.clone_cpu);
                emit_cpu("post_primary", primary_post_cpu);
                emit_cpu("post_active", active_post_cpu);
                emit_cpu("post_join_wait", post_join_cpu);
                emit_cpu("merge", merge_cpu);
                log_active12_mask_profile(deferred_mask_profile, frame_id);
            }
            slot.input = PreparedInput{};
            slot.acquire = {};
            slot.phase = Phase::empty;
        };

        // Calibration: strictly serial frames until the sample count is reached (see the header).
        perception::JitScheduleCalibration calibration;
        while (running_.load() && !calibration.frozen()) {
            FrameData frame;
            if (!input_queue_->pop(frame, std::chrono::milliseconds(100))) {
                update_heartbeat();
                continue;
            }
            prepare(slots[0], std::move(frame));
            submit(slots[0]);
            wait_ready(slots[0]);
            const auto pair_ms = ms_between(slots[0].submit_ns, slots[0].ready_seen_ns);
            const auto prep_ms = slots[0].prepare_ms + slots[0].clone_ms;
            const char* phase = calibration.seen() < calibration.warmup_count
                ? "CALIBRATION_WARMUP" : "CALIBRATING";
            publish(slots[0], 0, phase, -1.0);
            calibration.add(prep_ms, pair_ms);
        }
        if (calibration.frozen() && running_.load()) {
            const double observed_delay_ms = calibration.delay_ms();
            const double delay_ms = jit_fixed_bind_delay_ms_ >= 0.0
                ? jit_fixed_bind_delay_ms_ : observed_delay_ms;
            if (delay_ms > static_cast<double>(config_.timeout_ms())) {
                throw std::runtime_error("calibrated JIT delay exceeds configured inference timeout");
            }
            // Publish readiness only after the actual selected delay passed
            // validation. The measured schedule still uses the original double.
            jit_frozen_delay_us_.store(static_cast<std::int64_t>(std::llround(delay_ms * 1000.0)),
                std::memory_order_release);
            jit_calibration_complete_.store(true, std::memory_order_release);
            // One identity sample at freeze, not an extra syscall per frame.
            std::int64_t producer_linux_tid = -1;
#ifdef __linux__
            producer_linux_tid = static_cast<std::int64_t>(::syscall(SYS_gettid));
#endif
            LOG_WARN_M(name_, "D_FROZEN mode=jit D_ms=" << std::setprecision(12) << delay_ms
                << " producer_linux_tid=" << producer_linux_tid
                << " second_chance_enabled=" << static_cast<int>(jit_second_chance_)
                << " fixed_D=" << static_cast<int>(jit_fixed_bind_delay_ms_ >= 0.0)
                << " configured_D_ms=" << jit_fixed_bind_delay_ms_
                << " observed_D_ms=" << observed_delay_ms
                << " async_submit=" << static_cast<int>(jit_async_submit_)
                << " active_lead_us=" << active_lead_us_
                << " host_pair_median_ms=" << calibration.pair_median_ms()
                << " prepare_clone_p99_ms=" << calibration.prepare_p99_ms()
                << " margin_ms=2 warmup=120 samples=300"
                << " observed_formula=max(0,host_pair_median-prepare_clone_p99-2)"
                << " selection=" << (jit_fixed_bind_delay_ms_ >= 0.0 ? "fixed_env" : "calibrated")
                << " fresh_bind_enabled=" << static_cast<int>(jit_fresh_bind_)
                << " fresh_bind_budget_ns=" << perception::JIT_FRESH_BIND_BUDGET_NS
                << " fresh_bind_min_queue_age_ns=" << perception::JIT_FRESH_BIND_MIN_QUEUE_AGE_NS
                << " fresh_bind_budget_kind=requested_not_hard_realtime"
                << " device_clock_subtraction=0 phase=MEASURING");
            // Steady loop. current = the slot whose pair is on the device, -1 = none (start, or the
            // last next-frame take found nothing and the loop has to prime again).
            int current = -1;
            while (running_.load()) {
                if (current < 0) {
                    FrameData frame;
                    if (!input_queue_->pop(frame, std::chrono::milliseconds(100))) {
                        update_heartbeat();
                        continue;
                    }
                    current = 0;
                    prepare(slots[current], std::move(frame));
                    submit(slots[current]);
                }
                Slot& live = slots[current];
                const auto target_ns = live.submit_ns + static_cast<std::int64_t>(delay_ms * 1000000.0);
                const auto observed_ns = now_ns();
                if (target_ns > observed_ns) {
                    // One bounded sleep, no periodic polling. A stop request is
                    // checked before taking or submitting any next frame.
                    std::this_thread::sleep_for(std::chrono::nanoseconds(target_ns - observed_ns));
                }
                const int next = 1 - current;
                if (slots[next].phase != Phase::empty) {
                    throw std::logic_error("JIT next slot is still owned");
                }
                FrameData next_frame;
                NextPopTrace next_pop;
                perception::JitSecondChance second_chance(live.generation, live.input.frame_id);
                if (jit_fresh_bind_ && running_.load()) {
                    FreshBindTrace trace;
                    trace.normal_next = true;
                    trace.deadline_ns = perception::jit_fresh_bind_deadline_ns(target_ns);
                    // Convert only a remaining duration between clock APIs.
                    // Never reinterpret CLOCK_MONOTONIC nanoseconds as a
                    // steady_clock epoch. Sampling steady first is conservative.
                    const auto steady_anchor = std::chrono::steady_clock::now();
                    trace.wait_start_ns = now_ns();
                    const auto remaining_ns = std::max<std::int64_t>(
                        0, trace.deadline_ns - trace.wait_start_ns);
                    const auto steady_deadline = steady_anchor + std::chrono::nanoseconds(remaining_ns);
                    bool eligible = false;
                    const bool popped = input_queue_->try_pop_after_update_until(
                        next_frame, steady_deadline,
                        [&](const FrameData& candidate) {
                            trace.initial_frame_valid = true;
                            trace.initial_frame_id = candidate.frame_id;
                            trace.initial_push_begin_ns = candidate.pipeline_timing.frame_push_begin_ns;
                            trace.decision_ns = now_ns();
                            eligible = perception::jit_fresh_bind_should_wait(
                                trace.decision_ns, trace.deadline_ns, trace.initial_push_begin_ns);
                            return eligible && running_.load();
                        },
                        [](const FrameData& candidate) { return candidate.frame_id; }, trace.wait);
                    trace.wait_end_ns = now_ns();
                    trace.deadline_overrun_ns = std::max<std::int64_t>(
                        0, trace.wait_end_ns - trace.deadline_ns);
                    if (trace.wait.cleared) trace.reason = "cleared";
                    else if (!running_.load()) trace.reason = "stopped";
                    else if (!popped) trace.reason = "empty";
                    else if (trace.wait.updated) trace.reason = "updated";
                    else if (trace.wait.wait_requested) trace.reason = "timeout";
                    else if (trace.decision_ns >= trace.deadline_ns || eligible) trace.reason = "deadline_expired";
                    else trace.reason = "not_old";
                    if (popped && running_.load()) {
                        prepare(slots[next], std::move(next_frame));
                        slots[next].fresh_bind = trace;
                    } else {
                        if (!trace.wait.cleared && !running_.load()) trace.reason = "stopped";
                        LOG_INFO_M(name_, "JIT_FRESH_BIND_NO_FRAME parent_frame_id="
                            << live.input.frame_id << fresh_bind_fields(trace));
                    }
                } else if (!jit_fresh_bind_) {
                    // Preserve the original one running check and one try_pop.
                    // stopped means that check suppressed the attempt, not a
                    // later global-state sample. Other branches keep -1 times.
                    next_pop.stopped = !running_.load();
                    if (!next_pop.stopped) {
                        next_pop.attempted = true;
                        next_pop.begin_ns = now_ns();
                        next_pop.succeeded = input_queue_->try_pop(next_frame);
                        next_pop.end_ns = now_ns();
                        if (next_pop.succeeded) {
                            prepare(slots[next], std::move(next_frame));
                            slots[next].fresh_bind.normal_next = true;
                        } else if (jit_empty_wait_ready_ && running_.load()) {
                            // Bounded by the in-flight pair's own completion:
                            // non-blocking CUDA event queries, 1 ms queue waits.
                            const auto late_begin_ns = now_ns();
                            bool late = false;
                            while (running_.load()) {
                                if (input_queue_->pop(next_frame, std::chrono::milliseconds(1))) {
                                    late = true;
                                    break;
                                }
                                const auto primary_state = cudaEventQuery(primary_model_->diagnostic_events[3]);
                                const auto active_state = cudaEventQuery(active12_model_->diagnostic_events[3]);
                                if (primary_state != cudaSuccess && primary_state != cudaErrorNotReady) {
                                    check_cuda(primary_state, "query JIT primary output ready");
                                }
                                if (active_state != cudaSuccess && active_state != cudaErrorNotReady) {
                                    check_cuda(active_state, "query JIT active output ready");
                                }
                                if (primary_state == cudaSuccess && active_state == cudaSuccess) break;
                            }
                            if (late) {
                                const auto late_pop_end_ns = now_ns();
                                prepare(slots[next], std::move(next_frame));
                                slots[next].fresh_bind.normal_next = true;
                                slots[next].acquire.late_used = true;
                                slots[next].acquire.late_begin_ns = late_begin_ns;
                                slots[next].acquire.late_pop_end_ns = late_pop_end_ns;
                            }
                        }
                    }
                }
                // Empty JIT pop: instead of a serial reprime after this publish,
                // the submit lane waits for the next arriving frame, prepares it
                // serially and submits as soon as the device pair is free.
                bool acquire_started = false;
                if (jit_empty_acquire_ && !jit_fresh_bind_ && next_pop.attempted && !next_pop.succeeded &&
                    slots[next].phase == Phase::empty && running_.load() && submit_worker) {
                    {
                        std::lock_guard<std::mutex> lock(acquire_mutex);
                        acquire_device_free = false;
                    }
                    Slot& target = slots[next];
                    target.acquire = {};
                    target.acquire.used = true;
                    target.acquire.start_ns = now_ns();
                    submit_worker->start([&, next]() {
                        Slot& acquired = slots[next];
                        FrameData frame;
                        bool got = false;
                        while (!acquire_abort.load(std::memory_order_acquire) && running_.load()) {
                            if (input_queue_->pop(frame, std::chrono::milliseconds(5))) {
                                got = true;
                                break;
                            }
                        }
                        if (!got) return;
                        acquired.acquire.pop_end_ns = now_ns();
                        prepare(acquired, std::move(frame), true);
                        acquired.acquire.prepared_ns = acquired.prepared_ns;
                        {
                            std::unique_lock<std::mutex> lock(acquire_mutex);
                            acquire_cv.wait(lock, [&]() {
                                return acquire_device_free || acquire_abort.load(std::memory_order_acquire);
                            });
                        }
                        if (acquire_abort.load(std::memory_order_acquire) || !running_.load()) return;
                        acquired.acquire.device_free_ns = now_ns();
                        submit(acquired);
                        acquired.acquire.submit_ns = acquired.submit_ns;
                    });
                    acquire_started = true;
                }
                wait_ready(live);
                if (acquire_started) {
                    const std::function<void()> release_device = [&]() {
                        std::lock_guard<std::mutex> lock(acquire_mutex);
                        acquire_device_free = true;
                        acquire_cv.notify_all();
                    };
                    publish(live, current, "MEASURING", delay_ms, nullptr, &next_pop,
                            nullptr, &release_device);
                    submit_worker->join();
                    current = slots[next].phase == Phase::in_flight ? next : -1;
                    continue;
                }
                const perception::JitSecondChanceGate second_gate{
                    jit_second_chance_, true, jit_async_submit_, jit_fresh_bind_,
                    next_pop.attempted, next_pop.succeeded, next_pop.stopped,
                    live.phase == Phase::ready && !pair_in_flight, slots[next].phase == Phase::empty,
                    live.generation, live.input.frame_id};
                second_chance.attempt(second_gate, next_frame,
                    [&]() { return running_.load(); }, now_ns,
                    [&](FrameData& candidate) { return input_queue_->try_pop(candidate); },
                    [&](FrameData&& candidate) {
                        prepare(slots[next], std::move(candidate));
                        slots[next].fresh_bind.normal_next = true;
                    });
                const bool submit_next = running_.load() && slots[next].phase == Phase::prepared;
                if (submit_next && !jit_async_submit_) {
                    submit(slots[next]);
                }
                publish(live, current, "MEASURING", delay_ms,
                        submit_next && jit_async_submit_ ? &slots[next] : nullptr, &next_pop,
                        &second_chance.trace());
                // publish's task-pair join is the happens-before boundary for
                // every read of next.phase and its host submission timestamps.
                current = slots[next].phase == Phase::in_flight ? next : -1;
            }
        }
        // A normal stop may leave the next frame running or prepared. It is not
        // published as a fabricated/partial result; drain before releasing it.
        abort_acquire();
        if (submit_worker) submit_worker->stop();
        drain_tensor_rt_no_throw(*primary_model_);
        drain_tensor_rt_no_throw(*active12_model_);
        LOG_INFO_M(name_, "JIT_PROBE_DRAINED admitted=" << generation
            << " published_generation=" << last_published_generation);
        log_pace_summary();
    } catch (const std::exception& error) {
        abort_acquire();
        if (submit_worker) submit_worker->stop();
        if (primary_model_) drain_tensor_rt_no_throw(*primary_model_);
        if (active12_model_) drain_tensor_rt_no_throw(*active12_model_);
        schedule_probe_failed_.store(true);
        running_.store(false);
        LOG_ERROR_M(name_, "JIT_PROBE_FATAL mode=jit generation=" << generation
            << " device_generation=" << device_generation << " frame_id=" << device_frame_id
            << " error=" << error.what() << " reload=0 publish_failed_frame=0");
        log_pace_summary();
    } catch (...) {
        abort_acquire();
        if (submit_worker) submit_worker->stop();
        if (primary_model_) drain_tensor_rt_no_throw(*primary_model_);
        if (active12_model_) drain_tensor_rt_no_throw(*active12_model_);
        schedule_probe_failed_.store(true);
        running_.store(false);
        LOG_ERROR_M(name_, "JIT_PROBE_FATAL mode=jit unknown exception reload=0");
        log_pace_summary();
    }
}
#endif

// Both engine contracts come from the configuration and are checked against the sealed tensor
// and class contract before any engine is touched; the two branches must not share files.
bool InferenceModule::load_model() {
#ifdef HAS_TENSORRT
    try {
        release_tensor_rt();

        TensorRtModelContract primary_contract;
        primary_contract.label = "coco80";
        primary_contract.onnx_path = config_.model_path();
        primary_contract.engine_path = config_.trt_engine_path();
        primary_contract.input_size = config_.input_size();
        primary_contract.input_height = config_.input_size();
        primary_contract.detection_channels = PRIMARY_DETECTION_CHANNELS;
        primary_contract.detection_candidates = PRIMARY_DETECTION_CANDIDATES;
        primary_contract.mask_size = PRIMARY_MASK_SIZE;
        primary_contract.mask_height = PRIMARY_MASK_SIZE;
        primary_contract.workspace_mb = config_.trt_workspace_mb();
        primary_contract.use_fp16 = config_.use_fp16();
        primary_contract.use_int8 = config_.use_int8();
        primary_contract.int8_build = config_.int8_build();
        primary_contract.use_dla = config_.use_dla();
        primary_contract.dla_core = config_.dla_core();
        // This is the pre-existing production behavior: DLA builds always
        // allowed unsupported layers to fall back to GPU.
        primary_contract.allow_gpu_fallback = config_.use_dla();

        if (primary_contract.input_size != PRIMARY_INPUT_SIZE) {
            throw std::runtime_error(
                "dual-model coco80 input contract requires [1,3,640,640], configured input_size=" +
                std::to_string(primary_contract.input_size));
        }
        if (!initialize_tensor_rt_model(primary_contract, primary_model_)) {
            return false;
        }

        const auto& dual = config_.dual_model();
        // dual_model.input_shape selects one of the two sealed active12 inputs (postprocess.cpp,
        // sealed_active12_spec): the 896 x 896 square or its 896 x 608 window (rows 160..767). The
        // engine contract, preprocessing and decoding all follow that one sealed ModelSpec.
        const auto& active_spec = active12_spec();
        const bool active_square = dual.input_height == dual.input_size;
        if (dual.input_size != ACTIVE12_INPUT_SIZE || active_spec.input_width != dual.input_size ||
            active_spec.input_height != dual.input_height ||
            active_spec.expected_detection_channels != ACTIVE12_DETECTION_CHANNELS ||
            active_spec.expected_proto_width != ACTIVE12_MASK_SIZE ||
            (active_square && (active_spec.expected_num_boxes != ACTIVE12_DETECTION_CANDIDATES ||
                               active_spec.expected_proto_height != ACTIVE12_MASK_SIZE))) {
            throw std::runtime_error(
                "active12 sealed tensor contract must be input=[1,3,896,896], output0=[1,48,16464], "
                "output1=[1,32,224,224] or input=[1,3,608,896], output0=[1,48,11172], output1=[1,32,152,224]");
        }
        if (dual.local_class_names != active_spec.class_names ||
            dual.local_to_global != active_spec.local_to_global_class_ids ||
            dual.max_detections != active_spec.max_detections) {
            throw std::runtime_error(
                "active12 runtime class mapping/max_detections differs from sealed ModelSpec");
        }

        const auto normalized_path = [](const std::string& path) {
            return std::filesystem::absolute(std::filesystem::path(path))
                .lexically_normal()
                .string();
        };
        if (normalized_path(primary_contract.engine_path) ==
            normalized_path(dual.active_engine_path)) {
            throw std::runtime_error(
                "coco80 and active12 must not share a TensorRT engine path");
        }
        if (normalized_path(primary_contract.onnx_path) ==
            normalized_path(dual.active_model_path)) {
            throw std::runtime_error(
                "coco80 and active12 must not share an ONNX model path");
        }

        TensorRtModelContract active_contract;
        active_contract.label = "active12";
        active_contract.onnx_path = dual.active_model_path;
        active_contract.engine_path = dual.active_engine_path;
        active_contract.input_size = active_spec.input_width;
        active_contract.input_height = active_spec.input_height;
        active_contract.detection_channels = ACTIVE12_DETECTION_CHANNELS;
        active_contract.detection_candidates = static_cast<int>(active_spec.expected_num_boxes);
        active_contract.mask_size = active_spec.expected_proto_width;
        active_contract.mask_height = active_spec.expected_proto_height;
        active_contract.workspace_mb = dual.workspace_mb;
        active_contract.use_fp16 = dual.use_fp16;
        active_contract.use_int8 = dual.int8.enabled;
        active_contract.int8_build = dual.int8;
        active_contract.use_dla = dual.use_dla;
        active_contract.dla_core = dual.dla_core;
        active_contract.allow_gpu_fallback = dual.allow_gpu_fallback;
        if (!initialize_tensor_rt_model(active_contract, active12_model_)) {
            return false;
        }

        ensure_pinned_input_buffers();

        LOG_INFO_M(
            name_,
            "TensorRT model set ready, dual_enabled=" << dual_model_enabled_
                                                      << " workers=1 queues=1");
        return true;
    } catch (const std::exception& ex) {
        LOG_ERROR_M(name_, "failed to initialize TensorRT backend: " << ex.what());
        release_tensor_rt();
        return false;
    }
#else
    if (model_load_policy_ != ModelLoadPolicy::AllowBuild) {
        LOG_ERROR_M(name_, "ExistingEnginesOnly requires the actual TensorRT backend");
        return false;
    }
    LOG_WARN_M(name_, "built without TensorRT; inference will return empty detections");
    return true;
#endif
}

// The sealed active12 ModelSpec of the configured dual_model.input_shape (896 x 896 or the
// 896 x 608 window); throws for any other shape (the configuration already refuses those).
const ModelSpec& InferenceModule::active12_spec() const {
    return active12_seg_model_spec(
        ModelInputShape{config_.dual_model().input_size, config_.dual_model().input_height});
}

// Allocating form for the build without TensorRT (the TensorRT path writes the pinned buffer).
std::vector<float> InferenceModule::preprocess_active12(const cv::Mat& image, LetterboxInfo& info) const {
    const auto& spec = active12_spec();
    std::vector<float> data(static_cast<std::size_t>(3) * static_cast<std::size_t>(spec.input_width) *
                            static_cast<std::size_t>(spec.input_height));
    preprocess_window_into(image, ModelInputShape{spec.input_width, spec.input_height}, spec.input_window_top, info,
                           data.data(), data.size());
    return data;
}

#ifdef HAS_TENSORRT
// Load one engine (build it only under AllowBuild), select its DLA core and bind its buffers.
bool InferenceModule::initialize_tensor_rt_model(
    const TensorRtModelContract& contract,
    std::unique_ptr<TensorRtModelState>& state) {
    release_tensor_rt_model(state);
    state = std::make_unique<TensorRtModelState>(contract);

    try {
        LOG_INFO_M(
            name_,
            "TensorRT contract model=" << contract.label
                                        << " onnx=" << contract.onnx_path
                                        << " engine=" << contract.engine_path
                                        << " input=[1,3," << contract.input_height << ","
                                        << contract.input_size << "]"
                                        << " output0=[1," << contract.detection_channels << ","
                                        << contract.detection_candidates << "]"
                                        << " output1=[1," << contract.mask_channels << ","
                                        << contract.mask_height << "," << contract.mask_size << "]"
                                        << " requested_dla_core="
                                        << (contract.use_dla ? contract.dla_core : -1)
                                        << " gpu_fallback=" << contract.allow_gpu_fallback);

        if (!std::filesystem::exists(contract.engine_path)) {
            if (model_load_policy_ != ModelLoadPolicy::AllowBuild) {
                throw std::runtime_error("ExistingEnginesOnly forbids implicit TensorRT build: " +
                                         contract.engine_path);
            }
            LOG_INFO_M(
                name_,
                "TensorRT engine not found for model=" << contract.label
                                                       << "; building from ONNX="
                                                       << contract.onnx_path);
            if (!build_serialized_engine(contract)) {
                return false;
            }
        } else {
            LOG_INFO_M(
                name_,
                "loading cached TensorRT engine model=" << contract.label
                                                        << " engine=" << contract.engine_path);
        }

        const std::vector<char> engine_bytes = read_binary_file(contract.engine_path);
        state->runtime = nvinfer1::createInferRuntime(TRT_LOGGER);
        if (state->runtime == nullptr) {
            throw std::runtime_error(
                "createInferRuntime returned null for model=" + contract.label);
        }
        if (contract.use_dla) {
            const int runtime_dla_cores = state->runtime->getNbDLACores();
            if (contract.dla_core < 0 || contract.dla_core >= runtime_dla_cores) {
                throw std::runtime_error(
                    "model=" + contract.label + " requested runtime DLA core " +
                    std::to_string(contract.dla_core) +
                    " is unavailable; runtime detected cores=" +
                    std::to_string(runtime_dla_cores));
            }
            // Builder placement determines which layers use DLA; IRuntime
            // selects the physical DLA instance when the serialized plan is
            // deserialized. These are deliberately independent per branch:
            // production coco80 normally uses DLA0, active12 uses DLA1.
            state->runtime->setDLACore(contract.dla_core);
            if (state->runtime->getDLACore() != contract.dla_core) {
                throw std::runtime_error(
                    "TensorRT runtime rejected DLA core selection for model=" +
                    contract.label);
            }
        }
        LOG_INFO_M(
            name_,
            "TensorRT runtime model=" << contract.label
                                      << " selected_dla_core="
                                      << state->runtime->getDLACore()
                                      << " available_dla_cores="
                                      << state->runtime->getNbDLACores());
        check_cuda(cudaGetDevice(&state->cuda_device_id), "record TensorRT creation CUDA device");
        state->engine =
            state->runtime->deserializeCudaEngine(engine_bytes.data(), engine_bytes.size());
        if (state->engine == nullptr) {
            throw std::runtime_error(
                "failed to deserialize TensorRT engine model=" + contract.label +
                " path=" + contract.engine_path);
        }
        state->context = state->engine->createExecutionContext();
        if (state->context == nullptr) {
            throw std::runtime_error(
                "createExecutionContext returned null for model=" + contract.label);
        }
        if (!configure_tensor_buffers(*state)) {
            return false;
        }
        state->loaded = true;
        LOG_INFO_M(
            name_,
            "TensorRT engine ready model=" << contract.label
                                           << " engine=" << contract.engine_path);
        return true;
    } catch (...) {
        release_tensor_rt_model(state);
        throw;
    }
}

// Engine build from ONNX, used by the engine build tool; the service never gets here. DLA forces
// FP16; INT8 applies the configured INT8 plan (apply_int8_plan above).
bool InferenceModule::build_serialized_engine(const TensorRtModelContract& contract) {
    // Defense in depth: no alternate/reload call may bypass the missing-file
    // policy check above. The default retains the legacy build behavior.
    if (model_load_policy_ != ModelLoadPolicy::AllowBuild) {
        throw std::runtime_error("TensorRT engine construction forbidden by load policy");
    }
    if (!std::filesystem::exists(contract.onnx_path)) {
        throw std::runtime_error(
            "ONNX model not found for model=" + contract.label +
            ": " + contract.onnx_path);
    }

    TensorRtUniquePtr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(TRT_LOGGER));
    if (!builder) {
        throw std::runtime_error("createInferBuilder returned null");
    }
    // Explicit batch is mandatory and implicit in TensorRT 10. Referencing the
    // legacy flag there emits a deprecation warning (and breaks -Werror builds).
#if NV_TENSORRT_MAJOR >= 10
    const std::uint32_t network_flags = 0U;
#else
    const std::uint32_t network_flags =
        1U << static_cast<std::uint32_t>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#endif
    TensorRtUniquePtr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(network_flags));
    TensorRtUniquePtr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, TRT_LOGGER));
    TensorRtUniquePtr<nvinfer1::IBuilderConfig> builder_config(builder->createBuilderConfig());
    if (!network || !parser || !builder_config) {
        throw std::runtime_error("failed to create TensorRT ONNX build objects");
    }
    if (!parser->parseFromFile(
            contract.onnx_path.c_str(),
            static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        std::ostringstream errors;
        errors << "ONNX parse failed for model=" << contract.label;
        for (int i = 0; i < parser->getNbErrors(); ++i) {
            errors << "; " << parser->getError(i)->desc();
        }
        throw std::runtime_error(errors.str());
    }

    if (network->getNbInputs() != 1 || network->getNbOutputs() != 2) {
        throw std::runtime_error(
            "ONNX model=" + contract.label +
            " must expose exactly one input and two outputs");
    }
    const nvinfer1::Dims expected_input_dims =
        nvinfer1::Dims4(1, 3, contract.input_height, contract.input_size);
    nvinfer1::ITensor* network_input = network->getInput(0);
    if (network_input == nullptr ||
        network_input->getType() != nvinfer1::DataType::kFLOAT ||
        !shape_equals(network_input->getDimensions(), shape_vector(expected_input_dims))) {
        throw std::runtime_error(
            "ONNX model=" + contract.label +
            " input must be FLOAT [1,3," + std::to_string(contract.input_height) +
            "," + std::to_string(contract.input_size) + "]");
    }

    const std::vector<int64_t> expected_detection = {
        1, contract.detection_channels, contract.detection_candidates};
    const std::vector<int64_t> expected_prototype = {
        1, contract.mask_channels, contract.mask_height, contract.mask_size};
    bool found_detection = false;
    bool found_prototype = false;
    for (int i = 0; i < network->getNbOutputs(); ++i) {
        nvinfer1::ITensor* output = network->getOutput(i);
        if (output == nullptr || output->getType() != nvinfer1::DataType::kFLOAT) {
            throw std::runtime_error(
                "ONNX output must be FLOAT for model=" + contract.label);
        }
        const auto dims = output->getDimensions();
        LOG_INFO_M(
            name_,
            "ONNX tensor model=" << contract.label
                                  << " name=" << (output->getName() ? output->getName() : "<null>")
                                  << " dtype=" << data_type_to_string(output->getType())
                                  << " dims=" << dims_to_string(dims));
        found_detection = found_detection || shape_equals(dims, expected_detection);
        found_prototype = found_prototype || shape_equals(dims, expected_prototype);
    }
    if (!found_detection || !found_prototype) {
        throw std::runtime_error(
            "ONNX output contract mismatch for model=" + contract.label);
    }

    const int workspace_mb = std::max(1, contract.workspace_mb);
    builder_config->setMemoryPoolLimit(
        nvinfer1::MemoryPoolType::kWORKSPACE,
        static_cast<std::size_t>(workspace_mb) * 1024U * 1024U);
    if (contract.use_fp16 || contract.use_dla) {
        builder_config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    if (contract.use_int8) {
        builder_config->setFlag(nvinfer1::BuilderFlag::kINT8);
        apply_int8_plan(*network, *builder_config, contract.int8_build, contract.label, name_);
    }
    if (contract.use_dla) {
        const int dla_cores = builder->getNbDLACores();
        if (contract.dla_core < 0 || contract.dla_core >= dla_cores) {
            throw std::runtime_error(
                "model=" + contract.label + " requested DLA core " +
                std::to_string(contract.dla_core) +
                " is unavailable; detected cores=" + std::to_string(dla_cores));
        }
        builder_config->setDefaultDeviceType(nvinfer1::DeviceType::kDLA);
        builder_config->setDLACore(contract.dla_core);
        if (contract.allow_gpu_fallback) {
            builder_config->setFlag(nvinfer1::BuilderFlag::kGPU_FALLBACK);
        }
        LOG_INFO_M(
            name_,
            "building TensorRT engine model=" << contract.label
                                               << " DLA core=" << contract.dla_core
                                               << " allowGPUFallback="
                                               << contract.allow_gpu_fallback
                                               << " fp16=1 int8=" << (contract.use_int8 ? 1 : 0));
    }

    TensorRtUniquePtr<nvinfer1::IHostMemory> serialized(
        builder->buildSerializedNetwork(*network, *builder_config));
    if (!serialized) {
        throw std::runtime_error(
            "TensorRT buildSerializedNetwork failed for model=" + contract.label);
    }
    write_binary_file(contract.engine_path, serialized->data(), serialized->size());
    LOG_INFO_M(
        name_,
        "serialized TensorRT engine model=" << contract.label
                                            << " engine=" << contract.engine_path
                                            << " bytes=" << serialized->size());
    return true;
}

// The engine's tensors must match the contract (one FLOAT input, channel-major detections, the
// prototypes); then device buffers, pinned host outputs and the four stage events (probe/JIT
// modes only) are created and the addresses bound.
bool InferenceModule::configure_tensor_buffers(TensorRtModelState& state) {
    const auto& contract = state.contract;
    const std::vector<int64_t> expected_input = {
        1, 3, contract.input_height, contract.input_size};
    const std::vector<int64_t> expected_detection = {
        1, contract.detection_channels, contract.detection_candidates};
    const std::vector<int64_t> transposed_detection = {
        1, contract.detection_candidates, contract.detection_channels};
    const std::vector<int64_t> expected_prototype = {
        1, contract.mask_channels, contract.mask_height, contract.mask_size};

    int input_count = 0;
    for (int i = 0; i < state.engine->getNbIOTensors(); ++i) {
        const char* name = state.engine->getIOTensorName(i);
        if (name == nullptr ||
            state.engine->getTensorIOMode(name) != nvinfer1::TensorIOMode::kINPUT) {
            continue;
        }
        ++input_count;
        if (input_count > 1) {
            throw std::runtime_error(
                "multiple TensorRT inputs are not supported for model=" + contract.label);
        }
        state.input_tensor_name = name;
        const nvinfer1::Dims engine_dims = state.engine->getTensorShape(name);
        if (has_dynamic_dimension(engine_dims)) {
            nvinfer1::Dims4 runtime_dims(
                1, 3, contract.input_height, contract.input_size);
            if (!state.context->setInputShape(name, runtime_dims)) {
                throw std::runtime_error(
                    "failed to set dynamic input shape for model=" + contract.label +
                    " tensor=" + state.input_tensor_name);
            }
        }
    }
    if (state.input_tensor_name.empty()) {
        throw std::runtime_error(
            "TensorRT engine has no input tensor for model=" + contract.label);
    }

    int output_count = 0;
    for (int i = 0; i < state.engine->getNbIOTensors(); ++i) {
        const char* name = state.engine->getIOTensorName(i);
        if (name == nullptr) {
            throw std::runtime_error("TensorRT returned a null I/O tensor name");
        }
        const nvinfer1::TensorIOMode mode = state.engine->getTensorIOMode(name);
        const nvinfer1::Dims dims = state.context->getTensorShape(name);
        const nvinfer1::DataType data_type = state.engine->getTensorDataType(name);
        LOG_INFO_M(
            name_,
            "TensorRT binding model=" << contract.label
                                       << " engine=" << contract.engine_path
                                       << " name=" << name
                                       << " mode="
                                       << (mode == nvinfer1::TensorIOMode::kINPUT
                                               ? "INPUT"
                                               : "OUTPUT")
                                       << " dtype=" << data_type_to_string(data_type)
                                       << " dims=" << dims_to_string(dims));

        if (data_type != nvinfer1::DataType::kFLOAT) {
            throw std::runtime_error(
                "TensorRT I/O tensor model=" + contract.label +
                " name=" + std::string(name) +
                " must be FLOAT at the exchange boundary");
        }
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            if (!shape_equals(dims, expected_input)) {
                throw std::runtime_error(
                    "unexpected TensorRT input shape model=" + contract.label +
                    " actual=" + dims_to_string(dims) +
                    " expected=[1,3," + std::to_string(contract.input_height) +
                    "," + std::to_string(contract.input_size) + "]");
            }
        } else if (shape_equals(dims, expected_detection)) {
            ++output_count;
            state.detection_tensor_name = name;
            state.detection_shape = shape_vector(dims);
        } else if (shape_equals(dims, expected_prototype)) {
            ++output_count;
            state.prototype_tensor_name = name;
            state.prototype_shape = shape_vector(dims);
        } else if (shape_equals(dims, transposed_detection)) {
            throw std::runtime_error(
                "TensorRT detection tensor is transposed for model=" + contract.label +
                "; required channel-major layout is [1," +
                std::to_string(contract.detection_channels) + "," +
                std::to_string(contract.detection_candidates) + "]");
        } else {
            throw std::runtime_error(
                "unrecognized TensorRT output model=" + contract.label +
                " name=" + std::string(name) +
                " dims=" + dims_to_string(dims));
        }
    }

    if (output_count != 2 ||
        state.detection_tensor_name.empty() ||
        state.prototype_tensor_name.empty()) {
        throw std::runtime_error(
            "TensorRT outputs do not match the sealed contract for model=" +
            contract.label);
    }

    const nvinfer1::Dims input_dims =
        state.context->getTensorShape(state.input_tensor_name.c_str());
    const nvinfer1::Dims detection_dims =
        state.context->getTensorShape(state.detection_tensor_name.c_str());
    const nvinfer1::Dims prototype_dims =
        state.context->getTensorShape(state.prototype_tensor_name.c_str());
    state.input_bytes = element_count(input_dims) * sizeof(float);
    state.detection_elements = element_count(detection_dims);
    state.prototype_elements = element_count(prototype_dims);
    state.detection_bytes = state.detection_elements * sizeof(float);
    state.prototype_bytes = state.prototype_elements * sizeof(float);

    const std::string stream_operation =
        "cudaStreamCreate model=" + contract.label;
    check_cuda(
        cudaStreamCreate(&state.cuda_stream),
        stream_operation.c_str());
    if (schedule_probe_enabled_) {
        for (auto& event : state.diagnostic_events) {
            check_cuda(cudaEventCreateWithFlags(&event, cudaEventBlockingSync),
                      "create schedule-probe event");
        }
    }
    const std::string input_allocation =
        "cudaMalloc input model=" + contract.label;
    const std::string detection_allocation =
        "cudaMalloc detection model=" + contract.label;
    const std::string prototype_allocation =
        "cudaMalloc prototype model=" + contract.label;
    const std::string detection_host_allocation =
        "cudaMallocHost detection output model=" + contract.label;
    const std::string prototype_host_allocation =
        "cudaMallocHost prototype output model=" + contract.label;
    check_cuda(
        cudaMalloc(&state.input_device, state.input_bytes),
        input_allocation.c_str());
    check_cuda(
        cudaMalloc(&state.detection_device, state.detection_bytes),
        detection_allocation.c_str());
    check_cuda(
        cudaMalloc(&state.prototype_device, state.prototype_bytes),
        prototype_allocation.c_str());
    void* detection_host = nullptr;
    check_cuda(
        cudaMallocHost(&detection_host, state.detection_bytes),
        detection_host_allocation.c_str());
    state.detection_host = static_cast<float*>(detection_host);
    void* prototype_host = nullptr;
    check_cuda(
        cudaMallocHost(&prototype_host, state.prototype_bytes),
        prototype_host_allocation.c_str());
    state.prototype_host = static_cast<float*>(prototype_host);
    if (!state.context->setTensorAddress(
            state.input_tensor_name.c_str(), state.input_device) ||
        !state.context->setTensorAddress(
            state.detection_tensor_name.c_str(), state.detection_device) ||
        !state.context->setTensorAddress(
            state.prototype_tensor_name.c_str(), state.prototype_device)) {
        throw std::runtime_error(
            "failed to bind one or more TensorRT tensor addresses for model=" +
            contract.label);
    }
    LOG_INFO_M(
        name_,
        "pinned TensorRT outputs ready model=" << contract.label
                                                << " detection_elements="
                                                << state.detection_elements
                                                << " detection_bytes="
                                                << state.detection_bytes
                                                << " prototype_elements="
                                                << state.prototype_elements
                                                << " prototype_bytes="
                                                << state.prototype_bytes);
    return true;
}

// Pinned host inputs, allocated once; a different size after a reload would mean another engine.
void InferenceModule::ensure_pinned_input_buffers() {
    const auto ensure_buffer = [](float*& buffer,
                                  std::size_t& capacity_bytes,
                                  std::size_t required_bytes,
                                  const std::string& label) {
        if (buffer != nullptr) {
            if (capacity_bytes != required_bytes) {
                throw std::runtime_error(
                    "pinned input capacity changed for model=" + label);
            }
            return;
        }
        void* allocated = nullptr;
        const std::string operation =
            "cudaMallocHost input model=" + label;
        check_cuda(cudaMallocHost(&allocated, required_bytes), operation.c_str());
        buffer = static_cast<float*>(allocated);
        capacity_bytes = required_bytes;
    };

    if (primary_model_ == nullptr || !primary_model_->loaded) {
        throw std::runtime_error("cannot allocate primary pinned input before model load");
    }
    ensure_buffer(
        primary_input_host_,
        primary_input_host_bytes_,
        primary_model_->input_bytes,
        primary_model_->contract.label);

    if (active12_model_ == nullptr || !active12_model_->loaded) {
        throw std::runtime_error("cannot allocate active12 pinned input before model load");
    }
    ensure_buffer(
        active12_input_host_,
        active12_input_host_bytes_,
        active12_model_->input_bytes,
        active12_model_->contract.label);
    LOG_INFO_M(
        name_,
        "pinned TensorRT inputs ready primary_bytes=" << primary_input_host_bytes_
                                                       << " active12_bytes="
                                                       << active12_input_host_bytes_);
}

void InferenceModule::preprocess_model_inputs(
    const cv::Mat& image,
    LetterboxInfo& primary_info,
    LetterboxInfo& active12_info,
    ParallelTaskWorker* parallel_worker) {
    if (parallel_worker == nullptr) {
        throw std::logic_error(
            "dual-model preprocess requires a parallel task worker");
    }
    // Both branches only read the captured image and write to separate
    // letterbox metadata and pinned input buffers. Reuse the persistent
    // helper for active12 while this inference worker prepares primary.
    parallel_worker->run(
        [this, &image, &active12_info]() {
            // The square input is preprocess_into() itself (window_top 0, height == width).
            const auto& spec = active12_spec();
            preprocess_window_into(
                image,
                ModelInputShape{spec.input_width, spec.input_height},
                spec.input_window_top,
                active12_info,
                active12_input_host_,
                active12_input_host_bytes_ / sizeof(float));
        },
        [this, &image, &primary_info]() {
            preprocess_into(
                image,
                config_.input_size(),
                primary_info,
                primary_input_host_,
                primary_input_host_bytes_ / sizeof(float));
        });
}

void InferenceModule::validate_tensor_rt_input(
    const TensorRtModelState& state,
    const float* tensor_data,
    std::size_t tensor_bytes) const {
    if (!state.loaded || state.context == nullptr || state.cuda_stream == nullptr) {
        throw std::runtime_error(
            "TensorRT state is not ready for model=" + state.contract.label);
    }
    if (tensor_data == nullptr || tensor_bytes != state.input_bytes) {
        throw std::runtime_error(
            "preprocess tensor byte count does not match TensorRT input model=" +
            state.contract.label);
    }
    if (state.detection_host == nullptr || state.prototype_host == nullptr ||
        state.detection_elements == 0U || state.prototype_elements == 0U) {
        throw std::runtime_error(
            "pinned TensorRT outputs are not ready for model=" +
            state.contract.label);
    }
}

// H2D copy and enqueue on the model's stream; events 0..2 bracket the stages for the CUDA spans.
void InferenceModule::submit_tensor_rt(
    TensorRtModelState& state,
    const float* tensor_data,
    std::size_t tensor_bytes) {
    validate_tensor_rt_input(state, tensor_data, tensor_bytes);
    if (state.diagnostic_events[0]) {
        check_cuda(cudaEventRecord(state.diagnostic_events[0], state.cuda_stream),
                  "record schedule-probe H2D begin");
    }
    const std::string input_copy =
        "cudaMemcpyAsync input model=" + state.contract.label;
    check_cuda(
        cudaMemcpyAsync(
            state.input_device,
            tensor_data,
            state.input_bytes,
            cudaMemcpyHostToDevice,
            state.cuda_stream),
        input_copy.c_str());
    if (state.diagnostic_events[1]) {
        check_cuda(cudaEventRecord(state.diagnostic_events[1], state.cuda_stream),
                  "record schedule-probe H2D done");
    }
    if (!state.context->enqueueV3(state.cuda_stream)) {
        throw std::runtime_error(
            "TensorRT enqueueV3 failed for model=" + state.contract.label);
    }
    if (state.diagnostic_events[2]) {
        check_cuda(cudaEventRecord(state.diagnostic_events[2], state.cuda_stream),
                  "record schedule-probe execution stream end");
    }
}

void InferenceModule::copy_tensor_rt_outputs_and_wait(TensorRtModelState& state) {
    queue_tensor_rt_outputs(state, state.detection_host, state.prototype_host);
    const std::string synchronize =
        "cudaStreamSynchronize model=" + state.contract.label;
    check_cuda(cudaStreamSynchronize(state.cuda_stream), synchronize.c_str());
}

// D2H copies into the given host buffers (a JIT slot's own); event 3 marks both outputs as
// ready, which wait_ready and the empty-acquire poll wait on.
void InferenceModule::queue_tensor_rt_outputs(
    TensorRtModelState& state, float* detection, float* prototype) {
    if (detection == nullptr || prototype == nullptr) {
        throw std::invalid_argument("null schedule-probe output slot");
    }
    const std::string detection_copy =
        "cudaMemcpyAsync detection output model=" + state.contract.label;
    const std::string prototype_copy =
        "cudaMemcpyAsync prototype output model=" + state.contract.label;
    check_cuda(
        cudaMemcpyAsync(
            detection,
            state.detection_device,
            state.detection_bytes,
            cudaMemcpyDeviceToHost,
            state.cuda_stream),
        detection_copy.c_str());
    check_cuda(
        cudaMemcpyAsync(
            prototype,
            state.prototype_device,
            state.prototype_bytes,
            cudaMemcpyDeviceToHost,
            state.cuda_stream),
        prototype_copy.c_str());
    if (state.diagnostic_events[3]) {
        check_cuda(cudaEventRecord(state.diagnostic_events[3], state.cuda_stream),
                  "record schedule-probe D2H done");
    }
}

std::array<float, 4> InferenceModule::tensor_rt_diagnostic_intervals(
    const TensorRtModelState& state) const {
    std::array<float, 4> result{};
    const auto& events = state.diagnostic_events;
    if (!events[0] || !events[1] || !events[2] || !events[3]) {
        throw std::logic_error("schedule-probe events unavailable");
    }
    check_cuda(cudaEventElapsedTime(&result[0], events[0], events[1]), "H2D CUDA span");
    check_cuda(cudaEventElapsedTime(&result[1], events[1], events[2]), "execution CUDA span");
    check_cuda(cudaEventElapsedTime(&result[2], events[2], events[3]), "D2H CUDA span");
    check_cuda(cudaEventElapsedTime(&result[3], events[0], events[3]), "total CUDA span");
    return result;
}

void InferenceModule::drain_tensor_rt_no_throw(TensorRtModelState& state) noexcept {
    if (state.cuda_stream == nullptr) {
        return;
    }
    if (cudaStreamSynchronize(state.cuda_stream) != cudaSuccess) {
        // A failed drain makes the context unsafe for the next frame. The serial loop's failure
        // counter then reloads both engines; the JIT loop has already stopped.
        state.loaded = false;
    }
}

void InferenceModule::execute_tensor_rt_pair(
    TensorRtModelState& primary,
    const float* primary_tensor_data,
    std::size_t primary_tensor_bytes,
    TensorRtModelState& active,
    const float* active_tensor_data,
    std::size_t active_tensor_bytes,
    int active_lead_us) {
    // Validate both branches before the first asynchronous operation so a
    // predictable active-input error cannot strand a primary H2D transfer.
    validate_tensor_rt_input(primary, primary_tensor_data, primary_tensor_bytes);
    validate_tensor_rt_input(active, active_tensor_data, active_tensor_bytes);

    try {
        // Keep the established primary-first order unless the diagnostic
        // environment explicitly requests an Active12 lead. This isolates
        // launch scheduling from engines, placement, and output semantics.
        if (active_lead_us < 0) {
            submit_tensor_rt(primary, primary_tensor_data, primary_tensor_bytes);
            submit_tensor_rt(active, active_tensor_data, active_tensor_bytes);
        } else {
            submit_tensor_rt(active, active_tensor_data, active_tensor_bytes);
            if (active_lead_us > 0) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(active_lead_us));
            }
            submit_tensor_rt(primary, primary_tensor_data, primary_tensor_bytes);
        }
        // Phase 1 deliberately waits in the same call and in the established
        // model order. It creates no cross-frame overlap or extra ownership.
        copy_tensor_rt_outputs_and_wait(primary);
        copy_tensor_rt_outputs_and_wait(active);
    } catch (...) {
        // Pinned inputs and device/host outputs are reused on the next frame.
        // Drain both streams before any caller can overwrite those buffers.
        drain_tensor_rt_no_throw(primary);
        drain_tensor_rt_no_throw(active);
        throw;
    }
}

std::vector<Detection> InferenceModule::postprocess_primary(
    const TensorRtModelState& state,
    const LetterboxInfo& letterbox_info) const {
    validate_contiguous_channels(
        state.detection_host,
        state.detection_elements,
        state.contract.detection_channels,
        state.contract.detection_candidates);
    validate_contiguous_channels(
        state.prototype_host,
        state.prototype_elements,
        state.contract.mask_channels,
        state.contract.mask_height * state.contract.mask_size);

    // The COCO80 branch: this overload and every inference.* threshold, as the
    // retired single-model route used them (level-3 trim).
    auto detections = postprocess_seg(
        state.detection_host,
        state.detection_shape,
        state.prototype_host,
        state.prototype_shape,
        letterbox_info,
        config_.conf_threshold(),
        config_.per_class_confidence_thresholds(),
        config_.low_confidence_floor(),
        config_.low_confidence_max_candidates(),
        config_.iou_threshold(),
        config_.mask_threshold(),
        config_.input_size());
    // Keep the established bbox-local COCO contract. The decoder already
    // owns its output pixels; only the optional boundary deep copy changes.
    (void)perception::finalize_postprocess_mask_ownership(
        detections, mask_output_copy_mode_, false);
    return detections;
}

std::vector<Detection> InferenceModule::postprocess_active12(
    const TensorRtModelState& state,
    const LetterboxInfo& letterbox_info,
    std::uint64_t frame_id,
    Active12MaskProfile* deferred_profile) const {
    validate_contiguous_channels(
        state.detection_host,
        state.detection_elements,
        state.contract.detection_channels,
        state.contract.detection_candidates);
    validate_contiguous_channels(
        state.prototype_host,
        state.prototype_elements,
        state.contract.mask_channels,
        state.contract.mask_height * state.contract.mask_size);

    const auto& dual = config_.dual_model();
    const auto& spec = active12_spec();
    const bool profile_enabled = perception::inference_profile_enabled();
    Active12MaskProfile profile;
    Active12PaddingGuard padding_guard;
    auto detections = postprocess_seg(
        state.detection_host,
        state.detection_shape,
        state.prototype_host,
        state.prototype_shape,
        letterbox_info,
        spec,
        dual.conf_threshold,
        dual.nms_iou_threshold,
        dual.mask_threshold,
        ModelInputShape{spec.input_width, spec.input_height},
        active12_mask_implementation_,
        active12_mask_lanes_.get(),
        profile_enabled ? &profile : nullptr,
        active12_mask_binary_implementation_, execution_cpu_trace_,
        active12_padding_guard_ ? &padding_guard : nullptr);
    if (padding_guard.dropped != 0) {
        active12_padding_dropped_.fetch_add(padding_guard.dropped, std::memory_order_relaxed);
        active12_padding_frames_.fetch_add(1, std::memory_order_relaxed);
    }
    // The dedicated mask lanes have joined and all masks own their pixels.
    // Preserve the independent first-class-92 canvas in either copy mode.
    (void)perception::finalize_postprocess_mask_ownership(
        detections, mask_output_copy_mode_, true);
    if (profile_enabled) {
        if (deferred_profile) *deferred_profile = std::move(profile);
        else log_active12_mask_profile(profile, frame_id);
    }
    return detections;
}

// Mask/ROI area record of one inference frame (acceptance list, section 共载与场景): the mask raster is the ROI
// (box) the instance mask covers, the foreground its nonzero pixels; Active12 class 92 (the
// traversable area, T's input; global id as in dual_model_merge) is kept apart.
void InferenceModule::record_mask_areas(const std::vector<Detection>& coco, const std::vector<Detection>& active) const {
    using W = perception::MaskAreaWindow;
    constexpr int kTraversableClassId = 92;
    constexpr std::uint64_t kMaskAreaFrames = 300;  // about 12 s at 25 Hz, like ACTIVE_MASK_SUMMARY
    const auto add = [this](W::Branch branch, const Detection& d) {
        const std::uint64_t roi = d.mask.empty() ? static_cast<std::uint64_t>(std::max(0, d.bbox.area()))
                                                 : static_cast<std::uint64_t>(d.mask.total());
        const std::uint64_t foreground = d.mask.empty() ? 0 : static_cast<std::uint64_t>(cv::countNonZero(d.mask));
        mask_area_.add(branch, roi, foreground);
    };
    for (const auto& d : coco) add(W::kCoco80, d);
    for (const auto& d : active) add(d.class_id == kTraversableClassId ? W::kClass92 : W::kActive12, d);
    mask_area_.frame();
    if (mask_area_.frames() >= kMaskAreaFrames) print_mask_areas(false);
}

// One grid sample of the colour frame when a second has passed since the last one (from the JIT
// loop's prepare step; a few thousand pixel reads, timed). A frame that is not packed 8-bit BGR
// counts as an unusable sample.
void InferenceModule::sample_scene_luma(const cv::Mat& frame, std::uint64_t frame_id) {
    const std::int64_t now = perception::monotonic_now_ns();
    if (!scene_luma_.due(now)) return;
    const double mean = frame.type() == CV_8UC3
        ? perception::grid_luma_mean(frame.data, frame.rows, frame.cols, frame.step[0], frame.channels(),
                                     perception::SceneLumaWindow::kGridStep)
        : -1.0;
    const std::int64_t cost = perception::monotonic_now_ns() - now;
    if (scene_luma_.add(now, mean, frame_id, cost)) print_scene_luma(false);
}

// One SCENE_LUMA line (mean, min and max of the grid means of up to 60 samples at least a second
// apart, the first and last sample's monotonic time and frame id, the sampling cost), then a new
// window. The print itself is not in the cost.
void InferenceModule::print_scene_luma(bool partial) {
    if (scene_luma_.count() == 0) return;
    LOG_INFO_M(name_, "SCENE_LUMA samples=" << scene_luma_.count() << " unusable=" << scene_luma_.unusable()
        << " mean=" << scene_luma_.mean() << " min=" << scene_luma_.min() << " max=" << scene_luma_.max()
        << " first_mono_ns=" << scene_luma_.first_ns() << " last_mono_ns=" << scene_luma_.last_ns()
        << " first_frame_id=" << scene_luma_.first_frame() << " last_frame_id=" << scene_luma_.last_frame()
        << " cost_us_total=" << scene_luma_.cost_total_ns() / 1000 << " cost_us_max=" << scene_luma_.cost_max_ns() / 1000
        << " grid_step=" << perception::SceneLumaWindow::kGridStep << " partial=" << (partial ? 1 : 0));
    scene_luma_.reset();
}

// One MASK_AREA_SUMMARY line (mean and 99th-percentile bucket edge per branch), then a new window.
void InferenceModule::print_mask_areas(bool partial) const {
    using W = perception::MaskAreaWindow;
    if (mask_area_.frames() == 0) return;
    std::ostringstream line;
    line << "MASK_AREA_SUMMARY frames=" << mask_area_.frames();
    const char* names[W::kBranches] = {"coco80", "active12", "class92"};
    for (std::size_t b = 0; b < W::kBranches; ++b) {
        const auto& r = mask_area_.branch(static_cast<W::Branch>(b));
        line << " " << names[b] << "_instances=" << r.roi.n << " " << names[b] << "_roi_px_mean=" << W::mean(r.roi)
             << " " << names[b] << "_roi_px_p99_le=" << W::quantile_upper(r.roi, .99) << " " << names[b]
             << "_fg_px_mean=" << W::mean(r.foreground) << " " << names[b]
             << "_fg_px_p99_le=" << W::quantile_upper(r.foreground, .99);
    }
    line << " partial=" << (partial ? 1 : 0);
    LOG_INFO_M(name_, line.str());
    mask_area_.reset();
}

// One ACTIVE_MASK_SUMMARY line for the accumulated window, then a fresh window. `partial` marks the
// shorter last window that stop() prints.
void InferenceModule::print_mask_summary(bool partial) const {
    auto& sum = mask_summary_;
    if (sum.frames == 0) return;
    const double frames = static_cast<double>(sum.frames);
    const double instances = static_cast<double>(std::max<std::uint64_t>(1, sum.instances));
    LOG_INFO_M(name_, "ACTIVE_MASK_SUMMARY frames=" << sum.frames << " instances=" << sum.instances
        << " instances_per_frame=" << static_cast<double>(sum.instances) / frames
        << " bbox_pixels_per_instance=" << static_cast<double>(sum.bbox_pixels) / instances
        << " task_ms_per_instance=" << static_cast<double>(sum.task_ns) / instances / 1e6
        << " gemm_ms_per_instance=" << static_cast<double>(sum.gemm_ns) / instances / 1e6
        << " resize_ms_per_instance=" << static_cast<double>(sum.resize_ns) / instances / 1e6
        << " threshold_ms_per_instance=" << static_cast<double>(sum.threshold_ns) / instances / 1e6
        << " max_task_ms=" << static_cast<double>(sum.max_task_ns) / 1e6
        << " batch_wall_ms_per_frame=" << static_cast<double>(sum.batch_wall_ns) / frames / 1e6
        << " impl=" << active12_mask_implementation_name(active12_mask_implementation_)
        << " binary_impl=" << active12_mask_binary_implementation_name(active12_mask_binary_implementation_)
        << " padding_dropped_total=" << active12_padding_dropped_.load(std::memory_order_relaxed)
        << " padding_frames_total=" << active12_padding_frames_.load(std::memory_order_relaxed)
        << " partial=" << (partial ? 1 : 0));
    sum = MaskProfileSummary{};
}

void InferenceModule::log_active12_mask_profile(const Active12MaskProfile& profile, std::uint64_t frame_id) const {
        const char* implementation = active12_mask_implementation_name(active12_mask_implementation_);
        // Always: accumulate the summary (per-object mask cost and bbox area, i.e. the size of the
        // mask raster, not its foreground) and print it every kMaskSummaryFrames profiled frames.
        constexpr std::uint64_t kMaskSummaryFrames = 300;  // about 12 s at 25 Hz
        auto& sum = mask_summary_;
        ++sum.frames;
        sum.instances += profile.instances;
        sum.batch_wall_ns += profile.batch_wall_ns;
        for (const auto& task : profile.tasks) {
            sum.bbox_pixels += static_cast<std::uint64_t>(std::max<std::int64_t>(0, task.bbox_pixels));
            sum.task_ns += task.total_ns;
            sum.gemm_ns += task.gemm_ns;
            sum.resize_ns += task.resize_ns;
            sum.threshold_ns += task.threshold_convert_ns;
            sum.max_task_ns = std::max(sum.max_task_ns, task.total_ns);
        }
        if (sum.frames >= kMaskSummaryFrames) print_mask_summary(false);
        // Diagnostic only (PERCEPTION_ACTIVE12_MASK_LOG=1): one line per instance and per frame.
        if (!active12_mask_log_) return;
        for (const auto& task : profile.tasks) {
            LOG_INFO_M(name_, "ACTIVE_MASK_TASK frame_id=" << frame_id
                << " impl=" << implementation << " index=" << task.index
                << " binary_impl=" << active12_mask_binary_implementation_name(active12_mask_binary_implementation_)
                << " bbox_pixels=" << task.bbox_pixels
                << " gemm_ns=" << task.gemm_ns << " resize_ns=" << task.resize_ns
                << " threshold_convert_ns=" << task.threshold_convert_ns
                << " canvas_ns=" << task.canvas_ns << " total_ns=" << task.total_ns
                << " lane=" << task.lane << task.execution_cpu);
        }
        LOG_INFO_M(name_, "ACTIVE_MASK_PROFILE frame_id=" << frame_id
            << " impl=" << implementation << " opencv_threads=" << cv::getNumThreads()
            << " binary_impl=" << active12_mask_binary_implementation_name(active12_mask_binary_implementation_)
            << " instances=" << profile.instances << " batch_wall_ns=" << profile.batch_wall_ns
            << " total_ns=" << profile.total_ns
            << " parallel_used=" << static_cast<int>(profile.parallel_used)
            << " shadow_checked=" << static_cast<int>(profile.shadow_checked)
            << " binary_shadow_checked=" << static_cast<int>(profile.binary_shadow_checked)
            << " clock=CLOCK_MONOTONIC");
}

void InferenceModule::release_tensor_rt_model(
    std::unique_ptr<TensorRtModelState>& state) noexcept {
    if (state == nullptr) {
        return;
    }
    state->loaded = false;
    if (state->cuda_stream != nullptr) {
        cudaStreamSynchronize(state->cuda_stream);
    }
    for (auto& event : state->diagnostic_events) {
        if (event) {
            cudaEventDestroy(event);
            event = nullptr;
        }
    }
    if (state->input_device != nullptr) {
        cudaFree(state->input_device);
        state->input_device = nullptr;
    }
    if (state->detection_device != nullptr) {
        cudaFree(state->detection_device);
        state->detection_device = nullptr;
    }
    if (state->prototype_device != nullptr) {
        cudaFree(state->prototype_device);
        state->prototype_device = nullptr;
    }
    if (state->detection_host != nullptr) {
        cudaFreeHost(state->detection_host);
        state->detection_host = nullptr;
    }
    state->detection_elements = 0U;
    state->detection_bytes = 0U;
    if (state->prototype_host != nullptr) {
        cudaFreeHost(state->prototype_host);
        state->prototype_host = nullptr;
    }
    state->prototype_elements = 0U;
    state->prototype_bytes = 0U;
    if (state->cuda_stream != nullptr) {
        cudaStreamDestroy(state->cuda_stream);
        state->cuda_stream = nullptr;
    }
    destroy_tensor_rt_object(state->context);
    destroy_tensor_rt_object(state->engine);
    destroy_tensor_rt_object(state->runtime);
    state.reset();
}

void InferenceModule::release_tensor_rt() noexcept {
    release_tensor_rt_model(active12_model_);
    release_tensor_rt_model(primary_model_);
}

void InferenceModule::release_pinned_input_buffers() noexcept {
    if (active12_input_host_ != nullptr) {
        cudaFreeHost(active12_input_host_);
        active12_input_host_ = nullptr;
    }
    active12_input_host_bytes_ = 0U;
    if (primary_input_host_ != nullptr) {
        cudaFreeHost(primary_input_host_);
        primary_input_host_ = nullptr;
    }
    primary_input_host_bytes_ = 0U;
}
#endif

}  // namespace yolo
