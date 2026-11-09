/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: engine_build_main.cpp
 * Brief: Target-machine TensorRT engine build with a recorded build environment (19 section 13.2).
 *
 * Description:
 * 19 section 13.2: "must be built on the machine, no cross-machine engine copy; engine and
 * build_env digest stored together". Diagnostic/installation tool, never part of perception_node.
 *   perception_engine_build [--dry-run] [--model coco80|active12]...
 *
 * --model (2026-10-10): build only the named model(s). The engine of every other model must
 * already exist; the module loads it unchanged and no record is written for it. Without --model
 * both engines are built, as before.
 *
 * --dry-run: load and validate the configuration exactly as below, check the
 * ONNX inputs and the no-overwrite rule, print every target with its flags and
 * ONNX SHA-256, and exit without touching CUDA, TensorRT, GPU or DLA.
 *
 * Prepares the process exactly as perception_service_main.cpp does (same as
 * perception_psc4_seal), loads the resolved configuration, and builds every
 * REQUIRED engine that does not exist yet through the service's own
 * InferenceModule build path (AllowBuild; ONNX, precision, DLA core and GPU
 * fallback all from the configuration). It refuses to overwrite an existing
 * engine file. Next to each built engine it writes
 *
 *   <engine>.build_env.json   TensorRT / CUDA / L4T / host identity, build
 *                             flags, ONNX and engine SHA-256 and size, wall time,
 *                             and for an INT8 model the plan's files (calibration
 *                             cache and FP16 layer list: path, SHA-256, header,
 *                             counts), checked against the configuration before
 *                             the build (also in --dry-run)
 *   <engine>.layers.json      the engine inspector's layer list (names only
 *                             at the default profiling verbosity; DLA subgraphs
 *                             appear as ForeignNode entries)
 *
 * The build_env record is what the PSC-4 seal cites (RECORDED_AT_BUILD).
 */
#include "common/config.hpp"
#include "common/preprocess_mode.hpp"
#include "common/threadsafe_queue.hpp"
#include "common/types.hpp"
#include "modules/inference_module.hpp"
#include "modules/int8_plan.hpp"
#include "service/psc4_run.hpp"
#include "service/service_config.hpp"

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>
#include <sys/utsname.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace fs = std::filesystem;
using namespace perception::service;

namespace {
// TensorRT logger for the layer inspection: warnings and errors only.
class QuietLogger final : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) std::cerr << "[TensorRT] " << msg << std::endl;
    }
};

// Minimal JSON string for the record (control characters become spaces).
std::string json_string(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c == '\n') out += "\\n";
        else if (static_cast<unsigned char>(c) < 0x20) out += ' ';
        else out += c;
    }
    return out + "\"";
}

// First line of a system file (L4T release, device model, boot id); "" if absent.
std::string first_line(const std::string& path) {
    std::ifstream in(path);
    std::string line;
    std::getline(in, line);
    return line;
}

// One engine to build, with the flags taken from the configuration.
struct Target {
    std::string label, onnx, engine;
    bool fp16 = false, int8 = false, dla = false, gpu_fallback = false;
    int dla_core = -1, workspace_mb = 0, input_size = 0, input_height = 0;  // input_size = width
    yolo::Config::Int8BuildConfig int8_build;
};

// The INT8 plan's identity for the record: "null" for an FP16 model. load_int8_plan checks the
// files' SHA-256 against the configuration and parses them (refusal throws).
std::string int8_plan_json(const Target& t) {
    if (!t.int8) return "null";
    const yolo::Int8Plan plan = yolo::load_int8_plan(t.int8_build.ranges_path, t.int8_build.ranges_sha256,
                                                     t.int8_build.fp16_layers_path, t.int8_build.fp16_layers_sha256);
    std::ostringstream j;
    j << "{\"ranges\": {\"path\": " << json_string(plan.ranges_path) << ", \"sha256\": " << json_string(plan.ranges_sha256)
      << ", \"header\": " << json_string(plan.cache.header) << ", \"entries\": " << plan.cache.scales.size() << "}, "
      << "\"fp16_layers\": ";
    if (plan.fp16_layers_path.empty()) {
        j << "null";
    } else {
        j << "{\"path\": " << json_string(plan.fp16_layers_path) << ", \"sha256\": " << json_string(plan.fp16_layers_sha256)
          << ", \"count\": " << plan.fp16_layers.size() << "}";
    }
    j << "}";
    return j.str();
}

// Deserialize the new engine and dump the inspector's layer list ("null" when
// any step fails: the record then says so instead of guessing).
std::string layer_json(const Target& t) {
    std::ifstream in(t.engine, std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    QuietLogger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));
    if (!runtime) return "null";
    if (t.dla) runtime->setDLACore(t.dla_core);
    std::unique_ptr<nvinfer1::ICudaEngine> engine(runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine) return "null";
    std::unique_ptr<nvinfer1::IEngineInspector> inspector(engine->createEngineInspector());
    if (!inspector) return "null";
    const char* info = inspector->getEngineInformation(nvinfer1::LayerInformationFormat::kJSON);
    return info ? std::string(info) : "null";
}
}  // namespace

int main(int argc, char** argv, char** envp) {
    try {
        bool dry_run = false;
        std::vector<std::string> only;  // --model labels; empty: every model
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--dry-run") {
                dry_run = true;
            } else if (arg == "--model" && i + 1 < argc &&
                       (std::string(argv[i + 1]) == "coco80" || std::string(argv[i + 1]) == "active12")) {
                only.emplace_back(argv[++i]);
            } else {
                throw std::runtime_error("unknown argument: " + arg + " (expected --dry-run or --model coco80|active12)");
            }
        }
        // Same preparation order as perception_service_main.cpp run().
        reject_inherited_runtime_environment(envp);
        LoadInputs inputs;
        inputs.host = read_host_identity();
        const auto cfg = load_service_config(inputs);
        apply_runtime_environment(cfg);
        if (const int threads = process_open_cv_threads(cfg)) cv::setNumThreads(threads);
        yolo::Config config;
        if (!config.load_from_node(cfg.pipeline, cfg.resolved_path)) throw std::runtime_error("pipeline configuration rejected");
        (void)yolo::initialize_preprocess_mode_from_environment();

        // Required engines: coco80 and active12, always both (dual_model.enabled
        // is required and true; load_service_config refused anything else).
        std::vector<Target> targets;
        {
            Target c;
            c.label = "coco80"; c.onnx = config.model_path(); c.engine = config.trt_engine_path();
            c.fp16 = config.use_fp16(); c.int8 = config.use_int8(); c.dla = config.use_dla();
            c.int8_build = config.int8_build();
            c.gpu_fallback = config.use_dla();   // the module's rule for the primary model
            c.dla_core = config.dla_core(); c.workspace_mb = config.trt_workspace_mb(); c.input_size = config.input_size();
            c.input_height = config.input_size();
            targets.push_back(c);
        }
        {
            const auto& d = config.dual_model();
            Target a;
            a.label = "active12"; a.onnx = d.active_model_path; a.engine = d.active_engine_path;
            // Active12 is built from its own configuration block (dual_model.int8 and int8_* keys
            // since 2026-10-10); its GPU fallback is an explicit key.
            a.fp16 = d.use_fp16; a.int8 = d.int8.enabled; a.dla = d.use_dla; a.gpu_fallback = d.allow_gpu_fallback;
            a.int8_build = d.int8;
            // dual_model.input_shape: the module builds the square or the 896 x 608 window input (its
            // contract comes from the sealed ModelSpec); the record names the rows too.
            a.dla_core = d.dla_core; a.workspace_mb = d.workspace_mb; a.input_size = d.input_size;
            a.input_height = d.input_height;
            targets.push_back(a);
        }
        // --model: the other engines must exist (the module would otherwise build them too).
        if (!only.empty()) {
            std::vector<Target> selected;
            for (const auto& t : targets) {
                if (std::find(only.begin(), only.end(), t.label) != only.end()) {
                    selected.push_back(t);
                } else if (!fs::exists(t.engine)) {
                    throw std::runtime_error("model " + t.label + " not selected but its engine does not exist: " +
                                             t.engine);
                }
            }
            targets = selected;
        }
        // Never overwrite: an existing engine is either the one to keep or a
        // mistake; re-building is an explicit, separately authorised act.
        for (const auto& t : targets) {
            if (fs::exists(t.engine)) throw std::runtime_error("engine exists, refusing to overwrite: " + t.engine);
            if (!fs::exists(t.onnx)) throw std::runtime_error("ONNX not found: " + t.onnx);
            if (fs::exists(t.engine + ".build_env.json")) throw std::runtime_error("build record exists: " + t.engine);
        }
        // ONNX identities, recorded in the build record and the dry-run listing.
        std::vector<std::string> onnx_sha;
        std::vector<std::uint64_t> onnx_bytes(targets.size());
        for (std::size_t i = 0; i < targets.size(); ++i) onnx_sha.push_back(psc4::file_sha256(targets[i].onnx, &onnx_bytes[i]));
        // INT8 plans: files checked against the configuration before anything is built.
        std::vector<std::string> int8_plans;
        for (const auto& t : targets) int8_plans.push_back(int8_plan_json(t));
        if (dry_run) {
            std::cout << "{\"tool\":\"perception_engine_build\",\"dry_run\":true,\"resolved_config\":"
                      << json_string(cfg.resolved_path) << ",\"targets\":[";
            for (std::size_t i = 0; i < targets.size(); ++i) {
                const auto& t = targets[i];
                std::cout << (i ? "," : "") << "{\"model\":" << json_string(t.label) << ",\"onnx\":" << json_string(t.onnx)
                          << ",\"onnx_sha256\":" << json_string(onnx_sha[i]) << ",\"onnx_bytes\":" << onnx_bytes[i]
                          << ",\"engine\":" << json_string(t.engine) << ",\"fp16\":" << (t.fp16 ? "true" : "false")
                          << ",\"int8\":" << (t.int8 ? "true" : "false") << ",\"dla\":" << (t.dla ? "true" : "false")
                          << ",\"dla_core\":" << t.dla_core << ",\"gpu_fallback\":" << (t.gpu_fallback ? "true" : "false")
                          << ",\"workspace_mb\":" << t.workspace_mb << ",\"input_size\":" << t.input_size
                          << ",\"input_height\":" << t.input_height
                          << ",\"int8_plan\":" << int8_plans[i] << "}";
            }
            std::cout << "]}" << std::endl;
            return 0;
        }

        // Build through the service's own module (AllowBuild), so the engines
        // are exactly what the service would have built.
        const auto t0 = std::chrono::steady_clock::now();
        try {
            auto input = std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(2);
            auto output = std::make_shared<yolo::ThreadsafeQueue<yolo::DetectionResult>>(2);
            yolo::InferenceModule module(config, input, output, nullptr, yolo::InferenceModule::ModelLoadPolicy::AllowBuild);
            if (!module.start()) throw std::runtime_error("inference module did not build/start the engines");
            module.stop();
        } catch (...) {
            // An engine written before the failure has no record; this run created it (none existed,
            // checked above), so it is set aside as .failed.<epoch s>, never loaded, and a retry is
            // not refused by the no-overwrite rule.
            const auto stamp = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            for (const auto& t : targets) {
                std::error_code ec;
                if (!fs::exists(t.engine, ec)) continue;
                fs::rename(t.engine, t.engine + ".failed." + stamp, ec);
                std::cerr << "ENGINE_BUILD_SET_ASIDE engine=" << t.engine << " to=" << t.engine << ".failed." << stamp
                          << (ec ? " rename_error=" + ec.message() : std::string()) << "\n";
            }
            throw;
        }
        const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();

        // Build environment facts, observed now (this IS the build): TensorRT, CUDA, L4T, device,
        // kernel and boot id identify the machine and software stack the engine belongs to, and the
        // builder's own SHA-256 identifies the code that built it. A later PSC-4 seal cites this
        // record instead of re-observing a possibly different environment.
        int cuda_runtime = 0, cuda_driver = 0;
        (void)cudaRuntimeGetVersion(&cuda_runtime);
        (void)cudaDriverGetVersion(&cuda_driver);
        utsname un{};
        (void)uname(&un);
        char host[256] = {0};
        (void)gethostname(host, sizeof(host) - 1);
        std::string self_sha;
        try { self_sha = psc4::file_sha256("/proc/self/exe"); } catch (...) { self_sha = "unknown"; }

        // Per engine: the build_env record, the layer list and a summary entry.
        std::cout << "{\"tool\":\"perception_engine_build\",\"wall_ms\":" << wall_ms << ",\"engines\":[";
        for (std::size_t i = 0; i < targets.size(); ++i) {
            const auto& t = targets[i];
            std::uint64_t bytes = 0;
            const auto engine_sha = psc4::file_sha256(t.engine, &bytes);
            std::ostringstream r;
            r << "{\n"
              << "  \"record\": \"perception-engine-build-v1\",\n"
              << "  \"model\": " << json_string(t.label) << ",\n"
              << "  \"onnx\": {\"path\": " << json_string(t.onnx) << ", \"sha256\": " << json_string(onnx_sha[i])
              << ", \"bytes\": " << onnx_bytes[i] << "},\n"
              << "  \"engine\": {\"path\": " << json_string(t.engine) << ", \"sha256\": " << json_string(engine_sha)
              << ", \"bytes\": " << bytes << "},\n"
              << "  \"flags\": {\"fp16\": " << (t.fp16 ? "true" : "false") << ", \"int8\": " << (t.int8 ? "true" : "false")
              << ", \"dla\": " << (t.dla ? "true" : "false") << ", \"dla_core\": " << t.dla_core
              << ", \"gpu_fallback\": " << (t.gpu_fallback ? "true" : "false") << ", \"workspace_mb\": " << t.workspace_mb
              << ", \"input_size\": " << t.input_size << ", \"input_height\": " << t.input_height << "},\n"
              << "  \"int8_plan\": " << int8_plans[i] << ",\n"
              << "  \"tensorrt\": {\"header\": \"" << NV_TENSORRT_MAJOR << "." << NV_TENSORRT_MINOR << "." << NV_TENSORRT_PATCH
              << "." << NV_TENSORRT_BUILD << "\", \"library\": " << getInferLibVersion() << "},\n"
              << "  \"cuda\": {\"runtime\": " << cuda_runtime << ", \"driver\": " << cuda_driver << "},\n"
              << "  \"l4t\": " << json_string(first_line("/etc/nv_tegra_release")) << ",\n"
              << "  \"model_name\": " << json_string(first_line("/proc/device-tree/model")) << ",\n"
              << "  \"host\": " << json_string(host) << ", \"kernel\": " << json_string(std::string(un.release)) << ",\n"
              << "  \"boot_id\": " << json_string(first_line("/proc/sys/kernel/random/boot_id")) << ",\n"
              << "  \"builder_sha256\": " << json_string(self_sha) << ",\n"
              << "  \"resolved_config\": " << json_string(cfg.resolved_path) << ", \"pipeline_digest\": "
              << json_string(psc4::pipeline_digest(cfg)) << ", \"runtime_digest\": " << json_string(cfg.runtime_digest) << ",\n"
              << "  \"wall_ms_all_engines\": " << wall_ms << "\n"
              << "}\n";
            // The record and the layer list sit next to the engine, so they travel with it (the
            // layer list shows which layers run on DLA and which fell back to the GPU).
            std::ofstream(t.engine + ".build_env.json") << r.str();
            std::ofstream(t.engine + ".layers.json") << layer_json(t);
            std::cout << (i ? "," : "") << "{\"model\":\"" << t.label << "\",\"engine_sha256\":\"" << engine_sha
                      << "\",\"bytes\":" << bytes << ",\"record\":\"" << t.engine << ".build_env.json\"}";
        }
        std::cout << "]}" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "ENGINE_BUILD_REFUSED reason=\"" << e.what() << "\"\n";
        return 1;
    }
    return 0;
}
