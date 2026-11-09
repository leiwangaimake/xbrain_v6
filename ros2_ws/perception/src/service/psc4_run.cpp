/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: psc4_run.cpp
 * Brief: PSC-4 golden inference run and the service startup comparison.
 *
 * Description:
 * What problem this solves. Implements psc4_run.hpp. startup_check() first proves the identity of
 * everything the output depends on (engine bytes, input image, pipeline and runtime digests,
 * dual-model files) against each engine's sealed manifest, and only then runs the one golden
 * inference and compares the digests. The identity checks run first so a refusal names the
 * changed input instead of just "different output".
 */
#include "service/psc4_run.hpp"

#include "common/threadsafe_queue.hpp"
#include "common/time_utils.hpp"
#include "common/types.hpp"
#include "modules/inference_module.hpp"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <sstream>

#include <chrono>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <thread>

namespace perception::service::psc4 {
namespace {
// Elapsed-time clock for the run's own timings (load, first result, total); steady, never wall.
std::int64_t steady_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
// View a cv::Mat as the canonical matrix of psc4_golden (no copy).
CanonMat canon(const cv::Mat& m) {
    CanonMat c;
    c.rows = m.rows; c.cols = m.cols; c.type = m.type();
    if (m.empty()) return c;
    c.data = m.data;
    c.row_bytes = static_cast<std::size_t>(m.cols) * m.elemSize();
    c.stride = m.step[0];
    return c;
}
// The detection fields the canonical digest covers.
std::vector<CanonDetection> canon(const std::vector<yolo::Detection>& dets) {
    std::vector<CanonDetection> out;
    out.reserve(dets.size());
    for (const auto& d : dets) {
        CanonDetection c;
        c.x = d.bbox.x; c.y = d.bbox.y; c.w = d.bbox.width; c.h = d.bbox.height;
        c.confidence = d.confidence;
        c.class_id = d.class_id; c.original_class_id = d.original_class_id;
        c.source = static_cast<int>(d.source);
        c.class_name = d.class_name;
        c.mask = canon(d.mask);
        c.full_image_mask = canon(d.full_image_mask);
        out.push_back(c);
    }
    return out;
}
}  // namespace

// The engines whose output the service publishes: both partitions of the dual model. Each one
// must have its own sealed manifest next to it.
std::vector<RequiredEngine> required_engines(const yolo::Config& config) {
    return {{kSourceCoco80, config.trt_engine_path()}, {kSourceActive12, config.dual_model().active_engine_path}};
}

// Digest of the generated yolo::Config document (not the raw text), so a change
// that alters what the pipeline reads changes the digest.
std::string pipeline_digest(const ServiceConfig& cfg) {
    YAML::Emitter e;
    e << cfg.pipeline;
    return yolo::sha256_hex(std::string(e.c_str(), e.size()));
}

// SHA-256 of a whole file (engine or input image), optionally with its size. Read in one piece:
// the files are tens of MB at most and this runs once at startup.
std::string file_sha256(const std::string& path, std::uint64_t* bytes) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path);
    const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (bytes) *bytes = data.size();
    return yolo::sha256_hex(data);
}

// Row bytes only (stride padding excluded), so the digest describes pixels.
std::string pixels_sha256(const cv::Mat& bgr) {
    const auto c = canon(bgr);
    std::string bytes;
    bytes.reserve(c.row_bytes * static_cast<std::size_t>(c.rows));
    for (int r = 0; r < c.rows; ++r)
        bytes.append(reinterpret_cast<const char*>(c.data + static_cast<std::size_t>(r) * c.stride), c.row_bytes);
    return yolo::sha256_hex(bytes);
}

// Runs the real inference module on the sealed input image `repeats` times (the sealing tool uses
// repeats >= 2 to prove repeatability; the startup check uses 1) and digests every result.
GoldenRun run_golden(const yolo::Config& config, const cv::Mat& bgr, int repeats, std::uint32_t timeout_ms) {
    GoldenRun run;
    const auto t0 = steady_ms();
    // A dedicated module on private queues: the real pipeline is not running yet.
    auto input = std::make_shared<yolo::ThreadsafeQueue<yolo::FrameData>>(4);
    auto output = std::make_shared<yolo::ThreadsafeQueue<yolo::DetectionResult>>(4);
    // ExistingEnginesOnly: the check must test the sealed engine files, so a missing or stale
    // engine is a failure here, never a silent rebuild.
    yolo::InferenceModule inference(config, input, output, nullptr,
                                    yolo::InferenceModule::ModelLoadPolicy::ExistingEnginesOnly);
    if (!inference.start()) {
        run.error = "inference module did not start (engines missing, rejected or would need a rebuild)";
        return run;
    }
    run.started = true;
    run.start_ms = steady_ms() - t0;
    for (int i = 0; i < repeats; ++i) {
        // A fresh copy per repeat: the module may keep or modify the frame it is given.
        yolo::FrameData frame;
        frame.frame_id = static_cast<std::uint64_t>(i + 1);
        frame.frame = bgr.clone();
        frame.stamp_sec = static_cast<double>(i + 1);
        frame.timestamp = std::chrono::steady_clock::now();
        // The capture gives every frame a timing identity that the module's JIT
        // schedule requires (positive, ordered). The golden frame never leaves
        // this check, so every stamp is the monotonic clock (no wall clock).
        frame.pipeline_timing.capture_epoch_ns = perception::monotonic_now_ns();
        frame.pipeline_timing.capture_ready_ns = perception::monotonic_now_ns();
        const auto pushed = steady_ms();
        frame.pipeline_timing.frame_push_begin_ns = perception::monotonic_now_ns();
        input->push(std::move(frame));
        // Poll for the result, bounded by the sealed timeout.
        yolo::DetectionResult result;
        bool got = false;
        while (steady_ms() - pushed <= static_cast<std::int64_t>(timeout_ms)) {
            if (output->try_pop(result)) { got = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (!got) {
            run.error = "golden inference " + std::to_string(i + 1) + " timed out after " +
                        std::to_string(timeout_ms) + " ms";
            break;
        }
        if (i == 0) run.first_result_ms = steady_ms() - pushed;
        ++run.delivered;
        // One digest per model partition (coco80, active12).
        const auto dets = canon(result.detections);
        const auto maps = canon(result.map_detections);
        // Retired mask slot (level-3 trim): the safe traversable mask left the
        // pipeline and was always empty in the service (its switch safe_mask_build,
        // since retired too, was "0"), so the slot hashes the same empty matrix and
        // both sealed psc4-canon-v1 expected_sha256 values stay byte-identical.
        const auto retired_slot = canon(cv::Mat());
        run.outputs.push_back({digest_partition(kSourceCoco80, dets, maps, &retired_slot),
                               digest_partition(kSourceActive12, dets, maps, &retired_slot)});
    }
    inference.stop();
    run.total_ms = steady_ms() - t0;
    return run;
}

// PSC-4 at service start (19 section 7.4): refuse to start when the inference stack no longer
// reproduces its sealed output. Identity first (names what changed), then one real inference.
StartupCheck startup_check(const ServiceConfig& cfg, const yolo::Config& config) {
    StartupCheck r;
    const auto fail = [&](const std::string& key, const std::string& reason) {
        r.ok = false; r.key = key; r.reason = reason; return r;
    };
    const auto engines = required_engines(config);
    std::vector<Manifest> manifests;
    cv::Mat image;
    const auto pipeline = pipeline_digest(cfg);
    // Per required engine: its sealed manifest and the observed identities.
    for (const auto& e : engines) {
        // The golden assets live in a directory next to each engine file (asset_dir), so the
        // engine and its seal are deployed and replaced together.
        const auto dir = asset_dir(e.engine_path);
        const auto mpath = dir + "/manifest.json";
        std::string text;
        {
            std::ifstream in(mpath, std::ios::binary);
            if (!in) return fail(mpath, "missing golden asset manifest");
            text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        }
        Manifest m;
        try { m = parse_manifest(text); }
        catch (const std::exception& ex) { return fail(mpath, ex.what()); }
        const auto input = dir + "/" + m.input_file;
        Observed o;
        o.engine_file = std::filesystem::path(e.engine_path).filename().string();
        try {
            o.engine_sha256 = file_sha256(e.engine_path, &o.engine_bytes);
            o.input_sha256 = file_sha256(input);
        } catch (const std::exception& ex) { return fail(mpath, ex.what()); }
        // Decode the input exactly as the sealing tool did, and hash the pixels too.
        const cv::Mat decoded = cv::imread(input, cv::IMREAD_COLOR);
        if (decoded.empty()) return fail(input, "golden input does not decode");
        o.input_pixels_sha256 = pixels_sha256(decoded);
        o.input_rows = decoded.rows;
        o.input_cols = decoded.cols;
        o.pipeline_digest = pipeline;
        o.runtime_digest = cfg.runtime_digest;
        // The loader hashed exactly the bytes it parsed (always both files:
        // dual_model.enabled is required and true).
        o.dual_runtime_sha256 = config.dual_model().runtime_config_sha256;
        o.dual_schema_sha256 = config.dual_model().class_schema_sha256;
        const auto why = check_identity(m, o, partition_name(e.source));
        if (!why.empty()) return fail(mpath, why);
        // One joint run serves every engine, so all must be sealed on one input.
        if (!manifests.empty() && m.input_pixels_sha256 != manifests.front().input_pixels_sha256)
            return fail(mpath, "required engines were sealed on different inputs");
        if (image.empty()) image = decoded;
        manifests.push_back(m);
    }
    // One real inference; the longest sealed timeout bounds it.
    std::uint32_t timeout = 0;
    for (const auto& m : manifests) timeout = std::max(timeout, m.timeout_ms);
    const auto run = run_golden(config, image, 1, timeout);
    if (!run.error.empty() || run.delivered != 1)
        return fail(asset_dir(engines.front().engine_path), run.error.empty() ? "no golden result" : run.error);
    // Compare each partition with its sealed value; the summary is logged.
    std::ostringstream log;
    log << "PSC4_GOLDEN_OK rule=" << kRule;
    for (std::size_t i = 0; i < engines.size(); ++i) {
        const auto& out = run.outputs.front()[static_cast<std::size_t>(engines[i].source)];
        const auto why = check_output(manifests[i], out);
        if (!why.empty()) return fail(asset_dir(engines[i].engine_path) + "/manifest.json", why);
        log << " " << manifests[i].partition << "=" << out.sha256.substr(0, 12) << "/" << out.detections << "+"
            << out.map_detections << " build_env_" << manifests[i].partition << "=" << manifests[i].build_env.status;
    }
    log << " engine_load_ms=" << run.start_ms << " golden_result_ms=" << run.first_result_ms
        << " total_ms=" << run.total_ms;
    r.ok = true;
    r.summary = log.str();
    return r;
}

}  // namespace perception::service::psc4
