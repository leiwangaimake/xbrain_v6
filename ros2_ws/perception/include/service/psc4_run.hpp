/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: psc4_run.hpp
 * Brief: PSC-4 golden frame on the target: the actual inference run and the identity observation.
 *
 * Description:
 * Device code (TensorRT engines, OpenCV).
 * Why PSC-4 exists (19 section 11 / 13.2). An engine, a model file, a runtime switch or a
 * pipeline key can change the detections without any error. Before the service starts, one
 * fixed image is run through the real engines and its canonical output digest must equal the
 * value sealed at installation; any difference refuses the start (exit 78).
 *
 * Shared by the one-off seal tool and the service startup check so that the
 * sealed value and the startup comparison are produced by the SAME code path:
 * a dedicated InferenceModule built from the resolved configuration with
 * ExistingEnginesOnly (never a rebuild), fed the fixed input, its results
 * reduced with the canonical rule of psc4_golden.hpp.
 */
#pragma once

#include "common/config.hpp"
#include "service/psc4_golden.hpp"
#include "service/service_config.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace perception::service::psc4 {

// Required engines of this configuration: the primary (coco80) engine and the
// active12 engine, always both (dual_model.enabled is required and true).
struct RequiredEngine {
    int source = kSourceCoco80;
    std::string engine_path;
};
std::vector<RequiredEngine> required_engines(const yolo::Config&);

// Digest of the resolved pipeline section (canonical YAML dump) the inference
// output depends on. Any change there invalidates the seal.
std::string pipeline_digest(const ServiceConfig&);

// SHA-256 and size of a file; throws std::runtime_error when unreadable.
std::string file_sha256(const std::string& path, std::uint64_t* bytes = nullptr);
// SHA-256 of the decoded BGR pixels (row-major, no padding).
std::string pixels_sha256(const cv::Mat& bgr);

// Outcome of run_golden(): per-repeat partition digests and timings.
struct GoldenRun {
    bool started = false;
    std::string error;                               // empty = all repeats delivered
    int delivered = 0;
    std::vector<std::array<PartitionOutput, 2>> outputs;   // [coco80, active12] per repeat
    std::int64_t start_ms = -1;                       // module start (engine load)
    std::int64_t first_result_ms = -1;                // push -> first result
    std::int64_t total_ms = -1;
};
// Runs `repeats` golden inferences through a dedicated module and stops it.
// timeout_ms bounds the wait for each result. Never builds an engine. The
// process must already be prepared as the service prepares it (runtime
// environment, OpenCV threads, preprocess mode): the module reads them.
GoldenRun run_golden(const yolo::Config&, const cv::Mat& bgr, int repeats, std::uint32_t timeout_ms);

// The service startup check (PSC-4). Reads every required engine's sealed
// assets, checks their identity against the configured engine files, the input
// and the resolved configuration, runs ONE real golden inference and compares
// each partition with its sealed value. ok=false carries the offending asset
// path (key) and the reason; the caller refuses to start (exit 78).
struct StartupCheck {
    bool ok = false;
    std::string key, reason;
    std::string summary;   // one log line of what was checked, with timings
};
StartupCheck startup_check(const ServiceConfig&, const yolo::Config&);

}  // namespace perception::service::psc4
