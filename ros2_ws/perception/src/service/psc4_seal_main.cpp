/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: psc4_seal_main.cpp
 * Brief: PSC-4 golden assets: the one-off sealing step (installation or candidate time).
 *
 * Description:
 * Diagnostic tool, never part of perception_node.
 *   perception_psc4_seal <input.png> <repeats 5..200> <build_env_coco80.json>
 *                        <build_env_active12.json>
 *
 * Runs the fixed real input through the configured engines `repeats` times and
 * seals the result ONLY if every repeat agrees, per partition, and every
 * required partition produced a non-empty output. Writes, for every required
 * engine, <engine path>.psc4/{manifest.json,input.png}. Refuses to overwrite an
 * existing asset directory: re-sealing is an explicit, reviewed act.
 *
 * The process is prepared EXACTLY as perception_service_main.cpp prepares it
 * before its inference module exists (inherited switches refused, runtime
 * environment applied, OpenCV threads, pipeline configuration, preprocess
 * mode), so the sealed value comes from the service's own inference path.
 *
 * The build environment is NOT observed here: it is a record supplied from the
 * engine's own build evidence (or UNKNOWN), never today's environment.
 */
#include "common/config.hpp"
#include "common/preprocess_mode.hpp"
#include "service/psc4_golden.hpp"
#include "service/psc4_run.hpp"
#include "service/service_config.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace perception::service;

namespace {
// The build environment record supplied by the operator (see the header).
psc4::BuildEnv read_build_env(const std::string& path) {
    const auto node = YAML::LoadFile(path);
    psc4::BuildEnv env{node["status"].as<std::string>(), node["detail"].as<std::string>()};
    if (env.status != "RECORDED_AT_BUILD" && env.status != "RECOVERED_BY_ASSOCIATION" && env.status != "UNKNOWN")
        throw std::runtime_error("build env status must be RECORDED_AT_BUILD, RECOVERED_BY_ASSOCIATION or UNKNOWN");
    return env;
}
}  // namespace

int main(int argc, char** argv, char** envp) {
    if (argc != 5) {
        std::cerr << "usage: perception_psc4_seal <input.png> <repeats 5..200> <build_env_coco80.json> "
                     "<build_env_active12.json>\n";
        return 2;
    }
    const std::string input_path = argv[1];
    // At least 5 repeats to show repeatability. The 20 s per-result bound is
    // sealed into the manifest and becomes the startup check's timeout.
    const int repeats = std::atoi(argv[2]);
    if (repeats < 5 || repeats > 200) return 2;
    constexpr std::uint32_t kTimeoutMs = 20000;
    try {
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
        const psc4::BuildEnv env[2] = {read_build_env(argv[3]), read_build_env(argv[4])};

        // Never overwrite: re-sealing means removing the old assets on purpose.
        const auto engines = psc4::required_engines(config);
        for (const auto& e : engines)
            if (fs::exists(psc4::asset_dir(e.engine_path)))
                throw std::runtime_error("asset directory exists, refusing to overwrite: " + psc4::asset_dir(e.engine_path));

        // The input is hashed as a file and as decoded pixels, as the startup check will do.
        const cv::Mat image = cv::imread(input_path, cv::IMREAD_COLOR);
        if (image.empty()) throw std::runtime_error("cannot decode " + input_path);
        const auto input_sha = psc4::file_sha256(input_path);
        const auto pixels_sha = psc4::pixels_sha256(image);

        // The same run code as the startup check, repeated.
        const auto run = psc4::run_golden(config, image, repeats, kTimeoutMs);
        if (!run.error.empty()) throw std::runtime_error(run.error);
        if (run.delivered != repeats) throw std::runtime_error("not every repeat delivered a result");

        // Every engine is validated BEFORE anything is written: a failure on
        // the second engine must not leave a half-sealed asset set behind.
        std::vector<std::pair<std::string, psc4::Manifest>> sealed;
        for (const auto& e : engines) {
            const auto slot = static_cast<std::size_t>(e.source);
            const auto& ref = run.outputs.front()[slot];
            for (const auto& o : run.outputs)
                if (o[slot].sha256 != ref.sha256 || o[slot].detections != ref.detections ||
                    o[slot].map_detections != ref.map_detections)
                    throw std::runtime_error(std::string("partition ") + std::string(psc4::partition_name(e.source)) +
                                             " is not repeatable: refusing to seal");
            if (ref.detections + ref.map_detections == 0)
                throw std::runtime_error(std::string("partition ") + std::string(psc4::partition_name(e.source)) +
                                         " produced no output on this input: choose an input both models detect on");
            // The manifest records every identity startup_check compares (engine, input,
            // pipeline and runtime digests, dual-model files) and the sealed output itself.
            psc4::Manifest m;
            m.rule = std::string(psc4::kRule);
            m.partition = std::string(psc4::partition_name(e.source));
            m.engine_file = fs::path(e.engine_path).filename().string();
            m.engine_sha256 = psc4::file_sha256(e.engine_path, &m.engine_bytes);
            m.input_file = "input.png";
            m.input_sha256 = input_sha;
            m.input_pixels_sha256 = pixels_sha;
            m.input_rows = image.rows;
            m.input_cols = image.cols;
            m.pipeline_digest = psc4::pipeline_digest(cfg);
            m.runtime_digest = cfg.runtime_digest;
            m.dual_runtime_sha256 = config.dual_model().runtime_config_sha256;
            m.dual_schema_sha256 = config.dual_model().class_schema_sha256;
            m.expected_detections = ref.detections;
            m.expected_map_detections = ref.map_detections;
            m.expected_sha256 = ref.sha256;
            m.timeout_ms = kTimeoutMs;
            m.sealed_repeats = static_cast<std::uint32_t>(repeats);
            m.build_env = env[slot];
            (void)psc4::parse_manifest(psc4::write_manifest(m));   // the sealed file must pass its own reader
            sealed.emplace_back(psc4::asset_dir(e.engine_path), m);
        }
        // Write the asset directories and print one JSON summary line.
        std::cout << "{\"tool\":\"perception_psc4_seal\",\"repeats\":" << repeats << ",\"engine_load_ms\":" << run.start_ms
                  << ",\"first_result_ms\":" << run.first_result_ms << ",\"total_ms\":" << run.total_ms << ",\"engines\":[";
        // Only now are the asset directories created (input copy + manifest per engine).
        bool first = true;
        for (const auto& [dir, m] : sealed) {
            fs::create_directory(dir);
            fs::copy_file(input_path, dir + "/input.png");
            std::ofstream(dir + "/manifest.json") << psc4::write_manifest(m);
            if (!first) std::cout << ",";
            first = false;
            std::cout << "{\"partition\":\"" << m.partition << "\",\"engine_sha256\":\"" << m.engine_sha256
                      << "\",\"detections\":" << m.expected_detections << ",\"map_detections\":"
                      << m.expected_map_detections << ",\"expected_sha256\":\"" << m.expected_sha256
                      << "\",\"build_env\":\"" << m.build_env.status << "\",\"asset_dir\":\"" << dir << "\"}";
        }
        std::cout << "]}" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "PSC4_SEAL_REFUSED reason=\"" << e.what() << "\"\n";
        return 1;
    }
    return 0;
}
