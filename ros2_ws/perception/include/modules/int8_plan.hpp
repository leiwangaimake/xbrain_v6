/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: int8_plan.hpp
 * Brief: INT8 build plan for a DLA engine: tensor ranges from a calibration cache, FP16-kept layers.
 *
 * Description:
 * TensorRT 10.3 refuses a MinMax calibrator for a DLA network (it requires Entropy2), and Entropy2
 * lowered the COCO80 scores on Orin (2026-10-09, notes/DECISIONS.md st1009e). The INT8 engine is
 * therefore built from a TensorRT 10.3 calibration cache read as explicit dynamic ranges
 * (range = cache scale * 127), plus a list of layers kept in FP16 (the detect-head tail must be,
 * st1009o). Everything here is plain C++, so the refusals are unit-tested without TensorRT;
 * InferenceModule applies the resulting assignment to the TensorRT network.
 *
 * Files:
 *   ranges       the calibration cache text: a header line such as "TRT-100300-MinMaxCalibration",
 *                then one "<tensor name>: <8 hex digits>" line per tensor (the scale's float bits,
 *                most significant first)
 *   fp16 layers  one TensorRT layer name (the ONNX node name) per line; blank lines and lines
 *                starting with '#' are ignored
 * Both files are identified by SHA-256 in the configuration (the calibration source must be
 * traceable); a mismatch refuses the build.
 */
#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace yolo {

// The calibration cache: its header line and the scale of every tensor it names.
struct Int8RangeCache {
    std::string header;
    std::map<std::string, float> scales;
};

// Parses the cache text. Refuses an empty text, a header that is not TensorRT 10.3 ("TRT-100300-"
// followed by the calibrator kind, e.g. MinMaxCalibration), a line that is not
// "<name>: <8 hex digits>", a repeated name, and a scale that is not finite and positive or whose
// range (scale * 127) is not finite.
Int8RangeCache parse_int8_range_cache(const std::string& text);

// Parses the FP16 layer list. Refuses a repeated name, a list with no name at all, and a "halfq"
// line (a scratch-builder syntax for FP16 compute with an INT8 output, not supported here).
std::vector<std::string> parse_fp16_layer_list(const std::string& text);

// Symmetric INT8 dynamic range of a cache scale.
float int8_dynamic_range(float scale);

// The two files, read and checked against their configured SHA-256.
struct Int8Plan {
    std::string ranges_path;
    std::string ranges_sha256;
    std::string fp16_layers_path;     // empty: no layer is kept in FP16
    std::string fp16_layers_sha256;
    Int8RangeCache cache;
    std::vector<std::string> fp16_layers;
};

// Reads the ranges file (required) and the FP16 layer list (optional: empty path), checks each
// file's SHA-256 against the configured value and parses both.
Int8Plan load_int8_plan(const std::string& ranges_path, const std::string& ranges_sha256,
                        const std::string& fp16_layers_path, const std::string& fp16_layers_sha256);

// One network layer as the plan sees it: its name and its floating-point outputs.
struct Int8LayerView {
    struct Output {
        std::string tensor;
        bool network_output = false;
    };
    std::string name;
    std::vector<Output> float_outputs;
};

// What to do to the network.
struct Int8Assignment {
    std::vector<std::pair<std::string, float>> ranges;  // tensor name, dynamic range
    std::vector<std::size_t> fp16_layers;                // indices into the layer views
    std::vector<std::string> half_outputs;               // FP16 layers' outputs whose type becomes FP16
                                                         // (every float output except network outputs)
    std::size_t unranged_outputs = 0;                    // outputs of FP16 layers, left without a range
    std::size_t unused_cache_entries = 0;                // cache names that are not a network tensor
};

// Every FP16-listed name must be exactly one layer; the outputs of those layers get no range (so
// INT8 is impossible for them); every other floating-point tensor (network inputs and layer
// outputs) must have a cache scale. Refuses a listed name that is not a layer, a name that
// matches several layers, and a tensor that needs a range but has none.
Int8Assignment plan_int8_assignment(const std::vector<std::string>& float_inputs,
                                    const std::vector<Int8LayerView>& layers, const Int8Plan& plan);

}  // namespace yolo
