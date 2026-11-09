/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: preprocess_legacy_api.hpp
 * Brief: Declaration of the frozen original letterbox preprocess, the oracle for preprocess tests.
 *
 * Description:
 * What it is. Tests call this frozen legacy entry point directly and never use the environment
 * dispatcher as their own oracle, so a change in the dispatcher cannot hide behind itself.
 */

#pragma once
#include "common/preprocess.hpp"
// Tests call frozen original directly, never the ENV dispatcher as oracle.
namespace yolo {
void preprocess_into_legacy(const cv::Mat&, int, LetterboxInfo&, float*, std::size_t);
}
