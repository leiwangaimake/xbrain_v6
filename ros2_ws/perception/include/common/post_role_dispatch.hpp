/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: post_role_dispatch.hpp
 * Brief: Which lane runs which branch postprocess (PERCEPTION_POST_ROLE_SWAP).
 *
 * Description:
 * Default (and the production set, post_role_swap: "0"): COCO80 on the helper lane, Active12 on
 * the caller. The swap is a diagnostic option; Active12 keeps exception precedence either way.
 */

#pragma once

#include "common/parallel_dispatch_trace.hpp"
#include <cstring>
#include <stdexcept>

namespace perception {

inline bool parse_post_role_swap(const char* value) {
    if (!value || std::strcmp(value, "0") == 0) return false;
    if (std::strcmp(value, "1") == 0) return true;
    throw std::invalid_argument("PERCEPTION_POST_ROLE_SWAP must be exactly 0 or 1");
}

inline void validate_post_role_swap_config(bool enabled, bool profiled, bool jit, bool dual_model) {
    if (enabled && (!profiled || !jit || !dual_model))
        throw std::invalid_argument("PERCEPTION_POST_ROLE_SWAP requires profiled dual-model diagnostic JIT");
}

inline const char* post_helper_role(bool swapped) noexcept { return swapped ? "active" : "coco"; }
inline const char* post_caller_role(bool swapped) noexcept { return swapped ? "coco" : "active"; }

// Only dispatch placement changes. Callbacks retain their model-owned read-only
// inputs, destination vectors and CPU records; their caller merges in the same
// order after both complete. Active retains exception precedence in BOTH modes.
inline void run_post_tasks(ParallelTaskWorker& worker, bool swapped,
                         std::function<void()> coco, std::function<void()> active,
                         ExecutionCpuRecord* join_trace = nullptr) {
    if (swapped) {
        worker.run(std::move(active), std::move(coco), join_trace,
                   ParallelTaskWorker::ExceptionPriority::Helper);
    } else {
        worker.run(std::move(coco), std::move(active), join_trace);
    }
}

} // namespace perception
