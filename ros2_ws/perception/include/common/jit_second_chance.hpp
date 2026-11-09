/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: jit_second_chance.hpp
 * Brief: JIT option: one more non-blocking take when the next-frame take found nothing.
 *
 * Description:
 * Single owner, one shot per live generation; no SDK, sleep, thread or queue policy of its own.
 * Off in the production runtime set (jit_second_chance: "0"); the empty-acquire mode is used
 * instead. Kept as a selectable option of the JIT loop.
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace perception {

inline bool parse_jit_second_chance_enabled(const char* value) {
    if (!value || std::string_view(value) == "0") return false;
    if (std::string_view(value) == "1") return true;
    throw std::invalid_argument("second chance must be exactly 0 or 1");
}

inline void validate_jit_second_chance_config(bool enabled, bool jit, bool async_submit,
    bool profiling, bool fresh_bind, std::size_t frame_queue_size) {
    if (enabled && (!jit || !async_submit || !profiling || fresh_bind || frame_queue_size != 1))
        throw std::invalid_argument(
            "second chance requires JIT, async submit, profiling, fresh_bind=0 and frame_queue_size=1");
}

struct JitSecondChanceGate {
    bool enabled = false, measuring = false, async_submit = false, fresh_bind = false;
    bool initial_attempted = false, initial_succeeded = false, initial_stopped = false;
    bool current_ready = false, next_slot_empty = false;
    std::uint64_t current_generation = 0, current_frame_id = 0;
};

struct JitSecondChanceTrace {
    std::int64_t begin_ns = -1, end_ns = -1;
    bool attempted = false, succeeded = false, stopped = false, discarded_on_stop = false;
    // Valid only when succeeded. The original pre-push field is NOT proof of
    // queue insertion completion; no raw-depth arrival estimate is substituted.
    std::uint64_t candidate_frame_id = 0;
    std::int64_t candidate_push_begin_ns = -1;
};

class JitSecondChance {
public:
    JitSecondChance(std::uint64_t generation, std::uint64_t frame_id)
        : generation_(generation), frame_id_(frame_id) {
        if (!generation) throw std::invalid_argument("second chance needs a live generation");
    }

    // Called once after wait_ready, before current postprocess. prepare owns
    // the original successful Frame via move; this helper never modifies IDs,
    // marks inference completion, submits a model or fabricates a replacement.
    template<class Frame, class Running, class Now, class TryPop, class Prepare>
    bool attempt(const JitSecondChanceGate& gate, Frame& frame, Running running,
                 Now now, TryPop try_pop, Prepare prepare) {
        if (visited_) throw std::logic_error("second chance already visited this live generation");
        visited_ = true;
        if (!gate.enabled || !gate.measuring || !gate.async_submit || gate.fresh_bind ||
            !gate.initial_attempted || gate.initial_succeeded || gate.initial_stopped ||
            !gate.current_ready || !gate.next_slot_empty) return false;
        if (gate.current_generation != generation_ || gate.current_frame_id != frame_id_)
            throw std::logic_error("second chance current generation/frame changed");
        if (!running()) { trace_.stopped = true; return false; }
        trace_.attempted = true;
        trace_.begin_ns = now();
        trace_.succeeded = try_pop(frame);
        trace_.end_ns = now();
        if (!trace_.succeeded) return false;
        trace_.candidate_frame_id = frame.frame_id;
        trace_.candidate_push_begin_ns = frame.pipeline_timing.frame_push_begin_ns;
        if (!running()) {
            trace_.stopped = true;
            trace_.discarded_on_stop = true;
            // The caller's scratch frame retains ownership until scope exit.
            // It never reaches a prepared slot, submit or completion journal.
            return false;
        }
        prepare(std::move(frame));
        return true;
    }

    const JitSecondChanceTrace& trace() const noexcept { return trace_; }

private:
    std::uint64_t generation_, frame_id_;
    bool visited_ = false;
    JitSecondChanceTrace trace_;
};
} // namespace perception
