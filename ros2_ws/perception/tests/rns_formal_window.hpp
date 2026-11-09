/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_formal_window.hpp
 * Brief: Offline-testable formal observation window over Objects results; never controls scheduling.
 *
 * Description:
 * What it is. The single Objects owner of an offline cadence observation: completed counts
 * successful native puts, not selection attempts; each selection is committed only after its real
 * successful put, and failure, identity or phase drift cannot be skipped to find a replacement
 * cohort. No global calibration getter is an input. Used by tests/test_rns_formal_window.cpp.
 */

/* Offline-testable observation window only; never controls model scheduling. */
#pragma once
#include "common/types.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace rns_smoke_support {

inline const char* jit_phase_name(yolo::JitResultPhase phase) noexcept {
    switch (phase) {
    case yolo::JitResultPhase::CalibrationWarmup: return "CALIBRATION_WARMUP";
    case yolo::JitResultPhase::Calibrating: return "CALIBRATING";
    case yolo::JitResultPhase::Measuring: return "MEASURING";
    default: return "UNAVAILABLE";
    }
}

struct FormalSelection {
    bool formal = false, after = false;
    std::size_t index = 0;
};

// Single Objects owner. completed counts successful native puts, not selection
// attempts. Each selection must be committed after its real successful put;
// failure/identity/phase drift cannot be skipped to find a replacement cohort.
// No global calibration getter is an input.
class FormalWindow {
public:
    FormalWindow(std::uint64_t epoch, std::int64_t startup_ms,
                 std::int64_t warmup_ms, std::size_t target)
        : epoch_(epoch), startup_ms_(startup_ms), warmup_ms_(warmup_ms), target_(target) {
        if (!epoch || startup_ms < 0 || warmup_ms < 0 || !target)
            throw std::invalid_argument("formal window configuration");
    }

    FormalSelection select(yolo::JitResultPhase phase, double delay_ms,
        const perception::native_depth::SourceId& source, std::uint64_t epoch,
        std::int64_t before_ns, std::size_t completed) {
        if (faulted_ || pending_) fail("formal window failed/uncommitted; no replacement cohort");
        if (epoch != epoch_ || !source.stream_id || !source.sequence ||
            before_ns < 0 || before_ns / 1000000 < startup_ms_ || completed > target_ ||
            completed != completed_ || (completed > 0 && !bound_))
            fail("formal window source/clock/count invalid");
        const bool measuring = phase == yolo::JitResultPhase::Measuring;
        const bool calibration = phase == yolo::JitResultPhase::CalibrationWarmup ||
                                 phase == yolo::JitResultPhase::Calibrating;
        if ((!measuring && !calibration) || !std::isfinite(delay_ms) ||
            (measuring && (delay_ms < 0 || delay_ms >=
                static_cast<double>(std::numeric_limits<std::int64_t>::max() / 1000))) ||
            (calibration && delay_ms != -1.0))
            fail("formal window invalid per-result JIT observation");
        const bool eligible = measuring && before_ns / 1000000 - startup_ms_ >= warmup_ms_;
        if (bound_ && (!eligible || source.stream_id != stream_id_ || delay_ms != delay_ms_))
            fail("formal window phase/source/delay changed; no replacement cohort");
        if (eligible && !bound_) {
            bound_ = true;
            stream_id_ = source.stream_id;
            delay_ms_ = delay_ms;
        }
        pending_ = true;
        pending_formal_ = eligible && completed < target_;
        return {pending_formal_, eligible && completed == target_,
                eligible && completed < target_ ? completed + 1 : 0};
    }

    void commit_successful_put(bool keep_formal=true) {
        if (faulted_ || !pending_) fail("formal window commit without valid selection");
        // Duration shell alone may discover that native put began at/after its
        // fixed cut. Keep the real wire row as after, without replacing it.
        if (pending_formal_ && keep_formal) ++completed_;
        pending_ = false;
    }

    bool bound() const noexcept { return bound_; }
    std::uint64_t clock_epoch() const noexcept { return bound_ ? epoch_ : 0; }
    std::uint64_t stream_id() const noexcept { return stream_id_; }
    double delay_ms() const noexcept { return delay_ms_; }

private:
    [[noreturn]] void fail(const char* reason) {
        faulted_ = true;
        throw std::runtime_error(reason);
    }
    std::uint64_t epoch_, stream_id_ = 0;
    std::int64_t startup_ms_, warmup_ms_;
    std::size_t target_;
    std::size_t completed_ = 0;
    bool bound_ = false, faulted_ = false, pending_ = false, pending_formal_ = false;
    double delay_ms_ = -1.0;
};
} // namespace rns_smoke_support
