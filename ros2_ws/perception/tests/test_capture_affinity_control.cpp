/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_capture_affinity_control.cpp
 * Brief: Unit tests for pinning the capture thread to core 2 and restoring its inherited CPU set.
 *
 * Description:
 * What it checks. PERCEPTION_CAPTURE_AFFINITY_PIN2 narrows only the capture thread to core 2 after
 * its first complete frame and restores the inherited set on exit. With a mocked
 * sched_getaffinity/setaffinity it covers strict parsing, the disabled no-op, both modes running
 * the same first handoff, rejection of unreadable masks and of sets that lack core 2 or 3,
 * acceptance of a superset such as the U87 lent core (mask 140), set failures, readback checks and
 * restore. A final Linux-only case exercises the real system calls inside the runner's own mask.
 * Trap. The control must never widen a set or leave the loop silently: r7diag1_B1 showed a
 * rejected mask ending the native stream (fixed in bb0036c).
 */

#include "common/capture_affinity_control.hpp"
#include <array>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace perception;
int checks = 0;
void check(bool value) {
    ++checks;
    if (!value) throw std::runtime_error("capture affinity check " + std::to_string(checks));
}
struct Mock {
    int reads = 0, sets = 0, fail_set = 0;
    std::uint64_t mask = 12;
    std::array<std::uint64_t, 8> requested{};
    std::array<CaptureAffinitySnapshot, 8> overrides{};
    std::array<bool, 8> use_override{};
} mock;
CaptureAffinitySnapshot snapshot(int ordinal, std::uint64_t mask = 12) {
    CaptureAffinitySnapshot value;
    value.valid = true;
    value.reason = "ok";
    value.tid = 42;
    value.observed_ns = 1000 + ordinal * 100;
    value.mask = mask;
    value.policy = value.priority = value.nice = 0;
    return value;
}
CaptureAffinitySnapshot read_mock() noexcept {
    const auto index = static_cast<std::size_t>(mock.reads++);
    errno = EIO; // The control must not leak injected system-call errno.
    if (index < mock.use_override.size() && mock.use_override[index]) return mock.overrides[index];
    return snapshot(static_cast<int>(index), mock.mask);
}
int set_mock(std::uint64_t mask) noexcept {
    mock.requested[static_cast<std::size_t>(mock.sets++)] = mask;
    errno = EPERM;
    if (mock.fail_set == mock.sets) return EPERM;
    mock.mask = mask;
    return 0;
}
CaptureAffinityOps operations() { return {read_mock, set_mock}; }
void reset() { mock = Mock{}; }
void replace(int read_number, const CaptureAffinitySnapshot& value) {
    mock.use_override[static_cast<std::size_t>(read_number - 1)] = true;
    mock.overrides[static_cast<std::size_t>(read_number - 1)] = value;
}
bool reason(const CaptureAffinityRecord& record, const char* expected) {
    return std::string(record.reason) == expected;
}
struct Fault { const int* token; };

void actual_self_affinity() {
#ifdef __linux__
    errno = EDOM;
    const auto before = read_capture_affinity();
    check(errno == EDOM);
    if (!before.valid) {
        std::cout << "LINUX_SELF_AFFINITY_SKIP reason=" << before.reason << '\n';
        return;
    }
    // Narrow only within the inherited allowed mask. Never widen to CPUs 2,3
    // just for a test: runner test affinity is independent of camera affinity.
    const auto selected = before.mask & (~before.mask + 1U);
    struct Restore {
        std::uint64_t original;
        bool applied = false;
        ~Restore() { if (applied) (void)set_capture_affinity(original); }
    } restore{before.mask};
    const int result = set_capture_affinity(selected);
    if (result != 0) {
        std::cout << "LINUX_SELF_AFFINITY_SKIP reason=set_denied errno=" << result << '\n';
        return;
    }
    restore.applied = true;
    check(errno == EDOM);
    const auto narrowed = read_capture_affinity();
    check(narrowed.valid && narrowed.tid == before.tid && narrowed.mask == selected);
    check(same_capture_scheduling(before, narrowed));
    check(set_capture_affinity(before.mask) == 0);
    restore.applied = false;
    const auto after = read_capture_affinity();
    check(after.valid && after.mask == before.mask && after.tid == before.tid);
    check(same_capture_scheduling(before, after));
    std::cout << "LINUX_SELF_AFFINITY_PASS original_mask=" << before.mask
              << " narrowed_mask=" << selected << " restored_mask=" << after.mask << '\n';
    if ((before.mask & 12U) == 12U) {
        CaptureAffinityControl real(CaptureAffinityMode::Pin2);
        CaptureAffinityRestoreGuard guard(real, [](const CaptureAffinityRecord&) {});
        check(real.apply(7).success);
        check(real.restore().success);
        std::cout << "LINUX_SELF_CONTROL_PIN2_PASS\n";
    } else {
        std::cout << "LINUX_SELF_CONTROL_PIN2_SKIP reason=inherited_mask_lacks_2_3\n";
    }
#else
    errno = EDOM;
    const auto unsupported = read_capture_affinity();
    check(!unsupported.valid && std::string(unsupported.reason) == "unsupported_platform");
    check(set_capture_affinity(4) == ENOTSUP && errno == EDOM);
    std::cout << "LINUX_SELF_AFFINITY_SKIP reason=unsupported_platform\n";
#endif
}
} // namespace

int main() {
    try {
        check(parse_capture_affinity_control(nullptr) == CaptureAffinityMode::Disabled);
        check(parse_capture_affinity_control("0") == CaptureAffinityMode::Observe);
        check(parse_capture_affinity_control("1") == CaptureAffinityMode::Pin2);
        for (const char* value : {"", "true", "false", "01", "2", "-1", " 1", "1 ", "1\n"}) {
            bool rejected = false;
            try { (void)parse_capture_affinity_control(value); }
            catch (const std::invalid_argument& error) {
                rejected = std::string(error.what()).find("PERCEPTION_CAPTURE_AFFINITY_PIN2") != std::string::npos;
            }
            check(rejected);
        }

        // Absent variable is legacy no-op, including hosts with arbitrary masks:
        // no reads, no clock reads, no set calls, no restore or log callbacks.
        reset();
        mock.mask = 1;
        int disabled_logs = 0;
        {
            CaptureAffinityControl disabled(CaptureAffinityMode::Disabled, operations());
            CaptureAffinityRestoreGuard guard(disabled, [&](const CaptureAffinityRecord&) { ++disabled_logs; });
            check(!disabled.needs_apply() && !disabled.needs_restore());
            (void)disabled.apply(9);
            (void)disabled.restore();
            check(!disabled.needs_apply() && !disabled.needs_restore());
        }
        check(mock.reads == 0 && mock.sets == 0 && disabled_logs == 0 && mock.mask == 1);

        // Both arms first execute one common complete-frame handoff before any
        // affinity observation. Synthetic payload/call order is identical.
        std::array<std::vector<int>, 2> payloads;
        for (int pin = 0; pin < 2; ++pin) {
            reset();
            int logs = 0, complete_frames = 0;
            CaptureAffinityControl control(pin ? CaptureAffinityMode::Pin2 : CaptureAffinityMode::Observe, operations());
            {
                CaptureAffinityRestoreGuard guard(control, [&](const CaptureAffinityRecord& record) {
                    ++logs;
                    check(reason(record, "ok") && record.success);
                    check(std::string(record.event) == "restore" && record.frame_id == 5);
                    check(record.set_attempted == (pin != 0));
                });
                check(mock.reads == 0 && mock.sets == 0);
                payloads[pin] = {0, 12, 128, 255}; // stand-in, not SDK pixel validation
                ++complete_frames;
                const auto& applied = control.apply(5);
                check(complete_frames == 1 && applied.success && reason(applied, "ok"));
                check(applied.before.mask == 12 && applied.after.mask == (pin ? 4U : 12U));
                check(applied.before.tid == 42 && applied.after.tid == 42);
                check(applied.set_attempted == (pin != 0) && applied.set_errno == 0);
                check(same_capture_scheduling(applied.before, applied.after));
                check(control.needs_restore() && !control.needs_apply());
                check(mock.reads == 2 && mock.sets == pin);
                check(control.apply(999).frame_id == 5); // at most once
                check(mock.reads == 2 && mock.sets == pin);
                errno = EDOM;
                check(emit_capture_affinity(applied, [&](const CaptureAffinityRecord&) { ++logs; errno = ERANGE; }));
                check(errno == EDOM && logs == 1);
                std::ostringstream text;
                text << applied;
                check(text.str().find("CAPTURE_AFFINITY_CONTROL schema=1 clock=CLOCK_MONOTONIC mask_encoding=cpu_bitset_u64") == 0);
                check(text.str().find(" event=apply mode=" + std::string(pin ? "pin2" : "observe")) != std::string::npos);
                check(text.str().find(" frame_id=5 tid=42 before_ns=1000 after_ns=1100") != std::string::npos);
                check(text.str().find(" success=1 reason=ok") != std::string::npos);
                check(mock.reads == 2); // serialization never reads clocks
            }
            check(logs == 2 && mock.reads == 4 && mock.sets == pin * 2 && mock.mask == 12);
            check(!control.needs_restore());
            check(control.restore().success);
            check(mock.reads == 4 && mock.sets == pin * 2);
            if (pin) check(mock.requested[0] == 4 && mock.requested[1] == 12);
        }
        check(payloads[0] == payloads[1]);

        // No first complete frame => no apply, no restore, no system calls.
        reset();
        {
            CaptureAffinityControl never_started(CaptureAffinityMode::Pin2, operations());
            CaptureAffinityRestoreGuard guard(never_started, [](const CaptureAffinityRecord&) { check(false); });
        }
        check(mock.reads == 0 && mock.sets == 0);

        // Initial faults reject the experiment without ever trying a set: an
        // unreadable mask, a set without core 3 (2 only) and one without core 2 (3 and 7).
        for (int fault = 0; fault < 3; ++fault) {
            reset();
            auto bad = snapshot(0);
            if (fault == 0) { bad.valid = false; bad.error = EIO; bad.reason = "mock_read_failed"; }
            else bad.mask = fault == 1 ? 4U : 136U;
            replace(1, bad);
            CaptureAffinityControl control(CaptureAffinityMode::Pin2, operations());
            errno = EDOM;
            const auto& record = control.apply(1);
            check(errno == EDOM && !record.success && !record.set_attempted);
            check(reason(record, fault ? "initial_mask_lacks_2_3" : "initial_read_failed"));
            check(!control.needs_apply() && !control.needs_restore());
            check(mock.reads == 1 && mock.sets == 0);
        }
        // A superset of 2-3 (the U87 diagnostic lends core 7: mask 140) is accepted.
        // Pin2 still narrows to core 2 only and restore returns the whole inherited
        // set; Observe reads the inherited set back unchanged and never sets.
        for (int pin = 0; pin < 2; ++pin) {
            reset();
            mock.mask = 140;
            CaptureAffinityControl lent(pin ? CaptureAffinityMode::Pin2 : CaptureAffinityMode::Observe, operations());
            const auto& applied = lent.apply(3);
            check(applied.success && reason(applied, "ok"));
            check(applied.before.mask == 140 && applied.requested_mask == (pin ? 4U : 140U));
            check(applied.after.mask == (pin ? 4U : 140U) && mock.sets == pin);
            const auto& restored = lent.restore();
            check(restored.success && reason(restored, "ok") && mock.mask == 140);
            if (pin) check(mock.requested[0] == 4 && mock.requested[1] == 140);
        }
        reset();
        mock.fail_set = 1;
        CaptureAffinityControl denied(CaptureAffinityMode::Pin2, operations());
        check(reason(denied.apply(1), "setaffinity_failed"));
        check(!denied.needs_restore() && mock.reads == 2 && mock.sets == 1 && mock.mask == 12);
        check(denied.apply(2).set_errno == EPERM && mock.sets == 1);

        // A successful set requires readback validation; every failure still
        // restores its own changed mask on exit, never hides a failed apply.
        for (int fault = 0; fault < 5; ++fault) {
            reset();
            auto bad = snapshot(1, 4);
            const char* expected = nullptr;
            if (fault == 0) { bad.valid = false; bad.reason = "clock_failed"; bad.error = EIO; expected = "readback_failed"; }
            if (fault == 1) { bad.tid = 43; expected = "thread_identity_changed"; }
            if (fault == 2) { bad.observed_ns = 1; expected = "clock_regressed"; }
            if (fault == 3) { bad.mask = 12; expected = "readback_mask_mismatch"; }
            if (fault == 4) { bad.nice = 1; expected = "scheduling_metadata_changed"; }
            replace(2, bad);
            CaptureAffinityControl control(CaptureAffinityMode::Pin2, operations());
            check(reason(control.apply(1), expected) && !control.apply(1).success);
            check(control.needs_restore() && mock.mask == 4);
            check(control.restore().success && mock.mask == 12);
            check(mock.sets == 2 && mock.requested[1] == 12);
        }

        // Restore error cases remain explicit and are not silently retried.
        for (int fault = 0; fault < 7; ++fault) {
            reset();
            CaptureAffinityControl control(CaptureAffinityMode::Pin2, operations());
            check(control.apply(1).success);
            auto before = snapshot(2, 4), after = snapshot(3, 12);
            const char* expected = nullptr;
            if (fault == 0) { mock.fail_set = 2; expected = "restore_setaffinity_failed"; }
            if (fault == 1) { before.valid = false; before.reason = "clock_failed"; expected = "restore_snapshot_failed"; }
            if (fault == 2) { after.tid = 43; expected = "restore_thread_identity_mismatch"; }
            if (fault == 3) { after.observed_ns = 1; expected = "restore_clock_regressed"; }
            if (fault == 4) { before.mask = 1; expected = "restore_initial_mask_mismatch"; }
            if (fault == 5) { after.mask = 4; expected = "restore_readback_mask_mismatch"; }
            if (fault == 6) { after.policy = 1; expected = "restore_scheduling_metadata_changed"; }
            replace(3, before);
            replace(4, after);
            errno = EDOM;
            const auto& restored = control.restore();
            check(errno == EDOM && !restored.success && reason(restored, expected));
            check(!control.needs_restore() && mock.sets == 2);
            (void)control.restore();
            check(mock.sets == 2 && mock.reads == 4);
        }
        reset();
        CaptureAffinityControl wrong_thread(CaptureAffinityMode::Pin2, operations());
        check(wrong_thread.apply(1).success);
        auto foreign = snapshot(2, 4);
        foreign.tid = 99;
        replace(3, foreign);
        check(reason(wrong_thread.restore(), "restore_thread_identity_mismatch"));
        check(mock.sets == 1 && mock.reads == 3); // do not mutate a foreign calling task

        // Unwinding restores before the original exception is observed, and a
        // logging exception cannot escape the capture thread's exit guard.
        reset();
        const int token = 42;
        int restore_logs = 0;
        try {
            CaptureAffinityControl control(CaptureAffinityMode::Pin2, operations());
            CaptureAffinityRestoreGuard guard(control, [&](const CaptureAffinityRecord& record) {
                ++restore_logs;
                check(record.success && mock.mask == 12);
                throw std::runtime_error("log failed");
            });
            check(control.apply(8).success);
            throw Fault{&token};
        } catch (const Fault& fault) { check(fault.token == &token); }
        check(mock.mask == 12 && mock.sets == 2 && restore_logs == 1);
        errno = EDOM;
        check(!emit_capture_affinity(CaptureAffinityRecord{}, [](const CaptureAffinityRecord&) { errno = EIO; throw 1; }));
        check(errno == EDOM);

        actual_self_affinity();
        std::cout << "CAPTURE_AFFINITY_CONTROL_PASS checks=" << checks << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "CAPTURE_AFFINITY_CONTROL_FAIL " << error.what() << '\n';
        return 1;
    }
}
