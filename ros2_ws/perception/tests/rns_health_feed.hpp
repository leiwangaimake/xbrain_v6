/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_health_feed.hpp
 * Brief: Test feed for HealthWindow through the same interfaces the service uses.
 *
 * Description:
 * What it is. The service feeds HealthWindow only through the SourceId rate overloads and the
 * split publication pair (observe_publication_attempt, then resolve_publication), from
 * PublicationHealthHandoff::drain(). Host tests describe a rate sample by one millisecond capture
 * time and a put by one complete audit; these helpers turn both into those production calls, so
 * the window needs no test-only entry point.
 * Rate samples. rate_source(capture) builds a SourceId whose sequence, indices and device times all
 * grow with the capture and whose host ready time floors to the capture in milliseconds. An equal
 * capture is therefore an exact replay (Duplicate), a smaller one an older sequence (OutOfOrder)
 * and a negative one an invalid source (Invalid).
 * Puts. publish() gives the audit a fresh process-wide seq, records the attempt and, only when it
 * was accepted, resolves it at the same ingest time; otherwise it returns the attempt's code.
 * Every helper is an inline function at namespace scope, so a test that uses only some of them
 * still builds with -Wall -Werror (no unused-function warning).
 * Used by tests/test_rns_health_window.cpp, test_rns_health_pending.cpp,
 * test_rns_health_fields.cpp, test_rns_reason_flow.cpp and test_rns_zenoh_generated.cpp.
 */
#pragma once
#include "common/rgbd_source_identity.hpp"
#include "output/rns_health_window.hpp"
#include <cassert>
#include <cstdint>
#include <limits>

namespace perception::rns::test_feed {
// One synthetic native frame per millisecond capture (see the file description).
inline native_depth::SourceId rate_source(Millis capture) noexcept {
    constexpr Millis kNsPerMs = 1000000;
    // host_ready_mono_ns = capture * 1e6 + 1 must stay representable.
    assert(capture >= std::numeric_limits<std::int64_t>::min() / kNsPerMs &&
           capture <= (std::numeric_limits<std::int64_t>::max() - 1) / kNsPerMs);
    native_depth::SourceId source;
    source.stream_id = 1;
    // capture -1 gives sequence 0; any negative capture gives host_ready_mono_ns <= 0.
    source.sequence = static_cast<std::uint64_t>(capture + 1);
    source.color_index = static_cast<std::uint64_t>(capture);
    source.depth_index = static_cast<std::uint64_t>(capture);
    source.color_device_us = static_cast<std::uint64_t>(capture + 1);
    source.depth_device_us = static_cast<std::uint64_t>(capture + 1);
    source.host_ready_mono_ns = capture * kNsPerMs + 1;
    return source;
}
// Depth arrival of the frame captured at `capture` (window stream 0).
inline HealthEventCode depth(HealthWindow& w, Millis capture, Millis event, Millis ingest) noexcept {
    return w.observe_depth(rate_source(capture), event, ingest);
}
// Inference completion of the frame captured at `capture` (window stream 1).
inline HealthEventCode inference(HealthWindow& w, Millis capture, Millis event, Millis ingest) noexcept {
    return w.observe_inference(rate_source(capture), event, ingest);
}
// One complete put, as the handoff delivers it: Attempt, then Completion, same ingest time.
inline HealthEventCode publish(HealthWindow& w, Topic topic, PublishAudit audit, Millis ingest) noexcept {
    // Process-wide and strictly increasing, so every window sees a fresh seq on every topic.
    static std::uint64_t seq = 0;
    audit.seq = ++seq;
    const auto code = w.observe_publication_attempt(topic, audit.seq, audit.capture_ms,
                                                  audit.put_before_ms, ingest);
    // A refused attempt (Duplicate, OutOfOrder, Invalid, ...) leaves nothing to resolve.
    return code == HealthEventCode::Accepted ? w.resolve_publication(topic, audit, ingest) : code;
}
}  // namespace perception::rns::test_feed
