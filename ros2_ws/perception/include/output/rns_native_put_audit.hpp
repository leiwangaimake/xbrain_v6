/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_native_put_audit.hpp
 * Brief: Timing and codes of one native RT put, without any Zenoh type.
 *
 * Description:
 * ZenohRtTransport fills one NativePutAudit per put (rns_zenoh_transport.hpp). The Profile steady
 * loop (service/profile_steady_loop.hpp) and its timing ledger (service/profile_timing.hpp) only
 * read it, and those are also built into host tests that have no zenoh-c. So the type lives here,
 * unchanged, and rns_zenoh_transport.hpp includes this header (moved 2026-10-14, A19-TIME-2).
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace perception::rns {

// Timing and codes of the last put, for diagnostics (monotonic ns, -1 = not reached).
struct NativePutAudit {
    std::size_t bytes = 0;
    bool native_called = false, clock_valid = false;
    int copy_code = 0, put_code = 0;
    std::int64_t copy_begin_ns = -1, copy_end_ns = -1;
    // copy bracket includes taking a pool slot (or the malloc fallback), the copy,
    // byte ownership construction and put options. Finalizer bracket contains the
    // actual shared wire clock sample.
    std::int64_t finalizer_begin_ns = -1, finalizer_end_ns = -1;
    bool finalized = false, cancelled = false;
    std::int64_t put_begin_ns = -1, put_end_ns = -1;
    // Every pool slot was still held by zenoh-c, so this packet was copied into a
    // malloc'd buffer (the pre-pool path). Never expected in steady state.
    bool pool_fallback = false;
};

}  // namespace perception::rns
