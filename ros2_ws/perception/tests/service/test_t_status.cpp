/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_t_status.cpp
 * Brief: Status side of T in a simulated run: the 10 s capability flip, recovery and reason edges.
 *
 * Description:
 * The capability flips after 10 s without a slot publication and comes back only with a real
 * publication (19 section 3.3 / 5.1 / 5.2; the capability part of the A19-TIME-2 scenario, host
 * level, B9-13 (a)).
 * Status cuts every 10 ms (cut = last completed millisecond, sampled clock = cut + 1), the T owner
 * ready from 2 s, publications every 50 ms at 50 k + 1 ms (0.4 ms into that millisecond, so every
 * fifth cut has a publication in its still-open millisecond), a 12.5 s producer stall, then
 * publications again. Checks:
 *   1. before the T owner exists: not available, reason raised;
 *   2. while publishing: available on EVERY cut (the 1.8 bug: a publication in the open
 *      millisecond was dropped and the reason flapped), with some cuts counted in_open_ms;
 *   3. during the stall: still available while cut - last < 10000 ms, false exactly from 10000 ms;
 *      cuts (heartbeats) without a publication never bring it back;
 *   4. the first publication after the stall restores it;
 *   5. ReasonEdges sees exactly: warn (no T yet), info (T ready), warn (stall), info (recovery);
 *   6. a publication time after the sampled clock (another clock) is never used.
 */

#include "service/t_status.hpp"

#include "output/rns_reason_edges.hpp"

#include <cstdint>
#include <iostream>
#include <vector>

using perception::rns::ReasonEdge;
using perception::rns::ReasonEdges;
using perception::rns::ReasonEdgeSeverity;
using perception::service::seg_capability;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)
constexpr std::int64_t kWindow = 10000;
}  // namespace

int main() {
    const std::int64_t start = 100000, ready_at = start + 2000, stall_from = start + 20000,
                       stall_to = stall_from + 12500, end = stall_to + 3000;
    ReasonEdges edges(start, 1, 64);
    std::vector<ReasonEdge> seen;
    std::int64_t last_publish_ns = -1;
    std::int64_t in_open = 0, flip_at = -1, recovered_at = -1;
    bool publishing_always_available = true;
    // Publications at 50 k + 1 ms (0.4 ms into that millisecond) and cuts at multiples of 10 ms:
    // every fifth cut's still-open millisecond (cut + 1) holds a publication.
    std::int64_t next_publication = start + 1;
    for (std::int64_t cut = start; cut <= end; cut += 10) {
        const std::int64_t sampled = cut + 1;
        // Everything published up to the sampled clock (the Status owner reads the slot time
        // before it samples the clock).
        while (next_publication <= sampled) {
            if (next_publication >= ready_at && (next_publication < stall_from || next_publication >= stall_to))
                last_publish_ns = next_publication * 1000000 + 400000;
            next_publication += 50;
        }
        const bool ready = cut >= ready_at;
        const auto c = seg_capability(ready && last_publish_ns >= 0, last_publish_ns, cut, sampled, kWindow);
        CHECK(!c.clock_anomaly);
        in_open += c.in_open_ms;
        CHECK(c.raise_reason == !c.available);
        if (c.last_update_ms) CHECK(*c.last_update_ms <= cut);
        if (!ready) CHECK(!c.available);
        const std::int64_t last_ms = last_publish_ns / 1000000;
        if (ready && last_publish_ns >= 0 && cut < stall_from + kWindow - 50 && cut >= ready_at + 50)
            publishing_always_available &= c.available;
        if (cut >= stall_from && cut < stall_to) {
            // The same rule apply_health_fields checks, with the last publication before the stall.
            CHECK(c.available == (cut - last_ms < kWindow));
            if (!c.available && flip_at < 0) flip_at = cut;
        }
        if (cut >= stall_to && c.available && recovered_at < 0) recovered_at = cut;
        const auto update = edges.update(c.raise_reason ? 1U : 0U, cut);
        CHECK(update.code != perception::rns::ReasonEdgeCode::Overflow &&
              update.code != perception::rns::ReasonEdgeCode::Invalid);
        ReasonEdge e;
        while (edges.pop(e)) seen.push_back(e);
    }
    CHECK(publishing_always_available);
    CHECK(in_open > 0);
    const std::int64_t last_before_stall_ms = stall_from - 49;
    CHECK(flip_at >= last_before_stall_ms + kWindow && flip_at <= last_before_stall_ms + kWindow + 10);
    CHECK(recovered_at >= stall_to && recovered_at <= stall_to + 10);
    CHECK(seen.size() == 4);
    if (seen.size() == 4) {
        CHECK(seen[0].severity == ReasonEdgeSeverity::Warn && seen[0].observed_ms == start);
        CHECK(seen[1].severity == ReasonEdgeSeverity::Info && seen[1].observed_ms >= ready_at &&
              seen[1].observed_ms <= ready_at + 10);
        CHECK(seen[2].severity == ReasonEdgeSeverity::Warn && seen[2].observed_ms == flip_at);
        CHECK(seen[3].severity == ReasonEdgeSeverity::Info && seen[3].observed_ms == recovered_at);
    }
    // 6. Another clock: a time after the sampled clock is never used.
    const auto anomaly = seg_capability(true, (start + 500) * 1000000, start + 100, start + 101, kWindow);
    CHECK(anomaly.clock_anomaly && !anomaly.last_update_ms && !anomaly.available && anomaly.raise_reason);
    // The open-millisecond case on its own: published 0.4 ms after the cut, before the sampled clock.
    const auto open = seg_capability(true, (start + 100) * 1000000 + 400000 + 1000000, start + 100, start + 101, kWindow);
    CHECK(open.in_open_ms && open.available && open.last_update_ms == start + 100);
    std::cout << "test_t_status: " << (failures == 0 ? "PASS" : "FAIL") << " checks=" << checks
              << " in_open_cuts=" << in_open << " flip_at_ms=" << flip_at - start << " recovered_at_ms="
              << recovered_at - start << " edges=" << seen.size() << "\n";
    return failures == 0 ? 0 : 1;
}
