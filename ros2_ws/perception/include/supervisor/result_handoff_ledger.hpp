/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: result_handoff_ledger.hpp
 * Brief: frame_id ledger of the inference -> Supervisor result handoff.
 *
 * Description:
 *   1. Records which frame_ids the Supervisor consumed (popped), which the
 *      latest-wins slot overwrote and which stop cleared, so every result that
 *      was not published is accounted for by identity, not only by count.
 *   2. Online identity checks: popped ids and overwritten ids are each strictly
 *      increasing (one in-order producer, one consumer), and no id is both
 *      popped and discarded (checked against short rings of recent ids). These
 *      only count; R6 rejects a duplicate or out-of-order frame on its own.
 *   3. drain() hands the ids recorded since the previous drain to a log line.
 *      A window lists at most kWindowIds ids per kind and counts the rest.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace perception {

struct ResultHandoffWindow {
    // Cumulative since construction.
    std::uint64_t popped = 0, overwritten = 0, cleared = 0;
    std::uint64_t order_violations = 0, identity_conflicts = 0;
    // Since the previous drain().
    std::uint64_t window_popped = 0, window_first_popped_id = 0, window_last_popped_id = 0;
    std::vector<std::uint64_t> overwritten_ids, cleared_ids;
    std::uint64_t ids_not_listed = 0;
};

class ResultHandoffLedger {
public:
    static constexpr std::size_t kWindowIds = 64;
    static constexpr std::size_t kRecent = 16;

    /** @brief Consumer thread, once per result taken from the handoff. */
    void on_popped(std::uint64_t frame_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++out_.popped;
        if (have_popped_ && frame_id <= last_popped_) ++out_.order_violations;
        if (recent_discarded_.contains(frame_id)) ++out_.identity_conflicts;
        have_popped_ = true;
        last_popped_ = frame_id;
        recent_popped_.remember(frame_id);
        if (out_.window_popped++ == 0) out_.window_first_popped_id = frame_id;
        out_.window_last_popped_id = frame_id;
    }

    /** @brief Queue discard observer: an overwritten or a cleared result. */
    void on_discarded(std::uint64_t frame_id, bool cleared) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (recent_popped_.contains(frame_id) || recent_discarded_.contains(frame_id))
            ++out_.identity_conflicts;
        recent_discarded_.remember(frame_id);
        std::vector<std::uint64_t>& ids = cleared ? out_.cleared_ids : out_.overwritten_ids;
        if (cleared) {
            ++out_.cleared;
        } else {
            ++out_.overwritten;
            if (have_overwritten_ && frame_id <= last_overwritten_) ++out_.order_violations;
            have_overwritten_ = true;
            last_overwritten_ = frame_id;
        }
        if (ids.size() < kWindowIds) ids.push_back(frame_id);
        else ++out_.ids_not_listed;
    }

    /** @brief Cumulative totals plus the window since the previous drain(). */
    ResultHandoffWindow drain() {
        std::lock_guard<std::mutex> lock(mutex_);
        ResultHandoffWindow window = out_;
        out_.window_popped = out_.window_first_popped_id = out_.window_last_popped_id = 0;
        out_.overwritten_ids.clear();
        out_.cleared_ids.clear();
        out_.ids_not_listed = 0;
        return window;
    }

private:
    // The last kRecent ids of one kind; frame_id 0 is a real capture frame.
    struct Ring {
        std::array<std::uint64_t, kRecent> ids{};
        std::size_t filled = 0, next = 0;
        bool contains(std::uint64_t id) const {
            return std::find(ids.begin(), ids.begin() + static_cast<std::ptrdiff_t>(filled), id) !=
                ids.begin() + static_cast<std::ptrdiff_t>(filled);
        }
        void remember(std::uint64_t id) {
            ids[next] = id;
            next = (next + 1) % kRecent;
            filled = std::min(filled + 1, kRecent);
        }
    };

    std::mutex mutex_;
    ResultHandoffWindow out_;
    bool have_popped_ = false, have_overwritten_ = false;
    std::uint64_t last_popped_ = 0, last_overwritten_ = 0;
    Ring recent_popped_, recent_discarded_;
};

}  // namespace perception
