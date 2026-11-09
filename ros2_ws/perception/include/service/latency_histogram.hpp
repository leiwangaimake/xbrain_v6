/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: latency_histogram.hpp
 * Brief: Bounded single-owner latency ledger for runtime self-measurement (19 section 2.2).
 *
 * Description:
 * It measures G-P1 (callback return -> put return) and the put duration.
 * Fixed-width bins allocated once; add() never allocates. Samples beyond the
 * last bin are counted in an overflow bucket; the exact maximum is kept.
 * Quantiles use nearest rank ceil(q*N) (19 section 5.1, reply Q21) and report
 * the UPPER edge of the bin holding that rank (or the exact max when the rank
 * falls in overflow): never below the true sample quantile. Threshold counts
 * are exact ("> threshold") for the thresholds given at construction.
 * Negative samples are invalid and counted separately, never clamped.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace perception::service {

class LatencyHistogram {
public:
    static constexpr std::size_t kMaxThresholds = 4;

    // All storage is allocated here; the owner thread then only calls add().
    LatencyHistogram(std::int64_t bin_width_ns, std::size_t bins, std::vector<std::int64_t> thresholds_ns = {})
        : width_(bin_width_ns), counts_(bins, 0) {
        if (bin_width_ns <= 0 || bins == 0 || thresholds_ns.size() > kMaxThresholds)
            throw std::invalid_argument("latency histogram needs positive width, bins and <= 4 thresholds");
        for (std::size_t i = 0; i < thresholds_ns.size(); ++i) {
            if (thresholds_ns[i] < 0) throw std::invalid_argument("negative latency threshold");
            thresholds_[i] = thresholds_ns[i];
        }
        threshold_count_ = thresholds_ns.size();
    }

    // One sample: bin (or overflow), exact max, and every threshold counter.
    void add(std::int64_t sample_ns) noexcept {
        if (sample_ns < 0) {
            ++invalid_;
            return;
        }
        ++count_;
        max_ = std::max(max_, sample_ns);
        const auto bin = static_cast<std::uint64_t>(sample_ns / width_);
        if (bin < counts_.size()) ++counts_[static_cast<std::size_t>(bin)];
        else ++overflow_;
        for (std::size_t i = 0; i < threshold_count_; ++i)
            if (sample_ns > thresholds_[i]) ++over_[i];
    }

    // Valid samples recorded (overflow included, invalid excluded).
    std::uint64_t count() const noexcept { return count_; }
    // Negative samples (e.g. a clock read out of order), refused rather than clamped to 0.
    std::uint64_t invalid() const noexcept { return invalid_; }
    // Samples at or beyond bins * width; they still count and still update the maximum.
    std::uint64_t overflow() const noexcept { return overflow_; }
    // -1 when empty.
    std::int64_t max_ns() const noexcept { return count_ ? max_ : -1; }
    // Exact count of samples strictly greater than thresholds_ns[index].
    std::uint64_t over(std::size_t index) const noexcept { return index < threshold_count_ ? over_[index] : 0; }
    // The threshold at `index` as given at construction, -1 beyond the configured ones.
    std::int64_t threshold(std::size_t index) const noexcept { return index < threshold_count_ ? thresholds_[index] : -1; }

    // -1 when empty or q outside (0, 1].
    std::int64_t quantile_upper_ns(double q) const noexcept {
        if (count_ == 0 || !(q > 0) || q > 1) return -1;
        // Nearest rank, then the upper edge of the bin that holds it (capped by
        // the exact maximum, so the answer is never above any real sample max).
        const auto rank = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::ceil(q * static_cast<double>(count_))));
        std::uint64_t seen = 0;
        for (std::size_t i = 0; i < counts_.size(); ++i) {
            seen += counts_[i];
            if (seen >= rank) return std::min<std::int64_t>(static_cast<std::int64_t>(i + 1) * width_, max_);
        }
        return max_;
    }

private:
    // Bin width in ns (fixed at construction; the log lines print it as the resolution).
    std::int64_t width_;
    // counts_[i] holds samples in [i*width, (i+1)*width).
    std::vector<std::uint64_t> counts_;
    // Up to four exact "> threshold" counters (e.g. the 14 ms G-P1 bound), independent of the
    // bin width so the gate count is exact even when the bins are coarse.
    std::array<std::int64_t, kMaxThresholds> thresholds_{};
    std::array<std::uint64_t, kMaxThresholds> over_{};
    std::size_t threshold_count_ = 0;
    std::uint64_t count_ = 0, invalid_ = 0, overflow_ = 0;
    std::int64_t max_ = 0;
};

}  // namespace perception::service
