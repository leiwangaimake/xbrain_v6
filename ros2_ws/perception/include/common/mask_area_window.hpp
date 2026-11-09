/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: mask_area_window.hpp
 * Brief: Per-window box (ROI) and mask foreground areas per instance, per model branch.
 *
 * Description:
 * Acceptance list (Perception完整验收清单, section 共载与场景, anchor "mask/ROI面积"): every window
 * records the mask/ROI area.
 * User decision 2026-09-30 (B9-17 (b)): record both the ROI (bbox) area and the foreground
 * pixel count of every instance mask, separately for COCO80, Active12 without class 92, and the
 * Active12 traversable class 92 (whose masks feed T), with the mean and the 99th percentile per
 * window. The percentile comes from a fixed log-spaced histogram (4 buckets per octave, up to
 * 2^21 pixels, i.e. above 1280x800), reported as the bucket's upper edge ("p99_le"), like the
 * service's latency histograms. Pure counting: no OpenCV, no allocation, single writer.
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace perception {

class MaskAreaWindow {
public:
    enum Branch : std::size_t { kCoco80 = 0, kActive12 = 1, kClass92 = 2, kBranches = 3 };
    static constexpr std::size_t kBuckets = 4 * 21 + 1;  // [0,1], then 4 per octave up to 2^21

    struct Distribution {
        std::uint64_t n = 0, sum = 0;
        std::array<std::uint32_t, kBuckets> histogram{};
    };
    struct BranchRecord {
        Distribution roi, foreground;
    };

    void add(Branch branch, std::uint64_t roi_pixels, std::uint64_t foreground_pixels) noexcept {
        auto& b = branches_[branch];
        add(b.roi, roi_pixels);
        add(b.foreground, foreground_pixels);
    }
    void frame() noexcept { ++frames_; }
    std::uint64_t frames() const noexcept { return frames_; }
    const BranchRecord& branch(Branch b) const noexcept { return branches_[b]; }
    void reset() noexcept { *this = MaskAreaWindow{}; }

    static double mean(const Distribution& d) noexcept {
        return d.n ? static_cast<double>(d.sum) / static_cast<double>(d.n) : 0.0;
    }
    // Upper edge of the bucket holding the q-quantile (0 < q <= 1); 0 when empty.
    static std::uint64_t quantile_upper(const Distribution& d, double q) noexcept {
        if (d.n == 0) return 0;
        const auto rank = static_cast<std::uint64_t>(std::ceil(q * static_cast<double>(d.n)));
        std::uint64_t seen = 0;
        for (std::size_t i = 0; i < kBuckets; ++i) {
            seen += d.histogram[i];
            if (seen >= rank) return upper_edge(i);
        }
        return upper_edge(kBuckets - 1);
    }
    // Bucket i covers (upper_edge(i - 1), upper_edge(i)]; bucket 0 is [0, 1].
    static std::uint64_t upper_edge(std::size_t i) noexcept {
        if (i == 0) return 1;
        const std::size_t octave = (i - 1) / 4, step = (i - 1) % 4 + 1;
        // 2^octave * (1 + step/4), rounded up to an integer pixel count.
        const double edge = std::ldexp(1.0 + static_cast<double>(step) / 4.0, static_cast<int>(octave));
        return static_cast<std::uint64_t>(std::ceil(edge));
    }

private:
    static std::size_t bucket(std::uint64_t pixels) noexcept {
        if (pixels <= 1) return 0;
        for (std::size_t i = 1; i < kBuckets; ++i)
            if (pixels <= upper_edge(i)) return i;
        return kBuckets - 1;
    }
    static void add(Distribution& d, std::uint64_t pixels) noexcept {
        ++d.n;
        d.sum += pixels;
        ++d.histogram[bucket(pixels)];
    }
    std::array<BranchRecord, kBranches> branches_{};
    std::uint64_t frames_ = 0;
};

}  // namespace perception
