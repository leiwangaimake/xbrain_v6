/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: scene_luma_window.hpp
 * Brief: Scene brightness per window, numbers only, to tell a lighting change from a load change.
 *
 * Description:
 * Why (2026-10-16). On 2026-10-15 the Objects "bad state" switched together with the detection
 * mix, and the scene fingerprint (free bins, object count) could not say what changed in the room.
 * GPT question 18.4 suggested a synchronised frame fingerprint; the user allows numbers, no image.
 * Once a second the inference thread samples a fixed grid of the colour frame (every step-th pixel
 * in each direction, starting half a step in; 80 x 50 points for 1280 x 800 and step 16) and takes
 * the mean luma Y = (29 B + 150 G + 77 R) >> 8 (BT.601 weights in 1/256). A window of
 * kWindowSamples such samples, at least kSampleIntervalNs apart (about a minute in steady running,
 * longer when frames stop), gives one line: count, unusable samples, mean, min and max of the
 * per-sample means, the monotonic time and frame id of the first and the last sample, and what the
 * sampling itself cost (total and longest), so its overhead can be read on the target.
 * What it can and cannot say (GPT question 18 supplement): it is a relative reading of the camera's
 * output, not a photometric one (auto exposure moves it as well); a fixed grid misses changes off
 * the grid, and mean / min / max lose the order within a window. The same reading does not prove the
 * room unchanged, a change does not point only at the lights. The workload features (instances, box
 * and mask pixels per model) stay the scene fingerprint; this is a side reading.
 * Pure arithmetic on a raw pointer: no OpenCV, no allocation, one writer at a time.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace perception {

// Mean luma over the grid of a packed 8-bit BGR image (row_bytes between row starts); -1 when the
// image cannot be read that way (null data, empty, not 3 channels, step < 1, short rows).
inline double grid_luma_mean(const std::uint8_t* data, int rows, int cols, std::size_t row_bytes, int channels,
                             int step) noexcept {
    if (data == nullptr || rows <= 0 || cols <= 0 || channels != 3 || step < 1 ||
        row_bytes < static_cast<std::size_t>(cols) * 3U)
        return -1.0;
    std::uint64_t sum = 0, n = 0;
    for (int y = step / 2; y < rows; y += step) {
        const std::uint8_t* row = data + static_cast<std::size_t>(y) * row_bytes;
        for (int x = step / 2; x < cols; x += step) {
            const std::uint8_t* p = row + static_cast<std::size_t>(x) * 3U;
            sum += (29U * p[0] + 150U * p[1] + 77U * p[2]) >> 8;
            ++n;
        }
    }
    return n ? static_cast<double>(sum) / static_cast<double>(n) : -1.0;
}

class SceneLumaWindow {
public:
    static constexpr std::int64_t kSampleIntervalNs = 1000000000;  // one sample a second
    static constexpr std::uint32_t kWindowSamples = 60;           // one line per 60 samples
    static constexpr int kGridStep = 16;

    // Whether a sample is due at now_ns: the first call, then once kSampleIntervalNs has passed.
    bool due(std::int64_t now_ns) const noexcept { return !sampled_ || now_ns - last_ns_ >= kSampleIntervalNs; }
    // Records one sample taken at now_ns from frame frame_id, which cost cost_ns to take; a negative
    // mean counts as unusable. True when the window is full (print it, then reset()).
    bool add(std::int64_t now_ns, double mean, std::uint64_t frame_id = 0, std::int64_t cost_ns = 0) noexcept {
        sampled_ = true;
        last_ns_ = now_ns;
        if (count_ == 0) {
            first_sample_ns_ = now_ns;
            first_frame_ = frame_id;
        }
        last_frame_ = frame_id;
        if (cost_ns > 0) {
            cost_total_ns_ += cost_ns;
            if (cost_ns > cost_max_ns_) cost_max_ns_ = cost_ns;
        }
        ++count_;
        if (mean < 0.0) {
            ++unusable_;
        } else {
            if (usable() == 1 || mean < min_) min_ = mean;
            if (usable() == 1 || mean > max_) max_ = mean;
            sum_ += mean;
        }
        return count_ >= kWindowSamples;
    }
    std::uint32_t count() const noexcept { return count_; }
    std::uint32_t unusable() const noexcept { return unusable_; }
    std::uint32_t usable() const noexcept { return count_ - unusable_; }
    double mean() const noexcept { return usable() ? sum_ / static_cast<double>(usable()) : -1.0; }
    double min() const noexcept { return usable() ? min_ : -1.0; }
    double max() const noexcept { return usable() ? max_ : -1.0; }
    // First and last sample of the window (0 when empty), and the sampling cost in it.
    std::int64_t first_ns() const noexcept { return count_ ? first_sample_ns_ : 0; }
    std::int64_t last_ns() const noexcept { return count_ ? last_ns_ : 0; }
    std::uint64_t first_frame() const noexcept { return count_ ? first_frame_ : 0; }
    std::uint64_t last_frame() const noexcept { return count_ ? last_frame_ : 0; }
    std::int64_t cost_total_ns() const noexcept { return cost_total_ns_; }
    std::int64_t cost_max_ns() const noexcept { return cost_max_ns_; }
    // A new window; the sampling clock goes on (the next sample is still a second after the last).
    void reset() noexcept {
        count_ = unusable_ = 0;
        sum_ = min_ = max_ = 0.0;
        first_sample_ns_ = 0;
        first_frame_ = last_frame_ = 0;
        cost_total_ns_ = cost_max_ns_ = 0;
    }

private:
    bool sampled_ = false;
    std::int64_t last_ns_ = 0, first_sample_ns_ = 0, cost_total_ns_ = 0, cost_max_ns_ = 0;
    std::uint64_t first_frame_ = 0, last_frame_ = 0;
    std::uint32_t count_ = 0, unusable_ = 0;
    double sum_ = 0.0, min_ = 0.0, max_ = 0.0;
};

}  // namespace perception
