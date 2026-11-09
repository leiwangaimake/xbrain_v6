/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_post_work_counts.cpp
 * Brief: yolo::last_post_work_counts(): candidates, NMS survivors and COCO80 mask paths per call.
 *
 * Description:
 * Synthetic raw outputs of both models. The counters must equal what the decode did: the anchors
 * above the gate (before NMS), the kept ones (after NMS) and, for COCO80, one mask path per kept
 * instance (roi2 + roi + full = detections; the full-frame path when the ROI resize is switched
 * off); they are per thread, replaced by the next call on that thread, zero after a call that
 * throws, and reading them changes no output. Diagnostic counters only (Fable question 13.2).
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include "common/postprocess.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

int checks = 0;

void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

// Raw outputs with `count` boxes on a width x height image; box i has class cls(i) and score
// .95 - .05 i. `twins` extra anchors repeat box 0 (same class, IoU 1) so NMS has work to do.
struct RawSeg {
    std::vector<float> output, proto;
    yolo::LetterboxInfo info;
    std::int64_t anchors;
    int input, classes, proto_size;
    RawSeg(int width, int height, int count, int twins, bool active)
        : anchors(active ? 16464 : 8400), input(active ? 896 : 640), classes(active ? 12 : 80),
          proto_size(input / 4) {
        output.assign(static_cast<std::size_t>((4 + classes + 32) * anchors), 0.0F);
        proto.assign(static_cast<std::size_t>(32 * proto_size * proto_size), 0.0F);
        info.original_width = width;
        info.original_height = height;
        info.scale = std::min(static_cast<float>(input) / width, static_cast<float>(input) / height);
        info.new_width = static_cast<int>(std::round(width * info.scale));
        info.new_height = static_cast<int>(std::round(height * info.scale));
        info.pad_left = (input - info.new_width) / 2;
        info.pad_top = (input - info.new_height) / 2;
        for (int i = 0; i < proto_size * proto_size; ++i) proto[static_cast<std::size_t>(i)] = i % 7 == 0 ? -2.0F : 2.0F;
        for (int i = 0; i < count + twins; ++i) {
            const int k = i < count ? i : 0;  // a twin repeats box 0
            const float w = width * .30F * info.scale, h = height * .35F * info.scale;
            set(i, info.pad_left + width * (.05F + .22F * k) * info.scale + w / 2,
                info.pad_top + height * (.07F + .15F * k) * info.scale + h / 2, w, h,
                active ? (k == 0 ? 10 : k == 1 ? 9 : k) : (k == 1 ? 2 : k), .95F - k * .05F - (i >= count ? .01F : 0.0F));
        }
    }
    void set(int i, float cx, float cy, float w, float h, int cls, float score) {
        output[static_cast<std::size_t>(i)] = cx;
        output[static_cast<std::size_t>(anchors + i)] = cy;
        output[static_cast<std::size_t>(2 * anchors + i)] = w;
        output[static_cast<std::size_t>(3 * anchors + i)] = h;
        output[static_cast<std::size_t>((4 + cls) * anchors + i)] = score;
        output[static_cast<std::size_t>((4 + classes) * anchors + i)] = 1.0F;
    }
};

std::vector<yolo::Detection> coco(const RawSeg& r) {
    return yolo::postprocess_seg(r.output.data(), {1, 116, r.anchors}, r.proto.data(),
                                 {1, 32, r.proto_size, r.proto_size}, r.info, .25F, {}, 0.0F, 0, .7F, .5F, r.input);
}

std::vector<yolo::Detection> active(const RawSeg& r) {
    return yolo::postprocess_seg(r.output.data(), {1, 48, r.anchors}, r.proto.data(),
                                 {1, 32, r.proto_size, r.proto_size}, r.info, yolo::active12_seg_model_spec(), .25F,
                                 .7F, .5F, r.input);
}

bool same_detections(const std::vector<yolo::Detection>& a, const std::vector<yolo::Detection>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].bbox != b[i].bbox || a[i].class_id != b[i].class_id || a[i].confidence != b[i].confidence ||
            a[i].mask.size() != b[i].mask.size() || cv::norm(a[i].mask, b[i].mask, cv::NORM_INF) != 0)
            return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        // argv[1]: "roi2" (the production setting: ROI resize, ROI2 tried first), "roi" (ROI resize
        // without ROI2) or "full" (ROI resize switched off). Both switches are read once per process.
        const std::string mode = argc > 1 ? argv[1] : "roi2";
        if (mode == "full") setenv("PERCEPTION_COCO_MASK_RESIZE", "full", 1);
        else if (mode == "roi2") setenv("PERCEPTION_COCO_MASK_ROI2", "1", 1);
        else if (mode != "roi") throw std::invalid_argument("mode must be roi2, roi or full");
        // COCO80: four boxes and two twins of box 0 -> six candidates, four kept.
        const RawSeg c(1280, 800, 4, 2, false);
        const auto first = coco(c);
        const auto w = yolo::last_post_work_counts();
        check(w.candidates == 6 && w.selected == 4 && first.size() == 4,
              "coco candidates / selected: " + std::to_string(w.candidates) + " / " + std::to_string(w.selected));
        check(w.mask_roi2 + w.mask_roi + w.mask_full == first.size(), "one mask path per detection");
        // 1280 x 800 in a 640 input is an integer scale (2): every box here takes the ROI resize in
        // "roi", and ROI2 takes them all in "roi2".
        if (mode == "full") check(w.mask_full == first.size() && w.mask_roi2 == 0 && w.mask_roi == 0, "full mode: full path");
        if (mode == "roi") check(w.mask_roi == first.size() && w.mask_roi2 == 0 && w.mask_full == 0, "roi mode: roi path");
        if (mode == "roi2") check(w.mask_roi2 == first.size() && w.mask_roi == 0 && w.mask_full == 0, "roi2 mode: roi2 path");
        std::cout << "coco mask paths roi2/roi/full = " << w.mask_roi2 << "/" << w.mask_roi << "/" << w.mask_full
                  << " (mode " << mode << ")\n";
        // Reading the counters changes nothing; a second call gives the same counts.
        const auto second = coco(c);
        const auto w2 = yolo::last_post_work_counts();
        check(same_detections(first, second), "repeatable output");
        check(w2.candidates == w.candidates && w2.selected == w.selected && w2.mask_roi2 == w.mask_roi2 &&
              w2.mask_roi == w.mask_roi && w2.mask_full == w.mask_full, "repeatable counts");
        // Per thread: another thread's call does not touch this thread's counters, and a fresh thread
        // starts at zero.
        yolo::PostWorkCounts other{}, fresh{};
        std::thread t([&] {
            fresh = yolo::last_post_work_counts();
            const RawSeg small(1280, 800, 1, 0, false);
            (void)coco(small);
            other = yolo::last_post_work_counts();
        });
        t.join();
        check(fresh.candidates == 0 && fresh.selected == 0, "a fresh thread starts at zero");
        check(other.candidates == 1 && other.selected == 1, "the other thread's own call");
        check(yolo::last_post_work_counts().candidates == 6, "this thread unchanged");
        // Nothing above the gate: zero everywhere.
        RawSeg empty(1280, 800, 0, 0, false);
        check(coco(empty).empty() && yolo::last_post_work_counts().candidates == 0 &&
              yolo::last_post_work_counts().selected == 0, "no candidate");
        // A call that throws leaves the counters at zero.
        (void)coco(c);
        bool threw = false;
        try {
            (void)yolo::postprocess_seg(nullptr, {1, 116, c.anchors}, c.proto.data(), {1, 32, c.proto_size, c.proto_size},
                                        c.info, .25F, {}, 0.0F, 0, .7F, .5F, c.input);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        const auto after_throw = yolo::last_post_work_counts();
        check(threw && after_throw.candidates == 0 && after_throw.selected == 0 && after_throw.mask_full == 0 &&
              after_throw.mask_roi == 0 && after_throw.mask_roi2 == 0, "zero after a throw");
        // Active12: candidates and selected, no mask paths.
        const RawSeg a(1280, 800, 3, 1, true);
        const auto ad = active(a);
        const auto aw = yolo::last_post_work_counts();
        check(aw.candidates == 4 && aw.selected == ad.size() && aw.selected == 3,
              "active candidates / selected: " + std::to_string(aw.candidates) + " / " + std::to_string(aw.selected));
        check(aw.mask_roi2 == 0 && aw.mask_roi == 0 && aw.mask_full == 0, "active: no mask path counts");
        std::cout << "test_post_work_counts: PASS checks=" << checks << " mode=" << mode << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_post_work_counts: FAIL " << e.what() << "\n";
        return 1;
    }
}
