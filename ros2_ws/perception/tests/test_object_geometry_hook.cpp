/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_object_geometry_hook.cpp
 * Brief: The service's own object geometry hook: order of its checks around a coasting track.
 *
 * Description:
 * What it checks. measure_object_geometry() is the function the service wires into the Supervisor
 * (service/object_geometry_hook.hpp); this test calls that function, not a stand-in. A track that
 * was not observed in the frame (coasting) is never published, so its geometry is not built. That
 * skip must not hide anything the frame has to report:
 *   - builder not set up, or frame not on the registered raster: the frame's capability miss is
 *     raised and counted even when every track of the frame is coasting;
 *   - a mask the builder cannot read is still counted as such;
 *   - only after those checks is a coasting track skipped: no build, base_geometry_valid false,
 *     the skipped pixels counted;
 *   - an observed track gets exactly what a direct ObjectGeometryBuilder::build() of the same
 *     region, mask, band and pixel list returns (bit for bit, on a relief with pixels outside the
 *     band and a mask with holes), in both point paths (exact and float).
 * No camera, model, SDK or network.
 */

#include "service/object_geometry_hook.hpp"

#include <opencv2/core.hpp>

#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace perception;
using perception::service::ObjectGeometryShared;
using perception::service::measure_object_geometry;

std::size_t checks = 0;
void check(bool ok, int line) {
    ++checks;
    if (!ok) throw std::runtime_error("object geometry hook line " + std::to_string(line));
}
#define CHECK(x) check(static_cast<bool>(x), __LINE__)

constexpr int kWidth = 64, kHeight = 40;
const yolo::DepthIntrinsics kColour{50.0F, 50.0F, 31.5F, 19.5F, kWidth, kHeight};

std::shared_ptr<const RgbDepthRegistration> registration() {
    return std::make_shared<const RgbDepthRegistration>(kWidth, kHeight, 50.0, 50.0, 31.5, 19.5,
                                                        StandardDistortion{-0.03, 0.03, -0.01, 0, 0});
}

// Camera 0.8 m high, level, looking along base +x (optical z forward, x right, y down).
rns::RigidTransform base_from_colour() {
    rns::RigidTransform t;
    t.R = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    t.t = {0, 0, 0.8};
    return t;
}
std::unique_ptr<rns::ObjectGeometryBuilder> make_builder(const std::shared_ptr<const RgbDepthRegistration>& reg,
                                                         double float_margin_m) {
    rns::OpticalRayTable rays;
    rays.width = static_cast<std::uint32_t>(reg->width());
    rays.height = static_cast<std::uint32_t>(reg->height());
    rays.x = reg->ray_x();
    rays.y = reg->ray_y();
    rns::ObjectGeometryOptions options{5, 0.01, 0.02};
    options.float_margin_m = float_margin_m;
    return std::make_unique<rns::ObjectGeometryBuilder>(std::move(rays), base_from_colour(), options);
}
// The capability as the service sets it up after the capture reported its session calibration.
void make_ready(ObjectGeometryShared& shared, const std::shared_ptr<const RgbDepthRegistration>& reg,
                double float_margin_m) {
    const rns::RigidTransform transform = base_from_colour();
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.builder = make_builder(reg, float_margin_m);
    shared.colour = kColour;
    shared.registration = reg;
    for (std::size_t i = 0; i < 9; ++i) shared.base_from_colour_R[i] = static_cast<float>(transform.R[i]);
    shared.ready.store(true);
}

// One frame on the registered raster: a flat surface 2 m away.
yolo::DetectionResult frame(std::uint64_t frame_id, const std::shared_ptr<const RgbDepthRegistration>& reg) {
    yolo::DetectionResult result;
    result.frame_id = frame_id;
    result.depth = cv::Mat(kHeight, kWidth, CV_16UC1, cv::Scalar(2000));
    result.depth_scale = 0.001F;
    result.depth_intrinsics = kColour;
    result.depth_registration = reg;
    return result;
}

const cv::Rect kBox(20, 10, 16, 12);

// A detection that carries geometry from somewhere else, so "left untouched" and "reset" differ.
FusedDetection stale_detection(int last_seen_frame) {
    FusedDetection detection;
    detection.track_id = 3;
    detection.last_seen_frame = last_seen_frame;
    detection.base_geometry_valid = true;
    detection.base_footprint_xy = {{9.0F, 9.0F}, {8.0F, 8.0F}, {7.0F, 7.0F}};
    detection.base_z_min = 7.0F;
    detection.base_z_max = 9.0F;
    return detection;
}

struct Counts {
    std::uint64_t located, not_ready, raster_mismatch, mask_unusable, coast_skipped, coast_skipped_points,
        build_calls;
};
Counts counts(const ObjectGeometryShared& s) {
    return {s.located.load(), s.not_ready.load(), s.raster_mismatch.load(), s.mask_unusable.load(),
            s.coast_skipped.load(), s.coast_skipped_points.load(), s.build_calls.load()};
}

void call(ObjectGeometryShared& shared, const yolo::DetectionResult& result, const cv::Mat& mask,
          const std::vector<std::uint32_t>* selected, FusedDetection& detection) {
    measure_object_geometry(shared, result, kBox, mask, 1.9F, 2.1F, selected, detection);
}

// A frame whose tracks are all coasting still reports that the capability is missing.
void test_capability_miss_before_coast() {
    const auto reg = registration();
    const cv::Mat mask(kBox.height, kBox.width, CV_8UC1, cv::Scalar(1));
    // Builder not set up: every track of the frame is coasting, the miss is still raised.
    {
        ObjectGeometryShared shared;
        const auto result = frame(10, reg);
        for (int track = 0; track < 3; ++track) {
            auto detection = stale_detection(7);
            call(shared, result, mask, nullptr, detection);
            CHECK(!detection.base_geometry_valid);
        }
        const auto c = counts(shared);
        CHECK(c.not_ready == 3 && c.coast_skipped == 0 && c.build_calls == 0);
        // What the Objects owner reads once per frame (objects_ready).
        CHECK(shared.frame_capability_miss.exchange(false));
        CHECK(!shared.frame_capability_miss.load());
    }
    // Builder set up, frame not on the registered raster (another registration object, another
    // size, other intrinsics, no registration): miss raised, coasting or not.
    {
        ObjectGeometryShared shared;
        make_ready(shared, reg, 0.0);
        auto other_registration = frame(10, registration());
        auto no_registration = frame(10, nullptr);
        auto other_intrinsics = frame(10, reg);
        other_intrinsics.depth_intrinsics.fx = 51.0F;
        auto other_size = frame(10, reg);
        other_size.depth = cv::Mat(kHeight, kWidth + 2, CV_16UC1, cv::Scalar(2000));
        auto other_type = frame(10, reg);
        other_type.depth = cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(2.0));
        auto other_rows = frame(10, reg);
        other_rows.depth = cv::Mat(kHeight + 2, kWidth, CV_16UC1, cv::Scalar(2000));
        auto other_fy = frame(10, reg), other_cx = frame(10, reg), other_cy = frame(10, reg);
        other_fy.depth_intrinsics.fy = 51.0F;
        other_cx.depth_intrinsics.cx = 32.5F;
        other_cy.depth_intrinsics.cy = 20.5F;
        // The right size and type, but a view into a wider matrix: its rows are not adjacent.
        cv::Mat wide_depth(kHeight, kWidth + 8, CV_16UC1, cv::Scalar(2000));
        auto not_continuous = frame(10, reg);
        not_continuous.depth = wide_depth(cv::Rect(4, 0, kWidth, kHeight));
        CHECK(!not_continuous.depth.isContinuous() && not_continuous.depth.size() == cv::Size(kWidth, kHeight));
        std::uint64_t expected = 0;
        for (const auto* bad : {&other_registration, &no_registration, &other_intrinsics, &other_size, &other_type,
                                &other_rows, &other_fy, &other_cx, &other_cy, &not_continuous}) {
            for (const int last_seen : {7, 10}) {
                auto detection = stale_detection(last_seen);
                call(shared, *bad, mask, nullptr, detection);
                CHECK(!detection.base_geometry_valid);
                CHECK(shared.frame_capability_miss.exchange(false));
                CHECK(counts(shared).raster_mismatch == ++expected);
            }
        }
        const auto c = counts(shared);
        CHECK(c.coast_skipped == 0 && c.build_calls == 0 && c.not_ready == 0);
    }
}

// An unreadable mask is counted as such for a coasting track too; no miss (the frame is fine).
void test_mask_check_before_coast() {
    const auto reg = registration();
    ObjectGeometryShared shared;
    make_ready(shared, reg, 0.0);
    const auto result = frame(10, reg);
    const cv::Mat wrong_type(kBox.height, kBox.width, CV_16UC1, cv::Scalar(1));
    // A view of every second column of a wider mask: bbox-sized, right type, not continuous.
    cv::Mat wide(kBox.height, kBox.width + 4, CV_8UC1, cv::Scalar(1));
    const cv::Mat not_continuous = wide(cv::Rect(2, 0, kBox.width, kBox.height));
    CHECK(!not_continuous.isContinuous() && not_continuous.size() == kBox.size());
    std::uint64_t expected = 0;
    for (const auto* mask : {&wrong_type, &not_continuous}) {
        for (const int last_seen : {7, 10}) {
            auto detection = stale_detection(last_seen);
            call(shared, result, *mask, nullptr, detection);
            CHECK(!detection.base_geometry_valid);
            CHECK(counts(shared).mask_unusable == ++expected);
        }
    }
    const auto c = counts(shared);
    CHECK(c.coast_skipped == 0 && c.build_calls == 0);
    CHECK(!shared.frame_capability_miss.load());
}

// With the capability there: a coasting track is skipped, an observed one is built.
void test_coast_skipped_observed_built(double float_margin_m) {
    const auto reg = registration();
    ObjectGeometryShared shared;
    make_ready(shared, reg, float_margin_m);
    const auto result = frame(10, reg);
    const cv::Mat mask(kBox.height, kBox.width, CV_8UC1, cv::Scalar(1));
    // The sampler's list for this region: every pixel of the box, row-major, increasing.
    std::vector<std::uint32_t> selected;
    for (int y = kBox.y; y < kBox.y + kBox.height; ++y)
        for (int x = kBox.x; x < kBox.x + kBox.width; ++x)
            selected.push_back(static_cast<std::uint32_t>(y * kWidth + x));

    // Coasting (last seen in an earlier frame): nothing built, stale geometry reset, pixels counted.
    auto coasting = stale_detection(9);
    call(shared, result, mask, &selected, coasting);
    CHECK(!coasting.base_geometry_valid);
    auto c = counts(shared);
    CHECK(c.coast_skipped == 1 && c.coast_skipped_points == selected.size());
    CHECK(c.build_calls == 0 && c.located == 0);
    // Without a list only the track is counted.
    auto coasting_no_list = stale_detection(0);
    call(shared, result, mask, nullptr, coasting_no_list);
    c = counts(shared);
    CHECK(c.coast_skipped == 2 && c.coast_skipped_points == selected.size() && c.build_calls == 0);
    CHECK(!shared.frame_capability_miss.load());

    // Observed in this frame: built from the list, located, rotation handed over.
    auto observed = stale_detection(10);
    call(shared, result, mask, &selected, observed);
    c = counts(shared);
    CHECK(c.build_calls == 1 && c.located == 1 && c.coast_skipped == 2);
    CHECK(shared.selected_list_used.load() == 1 && shared.selected_list_rejected.load() == 0);
    CHECK(shared.float_points.load() == (float_margin_m > 0 ? 1U : 0U));
    CHECK(observed.base_geometry_valid && observed.base_footprint_xy.size() >= 3);
    CHECK(observed.base_from_colour_R == shared.base_from_colour_R);
    // A flat surface 2 m ahead: base x near 2 m; the box rows are above and below the optical
    // axis of a camera 0.8 m high, so the height interval lies around 0.8 m.
    for (const auto& vertex : observed.base_footprint_xy) CHECK(vertex[0] > 1.95F && vertex[0] < 2.05F);
    CHECK(observed.base_z_min < 0.8F && observed.base_z_max > 0.8F && observed.base_z_max < 1.3F);
    // The same track without the list: the region is scanned, same geometry.
    auto scanned = stale_detection(10);
    call(shared, result, mask, nullptr, scanned);
    CHECK(scanned.base_geometry_valid && scanned.base_footprint_xy == observed.base_footprint_xy);
    CHECK(scanned.base_z_min == observed.base_z_min && scanned.base_z_max == observed.base_z_max);
    CHECK(counts(shared).build_calls == 2 && shared.selected_list_used.load() == 1);

    // A frame id a track cannot hold (above INT_MAX) is "not observed": skipped, not built.
    auto huge = frame(static_cast<std::uint64_t>(std::numeric_limits<int>::max()) + 1U, reg);
    auto wrapped = stale_detection(std::numeric_limits<int>::min());
    call(shared, huge, mask, &selected, wrapped);
    CHECK(!wrapped.base_geometry_valid && counts(shared).coast_skipped == 3 && counts(shared).build_calls == 2);
}

// What an observed track receives is exactly what the builder returns for the same inputs: the
// hook passes the region, the mask, the band and the list on unchanged. A relief with a step (so
// the band leaves pixels out), a mask with holes, a box that hangs over the image edge, a mask of
// another size than the box (then the box centre is used, as DepthSampler does) and a band that
// selects too few points.
void test_observed_equals_direct_build(double float_margin_m) {
    const auto reg = registration();
    ObjectGeometryShared shared;
    make_ready(shared, reg, float_margin_m);
    const auto direct = make_builder(reg, float_margin_m);
    auto result = frame(10, reg);
    // 2.0 m with a few millimetres of relief on the left, 2.6 m on the right, some invalid pixels.
    for (int y = 0; y < kHeight; ++y) {
        auto* row = result.depth.ptr<std::uint16_t>(y);
        for (int x = 0; x < kWidth; ++x) {
            row[x] = static_cast<std::uint16_t>((x < 30 ? 2000 : 2600) + (x * 7 + y * 13) % 9);
            if ((x + 3 * y) % 11 == 0) row[x] = 0;
        }
    }
    result.depth_scale = 0.00098F;
    struct Case { cv::Rect box; int mask_kind; float lo, hi; bool with_list; };
    const Case cases[] = {
        {cv::Rect(18, 8, 24, 20), 1, 1.90F, 2.10F, true},   // band keeps the near side only
        {cv::Rect(18, 8, 24, 20), 1, 1.90F, 2.10F, false},  // the same without a list: region scan
        {cv::Rect(18, 8, 24, 20), 1, 2.40F, 2.70F, true},   // the far side
        {cv::Rect(18, 8, 24, 20), 1, 1.00F, 3.00F, true},   // both sides
        {cv::Rect(18, 8, 24, 20), 0, 1.90F, 2.70F, false},  // no mask: centre of the box
        {cv::Rect(18, 8, 24, 20), 2, 1.90F, 2.70F, false},  // mask of another size: centre of the box
        {cv::Rect(50, 30, 30, 20), 1, 1.00F, 3.00F, true},  // hangs over the right and bottom edges
        {cv::Rect(18, 8, 24, 20), 1, 3.50F, 4.00F, false},  // nothing in the band
    };
    std::size_t located = 0, not_located = 0;
    for (const Case& c : cases) {
        cv::Mat mask;
        if (c.mask_kind == 1) {
            mask = cv::Mat(c.box.height, c.box.width, CV_8UC1, cv::Scalar(255));
            for (int y = 0; y < mask.rows; ++y)
                for (int x = 0; x < mask.cols; ++x)
                    if ((x + 2 * y) % 5 == 0) mask.at<std::uint8_t>(y, x) = 0;
        } else if (c.mask_kind == 2) {
            mask = cv::Mat(c.box.height + 1, c.box.width, CV_8UC1, cv::Scalar(255));
        }
        const bool sampler_uses_mask = c.mask_kind == 1;
        // The selected pixels as DepthSampler lists them: region pixels (through the mask) whose
        // depth lies in the band, row-major.
        std::vector<std::uint32_t> selected;
        const cv::Rect roi = c.box & cv::Rect(0, 0, kWidth, kHeight);
        for (int y = roi.y; y < roi.y + roi.height; ++y)
            for (int x = roi.x; x < roi.x + roi.width; ++x) {
                if (sampler_uses_mask && mask.at<std::uint8_t>(y - c.box.y, x - c.box.x) == 0) continue;
                const float metres = static_cast<float>(result.depth.at<std::uint16_t>(y, x)) * result.depth_scale;
                if (result.depth.at<std::uint16_t>(y, x) != 0 && metres >= c.lo && metres <= c.hi)
                    selected.push_back(static_cast<std::uint32_t>(y * kWidth + x));
            }
        const bool give_list = c.with_list && sampler_uses_mask;
        // Direct build with the same inputs.
        rns::DepthRegion region;
        region.depth = result.depth.ptr<std::uint16_t>();
        region.width = kWidth;
        region.height = kHeight;
        region.depth_scale = result.depth_scale;
        region.bbox_x = c.box.x;
        region.bbox_y = c.box.y;
        region.bbox_width = c.box.width;
        region.bbox_height = c.box.height;
        region.mask = sampler_uses_mask ? mask.ptr<std::uint8_t>() : nullptr;
        if (give_list && !selected.empty()) {
            region.selected = selected.data();
            region.selected_count = selected.size();
        }
        const auto want = direct->build(region, c.lo, c.hi);
        // The hook.
        auto detection = stale_detection(10);
        const auto calls_before = shared.build_calls.load(), points_before = shared.build_points.load();
        measure_object_geometry(shared, result, c.box, mask, c.lo, c.hi, give_list ? &selected : nullptr, detection);
        CHECK(shared.build_calls.load() == calls_before + 1);
        CHECK(shared.build_points.load() == points_before + want.points);
        CHECK(detection.base_geometry_valid == (want.status == rns::ObjectGeometryStatus::Located));
        if (want.status != rns::ObjectGeometryStatus::Located) {
            ++not_located;
            continue;
        }
        ++located;
        CHECK(give_list ? want.used_selected : !want.used_selected);
        CHECK(detection.base_footprint_xy.size() == want.geometry->footprint_xy.size());
        for (std::size_t i = 0; i < detection.base_footprint_xy.size(); ++i) {
            CHECK(std::memcmp(&detection.base_footprint_xy[i][0], &want.geometry->footprint_xy[i].x, sizeof(float)) == 0);
            CHECK(std::memcmp(&detection.base_footprint_xy[i][1], &want.geometry->footprint_xy[i].y, sizeof(float)) == 0);
        }
        CHECK(std::memcmp(&detection.base_z_min, &want.geometry->z_min, sizeof(float)) == 0);
        CHECK(std::memcmp(&detection.base_z_max, &want.geometry->z_max, sizeof(float)) == 0);
        CHECK(detection.base_from_colour_R == shared.base_from_colour_R);
    }
    // The cases reach both outcomes, and the near and the far side really give different geometry.
    CHECK(located >= 6 && not_located >= 1);
    CHECK(shared.too_few_points.load() + shared.invalid_input.load() + shared.non_finite.load() == not_located);
    CHECK(shared.selected_list_rejected.load() == 0 && shared.coast_skipped.load() == 0);
}
}  // namespace

int main() {
    try {
        test_capability_miss_before_coast();
        test_mask_check_before_coast();
        test_coast_skipped_observed_built(0.0);
        test_coast_skipped_observed_built(0.001);
        test_observed_equals_direct_build(0.0);
        test_observed_equals_direct_build(0.001);
        std::cout << "{\"status\":\"PASS\",\"checks\":" << checks
                  << ",\"real_hook\":true,\"device\":false,\"model_execution\":false,\"network\":false}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
