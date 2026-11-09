/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_object_geometry_original.hpp
 * Brief: Declaration of the frozen original ObjectGeometryBuilder, the oracle for its tests.
 *
 * Description:
 * Used only by test_rns_object_geometry_scratch (namespace rns_object_geometry_original). The class is the
 * production declaration at commit 02d2665 (2026-10-10), before the footprint work lists became
 * builder scratch; the input and result types are the production ones, so both builders take the
 * same region and their results compare field by field.
 */

#pragma once

#include "output/rns_object_geometry.hpp"

#include <vector>

namespace rns_object_geometry_original {
using ::perception::rns::DepthRegion;
using ::perception::rns::ObjectGeometryOptions;
using ::perception::rns::ObjectGeometryResult;
using ::perception::rns::ObjectGeometryStatus;
using ::perception::rns::OpticalRayTable;
using ::perception::rns::Point;
using ::perception::rns::RigidTransform;
using ::perception::rns::valid_footprint;
using ::perception::rns::valid_rigid_transform;

class ObjectGeometryBuilder {
public:
    // Throws std::invalid_argument on an invalid table/transform/options.
    // Allocates bounded scratch once (grows only for a larger region).
    ObjectGeometryBuilder(OpticalRayTable rays, RigidTransform base_from_colour, ObjectGeometryOptions options);
    // selected_min_m/max_m: DepthSampler's selected depth band for this object.
    ObjectGeometryResult build(const DepthRegion& region, float selected_min_m, float selected_max_m);

private:
    OpticalRayTable rays_;
    RigidTransform base_from_colour_;
    ObjectGeometryOptions options_;
    // Per x-bucket min/max y and the x at which each was seen.
    std::vector<double> bucket_min_y_, bucket_max_y_, bucket_min_y_x_, bucket_max_y_x_;
    std::vector<double> points_x_, points_y_;  // selected base (x, y); grows to the largest region seen
};

}  // namespace rns_object_geometry_original
