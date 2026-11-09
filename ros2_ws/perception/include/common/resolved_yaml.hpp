/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: resolved_yaml.hpp
 * Brief: Bounded local adapter in front of the pinned upstream yaml_lite reader.
 *
 * Description:
 * The platform's yaml_lite (common/, used as is) reads the resolved perception config but has no
 * sequence node. These functions rewrite only our own numeric vectors into a form it reads and
 * refuse every construct it would silently misread; see the comments below.
 */
#pragma once

#include <cstddef>
#include <map>
#include <string>

namespace perception::config {

// yaml_lite intentionally has no sequence node. Adapt ONLY the three numeric
// vector leaves this process owns (perception.extrinsic_base_cam.translation,
// perception.extrinsic_base_cam.rotation_xyzw,
// perception.native_geometry_setup.prior_normal) from block style into the
// flow style yaml_lite keeps as one scalar; everything else is left to the
// upstream reader. Rejects duplicate paths, tabs, aliases/tags, unresolved
// ${...} references and any other sequence before yaml_lite's last-write-wins
// map could hide them. Empty leaves become explicit null. Throws
// std::invalid_argument with a dotted key path. Never writes a file.
std::string adapt_resolved_yaml_vectors(const std::string& text);

// Same bounded adapter for another document this process owns (e.g. the
// calibration record): `vectors` maps each dotted numeric-vector path to its
// exact length; `context` names the document in errors. Same rejections.
std::string adapt_yaml_vectors(const std::string& text, const std::map<std::string, std::size_t>& vectors,
                             const std::string& context, std::size_t max_bytes);

}  // namespace perception::config
