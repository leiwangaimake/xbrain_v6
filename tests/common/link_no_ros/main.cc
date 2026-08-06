/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: main.cc
 * Brief: Minimal translation unit that includes every deployed common/ header
 *        and links with no ROS library at all
 *
 * Description:
 * What problem this solves. 19 S1.2 states that common/ must never depend on
 * rclcpp or any ROS type, and gives the reason rather than a preference: the
 * consumers include chassis_relay, which 11 S1.1.6 places on the emergency-stop
 * path under CRL-4 and CRL-5. Prose cannot hold that. This file is the smallest
 * program that can hold it -- it includes the deployed headers, calls into them,
 * and is built by tests/common/link_no_ros/CMakeLists.txt with no ROS include
 * directory on the command line and no ROS library on the link line. If a header
 * ever grows a ROS include, this stops building.
 *
 * Why it CALLS the header instead of only including it. An empty main() links
 * cleanly no matter what the include directory contains, so a translation unit
 * that only includes would be the empty-shell implementation CLAUDE.md S3.2
 * warns about: green whether or not the library works. Calling CommonDigest and
 * FenceCrc32 forces both templates and both inline definitions to be
 * instantiated and emitted, so the compile and the link both have something to
 * do. The values are printed and not checked -- correctness of the digest is
 * CFG-CM-10's job, and duplicating its golden vectors here would create a second
 * place to update.
 *
 * Why the include list is generated rather than written out here. This gate only
 * covers headers this translation unit actually includes. A new header added
 * under common/include and not added to a hand-kept list would sit outside the
 * scan surface while the suite still reported green -- the "扫描面不声明" shape
 * from CLAUDE.md S3.2, and not a hypothetical one: a second header appeared
 * under common/include while this file was being written. So
 * xbrain/common/CMakeLists.txt globs the directory and emits
 * xbrain/common/all_deployed_headers.h with one include per header, and that is
 * what is included below. Adding a header now needs no edit here.
 * test_the_aggregate_covers_every_deployed_header walks the directory in Python
 * and compares, because a glob pattern that matched only .hpp would drop every
 * .h and still look complete.
 *
 * What this file does NOT establish, stated so the pass is not read as more
 * than it is:
 *   * it says nothing about CRL-4's no dynamic allocation rule. The digest
 *     header allocates through std::string by design; whether the relay hop
 *     allocates is a property of chassis_relay's sources.
 *   * a green build here does not prove ROS is absent from the machine. It
 *     proves this target was built without reaching for it. The companion
 *     assertions in the Python driver -- no ROS directory in the compile command
 *     and no ROS shared object in ldd -- are what carry that half.
 *
 * The one argument this program takes. With "glibcxx-assert-probe" it performs a
 * deliberate out-of-range std::vector access. That is undefined behaviour on
 * purpose and is the only way to show that -D_GLIBCXX_ASSERTIONS is LIVE rather
 * than merely present on a command line: under a debug build libstdc++ traps the
 * access and the process aborts. It is never reached without the argument, and
 * the driver only runs it against the debug build.
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

/* Every header under common/include, one include each. Emitted at configure time
 * by xbrain/common/CMakeLists.txt; see the Description for why it is generated.
 * The digest names used below arrive through it, which is deliberate: naming
 * canonical_digest.h separately here would let this file keep building if the
 * aggregate silently went empty. */
#include "xbrain/common/all_deployed_headers.h"

namespace {

/* Printed on the success path. The driver looks for this exact text, so an
 * executable that builds, starts and then does nothing useful is still caught. */
const char kOkToken[] = "link_no_ros ok";

/* Exercise the config-side fingerprint. A two key map rather than an empty one:
 * an empty map serialises to "{}" and would not reach the key ordering or the
 * quoting paths, so a header broken in either place would still link and run. */
std::string ExerciseCommonDigest() {
  using hachist::xbrain::digest::Value;
  hachist::xbrain::digest::ValuePtr root = Value::Map();
  root->Set("plane", Value::Str("rt"));
  root->Set("rate_hz", Value::Int(20));
  return hachist::xbrain::digest::CommonDigest(root);
}

/* Exercise the fence-side fingerprint, for the same reason: one polygon with one
 * vertex touches the number rendering and the field separators. */
std::string ExerciseFenceCrc32() {
  hachist::xbrain::digest::FenceSet fence;
  fence.fence_set_id = "link-no-ros";
  fence.rev = 1u;

  hachist::xbrain::digest::Polygon poly;
  poly.poly_id = "p0";
  poly.role = "keep_in";
  poly.winding = "ccw";
  poly.hard_enforce = true;
  poly.priority = 10;
  poly.vertices.push_back(hachist::xbrain::digest::Vertex{31.0, 121.0});
  fence.polygons.push_back(poly);

  return hachist::xbrain::digest::FenceCrc32(fence);
}

/* Deliberate out-of-range access, reached only through the explicit argument.
 *
 * The index is derived from argc so no compile-time bounds analysis can fold it
 * away and turn the probe into a no-op; that would leave the driver asserting
 * against a program that never misbehaves, which is an always-green assertion.
 * With -D_GLIBCXX_ASSERTIONS libstdc++ checks operator[] and aborts here. */
int RunGlibcxxAssertProbe(int argc) {
  std::vector<int> values;
  values.push_back(7);
  const std::size_t index = static_cast<std::size_t>(argc) + 40u;
  std::printf("probe read %d\n", values[index]);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  /* Compared with std::strcmp rather than by constructing a std::string, so the
   * argument handling itself cannot throw before the probe is reached. */
  if (argc > 1 && std::strcmp(argv[1], "glibcxx-assert-probe") == 0) {
    return RunGlibcxxAssertProbe(argc);
  }

  const std::string digest = ExerciseCommonDigest();
  const std::string crc = ExerciseFenceCrc32();

  /* English output, per CLAUDE.md S2.1. The two fingerprints are printed so a
   * failure that somehow produced an empty string is visible in the driver's
   * captured output rather than merely absent. */
  std::printf("common_digest %s\n", digest.c_str());
  std::printf("fence_crc32 %s\n", crc.c_str());
  std::printf("%s\n", kOkToken);
  return 0;
}
