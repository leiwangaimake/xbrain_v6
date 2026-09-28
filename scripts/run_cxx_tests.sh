#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_cxx_tests.sh
# Brief: Run every built C++ test binary with the cwd, argv and env it needs
#
# Description:
# What this solves. The C++ packages in ros2_ws that are ours -- quadruped,
# chassis_relay and sensor (perception is another team's, out of scope here) --
# build a set of test binaries between them, and five of those do NOT pass when
# they are simply executed. Each failure looks like a real defect and is not:
#
#   * test_rt_session prints "FAIL session_factory.py path not passed as
#     argv[1]" and exits non-zero. It has no default -- the comparison it
#     performs needs the other implementation of the same config, and its
#     CMakeLists comment says in as many words that a missing file must FAIL
#     rather than skip, "a comparison that cannot run is not a comparison".
#   * test_rt_keys falls back to the REPO-relative "docs/11-接口契约.md" when
#     argv is empty, so it only passes from the repository root and reports a
#     missing contract from anywhere else.
#   * test_chs_b / test_uplink / test_uplink_alloc link libddsc and librclcpp
#     out of /opt/ros/humble and die on the loader without that environment.
#
# Before this script the knowledge lived in one person's shell history, and
# the observed cost was reading five environment failures as five broken
# tests. ctest knows all of it (it is in the add_test lines), but ctest only
# runs what THIS build tree configured -- so the same five come back the
# moment anyone runs a binary directly, which is what a bisect, a debugger
# session and scripts/ci/cxx_mutants.py all do.
#
# What it does NOT do:
#   * it does not build. Run scripts/build_quadruped.sh first; a stale binary
#     passing is worse than a missing one, and this script cannot tell them
#     apart, so it reports what is on disk and says so.
#   * it does not replace ctest. ctest remains the in-tree gate that
#     build_quadruped.sh runs; this is the out-of-tree one that also covers
#     chassis_relay and sensor, and that a human can point at a single binary.
#   * it does not judge test CONTENT. Exit status is the whole verdict.
#
# Two properties that make it usable as a gate (CLAUDE.md 8.1):
#   * exit code is non-zero if any binary fails, so it chains with && ;
#   * a binary found on disk that this script has no entry for is a FAILURE,
#     not a silent skip. A runner whose scan surface is undeclared reports a
#     number nobody can interpret (CLAUDE.md 3.2 form 6), and the way that
#     goes wrong here is concrete: somebody adds test_foo.cc, everyone keeps
#     seeing "all passed", and test_foo never ran once.
#     This is not hypothetical: the guard below scanned only the quadruped and
#     chassis_relay build dirs until 2026-09-28, so the nine sensor binaries
#     were never listed AND never scanned -- the summary said "all passed" of a
#     surface that did not include them. The scan surface is the list of build
#     dirs in BUILD_DIRS; adding a row to TESTS without adding the package's
#     build dir there reinstates exactly that hole, which is why the two are
#     spelled once, side by side, rather than in two places.
#
# SKIP is never spelled as pass. When ROS is absent the three ROS-linked
# binaries are reported as SKIP with the reason, and the script still exits
# non-zero unless --allow-skip says that is acceptable on this host. The
# default is the strict one on purpose: a runner that quietly tolerates three
# missing tests is an assertion that is always green (CLAUDE.md 3.2 form 1).
#
# Usage:
#   scripts/run_cxx_tests.sh                 run everything, strict
#   scripts/run_cxx_tests.sh --allow-skip    tolerate a host with no ROS 2
#   scripts/run_cxx_tests.sh --list          print the table and exit
#   scripts/run_cxx_tests.sh test_rt_bridge [test_...]   run only these
set -euo pipefail

# Derive the root from this file, never hard-code it (CLAUDE.md 6): a
# hard-coded /opt/xbrain_v6 runs the wrong checkout on a machine with two.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
QUAD_DIR="$REPO_ROOT/ros2_ws/quadruped"
RELAY_DIR="$REPO_ROOT/ros2_ws/chassis_relay"
SENSOR_DIR="$REPO_ROOT/ros2_ws/sensor"
QUAD_BUILD="$QUAD_DIR/build"
RELAY_BUILD="$RELAY_DIR/build"
SENSOR_BUILD="$SENSOR_DIR/build"
ROS_SETUP="/opt/ros/humble/setup.bash"

# The scan surface of the drift guard, spelled ONCE. Every build dir here is
# both walked for unlisted binaries and named by at least one TESTS row; a
# package listed in TESTS but missing here is listed-but-unwatched, which is
# the hole that hid ros2_ws/sensor until 2026-09-28.
BUILD_DIRS=("$QUAD_BUILD" "$RELAY_BUILD" "$SENSOR_BUILD")

# The three argv payloads, spelled once. These are the same values the
# add_test lines compute from CMAKE_CURRENT_SOURCE_DIR -- grep them with
#   grep -n "add_test" ros2_ws/quadruped/CMakeLists.txt
# Absolute on purpose: with argv supplied, none of these binaries reads a
# relative default any more, so a wrong cwd can no longer turn into a missing
# input file. The cwd is still set per binary below, as the second half of the
# same belt.
CHS_A_GOLDEN="$QUAD_DIR/test/golden/chs_a_frames.txt"
CONTRACT_MD="$REPO_ROOT/docs/11-接口契约.md"
SESSION_FACTORY_PY="$REPO_ROOT/xbrain/common/zenoh/session_factory.py"

allow_skip=0
list_only=0
only=()
for arg in "$@"; do
  case "$arg" in
    --allow-skip) allow_skip=1 ;;
    --list)       list_only=1 ;;
    -h|--help)
      # Print the header block: line 2 through the last comment line before the
      # first line of code. A hard-coded end line (it was '60') silently starts
      # truncating -- or spilling code into the help text -- the first time the
      # header grows, which is what happened when sensor was added.
      sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | sed '$d'
      exit 0
      ;;
    -*)
      echo "unknown option: $arg (try --help)" >&2
      exit 64
      ;;
    *) only+=("$arg") ;;
  esac
done

# ---------------------------------------------------------------------------
# ROS 2. Sourced once for the whole run rather than per binary: the setup
# script is slow and it only ever ADDS to the loader path, so the binaries
# that do not need it are unaffected.
#
# set -u is off for the duration because the ROS setup scripts read unset
# variables; a nounset abort inside them would read as a failure of this
# script (the same note build_quadruped.sh carries, for the same reason).
# ---------------------------------------------------------------------------
have_ros=0
if [ -f "$ROS_SETUP" ]; then
  set +u
  # shellcheck disable=SC1090
  if . "$ROS_SETUP" > /dev/null 2>&1; then have_ros=1; fi
  set -u
fi
if [ "$have_ros" = "1" ]; then
  echo "ros: sourced $ROS_SETUP"
else
  echo "ros: $ROS_SETUP unusable -- the three ROS-linked binaries will SKIP"
fi

# ---------------------------------------------------------------------------
# THE TABLE. One row per binary:
#
#   name | build dir | cwd | needs_ros | argv...
#
# The fields are separated by a vertical bar because no path here contains
# one; argv entries are separated by a tab for the same reason (a path with a
# space is fine, a path with a tab is not, and none of these has either).
#
# Why each row has the cwd it has:
#   * every quadruped binary runs from the PACKAGE dir. With argv supplied
#     nothing reads a relative path today, but five of them still carry a
#     package-relative default ("test/golden/chs_a_frames.txt") for a bare
#     run, and a future test that adds one must not start failing here.
#   * test_rt_keys is the exception worth naming: its bare-run default is
#     REPO-relative ("docs/11-接口契约.md"), so it is the one binary whose
#     bare run works from the repo root and nowhere else. It gets the repo
#     root, which is also what makes the deliberate wrong-cwd self-test
#     described at the bottom of this header produce a red run.
#   * test_quadruped_config and test_relay_config WRITE fixtures. They are
#     given the build dir, exactly as their add_test lines do, so fixtures
#     land in a directory that is already ignored by git and never next to
#     the source. Passing no directory makes them fall back to /tmp, which
#     works but leaves the files behind under a name nobody recognises.
#   * the relay binaries run from the relay package dir for the same reason
#     the quadruped ones run from theirs.
#   * the sensor binaries run from the sensor package dir, except
#     test_rtk_config: it writes its fixture to the CWD and takes no argv, so
#     it is the one row whose cwd is a build dir (see the row's own note).
# ---------------------------------------------------------------------------
TESTS=(
  # -- quadruped: pure unit tests, no argv, no environment -----------------
  "test_chs_a_session|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_chassis_socket|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_dds_names|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_mode_machine|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_odometry|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_rt_bridge|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_rt_parse|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_tick_stats|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_tier1|$QUAD_BUILD|$QUAD_DIR|0|"
  "test_tx_owner|$QUAD_BUILD|$QUAD_DIR|0|"
  # -- quadruped: argv[1] = the golden capture taken off the real chassis --
  # A missing golden file is a FAIL inside these binaries, not a skip, which
  # is why the path is passed rather than left to the relative default.
  "test_chs_a_codec|$QUAD_BUILD|$QUAD_DIR|0|$CHS_A_GOLDEN"
  "test_chs_a_framer|$QUAD_BUILD|$QUAD_DIR|0|$CHS_A_GOLDEN"
  "test_chs_a_reports|$QUAD_BUILD|$QUAD_DIR|0|$CHS_A_GOLDEN"
  "test_process|$QUAD_BUILD|$QUAD_DIR|0|$CHS_A_GOLDEN"
  "test_rt_payloads|$QUAD_BUILD|$QUAD_DIR|0|$CHS_A_GOLDEN"
  # -- quadruped: argv[1] = a document or a second implementation ----------
  # test_rt_keys greps the contract for every key verbatim; test_rt_session
  # compares this package's session config against session_factory.py, and
  # refuses to run at all without the path (argc < 2 prints FAIL).
  "test_rt_keys|$QUAD_BUILD|$REPO_ROOT|0|$CONTRACT_MD"
  "test_rt_session|$QUAD_BUILD|$QUAD_DIR|0|$SESSION_FACTORY_PY"
  # -- quadruped: argv[1] = a writable directory for fixtures --------------
  "test_quadruped_config|$QUAD_BUILD|$QUAD_DIR|0|$QUAD_BUILD"
  # -- quadruped: needs the ROS 2 loader path ------------------------------
  # test_chs_b links libddsc (CycloneDDS ships inside ROS on this host) and
  # the two uplink binaries link librclcpp. Without the setup script they do
  # not start at all -- the loader error arrives before main.
  "test_chs_b|$QUAD_BUILD|$QUAD_DIR|1|"
  "test_uplink|$QUAD_BUILD|$QUAD_DIR|1|"
  "test_uplink_alloc|$QUAD_BUILD|$QUAD_DIR|1|"
  # -- chassis_relay -------------------------------------------------------
  "test_envelope_rebuild|$RELAY_BUILD|$RELAY_DIR|0|"
  "test_relay_core|$RELAY_BUILD|$RELAY_DIR|0|"
  "test_relay_keys|$RELAY_BUILD|$RELAY_DIR|0|$CONTRACT_MD"
  "test_relay_config|$RELAY_BUILD|$RELAY_DIR|0|$RELAY_BUILD"
  # -- sensor (rtk_driver): plain-CMake package, no ament, no rclcpp --------
  # Nine binaries from one foreach in ros2_ws/sensor/CMakeLists.txt, all
  # offline: no argv, no fixture file, no hardware, no ROS (the package is
  # built with plain cmake precisely so it links none).
  "test_clock_status|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_gnss_fix|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_gnss_heading|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_heading_resolver|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_nmea_parser|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_rtk_driver|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_serial_reopen|$SENSOR_BUILD|$SENSOR_DIR|0|"
  "test_yaml_lite|$SENSOR_BUILD|$SENSOR_DIR|0|"
  # -- sensor: writes a fixture into the CWD, so the cwd is the build dir ---
  # test_rtk_config writes "test_rtk_cfg_tmp.yaml" beside itself and takes NO
  # argv, so unlike test_quadruped_config / test_relay_config the directory
  # cannot be handed to it -- the cwd IS the choice. The build dir is what its
  # add_test gets from ctest (WORKING_DIRECTORY defaults to the binary dir) and
  # it is git-ignored, so an aborted run leaves the temp file somewhere already
  # ignored instead of in the source tree.
  "test_rtk_config|$SENSOR_BUILD|$SENSOR_BUILD|0|"
)

if [ "$list_only" = "1" ]; then
  printf '%s\n' "${TESTS[@]}"
  exit 0
fi

# ---------------------------------------------------------------------------
# Drift guard, run BEFORE anything else. Every test binary sitting in either
# build dir must have a row above. A binary with no row would otherwise never
# run while the summary still said "all passed" -- the exact shape of an
# assertion that cannot go red.
# ---------------------------------------------------------------------------
unlisted=0
for build_dir in "${BUILD_DIRS[@]}"; do
  [ -d "$build_dir" ] || continue
  for path in "$build_dir"/test_*; do
    [ -x "$path" ] || continue
    [ -f "$path" ] || continue
    found=0
    for row in "${TESTS[@]}"; do
      if [ "${row%%|*}" = "$(basename "$path")" ]; then found=1; break; fi
    done
    if [ "$found" = "0" ]; then
      printf 'UNLISTED %s -- add a row to %s\n' "$path" "${BASH_SOURCE[0]}" >&2
      unlisted=$((unlisted + 1))
    fi
  done
done

pass=0
fail=0
skip=0
skipped_names=""
failed_names=""

for row in "${TESTS[@]}"; do
  name="${row%%|*}"
  rest="${row#*|}"
  build_dir="${rest%%|*}"; rest="${rest#*|}"
  work_dir="${rest%%|*}";  rest="${rest#*|}"
  needs_ros="${rest%%|*}"; argv_raw="${rest#*|}"

  # A name on the command line selects a subset; everything else is silent,
  # including the summary, so a single-binary run reads like a single result.
  if [ "${#only[@]}" -gt 0 ]; then
    wanted=0
    for want in "${only[@]}"; do
      if [ "$want" = "$name" ]; then wanted=1; break; fi
    done
    [ "$wanted" = "1" ] || continue
  fi

  bin="$build_dir/$name"

  if [ ! -x "$bin" ]; then
    # Not built. Two very different causes, and conflating them is how a
    # genuinely missing binary hides behind a plausible excuse: a ROS-linked
    # target is simply not configured on a host with no ROS (SKIP), while
    # anything else missing means the build did not run or did not finish.
    if [ "$needs_ros" = "1" ] && [ "$have_ros" = "0" ]; then
      printf 'SKIP %s -- not configured: ROS 2 absent on this host\n' "$name"
      skip=$((skip + 1)); skipped_names="$skipped_names $name"
    else
      printf 'FAIL %s -- binary missing at %s (build first: %s)\n' \
             "$name" "$bin" "scripts/build_quadruped.sh" >&2
      fail=$((fail + 1)); failed_names="$failed_names $name"
    fi
    continue
  fi

  if [ "$needs_ros" = "1" ] && [ "$have_ros" = "0" ]; then
    # Built earlier on a host that had ROS, running now on one that does not.
    # Reported rather than attempted: the loader error is not a test result.
    printf 'SKIP %s -- ROS 2 absent, binary links librclcpp/libddsc\n' "$name"
    skip=$((skip + 1)); skipped_names="$skipped_names $name"
    continue
  fi

  # argv: empty string means no arguments. The array form keeps a path with a
  # space intact, which "$*" would not.
  argv=()
  if [ -n "$argv_raw" ]; then
    # Tab is the separator; no argv value in the table contains one.
    IFS=$'\t' read -r -a argv <<< "$argv_raw"
  fi

  # A subshell so the cd applies to this binary only. Without it one row's cwd
  # leaks into the next, which is the failure mode this whole file exists to
  # remove.
  if out="$( cd "$work_dir" && "$bin" "${argv[@]}" 2>&1 )"; then
    printf 'PASS %s\n' "$name"
    pass=$((pass + 1))
  else
    printf 'FAIL %s\n' "$name" >&2
    printf '%s\n' "$out" | tail -20 >&2
    fail=$((fail + 1)); failed_names="$failed_names $name"
  fi
done

echo
printf 'cxx: %d passed, %d failed, %d skipped (of %d listed)\n' \
       "$pass" "$fail" "$skip" "$((pass + fail + skip))"
[ -n "$failed_names" ]  && printf 'failed: %s\n' "$failed_names" >&2
[ -n "$skipped_names" ] && printf 'skipped: %s\n' "$skipped_names"

if [ "$unlisted" -gt 0 ]; then
  printf 'cxx: unlisted built test binaries: %d -- see the UNLISTED lines\n' \
         "$unlisted" >&2
  exit 1
fi
if [ "$fail" -gt 0 ]; then
  exit 1
fi
if [ "$skip" -gt 0 ] && [ "$allow_skip" = "0" ]; then
  # Strict by default. Passing --allow-skip is a statement about the HOST
  # ("this machine has no ROS 2"), never about the tests, and it has to be
  # typed every time so it cannot become the silent norm.
  printf 'cxx: %d skipped and --allow-skip was not given\n' "$skip" >&2
  exit 1
fi
exit 0
