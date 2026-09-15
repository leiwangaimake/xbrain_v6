#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: build_quadruped.sh
# Brief: Configure, build, test and install ros2_ws/quadruped into data/install
#
# Description:
# What problem this solves. DEC-15 (99 U83) fixed the install root at
# /opt/xbrain_v6/data/install and the layout at <pkg>/lib/<pkg>/<exe>, and the
# systemd unit hard-codes that exact path -- but nothing put a binary there.
# Typing the four cmake commands by hand is where a deployment drifts from the
# unit file: a --prefix typo installs to a path the unit will never look at,
# and the unit then skips cleanly (ConditionPathExists), which reads as
# "nothing to do" rather than "the binary is in the wrong place".
#
# Order is build, TEST, install -- never install first. An installed binary
# un-gates the systemd unit, so installing something whose tests have not run
# hands systemd a binary nobody has checked.
#
# ROS 2 is optional here, deliberately. The core and its tests do not need it
# (see ros2_ws/quadruped/CMakeLists.txt for why); if /opt/ros/humble is present
# it is sourced so the optional uplink and domain-0 targets are configured, and
# if it is absent the script says so and continues rather than failing. A build
# script that refuses to run without ROS would make the ROS-free core -- the
# part that carries every unit test -- untestable on any other machine.
#
# Usage:
#   scripts/build_quadruped.sh              build + test (no install)
#   scripts/build_quadruped.sh --install    build + test + install
#   scripts/build_quadruped.sh --clean      remove the build tree first
#
# Boundary: this does not enable or start the systemd unit. Enabling is an
# outward, on-reboot change (scripts/install_units.sh --enable owns it), and
# the B0 binary is a self-check that refuses to run as a service anyway.
set -euo pipefail

# Derive paths from this script's location; never hard-code a repo root
# (CLAUDE.md 6) -- a hard-coded root silently builds the wrong tree on a
# machine that has two checkouts.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
PKG_DIR="$REPO_ROOT/ros2_ws/quadruped"
BUILD_DIR="$PKG_DIR/build"
INSTALL_PREFIX="$REPO_ROOT/data/install/quadruped"
# The path the unit executes. Kept here as one string so the check below and
# the message quote the same thing the unit file does.
UNIT_BINARY="$INSTALL_PREFIX/lib/quadruped/quadruped_m20"
ROS_SETUP="/opt/ros/humble/setup.bash"

do_install=0
do_clean=0
for arg in "$@"; do
  case "$arg" in
    --install) do_install=1 ;;
    --clean)   do_clean=1 ;;
    -h|--help)
      sed -n '2,40p' "${BASH_SOURCE[0]}"
      exit 0
      ;;
    *)
      echo "unknown argument: $arg (try --help)" >&2
      exit 64
      ;;
  esac
done

if [ "$do_clean" = "1" ]; then
  # Explicit path, never a bare variable (CLAUDE.md 6): an empty BUILD_DIR here
  # would delete the filesystem root.
  rm -rf "$PKG_DIR/build"
  echo "removed $PKG_DIR/build"
fi

# Source ROS if present. `set -u` is off for the duration because the ROS setup
# scripts read unset variables, and a nounset abort there would look like a
# build failure.
if [ -f "$ROS_SETUP" ]; then
  set +u
  # shellcheck disable=SC1090
  . "$ROS_SETUP"
  set -u
  echo "ros: sourced $ROS_SETUP (optional uplink / domain-0 targets configurable)"
else
  echo "ros: $ROS_SETUP not found -- building the ROS-free core and tests only"
fi

echo "== configure =="
cmake -S "$PKG_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release

echo "== build =="
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "== test =="
# Tests run BEFORE any install: see the header for why that order is not
# cosmetic.
ctest --test-dir "$BUILD_DIR" --output-on-failure

if [ "$do_install" = "1" ]; then
  echo "== install =="
  cmake --install "$BUILD_DIR" --prefix "$INSTALL_PREFIX"
  # Verify the binary landed at the path the unit will execute. Checking the
  # install command's exit status is not enough: a wrong DESTINATION exits 0
  # and puts the file somewhere the unit never looks, and the unit then skips
  # silently.
  if [ ! -x "$UNIT_BINARY" ]; then
    echo "install did not produce $UNIT_BINARY -- the systemd unit executes" \
         "exactly that path (deploy/systemd/xbrain-quadruped.service)" >&2
    exit 1
  fi
  echo "installed: $UNIT_BINARY"
  echo
  echo "NOTE: the unit is now un-gated (ConditionPathExists is satisfied)."
  echo "      The B0 binary refuses to run as a service and exits 78; the unit"
  echo "      carries RestartPreventExitStatus=78 so that is one loud failure,"
  echo "      not a restart loop. Both go away when the process is real."
else
  echo
  echo "not installed (pass --install). Self-check the built binary with:"
  echo "  $BUILD_DIR/quadruped_m20 --selfcheck <resolved.yaml>"
fi
