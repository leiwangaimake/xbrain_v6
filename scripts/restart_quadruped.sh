#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: restart_quadruped.sh
# Brief: Restart the quadruped_m20 process from the build tree, one instance only
#
# Description:
# What this solves. Restarting quadruped_m20 by hand is a five-part command
# (kill, wait, cd, source ROS, setsid nohup) and every part has bitten this
# project at least once:
#
#   * Typing it as `ssh xbrain '<the whole thing>'` while ALREADY logged into
#     the ORIN makes the box ssh to itself (127.0.1.1), which fails on host key
#     verification. The failure text talks about keys, not about the restart,
#     so it reads as an access problem rather than as "you are already there".
#   * A blind `sleep 3` after pkill assumes the old process died in three
#     seconds. When it does not, the new one starts while the old one still
#     holds the chassis TCP socket -- and 13 CA-1 is explicit that a second
#     client makes the chassis answer axis commands with 0xE006 for two
#     seconds, which presents as "the robot does not move" and NOT as a
#     duplicate-process error. This script polls for the exit instead.
#   * Forgetting to source ROS gives "error while loading shared libraries:
#     libiceoryx_binding_c.so", which looks like a broken build.
#   * Forgetting RMW_IMPLEMENTATION leaves the default RMW, and channel three
#     (uplink, domain 42) then silently matches nobody -- 13 S5.4 records that
#     a DDS mismatch is indistinguishable from a dead network.
#
# What it does NOT do: it does not build (scripts/build_quadruped.sh owns
# that), does not install into data/install, and does not touch systemd. It
# runs the binary straight out of the build tree, which is how this process is
# operated during bring-up.
#
# Boundary: the chassis link state it prints at the end is READ from the log,
# not waited on. A restart that comes up with the chassis unplugged is a
# successful restart; the link line is there so the operator sees which of the
# two happened without having to go read the log themselves.
set -euo pipefail

# Derive the repo root from this script's own location. A hard-coded root
# (CLAUDE.md 6) would restart a different tree than the one the operator is
# standing in, which is exactly the failure this script exists to prevent.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
BUILD_DIR="$REPO_ROOT/ros2_ws/quadruped/build"
BINARY="$BUILD_DIR/quadruped_m20"
LOG_FILE="/tmp/q_ch2.log"

# The binary must exist before anything is killed. Killing first and finding
# out afterwards that there is nothing to start leaves the chassis with no
# client at all -- a worse state than the one we started in.
if [ ! -x "$BINARY" ]; then
  printf 'no binary at %s\n' "$BINARY" >&2
  printf 'run scripts/build_quadruped.sh first\n' >&2
  exit 1
fi

# pkill exits 1 when it matched nothing, which is a normal state here (first
# start after a reboot). `set -e` would abort on that, so the exit code is
# swallowed deliberately rather than by accident.
printf 'stopping any running quadruped_m20 ...\n'
pkill -x quadruped_m20 || true

# Poll for the exit rather than sleeping a fixed time. CA-1 (13 S2.2) forbids
# two live clients to the chassis, so the new process must not start until the
# old one is gone -- and "gone" is something to observe, not to assume.
for _ in $(seq 1 40); do
  if ! pgrep -x quadruped_m20 > /dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

# Still alive after ten seconds means SIGTERM did not take. Report it and stop:
# escalating to SIGKILL here would risk leaving the chassis socket in a state
# the next dial has to time out on, and that decision belongs to the operator.
if pgrep -x quadruped_m20 > /dev/null 2>&1; then
  printf 'quadruped_m20 still alive after SIGTERM:\n' >&2
  pgrep -af '[q]uadruped_m20' >&2
  exit 1
fi

# ROS is sourced for the loader path (channel three needs rclcpp's libraries).
# `set -u` is relaxed across the source because the ROS setup scripts read
# unset variables of their own; it is restored immediately after.
printf 'sourcing ROS 2 humble ...\n'
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# CycloneDDS is the RMW this project standardised on (13 S5.4). Exported rather
# than prefixed so it reaches the process through setsid.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# setsid detaches the process from this shell's session, so it survives the ssh
# connection closing. Without it the process dies with the terminal and the
# operator finds a dead chassis link minutes later with no clue why.
printf 'starting quadruped_m20 ...\n'
cd "$BUILD_DIR"
setsid nohup ./quadruped_m20 > "$LOG_FILE" 2>&1 < /dev/null &

# Give it a moment to either come up or fail, then assert it is actually there.
# Reporting success without checking is how a restart script becomes a thing
# people trust while it does nothing (CLAUDE.md 3.2 form 1).
sleep 4
if ! pgrep -x quadruped_m20 > /dev/null 2>&1; then
  printf 'quadruped_m20 did not stay up; last log lines:\n' >&2
  tail -20 "$LOG_FILE" >&2
  exit 1
fi

printf '\nrunning:\n'
pgrep -af '[q]uadruped_m20'

# The instance count is printed because a second copy is the one failure mode
# that looks fine from every other angle (CA-1: the chassis just refuses axis
# commands for two seconds at a time).
printf '\ninstances: %s (must be 1)\n' "$(pgrep -cx quadruped_m20)"

# Read whatever the link has said so far. Absent is not an error -- on a cold
# start the first probe cycle has not necessarily completed yet.
printf '\nchassis link (from %s):\n' "$LOG_FILE"
grep -a 'chassis link' "$LOG_FILE" 2>/dev/null | tail -3 || printf '  (nothing logged yet)\n'
