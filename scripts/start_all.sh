#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: start_all.sh
# Brief: CFG-BT-7 -- staged full-stack startup (manual entry point)
#
# Description:
# Manual-run counterpart to the systemd stage sequence in 10 S3.3.
# Same order, same gates; scripted so an operator can bring the
# stack up (or bring it down via stop_all.sh) without knowing which
# systemctl unit belongs to which stage.
#
# Stage sequence (10 S3.3.6 verbatim):
#   0     xbrain-probe.service        platform + GATE-6 net probe
#   0z-1  RT router (Zenoh)           tcp/7449
#   0z-2  GEN router (Zenoh)          tcp/7447
#   0z-3  chassis link probe          tcp/30003 (soft-gated)
#   0c    xbrain-config-freeze        assertion table + resolved/
#   1     RT-plane participants       quadruped -> perception -> rtk_driver ->
#                                       behavior_proxy -> nav2 -> zenoh-bridge -> teleop
#   2     cross-plane participants    chassis_relay -> p1_motion (GATE-3/4)
#   3     general-plane 5 processes   p2_core -> p3_task -> p4_agent -> p5_gateway
#   4     RELEASE (P2 internal)       NOT a systemd stage
#   5     AI runtime AFTER release    ai-asr + llm + payload (no on-board TTS)
#
# Stage 4 (release) is a P2 internal state, not a systemd unit,
# so this script cannot 'start' it -- it happens on its own once
# stage 3 is up.
#
# CFG-BT-7 variants (each MUST turn red in shellcheck / smoke test):
#   1) any `rm -rf $VAR/` (bare variable) -- shellcheck flags
#   2) drop `set -euo pipefail` -- test asserts a failing step
#      halts the script rather than continuing
#

set -euo pipefail

# Repo root derived from this script's own location. NEVER hard-code
# /opt/xbrain_v6 -- deployments may relocate the tree.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Optional: pass DRY_RUN=1 to just print the systemctl commands
# without invoking them (useful on a dev machine without the units).
DRY_RUN="${DRY_RUN:-0}"

# systemctl wrapper: prints then runs (or just prints if DRY_RUN=1).
# Return code of the real systemctl call propagates via set -e.
_sudo_systemctl() {
    local action="$1"; shift
    printf '  systemctl %s %s\n' "$action" "$*"
    if [[ "$DRY_RUN" != "1" ]]; then
        # sudo used because unit management is root-owned; a local
        # dev with DRY_RUN=1 skips this branch entirely.
        sudo systemctl "$action" "$@"
    fi
}

# Print stage header.
_stage_hdr() {
    printf '\n==> [stage %s] %s\n' "$1" "$2"
}

# --- Stage 0: platform probe ----------------------------------------
_stage_hdr 0 "platform + GATE-6 net probe"
_sudo_systemctl start xbrain-probe.service

# --- Stage 0z: Zenoh routers + chassis link -------------------------
_stage_hdr 0z-1 "RT router (Zenoh tcp/7449)"
_sudo_systemctl start xbrain-zenohd-rt.service

_stage_hdr 0z-2 "GEN router (Zenoh tcp/7447)"
_sudo_systemctl start xbrain-zenohd-gen.service

_stage_hdr 0z-3 "chassis link probe (tcp/30003, soft-gated)"
# 0z-3 soft gate (10 S3.3.8): the chassis-face TLS reachability probe is done
# BY xbrain-probe.service (its executor per S3.3.8), NOT a separate unit --
# there is no xbrain-chassis-probe.service. Soft gate means an unreachable
# chassis must NOT halt startup (10 S3.3.6: unlike the router hard gate, a
# chassis-down stays diagnosable -- HMI comes up and shows state/robot.conn=
# lost). Nothing to systemctl-start here; kept as an explicit stage marker so
# the sequence stays legible (and the CFG-BT-7 stage-header test still sees it).

# --- Stage 0c: config freeze ----------------------------------------
_stage_hdr 0c "config freeze (assertion table + resolved/)"
_sudo_systemctl start xbrain-config-freeze.service

# --- Stage 1: RT-plane participants ---------------------------------
_stage_hdr 1 "RT-plane participants"
# 10 S3.3 verbatim order: quadruped -> perception -> rtk_driver -> behavior_proxy,
# and alongside them Nav2 behavior_server + zenoh-bridge-ros2dds. These are the
# RT-plane pure participants; p1_motion is NOT here (it is a Stage-2 cross-plane
# point). teleop_input is an RT-plane pub-only participant (10 S3.1) the S3.3
# diagram does not draw -- placed here because it must precede p1_motion (which
# reads cmd/teleop) and, like the rest, is RT-plane so precedes Stage 2.
_sudo_systemctl start xbrain-quadruped.service
_sudo_systemctl start xbrain-perception.service
_sudo_systemctl start xbrain-rtk-driver.service
_sudo_systemctl start xbrain-behavior-proxy.service
_sudo_systemctl start xbrain-nav2-behavior.service
_sudo_systemctl start xbrain-zenoh-bridge.service
_sudo_systemctl start xbrain-teleop-input.service

# --- Stage 2: cross-plane participants (order fixed: GATE-3/4) -------
_stage_hdr 2 "cross-plane participants"
# 10 S3.3 / GATE-4: chassis_relay MUST precede p1_motion (P1 emits 20 Hz cmd_vel
# the instant it starts, so the estop link must already be up). GATE-3:
# quadruped (Stage 1) precedes P1 (the 11 S9.1.4 handshake is initiated by P1).
# This order is NOT interchangeable. perception is a Stage-1 participant, not
# here (the old script wrongly put it in Stage 2).
_sudo_systemctl start xbrain-chassis-relay.service
_sudo_systemctl start xbrain-p1-motion.service

# --- Stage 3: general-plane 5 processes -----------------------------
_stage_hdr 3 "general-plane 5 processes"
# 10 S3.3: p2_core -> p3_task -> p4_agent -> p5_gateway. behavior_proxy is NOT
# here -- it is an RT-plane Stage-1 participant (the old script wrongly listed
# it here). p2_core runs its internal Stage A -> B(BIT 19) -> C(WAV announce) ->
# D(release) machine; the release (Stage 4) is P2's own action, not a systemd
# unit, so this script cannot start it.
_sudo_systemctl start xbrain-p2-core.service
_sudo_systemctl start xbrain-p3-task.service
_sudo_systemctl start xbrain-p4-agent.service
_sudo_systemctl start xbrain-p5-gateway.service

# --- Stage 4: release ----------------------------------------------
# P2-internal state; not a systemd unit. Waits for p2_core to sees
# BIT results and releases; no action from this script.
_stage_hdr 4 "release (P2 internal; no systemd unit -- waits for BIT)"

# --- Stage 5: AI runtime (after release) ---------------------------
_stage_hdr 5 "AI runtime (LLM + ASR, AFTER release)"
# 10 S3.3: AI Runtime starts AFTER release (Stage 4) and is decoupled from it --
# a slow 1.9 GB LLM weight load must not delay 出勤 (BIT-03). Machine has NO
# on-board TTS (it lives in the GZH-2 device, U52), so no TTS unit here.
# Correct unit names: xbrain-ai-asr (the old script's xbrain-asr does not exist)
# and xbrain-payload (not xbrain-payload-service). payload-service is the GZH-2
# 三合一 peripheral (audio/lights/payload), started with the AI runtime tail.
_sudo_systemctl start xbrain-ai-asr.service
_sudo_systemctl start xbrain-llm.service
_sudo_systemctl start xbrain-payload.service

printf '\nstart_all.sh: done\n'
