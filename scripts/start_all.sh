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
#   1     RT-plane participants       p1_motion / quadruped / ...
#   2     cross-plane participants    chassis_relay / perception
#   3     general-plane 5 processes   p2_core / p3_task / ...
#   4     RELEASE (P2 internal)       NOT a systemd stage
#   5     AI runtime AFTER release    LLM + ASR services
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
# 0z-3 is soft-gated: chassis unreachable does not halt startup.
# We invert the fail-fast policy for this one call by wrapping.
set +e
_sudo_systemctl start xbrain-chassis-probe.service
set -e

# --- Stage 0c: config freeze ----------------------------------------
_stage_hdr 0c "config freeze (assertion table + resolved/)"
_sudo_systemctl start xbrain-config-freeze.service

# --- Stage 1: RT-plane participants ---------------------------------
_stage_hdr 1 "RT-plane participants"
# rtk_driver first so its rt/gnss/* + rt/clock/status are already publishing when
# p1_motion subscribes them (Zenoh reconnects regardless; this just avoids a first
# empty poll). 11 S3.3 / S3.2 / S3.11.
_sudo_systemctl start xbrain-rtk-driver.service
_sudo_systemctl start xbrain-p1-motion.service
_sudo_systemctl start xbrain-quadruped.service

# --- Stage 2: cross-plane participants ------------------------------
_stage_hdr 2 "cross-plane participants"
_sudo_systemctl start xbrain-chassis-relay.service
_sudo_systemctl start xbrain-perception.service

# --- Stage 3: general-plane 5 processes -----------------------------
_stage_hdr 3 "general-plane 5 processes"
_sudo_systemctl start xbrain-p2-core.service
_sudo_systemctl start xbrain-p3-task.service
_sudo_systemctl start xbrain-p4-agent.service
_sudo_systemctl start xbrain-p5-gateway.service
_sudo_systemctl start xbrain-behavior-proxy.service

# --- Stage 4: release ----------------------------------------------
# P2-internal state; not a systemd unit. Waits for p2_core to sees
# BIT results and releases; no action from this script.
_stage_hdr 4 "release (P2 internal; no systemd unit -- waits for BIT)"

# --- Stage 5: AI runtime (after release) ---------------------------
_stage_hdr 5 "AI runtime (LLM + ASR, AFTER release)"
_sudo_systemctl start xbrain-asr.service
_sudo_systemctl start xbrain-llm.service
_sudo_systemctl start xbrain-payload-service.service

printf '\nstart_all.sh: done\n'
