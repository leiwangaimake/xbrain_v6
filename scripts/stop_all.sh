#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: stop_all.sh
# Brief: CFG-BT-7 -- staged full-stack shutdown (manual entry point)
#
# Description:
# Reverse-order counterpart to start_all.sh. Shuts down every stage
# in strict reverse (5 -> 4 -> 3 -> 2 -> 1 -> 0c -> 0z -> 0) so
# dependents stop before their dependencies.
#
# Uses `systemctl stop` (not disable) so units come back on the next
# start_all.sh invocation without re-enabling.
#
# Failure policy: continue on error (a unit that is not running is
# expected). Report exit codes at the end.
#

set -u -o pipefail   # NO -e: unit-not-running is not a failure

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DRY_RUN="${DRY_RUN:-0}"

_sudo_systemctl_stop() {
    printf '  systemctl stop %s\n' "$*"
    if [[ "$DRY_RUN" != "1" ]]; then
        sudo systemctl stop "$@" || true
    fi
}

_stage_hdr() {
    printf '\n==> [stage %s down] %s\n' "$1" "$2"
}

# --- Stage 5 down --------------------------------------------------
_stage_hdr 5 "AI runtime"
_sudo_systemctl_stop xbrain-llm.service
_sudo_systemctl_stop xbrain-asr.service
_sudo_systemctl_stop xbrain-payload-service.service

# --- Stage 3 down --------------------------------------------------
_stage_hdr 3 "general-plane"
_sudo_systemctl_stop xbrain-behavior-proxy.service
_sudo_systemctl_stop xbrain-p5-gateway.service
_sudo_systemctl_stop xbrain-p4-agent.service
_sudo_systemctl_stop xbrain-p3-task.service
_sudo_systemctl_stop xbrain-p2-core.service

# --- Stage 2 down --------------------------------------------------
_stage_hdr 2 "cross-plane"
_sudo_systemctl_stop xbrain-perception.service
_sudo_systemctl_stop xbrain-chassis-relay.service

# --- Stage 1 down --------------------------------------------------
_stage_hdr 1 "RT-plane"
_sudo_systemctl_stop xbrain-quadruped.service
_sudo_systemctl_stop xbrain-p1-motion.service

# --- Stage 0c down -------------------------------------------------
_stage_hdr 0c "config freeze (oneshot; nothing to stop)"

# --- Stage 0z down -------------------------------------------------
_stage_hdr 0z "Zenoh routers + chassis probe"
_sudo_systemctl_stop xbrain-chassis-probe.service
_sudo_systemctl_stop xbrain-zenohd-gen.service
_sudo_systemctl_stop xbrain-zenohd-rt.service

# --- Stage 0 down --------------------------------------------------
_stage_hdr 0 "platform probe (oneshot; nothing to stop)"

printf '\nstop_all.sh: done\n'
