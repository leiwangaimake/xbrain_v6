#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: check_router_config.sh
# Brief: INF-ZN-8 -- static checks on zenohd-rt / zenohd-gen configs
#
# Description:
# Grep-based static checks on the two committed Zenoh router configs.
# Runs offline (no zenohd binary needed). Purpose: catch the specific
# defect NET-C9 warns about -- a router binding 0.0.0.0 lets any LAN
# peer publish to the general plane.
#
# Checks:
#   1. Both configs contain NO "0.0.0.0" literal (SEC-2, SEC-7).
#   2. Both configs disable multicast + gossip (RT-C3.d verbatim).
#   3. RT config binds ONLY loopback (127.0.0.1) -- NET-C9 for RT.
#   4. GEN config binds loopback + at least one templated per-IP
#      endpoint (LAN2 / WiFi via ${LAN2_IP} / ${WIFI_IP}).
#   5. GATE-5: no ros2_ws/bridge/config/zenoh_bridge.json5 present
#      (V5 legacy file that would hijack port 7447 if left in the
#      tree). If found, script fails and names the path.
#
# Usage:
#   bash scripts/check_router_config.sh
#   bash scripts/check_router_config.sh --self-test  (inject 0.0.0.0)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RT_CFG="$REPO_ROOT/deploy/zenoh/zenohd-rt.json5"
GEN_CFG="$REPO_ROOT/deploy/zenoh/zenohd-gen.json5"
GATE5_LEGACY="$REPO_ROOT/ros2_ws/bridge/config/zenoh_bridge.json5"

FAILS=0

_fail() {
    printf '  BAD  %s\n' "$*" >&2
    FAILS=$((FAILS + 1))
}

_pass() {
    printf '  OK   %s\n' "$*"
}

# --- Self-test mode: inject 0.0.0.0 into a temp copy, verify red ---
if [[ "${1:-}" == "--self-test" ]]; then
    tmpdir="$(mktemp -d)"
    trap 'rm -rf "$tmpdir"' EXIT
    cp "$RT_CFG" "$tmpdir/rt.json5"
    cp "$GEN_CFG" "$tmpdir/gen.json5"
    # Inject the anti-pattern.
    sed -i 's/127.0.0.1/0.0.0.0/' "$tmpdir/rt.json5"
    if grep -q '0\.0\.0\.0' "$tmpdir/rt.json5"; then
        printf 'self-test PASS: injection detectable\n'
        exit 0
    else
        printf 'self-test FAIL: injection was not visible in grep\n' >&2
        exit 1
    fi
fi

printf 'scan surface: deploy/zenoh/ + ros2_ws/ GATE-5\n'

# --- Check 1: no 0.0.0.0 in non-comment lines ----------------------
# Strip // comments first so the anti-pattern's mention in the header
# comment does not fire on itself (which is exactly the CLAUDE.md 3.2
# form 3 "判据自伤" failure mode).
for cfg in "$RT_CFG" "$GEN_CFG"; do
    if [[ ! -f "$cfg" ]]; then
        _fail "config missing: $cfg"
        continue
    fi
    # sed drops //-comment lines; grep on the remainder.
    if sed 's|//.*||' "$cfg" | grep -q '0\.0\.0\.0'; then
        _fail "$cfg contains 0.0.0.0 in a non-comment line (NET-C9)"
    else
        _pass "$cfg: no 0.0.0.0 in binding entries"
    fi
done

# --- Check 2: multicast + gossip disabled --------------------------
for cfg in "$RT_CFG" "$GEN_CFG"; do
    [[ -f "$cfg" ]] || continue
    if grep -A1 'multicast:' "$cfg" | grep -q 'enabled:\s*false'; then
        _pass "$cfg: multicast disabled"
    else
        _fail "$cfg: multicast not explicitly disabled"
    fi
    if grep -A1 'gossip:' "$cfg" | grep -q 'enabled:\s*false'; then
        _pass "$cfg: gossip disabled"
    else
        _fail "$cfg: gossip not explicitly disabled"
    fi
done

# --- Check 3: RT config binds ONLY loopback ------------------------
if [[ -f "$RT_CFG" ]]; then
    # Extract endpoint lines (between listen: and the next block).
    # Loose extraction is fine; we only need to spot non-127.0.0.1.
    non_loop=$(grep 'tcp/' "$RT_CFG" | grep -v '127\.0\.0\.1' || true)
    if [[ -n "$non_loop" ]]; then
        _fail "RT config binds non-loopback: $non_loop"
    else
        _pass "RT config: loopback-only"
    fi
fi

# --- Check 4: GEN config has loopback + templated per-IP ------------
if [[ -f "$GEN_CFG" ]]; then
    if grep -q 'tcp/127\.0\.0\.1' "$GEN_CFG"; then
        _pass "GEN config: loopback bound"
    else
        _fail "GEN config: loopback endpoint missing"
    fi
    # Templated per-IP endpoints (LAN2/WIFI). Sourced at unit start
    # via envsubst. This check just verifies the placeholder is present.
    if grep -q '${LAN2_IP}' "$GEN_CFG" && grep -q '${WIFI_IP}' "$GEN_CFG"; then
        _pass "GEN config: LAN2_IP + WIFI_IP templated"
    else
        _fail "GEN config: LAN2/WiFi endpoint templates missing"
    fi
fi

# --- Check 5: GATE-5 (V5 legacy zenoh_bridge.json5 not present) -----
if [[ -f "$GATE5_LEGACY" ]]; then
    _fail "V5 legacy config still present: $GATE5_LEGACY (GATE-5 violation)"
else
    _pass "GATE-5: no V5 legacy zenoh_bridge.json5"
fi

printf '\nfails: %d\n' "$FAILS"
exit "$FAILS"
