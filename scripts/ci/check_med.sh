#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: check_med.sh
# Brief: INF-MD-1 -- media plane MED static checks (C1..C4 + secrets)
#
# Description:
# Media plane (MED) is the fourth network plane in 11 S1.1 -- the
# RTSP + DNAT path from an operator to a PTZ / DVR. It is the plane
# that MUST NOT touch the safety loop; MED failures cannot slow, stop,
# or preempt motion (MED-C4). Media-plane rules also carry a heightened
# secret risk: default PTZ passwords, ONVIF creds. This script bundles
# the static portion of MED-C1..C4 into one runnable check.
#
# What is static-checkable here (this script):
#   * MED-C1 partial: nft template maps ONLY registered media ports
#     (554, 8554). Anything else in the media DNAT would surface.
#   * MED-C2: no DNAT with saddr 0.0.0.0/0. Delegates to check_net.sh
#     which already enforces this (SEC-2).
#   * SEC-5 overlap: onvif_credentials.json / any *.yaml under configs/
#     that lists a plain-text password fails. Delegates to SEC-05.
#
# What needs real device (runtime):
#   * MED-C1 runtime: portscan from LAN2 confirming PTZ Web/ONVIF closed
#   * MED-C3: default-password login must fail (= SEC-01)
#   * MED-C4: while RTSP failing, safety loop unaffected (needs live
#     P1 + fake RTSP source)
# Those live in scripts/sec/ (SEC-01) and integration tests.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

FAILS=0
_fail() { printf '  BAD  %s\n' "$*" >&2; FAILS=$((FAILS + 1)); }
_pass() { printf '  OK   %s\n' "$*"; }

# --- Check 1: media ports whitelist -------------------------------
# Only 554 (RTSP) + 8554 (RTSP alt) may appear as DNAT target ports.
# A PTZ Web port (80/443/8080/8443) in DNAT is a MED-C1 violation.
for nft in "$REPO_ROOT"/deploy/net/*.nft; do
    [[ -f "$nft" ]] || continue
    body=$(sed 's|#.*||' "$nft")
    # Extract dport values that appear inside a dnat clause on same line.
    bad=$(grep -oE 'tcp dport [0-9]+.*dnat' <<<"$body" \
          | grep -oE 'dport [0-9]+' \
          | awk '{print $2}' \
          | sort -u \
          | grep -vE '^(554|8554)$' \
          || true)
    if [[ -n "$bad" ]]; then
        _fail "$nft: DNAT to non-media port(s): $bad (MED-C1)"
    else
        _pass "$nft: DNAT ports whitelist (MED-C1)"
    fi
done

# --- Check 2: delegate MED-C2 (no 0.0.0.0/0 source) to check_net.sh
if bash "$REPO_ROOT"/scripts/check_net.sh >/dev/null 2>&1; then
    _pass "check_net.sh green (MED-C2 + FORWARD drop + no wildcard)"
else
    _fail "check_net.sh reports issues; run it standalone for detail"
fi

# --- Check 3: delegate SEC-5 (secret pattern grep) ----------------
if bash "$REPO_ROOT"/scripts/sec/checks/SEC-05-secrets-grep.sh \
   | grep -q '"status": "PASS"'; then
    _pass "SEC-05 green (no PTZ/ONVIF creds in configs/*.yaml)"
else
    _fail "SEC-05 red: secret patterns present"
fi

printf '\nfails: %d\n' "$FAILS"
exit "$FAILS"
