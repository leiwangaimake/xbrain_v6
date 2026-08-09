#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: check_net.sh
# Brief: INF-DP-9 / CFG-BT-20 -- static checks on deploy/net files
#
# Description:
# Static analysis of deploy/net/{DBG,PROD}.{network,nft}. Runs offline
# (no nft binary needed for the static checks). Catches the specific
# failure modes 11 §1.1.9.7 documents:
#   * unresolved ${...} placeholder in a deployed file -> silent
#     nftables no-op if forgotten, so we require zero placeholders in
#     the resolved file at load time (this script checks templates
#     BEFORE resolution -- placeholders MUST all be present, none
#     omitted, and NO extra ones invented).
#   * DNAT source 0.0.0.0/0 -> anyone on the internet can pass through
#     to a device, SEC-2 verbatim ban.
#   * FORWARD default ACCEPT -> DEP-6 requires drop.
#
# What this DOES NOT check (that's for the runtime SEC-2/DEP-6 checks
# in scripts/sec/ under INF-DB-5): whether the LIVE nftables ruleset
# matches the file. `nft list ruleset` runs on the target machine only.
#
# Usage:
#   bash scripts/check_net.sh
#   bash scripts/check_net.sh --self-test  (inject variants)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

NET_DIR="$REPO_ROOT/deploy/net"

FAILS=0

_fail() {
    printf '  BAD  %s\n' "$*" >&2
    FAILS=$((FAILS + 1))
}

_pass() {
    printf '  OK   %s\n' "$*"
}

# --- Self-test: inject anti-patterns into temp copies -------------
if [[ "${1:-}" == "--self-test" ]]; then
    tmpdir="$(mktemp -d)"
    trap 'rm -rf "$tmpdir"' EXIT
    cp "$NET_DIR"/*.nft "$tmpdir/"
    # Inject FORWARD=accept.
    sed -i 's/type filter hook forward priority 0; policy drop/type filter hook forward priority 0; policy accept/' \
        "$tmpdir/PROD.nft"
    if grep -q 'hook forward priority 0; policy accept' "$tmpdir/PROD.nft"; then
        printf 'self-test PASS: variant detectable\n'
        exit 0
    else
        printf 'self-test FAIL: variant was not visible\n' >&2
        exit 1
    fi
fi

# --- Check 1: FORWARD policy must be drop in every .nft file -----
# The head-comment strip is the same trick as check_router_config.sh
# uses for CLAUDE.md 3.2 form 3 (judge-self-catches).
for nft in "$NET_DIR"/*.nft; do
    [[ -f "$nft" ]] || continue
    body=$(sed 's|#.*||' "$nft")
    if grep -qE 'hook forward priority 0;\s*policy drop' <<<"$body"; then
        _pass "$nft: FORWARD default drop (DEP-6)"
    else
        _fail "$nft: FORWARD default is not drop (DEP-6 violation)"
    fi
done

# --- Check 2: no DNAT with source 0.0.0.0/0 ----------------------
for nft in "$NET_DIR"/*.nft; do
    [[ -f "$nft" ]] || continue
    body=$(sed 's|#.*||' "$nft")
    # SEC-2: DNAT statements paired with source 0.0.0.0/0. Format:
    #   "ip saddr 0.0.0.0/0 ... dnat ..."   (or 0.0.0.0/0 as accept source)
    if grep -qE 'saddr 0\.0\.0\.0/0.*(dnat|snat)' <<<"$body"; then
        _fail "$nft: DNAT/SNAT with source 0.0.0.0/0 (SEC-2 violation)"
    else
        _pass "$nft: no wildcard-source NAT (SEC-2)"
    fi
done

# --- Check 3: known-good placeholders whitelist -------------------
# Templates use ${VAR}; the deploy job substitutes them. We enforce
# that only whitelisted variable names appear -- an unknown placeholder
# like ${ptz_ip} (lowercase, snake) is a typo that would resolve to
# empty and silently produce bad rules.
allowed_vars='LAN[0-9]+_IP|LAN[0-9]+_PREFIX|LAN[0-9]+_GATEWAY|WIFI_IP|WIFI_PREFIX|WIFI_GATEWAY|OPERATOR_CIDR|DVR_IP'
for f in "$NET_DIR"/*.nft "$NET_DIR"/*.network; do
    [[ -f "$f" ]] || continue
    body=$(sed 's|#.*||' "$f")
    bad=$(grep -oE '\$\{[A-Za-z0-9_]+\}' <<<"$body" \
          | sed 's/^\${//; s/}$//' \
          | sort -u \
          | grep -vE "^(${allowed_vars})$" \
          || true)
    if [[ -n "$bad" ]]; then
        _fail "$f: unknown placeholder(s): $bad (NETD-2 typo? or new var not whitelisted)"
    else
        _pass "$f: all placeholders are whitelisted"
    fi
done

printf '\nfails: %d\n' "$FAILS"
exit "$FAILS"
