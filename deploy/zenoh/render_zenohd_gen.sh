#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: render_zenohd_gen.sh
# Brief: Render the GEN router config from network.env, skipping an unusable wifi endpoint
#
# Description:
# xbrain-zenohd-gen.service runs this as ExecStartPre to turn the template
# deploy/zenoh/zenohd-gen.json5 into /run/xbrain/zenoh/zenohd-gen.json5, with
# LAN2_IP / WIFI_IP from deploy/env/network.env.
#
# Why it is a script and not a bare `envsubst` (the unit's form until 2026-10-26):
# 99 U92 ruling 5 -- "wifi 也绑上, 但是 wifi 失效不要影响 188 自组网网络" -- and
# zenohd refuses the WHOLE listen set when one endpoint cannot be bound. Measured
# 2026-10-26 on .1.26: a listen list holding the same address twice makes zenohd
# exit 255 ("Address already in use"), and network.env's wifi placeholder is
# 127.0.0.1, which the template already lists. So the bare envsubst produced a
# router that could not start at all; an absent wifi address fails the same way.
# This script drops the wifi endpoint line when the address is unusable:
#   empty, 0.0.0.0, a loopback 127.x (the template has its own loopback), equal
#   to LAN2_IP, or not present on any interface.
# The dev launcher scripts/dev/run_stack_cloud.sh applies the same rule.
#
# LAN2 is NOT optional (U92: the delivery network): an empty, wildcard or absent
# LAN2_IP exits non-zero and the router does not start -- loudly, instead of
# binding loopback only and looking healthy.
#
# Boundaries: renders and validates the address list only. The unit's own
# ExecStartPre lines still refuse a leftover ${...} placeholder or any 0.0.0.0
# in the output; they are kept as a second, independent check.
#
# Usage: render_zenohd_gen.sh <template.json5> <output.json5>
#        (LAN2_IP / WIFI_IP from the environment, as EnvironmentFile= sets them)
#
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "usage: render_zenohd_gen.sh <template.json5> <output.json5>" >&2
    exit 64
fi
template="$1"
output="$2"

# The IPv4 addresses on this host's interfaces, one per line.
host_addrs() {
    ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1
}

# True when $1 is an address some interface carries.
have_ip() {
    host_addrs | grep -qxF "$1"
}

lan2="${LAN2_IP:-}"
wifi="${WIFI_IP:-}"

# LAN2: required, a real address, present on an interface.
# The explicit 0.0.0.0 tests here and in the wifi chain are EQUIVALENT to the
# have_ip checks after them (no interface carries 0.0.0.0, so have_ip refuses
# it too); they exist for the message that names NET-C9. Removing them changes
# only the wording, which is why no test kills that mutant (CLAUDE.md 7.2.1).
if [[ -z "$lan2" || "$lan2" == "0.0.0.0" ]]; then
    echo "render_zenohd_gen: LAN2_IP is empty or a wildcard -- refusing (NET-C9, U92)" >&2
    exit 1
fi
if ! have_ip "$lan2"; then
    echo "render_zenohd_gen: LAN2_IP $lan2 is not on any interface -- refusing (U92)" >&2
    exit 1
fi

# wifi: optional. Decide whether its endpoint line stays.
skip_wifi=""
if [[ -z "$wifi" ]]; then
    skip_wifi="unset"
elif [[ "$wifi" == "0.0.0.0" ]]; then
    skip_wifi="a wildcard"
elif [[ "$wifi" == 127.* ]]; then
    skip_wifi="loopback (the template already binds 127.0.0.1)"
elif [[ "$wifi" == "$lan2" ]]; then
    skip_wifi="the same address as LAN2"
elif ! have_ip "$wifi"; then
    skip_wifi="not on any interface"
fi

# Full-line // comments are dropped from the output. *** Not cosmetic: the
# template's comments spell out the rule they enforce -- the literal "0.0.0.0"
# and "${WIFI_IP}" both appear in them -- and the unit's two guards grep the
# WHOLE output. Kept, those comments made the 0.0.0.0 guard fail on every start
# (measured 2026-10-26: the bare-envsubst output tripped it), so the formal GEN
# router could never have started. Without them the guards judge the endpoints
# zenohd will actually bind, which is what they are for.
mkdir -p "$(dirname "$output")"
body() {
    grep -vE '^[[:space:]]*//' "$template"
}
if [[ -n "$skip_wifi" ]]; then
    echo "render_zenohd_gen: wifi endpoint skipped: WIFI_IP='$wifi' is $skip_wifi (U92: wifi is optional)"
    # Drop the wifi endpoint line before substituting, so nothing of it remains.
    body | grep -vF '"tcp/${WIFI_IP}:7447"' \
        | LAN2_IP="$lan2" envsubst '${LAN2_IP}' > "$output"
    echo "render_zenohd_gen: wrote $output (LAN2 $lan2, no wifi)"
else
    body | LAN2_IP="$lan2" WIFI_IP="$wifi" envsubst '${LAN2_IP} ${WIFI_IP}' > "$output"
    echo "render_zenohd_gen: wrote $output (LAN2 $lan2, wifi $wifi)"
fi
