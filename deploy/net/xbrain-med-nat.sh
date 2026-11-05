#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: xbrain-med-nat.sh
# Brief: MED-1 media-plane NAT for the PTZ cameras (apply / remove / status)
#
# Description:
# What this solves. The operator's Qt client cannot reach the PTZ cameras on
# 192.168.66.0/24 -- they sit behind the chassis switch on lan0. 10 NAT-1 maps
# them out through this host. Measured 2026-10-24 on .1.26 (four streams pulled
# from a workstation through it): a DNAT rule ALONE does not work. Three more
# pieces are required, and this script owns all of them:
#   * SNAT -- the cameras' default gateway is the PTZ ball's own router
#     (192.168.66.1), not this host. Without rewriting the source, the replies
#     go to that router and are lost.
#   * FORWARD accept -- the FORWARD policy is DROP (DEP-6, set by docker here).
#     Only the DNAT-ed flows and their replies are accepted; nothing else is
#     opened between the operator segment and the chassis segment.
#   * the client must use RTSP over TCP -- UDP RTP is sent by the camera to a
#     random port on this host that conntrack does not track. This is a client
#     setting; it is recorded here because it is part of the same contract.
# The absolute a=control:rtsp://192.168.66.13/... lines in the camera SDP do NOT
# break this: ffmpeg and live555 send SETUP/PLAY over the same control
# connection, so those URLs are only request-line text.
#
# Why own chains. Every rule lives in XBRAIN-MED-PRE / -POST (nat) and
# XBRAIN-MED-FWD (filter), entered by one jump each. apply flushes and refills
# them, so running it twice changes nothing; remove deletes the jumps and the
# chains and leaves every other rule on the host alone -- docker's in particular.
# The chains do not depend on docker's DOCKER-USER, which only exists while
# docker is installed and running.
#
# Inputs come from an env file (default /etc/xbrain/med-nat.env, template in
# deploy/net/med-nat.env). Every key is required; a missing one is a hard stop,
# never a guessed default (CLAUDE.md 3.1 spirit: a wrong NAT target silently
# shows a different camera, or none).
#
# Usage: xbrain-med-nat.sh apply|remove|status [--dry-run]
#   --dry-run prints the iptables commands instead of running them (no root).

set -Eeuo pipefail   # -E: the ERR rollback trap must fire inside functions too

ENV_FILE="${XBRAIN_MED_NAT_ENV:-/etc/xbrain/med-nat.env}"
ACTION="${1:-}"
DRY=0
if [[ "${2:-}" == "--dry-run" ]]; then DRY=1; fi

PRE=XBRAIN-MED-PRE
POST=XBRAIN-MED-POST
FWD=XBRAIN-MED-FWD
# MED-C1: only the registered media ports may be DNAT-ed (scripts/ci/check_med.sh).
ALLOWED_EXT_PORTS=" 554 8554 "

ipt() {
  if [[ "$DRY" == "1" ]]; then
    echo "iptables $*"
  else
    iptables "$@"
  fi
}

die() {
  echo "xbrain-med-nat: $*" >&2
  exit 1
}

load_env() {
  [[ -f "$ENV_FILE" ]] || die "env file $ENV_FILE not found"
  # shellcheck disable=SC1090
  . "$ENV_FILE"
  local key
  for key in EXT_IF EXT_IP INT_IF SNAT_IP ALLOW_SRC MAPS; do
    [[ -n "${!key:-}" ]] || die "$key is not set in $ENV_FILE"
  done
  local src
  for src in $ALLOW_SRC; do
    # SEC-2 / MED-C2: a wildcard source opens the cameras to every segment.
    case "$src" in
      0.0.0.0|0.0.0.0/0|*/0) die "ALLOW_SRC entry $src is a wildcard (SEC-2)" ;;
    esac
  done
  local map ext
  for map in $MAPS; do
    ext="${map%%=*}"
    [[ "$ALLOWED_EXT_PORTS" == *" $ext "* ]] \
      || die "MAPS entry $map: external port $ext is not a media port (MED-C1)"
    [[ "${map#*=}" == *:* ]] || die "MAPS entry $map: target must be host:port"
  done
}

# Insert a jump at the head of a built-in chain once; -C makes it idempotent.
ensure_jump() {
  local table="$1" chain="$2" target="$3"
  if [[ "$DRY" == "1" ]] || ! iptables -t "$table" -C "$chain" -j "$target" 2>/dev/null; then
    ipt -t "$table" -I "$chain" 1 -j "$target"
  fi
}

ensure_chain() {
  local table="$1" chain="$2"
  if [[ "$DRY" == "1" ]] || ! iptables -t "$table" -nL "$chain" >/dev/null 2>&1; then
    ipt -t "$table" -N "$chain"
  fi
  ipt -t "$table" -F "$chain"
}

# The rules added by hand during bring-up carried comments xbrain-med-*. Left in
# place they would duplicate the chains' rules, so apply removes them once.
remove_bringup_rules() {
  local table chain line
  for table_chain in nat:PREROUTING nat:POSTROUTING filter:DOCKER-USER filter:FORWARD; do
    table="${table_chain%%:*}"
    chain="${table_chain#*:}"
    iptables -t "$table" -nL "$chain" >/dev/null 2>&1 || continue
    while read -r line; do
      [[ -n "$line" ]] || continue
      # shellcheck disable=SC2086
      ipt -t "$table" ${line/-A /-D }
    done < <(iptables -t "$table" -S "$chain" | grep -- '--comment xbrain-med-' || true)
  done
}

# The FORWARD policy must already be DROP (DEP-6). On this host docker sets it;
# without docker (not installed / failed to start) it is ACCEPT, and turning
# ip_forward on would make this box a router between wifi and the chassis
# segment. Refuse instead of opening more than the two media flows (2026-10-24
# review).
require_forward_drop() {
  local policy
  policy="$(iptables -S FORWARD 2>/dev/null | awk '$1=="-P"{print $3}')"
  [[ "$policy" == "DROP" ]] \
    || die "FORWARD policy is '${policy:-unknown}', not DROP (DEP-6); refusing to enable forwarding"
}

# A failure half-way through apply would leave the jumps and part of the rules
# in place (oneshot start failure runs no ExecStop). Roll back to nothing.
rollback_on_error() {
  echo "xbrain-med-nat: apply failed, rolling back" >&2
  trap - ERR
  remove >/dev/null 2>&1 || true
}

apply() {
  load_env
  if [[ "$DRY" != "1" ]]; then
    require_forward_drop
    trap rollback_on_error ERR
  fi
  if [[ "$(cat /proc/sys/net/ipv4/ip_forward)" != "1" ]]; then
    if [[ "$DRY" == "1" ]]; then echo "sysctl -w net.ipv4.ip_forward=1"
    else sysctl -q -w net.ipv4.ip_forward=1; fi
  fi
  [[ "$DRY" == "1" ]] || remove_bringup_rules
  ensure_chain nat "$PRE"
  ensure_chain nat "$POST"
  ensure_chain filter "$FWD"
  ensure_jump nat PREROUTING "$PRE"
  ensure_jump nat POSTROUTING "$POST"
  ensure_jump filter FORWARD "$FWD"

  local map ext target host port src
  local -A seen=()
  for map in $MAPS; do
    ext="${map%%=*}"
    target="${map#*=}"
    host="${target%:*}"
    port="${target##*:}"
    for src in $ALLOW_SRC; do
      ipt -t nat -A "$PRE" -i "$EXT_IF" -s "$src" -d "$EXT_IP" -p tcp --dport "$ext" \
        -j DNAT --to-destination "$target"
      ipt -t filter -A "$FWD" -i "$EXT_IF" -o "$INT_IF" -s "$src" -d "$host" -p tcp \
        --dport "$port" -m conntrack --ctstate DNAT -j ACCEPT
    done
    if [[ -z "${seen[$target]:-}" ]]; then
      seen[$target]=1
      ipt -t nat -A "$POST" -o "$INT_IF" -d "$host" -p tcp --dport "$port" \
        -j SNAT --to-source "$SNAT_IP"
      ipt -t filter -A "$FWD" -i "$INT_IF" -o "$EXT_IF" -s "$host" \
        -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
    fi
  done
  trap - ERR
  echo "xbrain-med-nat: applied ($(wc -w <<<"$MAPS") map(s), sources: $ALLOW_SRC)"
}

remove() {
  local table chain target
  for spec in nat:PREROUTING:$PRE nat:POSTROUTING:$POST filter:FORWARD:$FWD; do
    IFS=: read -r table chain target <<<"$spec"
    if [[ "$DRY" == "1" ]]; then
      # ipt only prints under --dry-run, so the -C loop below would never end.
      ipt -t "$table" -D "$chain" -j "$target"
    else
      while iptables -t "$table" -C "$chain" -j "$target" 2>/dev/null; do
        ipt -t "$table" -D "$chain" -j "$target"
      done
    fi
    if [[ "$DRY" == "1" ]] || iptables -t "$table" -nL "$target" >/dev/null 2>&1; then
      ipt -t "$table" -F "$target"
      ipt -t "$table" -X "$target"
    fi
  done
  # ip_forward is left as found: docker needs it on, and turning it off here
  # would break containers that have nothing to do with the media plane.
  echo "xbrain-med-nat: removed"
}

status() {
  local t c
  for tc in nat:$PRE nat:$POST filter:$FWD; do
    t="${tc%%:*}"
    c="${tc#*:}"
    echo "== $t $c"
    iptables -t "$t" -nvL "$c" 2>/dev/null || echo "   (absent)"
  done
  echo "== ip_forward=$(cat /proc/sys/net/ipv4/ip_forward)"
}

case "$ACTION" in
  apply) apply ;;
  remove) remove ;;
  status) status ;;
  *) echo "usage: $0 apply|remove|status [--dry-run]" >&2; exit 64 ;;
esac
