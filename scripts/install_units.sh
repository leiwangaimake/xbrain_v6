#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: install_units.sh
# Brief: Install XBRAIN v6 systemd units + /etc/xbrain env templates (no auto-enable)
#
# Description:
# The problem this solves: deploy/systemd/ holds the unit files but nothing installs
# them, and hand-copying 18 units + wiring /etc/xbrain by hand is where a staged-boot
# deployment silently drifts from 10 S3.3. This script is the one installer.
#
# What it installs: every deploy/systemd/*.service + run-xbrain.mount EXCEPT the three
# AI drafts (ai-asr, llm, payload). Those carry a "草稿, 不安装" header because DEC-15
# (unit naming / install root / Stage placement, owner = 主会话) plus two 11 doc
# backfills (S11A.2.3 ledger, the payload OOM row) are still open -- installing them
# here would pre-empt that decision. They are installed separately once DEC-15 lands.
#
# What it does NOT do: it does NOT `systemctl enable` anything. Enabling wires units into
# the boot target -- an outward, on-reboot change to the host -- and the final Stage
# ordering is part of DEC-15. Install (copy + daemon-reload) is reversible and safe;
# enabling is a deliberate, separate `--enable` the operator runs after DEC-15. Until
# then the units are dormant: present, verifiable, started only on explicit `systemctl
# start` or via scripts/start_all.sh.
#
# /etc/xbrain templates: robot.env / network.env are copied from deploy/etc-xbrain/ ONLY
# if absent -- a real host's identity + per-site IPs must never be clobbered by a reinstall.
# The templates fail SAFE: robot.env leaves XBRAIN_ROBOT_ID unset (rtk_driver then refuses
# to start rather than publish under a bogus rid), network.env binds 127.0.0.1 (loopback,
# not a wildcard) until real site IPs are written.
#
# Modes: (default) install; --dry-run print actions only; --enable also enable (opt-in);
# --uninstall remove installed units + daemon-reload (never touches /etc/xbrain).
#
# Boundary: this does not build the C++ ros2_ws binaries (7 units are ConditionPathExists-
# gated and skip cleanly until built) and does not calibrate configs/. It only places
# unit + env files. Running the stack is scripts/start_all.sh; dev-direct is
# scripts/dev/run_stack_dev.sh.

set -euo pipefail

# Derive paths from this script's location, never hard-code (CLAUDE.md 6).
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
UNIT_SRC_DIR="$REPO_ROOT/deploy/systemd"
ENV_SRC_DIR="$REPO_ROOT/deploy/etc-xbrain"
SYSTEMD_DIR="/etc/systemd/system"
ETC_XBRAIN_DIR="/etc/xbrain"

# The three AI-service units are drafts (see file headers + deploy/systemd/README.md).
# Excluded from install until DEC-15 + the 11 backfills are settled.
DRAFT_UNITS=(
  "xbrain-ai-asr.service"
  "xbrain-llm.service"
  "xbrain-payload.service"
)

# The two per-host env templates and their fail-safe rationale live in the source files.
ENV_TEMPLATES=("robot.env" "network.env")

usage() {
  cat <<'EOF'
Usage: sudo install_units.sh [--dry-run | --enable | --uninstall | --help]

  (no flag)    Install non-draft units + env templates, daemon-reload. Does NOT enable.
  --dry-run    Print exactly what would be installed/copied; change nothing.
  --enable     Install, then `systemctl enable` the installed units (opt-in, on-boot).
  --uninstall  Remove installed xbrain units + run-xbrain.mount, daemon-reload.
               Leaves /etc/xbrain untouched (that is host identity, not ours to delete).
  --help       This text.

The three AI drafts (ai-asr, llm, payload) are never installed by this script.
EOF
}

# True if $1 (a unit filename) is in the DRAFT_UNITS exclusion list.
is_draft() {
  local u="$1" d
  for d in "${DRAFT_UNITS[@]}"; do
    [[ "$u" == "$d" ]] && return 0
  done
  return 1
}

# Build the install list: every *.service in deploy/systemd minus drafts, plus the mount.
# Printed one per line so callers can iterate; sorted for stable output.
installable_units() {
  local f base
  for f in "$UNIT_SRC_DIR"/xbrain-*.service; do
    base="$( basename "$f" )"
    is_draft "$base" && continue
    printf '%s\n' "$base"
  done | sort
  printf '%s\n' "run-xbrain.mount"
}

require_root() {
  # A dry-run only reads, so it may run unprivileged; every other mode writes /etc.
  if [[ "$MODE" != "dry-run" && "${EUID}" -ne 0 ]]; then
    echo "install_units: must run as root for mode '$MODE' (use sudo)" >&2
    exit 1
  fi
}

# Copy one unit source -> /etc/systemd/system, or just announce it under --dry-run.
install_one_unit() {
  local base="$1" src="$UNIT_SRC_DIR/$1" dst="$SYSTEMD_DIR/$1"
  if [[ ! -f "$src" ]]; then
    echo "install_units: source unit missing: $src" >&2
    exit 1
  fi
  if [[ "$MODE" == "dry-run" ]]; then
    echo "  would install $base -> $dst"
    return 0
  fi
  install -m 0644 "$src" "$dst"
  echo "  installed $base"
}

# Copy an env template into /etc/xbrain ONLY if the destination is absent, so a reinstall
# never overwrites a host's real robot.env / network.env.
install_env_template() {
  local name="$1" src="$ENV_SRC_DIR/$1" dst="$ETC_XBRAIN_DIR/$1"
  if [[ ! -f "$src" ]]; then
    echo "install_units: env template missing: $src" >&2
    exit 1
  fi
  if [[ -e "$dst" ]]; then
    echo "  kept existing $dst (not overwritten)"
    return 0
  fi
  if [[ "$MODE" == "dry-run" ]]; then
    echo "  would create $dst from template (fill in before enabling)"
    return 0
  fi
  install -d -m 0755 "$ETC_XBRAIN_DIR"
  # robot.env can hold a secret-ish identity; 0644 is fine (no credentials), but keep it
  # owner-writable only. network.env is non-sensitive. Both 0644.
  install -m 0644 "$src" "$dst"
  echo "  created $dst from template -- EDIT before enabling"
}

do_install() {
  local units base
  mapfile -t units < <( installable_units )

  echo "Installing ${#units[@]} unit(s) to $SYSTEMD_DIR (mode: $MODE):"
  for base in "${units[@]}"; do
    install_one_unit "$base"
  done

  echo "Env templates in $ETC_XBRAIN_DIR:"
  for base in "${ENV_TEMPLATES[@]}"; do
    install_env_template "$base"
  done

  if [[ "$MODE" == "dry-run" ]]; then
    echo "(dry-run) would run: systemctl daemon-reload"
    return 0
  fi
  systemctl daemon-reload
  echo "daemon-reload done."

  if [[ "$MODE" == "enable" ]]; then
    echo "Enabling installed units (on-boot):"
    for base in "${units[@]}"; do
      # .mount and .service both enable; a unit with [Install] WantedBy gets a wants-symlink.
      systemctl enable "$base" >/dev/null 2>&1 && echo "  enabled $base" \
        || echo "  skipped $base (no [Install] or already enabled)"
    done
  else
    echo
    echo "Units are INSTALLED but NOT enabled (dormant). Next steps:"
    echo "  1. Edit $ETC_XBRAIN_DIR/robot.env  (set XBRAIN_ROBOT_ID)"
    echo "  2. Edit $ETC_XBRAIN_DIR/network.env (set LAN2_IP / WIFI_IP for the GEN router)"
    echo "  3. Start the stack for a run:   sudo $REPO_ROOT/scripts/start_all.sh"
    echo "  4. To enable on boot (after DEC-15): sudo $0 --enable"
  fi
}

do_uninstall() {
  local units base dst removed=0
  echo "Uninstalling xbrain units from $SYSTEMD_DIR:"
  # Remove ONLY units this script installs (the installable set), by exact name -- NEVER
  # a xbrain-*.service glob: the host may carry unrelated xbrain-* units (e.g. a vendor
  # xbrain-maxfan.service) that a glob would wrongly disable+delete. Explicit names only.
  mapfile -t units < <( installable_units )
  for base in "${units[@]}"; do
    dst="$SYSTEMD_DIR/$base"
    [[ -e "$dst" ]] || continue
    systemctl disable "$base" >/dev/null 2>&1 || true
    rm -f "$dst"
    echo "  removed $dst"
    removed=$(( removed + 1 ))
  done
  systemctl daemon-reload
  echo "Removed $removed file(s); daemon-reload done. /etc/xbrain left untouched."
}

# ---- arg parse: exactly one optional mode flag ----
MODE="install"
case "${1:-}" in
  "")            MODE="install" ;;
  --dry-run)     MODE="dry-run" ;;
  --enable)      MODE="enable" ;;
  --uninstall)   MODE="uninstall" ;;
  --help|-h)     usage; exit 0 ;;
  *)             echo "install_units: unknown option '$1'" >&2; usage; exit 2 ;;
esac

require_root
if [[ "$MODE" == "uninstall" ]]; then
  do_uninstall
else
  do_install
fi
