#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: rollback.sh
# Brief: CHK-0-54 -- switch /opt/xbrain_v6 back to the previous version
#
# Description:
# Rollback finds the previous version by state files, not by mtime:
#   INSTALL_ROOT/.previous  -- version that CURRENT pointed at before this run.
#   INSTALL_ROOT/.current   -- version that CURRENT points at now.
# install.sh's next incarnation writes these two files before flipping the
# symlink. rollback.sh reads .previous, refuses if it is empty or missing
# (the operator has to name a version explicitly in that case), and does the
# SAME atomic switch install.sh does -- rename(2) via a tmp symlink.
#
# NOTE Fallback: when the state files are absent (fresh machine, or the tests'
# tmp install root that install.sh has not touched yet), an explicit --to
# argument still works. This is what the criterion's install-A -> install-B
# -> rollback path exercises: two installs write .previous, rollback reads it.
#
# Usage:
#   rollback.sh           read .previous, flip to it
#   rollback.sh --to VER  flip to the named version (no state read)
#
# Env: same as install.sh (INSTALL_ROOT / CURRENT_LINK / SYSTEMCTL / units).
#
set -euo pipefail

INSTALL_ROOT="${XBRAIN_INSTALL_ROOT:-/opt/xbrain_v6.versions}"
CURRENT_LINK="${XBRAIN_CURRENT_LINK:-/opt/xbrain_v6}"
SYSTEMCTL="${SYSTEMCTL:-systemctl}"

log() { echo "[rollback] $*"; }

TARGET_VER=""
if [[ "${1:-}" == "--to" ]]; then
  TARGET_VER="${2:?--to requires a version}"
elif [[ -f "${INSTALL_ROOT}/.previous" ]]; then
  TARGET_VER="$(cat "${INSTALL_ROOT}/.previous")"
fi

if [[ -z "${TARGET_VER}" ]]; then
  log "no .previous state and no --to; refusing (operator must name a version)"
  exit 1
fi

TARGET="${INSTALL_ROOT}/${TARGET_VER}"
if [[ ! -d "${TARGET}" ]]; then
  log "target version does not exist: ${TARGET}"
  exit 1
fi

# --- atomic switch (mutation d guards this) ---------------------------------
# Identical dance to install.sh; kept explicit rather than factored so a
# rollback script that lost this loop still fails on the test.
PARENT="$(dirname "${CURRENT_LINK}")"
NAME="$(basename "${CURRENT_LINK}")"
TMP_LINK="${PARENT}/.${NAME}.new.$$"
ln -s "${TARGET}" "${TMP_LINK}"
log "atomic switch: mv -T ${TMP_LINK} ${CURRENT_LINK}"
mv -T "${TMP_LINK}" "${CURRENT_LINK}"

# Update state files so a second rollback reverts back to the version we
# just left (bounces between two known-good versions rather than sliding
# further backward, which is usually not what an operator wants).
NEW_PREV=""
if [[ -f "${INSTALL_ROOT}/.current" ]]; then
  NEW_PREV="$(cat "${INSTALL_ROOT}/.current")"
fi
echo "${TARGET_VER}" > "${INSTALL_ROOT}/.current"
if [[ -n "${NEW_PREV}" ]]; then
  echo "${NEW_PREV}" > "${INSTALL_ROOT}/.previous"
fi

log "systemctl daemon-reload via ${SYSTEMCTL}"
"${SYSTEMCTL}" daemon-reload
for unit in ${XBRAIN_RESTART_UNITS:-}; do
  log "systemctl restart ${unit}"
  "${SYSTEMCTL}" restart "${unit}"
done

log "OK: rolled back to ${TARGET_VER} (current -> ${TARGET})"
