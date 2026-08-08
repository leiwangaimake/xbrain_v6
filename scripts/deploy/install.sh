#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: install.sh
# Brief: CHK-0-54 -- install a new build to /opt/xbrain_v6.versions/{ver}/
#        and atomically switch /opt/xbrain_v6 -> that version
#
# Description:
# Install layout, per CHK-0-54:
#   /opt/xbrain_v6.versions/
#     b377cfb/       <- versioned root, immutable after install
#     a1b2c3d/
#   /opt/xbrain_v6   <- SYMLINK to one of them (the current version)
#
# Two operations any install script MUST make atomic (else a partial upgrade
# strands the robot):
#   1) copy the version's tree in place BEFORE flipping the symlink; readers
#      of /opt/xbrain_v6 must never see a half-populated directory.
#   2) flip the symlink with `ln -sfn NEW _tmp && mv -T _tmp CURRENT`, so a
#      concurrent reader always sees either the old target or the new one,
#      never a missing path (CHK-0-54 ② is a 1000-reader stress on exactly
#      this). Naive `rm CURRENT && ln -s NEW CURRENT` is NOT atomic and is
#      the mutation (a) the test asserts red.
#
# Post-install:
#   * strip __pycache__ so the new version cannot inherit stale bytecode
#     from a build machine (CHK-0-54 ③ / mutation b).
#   * systemctl daemon-reload + restart the units the deploy config lists
#     (SYSTEMCTL="fake" env var lets tests inject a recording stub).
#
# Usage: install.sh VERSION SOURCE_DIR
#   VERSION    e.g. `git describe --always --dirty`
#   SOURCE_DIR the built tree (typically the CI staging area)
#
# Env:
#   XBRAIN_INSTALL_ROOT   default /opt/xbrain_v6.versions
#   XBRAIN_CURRENT_LINK   default /opt/xbrain_v6
#   SYSTEMCTL             command to run for reload/restart (default: systemctl)
#   XBRAIN_RESTART_UNITS  space-separated list of units to restart after switch
#                         (default: empty; deploy config supplies real list)
#
set -euo pipefail

VERSION="${1:?usage: install.sh VERSION SOURCE_DIR}"
SOURCE_DIR="${2:?usage: install.sh VERSION SOURCE_DIR}"

INSTALL_ROOT="${XBRAIN_INSTALL_ROOT:-/opt/xbrain_v6.versions}"
CURRENT_LINK="${XBRAIN_CURRENT_LINK:-/opt/xbrain_v6}"
SYSTEMCTL="${SYSTEMCTL:-systemctl}"

log() { echo "[install] $*"; }

# --- 1. copy source into a fresh versioned root ------------------------------
TARGET="${INSTALL_ROOT}/${VERSION}"
mkdir -p "${INSTALL_ROOT}"
if [[ -e "${TARGET}" ]]; then
  log "target exists, refusing to overwrite: ${TARGET}"
  log "(use rollback.sh to switch to an existing version, or delete first)"
  exit 1
fi
log "copying ${SOURCE_DIR}/ -> ${TARGET}/"
# -a preserves permissions/links; -T so cp does not turn TARGET into a
# subdir on a second run. --no-target-directory equivalent.
cp -a "${SOURCE_DIR}/" "${TARGET}"

# --- 2. scrub __pycache__ (mutation b guards this) --------------------------
log "stripping __pycache__ under ${TARGET}"
find "${TARGET}" -type d -name __pycache__ -exec rm -rf {} +

# --- 3. atomic symlink switch (mutation a guards this) ----------------------
# ln -sfn is documented but implementation-dependent for atomicity. The
# portable form is: create a NEW temporary symlink beside the current one,
# then `mv -T` (rename(2)) it into place -- rename() on the SAME filesystem
# is atomic per POSIX. Do the whole dance in the PARENT of CURRENT_LINK so
# both paths are on one filesystem.
PARENT="$(dirname "${CURRENT_LINK}")"
NAME="$(basename "${CURRENT_LINK}")"
TMP_LINK="${PARENT}/.${NAME}.new.$$"
# Point the tmp link at the versioned root. Absolute target so the symlink
# resolves the same from any cwd.
ln -s "${TARGET}" "${TMP_LINK}"

# Record who was current BEFORE the switch (so rollback.sh has a target).
# Written BEFORE the switch so a crash between them still leaves consistent
# state -- .current reflects the OLD version until the switch actually lands.
PREV_VER=""
if [[ -f "${INSTALL_ROOT}/.current" ]]; then
  PREV_VER="$(cat "${INSTALL_ROOT}/.current")"
fi

log "atomic switch: mv -T ${TMP_LINK} ${CURRENT_LINK}"
mv -T "${TMP_LINK}" "${CURRENT_LINK}"

# After the switch: update .current to the version now live, and .previous
# to whoever was live before (empty on first install).
if [[ -n "${PREV_VER}" && "${PREV_VER}" != "${VERSION}" ]]; then
  echo "${PREV_VER}" > "${INSTALL_ROOT}/.previous"
fi
echo "${VERSION}" > "${INSTALL_ROOT}/.current"

# --- 4. systemctl daemon-reload + restart -----------------------------------
log "systemctl daemon-reload via ${SYSTEMCTL}"
"${SYSTEMCTL}" daemon-reload

for unit in ${XBRAIN_RESTART_UNITS:-}; do
  log "systemctl restart ${unit}"
  "${SYSTEMCTL}" restart "${unit}"
done

log "OK: current -> ${TARGET}"
