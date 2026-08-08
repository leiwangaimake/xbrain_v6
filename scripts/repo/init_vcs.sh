#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: init_vcs.sh
# Brief: CHK-0-53 -- idempotent git-repo init + hygiene sanity for XBRAIN_V6
#
# Description:
# Runs the bring-up sequence CHK-0-53 asks for on a machine where the repo
# might already exist. Safe to re-run: every step is a no-op when the desired
# state is already true, so operators can invoke this from a deploy script
# without a "have we done this yet" check.
#
# What it does, and why each step exists:
#   1) git init          -- Phase 1 CHK-0-53 was written before the repo was
#                           tracked. It is now, so this is a no-op the first
#                           time and every subsequent time. Kept anyway: a
#                           machine that clones from a tar (no .git) still
#                           needs it, and the failure of git init on a real
#                           repo is a silent no-op.
#   2) .gitignore floor  -- verify the ignore file exists and covers the CHK-0-53
#                           paths (secrets, build/logs, model weights). We do
#                           not REWRITE the file; a missing rule surfaces from
#                           tests/repo/test_vcs_hygiene.py, whose diagnostic
#                           names the offending path and what should ignore it.
#   3) data/.gitkeep     -- INF-DP-11 needs data/ to be a tracked directory
#                           (deployment mkdirs `disk.data_root` under it).
#                           .gitkeep + README are the two files data/* -> !...
#                           lets through; touch them if missing.
#
# Not covered here (belongs elsewhere by design):
#   * git config user.email / user.name : per-operator, not per-repo.
#   * secrets-store setup: services/payload OWNS its own key material
#     (systemd LoadCredential); shell here would only race with it.
#
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"

log() { echo "[init_vcs] $*"; }

# --- 1. git init (idempotent) ------------------------------------------------
cd "${REPO_ROOT}"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  log "repo already tracked; git init skipped (idempotent)"
else
  log "initialising git repo at ${REPO_ROOT}"
  git init
fi

# --- 2. .gitignore floor (verify only; test file gives the diagnostic) ------
if [[ ! -f .gitignore ]]; then
  log "FATAL: .gitignore missing. Restore from source-control before proceeding."
  exit 1
fi

# --- 3. data/.gitkeep + README (INF-DP-11 requires tracked data/) ------------
mkdir -p data
if [[ ! -e data/.gitkeep ]]; then
  log "creating data/.gitkeep so data/ tracks as an empty directory"
  : > data/.gitkeep
fi
if [[ ! -e data/README.md ]]; then
  log "FATAL: data/README.md missing. This file is versioned and describes"
  log "       what deployment writes under data/. Do not proceed with an empty"
  log "       placeholder -- restore from source-control."
  exit 1
fi

log "OK: repo is initialised, .gitignore present, data/ tracked via .gitkeep + README"
