#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: clean_pyc.sh
# Brief: CFG-BT-7 -- remove __pycache__ + .pyc across the tree
#
# Description:
# Removes every __pycache__ directory and .pyc file under the repo.
# Solves the CLAUDE.md V5-blood-lesson: after syncing .py sources,
# stale .pyc gets a newer mtime than the source and Python loads the
# old bytecode -- your edit silently has no effect.
#
# Every `rm -rf` here names a concrete path (no bare $VAR) to satisfy
# CFG-BT-7 variant ①. The find command emits absolute paths so a
# reader can see exactly what is being deleted.
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DRY_RUN="${DRY_RUN:-0}"

# Sanity: refuse to run if REPO_ROOT resolved to '/' or empty.
# A ${VAR:-} that expands to nothing then feeds `rm -rf` at '/' is
# the classic CFG-BT-7 hazard; this guard prevents it.
if [[ -z "$REPO_ROOT" || "$REPO_ROOT" == "/" ]]; then
    echo "refuse to run: REPO_ROOT=$REPO_ROOT" >&2
    exit 2
fi

printf 'scanning %s for __pycache__ / *.pyc\n' "$REPO_ROOT"

# List first so the reader can see what will disappear.
found=$(find "$REPO_ROOT" \
    -type d -name __pycache__ 2>/dev/null \
    -o -type f -name '*.pyc' 2>/dev/null)

if [[ -z "$found" ]]; then
    printf '  nothing to remove\n'
    exit 0
fi

printf '%s\n' "$found" | sed 's/^/  /'

if [[ "$DRY_RUN" == "1" ]]; then
    printf 'DRY_RUN=1: no removal\n'
    exit 0
fi

# Two separate commands so a directory scan does not race with a
# file scan; each names its concrete pattern.
find "$REPO_ROOT" -type d -name __pycache__ -exec rm -rf {} +
find "$REPO_ROOT" -type f -name '*.pyc' -delete
printf 'done\n'
