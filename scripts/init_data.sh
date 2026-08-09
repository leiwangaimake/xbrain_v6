#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: init_data.sh
# Brief: INF-DP-11 -- initialise the four SQLite databases xbrain uses
#
# Description:
# Creates data/task.db, data/fence.db, data/geo.db, data/record.db
# under the data root (default /opt/xbrain_v6/data). Each is a bare
# SQLite file with PRAGMA user_version set to the current expected
# version. This script is idempotent -- running it on a filesystem
# where the DBs already exist is a no-op that logs their current
# versions.
#
# Why four DBs, not one. 15 S9 pins the ownership split:
#   task.db    P3 task queue     -- writer: p3_task
#   fence.db   geo fences        -- writer: p3_task
#   geo.db     coords/waypoints  -- writer: p3_task
#   record.db  event/fault log   -- writer: p5_gateway
# A single DB would put two writers in different processes on the
# same file, which needs contention handling that four separate DBs
# do not.
#
# Why the schema_version integers match what CFG-BT-1's probe checks
# for. The probe reads PRAGMA user_version at Stage 0; if this script
# creates DBs at version N but the probe expects N+1, Stage 0 fails
# and the fleet does not boot. Keep the two in step.
#
# Usage:
#   bash scripts/init_data.sh                       # default paths
#   XBRAIN_DATA_ROOT=/tmp/tst bash scripts/init_data.sh  # test override

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DATA_ROOT="${XBRAIN_DATA_ROOT:-$REPO_ROOT/data}"

# Table of (name, schema_version). Keep in step with the databases
# array in configs/probe/thresholds.yaml -- see head comment.
DBS=(
    "task.db:1"
    "fence.db:1"
    "geo.db:1"
    "record.db:1"
)

mkdir -p "$DATA_ROOT"

if ! command -v sqlite3 >/dev/null 2>&1; then
    echo "ERROR: sqlite3 not installed; cannot bootstrap DBs" >&2
    exit 2
fi

for entry in "${DBS[@]}"; do
    name="${entry%%:*}"
    ver="${entry##*:}"
    path="$DATA_ROOT/$name"
    if [[ -f "$path" ]]; then
        actual=$(sqlite3 "$path" "PRAGMA user_version;")
        printf 'exists: %s (user_version=%s, expected=%s)\n' "$path" "$actual" "$ver"
        if [[ "$actual" != "$ver" ]]; then
            printf '  WARN: version mismatch -- migration MUST be run\n' >&2
        fi
        continue
    fi
    # Create as an empty SQLite file with the target user_version.
    # sqlite3 opens on first write; the PRAGMA is that write.
    sqlite3 "$path" "PRAGMA user_version = $ver;"
    # Sanity: read it back to verify we did not miswrite.
    got=$(sqlite3 "$path" "PRAGMA user_version;")
    if [[ "$got" != "$ver" ]]; then
        echo "ERROR: $name: wrote $ver, read back $got" >&2
        exit 3
    fi
    printf 'created: %s (user_version=%s)\n' "$path" "$ver"
done

printf 'init_data: done. data_root=%s\n' "$DATA_ROOT"
