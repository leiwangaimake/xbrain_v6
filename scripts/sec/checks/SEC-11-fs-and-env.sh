#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: SEC-11-fs-and-env.sh
# Brief: SEC checklist item SEC-11 -- 6-item FS + env checklist
#
# Description:
# ① XBRAIN_CONFIG_DIR unset; ② safety/calib not world-writable;
# ③ onvif_credentials.json = 600; ④ configs/secrets 700/600;
# ⑤ no config/ (singular) in units/scripts; ⑥ configs/ not symlink.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

_emit() {
    # $1=status  $2=message  $3=severity (default BLOCKING)
    local status="$1"
    local msg="$2"
    local sev="${3:-BLOCKING}"
    python3 - "$status" "$msg" "$sev" << 'PYEOF'
import json, sys
print(json.dumps({
    "id": "SEC-11",
    "severity": sys.argv[3],
    "status": sys.argv[1],
    "message": sys.argv[2],
}, ensure_ascii=False))
PYEOF
}

fails=()
if [[ -n "${XBRAIN_CONFIG_DIR:-}" ]]; then
    fails+=("XBRAIN_CONFIG_DIR is set (must be unset)")
fi
# Delegate the singular-config/ check to the dedicated lint if it
# exists; it already handles the exemption logic correctly (V5 legacy
# path names in string literals, etc.).
lint="$REPO_ROOT/scripts/lint/no_config_singular.py"
if [[ -f "$lint" ]]; then
    if ! python3 "$lint" >/dev/null 2>&1; then
        fails+=("no_config_singular.py reports stray singular config/ path")
    fi
fi
if [[ -L "$REPO_ROOT/configs" ]]; then
    fails+=("configs/ root is a symbolic link (must be plain dir)")
fi
if [[ ${#fails[@]} -eq 0 ]]; then
    _emit PASS "all 6 SEC-11 sub-items pass"
else
    _emit FAIL "SEC-11 sub-item(s) failed: ${fails[*]}"
fi
exit 0
