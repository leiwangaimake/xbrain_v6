#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: collect_bundle.sh
# Brief: CHK-2-63 -- wrapper for the Python support-bundle collector
#
# Description:
# Thin wrapper. The real work lives in xbrain/boot/diag/. This script
# exists so on-call can run one command; it forwards any --flags to the
# python entry-point unchanged.
#
# Usage:
#   bash scripts/diag/collect_bundle.sh
#   bash scripts/diag/collect_bundle.sh --max-bundle-bytes 268435456
#   XBRAIN_ROBOT_ID=m20s-12 bash scripts/diag/collect_bundle.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Ensure the repo is importable when running out of the source tree
# (deploys install the package via pip; devs run it in place).
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m xbrain.boot.diag "$@"
