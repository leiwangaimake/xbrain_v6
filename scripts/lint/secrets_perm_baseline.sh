#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: secrets_perm_baseline.sh
# Brief: Verify configs/secrets/** against the assertion J directory-glob baseline
#
# Description:
# What problem this solves. 10 S5.4.4 assertion J item 3 (grep anchor:
# 凭据目录判据写成目录通配) fixes the permission baseline for the credential tree as a
# DIRECTORY GLOB, not a per-file name list. Before that ruling the chassis TLS
# material under configs/secrets/chassis_tls/ (client.key and friends, 13 TLS-3)
# was checked by nothing: assertion J did not name it, SEC-11 item 3 did not name
# it, and 13's own QC-15 sat among assertions with no executor, so a private key
# could ship world-readable at 0644 and no gate would go red. This script is the
# executable form of that glob, runnable standalone at deploy time and reusable by
# the freeze service (CFG-FZ-2) as the item-3 half of assertion J.
#
# Which criterion, verbatim (grep anchor: 甲-03, quoted so it is implementable as
# written): find <root> -type f ! -perm 600 must be zero-hit AND find <root>
# -type d ! -perm 700 must be zero-hit AND every entry is owned by the run user
# AND non-root cannot write. "Non-root cannot write" is ENTAILED by the two
# exact-perm checks, not a fourth scan: any group or other bit, write included,
# makes a file not exactly 0600 or a directory not exactly 0700, so it is already
# caught. Keeping it verbatim is a design order, grep anchor: 严禁改回逐个点名文件.
#
# What it deliberately does NOT do, so a green run is not read as more than it is:
#   * it does not create, read or move any credential. 99 U70/U77 put the
#     credential VALUE outside git (systemd LoadCredential, /etc/credstore); this
#     inspects permission bits only and never opens a secret.
#   * it does not stat the onvif plaintext file. 11 S7.4.9 / 99 U70 deleted it and
#     assertion J no longer names it; reviving a check for it here would undo that.
#   * it does not check file EXISTENCE. Whether a required credential is present is
#     a deploy concern; an empty tree at 0700 is a correct baseline (zero files to
#     be insecure) and reports PASS, rather than inventing a requirement the design
#     does not state.
#   * it is NOT the whole of assertion J. Root-is-not-a-symlink, the required-file
#     list, path escape and the obsolete-file blacklist are the other four items,
#     evaluated by the freeze service; this is only item 3's secrets glob.
#
# A trap worth naming. Verbatim ! -perm 600 also flags a STRICTER file (0400,
# read-only). That is intended: the criterion is exactly 0600, and a tool that
# quietly accepted 0400 would be a second, looser criterion nobody agreed to.
#
# Usage:
#   secrets_perm_baseline.sh [SECRETS_ROOT] [EXPECTED_OWNER]
#     SECRETS_ROOT    default: <repo>/configs/secrets
#     EXPECTED_OWNER  default: the invoking user (id -un)

set -euo pipefail

# Derive the repository root from this script's own location rather than hardcoding
# an absolute path (CLAUDE.md 6). The default secrets root is <repo>/configs/secrets,
# which resolves to the same path the design names but survives the still-open
# build-layout decision (deploy/systemd/README.md, DEC-15).
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"
DEFAULT_SECRETS_ROOT="${REPO_ROOT}/configs/secrets"

# Arg 1: the secrets root to check. Defaulted so deploy runs it with no arguments,
# while the test points it at a temporary tree it owns.
SECRETS_ROOT="${1:-${DEFAULT_SECRETS_ROOT}}"

# Arg 2: the user every entry must belong to, the "run user" of the criterion.
# Defaulted to the invoking user: at deploy the operator runs this AS the run user,
# and the test passes an explicit value to exercise the owner branch without having
# to chown across accounts. This is a deployment identity, not a safety parameter,
# so deriving it here is not the CLAUDE.md 3.1 "invent a value" trap.
EXPECTED_OWNER="${2:-$( id -un )}"

# An absent tree has nothing to verify: the two globs are vacuously zero-hit on it,
# and file presence is a deploy concern this tool does not own (see header). Report
# and pass, so the check is safe to wire in before any credential is provisioned.
if [ ! -e "${SECRETS_ROOT}" ]; then
  echo "secrets_perm_baseline: NOTE ${SECRETS_ROOT} does not exist; no credential material to verify (PASS)"
  exit 0
fi

# A non-directory at the root is a real misconfiguration, not an empty baseline.
if [ ! -d "${SECRETS_ROOT}" ]; then
  echo "secrets_perm_baseline: FAIL ${SECRETS_ROOT} exists but is not a directory"
  exit 1
fi

# Resolve to an absolute physical path so every violation line prints an ABSOLUTE
# path (assertion J: 必须打印绝对路径), independent of the caller's cwd or of a
# symlink in the argument.
SECRETS_ROOT="$( cd "${SECRETS_ROOT}" && pwd -P )"

echo "secrets_perm_baseline: root=${SECRETS_ROOT} expected_owner=${EXPECTED_OWNER}"
echo "  criterion 10 S5.4.4 assertion J item 3: files 0600, dirs 0700, owner is the run user"

# One counter across all three checks. A file can violate more than one at once
# (a 0644 file owned by the wrong user hits checks 1 and 3); that prints two lines,
# which is honest: both facts are true and both need fixing.
violations=0

# Check 1 -- files not exactly 0600. Verbatim: find -type f ! -perm 600.
# -print0 plus read -d '' so a path with spaces or newlines cannot split a line
# and hide a violation.
while IFS= read -r -d '' path; do
  echo "  BAD  file not 0600: ${path}"
  violations=$(( violations + 1 ))
done < <( find "${SECRETS_ROOT}" -type f ! -perm 600 -print0 )

# Check 2 -- directories not exactly 0700. Verbatim: find -type d ! -perm 700.
# This visits the root itself, so a 0755 secrets/ is caught here rather than being
# assumed correct.
while IFS= read -r -d '' path; do
  echo "  BAD  dir not 0700:  ${path}"
  violations=$(( violations + 1 ))
done < <( find "${SECRETS_ROOT}" -type d ! -perm 700 -print0 )

# Check 3 -- entries not owned by the run user, the third conjunct of the criterion.
while IFS= read -r -d '' path; do
  echo "  BAD  owner not ${EXPECTED_OWNER}: ${path}"
  violations=$(( violations + 1 ))
done < <( find "${SECRETS_ROOT}" ! -user "${EXPECTED_OWNER}" -print0 )

# Fail closed: any violation means the credential tree is not at the baseline, and
# the freeze service maps this to E_CONFIG_INVALID / detail.kind=config_perm_bad.
# That mapping is the gateway/freeze's job (11 S8.13.5), not this tool's, so no
# error code is spelled here -- only the human-readable facts and the exit status.
if [ "${violations}" -ne 0 ]; then
  echo "secrets_perm_baseline: FAIL ${violations} violation(s); credential tree is not at the assertion J baseline"
  exit 1
fi

echo "secrets_perm_baseline: PASS"
exit 0
