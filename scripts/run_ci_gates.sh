#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_ci_gates.sh
# Brief: Execution body for CLAUDE.md 8.2 gates 1/2/3 (ruff, mypy, clang-format, clang-tidy)
#
# Description:
# What this solves. CLAUDE.md 8.2 lists five PR gates. Gates 1 (ruff/flake8),
# 2 (mypy strict) and 3 (clang-tidy + clang-format) were written down and then
# never given anything that runs them: on 2026-09-27 an audit on the machine
# that actually runs the tests (ORIN) found all four binaries ABSENT. So for
# the whole life of the project the sentence "the PR gates pass" was true and
# meaningless for three of the five -- CLAUDE.md 3.2 form 1 (an assertion an
# empty implementation also satisfies), at the scale of a whole gate rather
# than a single test. docs/NEXT.md SW-27 registers that finding; this file is
# the execution body it was missing.
#
# What it does NOT do:
#   * it does not fix anything. There is no --fix here on purpose. The
#     baseline is four-digit in three of the four tools, and a runner that
#     can rewrite the tree is a runner somebody will point at the tree at
#     2am. Formatting and typing debt is paid down in its own reviewed
#     batches.
#   * it does not build. clang-tidy needs a compile database; this script
#     uses the ones already in the package build trees and reports a package
#     with no database as a SKIP, never as a pass. Run
#     scripts/build_quadruped.sh (or a plain cmake configure with
#     -DCMAKE_EXPORT_COMPILE_COMMANDS=ON) first.
#   * it does not run pytest or the C++ binaries. Those are gate 4 and live
#     in scripts/ci/run_all.sh and scripts/run_cxx_tests.sh.
#
# Three properties that make it usable as a gate (CLAUDE.md 8.1):
#   * exit code is non-zero if any tool reports a finding, so it chains
#     with && in front of git commit;
#   * SKIP IS NEVER SPELLED AS PASS. A tool that is not installed, or a C++
#     package with no compile database, is reported as SKIP and the script
#     still exits non-zero unless --allow-skip says that is acceptable on
#     this host. Same discipline as scripts/run_cxx_tests.sh, and for the
#     same reason: this whole script exists because four missing binaries
#     read as "gates pass" for months;
#   * THE SCAN SURFACE IS PRINTED BEFORE THE RUN. A count without its scan
#     surface is a number nobody can interpret (CLAUDE.md 3.2 form 6), and
#     the surface here is not obvious -- ros2_ws/perception is 43 MB of code
#     that is not ours to lint.
#
# Where the surface is declared: the Python half is pyproject.toml
# ([tool.ruff].exclude and [tool.mypy].exclude, kept in step with each
# other); the C++ half is the CXX_EXCLUDE_RE below plus each package's own
# compile database. There is deliberately no third copy of the list.
#
# A trap worth naming. clang-tidy is driven by .clang-tidy at the repo root,
# whose readability-identifier-naming.FunctionCase is lower_case while the
# C++ actually written here uses PascalCase function names (Google style,
# CLAUDE.md 5.1). That one option produces the large majority of the
# clang-tidy baseline. Do NOT "fix" it by relaxing the option to make the
# number small -- which of the two CLAUDE.md rules wins for C++ function
# names is a ruling, not a defect (CLAUDE.md 9.1), and it is registered in
# docs/NEXT.md SW-27 waiting for one.
#
# Usage:
#   scripts/run_ci_gates.sh                  run all four, strict
#   scripts/run_ci_gates.sh --allow-skip     tolerate missing tools
#   scripts/run_ci_gates.sh ruff mypy        run only these
#   scripts/run_ci_gates.sh --list           print the table and exit
set -euo pipefail

# Derive the repo root from this script's own location; never hard-code it
# (CLAUDE.md 6) -- a hard-coded root lints the wrong checkout on a machine
# that has two.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# pip --user installs land here and are NOT on a non-login shell's PATH, which
# is how these four can be installed and still read as missing from a script.
export PATH="$HOME/.local/bin:$PATH"

# The four gates, in the order CLAUDE.md 8.2 lists them.
ALL_GATES=(ruff mypy clang-format clang-tidy)

# C++ packages whose sources this repo owns. ros2_ws/perception is absent by
# ruling, not by oversight (docs/NEXT.md 2, 2026-09-27: delivered by the
# perception vendor, untracked, "不审查").
CXX_PACKAGES=(quadruped chassis_relay sensor)
# Files under common/ are reached through the packages' compile databases (they
# are included by those translation units and .clang-tidy's HeaderFilterRegex
# covers them), so common/ needs no entry of its own here.
CXX_EXCLUDE_RE='(^|/)(ros2_ws/perception|common/third_party)/'

allow_skip=0
list_only=0
selected=()
for arg in "$@"; do
  case "$arg" in
    --allow-skip) allow_skip=1 ;;
    --list)       list_only=1 ;;
    -h|--help)    sed -n '2,70p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*)           echo "unknown option: $arg (try --help)" >&2; exit 64 ;;
    *)            selected+=("$arg") ;;
  esac
done
if [ "${#selected[@]}" -eq 0 ]; then
  selected=("${ALL_GATES[@]}")
fi
for g in "${selected[@]}"; do
  found=0
  for known in "${ALL_GATES[@]}"; do
    [ "$g" = "$known" ] && found=1
  done
  if [ "$found" = "0" ]; then
    echo "unknown gate: $g (known: ${ALL_GATES[*]})" >&2
    exit 64
  fi
done

if [ "$list_only" = "1" ]; then
  printf 'gates:      %s\n' "${ALL_GATES[*]}"
  printf 'cxx pkgs:   %s\n' "${CXX_PACKAGES[*]}"
  printf 'cxx skip:   %s\n' "$CXX_EXCLUDE_RE"
  printf 'py surface: pyproject.toml [tool.ruff].exclude / [tool.mypy].exclude\n'
  exit 0
fi

# Counters. n_skip is tracked separately from n_fail so the summary can say
# WHY the exit code is what it is -- "3 failed" and "3 not run" call for
# completely different next actions.
n_pass=0
n_fail=0
n_skip=0
declare -a SUMMARY=()

note_pass() { n_pass=$((n_pass + 1)); SUMMARY+=("PASS $1"); printf 'PASS %s\n' "$1"; }
note_fail() { n_fail=$((n_fail + 1)); SUMMARY+=("FAIL $1 -- $2"); printf 'FAIL %s -- %s\n' "$1" "$2"; }
note_skip() { n_skip=$((n_skip + 1)); SUMMARY+=("SKIP $1 -- $2"); printf 'SKIP %s -- %s\n' "$1" "$2"; }

have() { command -v "$1" > /dev/null 2>&1; }

echo "== scan surface =="
printf '  repo root : %s\n' "$REPO_ROOT"
printf '  python    : pyproject.toml exclude lists (perception / third_party / docs-temp / data / models)\n'
printf '  c++ pkgs  : %s\n' "${CXX_PACKAGES[*]}"
printf '  c++ skip  : %s\n' "$CXX_EXCLUDE_RE"
echo

run_ruff() {
  if ! have ruff; then
    note_skip ruff "ruff not installed (pip install --user ruff)"
    return
  fi
  local out rc
  # --output-format=concise gives one line per finding, which is what the
  # count below counts. --no-fix is explicit rather than implied: ruff's
  # default changed once already, and a gate that silently edits the tree is
  # worse than no gate.
  set +e
  out="$(cd "$REPO_ROOT" && ruff check . --no-fix --output-format=concise 2>&1)"
  rc=$?
  set -e
  if [ "$rc" -eq 0 ]; then
    note_pass ruff
    return
  fi
  local n
  n="$(printf '%s\n' "$out" | grep -cE '^[^ ]+:[0-9]+:[0-9]+: [A-Z]+[0-9]+' || true)"
  printf '%s\n' "$out" | tail -n 20
  note_fail ruff "$n finding(s)"
}

run_mypy() {
  if ! have mypy; then
    note_skip mypy "mypy not installed (pip install --user mypy)"
    return
  fi
  local out rc
  set +e
  out="$(cd "$REPO_ROOT" && mypy . 2>&1)"
  rc=$?
  set -e
  if [ "$rc" -eq 0 ]; then
    note_pass mypy
    return
  fi
  # rc 2 with zero "error:" lines is mypy refusing to START (a bad config, an
  # unreadable file). That is NOT the same as finding type errors and must not
  # be reported as a count, or "mypy: 0 findings" ends up meaning "mypy checked
  # nothing" -- the exact failure this script was written after.
  local n
  n="$(printf '%s\n' "$out" | grep -cE ': error: ' || true)"
  printf '%s\n' "$out" | tail -n 20
  if [ "$n" -eq 0 ]; then
    note_fail mypy "mypy exited $rc WITHOUT checking (configuration error, not findings)"
  else
    note_fail mypy "$n error(s)"
  fi
}

# Every .cc/.cpp/.h/.hpp this repo owns, NUL-separated. Used by clang-format;
# clang-tidy needs a compile database and goes per package instead.
our_cxx_files() {
  (cd "$REPO_ROOT" && find common ros2_ws -type f \
      \( -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) \
      | grep -Ev "$CXX_EXCLUDE_RE" | sort)
}

run_clang_format() {
  if ! have clang-format; then
    note_skip clang-format "clang-format not installed (pip install --user clang-format)"
    return
  fi
  local files n_files out rc
  files="$(our_cxx_files)"
  n_files="$(printf '%s\n' "$files" | grep -c . || true)"
  if [ "$n_files" -eq 0 ]; then
    # An empty scan surface must be said out loud. A formatter that walked
    # zero files prints exactly what a clean tree prints (CLAUDE.md 3.2
    # form 1), and ros2_ws HAS been empty in this repo's history.
    note_skip clang-format "NO-TARGET: zero C++ files under common/ ros2_ws/"
    return
  fi
  set +e
  out="$(cd "$REPO_ROOT" && printf '%s\n' "$files" \
          | xargs clang-format --dry-run -Werror 2>&1)"
  rc=$?
  set -e
  if [ "$rc" -eq 0 ]; then
    note_pass "clang-format ($n_files files)"
    return
  fi
  local n n_bad
  n="$(printf '%s\n' "$out" | grep -cE 'code should be clang-formatted' || true)"
  n_bad="$(printf '%s\n' "$out" \
            | grep -oE '^[A-Za-z0-9_./-]+\.(cc|cpp|h|hpp):[0-9]+:[0-9]+: error' \
            | sed 's/:[0-9]*:[0-9]*: error//' | sort -u | grep -c . || true)"
  note_fail clang-format "$n violation(s) in $n_bad of $n_files file(s)"
}

run_clang_tidy() {
  if ! have clang-tidy; then
    note_skip clang-tidy "clang-tidy not installed (pip install --user clang-tidy)"
    return
  fi
  local missing=() total=0 tus=0 pkg db log
  log="$(mktemp)"
  for pkg in "${CXX_PACKAGES[@]}"; do
    db="$REPO_ROOT/ros2_ws/$pkg/build/compile_commands.json"
    if [ ! -f "$db" ]; then
      missing+=("$pkg")
      continue
    fi
    local tu_list
    tu_list="$(mktemp)"
    python3 - "$db" > "$tu_list" <<'PY'
import json, sys
seen = []
for entry in json.load(open(sys.argv[1])):
    f = entry["file"]
    if f not in seen:
        seen.append(f)
print("\n".join(seen))
PY
    local n_tu
    n_tu="$(grep -c . "$tu_list" || true)"
    tus=$((tus + n_tu))
    printf '  clang-tidy %s: %d translation unit(s)\n' "$pkg" "$n_tu"
    while read -r f; do
      [ -n "$f" ] || continue
      set +e
      clang-tidy -p "$REPO_ROOT/ros2_ws/$pkg/build" --quiet "$f" >> "$log" 2>/dev/null
      set -e
    done < "$tu_list"
    rm -f "$tu_list"
  done
  if [ "${#missing[@]}" -gt 0 ]; then
    # A package with no compile database is NOT clean -- it is unexamined.
    note_skip clang-tidy "no compile_commands.json for: ${missing[*]} (configure with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)"
    rm -f "$log"
    return
  fi
  total="$(grep -cE ': (warning|error): ' "$log" || true)"
  if [ "$total" -eq 0 ]; then
    rm -f "$log"
    note_pass "clang-tidy ($tus TUs)"
    return
  fi
  grep -oE '\[[a-z][a-z-]+\]$' "$log" | sort | uniq -c | sort -rn | head -n 8 | sed 's/^/    /'
  rm -f "$log"
  note_fail clang-tidy "$total finding(s) over $tus TU(s)"
}

for g in "${selected[@]}"; do
  echo "== $g =="
  case "$g" in
    ruff)         run_ruff ;;
    mypy)         run_mypy ;;
    clang-format) run_clang_format ;;
    clang-tidy)   run_clang_tidy ;;
  esac
  echo
done

echo "== summary =="
for line in "${SUMMARY[@]}"; do
  printf '  %s\n' "$line"
done
printf 'ci gates: %d passed, %d failed, %d skipped (of %d run)\n' \
       "$n_pass" "$n_fail" "$n_skip" "${#selected[@]}"

if [ "$n_fail" -gt 0 ]; then
  exit 1
fi
if [ "$n_skip" -gt 0 ] && [ "$allow_skip" = "0" ]; then
  echo "exiting non-zero because of the skip(s) above; pass --allow-skip to" \
       "accept them on this host" >&2
  exit 1
fi
exit 0
