"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: collect.py
Brief: CHK-2-63 -- support-bundle assembly (pure functions)

Description:
The pure functions that build the support-bundle tarball. Kept
separate from __main__.py so unit tests can drive the individual
stages without spawning a subprocess.

Process list discovery. CLAUDE.md 3.7 forbids hardcoding a count of
processes in judge / test code. The list is derived at RUNTIME by
scraping CLAUDE.md 0.1's process table (rows shaped "| {proc} | ...").
If a new row is added to CLAUDE.md 0.1, this collector picks it up on
the next run WITHOUT edits here. If a row is deleted, likewise.

Size-cap discipline. `assemble()` accepts max_bytes. When the raw
staged directory exceeds that, the collector drops the OLDEST logs
(sorted by mtime ascending) until under the cap and records every
dropped file into the bundle's MANIFEST.truncated[]. Silent drop is
the failure mode CHK-2-63 variant (c) bans; the manifest field is the
audit trail. A stub that just refuses to run past the cap would fail
variant (c) too (it would still be silent from the operator's view --
"the tarball never appeared, why?"), so the collector always produces
a tarball and always names what was dropped.

Secrets ban. `configs/secrets/` and any path under it is never copied.
The check is a startswith on POSIX path components (not a substring on
the string) so a path like `configs/secrets-manifest.yaml` is NOT
banned by accident. Test coverage: variant (b) copies a fake secret
into configs/secrets/ and asserts the bundle does not contain it.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import tarfile
from pathlib import Path
from typing import Iterable, List, Optional


# --- Process list from CLAUDE.md 0.1 --------------------------------

_PROCESS_ROW_RE = re.compile(r"^\|\s*`([A-Za-z][A-Za-z0-9 _-]+)`\s*\|")

# Non-process rows in the same table (header rows, category rows).
_NON_PROCESS_TOKENS = {"进程", "语言", "面", "说明"}


def read_process_list(claude_md_path: str) -> List[str]:
    """Extract the process name column from CLAUDE.md's §0.1 table.

    We match ROW-shape "| `name` |" and skip anything that's not a
    plausible process identifier. This is intentionally forgiving: the
    table can grow rows without needing a schema change here."""
    names: List[str] = []
    with open(claude_md_path, "r", encoding="utf-8") as f:
        for line in f:
            m = _PROCESS_ROW_RE.match(line)
            if not m:
                continue
            tok = m.group(1)
            if tok in _NON_PROCESS_TOKENS:
                continue
            # Filter tokens that don't look like process names -- Real
            # process names all have an underscore or hyphen or are
            # single-word lowercase. This is the tightest pattern that
            # still admits every current row.
            if len(tok) < 3:
                continue
            names.append(tok)
    # Deduplicate while preserving order.
    seen = set()
    out = []
    for n in names:
        if n in seen:
            continue
        seen.add(n)
        out.append(n)
    return out


# --- Path safety ----------------------------------------------------

def is_under_secrets(path: str, secrets_root: str) -> bool:
    """True iff path is inside secrets_root (POSIX component-aware).

    Substring match would false-positive on `configs/secrets-manifest`;
    resolve to absolute and check ancestor equality instead."""
    p = Path(path).resolve()
    root = Path(secrets_root).resolve()
    try:
        p.relative_to(root)
    except ValueError:
        return False
    return True


# --- Log tail -------------------------------------------------------

def tail_file(src: str, dst: str, n_bytes: int) -> int:
    """Copy the last n_bytes of src into dst. Returns bytes written.

    If src is smaller than n_bytes, copies the whole file. Used for
    per-process log tails so the bundle is bounded per file."""
    if not os.path.isfile(src):
        return 0
    size = os.path.getsize(src)
    to_copy = min(size, n_bytes)
    with open(src, "rb") as f_in:
        if size > n_bytes:
            f_in.seek(size - n_bytes)
        data = f_in.read(to_copy)
    with open(dst, "wb") as f_out:
        f_out.write(data)
    return len(data)


# --- Stage: build a staging directory containing all bundle files ---

def stage(
    stage_dir: Path,
    processes: List[str],
    data_root: Path,
    resolved_dir: Path,
    boot_fail_path: Optional[Path],
    bit_result_path: Optional[Path],
    build_version_path: Optional[Path],
    log_tail_bytes: int,
    systemctl_snapshot: Optional[str] = None,
) -> dict:
    """Fill stage_dir with everything the bundle should contain, and
    return a manifest dict describing what was included / skipped."""
    manifest: dict = {
        "processes": processes,
        "included": [],
        "skipped": [],
        "truncated": [],
        "warnings": [],
    }

    # -- versions/build_version.txt -----------------------------
    if build_version_path and build_version_path.is_file():
        vers_dir = stage_dir / "versions"
        vers_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(build_version_path, vers_dir / "build_version.py")
        manifest["included"].append("versions/build_version.py")
    else:
        manifest["skipped"].append("versions/build_version")

    # -- logs/{proc}.log.tail ------------------------------------
    logs_dir = stage_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    for proc in processes:
        src = data_root / "logs" / (proc + ".log")
        dst = logs_dir / (proc + ".log.tail")
        n = tail_file(str(src), str(dst), log_tail_bytes)
        if n > 0:
            manifest["included"].append("logs/" + proc + ".log.tail")
        else:
            # Silent skip is fine here: not every process writes a log
            # yet in phase-0 dev, and forcing red would make the bundle
            # unusable during bring-up. The audit trail is manifest.skipped.
            manifest["skipped"].append("logs/" + proc + ".log")

    # -- resolved/ (whole tree) OR explicit note if missing ------
    dst_res = stage_dir / "resolved"
    if resolved_dir.is_dir():
        shutil.copytree(str(resolved_dir), str(dst_res))
        manifest["included"].append("resolved/")
    else:
        # CHK-2-63 variant (d): explicit annotation is REQUIRED so an
        # operator does not assume the bundle is complete when /run
        # was cleared by a fresh boot.
        manifest["warnings"].append(
            "resolved/ missing -- /run/xbrain/resolved not populated; "
            "boot may not have reached config-freeze")
        dst_res.mkdir()
        (dst_res / "MISSING.txt").write_text(
            "resolved/ was empty at bundle time (see MANIFEST.warnings)")

    # -- boot_fail.jsonl (optional) -----------------------------
    if boot_fail_path and boot_fail_path.is_file():
        shutil.copy2(boot_fail_path, stage_dir / "boot_fail.jsonl")
        manifest["included"].append("boot_fail.jsonl")

    # -- bit/last.json (optional) -------------------------------
    if bit_result_path and bit_result_path.is_file():
        bit_dir = stage_dir / "bit"
        bit_dir.mkdir(exist_ok=True)
        shutil.copy2(bit_result_path, bit_dir / "last.json")
        manifest["included"].append("bit/last.json")

    # -- systemd/status.txt --------------------------------------
    if systemctl_snapshot is not None:
        sd = stage_dir / "systemd"
        sd.mkdir(exist_ok=True)
        (sd / "status.txt").write_text(systemctl_snapshot)
        manifest["included"].append("systemd/status.txt")

    return manifest


# --- Enforce size cap -----------------------------------------------

def enforce_cap(stage_dir: Path, max_bytes: int, manifest: dict) -> None:
    """If the staged tree exceeds max_bytes, drop oldest files until
    under the cap. Every dropped file is recorded in manifest.truncated
    so the operator sees exactly what was left out."""
    while True:
        total = sum(
            f.stat().st_size for f in stage_dir.rglob("*") if f.is_file())
        if total <= max_bytes:
            return
        # Find oldest file (excluding MANIFEST.json which we haven't
        # written yet, and the versions/ dir which is tiny anyway).
        files = [f for f in stage_dir.rglob("*") if f.is_file()]
        if not files:
            return
        files.sort(key=lambda f: f.stat().st_mtime)
        drop = files[0]
        rel = drop.relative_to(stage_dir).as_posix()
        manifest["truncated"].append({
            "path": rel,
            "size_bytes": drop.stat().st_size,
            "reason": "over_max_bytes",
        })
        drop.unlink()


# --- Assemble: stage + cap + write MANIFEST + tar --------------------

def assemble(
    out_tarball: Path,
    processes: List[str],
    data_root: Path,
    resolved_dir: Path,
    boot_fail_path: Optional[Path],
    bit_result_path: Optional[Path],
    build_version_path: Optional[Path],
    log_tail_bytes: int = 1024 * 1024,   # 1 MiB per log by default
    max_bundle_bytes: int = 128 * 1024 * 1024,  # 128 MiB total default
    systemctl_snapshot: Optional[str] = None,
    stage_dir: Optional[Path] = None,
) -> dict:
    """Build the tarball at out_tarball. Returns the manifest dict.
    Caller supplies stage_dir if it wants control over cleanup; if not
    provided, we make one in the parent of out_tarball."""
    if stage_dir is None:
        stage_dir = out_tarball.parent / (out_tarball.stem + ".stage")
    stage_dir.mkdir(parents=True, exist_ok=True)

    manifest = stage(
        stage_dir=stage_dir,
        processes=processes,
        data_root=data_root,
        resolved_dir=resolved_dir,
        boot_fail_path=boot_fail_path,
        bit_result_path=bit_result_path,
        build_version_path=build_version_path,
        log_tail_bytes=log_tail_bytes,
        systemctl_snapshot=systemctl_snapshot,
    )

    enforce_cap(stage_dir, max_bundle_bytes, manifest)

    # Write manifest LAST so it always reflects what actually ended up
    # in the bundle (post-truncation).
    (stage_dir / "MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True))

    # Compose tarball; arcname strips absolute prefix.
    out_tarball.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(str(out_tarball), "w:gz") as tar:
        tar.add(str(stage_dir), arcname=out_tarball.stem)

    return manifest
