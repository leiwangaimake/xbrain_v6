"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: checks.py
Brief: Stage 0 probe checks -- disk, memory, temperature, DB schema

Description:
The four non-network checks the Stage 0 probe performs. Each is a
pure function that returns None on pass or a detail dict on fail. The
caller (main.py) is responsible for translating those into E_* codes
and exiting.

Structure of each check:

  def check_disk(path, threshold_pct) -> Optional[dict]

Threshold values are injected, never defaulted. CLAUDE.md 3.1 forbids
defaulted safety parameters; a probe that runs with "disk full at
90%" hardcoded and it turns out 90% is wrong is a fail-silent hazard.
main.py reads the thresholds from configs and passes them in.

Why sqlite is imported directly here despite CLAUDE.md 4.1 forbidding
it outside persistence/. Stage 0 runs BEFORE the persistence layer is
imported. It only READs schema_version, never writes and never opens
a connection that outlives the check. A DAO round-trip via aiosqlite
would require an event loop AT STAGE 0, and it must NOT. The exemption
is per CLAUDE.md 4.1 last bullet: DAOs are for business modules; this
is a boot probe.
"""

import os
import shutil
import sqlite3  # BUSINESS-IMPORT-OK(probe-bootstrap): probe runs BEFORE aiosqlite loop is up and only needs schema+integrity_check on cold-open DBs -- persistence layer would be a chicken-and-egg dependency
from pathlib import Path
from typing import List, Optional


# --- Disk ---------------------------------------------------------

def check_disk(path: str, threshold_pct: float) -> Optional[dict]:
    """Fail if the filesystem containing `path` is >= threshold_pct
    used. threshold_pct is 0..100.

    Why a percentage not free-bytes: the failure mode is "database
    write fails because the fs is out of space" and that depends on
    free ratio, not absolute size."""
    usage = shutil.disk_usage(path)
    used_pct = 100.0 * usage.used / usage.total
    if used_pct >= threshold_pct:
        return {
            "kind": "disk_full",
            "path": path,
            "used_pct": round(used_pct, 2),
            "threshold_pct": threshold_pct,
        }
    return None


# --- Memory -------------------------------------------------------

def _read_meminfo() -> dict:
    """Parse /proc/meminfo into a dict of int kB values."""
    out = {}
    with open("/proc/meminfo", "r") as f:
        for line in f:
            k, _, rest = line.partition(":")
            v = rest.strip().split()
            if len(v) >= 1:
                try:
                    out[k] = int(v[0])
                except ValueError:
                    pass
    return out


def check_memory(min_free_kb: int, meminfo=None) -> Optional[dict]:
    """Fail if MemAvailable is below min_free_kb.

    Injecting meminfo lets tests substitute a dict without patching
    /proc. MemAvailable is the kernel's own estimate of allocatable
    memory including reclaimable page cache; that is the right number
    to gate on rather than MemFree which excludes reclaimable pages."""
    if meminfo is None:
        meminfo = _read_meminfo()
    avail = meminfo.get("MemAvailable", 0)
    if avail < min_free_kb:
        return {
            "kind": "memory_low",
            "mem_available_kb": avail,
            "threshold_kb": min_free_kb,
        }
    return None


# --- Temperature -------------------------------------------------

def _read_temp_c_from_sysfs(sensor_path: str) -> Optional[float]:
    """Return degrees C or None if the sensor file cannot be read.
    /sys/class/thermal/thermal_zoneN/temp yields millidegrees."""
    try:
        with open(sensor_path, "r") as f:
            raw = int(f.read().strip())
        return raw / 1000.0
    except (FileNotFoundError, ValueError, PermissionError):
        return None


def check_temperature(sensors: List[str], max_temp_c: float,
                      read_fn=_read_temp_c_from_sysfs
                      ) -> Optional[dict]:
    """Fail if any sensor in `sensors` reports > max_temp_c. Missing
    sensors are treated as no reading (not a failure) -- Orin sensor
    availability varies across board revisions.

    Injecting read_fn lets tests inject synthetic temperature values
    without needing sysfs entries."""
    hot: List[dict] = []
    for path in sensors:
        t = read_fn(path)
        if t is None:
            continue
        if t > max_temp_c:
            hot.append({"path": path, "temp_c": t})
    if hot:
        return {
            "kind": "temperature_high",
            "max_temp_c": max_temp_c,
            "hot_sensors": hot,
        }
    return None


# --- SQLite schema_version ---------------------------------------

_SCHEMA_QUERY = "PRAGMA user_version;"


def check_db_schema(db_path: str, expected_version: int) -> Optional[dict]:
    """Open db_path read-only, read PRAGMA user_version, compare.
    Returns None on match. On mismatch returns detail with the actual
    version; on corruption returns detail with kind='db_corrupt' so
    main.py can raise E_STORAGE_CORRUPT vs E_CONFIG_INVALID correctly.

    Why read-only URI. A defect in this probe writing a value could
    put the fleet in an invalid state at boot; refusing write access
    at the kernel level is a cheap defence in depth."""
    db_name = os.path.basename(db_path)
    if not os.path.isfile(db_path):
        return {
            "kind": "db_missing",
            "db_name": db_name,
            "db_path": db_path,
        }
    uri = "file:%s?mode=ro" % db_path
    try:
        conn = sqlite3.connect(uri, uri=True, timeout=2.0)
        try:
            cur = conn.execute(_SCHEMA_QUERY)
            row = cur.fetchone()
            actual = int(row[0]) if row and row[0] is not None else -1
        finally:
            conn.close()
    except sqlite3.DatabaseError as exc:
        # "database disk image is malformed" surfaces here.
        return {
            "kind": "db_corrupt",
            "db_name": db_name,
            "db_path": db_path,
            "sqlite_error": str(exc),
        }
    if actual != expected_version:
        return {
            "kind": "db_schema_mismatch",
            "db_name": db_name,
            "expected_version": expected_version,
            "actual_version": actual,
        }
    return None


# --- Timezone -----------------------------------------------------

#: Where a distro keeps the IANA database. The zone name is whatever
#: follows this prefix in the resolved path of /etc/localtime.
_ZONEINFO_ROOTS = ("/usr/share/zoneinfo/", "/usr/lib/zoneinfo/")


def resolve_system_zone(localtime_path: str = "/etc/localtime"
                        ) -> Optional[str]:
    """The IANA name /etc/localtime points at, or None if unresolvable.

    None is returned for BOTH "missing" and "present but not a link
    into the zoneinfo tree" -- and the caller must treat both as a
    failure. That second case is the one worth naming: some images
    ship /etc/localtime as a plain COPY of the zone file rather than a
    symlink. The bytes are correct, the clock is correct, and the zone
    NAME is simply not recoverable from the filesystem. Guessing it by
    comparing file contents against every file under zoneinfo would
    "work" until two zones share a definition (Asia/Shanghai and
    Asia/Chongqing are byte-identical), at which point the guess is a
    coin flip that reports success. A deploy-time check that cannot
    read the name must say so, not invent one.
    """
    try:
        # readlink -f semantics: follow the whole chain. A relative
        # link (../usr/share/zoneinfo/Asia/Shanghai) is normal here.
        real = os.path.realpath(localtime_path)
    except OSError:
        return None
    if not os.path.exists(localtime_path):
        return None
    for root in _ZONEINFO_ROOTS:
        if real.startswith(root):
            name = real[len(root):]
            # posix/Asia/Shanghai and right/Asia/Shanghai are the same
            # zone under a different leap-second model; strip the
            # qualifier so the comparison is against the IANA name.
            for prefix in ("posix/", "right/"):
                if name.startswith(prefix):
                    name = name[len(prefix):]
            return name or None
    return None


def check_timezone(expected_zone: Optional[str],
                   localtime_path: str = "/etc/localtime"
                   ) -> Optional[dict]:
    """Fail unless the system zone is EXACTLY `expected_zone`.

    Why this is a Stage 0 gate and not a warning (CHK-1-62). Two
    unrelated subsystems silently depend on the machine's local zone:

      1. CHS-A requires a "Time" field formatted in LOCAL time (see
         CLAUDE.md 5.5 / the vendor PDF). Send it in UTC and the
         chassis rejects the very first frame with 0xE002.
      2. Operating-window rules (p2_core time_window) are wall-clock
         windows in site-local terms. A machine that boots in UTC
         evaluates "22:00-06:00" against the wrong hours and either
         arms or fails to arm a restriction, with nothing logged.

    Both failures are invisible at boot and expensive in the field,
    and both share one precondition -- so the precondition is checked
    once, here, before anything is released.

    expected_zone of None means "not calibrated" (CLAUDE.md 3.1: an
    uncalibrated safety value is null in config and refuses to start;
    NO defaulting to UTC or to whatever the host happens to have,
    since that is exactly the fail-silent path above).
    """
    actual = resolve_system_zone(localtime_path)
    if expected_zone is None:
        return {
            "kind": "timezone_not_calibrated",
            "key": "common.timezone",
            "actual": actual,
        }
    if actual is None:
        # NOT the same as a mismatch: nothing to compare. Reported
        # separately so the operator fixes the link rather than
        # hunting a zone difference that does not exist.
        return {
            "kind": "timezone_unresolvable",
            "localtime": localtime_path,
            "expected": expected_zone,
        }
    if actual != expected_zone:
        # Both values in the detail: the operator needs to know which
        # of the two is wrong, and only they can decide.
        return {
            "kind": "timezone_mismatch",
            "expected": expected_zone,
            "actual": actual,
        }
    return None
