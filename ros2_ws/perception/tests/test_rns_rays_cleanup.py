#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: diaofanghaiyang@hachist.com
上海哈船智能船舶技术有限公司
File: test_rns_rays_cleanup.py
Brief: Linux-only cleanup test of the ray driver's owned process group; no SDK, camera or model call.

Description:
What it checks. The offline Orbbec rays test starts helper processes; this test proves only the
children created here are ever signalled and that each successful case finishes within 2
seconds. The ray driver's module is loaded without invoking its main, no files are written and
stdout carries the receipt. Registered by tests/orbbec_rays.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import types

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "tools/rns_rays_offline_target.py"


def require(condition, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def live_members(pgid: int) -> list[int]:
    """Independent complete inventory, not the driver's early-return helper."""
    result = []
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            raw = (path / "stat").read_text()
        except (FileNotFoundError, ProcessLookupError):
            continue
        # comm may contain spaces or parentheses; start after its final ')'.
        fields = raw[raw.rfind(")") + 1:].split()
        require(len(fields) >= 4, "short process stat")
        if int(fields[2]) == pgid:
            require(int(fields[3]) == pgid, "owned group has unexpected session")
            if fields[0] not in ("Z", "X", "x"):
                result.append(int(path.name))
    return sorted(result)


def emergency_cleanup(process: subprocess.Popen) -> None:
    """Failure cleanup stays restricted to this exact freshly created group."""
    require(process.pid != os.getpgrp(), "refusing to signal test's own group")
    if live_members(process.pid):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait(timeout=1)
    deadline = time.monotonic() + .5
    while live_members(process.pid) and time.monotonic() < deadline:
        time.sleep(.01)
    require(not live_members(process.pid), "test-created process group remains live")


def run_case(driver, descendant: bool) -> dict:
    begun = time.monotonic()
    # The sleeper inherits the leader's new session/group, but no output pipe:
    # communicate() must observe leader exit independently from child lifetime.
    code = (
        "import subprocess,sys; "
        "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'], "
        "stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); "
        "print(child.pid,flush=True)"
    ) if descendant else "pass"
    process = subprocess.Popen([sys.executable, "-B", "-c", code],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, start_new_session=True)
    require(process.pid != os.getpgrp(), "new child group unexpectedly equals test group")
    try:
        stdout, stderr = process.communicate(timeout=1)
        require(process.returncode == 0, f"leader failed: {stderr}")
        before = live_members(process.pid)
        child_pid = int(stdout.strip()) if descendant else None
        if descendant:
            require(child_pid in before, "expected live descendant after leader exit")
            require(os.getpgid(child_pid) == process.pid and os.getsid(child_pid) == process.pid,
                    "descendant is outside our exact owned PGID/SID")
            require(driver.group_alive(process.pid), "driver missed live descendant")
        else:
            require(not before and not driver.group_alive(process.pid), "normal leader left live members")
        receipt = driver.stop_group(process)
        after = live_members(process.pid)
        require(not receipt["live_group_after"] and not after, "cleanup left a live process")
        require(receipt["pgid"] == process.pid, "cleanup receipt changed group identity")
        require(receipt["term"] is descendant and not receipt["kill"],
                "ordinary sleeper should need TERM only; clean exit should need no signals")
        elapsed = time.monotonic() - begun
        require(elapsed < 2, f"case exceeded 2 seconds: {elapsed:.6f}")
        return {"case": "reaped_leader_live_descendant" if descendant else "normal_exit",
                "leader_pid": process.pid, "descendant_pid": child_pid,
                "leader_returncode": process.returncode, "live_before": before,
                "live_after": after, "cleanup": receipt, "elapsed_seconds": elapsed}
    finally:
        emergency_cleanup(process)


def main() -> int:
    # Fail before creating children on non-Linux hosts. No fake /proc fallback.
    if sys.platform != "linux" or not Path("/proc/self/stat").is_file():
        print("Linux /proc is required; no child process was launched", file=sys.stderr)
        return 2
    module = types.ModuleType("rns_rays_offline_target_under_test")
    module.__file__ = str(DRIVER)
    exec(compile(DRIVER.read_bytes(), str(DRIVER), "exec"), module.__dict__)
    # The module's __name__ differs from '__main__', so device/environment
    # preflight and build/test launch code remain unexecuted.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        module.on_signal(sig, None)  # callback only; do not signal this test process
        require(module.STOP_SIGNAL == sig, "driver signal callback did not latch its signal")
    module.STOP_SIGNAL = None
    results = [run_case(module, True), run_case(module, False)]
    print("RAYS_CLEANUP_TEST " + json.dumps({"status": "LINUX_OWNED_GROUP_PASS", "cases": results,
        "groups": 2, "signal_flag_checks": 3, "driver_main_called": False, "sdk_called": False,
        "camera_opened": False, "model_loaded": False, "live_members_remaining": []}))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ray driver owned-group cleanup test failed: {error}", file=sys.stderr)
        raise SystemExit(1)
