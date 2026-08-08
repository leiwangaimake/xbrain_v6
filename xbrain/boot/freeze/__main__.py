"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: xbrain-config-freeze entrypoint -- systemd calls `python -m
       xbrain.boot.freeze`

Description:
Runs a full freeze pass with values read from environment (config root) or
common facilities (boot_id, common_digest). Exit code is 0 on green, 1 on
any assertion failure or missing resource -- Type=oneshot units treat the
exit code as the whole verdict.

NOTE CFG-FZ-1 scope: this entrypoint wires the FRAMEWORK together. The
individual assertion bodies are still stubs (they will land per CFG-FZ-N);
so on the current tree it exits 0 with `assertions[*].status == "stub"`.
That is the DESIGNED state at CFG-FZ-1 landing; CFG-CF-9's milestone test
(configs almost empty => refuse to start) becomes real once J / A / M
have their real bodies.

Argument-parse discipline:
  Every argument has a default suitable for real deploy. --config-root
  defaults to the compiled-in /opt/xbrain_v6/configs (or env override),
  --resolved-root to /run/xbrain/resolved (the CFG-BT-22 mount), and
  --boot-id-path to /proc/.../boot_id. Tests exercise every path via
  temp overrides, so no path is untested and no path is opinionated at
  the test-vs-deploy boundary.

Failure modes worth spelling out (each maps to a distinct exit code path):
  * --config-root does not exist         -> J-style, printed loud, exit 1
  * resolved_root not a directory        -> CFG-BT-22 mount fail,   exit 1
  * an assertion runner raises           -> propagates, exit 1
  * any assertion returns status != pass -> currently NOT checked (all
    stubs return 'stub' at CFG-FZ-1; CFG-FZ-2..15 will add pass/fail
    reading and this file will grow the exit-nonzero branch then)
"""

import argparse
import os
import sys

# pipeline.run_freeze is the framework entry; RESOLVED_ROOT_DEFAULT is the
# tmpfs path CFG-BT-22 mounts. Importing them here keeps the wiring in
# ONE place; a call site elsewhere would fork the arg-passing contract.
from xbrain.boot.freeze.pipeline import RESOLVED_ROOT_DEFAULT, run_freeze
# boot_id read via the shared helper so the /proc path is the single source
# of truth (also used by the resolved loader; two callers, one path).
from xbrain.common.config.resolved import BOOT_ID_PATH, read_boot_id


def main() -> int:
    """Entry point wired into deploy/systemd/xbrain-config-freeze.service.

    Arg-parse then pre-check then run. Kept a single top-level function so
    the systemd invocation and any interactive `python -m xbrain.boot.freeze`
    call reach identical behaviour.
    """
    ap = argparse.ArgumentParser(prog="python -m xbrain.boot.freeze")
    # --config-root default: env XBRAIN_CONFIG_DIR (10 S5.4.3 L5 whitelist)
    # or the compiled-in path. Systemd sets neither by default, so on a
    # normal deploy the compiled path is what runs -- kept explicit so a
    # test rig can override without touching the unit file.
    # freeze __main__ IS the process that reads configs/ raw and
    # produces /run/xbrain/resolved. Two literals here (default +
    # help text) are the interface itself, not a consumer reaching
    # for the source path. Markers must attach to each literal line.
    ap.add_argument("--config-root",
                    default=os.environ.get(
                        "XBRAIN_CONFIG_DIR",
                        # CONFIG-SOURCE-OK(freeze): default source root
                        "/opt/xbrain_v6/configs"),
                    # CONFIG-SOURCE-OK(freeze): help text mirrors default
                    help=("config root (default: env XBRAIN_CONFIG_DIR or "
                          "/opt/xbrain_v6/configs)"))  # CONFIG-SOURCE-OK(freeze)
    # --resolved-root: where MANIFEST.json + per-process YAML go. Must exist
    # (CFG-BT-22 mounts the tmpfs); mkdir here would mask a mount failure.
    ap.add_argument("--resolved-root", default=RESOLVED_ROOT_DEFAULT,
                    help="tmpfs mount that receives MANIFEST.json + "
                    "per-process YAML (must exist; CFG-BT-22 owns it)")
    # --boot-id-path: exposed so tests can point at a fake proc entry
    # without touching /proc.
    ap.add_argument("--boot-id-path", default=BOOT_ID_PATH)
    args = ap.parse_args()   # exits nonzero on bad argv (argparse handles it)

    # Pre-check the config root even though this is J's territory. Two
    # reasons: (1) J is still a stub in the CFG-FZ-1 landing, so the real
    # "no config root" defect would otherwise surface as "assertion J
    # passed" (a stub returns status=stub, not fail), which is worse than
    # silence -- an operator would think the config was found; (2) the
    # pipeline needs the root path to exist before common_digest / layer
    # walks make sense, so a fail-fast here beats a stack trace later.
    if not os.path.isdir(args.config_root):
        print("FAIL: --config-root %r does not exist" % args.config_root,
              file=sys.stderr)
        return 1

    # boot_id read HERE (main), not inside pipeline, so a test can inject
    # a fake path via --boot-id-path and pipeline stays a pure function of
    # its args. Real deploys always use /proc/sys/kernel/random/boot_id.
    boot_id = read_boot_id(args.boot_id_path)
    # config_root_overridden distinguishes "using the deploy default" from
    # "operator set XBRAIN_CONFIG_DIR" -- MANIFEST records the fact so an
    # incident can name which set of yaml was frozen (a test rig vs prod).
    overridden = "XBRAIN_CONFIG_DIR" in os.environ

    # CFG-FZ-9 (materialiser) fills layers/processes; empty here is fine
    # for CFG-FZ-1 -- MANIFEST is well-formed with empty layers / processes
    # and the bidirectional-diff test targets ASSERTIONS, not those fields.
    # common_digest / config_rev: placeholders until CFG-CM-10 wires the
    # real digest chain (currently a visible "stub-not-yet-computed" token,
    # not a plausible-looking hash string -- silence-is-not-success).
    try:
        manifest = run_freeze(
            boot_id=boot_id,
            config_root=os.path.abspath(args.config_root),
            config_root_overridden=overridden,
            common_digest="stub-not-yet-computed",   # CFG-CM-10 lands the real
            config_rev="stub-not-yet-computed",      # digest chain
            resolved_root=args.resolved_root,
        )
    except FileNotFoundError as exc:
        # run_freeze raises this when resolved_root is absent (CFG-BT-22
        # did not run OR the tmpfs failed to mount). Print without a
        # traceback so systemd's journal shows a clean, actionable line
        # rather than a Python stack the operator has to squint at.
        print("FAIL: %s" % exc, file=sys.stderr)
        return 1

    # Loud summary so an operator watching journalctl sees which rows are
    # still stubs (the ones each CFG-FZ-N will replace). Print to stdout
    # per-line so systemd journal timestamps each; the stub warning goes
    # to stderr so a grep on WARNING catches it separately.
    # One line per assertion, name-column right-padded to 10 for readability
    # in journalctl. Passing None-status to .get avoids KeyError if a
    # runner returned a dict without status (still a defect, but this loop
    # should not be the place that reports it -- it reports what IS there).
    for name, result in manifest["assertions"].items():
        print("assertion %-10s %s" % (name, result.get("status")))
    # Count stubs so a summary line at the end tells the operator how many
    # of the CFG-FZ-N items still owe real bodies. Zero means every runner
    # is real (post CFG-FZ-15).
    stubs = sum(1 for r in manifest["assertions"].values()
                if r.get("status") == "stub")
    if stubs:
        # WARN not FAIL: bring-up should not be blocked by "not implemented
        # yet". Once every CFG-FZ-N replaces its stub, stubs == 0 and this
        # branch stops printing -- a clean deploy shows nothing here.
        print("NOTE: %d assertion(s) still stubbed; CFG-FZ-N items land "
              "the real bodies" % stubs, file=sys.stderr)
    # Deliberate: log the exact absolute path an operator can `cat` after
    # the unit finishes -- systemd journal timestamps this line so a
    # bring-up post-mortem knows exactly when freeze completed.
    print("MANIFEST written to %s/MANIFEST.json" % args.resolved_root)
    return 0


# Direct-invocation guard so `python -m xbrain.boot.freeze` runs main() but
# `import xbrain.boot.freeze.__main__` does not (a test that imported this
# module for coverage should not accidentally run the whole freeze).
if __name__ == "__main__":
    sys.exit(main())
