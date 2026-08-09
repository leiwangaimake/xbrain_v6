"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain.boot.diag -- support-bundle collector (CHK-2-63)

Description:
On-site support-bundle collector. Emits a tar.gz containing everything
an off-site engineer needs to reason about a live-machine problem:
process logs, resolved configs, MANIFEST, boot_fail record, latest
BIT result, recent events, build_version, systemctl status snapshot.

Why python not just bash. Bash's tar / find / du composition is fine
for the copy path; the tricky part is (a) never including secrets and
(b) truncating on a size cap while leaving an audit trail. Doing both
in bash requires careful quoting; python's shutil + tarfile do it
directly, and the collector can be tested with tmp_path fixtures.

The bundle layout inside the tarball:

  xbrain-diag-{robot_id}-{ts}/
    MANIFEST.json                  <-- bundle-side manifest (versions,
                                       what was included, what was
                                       truncated -- distinct from
                                       /run/xbrain/resolved/MANIFEST.json)
    versions/
        build_version.txt
        os-release
    logs/
        {proc}.log.tail             <-- last N MiB of data/logs/{proc}.log
    resolved/                        <-- full snapshot of /run/xbrain/resolved/
    boot_fail.jsonl                  <-- if present at data/boot_fail.jsonl
    bit/last.json                    <-- latest BIT result (if any)
    events/last.jsonl                <-- last M rows of record.db
    systemd/status.txt               <-- `systemctl status xbrain-*`

Boundary: this module DOES NOT ship the tarball anywhere. Uploading is
an operator action (scp / rsync). The bundle sits at
data/diag/xbrain-diag-{robot_id}-{ts}.tar.gz until removed.
"""
