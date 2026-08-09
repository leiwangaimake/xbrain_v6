"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain-probe.service (Stage 0) package marker

Description:
Stage 0 platform probe. Runs as a systemd Type=oneshot BEFORE all
runtime units and BEFORE the config-freeze service. If any check fails,
this process exits non-zero and systemd's Requires= chain refuses to
start Stage 0z / 0c / 1 / 2 / 3 -- the whole runtime stack stays down
(R class per 10 S3.3.6).

Why one probe process, not per-check individual ones. Each check needs
the same environment (root filesystem visible, /etc/xbrain readable,
python interpreter alive). Splitting one probe into 5 units multiplies
systemd unit management overhead 5x without gaining any independence
-- if disk is full every probe fails.

Boundary. This module DOES:
  * disk-usage / memory / temperature threshold checks (M-M-1 / M-M-2)
  * schema_version read on the four SQLite databases (S-8 in 10 S3.3.1)
  * GATE-6: enumerate /sys/class/net, compare each interface's IP /
    netmask / network against /etc/xbrain/hw_profile YAML
  * refuses if any two interfaces overlap network segments (NET-C1)

It DOES NOT:
  * bring up any network interface (systemd-networkd does)
  * write anything under /run/xbrain/resolved (config-freeze does)
  * touch service unit files or restart anything (systemd does)
  * decide the return code for other stages -- it only returns its own

Look-but-do-not-touch is intentional. Stage 0 is the earliest gate;
a bug that silently mutates state here would run before any of the
mechanisms that catch mutations exist.
"""
