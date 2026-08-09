"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p2_core process package (14 S3 arbiter + mode + BIT + PTZ boost)

Description:
p2_core is the arbitration + mode-state-machine + BIT + PTZ-boost
process. In the current codebase, the ARBITER LIBRARY lives at
xbrain/common/arbiter/ (shared across owning processes -- see 14 S3.3
for why the library-not-service split). This package is the P2 PROCESS
WRAPPER: it instantiates the seven arbiter domains that P2 owns
(mode arbitration for global mode transitions, PTZ preset requests,
etc.), owns the mode state machine, runs BIT, and hosts the PTZ boost
policy.

MVP status: __main__.py loads config, prints a heartbeat, and does
NOT yet wire Zenoh sessions or run arbiter ticks in earnest. This is
the same skeleton pattern as xbrain/p4_agent -- makes systemd unit
runnable so integration testing can proceed even before the full P2
runtime is done.

See BIZ-CM-1..5 (arbiter library), BIZ-P2-* (P2 process work).
"""
