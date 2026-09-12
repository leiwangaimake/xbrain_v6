"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p1 navigation host layer -- RNS candidate into the 20 Hz loop (P7.2)

Description:
The host side of 12 S4.2c: intake of the two mission entries (route_intake /
relmove_intake), the health-factor slot (health_factor), the output speed gate
around the RNS candidate (host_gate), the per-tick orchestration (nav_tick) and
the path_progress builder (progress). Everything here is pure (no Zenoh, no
clock) so it is unit-testable and mutant-testable; runtime/main_wiring.py owns
the sessions, the latest-value slots and the 20 Hz thread. RNS itself stays in
xbrain/p1_motion/rns, untouched by this package.
"""
