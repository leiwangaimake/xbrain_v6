"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: P1 speed gate f(d_free) + g(targets) + creep + observability

Description:
Package marker. See modules in this directory for the actual surface.

"+ audit" left this line on 2026-09-30 with gate/audit.py: that module held a
second, invented limiter closed set and had no production caller. The gate
ATTRIBUTION half of MOT-PM-9 lives in nav/host_gate.py, which consumes the
shared closed set xbrain.common.enums GATE_LIMITER (11 S9.6.5) -- so naming
"audit" here pointed readers at the wrong file.
"""
