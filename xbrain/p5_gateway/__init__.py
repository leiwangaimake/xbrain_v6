"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p5_gateway package root -- config guards land first (GWY-P5-17)

Description:
The P5 gateway process (event pipeline, HMI backend, state/link publisher) is
Phase 2. What lands in Phase 0 is its config surface: configs/p5_gateway.yaml
transcribed from 17 S10.1 and the static guards in config/ that refuse the
bind shapes 17 S10.2 forbids. The process itself imports from here later.
"""
