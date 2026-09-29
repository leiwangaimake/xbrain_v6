"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: tests for xbrain.p1_motion.rotation observability (12 S6A.8 OB-2)

Description:
Package marker so pytest imports these modules under a stable name. The
rotation JUDGE is exercised from tests/p1_motion/nav/test_nav_tick_rotation.py,
where it sits next to the NavTick stack that calls it; what lives here is the
layer above it -- turning the judge's per-tick verdict into events without
flooding the pipeline (12 S15 #54).
"""
