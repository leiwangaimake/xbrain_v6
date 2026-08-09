"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p1_motion process package (20 Hz cross-plane motion authority)

Description:
p1_motion is the SINGLE speed authority (10 S3.1: "唯一速度出口"). It
runs at 20 Hz, cross-plane (RT + GEN), consumes the arbitrated motion
factor from p2_core plus perception data, and emits cmd_vel to
chassis_relay. RNS (reactive navigation stack) is a p1_motion
INTERNAL module, not a separate process (10 S3.1 explicit note).

MVP status: __main__.py loads config, prints a heartbeat, and does
NOT yet run the 20 Hz loop or wire Zenoh. This is the same skeleton
pattern as xbrain/p2_core and xbrain/p4_agent -- makes the systemd
unit runnable so integration testing can proceed.
"""
