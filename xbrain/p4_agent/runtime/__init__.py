"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p4_agent runtime wiring (turn loop + intent dispatch + Zenoh session)

Description:
Runtime wiring for p4_agent: subscribes rt/audio/mic, runs a simple
energy-VAD to segment utterances, calls the local ASR service via
xbrain.p4_agent.ai_client.asr_client.transcribe, routes recognised
text through the intent classifier, dispatches to the 5 outbound
key families (cmd/audio/speak / cmd/task / cmd/ptz / cmd/payload /
cmd/motion/intent). Sub-modules are called from xbrain/p4_agent/
__main__.py.
"""
