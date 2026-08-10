"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p2_core runtime wiring (audio + speaker + payload + Zenoh session)

Description:
Runtime wiring for p2_core: opens Zenoh sessions (RT + GEN planes),
launches the audio_io MIC capture thread, subscribes cmd/audio/speak
into the payload TTS client, publishes rt/audio/gate half-duplex
frames. Sub-modules are called from xbrain/p2_core/__main__.py.
"""
