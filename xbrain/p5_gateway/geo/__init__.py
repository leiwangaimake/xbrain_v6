"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: P5 geo relay (state/geo/objects -> HMI map layers, 11 S7.10A)

Description:
P5 caches the geo geometry P3 broadcasts on state/geo/objects (11 S7.10A) and
relays it to the HMI as the snapshot's routes/waypoints layers -- P5 NEVER reads
geo.db (11 S7843 single writer). This package is the cache + the shape conversion
into what the frontend renderGeo expects.
"""
