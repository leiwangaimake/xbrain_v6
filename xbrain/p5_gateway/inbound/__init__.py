"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: P5 cloud inbound -- Qt frames enter here and are rebuilt

Description:
The gateway is the ONLY process holding a cloud link (11 S4.6). Everything
Qt sends arrives here, gets its envelope validated against v2.0 S1.1, and is
rebuilt into our internal shape before it reaches any business module.

Nothing downstream of this package should ever see a raw Qt frame: that is
the whole reason the gateway exists as a boundary.
"""
