"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ui_config.py
Brief: Project hmi.web.* config into the UI-config dict the frontend consumes

Description:
The problem this solves. 17 S6.10.2 makes the HMI's presentation values
(grid metres, fonts, panel sizes, wheel-zoom bounds, fence/route line styles,
waypoint marker shape/colour, ...) CONFIGURABLE (user 2026-08-12, keys U1..U7).
The frontend must not hardcode any of them: it reads a single ui_config object
served by the backend (17 S6.10.2 last line, "GET /api/hmi/ui_config or first-
paint inline"). This module is the one place that turns the resolved
p5_gateway.yaml `hmi.web` subtree into that object.

Which section this follows: 17 S6.10.2 (the U1..U7 key table) and the yaml
skeleton under configs/p5_gateway.yaml `hmi.web`.

What it does NOT do, and the boundary. It does NOT bind sockets, read any DB,
or touch the WS/REST routes -- it is a pure config-to-dict projection so it can
be unit-tested with a plain dict and no server. It does NOT invent values: an
absent `hmi.web` subtree is a config error the caller surfaces, not a silent
default here (the defaults live in the yaml, 3.1's home for values), because a
second copy of the defaults in code is exactly the drift 3.7 warns about.

Traps this file has already hit. These are UI params, NOT safety params, so
they legitimately carry defaults in the yaml -- do NOT apply the 3.1 "null ->
refuse" rule to grid_minor_m or a font size (that rule is only common.spec.*/
common.safety.*). The one value that IS deployment-critical, the bind
interfaces, lives in `hmi.bind` (not `hmi.web`) and keeps its null-refuses-start
behaviour in the web server, not here.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping


class UiConfigError(ValueError):
    """The resolved `hmi.web` subtree is missing or malformed.

    A dedicated type (not bare ValueError) so the web-server startup can catch
    exactly this and refuse to serve a half-configured UI, rather than shipping
    a page whose grid/fonts silently fall back to browser defaults -- which a
    reviewer would never notice because the page still renders.
    """


def build_ui_config(hmi_web: Mapping[str, Any]) -> Dict[str, Any]:
    """Return the UI-config dict the frontend renders from.

    `hmi_web` is the resolved `hmi.web` subtree (the runtime passes the freeze
    product, a test passes a fixture -- this module names no config source,
    same rule as the registry loader). Raises UiConfigError if a required
    presentation group is absent, because a page with no grid/zoom spec is not
    a page the operator can read a map on.

    The shape is deliberately the yaml's own shape, one-to-one: the frontend
    reads `cfg.map.grid.minor_m`, `cfg.font.plan.size_px`, etc., so a new UI
    param is added in ONE place (the yaml + 17 S6.10.2) and reaches the browser
    with no code change here.
    """
    if not isinstance(hmi_web, Mapping):
        raise UiConfigError("hmi.web must be a mapping, got %r" % type(hmi_web))
    # Required groups. Each maps a U-requirement (17 S6.10.2) to a subtree; a
    # missing one is a config defect, not something to paper over with a guess.
    required = ("map", "font", "layout", "fence", "route", "waypoint")
    missing = [k for k in required if k not in hmi_web]
    if missing:
        raise UiConfigError("hmi.web missing groups: %s" % sorted(missing))
    # Pass the presentation subtrees through verbatim. No transform: the yaml is
    # already the frontend's schema (17 S6.10.2), and re-mapping keys here would
    # be a second schema to keep in sync (3.7). `port`/`static_dir` are backend-
    # only (bind + static mount), so they are NOT forwarded to the browser.
    return {
        "map": dict(hmi_web["map"]),          # U1/U4: grid metres + zoom bounds
        "font": dict(hmi_web["font"]),        # U2: per-panel family + size_px
        "layout": dict(hmi_web["layout"]),    # U3: plan panel + status bar sizes
        "fence": dict(hmi_web["fence"]),      # U5: fence line by type (active/forbid/alarm)
        "route": dict(hmi_web["route"]),      # U5: recorded/realtime route lines
        "waypoint": dict(hmi_web["waypoint"]),  # U6: marker shape/colour/size
    }
