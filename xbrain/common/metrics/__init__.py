"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Re-export the INF-OB-1 latency primitive and the metric registry

Description:
INF-OB-1's primitive half. Lets a module write
`from xbrain.common.metrics import LatencyHistogram, REGISTRY`. The CI gate, the
per-module registration metatest and the P1-loop budget check are the other three
halves of INF-OB-1 and are blocked on the TS-8 bench artifact and on the timed
loops (P1) existing -- see histogram.py's docstring for why building them now
would be a green shell.
"""

from xbrain.common.metrics.histogram import (
    LatencyHistogram, MetricRegistry, REGISTRY,
)

__all__ = ["LatencyHistogram", "MetricRegistry", "REGISTRY"]
