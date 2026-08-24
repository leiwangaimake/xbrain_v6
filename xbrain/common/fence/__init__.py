"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: 跨进程共享围栏原语包 (11 S9A.2)

Description:
p1_motion(执行者)与 p3_task(权威源)共享的围栏几何与 crc32 单一真源. 见 geom.py
的边界说明. 本包只放[两侧都要用且必须逐字节一致]的东西, p3 私有的 painter /
录制期校验仍留在 p3_task/fence/.
"""

from xbrain.common.fence.geom import (Circle, Polygon, fence_set_crc32,
                                       point_in_circle, point_in_polygon)

__all__ = ["Circle", "Polygon", "point_in_circle", "point_in_polygon",
           "fence_set_crc32"]
