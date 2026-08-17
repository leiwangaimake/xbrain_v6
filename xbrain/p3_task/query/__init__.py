"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: P3 read-side task query (HMI task panel, 17 S6.8.4)

Description:
Package for the READ side of task.db that backs the HMI task panel (current +
history). Kept apart from dao/ (the write side) so the query projection and its
current/history split are unit-tested on their own, and so the batch-3 Zenoh
queryable handler has one place to call. It never writes.
"""
