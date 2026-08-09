"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p3_task process package (15 S1: 任务与充电，四库唯一写者)

Description:
p3_task owns the task queue, docking / charging orchestration, and
is the UNIQUE writer for task.db / fence.db / geo.db (record.db is
p5_gateway's). See 15 S9 for the writer split and the aiosqlite
discipline (persistence/ layer only; other layers must go through DAO).

MVP status: __main__.py loads config, prints a heartbeat, does NOT
yet run the task queue or the docking state machine.
"""
