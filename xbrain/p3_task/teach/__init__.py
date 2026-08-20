"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Teach recording session package (11 S12A)

Description:
The cmd/teach half of the geographic subsystem, kept apart from ingest/geo_* on
purpose. They are two different channels with two different shapes:

  cmd/geo   stateless CRUD on a finished object -- one command, one row.
  cmd/teach a multi-turn stateful SESSION that produces the geometry in the
            first place -- start, drive, mark, finish, name it, then commit.

11 S12A.0 gives that as the reason the F class could not be expressed as slot
filling: recording is a conversation with a machine behind it. Modules:

  session.py   the S12A.3 state machine + the seven arming preconditions
  sampling.py  the S12A.6 rule: a 1 Hz timer with 0.5 m dedup, in WGS84
  validate.py  the S12A.7 geometry checks run at finish and again at save
  command.py   the S12A.4 TeachCommand envelope
"""
