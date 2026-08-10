"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: tests/fixtures package marker (CHK-0-56 fixture root)

Description:
Fixture assets for the CHK-0-56 filled-config-set: the OVERRIDES map,
sites/lab.yaml + calib/lab_robot.yaml. The pytest fixture
`resolved_configs` in tests/fixtures/conftest.py materialises a full
L0-L6 tree at test-run by copying real configs/, applying overrides,
and symlinking configs/safety/ so the safety layer stays same-source
(ENV-2 / CHK-0-56 criterion iv).
"""
