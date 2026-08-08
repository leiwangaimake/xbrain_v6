"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Real bodies for the startup assertions declared in registry.py

Description:
Each CFG-FZ-N item replaces one AssertSpec.runner in the registry with
the assertion's real body, which lives in this subpackage as a module
named after the assertion (j_config_root, a_references, m_required, ...).
Keeping the bodies in separate modules -- rather than inline in
registry.py -- is CFG-FZ-1's "invisible if" guard: an assertion whose
body is a private if in the pipeline is invisible to the registry and
the bidirectional-diff test that keeps MANIFEST honest silently passes.

Convention: each assertion module exports one callable named
`run(ctx: dict) -> dict`. The result dict carries at minimum
`{"status": "pass"|"fail"|"skip", "assertion": "<name>"}`; failures
propagate as raised XbrainError (E_CONFIG_INVALID) rather than
status="fail" so the bring-up sequence stops on the first bad
assertion (CLAUDE.md S3.6 -- no fail-silent).
"""
