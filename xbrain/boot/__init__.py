"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Bring-up package -- xbrain-config-freeze main lives under freeze/

Description:
Every stage-0 orchestration step Python owns lives here (freeze service,
future stage-4 releaser). Anything that runs BEFORE the runtime processes
comes here rather than under xbrain/common/, because a shared-library
placement would suggest ordinary runtime callers may import it -- they
must not.
"""
