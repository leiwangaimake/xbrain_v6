"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Public surface of P4's configuration access, and nothing else

Description:
What this package is. The single seam between the P4 process and its
configuration. Everything P4 knows about where configuration comes from is
behind load_p4_config, so that "read the freeze line's product, never the
source" is something a P4 module gets by importing rather than something each
author has to remember (10 S5.4.1).

Why the surface is this small. Every name exported here is one a P4 module has
a reason to call. Anything wider invites a module to reach for a lower-level
piece -- the raw reader, a path constant -- and a module that assembles its own
path is outside every guarantee this package provides. In particular there is
deliberately NO exported constant naming the configuration source: a name that
resolved to it would be reachable from any P4 module, and scripts/lint/
no_config_source_read.py's rule 2 exists because that shape evades a
literal-string scan entirely.

Why there is no module-level state and nothing runs on import. Importing this
package must stay free of side effects: 10 S3.3.7's W-1 observation window
depends on a process being able to come up far enough to report WHY the stack
did not start, and a package that read a snapshot at import time would take
that ability down together with everything else.

What lives where, so the next addition lands in the right file:
  * loader.py -- reads the snapshot and performs the startup refusals volume 16
    owns (min_agent_version, the history closed set);
  * version.py -- ordering of version strings only, no configuration vocabulary.

What does NOT belong in this package: anything that consumes a configuration
VALUE to make a decision. Reading grammar.max_enum_items to size a GBNF
production, or gateway.* to build a client, belongs to the module that does that
work. Pulling those in here would make this the place where P4's behaviour is
decided, and the seam would stop being a seam.
"""

from .loader import (HISTORY_SCENARIOS, P4_PROCESS, P4ConfigError,
                     check_history_scenarios, check_min_agent_version,
                     check_no_unassigned_keys, load_p4_config, startup_report)
from .version import parse_version, satisfies

# The individual check_* functions are exported alongside load_p4_config on
# purpose. They are what the mutation tests drive directly: a test that could
# only call load_p4_config would have to construct a whole valid snapshot to
# probe one refusal, and the cost of that is tests that stop being written.
__all__ = ["P4_PROCESS", "HISTORY_SCENARIOS", "P4ConfigError",
           "load_p4_config", "startup_report", "check_min_agent_version",
           "check_no_unassigned_keys", "check_history_scenarios",
           "parse_version", "satisfies"]
