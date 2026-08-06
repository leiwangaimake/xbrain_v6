"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Zenoh transport helpers shared by every XBRAIN runtime process

Description:
What this package is for. The isolation between V6's two Zenoh routers lives
entirely in what each participant configures for itself -- see 11 S1.1.2 (grep
"RT 面配置与强制约束 RT-C1 ~ RT-C3"). Anything every participant must get right
belongs here, so that getting it right is an import rather than a paragraph five
processes each re-implement.

Today that is five things, and each is here because getting it wrong fails
silently rather than loudly:
  * the session-config factory, so the two routers stay isolated;
  * SubscriberRegistry, because a subscriber handle nobody holds is undeclared
    by the garbage collector with no log line on either side (CLAUDE.md 4.3);
  * EventBus, because the callback that handle survives to receive runs on a
    Rust-owned thread and must not touch the loop directly (CLAUDE.md 4.2);
  * the QoS table and its resolver, because a process that does not set QoS
    explicitly inherits the Zenoh defaults -- block, data, reliable, FIFO(256) --
    and 11 S2.4.0 traces each of those to a specific failure here, none of which
    announces itself;
  * the A-1 in-process self-check (PublisherThreadRegistry), because which thread
    publishes which key is knowledge only the process holds (10 S3.3.6 line 9),
    so a Q3 block sharing a thread with a Q1 cmd_vel is invisible to any
    config-static check and stalls a control tick in silence.
Nothing else is re-exported, and nothing should be added here as a placeholder
for work that has not landed -- CLAUDE.md 9.3 forbids reserving interfaces for
the future, and an empty name exported from a package is the cheapest form of
that. Only the A-1 half of the S2.4.8 anti-pattern self-check lives here, and
only because A-1 needs runtime thread<->publisher facts: it is INF-ZN-6, and it
consumes qos.py. The config-static half -- A-2..A-7, assertion F / INF-ZN-5 --
is NOT here; it runs in xbrain-config-freeze.service and reads qos.bindings.

*** The trap this package's NAME creates. It is called zenoh and the third-party
client library is also called zenoh. Python 3 resolves "import zenoh" from
session_factory.py absolutely, so it finds eclipse-zenoh and not this package --
but only while the directory on sys.path is the repository root. Put
xbrain/common on sys.path, directly or through a conftest, and this package
shadows the real client for the whole process. The symptom is an AttributeError
about Config, which names neither the shadowing nor the sys.path entry that
caused it, so session_factory.build_session_config checks for it explicitly and
says so. Do not "fix" a shadowing report by renaming the import; fix the path.

What does NOT belong here:
  * anything that opens a session or owns one. This package builds
    configuration; lifetime belongs to the process that runs the loop.
  * ROS types of any kind, matching the rule CLAUDE.md 5.3 states for the C++
    side of common/ -- its consumers include chassis_relay on the emergency-stop
    path.
"""

# Re-exported so a consumer writes "from xbrain.common.zenoh import
# build_session_config" and never has to know which module inside the package
# holds it. The module split is ours to change; this import list is the contract
# with the rest of the tree.
from .event_bus import (EventBus, Handler, HandlerContractError,
                        ThreadAffinityError)
# The QoS surface is deliberately narrow. load_qos_table is the only supported
# way to build a table, and the two error types are exported beside it so a
# caller that must distinguish "the document is wrong" from "this key cannot be
# given a QoS" does not have to reach into a private module path -- the shape
# people reach for instead is "except Exception", which CLAUDE.md 4.5 forbids.
# FROZEN_PROFILES and RT_OVERRIDE are exported for the cross-language metatest
# and for tooling that dumps the table; nothing in the runtime should be reading
# a profile directly instead of resolving a key.
from .qos import (FROZEN_PROFILES, RT_OVERRIDE, HandlerSpec, QosConfigError,
                  QosProfile, QosResolution, QosTable, QosViolation,
                  load_qos_table, parse_full_key)
from .session_factory import (PLANE_GEN, PLANE_RT, TRANSPORT_PLANES,
                              ZenohPlaneConfigError, build_session_config,
                              parse_plane, session_config_document,
                              session_config_json5)
# Imported after event_bus because subscriber_registry imports from it. The order
# is not required by Python -- absolute imports resolve either way -- but reading
# it in dependency order is what stops someone from "tidying" the two modules
# into a cycle later.
from .subscriber_registry import RegistryClosedError, SubscriberRegistry
# Imported after qos because publisher_thread_check consumes QosResolution and the
# BLOCK / PRIORITIES constants from it. The A-1 self-check (INF-ZN-6) is the one
# anti-pattern check that lives in this package, because it needs the runtime
# thread<->publisher binding a config-static check cannot see (10 S3.3.6 line 9).
from .publisher_thread_check import (ASSERTION_F_ANTI_PATTERNS,
                                     IN_PROCESS_ANTI_PATTERNS,
                                     MixedQosThreadError,
                                     PublisherThreadRegistry, ThreadMix,
                                     current_thread_name)

# Listed explicitly rather than left to the star-export default. Every name here
# is imported-and-unused from this file's point of view, which is precisely what
# a linter deletes as dead code; __all__ is the declaration that they are the
# public surface and not leftovers.
#
# The error types are exported alongside the classes that raise them. A caller
# that must catch exactly ThreadAffinityError should not have to import it from a
# private module path, because the alternative it will reach for is
# "except Exception", which also swallows the defects CLAUDE.md 4.5 requires to
# reach the process fault path untouched.
__all__ = ["PLANE_RT", "PLANE_GEN", "TRANSPORT_PLANES", "ZenohPlaneConfigError",
           "parse_plane", "session_config_document", "session_config_json5",
           "build_session_config", "EventBus", "Handler",
           "HandlerContractError", "ThreadAffinityError",
           "RegistryClosedError", "SubscriberRegistry",
           "FROZEN_PROFILES", "RT_OVERRIDE", "HandlerSpec", "QosProfile",
           "QosResolution", "QosTable", "QosConfigError", "QosViolation",
           "load_qos_table", "parse_full_key",
           "PublisherThreadRegistry", "MixedQosThreadError", "ThreadMix",
           "current_thread_name", "ASSERTION_F_ANTI_PATTERNS",
           "IN_PROCESS_ANTI_PATTERNS"]
