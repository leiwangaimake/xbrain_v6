// good: whitelist is a compile-time constant, unreachable from config.
static const char* const kAllowed[] = {"rt/safety/estop", "rt/chassis/ctrl"};
bool allowed(const char* key);
// good: the two env vars registered in the CRL-3 rule table. NOTIFY_SOCKET is
// the systemd sd_notify protocol socket -- dropping it switches off the
// watchdog on a process that sits on the estop path. XBRAIN_ROBOT_ID is the
// documented L5 override for the rid, the same mechanism the Python stack
// uses. Neither reads a config file, and neither is the whitelist.
bool notify_ready() { return std::getenv("NOTIFY_SOCKET") != nullptr; }
const char* rid_override() { return std::getenv("XBRAIN_ROBOT_ID"); }
