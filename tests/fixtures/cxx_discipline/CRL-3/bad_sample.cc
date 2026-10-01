// bad: whitelist read from a config file -- CRL-3 forbids this.
#include <yaml-cpp/yaml.h>
void load() { YAML::Node n = YAML::load_file("/opt/xbrain_v6/configs/relay.yaml"); }
// bad: an env var the rule table has NOT registered. Narrowing the getenv
// clause to the two registered names must not open this door.
const char* keys() { return std::getenv("XBRAIN_WHITELIST"); }
// bad: argument is a variable, so the name cannot be checked -- the lookahead
// only spares string literals, so this still reports. Direction is toward
// reporting, deliberately.
const char* indirect(const char* name) { return getenv(name); }
