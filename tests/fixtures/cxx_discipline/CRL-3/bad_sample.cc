// bad: whitelist read from a config file -- CRL-3 forbids this.
#include <yaml-cpp/yaml.h>
void load() { YAML::Node n = YAML::LoadFile("/opt/xbrain_v6/configs/relay.yaml"); }
