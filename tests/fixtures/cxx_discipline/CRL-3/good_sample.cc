// good: whitelist is a compile-time constant, unreachable from config.
static const char* const kAllowed[] = {"rt/safety/estop", "rt/chassis/ctrl"};
bool allowed(const char* key);
