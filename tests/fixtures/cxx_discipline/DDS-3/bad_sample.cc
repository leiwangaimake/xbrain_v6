// bad: process-wide env var ties both domains to one config.
#include <cstdlib>
void setup() { setenv("CYCLONEDDS_URI", "file:///etc/cyclone.xml", 1); }
