// bad: the gossip clause is gone, so zenoh picks its own default. Omitted
// and explicitly-set-to-the-default read identically in the source.
const char* cfg() { return "scouting:{multicast:{enabled:false}}"; }
