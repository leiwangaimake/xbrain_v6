// bad: multihop flipped to true -- the one condition the 2026-08-23 RT-C2
// correction rests on. Plane isolation is gone and nothing misbehaves.
const char* cfg() { return "scouting:{gossip:{enabled:true,multihop:true}}"; }
