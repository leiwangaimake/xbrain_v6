// good: gossip on (a peer's subscription table reaches a remote publisher
// only by gossip through the router), multihop off (RT-C2).
const char* cfg() { return "scouting:{gossip:{enabled:true,multihop:false}}"; }
