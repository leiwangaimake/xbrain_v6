// bad: no transport self-report at startup; a mis-wired domain is
// indistinguishable from a dead network.
void on_start() { start_readers(); }
