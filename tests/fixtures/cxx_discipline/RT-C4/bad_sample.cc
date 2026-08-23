// bad: quadruped opening a session on the general plane.
static const char* kEndpoint = "tcp/127.0.0.1:7447";
void connect() { open_session(kEndpoint); }
