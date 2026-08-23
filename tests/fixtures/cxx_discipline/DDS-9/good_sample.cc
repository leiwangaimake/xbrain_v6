// good: the three entities report their real domain ids.
void on_start() { hello_ack.runtime.transport = describe_domains(); }
