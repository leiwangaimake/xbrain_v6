// bad: realtime thread entry can throw; one escape kills the process.
void ctrl_thread_main() { run_loop(); }
