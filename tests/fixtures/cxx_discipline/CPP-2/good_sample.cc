// good: entry is noexcept, everything throwable is wrapped inside.
void ctrl_thread_main() noexcept { try { run_loop(); } catch (...) {} }
