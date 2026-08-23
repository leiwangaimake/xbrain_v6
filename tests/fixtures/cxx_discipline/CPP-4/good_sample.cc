// good: lock-free guard, skip the tick instead of blocking.
#include <atomic>
static std::atomic_flag tx_guard = ATOMIC_FLAG_INIT;
