// bad: a mutex makes the realtime side block on a lock.
#include <mutex>
static std::mutex tx_guard;
