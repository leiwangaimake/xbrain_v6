/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: thread_name.hpp
 * Brief: Observability only: a stable kernel thread name for each owned worker thread.
 *
 * Description:
 * The name lets per-thread CPU telemetry (/proc/<pid>/task/<tid>/comm) be attributed, e.g.
 * cap_loop, sup_process, depth_lane2, hevc_worker.
 * No scheduling, priority or affinity change. Names are truncated to 15 bytes.
 */

#pragma once

#include <cstring>
#ifdef __linux__
#include <pthread.h>
#endif

namespace perception {

inline void set_current_thread_name(const char* name) noexcept {
#ifdef __linux__
    if (name == nullptr) return;
    char buffer[16];
    std::strncpy(buffer, name, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    (void)pthread_setname_np(pthread_self(), buffer);
#else
    (void)name;
#endif
}

}  // namespace perception
