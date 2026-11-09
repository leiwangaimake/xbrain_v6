/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: module_base.cpp
 * Brief: Common module lifecycle state implementation.
 *
 * Description:
 *   1. Stores running state and heartbeat timestamps.
 *   2. Exposes shared diagnostic methods for supervisor checks.
 */

#include "modules/module_base.hpp"

#include <utility>

namespace yolo {

ModuleBase::ModuleBase(std::string name)
    : running_(false), last_heartbeat_(std::chrono::steady_clock::now()), name_(std::move(name)) {}

bool ModuleBase::is_running() const {
    return running_.load();
}

void ModuleBase::update_heartbeat() {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_heartbeat_ = std::chrono::steady_clock::now();
}

std::chrono::steady_clock::time_point ModuleBase::get_last_heartbeat() const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    return last_heartbeat_;
}

std::string ModuleBase::name() const {
    return name_;
}

}  // namespace yolo
