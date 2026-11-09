/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: module_base.hpp
 * Brief: Base interface for long-running detector pipeline modules.
 *
 * Description:
 *   1. Defines common start/stop lifecycle operations.
 *   2. Tracks running state and heartbeat timestamps for supervision.
 * In R6 the Supervisor heartbeat checks is_running() (and the encoder's failed()); the
 * heartbeat timestamp is still updated by the modules but nothing reads it.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

namespace yolo {

/**
 * @brief Abstract base class for capture, inference, and result modules.
 */
class ModuleBase {
public:
    /** @brief Creates a module with a human-readable name. */
    explicit ModuleBase(std::string name);

    /** @brief Ensures derived modules can be destroyed polymorphically. */
    virtual ~ModuleBase() = default;

    /** @brief Starts the module worker thread or resource loop. */
    virtual bool start() = 0;

    /** @brief Stops the module and releases owned runtime resources. */
    virtual void stop() = 0;

    /** @brief Returns whether the module is currently marked as running. */
    virtual bool is_running() const;

    /** @brief Updates the module heartbeat timestamp to now. */
    virtual void update_heartbeat();

    /** @brief Returns the last heartbeat timestamp. */
    virtual std::chrono::steady_clock::time_point get_last_heartbeat() const;

    /** @brief Returns the module name used in logs and diagnostics. */
    virtual std::string name() const;

protected:
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::string name_;

private:
    mutable std::mutex heartbeat_mutex_;
};

}  // namespace yolo
