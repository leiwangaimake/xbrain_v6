/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: parallel_dispatch_trace.hpp
 * Brief: ParallelTaskWorker: the helper lane that runs one of two tasks beside the caller.
 *
 * Description:
 * The inference module runs both letterbox preprocesses and both branch postprocesses on it
 * (helper plus caller). Same contract as persistent_task_pair.hpp: both tasks finish before any
 * exception is rethrown; the caller's exception wins unless Helper priority is asked for.
 */

#pragma once

#include "common/thread_name.hpp"
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "common/execution_cpu_trace.hpp"

namespace perception {

// Extracted from InferenceModule verbatim apart from the optional join-wait
// execution CPU record (PERCEPTION_EXECUTION_CPU_TRACE). The mutex/CV lifetime,
// notification order, callback order and exception precedence are unchanged.
// Production and std::thread tests share this class.
class ParallelTaskWorker {
public:
    enum class ExceptionPriority { Caller, Helper };
    ParallelTaskWorker() {
        worker_ = std::thread(&ParallelTaskWorker::worker_loop, this);
    }
    ~ParallelTaskWorker() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        task_condition_.notify_one();
        if (worker_.joinable()) worker_.join();
    }
    ParallelTaskWorker(const ParallelTaskWorker&) = delete;
    ParallelTaskWorker& operator=(const ParallelTaskWorker&) = delete;

    void run(std::function<void()> helper_task, std::function<void()> caller_task,
             ExecutionCpuRecord* join_trace = nullptr,
             ExceptionPriority exception_priority = ExceptionPriority::Caller) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ || task_ready_ || !task_done_)
                throw std::logic_error("parallel task worker is not idle");
            task_ = std::move(helper_task);
            helper_exception_ = nullptr;
            task_done_ = false;
            task_ready_ = true;
        }
        task_condition_.notify_one();

        std::exception_ptr caller_exception;
        try {
            caller_task();
        } catch (...) {
            caller_exception = std::current_exception();
        }

        std::exception_ptr helper_exception;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (join_trace) {
                ExecutionCpuScope scope(*join_trace, true);
                done_condition_.wait(lock, [this]() { return task_done_; });
            } else {
                done_condition_.wait(lock, [this]() { return task_done_; });
            }
            helper_exception = helper_exception_;
        }
        // Logical task priority can be preserved when an explicit diagnostic
        // swaps thread roles. Both callbacks have completed before any rethrow.
        if (exception_priority == ExceptionPriority::Helper && helper_exception != nullptr)
            std::rethrow_exception(helper_exception);
        if (caller_exception != nullptr) std::rethrow_exception(caller_exception);
        if (helper_exception != nullptr) std::rethrow_exception(helper_exception);
    }

private:
    void worker_loop() noexcept {
        set_current_thread_name("par_helper");
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                task_condition_.wait(lock, [this]() { return stopping_ || task_ready_; });
                if (stopping_ && !task_ready_) return;
                task = std::move(task_);
                task_ready_ = false;
            }
            std::exception_ptr task_exception;
            try {
                task();
            } catch (...) {
                task_exception = std::current_exception();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                helper_exception_ = task_exception;
                task_done_ = true;
            }
            done_condition_.notify_one();
        }
    }
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable task_condition_;
    std::condition_variable done_condition_;
    std::function<void()> task_;
    std::exception_ptr helper_exception_;
    bool task_ready_ = false;
    bool task_done_ = true;
    bool stopping_ = false;
};

} // namespace perception
