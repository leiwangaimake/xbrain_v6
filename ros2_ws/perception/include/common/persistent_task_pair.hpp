/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: persistent_task_pair.hpp
 * Brief: One persistent background thread paired with the calling thread.
 *
 * Description:
 * Used by the inference module for the JIT submit lane (async submit, and the empty-acquire
 * start/join form) and for the two Active12 mask lanes (parallel2). The thread lives as long as
 * the object, so no thread is created per frame. Exceptions are rethrown only after both sides
 * finished, so no caller can free resources a still-running callback uses.
 */

#pragma once

#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace perception {

// One persistent background lane plus the calling lane. run() never returns or
// propagates an exception until both callbacks have finished. Call run() from
// one owner only; stop() joins the background thread, including a pending task.
class PersistentTaskPair {
public:
    explicit PersistentTaskPair(std::function<void()> initialize = {}) {
        // All mutex/CV/state members are already constructed before launch.
        worker_ = std::thread([this, initialize = std::move(initialize)]() noexcept {
            std::exception_ptr error;
            try {
                if (initialize) initialize();
            } catch (...) {
                error = std::current_exception();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                initialization_error_ = error;
                initialized_ = true;
            }
            done_condition_.notify_all();
            if (error) return;
            worker_loop();
        });
        std::unique_lock<std::mutex> lock(mutex_);
        done_condition_.wait(lock, [this]() { return initialized_; });
        const auto error = initialization_error_;
        if (error) {
            lock.unlock();
            worker_.join();
            std::rethrow_exception(error);
        }
    }

    ~PersistentTaskPair() noexcept { stop(); }
    PersistentTaskPair(const PersistentTaskPair&) = delete;
    PersistentTaskPair& operator=(const PersistentTaskPair&) = delete;

    void run(std::function<void()> background, std::function<void()> foreground) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ || pending_ || !done_) {
                throw std::logic_error("persistent task pair is not idle");
            }
            task_ = std::move(background);
            background_error_ = nullptr;
            done_ = false;
            pending_ = true;
        }
        task_condition_.notify_one();
        std::exception_ptr foreground_error;
        try {
            foreground();
        } catch (...) {
            foreground_error = std::current_exception();
        }
        std::exception_ptr background_error;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            done_condition_.wait(lock, [this]() { return done_; });
            background_error = background_error_;
        }
        // As with the existing CPU postprocess helper, foreground wins if both
        // fail. No caller can drain/free background-owned resources prematurely.
        if (foreground_error) std::rethrow_exception(foreground_error);
        if (background_error) std::rethrow_exception(background_error);
    }

    // Asynchronous form of run(): dispatch only the background callback. The
    // owner must call join() exactly once before any other start()/run(); join
    // returns only after the background callback finished and rethrows its error.
    void start(std::function<void()> background) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ || pending_ || !done_) {
                throw std::logic_error("persistent task pair is not idle");
            }
            task_ = std::move(background);
            background_error_ = nullptr;
            done_ = false;
            pending_ = true;
        }
        task_condition_.notify_one();
    }

    void join() {
        std::exception_ptr background_error;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            done_condition_.wait(lock, [this]() { return done_; });
            background_error = background_error_;
            background_error_ = nullptr;
        }
        if (background_error) std::rethrow_exception(background_error);
    }

    bool idle() {
        std::lock_guard<std::mutex> lock(mutex_);
        return done_ && !pending_;
    }

    void stop() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        task_condition_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    void worker_loop() noexcept {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                task_condition_.wait(lock, [this]() { return stopping_ || pending_; });
                if (stopping_ && !pending_) return;
                task = std::move(task_);
                pending_ = false;
            }
            std::exception_ptr error;
            try {
                task();
            } catch (...) {
                error = std::current_exception();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                background_error_ = error;
                done_ = true;
            }
            done_condition_.notify_all();
        }
    }

    std::mutex mutex_;
    std::condition_variable task_condition_;
    std::condition_variable done_condition_;
    std::function<void()> task_;
    std::exception_ptr initialization_error_;
    std::exception_ptr background_error_;
    bool initialized_ = false;
    bool stopping_ = false;
    bool pending_ = false;
    bool done_ = true;
    std::thread worker_; // Last, and started in the constructor body.
};

} // namespace perception
