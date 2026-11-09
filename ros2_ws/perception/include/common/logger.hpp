/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: logger.hpp
 * Brief: Header-only structured logger for detector modules.
 *
 * Description:
 *   1. Provides stream-style LOG_ERROR/LOG_WARN/LOG_INFO/LOG_DEBUG macros.
 *   2. Supports runtime log levels, module names, and thread-safe output.
 *   3. PERCEPTION_ASYNC_LOG=1 (runtime switch async_log, production "1"): lines are formatted at
 *      the call site and written by one background thread (log_writer), so a slow stdout never
 *      blocks a pipeline thread; ERROR lines wait at most 50 ms for the queue, then go to fd 2
 *      directly. The queue is bounded (64 MB; one line at most 1 MB, the rest cut and marked).
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cerrno>
#include <unistd.h>

#if defined(__linux__)
#include <pthread.h>
#endif

namespace yolo {

/** @brief Runtime severity threshold used by the process-wide logger. */
enum class LogLevel {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3
};

/**
 * @brief Thread-safe process-wide logger with compact structured formatting.
 *
 * Logger is header-only so the stream-style macros can remain dependency-free
 * and existing call sites do not need a concrete logger object. Messages are
 * emitted only when their severity is enabled by the current global threshold.
 */
class Logger {
public:
    /** @brief Returns the singleton logger instance used by all macros. */
    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    /** @brief Sets the global minimum log level by enum value. */
    void set_level(LogLevel level) {
        level_.store(level);
    }

    /**
     * @brief Sets the global minimum log level from text.
     * @return true when the text matched ERROR, WARN, INFO, or DEBUG.
     */
    bool set_level(const std::string& level_text) {
        LogLevel parsed_level;
        if (!parse_level(level_text, parsed_level)) {
            return false;
        }

        set_level(parsed_level);
        return true;
    }

    /** @brief Returns the currently configured global minimum log level. */
    LogLevel level() const {
        return level_.load();
    }

    /** @brief Returns true when a message at message_level should be emitted. */
    bool should_log(LogLevel message_level) const {
        return static_cast<int>(message_level) <= static_cast<int>(level_.load());
    }

    /**
     * @brief Emits one fully formatted log line.
     *
     * The output stream is selected by severity: ERROR goes to stderr and the
     * remaining levels go to stdout. The mutex keeps each formatted line intact
     * even when multiple module threads write logs concurrently.
     */
    void log(LogLevel message_level, const std::string& module, const std::string& message) {
        if (static_cast<int>(message_level) > static_cast<int>(level_.load())) {
            return;
        }

        if (!async_) {
            std::lock_guard<std::mutex> lock(mutex_);
            std::ostream& output = (message_level == LogLevel::ERROR) ? std::cerr : std::cout;
            output << "[" << timestamp() << "] "
                   << "[" << level_name(message_level) << "] "
                   << "[" << module << "] "
                   << "[" << std::this_thread::get_id() << "] "
                   << message << std::endl;
            if (!output) {  // surfaced, never silent; the next line tries again
                sync_write_failures_.fetch_add(1, std::memory_order_relaxed);
                output.clear();
            }
            return;
        }

        // PERCEPTION_ASYNC_LOG=1: identical line text, timestamp and thread id taken here at the
        // call site; stdout lines are written by one background writer so a slow file/pipe write
        // never blocks a pipeline thread. ERROR lines first drain the queue (ordering and crash
        // context), then go to stderr synchronously exactly as before.
        std::ostringstream line;
        line << "[" << timestamp() << "] "
             << "[" << level_name(message_level) << "] "
             << "[" << module << "] "
             << "[" << std::this_thread::get_id() << "] "
             << message << '\n';
        if (message_level == LogLevel::ERROR) {
            // Preserve order when the writer keeps up, but never let a blocked
            // stdout (full pipe, stalled journal) hold an ERROR back: bounded wait,
            // then stderr under its own lock (the writer holds mutex_ while writing).
            if (flush_for(kErrorOrderWait) == FlushResult::TimedOut) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                ++error_order_timeouts_;
            }
            // Raw fd 2 write: std::cerr is tied to std::cout (it would flush the
            // blocked stdout first) and stdio locks are shared with the writer.
            const std::string text = line.str();
            bool ok = true;
            {
                std::lock_guard<std::mutex> lock(stderr_mutex_);
                std::size_t done = 0;
                while (done < text.size()) {
                    const ssize_t n = ::write(STDERR_FILENO, text.data() + done, text.size() - done);
                    if (n > 0) { done += static_cast<std::size_t>(n); continue; }
                    if (n < 0 && errno == EINTR) continue;
                    ok = false;
                    break;
                }
            }
            if (!ok) {
                std::lock_guard<std::mutex> stats_lock(queue_mutex_);
                ++write_failures_;
            }
            return;
        }
        enqueue(line.str());
    }

    enum class FlushResult { Drained, TimedOut, WriteFailed };
    /**
     * @brief Waits at most timeout for the queue to drain.
     * WriteFailed means drained but at least one line/flush failed since start
     * (lines were dropped and counted); synchronous mode reports failures only.
     */
    FlushResult flush_for(std::chrono::milliseconds timeout) {
        if (!async_) {
            return sync_write_failures_.load(std::memory_order_relaxed) ? FlushResult::WriteFailed
                                                                        : FlushResult::Drained;
        }
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (!drained_cv_.wait_for(lock, timeout, [this]() { return queue_.empty() && !writing_; }))
            return FlushResult::TimedOut;
        return write_failures_ ? FlushResult::WriteFailed : FlushResult::Drained;
    }

    /** @brief Blocks until the queue drained; false when any write failed (unbounded; prefer flush_for). */
    bool flush() {
        if (!async_) return sync_write_failures_.load(std::memory_order_relaxed) == 0;
        std::unique_lock<std::mutex> lock(queue_mutex_);
        drained_cv_.wait(lock, [this]() { return queue_.empty() && !writing_; });
        return write_failures_ == 0;
    }

    struct AsyncStats {
        bool enabled = false;
        std::uint64_t lines = 0;
        std::uint64_t blocked_enqueues = 0;
        std::uint64_t max_pending_bytes = 0;
        std::uint64_t writer_batches = 0;
        std::uint64_t truncated_lines = 0;
        std::uint64_t write_failures = 0;
        std::uint64_t failed_batch_lines = 0;  // upper bound of lines possibly lost
        std::uint64_t error_order_timeouts = 0;
    };
    AsyncStats async_stats() const {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return {async_, lines_, blocked_enqueues_, max_pending_bytes_, writer_batches_, truncated_lines_,
                write_failures_ + sync_write_failures_.load(std::memory_order_relaxed), failed_batch_lines_,
                error_order_timeouts_};
    }

    /** @brief Converts a log level enum to the fixed output label. */
    static const char* level_name(LogLevel level) {
        switch (level) {
            case LogLevel::ERROR:
                return "ERROR";
            case LogLevel::WARN:
                return "WARN";
            case LogLevel::INFO:
                return "INFO";
            case LogLevel::DEBUG:
                return "DEBUG";
        }

        return "INFO";
    }

    /** @brief Parses common uppercase or lowercase text into a log level. */
    static bool parse_level(const std::string& level_text, LogLevel& output_level) {
        std::string normalized = level_text;
        std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
            return static_cast<char>(std::toupper(ch));
        });

        if (normalized == "ERROR") {
            output_level = LogLevel::ERROR;
            return true;
        }
        if (normalized == "WARN" || normalized == "WARNING") {
            output_level = LogLevel::WARN;
            return true;
        }
        if (normalized == "INFO") {
            output_level = LogLevel::INFO;
            return true;
        }
        if (normalized == "DEBUG") {
            output_level = LogLevel::DEBUG;
            return true;
        }

        return false;
    }

private:
    /** @brief Creates a logger with INFO threshold by default. */
    Logger() : level_(LogLevel::INFO) {
        const char* async_value = std::getenv("PERCEPTION_ASYNC_LOG");
        async_ = async_value != nullptr && std::string(async_value) == "1";
        if (async_) {
            writer_ = std::thread([this]() { writer_loop(); });
        }
    }

    ~Logger() {
        if (!async_) return;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stopping_ = true;
        }
        queue_cv_.notify_all();
        if (writer_.joinable()) writer_.join();
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static constexpr std::size_t kMaxPendingBytes = 64U * 1024U * 1024U;
    // One line never exceeds this; the excess is cut with an explicit marker, so
    // a single oversize message can always be admitted (no unsatisfiable wait).
    static constexpr std::size_t kMaxLineBytes = 1024U * 1024U;
    static constexpr std::chrono::milliseconds kErrorOrderWait{50};

    void enqueue(std::string line) {
        bool truncated = false;
        if (line.size() > kMaxLineBytes) {
            const std::string marker = " ...[truncated " + std::to_string(line.size() - (kMaxLineBytes - 64)) + " bytes]\n";
            line.resize(kMaxLineBytes - 64);
            line += marker;
            truncated = true;
        }
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (truncated) ++truncated_lines_;
        if (pending_bytes_ + line.size() > kMaxPendingBytes) {
            ++blocked_enqueues_;
            space_cv_.wait(lock, [&]() {
                return pending_bytes_ == 0 || pending_bytes_ + line.size() <= kMaxPendingBytes || stopping_;
            });
        }
        pending_bytes_ += line.size();
        if (pending_bytes_ > max_pending_bytes_) max_pending_bytes_ = pending_bytes_;
        ++lines_;
        const bool was_empty = queue_.empty();
        queue_.push_back(std::move(line));
        lock.unlock();
        if (was_empty) queue_cv_.notify_one();
    }

    void writer_loop() {
#if defined(__linux__)
        pthread_setname_np(pthread_self(), "log_writer");
#endif
        std::vector<std::string> batch;
        for (;;) {
            std::size_t batch_bytes = 0;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]() { return !queue_.empty() || stopping_; });
                if (queue_.empty() && stopping_) return;
                batch.swap(queue_);
                for (const auto& l : batch) batch_bytes += l.size();
                writing_ = true;
                ++writer_batches_;
            }
            std::uint64_t failures = 0, failed_batch_lines = 0;
            {
                std::lock_guard<std::mutex> out_lock(mutex_);
                bool batch_failed = false;
                for (const auto& l : batch) {
                    std::cout.write(l.data(), static_cast<std::streamsize>(l.size()));
                    if (!std::cout) { ++failures; batch_failed = true; std::cout.clear(); }
                }
                std::cout.flush();
                if (!std::cout) { ++failures; batch_failed = true; std::cout.clear(); }
                // Buffered stdio cannot say which lines of a failed batch reached
                // the fd: count the whole batch (upper bound of lost lines).
                if (batch_failed) failed_batch_lines = batch.size();
            }
            batch.clear();
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pending_bytes_ -= batch_bytes;
                write_failures_ += failures;
                failed_batch_lines_ += failed_batch_lines;
                writing_ = false;
            }
            space_cv_.notify_all();
            drained_cv_.notify_all();
        }
    }

    /** @brief Formats the local wall-clock time as HH:MM:SS.mmm. */
    static std::string timestamp() {
        // LINT-ALLOW(clock): wall clock only for the human-readable log line (19 section 10 allows display use)
        const auto now = std::chrono::system_clock::now();
        const auto now_time = std::chrono::system_clock::to_time_t(now);  // LINT-ALLOW(clock): same log line
        const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now.time_since_epoch()) %
                            1000;

        std::tm local_time {};
#if defined(_WIN32)
        localtime_s(&local_time, &now_time);
#else
        localtime_r(&now_time, &local_time);
#endif

        std::ostringstream stream;
        stream << std::put_time(&local_time, "%H:%M:%S")
               << "." << std::setw(3) << std::setfill('0') << millis.count();
        return stream.str();
    }

    mutable std::mutex mutex_;
    std::mutex stderr_mutex_;
    std::atomic<std::uint64_t> sync_write_failures_{0};
    std::atomic<LogLevel> level_;
    bool async_ = false;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::condition_variable space_cv_;
    std::condition_variable drained_cv_;
    std::vector<std::string> queue_;
    std::size_t pending_bytes_ = 0;
    bool writing_ = false;
    bool stopping_ = false;
    std::uint64_t lines_ = 0;
    std::uint64_t blocked_enqueues_ = 0;
    std::uint64_t max_pending_bytes_ = 0;
    std::uint64_t writer_batches_ = 0;
    std::uint64_t truncated_lines_ = 0;
    std::uint64_t write_failures_ = 0;
    std::uint64_t failed_batch_lines_ = 0;
    std::uint64_t error_order_timeouts_ = 0;
    std::thread writer_;
};

}  // namespace yolo

/** @brief Internal helper for stream-style log macro expansion. */
#define YOLO_LOG_IMPL(level, module, message)                         \
    do {                                                              \
        if (::yolo::Logger::instance().should_log(level)) {            \
            std::ostringstream yolo_log_module_stream;                \
            std::ostringstream yolo_log_message_stream;               \
            yolo_log_module_stream << module;                         \
            yolo_log_message_stream << message;                       \
            ::yolo::Logger::instance().log(                           \
                level,                                                \
                yolo_log_module_stream.str(),                         \
                yolo_log_message_stream.str());                       \
        }                                                             \
    } while (false)

/** @brief Writes an error message with the default Main module name. */
#define LOG_ERROR(message) YOLO_LOG_IMPL(::yolo::LogLevel::ERROR, "Main", message)

/** @brief Writes a warning message with the default Main module name. */
#define LOG_WARN(message) YOLO_LOG_IMPL(::yolo::LogLevel::WARN, "Main", message)

/** @brief Writes an informational message with the default Main module name. */
#define LOG_INFO(message) YOLO_LOG_IMPL(::yolo::LogLevel::INFO, "Main", message)

/** @brief Writes a debug message with the default Main module name. */
#define LOG_DEBUG(message) YOLO_LOG_IMPL(::yolo::LogLevel::DEBUG, "Main", message)

/** @brief Writes an error message with an explicit module name. */
#define LOG_ERROR_M(module, message) YOLO_LOG_IMPL(::yolo::LogLevel::ERROR, module, message)

/** @brief Writes a warning message with an explicit module name. */
#define LOG_WARN_M(module, message) YOLO_LOG_IMPL(::yolo::LogLevel::WARN, module, message)

/** @brief Writes an informational message with an explicit module name. */
#define LOG_INFO_M(module, message) YOLO_LOG_IMPL(::yolo::LogLevel::INFO, module, message)

/** @brief Writes a debug message with an explicit module name. */
#define LOG_DEBUG_M(module, message) YOLO_LOG_IMPL(::yolo::LogLevel::DEBUG, module, message)
