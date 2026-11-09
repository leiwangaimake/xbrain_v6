/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: threadsafe_queue.hpp
 * Brief: Bounded thread-safe queue for pipeline modules.
 *
 * Description:
 *   1. Provides blocking pop with timeout and non-blocking try_pop.
 *   2. Limits queue growth by dropping the oldest item when full.
 *   3. kLatestWinsSwap: a one-item latest-wins handoff (19 P19-2). The writer
 *      swaps an owned pointer in; an older unconsumed item is overwritten in
 *      place, never queued. The reader takes the whole item by swapping the
 *      pointer out, so it can never observe a half-written item. The only lock
 *      is the empty handshake that keeps a waiting reader from missing a wakeup.
 *   4. Accounting in both policies: pushed == popped + dropped + cleared + size()
 *      whenever no push/pop is in flight; an optional observer sees every item
 *      that is dropped (overwritten) or cleared, with the reason, before it is
 *      destroyed.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <utility>

namespace yolo {

struct QueueUpdateWaitInfo {
    bool wait_requested = false;
    bool updated = false;
    bool cleared = false;
};

/** @brief Why an item left the queue without being popped. */
enum class QueueDiscard {
    kOverwritten,  // dropped by push(): FIFO full, or latest-wins replaced
    kCleared,      // discarded by clear()
};

/** @brief How a full queue treats a new item. */
enum class QueuePolicy {
    kDropOldestFifo,   // bounded FIFO; a full queue drops its oldest item
    kLatestWinsSwap,   // one-item pointer swap; a new item overwrites the unconsumed one
};

/**
 * @brief Bounded FIFO queue shared by capture, inference, and result modules.
 *
 * The queue is header-only because it is a template. When max_size_ is greater
 * than zero, push() keeps the newest data by removing the oldest entry if the
 * queue is already full.
 */
template <typename T>
class ThreadsafeQueue {
public:
    /** @brief Creates a queue with an optional maximum size. */
    explicit ThreadsafeQueue(std::size_t max_size = 0, QueuePolicy policy = QueuePolicy::kDropOldestFifo)
        : max_size_(policy == QueuePolicy::kLatestWinsSwap ? 1 : max_size), policy_(policy) {}

    ~ThreadsafeQueue() { delete slot_.exchange(nullptr, std::memory_order_acq_rel); }
    ThreadsafeQueue(const ThreadsafeQueue&) = delete;
    ThreadsafeQueue& operator=(const ThreadsafeQueue&) = delete;

    QueuePolicy policy() const { return policy_; }

    /**
     * @brief Sees every dropped or cleared item just before it is destroyed.
     * Set it before any concurrent use. Drops run on the pushing thread (inside
     * the queue lock for the FIFO policy), clears on the clearing thread after
     * the lock is released; it must be short and not re-enter this queue.
     */
    void set_discard_observer(std::function<void(const T&, QueueDiscard)> observer) {
        discard_observer_ = std::move(observer);
    }

    /** @brief Pushes an item and wakes one waiting consumer. */
    void push(const T& item) {
        if (policy_ == QueuePolicy::kLatestWinsSwap) {
            swap_in(new T(item));
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pushed_count_.fetch_add(1);
            if (max_size_ > 0 && queue_.size() >= max_size_) {
                drop_front();
            }
            queue_.push(item);
        }
        cv_.notify_one();
    }

    /** @brief Moves an item into the queue and wakes one waiting consumer. */
    void push(T&& item) {
        if (policy_ == QueuePolicy::kLatestWinsSwap) {
            swap_in(new T(std::move(item)));
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pushed_count_.fetch_add(1);
            if (max_size_ > 0 && queue_.size() >= max_size_) {
                drop_front();
            }
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    /**
     * @brief Pops one item, waiting up to the specified timeout.
     * @return true if an item was written to output, false on timeout.
     */
    bool pop(T& output, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        const uint64_t wake_generation = wake_generation_;
        if (policy_ == QueuePolicy::kLatestWinsSwap) {
            if (!cv_.wait_for(lock, timeout, [this, wake_generation]() {
                    return slot_.load(std::memory_order_acquire) != nullptr || wake_generation_ != wake_generation;
                })) {
                return false;
            }
            T* taken = slot_.exchange(nullptr, std::memory_order_acq_rel);
            lock.unlock();
            return take_out(taken, output);
        }
        if (!cv_.wait_for(lock, timeout, [this, wake_generation]() {
                return !queue_.empty() || wake_generation_ != wake_generation;
            })) {
            return false;
        }
        if (queue_.empty()) {
            return false;
        }

        output = std::move(queue_.front());
        queue_.pop();
        popped_count_.fetch_add(1);
        return true;
    }

    /**
     * @brief Attempts to pop one item without blocking.
     * @return true if an item was available, false if the queue was empty.
     */
    bool try_pop(T& output) {
        if (policy_ == QueuePolicy::kLatestWinsSwap) {
            return take_out(slot_.exchange(nullptr, std::memory_order_acq_rel), output);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }

        output = std::move(queue_.front());
        queue_.pop();
        popped_count_.fetch_add(1);
        return true;
    }

    /**
     * @brief Optionally waits for replacement of an existing front candidate.
     *
     * Empty queues return immediately. Both callbacks run under the queue lock;
     * should_wait is called once for the initial candidate, while key_of may
     * be called repeatedly and must return a stable, copyable identity key.
     * On timeout, pop the front present after reacquiring the lock (the old
     * candidate if not replaced). updated means its key changed, not that the
     * replacement arrived before the deadline. clear() cancels this operation,
     * even if a subsequent push has already filled the queue again. No timeout
     * is renewed by a wakeup. The deadline is a requested wait bound, not a
     * guarantee about OS scheduling or mutex reacquisition latency.
     * The latest-wins policy has no front candidate to wait on: it behaves as
     * try_pop() and reports no wait.
     */
    template <typename ShouldWait, typename KeyOf>
    bool try_pop_after_update_until(
        T& output,
        std::chrono::steady_clock::time_point deadline,
        ShouldWait should_wait,
        KeyOf key_of,
        QueueUpdateWaitInfo& info) {
        info = QueueUpdateWaitInfo{};
        if (policy_ == QueuePolicy::kLatestWinsSwap) {
            return try_pop(output);
        }
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        const auto initial_key = key_of(static_cast<const T&>(queue_.front()));
        const uint64_t wake_generation = wake_generation_;
        const bool requested = should_wait(static_cast<const T&>(queue_.front()));
        if (requested && std::chrono::steady_clock::now() < deadline) {
            info.wait_requested = true;
            cv_.wait_until(lock, deadline, [&]() {
                return wake_generation_ != wake_generation || queue_.empty() ||
                    key_of(static_cast<const T&>(queue_.front())) != initial_key;
            });
        }
        if (wake_generation_ != wake_generation) {
            info.cleared = true;
            return false;
        }
        if (queue_.empty()) {
            return false;
        }
        info.updated = key_of(static_cast<const T&>(queue_.front())) != initial_key;
        output = std::move(queue_.front());
        queue_.pop();
        popped_count_.fetch_add(1);
        return true;
    }

    /** @brief Returns the current number of queued items. */
    std::size_t size() const {
        if (policy_ == QueuePolicy::kLatestWinsSwap) {
            return slot_.load(std::memory_order_acquire) != nullptr ? 1U : 0U;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /** @brief Returns true when the queue has no items. */
    bool empty() const { return size() == 0U; }

    /** @brief Returns the number of items dropped (FIFO: full queue; latest-wins: overwritten). */
    uint64_t dropped_count() const { return dropped_count_.load(); }
    /** @brief Items accepted by push(). */
    uint64_t pushed_count() const { return pushed_count_.load(); }
    /** @brief Items handed to a consumer by pop(), try_pop() or try_pop_after_update_until(). */
    uint64_t popped_count() const { return popped_count_.load(); }
    /** @brief Items discarded by clear(). */
    uint64_t cleared_count() const { return cleared_count_.load(); }

    /** @brief Removes all queued items. */
    void clear() {
        T* discarded = nullptr;
        std::queue<T> discarded_fifo;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (policy_ == QueuePolicy::kLatestWinsSwap) {
                discarded = slot_.exchange(nullptr, std::memory_order_acq_rel);
                if (discarded != nullptr) cleared_count_.fetch_add(1);
            } else {
                cleared_count_.fetch_add(queue_.size());
                queue_.swap(discarded_fifo);
            }
            ++wake_generation_;
        }
        cv_.notify_all();
        if (discard_observer_) {
            if (discarded != nullptr) discard_observer_(*discarded, QueueDiscard::kCleared);
            for (; !discarded_fifo.empty(); discarded_fifo.pop())
                discard_observer_(discarded_fifo.front(), QueueDiscard::kCleared);
        }
        delete discarded;
    }

private:
    // FIFO policy, called with mutex_ held and the queue full.
    void drop_front() {
        dropped_count_.fetch_add(1);
        if (discard_observer_) discard_observer_(queue_.front(), QueueDiscard::kOverwritten);
        queue_.pop();
    }

    // Latest-wins policy: publish `fresh` (owned) and retire what it replaced.
    void swap_in(T* fresh) {
        pushed_count_.fetch_add(1);
        T* replaced = slot_.exchange(fresh, std::memory_order_acq_rel);
        if (replaced != nullptr) {
            dropped_count_.fetch_add(1);
            if (discard_observer_) discard_observer_(*replaced, QueueDiscard::kOverwritten);
            delete replaced;
        }
        // Empty critical section: a reader that checked the slot under the lock
        // either saw `fresh` or is already waiting, so the notify cannot be lost.
        { std::lock_guard<std::mutex> handshake(mutex_); }
        cv_.notify_one();
    }

    // Latest-wins policy: the caller already swapped `taken` out of the slot.
    bool take_out(T* taken, T& output) {
        if (taken == nullptr) return false;
        output = std::move(*taken);
        delete taken;
        popped_count_.fetch_add(1);
        return true;
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<T> queue_;
    std::size_t max_size_;
    const QueuePolicy policy_;
    std::atomic<T*> slot_{nullptr};  // kLatestWinsSwap only
    std::function<void(const T&, QueueDiscard)> discard_observer_;
    std::atomic<uint64_t> dropped_count_{0};
    std::atomic<uint64_t> pushed_count_{0};
    std::atomic<uint64_t> popped_count_{0};
    std::atomic<uint64_t> cleared_count_{0};
    uint64_t wake_generation_ = 0;
};

}  // namespace yolo
