/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_post_role_dispatch.cpp
 * Brief: Unit tests for swapping the COCO/active postprocess roles between the two JIT worker threads.
 *
 * Description:
 * What it checks. The post-role dispatch can run the active-model postprocess on the helper and
 * the COCO one on the caller, or the other way round. The tests check the configuration switch,
 * that both roles run concurrently in either assignment, that exceptions propagate with the active
 * role taking priority and the worker stays reusable, and that the switched-off path is equivalent
 * to the original single-role behaviour.
 */

#include "common/post_role_dispatch.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {
int checks = 0;
void check(bool value) {
    ++checks;
    if (!value) throw std::runtime_error("post role dispatch check " + std::to_string(checks));
}

void config() {
    check(!perception::parse_post_role_swap(nullptr));
    check(!perception::parse_post_role_swap("0"));
    check(perception::parse_post_role_swap("1"));
    for (const char* invalid : {"", "true", "false", "01", " 1", "1 ", "-1", "2", "1\n"}) {
        bool threw = false;
        try { (void)perception::parse_post_role_swap(invalid); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw);
    }
    for (bool profile : {false, true}) for (bool jit : {false, true}) for (bool dual : {false, true}) {
        perception::validate_post_role_swap_config(false, profile, jit, dual);
        bool threw = false;
        try { perception::validate_post_role_swap_config(true, profile, jit, dual); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw == !(profile && jit && dual));
    }
}

void concurrent_roles(perception::ParallelTaskWorker& worker, bool swap) {
    std::mutex mutex;
    std::condition_variable cv;
    int entered = 0;
    std::thread::id coco_tid, active_tid;
    const auto caller = std::this_thread::get_id();
    perception::ExecutionCpuRecord join;
    auto rendezvous = [&](std::thread::id& tid) {
        tid = std::this_thread::get_id();
        std::unique_lock<std::mutex> lock(mutex);
        ++entered;
        cv.notify_one();
        if (!cv.wait_for(lock, std::chrono::seconds(2), [&] { return entered == 2; }))
            throw std::runtime_error("parallel role rendezvous timeout");
    };
    perception::run_post_tasks(worker, swap, [&] { rendezvous(coco_tid); }, [&] { rendezvous(active_tid); }, &join);
    check(entered == 2 && coco_tid != std::thread::id() && active_tid != std::thread::id() && coco_tid != active_tid);
    check((swap ? coco_tid : active_tid) == caller);
    check(std::string(perception::post_helper_role(swap)) == (swap ? "active" : "coco"));
    check(std::string(perception::post_caller_role(swap)) == (swap ? "coco" : "active"));
#ifdef __linux__
    // Still on the calling thread: the join wait ran here.
    check(join.valid && join.tid == perception::execution_cpu_stamp().tid);
#else
    check(!join.valid && std::string(join.reason) == "unsupported_platform");
#endif
}

void exceptions_and_reuse(perception::ParallelTaskWorker& worker, bool swap) {
    for (int mask = 0; mask != 4; ++mask) {
        bool coco_done = false, active_done = false;
        std::string observed;
        try {
            perception::run_post_tasks(worker, swap, [&] {
                coco_done = true;
                if (mask & 1) throw std::runtime_error("coco");
            }, [&] {
                active_done = true;
                if (mask & 2) throw std::runtime_error("active");
            });
        } catch (const std::runtime_error& e) { observed = e.what(); }
        check(coco_done && active_done);
        check(observed == ((mask & 2) ? "active" : (mask & 1) ? "coco" : ""));
        int x = 0, y = 0;
        perception::run_post_tasks(worker, swap, [&] { x = 17; }, [&] { y = 19; });
        check(x == 17 && y == 19);
    }
    bool coco_done = false, active_done = false;
    int observed = 0;
    try {
        perception::run_post_tasks(worker, swap, [&] { coco_done = true; throw 11; },
            [&] { active_done = true; throw 22; });
    } catch (int value) { observed = value; }
    check(coco_done && active_done && observed == 22);
}

void off_equivalence(perception::ParallelTaskWorker& worker) {
    std::thread::id old_coco, old_active, new_coco, new_active;
    worker.run([&] { old_coco = std::this_thread::get_id(); },
               [&] { old_active = std::this_thread::get_id(); });
    perception::run_post_tasks(worker, false,
        [&] { new_coco = std::this_thread::get_id(); },
        [&] { new_active = std::this_thread::get_id(); }, nullptr);
    check(old_coco == new_coco && old_active == new_active);
    check(old_coco != std::thread::id() && old_coco != old_active);
    for (int iteration = 0; iteration < 100; ++iteration) {
        int a = 0, b = 0;
        perception::run_post_tasks(worker, iteration % 2 != 0, [&] { a = iteration; }, [&] { b = iteration + 1; });
        check(a == iteration && b == iteration + 1);
    }
}
}

int main() {
    try {
        config();
        perception::ParallelTaskWorker worker;
        off_equivalence(worker);
        for (bool swap : {false, true}) {
            concurrent_roles(worker, swap);
            exceptions_and_reuse(worker, swap);
        }
        std::cout << "PASS post_role_dispatch checks=" << checks
                  << " production_worker_shared=1 active_exception_priority_preserved=1 device_used=0\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
