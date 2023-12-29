/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#pragma once

#include <mutex>

/**
 * A Scoped object for real perf lock.
 * Only one among the existing PerfLock instances possibly acquires real perf lock.
 **/
class PerfLock final {
  public:
    // All Public APIs are gaurded by sMonitor
    PerfLock();
    ~PerfLock();

  private:
    // disable copy
    PerfLock(const PerfLock&) = delete;
    PerfLock& operator=(const PerfLock& x) = delete;

    // disable move
    PerfLock(PerfLock&& other) = delete;
    PerfLock& operator=(PerfLock&& other) = delete;

    // function mapping for dlsym
    using AcquirePerfLock = int (*)(int, int, int*, int);
    using ReleasePerfLock = int (*)(int);

    inline static AcquirePerfLock sAcquirePerfLock{nullptr};
    inline static ReleasePerfLock sReleasePerfLock{nullptr};
    // this mutex is a class monitor
    inline static std::mutex sMonitor;
    inline static bool sIsAcquired{false};
    inline static int kPerfLockOptsSize{4};
    inline static int kPerfLockOpts[4] = {0x40400000, 0x1, 0x40C00000, 0x1};

    static bool init();

    void acquire();
    void release();
    int mHandle{0};
};