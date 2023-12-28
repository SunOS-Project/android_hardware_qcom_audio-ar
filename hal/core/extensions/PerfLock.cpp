/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define LOG_TAG "AHAL_PerfLock_QTI"

#include <android-base/logging.h>
#include <dlfcn.h>
#include <extensions/PerfLock.h>

PerfLock::PerfLock() {
    static bool isInit = init();
    std::scoped_lock lock{sMonitor};
    acquire();
}

PerfLock::~PerfLock() {
    std::scoped_lock lock{sMonitor};
    release();
}

void PerfLock::acquire() {
    if (!sIsAcquired && sAcquirePerfLock != nullptr) {
        mHandle = sAcquirePerfLock(0, 0, kPerfLockOpts, kPerfLockOptsSize);
        if (mHandle > 0) {
            sIsAcquired = true;
            // LOG(INFO) << __func__ << ": PerfLock Handle:" << mHandle;
        }
    }
}

void PerfLock::release() {
    if (mHandle > 0 && sReleasePerfLock != nullptr) {
        sReleasePerfLock(mHandle);
        sIsAcquired = false;
        // LOG(INFO) << __func__ << ": PerfLock Handle:" << mHandle;
    }
}

// static
bool PerfLock::init() {
    const std::string kPerfLockLibrary{"libqti-perfd-client.so"};
    void* libHandle = dlopen(kPerfLockLibrary.c_str(), RTLD_LAZY);
    if (libHandle == nullptr) {
        const char* error = dlerror();
        LOG(INFO) << __func__ << " Failed to dlopen:" << kPerfLockLibrary << " " << error;
        dlclose(libHandle);
        return false;
    }

    sAcquirePerfLock = reinterpret_cast<AcquirePerfLock>(dlsym(libHandle, "perf_lock_acq"));
    if (sAcquirePerfLock == nullptr) {
        LOG(ERROR) << __func__ << ": \"perf_lock_acq\" symbol not found";
        dlclose(libHandle);
        return false;
    }

    sReleasePerfLock = reinterpret_cast<ReleasePerfLock>(dlsym(libHandle, "perf_lock_rel"));
    if (sReleasePerfLock == nullptr) {
        LOG(ERROR) << __func__ << ": \"perf_lock_rel\" symbol not found";
        dlclose(libHandle);
        return false;
    }
    LOG(INFO) << __func__ << ": PerfLock initialization successful";
    return true;
}