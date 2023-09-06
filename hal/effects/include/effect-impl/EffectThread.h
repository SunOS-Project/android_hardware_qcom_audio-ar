/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <android-base/thread_annotations.h>
#include <fmq/EventFlag.h>
#include <system/thread_defs.h>

#include "effect-impl/EffectContext.h"
#include "effect-impl/EffectTypes.h"

namespace aidl::qti::effects {

class EffectThread {
  public:
    // default priority is same as HIDL: ANDROID_PRIORITY_URGENT_AUDIO
    EffectThread();
    virtual ~EffectThread();

    // called by effect implementation.
    RetCode createThread(std::shared_ptr<EffectContext> context, const std::string& name,
                         int priority = ANDROID_PRIORITY_URGENT_AUDIO);
    RetCode destroyThread();
    RetCode startThread();
    RetCode stopThread();

    // Will call process() in a loop if the thread is running.
    void threadLoop();

    /**
     * @brief effectProcessImpl is running in worker thread which created in EffectThread.
     *
     * Effect implementation should think about concurrency in the implementation if necessary.
     * Parameter setting usually implemented in context (derived from EffectContext), and some
     * parameter maybe used in the processing, then effect implementation should consider using a
     * mutex to protect these parameter.
     *
     * EffectThread will make sure effectProcessImpl only be called after startThread() successful
     * and before stopThread() successful.
     *
     * effectProcessImpl implementation must not call any EffectThread interface, otherwise it will
     * cause deadlock.
     *
     * @param in address of input float buffer.
     * @param out address of output float buffer.
     * @param samples number of samples to process.
     * @return IEffect::Status
     */
    virtual IEffect::Status effectProcessImpl(float* in, float* out, int samples) = 0;

    /**
     * process() call effectProcessImpl() for effect data processing, it is necessary for the
     * processing to be called under Effect thread mutex mThreadMutex, to avoid the effect state
     * change before/during data processing, and keep the thread and effect state consistent.
     */
    virtual void process_l() REQUIRES(mThreadMutex);

  private:
    static constexpr int kMaxTaskNameLen = 15;

    std::mutex mThreadMutex;
    std::condition_variable mCv;
    bool mStop GUARDED_BY(mThreadMutex) = true;
    bool mExit GUARDED_BY(mThreadMutex) = false;
    std::shared_ptr<EffectContext> mThreadContext GUARDED_BY(mThreadMutex);

    struct EventFlagDeleter {
        void operator()(::android::hardware::EventFlag* flag) const {
            if (flag) {
                ::android::hardware::EventFlag::deleteEventFlag(&flag);
            }
        }
    };
    std::unique_ptr<::android::hardware::EventFlag, EventFlagDeleter> mEfGroup;
    std::thread mThread;
    int mPriority;
    std::string mName;
    bool mThreadCreated = false;
};
} // namespace aidl::qti::effects
