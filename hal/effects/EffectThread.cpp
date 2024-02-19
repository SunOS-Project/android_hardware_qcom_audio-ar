/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cstddef>
#include <memory>

#define LOG_TAG "AHAL_EffectThreadQti"
#include <android-base/logging.h>
#include <pthread.h>
#include <sys/resource.h>

#include "effect-impl/EffectThread.h"
#include "effect-impl/EffectTypes.h"

using ::android::hardware::EventFlag;
using ::aidl::android::hardware::audio::effect::kEventFlagNotEmpty;
namespace aidl::qti::effects {

#define RET_SUCCESS_IF_THREAD_NOT_CREATED(threadCreated)                    \
    {                                                                       \
        if (!threadCreated) {                                               \
            LOG(VERBOSE) << __func__ << " no-op as thread wasn't created "; \
            return RetCode::SUCCESS;                                        \
        }                                                                   \
    }

EffectThread::EffectThread() {
    LOG(VERBOSE) << __func__ << this;
}

EffectThread::~EffectThread() {
    destroyThread();
    LOG(VERBOSE) << __func__ << " done" << this;
}

void EffectThread::setContext(std::shared_ptr<EffectContext> context) {
    std::lock_guard lg(mThreadMutex);
    mThreadContext = std::move(context);
}

RetCode EffectThread::createThread(const std::string& name, int priority) {
    if (mThread.joinable()) {
        LOG(WARNING) << mName << __func__ << " thread already created, no-op";
        return RetCode::SUCCESS;
    }
    mName = name;
    mPriority = priority;
    {
        std::lock_guard lg(mThreadMutex);
        mStop = true;
        mExit = false;
        auto statusMQ = mThreadContext->getStatusFmq();
        EventFlag* efGroup = nullptr;
        ::android::status_t status =
                EventFlag::createEventFlag(statusMQ->getEventFlagWord(), &efGroup);
        if (status != ::android::OK || !efGroup) {
            LOG(ERROR) << mName << __func__ << " create EventFlagGroup failed " << status
                       << " efGroup " << efGroup;
            return RetCode::ERROR_THREAD;
        }
        mEfGroup.reset(efGroup);
        // kickoff and wait for commands (CommandId::START/STOP) or IEffect.close from client
        mEfGroup->wake(kEventFlagNotEmpty);
    }

    mThread = std::thread(&EffectThread::threadLoop, this);
    mThreadCreated = true;
    LOG(VERBOSE) << mName << __func__ << " priority " << mPriority << " done";
    return RetCode::SUCCESS;
}

RetCode EffectThread::destroyThread() {
    RET_SUCCESS_IF_THREAD_NOT_CREATED(mThreadCreated);

    {
        std::lock_guard lg(mThreadMutex);
        mStop = mExit = true;
    }
    mCv.notify_one();

    if (mThread.joinable()) {
        mThread.join();
    }

    {
        std::lock_guard lg(mThreadMutex);
        mThreadContext.reset();
    }
    LOG(DEBUG) << mName << __func__;
    return RetCode::SUCCESS;
}

RetCode EffectThread::startThread() {
    RET_SUCCESS_IF_THREAD_NOT_CREATED(mThreadCreated);

    {
        std::lock_guard lg(mThreadMutex);
        mStop = false;
        mCv.notify_one();
    }

    mEfGroup->wake(kEventFlagNotEmpty);
    LOG(DEBUG) << mName << __func__;
    return RetCode::SUCCESS;
}

RetCode EffectThread::stopThread() {
    RET_SUCCESS_IF_THREAD_NOT_CREATED(mThreadCreated);

    {
        std::lock_guard lg(mThreadMutex);
        mStop = true;
        mCv.notify_one();
    }

    mEfGroup->wake(kEventFlagNotEmpty);
    LOG(DEBUG) << mName << __func__;
    return RetCode::SUCCESS;
}

void EffectThread::threadLoop() {
    pthread_setname_np(pthread_self(), mName.substr(0, kMaxTaskNameLen - 1).c_str());
    setpriority(PRIO_PROCESS, 0, mPriority);
    LOG(VERBOSE) << mName << __func__ << "Enter: ";
    while (true) {
        /**
         * wait for the EventFlag without lock, it's ok because the mEfGroup pointer will not change
         * in the life cycle of workerThread (threadLoop).
         */
        uint32_t efState = 0;
        LOG(VERBOSE) << mName << __func__ << "wait: ";
        mEfGroup->wait(kEventFlagNotEmpty, &efState);

        {
            std::unique_lock l(mThreadMutex);
            ::android::base::ScopedLockAssertion lock_assertion(mThreadMutex);
            mCv.wait(l, [&]() REQUIRES(mThreadMutex) { return mExit || !mStop; });
            if (mExit) {
                LOG(VERBOSE) << __func__ << " EXIT!";
                return;
            }
            process_l();
        }
    }
}

void EffectThread::process_l() {
    RETURN_VALUE_IF(!mThreadContext, void(), "nullContext");
    LOG(VERBOSE) << mName << __func__ << "Enter: ";
    auto statusMQ = mThreadContext->getStatusFmq();
    auto inputMQ = mThreadContext->getInputDataFmq();
    auto outputMQ = mThreadContext->getOutputDataFmq();
    auto buffer = mThreadContext->getWorkBuffer();

    auto processSamples = inputMQ->availableToRead();
    if (processSamples) {
        inputMQ->read(buffer, processSamples);
        IEffect::Status status = effectProcessImpl(buffer, buffer, processSamples);
        outputMQ->write(buffer, status.fmqProduced);
        statusMQ->writeBlocking(&status, 1);
        LOG(VERBOSE) << mName << __func__ << ": done processing, effect consumed "
                     << status.fmqConsumed << " produced " << status.fmqProduced;
    }
}

} // namespace aidl::qti::effects
