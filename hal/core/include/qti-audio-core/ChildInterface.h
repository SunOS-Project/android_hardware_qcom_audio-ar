/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <android/binder_auto_utils.h>
#include <android/binder_ibinder_platform.h>
#include <system/thread_defs.h>

#include <memory>
#include <utility>

namespace qti::audio::core {

// Helper used for interfaces that require a persistent instance. We hold them
// via a strong pointer. The binder token is retained for a call to
// 'setMinSchedulerPolicy'.
template <class C>
struct ChildInterface : private std::pair<std::shared_ptr<C>, ndk::SpAIBinder> {
    ChildInterface() = default;
    ChildInterface& operator=(const std::shared_ptr<C>& c) {
        return operator=(std::shared_ptr<C>(c));
    }
    ChildInterface& operator=(std::shared_ptr<C>&& c) {
        this->first = std::move(c);
        return *this;
    }
    explicit operator bool() const { return !!this->first; }
    C& operator*() const { return *(this->first); }
    C* operator->() const { return this->first; }
    // Use 'getInstance' when returning the interface instance.
    std::shared_ptr<C> getInstance() {
        if (this->second.get() == nullptr) {
            this->second = this->first->asBinder();
            AIBinder_setMinSchedulerPolicy(this->second.get(), SCHED_NORMAL,
                                           ANDROID_PRIORITY_AUDIO);
        }
        return this->first;
    }
};

}  // namespace qti::audio::core
