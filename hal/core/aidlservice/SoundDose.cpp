/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#define LOG_TAG "AHAL_SoundDose"

#include <aidlservice/SoundDose.h>

#include <android-base/logging.h>

namespace qti::audio::core {

ndk::ScopedAStatus SoundDose::setOutputRs2(float in_rs2ValueDbA) {
    if (in_rs2ValueDbA < MIN_RS2 || in_rs2ValueDbA > DEFAULT_MAX_RS2) {
        LOG(ERROR) << __func__ << ": RS2 value is invalid: " << in_rs2ValueDbA;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    mRs2Value = in_rs2ValueDbA;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus SoundDose::getOutputRs2(float* _aidl_return) {
    *_aidl_return = mRs2Value;
    LOG(DEBUG) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus SoundDose::registerSoundDoseCallback(
        const std::shared_ptr<ISoundDose::IHalSoundDoseCallback>& in_callback) {
    if (in_callback.get() == nullptr) {
        LOG(ERROR) << __func__ << ": Callback is nullptr";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (mCallback != nullptr) {
        LOG(ERROR) << __func__ << ": Sound dose callback was already registered";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    mCallback = in_callback;
    LOG(DEBUG) << __func__ << ": Registered sound dose callback ";
    return ndk::ScopedAStatus::ok();
}

}  // namespace aidl::android::hardware::audio::core::sounddose
