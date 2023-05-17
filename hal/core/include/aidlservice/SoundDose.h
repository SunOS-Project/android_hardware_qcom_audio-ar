/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/sounddose/BnSoundDose.h>
#include <aidl/android/media/audio/common/AudioDevice.h>

#include <mutex>

using aidl::android::media::audio::common::AudioDevice;

namespace qti::audio::core {

class SoundDose
    : public ::aidl::android::hardware::audio::core::sounddose::BnSoundDose {
   public:
    SoundDose() : mRs2Value(DEFAULT_MAX_RS2){};

    ndk::ScopedAStatus setOutputRs2UpperBound(float in_rs2ValueDbA) override;
    ndk::ScopedAStatus getOutputRs2UpperBound(float* _aidl_return) override;
    ndk::ScopedAStatus registerSoundDoseCallback(
        const std::shared_ptr<ISoundDose::IHalSoundDoseCallback>& in_callback)
        override;

   private:
    std::shared_ptr<ISoundDose::IHalSoundDoseCallback> mCallback;
    float mRs2Value;
};

}  // namespace qti::audio::core
