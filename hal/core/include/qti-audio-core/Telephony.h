/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/BnTelephony.h>
#include <android/binder_enums.h>

namespace qti::audio::core {

class Telephony : public ::aidl::android::hardware::audio::core::BnTelephony {
   public:
    Telephony();

   private:
    ndk::ScopedAStatus getSupportedAudioModes(
        std::vector<::aidl::android::media::audio::common::AudioMode>*
            _aidl_return) override;
    ndk::ScopedAStatus switchAudioMode(
        ::aidl::android::media::audio::common::AudioMode in_mode) override;
    ndk::ScopedAStatus setTelecomConfig(const TelecomConfig& in_config,
                                        TelecomConfig* _aidl_return) override;

    const std::vector<::aidl::android::media::audio::common::AudioMode>
        mSupportedAudioModes = {
            ::aidl::android::media::audio::common::AudioMode::NORMAL,
            ::aidl::android::media::audio::common::AudioMode::RINGTONE,
            ::aidl::android::media::audio::common::AudioMode::IN_CALL,
            ::aidl::android::media::audio::common::AudioMode::IN_COMMUNICATION,
            // Omit CALL_SCREEN for a better VTS coverage.
        };
    TelecomConfig mTelecomConfig;
};

}  // namespace qti::audio::core