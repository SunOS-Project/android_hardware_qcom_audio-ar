/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <aidl/android/hardware/audio/core/AudioPatch.h>
#include <aidl/android/hardware/audio/core/AudioRoute.h>
#include <aidl/android/media/audio/common/AudioPort.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>
#include <aidl/android/media/audio/common/MicrophoneInfo.h>

#include <utils/AudioCollection.h>

namespace qti::audio::core {

static constexpr char kAudioHALServiceDumpPath[] =
    "/data/vendor/audio/audio_hal_service.dump";
static constexpr char* kPrimaryConfigFileName =
    "audio_module_config_primary.xml";
static constexpr char* kUSBConfigFileName = "audio_module_config_usb.xml";
static constexpr char* kRSubmixConfigFileName =
    "audio_module_config_rsubmix.xml";

class ModuleConfig {
   public:
    std::vector<::aidl::android::media::audio::common::MicrophoneInfo>
        mMicrophones;
    // All the mix ports and device ports(even template device AudioPorts).
    AudioCollection<::aidl::android::media::audio::common::AudioPort> mPorts;
    // Exclusive for external device ports and their possible profiles
    std::unordered_map<
        int32_t,
        std::vector<::aidl::android::media::audio::common::AudioProfile>>
        mExternalDevicePortProfiles;

    // maintain only active ports
    AudioCollection<::aidl::android::media::audio::common::AudioPortConfig>
        mActivePortConfigs;
    AudioCollection<::aidl::android::media::audio::common::AudioPortConfig>
        mDefaultPortConfigs;

    // Port id -> List of profiles to use when the device port state is set to
    // 'connected'.
    std::map<int32_t,
             std::vector<::aidl::android::media::audio::common::AudioProfile>>
        mConnectedProfiles;
    std::vector<::aidl::android::hardware::audio::core::AudioRoute> mRoutes;
    AudioCollection<::aidl::android::hardware::audio::core::AudioPatch>
        mPatches;
    // Port ids of attached devices
    std::vector<int32_t> mAttachedDevices;
    ::aidl::android::media::audio::common::AudioPort mDefaultOutputDevice;
    bool mIsSpeakerDrcEnabled;
    bool mIsCallScreenModeSupported;
    int32_t mNextPortId = 1;
    int32_t mNextPatchId = 1;

    // dumps the data of this object
    // input: valid fd to write
    // returns 0 on success
    int32_t dump(const int32_t fd = 0);
};

std::unique_ptr<ModuleConfig> getPrimaryConfiguration();

}  // namespace qti::audio::core
