/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/AudioPatch.h>
#include <aidl/android/hardware/audio/core/AudioRoute.h>
#include <aidl/android/media/audio/common/AudioPort.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>
#include <aidl/android/media/audio/common/MicrophoneInfo.h>

#include <map>
#include <unordered_map>
#include <memory>
#include <vector>

namespace qti::audio::core {

static const std::string kPrimaryModuleConfigFileName{
    "audio_module_config_primary.xml"};

class ModuleConfig {
   public:
    std::vector<::aidl::android::media::audio::common::MicrophoneInfo>
        microphones;
    std::vector<::aidl::android::media::audio::common::AudioPort> ports;
    // Exclusive for external device ports and their possible profiles
    std::unordered_map<
        int32_t,
        std::vector<::aidl::android::media::audio::common::AudioProfile>>
        mExternalDevicePortProfiles;
    std::vector<::aidl::android::media::audio::common::AudioPortConfig>
        portConfigs;
    std::vector<::aidl::android::media::audio::common::AudioPortConfig>
        initialConfigs;
    // Port id -> List of profiles to use when the device port state is set to
    // 'connected' in connection simulation mode.
    std::map<int32_t,
             std::vector<::aidl::android::media::audio::common::AudioProfile>>
        connectedProfiles;
    std::vector<::aidl::android::hardware::audio::core::AudioRoute> routes;
    std::vector<::aidl::android::hardware::audio::core::AudioPatch> patches;
    std::string toString() const;
    int32_t nextPortId = 1;
    int32_t nextPatchId = 1;
    static std::unique_ptr<ModuleConfig> getPrimaryConfiguration();
};

}  // namespace qti::audio::core