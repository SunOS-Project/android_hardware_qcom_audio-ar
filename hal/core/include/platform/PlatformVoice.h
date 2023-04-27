/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <aidl/android/media/audio/common/AudioDeviceDescription.h>

#include <memory>
#include <string>
#include <vector>

using aidl::android::media::audio::common::AudioDeviceDescription;

namespace qti::audio::core {

class PlatformVoice {
   public:
    PlatformVoice();
    ~PlatformVoice();

    int32_t route(const std::vector<AudioDeviceDescription>& devices);
    int32_t updateAudioMode(int32_t mode);
    int32_t setParameters(const std::string& keyValuePairs);
    int32_t getParameters(const std::string& keys, std::string& values);
    int32_t setVoiceVolume(float volume);
    int32_t getCallState(bool& state);
    int32_t setTtyMode(int32_t state);
    int32_t getTtyMode(int32_t& state);
    int32_t setHacEnabled(bool state);
    int32_t getHacENabled(bool& state);

    int32_t dump(int32_t fd);
};

}  // namespace qti::audio::core
