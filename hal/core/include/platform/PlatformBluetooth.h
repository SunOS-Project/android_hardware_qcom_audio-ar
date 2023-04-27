/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <memory>
#include <string>

namespace qti::audio::core {

class PlatformBluetooth {
   public:
    PlatformBluetooth();
    ~PlatformBluetooth();

    int32_t setHfpEnabled(bool enabled);
    int32_t getHfpEnabled(bool& enabled);
    int32_t setHfpSampleRate(int32_t sampleRate);
    int32_t getHfpSampleRate(int32_t& sampleRate);
    int32_t setHfpVolume(float volume);
    int32_t getHfpVolume(float& volume);

    int32_t setBtScoEnabled(bool enabled);
    int32_t getBtScoEnabled(bool& enabled);
    int32_t setBtScoHeadsetDebugName(const std::string& debugName);
    int32_t setBtScoNrecEnabled(bool enabled);
    int32_t getBtScoNrecEnabled(bool& enabled);
    int32_t setBtScoWidebandEnabled(bool enabled);
    int32_t getBtScoWidebandEnabled(bool& enabled);

    int32_t dump(int32_t fd);
};

}  // namespace qti::audio::core
