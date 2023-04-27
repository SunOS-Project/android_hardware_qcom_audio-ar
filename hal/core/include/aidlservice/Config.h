/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/BnConfig.h>
#include <system/audio_config.h>

#include "aidlservice/EngineConfigXmlConverter.h"


namespace qti::audio::core {

static const std::string kEngineConfigFileName =
    "audio_policy_engine_configuration.xml";


class Config : public ::aidl::android::hardware::audio::core::BnConfig {
    ndk::ScopedAStatus getSurroundSoundConfig(
        ::aidl::android::hardware::audio::core::SurroundSoundConfig*
            _aidl_return) override;
    ndk::ScopedAStatus getEngineConfig(
        ::aidl::android::media::audio::common::AudioHalEngineConfig*
            _aidl_return) override;
    // internal::AudioPolicyConfigXmlConverter mAudioPolicyConverter{
    //         ::android::audio_get_audio_policy_config_file()};
    EngineConfigXmlConverter mEngConfigConverter{
        ::android::audio_find_readable_configuration_file(
            kEngineConfigFileName.c_str())};
};

}  // namespace qti::audio::core
