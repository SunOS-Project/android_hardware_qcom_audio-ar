/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Config"
#include "aidlservice/Config.h"

#include <android-base/logging.h>
#include <system/audio_config.h>

using ::aidl::android::media::audio::common::AudioHalEngineConfig;
using ::aidl::android::hardware::audio::core::SurroundSoundConfig;

namespace qti::audio::core {
ndk::ScopedAStatus Config::getSurroundSoundConfig(
    SurroundSoundConfig* _aidl_return) {
    SurroundSoundConfig surroundSoundConfig;
    // TODO: parse from XML; for now, use empty config as default
    *_aidl_return = std::move(surroundSoundConfig);
    LOG(DEBUG) << __func__ << ": returning " << _aidl_return->toString();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Config::getEngineConfig(AudioHalEngineConfig* _aidl_return) {
    static const AudioHalEngineConfig returnEngCfg = [this]() {
        AudioHalEngineConfig engConfig;
        if (mEngConfigConverter.getStatus() == ::android::OK) {
            engConfig = mEngConfigConverter.getAidlEngineConfig();
        } else {
            // LOG(INFO) << __func__ << mEngConfigConverter.getError();
            // if (mAudioPolicyConverter.getStatus() == ::android::OK) {
            //     engConfig = mAudioPolicyConverter.getAidlEngineConfig();
            // } else {
            //     LOG(WARNING) << __func__ << mAudioPolicyConverter.getError();
            // }
        }
        return engConfig;
    }();
    *_aidl_return = returnEngCfg;
    LOG(DEBUG) << __func__ << ": returning " << _aidl_return->toString();
    return ndk::ScopedAStatus::ok();
}
}  // namespace qti::audio::core
