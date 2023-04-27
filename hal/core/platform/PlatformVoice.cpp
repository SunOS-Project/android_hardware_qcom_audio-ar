
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_PlatformVoice"

#include <android-base/logging.h>
#include <platform/PlatformVoice.h>

#include "PalApi.h"

namespace qti::audio::core {

PlatformVoice::PlatformVoice() { LOG(VERBOSE) << __func__ << ": Enter"; }

PlatformVoice::~PlatformVoice() { LOG(VERBOSE) << __func__ << ": Enter"; }

int32_t PlatformVoice::route(
    const std::vector<AudioDeviceDescription>& devices) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::updateAudioMode(int32_t mode) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
// AUDIO_PARAMETER_KEY_VSID,
// AUDIO_PARAMETER_KEY_TTY_MODE
// AUDIO_PARAMETER_KEY_VOLUME_BOOST
//  AUDIO_PARAMETER_KEY_SLOWTALK
// AUDIO_PARAMETER_KEY_HD_VOICE
// AUDIO_PARAMETER_KEY_DEVICE_MUTE
// AUDIO_PARAMETER_KEY_HAC,

int32_t PlatformVoice::setParameters(const std::string& keyValuePairs) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::getParameters(const std::string& keys,
                                     std::string& values) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::setVoiceVolume(float volume) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::getCallState(bool& state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::setTtyMode(int32_t state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::getTtyMode(int32_t& state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::setHacEnabled(bool state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}
int32_t PlatformVoice::getHacENabled(bool& state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}

int32_t PlatformVoice::dump(int32_t fd) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    return status;
}

}  // namespace qti::audio::core
