/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include <cstddef>
#define LOG_TAG "BundleContext"
#include <Utils.h>

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::android::hardware::audio::effect {

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;

RetCode OffloadBundleContext::init() {
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    return RetCode::SUCCESS;
}

void OffloadBundleContext::deInit() {
    std::lock_guard lg(mMutex);
}

RetCode OffloadBundleContext::enable() {
    if (mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    // Bass boost or Virtualizer can be temporarily disabled if playing over device speaker due to
    // their nature.
    bool tempDisabled = false;
    switch (mType) {
        case OffloadBundleEffectType::EQUALIZER:
            LOG(DEBUG) << __func__ << " enable bundle EQ";
            break;
        case OffloadBundleEffectType::BASS_BOOST:
            LOG(DEBUG) << __func__ << " enable bundle BB";
            break;
        case OffloadBundleEffectType::VIRTUALIZER:
            LOG(DEBUG) << __func__ << " enable bundle V";
            break;
    }
    mEnabled = true;
    return (tempDisabled ? RetCode::SUCCESS : enableOperatingMode());
}

RetCode OffloadBundleContext::enableOperatingMode() {
    return RetCode::SUCCESS;
}

RetCode OffloadBundleContext::disable() {
    if (!mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    switch (mType) {
        case OffloadBundleEffectType::EQUALIZER:
            LOG(DEBUG) << __func__ << " disable offloadbundle EQ";
            break;
        case OffloadBundleEffectType::BASS_BOOST:
            LOG(DEBUG) << __func__ << " disable offloadbundle Bassboost";
            break;
        case OffloadBundleEffectType::VIRTUALIZER:
            LOG(DEBUG) << __func__ << " disable offloadbundle Virtualizer";
            break;
    }
    mEnabled = false;
    return disableOperatingMode();
}

RetCode OffloadBundleContext::disableOperatingMode() {
    return RetCode::SUCCESS;
}

bool OffloadBundleContext::isDeviceSupportedBassBoost(
        const aidl::android::media::audio::common::AudioDeviceDescription& device) {
    return (device == AudioDeviceDescription{AudioDeviceType::OUT_SPEAKER, ""} ||
            device == AudioDeviceDescription{AudioDeviceType::OUT_CARKIT,
                                             AudioDeviceDescription::CONNECTION_BT_SCO} ||
            device == AudioDeviceDescription{AudioDeviceType::OUT_SPEAKER,
                                             AudioDeviceDescription::CONNECTION_BT_A2DP});
}

bool OffloadBundleContext::isDeviceSupportedVirtualizer(
        const aidl::android::media::audio::common::AudioDeviceDescription& device) {
    return (device == AudioDeviceDescription{AudioDeviceType::OUT_HEADSET,
                                             AudioDeviceDescription::CONNECTION_ANALOG} ||
            device == AudioDeviceDescription{AudioDeviceType::OUT_HEADPHONE,
                                             AudioDeviceDescription::CONNECTION_ANALOG} ||
            device == AudioDeviceDescription{AudioDeviceType::OUT_HEADPHONE,
                                             AudioDeviceDescription::CONNECTION_BT_A2DP} ||
            device == AudioDeviceDescription{AudioDeviceType::OUT_HEADSET,
                                             AudioDeviceDescription::CONNECTION_USB});
}

RetCode OffloadBundleContext::setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>& device) {
    mOutputDevice = device;
    return RetCode::SUCCESS;
}

RetCode OffloadBundleContext::setVolumeStereo(const Parameter::VolumeStereo& volume) {

    mVolumeStereo = volume;
    return RetCode::SUCCESS;
}

RetCode OffloadBundleContext::setEqualizerPreset(const std::size_t presetIdx) {
    return RetCode::SUCCESS;
}

RetCode OffloadBundleContext::setEqualizerBandLevels(const std::vector<Equalizer::BandLevel>& bandLevels) {

    return RetCode::SUCCESS;
}

std::vector<Equalizer::BandLevel> OffloadBundleContext::getEqualizerBandLevels() const {
    return {};
}

RetCode OffloadBundleContext::setBassBoostStrength(int strength) {
    return RetCode::SUCCESS;
}

RetCode OffloadBundleContext::setVolumeLevel(int level) {
    return RetCode::SUCCESS;
}

int OffloadBundleContext::getVolumeLevel() const {
    return 100;
}

RetCode OffloadBundleContext::setVolumeMute(bool mute) {
    return RetCode::SUCCESS;
}

RetCode OffloadBundleContext::setVirtualizerStrength(int strength) {
    return RetCode::SUCCESS;
}

}  // namespace aidl::android::hardware::audio::effect
