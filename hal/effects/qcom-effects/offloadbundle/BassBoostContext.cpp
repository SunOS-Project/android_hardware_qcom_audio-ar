/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Effect_BassBoostContext"

#include <cstddef>
#include <Utils.h>

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::qti::effects {

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;

BassBoostContext::BassBoostContext(int statusDepth, const Parameter::Common& common,
                  const OffloadBundleEffectType& type)
        : OffloadBundleContext(statusDepth, common, type) {
    LOG(DEBUG) << __func__ << type << " ioHandle " << common.ioHandle;
}

RetCode BassBoostContext::init() {
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    memset(&mOffloadBassBoostParams, 0, sizeof(struct bass_boost_params));
    mState = EffectState::INITIALIZED;
    return RetCode::SUCCESS;
}

void BassBoostContext::deInit() {
    std::lock_guard lg(mMutex);
    // reset if anything needed.
}

RetCode BassBoostContext::enable() {
    if (mState != EffectState::INITIALIZED) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = true;
    mState = EffectState::ACTIVE;
    mOffloadBassBoostParams.enable_flag = true;
    sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode BassBoostContext::disable() {
    if (mState != EffectState::ACTIVE) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = false;
    mState = EffectState::INITIALIZED;
    mOffloadBassBoostParams.enable_flag = false;
    sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode BassBoostContext::start(pal_stream_handle_t* palHandle) {
    std::lock_guard lg(mMutex);

    mPalHandle = palHandle;
    if (isEffectActive()) {
        sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG | OFFLOAD_SEND_BASSBOOST_STRENGTH);
    } else {
        LOG (INFO) <<"Not yet enabled";
    }
    return RetCode::SUCCESS;
}

RetCode BassBoostContext::stop() {
    std::lock_guard lg(mMutex);

    struct bass_boost_params bassParams;
    memset(&bassParams, 0, sizeof(struct bass_boost_params));
    bassParams.enable_flag = false;

    sendOffloadParametersToPal(&bassParams, OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
    mPalHandle = nullptr;
    return RetCode::SUCCESS;
}

/*
device == AUDIO_DEVICE_OUT_WIRED_HEADSET ||
device == AUDIO_DEVICE_OUT_WIRED_HEADPHONE ||
device == AUDIO_DEVICE_OUT_USB_HEADSET ||
device == AUDIO_DEVICE_OUT_USB_DEVICE ||
device == AUDIO_DEVICE_OUT_USB_ACCESSORY ||
device == AUDIO_DEVICE_OUT_BLUETOOTH_A2DP ||
device == AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES) {
*/
bool BassBoostContext::deviceSupportsEffect(const std::vector<AudioDeviceDescription>& devices) {

    for (const auto& device : devices) {
        if (device != AudioDeviceDescription{AudioDeviceType::OUT_HEADSET,
                                             AudioDeviceDescription::CONNECTION_ANALOG} &&
            device != AudioDeviceDescription{AudioDeviceType::OUT_HEADPHONE,
                                             AudioDeviceDescription::CONNECTION_ANALOG} &&
            device != AudioDeviceDescription{AudioDeviceType::OUT_HEADPHONE,
                                             AudioDeviceDescription::CONNECTION_BT_A2DP} &&
            device != AudioDeviceDescription{AudioDeviceType::OUT_HEADSET,
                                             AudioDeviceDescription::CONNECTION_USB}) {
            return false;
        }
    }
    return true;
}

RetCode BassBoostContext::setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>&
                    device) {
    mOutputDevice = device;
    if (deviceSupportsEffect(mOutputDevice)) {
        if (mTempDisabled) {
            if (isEffectActive()) {
                mOffloadBassBoostParams.enable_flag = true;
                sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
            }
        }
        mTempDisabled = false;
    } else if (!mTempDisabled) {
            if(isEffectActive()) {
                mOffloadBassBoostParams.enable_flag = false;
                sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
            }
            mTempDisabled = true;
    }

    return RetCode::SUCCESS;
}

RetCode BassBoostContext::setBassBoostStrength(int strength) {
    LOG(DEBUG) << __func__  << " strength " << strength;
    mStrength = strength;
    mOffloadBassBoostParams.strength = strength;
    sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG | OFFLOAD_SEND_BASSBOOST_STRENGTH);
    return RetCode::SUCCESS;
}

int BassBoostContext::getBassBoostStrength() {
    LOG(DEBUG) << __func__  << " strength " << mStrength;
    return mStrength;
}

int BassBoostContext::sendOffloadParametersToPal(uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, &mOffloadBassBoostParams, flags);
    } else {
        LOG (INFO) <<" PalHandle not set";
    }
    return 0;
}

int BassBoostContext::sendOffloadParametersToPal(bass_boost_params *bassParams, uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, bassParams, flags);
    } else {
        LOG (INFO) <<" PalHandle not set";
    }
    return 0;
}

}  // namespace aidl::qti::effects
