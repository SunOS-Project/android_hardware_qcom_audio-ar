/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Effect_BassBoostQti"

#include <Utils.h>
#include <cstddef>

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
    memset(&mBassParams, 0, sizeof(struct BassBoostParams));
    mState = EffectState::INITIALIZED;
    return RetCode::SUCCESS;
}

void BassBoostContext::deInit() {
    LOG(DEBUG) << __func__ << " ioHandle" << getIoHandle();
    stop();
}

RetCode BassBoostContext::enable() {
    LOG(INFO) << __func__ << " ioHandle" << getIoHandle();
    if (mState != EffectState::INITIALIZED) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = true;
    mState = EffectState::ACTIVE;
    mBassParams.mEnabled = 1;
    sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode BassBoostContext::disable() {
    LOG(INFO) << __func__ << " ioHandle" << getIoHandle();
    if (mState != EffectState::ACTIVE) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = false;
    mState = EffectState::INITIALIZED;
    mBassParams.mEnabled = 0;
    sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode BassBoostContext::start(pal_stream_handle_t* palHandle) {
    std::lock_guard lg(mMutex);
    LOG(INFO) << __func__ << " ioHandle" << getIoHandle();
    mPalHandle = palHandle;
    if (isEffectActive()) {
        sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG |
                                   OFFLOAD_SEND_BASSBOOST_STRENGTH);
    } else {
        LOG(DEBUG) << "Not yet enabled";
    }
    return RetCode::SUCCESS;
}

RetCode BassBoostContext::stop() {
    LOG(INFO) << __func__ << " ioHandle" << getIoHandle();
    std::lock_guard lg(mMutex);

    struct BassBoostParams bassParams;
    memset(&bassParams, 0, sizeof(struct BassBoostParams));
    bassParams.mEnabled = 0;

    sendOffloadParametersToPal(&bassParams, OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
    mPalHandle = nullptr;
    return RetCode::SUCCESS;
}

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
        const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>& device) {
    mOutputDevice = device;
    if (deviceSupportsEffect(mOutputDevice)) {
        if (mTempDisabled) {
            if (isEffectActive()) {
                mBassParams.mEnabled = 1;
                sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
            }
        }
        mTempDisabled = false;
    } else if (!mTempDisabled) {
        if (isEffectActive()) {
            mBassParams.mEnabled = 0;
            sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG);
        }
        mTempDisabled = true;
    }

    return RetCode::SUCCESS;
}

RetCode BassBoostContext::setBassBoostStrength(int strength) {
    LOG(INFO) << __func__ << " strength " << strength;
    mStrength = strength;
    mBassParams.mStrength = strength;
    sendOffloadParametersToPal(OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG |
                               OFFLOAD_SEND_BASSBOOST_STRENGTH);
    return RetCode::SUCCESS;
}

int BassBoostContext::getBassBoostStrength() {
    LOG(DEBUG) << __func__ << " strength " << mStrength;
    return mStrength;
}

int BassBoostContext::sendOffloadParametersToPal(uint64_t flags) {
    if (mPalHandle) {
        LOG(DEBUG) << " Strength " << mBassParams.mStrength << " enabled " << mBassParams.mEnabled;
        ParamDelegator::updatePalParameters(mPalHandle, &mBassParams, flags);
    } else {
        LOG(VERBOSE) << " PalHandle not set";
    }
    return 0;
}

int BassBoostContext::sendOffloadParametersToPal(BassBoostParams* bassParams, uint64_t flags) {
    if (mPalHandle) {
        LOG(DEBUG) << " Strength " << bassParams->mStrength << " enabled " << bassParams->mEnabled;
        ParamDelegator::updatePalParameters(mPalHandle, bassParams, flags);
    } else {
        LOG(VERBOSE) << " PalHandle not set";
    }
    return 0;
}

} // namespace aidl::qti::effects
