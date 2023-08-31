/*

 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Effect_VirtualizerQti"

#include <Utils.h>
#include <cstddef>

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::qti::effects {

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;
using android::media::audio::common::AudioChannelLayout;

VirtualizerContext::VirtualizerContext(int statusDepth, const Parameter::Common& common,
                                       const OffloadBundleEffectType& type)
    : OffloadBundleContext(statusDepth, common, type) {
    LOG(DEBUG) << __func__ << type << " ioHandle " << common.ioHandle;
}

RetCode VirtualizerContext::init() {
    LOG(DEBUG) << __func__;
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    memset(&mVirtParams, 0, sizeof(struct VirtualizerParams));
    mState = EffectState::INITIALIZED;
    return RetCode::SUCCESS;
}

void VirtualizerContext::deInit() {
    LOG(DEBUG) << __func__ << " ioHandle" << getIoHandle();
    stop();
}

RetCode VirtualizerContext::enable() {
    LOG(DEBUG) << __func__;
    if (mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = true;
    mState = EffectState::ACTIVE;
    mVirtParams.enable = 1;
    sendOffloadParametersToPal(OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG |
                               OFFLOAD_SEND_VIRTUALIZER_STRENGTH);
    return RetCode::SUCCESS;
}

RetCode VirtualizerContext::disable() {
    LOG(DEBUG) << __func__;
    if (!mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = false;
    mState = EffectState::ACTIVE;
    mVirtParams.enable = 0;
    sendOffloadParametersToPal(OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode VirtualizerContext::start(pal_stream_handle_t* palHandle) {
    LOG(DEBUG) << __func__;
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    mPalHandle = palHandle;
    if (isEffectActive()) {
        sendOffloadParametersToPal(OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG |
                                   OFFLOAD_SEND_VIRTUALIZER_STRENGTH);
    } else {
        LOG(DEBUG) << "Not yet enabled";
    }

    return RetCode::SUCCESS;
}

RetCode VirtualizerContext::stop() {
    LOG(DEBUG) << __func__;
    std::lock_guard lg(mMutex);

    struct VirtualizerParams virtParams;
    memset(&virtParams, 0, sizeof(struct VirtualizerParams));
    virtParams.enable = 0;

    sendOffloadParametersToPal(&virtParams, OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG);
    mPalHandle = nullptr;
    return RetCode::SUCCESS;
}

RetCode VirtualizerContext::setOutputDevice(
        const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>& device) {
    mOutputDevice = device;
    if (deviceSupportsEffect(mOutputDevice)) {
        if (mTempDisabled) {
            if (isEffectActive()) {
                mVirtParams.enable = 1;
                sendOffloadParametersToPal(OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG);
            }
        }
        mTempDisabled = false;
    } else if (!mTempDisabled) {
        if (isEffectActive()) {
            mVirtParams.enable = 0;
            sendOffloadParametersToPal(OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG);
        }
        mTempDisabled = true;
    }
    return RetCode::SUCCESS;
}

RetCode VirtualizerContext::setVirtualizerStrength(int strength) {
    LOG(DEBUG) << __func__ << " strength " << strength;
    mStrength = strength;
    sendOffloadParametersToPal(OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG |
                               OFFLOAD_SEND_VIRTUALIZER_STRENGTH);
    return RetCode::SUCCESS;
}

int VirtualizerContext::getVirtualizerStrength() const {
    LOG(DEBUG) << __func__ << " strength " << mStrength;
    return mStrength;
}

RetCode VirtualizerContext::setForcedDevice(const AudioDeviceDescription& device) {
    RETURN_VALUE_IF(true != deviceSupportsEffect({device}), RetCode::ERROR_EFFECT_LIB_ERROR,
                    " deviceUnsupported");
    mForcedDevice = device;
    LOG(DEBUG) << __func__ << " TODO impl";
    return RetCode::SUCCESS;
}

std::vector<Virtualizer::ChannelAngle> VirtualizerContext::getSpeakerAngles(
        const Virtualizer::SpeakerAnglesPayload payload) {
    std::vector<Virtualizer::ChannelAngle> angles;
    auto channels = ::aidl::android::hardware::audio::common::getChannelCount(payload.layout);
    RETURN_VALUE_IF(!isConfigSupported(channels, payload.device), angles, "unsupportedConfig");

    if (channels == 1) {
        angles = {{.channel = (int32_t)AudioChannelLayout::CHANNEL_FRONT_LEFT,
                   .azimuthDegree = 0,
                   .elevationDegree = 0}};
    } else {
        angles = {{.channel = (int32_t)AudioChannelLayout::CHANNEL_FRONT_LEFT,
                   .azimuthDegree = -90,
                   .elevationDegree = 0},
                  {.channel = (int32_t)AudioChannelLayout::CHANNEL_FRONT_RIGHT,
                   .azimuthDegree = 90,
                   .elevationDegree = 0}};
    }
    return angles;
}

int VirtualizerContext::sendOffloadParametersToPal(uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, &mVirtParams, flags);
    } else {
        LOG(VERBOSE) << " PalHandle not set";
    }
    return 0;
}

int VirtualizerContext::sendOffloadParametersToPal(VirtualizerParams* virtParams, uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, virtParams, flags);
    } else {
        LOG(VERBOSE) << " PalHandle not set";
    }
    return 0;
}

bool VirtualizerContext::isConfigSupported(size_t channelCount,
                                           const AudioDeviceDescription& device) {
    return ((channelCount == 1 || channelCount == 2) && deviceSupportsEffect({device}));
}

bool VirtualizerContext::deviceSupportsEffect(const std::vector<AudioDeviceDescription>& devices) {
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

} // namespace aidl::qti::effects
