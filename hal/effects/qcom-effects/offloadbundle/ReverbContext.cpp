/*

 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Effect_ReverbQti"
#include <Utils.h>
#include <cstddef>

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::qti::effects {

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;

bool ReverbContext::isPreset() {
    return (mType == OffloadBundleEffectType::AUX_PRESET_REVERB ||
            mType == OffloadBundleEffectType::INSERT_PRESET_REVERB);
}

ReverbContext::ReverbContext(const Parameter::Common& common, const OffloadBundleEffectType& type,
                             bool processData)
    : OffloadBundleContext(common, type, processData) {
    LOG(DEBUG) << __func__ << type << " ioHandle " << common.ioHandle;
}

RetCode ReverbContext::init() {
    LOG(DEBUG) << __func__ << "   " << mType;
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    memset(&mReverbParams, 0, sizeof(struct ReverbParams));
    if (isPreset()) {
        mPreset = PresetReverb::Presets::NONE;
        mNextPreset = PresetReverb::Presets::NONE;
    }

    return RetCode::SUCCESS;
}

void ReverbContext::deInit() {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle();
    stop();
}

RetCode ReverbContext::enable() {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle();
    if (mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = true;

    if (isPreset() && mNextPreset == PresetReverb::Presets::NONE) {
        return RetCode::SUCCESS;
    }
    mReverbParams.enable = 1;
    return RetCode::SUCCESS;
}

RetCode ReverbContext::disable() {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle();
    if (!mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = false;
    mReverbParams.enable = 0;
    sendOffloadParametersToPal(&mReverbParams, OFFLOAD_SEND_REVERB_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode ReverbContext::start(pal_stream_handle_t* palHandle) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle();
    std::lock_guard lg(mMutex);

    mPalHandle = palHandle;
    if (isEffectActive() && isPreset()) {
        sendOffloadParametersToPal(&mReverbParams,
                                   OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_PRESET);
    } else {
        LOG(DEBUG) << __func__ << mType << " inactive or non preset";
    }

    return RetCode::SUCCESS;
}

RetCode ReverbContext::stop() {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle();
    std::lock_guard lg(mMutex);
    struct ReverbParams reverbParam;
    sendOffloadParametersToPal(&reverbParam, OFFLOAD_SEND_REVERB_ENABLE_FLAG);
    mPalHandle = nullptr;
    return RetCode::SUCCESS;
}

RetCode ReverbContext::setOutputDevice(
        const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>& device) {
    mOutputDevice = device;
    return RetCode::SUCCESS;
}

RetCode ReverbContext::setPresetReverbPreset(const PresetReverb::Presets& preset) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << " preset " << toString(preset);
    mNextPreset = preset;
    mReverbParams.preset = static_cast<int32_t>(preset);
    if (preset != PresetReverb::Presets::NONE) {
        mReverbParams.enable = 1;
        sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_PRESET);
    }
    return RetCode::SUCCESS;
}

RetCode ReverbContext::setEnvironmentalReverbRoomLevel(int roomLevel) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << roomLevel;
    mRoomLevel = roomLevel;
    mReverbParams.roomLevel = roomLevel;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_ROOM_LEVEL);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbRoomLevel() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mRoomLevel;
    return mRoomLevel;
}

RetCode ReverbContext::setEnvironmentalReverbRoomHfLevel(int roomHfLevel) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << roomHfLevel;
    mRoomHfLevel = roomHfLevel;
    mReverbParams.roomHfLevel = roomHfLevel;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_ROOM_HF_LEVEL);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbRoomHfLevel() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mRoomHfLevel;
    return mRoomHfLevel;
}

RetCode ReverbContext::setEnvironmentalReverbDecayTime(int decayTime) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << decayTime;
    mDecayTime = decayTime;
    mReverbParams.decayTime = decayTime;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DECAY_TIME);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDecayTime() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mDecayTime;
    return mDecayTime;
}

RetCode ReverbContext::setEnvironmentalReverbDecayHfRatio(int decayHfRatio) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << decayHfRatio;
    mDecayHfRatio = decayHfRatio;
    mReverbParams.decayHfRatio = decayHfRatio;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG |
                               OFFLOAD_SEND_REVERB_DECAY_HF_RATIO);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDecayHfRatio() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mDecayHfRatio;
    return mDecayHfRatio;
}

RetCode ReverbContext::setEnvironmentalReverbLevel(int level) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << level;
    mLevel = level;
    mReverbParams.level = level;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_LEVEL);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbLevel() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mLevel;
    return mLevel;
}

RetCode ReverbContext::setEnvironmentalReverbDelay(int delay) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << delay;
    mDelay = delay;
    mReverbParams.delay = delay;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DELAY);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDelay() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mDelay;
    return mDelay;
}

RetCode ReverbContext::setEnvironmentalReverbDiffusion(int diffusion) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << diffusion;
    mDiffusion = diffusion;
    mReverbParams.diffusion = diffusion;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DIFFUSION);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDiffusion() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mDiffusion;
    return mDiffusion;
}

RetCode ReverbContext::setEnvironmentalReverbDensity(int density) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << density;
    mDensity = density;
    mReverbParams.density = density;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DENSITY);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDensity() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() <<"  " << mDensity;
    return mDensity;
}

RetCode ReverbContext::setEnvironmentalReverbBypass(bool bypass) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << bypass;
    mBypass = bypass;
    return RetCode::SUCCESS;
}

bool ReverbContext::getEnvironmentalReverbBypass() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mBypass;
    return mBypass;
}

RetCode ReverbContext::setReflectionsLevel(int level) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << level;
    mReflectionsLevel = level;
    mReverbParams.reflectionsLevel = level;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DENSITY);
    return RetCode::SUCCESS;
}

bool ReverbContext::getReflectionsLevel() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mReflectionsLevel;
    return mReflectionsLevel;
}

RetCode ReverbContext::setReflectionsDelay(int delay) {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << delay;
    mReflectionsDelay = delay;
    mReverbParams.reflectionsDelay = delay;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG |
                               OFFLOAD_SEND_REVERB_REFLECTIONS_DELAY);
    return RetCode::SUCCESS;
}

bool ReverbContext::getReflectionsDelay() const {
    LOG(DEBUG) << __func__ << " ioHandle " << getIoHandle() << "  " << mReflectionsDelay;
    return mReflectionsDelay;
}

int ReverbContext::sendOffloadParametersToPal(ReverbParams* reverbParams, uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, reverbParams, flags);
    } else {
        LOG(VERBOSE) << " PalHandle not set";
    }
    return 0;
}

int ReverbContext::sendOffloadParametersToPal(uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, &mReverbParams, flags);
    } else {
        LOG(VERBOSE) << " PalHandle not set";
    }
    return 0;
}

} // namespace aidl::qti::effects
