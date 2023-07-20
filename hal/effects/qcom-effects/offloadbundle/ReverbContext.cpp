/*

 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Effect_ReverbContext"
#include <cstddef>
#include <Utils.h>

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::qti::effects {

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;

bool ReverbContext::isPreset() {
    return (mType == OffloadBundleEffectType::AUX_PRESET_REVERB ||
            mType == OffloadBundleEffectType::INSERT_PRESET_REVERB);
}

ReverbContext::ReverbContext(int statusDepth, const Parameter::Common& common,
                  const OffloadBundleEffectType& type)
        : OffloadBundleContext(statusDepth, common, type) {
    LOG(DEBUG) << __func__ << type << " ioHandle " << common.ioHandle;
}

RetCode ReverbContext::init() {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    memset(&mOffloadReverbParams, 0, sizeof(struct reverb_params));
    if (isPreset()) {
        mPreset = PresetReverb::Presets::NONE;
        mNextPreset = PresetReverb::Presets::NONE;
    }

    return RetCode::SUCCESS;
}

void ReverbContext::deInit() {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    std::lock_guard lg(mMutex);
}

RetCode ReverbContext::enable() {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    if (mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = true;

    if (isPreset() && mNextPreset == PresetReverb::Presets::NONE) {
        return RetCode::SUCCESS;
    }
    mOffloadReverbParams.enable_flag = true;
    return RetCode::SUCCESS;
}

RetCode ReverbContext::disable() {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    if (!mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = false;
    mOffloadReverbParams.enable_flag = false;
    sendOffloadParametersToPal(&mOffloadReverbParams, OFFLOAD_SEND_REVERB_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode ReverbContext::start(pal_stream_handle_t* palHandle) {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    std::lock_guard lg(mMutex);

    mPalHandle = palHandle;
    if (isEffectActive() && isPreset()) {
        sendOffloadParametersToPal(&mOffloadReverbParams, OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_PRESET);
    } else {
        LOG(DEBUG) << __func__ << mType << " inactive or non preset";
    }

    mPalHandle = nullptr;
    return RetCode::SUCCESS;
}

RetCode ReverbContext::stop() {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    std::lock_guard lg(mMutex);
    struct reverb_params reverbParam;
    memset(&reverbParam, 0, sizeof(struct bass_boost_params));
    reverbParam.enable_flag = false;

    sendOffloadParametersToPal(&reverbParam, OFFLOAD_SEND_REVERB_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode ReverbContext::setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>&
                    device) {
    LOG(DEBUG) << __func__ <<"   "<< mType ;
    mOutputDevice = device;
    return RetCode::SUCCESS;
}

RetCode ReverbContext::setPresetReverbPreset(const PresetReverb::Presets& preset) {
    LOG(DEBUG) << __func__ <<"   "<< mType  <<" preset " << toString(preset);
    mNextPreset = preset;
    mOffloadReverbParams.preset = static_cast<int32_t>(preset);
    if (preset != PresetReverb::Presets::NONE) {
        mOffloadReverbParams.enable_flag = true;
        sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_PRESET);
    }
    return RetCode::SUCCESS;
}

RetCode ReverbContext::setEnvironmentalReverbRoomLevel(int roomLevel) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " << roomLevel;
    mRoomLevel = roomLevel;
    mOffloadReverbParams.room_level = roomLevel;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_ROOM_LEVEL);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbRoomLevel() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mRoomLevel;
    return mRoomLevel;
}

RetCode ReverbContext::setEnvironmentalReverbRoomHfLevel(int roomHfLevel) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  roomHfLevel;
    mRoomHfLevel = roomHfLevel;
    mOffloadReverbParams.room_hf_level = roomHfLevel;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_ROOM_HF_LEVEL);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbRoomHfLevel() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mRoomHfLevel;
    return mRoomHfLevel;
}

RetCode ReverbContext::setEnvironmentalReverbDecayTime(int decayTime) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  decayTime;
    mDecayTime = decayTime;
    mOffloadReverbParams.decay_time = decayTime;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DECAY_TIME);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDecayTime() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " << mDecayTime;
    return mDecayTime;
}

RetCode ReverbContext::setEnvironmentalReverbDecayHfRatio(int decayHfRatio) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  decayHfRatio;
    mDecayHfRatio = decayHfRatio;
    mOffloadReverbParams.decay_hf_ratio = decayHfRatio;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DECAY_HF_RATIO);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDecayHfRatio() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " << mDecayHfRatio;
    return mDecayHfRatio;
}

RetCode ReverbContext::setEnvironmentalReverbLevel(int level) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " << level;
    mLevel = level;
    mOffloadReverbParams.level = level;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_LEVEL);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbLevel() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mLevel;
    return mLevel;
}

RetCode ReverbContext::setEnvironmentalReverbDelay(int delay) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  delay;
    mDelay = delay;
    mOffloadReverbParams.delay = delay;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DELAY);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDelay() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mDelay;
    return mDelay;
}

RetCode ReverbContext::setEnvironmentalReverbDiffusion(int diffusion) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  diffusion;
    mDiffusion = diffusion;
    mOffloadReverbParams.diffusion = diffusion;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DIFFUSION);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDiffusion() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " << mDiffusion;
    return mDiffusion;
}

RetCode ReverbContext::setEnvironmentalReverbDensity(int density) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " << density;
    mDensity = density;
    mOffloadReverbParams.density = density;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DENSITY);
    return RetCode::SUCCESS;
}

int ReverbContext::getEnvironmentalReverbDensity() const {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mDensity;
    return mDensity;
}

RetCode ReverbContext::setEnvironmentalReverbBypass(bool bypass) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  bypass;
    mBypass = bypass;
    return RetCode::SUCCESS;
}

bool ReverbContext::getEnvironmentalReverbBypass() const{
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mBypass;
    return mBypass;
}

RetCode ReverbContext::setReflectionsLevel(int level) {
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  level;
    mReflectionLevel = level;
    mOffloadReverbParams.reflections_level = level;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_DENSITY);
    return RetCode::SUCCESS;
}

bool ReverbContext::getReflectionsLevel() const{
    LOG(DEBUG) << __func__ <<"   "<< mType <<"  " <<  mReflectionLevel;
    return mReflectionLevel;
}

RetCode ReverbContext::setReflectionsDelay (int delay) {
    mReflectionDelay = delay;
    mOffloadReverbParams.reflections_delay = delay;
    sendOffloadParametersToPal(OFFLOAD_SEND_REVERB_ENABLE_FLAG | OFFLOAD_SEND_REVERB_REFLECTIONS_DELAY);
    return RetCode::SUCCESS;
}

bool ReverbContext::getReflectionsDelay() const {
    return mReflectionDelay;
}

int ReverbContext::sendOffloadParametersToPal(reverb_params *reverbParams, uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, reverbParams, flags);
    } else {
        LOG (INFO) <<" PalHandle not set";
    }
    return 0;
}

int ReverbContext::sendOffloadParametersToPal(uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, &mOffloadReverbParams, flags);
    } else {
        LOG (INFO) <<" PalHandle not set";
    }
    return 0;
}

}  // namespace aidl::qti::effects
