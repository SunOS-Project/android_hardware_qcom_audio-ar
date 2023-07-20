/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Effect_EqualizerContext"

#include <Utils.h>
#include <cstddef>
#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::qti::effects {

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;

EqualizerContext::EqualizerContext(int statusDepth, const Parameter::Common& common,
                  const OffloadBundleEffectType& type)
        : OffloadBundleContext(statusDepth, common, type) {
    LOG(DEBUG) << __func__ << type << " ioHandle " << common.ioHandle;
}

RetCode EqualizerContext::init() {
    std::lock_guard lg(mMutex);
    // init with pre-defined preset NORMAL
    for (std::size_t i = 0; i < MAX_NUM_BANDS; i++) {
        mBandLevels[i] = kBandPresetLevels[0 /* normal */][i];
    }
    memset(&mOffloadEqualizerParams, 0, sizeof(struct eq_params));
    mOffloadEqualizerParams.config.preset_id = PRESET_INVALID;
    mOffloadEqualizerParams.config.eq_pregain = Q27_UNITY;
    return RetCode::SUCCESS;
}

void EqualizerContext::deInit() {
    std::lock_guard lg(mMutex);
}

RetCode EqualizerContext::enable() {
    std::lock_guard lg(mMutex);
    if (mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = true;
    mOffloadEqualizerParams.enable_flag = true;
    sendOffloadParametersToPal(OFFLOAD_SEND_EQ_ENABLE_FLAG | OFFLOAD_SEND_EQ_BANDS_LEVEL);
    return RetCode::SUCCESS;
}

RetCode EqualizerContext::disable() {
    std::lock_guard lg(mMutex);
    if (!mEnabled) return RetCode::ERROR_ILLEGAL_PARAMETER;
    mEnabled = false;
    mOffloadEqualizerParams.enable_flag = false;
    sendOffloadParametersToPal(OFFLOAD_SEND_EQ_ENABLE_FLAG);
    return RetCode::SUCCESS;
}

RetCode EqualizerContext::start(pal_stream_handle_t* palHandle) {
    std::lock_guard lg(mMutex);
    mPalHandle = palHandle;
    if (mEnabled) {
        sendOffloadParametersToPal(OFFLOAD_SEND_EQ_ENABLE_FLAG | OFFLOAD_SEND_EQ_BANDS_LEVEL);
    } else {
        LOG (INFO) <<"Not yet enabled";
    }

    return RetCode::SUCCESS;
}

RetCode EqualizerContext::stop() {
    std::lock_guard lg(mMutex);
    memset(&mOffloadEqualizerParams, 0, sizeof(struct eq_params));
    // use a dummy to disable, instead of Global
    mOffloadEqualizerParams.enable_flag = false;
    sendOffloadParametersToPal(OFFLOAD_SEND_EQ_ENABLE_FLAG);
    mPalHandle = NULL;
    return RetCode::SUCCESS;
}

RetCode EqualizerContext::setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>&
                    device){
    std::lock_guard lg(mMutex);
    mOutputDevice = device;
    // TODO send this mOutputDevice in offloadparams
    return RetCode::SUCCESS;
}

RetCode EqualizerContext::setEqualizerPreset(const std::size_t presetIdx) {
    if (presetIdx < 0 || presetIdx >= MAX_NUM_PRESETS) {
        return RetCode::ERROR_ILLEGAL_PARAMETER;
    }

    // Translation from existing implementation, first we update then send config to PAL.
    // ideally, send it to PAL and check if operation is successful then only update
    mCurrentPreset = presetIdx;
    for (std::size_t i = 0; i < MAX_NUM_BANDS; i++) {
        mBandLevels[i] = kBandPresetLevels[presetIdx][i];
    }

    updateOffloadParameters();

    sendOffloadParametersToPal(OFFLOAD_SEND_EQ_ENABLE_FLAG|OFFLOAD_SEND_EQ_PRESET);

    return RetCode::SUCCESS;
}

bool EqualizerContext::isBandLevelIndexInRange(
        const std::vector<Equalizer::BandLevel>& bandLevels) const {
    const auto [min, max] =
            std::minmax_element(bandLevels.begin(), bandLevels.end(),
                                [](const auto& a, const auto& b) { return a.index < b.index; });
    return min->index >= 0 && max->index < MAX_NUM_BANDS;
}

RetCode EqualizerContext::setEqualizerBandLevels(const std::vector<Equalizer::BandLevel>& bandLevels) {
    RETURN_VALUE_IF(bandLevels.size() > MAX_NUM_BANDS,
                    RetCode::ERROR_ILLEGAL_PARAMETER, "Exceeds Max Size");

    RETURN_VALUE_IF(bandLevels.empty(), RetCode::ERROR_ILLEGAL_PARAMETER, "Empty Bands");

    RETURN_VALUE_IF(!isBandLevelIndexInRange(bandLevels), RetCode::ERROR_ILLEGAL_PARAMETER,
                    "indexOutOfRange");

    // Translation from existing implementation, first we update then send config to PAL.
    // ideally, send it to PAL and check if operation is successful then only update
    for (auto &bandLevel : bandLevels) {
        LOG(INFO) << __func__ << " level " << bandLevel.index <<" level" << bandLevel.levelMb;
        mBandLevels[bandLevel.index] = bandLevel.levelMb;
        mCurrentPreset = PRESET_CUSTOM;
    }

    updateOffloadParameters();
    sendOffloadParametersToPal(OFFLOAD_SEND_EQ_ENABLE_FLAG|OFFLOAD_SEND_EQ_BANDS_LEVEL);

    return RetCode::SUCCESS;
}

std::vector<Equalizer::BandLevel> EqualizerContext::getEqualizerBandLevels() const {
    std::vector<Equalizer::BandLevel> bandLevels;
    bandLevels.reserve(MAX_NUM_BANDS);
    for (std::size_t i = 0; i < MAX_NUM_BANDS; i++) {
        bandLevels.emplace_back(Equalizer::BandLevel{static_cast<int32_t>(i), mBandLevels[i]});
    }
    return bandLevels;
}

std::vector<int32_t> EqualizerContext::getEqualizerCenterFreqs() {
    std::vector<int32_t> result;

    std::for_each(kBandFrequencies.begin(), kBandFrequencies.end(), [&](const auto &band)
                  { result.emplace_back((band.minMh + band.maxMh) / 2); });
    return result;
}

int EqualizerContext::updateOffloadParameters() {
    for (int i = 0; i < MAX_NUM_BANDS; i++) {
        mOffloadEqualizerParams.config.preset_id = mCurrentPreset;
        mOffloadEqualizerParams.per_band_cfg[i].band_idx = i;
        mOffloadEqualizerParams.per_band_cfg[i].filter_type = EQ_BAND_BOOST;
        mOffloadEqualizerParams.per_band_cfg[i].freq_millihertz = kPresetsFrequencies[i] * 1000;
        mOffloadEqualizerParams.per_band_cfg[i].gain_millibels = mBandLevels[i] * 100;
        mOffloadEqualizerParams.per_band_cfg[i].quality_factor = Q8_UNITY;
    }
    return 0;
}

int EqualizerContext::sendOffloadParametersToPal(uint64_t flags) {
    if (mPalHandle) {
        ParamDelegator::updatePalParameters(mPalHandle, &mOffloadEqualizerParams, flags);
    } else {
        LOG (INFO) <<" PalHandle not set";
    }
    return 0;
}

}  // namespace aidl::qti::effects
