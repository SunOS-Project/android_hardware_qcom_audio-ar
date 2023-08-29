/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <algorithm>
#include <cstddef>
#include <memory>
#define LOG_TAG "AHAL_Effect_VolumeListener"
#include <unordered_set>

#include <android-base/logging.h>
#include "VolumeListenerContext.h"
#include "GlobalVolumeListenerSession.h"

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;

namespace aidl::qti::effects {

float VolumeListenerContext::sCurrentVolume = 0.0f;
int VolumeListenerContext::sCurrentGainDepCalLevel = 1;
bool VolumeListenerContext::sHeadsetCalEnabled = GlobalVolumeListenerSession::getSession().getConfig().isHeadsetCalEnabled();
struct pal_amp_db_and_gain_table* VolumeListenerContext::sGainTable = GlobalVolumeListenerSession::getSession().getConfig().getGainTable();
int VolumeListenerContext::sTotalVolumeSteps = GlobalVolumeListenerSession::getSession().getConfig().getVolumeCalSteps();

bool VolumeListenerContext:: isEarpiece(AudioDeviceDescriptionVector & devices) {
    for (const auto& device : devices) {
        if (device != AudioDeviceDescription{AudioDeviceType::OUT_SPEAKER_EARPIECE, ""}) {
            return false;
        }
    }
    return true;
}

bool VolumeListenerContext::isWiredHeadset(AudioDeviceDescriptionVector & devices) {
    for (const auto& device : devices) {
        if (device != AudioDeviceDescription{AudioDeviceType::OUT_HEADSET ,
                                             AudioDeviceDescription::CONNECTION_ANALOG} &&
            device != AudioDeviceDescription{AudioDeviceType::OUT_HEADPHONE,
                                            AudioDeviceDescription::CONNECTION_ANALOG}) {
            return false;
        }
    }
    return true;
}

// Adapted from dev & OUT_SPK, any of the device is a speaker device.
bool VolumeListenerContext:: isSpeaker(AudioDeviceDescriptionVector & devices) {
    const AudioDeviceDescriptionVector & dev = devices;
    return isSpeaker(dev);
}

bool VolumeListenerContext:: isSpeaker(const AudioDeviceDescriptionVector & devices) {
    for (const auto& device : devices) {
        if (device == AudioDeviceDescription{AudioDeviceType::OUT_SPEAKER, ""} ||
            device == AudioDeviceDescription{AudioDeviceType::OUT_SPEAKER_SAFE, ""}) {
            return true;
        }
    }
    return false;
}

VolumeListenerContext::VolumeListenerContext(int statusDepth, const Parameter::Common& common, VolumeListenerType type)
    : EffectContext(statusDepth, common) {
    LOG(DEBUG) << __func__ << " type " << type;
    mType = type;
    mState = VolumeListenerState::INITIALIZED;
}

VolumeListenerContext::~VolumeListenerContext() {
    LOG(DEBUG) << __func__;
    mState = VolumeListenerState::UNINITIALIZED;
}

bool VolumeListenerContext:: isValidVoiceCallContext() {
    return mType == VolumeListenerType::VOICECALL && sHeadsetCalEnabled &&
            (isEarpiece(mOutputDevice) || isWiredHeadset(mOutputDevice));
}

bool VolumeListenerContext::isValidContext() {
    return isValidVoiceCallContext() || isSpeaker(mOutputDevice);
}

void VolumeListenerContext::checkAndSetGaindDepCal() {
     // iterate through list and make decision to set new gain dep cal level for speaker device
     // 1. find all usecase active on speaker
     // 2. find energy sum for each usecase
     // 3. find the highest of all the active usecase
     // 4. if new value is different than the current value then load new calibration

    float newVolume = -1.0f, sumEnergy = 0.0f, tempVolume = 0.0f;
    bool sumEnergyUsed = false;
    uint32_t gain;
     auto activeSessions = GlobalVolumeListenerSession::getSession().getActiveSessions(); 
     for (auto &sessionContext : activeSessions) {
        if (sessionContext->isEffectActiveAndApplicable()) {
            sumEnergyUsed = true;
            tempVolume = fmax(sessionContext->leftVolume(), sessionContext->rightVolume());
            sumEnergy += tempVolume * tempVolume;
            LOG (INFO) << __func__ << " size " << activeSessions.size() << " sum energy " << sumEnergy; 
        }
    }

     if (sumEnergyUsed) {
         newVolume = fmin(sqrt(sumEnergy), 1.0);
     }

     gain = (uint32_t)(round(newVolume * (1 << LIN_VOLUME_QFACTOR_28)));
     sendLinearGain(gain);
     applyUpdatedCalibration(newVolume);
}

void VolumeListenerContext::applyUpdatedCalibration(float newVolume) {

    if (newVolume != sCurrentVolume) {
        // send Gain dep cal level
        int gainDepCalLevel = -1;
        if (newVolume >= 1 && sTotalVolumeSteps > 0) { // max amplitude, use highest DRC level
            gainDepCalLevel = sGainTable[sTotalVolumeSteps - 1].level;
        } else if (newVolume == -1) {
            gainDepCalLevel = DEFAULT_CAL_STEP;
        } else if (newVolume == 0) {
            gainDepCalLevel = sGainTable[0].level;
        } else {
            for (int max_level = 0; max_level + 1 < sTotalVolumeSteps; max_level++) {
                if (newVolume < sGainTable[max_level + 1].amp && newVolume >= sGainTable[max_level].amp) {
                    gainDepCalLevel = sGainTable[max_level].level;
                    break;
                }
            }
        }

        if (gainDepCalLevel != sCurrentGainDepCalLevel) {
            // decision made .. send new level now
            if (!sendGainDepCalibration(gainDepCalLevel)) {
                ALOGE("%s: Failed to set gain dep cal level", __func__);
            } else {
                // Success in setting the gain dep cal level, store new level and Volume
                sCurrentGainDepCalLevel = gainDepCalLevel;
                sCurrentVolume = newVolume;
            }
        }
    } else {
        LOG(VERBOSE) << "volume unchanged";
    }
}

RetCode VolumeListenerContext::enable() {
    LOG(DEBUG) << __func__ ;
    if (mState != VolumeListenerState::INITIALIZED) {
        LOG(ERROR) << __func__ << "state not initialized";
        return RetCode::ERROR_EFFECT_LIB_ERROR;
    }
    mState = VolumeListenerState::ACTIVE;
    // verfiy the context and check and set gain
    if (isValidContext()) {
        checkAndSetGaindDepCal();
    }
    return RetCode::SUCCESS;
}

RetCode VolumeListenerContext::disable() {
    LOG(DEBUG) << __func__;
    if (mState != VolumeListenerState::ACTIVE) {
        LOG(ERROR) << __func__ << "state not active";
        return RetCode::ERROR_EFFECT_LIB_ERROR;
    }
    mState = VolumeListenerState::INITIALIZED;
    // verfiy the context and check and set gain
    if (isValidContext()) {
        checkAndSetGaindDepCal();
    }
    return RetCode::SUCCESS;
}

void VolumeListenerContext::reset() {
    LOG(DEBUG) << __func__;
    disable();
    resetBuffer();
}

RetCode VolumeListenerContext::setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>& devices) {

    for (const auto & dev : devices) {
        LOG(DEBUG) << __func__ <<" " << dev.toString();
    }
    // check if old or new device is speaker for playback usecase
    if (isValidContext() || isSpeaker(devices)) {
        checkAndSetGaindDepCal();
    }
    mOutputDevice = devices;
    return RetCode::SUCCESS;
}

RetCode VolumeListenerContext::setVolumeStereo(const Parameter::VolumeStereo& volumeStereo) {

    // recompute gan dep cal level only if volume changed on speaker device
    // verfiy the context and check and set gain
    LOG(DEBUG) << __func__ <<"left " << volumeStereo.left << " right " << volumeStereo.right;
    mVolumeStereo = volumeStereo;
    if (isValidContext()) {
        checkAndSetGaindDepCal();
    }
    return RetCode::SUCCESS;
}

void  VolumeListenerContext::dump() {
    LOG(DEBUG) << __func__ <<"type " << mType << "sessionId " << mSessionId << "left " << mVolumeStereo.left << " right " << mVolumeStereo.right;
}

bool VolumeListenerContext::sendGainDepCalibration(int level) {
    int32_t ret = 0;
    pal_param_gain_lvl_cal_t gainLevelCal;
    gainLevelCal.level = level;
    ALOGE("%s: level %d", __func__, level);
    ret = pal_set_param(PAL_PARAM_ID_GAIN_LVL_CAL, (void*)&gainLevelCal, sizeof(pal_param_gain_lvl_cal_t));
    if (ret != 0) {
        LOG(ERROR) << "fail to set PAL_PARAM_ID_GAIN_LVL_CAL " << ret;
    }

    return (ret == 0);
}

bool VolumeListenerContext::sendLinearGain(int32_t gain) {
    int32_t ret = 0;
    pal_param_mspp_linear_gain_t linearGain;
    linearGain.gain = gain;
    ALOGE("%s: gain %d", __func__, gain);
    ret = pal_set_param(PAL_PARAM_ID_MSPP_LINEAR_GAIN, (void*)&linearGain, sizeof(pal_param_mspp_linear_gain_t));
    if (ret != 0) {
        LOG(ERROR) << "fail to set PAL_PARAM_ID_MSPP_LINEAR_GAIN " << ret;
    }

    return (ret == 0);
}

}  // namespace aidl::qti::effects
