/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include "VolumeListenerTypes.h"
#include "effect-impl/EffectContext.h"
#include "PalApi.h"
#include "PalDefs.h"

#define LIN_VOLUME_QFACTOR_28 28
#define MAX_VOLUME_CAL_STEPS 15
#define MAX_GAIN_LEVELS 5
#define DEFAULT_CAL_STEP 0

namespace aidl::qti::effects {

enum class VolumeListenerState {
    UNINITIALIZED,
    INITIALIZED,
    ACTIVE,
};

using AudioDeviceDescriptionVector = std::vector<aidl::android::media::audio::common::AudioDeviceDescription>;

class VolumeListenerContext final : public EffectContext {
public:
    VolumeListenerContext(int statusDepth, const Parameter::Common& common, VolumeListenerType type);
    ~VolumeListenerContext();
    virtual RetCode setOutputDevice(const AudioDeviceDescriptionVector& devices) override;
    virtual RetCode setVolumeStereo(const Parameter::VolumeStereo& volumeStereo) override;
    RetCode enable();
    RetCode disable();
    void reset();
    int getSessionId() { return mSessionId;}
    void checkAndSetGaindDepCal();
    void dump ();

    bool isActive() { return mState == VolumeListenerState::ACTIVE;}
    bool isEffectActiveAndApplicable() { return isActive() && isValidContext(); }
    float leftVolume() { return mVolumeStereo.left;}
    float rightVolume() { return mVolumeStereo.right;}
private:

    // Helper functions to check devices, // TODO 1) move to utils 
    bool isValidVoiceCallContext();
    bool isValidContext();
    bool isSpeaker(AudioDeviceDescriptionVector & devices);
    bool isSpeaker(const AudioDeviceDescriptionVector & devices);
    bool isWiredHeadset(AudioDeviceDescriptionVector & devices);
    bool isEarpiece(AudioDeviceDescriptionVector & devices);

    static bool sHeadsetCalEnabled;
    static struct pal_amp_db_and_gain_table * sGainTable;
    static int sTotalVolumeSteps;
    static float sCurrentVolume;
    static int sCurrentGainDepCalLevel;

    VolumeListenerState mState;
    VolumeListenerType mType;
    //Parameter::VolumeStereo mVolumeStereo;
    int mSessionId;
    // methods moved from Gain.cpp

    bool sendGainDepCalibration(int level);
    bool sendLinearGain(int32_t gain);
    void applyUpdatedCalibration(float newVolume);
};

}  // namespace aidl::qti::effects
