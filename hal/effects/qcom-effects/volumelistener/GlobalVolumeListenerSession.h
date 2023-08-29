/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <algorithm>
#include <memory>
#include <unordered_map>

#include <android-base/logging.h>
#include <android-base/thread_annotations.h>

#include "VolumeListenerContext.h"
#include "VolumeListenerTypes.h"

namespace aidl::qti::effects {

using VolumeListenerContextList = std::vector<std::shared_ptr<VolumeListenerContext>>;

#define LIN_VOLUME_QFACTOR_28 28
#define MAX_VOLUME_CAL_STEPS 15
#define MAX_GAIN_LEVELS 5
#define DEFAULT_CAL_STEP 0

class GlobalConfigs {
    public :
        GlobalConfigs();
        int getVolumeCalSteps() { return mTotalVolumeCalSteps;}
        bool isHeadsetCalEnabled() { return mHeadsetCalEnabled;}
        struct pal_amp_db_and_gain_table * getGainTable() { return mGainMappingTable; }

    private:
        void initGainMappings();
        void printVolumeTable();
    struct pal_amp_db_and_gain_table  mGainMappingTable [MAX_VOLUME_CAL_STEPS] = {
        /* Level 0 in the calibration database contains default calibration */
        { 0.001774, -55, 5 },
        { 0.501187,  -6, 4 },
        { 0.630957,  -4, 3 },
        { 0.794328,  -2, 2 },
        { 1.0,        0, 1 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
        { 0, 0, 0 },
    };

    int mTotalVolumeCalSteps = MAX_GAIN_LEVELS;
    bool mHeadsetCalEnabled = false;
};

class GlobalVolumeListenerSession {
    public :
        static GlobalVolumeListenerSession& getSession() {
            static GlobalVolumeListenerSession instance;
            return instance;
        }
        std::shared_ptr<VolumeListenerContext> createSession(const VolumeListenerType& type, int statusDepth,
                                                    const Parameter::Common& common);
        void releaseSession(int sessionId);
        VolumeListenerContextList getActiveSessions ();
        void dumpSessions();
        GlobalConfigs& getConfig() { return mConfig;}
    private:
        GlobalVolumeListenerSession() {};

        static bool findEffectTypeInList(VolumeListenerContextList & list,
                                     int sessionId, bool remove = false);
        GlobalConfigs mConfig;
        std::mutex mMutex;
        // list of all streams volume contexts
        VolumeListenerContextList mSessionsList;
};

}  // namespace aidl::qti::effects