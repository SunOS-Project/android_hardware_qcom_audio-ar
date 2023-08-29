/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <algorithm>
#include <cstddef>
#include <memory>
#define LOG_TAG "AHAL_Effect_VolumeListener"
#include <unordered_set>
#include <cutils/properties.h>

#include "GlobalVolumeListenerSession.h"

namespace aidl::qti::effects {

GlobalConfigs::GlobalConfigs() {
    initGainMappings();
    mHeadsetCalEnabled = property_get_bool("vendor.audio.volume.headset.gain.depcal", false);
}

void GlobalConfigs::initGainMappings() {
    size_t payloadSize = 0;
    pal_param_gain_lvl_map_t gainLevelMap;
    gainLevelMap.mapping_tbl = mGainMappingTable;
    gainLevelMap.table_size  = MAX_VOLUME_CAL_STEPS;
    gainLevelMap.filled_size = 0;

    printVolumeTable();

    int ret = pal_get_param(PAL_PARAM_ID_GAIN_LVL_MAP,
            (void **)&gainLevelMap,
            &payloadSize, nullptr);

    if (ret != 0) {
        LOG(ERROR) << "fail to get PAL_PARAM_ID_GAIN_LVL_MAP " << ret;
        gainLevelMap.filled_size = 0;
    }

    int maxTableEntries = gainLevelMap.filled_size;

    if (maxTableEntries > 0 && maxTableEntries <= MAX_VOLUME_CAL_STEPS) {
        if (maxTableEntries < mTotalVolumeCalSteps) {
            for (int i = maxTableEntries; i < mTotalVolumeCalSteps; i++ ) {
                mGainMappingTable[i].amp = 0;
                mGainMappingTable[i].db = 0;
                mGainMappingTable[i].level = 0;
            }
        }
        mTotalVolumeCalSteps = maxTableEntries;
        LOG(INFO) <<"Using custom volume table";
    } else {
        LOG(INFO) <<"Using default volume table";
    }
    printVolumeTable();
}

void GlobalConfigs::printVolumeTable() {
    for (int i = 0; i < mTotalVolumeCalSteps; i++) {
        LOG(DEBUG) << "Index: " << i << " (amp, db, level)" << mGainMappingTable[i].amp << " " << mGainMappingTable[i].db << " "<<mGainMappingTable[i].level;
    }
}

bool GlobalVolumeListenerSession::findEffectTypeInList(VolumeListenerContextList &list,
                                                       int sessionId, bool remove)
{
    auto itr = std::find_if(list.begin(), list.end(),
                            [sessionId](const std::shared_ptr<VolumeListenerContext> &obj)
                            {
                                return obj->getSessionId() == sessionId;
                            });
    if (itr == list.end()) {
        return false;
    }
    if (remove) {
        list.erase(itr);
    }

    return true;
}

std::shared_ptr<VolumeListenerContext> GlobalVolumeListenerSession :: createSession
                                                    (const VolumeListenerType &type,
                                                     int statusDepth,
                                                     const Parameter::Common &common)
{
    int sessionId = common.session;
    LOG(DEBUG) << __func__ << type << " with sessionId " << sessionId;
    std::lock_guard lg(mMutex);

    auto context = std::make_shared<VolumeListenerContext>(statusDepth, common, type);
    RETURN_VALUE_IF(!context, nullptr, "failedToCreateContext");
    mSessionsList.push_back(context);

    return context;
}

void GlobalVolumeListenerSession::releaseSession(int sessionId)
{
    LOG(DEBUG) << __func__ << " sessionId " << sessionId;
    std::lock_guard lg(mMutex);
    if (!findEffectTypeInList(mSessionsList, sessionId, true /* remove */)) {
        LOG(ERROR) << __func__ << " can't find "
                   << " session " << sessionId;
    }
}

VolumeListenerContextList GlobalVolumeListenerSession::getActiveSessions() {
    std::lock_guard lg(mMutex);
    return mSessionsList;
}

void GlobalVolumeListenerSession::dumpSessions() {
    std::lock_guard lg(mMutex);
    for (auto &session : mSessionsList) {
        session->dump();
    }
}

}  // namespace aidl::qti::effects
