/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#define LOG_TAG "AHAL_Effect_GlobalOffloadSession"

#include <algorithm>
#include <memory>
#include <unordered_map>

#include <android-base/logging.h>
#include <android-base/thread_annotations.h>

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"

namespace aidl::qti::effects{

/**
 * @brief Maintain all effect offload bundle sessions.
 *
 */
class GlobalOffloadSession {
  public:
    static GlobalOffloadSession& getGlobalSession() {
        static GlobalOffloadSession instance;
        return instance;
    }

    static bool findBundleTypeInList(std::vector<std::shared_ptr<OffloadBundleContext>>& list,
                                     const OffloadBundleEffectType& type, bool remove = false) {
        auto itr = std::find_if(list.begin(), list.end(),
                                  [type](const std::shared_ptr<OffloadBundleContext>& bundle) {
                                      return bundle->getBundleType() == type;
                                  });
        if (itr == list.end()) {
            return false;
        }
        if (remove) {
            (*itr)->deInit(); // call release inside of it.
            list.erase(itr);
        }
        return true;
    }

    std::shared_ptr<OffloadBundleContext> createContext(const OffloadBundleEffectType& type, int statusDepth,
                                                 const Parameter::Common& common) {
        switch (type) {
            case OffloadBundleEffectType::BASS_BOOST:
                return std::make_shared<BassBoostContext> (statusDepth, common, type);
            case OffloadBundleEffectType::EQUALIZER:
                return std::make_shared<EqualizerContext>(statusDepth, common, type);
            case OffloadBundleEffectType::VIRTUALIZER:
                return std::make_shared<VirtualizerContext>(statusDepth, common, type);
            case OffloadBundleEffectType::AUX_ENV_REVERB:
            case OffloadBundleEffectType::INSERT_ENV_REVERB:
            case OffloadBundleEffectType::AUX_PRESET_REVERB:
            case OffloadBundleEffectType::INSERT_PRESET_REVERB:
                return std::make_shared<ReverbContext>(statusDepth, common, type);
        }
        return nullptr;
    }

    /**
     * Create a certain type of BundleContext in shared_ptr container, each session must not have
     * more than one session for each type.
     */
    std::shared_ptr<OffloadBundleContext> createSession(const OffloadBundleEffectType& type, int statusDepth,
                                                 const Parameter::Common& common) {
        int sessionId = common.session;
        LOG(INFO) << __func__ <<"  " << type << " sessionId " << sessionId <<" effects "
                << mSessionMap.size();
        std::lock_guard lg(mMutex);

        if (mSessionMap.count(sessionId)) {
            if (findBundleTypeInList(mSessionMap[sessionId], type)) {
                LOG(ERROR) << __func__ << type << " already exist in  " << sessionId;
                return nullptr;
            }
        }

        auto& list = mSessionMap[sessionId];
        auto context = createContext(type, statusDepth, common);
        RETURN_VALUE_IF(!context, nullptr, "failedToCreateContext");

        RetCode ret = context->init();
        if (RetCode::SUCCESS != ret) {
            LOG(ERROR) << __func__ << " context init ret " << ret;
            return nullptr;
        }
        list.push_back(context);

        // find ioHandle in the mActiveIoHandles
        // for (auto pair : mActiveSessions) {
        //     if (pair.first == sessionId) {
        //         context->start(pair.second);
        //     }
        // }
        return context;
    }

    void releaseSession(const OffloadBundleEffectType& type, int sessionId) {
        LOG(DEBUG) << __func__ << type << " sessionId " << sessionId;
        std::lock_guard lg(mMutex);
        if (mSessionMap.count(sessionId)) {
            auto& list = mSessionMap[sessionId];
            if (!findBundleTypeInList(list, type, true /* remove */)) {
                LOG(ERROR) << __func__ << " can't find " << type << "in session " << sessionId;
                return;
            }
            if (list.empty()) {
                mSessionMap.erase(sessionId);
            }
        }
    }

    // Used by AudioHal to link effect with output.
    // void startEffect (int sessionId, pal_stream_handle_t* palHandle) {

    //     std::lock_guard lg(mMutex);
    //     // check if sessionId is created.
    //     if (mSessionMap.count(sessionId)) {
    //         auto &list = mSessionMap[sessionId]; // list of effects.
    //         for (const auto & context : list) {
    //             context->start(palHandle);
    //         }
    //     }
    //     mActiveSessions.push_back(std::make_pair(sessionId, palHandle));
    // }

    // Used by AudioHal to link effect with output.
    // void stopEffect (int sessionId) {
    //     std::lock_guard lg(mMutex);
    //     // check if sessionId is created.
    //     if (mSessionMap.count(sessionId)) {
    //         auto &list = mSessionMap[sessionId]; // list of effects.
    //         for (const auto & context : list) {
    //             context->stop();
    //         }
    //     }
    //     // // remove entry of sessionId from mActiveIoHandles;
    //     // auto itr = std::find_if(mActiveSessions.begin(), mActiveSessions.end(),
    //     //                           [sessionId](const std::pair<int, pal_stream_handle_t*> handles) {
    //     //                               return sessionId == handles.first;
    //     //                           });
    //     // if (itr == mActiveSessions.end()) {
    //     //     LOG(ERROR) <<" should not happen"; // do better error handling
    //     // } else {
    //     //     mActiveSessions.erase(itr);
    //     // }
    // }

  private:
    // Lock for mSessionMap access.
    std::mutex mMutex;

    // map between sessionId and list of effect contexts for that sessionId
    std::unordered_map<int /* sessionId */, std::vector<std::shared_ptr<OffloadBundleContext>>>
            mSessionMap GUARDED_BY(mMutex);
    
    // keep track of existing active sessions
    std::vector<std::pair<int /*sessionId*/, pal_stream_handle_t* /*palHandle*/> > mActiveSessions;
};
}  // namespace aidl::qti::effects
