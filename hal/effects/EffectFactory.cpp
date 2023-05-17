/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <iterator>
#include <memory>
#include <tuple>
#include "include/effect-impl/EffectTypes.h"
#define LOG_TAG "AHAL_EffectFactory"
#include <dlfcn.h>
#include <unordered_set>

#include <android-base/logging.h>
#include <android/binder_ibinder_platform.h>
#include <system/thread_defs.h>

#include "effect-impl/EffectTypes.h"
#include "effect-impl/EffectUUID.h"
#include "effectFactory-impl/EffectFactory.h"

using aidl::android::media::audio::common::AudioUuid;

namespace aidl::android::hardware::audio::effect {

Factory::Factory(const std::string& file) : mConfig(EffectConfig(file)) {
    LOG(DEBUG) << __func__ << " with config file: " << file;
    //loadEffectLibs();
}

Factory::~Factory() {
    if (auto count = mEffectMap.size()) {
        LOG(ERROR) << __func__ << " remaining " << count
                   << " effect instances not destroyed indicating resource leak!";
        for (const auto& it : mEffectMap) {
            if (auto spEffect = it.first.lock()) {
                LOG(ERROR) << __func__ << " erase remaining instance UUID "
                           << it.second.first.toString();
                destroyEffectImpl(spEffect);
            }
        }
    }
}

ndk::ScopedAStatus Factory::queryEffects(const std::optional<AudioUuid>& in_type_uuid,
                                         const std::optional<AudioUuid>& in_impl_uuid,
                                         const std::optional<AudioUuid>& in_proxy_uuid,
                                         std::vector<Descriptor>* _aidl_return) {
    // get the matching list
    std::vector<Descriptor::Identity> idList;
    std::copy_if(mIdentitySet.begin(), mIdentitySet.end(), std::back_inserter(idList),
                 [&](auto& id) {
                     return (!in_type_uuid.has_value() || in_type_uuid.value() == id.type) &&
                            (!in_impl_uuid.has_value() || in_impl_uuid.value() == id.uuid) &&
                            (!in_proxy_uuid.has_value() ||
                             (id.proxy.has_value() && in_proxy_uuid.value() == id.proxy.value()));
                 });
    // query through the matching list
    for (const auto& id : idList) {
        if (mEffectLibMap.count(id.uuid)) {
            Descriptor desc;
            auto& entry = mEffectLibMap[id.uuid];
            getDlSyms(entry);
            auto& libInterface = std::get<kMapEntryInterfaceIndex>(entry);
            RETURN_IF(!libInterface || !libInterface->queryEffectFunc, EX_NULL_POINTER,
                      "dlNullQueryEffectFunc");
            RETURN_IF_BINDER_EXCEPTION(libInterface->queryEffectFunc(&id.uuid, &desc));
            _aidl_return->emplace_back(std::move(desc));
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Factory::queryProcessing(const std::optional<Processing::Type>& in_type,
                                            std::vector<Processing>* _aidl_return) {
    // TODO: implement this with audio_effect.xml.
    if (in_type.has_value()) {
        // return all matching process filter
        LOG(DEBUG) << __func__ << " process type: " << in_type.value().toString();
    }
    LOG(DEBUG) << __func__ << " return " << _aidl_return->size();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Factory::createEffect(const AudioUuid& in_impl_uuid,
                                         std::shared_ptr<IEffect>* _aidl_return) {
    LOG(DEBUG) << __func__ << ": UUID " << in_impl_uuid.toString();
    if (mEffectLibMap.count(in_impl_uuid)) {
        auto& entry = mEffectLibMap[in_impl_uuid];
        getDlSyms(entry);

        auto& libInterface = std::get<kMapEntryInterfaceIndex>(entry);
        RETURN_IF(!libInterface || !libInterface->createEffectFunc, EX_NULL_POINTER,
                  "dlNullcreateEffectFunc");
        std::shared_ptr<IEffect> effectSp;
        RETURN_IF_BINDER_EXCEPTION(libInterface->createEffectFunc(&in_impl_uuid, &effectSp));
        if (!effectSp) {
            LOG(ERROR) << __func__ << ": library created null instance without return error!";
            return ndk::ScopedAStatus::fromExceptionCode(EX_TRANSACTION_FAILED);
        }
        *_aidl_return = effectSp;
        ndk::SpAIBinder effectBinder = effectSp->asBinder();
        AIBinder_setMinSchedulerPolicy(effectBinder.get(), SCHED_NORMAL, ANDROID_PRIORITY_AUDIO);
        mEffectMap[std::weak_ptr<IEffect>(effectSp)] =
                std::make_pair(in_impl_uuid, std::move(effectBinder));
        LOG(DEBUG) << __func__ << ": instance " << effectSp.get() << " created successfully";
        return ndk::ScopedAStatus::ok();
    } else {
        LOG(ERROR) << __func__ << ": library doesn't exist";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Factory::destroyEffectImpl(const std::shared_ptr<IEffect>& in_handle) {
    std::weak_ptr<IEffect> wpHandle(in_handle);
    // find the effect entry with key (std::weak_ptr<IEffect>)
    if (auto effectIt = mEffectMap.find(wpHandle); effectIt != mEffectMap.end()) {
        auto& uuid = effectIt->second.first;
        // find implementation library with UUID
        if (auto libIt = mEffectLibMap.find(uuid); libIt != mEffectLibMap.end()) {
            auto& interface = std::get<kMapEntryInterfaceIndex>(libIt->second);
            RETURN_IF(!interface || !interface->destroyEffectFunc, EX_NULL_POINTER,
                      "dlNulldestroyEffectFunc");
            RETURN_IF_BINDER_EXCEPTION(interface->destroyEffectFunc(in_handle));
        } else {
            LOG(ERROR) << __func__ << ": UUID " << uuid.toString() << " does not exist in libMap!";
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
        mEffectMap.erase(effectIt);
        return ndk::ScopedAStatus::ok();
    } else {
        LOG(ERROR) << __func__ << ": instance " << in_handle << " does not exist!";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
}

// go over the map and cleanup all expired weak_ptrs.
void Factory::cleanupEffectMap() {
    for (auto it = mEffectMap.begin(); it != mEffectMap.end();) {
        if (nullptr == it->first.lock()) {
            it = mEffectMap.erase(it);
        } else {
            ++it;
        }
    }
}

ndk::ScopedAStatus Factory::destroyEffect(const std::shared_ptr<IEffect>& in_handle) {
    LOG(DEBUG) << __func__ << ": instance " << in_handle.get();
    ndk::ScopedAStatus status = destroyEffectImpl(in_handle);
    // always do the cleanup
    cleanupEffectMap();
    return status;
}

// TODO move to utils
std::string printAudioUuid(const AudioUuid &uuid) {
    std::ostringstream os;
    char str[64];
    snprintf(str, sizeof(str), "%08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
             uuid.timeLow, uuid.timeMid, uuid.timeHiAndVersion, uuid.clockSeq,
             uuid.node[0], uuid.node[1], uuid.node[2], uuid.node[3], uuid.node[4], uuid.node[5]);

    os << "Uuid : [";
    os << str;
    os << "]";
    return os.str();
}

bool Factory::openEffectLibrary(const AudioUuid& impl, const std::string& path) {
    std::function<void(void*)> dlClose = [](void* handle) -> void {
        if (handle && dlclose(handle)) {
            LOG(ERROR) << "dlclose failed " << dlerror();
        }
    };

    auto libHandle =
            std::unique_ptr<void, decltype(dlClose)>{dlopen(path.c_str(), RTLD_LAZY), dlClose};
    if (!libHandle) {
        LOG(ERROR) << __func__ << ": dlopen failed, err: " << dlerror();
        return false;
    }

    LOG(INFO) << __func__ << " dlopen lib: " << path.c_str() << " Impl " << printAudioUuid(impl)
              << "handle:" << libHandle;
    auto interface = new effect_dl_interface_s{nullptr, nullptr, nullptr};
    mEffectLibMap.insert(
            {impl,
             std::make_tuple(std::move(libHandle),
                             std::unique_ptr<struct effect_dl_interface_s>(interface), path)});
    return true;
}

void Factory::createIdentityWithConfig(const EffectConfig::LibraryUuid& configLib,
                                       const AudioUuid& typeUuid,
                                       const std::optional<AudioUuid> proxyUuid) {
    static const auto& libMap = mConfig.getLibraryMap();
    const std::string& libName = configLib.name;
    if (auto path = libMap.find(libName); path != libMap.end()) {
        Descriptor::Identity id;
        id.type = typeUuid;
        id.uuid = configLib.uuid;
        id.proxy = proxyUuid;
        LOG(INFO) << __func__ <<" name: " << libName << ": Type " << printAudioUuid(id.type) <<
                    ": Impl " << printAudioUuid(id.uuid) << 
                    ": Proxy " << (proxyUuid.has_value() ? printAudioUuid(proxyUuid.value()) : "null");

        if (openEffectLibrary(id.uuid, path->second)) {
            mIdentitySet.insert(std::move(id));
        }
    } else {
        LOG(ERROR) << __func__ << ": library " << libName << " not exist!";
        return;
    }
}

void Factory::loadEffectLibs() {
    const auto& configEffectsMap = mConfig.getEffectsMap();
    for (const auto& configEffects : configEffectsMap) {
        if (auto typeUuid = kUuidNameTypeMap.find(configEffects.first /* effect name */);
            typeUuid != kUuidNameTypeMap.end()) {
            const auto& configLibs = configEffects.second;
            std::optional<AudioUuid> proxyUuid;
            if (configLibs.proxyLibrary.has_value()) {
                const auto& proxyLib = configLibs.proxyLibrary.value();
                proxyUuid = proxyLib.uuid;
            }
            for (const auto& configLib : configLibs.libraries) {
                createIdentityWithConfig(configLib, typeUuid->second, proxyUuid);
            }
        } else {
            LOG(ERROR) << __func__ << ": can not find type UUID for effect " << configEffects.first
                       << " skipping!";
        }
    }
}

void Factory::getDlSyms(DlEntry& entry) {
    auto& dlHandle = std::get<kMapEntryHandleIndex>(entry);
    RETURN_VALUE_IF(!dlHandle, void(), "dlNullHandle");
    // Get the reference of the DL interfaces in library map tuple.
    auto& dlInterface = std::get<kMapEntryInterfaceIndex>(entry);
    // return if interface already exist
    if (!dlInterface->createEffectFunc) {
        dlInterface->createEffectFunc = (EffectCreateFunctor)dlsym(dlHandle.get(), "createEffect");
    }
    if (!dlInterface->queryEffectFunc) {
        dlInterface->queryEffectFunc = (EffectQueryFunctor)dlsym(dlHandle.get(), "queryEffect");
    }
    if (!dlInterface->destroyEffectFunc) {
        dlInterface->destroyEffectFunc =
                (EffectDestroyFunctor)dlsym(dlHandle.get(), "destroyEffect");
    }

    if (!dlInterface->createEffectFunc || !dlInterface->destroyEffectFunc ||
        !dlInterface->queryEffectFunc) {
        LOG(ERROR) << __func__ << ": create (" << dlInterface->createEffectFunc << "), query ("
                   << dlInterface->queryEffectFunc << "), or destroy ("
                   << dlInterface->destroyEffectFunc
                   << ") not exist in library: " << std::get<kMapEntryLibNameIndex>(entry)
                   << " handle: " << dlHandle << " with dlerror: " << dlerror();
        return;
    }
}

}  // namespace aidl::android::hardware::audio::effect
