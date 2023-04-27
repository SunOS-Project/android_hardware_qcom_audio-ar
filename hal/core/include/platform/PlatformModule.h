/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#pragma once

#include <aidl/android/hardware/audio/core/StreamDescriptor.h>
#include <aidl/android/media/audio/common/AudioDeviceDescription.h>
#include <aidl/android/media/audio/common/AudioDeviceType.h>
#include <aidl/android/media/audio/common/AudioProfile.h>
#include <aidl/android/media/audio/common/AudioMode.h>
#include <utils/Conversion.h>
#include <aidlservice/Module.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "PalDefs.h"

namespace {
// see boost::hash_combine
#if defined(__clang__)
__attribute__((no_sanitize("unsigned-integer-overflow")))
#endif
static size_t
hash_combine(size_t seed, size_t v) {
    return std::hash<size_t>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
}  // namespace

namespace std {
template <>
struct hash<aidl::android::media::audio::common::AudioDeviceDescription> {
    std::size_t operator()(
        const aidl::android::media::audio::common::AudioDeviceDescription& add)
        const noexcept {
        return hash_combine(
            std::hash<::aidl::android::media::audio::common::AudioDeviceType>{}(
                add.type),
            std::hash<std::string>{}(add.connection));
    }
};

template <>
struct hash<aidl::android::media::audio::common::AudioFormatDescription> {
    std::size_t operator()(
        const aidl::android::media::audio::common::AudioFormatDescription& aft)
        const noexcept {
        return std::hash<std::string>{}(aft.toString());
    }
};
}  // namespace std

namespace qti::audio::core {
class PlatformModuleParameterHandler;
class PlatformModule : public std::enable_shared_from_this<PlatformModule> {
   public:
    PlatformModule(std::weak_ptr<Module> module);
    PlatformModule();
    ~PlatformModule();
    void init();
    int32_t setParameters(const std::vector<std::string>& keyValuePairs);

    int32_t getParameters(const std::string& keys, std::string& values);

    using DeviceGlobalCallback = std::function<int32_t(
        uint32_t /*event_id*/, uint32_t* /*event_data*/, uint64_t /*cookie*/)>;
    static int32_t registerGlobalCallback(uint32_t eventId, uint32_t* eventData,
                                          uint64_t cookie);

    int32_t setMicMute(bool state);

    int32_t getMicMute(bool& state);

    // virtual int32_t getMicrophones(std::vector<MicrophoneInfo>& micInfo) = 0;

    // dumps the data of the platform module
    // input: valid fd to write
    // returns 0 on success
    int32_t dump(const int32_t fd = 0);

    int32_t setGEFParam(void* data, int length);

    int32_t getGEFParam(void* data, int* length);

    // int setMode(const AudioMode mode) ;

    void setChargingMode(bool charging);

    // return success if handled well
    bool handleDeviceConnectionChange(
        const ::aidl::android::media::audio::common::AudioPort& deviceAudioPort,
        const bool isConnect);

    // return a dynamicProfiles for a supported device port
    std::vector<::aidl::android::media::audio::common::AudioProfile> getDynamicProfiles(
        const ::aidl::android::media::audio::common::AudioPort&
            dynamicAudioPort);

   private:
    void initDeviceMap();
    void dumpDeviceMap();

    AidlToPalDeviceMap mAidlToPalDeviceMap;
    AidlToPalAudioFormatMap mAidlToPalAudioFormatMap;
    ChannelCountToPalChannelInfoMap mChannelCountToPalInfoMap;
    std::weak_ptr<Module> mModule;  // weak reference to Module
    std::shared_ptr<PlatformModuleParameterHandler> mPlatformModuleParamHandler;
};

// TODO move to paramhandler header
class PlatformModuleParameterHandler {
   public:
    PlatformModuleParameterHandler(std::weak_ptr<PlatformModule> module) {
        mPlatformModule = module;
    }

    int32_t setParameters(const std::vector<std::string>& keyValuePairs);

    int32_t getParameters(const std::string& keys, std::string& values);

   private:
    std::weak_ptr<PlatformModule> mPlatformModule;  // weak reference to Module
};
}  // namespace qti::audio::core
