/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Module.h>
#include <qti-audio-core/Platform.h>

namespace qti::audio::core {

class ModulePrimary final : public Module {
   public:
    ModulePrimary() : Module(Type::DEFAULT) {}

    std::string toStringInternal();
    void dumpInternal() ;

   protected:
    binder_status_t dump(int fd, const char** args, uint32_t numArgs) override;
    ndk::ScopedAStatus getTelephony(
        std::shared_ptr<::aidl::android::hardware::audio::core::ITelephony>*
            _aidl_return) override;

    ndk::ScopedAStatus createInputStream(
        StreamContext&& context,
        const ::aidl::android::hardware::audio::common::SinkMetadata&
            sinkMetadata,
        const std::vector<
            ::aidl::android::media::audio::common::MicrophoneInfo>& microphones,
        std::shared_ptr<StreamIn>* result) override;
    ndk::ScopedAStatus createOutputStream(
        StreamContext&& context,
        const ::aidl::android::hardware::audio::common::SourceMetadata&
            sourceMetadata,
        const std::optional<
            ::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo,
        std::shared_ptr<StreamOut>* result) override;
    std::vector<::aidl::android::media::audio::common::AudioProfile>
    getDynamicProfiles(
        const ::aidl::android::media::audio::common::AudioPort& audioPort) override;
    void onNewPatchCreation(
        const std::vector<
            ::aidl::android::media::audio::common::AudioPortConfig*>& sources,
        const std::vector<
            ::aidl::android::media::audio::common::AudioPortConfig*>& sinks,
        ::aidl::android::hardware::audio::core::AudioPatch& newPatch) override;
    void onExternalDeviceConnectionChanged(
        const ::aidl::android::media::audio::common::AudioPort& audioPort,
        bool connected) override;


   protected:
    ChildInterface<::aidl::android::hardware::audio::core::ITelephony>
        mTelephony;
    Platform& mPlatform{Platform::getInstance()};
};

}  // namespace qti::audio::core