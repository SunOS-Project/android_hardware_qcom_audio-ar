/*
 * Copyright (C) 2023 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * ​​​​​Changes from Qualcomm Innovation Center are provided under the following license:
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
    ndk::ScopedAStatus setVendorParameters(
        const std::vector<::aidl::android::hardware::audio::core::VendorParameter>& in_parameters,
        bool in_async) override;
    ndk::ScopedAStatus getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>* _aidl_return) override;
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