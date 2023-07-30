/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Module.h>

namespace qti::audio::core {

class ModulePrimary final : public Module {
   public:
    ModulePrimary() : Module(Type::DEFAULT) {}

   protected:
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

   private:
    ChildInterface<::aidl::android::hardware::audio::core::ITelephony>
        mTelephony;
};

}  // namespace qti::audio::core