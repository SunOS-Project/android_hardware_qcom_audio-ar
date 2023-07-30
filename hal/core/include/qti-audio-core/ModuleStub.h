/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Module.h>

namespace qti::audio::core {

class ModuleStub final : public Module {
   public:
    ModuleStub() : Module(Type::STUB) {}

   protected:
    ndk::ScopedAStatus getBluetooth(
        std::shared_ptr<::aidl::android::hardware::audio::core::IBluetooth>*
            _aidl_return) override;
    ndk::ScopedAStatus getBluetoothA2dp(
        std::shared_ptr<::aidl::android::hardware::audio::core::IBluetoothA2dp>*
            _aidl_return) override;
    ndk::ScopedAStatus getBluetoothLe(
        std::shared_ptr<::aidl::android::hardware::audio::core::IBluetoothLe>*
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
    ChildInterface<::aidl::android::hardware::audio::core::IBluetooth>
        mBluetooth;
    ChildInterface<::aidl::android::hardware::audio::core::IBluetoothA2dp>
        mBluetoothA2dp;
    ChildInterface<::aidl::android::hardware::audio::core::IBluetoothLe>
        mBluetoothLe;
};

}  // namespace qti::audio::core