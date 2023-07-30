/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Stream.h>

namespace qti::audio::core {

class DriverStub : public DriverInterface {
   public:
    DriverStub(const StreamContext& context, bool isInput);
    ::android::status_t init() override;
    ::android::status_t setConnectedDevices(
        const std::vector<::aidl::android::media::audio::common::AudioDevice>&
            connectedDevices) override;
    ::android::status_t drain(
        ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode)
        override;
    ::android::status_t flush() override;
    ::android::status_t pause() override;
    ::android::status_t transfer(void* buffer, size_t frameCount,
                                 size_t* actualFrameCount,
                                 int32_t* latencyMs) override;
    ::android::status_t standby() override;
    ::android::status_t close() override;
    ::android::status_t prepareToClose() override;
    ::android::status_t updateHwAvSyncId(int32_t in_hwAvSyncId) override;
    ::android::status_t getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
            _aidl_return) override;
    ::android::status_t setVendorParameters(
        const std::vector<
            ::aidl::android::hardware::audio::core::VendorParameter>&
            in_parameters,
        bool in_async) override;
    ::android::status_t addEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;
    ::android::status_t removeEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;

   private:
    const size_t mFrameSizeBytes;
    const int mSampleRate;
    const bool mIsAsynchronous;
    const bool mIsInput;
};

class StreamInStub final : public StreamIn {
   public:
    static ndk::ScopedAStatus createInstance(
        const ::aidl::android::hardware::audio::common::SinkMetadata&
            sinkMetadata,
        StreamContext&& context,
        const std::vector<
            ::aidl::android::media::audio::common::MicrophoneInfo>& microphones,
        std::shared_ptr<StreamIn>* result);

   private:
    friend class ndk::SharedRefBase;
    StreamInStub(const ::aidl::android::hardware::audio::common::SinkMetadata&
                     sinkMetadata,
                 StreamContext&& context,
                 const std::vector<
                     ::aidl::android::media::audio::common::MicrophoneInfo>&
                     microphones);
};

class StreamOutStub final : public StreamOut {
   public:
    static ndk::ScopedAStatus createInstance(
        const ::aidl::android::hardware::audio::common::SourceMetadata&
            sourceMetadata,
        StreamContext&& context,
        const std::optional<
            ::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo,
        std::shared_ptr<StreamOut>* result);

   private:
    friend class ndk::SharedRefBase;
    StreamOutStub(
        const ::aidl::android::hardware::audio::common::SourceMetadata&
            sourceMetadata,
        StreamContext&& context,
        const std::optional<
            ::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo);
};

}  // namespace qti::audio::core