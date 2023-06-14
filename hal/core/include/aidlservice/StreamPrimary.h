/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidlservice/Stream.h>
#include <qti-audio-core/Platform.h>

namespace qti::audio::core {

class DriverPrimary : public DriverInterface {
   public:
    static std::unique_ptr<AudioUsecase> createAudioUsecase(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig,
        const std::optional<
            ::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo,
        std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback>
            mAsyncCallback,
        const bool isInput);
    DriverPrimary(const StreamContext& context,
                  const std::optional<
                      ::aidl::android::media::audio::common::AudioOffloadInfo>&
                      offloadInfo,
                  const bool isInput);
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

   public:
    bool isPaused() const { return mIsPaused; }
    void setPaused(bool isPaused) { mIsPaused = isPaused; }
    bool isConfigured() const { return mIsConfigured; }
    void setConfigured(bool isConfigured) { mIsConfigured = isConfigured; }

   public:
    std::unique_ptr<AudioUsecase> mUsecase;

   private:
    bool mIsPaused{false};
    bool mIsConfigured{false};
    const size_t mFrameSizeBytes;
    const int mSampleRate;
    const bool mIsAsynchronous;
    const bool mIsInput;
};

class StreamInPrimary final : public StreamIn {
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
    StreamInPrimary(
        const ::aidl::android::hardware::audio::common::SinkMetadata&
            sinkMetadata,
        StreamContext&& context,
        const std::vector<
            ::aidl::android::media::audio::common::MicrophoneInfo>&
            microphones);
};

class StreamOutPrimary final : public StreamOut {
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
    StreamOutPrimary(
        const ::aidl::android::hardware::audio::common::SourceMetadata&
            sourceMetadata,
        StreamContext&& context,
        const std::optional<
            ::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo);
};

}  // namespace qti::audio::core