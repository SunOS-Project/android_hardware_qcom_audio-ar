/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Stream.h>
#include <qti-audio-core/AudioUsecase.h>
#include <system/audio_effects/effect_uuid.h>
namespace qti::audio::core {

class StreamInPrimary: public StreamIn, public StreamCommonImpl {
    public:
    friend class ndk::SharedRefBase;
    StreamInPrimary(
            StreamContext&& context,
            const ::aidl::android::hardware::audio::common::SinkMetadata& sinkMetadata,
            const std::vector<::aidl::android::media::audio::common::MicrophoneInfo>& microphones);
    
    virtual ~StreamInPrimary() override;

    std::string toString() const noexcept;

    // Methods of 'DriverInterface'.
    ::android::status_t init() override;
    ::android::status_t drain(::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode) override;
    ::android::status_t flush() override;
    ::android::status_t pause() override;
    ::android::status_t standby() override;
    ::android::status_t start() override;
    ::android::status_t transfer(void* buffer, size_t frameCount, size_t* actualFrameCount,
                                 int32_t* latencyMs) override;
     ::android::status_t refinePosition(
        ::aidl::android::hardware::audio::core::StreamDescriptor::
            Position* /*position*/) override;
    void shutdown() override;
     
    // methods of StreamCommonInterface

    ndk::ScopedAStatus getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
            _aidl_return) override;
    ndk::ScopedAStatus setVendorParameters(
        const std::vector<
            ::aidl::android::hardware::audio::core::VendorParameter>&
            in_parameters,
        bool in_async) override;
    ndk::ScopedAStatus addEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;
    ndk::ScopedAStatus removeEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;

    ndk::ScopedAStatus updateMetadataCommon(const Metadata& metadata) override;

    // Methods called IModule
    ndk::ScopedAStatus setConnectedDevices(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices)
            override;
    
    void onClose() override { defaultOnClose(); }

    protected:
    // This opens, configures and starts pal stream 
    void configure();
    void resume();
    size_t getPeriodSize() const noexcept;
    size_t getPeriodCount() const noexcept;
    size_t getPlatformDelay() const noexcept;

    protected:
    const Usecase mTag;
    const size_t mFrameSizeBytes;
    const int mSampleRate;
    const bool mIsAsynchronous;
    const bool mIsInput;
    pal_stream_handle_t* mPalHandle{nullptr};
    // used to verify a successful configuration of pal stream
    bool mIsConfigured{false};
    std::variant<std::monostate, PcmRecord, CompressCapture, VoipRecord> mExt;
    // references
    Platform& mPlatform {Platform::getInstance()};
    const ::aidl::android::media::audio::common::AudioPortConfig& mMixPortConfig{mContext.getMixPortConfig()};

private:
    bool mAECEnabled = false;
    bool mNSEnabled = false;
};

}  // namespace qti::audio::core