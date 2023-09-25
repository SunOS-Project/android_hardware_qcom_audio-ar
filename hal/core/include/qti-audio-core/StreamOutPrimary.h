/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Stream.h>
#include <qti-audio-core/AudioUsecase.h>
#include <qti-audio-core/HalOffloadEffects.h>

namespace qti::audio::core {

class StreamOutPrimary: public StreamOut, public StreamCommonImpl {
    public:
    friend class ndk::SharedRefBase;
    StreamOutPrimary(StreamContext&& context,
                  const ::aidl::android::hardware::audio::common::SourceMetadata& sourceMetadata,
                  const std::optional<::aidl::android::media::audio::common::AudioOffloadInfo>&
                          offloadInfo);

    virtual ~StreamOutPrimary() override;
    int32_t setAggregateSourceMetadata(bool voiceActive) override;

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

    // Methods of IStreamOut
    ndk::ScopedAStatus updateOffloadMetadata(
        const ::aidl::android::hardware::audio::common::AudioOffloadMetadata&
            in_offloadMetadata) override;

    ndk::ScopedAStatus getHwVolume(std::vector<float>* _aidl_return) override;
    ndk::ScopedAStatus setHwVolume(
        const std::vector<float>& in_channelVolumes) override;

    // Methods called IModule
    ndk::ScopedAStatus setConnectedDevices(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices)
            override;

    void onClose() override { defaultOnClose(); }
    AudioExtension& mAudExt{AudioExtension::getInstance()};
    bool isStreamOutPrimary() { return (mTag == Usecase::PRIMARY_PLAYBACK) ? true: false; }
    static std::mutex sourceMetadata_mutex_;
    protected:
    // This opens, configures and starts pal stream 
    void configure();
    void resume();
    size_t getPeriodSize() const noexcept;
    size_t getPeriodCount() const noexcept;
    size_t getPlatformDelay() const noexcept;

    private:
    const Usecase mTag;
    const size_t mFrameSizeBytes;
    const int mSampleRate;
    const bool mIsAsynchronous;
    const bool mIsInput;
    bool mIsInitialized{false};  // Used for validating the state machine logic.
    bool mIsStandby{true};       // Used for validating the state machine logic.
    bool mIsPaused{false};
    std::vector<float> mVolumes{};
    pal_stream_handle_t* mPalHandle{nullptr};
    /**
     * used to verify a successful configuration of pal stream
     * on true expected mPalHandle is valid pal stream handle (hardware up)
     * on false expected mPalHandle is nullptr (hardware down)
     **/
    bool mIsConfigured{false};
    std::variant<std::monostate, PrimaryPlayback, DeepBufferPlayback,
                 CompressPlayback, PcmOffloadPlayback, VoipPlayback,
                 SpatialPlayback>
        mExt;
    // references
    Platform& mPlatform {Platform::getInstance()};
    const ::aidl::android::media::audio::common::AudioPortConfig& mMixPortConfig{mContext.getMixPortConfig()};
    int mIoHandle;

    HalOffloadEffects& mHalEffects {HalOffloadEffects::getInstance()};

    // This API calls startEffect/stopEffect only on offload/pcm offload outputs.
    void enableOffloadEffects(bool enable);
};


}  // namespace qti::audio::core
