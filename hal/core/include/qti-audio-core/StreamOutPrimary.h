/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/AudioUsecase.h>
#include <qti-audio-core/HalOffloadEffects.h>
#include <qti-audio-core/Stream.h>

namespace qti::audio::core {

class StreamOutPrimary : public StreamOut, public StreamCommonImpl {
  public:
    friend class ndk::SharedRefBase;
    StreamOutPrimary(StreamContext&& context,
                     const ::aidl::android::hardware::audio::common::SourceMetadata& sourceMetadata,
                     const std::optional<::aidl::android::media::audio::common::AudioOffloadInfo>&
                             offloadInfo);

    virtual ~StreamOutPrimary() override;
    int32_t setAggregateSourceMetadata(bool voiceActive) override;

    operator const char*() const noexcept;

    // Methods of 'DriverInterface'.
    ::android::status_t init() override;
    ::android::status_t drain(
            ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode) override;
    ::android::status_t flush() override;
    ::android::status_t pause() override;
    ::android::status_t standby() override;
    ::android::status_t start() override;
    ::android::status_t transfer(void* buffer, size_t frameCount, size_t* actualFrameCount,
                                 int32_t* latencyMs) override;
    ::android::status_t refinePosition(
            ::aidl::android::hardware::audio::core::StreamDescriptor::Reply*
            /*reply*/) override;
    void shutdown() override;

    // methods of StreamCommonInterface

    ndk::ScopedAStatus getVendorParameters(
            const std::vector<std::string>& in_ids,
            std::vector<::aidl::android::hardware::audio::core::VendorParameter>* _aidl_return)
            override;
    ndk::ScopedAStatus setVendorParameters(
            const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
                    in_parameters,
            bool in_async) override;
    ndk::ScopedAStatus addEffect(
            const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>& in_effect)
            override;
    ndk::ScopedAStatus removeEffect(
            const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>& in_effect)
            override;

    ndk::ScopedAStatus updateMetadataCommon(const Metadata& metadata) override;

    // Methods of IStreamOut
    ndk::ScopedAStatus updateOffloadMetadata(
            const ::aidl::android::hardware::audio::common::AudioOffloadMetadata&
                    in_offloadMetadata) override;

    ndk::ScopedAStatus getHwVolume(std::vector<float>* _aidl_return) override;
    ndk::ScopedAStatus setHwVolume(const std::vector<float>& in_channelVolumes) override;

    // Methods called IModule
    ndk::ScopedAStatus setConnectedDevices(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices)
            override;
    ndk::ScopedAStatus configureMMapStream(int32_t* fd, int64_t* burstSizeFrames, int32_t* flags,
                                           int32_t* bufferSizeFrames) override;

    void onClose() override { defaultOnClose(); }

    bool isStreamOutPrimary() { return (mTag == Usecase::PRIMARY_PLAYBACK) ? true : false; }
    static std::mutex sourceMetadata_mutex_;

  protected:
    /*
     * This API opens, configures and starts pal stream.
     * also responsible for validity of pal handle.
     */
    void configure();
    void resume();
    size_t getPeriodSize() const noexcept;
    size_t getPeriodCount() const noexcept;
    size_t getPlatformDelay() const noexcept;
    ::android::status_t onWriteError(const size_t sleepFrameCount);

    // This API calls startEffect/stopEffect only on offload/pcm offload
    // outputs.
    void enableOffloadEffects(const bool enable);

  protected:
    const Usecase mTag;
    const std::string mTagName;
    const size_t mFrameSizeBytes;
    bool mIsPaused{false};
    std::vector<float> mVolumes{};

    // All the public must check the validity of this resource, if using
    pal_stream_handle_t* mPalHandle{nullptr};

    std::variant<std::monostate, PrimaryPlayback, DeepBufferPlayback, CompressPlayback,
                 PcmOffloadPlayback, VoipPlayback, SpatialPlayback, MMapPlayback, UllPlayback,
                 InCallMusic>
            mExt;
    // references
    Platform& mPlatform{Platform::getInstance()};
    const ::aidl::android::media::audio::common::AudioPortConfig& mMixPortConfig;
    HalOffloadEffects& mHalEffects{HalOffloadEffects::getInstance()};
    AudioExtension& mAudExt{AudioExtension::getInstance()};
};

} // namespace qti::audio::core
