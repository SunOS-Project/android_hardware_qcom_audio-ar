/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#pragma once

#include <qti-audio-core/AudioUsecaseExt.h>
#include <Utils.h>
#include <aidl/android/hardware/audio/core/StreamDescriptor.h>
#include <aidl/android/media/audio/common/AudioChannelLayout.h>
#include <aidl/android/media/audio/common/AudioDevice.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>
#include <aidl/android/hardware/audio/core/VendorParameter.h>
#include <aidl/android/media/audio/common/AudioOffloadInfo.h>
#include <aidl/android/hardware/audio/effect/IEffect.h>
#include <android-base/properties.h>

namespace qti::audio::core {

class AudioUsecase {
   public:
    enum class Tag : uint16_t {
        INVALID = 0,
        PRIMARY_PLAYBACK,
        DEEP_BUFFER_PLAYBACK,
        LOW_LATENCY_PLAYBACK,
        PCM_RECORD,
        COMPRESS_OFFLOAD_PLAYBACK,
        COMPRESS_CAPTURE,
        PCM_OFFLOAD_PLAYBACK,
        VOIP_PLAYBACK,
        SPATIAL_PLAYBACK,
        VOIP_RECORD,
    };

   public:
    static Tag getUsecaseTag(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig);
    static std::string getName(const Tag tag);

    AudioUsecase(
        const ::aidl::android::media::audio::common::AudioPortConfig& pc,
        const std::optional<::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo, std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback> mAsyncCallback,
        const bool isInput);
    virtual ~AudioUsecase();
    std::string toString() const noexcept;

    bool configure();
    uint32_t getLatency();
    int32_t pause();
    int32_t resume();
    int32_t flush();
    int32_t write(uint8_t* dataPtr, size_t frameCount);
    int32_t read(uint8_t* data, size_t frameCount, size_t* actualFrameCount);
    int32_t drain(
        ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode
            mode);
    int32_t getPresentationPosition(int64_t* positionInFrames);
    int32_t standBy();
    bool isConfigured() const { return mIsConfigured; }
    bool isPaused() const { return mIsPaused; }

    /* StreamCommon APIs start */
    int32_t close();
    int32_t prepareToClose();
    int32_t updateHwAvSyncId(int32_t in_hwAvSyncId);
    int32_t getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
            _aidl_return);
    int32_t setVendorParameters(
        const std::vector<
            ::aidl::android::hardware::audio::core::VendorParameter>&
            in_parameters,
        bool in_async);
    int32_t addEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect);
    int32_t removeEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect);
    /* StreamCommon APIs end */

    const ::aidl::android::media::audio::common::AudioPortConfig&
    getPortConfig() {
        return mPortConfig;
    }
    void setConnectedDevices(
        const std::vector<::aidl::android::media::audio::common::AudioDevice>&
            connectedDevices);
    const std::vector<::aidl::android::media::audio::common::AudioDevice>&
    getConnectedDevices() {
        return mConnectedDevices;
    }

   protected:
    int32_t getSampleRate() const {
        return mPortConfig.sampleRate.value().value;
    }
    ::aidl::android::media::audio::common::AudioChannelLayout&
    getChannelLayout() {
        return mPortConfig.channelMask.value();
    }
    ::aidl::android::media::audio::common::AudioFormatDescription& getFormat() {
        return mPortConfig.format.value();
    }
    std::optional<::aidl::android::media::audio::common::AudioSource>
    getMixInputSourceType() {
        auto& mixUsecase = mPortConfig.ext
                               .get<::aidl::android::media::audio::common::
                                        AudioPortExt::Tag::mix>()
                               .usecase;
        if (mixUsecase.getTag() != ::aidl::android::media::audio::common::
                                       AudioPortMixExtUseCase::Tag::source) {
            return std::nullopt;
        }
        return mixUsecase.get<::aidl::android::media::audio::common::
                                  AudioPortMixExtUseCase::Tag::source>();
    }
    std::optional<::aidl::android::media::audio::common::AudioStreamType>
    getMixOutputStreamType() {
        auto& mixUsecase = mPortConfig.ext
                               .get<::aidl::android::media::audio::common::
                                        AudioPortExt::Tag::mix>()
                               .usecase;
        if (mixUsecase.getTag() != ::aidl::android::media::audio::common::
                                       AudioPortMixExtUseCase::Tag::stream) {
            return std::nullopt;
        }
        return mixUsecase.get<::aidl::android::media::audio::common::
                                  AudioPortMixExtUseCase::Tag::stream>();
    }

    void configurePalDevices(std::vector<pal_device>& palDevices);
    size_t getPeriodSize() const noexcept;
    size_t getPeriodCount() const noexcept;
    uint32_t getLatency() const;
    uint32_t getPlatformDelay() const;
    

   protected:
    pal_stream_handle_t* mPalHandle{nullptr};
    bool mIsPaused{false};
    bool mIsConfigured{false};
    // mix port
    ::aidl::android::media::audio::common::AudioPortConfig mPortConfig;
    const Tag mTag;
    const bool mIsInput;
    const size_t mFrameSize;
    std::vector<::aidl::android::media::audio::common::AudioDevice>
        mConnectedDevices;
    std::variant<std::monostate, PrimaryPlayback, DeepBufferPlayback, PcmRecord,
                 CompressPlayback, CompressCapture, PcmOffloadPlayback,
                 VoipPlayback, SpatialPlayback, VoipRecord>
        mExt;
};

}  // namespace qti::audio::core