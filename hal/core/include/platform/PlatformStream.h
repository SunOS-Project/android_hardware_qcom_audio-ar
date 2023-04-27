/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <PalDefs.h>
#include <aidlservice/Stream.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::MicrophoneInfo;

namespace qti::audio::core {

enum class InputUseCase : int8_t {
    PCM_RECORD = 0,
    COMPRESS_RECORD,
};
enum class OutputUseCase : int8_t {
    DEEPBUFFER_PLAYBACK = 0,
    COMPRESS_PLAYBACK,
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

    // StreamCommon APIs
    virtual ndk::ScopedAStatus close() override;
    virtual ndk::ScopedAStatus updateHwAvSyncId(int32_t in_hwAvSyncId) override;
    virtual ndk::ScopedAStatus getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
            _aidl_return) override;
    virtual ndk::ScopedAStatus setVendorParameters(
        const std::vector<
            ::aidl::android::hardware::audio::core::VendorParameter>&
            in_parameters,
        bool in_async) override;
    virtual ndk::ScopedAStatus addEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;
    virtual ndk::ScopedAStatus removeEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;

   private:
    friend class ndk::SharedRefBase;
    StreamInPrimary(
        const ::aidl::android::hardware::audio::common::SinkMetadata&
            sinkMetadata,
        StreamContext&& context,
        const std::vector<
            ::aidl::android::media::audio::common::MicrophoneInfo>&
            microphones);
    pal_stream_handle_t mPalStreamHandle{0};
    InputUseCase mUsecase;
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
    // StreamCommon APIs
    virtual ndk::ScopedAStatus close() override;
    virtual ndk::ScopedAStatus updateHwAvSyncId(int32_t in_hwAvSyncId) override;
    virtual ndk::ScopedAStatus getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
            _aidl_return) override;
    virtual ndk::ScopedAStatus setVendorParameters(
        const std::vector<
            ::aidl::android::hardware::audio::core::VendorParameter>&
            in_parameters,
        bool in_async) override;
    virtual ndk::ScopedAStatus addEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;
    virtual ndk::ScopedAStatus removeEffect(
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;

   private:
    friend class ndk::SharedRefBase;
    StreamOutPrimary(
        const ::aidl::android::hardware::audio::common::SourceMetadata&
            sourceMetadata,
        StreamContext&& context,
        const std::optional<
            ::aidl::android::media::audio::common::AudioOffloadInfo>&
            offloadInfo);
    pal_stream_handle_t mPalStreamHandle{0};
    OutputUseCase mUsecase;
};

class PlatformDriverIn : public DriverInterface {
   public:
    PlatformDriverIn(const StreamContext& context,
                     StreamInPrimary& platformStream);
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

   private:
    const size_t mFrameSizeBytes;
    StreamInPrimary& mPlatformStream;
};

class PlatformDriverOut : public DriverInterface {
   public:
    PlatformDriverOut(const StreamContext& context,
                      StreamOutPrimary& platformStream);
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

   private:
    const size_t mFrameSizeBytes;
    StreamOutPrimary& mPlatformStream;
};

//    virtual ~PlatformStream();

//    virtual int32_t route(const std::vector<AudioDeviceDescription>&
//    devices);

//    virtual int32_t start();
//    virtual int32_t stop();
//    virtual int32_t pause();
//    virtual int32_t resume();
//    virtual int32_t flush();
//    virtual int32_t drain(int32_t drainType /*TBD*/);
//    virtual int32_t standby();
//    virtual ssize_t read(void* buffer, size_t bytes);
//    virtual int32_t write(const void* buffer, size_t bytes);

//    virtual int32_t createMmapBuffer(int32_t minSizeFrames,
//                                     MmapDescriptor& mmapDesc);
//    virtual int32_t getFrames(uint64_t& frames);
//    virtual int32_t getBufferSize(long& bufferSize);
//    virtual int32_t getPosition(long& frames, long& timeNs);
//    virtual int32_t getLatency(uint32_t& latency);

//    virtual int32_t updateMetadata(bool isVoiceActive);
//    virtual int32_t setGain(float gain);
//    virtual int32_t setHwVolume(const std::vector<float>& channelVolumes);

//    using StreamCb =
//        std::function<int32_t(uint32_t /*event_id*/, uint32_t*
//        /*event_data*/,
//                              uint32_t /*event_size*/, uint64_t
//                              /*cookie*/)>;
//    virtual setCallback(StreamCb callback);

//    virtual int32_t setParameters(const std::string& keyValuePairs);
//    virtual int32_t getParameters(const std::string& keys, std::string&
//    values);

//    // virtual int32_t
//    getActiveMicrophones(std::vector<MicrophoneDynamicInfo>
//    // micDynInfo); virtual int32_t setMicrophoneDirection(const
//    // MicrophoneDirection& direction); virtual int32_t
//    // getMicrophoneDirection(MicrophoneDirection& direction);
//    virtual int32_t setMicrophoneFieldDimension(float zoom);
//    virtual int32_t getMicrophoneFieldDimension(float& zoom);

//    virtual int32_t dump(int32_t fd);
}  // namespace qti::audio::core
