/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <qti-audio-core/Stream.h>

namespace qti::audio::core {

class StreamStub : public StreamCommonImpl {
  public:
      StreamStub(StreamContext* context, const Metadata& metadata);
    // Methods of 'DriverInterface'.
    ::android::status_t init() override;
    ::android::status_t drain(::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode) override;
    ::android::status_t flush() override;
    ::android::status_t pause() override;
    ::android::status_t standby() override;
    ::android::status_t start() override;
    ::android::status_t transfer(void* buffer, size_t frameCount, size_t* actualFrameCount,
                                 int32_t* latencyMs) override;
    void shutdown() override;

  private:
    const size_t mFrameSizeBytes;
    const int mSampleRate;
    const bool mIsAsynchronous;
    const bool mIsInput;
    bool mIsInitialized = false;  // Used for validating the state machine logic.
    bool mIsStandby = true;       // Used for validating the state machine logic.
};

class StreamInStub final : public StreamIn, public StreamStub {
  public:
    friend class ndk::SharedRefBase;
    StreamInStub(
            StreamContext&& context,
            const ::aidl::android::hardware::audio::common::SinkMetadata& sinkMetadata,
            const std::vector<::aidl::android::media::audio::common::MicrophoneInfo>& microphones);

  private:
    void onClose() override { defaultOnClose(); }
};

class StreamOutStub final : public StreamOut, public StreamStub {
  public:
    friend class ndk::SharedRefBase;
    StreamOutStub(StreamContext&& context,
                  const ::aidl::android::hardware::audio::common::SourceMetadata& sourceMetadata,
                  const std::optional<::aidl::android::media::audio::common::AudioOffloadInfo>&
                          offloadInfo);

  private:
    void onClose() override { defaultOnClose(); }
};


}  // namespace qti::audio::core