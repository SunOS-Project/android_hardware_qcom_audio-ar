/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cmath>

#define LOG_TAG "AHAL_Stream"

#include <android-base/logging.h>
#include <audio_utils/clock.h>

#include <qti-audio-core/Module.h>
#include <qti-audio-core/StreamStub.h>

using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::MicrophoneInfo;

namespace qti::audio::core {

DriverStub::DriverStub(const StreamContext& context, bool isInput)
    : mFrameSizeBytes(context.getFrameSize()),
      mSampleRate(context.getSampleRate()),
      mIsAsynchronous(!!context.getAsyncCallback()),
      mIsInput(isInput) {}

::android::status_t DriverStub::init() {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverStub::drain(
    ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode) {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverStub::flush() {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverStub::pause() {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverStub::transfer(void* buffer, size_t frameCount,
                                         size_t* actualFrameCount,
                                         int32_t* latencyMs) {
    static constexpr float kMicrosPerSecond = MICROS_PER_SECOND;
    static constexpr float kScaleFactor = .8f;
    if (mIsAsynchronous) {
        usleep(500);
    } else {
        const size_t delayUs = static_cast<size_t>(std::roundf(
            kScaleFactor * frameCount * kMicrosPerSecond / mSampleRate));
        usleep(delayUs);
    }
    if (mIsInput) {
        uint8_t* byteBuffer = static_cast<uint8_t*>(buffer);
        for (size_t i = 0; i < frameCount * mFrameSizeBytes; ++i) {
            byteBuffer[i] = std::rand() % 255;
        }
    }
    *actualFrameCount = frameCount;
    *latencyMs = Module::kLatencyMs;
    return ::android::OK;
}

::android::status_t DriverStub::standby() {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverStub::setConnectedDevices(
    const std::vector<AudioDevice>& connectedDevices __unused) {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverStub::close() { return 0; }
::android::status_t DriverStub::prepareToClose() { return 0; }
::android::status_t DriverStub::updateHwAvSyncId(int32_t in_hwAvSyncId) { return 0; }
::android::status_t DriverStub::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    return 0;
}
::android::status_t DriverStub::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    return 0;
}
::android::status_t DriverStub::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return 0;
}
::android::status_t DriverStub::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return 0;
}

// static
ndk::ScopedAStatus StreamInStub::createInstance(
    const SinkMetadata& sinkMetadata, StreamContext&& context,
    const std::vector<MicrophoneInfo>& microphones,
    std::shared_ptr<StreamIn>* result) {
    std::shared_ptr<StreamIn> stream = ndk::SharedRefBase::make<StreamInStub>(
        sinkMetadata, std::move(context), microphones);
    if (auto status = initInstance(stream); !status.isOk()) {
        return status;
    }
    *result = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

StreamInStub::StreamInStub(const SinkMetadata& sinkMetadata,
                           StreamContext&& context,
                           const std::vector<MicrophoneInfo>& microphones)
    : StreamIn(
          sinkMetadata, std::move(context),
          [](const StreamContext& ctx) -> DriverInterface* {
              return new DriverStub(ctx, true /*isInput*/);
          },
          [](const StreamContext& ctx,
             DriverInterface* driver) -> StreamWorkerInterface* {
              // The default worker implementation is used.
              return new StreamInWorker(ctx, driver);
          },
          microphones) {}

// static
ndk::ScopedAStatus StreamOutStub::createInstance(
    const SourceMetadata& sourceMetadata, StreamContext&& context,
    const std::optional<AudioOffloadInfo>& offloadInfo,
    std::shared_ptr<StreamOut>* result) {
    std::shared_ptr<StreamOut> stream = ndk::SharedRefBase::make<StreamOutStub>(
        sourceMetadata, std::move(context), offloadInfo);
    if (auto status = initInstance(stream); !status.isOk()) {
        return status;
    }
    *result = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

StreamOutStub::StreamOutStub(const SourceMetadata& sourceMetadata,
                             StreamContext&& context,
                             const std::optional<AudioOffloadInfo>& offloadInfo)
    : StreamOut(
          sourceMetadata, std::move(context),
          [](const StreamContext& ctx) -> DriverInterface* {
              return new DriverStub(ctx, false /*isInput*/);
          },
          [](const StreamContext& ctx,
             DriverInterface* driver) -> StreamWorkerInterface* {
              // The default worker implementation is used.
              return new StreamOutWorker(ctx, driver);
          },
          offloadInfo) {}

}  // namespace qti::audio::core