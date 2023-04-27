/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_PlatformStream"

#include <android-base/logging.h>
#include <platform/PlatformStream.h>

namespace qti::audio::core {

/* ============= start of StreamInPrimary =========== */
StreamInPrimary::StreamInPrimary(const SinkMetadata& sinkMetadata,
                                 StreamContext&& context,
                                 const std::vector<MicrophoneInfo>& microphones)
    : StreamIn(
          sinkMetadata, std::move(context),
          [&, this](const StreamContext& ctx) -> DriverInterface* {
              return new PlatformDriverIn(ctx, *this);
          },
          [](const StreamContext& ctx,
             DriverInterface* driver) -> StreamWorkerInterface* {
              // The default worker implementation is used.
              return new StreamInWorker(ctx, driver);
          },
          microphones) {}

ndk::ScopedAStatus StreamInPrimary::createInstance(
    const SinkMetadata& sinkMetadata, StreamContext&& context,
    const std::vector<MicrophoneInfo>& microphones,
    std::shared_ptr<StreamIn>* result) {
    std::shared_ptr<StreamIn> stream =
        ndk::SharedRefBase::make<StreamInPrimary>(
            sinkMetadata, std::move(context), microphones);
    if (auto status = initInstance(stream); !status.isOk()) {
        return status;
    }
    *result = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamInPrimary::close() { return ndk::ScopedAStatus::ok(); }
ndk::ScopedAStatus StreamInPrimary::updateHwAvSyncId(int32_t in_hwAvSyncId) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamInPrimary::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamInPrimary::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamInPrimary::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamInPrimary::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return ndk::ScopedAStatus::ok();
}

PlatformDriverIn::PlatformDriverIn(const StreamContext& context,
                                   StreamInPrimary& platformStream)
    : mFrameSizeBytes(context.getFrameSize()),
      mPlatformStream(platformStream) {}

::android::status_t PlatformDriverIn::init() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverIn::drain(
    ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode) {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverIn::flush() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverIn::pause() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverIn::transfer(void* buffer, size_t frameCount,
                                               size_t* actualFrameCount,
                                               int32_t* latencyMs) {
    usleep(3000);
    uint8_t* byteBuffer = static_cast<uint8_t*>(buffer);
    for (size_t i = 0; i < frameCount * mFrameSizeBytes; ++i) {
        byteBuffer[i] = std::rand() % 255;
    }
    *actualFrameCount = frameCount;
    // Todo below
    // *latencyMs = Module::kLatencyMs;
    *latencyMs = 10;
    return ::android::OK;
}

::android::status_t PlatformDriverIn::standby() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverIn::setConnectedDevices(
    const std::vector<AudioDevice>& connectedDevices __unused) {
    return ::android::OK;
}

/* ============= start of StreamOutPrimary =========== */
StreamOutPrimary::StreamOutPrimary(
    const SourceMetadata& sourceMetadata, StreamContext&& context,
    const std::optional<AudioOffloadInfo>& offloadInfo)
    : StreamOut(
          sourceMetadata, std::move(context),
          [&, this](const StreamContext& ctx) -> DriverInterface* {
              return new PlatformDriverOut(ctx, *this);
          },
          [](const StreamContext& ctx,
             DriverInterface* driver) -> StreamWorkerInterface* {
              // The default worker implementation is used.
              return new StreamOutWorker(ctx, driver);
          },
          offloadInfo) {}

// static
ndk::ScopedAStatus StreamOutPrimary::createInstance(
    const SourceMetadata& sourceMetadata, StreamContext&& context,
    const std::optional<AudioOffloadInfo>& offloadInfo,
    std::shared_ptr<StreamOut>* result) {
    std::shared_ptr<StreamOut> stream =
        ndk::SharedRefBase::make<StreamOutPrimary>(
            sourceMetadata, std::move(context), offloadInfo);
    if (auto status = initInstance(stream); !status.isOk()) {
        return status;
    }
    *result = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamOutPrimary::close() {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamOutPrimary::updateHwAvSyncId(int32_t in_hwAvSyncId) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamOutPrimary::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamOutPrimary::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamOutPrimary::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return ndk::ScopedAStatus::ok();
}
ndk::ScopedAStatus StreamOutPrimary::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return ndk::ScopedAStatus::ok();
}

PlatformDriverOut::PlatformDriverOut(const StreamContext& context,
                                     StreamOutPrimary& platformStream)
    : mFrameSizeBytes(context.getFrameSize()),
      mPlatformStream(platformStream) {}

::android::status_t PlatformDriverOut::init() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverOut::drain(
    ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode) {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverOut::flush() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverOut::pause() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverOut::transfer(void* buffer, size_t frameCount,
                                                size_t* actualFrameCount,
                                                int32_t* latencyMs) {
    usleep(3000);
    *actualFrameCount = frameCount;
    // Todo below
    // *latencyMs = Module::kLatencyMs;
    *latencyMs = 10;
    return ::android::OK;
}

::android::status_t PlatformDriverOut::standby() {
    usleep(1000);
    return ::android::OK;
}

::android::status_t PlatformDriverOut::setConnectedDevices(
    const std::vector<AudioDevice>& connectedDevices __unused) {
    return ::android::OK;
}

}  // namespace qti::audio::core
