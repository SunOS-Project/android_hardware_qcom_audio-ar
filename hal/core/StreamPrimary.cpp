/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cmath>

#define LOG_TAG "AHAL_StreamPrimary"

#include <android-base/logging.h>
#include <audio_utils/clock.h>

#include <qti-audio-core/Module.h>
#include <qti-audio-core/StreamPrimary.h>

using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::MicrophoneInfo;
using aidl::android::media::audio::common::AudioIoFlags;
using aidl::android::media::audio::common::AudioInputFlags;
using aidl::android::media::audio::common::AudioOutputFlags;
using ::aidl::android::hardware::audio::common::isBitPositionFlagSet;


namespace qti::audio::core {

std::unique_ptr<AudioUsecase> DriverPrimary::createAudioUsecase(
    const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig,
    const std::optional<
        ::aidl::android::media::audio::common::AudioOffloadInfo>& offloadInfo,
    std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback>
        asyncCallback,
    const bool isInput) {
    auto usecase = std::make_unique<AudioUsecase>(mixPortConfig, offloadInfo,
                                                  asyncCallback, isInput);
    return std::move(usecase);
}

DriverPrimary::DriverPrimary(
    const StreamContext& context,
    const std::optional<
        ::aidl::android::media::audio::common::AudioOffloadInfo>& offloadInfo,
    bool isInput)
    : mFrameSizeBytes(context.getFrameSize()),
      mSampleRate(context.getSampleRate()),
      mIsAsynchronous(!!context.getAsyncCallback()),
      mIsInput(isInput),
      mUsecase(createAudioUsecase(context.getMixPortConfig(), offloadInfo,
                                  context.getAsyncCallback(), isInput)) {}

::android::status_t DriverPrimary::init() {
    usleep(500);
    return ::android::OK;
}

::android::status_t DriverPrimary::drain(
    ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode mode) {
    if (mUsecase) {
        return mUsecase->drain(mode);
    }
    return ::android::OK;
}

::android::status_t DriverPrimary::flush() {
    if (mUsecase) {
        return mUsecase->flush();
    }
    return ::android::OK;
}

::android::status_t DriverPrimary::pause() {
    if(mUsecase){
        return mUsecase->pause();
    }
    return ::android::OK;
}

::android::status_t DriverPrimary::transfer(void* buffer, size_t frameCount,
                                         size_t* actualFrameCount,
                                         int32_t* latencyMs) {
    static constexpr float kMicrosPerSecond = MICROS_PER_SECOND;
    static constexpr float kScaleFactor = .8f;
    if (!mIsInput) {
        if (mIsAsynchronous) {
            usleep(500);
        } else {
            if (mUsecase) {
                if (!mUsecase->isConfigured()) {
                    const bool isConfigured = mUsecase->configure();
                }
                int32_t ret = 0;
                if (mUsecase->isPaused()) {
                    mUsecase->resume();
                }
                ret = mUsecase->write((uint8_t*)buffer, frameCount);
                *actualFrameCount = frameCount;
                return ret;
            }
        }
    }
    if (mIsInput) {
        if (mUsecase) {
            if(!mUsecase->isConfigured()){
                const bool isConfigured = mUsecase->configure();
            }
            int32_t ret =0;
            if (mUsecase->isPaused()){
                mUsecase->resume();
            }
            ret =
                mUsecase->read((uint8_t*)buffer, frameCount, actualFrameCount);
            return ret;
        }
    }
    *latencyMs = Module::kLatencyMs;
    return ::android::OK;
}

::android::status_t DriverPrimary::standby() {
    if (mUsecase) {
        return mUsecase->standBy();
    }
    return ::android::OK;
}

::android::status_t DriverPrimary::setConnectedDevices(
    const std::vector<AudioDevice>& connectedDevices __unused) {
    if(mUsecase){
        mUsecase->setConnectedDevices(connectedDevices);
    }
    return ::android::OK;
}

::android::status_t DriverPrimary::close() {
    mUsecase->close();
    return 0;
}
::android::status_t DriverPrimary::prepareToClose() { return 0; }
::android::status_t DriverPrimary::updateHwAvSyncId(int32_t in_hwAvSyncId) { return 0; }
::android::status_t DriverPrimary::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    return 0;
}
::android::status_t DriverPrimary::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    return 0;
}
::android::status_t DriverPrimary::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return 0;
}
::android::status_t DriverPrimary::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return 0;
}

// static
ndk::ScopedAStatus StreamInPrimary::createInstance(
    const SinkMetadata& sinkMetadata, StreamContext&& context,
    const std::vector<MicrophoneInfo>& microphones,
    std::shared_ptr<StreamIn>* result) {
    std::shared_ptr<StreamIn> stream = ndk::SharedRefBase::make<StreamInPrimary>(
        sinkMetadata, std::move(context), microphones);
    if (auto status = initInstance(stream); !status.isOk()) {
        return status;
    }
    *result = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

StreamInPrimary::StreamInPrimary(const SinkMetadata& sinkMetadata,
                                 StreamContext&& context,
                                 const std::vector<MicrophoneInfo>& microphones)
    : StreamIn(
          sinkMetadata, std::move(context),
          [&](const StreamContext& ctx) -> DriverInterface* {
              return new DriverPrimary(ctx, std::nullopt, true /*isInput*/);
          },
          [](const StreamContext& ctx,
             DriverInterface* driver) -> StreamWorkerInterface* {
              // The default worker implementation is used.
              return new StreamInWorker(ctx, driver);
          },
          microphones) {}

// static
ndk::ScopedAStatus StreamOutPrimary::createInstance(
    const SourceMetadata& sourceMetadata, StreamContext&& context,
    const std::optional<AudioOffloadInfo>& offloadInfo,
    std::shared_ptr<StreamOut>* result) {
    std::shared_ptr<StreamOut> stream = ndk::SharedRefBase::make<StreamOutPrimary>(
        sourceMetadata, std::move(context), offloadInfo);
    if (auto status = initInstance(stream); !status.isOk()) {
        return status;
    }
    *result = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

StreamOutPrimary::StreamOutPrimary(
    const SourceMetadata& sourceMetadata, StreamContext&& context,
    const std::optional<AudioOffloadInfo>& offloadInfo)
    : StreamOut(
          sourceMetadata, std::move(context),
          [&](const StreamContext& ctx) -> DriverInterface* {
              return new DriverPrimary(ctx, offloadInfo, false /*isInput*/);
          },
          [](const StreamContext& ctx,
             DriverInterface* driver) -> StreamWorkerInterface* {
              // The default worker implementation is used.
              return new StreamOutWorker(ctx, driver);
          },
          offloadInfo) {}

}  // namespace qti::audio::core