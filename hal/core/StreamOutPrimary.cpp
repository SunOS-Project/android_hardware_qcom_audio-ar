/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cmath>

#define LOG_TAG "AHAL_QStreamOut"

#include <android-base/logging.h>
#include <audio_utils/clock.h>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/StreamOutPrimary.h>

using aidl::android::hardware::audio::common::AudioOffloadMetadata;
using aidl::android::hardware::audio::common::getChannelCount;
using aidl::android::hardware::audio::common::getFrameSizeInBytes;
using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioDualMonoMode;
using aidl::android::media::audio::common::AudioLatencyMode;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::AudioPlaybackRate;
using aidl::android::media::audio::common::MicrophoneDynamicInfo;
using aidl::android::media::audio::common::MicrophoneInfo;

using ::aidl::android::hardware::audio::common::getChannelCount;
using ::aidl::android::hardware::audio::common::getFrameSizeInBytes;
using ::aidl::android::hardware::audio::core::IStreamCallback;
using ::aidl::android::hardware::audio::core::IStreamCommon;
using ::aidl::android::hardware::audio::core::StreamDescriptor;
using ::aidl::android::hardware::audio::core::VendorParameter;

namespace qti::audio::core {

StreamOutPrimary::StreamOutPrimary(
    StreamContext&& context, const SourceMetadata& sourceMetadata,
    const std::optional<AudioOffloadInfo>& offloadInfo)
    : StreamOut(std::move(context), offloadInfo),
      StreamCommonImpl(&(StreamOut::mContext), sourceMetadata),
      mTag(getUsecaseTag(getContext().getMixPortConfig())),
      mFrameSizeBytes(getContext().getFrameSize()),
      mSampleRate(getContext().getSampleRate()),
      mIsAsynchronous(!!getContext().getAsyncCallback()),
      mIsInput(false) {
    if (mTag == Usecase::PRIMARY_PLAYBACK) {
        mExt.emplace<PrimaryPlayback>();
    } else if (mTag == Usecase::DEEP_BUFFER_PLAYBACK) {
        mExt.emplace<DeepBufferPlayback>();
    } else if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        mExt.emplace<CompressPlayback>(offloadInfo.value(),
                                       getContext().getAsyncCallback());
    } else if (mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        mExt.emplace<PcmOffloadPlayback>();
    } else if (mTag == Usecase::VOIP_PLAYBACK) {
        mExt.emplace<VoipPlayback>();
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        mExt.emplace<SpatialPlayback>();
    }
}

StreamOutPrimary::~StreamOutPrimary() {
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
}

std::string StreamOutPrimary::toString() const noexcept {
    std::ostringstream os;
    return os.str();
}

// start of methods called from IModule
ndk::ScopedAStatus StreamOutPrimary::setConnectedDevices(
    const std::vector<::aidl::android::media::audio::common::AudioDevice>&
        devices) {
    mWorker->setIsConnected(!devices.empty());
    mConnectedDevices = devices;
    auto connectedPalDevices = mPlatform.getPalDevices(mConnectedDevices);

    if (this->mPalHandle != nullptr && connectedPalDevices.size() > 0) {
        if (int32_t ret = ::pal_stream_set_device(this->mPalHandle,
                                                  connectedPalDevices.size(),
                                                  connectedPalDevices.data());
            ret) {
            LOG(ERROR) << __func__
                       << " failed to set devices on stream, ret:" << ret;
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
    }

    auto devicesString = [](std::string prev, const auto& device) {
        return std::move(prev) + ';' + device.toString();
    };

    LOG(VERBOSE) << __func__ << " stream is connected to devices:"
                 << std::accumulate(mConnectedDevices.cbegin(),
                                    mConnectedDevices.cend(), std::string(""),
                                    devicesString);

    return ndk::ScopedAStatus::ok();
}

// end of methods called from IModule

// start of DriverInterface Methods

::android::status_t StreamOutPrimary::init() {
    mIsInitialized = true;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::drain(
    ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode mode) {
    auto palDrainMode = mode == ::aidl::android::hardware::audio::core::
                                    StreamDescriptor::DrainMode::DRAIN_ALL
                            ? PAL_DRAIN
                            : PAL_DRAIN_PARTIAL;
    if (int32_t ret = ::pal_stream_drain(mPalHandle, palDrainMode); ret) {
        LOG(ERROR) << __func__ << " failed to drain the stream, ret:" << ret;
        return ret;
    }
    LOG(VERBOSE) << __func__ << " drain successful";
    return ::android::OK;
}

::android::status_t StreamOutPrimary::flush() {
    if (int32_t ret = ::pal_stream_flush(mPalHandle); ret) {
        LOG(ERROR) << __func__ << " failed to flush the stream, ret:" << ret;
        return ret;
    }
    LOG(VERBOSE) << __func__ << " flush successful";
    return ::android::OK;
}

::android::status_t StreamOutPrimary::pause() {
    if (int32_t ret = pal_stream_pause(mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " failed to pause the stream, ret:"
                   << std::to_string(ret);
        return ret;
    }
    mIsPaused = true;
    LOG(VERBOSE) << __func__ << " pause successful";
    return ::android::OK;
}

void StreamOutPrimary::resume() {
    if (int32_t ret = ::pal_stream_resume(mPalHandle); ret) {
        LOG(ERROR) << __func__ << " failed to resume the stream, ret:" << ret;
    }
    LOG(VERBOSE) << __func__ << " resume successful";
    mIsPaused = false;
}

::android::status_t StreamOutPrimary::standby() {
    mIsStandby = true;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::start() {
    mIsStandby = false;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::transfer(void* buffer, size_t frameCount,
                                               size_t* actualFrameCount,
                                               int32_t* latencyMs) {
    if (!mIsConfigured) {
        // configure on first transfer
        configure();
    }
    if(mIsPaused){
        resume();
    }
    pal_buffer palBuffer{};
    palBuffer.buffer = static_cast<uint8_t*>(buffer);
    palBuffer.size = frameCount * mFrameSizeBytes;
    ssize_t bytesWritten = ::pal_stream_write(mPalHandle, &palBuffer);
    if (bytesWritten < 0) {
        LOG(ERROR) << __func__ << " write failed, ret:" << bytesWritten;
        return ::android::FAILED_TRANSACTION;
    }

    *actualFrameCount = static_cast<size_t>(bytesWritten / mFrameSizeBytes);
    // Todo findout write latency
    *latencyMs = Module::kLatencyMs;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::refinePosition(
    ::aidl::android::hardware::audio::core::StreamDescriptor::Position*
        position) {

    if(!mIsConfigured){
        return ::android::OK;
    }

    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        int64_t frames = 0;
        std::get<CompressPlayback>(mExt).getPositionInFrames(&frames);
        LOG(VERBOSE) << __func__ << " dspFrames consumed:" << frames;
        const auto latencyMs =
            mPlatform.getBluetoothLatencyMs(mConnectedDevices);
        const auto offset = latencyMs * mSampleRate / 1000;
        frames = (frames > offset) ? (frames - offset) : 0;
        position->frames = frames;
    }

    return ::android::OK;
}

void StreamOutPrimary::shutdown() {
    mIsInitialized = false;
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
    mIsConfigured = false;
    mPalHandle = nullptr;
}

// end of DriverInterface Methods

// start of IStreamOut Methods
ndk::ScopedAStatus StreamOutPrimary::updateOffloadMetadata(
    const AudioOffloadMetadata& in_offloadMetadata) {
    LOG(DEBUG) << __func__;
    if (isClosed()) {
        LOG(ERROR) << __func__ << ": stream was closed";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    if (!mOffloadInfo.has_value()) {
        LOG(ERROR) << __func__ << ": not a compressed offload stream";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    if (in_offloadMetadata.sampleRate < 0) {
        LOG(ERROR) << __func__ << ": invalid sample rate value: "
                   << in_offloadMetadata.sampleRate;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_offloadMetadata.averageBitRatePerSecond < 0) {
        LOG(ERROR) << __func__ << ": invalid average BPS value: "
                   << in_offloadMetadata.averageBitRatePerSecond;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_offloadMetadata.delayFrames < 0) {
        LOG(ERROR) << __func__ << ": invalid delay frames value: "
                   << in_offloadMetadata.delayFrames;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_offloadMetadata.paddingFrames < 0) {
        LOG(ERROR) << __func__ << ": invalid padding frames value: "
                   << in_offloadMetadata.paddingFrames;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (mTag != Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        LOG(WARNING) << __func__
                     << ": expected COMPRESS_OFFLOAD_PLAYBACK instead of "
                     << getName(mTag);
        return ndk::ScopedAStatus::ok();
    }

    mOffloadMetadata = in_offloadMetadata;
    auto& compressPlayback = std::get<CompressPlayback>(mExt);
    compressPlayback.updateOffloadMetadata(mOffloadMetadata.value());

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamOutPrimary::getHwVolume(std::vector<float>* _aidl_return) {
    LOG(DEBUG) << __func__;
    *_aidl_return = mVolumes;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamOutPrimary::setHwVolume(
    const std::vector<float>& in_channelVolumes) {

    if (!mIsConfigured) {
        mVolumes = in_channelVolumes;
        return ndk::ScopedAStatus::ok();
    }

    auto palVolumes = mPlatform.getPalVolumeData(in_channelVolumes);
    if (palVolumes.size() == 0) {
        mVolumes = {};
        LOG(ERROR) << __func__ << ": unable to fetch volumes";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    if (int32_t ret = ::pal_stream_set_volume(
            mPalHandle, reinterpret_cast<pal_volume_data*>(palVolumes.data()));
        ret) {
        mVolumes = {};
        LOG(ERROR) << __func__ << ": pal_stream_set_volume failed!!!";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    mVolumes = in_channelVolumes;

    LOG(VERBOSE) << __func__ << ": stream volume updated";
    return ndk::ScopedAStatus::ok();
}

// end of IStreamOut Methods

// start of StreamCommonInterface Methods

ndk::ScopedAStatus StreamOutPrimary::updateMetadataCommon(
    const Metadata& metadata) {

    if (isClosed()) {
        LOG(ERROR) << __func__ << ": stream was closed";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    if (metadata.index() != mMetadata.index()) {
        LOG(FATAL) << __func__ << ": changing metadata variant is not allowed";
    }
    mMetadata = metadata;

    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        compressPlayback.updateSourceMetadata(
            std::get<SourceMetadata>(mMetadata));
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamOutPrimary::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<VendorParameter>* _aidl_return) {
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        return compressPlayback.getVendorParameters(in_ids, _aidl_return);
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus StreamOutPrimary::setVendorParameters(
    const std::vector<VendorParameter>& in_parameters, bool in_async) {
    LOG(VERBOSE) << __func__;
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        return compressPlayback.setVendorParameters(in_parameters, in_async);
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus StreamOutPrimary::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    if (in_effect == nullptr) {
        LOG(DEBUG) << __func__ << ": null effect";
    } else {
        LOG(DEBUG) << __func__ << ": effect Binder"
                   << in_effect->asBinder().get();
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus StreamOutPrimary::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    if (in_effect == nullptr) {
        LOG(DEBUG) << __func__ << ": null effect";
    } else {
        LOG(DEBUG) << __func__ << ": effect Binder"
                   << in_effect->asBinder().get();
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

// end of StreamCommonInterface Methods

size_t StreamOutPrimary::getPeriodSize() const noexcept {
    if (mTag == Usecase::PRIMARY_PLAYBACK) {
        return PrimaryPlayback::kPeriodSize * mFrameSizeBytes;
    } else if (mTag == Usecase::DEEP_BUFFER_PLAYBACK) {
        return DeepBufferPlayback::kPeriodSize * mFrameSizeBytes;
    } else if (mTag == Usecase::LOW_LATENCY_PLAYBACK) {
        return LowLatencyPlayback::kPeriodSize * mFrameSizeBytes;
    } else if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        return CompressPlayback::getPeriodBufferSize(
            mMixPortConfig.format.value());
    } else if (mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        return PcmOffloadPlayback::getPeriodSize(
            mMixPortConfig.format.value(), mMixPortConfig.channelMask.value(),
            mMixPortConfig.sampleRate.value().value);
    } else if (mTag == Usecase::VOIP_PLAYBACK) {
        return VoipPlayback::getPeriodSize(
            mMixPortConfig.format.value(), mMixPortConfig.channelMask.value(),
            mMixPortConfig.sampleRate.value().value);
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        return PrimaryPlayback::kPeriodSize * mFrameSizeBytes;
    }
    return 0;
}

size_t StreamOutPrimary::getPeriodCount() const noexcept {
    if (mTag == Usecase::PRIMARY_PLAYBACK) {
        return PrimaryPlayback::kPeriodCount;
    } else if (mTag == Usecase::DEEP_BUFFER_PLAYBACK) {
        return DeepBufferPlayback::kPeriodCount;
    } else if (mTag == Usecase::LOW_LATENCY_PLAYBACK) {
        return LowLatencyPlayback::kPeriodCount;
    } else if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        return CompressPlayback::kPeriodCount;
    } else if (mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        return PcmOffloadPlayback::kPeriodCount;
    } else if (mTag == Usecase::VOIP_PLAYBACK) {
        return VoipPlayback::kPeriodCount;
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        return SpatialPlayback::kPeriodCount;
    }
    return 0;
}

size_t StreamOutPrimary::getPlatformDelay() const noexcept {
    return 0;
}

void StreamOutPrimary::configure() {
    auto attr = mPlatform.getPalStreamAttributes(mMixPortConfig, false);
    if (!attr) {
        LOG(ERROR) << __func__ << " no pal attributes";
        return;
    }

    if (mTag == Usecase::DEEP_BUFFER_PLAYBACK || mTag == Usecase::PRIMARY_PLAYBACK) {
        attr->type = PAL_STREAM_DEEP_BUFFER;
    } else if (mTag == Usecase::LOW_LATENCY_PLAYBACK) {
        attr->type = PAL_STREAM_LOW_LATENCY;
    } else if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        attr->type = PAL_STREAM_COMPRESSED;
    } else if (mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        attr->type = PAL_STREAM_PCM_OFFLOAD;
    } else if (mTag == Usecase::VOIP_PLAYBACK) {
        attr->type = PAL_STREAM_VOIP_TX;
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        attr->type = PAL_STREAM_SPATIAL_AUDIO;
    } else {
        LOG(VERBOSE) << __func__
                     << " invalid usecase to configure";
        return;
    }

    LOG(VERBOSE) << __func__ << " assigned pal stream type:" << attr->type
                 << " for " << getName(mTag);

    auto palDevices = mPlatform.getPalDevices(getConnectedDevices());
    if (!palDevices.size()) {
        LOG(ERROR) << __func__
                     << " no connected devices on stream!!";
        return;
    }

    uint64_t cookie = reinterpret_cast<uint64_t>(this);
    pal_stream_callback palFn = nullptr;
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        cookie = reinterpret_cast<uint64_t>(&compressPlayback);
        palFn = CompressPlayback::palCallback;
        /* TODO check any dynamic update as per offload metadata or source
         * metadata */
        attr->out_media_config.bit_width = mOffloadInfo.value().bitWidth;
        attr->flags =
            static_cast<pal_stream_flags_t>(PAL_STREAM_FLAG_NON_BLOCKING);
    }

    if (int32_t ret =
            ::pal_stream_open(attr.get(), palDevices.size(), palDevices.data(),
                              0, nullptr, palFn, cookie, &(this->mPalHandle));
        ret) {
        LOG(ERROR) << __func__
                   << " pal stream open failed!!! ret:" << std::to_string(ret);
        return;
    }

    const size_t ringBufSizeInBytes = getPeriodSize();
    const size_t ringBufCount = getPeriodCount();
    auto palBufferConfig =
        mPlatform.getPalBufferConfig(ringBufSizeInBytes, ringBufCount);
    LOG(VERBOSE) << __func__ << " pal stream set buffer size "
                 << std::to_string(ringBufSizeInBytes) << " with count "
                 << std::to_string(ringBufCount);
    if (int32_t ret = ::pal_stream_set_buffer_size(
            this->mPalHandle, (mIsInput ? palBufferConfig.get() : nullptr),
            ((!mIsInput) ? palBufferConfig.get() : nullptr));
        ret) {
        LOG(ERROR) << __func__ << " pal stream set buffer size failed!!! ret:"
                   << std::to_string(ret);
        return;
    }
    LOG(VERBOSE) << __func__ << " pal stream set buffer size successful";
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        auto palParamPayload = compressPlayback.getPayloadCodecInfo();
        if (int32_t ret = ::pal_stream_set_param(
                this->mPalHandle, PAL_PARAM_ID_CODEC_CONFIGURATION,
                reinterpret_cast<pal_param_payload*>(palParamPayload.get()));
            ret) {
            LOG(VERBOSE) << __func__
                         << " pal stream set param failed!!! ret:" << ret;
            return;
        }
        LOG(VERBOSE) << __func__
                     << " pal stream set param: "
                        "PAL_PARAM_ID_CODEC_CONFIGURATION successful";
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " pal stream start failed!! ret:" << std::to_string(ret);
        return;
    }
    LOG(VERBOSE) << __func__ << " pal stream start successful";

    // configure mExt
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        std::get<CompressPlayback>(mExt).setPalHandle(mPalHandle);
    }

    LOG(INFO) << __func__ << ": stream is configured for " << getName(mTag);
    mIsConfigured = true;

    // after configuration operations

    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        // if any cached volume update
        mVolumes.size() > 0 ? (void)setHwVolume(mVolumes) : (void)0;
    }
}

}  // namespace qti::audio::core