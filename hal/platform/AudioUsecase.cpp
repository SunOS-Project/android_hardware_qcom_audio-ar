/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_AudioUsecase"

#include <aidl/android/media/audio/common/AudioIoFlags.h>
#include <aidl/android/media/audio/common/AudioInputFlags.h>
#include <aidl/android/media/audio/common/AudioOutputFlags.h>

#include <qti-audio-core/Platform.h>
#include <android-base/logging.h>

using ::aidl::android::media::audio::common::AudioIoFlags;
using ::aidl::android::media::audio::common::AudioInputFlags;
using ::aidl::android::media::audio::common::AudioOutputFlags;
using ::aidl::android::media::audio::common::AudioSource;
using ::aidl::android::media::audio::common::AudioStreamType;
using ::aidl::android::hardware::audio::common::isBitPositionFlagSet;
using ::aidl::android::hardware::audio::common::getChannelCount;
using ::aidl::android::hardware::audio::common::getFrameSizeInBytes;
using ::aidl::android::media::audio::common::AudioPortExt;
using ::aidl::android::media::audio::common::AudioPortMixExtUseCase;

namespace qti::audio::core {

// static
AudioUsecase::Tag AudioUsecase::getUsecaseTag(
    const ::aidl::android::media::audio::common::AudioPortConfig&
        mixPortConfig) {
    AudioUsecase::Tag tag = AudioUsecase::Tag::INVALID;
    if (!mixPortConfig.flags ||
        mixPortConfig.ext.getTag() != AudioPortExt::Tag::mix) {
        LOG(ERROR) << __func__
                   << " cannot determine usecase, no flags set for mix port "
                      "config or it isn't mix port, "
                   << mixPortConfig.toString();
        return tag;
    }
    const auto& mixUsecase =
        mixPortConfig.ext.get<AudioPortExt::Tag::mix>().usecase;
    const auto mixUsecaseTag = mixUsecase.getTag();

    const auto& flagsTag = mixPortConfig.flags.value().getTag();
    constexpr auto flagCastToint = [](auto flag) {
        return static_cast<int32_t>(flag);
    };

    constexpr auto PrimaryPlaybackFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioOutputFlags::PRIMARY));
    constexpr auto deepBufferPlaybackFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioOutputFlags::DEEP_BUFFER));
    constexpr auto compressOffloadPlaybackFlags = static_cast<int32_t>(
        1 << flagCastToint(AudioOutputFlags::DIRECT) |
        1 << flagCastToint(AudioOutputFlags::COMPRESS_OFFLOAD) |
        1 << flagCastToint(AudioOutputFlags::NON_BLOCKING) |
        1 << flagCastToint(AudioOutputFlags::GAPLESS_OFFLOAD));
    constexpr auto compressCaptureFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioInputFlags::DIRECT));
    constexpr auto lowLatencyPlaybackFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioOutputFlags::PRIMARY) |
                             1 << flagCastToint(AudioOutputFlags::FAST));
    constexpr auto pcmOffloadPlaybackFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioOutputFlags::DIRECT));
    constexpr auto voipPlaybackFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioOutputFlags::VOIP_RX));
    constexpr auto spatialPlaybackFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioOutputFlags::SPATIALIZER));
    constexpr auto recordVoipFlags =
        static_cast<int32_t>(1 << flagCastToint(AudioInputFlags::VOIP_TX));

    if (flagsTag == AudioIoFlags::Tag::output) {
        auto& outFlags =
            mixPortConfig.flags.value().get<AudioIoFlags::Tag::output>();
        if(outFlags == PrimaryPlaybackFlags) {
            tag = AudioUsecase::Tag::PRIMARY_PLAYBACK;
        } else if (outFlags == deepBufferPlaybackFlags ) {
            tag = AudioUsecase::Tag::DEEP_BUFFER_PLAYBACK;
        } else if (outFlags == lowLatencyPlaybackFlags) {
            tag = AudioUsecase::Tag::LOW_LATENCY_PLAYBACK;
        } else if (outFlags == compressOffloadPlaybackFlags) {
            tag = AudioUsecase::Tag::COMPRESS_OFFLOAD_PLAYBACK;
        } else if (outFlags == pcmOffloadPlaybackFlags) {
            tag = AudioUsecase::Tag::PCM_OFFLOAD_PLAYBACK;
        } else if (outFlags == voipPlaybackFlags) {
            tag = AudioUsecase::Tag::VOIP_PLAYBACK;
        } else if (outFlags == spatialPlaybackFlags) {
            tag = AudioUsecase::Tag::SPATIAL_PLAYBACK;
        }
    } else if (flagsTag == AudioIoFlags::Tag::input) {
        auto& inFlags =
            mixPortConfig.flags.value().get<AudioIoFlags::Tag::input>();
        if (inFlags == 0) {
            tag = AudioUsecase::Tag::PCM_RECORD;
        } else if (inFlags == compressCaptureFlags) {
            tag = AudioUsecase::Tag::COMPRESS_CAPTURE;
        } else if (inFlags == recordVoipFlags &&
                   mixUsecaseTag == AudioPortMixExtUseCase::source &&
                   mixUsecase.get<AudioPortMixExtUseCase::source>() ==
                       AudioSource::VOICE_COMMUNICATION) {
            tag = AudioUsecase::Tag::VOIP_RECORD;
        }
    }
    LOG(VERBOSE) << __func__ << " choosen tag:" << getName(tag)
              << " for mix port config " << mixPortConfig.toString();
    return tag;
}

// static
std::string AudioUsecase::getName(const AudioUsecase::Tag tag) {
    switch (tag) {
        case Tag::INVALID:
            return "INVALID";
        case Tag::PRIMARY_PLAYBACK:
            return "PRIMARY_PLAYBACK";
        case Tag::DEEP_BUFFER_PLAYBACK:
            return "DEEP_BUFFER_PLAYBACK";
        case Tag::LOW_LATENCY_PLAYBACK:
            return "LOW_LATENCY_PLAYBACK";
        case Tag::PCM_RECORD:
            return "PCM_RECORD";
        case Tag::COMPRESS_OFFLOAD_PLAYBACK:
            return "COMPRESS_OFFLOAD_PLAYBACK";
        case Tag::COMPRESS_CAPTURE:
            return "COMPRESS_CAPTURE";
        case Tag::PCM_OFFLOAD_PLAYBACK:
            return "PCM_OFFLOAD_PLAYBACK";
        case Tag::VOIP_PLAYBACK:
            return "VOIP_PLAYBACK";
        case Tag::SPATIAL_PLAYBACK:
            return "SPATIAL_PLAYBACK";
        case Tag::VOIP_RECORD:
            return "VOIP_RECORD";
        default:
            return std::to_string(static_cast<uint16_t>(tag));
    }
}

AudioUsecase::AudioUsecase(
    const ::aidl::android::media::audio::common::AudioPortConfig& pc,
    const std::optional<
        ::aidl::android::media::audio::common::AudioOffloadInfo>& offloadInfo,
    std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback>
        asyncCallback,
    const bool isInput)
    : mPortConfig(pc),
      mTag(AudioUsecase::getUsecaseTag(pc)),
      mIsInput(isInput),
      mFrameSize(getFrameSizeInBytes(mPortConfig.format.value(),
                                     mPortConfig.channelMask.value())) {
    if (mTag == Tag::PRIMARY_PLAYBACK) {
        mExt.emplace<PrimaryPlayback>();
    } else if (mTag == Tag::DEEP_BUFFER_PLAYBACK) {
        mExt.emplace<DeepBufferPlayback>();
    } else if (mTag == Tag::PCM_RECORD) {
        mExt.emplace<PcmRecord>();
    } else if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        if (offloadInfo.has_value()) {
            mExt.emplace<CompressPlayback>(mPortConfig.sampleRate.value().value,
                                           mPortConfig.format.value(),
                                           offloadInfo.value(), asyncCallback);
        }
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        mExt.emplace<CompressCapture>(mPortConfig.format.value(),
                                      mPortConfig.sampleRate.value().value,
                                      mPortConfig.channelMask.value());
    } else if (mTag == Tag::PCM_OFFLOAD_PLAYBACK) {
        mExt.emplace<PcmOffloadPlayback>();
    } else if (mTag == Tag::VOIP_PLAYBACK) {
        mExt.emplace<VoipPlayback>();
    } else if (mTag == Tag::SPATIAL_PLAYBACK) {
        mExt.emplace<SpatialPlayback>();
    } else if(mTag == AudioUsecase::Tag::VOIP_RECORD){
        mExt.emplace<VoipRecord>();
    }
}

AudioUsecase::~AudioUsecase() {
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
}

size_t AudioUsecase::getPeriodSize() const noexcept {
    if (mTag == Tag::PRIMARY_PLAYBACK) {
        return PrimaryPlayback::kPeriodSize * mFrameSize;
    } else if (mTag == Tag::DEEP_BUFFER_PLAYBACK) {
        return DeepBufferPlayback::kPeriodSize * mFrameSize;
    } else if (mTag == Tag::LOW_LATENCY_PLAYBACK) {
        return LowLatencyPlayback::kPeriodSize * mFrameSize;
    } else if (mTag == Tag::PCM_RECORD) {
        return (PcmRecord::kCaptureDurationMs *
                mPortConfig.sampleRate.value().value * mFrameSize) /
               1000;
    } else if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        return CompressPlayback::getPeriodBufferSize(
            mPortConfig.format.value());
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        return CompressCapture::getPeriodBufferSize(mPortConfig.format.value());
    } else if (mTag == Tag::PCM_OFFLOAD_PLAYBACK) {
        return PcmOffloadPlayback::getPeriodSize(
            mPortConfig.format.value(), mPortConfig.channelMask.value(),
            mPortConfig.sampleRate.value().value);
    } else if (mTag == Tag::VOIP_PLAYBACK) {
        return VoipPlayback::getPeriodSize(
            mPortConfig.format.value(), mPortConfig.channelMask.value(),
            mPortConfig.sampleRate.value().value);
    } else if (mTag == Tag::SPATIAL_PLAYBACK) {
        return PrimaryPlayback::kPeriodSize * mFrameSize;
    } else if (mTag == Tag::VOIP_RECORD) {
        return (VoipRecord::kCaptureDurationMs *
                mPortConfig.sampleRate.value().value * mFrameSize) /
               1000;
    }
    return 0;
}

size_t AudioUsecase::getPeriodCount() const noexcept {
    if (mTag == Tag::PRIMARY_PLAYBACK) {
        return PrimaryPlayback::kPeriodCount;
    } else if (mTag == Tag::DEEP_BUFFER_PLAYBACK) {
        return DeepBufferPlayback::kPeriodCount;
    } else if (mTag == Tag::LOW_LATENCY_PLAYBACK) {
        return LowLatencyPlayback::kPeriodCount;
    } else if (mTag == Tag::PCM_RECORD) {
        return PcmRecord::kPeriodCount;
    } else if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        return CompressPlayback::kPeriodCount;
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        return CompressCapture::kPeriodCount;
    } else if (mTag == Tag::PCM_OFFLOAD_PLAYBACK) {
        return PcmOffloadPlayback::kPeriodCount;
    } else if (mTag == Tag::VOIP_PLAYBACK) {
        return VoipPlayback::kPeriodCount;
    } else if (mTag == Tag::SPATIAL_PLAYBACK) {
        return SpatialPlayback::kPeriodCount;
    } else if (mTag == Tag::VOIP_RECORD) {
        return VoipRecord::kPeriodCount;
    }
    return 0;
}

std::string AudioUsecase::toString() const noexcept {
    std::ostringstream os;
    os << "usecase: portConfig: " << mPortConfig.toString();
    os << " connected devices: ";
    std::for_each(mConnectedDevices.cbegin(), mConnectedDevices.cend(),
                  [&os](auto& ele) { os << ele.toString() << ", "; });

    return os.str();
}

uint32_t AudioUsecase::getPlatformDelay() const {
    const uint32_t kDeepBufferPlatformDelayUs = (29 * 1000LL);
    const uint32_t kLowLatencyPlatformDelayUs = (13 * 1000LL);
    return 0;
}

uint32_t AudioUsecase::getLatency() {
    return 0;
}

void AudioUsecase::configurePalDevices(std::vector<pal_device>& palDevices) {
    if (mTag == Tag::PCM_RECORD) {
        const auto& isSourceAvailable = getMixInputSourceType();
        std::get<PcmRecord>(mExt).configurePalDevices(
            isSourceAvailable.has_value() ? isSourceAvailable.value()
                                          : AudioSource::SYS_RESERVED_INVALID,
            getSampleRate(), getChannelLayout(), palDevices);
    }
}


void AudioUsecase::setConnectedDevices(
    const std::vector<::aidl::android::media::audio::common::AudioDevice>&
        connectedDevices) {
    const auto& platform = Platform::getInstance();
    mConnectedDevices = connectedDevices;
    auto connectedPalDevices = platform.getPalDevices(mConnectedDevices);
    configurePalDevices(connectedPalDevices);
    if (this->mPalHandle != nullptr && connectedPalDevices.size() > 0) {
        if (int32_t ret = ::pal_stream_set_device(this->mPalHandle,
                                                  connectedPalDevices.size(),
                                                  connectedPalDevices.data());
            ret) {
            LOG(ERROR) << __func__
                       << " failed to set devices on stream, ret:"
                       << std::to_string(ret);
            return;
        }
        LOG(VERBOSE) << __func__
                     << " devices set on stream successfully";
    }
    return;
}

int32_t AudioUsecase::write(uint8_t* dataPtr, size_t frameCount) {
    pal_buffer palBuffer{};
    palBuffer.buffer = static_cast<uint8_t*>(dataPtr);
    palBuffer.size = frameCount * mFrameSize;
    int32_t dataWritten = ::pal_stream_write(mPalHandle, &palBuffer);
    if (dataWritten < 0) {
        LOG(ERROR) << __func__
                   << " write failed, ret:" << std::to_string(dataWritten);
        return -EINVAL;
    }
    return 0;
}

int32_t AudioUsecase::getPresentationPosition(int64_t* positionInFrames) {
    int64_t frames = 0;
    if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        if (int32_t ret = getPresentationPosition(&frames); ret) {
            return ret;
        }
    }

    if (frames == 0) {
        return -EINVAL;
    }

    auto& platform = Platform::getInstance();
    auto latencyMs = platform.getBluetoothLatencyMs(mConnectedDevices);
    const auto offset = latencyMs * getSampleRate() / 1000;
    frames = (frames > offset) ? (frames - offset) : 0;
    *positionInFrames = frames;
    return 0;
}

int32_t AudioUsecase::read(uint8_t* dataPtr, size_t frameCount,
                           size_t* actualFrameCount) {
    pal_buffer palBuffer{};
    palBuffer.buffer = static_cast<uint8_t*>(dataPtr);
    palBuffer.size = frameCount * mFrameSize;
    int32_t bytesRead = ::pal_stream_read(mPalHandle, &palBuffer);
    if (bytesRead < 0) {
        LOG(ERROR) << __func__
                   << " read failed, ret:" << std::to_string(bytesRead);
        return -EINVAL;
    }
    if (mTag == Tag::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        compressCapture.mNumReadCalls++;
        *actualFrameCount =
            compressCapture.mNumReadCalls * compressCapture.mPCMSamplesPerFrame;
    } else {
        *actualFrameCount =
            static_cast<size_t>(bytesRead) / mFrameSize;
    }

    return 0;
}

int32_t AudioUsecase::drain(
    ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode mode) {
    auto palDrainMode = mode == ::aidl::android::hardware::audio::core::
                                    StreamDescriptor::DrainMode::DRAIN_ALL
                            ? PAL_DRAIN
                            : PAL_DRAIN_PARTIAL;
    if (int32_t ret = ::pal_stream_drain(mPalHandle, palDrainMode); ret) {
        LOG(ERROR) << __func__
                   << " failed to drain the stream, ret:"
                   << std::to_string(ret);
        return ret;
    }
    LOG(VERBOSE) << __func__ << " drain successful";
    return 0;
}

int32_t AudioUsecase::flush() {
    if (int32_t ret = ::pal_stream_flush(mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " failed to flush the stream, ret:"
                   << std::to_string(ret);
        return ret;
    }
    LOG(VERBOSE) << __func__ << " flush successful";
    return 0;
}

int32_t AudioUsecase::standBy() {
    int32_t ret = ::pal_stream_stop(mPalHandle);
    if (ret) {
        LOG(ERROR) << __func__
                   << " failed to stop the stream, ret:" << std::to_string(ret);
    }
    mIsConfigured = false;
    ret = ::pal_stream_close(mPalHandle);
    if (ret) {
        LOG(ERROR) << __func__
                   << " failed to close the stream, ret:"
                   << std::to_string(ret);
        return ret;
    }
    mPalHandle = nullptr;
    LOG(VERBOSE) << __func__ << " pal stream closed on standby";
    return ret;
}

int32_t AudioUsecase::pause() {
    if (int32_t ret = pal_stream_pause(mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " failed to pause the stream, ret:"
                   << std::to_string(ret);
        return ret;
    }
    LOG(VERBOSE) << __func__ << " pause successful";
    mIsPaused = true;
    return 0;
}

int32_t AudioUsecase::resume() {
    if (int32_t ret = ::pal_stream_resume(mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " failed to resume the stream, ret:"
                   << std::to_string(ret);
        return ret;
    }
    LOG(VERBOSE) << __func__ << " resume successful";
    mIsPaused = false;
    return 0;
}

int32_t AudioUsecase::close() {
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
    mIsConfigured = false;
    mPalHandle = nullptr;
    return 0;
}

int32_t AudioUsecase::prepareToClose() { return 0; }
int32_t AudioUsecase::updateHwAvSyncId(int32_t in_hwAvSyncId) { return 0; }
int32_t AudioUsecase::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        return compressPlayback.getVendorParameters(in_ids, _aidl_return);
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        return compressCapture.getVendorParameters(in_ids, _aidl_return);
    }
    return 0;
}
int32_t AudioUsecase::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        return compressPlayback.setVendorParameters(in_parameters, in_async);
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        return compressCapture.setVendorParameters(in_parameters, in_async);
    }
    return 0;
}
int32_t AudioUsecase::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return 0;
}
int32_t AudioUsecase::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    return 0;
}

bool AudioUsecase::configure() {
    auto& platform = Platform::getInstance();
    auto attr = platform.getPalStreamAttributes(mPortConfig, mIsInput);
    if (!attr) {
        return false;
    }
    if (mTag == Tag::DEEP_BUFFER_PLAYBACK || mTag == Tag::PRIMARY_PLAYBACK ||
        mTag == Tag::PCM_RECORD) {
        attr->type = PAL_STREAM_DEEP_BUFFER;
    } else if (mTag == Tag::LOW_LATENCY_PLAYBACK) {
        attr->type = PAL_STREAM_LOW_LATENCY;
    } else if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        attr->type = PAL_STREAM_COMPRESSED;
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        attr->type = PAL_STREAM_COMPRESSED;
    } else if (mTag == Tag::PCM_OFFLOAD_PLAYBACK) {
        attr->type = PAL_STREAM_PCM_OFFLOAD;
    } else if (mTag == Tag::VOIP_PLAYBACK) {
        attr->type = PAL_STREAM_VOIP_TX;
    } else if (mTag == Tag::SPATIAL_PLAYBACK) {
        attr->type = PAL_STREAM_SPATIAL_AUDIO;
    } else if (mTag == Tag::VOIP_RECORD) {
        attr->type = PAL_STREAM_VOIP_TX;
    } else {
        LOG(VERBOSE) << __func__
                     << " invalid usecase to configure";
        return false;
    }

    LOG(VERBOSE) << __func__ << " assigned pal stream type:" << attr->type
                 << " for " << AudioUsecase::getName(mTag);

    auto palDevices = platform.getPalDevices(getConnectedDevices());
    if (!palDevices.size()) {
        return false;
    }

    uint64_t cookie = reinterpret_cast<uint64_t>(this);
    pal_stream_callback palFn = nullptr;
    if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        auto& compressPlayback = std::get<CompressPlayback>(mExt);
        cookie = reinterpret_cast<uint64_t>(&compressPlayback);
        palFn = CompressPlayback::palCallback;
    }

    if (int32_t ret =
            ::pal_stream_open(attr.get(), palDevices.size(), palDevices.data(),
                              0, nullptr, palFn, cookie, &(this->mPalHandle));
        ret) {
        LOG(ERROR) << __func__
                   << " pal stream open failed!!! ret:" << std::to_string(ret);
        return false;
    }

    const size_t ringBufSizeInBytes = getPeriodSize();
    const size_t ringBufCount = getPeriodCount();
    auto palBufferConfig =
        platform.getPalBufferConfig(ringBufSizeInBytes, ringBufCount);
    LOG(VERBOSE) << __func__ << " pal stream set buffer size "
                 << std::to_string(ringBufSizeInBytes) << " with count "
                 << std::to_string(ringBufCount);
    if (int32_t ret = ::pal_stream_set_buffer_size(
            this->mPalHandle, (mIsInput ? palBufferConfig.get() : nullptr),
            ((!mIsInput) ? palBufferConfig.get() : nullptr));
        ret) {
        LOG(ERROR) << __func__ << " pal stream set buffer size failed!!! ret:"
                   << std::to_string(ret);
        return false;
    }
    LOG(VERBOSE) << __func__ << " pal stream set buffer size successful";
    if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK ||
        mTag == Tag::COMPRESS_CAPTURE) {
        auto palParamPayload =
            mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK
                ? std::get<CompressPlayback>(mExt).getPayloadCodecInfo()
                : std::get<CompressCapture>(mExt).getPayloadCodecInfo();
        if (int32_t ret = ::pal_stream_set_param(
                this->mPalHandle, PAL_PARAM_ID_CODEC_CONFIGURATION,
                reinterpret_cast<pal_param_payload*>(palParamPayload.get()));
            ret) {
            LOG(VERBOSE) << __func__
                         << " pal stream set param failed!!! ret:" << ret;
            return false;
        }
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " pal stream start failed!! ret:" << std::to_string(ret);
        return false;
    }
    LOG(VERBOSE) << __func__ << " pal stream start successful";
    mIsConfigured = true;

    // configure mExt
    if (mTag == Tag::COMPRESS_OFFLOAD_PLAYBACK) {
        std::get<CompressPlayback>(mExt).setPalHandle(mPalHandle);
    } else if (mTag == Tag::COMPRESS_CAPTURE) {
        std::get<CompressPlayback>(mExt).setPalHandle(mPalHandle);
    }
    return mIsConfigured;
}

}  // namespace qti::audio::core