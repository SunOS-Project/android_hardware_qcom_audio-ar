/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#define LOG_TAG "AHAL_QStreamIn"

#include <cmath>

#include <aidl/android/hardware/audio/effect/IEffect.h>

#include <android-base/logging.h>
#include <audio_utils/clock.h>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/StreamInPrimary.h>

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
using ::aidl::android::hardware::audio::effect::getEffectTypeUuidAcousticEchoCanceler;
using ::aidl::android::hardware::audio::effect::getEffectTypeUuidNoiseSuppression;
namespace qti::audio::core {

StreamInPrimary::StreamInPrimary(StreamContext&& context,
                                 const SinkMetadata& sinkMetadata,
                                 const std::vector<MicrophoneInfo>& microphones)
    : StreamIn(std::move(context), microphones),
      StreamCommonImpl(&(StreamIn::mContext), sinkMetadata),
      mTag(getUsecaseTag(getContext().getMixPortConfig())),
      mFrameSizeBytes(getContext().getFrameSize()),
      mSampleRate(getContext().getSampleRate()),
      mIsAsynchronous(!!getContext().getAsyncCallback()),
      mIsInput(true) {
    if (mTag == Usecase::PCM_RECORD) {
        mExt.emplace<PcmRecord>();
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        mExt.emplace<CompressCapture>(mMixPortConfig.format.value(),
                                      mMixPortConfig.sampleRate.value().value,
                                      mMixPortConfig.channelMask.value());
    } else if (mTag == Usecase::VOIP_RECORD) {
        mExt.emplace<VoipRecord>();
    }
}

StreamInPrimary::~StreamInPrimary() {
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
}

std::string StreamInPrimary::toString() const noexcept {
    std::ostringstream os;
    return os.str();
}

// start of methods called from IModule
ndk::ScopedAStatus StreamInPrimary::setConnectedDevices(
    const std::vector<::aidl::android::media::audio::common::AudioDevice>&
        devices) {
    mWorker->setIsConnected(!devices.empty());
    mConnectedDevices = devices;
    auto connectedPalDevices = mPlatform.getPalDevices(mConnectedDevices);
    if (mTag == Usecase::PCM_RECORD) {
        std::get<PcmRecord>(mExt).configurePalDevices(mMixPortConfig,
                                                      connectedPalDevices);
    }

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

// start of driverInterface methods

::android::status_t StreamInPrimary::init() {
    mIsInitialized = true;
    return ::android::OK;
}

::android::status_t StreamInPrimary::drain(
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

::android::status_t StreamInPrimary::flush() {
    if (int32_t ret = ::pal_stream_flush(mPalHandle); ret) {
        LOG(ERROR) << __func__ << " failed to flush the stream, ret:" << ret;
        return ret;
    }
    LOG(VERBOSE) << __func__ << " flush successful";
    return ::android::OK;
}

::android::status_t StreamInPrimary::pause() {
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

void StreamInPrimary::resume() {
    if (int32_t ret = ::pal_stream_resume(mPalHandle); ret) {
        LOG(ERROR) << __func__ << " failed to resume the stream, ret:" << ret;
    }
    LOG(VERBOSE) << __func__ << " resume successful";
    mIsPaused = false;
}

::android::status_t StreamInPrimary::standby() {
    int32_t ret = ::pal_stream_stop(mPalHandle);
    if (ret) {
        LOG(ERROR) << __func__ << " failed to stop the stream, ret:" << ret;
    }
    mIsConfigured = false;
    ret = ::pal_stream_close(mPalHandle);
    if (ret) {
        LOG(ERROR) << __func__ << " failed to close the stream, ret:" << ret;
        return ret;
    }
    mPalHandle = nullptr;
    LOG(VERBOSE) << __func__ << " pal stream closed on standby";
    mIsStandby = true;
    return ::android::OK;
}

::android::status_t StreamInPrimary::start() {
    mIsStandby = false;
    return ::android::OK;
}

::android::status_t StreamInPrimary::transfer(void* buffer, size_t frameCount,
                                              size_t* actualFrameCount,
                                              int32_t* latencyMs) {
    if (!mIsConfigured) {
        // configure on first transfer
        configure();
    }

    if (mIsPaused) {
        resume();
    }
    pal_buffer palBuffer{};
    palBuffer.buffer = static_cast<uint8_t*>(buffer);
    palBuffer.size = frameCount * mFrameSizeBytes;
    int32_t bytesRead = ::pal_stream_read(mPalHandle, &palBuffer);
    if (bytesRead < 0) {
        LOG(ERROR) << __func__
                   << " read failed, ret:" << std::to_string(bytesRead);
        return ::android::NOT_ENOUGH_DATA;
    }
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        compressCapture.mNumReadCalls++;
    }
    *actualFrameCount = static_cast<size_t>(bytesRead) / mFrameSizeBytes;

    // Todo fix latency
    *latencyMs = Module::kLatencyMs;
    return ::android::OK;
}

::android::status_t StreamInPrimary::refinePosition(
    ::aidl::android::hardware::audio::core::StreamDescriptor::Position*
        position) {
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        position->frames =
            compressCapture.mNumReadCalls * compressCapture.mPCMSamplesPerFrame;
    }

    return ::android::OK;
}

void StreamInPrimary::shutdown() {
    mIsInitialized = false;
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
    mIsConfigured = false;
    mPalHandle = nullptr;
}

// end of driverInterface methods

// start of StreamCommonInterface methods

ndk::ScopedAStatus StreamInPrimary::updateMetadataCommon(
    const Metadata& metadata) {
    LOG(DEBUG) << __func__;
    if (!isClosed()) {
        if (metadata.index() != mMetadata.index()) {
            LOG(FATAL) << __func__
                       << ": changing metadata variant is not allowed";
        }
        mMetadata = metadata;
        return ndk::ScopedAStatus::ok();
    }
    LOG(ERROR) << __func__ << ": stream was closed";
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
}

ndk::ScopedAStatus StreamInPrimary::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<VendorParameter>* _aidl_return) {
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        return compressCapture.getVendorParameters(in_ids, _aidl_return);
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus StreamInPrimary::setVendorParameters(
    const std::vector<VendorParameter>& in_parameters, bool in_async) {
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        return compressCapture.setVendorParameters(in_parameters, in_async);
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus StreamInPrimary::addEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    if (in_effect == nullptr) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    ::aidl::android::hardware::audio::effect::Descriptor desc;
    auto status = in_effect->getDescriptor(&desc);
    if (!status.isOk()) {
        LOG(ERROR) << __func__ << "error fetching descriptor";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    const auto& typeUUID = desc.common.id.type;

    if (typeUUID == getEffectTypeUuidAcousticEchoCanceler()) {
        if (!mAECEnabled) {
            auto tag = mNSEnabled ? PAL_AUDIO_EFFECT_ECNS : PAL_AUDIO_EFFECT_EC;
            LOG(VERBOSE) << __func__ << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, true); !ret) {
                mAECEnabled = true;
            } else {
                LOG(ERROR) << __func__ << "failed for AEC";
            }
        }
    } else if (typeUUID == getEffectTypeUuidNoiseSuppression()){
        if (!mNSEnabled) {
            auto tag = mAECEnabled ? PAL_AUDIO_EFFECT_ECNS : PAL_AUDIO_EFFECT_NS;
            LOG(VERBOSE) << __func__ << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, true); !ret) {
                mNSEnabled = true;
            } else {
                LOG(ERROR) << __func__ << "failed for NS";
            }
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamInPrimary::removeEffect(
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    if (in_effect == nullptr) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    ::aidl::android::hardware::audio::effect::Descriptor desc;
    auto status = in_effect->getDescriptor(&desc);
    if (!status.isOk()) {
        LOG(ERROR) << __func__ << "error fetching descriptor";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    const auto& typeUUID = desc.common.id.type;
    /*
     * While disabling AEC or NS check the other effect is enabled or not.
     * Take action accordingly. In case AEC is disabled, but NS is still
     * enabled then NS has to be enabled alone, otherwise disable both ECNS
     * This can be decided based on the state of other effect.
     * TODO check if https://review-android.quicinc.com/#/c/3346149 needed
     */
    if (typeUUID == getEffectTypeUuidAcousticEchoCanceler()) {
        if (mAECEnabled) {
            auto tag = mNSEnabled ? PAL_AUDIO_EFFECT_NS : PAL_AUDIO_EFFECT_ECNS;
            LOG(DEBUG) << __func__ << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, mNSEnabled); !ret) {
                mAECEnabled = false;
            } else {
                LOG(ERROR) << __func__ << "failed for AEC";
            }
        }
    } else if (typeUUID == getEffectTypeUuidNoiseSuppression()) {
        if (!mNSEnabled) {
            auto tag = mAECEnabled ? PAL_AUDIO_EFFECT_ECNS : PAL_AUDIO_EFFECT_NS;
            LOG(DEBUG) << __func__ << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, mAECEnabled); !ret) {
                mNSEnabled = false;
            } else {
                LOG(ERROR) << __func__ << "failed for NS";
            }
        }
    }
    return ndk::ScopedAStatus::ok();
}

// end of StreamCommonInterface methods

size_t StreamInPrimary::getPeriodSize() const noexcept {
    if (mTag == Usecase::PCM_RECORD) {
        return (PcmRecord::kCaptureDurationMs *
                mMixPortConfig.sampleRate.value().value * mFrameSizeBytes) /
               1000;
    }  else if (mTag == Usecase::COMPRESS_CAPTURE) {
        return CompressCapture::getPeriodBufferSize(mMixPortConfig.format.value());
    } else if (mTag == Usecase::VOIP_RECORD) {
        return (VoipRecord::kCaptureDurationMs *
                mMixPortConfig.sampleRate.value().value * mFrameSizeBytes) /
               1000;
    }
    return 0;
}

size_t StreamInPrimary::getPeriodCount() const noexcept {
    if (mTag == Usecase::PCM_RECORD) {
        return PcmRecord::kPeriodCount;
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        return CompressCapture::kPeriodCount;
    } else if (mTag == Usecase::VOIP_RECORD) {
        return VoipRecord::kPeriodCount;
    }
    return 0;
}

size_t StreamInPrimary::getPlatformDelay() const noexcept {
    return 0;
}

void StreamInPrimary::configure() {
    auto attr = mPlatform.getPalStreamAttributes(mMixPortConfig, true);
    if (!attr) {
        LOG(ERROR) << __func__ << " no pal attributes";
        return;
    }
    if (mTag == Usecase::PCM_RECORD) {
        attr->type = PAL_STREAM_DEEP_BUFFER;
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        attr->type = PAL_STREAM_COMPRESSED;
    } else if (mTag == Usecase::VOIP_RECORD) {
        attr->type = PAL_STREAM_VOIP_TX;
    } else {
        LOG(ERROR) << __func__
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
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto palParamPayload = std::get<CompressCapture>(mExt).getPayloadCodecInfo();
        if (int32_t ret = ::pal_stream_set_param(
                this->mPalHandle, PAL_PARAM_ID_CODEC_CONFIGURATION,
                reinterpret_cast<pal_param_payload*>(palParamPayload.get()));
            ret) {
            LOG(VERBOSE) << __func__
                         << " pal stream set param failed!!! ret:" << ret;
            return;
        }
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__
                   << " pal stream start failed!! ret:" << std::to_string(ret);
        return;
    }
    LOG(VERBOSE) << __func__ << " pal stream start successful";

    // configure mExt
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        std::get<CompressCapture>(mExt).setPalHandle(mPalHandle);
    }

    mIsConfigured = true;
}


}  // namespace qti::audio::core