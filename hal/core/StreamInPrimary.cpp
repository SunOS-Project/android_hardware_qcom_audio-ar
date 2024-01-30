/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_StreamIn_QTI"

#include <cmath>

#include <aidl/android/hardware/audio/effect/IEffect.h>

#include <android-base/logging.h>
#include <audio_utils/clock.h>
#include <hardware/audio.h>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/ModulePrimary.h>
#include <qti-audio-core/StreamInPrimary.h>
#include <system/audio.h>
#include <extensions/PerfLock.h>

using aidl::android::hardware::audio::common::AudioOffloadMetadata;
using aidl::android::hardware::audio::common::getChannelCount;
using aidl::android::hardware::audio::common::getFrameSizeInBytes;
using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioDualMonoMode;
using aidl::android::media::audio::common::AudioLatencyMode;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::AudioPortExt;
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

// uncomment this to enable logging of very verbose logs like burst commands.
// #define VERY_VERBOSE_LOGGING 1

namespace qti::audio::core {

std::mutex StreamInPrimary::sinkMetadata_mutex_;

StreamInPrimary::StreamInPrimary(StreamContext&& context, const SinkMetadata& sinkMetadata,
                                 const std::vector<MicrophoneInfo>& microphones)
    : StreamIn(std::move(context), microphones),
      StreamCommonImpl(&(StreamIn::mContext), sinkMetadata),
      mTag(getUsecaseTag(getContext().getMixPortConfig())),
      mTagName(getName(mTag)),
      mFrameSizeBytes(getContext().getFrameSize()),
      mMixPortConfig(getContext().getMixPortConfig()) {
    if (mTag == Usecase::PCM_RECORD) {
        mExt.emplace<PcmRecord>();
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        mExt.emplace<CompressCapture>(mMixPortConfig.format.value(),
                                      mMixPortConfig.sampleRate.value().value,
                                      mMixPortConfig.channelMask.value());
    } else if (mTag == Usecase::VOIP_RECORD) {
        mExt.emplace<VoipRecord>();
    } else if (mTag == Usecase::MMAP_RECORD) {
        mExt.emplace<MMapRecord>();
    } else if (mTag == Usecase::VOICE_CALL_RECORD) {
        mExt.emplace<VoiceCallRecord>();
    } else if (mTag == Usecase::FAST_RECORD) {
        mExt.emplace<FastRecord>();
    } else if (mTag == Usecase::ULTRA_FAST_RECORD) {
        mExt.emplace<UltraFastRecord>();
    } else if (mTag == Usecase::HOTWORD_RECORD) {
        mExt.emplace<HotwordRecord>();
    }
    LOG(DEBUG) << __func__ << *this;
}

StreamInPrimary::~StreamInPrimary() {
    shutdown();
    LOG(DEBUG) << __func__ << *this;
}

StreamInPrimary::operator const char*() const noexcept {
    std::ostringstream os;
    os << " : usecase: " << mTagName;
    os << " IoHandle:" << mMixPortConfig.ext.get<AudioPortExt::Tag::mix>().handle;
    return os.str().c_str();
}

// start of methods called from IModule
ndk::ScopedAStatus StreamInPrimary::setConnectedDevices(
        const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices) {
    mWorker->setIsConnected(!devices.empty());
    mConnectedDevices = devices;
    auto connectedPalDevices =
            mPlatform.configureAndFetchPalDevices(mMixPortConfig, mTag, mConnectedDevices);
    if (mTag == Usecase::PCM_RECORD) {
        std::get<PcmRecord>(mExt).configurePalDevices(mMixPortConfig, connectedPalDevices);
    } else if (mTag == Usecase::ULTRA_FAST_RECORD) {
        auto countProxyDevices = std::count_if(mConnectedDevices.cbegin(), mConnectedDevices.cend(),
                                               isInputAFEProxyDevice);
        if (countProxyDevices > 0) {
            std::get<UltraFastRecord>(mExt).mIsWFDCapture = true;
            LOG(INFO) << __func__ << *this
                      << ": ultra fast record on input AFE proxy (WFD client AHAL CAPTURE)";
        } else {
            std::get<UltraFastRecord>(mExt).mIsWFDCapture = false;
            auto channelCount = getChannelCount(mMixPortConfig.channelMask.value());
            if (channelCount == 2) {
                mPlatform.configurePalDevicesCustomKey(connectedPalDevices, "dual-mic");
            }
        }
    }

    if (this->mPalHandle != nullptr && connectedPalDevices.size() > 0) {
        if (int32_t ret = ::pal_stream_set_device(this->mPalHandle, connectedPalDevices.size(),
                                                  connectedPalDevices.data());
            ret) {
            LOG(ERROR) << __func__ << *this << " failed pal_stream_set_device, ret:" << ret;
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
    }

    auto devicesString = [](std::string prev, const auto& device) {
        return std::move(prev) + ';' + device.toString();
    };

    LOG(DEBUG) << __func__ << *this << " stream is connected to devices:"
                 << std::accumulate(mConnectedDevices.cbegin(), mConnectedDevices.cend(),
                                    std::string(""), devicesString);

    return ndk::ScopedAStatus::ok();
}

void StreamInPrimary::setStreamMicMute(const bool muted) {
    if (mPalHandle == nullptr) {
        return;
    }
    if (!mPlatform.setStreamMicMute(mPalHandle, muted)) {
        LOG(ERROR) << __func__ << *this << " failed";
        return;
    }
}

ndk::ScopedAStatus StreamInPrimary::configureMMapStream(int32_t* fd, int64_t* burstSizeFrames,
                                                        int32_t* flags, int32_t* bufferSizeFrames) {
    PerfLock perfLock;
    if (mTag != Usecase::MMAP_RECORD) {
        LOG(ERROR) << __func__ << *this << " cannot call on non-MMAP stream types";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    auto attr = mPlatform.getPalStreamAttributes(mMixPortConfig, true);
    if (!attr) {
        LOG(ERROR) << __func__ << *this << " no pal attributes";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    attr->type = PAL_STREAM_ULTRA_LOW_LATENCY;
    auto palDevices =
            mPlatform.configureAndFetchPalDevices(mMixPortConfig, mTag, mConnectedDevices);
    if (!palDevices.size()) {
        LOG(ERROR) << __func__ << *this << " no connected devices on stream";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    uint64_t cookie = reinterpret_cast<uint64_t>(this);
    pal_stream_callback palFn = nullptr;
    attr->flags = static_cast<pal_stream_flags_t>(PAL_STREAM_FLAG_MMAP_NO_IRQ);

    if (int32_t ret = ::pal_stream_open(attr.get(), palDevices.size(), palDevices.data(), 0,
                                        nullptr, palFn, cookie, &(this->mPalHandle));
        ret) {
        LOG(ERROR) << __func__ << *this << " pal_stream_open failed, ret:" << std::to_string(ret);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    const size_t ringBufSizeInBytes = getPeriodSize();
    const size_t ringBufCount = getPeriodCount();
    auto palBufferConfig = mPlatform.getPalBufferConfig(ringBufSizeInBytes, ringBufCount);
    LOG(DEBUG) << __func__ << *this << " set pal_stream_set_buffer_size to "
                 << std::to_string(ringBufSizeInBytes) << " with count "
                 << std::to_string(ringBufCount);
    if (int32_t ret =
                ::pal_stream_set_buffer_size(this->mPalHandle, palBufferConfig.get(), nullptr);
        ret) {
        LOG(ERROR) << __func__ << *this
                   << " pal_stream_set_buffer_size failed, ret:" << std::to_string(ret);
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    std::get<MMapRecord>(mExt).setPalHandle(mPalHandle);

    const auto frameSize = ::aidl::android::hardware::audio::common::getFrameSizeInBytes(
            mMixPortConfig.format.value(), mMixPortConfig.channelMask.value());
    int32_t ret = std::get<MMapRecord>(mExt).createMMapBuffer(frameSize, fd, burstSizeFrames, flags,
                                                              bufferSizeFrames);
    if (ret != 0) {
        LOG(ERROR) << __func__ << *this << " createMMapBuffer failed";
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__ << *this << " pal_stream_start failed!! ret:" << std::to_string(ret);
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    LOG(INFO) << __func__ << *this << " stream is configured";

    return ndk::ScopedAStatus::ok();
}

// end of methods called from IModule

// start of driverInterface methods

::android::status_t StreamInPrimary::init() {
    return ::android::OK;
}

::android::status_t StreamInPrimary::drain(
        ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode mode) {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << *this << " stream is not configured";
        return ::android::OK;
    }
    // No op
    return ::android::OK;
}

::android::status_t StreamInPrimary::flush() {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << *this << " stream is not configured";
        return ::android::OK;
    }
    // No op
    return ::android::OK;
}

::android::status_t StreamInPrimary::pause() {
    // Todo check whether pause is possible in PAL
    shutdown();
    return ::android::OK;
}

void StreamInPrimary::resume() {
    // No op
}

::android::status_t StreamInPrimary::standby() {
    shutdown();
    return ::android::OK;
}

::android::status_t StreamInPrimary::start() {
    return ::android::OK;
}

::android::status_t StreamInPrimary::transfer(void* buffer, size_t frameCount,
                                              size_t* actualFrameCount, int32_t* latencyMs) {
    if (!mPalHandle) {
        PerfLock perfLock;
        configure();
        if (!mPalHandle) {
            LOG(ERROR) << __func__ << *this << ": failed to configure";
            return ::android::UNEXPECTED_NULL;
        }
    }

    pal_buffer palBuffer{};
    palBuffer.buffer = static_cast<uint8_t*>(buffer);
    palBuffer.size = frameCount * mFrameSizeBytes;
#ifdef VERY_VERBOSE_LOGGING
    LOG(VERBOSE) << __func__ << *this << ": framecount " << frameCount << " mFrameSizeBytes "
                 << mFrameSizeBytes;
#endif
    int32_t bytesRead = ::pal_stream_read(mPalHandle, &palBuffer);
    if (bytesRead < 0) {
        LOG(ERROR) << __func__ << *this << " read failed, ret:" << std::to_string(bytesRead);
        return ::android::NOT_ENOUGH_DATA;
    }

    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        compressCapture.mNumReadCalls++;
        *latencyMs = compressCapture.getLatencyMs();
    } else if (mTag == Usecase::PCM_RECORD || mTag == Usecase::HOTWORD_RECORD) {
        *latencyMs = PcmRecord::kCaptureDurationMs;
    } else {
        // default latency
        *latencyMs = Module::kLatencyMs;
    }
    *actualFrameCount = static_cast<size_t>(bytesRead) / mFrameSizeBytes;

#ifdef VERY_VERBOSE_LOGGING
    LOG(VERBOSE) << __func__ << *this << ": bytes read " << bytesRead << ", return frame count "
                 << *actualFrameCount;
#endif

    return ::android::OK;
}

::android::status_t StreamInPrimary::refinePosition(
        ::aidl::android::hardware::audio::core::StreamDescriptor::Reply* reply) {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << *this << " stream is not configured";
        return ::android::OK;
    }
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto& compressCapture = std::get<CompressCapture>(mExt);
        reply->observable.frames =
                compressCapture.mNumReadCalls * compressCapture.mPCMSamplesPerFrame;
    } else if (mTag == Usecase::MMAP_RECORD) {
        int64_t frames = 0;
        int64_t timeNs = 0;
        int32_t ret = 0;

        ret = std::get<MMapRecord>(mExt).getMMapPosition(&frames, &timeNs);
        if (ret == 0) {
            reply->hardware.frames = frames;
            reply->hardware.timeNs = timeNs;
            LOG(VERBOSE) << __func__ << *this << ": returning MMAP position: frames "
                         << reply->hardware.frames << " timeNs " << reply->hardware.timeNs;
        } else {
            LOG(ERROR) << __func__ << ": getMmapPosition failed, ret= " << ret;
            return ::android::base::ERROR;
        }
    }

    return ::android::OK;
}

void StreamInPrimary::shutdown() {
    LOG(DEBUG) << __func__ << *this;
    if (mPalHandle != nullptr) {
        if (mTag == Usecase::HOTWORD_RECORD) {
            ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_STOP_BUFFERING, nullptr);
        } else {
            ::pal_stream_stop(mPalHandle);
            ::pal_stream_close(mPalHandle);
        }
    }
    mPalHandle = nullptr;
}

// end of driverInterface methods

// start of StreamCommonInterface methods

ndk::ScopedAStatus StreamInPrimary::updateMetadataCommon(const Metadata& metadata) {
    if (!isClosed()) {
        if (metadata.index() != mMetadata.index()) {
            LOG(FATAL) << __func__ << *this << ": changing metadata variant is not allowed";
        }
        mMetadata = metadata;
    }
    int callState = mPlatform.getCallState();
    int callMode = mPlatform.getCallMode();
    bool voiceActive = ((callState == 2) || (callMode == 2));

    StreamInPrimary::sinkMetadata_mutex_.lock();
    setAggregateSinkMetadata(voiceActive);
    StreamInPrimary::sinkMetadata_mutex_.unlock();
    LOG(ERROR) << __func__ << *this << ": stream was closed";
    // return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    return ndk::ScopedAStatus::ok();
}

int32_t StreamInPrimary::setAggregateSinkMetadata(bool voiceActive) {
    ssize_t track_count_total = 0;

    std::vector<record_track_metadata_t> total_tracks;
    sink_metadata_t btSinkMetadata;

    ModulePrimary::inListMutex.lock();
    std::vector<std::weak_ptr<StreamIn>>& inStreams = ModulePrimary::getInStreams();
    // Dont send metadata if voice is active
    if (voiceActive || inStreams.empty()) {
        ModulePrimary::inListMutex.unlock();
        return 0;
    }
    auto removeStreams = [&](std::weak_ptr<StreamIn> streamIn) -> bool {
        if (!streamIn.lock()) return true;
        return streamIn.lock()->isClosed();
    };

    inStreams.erase(std::remove_if(inStreams.begin(), inStreams.end(), removeStreams),
                    inStreams.end());

    LOG(VERBOSE) << __func__ << *this << " in streams not empty size is " << inStreams.size();
    for (auto it = inStreams.begin(); it < inStreams.end(); it++) {
        if (it->lock() && !it->lock()->isClosed()) {
            ::aidl::android::hardware::audio::common::SinkMetadata sinkMetadata;
            it->lock()->getMetadata(sinkMetadata);
            track_count_total += sinkMetadata.tracks.size();
        } else {
        }
    }
    LOG(VERBOSE) << __func__ << *this << " total tracks count is " << track_count_total;
    if (track_count_total == 0) {
        ModulePrimary::inListMutex.unlock();
        return 0;
    }

    total_tracks.resize(track_count_total);
    btSinkMetadata.track_count = track_count_total;
    btSinkMetadata.tracks = total_tracks.data();

    for (auto it = inStreams.begin(); it != inStreams.end(); it++) {
        ::aidl::android::hardware::audio::common::SinkMetadata sinkMetadata;
        if (it->lock()) {
            it->lock()->getMetadata(sinkMetadata);
            for (auto& item : sinkMetadata.tracks) {
                btSinkMetadata.tracks->source = static_cast<audio_source_t>(item.source);
                ++btSinkMetadata.tracks;
            }
        }
    }

    btSinkMetadata.tracks = total_tracks.data();
    LOG(VERBOSE) << __func__ << *this << " sending sink metadata to PAL";
    pal_set_param(PAL_PARAM_ID_SET_SINK_METADATA, (void*)&btSinkMetadata, 0);
    LOG(VERBOSE) << __func__ << *this << " after sending sink metadata to PAL";
    ModulePrimary::inListMutex.unlock();
    return 0;
}

ndk::ScopedAStatus StreamInPrimary::getVendorParameters(
        const std::vector<std::string>& in_ids, std::vector<VendorParameter>* _aidl_return) {
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
        const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>& in_effect) {
    if (in_effect == nullptr) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    ::aidl::android::hardware::audio::effect::Descriptor desc;
    auto status = in_effect->getDescriptor(&desc);
    if (!status.isOk()) {
        LOG(ERROR) << __func__ << *this << "error fetching descriptor";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    const auto& typeUUID = desc.common.id.type;

    if (typeUUID == getEffectTypeUuidAcousticEchoCanceler()) {
        if (!mAECEnabled) {
            auto tag = mNSEnabled ? PAL_AUDIO_EFFECT_ECNS : PAL_AUDIO_EFFECT_EC;
            LOG(DEBUG) << __func__ << *this << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, true); !ret) {
                mAECEnabled = true;
            } else {
                LOG(ERROR) << __func__ << *this << "failed for AEC";
            }
        }
    } else if (typeUUID == getEffectTypeUuidNoiseSuppression()) {
        if (!mNSEnabled) {
            auto tag = mAECEnabled ? PAL_AUDIO_EFFECT_ECNS : PAL_AUDIO_EFFECT_NS;
            LOG(DEBUG) << __func__ << *this << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, true); !ret) {
                mNSEnabled = true;
            } else {
                LOG(ERROR) << __func__ << *this << "failed for NS";
            }
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamInPrimary::removeEffect(
        const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>& in_effect) {
    if (in_effect == nullptr) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    ::aidl::android::hardware::audio::effect::Descriptor desc;
    auto status = in_effect->getDescriptor(&desc);
    if (!status.isOk()) {
        LOG(ERROR) << __func__ << *this << "error fetching descriptor";
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
            LOG(DEBUG) << __func__ << *this << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, mNSEnabled); !ret) {
                mAECEnabled = false;
            } else {
                LOG(ERROR) << __func__ << *this << "failed for AEC";
            }
        }
    } else if (typeUUID == getEffectTypeUuidNoiseSuppression()) {
        if (!mNSEnabled) {
            auto tag = mAECEnabled ? PAL_AUDIO_EFFECT_ECNS : PAL_AUDIO_EFFECT_NS;
            LOG(DEBUG) << __func__ << *this << "effectType " << tag;
            if (auto ret = pal_add_remove_effect(mPalHandle, tag, mAECEnabled); !ret) {
                mNSEnabled = false;
            } else {
                LOG(ERROR) << __func__ << *this << "failed for NS";
            }
        }
    }
    return ndk::ScopedAStatus::ok();
}

// end of StreamCommonInterface methods

size_t StreamInPrimary::getPeriodSize() const noexcept {
    if (mTag == Usecase::PCM_RECORD) {
        return PcmRecord::getMinFrames(mMixPortConfig) * mFrameSizeBytes;
    } else if (mTag == Usecase::FAST_RECORD) {
        return FastRecord::getPeriodSize(mMixPortConfig);
    } else if (mTag == Usecase::ULTRA_FAST_RECORD) {
        return UltraFastRecord::kPeriodSize * mFrameSizeBytes;
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        return CompressCapture::getPeriodBufferSize(mMixPortConfig.format.value());
    } else if (mTag == Usecase::VOIP_RECORD) {
        return (VoipRecord::kCaptureDurationMs * mMixPortConfig.sampleRate.value().value *
                mFrameSizeBytes) /
               1000;
    } else if (mTag == Usecase::MMAP_RECORD) {
        return MMapRecord::getPeriodSize(mMixPortConfig.format.value(),
                                         mMixPortConfig.channelMask.value());
    } else if (mTag == Usecase::VOICE_CALL_RECORD) {
        return VoiceCallRecord::getPeriodSize(mMixPortConfig);
    }
    return 0;
}

size_t StreamInPrimary::getPeriodCount() const noexcept {
    if (mTag == Usecase::PCM_RECORD) {
        return PcmRecord::kPeriodCount;
    } else if (mTag == Usecase::FAST_RECORD) {
        return FastRecord::kPeriodCount;
    } else if (mTag == Usecase::ULTRA_FAST_RECORD) {
        return UltraFastRecord::kPeriodCount;
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        return CompressCapture::kPeriodCount;
    } else if (mTag == Usecase::VOIP_RECORD) {
        return VoipRecord::kPeriodCount;
    } else if (mTag == Usecase::MMAP_RECORD) {
        return MMapRecord::kPeriodCount;
    } else if (mTag == Usecase::VOICE_CALL_RECORD) {
        return VoiceCallRecord::kPeriodCount;
    }
    return 0;
}

size_t StreamInPrimary::getPlatformDelay() const noexcept {
    return 0;
}

void StreamInPrimary::configure() {
    auto attr = mPlatform.getPalStreamAttributes(mMixPortConfig, true);
    if (!attr) {
        LOG(ERROR) << __func__ << *this << " no pal attributes";
        return;
    }
    if (mTag == Usecase::PCM_RECORD) {
        attr->type = PAL_STREAM_DEEP_BUFFER;
    } else if (mTag == Usecase::COMPRESS_CAPTURE) {
        attr->type = PAL_STREAM_COMPRESSED;
    } else if (mTag == Usecase::VOIP_RECORD) {
        attr->type = PAL_STREAM_VOIP_TX;
    } else if (mTag == Usecase::VOICE_CALL_RECORD) {
        attr->type = PAL_STREAM_VOICE_CALL_RECORD;
        attr->info.voice_rec_info.record_direction =
                std::get<VoiceCallRecord>(mExt).getRecordDirection(mMixPortConfig);
    } else if (mTag == Usecase::FAST_RECORD) {
        attr->type = PAL_STREAM_LOW_LATENCY;
    } else if (mTag == Usecase::ULTRA_FAST_RECORD) {
        if (std::get<UltraFastRecord>(mExt).mIsWFDCapture) {
            attr->type = PAL_STREAM_PROXY;
            attr->info.opt_stream_info.tx_proxy_type = PAL_STREAM_PROXY_TX_WFD;
        } else {
            attr->type = PAL_STREAM_ULTRA_LOW_LATENCY;
            attr->flags = PAL_STREAM_FLAG_MMAP;
        }
    } else if (mTag == Usecase::HOTWORD_RECORD) {
        mPalHandle = std::get<HotwordRecord>(mExt).getPalHandle(mMixPortConfig);
        return;
    } else {
        LOG(ERROR) << __func__ << *this << " invalid usecase to configure";
        return;
    }

    LOG(VERBOSE) << __func__ << *this << " assigned pal stream type:" << attr->type;

    auto palDevices =
            mPlatform.configureAndFetchPalDevices(mMixPortConfig, mTag, mConnectedDevices);
    if (!palDevices.size()) {
        LOG(ERROR) << __func__ << *this << " no connected devices on stream!!";
        return;
    }

    uint64_t cookie = reinterpret_cast<uint64_t>(this);
    pal_stream_callback palFn = nullptr;

    if (int32_t ret = ::pal_stream_open(attr.get(), palDevices.size(), palDevices.data(), 0,
                                        nullptr, palFn, cookie, &(mPalHandle));
        ret) {
        LOG(ERROR) << __func__ << *this << " pal_stream_open failed!!! ret:" << ret;
        mPalHandle = nullptr;
        return;
    }

    const size_t ringBufSizeInBytes = getPeriodSize();
    const size_t ringBufCount = getPeriodCount();
    auto palBufferConfig = mPlatform.getPalBufferConfig(ringBufSizeInBytes, ringBufCount);
    LOG(VERBOSE) << __func__ << *this << " set pal_stream_set_buffer_size to " << ringBufSizeInBytes
                 << " with count " << ringBufCount;
    if (int32_t ret = ::pal_stream_set_buffer_size(mPalHandle, palBufferConfig.get(), nullptr);
        ret) {
        LOG(ERROR) << __func__ << *this << " pal_stream_set_buffer_size failed!!! ret:" << ret;
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return;
    }
    LOG(VERBOSE) << __func__ << *this << " pal_stream_set_buffer_size successful";
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        auto palParamPayload = std::get<CompressCapture>(mExt).getPayloadCodecInfo();
        if (int32_t ret = ::pal_stream_set_param(
                    this->mPalHandle, PAL_PARAM_ID_CODEC_CONFIGURATION,
                    reinterpret_cast<pal_param_payload*>(palParamPayload.get()));
            ret) {
            LOG(VERBOSE) << __func__ << *this << " pal_stream_set_param failed!!! ret:" << ret;
            ::pal_stream_close(mPalHandle);
            mPalHandle = nullptr;
            return;
        }
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__ << *this << " pal_stream_start failed!! ret:" << ret;
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return;
    }

    LOG(VERBOSE) << __func__ << *this << " pal_stream_start successful";

    // configure mExt
    if (mTag == Usecase::COMPRESS_CAPTURE) {
        std::get<CompressCapture>(mExt).setPalHandle(mPalHandle);
    }

    LOG(DEBUG) << __func__ << *this << " : stream is configured";
}

} // namespace qti::audio::core
