/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cmath>

#define LOG_TAG "AHAL_QStreamOut"

#include <android-base/logging.h>
#include <audio_utils/clock.h>
#include <hardware/audio.h>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/ModulePrimary.h>
#include <qti-audio-core/StreamOutPrimary.h>
#include <system/audio.h>

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

using aidl::android::media::audio::common::AudioPortExt;

static bool karaoke = false;

namespace qti::audio::core {

std::mutex StreamOutPrimary::sourceMetadata_mutex_;

StreamOutPrimary::StreamOutPrimary(StreamContext&& context, const SourceMetadata& sourceMetadata,
                                   const std::optional<AudioOffloadInfo>& offloadInfo)
    : StreamOut(std::move(context), offloadInfo),
      StreamCommonImpl(&(StreamOut::mContext), sourceMetadata),
      mTag(getUsecaseTag(getContext().getMixPortConfig())),
      mTagName(getName(mTag)),
      mFrameSizeBytes(getContext().getFrameSize()),
      mMixPortConfig(getContext().getMixPortConfig()) {
    if (mTag == Usecase::PRIMARY_PLAYBACK) {
        mExt.emplace<PrimaryPlayback>();
    } else if (mTag == Usecase::DEEP_BUFFER_PLAYBACK) {
        mExt.emplace<DeepBufferPlayback>();
    } else if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        mExt.emplace<CompressPlayback>(offloadInfo.value(), getContext().getAsyncCallback());
    } else if (mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        mExt.emplace<PcmOffloadPlayback>();
    } else if (mTag == Usecase::VOIP_PLAYBACK) {
        mExt.emplace<VoipPlayback>();
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        mExt.emplace<SpatialPlayback>();
    } else if (mTag == Usecase::MMAP_PLAYBACK) {
        mExt.emplace<MMapPlayback>();
    } else if (mTag == Usecase::ULL_PLAYBACK) {
        mExt.emplace<UllPlayback>();
    } else if (mTag == Usecase::IN_CALL_MUSIC) {
        mExt.emplace<InCallMusic>();
    }

    LOG(VERBOSE) << __func__ << ": " << *this;
}

StreamOutPrimary::~StreamOutPrimary() {
    if (mPalHandle != nullptr) {
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
    if (karaoke) mAudExt.mKarokeExtension->karaoke_stop();
    LOG(VERBOSE) << __func__ << ": " << *this;
}

StreamOutPrimary::operator const char*() const noexcept {
    std::ostringstream os;
    os << " " << mTagName;
    os << " IoHandle:" << mMixPortConfig.ext.get<AudioPortExt::Tag::mix>().handle;
    return os.str().c_str();
}

// start of methods called from IModule
ndk::ScopedAStatus StreamOutPrimary::setConnectedDevices(
        const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices) {
    mWorker->setIsConnected(!devices.empty());
    mConnectedDevices = devices;

    if (mTag == Usecase::PRIMARY_PLAYBACK) {
        mPlatform.setPrimaryPlaybackDevices(mConnectedDevices);
        LOG(VERBOSE) << __func__ << ": primary playback devices updated " << mConnectedDevices;
    }

    if (!mPalHandle) {
        LOG(WARNING) << __func__ << ": stream not configured";
        return ndk::ScopedAStatus::ok();
    }
    if (mConnectedDevices.empty()) {
        LOG(VERBOSE) << __func__ << ": stream is not connected";
        return ndk::ScopedAStatus::ok();
    }

    auto connectedPalDevices = mPlatform.getPalDevices(mConnectedDevices);

    if (connectedPalDevices.size() != mConnectedDevices.size()) {
        LOG(ERROR) << __func__ << ": pal devices size != aidl devices size";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    if (!connectedPalDevices.empty() && connectedPalDevices[0].id == PAL_DEVICE_OUT_BLUETOOTH_SCO) {
        if (!mPlatform.isA2dpSuspended()) {
            LOG(ERROR) << __func__ << " Cannot route stream to SCO if A2dp is not suspended";
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
    }

    if (int32_t ret = ::pal_stream_set_device(mPalHandle, connectedPalDevices.size(),
                                              connectedPalDevices.data());
        ret) {
        LOG(ERROR) << __func__ << " failed to set devices on stream, ret:" << ret;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    auto devicesString = [](std::string& prev, const auto& device) {
        return std::move(prev.append(" | ").append(device.toString()));
    };

    LOG(VERBOSE) << __func__ << ": stream is connected to "
                 << std::accumulate(mConnectedDevices.cbegin(), mConnectedDevices.cend(),
                                    std::string(""), devicesString);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus StreamOutPrimary::configureMMapStream(int32_t* fd, int64_t* burstSizeFrames,
                                                         int32_t* flags,
                                                         int32_t* bufferSizeFrames) {
    if (mTag != Usecase::MMAP_PLAYBACK) {
        LOG(ERROR) << __func__ << " Cannot call on non-MMAP stream types";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    auto attr = mPlatform.getPalStreamAttributes(mMixPortConfig, false);
    if (!attr) {
        LOG(ERROR) << __func__ << " no pal attributes";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    attr->type = PAL_STREAM_ULTRA_LOW_LATENCY;
    auto palDevices = mPlatform.getPalDevices(getConnectedDevices());
    if (!palDevices.size()) {
        LOG(ERROR) << __func__ << " no connected devices on stream!!";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    uint64_t cookie = reinterpret_cast<uint64_t>(this);
    pal_stream_callback palFn = nullptr;
    attr->flags = static_cast<pal_stream_flags_t>(PAL_STREAM_FLAG_MMAP_NO_IRQ);
    if (int32_t ret = ::pal_stream_open(attr.get(), palDevices.size(), palDevices.data(), 0,
                                        nullptr, palFn, cookie, &(this->mPalHandle));
        ret) {
        LOG(ERROR) << __func__ << " pal stream open failed!!! ret:" << std::to_string(ret);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    const size_t ringBufSizeInBytes = getPeriodSize();
    const size_t ringBufCount = getPeriodCount();
    auto palBufferConfig = mPlatform.getPalBufferConfig(ringBufSizeInBytes, ringBufCount);
    LOG(VERBOSE) << __func__ << " pal stream set buffer size " << std::to_string(ringBufSizeInBytes)
                 << " with count " << std::to_string(ringBufCount);
    if (int32_t ret =
                ::pal_stream_set_buffer_size(this->mPalHandle, nullptr, palBufferConfig.get());
        ret) {
        LOG(ERROR) << __func__
                   << " pal stream set buffer size failed!!! ret:" << std::to_string(ret);
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    LOG(VERBOSE) << __func__ << " pal stream set buffer size successful";
    std::get<MMapPlayback>(mExt).setPalHandle(mPalHandle);

    const auto frameSize = ::aidl::android::hardware::audio::common::getFrameSizeInBytes(
            mMixPortConfig.format.value(), mMixPortConfig.channelMask.value());
    int32_t ret = std::get<MMapPlayback>(mExt).createMMapBuffer(frameSize, fd, burstSizeFrames,
                                                                flags, bufferSizeFrames);
    if (ret != 0) {
        LOG(ERROR) << __func__ << " Create MMap buffer failed!";
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__ << " pal stream start failed!! ret:" << std::to_string(ret);
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    LOG(INFO) << __func__ << ": stream is configured for " << mTagName;

    return ndk::ScopedAStatus::ok();
}

// end of methods called from IModule

// start of DriverInterface Methods

::android::status_t StreamOutPrimary::init() {
    return ::android::OK;
}

::android::status_t StreamOutPrimary::drain(
        ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode mode) {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << ": stream not configured " << mTagName;
        return ::android::OK;
    }
    auto palDrainMode =
            mode == ::aidl::android::hardware::audio::core::StreamDescriptor::DrainMode::DRAIN_ALL
                    ? PAL_DRAIN
                    : PAL_DRAIN_PARTIAL;
    if (int32_t ret = ::pal_stream_drain(mPalHandle, palDrainMode); ret) {
        LOG(ERROR) << __func__ << " failed to drain the stream, ret:" << ret;
        return ret;
    }
    LOG(VERBOSE) << __func__ << " drained " << mTagName;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::flush() {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << ": stream not configured " << mTagName;
        return ::android::OK;
    }
    if (mTag == Usecase::MMAP_PLAYBACK) {
        LOG(WARNING) << __func__ << " Flushing of MMAP streams is unsupported.";
        return ::android::OK;
    }
    if (int32_t ret = ::pal_stream_flush(mPalHandle); ret) {
        LOG(ERROR) << __func__ << " failed to flush the stream, ret:" << ret;
        return ret;
    }
    LOG(VERBOSE) << __func__ << " " << mTagName;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::pause() {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << ": stream not configured " << mTagName;
        return ::android::OK;
    }
    if (mTag == Usecase::MMAP_PLAYBACK) {
        if (int32_t ret = pal_stream_stop(mPalHandle); ret) {
            LOG(ERROR) << __func__ << " failed to stop MMAP stream, ret:" << std::to_string(ret);
            return ret;
        }
    } else {
        if (int32_t ret = pal_stream_pause(mPalHandle); ret) {
            LOG(ERROR) << __func__ << " failed to pause the stream, ret:" << std::to_string(ret);
            return ret;
        }
    }
    mIsPaused = true;
    LOG(VERBOSE) << __func__ << " " << mTagName;
    return ::android::OK;
}

void StreamOutPrimary::resume() {
    if (mTag == Usecase::MMAP_PLAYBACK) {
        if (int32_t ret = pal_stream_start(mPalHandle); ret) {
            LOG(ERROR) << __func__ << " failed to start the stream, ret:" << ret;
        }
    } else {
        if (int32_t ret = ::pal_stream_resume(mPalHandle); ret) {
            LOG(ERROR) << __func__ << " failed to resume the stream, ret:" << ret;
        }
    }
    LOG(VERBOSE) << __func__ << " " << mTagName;
    mIsPaused = false;
}

::android::status_t StreamOutPrimary::standby() {
    if (!mPalHandle) {
        LOG(WARNING) << __func__ << ": stream not configured " << mTagName;
        return ::android::OK;
    }
    if (mTag == Usecase::MMAP_PLAYBACK)
        return ::android::OK;
    else {
        shutdown();
        LOG(VERBOSE) << __func__ << " " << mTagName;
        return ::android::OK;
    }
}

::android::status_t StreamOutPrimary::start() {
    // hardware is expected to up on start
    // but we are doing on first write
    LOG(VERBOSE) << __func__ << " " << mTagName;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::transfer(void* buffer, size_t frameCount,
                                               size_t* actualFrameCount, int32_t* latencyMs) {
    if (!mPalHandle) {
        // configure on first transfer or after stand by
        configure();
        if (!mPalHandle) {
            LOG(ERROR) << __func__ << ": failed to configure " << mTagName;
            return ::android::UNEXPECTED_NULL;
        }
    }

    if (mIsPaused) {
        resume();
    }
    pal_buffer palBuffer{};
    palBuffer.buffer = static_cast<uint8_t*>(buffer);
    palBuffer.size = frameCount * mFrameSizeBytes;
    if (palBuffer.size == 0) {
        // resume comes with 0 frameCount
        return ::android::OK;
    }
    ssize_t bytesWritten = ::pal_stream_write(mPalHandle, &palBuffer);
    if (bytesWritten < 0) {
        LOG(ERROR) << __func__ << " write failed, ret:" << bytesWritten;
        return ::android::FAILED_TRANSACTION;
    }
    LOG(DEBUG) << __func__ << ": byteswritten:" << bytesWritten;
    *actualFrameCount = static_cast<size_t>(bytesWritten / mFrameSizeBytes);
    // Todo findout write latency
    *latencyMs = Module::kLatencyMs;
    return ::android::OK;
}

::android::status_t StreamOutPrimary::refinePosition(
        ::aidl::android::hardware::audio::core::StreamDescriptor::Reply* reply) {
    if (!mPalHandle) {
        return ::android::OK;
    }

    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        int64_t frames = 0;
        std::get<CompressPlayback>(mExt).getPositionInFrames(&frames);
        LOG(VERBOSE) << __func__ << " dspFrames consumed:" << frames;
        const auto latencyMs = mPlatform.getBluetoothLatencyMs(mConnectedDevices);
        const auto sampleRate = mMixPortConfig.sampleRate.value().value;
        const auto offset = latencyMs * sampleRate / 1000;
        frames = (frames > offset) ? (frames - offset) : 0;
        reply->observable.frames = frames;
    } else if (mTag == Usecase::MMAP_PLAYBACK) {
        int64_t frames = 0;
        int64_t timeNs = 0;
        int32_t ret = 0;

        ret = std::get<MMapPlayback>(mExt).getMMapPosition(&frames, &timeNs);
        if (ret == 0) {
            reply->hardware.frames = frames;
            reply->hardware.timeNs = timeNs;
            LOG(VERBOSE) << __func__ << ": Returning MMAP position: frames "
                         << reply->hardware.frames << " timeNs " << reply->hardware.timeNs;
        } else {
            LOG(ERROR) << __func__ << ": getMmapPosition failed, ret= " << ret;
            return ::android::base::ERROR;
        }
    }

    return ::android::OK;
}

void StreamOutPrimary::shutdown() {
    if (mPalHandle != nullptr) {
        enableOffloadEffects(false);
        ::pal_stream_stop(mPalHandle);
        ::pal_stream_close(mPalHandle);
    }
    if (karaoke) mAudExt.mKarokeExtension->karaoke_stop();

    mIsPaused = false;
    mPalHandle = nullptr;
    LOG(VERBOSE) << __func__ << " " << mTagName;
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
        LOG(ERROR) << __func__ << ": invalid sample rate value: " << in_offloadMetadata.sampleRate;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_offloadMetadata.averageBitRatePerSecond < 0) {
        LOG(ERROR) << __func__
                   << ": invalid average BPS value: " << in_offloadMetadata.averageBitRatePerSecond;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_offloadMetadata.delayFrames < 0) {
        LOG(ERROR) << __func__
                   << ": invalid delay frames value: " << in_offloadMetadata.delayFrames;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_offloadMetadata.paddingFrames < 0) {
        LOG(ERROR) << __func__
                   << ": invalid padding frames value: " << in_offloadMetadata.paddingFrames;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (mTag != Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        LOG(WARNING) << __func__ << ": expected COMPRESS_OFFLOAD_PLAYBACK instead of " << mTagName;
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

ndk::ScopedAStatus StreamOutPrimary::setHwVolume(const std::vector<float>& in_channelVolumes) {
    if (!mPalHandle) {
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

ndk::ScopedAStatus StreamOutPrimary::updateMetadataCommon(const Metadata& metadata) {
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
        compressPlayback.updateSourceMetadata(std::get<SourceMetadata>(mMetadata));
    }
    int callState = mPlatform.getCallState();
    int callMode = mPlatform.getCallMode();
    bool voiceActive = ((callState == 2) || (callMode == 2));

    StreamOutPrimary::sourceMetadata_mutex_.lock();
    setAggregateSourceMetadata(voiceActive);
    StreamOutPrimary::sourceMetadata_mutex_.unlock();

    return ndk::ScopedAStatus::ok();
}

int32_t StreamOutPrimary::setAggregateSourceMetadata(bool voiceActive) {
    ssize_t track_count_total = 0;

    std::vector<playback_track_metadata_t> total_tracks;
    source_metadata_t btSourceMetadata;

    ModulePrimary::outListMutex.lock();
    std::vector<std::weak_ptr<StreamOut>>& outStreams = ModulePrimary::getOutStreams();
    if (voiceActive || outStreams.empty()) {
        ModulePrimary::outListMutex.unlock();
        return 0;
    }
    auto removeStreams = [&](std::weak_ptr<StreamOut> streamOut) -> bool {
        if (!streamOut.lock()) return true;
        return streamOut.lock()->isClosed();
    };
    outStreams.erase(std::remove_if(outStreams.begin(), outStreams.end(), removeStreams),
                     outStreams.end());

    LOG(DEBUG) << __func__ << " out streams not empty size is " << outStreams.size();

    for (auto it = outStreams.begin(); it < outStreams.end(); it++) {
        if (it->lock() && !it->lock()->isClosed()) {
            ::aidl::android::hardware::audio::common::SourceMetadata srcMetadata;
            it->lock()->getMetadata(srcMetadata);
            track_count_total += srcMetadata.tracks.size();
        } else {
        }
    }
    LOG(DEBUG) << __func__ << " out streams size after deleting : " << outStreams.size();
    LOG(DEBUG) << __func__ << " total tracks count is " << track_count_total;

    if (track_count_total <= 0) {
        ModulePrimary::outListMutex.unlock();
        return 0;
    }

    total_tracks.resize(track_count_total);
    btSourceMetadata.track_count = track_count_total;
    btSourceMetadata.tracks = total_tracks.data();

    for (auto it = outStreams.begin(); it != outStreams.end(); it++) {
        ::aidl::android::hardware::audio::common::SourceMetadata srcMetadata;
        if (it->lock()) {
            it->lock()->getMetadata(srcMetadata);
            for (auto& item : srcMetadata.tracks) {
                /* currently after cs call ends, we are getting metadata as
                * usage voice and content speech, this is causing BT to again
                * open call session, so added below check to send metadata of
                * voice only if call is active, else discard it
                */
                if (!voiceActive && (mPlatform.getCallMode() != 3) &&
                    (AUDIO_USAGE_VOICE_COMMUNICATION == static_cast<audio_usage_t>(item.usage)) &&
                    (AUDIO_CONTENT_TYPE_SPEECH ==
                     static_cast<audio_content_type_t>(item.contentType))) {
                    btSourceMetadata.track_count--;
                } else {
                    btSourceMetadata.tracks->usage = static_cast<audio_usage_t>(item.usage);
                    btSourceMetadata.tracks->content_type =
                            static_cast<audio_content_type_t>(item.contentType);
                    LOG(DEBUG) << __func__ << " source metadata usage is "
                               << btSourceMetadata.tracks->usage << " content is "
                               << btSourceMetadata.tracks->content_type;
                    ++btSourceMetadata.tracks;
                }
            }
        }
    }
    btSourceMetadata.tracks = total_tracks.data();
    LOG(DEBUG) << __func__ << " sending source metadata to PAL";
    pal_set_param(PAL_PARAM_ID_SET_SOURCE_METADATA, (void*)&btSourceMetadata, 0);
    LOG(DEBUG) << __func__ << " after sending source metadata to PAL";
    ModulePrimary::outListMutex.unlock();
    return 0;
}

ndk::ScopedAStatus StreamOutPrimary::getVendorParameters(
        const std::vector<std::string>& in_ids, std::vector<VendorParameter>* _aidl_return) {
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
        const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>& in_effect) {
    if (in_effect == nullptr) {
        LOG(DEBUG) << __func__ << ": null effect";
    } else {
        LOG(DEBUG) << __func__ << ": effect Binder" << in_effect->asBinder().get();
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus StreamOutPrimary::removeEffect(
        const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>& in_effect) {
    if (in_effect == nullptr) {
        LOG(DEBUG) << __func__ << ": null effect";
    } else {
        LOG(DEBUG) << __func__ << ": effect Binder" << in_effect->asBinder().get();
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
        return CompressPlayback::getPeriodBufferSize(mMixPortConfig.format.value());
    } else if (mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        return PcmOffloadPlayback::getPeriodSize(mMixPortConfig);
    } else if (mTag == Usecase::VOIP_PLAYBACK) {
        return VoipPlayback::getPeriodSize(mMixPortConfig);
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        return PrimaryPlayback::kPeriodSize * mFrameSizeBytes;
    } else if (mTag == Usecase::ULL_PLAYBACK) {
        return UllPlayback::getPeriodSize(mMixPortConfig.format.value(),
                                          mMixPortConfig.channelMask.value());
    } else if (mTag == Usecase::MMAP_PLAYBACK) {
        return MMapPlayback::getPeriodSize(mMixPortConfig.format.value(),
                                           mMixPortConfig.channelMask.value());
    } else if (mTag == Usecase::IN_CALL_MUSIC) {
        return InCallMusic::kPeriodSize;
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
    } else if (mTag == Usecase::ULL_PLAYBACK) {
        return UllPlayback::kPeriodCount;
    } else if (mTag == Usecase::MMAP_PLAYBACK) {
        return MMapPlayback::kPeriodCount;
    } else if (mTag == Usecase::IN_CALL_MUSIC) {
        return InCallMusic::kPeriodCount;
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
        attr->type = PAL_STREAM_VOIP_RX;
    } else if (mTag == Usecase::SPATIAL_PLAYBACK) {
        attr->type = PAL_STREAM_SPATIAL_AUDIO;
    } else if (mTag == Usecase::ULL_PLAYBACK) {
        attr->type = PAL_STREAM_ULTRA_LOW_LATENCY;
    } else if (mTag == Usecase::IN_CALL_MUSIC) {
        attr->type = PAL_STREAM_VOICE_CALL_MUSIC;
        attr->info.incall_music_info.local_playback = mPlatform.getInCallMusicState();
    } else {
        LOG(VERBOSE) << __func__ << " invalid usecase to configure";
        return;
    }

    LOG(VERBOSE) << __func__ << " assigned pal stream type:" << attr->type << " for " << mTagName;

    auto palDevices = mPlatform.getPalDevices(getConnectedDevices());
    if (!palDevices.size()) {
        LOG(ERROR) << __func__ << " no connected devices on stream!!";
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
        if (mOffloadInfo.value().bitWidth != 0) {
            attr->out_media_config.bit_width = mOffloadInfo.value().bitWidth;
        }
        attr->flags = static_cast<pal_stream_flags_t>(PAL_STREAM_FLAG_NON_BLOCKING);
    }
    if (mTag == Usecase::ULL_PLAYBACK) {
        attr->flags = static_cast<pal_stream_flags_t>(PAL_STREAM_FLAG_MMAP);
    }

    if (int32_t ret = ::pal_stream_open(attr.get(), palDevices.size(), palDevices.data(), 0,
                                        nullptr, palFn, cookie, &(this->mPalHandle));
        ret) {
        LOG(ERROR) << __func__ << " pal stream open failed!!! ret:" << ret;
        mPalHandle = nullptr;
        return;
    }
    if (karaoke) {
        int size = palDevices.size();
        mAudExt.mKarokeExtension->karaoke_open(palDevices[size - 1].id, palFn,
                                               attr.get()->out_media_config.ch_info);
    }
    const size_t ringBufSizeInBytes = getPeriodSize();
    const size_t ringBufCount = getPeriodCount();
    auto palBufferConfig = mPlatform.getPalBufferConfig(ringBufSizeInBytes, ringBufCount);
    LOG(VERBOSE) << __func__ << " pal stream set buffer size " << std::to_string(ringBufSizeInBytes)
                 << " with count " << std::to_string(ringBufCount);
    if (int32_t ret =
                ::pal_stream_set_buffer_size(this->mPalHandle, nullptr, palBufferConfig.get());
        ret) {
        LOG(ERROR) << __func__ << " pal stream set buffer size failed!!! ret:" << ret;
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
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
            LOG(ERROR) << __func__ << " pal stream set param failed!!! ret:" << ret;
            ::pal_stream_close(mPalHandle);
            mPalHandle = nullptr;
            return;
        }
        LOG(VERBOSE) << __func__ << " pal stream set param: "
                                    "PAL_PARAM_ID_CODEC_CONFIGURATION successful";
    }

    if (int32_t ret = ::pal_stream_start(this->mPalHandle); ret) {
        LOG(ERROR) << __func__ << " pal stream start failed!! ret:" << ret;
        ::pal_stream_close(mPalHandle);
        mPalHandle = nullptr;
        return;
    }

    if (karaoke) mAudExt.mKarokeExtension->karaoke_start();

    LOG(VERBOSE) << __func__ << " pal stream start successful";

    // configure mExt
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        std::get<CompressPlayback>(mExt).setPalHandle(mPalHandle);
    }

    LOG(INFO) << __func__ << ": stream is configured for " << mTagName;
    enableOffloadEffects(true);

    // after configuration operations

    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        // if any cached volume update
        mVolumes.size() > 0 ? (void)setHwVolume(mVolumes) : (void)0;
    }
}

void StreamOutPrimary::enableOffloadEffects(const bool enable) {
    if (mTag == Usecase::COMPRESS_OFFLOAD_PLAYBACK || mTag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        auto& ioHandle = mMixPortConfig.ext.get<AudioPortExt::Tag::mix>().handle;
        if (enable) {
            mHalEffects.startEffect(ioHandle, mPalHandle);
            LOG(VERBOSE) << __func__ << ": IOHandle: " << ioHandle;
        } else {
            mHalEffects.stopEffect(ioHandle);
        }
    }
}

} // namespace qti::audio::core
