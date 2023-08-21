/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define LOG_TAG "AHAL_Usecase"

#include <qti-audio-core/AudioUsecase.h>
#include <qti-audio-core/Platform.h>
#include <qti-audio-core/PlatformUtils.h>
#include <Utils.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <media/stagefright/foundation/MediaDefs.h>
#include <aidl/qti/audio/core/VString.h>

using ::aidl::android::hardware::audio::common::getChannelCount;
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

Usecase getUsecaseTag(
    const ::aidl::android::media::audio::common::AudioPortConfig&
        mixPortConfig) {
    Usecase tag = Usecase::INVALID;
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
            tag = Usecase::PRIMARY_PLAYBACK;
        } else if (outFlags == deepBufferPlaybackFlags ) {
            tag = Usecase::DEEP_BUFFER_PLAYBACK;
        } else if (outFlags == lowLatencyPlaybackFlags) {
            tag = Usecase::LOW_LATENCY_PLAYBACK;
        } else if (outFlags == compressOffloadPlaybackFlags) {
            tag = Usecase::COMPRESS_OFFLOAD_PLAYBACK;
        } else if (outFlags == pcmOffloadPlaybackFlags) {
            tag = Usecase::PCM_OFFLOAD_PLAYBACK;
        } else if (outFlags == voipPlaybackFlags) {
            tag = Usecase::VOIP_PLAYBACK;
        } else if (outFlags == spatialPlaybackFlags) {
            tag = Usecase::SPATIAL_PLAYBACK;
        }
    } else if (flagsTag == AudioIoFlags::Tag::input) {
        auto& inFlags =
            mixPortConfig.flags.value().get<AudioIoFlags::Tag::input>();
        if (inFlags == 0) {
            tag = Usecase::PCM_RECORD;
        } else if (inFlags == compressCaptureFlags) {
            tag = Usecase::COMPRESS_CAPTURE;
        } else if (inFlags == recordVoipFlags &&
                   mixUsecaseTag == AudioPortMixExtUseCase::source &&
                   mixUsecase.get<AudioPortMixExtUseCase::source>() ==
                       AudioSource::VOICE_COMMUNICATION) {
            tag = Usecase::VOIP_RECORD;
        }
    }
    LOG(VERBOSE) << __func__ << " choosen tag:" << getName(tag)
              << " for mix port config " << mixPortConfig.toString();
    return tag;
}

std::string getName(const Usecase tag) {
    switch (tag) {
        case Usecase::INVALID:
            return "INVALID";
        case Usecase::PRIMARY_PLAYBACK:
            return "PRIMARY_PLAYBACK";
        case Usecase::DEEP_BUFFER_PLAYBACK:
            return "DEEP_BUFFER_PLAYBACK";
        case Usecase::LOW_LATENCY_PLAYBACK:
            return "LOW_LATENCY_PLAYBACK";
        case Usecase::PCM_RECORD:
            return "PCM_RECORD";
        case Usecase::COMPRESS_OFFLOAD_PLAYBACK:
            return "COMPRESS_OFFLOAD_PLAYBACK";
        case Usecase::COMPRESS_CAPTURE:
            return "COMPRESS_CAPTURE";
        case Usecase::PCM_OFFLOAD_PLAYBACK:
            return "PCM_OFFLOAD_PLAYBACK";
        case Usecase::VOIP_PLAYBACK:
            return "VOIP_PLAYBACK";
        case Usecase::SPATIAL_PLAYBACK:
            return "SPATIAL_PLAYBACK";
        case Usecase::VOIP_RECORD:
            return "VOIP_RECORD";
        default:
            return std::to_string(static_cast<uint16_t>(tag));
    }
}

PcmRecord::HdrMode PcmRecord::getHdrMode() {
    const auto& platform = Platform::getInstance();
    const std::string kHdrSpfProperty{"vendor.audio.hdr.spf.record.enable"};
    const bool isSPFEnabled =
        ::android::base::GetBoolProperty(kHdrSpfProperty, false);
    if (isSPFEnabled) {
        return HdrMode::SPF;
    }
    const std::string kHdrArmProperty{"vendor.audio.hdr.record.enable"};
    const bool isArmEnabled =
        ::android::base::GetBoolProperty(kHdrArmProperty, false);
    const bool isHdrSetOnPlatform =
        platform.getParameter("hdr_record_on") == "true" ? true : false;
    if (isArmEnabled && isHdrSetOnPlatform) {
        return HdrMode::ARM;
    }
    return HdrMode::NONE;
}

void PcmRecord::setHdrOnPalDevice(pal_device* palDeviceIn) {
    const auto& platform = Platform::getInstance();
    const bool isOrientationLandscape =
        platform.getParameter("orientation") == "landscape" ? true : false;
    const bool isInverted =
        platform.getParameter("inverted") == "true" ? true : false;
    if (isOrientationLandscape && !isInverted) {
        strlcpy(palDeviceIn->custom_config.custom_key,
                "unprocessed-hdr-mic-landscape",
                sizeof(palDeviceIn->custom_config.custom_key));
    } else if (!isOrientationLandscape && !isInverted) {
        strlcpy(palDeviceIn->custom_config.custom_key,
                "unprocessed-hdr-mic-portrait",
                sizeof(palDeviceIn->custom_config.custom_key));
    } else if (isOrientationLandscape && isInverted) {
        strlcpy(palDeviceIn->custom_config.custom_key,
                "unprocessed-hdr-mic-inverted-landscape",
                sizeof(palDeviceIn->custom_config.custom_key));
    } else if (!isOrientationLandscape && isInverted) {
        strlcpy(palDeviceIn->custom_config.custom_key,
                "unprocessed-hdr-mic-inverted-portrait",
                sizeof(palDeviceIn->custom_config.custom_key));
    }
    LOG(DEBUG) << __func__ << " setting custom config:"
               << std::string(palDeviceIn->custom_config.custom_key);
}

void PcmRecord::configurePalDevices(
    const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig,
    std::vector<pal_device>& palDevices) {
    const auto& mixUsecase =
        mixPortConfig.ext
            .get<
                ::aidl::android::media::audio::common::AudioPortExt::Tag::mix>()
            .usecase;
    if (mixUsecase.getTag() != ::aidl::android::media::audio::common::
                                   AudioPortMixExtUseCase::Tag::source) {
        LOG(ERROR) << __func__
                   << " expected mix usecase as source instead found, "
                   << mixUsecase.toString();
        return;
    }
    const auto& sampleRate = mixPortConfig.sampleRate.value().value;
    const auto& channelLayout = mixPortConfig.channelMask.value();
    const ::aidl::android::media::audio::common::AudioSource& audioSourceType =
        mixUsecase.get<::aidl::android::media::audio::common::
                           AudioPortMixExtUseCase::Tag::source>();
    const bool isSourceUnprocessed =
        audioSourceType ==
        ::aidl::android::media::audio::common::AudioSource::UNPROCESSED;
    const bool isSourceCamCorder =
        audioSourceType ==
        ::aidl::android::media::audio::common::AudioSource::CAMCORDER;
    const bool isMic = audioSourceType ==
                       ::aidl::android::media::audio::common::AudioSource::MIC;
    const HdrMode hdrMode = getHdrMode();
    if ((isSourceUnprocessed && sampleRate == 48000 &&
         getChannelCount(channelLayout) == 4 && hdrMode == HdrMode::ARM) ||
        (hdrMode == HdrMode::ARM) ||
        (hdrMode == HdrMode::SPF && (isSourceCamCorder || isMic))) {
        std::for_each(
            palDevices.begin(), palDevices.end(),
            [&](auto& palDevice) { this->setHdrOnPalDevice(&palDevice); });
    }
}

// start of compress playback
CompressPlayback::CompressPlayback(
    const ::aidl::android::media::audio::common::AudioOffloadInfo& offloadInfo,
    std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback>
        asyncCallback)
    : mOffloadInfo(offloadInfo), mAsyncCallback(asyncCallback) {
    configureDefault();
}

void CompressPlayback::configureDefault() {

    mSampleRate = mOffloadInfo.base.sampleRate;
    mCompressFormat = mOffloadInfo.base.format;
    mChannelLayout = mOffloadInfo.base.channelMask;
    mBitWidth = mOffloadInfo.bitWidth;

    if (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_MP4 ||
        mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADIF ||
        mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS ||
        mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC ||
        mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC) {
        mPalSndDec.aac_dec.audio_obj_type = 29;
        mPalSndDec.aac_dec.pce_bits_size = 0;
    }

    LOG(INFO) << __func__ << ": " << mOffloadInfo.toString();
    return;
}

void CompressPlayback::setPalHandle(pal_stream_handle_t* handle) {
    mCompressPlaybackHandle = handle;
}

ndk::ScopedAStatus CompressPlayback::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    return ndk::ScopedAStatus::ok();
}

// static
int32_t CompressPlayback::palCallback(pal_stream_handle_t* palHandle,
                                      uint32_t eventId, uint32_t* eventData,
                                      uint32_t eventSize, uint64_t cookie) {

    auto compressPlayback = reinterpret_cast<CompressPlayback*>(cookie);

    switch (eventId) {
        case PAL_STREAM_CBK_EVENT_WRITE_READY: {
            LOG(VERBOSE) << __func__ << " ready to write";
            compressPlayback->mAsyncCallback->onTransferReady();
        } break;

        case PAL_STREAM_CBK_EVENT_DRAIN_READY: {
            LOG(VERBOSE) << __func__ << " drain ready";
            compressPlayback->mAsyncCallback->onDrainReady();
        } break;
        case PAL_STREAM_CBK_EVENT_PARTIAL_DRAIN_READY: {
            LOG(VERBOSE) << __func__ << " partial drain ready";
            compressPlayback->mAsyncCallback->onDrainReady();
        } break;
        case PAL_STREAM_CBK_EVENT_ERROR:
            LOG(ERROR) << __func__ << " error!!!";
            compressPlayback->mAsyncCallback->onError();
            break;
        default:
            LOG(ERROR) << __func__ << " invalid!!! event id:" << eventId;
            return -EINVAL;
    }
    return 0;
}

auto getIntValueFromVString =
    [](const std::vector<
           ::aidl::android::hardware::audio::core::VendorParameter>& parameters,
       const std::string& searchKey) -> std::optional<int32_t> {
    std::optional<::aidl::qti::audio::core::VString> parcel;
    for (const auto& p : parameters) {
        if (p.id == searchKey && p.ext.getParcelable(&parcel) == ::android::OK &&
            parcel.has_value()) {
            int32_t value = strtol(parcel.value().value.c_str(), nullptr, 10);
            return value;
        }
    }
    return std::nullopt;
};

ndk::ScopedAStatus CompressPlayback::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    LOG(VERBOSE) << __func__ << ": parameter count" << in_parameters.size()
                 << " parsing for " << mCompressFormat.encoding;
    if (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_FLAC) {
        if (auto value = getIntValueFromVString(in_parameters,Flac::kMinBlockSize); value) {
            mPalSndDec.flac_dec.min_blk_size = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Flac::kMaxBlockSize); value) {
            mPalSndDec.flac_dec.max_blk_size = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Flac::kMinFrameSize); value) {
            mPalSndDec.flac_dec.min_frame_size = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Flac::kMaxFrameSize); value) {
            mPalSndDec.flac_dec.max_frame_size = value.value();
        }
        // exception
        auto bitWidth = mCompressBitWidth == 32 ? 24 : mCompressBitWidth;
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_ALAC) {
        if (auto value = getIntValueFromVString(in_parameters,Alac::kFrameLength); value) {
            mPalSndDec.alac_dec.frame_length = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kCompatVer); value) {
            mPalSndDec.alac_dec.compatible_version = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kBitDepth); value) {
            mPalSndDec.alac_dec.bit_depth = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kPb); value) {
            mPalSndDec.alac_dec.pb = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kMb); value) {
            mPalSndDec.alac_dec.mb = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kKb); value) {
            mPalSndDec.alac_dec.kb = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kNumChannels); value) {
            mPalSndDec.alac_dec.num_channels = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kMaxRun); value) {
            mPalSndDec.alac_dec.max_run = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kMaxFrameBytes); value) {
            mPalSndDec.alac_dec.max_frame_bytes = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kBitRate); value) {
            mPalSndDec.alac_dec.avg_bit_rate = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kSamplingRate); value) {
            mPalSndDec.alac_dec.sample_rate = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Alac::kChannelLayoutTag); value) {
            mPalSndDec.alac_dec.channel_layout_tag = value.value();
        }
    } else if (mCompressFormat.encoding ==
                   ::android::MEDIA_MIMETYPE_AUDIO_AAC_MP4 ||
               mCompressFormat.encoding ==
                   ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADIF ||
               mCompressFormat.encoding ==
                   ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS ||
               mCompressFormat.encoding ==
                   ::android::MEDIA_MIMETYPE_AUDIO_AAC) {
        mPalSndDec.aac_dec.audio_obj_type = 29;
        mPalSndDec.aac_dec.pce_bits_size = 0;
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_VORBIS) {
        if (auto value = getIntValueFromVString(in_parameters,Vorbis::kBitStreamFormat); value) {
            mPalSndDec.vorbis_dec.bit_stream_fmt = value.value();
        }
    } else if (mCompressFormat.encoding == "audio/x-ape") {
        if (auto value = getIntValueFromVString(in_parameters,Ape::kCompatibleVersion); value) {
            mPalSndDec.ape_dec.compatible_version = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kCompressionLevel); value) {
            mPalSndDec.ape_dec.compression_level = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kFormatFlags); value) {
            mPalSndDec.ape_dec.format_flags = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kBlocksPerFrame); value) {
            mPalSndDec.ape_dec.blocks_per_frame = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kFinalFrameBlocks); value) {
            mPalSndDec.ape_dec.final_frame_blocks = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kTotalFrames); value) {
            mPalSndDec.ape_dec.total_frames = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kBitsPerSample); value) {
            mPalSndDec.ape_dec.bits_per_sample = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kNumChannels); value) {
            mPalSndDec.ape_dec.num_channels = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kSampleRate); value) {
            mPalSndDec.ape_dec.sample_rate = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Ape::kSeekTablePresent); value) {
            mPalSndDec.ape_dec.seek_table_present = value.value();
        }
    } else if (mCompressFormat.encoding ==
                   ::android::MEDIA_MIMETYPE_AUDIO_WMA ||
               mCompressFormat.encoding == "audio/x-ms-wma.pro") {
        if (auto value = getIntValueFromVString(in_parameters,Wma::kFormatTag); value) {
            mPalSndDec.wma_dec.fmt_tag = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,kAvgBitRate); value) {
            mPalSndDec.wma_dec.avg_bit_rate = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Wma::kBlockAlign); value) {
            mPalSndDec.wma_dec.super_block_align = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Wma::kBitPerSample); value) {
            mPalSndDec.wma_dec.bits_per_sample = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Wma::kChannelMask); value) {
            mPalSndDec.wma_dec.channelmask = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Wma::kEncodeOption); value) {
            mPalSndDec.wma_dec.encodeopt = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Wma::kEncodeOption1); value) {
            mPalSndDec.wma_dec.encodeopt1 = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Wma::kEncodeOption2); value) {
            mPalSndDec.wma_dec.encodeopt2 = value.value();
        }
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_OPUS) {
        if (auto value = getIntValueFromVString(in_parameters,Opus::kBitStreamFormat); value) {
            mPalSndDec.opus_dec.bitstream_format = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kPayloadType); value) {
            mPalSndDec.opus_dec.payload_type = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kVersion); value) {
            mPalSndDec.opus_dec.version = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kNumChannels); value) {
            mPalSndDec.opus_dec.num_channels = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kPreSkip); value) {
            mPalSndDec.opus_dec.pre_skip = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kOutputGain); value) {
            mPalSndDec.opus_dec.output_gain = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kMappingFamily); value) {
            mPalSndDec.opus_dec.mapping_family = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kStreamCount); value) {
            mPalSndDec.opus_dec.stream_count = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kCoupledCount); value) {
            mPalSndDec.opus_dec.coupled_count = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap0); value) {
            mPalSndDec.opus_dec.channel_map[0] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap1); value) {
            mPalSndDec.opus_dec.channel_map[1] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap2); value) {
            mPalSndDec.opus_dec.channel_map[2] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap3); value) {
            mPalSndDec.opus_dec.channel_map[3] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap4); value) {
            mPalSndDec.opus_dec.channel_map[4] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap5); value) {
            mPalSndDec.opus_dec.channel_map[5] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap6); value) {
            mPalSndDec.opus_dec.channel_map[6] = value.value();
        }
        if (auto value = getIntValueFromVString(in_parameters,Opus::kChannelMap7); value) {
            mPalSndDec.opus_dec.channel_map[7] = value.value();
        }
        mPalSndDec.opus_dec.sample_rate = mSampleRate;
    }
    return ndk::ScopedAStatus::ok();
}

void CompressPlayback::updateOffloadMetadata(
    const ::aidl::android::hardware::audio::common::AudioOffloadMetadata&
        offloadMetaData) {
    mOffloadMetadata = &offloadMetaData;
    mSampleRate = mOffloadMetadata->sampleRate;
    mChannelLayout = mOffloadMetadata->channelMask;
    // TODO check for any pal update
    LOG(INFO) << __func__ << ": " << mOffloadMetadata->toString();
    return;
}

void CompressPlayback::updateSourceMetadata(
    const ::aidl::android::hardware::audio::common::SourceMetadata&
        sourceMetaData) {
    mSourceMetadata = &sourceMetaData;
    // TODO check for any pal update
    LOG(INFO) << __func__ << ": " << mSourceMetadata->toString();
    return;
}

std::unique_ptr<uint8_t[]> CompressPlayback::getPayloadCodecInfo() {
    auto dataPtr = std::make_unique<uint8_t[]>(sizeof(pal_param_payload) +
                                               sizeof(pal_snd_dec_t));
    auto palParamPayload = reinterpret_cast<pal_param_payload*>(dataPtr.get());
    palParamPayload->payload_size = sizeof(pal_snd_dec_t);
    memcpy(palParamPayload->payload, &mPalSndDec, sizeof(pal_snd_dec_t));
    return std::move(dataPtr);
}

// static
size_t CompressPlayback::getPeriodBufferSize(
    const ::aidl::android::media::audio::common::AudioFormatDescription&
        format) {
    size_t periodSize = CompressPlayback::kPeriodSize;
    if (format.encoding == ::android::MEDIA_MIMETYPE_AUDIO_FLAC) {
        periodSize = Flac::kPeriodSize;
    }

    const std::string kCompressPeriodSizeProp{
        "vendor.audio.offload.buffer.size.kb"};
    auto propPeriodSize = ::android::base::GetUintProperty<size_t>(
        kCompressPeriodSizeProp, CompressPlayback::kPeriodSize);
    if (propPeriodSize > periodSize) {
        periodSize = propPeriodSize;
    }
    return periodSize;
}

void CompressPlayback::getPositionInFrames(int64_t* dspFrames) {
    pal_session_time tstamp;
    if (int32_t ret = pal_get_timestamp(mCompressPlaybackHandle, &tstamp);
        ret) {
        LOG(ERROR) << __func__ << " pal_get_timestamp failure, ret:" << ret;
        return;
    }

    uint64_t sessionTimeUs =
        ((static_cast<decltype(sessionTimeUs)>(tstamp.session_time.value_msw))
             << 32 |
         tstamp.session_time.value_lsw);
    // sessionTimeUs to frames
    *dspFrames =
        static_cast<int64_t>(sessionTimeUs / 1000 * mSampleRate / 1000);
    return;
}

// end of compress playback

// start of compress capture
CompressCapture::CompressCapture(
    const ::aidl::android::media::audio::common::AudioFormatDescription& format,
    const int32_t sampleRate,
    const ::aidl::android::media::audio::common::AudioChannelLayout&
        channelLayout)
    : mCompressFormat(format),
      mSampleRate(sampleRate),
      mChannelLayout(channelLayout) {
    if (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC ||
        mCompressFormat.encoding ==
            ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_LC) {
        mPalSndEnc.aac_enc.enc_cfg.aac_enc_mode = Aac::EncodingMode::LC;
        mPalSndEnc.aac_enc.enc_cfg.aac_fmt_flag = Aac::EncodingFormat::ADTS;
        mPalSndEnc.aac_enc.aac_bit_rate = Aac::kAacDefaultBitrate;
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V1) {
        mPalSndEnc.aac_enc.enc_cfg.aac_enc_mode = Aac::EncodingMode::SBR;
        mPalSndEnc.aac_enc.enc_cfg.aac_fmt_flag = Aac::EncodingFormat::ADTS;
        mPalSndEnc.aac_enc.aac_bit_rate = Aac::kAacDefaultBitrate;
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V2) {
        mPalSndEnc.aac_enc.enc_cfg.aac_enc_mode = Aac::EncodingMode::PS;
        mPalSndEnc.aac_enc.enc_cfg.aac_fmt_flag = Aac::EncodingFormat::ADTS;
        mPalSndEnc.aac_enc.aac_bit_rate = Aac::kAacDefaultBitrate;
    }
    mPCMSamplesPerFrame =
        (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC ||
         mCompressFormat.encoding ==
             ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_LC)
            ? Aac::kAacLcPCMSamplesPerFrame
            : Aac::kHeAacPCMSamplesPerFrame;
}

void CompressCapture::setPalHandle(pal_stream_handle_t* handle) {
    mCompressHandle = handle;
}

std::unique_ptr<uint8_t[]> CompressCapture::getPayloadCodecInfo() const {
    auto dataPtr = std::make_unique<uint8_t[]>(sizeof(pal_param_payload) +
                                               sizeof(pal_snd_enc_t));
    auto palParamPayload = reinterpret_cast<pal_param_payload*>(dataPtr.get());
    palParamPayload->payload_size = sizeof(pal_snd_enc_t);
    memcpy(palParamPayload->payload, &mPalSndEnc, sizeof(pal_snd_enc_t));
    return std::move(dataPtr);
}

ndk::ScopedAStatus CompressCapture::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    LOG(VERBOSE) << __func__ << " parsing for " << mCompressFormat.encoding;
    if (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC ||
        mCompressFormat.encoding ==
            ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V1 ||
        mCompressFormat.encoding ==
            ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V2 ||
        mCompressFormat.encoding ==
            ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_LC) {
        if (auto value = getIntValueFromVString(in_parameters,Aac::kDSPAacBitRate); value) {
            auto requested = value.value();
            const auto min = getAACMinBitrateValue();
            const auto max = getAACMaxBitrateValue();
            mPalSndEnc.aac_enc.aac_bit_rate =
                requested < min ? min : (requested > max ? max : requested);
            mCompressHandle != nullptr ? (void)setAACDSPBitRate() : (void)0;
        }
        if (auto value = getIntValueFromVString(in_parameters,Aac::kDSPAacGlobalCutoffFrequency);
            value) {
            if (mCompressFormat.encoding ==
                ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC) {
                mPalSndEnc.aac_enc.global_cutoff_freq = value.value();
            }
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus CompressCapture::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    return ndk::ScopedAStatus::ok();
}

void CompressCapture::setAACDSPBitRate() {
    const auto palSndEncSize = sizeof(pal_snd_enc_t);
    auto payload =
        std::make_unique<uint8_t[]>(sizeof(pal_param_payload) + palSndEncSize);
    auto paramPayload = (pal_param_payload*)payload.get();
    paramPayload->payload_size = palSndEncSize;
    memcpy(paramPayload->payload, &mPalSndEnc, paramPayload->payload_size);

    if (int32_t ret = ::pal_stream_set_param(
            mCompressHandle, PAL_PARAM_ID_RECONFIG_ENCODER, paramPayload);
        ret) {
        LOG(ERROR) << __func__
                   << "pal set param PAL_PARAM_ID_RECONFIG_ENCODER failed:"
                   << ret;
    }
}

int32_t CompressCapture::getAACMinBitrateValue() {
    const auto channelCount =
        ::aidl::android::hardware::audio::common::getChannelCount(
            mChannelLayout);
    if (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC ||
        mCompressFormat.encoding ==
            ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_LC) {
        if (channelCount == 1) {
            return Aac::kAacLcMonoMinSupportedBitRate;
        } else {
            return Aac::kAacLcStereoMinSupportedBitRate;
        }
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V1) {
        if (channelCount == 1) {
            return (mSampleRate == 24000 || mSampleRate == 32000)
                       ? Aac::kHeAacMonoMinSupportedBitRate1
                       : Aac::kHeAacMonoMinSupportedBitRate2;
        } else {
            return (mSampleRate == 24000 || mSampleRate == 32000)
                       ? Aac::kHeAacStereoMinSupportedBitRate1
                       : Aac::kHeAacStereoMinSupportedBitRate2;
        }
    } else {
        // AUDIO_FORMAT_AAC_ADTS_HE_V2
        return (mSampleRate == 24000 || mSampleRate == 32000)
                   ? Aac::kHeAacPsStereoMinSupportedBitRate1
                   : Aac::kHeAacPsStereoMinSupportedBitRate2;
    }
}

int32_t CompressCapture::getAACMaxBitrateValue() {
    const auto channelCount =
        ::aidl::android::hardware::audio::common::getChannelCount(
            mChannelLayout);
    if (mCompressFormat.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC ||
        mCompressFormat.encoding ==
            ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_LC) {
        if (channelCount == 1) {
            return std::min(Aac::kAacLcMonoMaxSupportedBitRate,
                            6 * mSampleRate);
        } else {
            return std::min(Aac::kAacLcStereoMaxSupportedBitRate,
                            12 * mSampleRate);
        }
    } else if (mCompressFormat.encoding ==
               ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V1) {
        if (channelCount == 1) {
            return std::min(Aac::kHeAacMonoMaxSupportedBitRate,
                            6 * mSampleRate);
        } else {
            return std::min(Aac::kHeAacStereoMaxSupportedBitRate,
                            12 * mSampleRate);
        }
    } else {
        // AUDIO_FORMAT_AAC_ADTS_HE_V2
        return std::min(Aac::kHeAacPstereoMaxSupportedBitRate, 6 * mSampleRate);
    }
}

uint32_t CompressCapture::getAACMaxBufferSize() {
    int32_t maxBitRate = getAACMaxBitrateValue();
    /**
     * AAC Encoder 1024 PCM samples => 1 compress AAC frame;
     * 1 compress AAC frame => max possible length => max-bitrate bits;
     * let's take example of 48K HZ;
     * 1 second ==> 384000 bits ; 1 second ==> 48000 PCM samples;
     * 1 AAC frame ==> 1024 PCM samples;
     * Max buffer size possible;
     * 48000/1024 = (8/375) seconds ==> ( 8/375 ) * 384000 bits
     *     ==> ( (8/375) * 384000 / 8 ) bytes;
     **/
    return (uint32_t)((((((double)mPCMSamplesPerFrame) / mSampleRate) *
                        ((uint32_t)(maxBitRate))) /
                       8) +
                      /* Just in case; not to miss precision */ 1);
}

// static
size_t CompressCapture::getPeriodBufferSize(
    const ::aidl::android::media::audio::common::AudioFormatDescription&
        format) {
    if (format.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_LC ||
        format.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_LC ||
        format.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V1 ||
        format.encoding == ::android::MEDIA_MIMETYPE_AUDIO_AAC_ADTS_HE_V2) {
        return Aac::KAacMaxOutputSize;
    }
    return 0;
}

// end of compress capture


//start of PcmOffloadPlayback

//static
size_t PcmOffloadPlayback::getPeriodSize(
    const ::aidl::android::media::audio::common::AudioFormatDescription&
        formatDescription,
    const ::aidl::android::media::audio::common::AudioChannelLayout&
        channelLayout,
    const int32_t sampleRate) {
    const auto frameSize =
        ::aidl::android::hardware::audio::common::getFrameSizeInBytes(
            formatDescription, channelLayout);
    size_t periodSize = sampleRate * (kPeriodDurationMs / 1000) * frameSize;
    periodSize = getNearestMultiple(
        periodSize,
        std::lcm(
            32,
            ::aidl::android::hardware::audio::common::getPcmSampleSizeInBytes(
                formatDescription.pcm)));
    return periodSize;
}

// end of PcmOffloadPlayback

//start of VoipPlayback

size_t VoipPlayback::getPeriodSize(
    const ::aidl::android::media::audio::common::AudioFormatDescription&
        formatDescription,
    const ::aidl::android::media::audio::common::AudioChannelLayout&
        channelLayout,
    const int32_t sampleRate) {
    const auto frameSize =
        ::aidl::android::hardware::audio::common::getFrameSizeInBytes(
            formatDescription, channelLayout);
    size_t periodSize =
        sampleRate * (VoipPlayback::kBufferDurationMs / 1000) * frameSize;
    return periodSize;
}

// end of VoipPlayback

}  // namespace qti::audio::core