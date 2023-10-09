/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <PalDefs.h>
#include <aidl/android/hardware/audio/common/SinkMetadata.h>
#include <aidl/android/hardware/audio/common/SourceMetadata.h>
#include <aidl/android/hardware/audio/core/IStreamCallback.h>
#include <aidl/android/hardware/audio/core/IStreamOutEventCallback.h>
#include <aidl/android/hardware/audio/core/VendorParameter.h>
#include <aidl/android/media/audio/common/AudioChannelLayout.h>
#include <aidl/android/media/audio/common/AudioDevice.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioOffloadInfo.h>
#include <aidl/android/hardware/audio/common/AudioOffloadMetadata.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>
#include <aidl/android/media/audio/common/Int.h>
#include <android/binder_auto_utils.h>

#include <algorithm>
#include <numeric>

namespace qti::audio::core {

enum class Usecase : uint16_t {
    INVALID = 0,
    PRIMARY_PLAYBACK,
    DEEP_BUFFER_PLAYBACK,
    LOW_LATENCY_PLAYBACK,
    PCM_RECORD,
    COMPRESS_OFFLOAD_PLAYBACK,
    COMPRESS_CAPTURE,
    PCM_OFFLOAD_PLAYBACK,
    VOIP_PLAYBACK,
    SPATIAL_PLAYBACK,
    VOIP_RECORD,
    ULL_PLAYBACK,
    MMAP_PLAYBACK,
    MMAP_RECORD,
    VOICE_CALL_RECORD,
    IN_CALL_MUSIC,
};

Usecase getUsecaseTag(
    const ::aidl::android::media::audio::common::AudioPortConfig&
        mixPortConfig);

std::string getName(const Usecase tag);

/**
 * This port is opened by default and receives routing, audio mode and volume
 * controls related to voice calls
 **/
class PrimaryPlayback final {
   public:
    constexpr static size_t kPeriodSize = 1920;
    constexpr static size_t kPeriodCount = 2;
};

class DeepBufferPlayback final {
   public:
    constexpr static size_t kPeriodSize = 1920;
    constexpr static size_t kPeriodCount = 2;
};

class LowLatencyPlayback final {
   public:
    constexpr static size_t kPeriodSize = 240;
    constexpr static size_t kPeriodCount = 2;
};

class PcmRecord final {
   public:
    constexpr static uint32_t kCaptureDurationMs = 20;
    constexpr static uint32_t kPeriodCount = 4;
    constexpr static size_t kFMQMinFrameSize = 256;
    static size_t getMinFrames(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig);
    enum class HdrMode : uint8_t {
        NONE = 0,
        ARM,
        SPF,
    };

    void configurePalDevices(
    const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig,
    std::vector<pal_device>& palDevices);
    void setHdrOnPalDevice(pal_device* palDeviceIn);
    HdrMode getHdrMode();
};

class CompressPlayback final{
    public:
     static constexpr size_t kPeriodSize = 32 * 1024;
     static constexpr size_t kPeriodCount = 4;
     inline const static std::string kAvgBitRate{"music_offload_avg_bit_rate"};
     class Flac final{
        public:
        static constexpr size_t kPeriodSize = 256 * 1024;
         inline const static std::string kMinBlockSize{
             "music_offload_flac_min_blk_size"};
         inline const static std::string kMaxBlockSize{
             "music_offload_flac_max_blk_size"};
         inline const static std::string kMinFrameSize{
             "music_offload_flac_min_frame_size"};
         inline const static std::string kMaxFrameSize{
             "music_offload_flac_max_frame_size"};
     };
     class Alac final {
        public:
         inline const static std::string kFrameLength{
             "music_offload_alac_frame_length"};
         inline const static std::string kCompatVer{
             "music_offload_alac_compatible_version"};
         inline const static std::string kBitDepth{
             "music_offload_alac_bit_depth"};
         inline const static std::string kPb{"music_offload_alac_pb"};
         inline const static std::string kMb{"music_offload_alac_mb"};
         inline const static std::string kKb{"music_offload_alac_kb"};
         inline const static std::string kNumChannels{
             "music_offload_alac_num_channels"};
         inline const static std::string kMaxRun{"music_offload_alac_max_run"};
         inline const static std::string kMaxFrameBytes{
             "music_offload_alac_max_frame_bytes"};
         inline const static std::string kBitRate{
             "music_offload_alac_avg_bit_rate"};
         inline const static std::string kSamplingRate{
             "music_offload_alac_sampling_rate"};
         inline const static std::string kChannelLayoutTag{
             "music_offload_alac_channel_layout_tag"};
     };
     class Vorbis final {
        public:
         inline const static std::string kBitStreamFormat{
             "music_offload_vorbis_bitstream_fmt"};
     };
     class Ape final {
        public:
         inline const static std::string kCompatibleVersion{
             "music_offload_ape_compatible_version"};
         inline const static std::string kCompressionLevel{
             "music_offload_ape_compression_level"};
         inline const static std::string kFormatFlags{
             "music_offload_ape_format_flags"};
         inline const static std::string kBlocksPerFrame{
             "music_offload_ape_blocks_per_frame"};
         inline const static std::string kFinalFrameBlocks{
             "music_offload_ape_final_frame_blocks"};
         inline const static std::string kTotalFrames{
             "music_offload_ape_total_frames"};
         inline const static std::string kBitsPerSample{
             "music_offload_ape_bits_per_sample"};
         inline const static std::string kNumChannels{
             "music_offload_ape_num_channels"};
         inline const static std::string kSampleRate{
             "music_offload_ape_sample_rate"};
         inline const static std::string kSeekTablePresent{
             "music_offload_seek_table_present"};
     };
     class Wma final {
        public:
         inline const static std::string kFormatTag{
             "music_offload_wma_format_tag"};
         inline const static std::string kBlockAlign{
             "music_offload_wma_block_align"};
         inline const static std::string kBitPerSample{
             "music_offload_wma_bit_per_sample"};
         inline const static std::string kChannelMask{
             "music_offload_wma_channel_mask"};
         inline const static std::string kEncodeOption{
             "music_offload_wma_encode_option"};
         inline const static std::string kEncodeOption1{
             "music_offload_wma_encode_option1"};
         inline const static std::string kEncodeOption2{
             "music_offload_wma_encode_option2"};
     };
     class Opus final {
        public:
         inline const static std::string kBitStreamFormat{
             "music_offload_opus_bitstream_format"};
         inline const static std::string kPayloadType{
             "music_offload_opus_payload_type"};
         inline const static std::string kVersion{"music_offload_opus_version"};
         inline const static std::string kNumChannels{
             "music_offload_opus_num_channels"};
         inline const static std::string kPreSkip{
             "music_offload_opus_pre_skip"};
         inline const static std::string kOutputGain{
             "music_offload_opus_output_gain"};
         inline const static std::string kMappingFamily{
             "music_offload_opus_mapping_family"};
         inline const static std::string kStreamCount{
             "music_offload_opus_stream_count"};
         inline const static std::string kCoupledCount{
             "music_offload_opus_coupled_count"};
         inline const static std::string kChannelMap0{
             "music_offload_opus_channel_map0"};
         inline const static std::string kChannelMap1{
             "music_offload_opus_channel_map1"};
         inline const static std::string kChannelMap2{
             "music_offload_opus_channel_map2"};
         inline const static std::string kChannelMap3{
             "music_offload_opus_channel_map3"};
         inline const static std::string kChannelMap4{
             "music_offload_opus_channel_map4"};
         inline const static std::string kChannelMap5{
             "music_offload_opus_channel_map5"};
         inline const static std::string kChannelMap6{
             "music_offload_opus_channel_map6"};
         inline const static std::string kChannelMap7{
             "music_offload_opus_channel_map7"};
     };

     static int32_t palCallback(pal_stream_handle_t* palHandle,
                                uint32_t eventId, uint32_t* eventData,
                                uint32_t eventSize, uint64_t cookie);
     static size_t getPeriodBufferSize(
         const ::aidl::android::media::audio::common::AudioFormatDescription&
             format);
     explicit CompressPlayback(
         const ::aidl::android::media::audio::common::AudioOffloadInfo&
             offloadInfo,
         std::shared_ptr<
             ::aidl::android::hardware::audio::core::IStreamCallback>
             asyncCallback);
     void configureDefault();
     void setPalHandle(pal_stream_handle_t* handle);
     ndk::ScopedAStatus getVendorParameters(
         const std::vector<std::string>& in_ids,
         std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
             _aidl_return);
     ndk::ScopedAStatus setVendorParameters(
         const std::vector<
             ::aidl::android::hardware::audio::core::VendorParameter>&
             in_parameters,
         bool in_async); 
     void getPositionInFrames(int64_t* dspFrames);
     void updateOffloadMetadata(
         const ::aidl::android::hardware::audio::common::AudioOffloadMetadata&
             offloadMetaData);
     void updateSourceMetadata(
         const ::aidl::android::hardware::audio::common::SourceMetadata&
             sourceMetaData);
     std::unique_ptr<uint8_t[]> getPayloadCodecInfo();
     bool isCodecConfigured() const { return mIsCodecConfigured; }
     pal_snd_dec_t getPalSndDec();


    private:
    // dynamic compress info
     const ::aidl::android::hardware::audio::common::AudioOffloadMetadata*
         mOffloadMetadata{nullptr};
     const ::aidl::android::hardware::audio::common::SourceMetadata*
         mSourceMetadata{nullptr};
    // this is static info at the stream creation, for dynamic info check AudioOffloadMetadata
     const ::aidl::android::media::audio::common::AudioOffloadInfo& mOffloadInfo;
     std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback>
         mAsyncCallback;
     uint16_t mCompressBitWidth{0};
     bool mIsCodecConfigured{false};
     pal_stream_handle_t* mCompressPlaybackHandle{nullptr};
     pal_snd_dec_t mPalSndDec{};
     int32_t mSampleRate;
     ::aidl::android::media::audio::common::AudioFormatDescription
         mCompressFormat;
     ::aidl::android::media::audio::common::AudioChannelLayout mChannelLayout;
     int32_t mBitWidth;
};

class CompressCapture final{
    public:
     class Aac final {
        public:
         inline static const std::string kDSPAacBitRate{
             "dsp_aac_audio_bitrate"};
         inline static const std::string
             kDSPAacGlobalCutoffFrequency{
                 "dsp_aac_audio_global_cutoff_frequency"};
         enum EncodingMode {
             LC = 0x02,
             SBR = 0x05,
             PS = 0x1D,
         };
         enum EncodingFormat {
             ADTS = 0x00,
             LOAS = 0x01,
             RAW = 0x03,
             LATM = 0x04,
         };

         constexpr static uint32_t kAacLcPCMSamplesPerFrame = 1024;
         constexpr static uint32_t kHeAacPCMSamplesPerFrame = 2048;
         constexpr static int32_t kAacLcMonoMinSupportedBitRate = 8000;
         constexpr static int32_t kAacLcStereoMinSupportedBitRate = 16000;
         constexpr static int32_t kHeAacMonoMinSupportedBitRate1 = 10000;
         constexpr static int32_t kHeAacMonoMinSupportedBitRate2 = 12000;
         constexpr static int32_t kHeAacStereoMinSupportedBitRate1 = 18000;
         constexpr static int32_t kHeAacStereoMinSupportedBitRate2 = 24000;
         constexpr static int32_t kHeAacPsStereoMinSupportedBitRate1 = 10000;
         constexpr static int32_t kHeAacPsStereoMinSupportedBitRate2 = 12000;

         constexpr static int32_t kAacLcMonoMaxSupportedBitRate = 192000;
         constexpr static int32_t kAacLcStereoMaxSupportedBitRate = 384000;
         constexpr static int32_t kHeAacMonoMaxSupportedBitRate = 192000;
         constexpr static int32_t kHeAacStereoMaxSupportedBitRate = 192000;
         constexpr static int32_t kHeAacPstereoMaxSupportedBitRate = 192000;
         static const uint32_t KAacMaxOutputSize = 2048;   // bytes
         static const int32_t kAacDefaultBitrate = 36000;  // bps
     };

     constexpr static size_t kPeriodCount = 4;
     explicit CompressCapture(
         const ::aidl::android::media::audio::common::AudioFormatDescription&
             format,
         int32_t sampleRate,
         const ::aidl::android::media::audio::common::AudioChannelLayout&
             channelLayout);
     void setPalHandle(pal_stream_handle_t* handle);
     ndk::ScopedAStatus setVendorParameters(
         const std::vector<
             ::aidl::android::hardware::audio::core::VendorParameter>&
             in_parameters,
         bool in_async);
    size_t getLatencyMs();
     ndk::ScopedAStatus getVendorParameters(
         const std::vector<std::string>& in_ids,
         std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
             _aidl_return);
     std::unique_ptr<uint8_t[]> getPayloadCodecInfo() const;
     int32_t getAACMinBitrateValue();

     int32_t getAACMaxBitrateValue();

     uint32_t getAACMaxBufferSize();
     void setAACDSPBitRate();
     static size_t getPeriodBufferSize(
         const ::aidl::android::media::audio::common::AudioFormatDescription&
             format);

     const ::aidl::android::media::audio::common::AudioFormatDescription&
         mCompressFormat;
     const ::aidl::android::media::audio::common::AudioChannelLayout&
         mChannelLayout;
     int32_t mSampleRate{};
     size_t mPCMSamplesPerFrame{0};
     pal_stream_handle_t* mCompressHandle{nullptr};
     size_t mNumReadCalls{0};
     pal_snd_enc_t mPalSndEnc{};
};

class PcmOffloadPlayback final {
    public:
     constexpr static size_t kPeriodDurationMs = 80;
     constexpr static size_t kPeriodCount = 4;
     constexpr static size_t kPlatformDelayMs = 30;
     static size_t getPeriodSize(
         const ::aidl::android::media::audio::common::AudioFormatDescription&
             formatDescription,
         const ::aidl::android::media::audio::common::AudioChannelLayout&
             channelLayout,
         const int32_t sampleRate);

    private:
     int64_t mCachePresentationPosition;
};

class VoipPlayback final {
    public:
    constexpr static size_t kBufferDurationMs = 20;
    constexpr static size_t kPeriodCount = 2;
    static size_t getPeriodSize(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig);
};

class SpatialPlayback {
    public:
    constexpr static size_t kPeriodDurationMs = 10;
    constexpr static size_t kPeriodSize = 480; // 10ms
    constexpr static size_t kPeriodCount = 2;
    constexpr static size_t kPlatformDelayMs = 13;
};

class VoipRecord {
   public:
    constexpr static uint32_t kCaptureDurationMs = 20;
    constexpr static uint32_t kPeriodCount = 4;
    static size_t getPeriodSize(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig);
};

class VoiceCallRecord {
   public:
    constexpr static size_t kCaptureDurationMs = 20;
    constexpr static size_t kPeriodCount = 2;
    static size_t getPeriodSize(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig);
    pal_incall_record_direction getRecordDirection(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            mixPortConfig);
};

class UllPlayback {
    public:
    constexpr static size_t kPeriodSize = 48; //1ms
    constexpr static size_t kPeriodMultiplier = 3;
    constexpr static size_t kPlatformDelayMs = (4 * 1000LL);
    constexpr static uint32_t kPeriodCount = 512;
    static size_t getPeriodSize(
        const ::aidl::android::media::audio::common::AudioFormatDescription&
            formatDescription,
        const ::aidl::android::media::audio::common::AudioChannelLayout&
            channelLayout);
};

class MMapPlayback {
    public:
    constexpr static size_t kPeriodSize = 48; //1ms
    constexpr static size_t kPlatformDelayMs = (3 * 1000LL);
    constexpr static uint32_t kPeriodCount = 512;
    void setPalHandle(pal_stream_handle_t* handle);
    int32_t createMMapBuffer(int64_t frameSize, int32_t* fd, int64_t* burstSizeFrames,
                             int32_t* flags, int32_t* bufferSizeFrames);
    int32_t getMMapPosition(int64_t* frames, int64_t* timeNs);
    static size_t getPeriodSize(
        const ::aidl::android::media::audio::common::AudioFormatDescription&
            formatDescription,
        const ::aidl::android::media::audio::common::AudioChannelLayout&
            channelLayout);
    pal_stream_handle_t* mPalHandle{nullptr};


};

class MMapRecord {
    public:
    constexpr static uint32_t kPeriodSize = 48; //Same as Playback?
    constexpr static size_t kPeriodCount = 512;
    void setPalHandle(pal_stream_handle_t* handle);
    int32_t createMMapBuffer(int64_t frameSize, int32_t* fd, int64_t* burstSizeFrames,
                             int32_t* flags, int32_t* bufferSizeFrames);
    int32_t getMMapPosition(int64_t* frames, int64_t* timeNs);
    static size_t getPeriodSize(
        const ::aidl::android::media::audio::common::AudioFormatDescription&
            formatDescription,
        const ::aidl::android::media::audio::common::AudioChannelLayout&
            channelLayout);
    pal_stream_handle_t* mPalHandle{nullptr};
};

class InCallMusic {
   public:
    constexpr static size_t kPeriodSize = 960 * 4;
    constexpr static size_t kPeriodCount = 4;
};

}  // namespace qti::audio::core
