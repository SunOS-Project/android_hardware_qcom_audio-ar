/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <PalDefs.h>
#include <aidl/android/media/audio/common/AudioDevice.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioPort.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>

#include <sstream>
#include <unordered_map>
#include <vector>

#include <AudioUsecase.h>

namespace qti::audio::core {
// Singleton
class Platform {
   private:
    explicit Platform();

    Platform(const Platform&) = delete;
    Platform& operator=(const Platform& x) = delete;

    Platform(Platform&& other) = delete;
    Platform& operator=(Platform&& other) = delete;

   public:
    constexpr static uint32_t kDefaultOutputSampleRate = 48000;
    constexpr static uint32_t kDefaultPCMBidWidth = 16;
    constexpr static pal_audio_fmt_t kDefaultPalPCMFormat =
        PAL_AUDIO_FMT_PCM_S16_LE;
    std::unordered_map<std::string, std::string> mParameters;

   public:
    static Platform& getInstance();
    bool setParameter(const std::string& key, const std::string& value);
    std::string getParameter(const std::string& key) const;
    std::string toString() const;
    bool isFormatTypePCM(
        const ::aidl::android::media::audio::common::AudioFormatDescription&)
        const noexcept;
    bool isUsbDevice(const ::aidl::android::media::audio::common::AudioDevice&)
        const noexcept;
    bool isInputDevice(
        const ::aidl::android::media::audio::common::AudioDevice&)
        const noexcept;
    bool isOutputDevice(
        const ::aidl::android::media::audio::common::AudioDevice&)
        const noexcept;
    std::unique_ptr<pal_stream_attributes> getPalStreamAttributes(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            portConfig,
        const bool isInput) const;
    std::vector<pal_device> getPalDevices(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            portConfig,
        const std::vector<::aidl::android::media::audio::common::AudioDevice>&
            setDevices,
        const bool isInput) const;
    std::vector<uint8_t> getPalVolumeData(
        const std::vector<float>& in_channelVolumes) const;
    std::unique_ptr<pal_buffer_config_t> getPalBufferConfig(
        const size_t bufferSize, const size_t bufferCount) const;
    std::vector<::aidl::android::media::audio::common::AudioProfile>
    getDynamicProfiles(const ::aidl::android::media::audio::common::AudioPort&
                           dynamicDeviceAudioPort) const;
    bool handleDeviceConnectionChange(
        const ::aidl::android::media::audio::common::AudioPort& deviceAudioPort,
        const bool isConnect) const;
    std::unique_ptr<AudioUsecase> createAudioUsecase(
        const ::aidl::android::media::audio::common::AudioPortConfig& mixport,
        const bool isInput) const;
};
}  // namespace qti::audio::core