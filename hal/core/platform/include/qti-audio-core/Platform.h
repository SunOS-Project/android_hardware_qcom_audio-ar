/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once
#include <aidl/android/hardware/audio/core/VendorParameter.h>
#include <aidl/android/media/audio/common/AudioDevice.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioPort.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>
#include <extensions/AudioExtension.h>

#include <PalApi.h>

namespace qti::audio::core {
class Platform {
  private:
    explicit Platform();

    Platform(const Platform&) = delete;
    Platform& operator=(const Platform& x) = delete;

    Platform(Platform&& other) = delete;
    Platform& operator=(Platform&& other) = delete;
    static int palGlobalCallback(uint32_t event_id, uint32_t* event_data, uint64_t cookie);

  public:
    // BT related params used across
    bool bt_lc3_speech_enabled;
    static btsco_lc3_cfg_t btsco_lc3_cfg;

    int mCallState;
    int mCallMode;
    static Platform& getInstance();
    bool setParameter(const std::string& key, const std::string& value);
    bool setBluetoothParameters(const char* kvpairs);
    bool setVendorParameters(
            const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
                    in_parameters,
            bool in_async);

    std::string getParameter(const std::string& key) const;
    std::string toString() const;
    bool isFormatTypePCM(const ::aidl::android::media::audio::common::AudioFormatDescription&) const
            noexcept;
    bool isUsbDevice(const ::aidl::android::media::audio::common::AudioDevice&) const noexcept;
    bool isHdmiDevice(const ::aidl::android::media::audio::common::AudioDevice&) const noexcept;
    bool isInputDevice(const ::aidl::android::media::audio::common::AudioDevice&) const noexcept;
    bool isOutputDevice(const ::aidl::android::media::audio::common::AudioDevice&) const noexcept;
    bool isBluetoothDevice(const ::aidl::android::media::audio::common::AudioDevice& d) const
            noexcept;
    bool isSoundCardUp() const noexcept;
    bool isSoundCardDown() const noexcept;
    size_t getIOBufferSizeInFrames(
            const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig) const;
    size_t getMinimumStreamSizeFrames(
            const std::vector<::aidl::android::media::audio::common::AudioPortConfig*>& sources,
            const std::vector<::aidl::android::media::audio::common::AudioPortConfig*>& sinks)
            const;
    std::unique_ptr<pal_stream_attributes> getPalStreamAttributes(
            const ::aidl::android::media::audio::common::AudioPortConfig& portConfig,
            const bool isInput) const;
    std::vector<pal_device> getPalDevices(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& setDevices)
            const;
    std::vector<uint8_t> getPalVolumeData(const std::vector<float>& in_channelVolumes) const;
    std::unique_ptr<pal_buffer_config_t> getPalBufferConfig(const size_t bufferSize,
                                                            const size_t bufferCount) const;
    std::vector<::aidl::android::media::audio::common::AudioProfile> getDynamicProfiles(
            const ::aidl::android::media::audio::common::AudioPort& dynamicDeviceAudioPort) const;
    bool handleDeviceConnectionChange(
            const ::aidl::android::media::audio::common::AudioPort& deviceAudioPort,
            const bool isConnect) const;
    uint32_t getBluetoothLatencyMs(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>&
                    bluetoothDevices);
    std::unique_ptr<pal_stream_attributes> getDefaultTelephonyAttributes() const;
    void configurePalDevicesCustomKey(std::vector<pal_device>& palDevices,
                                      const std::string& key) const;

    bool setStreamMicMute(pal_stream_handle_t* streamHandlePtr, const bool muted);

    std::vector<::aidl::android::media::audio::common::AudioDevice> getPrimaryPlaybackDevices()
            const {
        return mPrimaryPlaybackDevices;
    }

    void setPrimaryPlaybackDevices(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices) {
        mPrimaryPlaybackDevices = devices;
    }

    void setInCallMusicState(const bool state) noexcept { mInCallMusicEnabled = state; }
    bool getInCallMusicState() noexcept { return mInCallMusicEnabled; }

    void updateCallState(int callState) { mCallState = callState; }
    void updateCallMode(int callMode) { mCallMode = callMode; }

    int getCallState() { return mCallState; }
    int getCallMode() { return mCallMode; }
    bool isA2dpSuspended();

  private:
    bool getBtConfig(pal_param_bta2dp_t* bTConfig);

  public:
    constexpr static uint32_t kDefaultOutputSampleRate = 48000;
    constexpr static uint32_t kDefaultPCMBidWidth = 16;
    constexpr static pal_audio_fmt_t kDefaultPalPCMFormat = PAL_AUDIO_FMT_PCM_S16_LE;

  private:
    std::vector<::aidl::android::media::audio::common::AudioDevice> mPrimaryPlaybackDevices{};

    std::map<std::string, std::string> mParameters;
    card_status_t mSndCardStatus{CARD_STATUS_OFFLINE};
    bool mInCallMusicEnabled{false};
};
} // namespace qti::audio::core
