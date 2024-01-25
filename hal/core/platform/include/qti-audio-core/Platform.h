/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once
#include <aidl/android/hardware/audio/core/IModule.h>
#include <aidl/android/hardware/audio/core/VendorParameter.h>
#include <aidl/android/media/audio/common/AudioDevice.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioPlaybackRate.h>
#include <aidl/android/media/audio/common/AudioPort.h>
#include <aidl/android/media/audio/common/AudioPortConfig.h>
#include <extensions/AudioExtension.h>

#include <PalApi.h>
#include <qti-audio-core/AudioUsecase.h>

namespace qti::audio::core {

struct HdmiParameters {
    int controller;
    int stream;
    pal_device_id_t deviceId;
};

enum class PlaybackRateStatus { SUCCESS, UNSUPPORTED, ILLEGAL_ARGUMENT };

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
    std::vector<pal_device> convertToPalDevices(
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& devices)
            const noexcept;
    std::vector<pal_device> configureAndFetchPalDevices(
            const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig,
            const Usecase& tag,
            const std::vector<::aidl::android::media::audio::common::AudioDevice>& setDevices)
            const;

    /*
    * @brief creates a pal payload for a pal volume and sets to PAL
    * @param handle : valid pal stream handle
    * @param volumes vector of volumes in floats
    * return 0 in success, error code otherwise
    */
    int setVolume(pal_stream_handle_t* handle, const std::vector<float>& volumes) const;

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
                                      const std::string& customKey) const;

    bool setStreamMicMute(pal_stream_handle_t* streamHandlePtr, const bool muted);
    bool updateScreenState(const bool isTurnedOn) noexcept;
    bool isScreenTurnedOn() const noexcept;

    /*
    * @brief creates a pal payload for a speed factor and sets to PAL
    * @param handle : pal stream handle
    * @param tag usecase tag
    * @param playbackRate  playback rate to be set
    * return PlaybackRateStatus::SUCCESS on success, or if stream handle is not set.
    * return PlaybackRateStatus::UNSUPPORTED operation, usecase does not support speed operations
    * or speed parameters are not in the range
    * return PlaybackRateStatus::ILLEGAL_ARGUMENT in case of any other failure
    */
    PlaybackRateStatus setPlaybackRate(
            pal_stream_handle_t* handle, const Usecase& tag,
            const ::aidl::android::media::audio::common::AudioPlaybackRate& playbackRate);
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

    void setWFDProxyChannels(const uint32_t numProxyChannels) noexcept;
    uint32_t getWFDProxyChannels() const noexcept;

    void setHapticsVolume(const float hapticsVolume) const noexcept;
    void setHapticsIntensity(const int hapticsIntensity) const noexcept;

    void updateUHQA(const bool enable) noexcept;
    bool isUHQAEnabled() const noexcept;
    void setFTMSpeakerProtectionMode(uint32_t const heatUpTime, uint32_t const runTime,
                                     bool const isFactoryTest, bool const isValidationMode,
                                     bool const isDynamicCalibration) const noexcept;
    std::optional<std::string> getFTMResult() const noexcept;
    std::optional<std::string> getSpeakerCalibrationResult() const noexcept;

    void updateScreenRotation(const ::aidl::android::hardware::audio::core::IModule::ScreenRotation
                                      in_rotation) noexcept;
    ::aidl::android::hardware::audio::core::IModule::ScreenRotation getCurrentScreenRotation() const
            noexcept;

    bool platformSupportsOffloadSpeed() { return mOffloadSpeedSupported; }
    bool usecaseSupportsOffloadSpeed(const Usecase& tag) {
        return platformSupportsOffloadSpeed() && isOffload(tag);
    }

    bool isOffload(const Usecase& tag) { return tag == Usecase::COMPRESS_OFFLOAD_PLAYBACK; }
    int setLatencyMode(uint32_t mode);
    int getRecommendedLatencyModes(
          std::vector<::aidl::android::media::audio::common::AudioLatencyMode>* _aidl_return);

  private:
    void customizePalDevices(
            const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig,
            const Usecase& tag, std::vector<pal_device>& palDevices) const noexcept;
    void configurePalDevicesForHIFIPCMFilter(std::vector<pal_device>&) const noexcept;
    bool getBtConfig(pal_param_bta2dp_t* bTConfig);
    std::vector<::aidl::android::media::audio::common::AudioProfile> getUsbProfiles(
            const ::aidl::android::media::audio::common::AudioPort& port) const;

    std::optional<struct HdmiParameters> getHdmiParameters(
            const ::aidl::android::media::audio::common::AudioDevice&) const;

  public:
    constexpr static uint32_t kDefaultOutputSampleRate = 48000;
    constexpr static uint32_t kDefaultPCMBidWidth = 16;
    constexpr static pal_audio_fmt_t kDefaultPalPCMFormat = PAL_AUDIO_FMT_PCM_S16_LE;

  private:
    std::vector<::aidl::android::media::audio::common::AudioDevice> mPrimaryPlaybackDevices{};

    std::map<std::string, std::string> mParameters;
    card_status_t mSndCardStatus{CARD_STATUS_OFFLINE};
    bool mInCallMusicEnabled{false};
    bool mIsScreenTurnedOn{false};
    uint32_t mWFDProxyChannels{0};
    bool mIsUHQAEnabled{false};
    ::aidl::android::hardware::audio::core::IModule::ScreenRotation mCurrentScreenRotation{
            ::aidl::android::hardware::audio::core::IModule::ScreenRotation::DEG_0};
    bool mOffloadSpeedSupported = false;
};
} // namespace qti::audio::core
