/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_NDEBUG 0
#define LOG_TAG "AHAL_Platform_QTI"

#include <Utils.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <cutils/str_parms.h>
#include <hardware/audio.h>
#include <qti-audio-core/AudioUsecase.h>
#include <qti-audio-core/Platform.h>
#include <qti-audio-core/PlatformUtils.h>
#include <qti-audio/PlatformConverter.h>

#include <aidl/qti/audio/core/VString.h>
#include <cutils/properties.h>
#include <dlfcn.h>
#include <extensions/AudioExtension.h>
#include <unistd.h>

#define LC3_SWB_CODEC_CONFIG_INDEX 4
#define LC3_BROADCAST_TRANSIT_MODE 1
#define LC3_HFP_TRANSIT_MODE 3

using ::aidl::android::media::audio::common::AudioChannelLayout;
using ::aidl::android::media::audio::common::AudioDevice;
using ::aidl::android::media::audio::common::AudioDeviceAddress;
using ::aidl::android::media::audio::common::AudioDeviceDescription;
using ::aidl::android::media::audio::common::AudioDeviceType;
using ::aidl::android::media::audio::common::AudioFormatDescription;
using ::aidl::android::media::audio::common::AudioFormatType;
using ::aidl::android::media::audio::common::AudioIoFlags;
using ::aidl::android::media::audio::common::AudioOutputFlags;
using ::aidl::android::media::audio::common::AudioPort;
using ::aidl::android::media::audio::common::AudioPortConfig;
using ::aidl::android::media::audio::common::AudioPortDeviceExt;
using ::aidl::android::media::audio::common::AudioPortExt;
using ::aidl::android::media::audio::common::AudioProfile;
using ::aidl::android::media::audio::common::PcmType;

using ::aidl::android::hardware::audio::common::getChannelCount;
using ::aidl::android::hardware::audio::common::getFrameSizeInBytes;
using ::aidl::android::hardware::audio::common::isBitPositionFlagSet;
using ::aidl::android::hardware::audio::core::IModule;

namespace qti::audio::core {

btsco_lc3_cfg_t Platform::btsco_lc3_cfg = {};

size_t Platform::getFrameCount(
        const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig) const {
    Usecase tag = getUsecaseTag(mixPortConfig);
    size_t numFrames = 0;
    if (tag == Usecase::DEEP_BUFFER_PLAYBACK) {
        numFrames = DeepBufferPlayback::kPeriodSize;
    } else if (tag == Usecase::PCM_OFFLOAD_PLAYBACK) {
        numFrames = PcmOffloadPlayback::getFrameCount(mixPortConfig);
    } else if (tag == Usecase::LOW_LATENCY_PLAYBACK) {
        numFrames = LowLatencyPlayback::kPeriodSize;
    } else if (tag == Usecase::PCM_RECORD) {
        numFrames = PcmRecord::getMinFrames(mixPortConfig);
    } else if (tag == Usecase::FAST_RECORD) {
        numFrames = FastRecord::getPeriodSize(mixPortConfig) /
                    getFrameSizeInBytes(mixPortConfig.format.value(),
                                        mixPortConfig.channelMask.value());
    } else if (tag == Usecase::ULTRA_FAST_RECORD) {
        numFrames = UltraFastRecord::kPeriodSize;
    } else if (tag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        const size_t numBytes = CompressPlayback::getPeriodBufferSize(mixPortConfig.format.value());
        constexpr size_t compressFrameSize = 1;
        numFrames = numBytes / compressFrameSize;
    } else if (tag == Usecase::COMPRESS_CAPTURE) {
        numFrames = CompressCapture::getPeriodBufferSize(mixPortConfig.format.value());
    } else if (tag == Usecase::ULL_PLAYBACK) {
        numFrames = UllPlayback::kPeriodSize;
    } else if (tag == Usecase::MMAP_PLAYBACK) {
        numFrames = MMapPlayback::kPeriodSize;
    } else if (tag == Usecase::MMAP_RECORD) {
        numFrames = MMapRecord::kPeriodSize;
    } else if (tag == Usecase::VOIP_PLAYBACK) {
        numFrames = VoipPlayback::getPeriodSize(mixPortConfig);
    } else if (tag == Usecase::VOIP_RECORD) {
        numFrames = VoipRecord::getPeriodSize(mixPortConfig);
    } else if (tag == Usecase::VOICE_CALL_RECORD) {
        numFrames = VoiceCallRecord::getPeriodSize(mixPortConfig);
    } else if (tag == Usecase::IN_CALL_MUSIC) {
        numFrames = InCallMusic::kPeriodSize;
    } else if (tag == Usecase::SPATIAL_PLAYBACK) {
        numFrames = SpatialPlayback::kPeriodSize;
    } else if (tag == Usecase::HOTWORD_RECORD) {
        numFrames = HotwordRecord::getMinFrames(mixPortConfig);
    } else if (tag == Usecase::HAPTICS_PLAYBACK) {
        numFrames = HapticsPlayback::kPeriodSize;
    }
    LOG(VERBOSE) << __func__ << " frames: " << numFrames << " for " << getName(tag);
    return numFrames;
}

size_t Platform::getMinimumStreamSizeFrames(const std::vector<AudioPortConfig*>& sources,
                                            const std::vector<AudioPortConfig*>& sinks) const {
    if (sources.size() > 1) {
        LOG(WARNING) << __func__ << " unable to decide the minimum stream size for sources "
                                    "more than one; actual size:"
                     << sources.size();
        return 0;
    }
    // choose the mix port
    auto isMixPortConfig = [](const auto& audioPortConfig) {
        return audioPortConfig.ext.getTag() == AudioPortExt::Tag::mix;
    };

    const auto& mixPortConfig = isMixPortConfig(*sources.at(0)) ? *(sources.at(0)) : *(sinks.at(0));
    return getFrameCount(mixPortConfig);
}

std::unique_ptr<pal_stream_attributes> Platform::getPalStreamAttributes(
        const AudioPortConfig& portConfig, const bool isInput) const {
    const auto& audioFormat = portConfig.format.value();
    const auto palFormat = PlatformConverter::getPalFormatId(audioFormat);
    if (palFormat == PAL_AUDIO_FMT_COMPRESSED_RANGE_END) {
        return nullptr;
    }

    const auto& audioChannelLayout = portConfig.channelMask.value();
    auto palChannelInfo = PlatformConverter::getPalChannelInfoForChannelCount(
            getChannelCount(audioChannelLayout));
    if (palChannelInfo == nullptr) {
        LOG(ERROR) << __func__ << " failed to find corresponding pal channel info for "
                   << audioChannelLayout.toString();
        return nullptr;
    }
    const auto sampleRate = portConfig.sampleRate.value().value;
    if (!sampleRate) {
        LOG(ERROR) << __func__ << " invalid sample rate " << std::to_string(sampleRate);
        return nullptr;
    }

    auto attributes = std::make_unique<pal_stream_attributes>();
    auto bitWidth = PlatformConverter::getBitWidthForAidlPCM(audioFormat);
    bitWidth == 0 ? (void)(bitWidth = kDefaultPCMBidWidth) : (void)0;

    if (!isInput) {
        attributes->direction = PAL_AUDIO_OUTPUT;
        attributes->out_media_config.sample_rate = sampleRate;
        attributes->out_media_config.aud_fmt_id = palFormat;
        attributes->out_media_config.ch_info = *(palChannelInfo);
        attributes->out_media_config.bit_width = bitWidth;
    } else {
        attributes->direction = PAL_AUDIO_INPUT;
        attributes->in_media_config.sample_rate = sampleRate;
        attributes->in_media_config.aud_fmt_id = palFormat;
        attributes->in_media_config.ch_info = *(palChannelInfo);
        attributes->in_media_config.bit_width = bitWidth;
    }

    return std::move(attributes);
}

std::unique_ptr<pal_stream_attributes> Platform::getDefaultTelephonyAttributes() const {
    auto attributes = std::make_unique<pal_stream_attributes>();
    auto inChannelInfo = PlatformConverter::getPalChannelInfoForChannelCount(1);
    auto outChannelInfo = PlatformConverter::getPalChannelInfoForChannelCount(2);
    attributes->type = PAL_STREAM_VOICE_CALL;
    attributes->direction = PAL_AUDIO_INPUT_OUTPUT;
    attributes->in_media_config.sample_rate = kDefaultOutputSampleRate;
    attributes->in_media_config.ch_info = *inChannelInfo;
    attributes->in_media_config.bit_width = kDefaultPCMBidWidth;
    attributes->in_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
    attributes->out_media_config.sample_rate = kDefaultOutputSampleRate;
    attributes->out_media_config.ch_info = *outChannelInfo;
    attributes->out_media_config.bit_width = kDefaultPCMBidWidth;
    attributes->out_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
    return std::move(attributes);
}

std::unique_ptr<pal_stream_attributes> Platform::getDefaultCRSTelephonyAttributes() const {
    auto attributes = std::make_unique<pal_stream_attributes>();
    auto outChannelInfo = PlatformConverter::getPalChannelInfoForChannelCount(2);
    attributes->type = PAL_STREAM_LOOPBACK;
    attributes->info.opt_stream_info.loopback_type = PAL_STREAM_LOOPBACK_PLAYBACK_ONLY;
    attributes->direction = PAL_AUDIO_OUTPUT;
    attributes->out_media_config.sample_rate = kDefaultOutputSampleRate;
    attributes->out_media_config.ch_info = *outChannelInfo;
    attributes->out_media_config.bit_width = kDefaultPCMBidWidth;
    attributes->out_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
    return std::move(attributes);
}

void Platform::configurePalDevicesCustomKey(std::vector<pal_device>& palDevices,
                                            const std::string& customKey) const {
    for (auto& palDevice : palDevices) {
        setPalDeviceCustomKey(palDevice, customKey);
    }
}

bool Platform::setStreamMicMute(pal_stream_handle_t* streamHandlePtr, const bool muted) {
    if (int32_t ret = ::pal_stream_set_mute(streamHandlePtr, muted); ret) {
        return false;
    }
    return true;
}

bool Platform::updateScreenState(const bool isTurnedOn) noexcept {
    mIsScreenTurnedOn = isTurnedOn;
    pal_param_screen_state_t screenState{.screen_state = mIsScreenTurnedOn};
    if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_SCREEN_STATE, &screenState,
                                      sizeof(pal_param_screen_state_t));
        ret) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_SCREEN_STATE failed";
        return false;
    }
    return true;
}

bool Platform::isScreenTurnedOn() const noexcept {
    return mIsScreenTurnedOn;
}

void Platform::configurePalDevicesForHIFIPCMFilter(
        std::vector<pal_device>& palDevices) const noexcept {
    if (palDevices.size() == 0) {
        return;
    }

    bool isEnabled = false;

    auto getStatus = [&]() -> bool {
        bool status = false;
        bool* payLoad = &status;
        size_t payLoadSize = 0;
        if (int32_t ret =
                    ::pal_get_param(PAL_PARAM_ID_HIFI_PCM_FILTER,
                                    reinterpret_cast<void**>(&payLoad), &payLoadSize, nullptr);
            ret) {
            LOG(ERROR) << ": failed to get PAL_PARAM_ID_HIFI_PCM_FILTER status";
            return false;
        }
        return status;
    };

    for (auto& palDevice : palDevices) {
        if ((palDevice.id == PAL_DEVICE_OUT_WIRED_HEADSET ||
             palDevice.id == PAL_DEVICE_OUT_WIRED_HEADPHONE)) {
            if (!isEnabled) {
                isEnabled = getStatus();
            }
            if (isEnabled) {
                setPalDeviceCustomKey(palDevice, "hifi-filter_custom_key");
            }
        }
    }
}

void Platform::customizePalDevices(const AudioPortConfig& mixPortConfig, const Usecase& tag,
                                   std::vector<pal_device>& palDevices) const noexcept {
    const auto& sampleRate = mixPortConfig.sampleRate.value().value;
    if (sampleRate != 384000 && sampleRate != 352800) {
        configurePalDevicesForHIFIPCMFilter(palDevices);
    }
}

std::vector<pal_device> Platform::convertToPalDevices(
        const std::vector<AudioDevice>& devices) const noexcept {
    if (devices.size() == 0) {
        LOG(ERROR) << __func__ << " the set devices is empty";
        return {};
    }
    std::vector<pal_device> palDevices{devices.size()};

    size_t i = 0;
    for (auto& device : devices) {
        const auto palDeviceId = PlatformConverter::getPalDeviceId(device.type);
        if (palDeviceId == PAL_DEVICE_OUT_MIN) {
            return {};
        }
        palDevices[i].id = palDeviceId;

        /* Todo map each AIDL device type to alteast one PAL device */
        if (palDevices[i].id == PAL_DEVICE_OUT_SPEAKER &&
            device.type.type == AudioDeviceType::OUT_SPEAKER_SAFE) {
            setPalDeviceCustomKey(palDevices[i], "speaker-safe");
        } else if (palDevices[i].id == PAL_DEVICE_OUT_SPEAKER &&
                   device.type.type == AudioDeviceType::OUT_SPEAKER) {
            const auto isMSPPEnabled =
                    ::android::base::GetBoolProperty("vendor.audio.mspp.enable", false);
            if (isMSPPEnabled) {
                setPalDeviceCustomKey(palDevices[i], "mspp");
            }
        }

        palDevices[i].config.sample_rate = kDefaultOutputSampleRate;
        palDevices[i].config.bit_width = kDefaultPCMBidWidth;
        palDevices[i].config.aud_fmt_id = kDefaultPalPCMFormat;

        if (isUsbDevice(device)) {
            const auto& deviceAddress = device.address;
            if (deviceAddress.getTag() != AudioDeviceAddress::Tag::alsa) {
                LOG(ERROR) << __func__ << " failed to find alsa address for given usb device "
                           << device.toString();
                return {};
            }
            const auto& deviceAddressAlsa = deviceAddress.get<AudioDeviceAddress::Tag::alsa>();
            palDevices[i].address.card_id = deviceAddressAlsa[0];
            palDevices[i].address.device_num = deviceAddressAlsa[1];
        } else if (isHdmiDevice(device)) {
            if (auto result = getHdmiParameters(device)) {
                palDevices[i].id = result->deviceId;
            } else {
                return {};
            }
        }
        i++;
    }
    return palDevices;
}

std::vector<pal_device> Platform::configureAndFetchPalDevices(
        const AudioPortConfig& mixPortConfig, const Usecase& tag,
        const std::vector<AudioDevice>& devices) const {
    if (devices.size() == 0) {
        LOG(ERROR) << __func__ << " the set devices is empty";
        return {};
    }
    auto palDevices = convertToPalDevices(devices);

    customizePalDevices(mixPortConfig,tag,palDevices);

    return palDevices;
}

int Platform::setVolume(pal_stream_handle_t* handle, const std::vector<float>& volumes) const {
    auto data = makePalVolumes(volumes);
    if (data.empty()) {
        LOG(ERROR) << __func__ << ": failed to configure volume";
        return -1;
    }
    auto palVolumeData = reinterpret_cast<pal_volume_data*>(data.data());

    return ::pal_stream_set_volume(handle, palVolumeData);
}

std::unique_ptr<pal_buffer_config_t> Platform::getPalBufferConfig(const size_t bufferSize,
                                                                  const size_t bufferCount) const {
    auto palBufferConfig = std::make_unique<pal_buffer_config_t>();
    palBufferConfig->buf_size = bufferSize;
    palBufferConfig->buf_count = bufferCount;
    return std::move(palBufferConfig);
}

std::vector<::aidl::android::media::audio::common::AudioProfile> Platform::getUsbProfiles(
        const AudioPort& port) const {
    const auto& devicePortExt = port.ext.get<AudioPortExt::Tag::device>();
    auto& audioDeviceDesc = devicePortExt.device.type;
    const auto palDeviceId = PlatformConverter::getPalDeviceId(audioDeviceDesc);
    if (palDeviceId == PAL_DEVICE_OUT_MIN) {
        return {};
    }

    const auto& addressTag = devicePortExt.device.address.getTag();
    if (addressTag != AudioDeviceAddress::Tag::alsa) {
        LOG(ERROR) << __func__ << ": no alsa address provided for the AudioPort" << port.toString();
        return {};
    }
    const auto& deviceAddressAlsa =
            devicePortExt.device.address.get<AudioDeviceAddress::Tag::alsa>();
    const auto cardId = deviceAddressAlsa[0];
    const auto deviceId = deviceAddressAlsa[1];

    // get capability from device of USB
    auto deviceCapability = std::make_unique<pal_param_device_capability_t>();
    if (!deviceCapability) {
        LOG(ERROR) << __func__ << ": allocation failed ";
        return {};
    }

    auto dynamicMediaConfig = std::make_unique<dynamic_media_config_t>();
    if (!dynamicMediaConfig) {
        LOG(ERROR) << __func__ << ": allocation failed ";
        return {};
    }

    size_t payloadSize = 0;
    deviceCapability->id = palDeviceId;
    deviceCapability->addr.card_id = cardId;
    deviceCapability->addr.device_num = deviceId;
    deviceCapability->config = dynamicMediaConfig.get();
    deviceCapability->is_playback = isOutputDevice(devicePortExt.device);

    void* deviceCapabilityPtr = deviceCapability.get();
    if (int32_t ret = pal_get_param(PAL_PARAM_ID_DEVICE_CAPABILITY, &deviceCapabilityPtr,
                                    &payloadSize, nullptr);
        ret != 0) {
        LOG(ERROR) << __func__ << " PAL get param failed for PAL_PARAM_ID_DEVICE_CAPABILITY" << ret;
        return {};
    }
    if (!dynamicMediaConfig->jack_status) {
        LOG(ERROR) << __func__ << " false usb jack status ";
        return {};
    }

    return getSupportedAudioProfiles(deviceCapability.get(), "usb");
}

std::vector<AudioProfile> Platform::getDynamicProfiles(
        const AudioPort& dynamicDeviceAudioPort) const {
    const auto& deviceExtTag = dynamicDeviceAudioPort.ext.getTag();
    if (deviceExtTag != AudioPortExt::Tag::device) {
        LOG(ERROR) << __func__ << ": provided AudioPort is not device port"
                   << dynamicDeviceAudioPort.toString();
        return {};
    }

    LOG(VERBOSE) << __func__ << ": fetching dynamic profiles for "
                 << dynamicDeviceAudioPort.toString();

    const auto& devicePortExt = dynamicDeviceAudioPort.ext.get<AudioPortExt::Tag::device>();

    if (isUsbDevice(devicePortExt.device)) {
        return getUsbProfiles(dynamicDeviceAudioPort);
    }

    LOG(VERBOSE) << __func__ << " unsupported " << dynamicDeviceAudioPort.toString();
    return {};
}

std::optional<struct HdmiParameters> Platform::getHdmiParameters(
        const ::aidl::android::media::audio::common::AudioDevice& device) const {
    const auto& addressTag = device.address.getTag();
    if (addressTag != AudioDeviceAddress::Tag::id ||
        device.address.get<AudioDeviceAddress::Tag::id>().empty()) {
        LOG(ERROR) << __func__ << ": no hdmi address controller/stream provided for the device"
                   << device.toString();
        return std::nullopt;
    }
    const auto hdmiAddress = device.address.get<AudioDeviceAddress::Tag::id>();
    int controller = -1;
    int stream = -1;

    int status = std::sscanf(hdmiAddress.c_str(), "controller=%d;stream=%d", &controller, &stream);
    if (status != 2) {
        LOG(ERROR) << __func__ << ": failed to extract HDMI parameter from device"
                   << device.toString();
        return std::nullopt;
    }
    pal_device_id_t deviceId = PAL_DEVICE_OUT_AUX_DIGITAL;
    LOG(DEBUG) << __func__ << " controller " << controller << " stream " << stream;
    if (stream) {
        deviceId = PAL_DEVICE_OUT_AUX_DIGITAL_1;
        LOG(DEBUG) << __func__ << " override palDevice with PAL_DEVICE_OUT_AUX_DIGITAL_1";
    }
    struct HdmiParameters hdmiParam = {
            .controller = controller, .stream = stream, .deviceId = deviceId};
    return hdmiParam;
}

bool Platform::handleDeviceConnectionChange(const AudioPort& deviceAudioPort,
                                            const bool isConnect) const {
    const auto& devicePortExt = deviceAudioPort.ext.get<AudioPortExt::Tag::device>();

    auto& audioDeviceDesc = devicePortExt.device.type;
    const auto palDeviceId = PlatformConverter::getPalDeviceId(audioDeviceDesc);
    if (palDeviceId == PAL_DEVICE_OUT_MIN) {
        return false;
    }

    void* v = nullptr;
    const auto deviceConnection = std::make_unique<pal_param_device_connection_t>();
    if (!deviceConnection) {
        LOG(ERROR) << __func__ << ": allocation failed ";
        return false;
    }

    deviceConnection->connection_state = isConnect;
    deviceConnection->id = palDeviceId;

    if (isUsbDevice(devicePortExt.device)) {
        const auto& addressTag = devicePortExt.device.address.getTag();
        if (addressTag != AudioDeviceAddress::Tag::alsa) {
            LOG(ERROR) << __func__ << ": no alsa address provided for the AudioPort"
                       << deviceAudioPort.toString();
            return false;
        }
        const auto& deviceAddressAlsa =
                devicePortExt.device.address.get<AudioDeviceAddress::Tag::alsa>();
        const auto cardId = deviceAddressAlsa[0];
        const auto deviceId = deviceAddressAlsa[1];
        deviceConnection->device_config.usb_addr.card_id = cardId;
        deviceConnection->device_config.usb_addr.device_num = deviceId;

    } else if (isHdmiDevice(devicePortExt.device)) {
        if (auto result = getHdmiParameters(devicePortExt.device)) {
            deviceConnection->device_config.dp_config.controller = result->controller;
            deviceConnection->device_config.dp_config.stream = result->stream;
            deviceConnection->id = result->deviceId;
        } else {
            return false;
        }
    }

    v = deviceConnection.get();
    if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_DEVICE_CONNECTION, v,
                                      sizeof(pal_param_device_connection_t));
        ret != 0) {
        LOG(ERROR) << __func__ << ": pal set param failed for PAL_PARAM_ID_DEVICE_CONNECTION";
        return false;
    }
    LOG(INFO) << __func__ << devicePortExt.device.toString()
              << (isConnect ? ": connected" : "disconnected");

    return true;
}

void Platform::setWFDProxyChannels(const uint32_t numProxyChannels) noexcept {
    mWFDProxyChannels = numProxyChannels;
    pal_param_proxy_channel_config_t paramProxyChannelConfig{.num_proxy_channels =
                                                                     mWFDProxyChannels};
    if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_PROXY_CHANNEL_CONFIG, &paramProxyChannelConfig,
                                      sizeof(pal_param_proxy_channel_config_t));
        ret) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_PROXY_CHANNEL_CONFIG failed: " << ret;
        return;
    }
}

uint32_t Platform::getWFDProxyChannels() const noexcept {
    return mWFDProxyChannels;
}

void Platform::updateUHQA(const bool enable) noexcept {
    mIsUHQAEnabled = enable;
    pal_param_uhqa_t paramUHQAFlags{.uhqa_state = mIsUHQAEnabled};
    if (int32_t ret =
                ::pal_set_param(PAL_PARAM_ID_UHQA_FLAG, &paramUHQAFlags, sizeof(pal_param_uhqa_t));
        ret) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_UHQA_FLAG failed: " << ret;
        return;
    }
    return;
}

bool Platform::isUHQAEnabled() const noexcept {
    return mIsUHQAEnabled;
}

void Platform::setFTMSpeakerProtectionMode(uint32_t const heatUpTime, uint32_t const runTime,
                                           bool const isFactoryTest, bool const isValidationMode,
                                           bool const isDynamicCalibration) const noexcept {
    pal_spkr_prot_payload spPayload{
            .spkrHeatupTime = heatUpTime, .operationModeRunTime = runTime,
    };

    if (isFactoryTest)
        spPayload.operationMode = PAL_SP_MODE_FACTORY_TEST;
    else if (isValidationMode)
        spPayload.operationMode = PAL_SP_MODE_V_VALIDATION;
    else if (isDynamicCalibration)
        spPayload.operationMode = PAL_SP_MODE_DYNAMIC_CAL;
    else
        return;

    if (int32_t ret =
                ::pal_set_param(PAL_PARAM_ID_SP_MODE, &spPayload, sizeof(pal_spkr_prot_payload));
        ret) {
        LOG(ERROR) << ": PAL_PARAM_ID_SP_MODE failed, ret:" << ret;
        return;
    }
}

std::optional<std::string> Platform::getFTMResult() const noexcept {
    char ftmValue[255];
    size_t dataSize = 0;
    if (int32_t ret = ::pal_get_param(PAL_PARAM_ID_SP_MODE, reinterpret_cast<void**>(&ftmValue),
                                      &dataSize, nullptr);
        (ret || dataSize <= 0)) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_SP_MODE failed, ret:" << ret
                   << ", data size:" << dataSize;
        return std::nullopt;
    }
    return std::string(ftmValue, dataSize);
}

std::optional<std::string> Platform::getSpeakerCalibrationResult() const noexcept {
    char calValue[255];
    size_t dataSize = 0;
    if (int32_t ret = ::pal_get_param(PAL_PARAM_ID_SP_GET_CAL, reinterpret_cast<void**>(&calValue),
                                      &dataSize, nullptr);
        (ret || dataSize <= 0)) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_SP_GET_CAL failed, ret:" << ret
                   << ", data size:" << dataSize;
        return std::nullopt;
    }
    return std::string(calValue, dataSize);
}

void Platform::updateScreenRotation(const IModule::ScreenRotation in_rotation) noexcept {
    pal_param_device_rotation_t paramDeviceRotation{};

    auto notifyDeviceRotation = [&]() -> void {
        if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_DEVICE_ROTATION, &paramDeviceRotation,
                                          sizeof(pal_param_device_rotation_t));
            ret) {
            LOG(ERROR) << ": PAL_PARAM_ID_DEVICE_ROTATION failed";
        }
    };

    if (in_rotation == IModule::ScreenRotation::DEG_270 &&
        mCurrentScreenRotation != IModule::ScreenRotation::DEG_270) {
        /* Device rotated from normal position to inverted landscape. */
        paramDeviceRotation.rotation_type = PAL_SPEAKER_ROTATION_RL;
        notifyDeviceRotation();
    } else if (in_rotation != IModule::ScreenRotation::DEG_270 &&
               mCurrentScreenRotation == IModule::ScreenRotation::DEG_270) {
        /* Phone was in inverted landspace and now is changed to portrait or inverted portrait. */
        paramDeviceRotation.rotation_type = PAL_SPEAKER_ROTATION_LR;
        notifyDeviceRotation();
    }

    mCurrentScreenRotation = in_rotation;
}

IModule::ScreenRotation Platform::getCurrentScreenRotation() const noexcept {
    return mCurrentScreenRotation;
}

void Platform::setHapticsVolume(const float hapticsVolume) const noexcept {
    auto data = makePalVolumes({hapticsVolume});
    if (data.empty()) {
        LOG(ERROR) << __func__ << ": failed to configure haptics volume";
        return;
    }
    auto payloadPtr = reinterpret_cast<pal_volume_data*>(data.data());
    if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_HAPTICS_VOLUME, payloadPtr, data.size()); ret) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_HAPTICS_VOLUME failed: " << ret;
        return;
    }
}

void Platform::setHapticsIntensity(const int hapticsIntensity) const noexcept {
    pal_param_haptics_intensity_t paramHapticsIntensity{.intensity = hapticsIntensity};
    if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_HAPTICS_INTENSITY, &paramHapticsIntensity,
                                      sizeof(pal_param_haptics_intensity_t));
        ret) {
        LOG(ERROR) << __func__ << ": PAL_PARAM_ID_HAPTICS_INTENSITY failed: " << ret;
        return;
    }
}

bool Platform::setVendorParameters(
        const std::vector<::aidl::android::hardware::audio::core::VendorParameter>& in_parameters,
        bool in_async) {
    std::string kvpairs = getkvPairsForVendorParameter(in_parameters);
    if (!kvpairs.empty()) {
        setBluetoothParameters(kvpairs.c_str());
    }
    return true;
}

bool Platform::setBluetoothParameters(const char* kvpairs) {
    struct str_parms* parms = NULL;
    int ret = 0, val = 0;
    char value[256];
    LOG(VERBOSE) << __func__ << "kvpairs " << kvpairs;
    parms = str_parms_create_str(kvpairs);
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_RECONFIG_A2DP, value, sizeof(value));
    if (ret >= 0) {
        pal_param_bta2dp_t param_bt_a2dp;
        param_bt_a2dp.reconfig = true;

        LOG(VERBOSE) << __func__ << " BT A2DP Reconfig command received";
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_RECONFIG, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
    }
    ret = str_parms_get_str(parms, "A2dpSuspended", value, sizeof(value));
    if (ret >= 0) {
        pal_param_bta2dp_t param_bt_a2dp;
        param_bt_a2dp.is_suspend_setparam = true;

        if (strncmp(value, "true", 4) == 0)
            param_bt_a2dp.a2dp_suspended = true;
        else
            param_bt_a2dp.a2dp_suspended = false;

        param_bt_a2dp.dev_id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;

        LOG(VERBOSE) << __func__ << " BT A2DP Suspended = " << value;
        std::unique_lock<std::mutex> guard(AudioExtension::reconfig_wait_mutex_);
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_SUSPENDED, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
    }
    ret = str_parms_get_str(parms, "TwsChannelConfig", value, sizeof(value));
    if (ret >= 0) {
        pal_param_bta2dp_t param_bt_a2dp;

        LOG(VERBOSE) << __func__ << " Setting tws channel mode to = " << value;
        if (!(strncmp(value, "mono", strlen(value))))
            param_bt_a2dp.is_tws_mono_mode_on = true;
        else if (!(strncmp(value, "dual-mono", strlen(value))))
            param_bt_a2dp.is_tws_mono_mode_on = false;
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_TWS_CONFIG, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
    }
    ret = str_parms_get_str(parms, "LEAMono", value, sizeof(value));
    if (ret >= 0) {
        pal_param_bta2dp_t param_bt_a2dp;

        LOG(VERBOSE) << __func__ << " Setting LC3 channel mode to = " << value;
        if (!(strncmp(value, "true", strlen(value))))
            param_bt_a2dp.is_lc3_mono_mode_on = true;
        else
            param_bt_a2dp.is_lc3_mono_mode_on = false;
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_LC3_CONFIG, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
    }

    /* SCO parameters */
    ret = str_parms_get_str(parms, "BT_SCO", value, sizeof(value));
    if (ret >= 0) {
        pal_param_btsco_t param_bt_sco;
        memset(&param_bt_sco, 0, sizeof(pal_param_btsco_t));
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0) {
            param_bt_sco.bt_sco_on = true;
        } else {
            param_bt_sco.bt_sco_on = false;
        }

        LOG(VERBOSE) << __func__ << " BTSCO on = " << param_bt_sco.bt_sco_on;
        ret = pal_set_param(PAL_PARAM_ID_BT_SCO, (void*)&param_bt_sco, sizeof(pal_param_btsco_t));
#if 0
        if (param_bt_sco.bt_sco_on == true) {
            if (crs_device.size() == 0) {
                crs_device.insert(AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET);
                voice_->RouteStream(crs_device);
            } else {
                pos = std::find(crs_device.begin(), crs_device.end(), AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET);
                if (pos != crs_device.end()) {
                    AHAL_INFO("same device has added");
                } else {
                    crs_device.insert(AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET);
                    voice_->RouteStream({AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET});
                }
            }
        } else if (param_bt_sco.bt_sco_on == false) {
            pos = std::find(crs_device.begin(), crs_device.end(), AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET);
            if (pos != crs_device.end()) {
                crs_device.erase(pos);
                if (crs_device.size() >= 1) {
                    voice_->RouteStream(crs_device);
                    AHAL_INFO("route to device 0x%x", AudioExtn::get_device_types(crs_device));
                } else {
                    crs_device.clear();
                    voice_->RouteStream({AUDIO_DEVICE_OUT_SPEAKER});
                }
            }
        }
#endif
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_BT_SCO_WB, value, sizeof(value));
    if (ret >= 0) {
        pal_param_btsco_t param_bt_sco = {};
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0)
            param_bt_sco.bt_wb_speech_enabled = true;
        else
            param_bt_sco.bt_wb_speech_enabled = false;

        LOG(VERBOSE) << __func__ << " BTSCO WB mode = " << param_bt_sco.bt_wb_speech_enabled;
        ret = pal_set_param(PAL_PARAM_ID_BT_SCO_WB, (void*)&param_bt_sco,
                            sizeof(pal_param_btsco_t));
    }
    ret = str_parms_get_str(parms, "bt_swb", value, sizeof(value));
    if (ret >= 0) {
        pal_param_btsco_t param_bt_sco = {};

        val = atoi(value);
        param_bt_sco.bt_swb_speech_mode = val;
        LOG(VERBOSE) << __func__ << " BTSCO SWB mode = " << val;
        ret = pal_set_param(PAL_PARAM_ID_BT_SCO_SWB, (void*)&param_bt_sco,
                            sizeof(pal_param_btsco_t));
    }

    ret = str_parms_get_str(parms, "bt_ble", value, sizeof(value));
    if (ret >= 0) {
        pal_param_btsco_t param_bt_sco = {};
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0) {
            bt_lc3_speech_enabled = true;

            // turn off wideband, super-wideband
            param_bt_sco.bt_wb_speech_enabled = false;
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_WB, (void*)&param_bt_sco,
                                sizeof(pal_param_btsco_t));

            param_bt_sco.bt_swb_speech_mode = 0xFFFF;
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_SWB, (void*)&param_bt_sco,
                                sizeof(pal_param_btsco_t));
        } else {
            bt_lc3_speech_enabled = false;
            param_bt_sco.bt_lc3_speech_enabled = false;
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_LC3, (void*)&param_bt_sco,
                                sizeof(pal_param_btsco_t));

            // clear btsco_lc3_cfg to avoid stale and partial cfg being used in next round
            memset(&btsco_lc3_cfg, 0, sizeof(btsco_lc3_cfg_t));
        }
        LOG(VERBOSE) << __func__ << " BTSCO LC3 mode = " << bt_lc3_speech_enabled;
    }

    ret = str_parms_get_str(parms, "bt_lc3_swb", value, sizeof(value));
    if (ret >= 0) {
        pal_param_btsco_t param_bt_sco_swb = {};
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0) {
            // turn off wideband, super-wideband
            param_bt_sco_swb.bt_wb_speech_enabled = false;
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_WB, (void*)&param_bt_sco_swb,
                                sizeof(pal_param_btsco_t));

            param_bt_sco_swb.bt_swb_speech_mode = 0xFFFF;
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_SWB, (void*)&param_bt_sco_swb,
                                sizeof(pal_param_btsco_t));

            char streamMap[PAL_LC3_MAX_STRING_LEN] = "(0, 0, M, 0, 1, M)";
            char vendor[PAL_LC3_MAX_STRING_LEN] = "00,00,00,00,00,00,00,00,00,02,00,00,00,0A,00,00";
            param_bt_sco_swb.bt_lc3_speech_enabled = true;
            param_bt_sco_swb.lc3_cfg.num_blocks = 1;
            param_bt_sco_swb.lc3_cfg.rxconfig_index = LC3_SWB_CODEC_CONFIG_INDEX;
            param_bt_sco_swb.lc3_cfg.txconfig_index = LC3_SWB_CODEC_CONFIG_INDEX;
            param_bt_sco_swb.lc3_cfg.api_version = 21;
            param_bt_sco_swb.lc3_cfg.mode = LC3_HFP_TRANSIT_MODE;
            strlcpy(param_bt_sco_swb.lc3_cfg.streamMap, streamMap, PAL_LC3_MAX_STRING_LEN);
            strlcpy(param_bt_sco_swb.lc3_cfg.vendor, vendor, PAL_LC3_MAX_STRING_LEN);

            // AHAL_INFO("BTSCO LC3 SWB mode = on, sending..");
            LOG(VERBOSE) << __func__ << " BTSCO LC3 SWB mode = on, sending..";
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_LC3, (void*)&param_bt_sco_swb,
                                sizeof(pal_param_btsco_t));
        } else {
            param_bt_sco_swb.bt_lc3_speech_enabled = false;

            LOG(VERBOSE) << __func__ << " BTSCO LC3 SWB mode = off, sending..";
            ret = pal_set_param(PAL_PARAM_ID_BT_SCO_LC3, (void*)&param_bt_sco_swb,
                                sizeof(pal_param_btsco_t));
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_BT_NREC, value, sizeof(value));
    if (ret >= 0) {
        pal_param_btsco_t param_bt_sco = {};
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0) {
            // AHAL_INFO("BTSCO NREC mode = ON");
            LOG(VERBOSE) << __func__ << " BTSCO NREC mode = ON";
            param_bt_sco.bt_sco_nrec = true;
        } else {
            LOG(VERBOSE) << __func__ << " BTSCO NREC mode = OFF";
            param_bt_sco.bt_sco_nrec = false;
        }
        ret = pal_set_param(PAL_PARAM_ID_BT_SCO_NREC, (void*)&param_bt_sco,
                            sizeof(pal_param_btsco_t));
    }

    for (auto& key : lc3_reserved_params) {
        ret = str_parms_get_str(parms, key, value, sizeof(value));
        if (ret < 0) continue;

        if (!strcmp(key, "Codec") && (!strcmp(value, "LC3"))) {
            btsco_lc3_cfg.fields_map |= LC3_CODEC_BIT;
        } else if (!strcmp(key, "StreamMap")) {
            strlcpy(btsco_lc3_cfg.streamMap, value, PAL_LC3_MAX_STRING_LEN);
            btsco_lc3_cfg.fields_map |= LC3_STREAM_MAP_BIT;
        } else if (!strcmp(key, "FrameDuration")) {
            btsco_lc3_cfg.frame_duration = atoi(value);
            btsco_lc3_cfg.fields_map |= LC3_FRAME_DURATION_BIT;
        } else if (!strcmp(key, "Blocks_forSDU")) {
            btsco_lc3_cfg.num_blocks = atoi(value);
            btsco_lc3_cfg.fields_map |= LC3_BLOCKS_FORSDU_BIT;
        } else if (!strcmp(key, "rxconfig_index")) {
            btsco_lc3_cfg.rxconfig_index = atoi(value);
            btsco_lc3_cfg.fields_map |= LC3_RXCFG_IDX_BIT;
        } else if (!strcmp(key, "txconfig_index")) {
            btsco_lc3_cfg.txconfig_index = atoi(value);
            btsco_lc3_cfg.fields_map |= LC3_TXCFG_IDX_BIT;
        } else if (!strcmp(key, "version")) {
            btsco_lc3_cfg.api_version = atoi(value);
            btsco_lc3_cfg.fields_map |= LC3_VERSION_BIT;
        } else if (!strcmp(key, "vendor")) {
            strlcpy(btsco_lc3_cfg.vendor, value, PAL_LC3_MAX_STRING_LEN);
            btsco_lc3_cfg.fields_map |= LC3_VENDOR_BIT;
        }
    }

    if (((btsco_lc3_cfg.fields_map & LC3_BIT_MASK) == LC3_BIT_VALID) &&
        (bt_lc3_speech_enabled == true)) {
        pal_param_btsco_t param_bt_sco = {};
        param_bt_sco.bt_lc3_speech_enabled = bt_lc3_speech_enabled;
        param_bt_sco.lc3_cfg.frame_duration = btsco_lc3_cfg.frame_duration;
        param_bt_sco.lc3_cfg.num_blocks = btsco_lc3_cfg.num_blocks;
        param_bt_sco.lc3_cfg.rxconfig_index = btsco_lc3_cfg.rxconfig_index;
        param_bt_sco.lc3_cfg.txconfig_index = btsco_lc3_cfg.txconfig_index;
        param_bt_sco.lc3_cfg.api_version = btsco_lc3_cfg.api_version;
        param_bt_sco.lc3_cfg.mode = LC3_BROADCAST_TRANSIT_MODE;
        strlcpy(param_bt_sco.lc3_cfg.streamMap, btsco_lc3_cfg.streamMap, PAL_LC3_MAX_STRING_LEN);
        strlcpy(param_bt_sco.lc3_cfg.vendor, btsco_lc3_cfg.vendor, PAL_LC3_MAX_STRING_LEN);

        LOG(VERBOSE) << __func__ << " BTSCO LC3 mode = on, sending..";
        ret = pal_set_param(PAL_PARAM_ID_BT_SCO_LC3, (void*)&param_bt_sco,
                            sizeof(pal_param_btsco_t));

        memset(&btsco_lc3_cfg, 0, sizeof(btsco_lc3_cfg_t));
    }
    ret = str_parms_get_str(parms, "A2dpCaptureSuspend", value, sizeof(value));
    if (ret >= 0) {
        pal_param_bta2dp_t param_bt_a2dp;
        param_bt_a2dp.is_suspend_setparam = true;

        if (strncmp(value, "true", 4) == 0)
            param_bt_a2dp.a2dp_capture_suspended = true;
        else
            param_bt_a2dp.a2dp_capture_suspended = false;

        param_bt_a2dp.dev_id = PAL_DEVICE_IN_BLUETOOTH_A2DP;

        LOG(VERBOSE) << __func__ << " BT A2DP Capture Suspended " << value << "command received";
        std::unique_lock<std::mutex> guard(AudioExtension::reconfig_wait_mutex_);
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_CAPTURE_SUSPENDED, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
    }
    ret = str_parms_get_str(parms, "LeAudioSuspended", value, sizeof(value));
    if (ret >= 0) {
        pal_param_bta2dp_t param_bt_a2dp;
        param_bt_a2dp.is_suspend_setparam = true;

        if (strcmp(value, "true") == 0) {
            param_bt_a2dp.a2dp_suspended = true;
            param_bt_a2dp.a2dp_capture_suspended = true;
        } else {
            param_bt_a2dp.a2dp_suspended = false;
            param_bt_a2dp.a2dp_capture_suspended = false;
        }

        LOG(INFO) << __func__ << " BT LEA Suspended = ," << value << " command received";
        // Synchronize the suspend/resume calls from setparams and reconfig_cb
        std::unique_lock<std::mutex> guard(AudioExtension::reconfig_wait_mutex_);
        param_bt_a2dp.dev_id = PAL_DEVICE_OUT_BLUETOOTH_BLE;
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_SUSPENDED, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));

        param_bt_a2dp.dev_id = PAL_DEVICE_IN_BLUETOOTH_BLE;
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_CAPTURE_SUSPENDED, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
        param_bt_a2dp.dev_id = PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST;
        ret = pal_set_param(PAL_PARAM_ID_BT_A2DP_SUSPENDED, (void*)&param_bt_a2dp,
                            sizeof(pal_param_bta2dp_t));
    }
    return true;
}

bool Platform::setParameter(const std::string& key, const std::string& value) {
    // Todo check for validity of key
    const auto & [ first, second ] = mParameters.insert_or_assign(key, value);
    LOG(VERBOSE) << __func__ << " platform parameter with key:" << key << " "
                 << (second ? "inserted" : "re-assigned") << " with value:" << value;
    return true;
}

std::string Platform::getParameter(const std::string& key) const {
    if (mParameters.find(key) != mParameters.cend()) {
        return mParameters.at(key);
    }
    return "";
}

bool Platform::isFormatTypePCM(const AudioFormatDescription& f) const noexcept {
    if (f.type == AudioFormatType::PCM) {
        return true;
    }
    return false;
}

bool Platform::isOutputDevice(const AudioDevice& d) const noexcept {
    if (d.type.type >= AudioDeviceType::OUT_DEFAULT) {
        return true;
    }
    return false;
}

bool Platform::isInputDevice(const AudioDevice& d) const noexcept {
    if (d.type.type < AudioDeviceType::OUT_DEFAULT) {
        return true;
    }
    return false;
}

bool Platform::isUsbDevice(const AudioDevice& d) const noexcept {
    if (d.type.connection == AudioDeviceDescription::CONNECTION_USB) {
        return true;
    }
    return false;
}

bool Platform::isHdmiDevice(const AudioDevice& d) const noexcept {
    if (d.type.connection == AudioDeviceDescription::CONNECTION_HDMI) {
        return true;
    }
    return false;
}

bool Platform::isBluetoothDevice(const AudioDevice& d) const noexcept {
    if (d.type.connection == AudioDeviceDescription::CONNECTION_BT_A2DP ||
        d.type.connection == AudioDeviceDescription::CONNECTION_BT_LE) {
        return true;
    }
    return false;
}

bool Platform::isSoundCardUp() const noexcept {
    if (mSndCardStatus == CARD_STATUS_ONLINE) {
        return true;
    }
    return false;
}

bool Platform::isSoundCardDown() const noexcept {
    if (mSndCardStatus == CARD_STATUS_OFFLINE || mSndCardStatus == CARD_STATUS_STANDBY) {
        return true;
    }
    return false;
}

uint32_t Platform::getBluetoothLatencyMs(const std::vector<AudioDevice>& bluetoothDevices) {
    pal_param_bta2dp_t btConfig{};
    for (const auto& device : bluetoothDevices) {
        btConfig.dev_id = PlatformConverter::getPalDeviceId(device.type);
        // first bluetooth device
        if (btConfig.dev_id == PAL_DEVICE_OUT_BLUETOOTH_A2DP ||
            btConfig.dev_id == PAL_DEVICE_OUT_BLUETOOTH_BLE ||
            btConfig.dev_id == PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST) {
            if (getBtConfig(&btConfig)) {
                return btConfig.latency;
            }
        }
    }
    return 0;
}

bool Platform::isA2dpSuspended() {
    int ret = 0;
    size_t bt_param_size = 0;
    pal_param_bta2dp_t *param_bt_a2dp_ptr, param_bt_a2dp;
    param_bt_a2dp_ptr = &param_bt_a2dp;
    param_bt_a2dp_ptr->dev_id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
    ret = pal_get_param(PAL_PARAM_ID_BT_A2DP_SUSPENDED, (void**)&param_bt_a2dp_ptr, &bt_param_size,
                        nullptr);
    if (!ret && bt_param_size && param_bt_a2dp_ptr && !param_bt_a2dp_ptr->a2dp_suspended) {
        LOG(DEBUG) << __func__ << " A2dp suspended " << param_bt_a2dp_ptr->a2dp_suspended;
        return param_bt_a2dp_ptr->a2dp_suspended;
    }
    return true;
}

PlaybackRateStatus Platform::setPlaybackRate(
        pal_stream_handle_t* handle, const Usecase& tag,
        const ::aidl::android::media::audio::common::AudioPlaybackRate& playbackRate) {
    if (!isValidPlaybackRate(playbackRate)) {
        return PlaybackRateStatus::ILLEGAL_ARGUMENT;
    }

    if (!usecaseSupportsOffloadSpeed(tag)) {
        return PlaybackRateStatus::UNSUPPORTED;
    }

    if (!handle) {
        LOG(DEBUG) << __func__ << " stream inactive ";
        return PlaybackRateStatus::SUCCESS;
    }

    auto allocSize = sizeof(pal_param_payload) + sizeof(pal_param_playback_rate_t);
    auto payload =
            VALUE_OR_EXIT(allocate<pal_param_payload>(allocSize), PlaybackRateStatus::UNSUPPORTED);
    pal_param_payload* payloadPtr = payload.get();
    payloadPtr->payload_size = sizeof(pal_param_playback_rate_t);

    auto palPlaybackRatePtr = reinterpret_cast<pal_param_playback_rate_t*>(payloadPtr->payload);
    palPlaybackRatePtr->speed = playbackRate.speed;
    palPlaybackRatePtr->pitch = playbackRate.pitch;

    if (auto ret = pal_stream_set_param(handle, PAL_PARAM_ID_TIMESTRETCH_PARAMS, payloadPtr); ret) {
        LOG(ERROR) << __func__ << " failed to set " << playbackRate.toString();
        return PlaybackRateStatus::UNSUPPORTED;
    }
    return PlaybackRateStatus::SUCCESS;
}

int Platform::getRecommendedLatencyModes(
          std::vector<::aidl::android::media::audio::common::AudioLatencyMode>* _aidl_return) {

     size_t size;
     int ret = 0;
     auto palLatencyModeInfo = std::make_unique<pal_param_latency_mode_t>();
     if (!palLatencyModeInfo) {
         LOG(ERROR) << __func__ << ": allocation failed ";
         return -ENOMEM;
     }

     palLatencyModeInfo->dev_id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
     palLatencyModeInfo->num_modes = PAL_MAX_LATENCY_MODES;
     void *palLatencyModeInfoPtr = palLatencyModeInfo.get();

     ret = pal_get_param(PAL_PARAM_ID_LATENCY_MODE,
                        (void **)&palLatencyModeInfoPtr, &size, nullptr);
     if (ret) {
         LOG(ERROR) << __func__ << " get param latency mode failed";
         return ret;
     }

     LOG(VERBOSE) << __func__ << " actual modes returned: " << palLatencyModeInfo->num_modes;

     for (int count = 0; count < palLatencyModeInfo->num_modes; count++)
     {
        _aidl_return->push_back(
         (::aidl::android::media::audio::common::AudioLatencyMode)palLatencyModeInfo->modes[count]);
     }

     return ret;
}

int Platform::setLatencyMode(uint32_t mode) {

     int ret = 0;
     auto palLatencyModeInfo = std::make_unique<pal_param_latency_mode_t>();
     if (!palLatencyModeInfo) {
         LOG(ERROR) << __func__ << ": allocation failed ";
         return -ENOMEM;
     }

     palLatencyModeInfo->dev_id = PAL_DEVICE_OUT_BLUETOOTH_A2DP;
     palLatencyModeInfo->num_modes = 1;
     palLatencyModeInfo->modes[0] = (uint32_t)mode;

     ret = pal_set_param(PAL_PARAM_ID_LATENCY_MODE,
              (void *)palLatencyModeInfo.get(), sizeof(pal_param_latency_mode_t));

     return ret;
}

std::optional<std::pair<audio_format_t, audio_format_t>> Platform::requiresBufferReformat(
        const AudioPortConfig& portConfig) {
    const auto& audioFormat = portConfig.format.value();

    if (audioFormat.pcm == PcmType::FLOAT_32_BIT) {
        return std::make_pair(AUDIO_FORMAT_PCM_FLOAT, AUDIO_FORMAT_PCM_32_BIT);
    }
    return std::nullopt;
}

// start of private
bool Platform::getBtConfig(pal_param_bta2dp_t* bTConfig) {
    if (bTConfig == nullptr) {
        LOG(ERROR) << __func__ << " invalid bt config";
        return false;
    }
    size_t payloadSize = 0;
    if (int32_t ret = ::pal_get_param(PAL_PARAM_ID_BT_A2DP_ENCODER_LATENCY,
                                      reinterpret_cast<void**>(&bTConfig), &payloadSize, nullptr);
        ret) {
        LOG(ERROR) << __func__ << " failure in PAL_PARAM_ID_BT_A2DP_ENCODER_LATENCY, ret :" << ret;
        return false;
    }
    if (payloadSize == 0) {
        LOG(ERROR) << __func__ << " empty payload size!!!";
        return false;
    }
    return true;
}
// end of private

std::string Platform::toString() const {
    std::ostringstream os;
    os << " === platform start ===" << std::endl;
    os << "sound card status: " << mSndCardStatus << std::endl;
    for (const auto & [ key, value ] : mParameters) {
        os << key << "=>" << value << std::endl;
    }
    os << PlatformConverter::toString() << std::endl;
    os << " === platform end ===" << std::endl;
    return os.str();
}

// static
int Platform::palGlobalCallback(uint32_t event_id, uint32_t* event_data, uint64_t cookie) {
    auto platform = reinterpret_cast<Platform*>(cookie);
    switch (event_id) {
        case PAL_SND_CARD_STATE:
            platform->mSndCardStatus = static_cast<card_status_t>(*event_data);
            LOG(INFO) << __func__ << " card status changed to " << platform->mSndCardStatus;
            break;
        default:
            LOG(ERROR) << __func__ << " invalid event id" << event_id;
            return -EINVAL;
    }
    return 0;
}

Platform::Platform() {
    if (int32_t ret = pal_init(); ret) {
        LOG(ERROR) << __func__ << "pal init failed!!! ret:" << ret;
        return;
    }
    LOG(VERBOSE) << __func__ << " pal init successful";
    if (int32_t ret =
                pal_register_global_callback(&palGlobalCallback, reinterpret_cast<uint64_t>(this));
        ret) {
        LOG(ERROR) << __func__ << "pal register global callback failed!!! ret:" << ret;
        return;
    }
    mSndCardStatus = CARD_STATUS_ONLINE;
    LOG(VERBOSE) << __func__ << " pal register global callback successful";
    mOffloadSpeedSupported = property_get_bool("vendor.audio.offload.playspeed", true);
}

// static
Platform& Platform::getInstance() {
    static const auto kPlatform = []() {
        std::unique_ptr<Platform> platform{new Platform()};
        return std::move(platform);
    }();
    return *(kPlatform.get());
}

} // namespace qti::audio::core
