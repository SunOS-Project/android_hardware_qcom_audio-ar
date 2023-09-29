/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_NDEBUG 0
#define LOG_TAG "AHAL_Platform"

#include <qti-audio-core/Platform.h>
#include <qti-audio-core/AudioUsecase.h>
#include <Utils.h>
#include <android-base/logging.h>

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
using ::aidl::android::hardware::audio::common::isBitPositionFlagSet;

namespace qti::audio::core {

size_t Platform::getIOBufferSizeInFrames(
    const ::aidl::android::media::audio::common::AudioPortConfig& mixPortConfig)
    const {
    Usecase tag = getUsecaseTag(mixPortConfig);
    size_t numFrames = 0;
    if (tag == Usecase::DEEP_BUFFER_PLAYBACK) {
        numFrames = DeepBufferPlayback::kPeriodSize;
    } else if (tag == Usecase::LOW_LATENCY_PLAYBACK) {
        numFrames = LowLatencyPlayback::kPeriodSize;
    } else if (tag == Usecase::PCM_RECORD) {
        numFrames = PcmRecord::getMinFrames(mixPortConfig);
    } else if (tag == Usecase::COMPRESS_OFFLOAD_PLAYBACK) {
        const size_t numBytes =
            CompressPlayback::getPeriodBufferSize(mixPortConfig.format.value());
        constexpr size_t compressFrameSize = 1;
        numFrames = numBytes / compressFrameSize;
    } else if (tag == Usecase::COMPRESS_CAPTURE) {
        numFrames =
            CompressCapture::getPeriodBufferSize(mixPortConfig.format.value());
    }
    LOG(VERBOSE) << __func__
                 << " IOBufferSizeInFrames:" << std::to_string(numFrames)
                 << " for " << getName(tag);
    return numFrames;
}

size_t Platform::getMinimumStreamSizeFrames(
    const std::vector<AudioPortConfig*>& sources,
    const std::vector<AudioPortConfig*>& sinks) const {
    if (sources.size() > 1) {
        LOG(WARNING) << __func__
                     << " unable to decide the minimum stream size for sources "
                        "more than one; actual size:"
                     << sources.size();
        return 0;
    }
    // choose the mix port
    auto isMixPortConfig = [](const auto& audioPortConfig) {
        return audioPortConfig.ext.getTag() == AudioPortExt::Tag::mix;
    };

    const auto& mixPortConfig =
        isMixPortConfig(*sources.at(0)) ? *(sources.at(0)) : *(sinks.at(0));
    return getIOBufferSizeInFrames(mixPortConfig);
}

std::unique_ptr<pal_stream_attributes> Platform::getPalStreamAttributes(
    const AudioPortConfig& portConfig, const bool isInput) const {
    const auto& audioFormat = portConfig.format.value();
    const auto palFormat = mTypeConverter.getPalFormatId(audioFormat);
    if (palFormat == PAL_AUDIO_FMT_COMPRESSED_RANGE_END) {
        return nullptr;
    }

    const auto& audioChannelLayout = portConfig.channelMask.value();
    auto palChannelInfo = mTypeConverter.getPalChannelInfoForChannelCount(
        getChannelCount(audioChannelLayout));
    if (palChannelInfo == nullptr) {
        LOG(ERROR) << __func__
                   << " failed to find corresponding pal channel info for "
                   << audioChannelLayout.toString();
        return nullptr;
    }
    const auto sampleRate = portConfig.sampleRate.value().value;
    if (!sampleRate) {
        LOG(ERROR) << __func__ << " invalid sample rate "
                   << std::to_string(sampleRate);
        return nullptr;
    }

    auto attributes = std::make_unique<pal_stream_attributes>();
    auto bitWidth = mTypeConverter.getBitWidthForAidlPCM(audioFormat);
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

std::unique_ptr<pal_stream_attributes> Platform::getDefaultTelephonyAttributes()
    const {
    auto attributes = std::make_unique<pal_stream_attributes>();
    auto inChannelInfo = mTypeConverter.getPalChannelInfoForChannelCount(1);
    auto outChannelInfo = mTypeConverter.getPalChannelInfoForChannelCount(2);
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

void Platform::configurePalDevicesCustomKey(
    std::vector<pal_device>& palDevices, const std::string& key) const {
    for (auto& palDevice : palDevices) {
        strlcpy(palDevice.custom_config.custom_key, key.c_str(), key.size());
    }
    return;
}

std::vector<pal_device> Platform::getPalDevices(
    const std::vector<AudioDevice>& setDevices) const {
    if (setDevices.size() == 0) {
        LOG(ERROR) << __func__
                   << " the set devices is empty";
        return {};
    }
    std::vector<pal_device> palDevices{setDevices.size()};

    size_t i = 0;
    for (auto& device : setDevices) {
        const auto palDeviceId = mTypeConverter.getPalDeviceId(device.type);
        if (palDeviceId == PAL_DEVICE_OUT_MIN) {
            return {};
        }
        palDevices[i].id = palDeviceId;
        palDevices[i].config.sample_rate = kDefaultOutputSampleRate;
        palDevices[i].config.bit_width = kDefaultPCMBidWidth;
        palDevices[i].config.aud_fmt_id = kDefaultPalPCMFormat;

        if (isUsbDevice(device)) {
            const auto& deviceAddress = device.address;
            if (deviceAddress.getTag() != AudioDeviceAddress::Tag::alsa) {
                LOG(ERROR)
                    << __func__
                    << " failed to find alsa address for given usb device "
                    << device.toString();
                return {};
            }
            const auto& deviceAddressAlsa =
                deviceAddress.get<AudioDeviceAddress::Tag::alsa>();
            palDevices[i].address.card_id = deviceAddressAlsa[0];
            palDevices[i].address.device_num = deviceAddressAlsa[0];
        }
        i++;
    }
    return palDevices;
}

std::vector<uint8_t> Platform::getPalVolumeData(
    const std::vector<float>& in_channelVolumes) const {
    const auto volumeSizes = in_channelVolumes.size();
    if (volumeSizes == 0 || volumeSizes > 2) {
        LOG(ERROR) << __func__
                   << "length channel volumes is"
                   << std::to_string(volumeSizes);
        return {};
    }
    const auto dataLength =
        sizeof(pal_volume_data) + sizeof(pal_channel_vol_kv) * volumeSizes;
    auto data = std::vector<uint8_t>(dataLength);
    auto palVolumeData = reinterpret_cast<pal_volume_data*>(data.data());
    palVolumeData->no_of_volpair = volumeSizes;

    size_t i = 0;
    for (auto& f : in_channelVolumes) {
        palVolumeData->volume_pair[i].channel_mask = 0x1 << i;
        palVolumeData->volume_pair[i].vol = f;
        i++;
    }

    return data;
}

std::unique_ptr<pal_buffer_config_t> Platform::getPalBufferConfig(
    const size_t bufferSize, const size_t bufferCount) const {
    auto palBufferConfig = std::make_unique<pal_buffer_config_t>();
    palBufferConfig->buf_size = bufferSize;
    palBufferConfig->buf_count = bufferCount;
    return std::move(palBufferConfig);
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

    const auto& devicePortExt =
        dynamicDeviceAudioPort.ext.get<AudioPortExt::Tag::device>();

    if (!isUsbDevice(devicePortExt.device)) {
        LOG(ERROR) << __func__ << " device is not USB type ";
        return {};
    }
    auto& audioDeviceDesc = devicePortExt.device.type;
    const auto palDeviceId = mTypeConverter.getPalDeviceId(audioDeviceDesc);
    if(palDeviceId == PAL_DEVICE_OUT_MIN){
        return {};
    }

    const auto& addressTag = devicePortExt.device.address.getTag();
    if (addressTag != AudioDeviceAddress::Tag::alsa) {
        LOG(ERROR) << __func__ << ": no alsa address provided for the AudioPort"
                   << dynamicDeviceAudioPort.toString();
        return {};
    }
    const auto& deviceAddressAlsa =
        devicePortExt.device.address.get<AudioDeviceAddress::Tag::alsa>();
    const auto cardId = deviceAddressAlsa[0];
    const auto deviceId = deviceAddressAlsa[1];

    void* v = nullptr;

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

    size_t payload_size = 0;
    deviceCapability->id = palDeviceId;
    deviceCapability->addr.card_id = cardId;
    deviceCapability->addr.device_num = deviceId;
    deviceCapability->config = dynamicMediaConfig.get();
    deviceCapability->is_playback = isOutputDevice(devicePortExt.device);
    v = deviceCapability.get();
    if (int32_t ret = pal_get_param(PAL_PARAM_ID_DEVICE_CAPABILITY, &v,
                                    &payload_size, nullptr);
        ret != 0) {
        LOG(ERROR) << __func__
                   << " PAL get param failed for PAL_PARAM_ID_DEVICE_CAPABILITY"
                   << ret;
        return {};
    }
    if (!dynamicMediaConfig->jack_status) {
        LOG(ERROR) << __func__
                   << " false usb jack status ";
        return {};
    }

    std::vector<AudioProfile> supportedProfiles;
    const auto sampleRatesSupported = [&dynamicMediaConfig]() {
        int i = 0;
        std::vector<int32_t> sampleRates;
        while (i <= MAX_SUPPORTED_SAMPLE_RATES &&
               dynamicMediaConfig->sample_rate[i] != 0) {
            sampleRates.push_back(dynamicMediaConfig->sample_rate[i]);
            ++i;
        }
        return sampleRates;
    }();
    const auto channelsSupported = [&dynamicMediaConfig]() {
        int i = 0;
        std::vector<AudioChannelLayout> channels;
        while (i <= MAX_SUPPORTED_CHANNEL_MASKS &&
               dynamicMediaConfig->mask[i] != 0) {
            // Todo change channels return type in dynamicMediaConfig
            // channels.push_back(dynamicMediaConfig->mask[i]);
            channels.push_back(
                AudioChannelLayout::make<AudioChannelLayout::Tag::layoutMask>(
                    AudioChannelLayout::LAYOUT_STEREO));
            ++i;
        }
        return channels;
    }();
    for (int i = 0;
         i <= MAX_SUPPORTED_FORMATS && dynamicMediaConfig->format[i] != 0;
         ++i) {
        AudioProfile p;
        p.format.type = AudioFormatType::PCM;
        // TODO check remaining formats
        if (dynamicMediaConfig->format[i] == PCM_24_BIT_PACKED) {
            p.format.pcm = PcmType::INT_24_BIT;
        } else if (dynamicMediaConfig->format[i] == PCM_32_BIT) {
            p.format.pcm = PcmType::INT_32_BIT;
        } else if (dynamicMediaConfig->format[i] == PCM_16_BIT) {
            p.format.pcm = PcmType::INT_16_BIT;
        } else {
            // Todo check the default one
            p.format.pcm = PcmType::INT_16_BIT;
        }
        p.sampleRates = sampleRatesSupported;
        p.channelMasks = channelsSupported;

        p.name = (isOutputDevice(devicePortExt.device) ? "usb_playback_profile"
                                                       : "usb_record_profile") +
                 p.format.toString();
        supportedProfiles.emplace_back(p);
    }
    return supportedProfiles;
}

bool Platform::handleDeviceConnectionChange(const AudioPort& deviceAudioPort,
                                            const bool isConnect) const {
    const auto& devicePortExt =
        deviceAudioPort.ext.get<AudioPortExt::Tag::device>();

    auto& audioDeviceDesc = devicePortExt.device.type;
    const auto palDeviceId = mTypeConverter.getPalDeviceId(audioDeviceDesc);
    if(palDeviceId == PAL_DEVICE_OUT_MIN){
        return false;
    }

    void* v = nullptr;
    const auto deviceConnection =
        std::make_unique<pal_param_device_connection_t>();
    if (!deviceConnection) {
        LOG(ERROR) << __func__ << ": allocation failed ";
        return false;
    }

    deviceConnection->connection_state = isConnect;
    deviceConnection->id = palDeviceId;

    if (isUsbDevice(devicePortExt.device)) {
        const auto& addressTag = devicePortExt.device.address.getTag();
        if (addressTag != AudioDeviceAddress::Tag::alsa) {
            LOG(ERROR) << __func__
                       << ": no alsa address provided for the AudioPort"
                       << deviceAudioPort.toString();
            return false;
        }
        const auto& deviceAddressAlsa =
            devicePortExt.device.address.get<AudioDeviceAddress::Tag::alsa>();
        const auto cardId = deviceAddressAlsa[0];
        const auto deviceId = deviceAddressAlsa[1];
        deviceConnection->device_config.usb_addr.card_id = cardId;
        deviceConnection->device_config.usb_addr.device_num = deviceId;
    }

    v = deviceConnection.get();
    if (int32_t ret = ::pal_set_param(PAL_PARAM_ID_DEVICE_CONNECTION, v,
                                      sizeof(pal_param_device_connection_t));
        ret != 0) {
        LOG(ERROR)
            << __func__
            << ": pal set param failed for PAL_PARAM_ID_DEVICE_CONNECTION";
        return false;
    }
    LOG(INFO) << __func__ << devicePortExt.device.toString()
              << (isConnect ? ": connected" : "disconnected");

    return true;
}

bool Platform::setParameter(const std::string& key, const std::string& value) {
    // Todo check for validity of key
    const auto& [first, second] = mParameters.insert_or_assign(key, value);
    LOG(VERBOSE) << __func__
                 << " platform parameter with key:" << key << " "
                 << (second ? "inserted" : "re-assigned")
                 << " with value:" << value;
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
    if (mSndCardStatus == CARD_STATUS_OFFLINE ||
        mSndCardStatus == CARD_STATUS_STANDBY) {
        return true;
    }
    return false;
}

uint32_t Platform::getBluetoothLatencyMs(
    const std::vector<::aidl::android::media::audio::common::AudioDevice>&
        bluetoothDevices) {
    pal_param_bta2dp_t btConfig{};
    for (const auto& device : bluetoothDevices) {
        if (isBluetoothDevice(device)) {
            btConfig.dev_id = mTypeConverter.getPalDeviceId(device.type);
            // first bluetooth device
            break;
        }
    }
    if (btConfig.dev_id == PAL_DEVICE_OUT_BLUETOOTH_A2DP ||
        btConfig.dev_id == PAL_DEVICE_OUT_BLUETOOTH_BLE ||
        btConfig.dev_id == PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST) {
        if (getBtConfig(&btConfig)) {
            return btConfig.latency;
        }
    }
    return 0;
}

// start of private
bool Platform::getBtConfig(pal_param_bta2dp_t* bTConfig) {
    if (bTConfig == nullptr) {
        LOG(ERROR) << __func__ << " invalid bt config";
        return false;
    }
    size_t payloadSize = 0;
    if (int32_t ret = ::pal_get_param(PAL_PARAM_ID_BT_A2DP_ENCODER_LATENCY,
                                      reinterpret_cast<void**>(&bTConfig),
                                      &payloadSize, nullptr);
        ret) {
        LOG(ERROR) << __func__
                   << " failure in PAL_PARAM_ID_BT_A2DP_ENCODER_LATENCY, ret :"
                   << ret;
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
    for (const auto& [key, value] : mParameters) {
        os << key << "=>" << value << std::endl;
    }
    os << mTypeConverter.toString() << std::endl;
    os << " === platform end ===" << std::endl;
    return os.str();
}

// static
int Platform::palGlobalCallback(uint32_t event_id, uint32_t* event_data,
                                uint64_t cookie) {
    auto platform = reinterpret_cast<Platform*>(cookie);
    switch (event_id) {
        case PAL_SND_CARD_STATE:
            platform->mSndCardStatus = static_cast<card_status_t>(*event_data);
            LOG(INFO) << __func__ << " card status changed to "
                      << platform->mSndCardStatus;
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
    if (int32_t ret = pal_register_global_callback(
            &palGlobalCallback, reinterpret_cast<uint64_t>(this));
        ret) {
        LOG(ERROR) << __func__
                   << "pal register global callback failed!!! ret:" << ret;
        return;
    }
    mSndCardStatus = CARD_STATUS_ONLINE;
    LOG(VERBOSE) << __func__ << " pal register global callback successful";
}

// static
Platform& Platform::getInstance() {
    static const auto kPlatform = []() {
        std::unique_ptr<Platform> platform{new Platform()};
        return std::move(platform);
    }();
    return *(kPlatform.get());
}

}  // namespace qti::audio::core