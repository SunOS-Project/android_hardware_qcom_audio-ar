/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_PlatformModule"

#include <aidl/android/media/audio/common/AudioInputFlags.h>
#include <aidl/android/media/audio/common/AudioOutputFlags.h>

#include <android-base/logging.h>
#include <platform/PlatformModule.h>
#include <utils/Helpers.h>

#include "PalApi.h"

using aidl::android::media::audio::common::AudioChannelLayout;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioDeviceAddress;
using aidl::android::media::audio::common::AudioFormatDescription;
using aidl::android::media::audio::common::AudioFormatType;
using aidl::android::media::audio::common::AudioInputFlags;
using aidl::android::media::audio::common::AudioIoFlags;
using aidl::android::media::audio::common::AudioMMapPolicy;
using aidl::android::media::audio::common::AudioMMapPolicyInfo;
using aidl::android::media::audio::common::AudioMMapPolicyType;
using aidl::android::media::audio::common::AudioMode;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::AudioOutputFlags;
using aidl::android::media::audio::common::AudioPort;
using aidl::android::media::audio::common::AudioPortConfig;
using aidl::android::media::audio::common::AudioPortExt;
using aidl::android::media::audio::common::AudioProfile;
using aidl::android::media::audio::common::Boolean;
using aidl::android::media::audio::common::Int;
using aidl::android::media::audio::common::MicrophoneInfo;
using aidl::android::media::audio::common::PcmType;

namespace qti::audio::core {

PlatformModule::PlatformModule(std::weak_ptr<Module> module) {
    LOG(VERBOSE) << __func__ << ": Enter";
    mModule = module;
    int ret = pal_init();
    LOG(VERBOSE) << __func__ << ": Exit";
}

PlatformModule::PlatformModule() {
    LOG(VERBOSE) << __func__ << ": Enter";
    int ret = pal_init();
    ret = pal_register_global_callback(&registerGlobalCallback, (uint64_t)this);
    LOG(VERBOSE) << __func__ << ": Exit";
}

PlatformModule::~PlatformModule() {
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
}

int32_t PlatformModule::registerGlobalCallback(uint32_t eventId,
                                               uint32_t* eventData,
                                               uint64_t cookie) {
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return 0;
}

void PlatformModule::init() {
    LOG(VERBOSE) << __func__ << ": Enter";
    mAidlToPalDeviceMap =
        make_DirectMap<AudioDeviceDescription, pal_device_id_t>(
            getDevicePairs());
    mAidlToPalAudioFormatMap =
        make_ReverseMap<pal_audio_fmt_t, AudioFormatDescription>(
            getFormatPairs());
    mPlatformModuleParamHandler =
        std::make_shared<PlatformModuleParameterHandler>(shared_from_this());
    mChannelCountToPalInfoMap = buildPalChannelInfos();
    LOG(VERBOSE) << __func__ << ": Exit";
}

int32_t PlatformModule::setMicMute(bool state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

int32_t PlatformModule::getMicMute(bool& state) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

int32_t PlatformModule::dump(const int32_t fd) {
    std::ostringstream os;

    os << "--PlatformModule dump start--" << std::endl;
    os << "devices: Aidl to PAL" << std::endl;
    for (const auto& entry : mAidlToPalDeviceMap) {
        os << entry.first.toString().c_str() << " "
           << deviceNameLUT.at(entry.second).c_str() << std::endl;
    }
    os << "formats: Aidl to PAL ";
    for (const auto& entry : mAidlToPalAudioFormatMap) {
        os << entry.first.toString().c_str() << " "
           << deviceNameLUT.at(entry.second).c_str() << std::endl;
    }
    os << "--PlatformModule dump end--" << std::endl;

    const auto& dumpInfo = os.str();
    if (fd > 0) {
        auto dumpInfoSize = dumpInfo.size();
        // TODO remove fd and add fstream support
        auto b = ::write(fd, dumpInfo.c_str(), dumpInfoSize);
        if (b != dumpInfoSize) {
            LOG(ERROR) << __func__ << ": dump failed";
            return -EIO;
        }
    } else {
        LOG(INFO) << dumpInfo.c_str();
    }

    int32_t status = 0;
    // call memeber objects dump

    return status;
}

int32_t PlatformModule::setGEFParam(void* data, int length) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

int32_t PlatformModule::getGEFParam(void* data, int* length) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

void PlatformModule::setChargingMode(bool charging) {
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
}

int32_t PlatformModule::getParameters(const std::string& keys,
                                      std::string& values) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    status = mPlatformModuleParamHandler->getParameters(keys, values);
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

int32_t PlatformModule::setParameters(
    const std::vector<std::string>& keyValuePairs) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    status = mPlatformModuleParamHandler->setParameters(keyValuePairs);
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

void PlatformModule::initDeviceMap() {
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
}

void PlatformModule::dumpDeviceMap() {
    for (const auto& entry : mAidlToPalDeviceMap) {
        LOG(INFO) << entry.first.toString().c_str() << " "
                  << deviceNameLUT.at(entry.second).c_str();
    }
}

std::vector<AudioProfile> PlatformModule::getDynamicProfiles(
    const AudioPort& dynamicAudioPort) {
    const auto& deviceExtTag = dynamicAudioPort.ext.getTag();
    if (deviceExtTag != AudioPortExt::Tag::device) {
        LOG(ERROR) << __func__ << ": provided AudioPort is not device port"
                   << dynamicAudioPort.toString();
        return {};
    }

    LOG(DEBUG) << __func__ << ": fetching dynamic profiles for "
               << dynamicAudioPort.toString();

    const auto& devicePortExt =
        dynamicAudioPort.ext.get<AudioPortExt::Tag::device>();
    const auto& aidlDeviceAddress = makeAudioDeviceDescription(
        devicePortExt.device.type.type, devicePortExt.device.type.connection);
    if (mAidlToPalDeviceMap.find(aidlDeviceAddress) ==
        mAidlToPalDeviceMap.cend()) {
        LOG(ERROR) << __func__ << "no compatible PAL device for port "
                   << dynamicAudioPort.toString();
        return {};
    }
    const auto palDeviceId = mAidlToPalDeviceMap.at(aidlDeviceAddress);

    // if the device is usb
    if (is_usb_device(devicePortExt.device)) {
        const auto& addressTag = devicePortExt.device.address.getTag();
        if (addressTag != AudioDeviceAddress::Tag::alsa) {
            LOG(ERROR) << __func__
                       << ": no alsa address provided for the AudioPort"
                       << dynamicAudioPort.toString();
            return {};
        }
        const auto& deviceAddressAlsa =
            devicePortExt.device.address.get<AudioDeviceAddress::Tag::alsa>();
        const auto cardId = deviceAddressAlsa[0];
        const auto deviceId = deviceAddressAlsa[1];

        void* v = nullptr;

        // get capability from device of USB
        auto deviceCapability =
            std::make_unique<pal_param_device_capability_t>();
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
        deviceCapability->is_playback = is_output_device(devicePortExt.device);
        v = deviceCapability.get();
        int32_t ret;
        ret = pal_get_param(PAL_PARAM_ID_DEVICE_CAPABILITY, &v, &payload_size,
                            nullptr);
        if (ret < 0) {
            LOG(ERROR) << __func__ << ": PAL get param failed: " << ret;
            return {};
        }
        if (!dynamicMediaConfig->jack_status) {
            LOG(ERROR) << __func__ << ": false usb jack status ";
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
                channels.push_back(AudioChannelLayout::make<
                                   AudioChannelLayout::Tag::layoutMask>(
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

            p.name = (is_output_device(devicePortExt.device)
                          ? "usb_playback_profile"
                          : "usb_record_profile") +
                     p.format.toString();
            supportedProfiles.emplace_back(p);
        }
        return supportedProfiles;
    }

    return {};
}

bool PlatformModule::handleDeviceConnectionChange(
    const AudioPort& deviceAudioPort, const bool isConnect) {
    const auto& devicePortExt =
        deviceAudioPort.ext.get<AudioPortExt::Tag::device>();
    const auto& aidlDeviceAddress = makeAudioDeviceDescription(
        devicePortExt.device.type.type, devicePortExt.device.type.connection);
    if (mAidlToPalDeviceMap.find(aidlDeviceAddress) ==
        mAidlToPalDeviceMap.cend()) {
        LOG(ERROR) << __func__ << "no compatible PAL device for port "
                   << deviceAudioPort.toString();
        return false;
    }
    const auto palDeviceId = mAidlToPalDeviceMap.at(aidlDeviceAddress);

    void* v = nullptr;
    const auto deviceConnection =
        std::make_unique<pal_param_device_connection_t>();
    if (!deviceConnection) {
        LOG(ERROR) << __func__ << ": allocation failed ";
        return false;
    }

    deviceConnection->connection_state = isConnect;
    deviceConnection->id = palDeviceId;

    if (is_usb_device(devicePortExt.device)) {
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
    int32_t ret = pal_set_param(PAL_PARAM_ID_DEVICE_CONNECTION, &v,
                                sizeof(pal_param_device_connection_t));
    if (ret != 0) {
        LOG(ERROR)
            << __func__
            << ": pal set param failed for PAL_PARAM_ID_DEVICE_CONNECTION";
        return false;
    }
    LOG(DEBUG) << __func__ << "device "
               << (isConnect ? ": connected" : "disconnected");

    return true;
}

}  // namespace qti::audio::core
