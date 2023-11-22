/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_QUtils"

#include <android-base/logging.h>
#include <qti-audio-core/Utils.h>
#include <aidl/android/media/audio/common/AudioInputFlags.h>
#include <aidl/android/media/audio/common/AudioOutputFlags.h>

using ::aidl::android::media::audio::common::AudioDevice;
using ::aidl::android::media::audio::common::AudioDeviceDescription;
using ::aidl::android::media::audio::common::AudioDeviceType;
using ::aidl::android::media::audio::common::AudioDeviceAddress;
using ::aidl::android::media::audio::common::AudioPortConfig;
using ::aidl::android::media::audio::common::AudioPortExt;
using ::aidl::android::media::audio::common::AudioIoFlags;
using ::aidl::android::media::audio::common::AudioInputFlags;
using ::aidl::android::media::audio::common::AudioOutputFlags;

using ::aidl::android::hardware::audio::core::VendorParameter;

using ::aidl::qti::audio::core::VString;

namespace qti::audio::core {

bool isMixPortConfig(const AudioPortConfig& audioPortConfig) noexcept {
    return audioPortConfig.ext.getTag() == AudioPortExt::Tag::mix;
};

bool isInputMixPortConfig(const AudioPortConfig& audioPortConfig) noexcept {
    return isMixPortConfig(audioPortConfig) && audioPortConfig.flags &&
           audioPortConfig.flags.value().getTag() == AudioIoFlags::Tag::input;
}

bool isDevicePortConfig(const AudioPortConfig& audioPortConfig) noexcept {
    return audioPortConfig.ext.getTag() == AudioPortExt::Tag::device;
};

bool isTelephonyRXDevice(const AudioDevice& device) noexcept {
    return device.type.type == AudioDeviceType::IN_TELEPHONY_RX;
};

bool isTelephonyTXDevice(const AudioDevice& device) noexcept {
    return device.type.type == AudioDeviceType::OUT_TELEPHONY_TX;
};

bool isBluetoothDevice(const AudioDevice& device) noexcept {
    return (device.type.connection == AudioDeviceDescription::CONNECTION_BT_A2DP ||
            device.type.connection == AudioDeviceDescription::CONNECTION_BT_LE);
}

bool hasBluetoothDevice(const std::vector<AudioDevice>& devices) noexcept {
    auto itr = std::find_if(devices.cbegin(), devices.cend(), isBluetoothDevice);
    return itr != devices.cend();
}

bool isInputMMap(const AudioIoFlags& ioFlags) noexcept {
    if (ioFlags.getTag() == AudioIoFlags::Tag::input) {
        constexpr auto inputMMapFlag = static_cast<int32_t>(
            1 << static_cast<int32_t>(AudioInputFlags::MMAP_NOIRQ));
        return ((inputMMapFlag & ioFlags.get<AudioIoFlags::Tag::input>()) != 0);
    }
    return false;
}

bool isOutputMMap(const AudioIoFlags& ioFlags) noexcept {
    if (ioFlags.getTag() == AudioIoFlags::Tag::output) {
        constexpr auto outputMMapFlag = static_cast<int32_t>(
            1 << static_cast<int32_t>(AudioOutputFlags::MMAP_NOIRQ));
        return ((outputMMapFlag & ioFlags.get<AudioIoFlags::Tag::output>()) !=
                0);
    }
    return false;
}

bool isMMap(const AudioIoFlags& ioFlags) noexcept {
    return (isInputMMap(ioFlags) || isOutputMMap(ioFlags));
}

bool isInputAFEProxyDevice(const AudioDevice& device) noexcept {
    return device.type.type == AudioDeviceType::IN_AFE_PROXY;
}

std::vector<int32_t> getActiveInputMixPortConfigIds(
        const std::vector<AudioPortConfig>& activePortConfigs) {
    std::vector<int32_t> result;
    for (const auto& activePortConfig : activePortConfigs) {
        if (isInputMixPortConfig(activePortConfig)) {
            result.emplace_back(activePortConfig.id);
        }
    }
    return result;
}

int64_t getInt64FromString(const std::string& s) noexcept {
    // Todo handle actual value 0
    return static_cast<int64_t>(strtol(s.c_str(), nullptr, 10));
}

bool getBoolFromString(const std::string& s) noexcept {
    return (s == "true");
}

bool setParameter(const VString& parcel, VendorParameter& parameter) noexcept {
    if (parameter.ext.setParcelable(parcel) != android::OK) {
        LOG(ERROR) << __func__ << ": failed to set parcel for " << parameter.id;
        return false;
    }
    return true;
}

} // namespace qti::audio::core
