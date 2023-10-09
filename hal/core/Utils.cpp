/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_QUtils"

#include <android-base/logging.h>
#include <qti-audio-core/Utils.h>

using ::aidl::android::media::audio::common::AudioDevice;
using ::aidl::android::media::audio::common::AudioDeviceDescription;
using ::aidl::android::media::audio::common::AudioDeviceType;
using ::aidl::android::media::audio::common::AudioDeviceAddress;
using ::aidl::android::media::audio::common::AudioPortConfig;
using ::aidl::android::media::audio::common::AudioPortExt;

using ::aidl::android::hardware::audio::core::VendorParameter;

using ::aidl::qti::audio::core::VString;

namespace qti::audio::core {

bool isMixPortConfig(const AudioPortConfig& audioPortConfig) noexcept {
    return audioPortConfig.ext.getTag() == AudioPortExt::Tag::mix;
};

bool isDevicePortConfig(const AudioPortConfig& audioPortConfig) noexcept {
    return audioPortConfig.ext.getTag() == AudioPortExt::Tag::device;
};

bool isTelephonyRXDevice(const AudioDevice& device) noexcept {
    return device.type.type == AudioDeviceType::IN_TELEPHONY_RX;
};

bool isTelephonyTXDevice(const AudioDevice& device) noexcept {
    return device.type.type == AudioDeviceType::OUT_TELEPHONY_TX;
};

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

}  // namespace qti::audio::core
