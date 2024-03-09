/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Utils_QTI"

#include <aidl/android/media/audio/common/AudioInputFlags.h>
#include <aidl/android/media/audio/common/AudioOutputFlags.h>
#include <android-base/logging.h>
#include <audio_utils/format.h>
#include <qti-audio-core/Utils.h>

using ::aidl::android::media::audio::common::AudioDevice;
using ::aidl::android::media::audio::common::AudioDeviceDescription;
using ::aidl::android::media::audio::common::AudioDeviceType;
using ::aidl::android::media::audio::common::AudioDeviceAddress;
using ::aidl::android::media::audio::common::AudioPortConfig;
using ::aidl::android::media::audio::common::AudioPortExt;
using ::aidl::android::media::audio::common::AudioIoFlags;
using ::aidl::android::media::audio::common::AudioInputFlags;
using ::aidl::android::media::audio::common::AudioOutputFlags;
using ::aidl::android::media::audio::common::AudioPortExt;
using ::aidl::android::media::audio::common::AudioSource;
using ::aidl::android::media::audio::common::AudioPortMixExtUseCase;

using ::aidl::android::hardware::audio::core::VendorParameter;

using ::aidl::qti::audio::core::VString;

namespace qti::audio::core {

BufferFormatConverter::BufferFormatConverter(audio_format_t inFormat, audio_format_t outFormat,
                                             size_t bufSize) {
    mInFormat = inFormat;
    mOutFormat = outFormat;
    mAllocSize = bufSize;
    mInBytesPerSample = audio_bytes_per_sample(inFormat);
    mOutBytesPerSample = audio_bytes_per_sample(outFormat);
    int sizeoffloat = sizeof(float);
    mBuffer = std::make_unique<uint8_t[]>(mAllocSize);
    if (!mBuffer) {
        LOG(ERROR) << __func__ << " failed to init convert buffer";
        // alloc size to 0, so convert won't operate
        mAllocSize = 0;
    }
    LOG(VERBOSE) << __func__ << "inFormat " << inFormat << " outFormat " << mOutFormat
                 << " inBytesPerSample " << mInBytesPerSample << " outBytesPerSample "
                 << mOutBytesPerSample << " size " << mAllocSize;
}

std::optional<std::pair<uint8_t*, size_t>> BufferFormatConverter::convert(const void* buffer,
                                                                          size_t bytes) {
    if (bytes > mAllocSize) {
        LOG(ERROR) << " Error writing" << bytes << " to convertBuffer of capacity " << mAllocSize;
        return std::nullopt;
    }
    size_t frames = bytes / mInBytesPerSample;
    memcpy_by_audio_format(mBuffer.get(), mOutFormat, buffer, mInFormat, frames);
    uint8_t* outBuffer = reinterpret_cast<uint8_t*>(mBuffer.get());
    return std::make_pair(outBuffer, frames * mOutBytesPerSample);
}

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

bool isBluetoothA2dpDevice(const AudioDevice& device) noexcept {
    return (device.type.connection == AudioDeviceDescription::CONNECTION_BT_A2DP);
}

bool hasBluetoothA2dpDevice(const std::vector<AudioDevice>& devices) noexcept {
    auto itr = std::find_if(devices.cbegin(), devices.cend(), isBluetoothA2dpDevice);
    return itr != devices.cend();
}

bool hasInputMMapFlag(const AudioIoFlags& ioFlags) noexcept {
    if (ioFlags.getTag() == AudioIoFlags::Tag::input) {
        constexpr auto inputMMapFlag = static_cast<int32_t>(
            1 << static_cast<int32_t>(AudioInputFlags::MMAP_NOIRQ));
        return ((inputMMapFlag & ioFlags.get<AudioIoFlags::Tag::input>()) != 0);
    }
    return false;
}

bool hasOutputMMapFlag(const AudioIoFlags& ioFlags) noexcept {
    if (ioFlags.getTag() == AudioIoFlags::Tag::output) {
        constexpr auto outputMMapFlag = static_cast<int32_t>(
            1 << static_cast<int32_t>(AudioOutputFlags::MMAP_NOIRQ));
        return ((outputMMapFlag & ioFlags.get<AudioIoFlags::Tag::output>()) !=
                0);
    }
    return false;
}

bool hasMMapFlagsEnabled(const AudioIoFlags& ioFlags) noexcept {
    return (hasInputMMapFlag(ioFlags) || hasOutputMMapFlag(ioFlags));
}

bool isInputAFEProxyDevice(const AudioDevice& device) noexcept {
    return device.type.type == AudioDeviceType::IN_AFE_PROXY;
}

bool hasOutputDirectFlag(const AudioIoFlags& ioFlags) noexcept {
    if (ioFlags.getTag() == AudioIoFlags::Tag::output) {
        constexpr auto directFlag =
                static_cast<int32_t>(1 << static_cast<int32_t>(AudioOutputFlags::DIRECT));
        return ((directFlag & ioFlags.get<AudioIoFlags::Tag::output>()) != 0);
    }
    return false;
}

bool hasOutputCompressOffloadFlag(const AudioIoFlags& ioFlags) noexcept {
    if (ioFlags.getTag() == AudioIoFlags::Tag::output) {
        constexpr auto compressOffloadFlag =
                static_cast<int32_t>(1 << static_cast<int32_t>(AudioOutputFlags::COMPRESS_OFFLOAD));
        return ((compressOffloadFlag & ioFlags.get<AudioIoFlags::Tag::output>()) != 0);
    }
    return false;
}

std::optional<AudioSource> getAudioSource(const AudioPortConfig& mixPortconfig) noexcept {
    if (mixPortconfig.ext.getTag() != AudioPortExt::Tag::mix) {
        LOG(ERROR) << __func__ << ": not a mix port, " << mixPortconfig.toString();
        return std::nullopt;
    }
    if (mixPortconfig.ext.get<AudioPortExt::Tag::mix>().usecase.getTag() !=
        AudioPortMixExtUseCase::Tag::source) {
        LOG(ERROR) << __func__ << ": no source provided, " << mixPortconfig.toString();
        return std::nullopt;
    }
    return mixPortconfig.ext.get<AudioPortExt::Tag::mix>()
            .usecase.get<AudioPortMixExtUseCase::Tag::source>();
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

float getFloatFromString(const std::string& s) noexcept {
    // Todo handle actual value 0
    return strtof(s.c_str(), nullptr);
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

VendorParameter makeVendorParameter(const std::string& id, const std::string& value) {
    VString parcel;
    parcel.value = value;
    VendorParameter param;
    param.id = id;
    if (param.ext.setParcelable(parcel) != android::OK) {
        LOG(ERROR) << __func__ << ": failed to set parcel for " << param.id;
    }
    return param;
}

std::string makeParamValue(bool const& isTrue) noexcept {
    return isTrue ? "true" : "false";
}

} // namespace qti::audio::core
