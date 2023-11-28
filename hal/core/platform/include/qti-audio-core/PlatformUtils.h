/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <PalApi.h>
#include <aidl/android/hardware/audio/core/VendorParameter.h>
#include <aidl/qti/audio/core/VString.h>

namespace qti::audio::core {

using ::aidl::android::media::audio::common::AudioProfile;
using ::aidl::android::media::audio::common::AudioChannelLayout;

constexpr size_t getNearestMultiple(size_t num, size_t multiplier) {
    size_t remainder = 0;

    if (!multiplier) return num;

    remainder = num % multiplier;
    if (remainder) num += (multiplier - remainder);

    return num;
}

// std::string getStringForVendorParameter(
auto getkvPairsForVendorParameter =
        [](const std::vector<::aidl::android::hardware::audio::core::VendorParameter>& param)
        -> std::string {
            std::string str = "";
            std::optional<::aidl::qti::audio::core::VString> parcel;
            for (const auto& p : param) {
                if (p.ext.getParcelable(&parcel) == STATUS_OK && parcel.has_value()) {
                    std::string keyvalue = p.id + "=" + parcel.value().value + ";";
                    str.append(keyvalue);
                }
            }
            return str;
        };

auto getBoolValueFromVString = [](
        const std::vector<::aidl::android::hardware::audio::core::VendorParameter>& parameters,
        const std::string& searchKey) -> std::optional<bool> {
    std::optional<::aidl::qti::audio::core::VString> parcel;
    for (const auto& p : parameters) {
        if (p.id == searchKey && p.ext.getParcelable(&parcel) == STATUS_OK && parcel.has_value()) {
            return parcel.value().value == "true";
        }
    }
    return std::nullopt;
};

std::vector<AudioProfile> getSupportedAudioProfiles(pal_param_device_capability_t* capability,
                                                    std::string devName);
std::vector<AudioChannelLayout> getChannelMasksFromProfile(
        pal_param_device_capability_t* capability);
std::vector<int> getSampleRatesFromProfile(pal_param_device_capability_t* capability);
AudioChannelLayout getChannelIndexMaskFromChannelCount(unsigned int channelCount);
AudioChannelLayout getChannelLayoutMaskFromChannelCount(unsigned int channelCount, int isInput);

} // namespace qti::audio::core
