/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/VendorParameter.h>
#include <aidl/qti/audio/core/VString.h>

namespace qti::audio::core {

constexpr size_t getNearestMultiple(size_t num, size_t multiplier) {
    size_t remainder = 0;

    if (!multiplier) return num;

    remainder = num % multiplier;
    if (remainder) num += (multiplier - remainder);

    return num;
}



auto getBoolValueFromVString =
    [](const std::vector<
           ::aidl::android::hardware::audio::core::VendorParameter>& parameters,
       const std::string& searchKey) -> std::optional<bool> {
    std::optional<::aidl::qti::audio::core::VString> parcel;
    for (const auto& p : parameters) {
        if (p.id == searchKey && p.ext.getParcelable(&parcel) == STATUS_OK &&
            parcel.has_value()) {
            return parcel.value().value == "true";
        }
    }
    return std::nullopt;
};

}  // namespace qti::audio::core