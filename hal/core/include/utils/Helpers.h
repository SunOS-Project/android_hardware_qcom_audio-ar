/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/media/audio/common/AudioDevice.h>
#include <aidl/android/media/audio/common/AudioDeviceDescription.h>
#include <aidl/android/media/audio/common/AudioDeviceType.h>

static inline bool is_output_device(
    const ::aidl::android::media::audio::common::AudioDevice& d) noexcept {
    if (d.type.type >=
        ::aidl::android::media::audio::common::AudioDeviceType::OUT_DEFAULT) {
        return true;
    }
    return false;
}

static inline bool is_input_device(
    const ::aidl::android::media::audio::common::AudioDevice& d) noexcept {
    if (d.type.type <
        ::aidl::android::media::audio::common::AudioDeviceType::OUT_DEFAULT) {
        return true;
    }
    return false;
}

static inline bool is_usb_device(
    const ::aidl::android::media::audio::common::AudioDevice& d) noexcept {
    if (d.type.connection == ::aidl::android::media::audio::common::
                                 AudioDeviceDescription::CONNECTION_USB) {
        return true;
    }
    return false;
}
