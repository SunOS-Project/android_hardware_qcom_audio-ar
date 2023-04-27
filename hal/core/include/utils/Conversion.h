/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include<android-base/logging.h>

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <media/stagefright/foundation/MediaDefs.h>

#include <aidl/android/media/audio/common/AudioDeviceDescription.h>
#include <aidl/android/media/audio/common/AudioDeviceType.h>
#include <aidl/android/media/audio/common/AudioChannelLayout.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioFormatType.h>
#include <aidl/android/media/audio/common/PcmType.h>

#include "PalDefs.h"

using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;
using aidl::android::media::audio::common::AudioChannelLayout;
using aidl::android::media::audio::common::AudioFormatDescription;
using aidl::android::media::audio::common::AudioFormatType;
using aidl::android::media::audio::common::PcmType;

using DevicePair = std::pair<AudioDeviceDescription, pal_device_id_t>;
using DevicePairs = std::vector<DevicePair>;
using AidlToPalDeviceMap =
    std::unordered_map<AudioDeviceDescription, pal_device_id_t>;

using FormatPair = std::pair<pal_audio_fmt_t, AudioFormatDescription>;
using FormatPairs = std::vector<FormatPair>;
using AidlToPalAudioFormatMap =
    std::unordered_map<AudioFormatDescription, pal_audio_fmt_t>;

using ChannelCountToPalChannelInfoMap =
    std::unordered_map<int, pal_channel_info>;

template<typename S, typename T>
std::unordered_map<S, T> make_DirectMap(const std::vector<std::pair<S, T>>& v) {
    std::unordered_map<S, T> result(v.begin(), v.end());
    if (result.size() != v.size()) {
        LOG(FATAL) << __func__ << "Duplicate key elements detected";
    }
    return result;
}

template <typename S, typename T>
std::unordered_map<T, S> make_ReverseMap(
    const std::vector<std::pair<S, T>>& v) {
    std::unordered_map<T, S> result;
    std::transform(v.begin(), v.end(), std::inserter(result, result.begin()),
                   [](const std::pair<S, T>& p) {
                       return std::make_pair(p.second, p.first);
                   });
    if (result.size() != v.size()) {
        LOG(FATAL) << __func__ << "Duplicate key elements detected";
    }
    return result;
}

AudioDeviceDescription makeAudioDeviceDescription(
    AudioDeviceType type, const std::string& connection = "");

const DevicePairs& getDevicePairs();

// start audio format

AudioFormatDescription make_AudioFormatDescription(AudioFormatType type);

AudioFormatDescription make_AudioFormatDescription(PcmType pcm);

AudioFormatDescription make_AudioFormatDescription(const std::string& encoding);

AudioFormatDescription make_AudioFormatDescription(PcmType transport,
                                                   const std::string& encoding);


const FormatPairs& getFormatPairs();

// start audio channels

ChannelCountToPalChannelInfoMap& buildPalChannelInfos();
