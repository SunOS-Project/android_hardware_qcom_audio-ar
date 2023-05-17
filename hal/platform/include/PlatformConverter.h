/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <algorithm>
#include <sstream>
#include <unordered_map>
#include <vector>

/* AIDL types */
#include <aidl/android/media/audio/common/AudioChannelLayout.h>
#include <aidl/android/media/audio/common/AudioDeviceAddress.h>
#include <aidl/android/media/audio/common/AudioDeviceDescription.h>
#include <aidl/android/media/audio/common/AudioDeviceType.h>
#include <aidl/android/media/audio/common/AudioFormatDescription.h>
#include <aidl/android/media/audio/common/AudioFormatType.h>
#include <aidl/android/media/audio/common/PcmType.h>
#include <media/stagefright/foundation/MediaDefs.h>

/* PAL types */
#include <PalDefs.h>

namespace {
// see boost::hash_combine
#if defined(__clang__)
__attribute__((no_sanitize("unsigned-integer-overflow")))
#endif
static size_t
hash_combine(size_t seed, size_t v) {
    return std::hash<size_t>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
}  // namespace

namespace std {
template <>
struct hash<::aidl::android::media::audio::common::AudioDeviceDescription> {
    std::size_t operator()(
        const ::aidl::android::media::audio::common::AudioDeviceDescription&
            add) const noexcept {
        return hash_combine(
            std::hash<::aidl::android::media::audio::common::AudioDeviceType>{}(
                add.type),
            std::hash<std::string>{}(add.connection));
    }
};

template <>
struct hash<::aidl::android::media::audio::common::AudioFormatDescription> {
    std::size_t operator()(
        const ::aidl::android::media::audio::common::AudioFormatDescription&
            aft) const noexcept {
        return std::hash<std::string>{}(aft.toString());
    }
};
}  // namespace std

using AidlToPalDeviceMap = std::unordered_map<
    ::aidl::android::media::audio::common::AudioDeviceDescription,
    pal_device_id_t>;
using AidlToPalAudioFormatMap = std::unordered_map<
    ::aidl::android::media::audio::common::AudioFormatDescription,
    pal_audio_fmt_t>;
using ChannelCountToPalChannelInfoMap =
    std::unordered_map<int, std::unique_ptr<pal_channel_info>>;

namespace qti::audio {

// singleton class
class PlatformConverter {
   private:
    explicit PlatformConverter() = default;

    PlatformConverter(const PlatformConverter&) = delete;
    PlatformConverter& operator=(const PlatformConverter& x) = delete;

    PlatformConverter(PlatformConverter&& other) = delete;
    PlatformConverter& operator=(PlatformConverter&& other) = delete;

   public:
    static const PlatformConverter& getInstance();
    const AidlToPalDeviceMap& getAidlToPalDeviceMap() const;
    const AidlToPalAudioFormatMap& getAidlToPalAudioFormatMap() const;

    uint16_t getBitWidthForAidlPCM(
        const ::aidl::android::media::audio::common::AudioFormatDescription&)
        const;

    std::unique_ptr<pal_channel_info> getPalChannelInfoForChannelCount(
        int count) const;
    std::string toString() const;

   private:
    AidlToPalDeviceMap mAidlToPalDeviceMap;
    AidlToPalAudioFormatMap mAidlToPalAudioFormatMap;
};
}  // namespace qti::audio
