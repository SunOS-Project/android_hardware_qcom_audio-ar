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

// clang-format off
namespace {
__attribute__((no_sanitize("unsigned-integer-overflow")))
static void hash_combiner(std::size_t& seed, const std::size_t& v) {
    // see boost::hash_combine
    seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
}  // namespace
// clang-format on

namespace std {
template <>
struct hash<::aidl::android::media::audio::common::AudioDeviceDescription> {
    std::size_t operator()(const ::aidl::android::media::audio::common::AudioDeviceDescription& add)
            const noexcept {
        std::size_t seed = 0;
        hash_combiner(seed, std::hash<::aidl::android::media::audio::common::AudioDeviceType>{}(
                                    add.type));
        hash_combiner(seed, std::hash<std::string>{}(add.connection));
        return seed;
    }
};

template <>
struct hash<::aidl::android::media::audio::common::AudioFormatDescription> {
    std::size_t operator()(const ::aidl::android::media::audio::common::AudioFormatDescription& aft)
            const noexcept {
        std::size_t seed = 0;
        hash_combiner(seed, std::hash<::aidl::android::media::audio::common::AudioFormatType>{}(
                                    aft.type));
        hash_combiner(seed, std::hash<::aidl::android::media::audio::common::PcmType>{}(aft.pcm));
        hash_combiner(seed, std::hash<std::string>{}(aft.encoding));
        return seed;
    }
};
} // namespace std

namespace qti::audio::core {

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
    pal_audio_fmt_t getPalFormatId(
            const ::aidl::android::media::audio::common::AudioFormatDescription& formatDescription)
            const;
    pal_device_id_t getPalDeviceId(
            const ::aidl::android::media::audio::common::AudioDeviceDescription& deviceDescription)
            const;

    uint16_t getBitWidthForAidlPCM(
            const ::aidl::android::media::audio::common::AudioFormatDescription&) const;

    std::unique_ptr<pal_channel_info> getPalChannelInfoForChannelCount(int count) const;
    std::string toString() const;

  private:
    using AidlToPalDeviceMap =
            std::map<::aidl::android::media::audio::common::AudioDeviceDescription,
                     pal_device_id_t>;
    using AidlToPalAudioFormatMap =
            std::map<::aidl::android::media::audio::common::AudioFormatDescription,
                     pal_audio_fmt_t>;
    AidlToPalDeviceMap mAidlToPalDeviceMap;
    AidlToPalAudioFormatMap mAidlToPalAudioFormatMap;
};
} // namespace qti::audio::core
