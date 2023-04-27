/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#pragma once

#include <android-base/logging.h>
#include <android-base/thread_annotations.h>
#include <array>
#include <cstddef>

#include "OffloadBundleTypes.h"
#include "effect-impl/EffectContext.h"

namespace aidl::android::hardware::audio::effect {

class OffloadBundleContext final : public EffectContext {
  public:
    OffloadBundleContext(int statusDepth, const Parameter::Common& common,
                  const OffloadBundleEffectType& type)
        : EffectContext(statusDepth, common), mType(type) {
        LOG(DEBUG) << __func__ << type;
    }
    ~OffloadBundleContext() override {
        LOG(DEBUG) << __func__;
        deInit();
    }

    RetCode init();
    void deInit();
    OffloadBundleEffectType getBundleType() const { return mType; }

    RetCode enable();
    RetCode enableOperatingMode();
    RetCode disable();
    RetCode disableOperatingMode();


    bool isDeviceSupportedBassBoost(
            const aidl::android::media::audio::common::AudioDeviceDescription& device);
    bool isDeviceSupportedVirtualizer(
            const aidl::android::media::audio::common::AudioDeviceDescription& device);
    virtual RetCode setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>& device) override;

    RetCode setEqualizerPreset(const std::size_t presetIdx);
    RetCode setEqualizerBandLevels(const std::vector<Equalizer::BandLevel>& bandLevels);
    std::vector<Equalizer::BandLevel> getEqualizerBandLevels() const;

    RetCode setBassBoostStrength(int strength);

    RetCode setVolumeLevel(int level);
    int getVolumeLevel() const;

    RetCode setVolumeMute(bool mute);

    RetCode setVirtualizerStrength(int strength);

    RetCode setVolumeStereo(const Parameter::VolumeStereo& volumeStereo) override;

  private:
    std::mutex mMutex;
    const OffloadBundleEffectType mType;
    bool mEnabled = false;

};

}  // namespace aidl::android::hardware::audio::effect

