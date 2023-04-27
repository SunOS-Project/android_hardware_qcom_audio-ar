/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#pragma once
#include <array>

#include <aidl/android/hardware/audio/effect/BnEffect.h>
#include "effect-impl/EffectUUID.h"
#include "effect-impl/EffectTypes.h"


namespace aidl::android::hardware::audio::effect {

static const Capability kEqCap = {};

static const std::string kEqualizerEffectName = "Qti-Offload-Equalizer";

/*
# define EQ_FLAG EFFECT_FLAG_TYPE_INSERT | EFFECT_FLAG_HW_ACC_TUNNEL | EFFECT_FLAG_VOLUME_CTRL
*/
static const Descriptor kEqualizerDesc = {
        .common = {.id = {.type = kEqualizerTypeUUID,
                          .uuid = kEqualizerOffloadQtiUUID,
                          .proxy = kEqualizerProxyUUID},
                   .flags = {.type = Flags::Type::INSERT,
                             .volume = Flags::Volume::CTRL,
                             .hwAcceleratorMode = Flags::HardwareAccelerator::TUNNEL,},
                   .name = kEqualizerEffectName,
                   .implementor = "Qualcomm Technologies Inc."},
        .capability = kEqCap};

static const bool mStrengthSupported = true;

static const Capability kBassBoostCap = {};

static const std::string kBassBoostEffectName = "Qti-Offload-BassBoost";

/*
#define BB_FLAG         (EFFECT_FLAG_TYPE_INSERT | EFFECT_FLAG_DEVICE_IND | EFFECT_FLAG_HW_ACC_TUNNEL |
          EFFECT_FLAG_VOLUME_CTRL),
*/
static const Descriptor kBassBoostDesc = {
        .common = {.id = {.type = kBassBoostTypeUUID,
                          .uuid = kBassBoostOffloadQtiUUID,
                          .proxy = kBassBoostProxyUUID},
                   .flags = {.type = Flags::Type::INSERT,
                             .volume = Flags::Volume::CTRL,
                             .hwAcceleratorMode = Flags::HardwareAccelerator::TUNNEL,
                             .deviceIndication = true},
                   .name = kBassBoostEffectName,
                   .implementor = "Qualcomm Technologies Inc."},
        .capability = kBassBoostCap};

/*
#define VIRTUALIZER_FLAGS        (EFFECT_FLAG_TYPE_INSERT | EFFECT_FLAG_DEVICE_IND | EFFECT_FLAG_HW_ACC_TUNNEL |
         EFFECT_FLAG_VOLUME_CTRL),
*/
static const Capability kVirtualizerCap = {};

static const std::string kVirtualizerEffectName = "Qti-Offload-Virtualizer";

static const Descriptor kVirtualizerDesc = {
        .common = {.id = {.type = kVirtualizerTypeUUID,
                          .uuid = kVirtualizerOffloadQtiUUID,
                          .proxy = kVirtualizerProxyUUID},
                   .flags = {.type = Flags::Type::INSERT,
                             .volume = Flags::Volume::CTRL,
                              .hwAcceleratorMode = Flags::HardwareAccelerator::TUNNEL,
                             .deviceIndication = true},
                   .name = kVirtualizerEffectName,
                   .implementor = "Qualcomm Technologies Inc."},
        .capability = kVirtualizerCap};

enum class OffloadBundleEffectType {
    BASS_BOOST,
    VIRTUALIZER,
    EQUALIZER,
};

inline std::ostream& operator<<(std::ostream& out, const OffloadBundleEffectType& type) {
    switch (type) {
        case OffloadBundleEffectType::BASS_BOOST:
            return out << "BASS_BOOST";
        case OffloadBundleEffectType::VIRTUALIZER:
            return out << "VIRTUALIZER";
        case OffloadBundleEffectType::EQUALIZER:
            return out << "EQUALIZER";
    }
    return out << "EnumOffloadBundleEffectTypeError";
}

}  // namespace aidl::android::hardware::audio::effect
