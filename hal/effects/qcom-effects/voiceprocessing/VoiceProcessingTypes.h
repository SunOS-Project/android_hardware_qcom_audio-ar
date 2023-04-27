/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/effect/BnEffect.h>
#include "effect-impl/EffectImpl.h"
#include "effect-impl/EffectUUID.h"

namespace aidl::android::hardware::audio::effect {

enum class VoiceProcessingType {
    AcousticEchoCanceler,
    NoiseSuppression,
};

static const std::string kAcousticEchoCancelerEffectName = "aec";
static const std::string kNoiseSuppressionEffectName = "ns";

/*
#define flags        (EFFECT_FLAG_TYPE_PRE_PROC | \
                      EFFECT_FLAG_DEVICE_IND | \
                      EFFECT_FLAG_HW_ACC_TUNNEL| \ 
                      EFFECT_FLAG_OFFLOAD_SUPPORTED)
*/

static Flags kVoiceProcessingFlags = {
                      .type = Flags::Type::PRE_PROC,
                      .deviceIndication = true,
                      .hwAcceleratorMode = Flags::HardwareAccelerator::TUNNEL,
                      .offloadIndication = true,
                    };

static const Descriptor kAcousticEchoCancelerDesc = {
        .common = {.id = {.type = kAcousticEchoCancelerTypeUUID,
                          .uuid = kAcousticEchoCancelerQtiUUID,
                          .proxy = std::nullopt},
                   .flags = kVoiceProcessingFlags,
                   .name = kAcousticEchoCancelerEffectName,
                   .implementor = "Qualcomm Technologies Inc"
                   }
};

static const Descriptor kNoiseSuppressionDesc = {
        .common = {.id = {.type = kNoiseSuppressionTypeUUID,
                          .uuid = kNoiseSuppressionQtiUUID,
                          .proxy = std::nullopt},
                   .flags = kVoiceProcessingFlags,
                   .name = kNoiseSuppressionEffectName,
                   .implementor = "Qualcomm Technologies Inc"
                   }
};

inline std::ostream& operator<<(std::ostream& out, const VoiceProcessingType& type) {
    switch (type) {
        case VoiceProcessingType::AcousticEchoCanceler:
            return out << "AcousticEchoCanceler";
        case VoiceProcessingType::NoiseSuppression:
            return out << "NoiseSuppression";
    }
    return out << "Enum_VoiceProcessingError";
}

}  // namespace aidl::android::hardware::audio::effect