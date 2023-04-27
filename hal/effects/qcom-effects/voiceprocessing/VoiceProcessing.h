/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/effect/BnEffect.h>

#include "effect-impl/EffectImpl.h"
#include "effect-impl/EffectUUID.h"
#include "VoiceProcessingContext.h"
#include "VoiceProcessingTypes.h"
#include "GlobalVoiceProcessingSession.h"

namespace aidl::android::hardware::audio::effect {

class VoiceProcessing final : public EffectImpl {
  public:
    const Descriptor* mDescriptor;
    const std::string* mEffectName;
    VoiceProcessingType mType = VoiceProcessingType::AcousticEchoCanceler;

    VoiceProcessing(const AudioUuid& uuid);
    ~VoiceProcessing() {
        cleanUp();
        LOG(DEBUG) << __func__;
    }

    ndk::ScopedAStatus commandImpl(CommandId command) override;
    ndk::ScopedAStatus getDescriptor(Descriptor* _aidl_return) override;

    ndk::ScopedAStatus setParameterSpecific(const Parameter::Specific& specific) override;
    ndk::ScopedAStatus getParameterSpecific(const Parameter::Id& id,
                                            Parameter::Specific* specific) override;

    std::shared_ptr<EffectContext> createContext(const Parameter::Common& common) override;
    std::shared_ptr<EffectContext> getContext() override;
    RetCode releaseContext() override;

    std::string getEffectName() override { return *mEffectName; };
    IEffect::Status effectProcessImpl(float* in, float* out, int samples) override;

  private:
    std::shared_ptr<VoiceProcessingContext> mContext;
};
}  // namespace aidl::android::hardware::audio::effect