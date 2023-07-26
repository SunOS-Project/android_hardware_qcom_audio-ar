/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/effect/BnEffect.h>
#include <fmq/AidlMessageQueue.h>
#include <cstdlib>
#include <memory>

#include "effect-impl/EffectImpl.h"
#include "effect-impl/EffectUUID.h"

#include "VolumeListenerContext.h"
#include "VolumeListenerTypes.h"
#include "GlobalVolumeListenerSession.h"
namespace aidl::qti::effects {

class VolumeListener final : public EffectImpl {
  public:
    const Descriptor* mDescriptor;
    const std::string* mEffectName;
    VolumeListenerType mType = VolumeListenerType::ALARM;

    VolumeListener(const AudioUuid& uuid);
    ~VolumeListener() {
        cleanUp();
        LOG(DEBUG) << __func__;
    }

    ndk::ScopedAStatus commandImpl(CommandId command) override;
    ndk::ScopedAStatus getDescriptor(Descriptor* _aidl_return) override;

    std::shared_ptr<EffectContext> createContext(const Parameter::Common& common) override;
    std::shared_ptr<EffectContext> getContext() override;
    RetCode releaseContext() override;

    ndk::ScopedAStatus setParameterSpecific(const Parameter::Specific& specific) override;
    ndk::ScopedAStatus getParameterSpecific(const Parameter::Id& id,
                                            Parameter::Specific* specific) override;


    std::string getEffectName() override { return * mEffectName; };
    IEffect::Status effectProcessImpl(float* in, float* out, int samples) override;

  private:
    std::shared_ptr<VolumeListenerContext> mContext;

};
}  // namespace aidl::qti::effects
