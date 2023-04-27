/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once
#include <functional>
#include <map>
#include <memory>

#include <aidl/android/hardware/audio/effect/BnEffect.h>
#include <android-base/logging.h>

#include "effect-impl/EffectImpl.h"
#include "effect-impl/EffectUUID.h"

#include "OffloadBundleContext.h"
#include "OffloadBundleTypes.h"
#include "GlobalOffloadSession.h"

namespace aidl::android::hardware::audio::effect {

class OffloadBundleAidl final : public EffectImpl {
  public:
    explicit OffloadBundleAidl(const AudioUuid& uuid);
    ~OffloadBundleAidl() override;

    ndk::ScopedAStatus getDescriptor(Descriptor* _aidl_return) override;
    ndk::ScopedAStatus setParameterCommon(const Parameter& param) override;
    ndk::ScopedAStatus setParameterSpecific(const Parameter::Specific& specific) override;
    ndk::ScopedAStatus getParameterSpecific(const Parameter::Id& id,
                                            Parameter::Specific* specific) override;

    std::shared_ptr<EffectContext> createContext(const Parameter::Common& common) override;
    std::shared_ptr<EffectContext> getContext() override;
    RetCode releaseContext() override;

    IEffect::Status effectProcessImpl(float* in, float* out, int samples) override;

    ndk::ScopedAStatus commandImpl(CommandId command) override;

    std::string getEffectName() override { return *mEffectName; }

  private:
    std::shared_ptr<OffloadBundleContext> mContext;
    const Descriptor* mDescriptor;
    const std::string* mEffectName;
    OffloadBundleEffectType mType = OffloadBundleEffectType::EQUALIZER;

    IEffect::Status status(binder_status_t status, size_t consumed, size_t produced);

    ndk::ScopedAStatus setParameterBassBoost(const Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterBassBoost(const BassBoost::Id& id,
                                             Parameter::Specific* specific);

    ndk::ScopedAStatus setParameterEqualizer(const Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterEqualizer(const Equalizer::Id& id,
                                             Parameter::Specific* specific);
    ndk::ScopedAStatus setParameterVirtualizer(const Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterVirtualizer(const Virtualizer::Id& id,
                                               Parameter::Specific* specific);
};

}  // namespace aidl::android::hardware::audio::effect
