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

namespace aidl::qti::effects {

class OffloadBundleAidl final : public EffectImpl {
  public:
    explicit OffloadBundleAidl(const AudioUuid& uuid);
    ~OffloadBundleAidl() override;

    ndk::ScopedAStatus getDescriptor(aidl::android::hardware::audio::effect::Descriptor* _aidl_return) override;
    ndk::ScopedAStatus setParameterCommon(const aidl::android::hardware::audio::effect::Parameter& param) override;
    ndk::ScopedAStatus setParameterSpecific(const aidl::android::hardware::audio::effect::Parameter::Specific& specific) override;
    ndk::ScopedAStatus getParameterSpecific(const aidl::android::hardware::audio::effect::Parameter::Id& id,
                                            aidl::android::hardware::audio::effect::Parameter::Specific* specific) override;

    std::shared_ptr<EffectContext> createContext(const aidl::android::hardware::audio::effect::Parameter::Common& common) override;
    std::shared_ptr<EffectContext> getContext() override;
    RetCode releaseContext() override;

    aidl::android::hardware::audio::effect::IEffect::Status effectProcessImpl(float* in, float* out, int samples) override;

    ndk::ScopedAStatus commandImpl(aidl::android::hardware::audio::effect::CommandId command) override;

    std::string getEffectName() override { return *mEffectName; }

    ndk::ScopedAStatus addEffect(uint64_t *palHandle) override;
    ndk::ScopedAStatus removeEffect() override;

  private:
    std::shared_ptr<OffloadBundleContext> mContext;
    const aidl::android::hardware::audio::effect::Descriptor* mDescriptor;
    const std::string* mEffectName;
    OffloadBundleEffectType mType = OffloadBundleEffectType::EQUALIZER;

    aidl::android::hardware::audio::effect::IEffect::Status status(binder_status_t status, size_t consumed, size_t produced);

    ndk::ScopedAStatus setParameterBassBoost(const aidl::android::hardware::audio::effect::Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterBassBoost(const aidl::android::hardware::audio::effect::BassBoost::Id& id,
                                             aidl::android::hardware::audio::effect::Parameter::Specific* specific);

    ndk::ScopedAStatus setParameterEqualizer(const aidl::android::hardware::audio::effect::Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterEqualizer(const aidl::android::hardware::audio::effect::Equalizer::Id& id,
                                             aidl::android::hardware::audio::effect::Parameter::Specific* specific);
    ndk::ScopedAStatus setParameterVirtualizer(const aidl::android::hardware::audio::effect::Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterVirtualizer(const aidl::android::hardware::audio::effect::Virtualizer::Id& id,
                                               aidl::android::hardware::audio::effect::Parameter::Specific* specific);
    ndk::ScopedAStatus setParameterPresetReverb(const Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterPresetReverb(const PresetReverb::Id& id,
                                               Parameter::Specific* specific);

    ndk::ScopedAStatus setParameterEnvironmentalReverb(const Parameter::Specific& specific);
    ndk::ScopedAStatus getParameterEnvironmentalReverb(const EnvironmentalReverb::Id& id,
                                                       Parameter::Specific* specific);
};

}  // namespace aidl::qti::effects
