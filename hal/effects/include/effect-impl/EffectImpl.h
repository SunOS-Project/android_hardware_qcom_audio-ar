/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#pragma once
#include <cstdlib>
#include <memory>

#include <aidl/android/hardware/audio/effect/BnEffect.h>
#include <fmq/AidlMessageQueue.h>

#include "EffectContext.h"
#include "EffectThread.h"
#include "EffectTypes.h"
#include "effect-impl/EffectContext.h"
#include "effect-impl/EffectThread.h"
#include "effect-impl/EffectTypes.h"

extern "C" binder_exception_t destroyEffect(
        const std::shared_ptr<aidl::android::hardware::audio::effect::IEffect>& instanceSp);

using aidl::android::hardware::audio::effect::IEffect;
using aidl::android::hardware::audio::effect::BnEffect;
using aidl::android::hardware::audio::effect::Parameter;
using aidl::android::hardware::audio::effect::CommandId;
using aidl::android::hardware::audio::effect::Descriptor;
using aidl::android::hardware::audio::effect::State;

namespace aidl::qti::effects {

class EffectImpl : public BnEffect, public EffectThread {
  public:
    EffectImpl() = default;
    virtual ~EffectImpl() = default;

    virtual ndk::ScopedAStatus open(const Parameter::Common& common,
                                    const std::optional<Parameter::Specific>& specific,
                                    OpenEffectReturn* ret) override;
    virtual ndk::ScopedAStatus close() override;
    virtual ndk::ScopedAStatus command(CommandId id) override;

    virtual ndk::ScopedAStatus getState(State* state) override;
    virtual ndk::ScopedAStatus setParameter(const Parameter& param) override;
    virtual ndk::ScopedAStatus getParameter(const Parameter::Id& id, Parameter* param) override;

    virtual ndk::ScopedAStatus setParameterCommon(const Parameter& param);
    virtual ndk::ScopedAStatus getParameterCommon(const Parameter::Tag& tag, Parameter* param);

    /* Methods MUST be implemented by each effect instances */
    virtual ndk::ScopedAStatus getDescriptor(Descriptor* desc) = 0;
    virtual ndk::ScopedAStatus setParameterSpecific(const Parameter::Specific& specific) = 0;
    virtual ndk::ScopedAStatus getParameterSpecific(const Parameter::Id& id,
                                                    Parameter::Specific* specific) = 0;

    virtual std::string getEffectName() = 0;
    virtual IEffect::Status effectProcessImpl(float* in, float* out, int samples) override;

    /**
     * Effect context methods must be implemented by each effect.
     * Each effect can derive from EffectContext and define its own context, but must upcast to
     * EffectContext for EffectImpl to use.
     */
    virtual std::shared_ptr<EffectContext> createContext(const Parameter::Common& common) = 0;
    virtual std::shared_ptr<EffectContext> getContext() = 0;
    virtual RetCode releaseContext() = 0;

  protected:
    State mState = State::INIT;

    IEffect::Status status(binder_status_t status, size_t consumed, size_t produced);
    void cleanUp();

    /**
     * Optional CommandId handling methods for effects to override.
     * For CommandId::START, EffectImpl call commandImpl before starting the EffectThread
     * processing.
     * For CommandId::STOP and CommandId::RESET, EffectImpl call commandImpl after stop the
     * EffectThread processing.
     */
    virtual ndk::ScopedAStatus commandImpl(CommandId id);
};
}  // namespace aidl::qti::effects
