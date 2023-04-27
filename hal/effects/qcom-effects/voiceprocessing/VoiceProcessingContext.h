/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include "VoiceProcessingTypes.h"
#include "effect-impl/EffectContext.h"

namespace aidl::android::hardware::audio::effect {

enum VoiceProcessingState {
    UNINITIALIZED,
    INITIALIZED,
    ACTIVE,
};

class VoiceProcessingContext final : public EffectContext {
  public:
    VoiceProcessingContext(int statusDepth, const Parameter::Common& common, const
                            VoiceProcessingType & type);
    ~VoiceProcessingContext();
    VoiceProcessingType getVoiceProcessingType() const { return mType; }
    RetCode enable();
    RetCode disable();
    void reset();
  private:
    VoiceProcessingState mState = UNINITIALIZED;
    VoiceProcessingType mType;
};

}  // namespace aidl::android::hardware::audio::effect