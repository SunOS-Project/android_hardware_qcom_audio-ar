/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_VoiceProcessing"

#include <android-base/logging.h>
#include <Utils.h>
#include "VoiceProcessing.h"

using aidl::android::hardware::audio::effect::Descriptor;
using aidl::android::hardware::audio::effect::IEffect;
using aidl::android::hardware::audio::effect::kAcousticEchoCancelerQtiUUID;
using aidl::android::hardware::audio::effect::kNoiseSuppressionQtiUUID;
using aidl::android::media::audio::common::AudioUuid;
using aidl::android::hardware::audio::effect::VoiceProcessing;

bool isUuidSupported(const AudioUuid* uuid) {
    return (*uuid == kAcousticEchoCancelerQtiUUID || *uuid == kNoiseSuppressionQtiUUID);
}

extern "C" binder_exception_t createEffect(const AudioUuid* uuid,
                                           std::shared_ptr<IEffect>* instanceSpp) {
    if (!uuid || !isUuidSupported(uuid)) {
        LOG(ERROR) << __func__ << "uuid not supported";
        return EX_ILLEGAL_ARGUMENT;
    }
    if (instanceSpp) {
        *instanceSpp = ndk::SharedRefBase::make<VoiceProcessing>(*uuid);
        LOG(DEBUG) << __func__ << " instance " << instanceSpp->get() << " created";
        return EX_NONE;
    } else {
        LOG(ERROR) << __func__ << " invalid input parameter!";
        return EX_ILLEGAL_ARGUMENT;
    }
}

extern "C" binder_exception_t queryEffect(const AudioUuid* uuid, Descriptor* _aidl_return) {
    if (uuid == nullptr || !isUuidSupported(uuid)) {
        LOG(ERROR) << __func__ << "uuid not supported";
        return EX_ILLEGAL_ARGUMENT;
    }
    if (*uuid == kAcousticEchoCancelerQtiUUID) {
        *_aidl_return = aidl::android::hardware::audio::effect::kAcousticEchoCancelerDesc;
    } else if (*uuid == kNoiseSuppressionQtiUUID) {
        *_aidl_return = aidl::android::hardware::audio::effect::kNoiseSuppressionDesc;
    }
    return EX_NONE;
}

namespace aidl::android::hardware::audio::effect {

VoiceProcessing::VoiceProcessing(const AudioUuid& uuid) {
    LOG(DEBUG) << __func__ << uuid.toString();
    if (uuid == kAcousticEchoCancelerQtiUUID) {
        mType = VoiceProcessingType::AcousticEchoCanceler;
        mDescriptor = &kAcousticEchoCancelerDesc;
        mEffectName = &kAcousticEchoCancelerEffectName;
    } else if (uuid == kNoiseSuppressionQtiUUID) {
        mType = VoiceProcessingType::NoiseSuppression;
        mDescriptor = &kNoiseSuppressionDesc;
        mEffectName = &kNoiseSuppressionEffectName;
    } else {
        LOG(ERROR) << __func__ << uuid.toString() << " not supported yet!";
    }
}

ndk::ScopedAStatus VoiceProcessing::getDescriptor(Descriptor* _aidl_return) {
    RETURN_IF(!_aidl_return, EX_ILLEGAL_ARGUMENT, "Parameter:nullptr");
    //LOG(DEBUG) << __func__ << *mDescriptor.toString();
    *_aidl_return = *mDescriptor;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VoiceProcessing::commandImpl(CommandId command) {
    RETURN_IF(!mContext, EX_NULL_POINTER, "nullContext");
    switch (command) {
        case CommandId::START:
            mContext->enable();
            break;
        case CommandId::STOP:
            mContext->disable();
            break;
        case CommandId::RESET:
            mContext->reset();
            break;
        default:
            LOG(ERROR) << __func__ << " commandId " << toString(command) << " not supported";
            return ndk::ScopedAStatus::fromExceptionCodeWithMessage(EX_ILLEGAL_ARGUMENT,
                                                                    "commandIdNotSupported");
    }
    return ndk::ScopedAStatus::ok();
}

std::shared_ptr<EffectContext> VoiceProcessing::createContext(const Parameter::Common& common) {
    if (mContext) {
        LOG(DEBUG) << __func__ << " context already exist";
    } else {
        // GlobalVoiceProcessingSession is a singleton
        mContext = GlobalVoiceProcessingSession::getSession().createSession(mType, 1 /* statusFmqDepth */,
                                                                   common);
    }

    return mContext;
}

std::shared_ptr<EffectContext> VoiceProcessing::getContext() {
    return mContext;
}

RetCode VoiceProcessing::releaseContext() {
    if (mContext) {
        GlobalVoiceProcessingSession::getSession().releaseSession(mType, mContext->getSessionId());
        mContext.reset();
    }
    return RetCode::SUCCESS;
}

ndk::ScopedAStatus VoiceProcessing::setParameterSpecific(const Parameter::Specific& specific) {
    return ndk::ScopedAStatus::ok();
}


ndk::ScopedAStatus VoiceProcessing::getParameterSpecific(const Parameter::Id& id,
                                                      Parameter::Specific* specific) {
    return ndk::ScopedAStatus::ok();
}

// Processing method running in EffectWorker thread.
IEffect::Status VoiceProcessing::effectProcessImpl(float* in, float* out, int sampleToProcess) {
    if (!mContext) {
        LOG(ERROR) << __func__ << " nullContext";
        return {EX_NULL_POINTER, 0, 0};
    }
    return {EX_NONE, 0, 0};
}

}  // namespace aidl::android::hardware::audio::effect