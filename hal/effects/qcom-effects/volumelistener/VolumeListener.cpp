/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <algorithm>
#include <cstddef>
#include <memory>
#define LOG_TAG "AHAL_Effect_VolumeListener"

#include <unordered_set>
#include <Utils.h>
#include <android-base/logging.h>

#include "VolumeListener.h"

using aidl::android::hardware::audio::effect::Descriptor;
using aidl::android::hardware::audio::effect::IEffect;
using aidl::android::hardware::audio::effect::State;
using aidl::qti::effects::kMusicVolumeListenerUUID;
using aidl::qti::effects::kRingVolumeListenerUUID;
using aidl::qti::effects::kAlarmVolumeListenerUUID;
using aidl::qti::effects::kVoiceCallVolumeListenerUUID;
using aidl::qti::effects::kNotificationVolumeListenerUUID;
using aidl::qti::effects::VolumeListener;
using aidl::android::media::audio::common::AudioUuid;

bool isUuidSupported(const AudioUuid* uuid) {
    return (*uuid == kMusicVolumeListenerUUID || *uuid == kRingVolumeListenerUUID ||
            *uuid == kAlarmVolumeListenerUUID || *uuid == kVoiceCallVolumeListenerUUID ||
            *uuid == kNotificationVolumeListenerUUID);
}

extern "C" binder_exception_t createEffect(const AudioUuid* uuid,
                                           std::shared_ptr<IEffect>* instanceSpp) {
    if (uuid == nullptr || !isUuidSupported(uuid)) {
        LOG(ERROR) << __func__ << "uuid not supported";
        return EX_ILLEGAL_ARGUMENT;
    }
    if (instanceSpp) {
        *instanceSpp = ndk::SharedRefBase::make<VolumeListener>(*uuid);
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
    if (*uuid == kAlarmVolumeListenerUUID) {
        *_aidl_return = aidl::qti::effects::kAlarmVolumeListenerDesc;
    } else if (*uuid == kMusicVolumeListenerUUID) {
        *_aidl_return = aidl::qti::effects::kMusicVolumeListenerDesc;
    } else if (*uuid == kNotificationVolumeListenerUUID) {
        *_aidl_return = aidl::qti::effects::kNotificationVolumeListenerDesc;
    } else if (*uuid == kVoiceCallVolumeListenerUUID) {
        *_aidl_return = aidl::qti::effects::kVoiceCallVolumeListenerDesc;
    } else if (*uuid == kRingVolumeListenerUUID) {
        *_aidl_return = aidl::qti::effects::kRingVolumeListenerDesc;
    }
    return EX_NONE;
}

namespace aidl::qti::effects {

VolumeListener::VolumeListener(const AudioUuid& uuid) {
    LOG(DEBUG) << __func__ << uuid.toString();
    if (uuid == kAlarmVolumeListenerUUID) {
        mType = VolumeListenerType::ALARM;
        mDescriptor = &kAlarmVolumeListenerDesc;
        mEffectName = &kAlarmVolumeListenerEffectName;
    } else if (uuid == kMusicVolumeListenerUUID) {
        mType = VolumeListenerType::MUSIC;
        mDescriptor = &kMusicVolumeListenerDesc;
        mEffectName = &kMusicVolumeListenerEffectName;
    } else if (uuid == kNotificationVolumeListenerUUID) {
        mType = VolumeListenerType::NOTIFICATION;
        mDescriptor = &kNotificationVolumeListenerDesc;
        mEffectName = &kNotificationVolumeListenerEffectName;
    } else if (uuid == kVoiceCallVolumeListenerUUID) {
        mType = VolumeListenerType::VOICECALL;
        mDescriptor = &kVoiceCallVolumeListenerDesc;
        mEffectName = &kNotificationVolumeListenerEffectName;
    } else if (uuid == kRingVolumeListenerUUID) {
        mType = VolumeListenerType::RING;
        mDescriptor = &kRingVolumeListenerDesc;
        mEffectName = &kRingVolumeListenerEffectName;
    } else {
        LOG(ERROR) << __func__ << uuid.toString() << " not supported yet!";
    }
}

ndk::ScopedAStatus VolumeListener::getDescriptor(Descriptor* _aidl_return) {
    LOG(DEBUG) << __func__ << (*mDescriptor).toString();
    *_aidl_return = *mDescriptor;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VolumeListener::commandImpl(CommandId command) {
    RETURN_IF(!mContext, EX_NULL_POINTER, "nullContext");
    LOG(VERBOSE) << __func__ << toString(command);
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

std::shared_ptr<EffectContext> VolumeListener::createContext(const Parameter::Common& common) {
    LOG(VERBOSE) << __func__;
    if (mContext) {
        LOG(DEBUG) << __func__ << " context already exist";
    } else {
        // GlobalVolumeListenerSession is a singleton
        mContext = GlobalVolumeListenerSession::getSession().createSession(mType, 1 /* statusFmqDepth */,
                                                                   common);
    }
    return mContext;
}

std::shared_ptr<EffectContext> VolumeListener::getContext() {
    return mContext;
}

RetCode VolumeListener::releaseContext() {
    LOG(VERBOSE) << __func__;
    if (mContext) {
        GlobalVolumeListenerSession::getSession().releaseSession(mContext->getSessionId());
        mContext.reset();
    }
    return RetCode::SUCCESS;
}

ndk::ScopedAStatus VolumeListener::setParameterSpecific(const Parameter::Specific& specific) {
    LOG(VERBOSE) << __func__;
    return ndk::ScopedAStatus::ok();
}


ndk::ScopedAStatus VolumeListener::getParameterSpecific(const Parameter::Id& id,
                                                      Parameter::Specific* specific) {
    LOG(VERBOSE) << __func__;
    return ndk::ScopedAStatus::ok();
}

// Processing method running in EffectWorker thread.
IEffect::Status VolumeListener::effectProcessImpl(float* in, float* out, int samples) {
    // noProcess effect, no need to provide impl
    LOG(VERBOSE) << __func__;
    return {STATUS_OK, samples, samples};
}

}  // namespace aidl::qti::effects
