/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Visualizer"

#include <android-base/logging.h>
#include "VisualizerOffload.h"

using aidl::android::hardware::audio::effect::Descriptor;
using aidl::android::hardware::audio::effect::Capability;
using aidl::android::hardware::audio::effect::Range;
using aidl::android::hardware::audio::effect::IEffect;
using aidl::android::hardware::audio::effect::Flags;
using aidl::qti::effects::VisualizerOffload;
using aidl::qti::effects::kVisualizerOffloadQtiUUID;
using aidl::android::hardware::audio::effect::State;
using aidl::android::media::audio::common::AudioUuid;

// static std::string printAudioUuid(const AudioUuid &uuid) {
//     std::ostringstream os;
//     char str[64];
//     snprintf(str, sizeof(str), "%08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
//              uuid.timeLow, uuid.timeMid, uuid.timeHiAndVersion, uuid.clockSeq,
//              uuid.node[0], uuid.node[1], uuid.node[2], uuid.node[3], uuid.node[4], uuid.node[5]);

//     os << "Uuid : [";
//     os << str;
//     os << "]";
//     return os.str();
// }

extern "C" binder_exception_t createEffect(const AudioUuid* in_impl_uuid,
                                           std::shared_ptr<IEffect>* instanceSpp) {
    if (!in_impl_uuid || *in_impl_uuid != kVisualizerOffloadQtiUUID) {
        LOG(ERROR) << __func__ << "uuid not supported" << aidl::qti::effects::toString (*in_impl_uuid) << "vs " << aidl::qti::effects::toString (kVisualizerOffloadQtiUUID);
        return EX_ILLEGAL_ARGUMENT;
    }
    if (instanceSpp) {
        *instanceSpp = ndk::SharedRefBase::make<VisualizerOffload>();
        LOG(DEBUG) << __func__ << " instance " << instanceSpp->get() << " created";
        return EX_NONE;
    } else {
        LOG(ERROR) << __func__ << " invalid input parameter!";
        return EX_ILLEGAL_ARGUMENT;
    }
}

extern "C" binder_exception_t queryEffect(const AudioUuid* in_impl_uuid, Descriptor* _aidl_return) {
    if (!in_impl_uuid || *in_impl_uuid != kVisualizerOffloadQtiUUID) {
        LOG(ERROR) << __func__ << "uuid not supported" << aidl::qti::effects::toString (*in_impl_uuid) << "vs " << aidl::qti::effects::toString (kVisualizerOffloadQtiUUID);
        return EX_ILLEGAL_ARGUMENT;
    }
    *_aidl_return = VisualizerOffload::kDescriptor;
    return EX_NONE;
}

namespace aidl::qti::effects {

const std::string VisualizerOffload::kEffectName = "Visualizer";
const std::vector<Range::VisualizerRange> VisualizerOffload::kRanges = {
        MAKE_RANGE(Visualizer, latencyMs, 0, VisualizerOffloadContext::kMaxLatencyMs),
        MAKE_RANGE(Visualizer, captureSamples, 0, VisualizerOffloadContext::kMaxCaptureBufSize),
        /* get only parameters, set invalid range (min > max) to indicate not support set */
        MAKE_RANGE(Visualizer, measurement, Visualizer::Measurement({.peak = 1, .rms = 1}),
                   Visualizer::Measurement({.peak = 0, .rms = 0})),
        MAKE_RANGE(Visualizer, captureSampleBuffer, std::vector<uint8_t>({1}),
                   std::vector<uint8_t>({0}))};
const Capability VisualizerOffload::kCapability = {
        .range = Range::make<Range::visualizer>(VisualizerOffload::kRanges)};
const Descriptor VisualizerOffload::kDescriptor = {
        .common = {.id = {.type = kVisualizerTypeUUID,
                          .uuid = kVisualizerOffloadQtiUUID,
                          .proxy = std::nullopt},
                   .flags = {.type = Flags::Type::INSERT,
                             .insert = Flags::Insert::LAST,
                             .volume = Flags::Volume::CTRL},
                   .name = VisualizerOffload::kEffectName,
                   .implementor = "The Android Open Source Project"},
        .capability = VisualizerOffload::kCapability};

ndk::ScopedAStatus VisualizerOffload::getDescriptor(Descriptor* _aidl_return) {
    RETURN_IF(!_aidl_return, EX_ILLEGAL_ARGUMENT, "Parameter:nullptr");
    LOG(DEBUG) << __func__ << kDescriptor.toString();
    *_aidl_return = kDescriptor;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VisualizerOffload::commandImpl(CommandId command) {
    RETURN_IF(!mContext, EX_NULL_POINTER, "nullContext");
    switch (command) {
        case CommandId::START:
            mContext->enable();
            break;
        case CommandId::STOP:
            mContext->disable();
            break;
        case CommandId::RESET:
            mContext->disable();
            mContext->resetBuffer();
            break;
        default:
            LOG(ERROR) << __func__ << " commandId " << toString(command) << " not supported";
            return ndk::ScopedAStatus::fromExceptionCodeWithMessage(EX_ILLEGAL_ARGUMENT,
                                                                    "commandIdNotSupported");
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VisualizerOffload::setParameterSpecific(const Parameter::Specific& specific) {
    RETURN_IF(Parameter::Specific::visualizer != specific.getTag(), EX_ILLEGAL_ARGUMENT,
              "EffectNotSupported");
    RETURN_IF(!mContext, EX_NULL_POINTER, "nullContext");

    auto& param = specific.get<Parameter::Specific::visualizer>();
    RETURN_IF(!inRange(param, kRanges), EX_ILLEGAL_ARGUMENT, "outOfRange");
    const auto tag = param.getTag();
    switch (tag) {
        case Visualizer::captureSamples: {
            RETURN_IF(mContext->setCaptureSamples(param.get<Visualizer::captureSamples>()) !=
                              RetCode::SUCCESS,
                      EX_ILLEGAL_ARGUMENT, "setCaptureSizeFailed");
            return ndk::ScopedAStatus::ok();
        }
        case Visualizer::scalingMode: {
            RETURN_IF(mContext->setScalingMode(param.get<Visualizer::scalingMode>()) !=
                              RetCode::SUCCESS,
                      EX_ILLEGAL_ARGUMENT, "setScalingModeFailed");
            return ndk::ScopedAStatus::ok();
        }
        case Visualizer::measurementMode: {
            RETURN_IF(mContext->setMeasurementMode(param.get<Visualizer::measurementMode>()) !=
                              RetCode::SUCCESS,
                      EX_ILLEGAL_ARGUMENT, "setMeasurementModeFailed");
            return ndk::ScopedAStatus::ok();
        }
        case Visualizer::latencyMs: {
            RETURN_IF(mContext->setDownstreamLatency(param.get<Visualizer::latencyMs>()) !=
                              RetCode::SUCCESS,
                      EX_ILLEGAL_ARGUMENT, "setLatencyFailed");
            return ndk::ScopedAStatus::ok();
        }
        default: {
            LOG(ERROR) << __func__ << " unsupported tag: " << toString(tag);
            return ndk::ScopedAStatus::fromExceptionCodeWithMessage(
                    EX_ILLEGAL_ARGUMENT, "VisualizerTagNotSupported");
        }
    }
}

ndk::ScopedAStatus VisualizerOffload::getParameterSpecific(const Parameter::Id& id,
                                                        Parameter::Specific* specific) {
    RETURN_IF(!specific, EX_NULL_POINTER, "nullPtr");
    auto tag = id.getTag();
    RETURN_IF(Parameter::Id::visualizerTag != tag, EX_ILLEGAL_ARGUMENT, "wrongIdTag");
    auto specificId = id.get<Parameter::Id::visualizerTag>();
    auto specificTag = specificId.getTag();
    switch (specificTag) {
        case Visualizer::Id::commonTag: {
            return getParameterVisualizer(specificId.get<Visualizer::Id::commonTag>(), specific);
        }
        default: {
            LOG(ERROR) << __func__ << " unsupported tag: " << toString(specificTag);
            return ndk::ScopedAStatus::fromExceptionCodeWithMessage(EX_ILLEGAL_ARGUMENT,
                                                                    "VisualizerTagNotSupported");
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VisualizerOffload::getParameterVisualizer(const Visualizer::Tag& tag,
                                                          Parameter::Specific* specific) {
    RETURN_IF(!mContext, EX_NULL_POINTER, "nullContext");

    Visualizer param;
    switch (tag) {
        case Visualizer::captureSamples: {
            param.set<Visualizer::captureSamples>(mContext->getCaptureSamples());
            break;
        }
        case Visualizer::scalingMode: {
            param.set<Visualizer::scalingMode>(mContext->getScalingMode());
            break;
        }
        case Visualizer::measurementMode: {
            param.set<Visualizer::measurementMode>(mContext->getMeasurementMode());
            break;
        }
        case Visualizer::measurement: {
            param.set<Visualizer::measurement>(mContext->getMeasure());
            break;
        }
        case Visualizer::captureSampleBuffer: {
            param.set<Visualizer::captureSampleBuffer>(mContext->capture());
            break;
        }
        case Visualizer::latencyMs: {
            param.set<Visualizer::latencyMs>(mContext->getDownstreamLatency());
            break;
        }
        default: {
            LOG(ERROR) << __func__ << " unsupported tag: " << toString(tag);
            return ndk::ScopedAStatus::fromExceptionCodeWithMessage(
                    EX_ILLEGAL_ARGUMENT, "VisualizerTagNotSupported");
        }
    }

    specific->set<Parameter::Specific::visualizer>(param);
    return ndk::ScopedAStatus::ok();
}

std::shared_ptr<EffectContext> VisualizerOffload::createContext(const Parameter::Common& common) {
    if (mContext) {
        LOG(DEBUG) << __func__ << " context already exist";
        return mContext;
    }

    mContext = std::make_shared<VisualizerOffloadContext>(1 /* statusFmqDepth */, common);
    mContext->initParams(common);
    return mContext;
}

RetCode VisualizerOffload::releaseContext() {
    if (mContext) {
        mContext->disable();
        mContext->resetBuffer();
    }
    return RetCode::SUCCESS;
}

// Processing method running in EffectWorker thread.
IEffect::Status VisualizerOffload::effectProcessImpl(float* in, float* out, int samples) {
    IEffect::Status status = {EX_NULL_POINTER, 0, 0};
    RETURN_VALUE_IF(!mContext, status, "nullContext");
    return mContext->process(in, out, samples);
}

}  // namespace aidl::qti::effects
