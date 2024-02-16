/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <memory>
#define LOG_TAG "AHAL_EffectContextQti"
#include "effect-impl/EffectContext.h"
#include "include/effect-impl/EffectTypes.h"

using aidl::android::hardware::audio::common::getChannelCount;
using aidl::android::hardware::audio::common::getFrameSizeInBytes;
using aidl::android::hardware::audio::effect::IEffect;
using aidl::android::media::audio::common::PcmType;
using ::android::hardware::EventFlag;

namespace aidl::qti::effects {

EffectContext::EffectContext(const Parameter::Common &common, bool processData) {
    mCommon = common;
    initMessageQueues(processData);
}

EffectContext::~EffectContext() {}

void EffectContext::initMessageQueues(bool processData) {
    if (processData) {
        auto &input = mCommon.input;
        auto &output = mCommon.output;

        LOG_ALWAYS_FATAL_IF(
                input.base.format.pcm != aidl::android::media::audio::common::PcmType::FLOAT_32_BIT,
                "inputFormatNotFloat");
        LOG_ALWAYS_FATAL_IF(output.base.format.pcm !=
                                    aidl::android::media::audio::common::PcmType::FLOAT_32_BIT,
                            "outputFormatNotFloat");
        mInputFrameSize = ::aidl::android::hardware::audio::common::getFrameSizeInBytes(
                input.base.format, input.base.channelMask);
        mOutputFrameSize = ::aidl::android::hardware::audio::common::getFrameSizeInBytes(
                output.base.format, output.base.channelMask);
        // in/outBuffer size in float (FMQ data format defined for DataMQ)
        size_t inBufferSizeInFloat = input.frameCount * mInputFrameSize / sizeof(float);
        size_t outBufferSizeInFloat = output.frameCount * mOutputFrameSize / sizeof(float);

        // only status FMQ use the EventFlag
        mStatusMQ = std::make_shared<StatusMQ>(1 /*numElementsInQueue*/,
                                               true /*configureEventFlagWord*/);
        mInputMQ = std::make_shared<DataMQ>(inBufferSizeInFloat);
        mOutputMQ = std::make_shared<DataMQ>(outBufferSizeInFloat);

        if (!mStatusMQ->isValid() || !mInputMQ->isValid() || !mOutputMQ->isValid()) {
            LOG(ERROR) << __func__ << " created invalid FMQ";
        }
        mWorkBuffer.reserve(std::max(inBufferSizeInFloat, outBufferSizeInFloat));
    }
    mProcessData = processData;
    LOG(ERROR) << __func__ << " context created";
}

// reset buffer status by abandon input data in FMQ
void EffectContext::resetBuffer() {
    if (mProcessData) {
        auto buffer = static_cast<float *>(mWorkBuffer.data());
        std::vector<IEffect::Status> status(mStatusMQ->availableToRead());
        mInputMQ->read(buffer, mInputMQ->availableToRead());
    }
}

void EffectContext::dupeFmq(IEffect::OpenEffectReturn *effectRet) {
    if (effectRet && mProcessData) {
        effectRet->statusMQ = mStatusMQ->dupeDesc();
        effectRet->inputDataMQ = mInputMQ->dupeDesc();
        effectRet->outputDataMQ = mOutputMQ->dupeDesc();
    }
}

float *EffectContext::getWorkBuffer() {
    return static_cast<float *>(mWorkBuffer.data());
}
std::shared_ptr<EffectContext::StatusMQ> EffectContext::getStatusFmq() const {
    return mStatusMQ;
}

std::shared_ptr<EffectContext::DataMQ> EffectContext::getInputDataFmq() const {
    return mInputMQ;
}

std::shared_ptr<EffectContext::DataMQ> EffectContext::getOutputDataFmq() const {
    return mOutputMQ;
}

size_t EffectContext::getInputFrameSize() const {
    return mInputFrameSize;
}

size_t EffectContext::getOutputFrameSize() const {
    return mOutputFrameSize;
}

int EffectContext::getSessionId() const {
    return mCommon.session;
}

int EffectContext::getIoHandle() const {
    return mCommon.ioHandle;
}

RetCode EffectContext::setOutputDevice(
        const std::vector<aidl::android::media::audio::common::AudioDeviceDescription> &device) {
    mOutputDevice = device;
    return RetCode::SUCCESS;
}

std::vector<aidl::android::media::audio::common::AudioDeviceDescription>
        EffectContext::getOutputDevice() {
    return mOutputDevice;
}

RetCode EffectContext::setAudioMode(const aidl::android::media::audio::common::AudioMode &mode) {
    mMode = mode;
    return RetCode::SUCCESS;
}
aidl::android::media::audio::common::AudioMode EffectContext::getAudioMode() {
    return mMode;
}

RetCode EffectContext::setAudioSource(
        const aidl::android::media::audio::common::AudioSource &source) {
    mSource = source;
    return RetCode::SUCCESS;
}

aidl::android::media::audio::common::AudioSource EffectContext::getAudioSource() {
    return mSource;
}

RetCode EffectContext::setVolumeStereo(const Parameter::VolumeStereo &volumeStereo) {
    mVolumeStereo = volumeStereo;
    return RetCode::SUCCESS;
}

Parameter::VolumeStereo EffectContext::getVolumeStereo() {
    return mVolumeStereo;
}

RetCode EffectContext::setCommon(const Parameter::Common &common) {
    mCommon = common;
    LOG(INFO) << __func__ << mCommon.toString();
    return RetCode::SUCCESS;
}

Parameter::Common EffectContext::getCommon() {
    LOG(INFO) << __func__ << mCommon.toString();
    return mCommon;
}

RetCode EffectContext::setOffload(bool offload) {
    mOffload = offload;
    return RetCode::SUCCESS;
}

} // namespace aidl::qti::effects