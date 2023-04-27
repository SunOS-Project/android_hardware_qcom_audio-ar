/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#pragma once
#include <Utils.h>
#include <memory>
#include <vector>

#include <android-base/logging.h>
#include <fmq/AidlMessageQueue.h>

#include <aidl/android/hardware/audio/effect/BnEffect.h>
#include "EffectTypes.h"

namespace aidl::android::hardware::audio::effect {

class EffectContext {
  public:
    typedef ::android::AidlMessageQueue<
            IEffect::Status, ::aidl::android::hardware::common::fmq::SynchronizedReadWrite>
            StatusMQ;
    typedef ::android::AidlMessageQueue<
            float, ::aidl::android::hardware::common::fmq::SynchronizedReadWrite>
            DataMQ;

    EffectContext(size_t statusDepth, const Parameter::Common& common) {
        auto& input = common.input;
        auto& output = common.output;

        LOG_ALWAYS_FATAL_IF(
                input.base.format.pcm != aidl::android::media::audio::common::PcmType::FLOAT_32_BIT,
                "inputFormatNotFloat");
        LOG_ALWAYS_FATAL_IF(output.base.format.pcm !=
                                    aidl::android::media::audio::common::PcmType::FLOAT_32_BIT,
                            "outputFormatNotFloat");
        mInputFrameSize = ::android::hardware::audio::common::getFrameSizeInBytes(
                input.base.format, input.base.channelMask);
        mOutputFrameSize = ::android::hardware::audio::common::getFrameSizeInBytes(
                output.base.format, output.base.channelMask);
        // in/outBuffer size in float (FMQ data format defined for DataMQ)
        size_t inBufferSizeInFloat = input.frameCount * mInputFrameSize / sizeof(float);
        size_t outBufferSizeInFloat = output.frameCount * mOutputFrameSize / sizeof(float);

        mStatusMQ = std::make_shared<StatusMQ>(statusDepth, true /*configureEventFlagWord*/);
        mInputMQ = std::make_shared<DataMQ>(inBufferSizeInFloat);
        mOutputMQ = std::make_shared<DataMQ>(outBufferSizeInFloat);

        if (!mStatusMQ->isValid() || !mInputMQ->isValid() || !mOutputMQ->isValid()) {
            LOG(ERROR) << __func__ << " created invalid FMQ";
        }
        mWorkBuffer.reserve(std::max(inBufferSizeInFloat, outBufferSizeInFloat));
        mCommon = common;
    }
    virtual ~EffectContext() {}

    std::shared_ptr<StatusMQ> getStatusFmq() { return mStatusMQ; }
    std::shared_ptr<DataMQ> getInputDataFmq() { return mInputMQ; }
    std::shared_ptr<DataMQ> getOutputDataFmq() { return mOutputMQ; }

    float* getWorkBuffer() { return static_cast<float*>(mWorkBuffer.data()); }

    // reset buffer status by abandon input data in FMQ
    void resetBuffer() {
        auto buffer = static_cast<float*>(mWorkBuffer.data());
        std::vector<IEffect::Status> status(mStatusMQ->availableToRead());
        mInputMQ->read(buffer, mInputMQ->availableToRead());
    }

    void dupeFmq(IEffect::OpenEffectReturn* effectRet) {
        if (effectRet) {
            effectRet->statusMQ = mStatusMQ->dupeDesc();
            effectRet->inputDataMQ = mInputMQ->dupeDesc();
            effectRet->outputDataMQ = mOutputMQ->dupeDesc();
        }
    }
    size_t getInputFrameSize() { return mInputFrameSize; }
    size_t getOutputFrameSize() { return mOutputFrameSize; }
    int getSessionId() { return mCommon.session; }
    int getIoHandle() { return mCommon.ioHandle; }

    virtual RetCode setOutputDevice(
            const std::vector<aidl::android::media::audio::common::AudioDeviceDescription>&
                    device) {
        mOutputDevice = device;
        return RetCode::SUCCESS;
    }

    virtual std::vector<aidl::android::media::audio::common::AudioDeviceDescription>
    getOutputDevice() {
        return mOutputDevice;
    }

    virtual RetCode setAudioMode(const aidl::android::media::audio::common::AudioMode& mode) {
        mMode = mode;
        return RetCode::SUCCESS;
    }
    virtual aidl::android::media::audio::common::AudioMode getAudioMode() { return mMode; }

    virtual RetCode setAudioSource(const aidl::android::media::audio::common::AudioSource& source) {
        mSource = source;
        return RetCode::SUCCESS;
    }
    virtual aidl::android::media::audio::common::AudioSource getAudioSource() { return mSource; }

    virtual RetCode setVolumeStereo(const Parameter::VolumeStereo& volumeStereo) {
        mVolumeStereo = volumeStereo;
        return RetCode::SUCCESS;
    }
    virtual Parameter::VolumeStereo getVolumeStereo() { return mVolumeStereo; }

    virtual RetCode setCommon(const Parameter::Common& common) {
        mCommon = common;
        LOG(INFO) << __func__ << mCommon.toString();
        return RetCode::SUCCESS;
    }
    virtual Parameter::Common getCommon() {
        LOG(INFO) << __func__ << mCommon.toString();
        return mCommon;
    }

  protected:
    // common parameters
    size_t mInputFrameSize;
    size_t mOutputFrameSize;
    Parameter::Common mCommon;
    std::vector<aidl::android::media::audio::common::AudioDeviceDescription> mOutputDevice;
    aidl::android::media::audio::common::AudioMode mMode;
    aidl::android::media::audio::common::AudioSource mSource;
    Parameter::VolumeStereo mVolumeStereo;

  private:
    // fmq and buffers
    std::shared_ptr<StatusMQ> mStatusMQ;
    std::shared_ptr<DataMQ> mInputMQ;
    std::shared_ptr<DataMQ> mOutputMQ;
    // TODO handle effect process input and output
    // work buffer set by effect instances, the access and update are in same thread
    std::vector<float> mWorkBuffer;
};
}  // namespace aidl::android::hardware::audio::effect
