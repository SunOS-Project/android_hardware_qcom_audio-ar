/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#pragma once
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include "EffectContext.h"
#include "EffectThread.h"

namespace aidl::android::hardware::audio::effect {

std::string toString(RetCode& code);

class EffectWorker : public EffectThread {
  public:
    // set effect context for worker, suppose to only happen once here
    void setContext(std::shared_ptr<EffectContext> context) {
        std::call_once(mOnceFlag, [&]() { mContext = context; });
    };

    // handle FMQ and call effect implemented virtual function
    void process() override {
        RETURN_VALUE_IF(!mContext, void(), "nullContext");
        std::shared_ptr<EffectContext::StatusMQ> statusMQ = mContext->getStatusFmq();
        std::shared_ptr<EffectContext::DataMQ> inputMQ = mContext->getInputDataFmq();
        std::shared_ptr<EffectContext::DataMQ> outputMQ = mContext->getOutputDataFmq();

        // Only this worker will read from input data MQ and write to output data MQ.
        auto readSamples = inputMQ->availableToRead(), writeSamples = outputMQ->availableToWrite();
        if (readSamples && writeSamples) {
            auto processSamples = std::min(readSamples, writeSamples);
            LOG(DEBUG) << __func__ << " available to read " << readSamples << " available to write "
                       << writeSamples << " process " << processSamples;

            auto buffer = mContext->getWorkBuffer();
            inputMQ->read(buffer, processSamples);

            IEffect::Status status = effectProcessImpl(buffer, buffer, processSamples);
            outputMQ->write(buffer, status.fmqProduced);
            statusMQ->writeBlocking(&status, 1);
            LOG(DEBUG) << __func__ << " done processing, effect consumed " << status.fmqConsumed
                       << " produced " << status.fmqProduced;
        } else {
            // TODO: maybe add some sleep here to avoid busy waiting
        }
    }

    // must implement by each effect implementation
    // TODO: consider if this interface need adjustment to handle in-place processing
    virtual IEffect::Status effectProcessImpl(float* in, float* out, int samples) = 0;

  private:
    // make sure the context only set once.
    std::once_flag mOnceFlag;
    std::shared_ptr<EffectContext> mContext;
};

}  // namespace aidl::android::hardware::audio::effect
