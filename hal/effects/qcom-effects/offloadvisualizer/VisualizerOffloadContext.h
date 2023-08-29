/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <android-base/thread_annotations.h>

#include "effect-impl/EffectContext.h"

using aidl::android::hardware::audio::effect::Parameter;
using aidl::android::hardware::audio::effect::CommandId;
using aidl::android::hardware::audio::effect::Capability;
using aidl::android::hardware::audio::effect::Descriptor;
using aidl::android::hardware::audio::effect::Range;
using aidl::android::hardware::audio::effect::Visualizer;

namespace aidl::qti::effects {

class VisualizerOffloadContext final : public EffectContext {
  public:
    static const uint32_t kMaxCaptureBufSize = 65536;
    static const uint32_t kMaxLatencyMs = 3000;  // 3 seconds of latency for audio pipeline

    VisualizerOffloadContext(int statusDepth, const Parameter::Common& common);
    ~VisualizerOffloadContext();

    RetCode initParams(const Parameter::Common& common);

    RetCode enable();
    RetCode disable();
    // keep all parameters and reset buffer.
    void reset();

    RetCode setCaptureSamples(int captureSize);
    int getCaptureSamples();
    RetCode setMeasurementMode(Visualizer::MeasurementMode mode);
    Visualizer::MeasurementMode getMeasurementMode();
    RetCode setScalingMode(Visualizer::ScalingMode mode);
    Visualizer::ScalingMode getScalingMode();
    RetCode setDownstreamLatency(int latency);
    int getDownstreamLatency();

    IEffect::Status process(float* in, float* out, int samples);
    // Gets the current measurements, measured by process() and consumed by getParameter()
    Visualizer::Measurement getMeasure();
    // Gets the latest PCM capture, data captured by process() and consumed by getParameter()
    std::vector<uint8_t> capture();

    struct BufferStats {
        bool mIsValid;
        uint16_t mPeakU16; // the positive peak of the absolute value of the samples in a buffer
        float mRmsSquared; // the average square of the samples in a buffer
    };

    enum State {
        UNINITIALIZED,
        INITIALIZED,
        ACTIVE,
    };

  private:
    // maximum time since last capture buffer update before resetting capture buffer. This means
    // that the framework has stopped playing audio and we must start returning silence
    static const uint32_t kMaxStallTimeMs = 1000;
    // discard measurements older than this number of ms
    static const uint32_t kDiscardMeasurementsTimeMs = 2000;
    // maximum number of buffers for which we keep track of the measurements
    // note: buffer index is stored in uint8_t
    static const uint32_t kMeasurementWindowMaxSizeInBuffers = 25;

    // serialize process() and parameter setting
    std::mutex mMutex;
    Parameter::Common mCommon GUARDED_BY(mMutex);
    State mState GUARDED_BY(mMutex) = State::UNINITIALIZED;
    uint32_t mCaptureIdx GUARDED_BY(mMutex) = 0;
    uint32_t mLastCaptureIdx GUARDED_BY(mMutex) = 0;
    Visualizer::ScalingMode mScalingMode GUARDED_BY(mMutex) = Visualizer::ScalingMode::NORMALIZED;
    struct timespec mBufferUpdateTime GUARDED_BY(mMutex);
    // capture buf with 8 bits PCM
    std::array<uint8_t, kMaxCaptureBufSize> mCaptureBuf GUARDED_BY(mMutex);
    uint32_t mDownstreamLatency GUARDED_BY(mMutex) = 0;
    uint32_t mCaptureSamples GUARDED_BY(mMutex) = kMaxCaptureBufSize;

    // to avoid recomputing it every time a buffer is processed
    uint8_t mChannelCount GUARDED_BY(mMutex) = 0;
    Visualizer::MeasurementMode mMeasurementMode GUARDED_BY(mMutex) =
            Visualizer::MeasurementMode::NONE;
    uint8_t mMeasurementWindowSizeInBuffers = kMeasurementWindowMaxSizeInBuffers;
    uint8_t mMeasurementBufferIdx GUARDED_BY(mMutex) = 0;
    std::array<BufferStats, kMeasurementWindowMaxSizeInBuffers> mPastMeasurements;
    void init_params();

    uint32_t getDeltaTimeMsFromUpdatedTime_l() REQUIRES(mMutex);
};
}  // namespace aidl::qti::effects
