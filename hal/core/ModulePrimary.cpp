/*
 * Copyright (C) 2023 The Android Open Source Project
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
 * ​​​​​Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <vector>

#define LOG_TAG "AHAL_QModulePri"
#include <Utils.h>
#include <android-base/logging.h>

#include <qti-audio-core/ModulePrimary.h>
#include <qti-audio-core/StreamInPrimary.h>
#include <qti-audio-core/StreamOutPrimary.h>
#include <qti-audio-core/StreamStub.h>
#include <qti-audio-core/Telephony.h>

using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::AudioPort;
using aidl::android::media::audio::common::AudioPortExt;
using aidl::android::media::audio::common::AudioPortConfig;
using aidl::android::media::audio::common::MicrophoneInfo;
using aidl::android::media::audio::common::Boolean;

using ::aidl::android::hardware::audio::common::getFrameSizeInBytes;
using ::aidl::android::hardware::audio::common::isBitPositionFlagSet;
using ::aidl::android::hardware::audio::common::isValidAudioMode;
using ::aidl::android::hardware::audio::common::SinkMetadata;
using ::aidl::android::hardware::audio::common::SourceMetadata;

using ::aidl::android::hardware::audio::core::AudioPatch;
using ::aidl::android::hardware::audio::core::AudioRoute;
using ::aidl::android::hardware::audio::core::IStreamIn;
using ::aidl::android::hardware::audio::core::IStreamOut;
using ::aidl::android::hardware::audio::core::ITelephony;
using ::aidl::android::hardware::audio::core::VendorParameter;

namespace qti::audio::core {

std::string ModulePrimary::toStringInternal() {
    std::ostringstream os;
    os << "--- ModulePrimary start ---" << std::endl;
    os << getConfig().toString() << std::endl;

    os << mPlatform.toString() << std::endl;
    os << "--- ModulePrimary end ---" << std::endl;
    return os.str();
}

void ModulePrimary::dumpInternal() {
    const std::string kDumpPath{"/data/vendor/audio/audio_hal_service.dump"};

    auto fd = ::open(kDumpPath.c_str(), O_CREAT | O_WRONLY | O_TRUNC,
                     S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd <= 0) {
        LOG(ERROR) << ": dump internal failed; fd:" << fd
                   << " unable to open file:" << kDumpPath;
        return;
    }
    auto dumpData = toStringInternal();
    auto b = ::write(fd, dumpData.c_str(), dumpData.size());
    if (b != static_cast<decltype(b)>(dumpData.size())) {
        LOG(ERROR) << __func__ << ": dump internal failed to write in "
                   << kDumpPath;
    }
    LOG(INFO) << "dump internal successful to " << kDumpPath;
    ::close(fd);
    return;
}

binder_status_t ModulePrimary::dump(int fd, const char** args, uint32_t numArgs) {
    if (fd <= 0) {
        LOG(ERROR) << ": fd:" << fd << " dump error";
        return -EINVAL;
    }
    auto dumpData = toStringInternal();
    auto b = ::write(fd, dumpData.c_str(), dumpData.size());
    if (b != static_cast<decltype(b)>(dumpData.size())) {
        LOG(ERROR) << __func__ << " write error in dump";
        return -EIO;
    }
    LOG(INFO) << __func__ <<" :success";
    return 0;
}

ndk::ScopedAStatus ModulePrimary::getTelephony(
    std::shared_ptr<ITelephony>* _aidl_return) {
    if (!mTelephony) {
        mTelephony = ndk::SharedRefBase::make<Telephony>();
    }
    *_aidl_return = mTelephony.getInstance();
    LOG(DEBUG) << __func__
               << ": returning instance of ITelephony: " << _aidl_return->get()->asBinder().get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::createInputStream(StreamContext&& context,
                                                    const SinkMetadata& sinkMetadata,
                                                    const std::vector<MicrophoneInfo>& microphones,
                                                    std::shared_ptr<StreamIn>* result) {
    return createStreamInstance<StreamInPrimary>(result, std::move(context), sinkMetadata,
                                              microphones);
}

ndk::ScopedAStatus ModulePrimary::createOutputStream(
        StreamContext&& context, const SourceMetadata& sourceMetadata,
        const std::optional<AudioOffloadInfo>& offloadInfo, std::shared_ptr<StreamOut>* result) {
    return createStreamInstance<StreamOutPrimary>(result, std::move(context), sourceMetadata,
                                               offloadInfo);
}

std::vector<::aidl::android::media::audio::common::AudioProfile>
ModulePrimary::getDynamicProfiles(
    const ::aidl::android::media::audio::common::AudioPort& audioPort) {
    if (mPlatform.isUsbDevice(
            audioPort.ext.get<AudioPortExt::Tag::device>().device)) {
        /* as of now, we do dynamic fetching for usb devices*/
        auto dynamicProfiles = mPlatform.getDynamicProfiles(audioPort);
        return dynamicProfiles;
    }
    return {};
}

void ModulePrimary::onNewPatchCreation(const std::vector<AudioPortConfig*>& sources,
                                const std::vector<AudioPortConfig*>& sinks,
                                AudioPatch& newPatch) {
    auto numFrames = mPlatform.getMinimumStreamSizeFrames(
        sources, sinks);
    numFrames != 0 ? (void)(newPatch.minimumStreamBufferSizeFrames = numFrames)
                   : (void)0;
}

void ModulePrimary::onExternalDeviceConnectionChanged(
    const ::aidl::android::media::audio::common::AudioPort& audioPort,
    bool connected) {
    if (!mPlatform.handleDeviceConnectionChange(audioPort, connected)) {
        LOG(WARNING) << __func__
                     << " failed to handle device connection change:"
                     << (connected ? " connect" : "disconnect") << " for "
                     << audioPort.toString();
    }
}

namespace {

template <typename W>
bool extractParameter(const VendorParameter& p, decltype(W::value)* v) {
    std::optional<W> value;
    binder_status_t result = p.ext.getParcelable(&value);
    if (result == STATUS_OK && value.has_value()) {
        *v = value.value().value;
        return true;
    }
    LOG(ERROR) << __func__ << ": failed to read the value of the parameter \"" << p.id
               << "\": " << result;
    return false;
}

}

ndk::ScopedAStatus ModulePrimary::setVendorParameters(
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    bool in_async) {
    LOG(DEBUG) << __func__ << ": parameter count " << in_parameters.size()
               << ", async: " << in_async;
    bool allParametersKnown = true;
    for (const auto& p : in_parameters) {
        if (p.id == VendorDebug::kForceTransientBurstName) {
            if (!extractParameter<Boolean>(p, &mVendorDebug.forceTransientBurst)) {
                return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
            }
        } else if (p.id == VendorDebug::kForceSynchronousDrainName) {
            if (!extractParameter<Boolean>(p, &mVendorDebug.forceSynchronousDrain)) {
                return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
            }
        } else {
            allParametersKnown = false;
            LOG(ERROR) << __func__ << ": unrecognized parameter \"" << p.id << "\"";
        }
    }
    if (allParametersKnown) return ndk::ScopedAStatus::ok();
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus ModulePrimary::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    LOG(DEBUG) << __func__ << ": id count: " << in_ids.size();
    bool allParametersKnown = true;
    for (const auto& id : in_ids) {
        if (id == VendorDebug::kForceTransientBurstName) {
            VendorParameter forceTransientBurst{.id = id};
            forceTransientBurst.ext.setParcelable(Boolean{mVendorDebug.forceTransientBurst});
            _aidl_return->push_back(std::move(forceTransientBurst));
        } else if (id == VendorDebug::kForceSynchronousDrainName) {
            VendorParameter forceSynchronousDrain{.id = id};
            forceSynchronousDrain.ext.setParcelable(Boolean{mVendorDebug.forceSynchronousDrain});
            _aidl_return->push_back(std::move(forceSynchronousDrain));
        } else {
            allParametersKnown = false;
            LOG(ERROR) << __func__ << ": unrecognized parameter \"" << id << "\"";
        }
    }
    if (allParametersKnown) return ndk::ScopedAStatus::ok();
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

}  // namespace qti::audio::core
