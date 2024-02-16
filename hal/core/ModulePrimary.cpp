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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <vector>

#define LOG_TAG "AHAL_ModulePrimary_QTI"
#include <Utils.h>
#include <android-base/logging.h>
#include <cutils/str_parms.h>

#include <aidl/qti/audio/core/VString.h>
#include <qti-audio-core/Bluetooth.h>
#include <qti-audio-core/ModulePrimary.h>
#include <qti-audio-core/Parameters.h>
#include <qti-audio-core/PlatformUtils.h>
#include <qti-audio-core/StreamInPrimary.h>
#include <qti-audio-core/StreamOutPrimary.h>
#include <qti-audio-core/StreamStub.h>
#include <qti-audio-core/Telephony.h>
#include <qti-audio-core/Utils.h>

using aidl::android::hardware::audio::common::SinkMetadata;
using aidl::android::hardware::audio::common::SourceMetadata;
using aidl::android::media::audio::common::AudioOffloadInfo;
using aidl::android::media::audio::common::AudioPort;
using aidl::android::media::audio::common::AudioPortExt;
using aidl::android::media::audio::common::AudioDevice;
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
using ::aidl::qti::audio::core::VString;
using ::aidl::android::hardware::audio::core::IBluetooth;
using ::aidl::android::hardware::audio::core::IBluetoothA2dp;
using ::aidl::android::hardware::audio::core::IBluetoothLe;

namespace qti::audio::core {

std::vector<std::weak_ptr<::qti::audio::core::StreamOut>> ModulePrimary::mStreamsOut;
std::vector<std::weak_ptr<::qti::audio::core::StreamIn>> ModulePrimary::mStreamsIn;

std::mutex ModulePrimary::outListMutex;
std::mutex ModulePrimary::inListMutex;

std::string ModulePrimary::toStringInternal() {
    std::ostringstream os;
    os << "--- ModulePrimary start ---" << std::endl;
    os << getConfig().toString() << std::endl;

    os << std::endl << " --- mPatches ---" << std::endl;
    std::for_each(mPatches.cbegin(), mPatches.cend(), [&](const auto& pair) {
        os << "PortConfigId/PortId:" << pair.first << " Patch Id:" << pair.second << std::endl;
    });
    os << std::endl << " --- mPatches end ---" << std::endl << std::endl;

    os << mStreams.toString();

    os << mPlatform.toString() << std::endl;
    os << "--- ModulePrimary end ---" << std::endl;
    return os.str();
}

void ModulePrimary::dumpInternal(const std::string& identifier) {
    const auto realTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count();
    const std::string kDumpPath{std::string("/data/vendor/audio/audio_hal_service_")
                                        .append(identifier)
                                        .append("_")
                                        .append(std::to_string(realTimeMs))
                                        .append(".dump")};

    const auto fd = ::open(kDumpPath.c_str(), O_CREAT | O_WRONLY | O_TRUNC,
                           S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd <= 0) {
        LOG(ERROR) << __func__ << ": dump internal failed; fd:" << fd
                   << " unable to open file:" << kDumpPath;
        return;
    }
    const auto dumpData = toStringInternal();
    auto b = ::write(fd, dumpData.c_str(), dumpData.size());
    if (b != static_cast<decltype(b)>(dumpData.size())) {
        LOG(ERROR) << __func__ << ": dump internal failed to write in " << kDumpPath;
    }
    LOG(DEBUG) << __func__ << ": at: " << kDumpPath;
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
    LOG(INFO) << __func__ << " :success";
    return 0;
}

ModulePrimary::ModulePrimary() : Module(Type::DEFAULT) {
    mOffloadSpeedSupported = mPlatform.platformSupportsOffloadSpeed();
}

ndk::ScopedAStatus ModulePrimary::getMicMute(bool* _aidl_return) {
    *_aidl_return = mMicMute;
    LOG(VERBOSE) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::setMicMute(bool in_mute) {
    LOG(VERBOSE) << __func__ << ": " << in_mute;
    mMicMute = in_mute;

    mTelephony->setMicMute(mMicMute);

    int ret = mAudExt.mHfpExtension->audio_extn_hfp_set_mic_mute(in_mute);

    for (const auto& inputMixPortConfigId :
         getActiveInputMixPortConfigIds(getConfig().portConfigs)) {
        mStreams.setStreamMicMute(inputMixPortConfigId, mMicMute);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::updateScreenState(bool in_isTurnedOn) {
    LOG(VERBOSE) << __func__ << ": " << in_isTurnedOn;
    mPlatform.updateScreenState(in_isTurnedOn);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::updateScreenRotation(ScreenRotation in_rotation) {
    LOG(VERBOSE) << __func__ << ": " << toString(in_rotation);
    mPlatform.updateScreenRotation(in_rotation);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::getBluetooth(std::shared_ptr<IBluetooth>* _aidl_return) {
    if (!mBluetooth) {
        mBluetooth = ndk::SharedRefBase::make<::qti::audio::core::Bluetooth>();
    }
    *_aidl_return = mBluetooth.getInstance();
    LOG(DEBUG) << __func__
               << ": returning instance of IBluetooth: " << _aidl_return->get()->asBinder().get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::getBluetoothA2dp(std::shared_ptr<IBluetoothA2dp>* _aidl_return) {
    if (!mBluetoothA2dp) {
        mBluetoothA2dp = ndk::SharedRefBase::make<::qti::audio::core::BluetoothA2dp>();
    }
    *_aidl_return = mBluetoothA2dp.getInstance();
    LOG(DEBUG) << __func__ << ": returning instance of IBluetoothA2dp: "
               << _aidl_return->get()->asBinder().get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::getBluetoothLe(std::shared_ptr<IBluetoothLe>* _aidl_return) {
    if (!mBluetoothLe) {
        mBluetoothLe = ndk::SharedRefBase::make<::qti::audio::core::BluetoothLe>();
    }
    *_aidl_return = mBluetoothLe.getInstance();
    LOG(DEBUG) << __func__
               << ": returning instance of IBluetoothLe: " << _aidl_return->get()->asBinder().get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::getTelephony(std::shared_ptr<ITelephony>* _aidl_return) {
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
    createStreamInstance<StreamInPrimary>(result, std::move(context), sinkMetadata, microphones);
    ModulePrimary::inListMutex.lock();
    ModulePrimary::updateStreamInList(*result);
    if (mTelephony) mTelephony->mStreamInPrimary = *result;
    ModulePrimary::inListMutex.unlock();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus ModulePrimary::createOutputStream(
        StreamContext&& context, const SourceMetadata& sourceMetadata,
        const std::optional<AudioOffloadInfo>& offloadInfo, std::shared_ptr<StreamOut>* result) {
    if (mPlatform.isSoundCardDown() &&
        (hasOutputDirectFlag(context.getMixPortConfig().flags.value()) ||
         hasOutputCompressOffloadFlag(context.getMixPortConfig().flags.value()))) {
        LOG(ERROR) << __func__ << ": avoid direct or compress streams as sound card is down";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    createStreamInstance<StreamOutPrimary>(result, std::move(context), sourceMetadata, offloadInfo);
    ModulePrimary::outListMutex.lock();
    ModulePrimary::updateStreamOutList(*result);
    // save primary out stream weak ptr, as some other modules need it.
    if (mTelephony) mTelephony->mStreamOutPrimary = *result;
    ModulePrimary::outListMutex.unlock();
    return ndk::ScopedAStatus::ok();
}

std::vector<::aidl::android::media::audio::common::AudioProfile> ModulePrimary::getDynamicProfiles(
        const ::aidl::android::media::audio::common::AudioPort& audioPort) {
    if (mPlatform.isUsbDevice(audioPort.ext.get<AudioPortExt::Tag::device>().device)) {
        /* as of now, we do dynamic fetching for usb devices*/
        auto dynamicProfiles = mPlatform.getDynamicProfiles(audioPort);
        return dynamicProfiles;
    }
    return {};
}

void ModulePrimary::onNewPatchCreation(const std::vector<AudioPortConfig*>& sources,
                                       const std::vector<AudioPortConfig*>& sinks,
                                       AudioPatch& newPatch) {
    if (!isMixPortConfig(*(sources.at(0))) && !isMixPortConfig(*(sinks.at(0)))) {
        LOG(VERBOSE) << __func__ << ": no mix ports detected";
        return;
    }
    auto numFrames = mPlatform.getMinimumStreamSizeFrames(sources, sinks);
    if (numFrames < kMinimumStreamBufferSizeFrames) {
        LOG(DEBUG) << __func__ << ": got invalid stream size frames " << numFrames
                   << " adjusting to " << kMinimumStreamBufferSizeFrames;
        numFrames = kMinimumStreamBufferSizeFrames;
    }
    newPatch.minimumStreamBufferSizeFrames = numFrames;
}

void ModulePrimary::setAudioPatchTelephony(const std::vector<AudioPortConfig*>& sources,
                                           const std::vector<AudioPortConfig*>& sinks,
                                           const AudioPatch& patch) {
    std::string patchDetails = getPatchDetails(patch);
    if (!mTelephony) {
        LOG(ERROR) << __func__ << ": Telephony not created " << patchDetails << patch.toString();
        return;
    }

    if (!isDevicePortConfig(*(sources.at(0))) || !isDevicePortConfig(*(sinks.at(0)))) {
        return;
    }
    bool updateRx = isTelephonyRXDevice(sources.at(0)->ext.get<AudioPortExt::Tag::device>().device);
    bool updateTx = isTelephonyTXDevice(sinks.at(0)->ext.get<AudioPortExt::Tag::device>().device);

    if (!updateRx && !updateTx) {
        LOG(ERROR) << __func__ << ": neither RX nor TX update " << patchDetails << patch.toString();
        return;
    }

    const auto& portConfigsForDeviceChange = updateRx ? (sinks) : (sources);

    std::vector<AudioDevice> devices;
    for (const auto portConfig : portConfigsForDeviceChange) {
        devices.push_back(portConfig->ext.get<AudioPortExt::Tag::device>().device);
    }

    mTelephony->setDevices(devices, updateRx);
    LOG(DEBUG) << __func__ << ": device patch : " << patchDetails << patch.toString();
}

void ModulePrimary::onExternalDeviceConnectionChanged(
        const ::aidl::android::media::audio::common::AudioPort& audioPort, bool connected) {
    if (!mPlatform.handleDeviceConnectionChange(audioPort, connected)) {
        LOG(WARNING) << __func__ << " failed to handle device connection change:"
                     << (connected ? " connect" : "disconnect") << " for " << audioPort.toString();
    }
    if (connected) {
        mTelephony->updateDevicesFromPrimaryPlayback();
    }
}

ndk::ScopedAStatus ModulePrimary::getSupportedPlaybackRateFactors(
        SupportedPlaybackRateFactors* _aidl_return) {
    LOG(DEBUG) << __func__ << " speed supported " << mOffloadSpeedSupported;
    if (mOffloadSpeedSupported) {
        _aidl_return->minSpeed = 0.1f;
        _aidl_return->maxSpeed = 2.0f;
        _aidl_return->minPitch = 1.0f;
        _aidl_return->maxPitch = 1.0f;
        return ndk::ScopedAStatus::ok();
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}
// start of module parameters handling

ndk::ScopedAStatus ModulePrimary::setVendorParameters(
        const std::vector<::aidl::android::hardware::audio::core::VendorParameter>& in_parameters,
        bool in_async) {
    LOG(VERBOSE) << __func__ << ": parameter count " << in_parameters.size()
               << ", async: " << in_async;
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
            struct str_parms* parms = NULL;
            std::string kvpairs = getkvPairsForVendorParameter(in_parameters);
            if (!kvpairs.empty()) {
                parms = str_parms_create_str(kvpairs.c_str());
                mAudExt.audio_extn_set_parameters(parms);
            }

            mPlatform.setVendorParameters(in_parameters, in_async);
        }
    }
    processSetVendorParameters(in_parameters);
    return ndk::ScopedAStatus::ok();
}

bool ModulePrimary::processSetVendorParameters(const std::vector<VendorParameter>& parameters) {
    FeatureToVendorParametersMap pendingActions{};
    for (const auto& p : parameters) {
        const auto searchId = mSetParameterToFeatureMap.find(p.id);
        if (searchId == mSetParameterToFeatureMap.cend()) {
            LOG(VERBOSE) << __func__ << ": not configured " << p.id;
            continue;
        }

        auto itr = pendingActions.find(searchId->second);
        if (itr == pendingActions.cend()) {
            pendingActions[searchId->second] = std::vector<VendorParameter>({p});
            continue;
        }
        itr->second.push_back(p);
    }

    for (const auto & [ key, value ] : pendingActions) {
        const auto search = mFeatureToSetHandlerMap.find(key);
        if (search == mFeatureToSetHandlerMap.cend()) {
            LOG(VERBOSE) << __func__
                         << ": no handler set on Feature:" << static_cast<int>(search->first);
            continue;
        }
        auto handler = std::bind(search->second, this, value);
        handler(); // a dynamic dispatch to a SetHandler
    }
    return true;
}

void ModulePrimary::onSetGenericParameters(const std::vector<VendorParameter>& params) {
    for (const auto& param : params) {
        std::string paramValue{};
        if (!extractParameter<VString>(param, &paramValue)) {
            LOG(ERROR) << ": extraction failed for " << param.id;
            continue;
        }
        if (Parameters::kInCallMusic == param.id) {
            const auto isOn = getBoolFromString(paramValue);
            mPlatform.setInCallMusicState(isOn);
            LOG(INFO) << __func__ << ": ICMD playback:" << isOn;
        } else if (Parameters::kUHQA == param.id) {
            const bool enable = paramValue == "on" ? true : false;
            mPlatform.updateUHQA(enable);
        }
    }
}

void ModulePrimary::onSetHDRParameters(const std::vector<VendorParameter>& params) {
    for (const auto& param : params) {
        LOG(VERBOSE) << __func__ << param.id;
    }
    // LOG(VERBOSE) << __func__;
    return;
};

void ModulePrimary::onSetTelephonyParameters(const std::vector<VendorParameter>& parameters) {
    if (!mTelephony) {
        LOG(ERROR) << __func__ << ": Telephony not created";
        return;
    }

    Telephony::SetUpdates setUpdates{};
    bool isSetUpdate = false;

    bool isDeviceMuted = false;
    std::string muteDirection{""};
    bool isDeviceMuteUpdate = false;

    for (const auto& p : parameters) {
        std::string paramValue{};
        if (!extractParameter<VString>(p, &paramValue)) {
            LOG(ERROR) << ": extraction failed for " << p.id;
            continue;
        }
        if (Parameters::kVoiceCallState == p.id) {
            setUpdates.mCallState =
                    static_cast<Telephony::CallState>(getInt64FromString(paramValue));
            isSetUpdate = true;
        } else if (Parameters::kVoiceVSID == p.id) {
            setUpdates.mVSID = static_cast<Telephony::VSID>(getInt64FromString(paramValue));
            isSetUpdate = true;
        } else if (Parameters::kVoiceCallType == p.id) {
            setUpdates.mCallType = std::move(paramValue);
            isSetUpdate = true;
        } else if (Parameters::kVoiceCRSCall == p.id) {
            setUpdates.mIsCrsCall = paramValue == "true" ? true : false;
        } else if (Parameters::kVoiceCRSVolume == p.id) {
            mTelephony->setCRSVolumeFromIndex(getInt64FromString(paramValue));
        } else if (Parameters::kVolumeBoost == p.id) {
            const bool enable = paramValue == "on" ? true : false;
            mTelephony->updateVolumeBoost(enable);
        } else if (Parameters::kVoiceSlowTalk == p.id) {
            const bool enable = paramValue == "true" ? true : false;
            mTelephony->updateSlowTalk(enable);
        } else if (Parameters::kVoiceHDVoice == p.id) {
            const bool enable = paramValue == "true" ? true : false;
            mTelephony->updateHDVoice(enable);
        } else if (Parameters::kVoiceDeviceMute == p.id) {
            isDeviceMuted = paramValue == "true" ? true : false;
            isDeviceMuteUpdate = true;
        } else if (Parameters::kVoiceDirection == p.id) {
            muteDirection = paramValue;
        }
    }

    if (isSetUpdate) {
        mTelephony->reconfigure(setUpdates);
    }
    if (isDeviceMuteUpdate) {
        mTelephony->updateDeviceMute(isDeviceMuted, muteDirection);
    }

    return;
}

void ModulePrimary::onSetWFDParameters(const std::vector<VendorParameter>& parameters) {
    for (const auto& p : parameters) {
        std::string paramValue{};
        if (!extractParameter<VString>(p, &paramValue)) {
            LOG(ERROR) << ": extraction failed for " << p.id;
            continue;
        }
        if (Parameters::kWfdChannelMap == p.id) {
            auto numProxyChannels = static_cast<uint32_t>(getInt64FromString(paramValue));
            mPlatform.setWFDProxyChannels(numProxyChannels);
        }
    }
    return;
}

void ModulePrimary::onSetFTMParameters(const std::vector<VendorParameter>& parameters) {
    auto itrForCfgWaitTime =
            std::find_if(parameters.cbegin(), parameters.cend(),
                         [](const auto& p) { return p.id == Parameters::kFbspCfgWaitTime; });
    auto itrForFTMWaitTime =
            std::find_if(parameters.cbegin(), parameters.cend(),
                         [](const auto& p) { return p.id == Parameters::kFbspFTMWaitTime; });
    auto itrForValiWaitTime =
            std::find_if(parameters.cbegin(), parameters.cend(),
                         [](const auto& p) { return p.id == Parameters::kFbspValiWaitTime; });
    auto itrForValiValiTime =
            std::find_if(parameters.cbegin(), parameters.cend(),
                         [](const auto& p) { return p.id == Parameters::kFbspValiValiTime; });
    auto itrForTriggerSpeakerCall =
            std::find_if(parameters.cbegin(), parameters.cend(),
                         [](const auto& p) { return p.id == Parameters::kTriggerSpeakerCall; });

    if (itrForCfgWaitTime != parameters.cend() && itrForFTMWaitTime != parameters.cend()) {
        std::string heatTime{}, runTime{};
        if ((!extractParameter<VString>(*itrForCfgWaitTime, &heatTime)) ||
            (!extractParameter<VString>(*itrForFTMWaitTime, &runTime))) {
            LOG(ERROR) << __func__ << ": extraction failed!!!";
            return;
        }
        mPlatform.setFTMSpeakerProtectionMode(static_cast<uint32_t>(getInt64FromString(heatTime)),
                                              static_cast<uint32_t>(getInt64FromString(runTime)),
                                              true, false, false);
    } else if (itrForValiWaitTime != parameters.cend() && itrForValiValiTime != parameters.cend()) {
        std::string heatTime{}, runTime{};
        if ((!extractParameter<VString>(*itrForValiWaitTime, &heatTime)) ||
            (!extractParameter<VString>(*itrForValiValiTime, &runTime))) {
            LOG(ERROR) << __func__ << ": extraction failed!!!";
            return;
        }
        mPlatform.setFTMSpeakerProtectionMode(static_cast<uint32_t>(getInt64FromString(heatTime)),
                                              static_cast<uint32_t>(getInt64FromString(runTime)),
                                              false, true, false);
    } else if (itrForTriggerSpeakerCall != parameters.cend()) {
        mPlatform.setFTMSpeakerProtectionMode(0, 0, false, false, true);
    }

    return;
}

void ModulePrimary::onSetHapticsParameters(const std::vector<VendorParameter>& parameters) {
    for (const auto& param : parameters) {
        std::string paramValue{};
        if (!extractParameter<VString>(param, &paramValue)) {
            LOG(ERROR) << ": extraction failed for " << param.id;
            continue;
        }
        if (Parameters::kHapticsVolume == param.id) {
            const float hapticsVolume = getFloatFromString(paramValue);
            mPlatform.setHapticsVolume(hapticsVolume);
        } else if (Parameters::kHapticsIntensity == param.id) {
            const int hapticsIntensity = getInt64FromString(paramValue);
            mPlatform.setHapticsIntensity(hapticsIntensity);
        }
    }
    return;
}

// static
ModulePrimary::SetParameterToFeatureMap ModulePrimary::fillSetParameterToFeatureMap() {
    SetParameterToFeatureMap map{{Parameters::kHdrRecord, Feature::HDR},
                                 {Parameters::kWnr, Feature::HDR},
                                 {Parameters::kAns, Feature::HDR},
                                 {Parameters::kOrientation, Feature::HDR},
                                 {Parameters::kInverted, Feature::HDR},
                                 {Parameters::kHdrChannelCount, Feature::HDR},
                                 {Parameters::kHdrSamplingRate, Feature::HDR},
                                 {Parameters::kVoiceCallState, Feature::TELEPHONY},
                                 {Parameters::kVoiceCallType, Feature::TELEPHONY},
                                 {Parameters::kVoiceVSID, Feature::TELEPHONY},
                                 {Parameters::kVoiceCRSCall, Feature::TELEPHONY},
                                 {Parameters::kVoiceCRSVolume, Feature::TELEPHONY},
                                 {Parameters::kVolumeBoost, Feature::TELEPHONY},
                                 {Parameters::kVoiceSlowTalk, Feature::TELEPHONY},
                                 {Parameters::kVoiceHDVoice, Feature::TELEPHONY},
                                 {Parameters::kVoiceDeviceMute, Feature::TELEPHONY},
                                 {Parameters::kVoiceDirection, Feature::TELEPHONY},
                                 {Parameters::kInCallMusic, Feature::GENERIC},
                                 {Parameters::kUHQA, Feature::GENERIC},
                                 {Parameters::kFbspCfgWaitTime, Feature::FTM},
                                 {Parameters::kFbspFTMWaitTime, Feature::FTM},
                                 {Parameters::kFbspValiWaitTime, Feature::FTM},
                                 {Parameters::kFbspValiValiTime, Feature::FTM},
                                 {Parameters::kTriggerSpeakerCall, Feature::FTM},
                                 {Parameters::kWfdChannelMap, Feature::WFD},
                                 {Parameters::kHapticsVolume, Feature::HAPTICS},
                                 {Parameters::kHapticsIntensity, Feature::HAPTICS}};
    return map;
}

// static
ModulePrimary::FeatureToSetHandlerMap ModulePrimary::fillFeatureToSetHandlerMap() {
    FeatureToSetHandlerMap map{
            {Feature::GENERIC, &ModulePrimary::onSetGenericParameters},
            {Feature::HDR, &ModulePrimary::onSetHDRParameters},
            {Feature::TELEPHONY, &ModulePrimary::onSetTelephonyParameters},
            {Feature::WFD, &ModulePrimary::onSetWFDParameters},
            {Feature::FTM, &ModulePrimary::onSetFTMParameters},
            {Feature::HAPTICS, &ModulePrimary::onSetHapticsParameters},
    };
    return map;
}

ndk::ScopedAStatus ModulePrimary::getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>* _aidl_return) {
    LOG(DEBUG) << __func__ << ": id count: " << in_ids.size();
    for (const auto& id : in_ids) {
        if (id == VendorDebug::kForceTransientBurstName) {
            VendorParameter forceTransientBurst{.id = id};
            forceTransientBurst.ext.setParcelable(Boolean{mVendorDebug.forceTransientBurst});
            _aidl_return->push_back(std::move(forceTransientBurst));
        } else if (id == VendorDebug::kForceSynchronousDrainName) {
            VendorParameter forceSynchronousDrain{.id = id};
            forceSynchronousDrain.ext.setParcelable(Boolean{mVendorDebug.forceSynchronousDrain});
            _aidl_return->push_back(std::move(forceSynchronousDrain));
        }
    }

    auto results = processGetVendorParameters(in_ids);
    std::move(results.begin(), results.end(), std::back_inserter(*_aidl_return));

    return ndk::ScopedAStatus::ok();
}

std::vector<VendorParameter> ModulePrimary::processGetVendorParameters(
        const std::vector<std::string>& ids) {
    FeatureToStringMap pendingActions{};
    // only group of features are mapped to Feature, rest are kept as generic.
    // If the key is found in feature map, use the feature otherwise call GENERIC feature.
    for (const auto& id : ids) {
        auto search = mGetParameterToFeatureMap.find(id);
        Feature mappedFeature = Feature::GENERIC;
        if (search != mGetParameterToFeatureMap.cend()) {
            mappedFeature = search->second;
        }
        auto itr = pendingActions.find(mappedFeature);
        if (itr == pendingActions.cend()) {
            pendingActions[mappedFeature] = std::vector<std::string>({id});
            continue;
        }
        itr->second.push_back(id);
    }

    std::vector<VendorParameter> result{};
    for (const auto & [ key, value ] : pendingActions) {
        const auto search = mFeatureToGetHandlerMap.find(key);
        if (search == mFeatureToGetHandlerMap.cend()) {
            LOG(ERROR) << __func__
                       << ": no handler set on Feature:" << static_cast<int>(search->first);
            continue;
        }
        auto handler = std::bind(search->second, this, value);
        auto keyResult = handler(); // a dynamic dispatch to GetHandler
        result.insert(result.end(), keyResult.begin(), keyResult.end());
    }
    return result;
}

std::vector<VendorParameter> ModulePrimary::onGetAudioExtnParams(
        const std::vector<std::string>& ids) {
    std::vector<VendorParameter> results{};
    for (const auto& id : ids) {
        if (id == Parameters::kFMStatus) {
            bool fm_status = mAudExt.mFmExtension->audio_extn_fm_get_status();
            VendorParameter param;
            param.id = id;
            VString parcel;
            parcel.value = fm_status ? "true" : "false";
            setParameter(parcel, param);
            results.push_back(param);
        } else if (id == Parameters::kCanOpenProxy) {
            VendorParameter param;
            param.id = id;
            VString parcel;
            parcel.value = "1";
            setParameter(parcel, param);
            results.push_back(param);
        }
    }
    return results;
}

std::vector<VendorParameter> ModulePrimary::onGetGenericParams(
        const std::vector<std::string>& ids) {
    std::vector<VendorParameter> results{};
    for (const auto& id : ids) {
        if (id == Parameters::kOffloadPlaySpeedSupported) {
            LOG(DEBUG) << __func__ << " " << id << " supported " << mOffloadSpeedSupported;
            std::string value = (mOffloadSpeedSupported ? "true" : "false");
            auto param = makeVendorParameter(id, value);
            results.push_back(param);
        }
    }
    return results;
}

std::vector<VendorParameter> ModulePrimary::onGetBluetoothParams(
        const std::vector<std::string>& ids) {
    if (!mBluetoothA2dp) {
        LOG(ERROR) << __func__ << ": Bluetooth not created";
        return {};
    }
    std::vector<VendorParameter> results{};
    for (const auto& id : ids) {
        if (id == Parameters::kA2dpSuspended) {
            VendorParameter param;
            bool a2dpSuspended = false;
            param.id = id;
            VString parcel;
            mBluetoothA2dp->isEnabled(&a2dpSuspended);
            parcel.value = a2dpSuspended ? "1" : "0";
            setParameter(parcel, param);
            results.push_back(param);
        }
    }
    return results;
}

std::vector<VendorParameter> ModulePrimary::onGetTelephonyParameters(
        const std::vector<std::string>& ids) {
    if (!mTelephony) {
        LOG(ERROR) << __func__ << ": Telephony not created";
        return {};
    }
    std::vector<VendorParameter> results{};
    for (const auto& id : ids) {
        if (id == Parameters::kVoiceIsCRsSupported) {
            VendorParameter param;
            param.id = id;
            VString parcel;
            parcel.value = mTelephony->isCrsCallSupported() ? "1" : "0";
            setParameter(parcel, param);
            results.push_back(param);
        }
    }
    return results;
}

std::vector<VendorParameter> ModulePrimary::onGetWFDParameters(
        const std::vector<std::string>& ids) {
    std::vector<VendorParameter> results{};
    for (const auto& id : ids) {
        if (id == Parameters::kCanOpenProxy) {
            VendorParameter param;
            param.id = id;
            VString parcel;
            parcel.value = "1"; // This "1" indicates WFD client can try AHAL Capture.
            setParameter(parcel, param);
            results.push_back(param);
        } else {
            LOG(ERROR) << __func__ << ": unknown parameter in WFD feature. id:" << id;
        }
    }
    return results;
}

std::vector<VendorParameter> ModulePrimary::onGetFTMParameters(
        const std::vector<std::string>& ids) {
    std::vector<VendorParameter> results{};
    for (const auto& id : ids) {
        VendorParameter param;
        VString parcel;
        if (id == Parameters::kFTMParam) {
            param.id = id;
            const auto& ftmResult = mPlatform.getFTMResult();
            if (ftmResult) {
                parcel.value = ftmResult.value();
            } else {
                parcel.value = "";
            }
            setParameter(parcel, param);
            results.push_back(param);
        } else if (id == Parameters::kFTMSPKRParam) {
            param.id = id;
            const auto& calResult = mPlatform.getSpeakerCalibrationResult();
            if (calResult) {
                parcel.value = calResult.value();
            } else {
                parcel.value = "false";
            }
            setParameter(parcel, param);
            results.push_back(param);
        } else {
            LOG(ERROR) << __func__ << ": unknown parameter in FTM feature. id:" << id;
        }
    }
    return results;
}

// static
ModulePrimary::GetParameterToFeatureMap ModulePrimary::fillGetParameterToFeatureMap() {
    GetParameterToFeatureMap map{{Parameters::kVoiceIsCRsSupported, Feature::TELEPHONY},
                                 {Parameters::kA2dpSuspended, Feature::BLUETOOTH},
                                 {Parameters::kCanOpenProxy, Feature::WFD},
                                 {Parameters::kFTMParam, Feature::FTM},
                                 {Parameters::kFTMSPKRParam, Feature::FTM},
                                 {Parameters::kFMStatus, Feature::AUDIOEXTENSION}};
    return map;
}

// static
ModulePrimary::FeatureToGetHandlerMap ModulePrimary::fillFeatureToGetHandlerMap() {
    FeatureToGetHandlerMap map{{Feature::TELEPHONY, &ModulePrimary::onGetTelephonyParameters},
                               {Feature::BLUETOOTH, &ModulePrimary::onGetBluetoothParams},
                               {Feature::WFD, &ModulePrimary::onGetWFDParameters},
                               {Feature::FTM, &ModulePrimary::onGetFTMParameters},
                               {Feature::AUDIOEXTENSION, &ModulePrimary::onGetAudioExtnParams},
                               {Feature::GENERIC, &ModulePrimary::onGetGenericParams}};
    return map;
}

// end of module parameters handling

} // namespace qti::audio::core
