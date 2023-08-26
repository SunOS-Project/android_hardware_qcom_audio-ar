/*
 * Copyright (C) 2022 The Android Open Source Project
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

#define LOG_TAG "AHAL_QTele"
#include <Utils.h>
#include <android-base/logging.h>
#include <android/binder_to_string.h>
#include <qti-audio-core/Telephony.h>
#include <qti-audio-core/Utils.h>

using aidl::android::hardware::audio::common::isValidAudioMode;
using aidl::android::media::audio::common::AudioDevice;
using aidl::android::media::audio::common::AudioDeviceAddress;
using aidl::android::media::audio::common::AudioDeviceDescription;
using aidl::android::media::audio::common::AudioDeviceType;
using aidl::android::media::audio::common::AudioMode;
using aidl::android::media::audio::common::Boolean;
using aidl::android::media::audio::common::Float;

namespace qti::audio::core {

const AudioDevice Telephony::kDefaultRxDevice =
    AudioDevice{.type.type = AudioDeviceType::OUT_SPEAKER_EARPIECE};

Telephony::Telephony() {
    mTelecomConfig.voiceVolume = Float{TelecomConfig::VOICE_VOLUME_MAX};
    mTelecomConfig.ttyMode = TelecomConfig::TtyMode::OFF;
    mTelecomConfig.isHacEnabled = Boolean{false};
    // Todo check on default RX device
    mRxDevices = {kDefaultRxDevice};
    mTxDevices = getMatchingTxDevices(mRxDevices);
}

Telephony::~Telephony() { stopCall(); }

ndk::ScopedAStatus Telephony::getSupportedAudioModes(
    std::vector<AudioMode>* _aidl_return) {
    std::scoped_lock lock{mLock};

    *_aidl_return = mSupportedAudioModes;
    LOG(DEBUG) << __func__ << ": returning "
               << ::android::internal::ToString(*_aidl_return);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Telephony::switchAudioMode(AudioMode newAudioMode) {
    std::scoped_lock lock{mLock};

    if (std::find(mSupportedAudioModes.begin(), mSupportedAudioModes.end(),
                  newAudioMode) == mSupportedAudioModes.end()) {
        LOG(ERROR) << __func__ << ": unsupported mode "
                   << toString(newAudioMode);
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    if (mAudioMode == newAudioMode) {
        LOG(VERBOSE) << __func__ << ": no change" << toString(newAudioMode);
        return ndk::ScopedAStatus::ok();
    }

    if (newAudioMode == AudioMode::IN_CALL && mAudioMode == AudioMode::NORMAL) {
        // means call ready to start but defer start on set parameters
        LOG(VERBOSE) << __func__ << ": defer start call on call state ACTIVE";
    } else if (newAudioMode == AudioMode::NORMAL &&
               mAudioMode == AudioMode::IN_CALL) {
        // safe to stop now
        stopCall();
    }

    mAudioMode = newAudioMode;
    LOG(VERBOSE) << __func__
                 << ": switching to AudioMode:" << toString(mAudioMode);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Telephony::setTelecomConfig(const TelecomConfig& in_config,
                                               TelecomConfig* _aidl_return) {
    std::scoped_lock lock{mLock};

    if (in_config.voiceVolume.has_value() &&
        (in_config.voiceVolume.value().value <
             TelecomConfig::VOICE_VOLUME_MIN ||
         in_config.voiceVolume.value().value >
             TelecomConfig::VOICE_VOLUME_MAX)) {
        LOG(ERROR) << __func__ << ": voice volume value is invalid: "
                   << in_config.voiceVolume.value().value;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_config.voiceVolume.has_value()) {
        mTelecomConfig.voiceVolume = in_config.voiceVolume;
        // safe to update when there is volume provided
        updateVoiceVolume();
    }
    if (in_config.ttyMode != TelecomConfig::TtyMode::UNSPECIFIED) {
        mTelecomConfig.ttyMode = in_config.ttyMode;
        // safe to update when there is ttymode is provided
        updateTtyMode();
    }
    if (in_config.isHacEnabled.has_value()) {
        mTelecomConfig.isHacEnabled = in_config.isHacEnabled;
    }
    *_aidl_return = mTelecomConfig;
    LOG(DEBUG) << __func__ << ": received " << in_config.toString()
               << ", returning " << _aidl_return->toString();
    return ndk::ScopedAStatus::ok();
}

bool Telephony::isCrsCallSupported() { return false; }

void Telephony::setDevicesFromPatch(const std::vector<AudioDevice>& devices,
                                    const bool isRxUpdate) {
    std::scoped_lock lock{mLock};

    if (isRxUpdate) {
        mRxDevices = devices;
    } else {
        mTxDevices = devices;
        /* update the voice call devices only on TX devices update. Because Rx
         * devices patch is followed by Tx Devices patch */
        updateDevices();
    }
    return;
}

void Telephony::updateDevicesFromPrimaryPlayback() {
    std::scoped_lock lock{mLock};

    if (mPalHandle == nullptr) {
        return;
    }
    auto primaryDevices = mPlatform.getPrimaryPlaybackDevices();
    if (primaryDevices.size() == 0) {
        // Todo check on none devices on primary playback stream
        primaryDevices = {kDefaultRxDevice};
    }

    if (mRxDevices == primaryDevices) {
        LOG(VERBOSE) << __func__ << ": same devices";
        return;
    }

    mRxDevices = primaryDevices;
    mTxDevices = getMatchingTxDevices(mRxDevices);
    if (mRxDevices.size() != mTxDevices.size()) {
        LOG(VERBOSE) << __func__ << ": no Tx found for" << mRxDevices;
        return;
    }
    updateDevices();
}

std::vector<AudioDevice> Telephony::getMatchingTxDevices(
    const std::vector<AudioDevice>& rxDevices) {
    std::vector<AudioDevice> txDevices;
    for (const auto& rxDevice : rxDevices) {
        if (rxDevice.type.type == AudioDeviceType::OUT_SPEAKER_EARPIECE) {
            txDevices.emplace_back(
                AudioDevice{.type.type = AudioDeviceType::IN_MICROPHONE});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_SPEAKER) {
            txDevices.emplace_back(
                AudioDevice{.type.type = AudioDeviceType::IN_MICROPHONE_BACK});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADSET &&
                   rxDevice.type.connection ==
                       AudioDeviceDescription::CONNECTION_ANALOG) {
            txDevices.emplace_back(AudioDevice{
                .type.type = AudioDeviceType::IN_HEADSET,
                .type.connection = AudioDeviceDescription::CONNECTION_ANALOG,
                .address = rxDevice.address});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADPHONE &&
                   rxDevice.type.connection ==
                       AudioDeviceDescription::CONNECTION_ANALOG) {
            txDevices.emplace_back(
                AudioDevice{.type.type = AudioDeviceType::IN_MICROPHONE});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADSET &&
                   rxDevice.type.connection ==
                       AudioDeviceDescription::CONNECTION_USB) {
            txDevices.emplace_back(AudioDevice{
                .type.type = AudioDeviceType::IN_HEADSET,
                .type.connection = AudioDeviceDescription::CONNECTION_USB,
                .address = rxDevice.address});
        } else {
            LOG(ERROR) << __func__ << ": unable to find matching TX device for "
                       << rxDevice.toString();
        }
    }
    return txDevices;
}

void Telephony::reconfigure(const SetUpdates& newUpdates) {
    std::scoped_lock lock{mLock};

    LOG(VERBOSE) << __func__ << " current setUpdates" << mSetUpdates.toString()
                 << " new setUpdates" << newUpdates.toString();
    // Todo Implement
    if (newUpdates.mCallState == CallState::IN_ACTIVE) {
        // this is a sign to stop the call no matter what
        mSetUpdates = newUpdates;
        stopCall();
        return;
    }

    if (mSetUpdates.mCallState != CallState::ACTIVE &&
        newUpdates.mCallState == CallState::ACTIVE) {
        // this is a clear sign that to start a call
        // other parameters value are needed
        mSetUpdates = newUpdates;
        startCall();
        return;
    }
}

void Telephony::updateVoiceVolume() {
    if (mPalHandle == nullptr) {
        return;
    }
    float volumeFloat = mTelecomConfig.voiceVolume
                            ? mTelecomConfig.voiceVolume.value().value
                            : 1.0;
    auto palVolumes = mPlatform.getPalVolumeData({volumeFloat});
    if (int32_t ret = ::pal_stream_set_volume(
            mPalHandle, reinterpret_cast<pal_volume_data*>(palVolumes.data()));
        ret) {
        LOG(ERROR) << __func__ << ": pal stream set volume failed !!" << ret;
        return;
    }
    LOG(VERBOSE) << __func__ << ": updated voice volume value as "
                 << volumeFloat;
}

void Telephony::updateTtyMode() {
    if (mPalHandle == nullptr) {
        return;
    }
    // Todo fix this bad size conversion
    const size_t ttyModeSizeInBytes = 4;
    auto bytes = std::make_unique<uint8_t[]>(sizeof(pal_param_payload) +
                                             ttyModeSizeInBytes);
    auto paramPtr = reinterpret_cast<pal_param_payload*>(bytes.get());
    if (int32_t ret =
            ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_TTY_MODE, paramPtr);
        ret) {
        LOG(ERROR) << __func__ << ": failed to set PAL_PARAM_ID_TTY_MODE";
        return;
    }
    LOG(VERBOSE) << __func__ << ": success";
    return;
}

void Telephony::startCall() {
    auto attributes = mPlatform.getDefaultTelephonyAttributes();

    auto palDevices =
        mPlatform.getPalDevices({mRxDevices.at(0), mTxDevices.at(0)});

    attributes->info.voice_call_info.VSID =
        static_cast<uint32_t>(mSetUpdates.mVSID);
    {
        const auto ttyMode = mTtyMap.find(mTelecomConfig.ttyMode);
        attributes->info.voice_call_info.tty_mode =
            ttyMode != mTtyMap.cend() ? ttyMode->second : PAL_TTY_OFF;
    }

    const size_t numDevices = 2;
    if (int32_t ret = ::pal_stream_open(
            attributes.get(), numDevices,
            reinterpret_cast<pal_device*>(palDevices.data()), 0, nullptr,
            nullptr, reinterpret_cast<uint64_t>(this), &mPalHandle);
        ret) {
        LOG(ERROR) << __func__ << ": pal stream open failed !!" << ret;
        return;
    }
    if (int32_t ret = ::pal_stream_start(mPalHandle); ret) {
        LOG(ERROR) << __func__ << ": pal stream open failed !!" << ret;
        return;
    }
    updateVoiceVolume();
}

void Telephony::stopCall() {
    if (mPalHandle == nullptr) {
        return;
    }
    ::pal_stream_stop(mPalHandle);
    ::pal_stream_close(mPalHandle);
    mPalHandle == nullptr;
}

void Telephony::updateDevices() {
    auto palDevices =
        mPlatform.getPalDevices({mRxDevices.at(0), mTxDevices.at(0)});

    // TODO configure pal devices with custom key if any

    if (int32_t ret = ::pal_stream_set_device(
            mPalHandle, 2, reinterpret_cast<pal_device*>(palDevices.data()));
        ret) {
        LOG(ERROR) << __func__ << ": failed to set devices";
        return;
    }
    LOG(VERBOSE) << __func__ << ": set devices Rx: " << mRxDevices
                 << " Tx: " << mTxDevices;
}

std::ostream& operator<<(std::ostream& os, const Telephony::CallState& state) {
    switch (state) {
        case Telephony::CallState::INVALID:
            os << "INVALID";
            break;
        case Telephony::CallState::IN_ACTIVE:
            os << "IN_ACTIVE";
            break;
        case Telephony::CallState::ACTIVE:
            os << "ACTIVE";
            break;
        default:
            os << "UNKNOWN";
            break;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Telephony::VSID& vsid) {
    switch (vsid) {
        case Telephony::VSID::INVALID:
            os << "INVALID";
            break;
        case Telephony::VSID::VSID_1:
            os << "VSID_1";
            break;
        case Telephony::VSID::VSID_2:
            os << "VSID_2";
            break;
        default:
            os << "UNKNOWN";
            break;
    }
    return os;
}

}  // namespace qti::audio::core
