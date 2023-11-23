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
#include <hardware/audio.h>
#include <qti-audio-core/StreamInPrimary.h>
#include <qti-audio-core/StreamOutPrimary.h>
#include <qti-audio-core/Telephony.h>
#include <qti-audio-core/Utils.h>
#include <system/audio.h>

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

Telephony::~Telephony() {
    stopCall();
}

ndk::ScopedAStatus Telephony::getSupportedAudioModes(std::vector<AudioMode>* _aidl_return) {
    std::scoped_lock lock{mLock};

    *_aidl_return = mSupportedAudioModes;
    LOG(DEBUG) << __func__ << ": returning " << ::android::internal::ToString(*_aidl_return);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Telephony::switchAudioMode(AudioMode newAudioMode) {
    std::scoped_lock lock{mLock};

    if (std::find(mSupportedAudioModes.begin(), mSupportedAudioModes.end(), newAudioMode) ==
        mSupportedAudioModes.end()) {
        LOG(ERROR) << __func__ << ": unsupported mode " << toString(newAudioMode);
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    mPlatform.updateCallMode((int)newAudioMode);

    if (mAudioMode == newAudioMode) {
        LOG(VERBOSE) << __func__ << ": no change" << toString(newAudioMode);
        return ndk::ScopedAStatus::ok();
    }
    if (newAudioMode == AudioMode::IN_CALL && mAudioMode == AudioMode::NORMAL) {
        // means call ready to start but defer start on set parameters
        LOG(VERBOSE) << __func__ << ": defer start call on call state ACTIVE";
    } else if (newAudioMode == AudioMode::NORMAL && mAudioMode == AudioMode::IN_CALL) {
        // safe to stop now
        stopCall();
    }

    mAudioMode = newAudioMode;
    LOG(VERBOSE) << __func__ << ": switching to AudioMode:" << toString(mAudioMode);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Telephony::setTelecomConfig(const TelecomConfig& in_config,
                                               TelecomConfig* _aidl_return) {
    std::scoped_lock lock{mLock};

    if (in_config.voiceVolume.has_value() &&
        (in_config.voiceVolume.value().value < TelecomConfig::VOICE_VOLUME_MIN ||
         in_config.voiceVolume.value().value > TelecomConfig::VOICE_VOLUME_MAX)) {
        LOG(ERROR) << __func__
                   << ": voice volume value is invalid: " << in_config.voiceVolume.value().value;
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
    LOG(DEBUG) << __func__ << ": received " << in_config.toString() << ", returning "
               << _aidl_return->toString();
    return ndk::ScopedAStatus::ok();
}

void Telephony::setMicMute(const bool muted) {
    std::scoped_lock lock{mLock};
    if (mPalHandle == nullptr) {
        return;
    }
    if (!mPlatform.setStreamMicMute(mPalHandle, muted)) {
        LOG(ERROR) << __func__ << ": failed";
    }
}

bool Telephony::isCrsCallSupported() {
    return false;
}

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

void Telephony::updateVoiceMetadataForBT(bool call_active) {
    ssize_t track_count = 1;
    std::vector<playback_track_metadata_t> sourceTracks;
    std::vector<record_track_metadata_t> sinkTracks;
    sourceTracks.resize(track_count);
    sinkTracks.resize(track_count);
    int32_t ret = 0;

    source_metadata_t btSourceMetadata;
    sink_metadata_t btSinkMetadata;

    if (call_active) {
        btSourceMetadata.track_count = track_count;
        btSourceMetadata.tracks = sourceTracks.data();

        btSourceMetadata.tracks->usage = AUDIO_USAGE_VOICE_COMMUNICATION;
        btSourceMetadata.tracks->content_type = AUDIO_CONTENT_TYPE_SPEECH;

        LOG(DEBUG) << __func__
                   << "Source metadata for voice call usage: " << btSourceMetadata.tracks->usage
                   << "content_type: " << btSourceMetadata.tracks->content_type;
        // Pass the source metadata to PAL
        pal_set_param(PAL_PARAM_ID_SET_SOURCE_METADATA, (void*)&btSourceMetadata, 0);

        btSinkMetadata.track_count = track_count;
        btSinkMetadata.tracks = sinkTracks.data();

        btSinkMetadata.tracks->source = AUDIO_SOURCE_VOICE_CALL;

        LOG(DEBUG) << __func__
                   << "Sink metadata for voice call source: " << btSinkMetadata.tracks->source;
        // Pass the sink metadata to PAL
        pal_set_param(PAL_PARAM_ID_SET_SINK_METADATA, (void*)&btSinkMetadata, 0);
    } else {
        /* When voice call ends, we need to restore metadata configuration for
         * source and sink sessions same as prior to the call. Send source
         * and sink metadata separately to BT.
         */
        if (mStreamOutPrimary.lock()) {
            StreamOutPrimary::sourceMetadata_mutex_.lock();
            ret = mStreamOutPrimary.lock()->setAggregateSourceMetadata(false);
            if (ret != 0) {
                LOG(ERROR) << __func__ << " Set PAL_PARAM_ID_SET_SOURCE_METADATA for" << ret
                           << " failed";
            }
            StreamOutPrimary::sourceMetadata_mutex_.unlock();
        }

        if (mStreamInPrimary.lock()) {
            StreamInPrimary::sinkMetadata_mutex_.lock();
            ret = mStreamInPrimary.lock()->setAggregateSinkMetadata(false);
            if (ret != 0) {
                LOG(ERROR) << __func__ << " Set PAL_PARAM_ID_SET_SINK_METADATA for" << ret
                           << " failed";
            }
            StreamInPrimary::sinkMetadata_mutex_.unlock();
        }
    }
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
            txDevices.emplace_back(AudioDevice{.type.type = AudioDeviceType::IN_MICROPHONE});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_SPEAKER) {
            txDevices.emplace_back(AudioDevice{.type.type = AudioDeviceType::IN_MICROPHONE_BACK});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADSET &&
                   rxDevice.type.connection == AudioDeviceDescription::CONNECTION_ANALOG) {
            txDevices.emplace_back(
                    AudioDevice{.type.type = AudioDeviceType::IN_HEADSET,
                                .type.connection = AudioDeviceDescription::CONNECTION_ANALOG,
                                .address = rxDevice.address});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADPHONE &&
                   rxDevice.type.connection == AudioDeviceDescription::CONNECTION_ANALOG) {
            txDevices.emplace_back(AudioDevice{.type.type = AudioDeviceType::IN_MICROPHONE});
        } else if ((rxDevice.type.type == AudioDeviceType::OUT_DEVICE ||
                    rxDevice.type.type == AudioDeviceType::OUT_HEADSET) &&
                   rxDevice.type.connection == AudioDeviceDescription::CONNECTION_BT_SCO) {
            txDevices.emplace_back(
                    AudioDevice{.type.type = AudioDeviceType::IN_HEADSET,
                                .type.connection = AudioDeviceDescription::CONNECTION_BT_SCO});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADSET &&
                   rxDevice.type.connection == AudioDeviceDescription::CONNECTION_BT_LE) {
            txDevices.emplace_back(
                    AudioDevice{.type.type = AudioDeviceType::IN_HEADSET,
                                .type.connection = AudioDeviceDescription::CONNECTION_BT_LE});
        } else if (rxDevice.type.type == AudioDeviceType::OUT_HEADSET &&
                   rxDevice.type.connection == AudioDeviceDescription::CONNECTION_USB) {
            txDevices.emplace_back(
                    AudioDevice{.type.type = AudioDeviceType::IN_HEADSET,
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

    LOG(VERBOSE) << __func__ << " current setUpdates" << mSetUpdates.toString() << " new setUpdates"
                 << newUpdates.toString();
    // Todo Implement
    mPlatform.updateCallState((int)mSetUpdates.mCallState);

    if (mSetUpdates.mCallState != CallState::ACTIVE && newUpdates.mCallState == CallState::ACTIVE) {
        // this is a clear sign that to start a call
        // other parameters value are needed
        LOG(VERBOSE) << __func__ << "INACTIVE -> ACTIVE";
        mSetUpdates = newUpdates;
        updateVoiceMetadataForBT(true);
        startCall();
        return;
    }
    if (newUpdates.mCallState == CallState::IN_ACTIVE && mSetUpdates.mCallState == CallState::ACTIVE) {
        // this is a sign to stop the call no matter what
        LOG(VERBOSE) << __func__ << "ACTIVE -> INACTIVE";
        mSetUpdates = newUpdates;
        stopCall();
        return;
    }
}

void Telephony::updateVolumeBoost(const bool enable) {
    std::scoped_lock lock{mLock};
    mIsVolumeBoostEnabled = enable;
    LOG(INFO) << __func__ << ": is enabled: " << mIsVolumeBoostEnabled;
    configureVolumeBoost();
}

void Telephony::updateSlowTalk(const bool enable) {
    std::scoped_lock lock{mLock};
    mIsSlowTalkEnabled = enable;
    LOG(INFO) << __func__ << ": is enabled: " << mIsSlowTalkEnabled;
    configureSlowTalk();
}

void Telephony::updateHDVoice(const bool enable) {
    std::scoped_lock lock{mLock};
    mIsHDVoiceEnabled = enable;
    LOG(INFO) << __func__ << ": is enabled: " << mIsHDVoiceEnabled;
    configureHDVoice();
}

void Telephony::updateDeviceMute(const bool isMute, const std::string& muteDirection) {
    std::scoped_lock lock{mLock};
    mIsDeviceMuted = isMute;
    mMuteDirection = muteDirection;
    LOG(INFO) << __func__ << ": is muted: " << mIsDeviceMuted
              << ", mute direction: " << mMuteDirection;
    configureDeviceMute();
}

void Telephony::configureVolumeBoost() {
    if (mPalHandle == nullptr) {
        LOG(ERROR) << __func__ << ": invalid pal handle";
        return;
    }
    auto byteSize = sizeof(pal_param_payload) + sizeof(bool);
    auto bytes = std::make_unique<uint8_t[]>(byteSize);
    auto palParamPayload = reinterpret_cast<pal_param_payload*>(bytes.get());
    palParamPayload->payload_size = sizeof(bool);
    palParamPayload->payload[0] = mIsVolumeBoostEnabled;
    if (int32_t ret =
                ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_VOLUME_BOOST, palParamPayload);
        ret) {
        LOG(ERROR) << __func__ << ": failed to set PAL_PARAM_ID_VOLUME_BOOST";
        return;
    }
}

void Telephony::configureSlowTalk() {
    if (mPalHandle == nullptr) {
        LOG(ERROR) << __func__ << ": invalid pal handle";
        return;
    }
    auto byteSize = sizeof(pal_param_payload) + sizeof(bool);
    auto bytes = std::make_unique<uint8_t[]>(byteSize);
    auto palParamPayload = reinterpret_cast<pal_param_payload*>(bytes.get());
    palParamPayload->payload_size = sizeof(bool);
    palParamPayload->payload[0] = mIsSlowTalkEnabled;
    if (int32_t ret = ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_SLOW_TALK, palParamPayload);
        ret) {
        LOG(ERROR) << __func__ << ": failed to set PAL_PARAM_ID_SLOW_TALK";
        return;
    }
}

void Telephony::configureHDVoice() {
    if (mPalHandle == nullptr) {
        LOG(ERROR) << __func__ << ": invalid pal handle";
        return;
    }
    auto byteSize = sizeof(pal_param_payload) + sizeof(bool);
    auto bytes = std::make_unique<uint8_t[]>(byteSize);
    auto palParamPayload = reinterpret_cast<pal_param_payload*>(bytes.get());
    palParamPayload->payload_size = sizeof(bool);
    palParamPayload->payload[0] = mIsHDVoiceEnabled;
    if (int32_t ret = ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_HD_VOICE, palParamPayload);
        ret) {
        LOG(ERROR) << __func__ << ": failed to set PAL_PARAM_ID_HD_VOICE";
        return;
    }
}

void Telephony::configureDeviceMute() {
    if (mPalHandle == nullptr) {
        LOG(ERROR) << __func__ << ": invalid pal handle";
        return;
    }
    auto byteSize = sizeof(pal_param_payload) + sizeof(pal_device_mute_t);
    auto bytes = std::make_unique<uint8_t[]>(byteSize);
    auto palParamPayload = reinterpret_cast<pal_param_payload*>(bytes.get());
    palParamPayload->payload_size = sizeof(pal_device_mute_t);
    auto palDeviceMute =
            reinterpret_cast<pal_device_mute_t*>(palParamPayload + sizeof(pal_param_payload));
    palDeviceMute->mute = mIsDeviceMuted;
    if (mMuteDirection == "rx") {
        palDeviceMute->dir = PAL_AUDIO_OUTPUT;
    } else {
        palDeviceMute->dir = PAL_AUDIO_INPUT;
    }
    if (int32_t ret = ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_DEVICE_MUTE, palParamPayload);
        ret) {
        LOG(ERROR) << __func__ << ": failed to set PAL_PARAM_ID_DEVICE_MUTE";
        return;
    }
}

void Telephony::updateVoiceVolume() {
    if (mPalHandle == nullptr) {
        return;
    }
    float volumeFloat = mTelecomConfig.voiceVolume ? mTelecomConfig.voiceVolume.value().value : 1.0;
    auto palVolumes = mPlatform.getPalVolumeData({volumeFloat});
    if (int32_t ret = ::pal_stream_set_volume(
                mPalHandle, reinterpret_cast<pal_volume_data*>(palVolumes.data()));
        ret) {
        LOG(ERROR) << __func__ << ": pal stream set volume failed !!" << ret;
        return;
    }
    LOG(VERBOSE) << __func__ << ": updated voice volume value as " << volumeFloat;
}

void Telephony::updateTtyMode() {
    if (mPalHandle == nullptr) {
        return;
    }
    // Todo fix this bad size conversion
    const size_t ttyModeSizeInBytes = 4;
    auto bytes = std::make_unique<uint8_t[]>(sizeof(pal_param_payload) + ttyModeSizeInBytes);
    auto paramPtr = reinterpret_cast<pal_param_payload*>(bytes.get());
    if (int32_t ret = ::pal_stream_set_param(mPalHandle, PAL_PARAM_ID_TTY_MODE, paramPtr); ret) {
        LOG(ERROR) << __func__ << ": failed to set PAL_PARAM_ID_TTY_MODE";
        return;
    }
    LOG(VERBOSE) << __func__ << ": success";
    return;
}

void Telephony::startCall() {
    LOG(VERBOSE) << __func__ << ": Enter";
    auto attributes = mPlatform.getDefaultTelephonyAttributes();

    auto palDevices = mPlatform.getPalDevices({mRxDevices.at(0), mTxDevices.at(0)});

    attributes->info.voice_call_info.VSID = static_cast<uint32_t>(mSetUpdates.mVSID);
    {
        const auto ttyMode = mTtyMap.find(mTelecomConfig.ttyMode);
        attributes->info.voice_call_info.tty_mode =
                ttyMode != mTtyMap.cend() ? ttyMode->second : PAL_TTY_OFF;
    }

    const size_t numDevices = 2;
    if (int32_t ret = ::pal_stream_open(
                attributes.get(), numDevices, reinterpret_cast<pal_device*>(palDevices.data()), 0,
                nullptr, nullptr, reinterpret_cast<uint64_t>(this), &mPalHandle);
        ret) {
        LOG(ERROR) << __func__ << ": pal stream open failed !!" << ret;
        return;
    }
    if (int32_t ret = ::pal_stream_start(mPalHandle); ret) {
        LOG(ERROR) << __func__ << ": pal stream open failed !!" << ret;
        return;
    }
    updateVoiceVolume();
    LOG(VERBOSE) << __func__ << ": Exit";
}

void Telephony::stopCall() {
    LOG(VERBOSE) << __func__ << ": Enter";
    if (mPalHandle == nullptr) {
        return;
    }
    ::pal_stream_stop(mPalHandle);
    ::pal_stream_close(mPalHandle);
    updateVoiceMetadataForBT(false);
    mPalHandle = nullptr;
    LOG(VERBOSE) << __func__ << ": EXIT";
}

void Telephony::updateDevices() {
    auto palDevices = mPlatform.getPalDevices({mRxDevices.at(0), mTxDevices.at(0)});

    pal_param_bta2dp_t* param_bt_a2dp_ptr = nullptr;
    bool a2dp_capture_suspended = false;
    size_t bt_param_size = 0;
    bool a2dp_suspended = false;
    int ret = 0;
    int retry_cnt = 20;
    const int retry_period_ms = 100;
    bool is_suspend_setparam = false;

    // TODO configure pal devices with custom key if any
    if (mSetUpdates.mCallState == CallState::ACTIVE) {
        updateVoiceMetadataForBT(true);
        /*In case of active LEA profile, if voice call accepted by an inactive legacy headset
         * over SCO profile. APM is not aware about SCO active profile until BT_SCO=ON
         * event triggers from BT. In meantime before BT_SCO=ON, when LEA is suspended via
         * setparam call, APM tries to route voice call to BLE device.
         * In RouteStream call, if suspended state for LEA is true it keep checks over a
         * sleep of 2 secs. This causes timecheck issue in audioservice. Thus check for
         * is_suspend_setparam flag to know whether BLE suspended due to the actual setparam
         * or reconfig_cb(suspend->resume).
       */
        if ((palDevices[0].id == PAL_DEVICE_OUT_BLUETOOTH_BLE) &&
            (palDevices[1].id == PAL_DEVICE_IN_BLUETOOTH_BLE)) {
            pal_param_bta2dp_t param_bt_a2dp;
            do {
                std::unique_lock<std::mutex> guard(AudioExtension::reconfig_wait_mutex_);
                param_bt_a2dp_ptr = &param_bt_a2dp;
                param_bt_a2dp_ptr->dev_id = PAL_DEVICE_OUT_BLUETOOTH_BLE;

                ret = pal_get_param(PAL_PARAM_ID_BT_A2DP_SUSPENDED, (void**)&param_bt_a2dp_ptr,
                                    &bt_param_size, nullptr);
                if (!ret && bt_param_size && param_bt_a2dp_ptr) {
                    a2dp_suspended = param_bt_a2dp_ptr->a2dp_suspended;
                    is_suspend_setparam = param_bt_a2dp_ptr->is_suspend_setparam;
                } else {
                    LOG(ERROR) << __func__ << "getparam for PAL_PARAM_ID_BT_A2DP_SUSPENDED failed";
                }
                param_bt_a2dp_ptr = &param_bt_a2dp;
                param_bt_a2dp_ptr->dev_id = PAL_DEVICE_IN_BLUETOOTH_BLE;
                bt_param_size = 0;
                ret = pal_get_param(PAL_PARAM_ID_BT_A2DP_CAPTURE_SUSPENDED,
                                    (void**)&param_bt_a2dp_ptr, &bt_param_size, nullptr);
                if (!ret && bt_param_size && param_bt_a2dp_ptr)
                    a2dp_capture_suspended = param_bt_a2dp_ptr->a2dp_capture_suspended;
                else
                    LOG(ERROR) << __func__ << "getparam for BT_A2DP_CAPTURE_SUSPENDED failed";
                param_bt_a2dp_ptr = nullptr;
                bt_param_size = 0;
            } while (!is_suspend_setparam && (a2dp_suspended || a2dp_capture_suspended) &&
                     retry_cnt-- && !usleep(retry_period_ms * 1000));
            LOG(INFO) << __func__ << "a2dp_suspended status: " << a2dp_suspended
                      << "and a2dp_capture_suspended status: " << a2dp_capture_suspended;
        }
    }
    if (int32_t ret = ::pal_stream_set_device(mPalHandle, 2,
                                              reinterpret_cast<pal_device*>(palDevices.data()));
        ret) {
        LOG(ERROR) << __func__ << ": failed to set devices";
        return;
    }
    LOG(VERBOSE) << __func__ << ": set devices Rx: " << mRxDevices << " Tx: " << mTxDevices;
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

} // namespace qti::audio::core
