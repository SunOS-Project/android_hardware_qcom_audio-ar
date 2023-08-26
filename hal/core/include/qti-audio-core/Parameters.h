/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <string>

namespace qti::audio::core::Parameters {

/**
 * Since the parameters from the Android framework enables or disables features
 * which would impact small to big level, It is highly recommended to write
 * verbose comments for each parameter. As Parameter is composition 'id' and 'its
 * possibles values', hence list all the values with verbose explaination
 **/

// HDR Recording
const static std::string kHdrRecord{"hdr_record_on"};
const static std::string kWnr{"wnr_on"};
const static std::string kAns{"ans_on"};
const static std::string kOrientation{"orientation"};
const static std::string kInverted{"inverted"};
const static std::string kHdrChannelCount{"hdr_audio_channel_count"};
const static std::string kHdrSamplingRate{"hdr_audio_sampling_rate"};

// voice
const static std::string kVoiceCallState{"call_state"};
const static std::string kVoiceCallType{"call_type"};
const static std::string kVoiceVSID{"vsid"};
const static std::string kVoiceDeviceMute{"device_mute"};
const static std::string kVoiceDirection{"direction"};
const static std::string kVoiceSlowTalk{"st_enable"};
const static std::string kVoiceHDVoice{"hd_voice"};
const static std::string kVoiceIsCRsSupported{"isCRSsupported"};
const static std::string kVoiceCRSCall{"crs_call"};
const static std::string kVoiceCRSVolume{"CRS_volume"};

// others
const static std::string kUHQA{"UHQA"};
const static std::string kFbspCfgWaitTime{"fbsp_cfg_wait_time"};
const static std::string kFbspValiWaitTime{"fbsp_v_vali_wait_time"};
const static std::string kTriggerSpeakerCall{"trigger_spkr_cal"};
const static std::string kWfdChannelMap{"wfd_channel_cap"};
const static std::string kHapticsIntensity{"haptics_intensity"};
const static std::string kIcmdPlayback{"icmd_playback"};

};  // namespace qti::audio::core::Parameters