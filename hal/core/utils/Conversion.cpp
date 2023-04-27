/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <utils/Conversion.h>

// Device Conversions
AudioDeviceDescription makeAudioDeviceDescription(AudioDeviceType type,
        const std::string& connection) {
    AudioDeviceDescription result;
    result.type = type;
    result.connection = connection;
    return result;
}

const DevicePairs& getDevicePairs() {
    static const DevicePairs pairs = []() {
        DevicePairs pairs = {{
            {
                AudioDeviceDescription{}, PAL_DEVICE_NONE
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEFAULT), PAL_DEVICE_OUT_SPEAKER
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_SPEAKER_EARPIECE), PAL_DEVICE_OUT_HANDSET
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_SPEAKER), PAL_DEVICE_OUT_SPEAKER
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_HEADPHONE,
                        AudioDeviceDescription::CONNECTION_ANALOG),
                        PAL_DEVICE_OUT_WIRED_HEADPHONE
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEVICE,
                        AudioDeviceDescription::CONNECTION_BT_SCO), PAL_DEVICE_OUT_BLUETOOTH_SCO
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_CARKIT,
                        AudioDeviceDescription::CONNECTION_BT_SCO), PAL_DEVICE_OUT_BLUETOOTH_SCO
            },
            {
                makeAudioDeviceDescription(
                        AudioDeviceType::OUT_TELEPHONY_TX), PAL_DEVICE_NONE
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_LINE_AUX),PAL_DEVICE_OUT_AUX_LINE
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_SPEAKER_SAFE),PAL_DEVICE_OUT_SPEAKER
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_HEARING_AID,
                        AudioDeviceDescription::CONNECTION_WIRELESS), PAL_DEVICE_OUT_HEARING_AID
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_SPEAKER,
                        AudioDeviceDescription::CONNECTION_BT_LE),PAL_DEVICE_OUT_BLUETOOTH_BLE
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_BROADCAST,
                        AudioDeviceDescription::CONNECTION_BT_LE),PAL_DEVICE_OUT_BLUETOOTH_BLE_BROADCAST
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_DEFAULT),PAL_DEVICE_IN_HANDSET_MIC
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_MICROPHONE),PAL_DEVICE_IN_HANDSET_MIC
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_MICROPHONE_BACK),PAL_DEVICE_IN_SPEAKER_MIC
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_TELEPHONY_RX),PAL_DEVICE_IN_TELEPHONY_RX
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_ECHO_REFERENCE),PAL_DEVICE_IN_ECHO_REF
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_HEADSET,
                        AudioDeviceDescription::CONNECTION_ANALOG),PAL_DEVICE_IN_WIRED_HEADSET
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_HEADSET,
                        AudioDeviceDescription::CONNECTION_ANALOG),PAL_DEVICE_OUT_WIRED_HEADSET
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_HEADSET,
                        AudioDeviceDescription::CONNECTION_BT_SCO),PAL_DEVICE_IN_BLUETOOTH_SCO_HEADSET
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_HEADSET,
                        AudioDeviceDescription::CONNECTION_BT_SCO),PAL_DEVICE_OUT_BLUETOOTH_SCO
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_DEVICE,
                        AudioDeviceDescription::CONNECTION_HDMI),PAL_DEVICE_IN_HDMI
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEVICE,
                        AudioDeviceDescription::CONNECTION_HDMI),PAL_DEVICE_OUT_HDMI
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_ACCESSORY,
                        AudioDeviceDescription::CONNECTION_USB),PAL_DEVICE_IN_USB_ACCESSORY
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_FM_TUNER),PAL_DEVICE_IN_FM_TUNER
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_FM),PAL_DEVICE_OUT_FM
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_DEVICE,
                        AudioDeviceDescription::CONNECTION_ANALOG),PAL_DEVICE_IN_LINE
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEVICE,
                        AudioDeviceDescription::CONNECTION_ANALOG),PAL_DEVICE_OUT_WIRED_HEADPHONE
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_DEVICE,
                        AudioDeviceDescription::CONNECTION_SPDIF),PAL_DEVICE_IN_SPDIF
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEVICE,
                        AudioDeviceDescription::CONNECTION_SPDIF),PAL_DEVICE_OUT_SPDIF
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_DEVICE,
                        AudioDeviceDescription::CONNECTION_BT_A2DP),PAL_DEVICE_IN_BLUETOOTH_A2DP
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEVICE,
                        AudioDeviceDescription::CONNECTION_BT_A2DP),PAL_DEVICE_OUT_BLUETOOTH_A2DP
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_AFE_PROXY),PAL_DEVICE_IN_PROXY
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_AFE_PROXY),PAL_DEVICE_OUT_PROXY
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_DEVICE,
                        AudioDeviceDescription::CONNECTION_USB),PAL_DEVICE_OUT_USB_DEVICE
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_HEADSET,
                        AudioDeviceDescription::CONNECTION_USB),PAL_DEVICE_IN_USB_HEADSET
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_HEADSET,
                        AudioDeviceDescription::CONNECTION_USB),PAL_DEVICE_OUT_USB_HEADSET
            },

            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::IN_HEADSET,
                        AudioDeviceDescription::CONNECTION_BT_LE),PAL_DEVICE_IN_BLUETOOTH_BLE
            },
            {
                 makeAudioDeviceDescription(
                        AudioDeviceType::OUT_HEADSET,
                        AudioDeviceDescription::CONNECTION_BT_LE),PAL_DEVICE_OUT_BLUETOOTH_BLE
            }
        }};

	return pairs;
    }();
    return pairs;
}

// AudioFormat Conversions
AudioFormatDescription make_AudioFormatDescription(AudioFormatType type) {
    AudioFormatDescription result;
    result.type = type;
    return result;
}

AudioFormatDescription make_AudioFormatDescription(PcmType pcm) {
    auto result = make_AudioFormatDescription(AudioFormatType::PCM);
    result.pcm = pcm;
    return result;
}

AudioFormatDescription make_AudioFormatDescription(const std::string& encoding) {
    AudioFormatDescription result;
    result.encoding = encoding;
    return result;
}

AudioFormatDescription make_AudioFormatDescription(PcmType transport,
        const std::string& encoding) {
    auto result = make_AudioFormatDescription(encoding);
    result.pcm = transport;
    return result;
}

const FormatPairs& getFormatPairs() {
    // TODO other formats
    static const FormatPairs pairs = {{
        {
            PAL_AUDIO_FMT_PCM_S16_LE, make_AudioFormatDescription(PcmType::INT_16_BIT)
        },
        {
            PAL_AUDIO_FMT_PCM_S8, make_AudioFormatDescription(PcmType::UINT_8_BIT)
        },
        {
            PAL_AUDIO_FMT_PCM_S32_LE, make_AudioFormatDescription(PcmType::INT_32_BIT)
        },
        {
            PAL_AUDIO_FMT_PCM_S24_LE, make_AudioFormatDescription(PcmType::FIXED_Q_8_24)
        },
        {
            PAL_AUDIO_FMT_PCM_S32_LE, make_AudioFormatDescription(PcmType::FLOAT_32_BIT)
        },
        {
            PAL_AUDIO_FMT_PCM_S24_3LE, make_AudioFormatDescription(PcmType::INT_24_BIT)
        },
        {
            PAL_AUDIO_FMT_AAC, make_AudioFormatDescription(android::MEDIA_MIMETYPE_AUDIO_AAC_LC)
        },
    }};
    return pairs;
}

// Todo verify mappings
static auto getPalChannelInfoForChannelCount(int count) {
    auto ch_info = std::make_shared<pal_channel_info>();
    if (count == 1) {
        ch_info->channels = 1;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
    } else if (count == 2) {
        ch_info->channels = 2;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
    } else if (count == 3) {
        ch_info->channels = 3;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        ch_info->ch_map[2] = PAL_CHMAP_CHANNEL_C;
    } else if (count == 4) {
        ch_info->channels = 4;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        ch_info->ch_map[2] = PAL_CHMAP_CHANNEL_C;
        ch_info->ch_map[3] = PAL_CHMAP_CHANNEL_LFE;
    } else if (count == 5) {
        ch_info->channels = 5;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        ch_info->ch_map[2] = PAL_CHMAP_CHANNEL_C;
        ch_info->ch_map[3] = PAL_CHMAP_CHANNEL_LFE;
        ch_info->ch_map[4] = PAL_CHMAP_CHANNEL_RC;
    } else if (count == 6) {
        ch_info->channels = 6;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        ch_info->ch_map[2] = PAL_CHMAP_CHANNEL_C;
        ch_info->ch_map[3] = PAL_CHMAP_CHANNEL_LFE;
        ch_info->ch_map[4] = PAL_CHMAP_CHANNEL_LB;
        ch_info->ch_map[5] = PAL_CHMAP_CHANNEL_RB;
    } else if (count == 7) {
        ch_info->channels = 7;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        ch_info->ch_map[2] = PAL_CHMAP_CHANNEL_C;
        ch_info->ch_map[3] = PAL_CHMAP_CHANNEL_LFE;
        ch_info->ch_map[4] = PAL_CHMAP_CHANNEL_LB;
        ch_info->ch_map[5] = PAL_CHMAP_CHANNEL_RB;
        ch_info->ch_map[6] = PAL_CHMAP_CHANNEL_LS;
    } else if (count == 8) {
        ch_info->channels = 8;
        ch_info->ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        ch_info->ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        ch_info->ch_map[2] = PAL_CHMAP_CHANNEL_C;
        ch_info->ch_map[3] = PAL_CHMAP_CHANNEL_LFE;
        ch_info->ch_map[4] = PAL_CHMAP_CHANNEL_LB;
        ch_info->ch_map[5] = PAL_CHMAP_CHANNEL_RB;
        ch_info->ch_map[6] = PAL_CHMAP_CHANNEL_LS;
        ch_info->ch_map[7] = PAL_CHMAP_CHANNEL_RS;
    } else {
        ch_info->channels = 0;
    }
    return ch_info;
}

ChannelCountToPalChannelInfoMap& buildPalChannelInfos() {
    // TODO verify based tag ? like AudioChannelLayout::Tag::layoutMask
    static ChannelCountToPalChannelInfoMap sMap = {
        {1, *getPalChannelInfoForChannelCount(1)},
        {2, *getPalChannelInfoForChannelCount(2)},
        {3, *getPalChannelInfoForChannelCount(3)},
        {4, *getPalChannelInfoForChannelCount(4)},
        {5, *getPalChannelInfoForChannelCount(5)},
        {6, *getPalChannelInfoForChannelCount(6)},
        {7, *getPalChannelInfoForChannelCount(7)},
        {8, *getPalChannelInfoForChannelCount(8)},
    };

    return sMap;
}
