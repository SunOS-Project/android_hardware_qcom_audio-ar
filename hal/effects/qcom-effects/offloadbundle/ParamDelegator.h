/*

 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <errno.h>
#include <errno.h>
#include <log/log.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "PalApi.h"
#include "kvh2xml.h"

namespace aidl::qti::effects {

// #define EQ_PARAM_LATENCY 0x80000000
// #define EQ_PARAM_CUSTOM_HEADER_SIZE 12 // 3*sizeof(uint32_t)
// #define EQ_PARAM_NORMAL_HEADER_SIZE 8    // 2*sizeof(uint32_t)
// #define EQ_ENABLE_CMD_SIZE 4
// #define EQ_PRESET_CMD_SIZE 12	//pregain, preset, num band = 0

#define VIRTUALIZER_MODULE 0x00001000
#define VIRTUALIZER_ENABLE 0x00001001
#define VIRTUALIZER_STRENGTH 0x00001002
#define VIRTUALIZER_OUT_TYPE 0x00001003
#define VIRTUALIZER_GAIN_ADJUST 0x00001004
#define VIRTUALIZER_ENABLE_PARAM_LEN 1
#define VIRTUALIZER_STRENGTH_PARAM_LEN 1
#define VIRTUALIZER_OUT_TYPE_PARAM_LEN 1
#define VIRTUALIZER_GAIN_ADJUST_PARAM_LEN 1

#define REVERB_MODULE 0x00002000
#define REVERB_ENABLE 0x00002001
#define REVERB_MODE 0x00002002
#define REVERB_PRESET 0x00002003
#define REVERB_WET_MIX 0x00002004
#define REVERB_GAIN_ADJUST 0x00002005
#define REVERB_ROOM_LEVEL 0x00002006
#define REVERB_ROOM_HF_LEVEL 0x00002007
#define REVERB_DECAY_TIME 0x00002008
#define REVERB_DECAY_HF_RATIO 0x00002009
#define REVERB_REFLECTIONS_LEVEL 0x0000200a
#define REVERB_REFLECTIONS_DELAY 0x0000200b
#define REVERB_LEVEL 0x0000200c
#define REVERB_DELAY 0x0000200d
#define REVERB_DIFFUSION 0x0000200e
#define REVERB_DENSITY 0x0000200f
#define REVERB_ENABLE_PARAM_LEN 1
#define REVERB_MODE_PARAM_LEN 1
#define REVERB_PRESET_PARAM_LEN 1
#define REVERB_WET_MIX_PARAM_LEN 1
#define REVERB_GAIN_ADJUST_PARAM_LEN 1
#define REVERB_ROOM_LEVEL_PARAM_LEN 1
#define REVERB_ROOM_HF_LEVEL_PARAM_LEN 1
#define REVERB_DECAY_TIME_PARAM_LEN 1
#define REVERB_DECAY_HF_RATIO_PARAM_LEN 1
#define REVERB_REFLECTIONS_LEVEL_PARAM_LEN 1
#define REVERB_REFLECTIONS_DELAY_PARAM_LEN 1
#define REVERB_LEVEL_PARAM_LEN 1
#define REVERB_DELAY_PARAM_LEN 1
#define REVERB_DIFFUSION_PARAM_LEN 1
#define REVERB_DENSITY_PARAM_LEN 1

#define BASS_BOOST_MODULE 0x00003000
#define BASS_BOOST_ENABLE 0x00003001
#define BASS_BOOST_MODE 0x00003002
#define BASS_BOOST_STRENGTH 0x00003003
#define BASS_BOOST_ENABLE_PARAM_LEN 1
#define BASS_BOOST_MODE_PARAM_LEN 1
#define BASS_BOOST_STRENGTH_PARAM_LEN 1

#define EQ_MODULE 0x00004000
#define EQ_ENABLE 0x00004001
#define EQ_CONFIG 0x00004002
#define EQ_NUM_BANDS 0x00004003
#define EQ_BAND_LEVELS 0x00004004
#define EQ_BAND_LEVEL_RANGE 0x00004005
#define EQ_BAND_FREQS 0x00004006
#define EQ_SINGLE_BAND_FREQ_RANGE 0x00004007
#define EQ_SINGLE_BAND_FREQ 0x00004008
#define EQ_BAND_INDEX 0x00004009
#define EQ_PRESET_ID 0x0000400a
#define EQ_NUM_PRESETS 0x0000400b
#define EQ_PRESET_NAME 0x0000400c
#define EQ_ENABLE_PARAM_LEN 1
#define EQ_CONFIG_PARAM_LEN 3
#define EQ_CONFIG_PER_BAND_PARAM_LEN 5
#define EQ_NUM_BANDS_PARAM_LEN 1
#define EQ_BAND_LEVELS_PARAM_LEN 13
#define EQ_BAND_LEVEL_RANGE_PARAM_LEN 2
#define EQ_BAND_FREQS_PARAM_LEN 13
#define EQ_SINGLE_BAND_FREQ_RANGE_PARAM_LEN 2
#define EQ_SINGLE_BAND_FREQ_PARAM_LEN 1
#define EQ_BAND_INDEX_PARAM_LEN 1
#define EQ_PRESET_ID_PARAM_LEN 1
#define EQ_NUM_PRESETS_PARAM_LEN 1
#define EQ_PRESET_NAME_PARAM_LEN 32

#define EQ_TYPE_NONE 0
#define EQ_BASS_BOOST 1
#define EQ_BASS_CUT 2
#define EQ_TREBLE_BOOST 3
#define EQ_TREBLE_CUT 4
#define EQ_BAND_BOOST 5
#define EQ_BAND_CUT 6

/* Command Payload length and size for Non-IID commands */
#define COMMAND_PAYLOAD_LEN 3
#define COMMAND_PAYLOAD_SZ (COMMAND_PAYLOAD_LEN * sizeof(__u32))
/* Command Payload length and size for IID commands */
#define COMMAND_IID_PAYLOAD_LEN 4
#define COMMAND_IID_PAYLOAD_SZ (COMMAND_IID_PAYLOAD_LEN * sizeof(__u32))
#define MAX_INBAND_PARAM_SZ 4096
#define Q27_UNITY (1 << 27)
#define Q8_UNITY (1 << 8)
#define CUSTOM_OPENSL_PRESET 18

struct virtualizer_params {
    __u32 device;
    __u32 enable_flag;
    __u32 strength;
    __u32 out_type;
    __s32 gain_adjust;
};

#define NUM_OSL_REVERB_PRESETS_SUPPORTED 6

struct reverb_params {
    __u32 device;
    __u32 enable_flag;
    __u32 mode;
    __u32 preset;
    __u32 wet_mix;
    __s32 gain_adjust;
    __s32 room_level;
    __s32 room_hf_level;
    __u32 decay_time;
    __u32 decay_hf_ratio;
    __s32 reflections_level;
    __u32 reflections_delay;
    __s32 level;
    __u32 delay;
    __u32 diffusion;
    __u32 density;
};

struct bass_boost_params {
    __u32 device;
    __u32 enable_flag;
    __u32 mode;
    __u32 strength;
};

struct BassBoostParams {
    bool mEnabled = false;
    int mStrength = 0;
};

#define MAX_EQ_BANDS 12
#define MAX_OSL_EQ_BANDS 5
struct eq_config_t {
    __s32 eq_pregain;
    __s32 preset_id;
    __u32 num_bands;
};
struct eq_per_band_config_t {
    __s32 band_idx;
    __u32 filter_type;
    __u32 freq_millihertz;
    __s32 gain_millibels;
    __u32 quality_factor;
};

// TODO remove unused.
// struct eq_per_band_freq_range_t {
// 	__u32 band_index;
// 	__u32 min_freq_millihertz;
// 	__u32 max_freq_millihertz;
// };

struct eq_params {
    __u32 device;
    __u32 enable_flag;
    struct eq_config_t config;
    struct eq_per_band_config_t per_band_cfg[MAX_EQ_BANDS];
    // struct eq_per_band_freq_range_t per_band_freq_range[MAX_EQ_BANDS]; // TODO usused.
    __u32 band_index;
    __u32 freq_millihertz;
};

#define PARAM_ID_MODULE_ENABLE 0x8001026
#define PARAM_ID_EQ_CONFIG 0x800110c
#define PARAM_ID_PBE_PARAMS_CONFIG 0x8001150
#define PARAM_ID_BASS_BOOST_MODE 0x800112c
#define PARAM_ID_BASS_BOOST_STRENGTH 0x800112D
#define PARAM_ID_REVERB_MODE 0x80010fd
#define PARAM_ID_REVERB_PRESET 0x80010fe
#define PARAM_ID_REVERB_WET_MIX 0x80010ff
#define PARAM_ID_REVERB_GAIN_ADJUST 0x8001100
#define PARAM_ID_REVERB_ROOM_LEVEL 0x8001101
#define PARAM_ID_REVERB_ROOM_HF_LEVEL 0x8001102
#define PARAM_ID_REVERB_DECAY_TIME 0x8001103
#define PARAM_ID_REVERB_DECAY_HF_RATIO 0x8001104
#define PARAM_ID_REVERB_REFLECTIONS_LEVEL 0x8001105
#define PARAM_ID_REVERB_REFLECTIONS_DELAY 0x8001106
#define PARAM_ID_REVERB_LEVEL 0x8001107
#define PARAM_ID_REVERB_DELAY 0x8001108
#define PARAM_ID_REVERB_DIFFUSION 0x8001109
#define PARAM_ID_REVERB_DENSITY 0x800110a

#define PARAM_ID_VIRTUALIZER_STRENGTH 0x8001136
#define PARAM_ID_VIRTUALIZER_OUT_TYPE 0x8001137
#define PARAM_ID_VIRTUALIZER_GAIN_ADJUST 0x8001138

#define OFFLOAD_SEND_PBE_ENABLE_FLAG (1 << 0)
#define OFFLOAD_SEND_PBE_CONFIG (OFFLOAD_SEND_PBE_ENABLE_FLAG << 1)

#define OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG (1 << 0)
#define OFFLOAD_SEND_BASSBOOST_STRENGTH (OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG << 1)
#define OFFLOAD_SEND_BASSBOOST_MODE (OFFLOAD_SEND_BASSBOOST_STRENGTH << 1)

#define OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG (1 << 0)
#define OFFLOAD_SEND_VIRTUALIZER_STRENGTH (OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG << 1)
#define OFFLOAD_SEND_VIRTUALIZER_OUT_TYPE (OFFLOAD_SEND_VIRTUALIZER_STRENGTH << 1)
#define OFFLOAD_SEND_VIRTUALIZER_GAIN_ADJUST (OFFLOAD_SEND_VIRTUALIZER_OUT_TYPE << 1)

#define OFFLOAD_SEND_EQ_ENABLE_FLAG (1 << 0)
#define OFFLOAD_SEND_EQ_PRESET (OFFLOAD_SEND_EQ_ENABLE_FLAG << 1)
#define OFFLOAD_SEND_EQ_BANDS_LEVEL (OFFLOAD_SEND_EQ_PRESET << 1)

#define OFFLOAD_SEND_REVERB_ENABLE_FLAG (1 << 0)
#define OFFLOAD_SEND_REVERB_MODE (OFFLOAD_SEND_REVERB_ENABLE_FLAG << 1)
#define OFFLOAD_SEND_REVERB_PRESET (OFFLOAD_SEND_REVERB_MODE << 1)
#define OFFLOAD_SEND_REVERB_WET_MIX (OFFLOAD_SEND_REVERB_PRESET << 1)
#define OFFLOAD_SEND_REVERB_GAIN_ADJUST (OFFLOAD_SEND_REVERB_WET_MIX << 1)
#define OFFLOAD_SEND_REVERB_ROOM_LEVEL (OFFLOAD_SEND_REVERB_GAIN_ADJUST << 1)
#define OFFLOAD_SEND_REVERB_ROOM_HF_LEVEL (OFFLOAD_SEND_REVERB_ROOM_LEVEL << 1)
#define OFFLOAD_SEND_REVERB_DECAY_TIME (OFFLOAD_SEND_REVERB_ROOM_HF_LEVEL << 1)
#define OFFLOAD_SEND_REVERB_DECAY_HF_RATIO (OFFLOAD_SEND_REVERB_DECAY_TIME << 1)
#define OFFLOAD_SEND_REVERB_REFLECTIONS_LEVEL (OFFLOAD_SEND_REVERB_DECAY_HF_RATIO << 1)
#define OFFLOAD_SEND_REVERB_REFLECTIONS_DELAY (OFFLOAD_SEND_REVERB_REFLECTIONS_LEVEL << 1)
#define OFFLOAD_SEND_REVERB_LEVEL (OFFLOAD_SEND_REVERB_REFLECTIONS_DELAY << 1)
#define OFFLOAD_SEND_REVERB_DELAY (OFFLOAD_SEND_REVERB_LEVEL << 1)
#define OFFLOAD_SEND_REVERB_DIFFUSION (OFFLOAD_SEND_REVERB_DELAY << 1)
#define OFFLOAD_SEND_REVERB_DENSITY (OFFLOAD_SEND_REVERB_DIFFUSION << 1)

struct ParamDelegator {
  public:
    static int updatePalParameters(pal_stream_handle_t *handle, struct bass_boost_params *bassboost,
                                   unsigned flags);

    static int updatePalParameters(pal_stream_handle_t *handle, struct BassBoostParams *bassboost,
                                   unsigned flags);

    // static int updatePalParameters(pal_stream_handle_t *handle,
    //                                 struct pbe_params *pbe,
    //                                 unsigned flags);

    static int updatePalParameters(pal_stream_handle_t *handle,
                                   struct virtualizer_params *virtualizer, unsigned flags);

    static int updatePalParameters(pal_stream_handle_t *handle, struct eq_params *eq,
                                   unsigned flags);

    static int updatePalParameters(pal_stream_handle_t *handle, struct reverb_params *reverb,
                                   unsigned flags);

  private:
    static int sendKvPayload(pal_stream_handle_t *handle, uint32_t tag, pal_key_vector_t *kvp);
    static int sendCustomPayload(pal_stream_handle_t *hanlde, uint32_t tag,
                                 pal_effect_custom_payload_t *data, uint32_t custom_data_sz);
};
} // namespace aidl::qti::effects