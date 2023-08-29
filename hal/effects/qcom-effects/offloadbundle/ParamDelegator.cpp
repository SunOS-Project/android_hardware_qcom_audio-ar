/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#define LOG_TAG "AHAL_Effect_ParamDelegator"

#include "ParamDelegator.h"

namespace aidl::qti::effects {

#define OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL 19
const int kEqualizerOpenSlToOffloadMap[] = {
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL,   /* Normal Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 1, /* Classical Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 2, /* Dance Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 3, /* Flat Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 4, /* Folk Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 5, /* Heavy Metal Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 6, /* Hip Hop Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 7, /* Jazz Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 8, /* Pop Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 9, /* Rock Preset */
    OFFLOAD_PRESET_START_OFFSET_FOR_OPENSL + 10 /* FX Booster */
};

const int map_reverb_opensl_preset_2_offload_preset
                  [NUM_OSL_REVERB_PRESETS_SUPPORTED][2] = {
    {1, 15},
    {2, 16},
    {3, 17},
    {4, 18},
    {5, 3},
    {6, 20}
};

#define EXIT_IF_NO_MEMORY(payload)                                      \
    {                                                                         \
        if (!payload) {                                                       \
            ALOGE("%s:%d calloc failed", __func__, __LINE__);                  \
            return -ENOMEM;                                                    \
        }                                                                      \
    }

#define EXIT_IF_SET_PARAM_FAILS(ret)                                \
        if (ret) {                                                              \
            ALOGE("%s: pal_stream_set_param failed. ret = %d", __func__, ret);  \
            return ret;                                                         \
        }

int ParamDelegator::sendKvPayload(pal_stream_handle_t *handle,
                            uint32_t tag, pal_key_vector_t *kvp)
{
   int ret = 0;
   pal_param_payload *palPayload;
   effect_pal_payload_t * effectPalPayload = NULL;
   uint8_t *payload = NULL;
   pal_key_vector_t * palKeyVector = NULL;
   uint32_t payloadSize  = sizeof(pal_param_payload) + sizeof(effect_pal_payload_t) +
                        sizeof(pal_key_vector_t) + kvp->num_tkvs * sizeof(pal_key_value_pair_t);

   payload = (uint8_t *) calloc (1, payloadSize);

   EXIT_IF_NO_MEMORY(payload);

   palPayload = (pal_param_payload *) payload;
   palPayload->payload_size = sizeof(effect_pal_payload_t) +
                                  sizeof(pal_key_vector_t) +
                                  kvp->num_tkvs * sizeof(pal_key_value_pair_t);

    effectPalPayload = (effect_pal_payload_t *)(payload + sizeof(pal_param_payload));
    effectPalPayload->isTKV = PARAM_TKV;
    effectPalPayload->tag = tag;
    effectPalPayload->payloadSize = sizeof(pal_key_vector_t) +
                                  kvp->num_tkvs * sizeof(pal_key_value_pair_t);
    palKeyVector = (pal_key_vector_t *)(payload +
                                              sizeof(pal_param_payload) +
                                              sizeof(effect_pal_payload_t));

    palKeyVector->num_tkvs = kvp->num_tkvs;
    memcpy(palKeyVector->kvp, kvp->kvp,
                        (kvp->num_tkvs * sizeof(pal_key_value_pair_t)));
    ret = pal_stream_set_param(handle, PAL_PARAM_ID_UIEFFECT,
                               palPayload);
    free(palPayload);
done:
    return ret;
}

int ParamDelegator::sendCustomPayload(pal_stream_handle_t *handle,
                              uint32_t tag, pal_effect_custom_payload_t *data,
                              uint32_t customDataSize)
{
    int ret = 0;
    pal_param_payload *palPayload;
    effect_pal_payload_t *effectPalPayload = NULL;
    uint8_t *payload = NULL;
    pal_effect_custom_payload_t *customPayload = NULL;
    uint32_t payloadSize = 0;
    payloadSize = sizeof(pal_param_payload) + sizeof(effect_pal_payload_t) +
                   sizeof(pal_effect_custom_payload_t) + customDataSize;

    payload = (uint8_t *) calloc (1, payloadSize);
    EXIT_IF_NO_MEMORY(payload);

    palPayload = (pal_param_payload *) payload;
    palPayload->payload_size = sizeof(effect_pal_payload_t) +
                                  sizeof(pal_effect_custom_payload_t) +
                                  customDataSize;

    effectPalPayload = (effect_pal_payload_t *)(payload + sizeof(pal_param_payload));
    effectPalPayload->isTKV = PARAM_NONTKV;
    effectPalPayload->tag = tag;
    effectPalPayload->payloadSize = sizeof(pal_effect_custom_payload_t) +
                                  customDataSize;
    customPayload = (pal_effect_custom_payload_t *)(payload +
                                                   sizeof(pal_param_payload) +
                                                   sizeof(effect_pal_payload_t));

    customPayload->paramId = data->paramId;
    memcpy(customPayload->data, data->data, customDataSize);
    ret = pal_stream_set_param(handle, PAL_PARAM_ID_UIEFFECT,
                              palPayload);
    free(palPayload);
done:
    return ret;
}

int ParamDelegator::updatePalParameters(pal_stream_handle_t *handle,
                                  struct bass_boost_params *bassboost,
                                 unsigned flags)
{
    int ret = 0;
    pal_effect_custom_payload_t *customPayload = NULL;

    if (flags & OFFLOAD_SEND_BASSBOOST_ENABLE_FLAG) {
        //enableBassBoost(handle, bassboost);
        uint32_t num_kvs = 1;
        pal_key_vector_t *palKeyVector = (pal_key_vector_t *) calloc(1, sizeof(pal_key_vector_t) +
                                            num_kvs * sizeof(pal_key_value_pair_t));

        EXIT_IF_NO_MEMORY(palKeyVector);

        palKeyVector->num_tkvs = num_kvs;
        palKeyVector->kvp[0].key= BASS_BOOST_SWITCH;
        palKeyVector->kvp[0].value = bassboost->enable_flag;

        ret = sendKvPayload(handle, TAG_STREAM_BASS_BOOST, palKeyVector);
        free(palKeyVector);
        EXIT_IF_SET_PARAM_FAILS(ret);
    }

    if (flags & OFFLOAD_SEND_BASSBOOST_STRENGTH) {
        //sendStrengthBassBoost(handle, bassboost);
        uint32_t customDataSize = BASS_BOOST_STRENGTH_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);

        customPayload->paramId = PARAM_ID_BASS_BOOST_STRENGTH;
        customPayload->data[0] = bassboost->strength;
        ret = sendCustomPayload(handle, TAG_STREAM_BASS_BOOST, customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }

    if (flags & OFFLOAD_SEND_BASSBOOST_MODE) {
        //sendModeBassBoost(handle, bassboost);
        uint32_t customDataSize = BASS_BOOST_STRENGTH_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_BASS_BOOST_MODE;
        customPayload->data[0] = bassboost->strength;

        ret = sendCustomPayload(handle, TAG_STREAM_BASS_BOOST,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }

done:
    return ret;
}

int ParamDelegator::updatePalParameters(pal_stream_handle_t *handle,
                                    struct virtualizer_params *virtualizer,
                                   unsigned flags)
{
    int ret = 0;
    pal_effect_custom_payload_t *customPayload = NULL;

    ALOGV("%s: flags 0x%x", __func__, flags);
    if (flags & OFFLOAD_SEND_VIRTUALIZER_ENABLE_FLAG) {
        // enableVirtualizer(handle, virtualizer);
        uint32_t num_kvs = 1;
        pal_key_vector_t *palKeyVector = NULL;
        palKeyVector = (pal_key_vector_t *) calloc (1, sizeof(pal_key_vector_t) +
                                         num_kvs * sizeof(pal_key_value_pair_t));
        EXIT_IF_NO_MEMORY(palKeyVector);

        palKeyVector->num_tkvs = num_kvs;
        palKeyVector->kvp[0].key= VIRTUALIZER_SWITCH;
        palKeyVector->kvp[0].value =  virtualizer->enable_flag;

        ret = sendKvPayload(handle, TAG_STREAM_VIRTUALIZER, palKeyVector);
        free(palKeyVector);
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_VIRTUALIZER_STRENGTH) {
        // sendStrengthVirtualizer(handle, virtualizer);
        uint32_t customDataSize = VIRTUALIZER_STRENGTH_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                          sizeof(pal_effect_custom_payload_t) +
                                          customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_VIRTUALIZER_STRENGTH;
        customPayload->data[0] = virtualizer->strength;

        ret = sendCustomPayload(handle, TAG_STREAM_VIRTUALIZER,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_VIRTUALIZER_OUT_TYPE) {
        // sendVirtualizerType(handle, virtualizer);
        uint32_t customDataSize = VIRTUALIZER_STRENGTH_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                         sizeof(pal_effect_custom_payload_t) +
                                         customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_VIRTUALIZER_OUT_TYPE;
        customPayload->data[0] = virtualizer->out_type;
        ret = sendCustomPayload(handle, TAG_STREAM_VIRTUALIZER,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;

        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_VIRTUALIZER_GAIN_ADJUST) {
        // sendGainVirtualizere(handle, virtualizer);
        uint32_t customDataSize = VIRTUALIZER_STRENGTH_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                                sizeof(pal_effect_custom_payload_t) +
                                                customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_VIRTUALIZER_GAIN_ADJUST;
        customPayload->data[0] = virtualizer->gain_adjust;

        ret = sendCustomPayload(handle, TAG_STREAM_VIRTUALIZER,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
done:
    return ret;
}

int ParamDelegator::updatePalParameters(pal_stream_handle_t *handle, struct eq_params *eq,
                          unsigned flags)
{
    uint32_t i = 0, index = 0;
    int ret = 0;
    pal_effect_custom_payload_t *customPayload = NULL;

    if (!handle) {
        ALOGW("%s: stream is not opened, invalid pal handle", __func__);
        return -EINVAL;
    }
    ALOGV("%s: flags 0x%x", __func__, flags);
    if ((eq->config.preset_id < -1) ||
            ((flags & OFFLOAD_SEND_EQ_PRESET) && (eq->config.preset_id == -1))) {
        ALOGV("No Valid preset to set");
        return 0;
    }

    if (flags & OFFLOAD_SEND_EQ_ENABLE_FLAG) {
        uint32_t num_kvs = 1;
        pal_key_vector_t *palKeyVector = NULL;
        palKeyVector = (pal_key_vector_t *) calloc (1, sizeof(pal_key_vector_t) +
                                           num_kvs * sizeof(pal_key_value_pair_t));
        EXIT_IF_NO_MEMORY(palKeyVector);

        palKeyVector->num_tkvs = num_kvs;
        palKeyVector->kvp[0].key= EQUALIZER_SWITCH;
        palKeyVector->kvp[0].value = eq->enable_flag;
        ret = sendKvPayload(handle, TAG_STREAM_EQUALIZER, palKeyVector);
        free(palKeyVector);
        EXIT_IF_SET_PARAM_FAILS(ret);
    }

    if (flags & OFFLOAD_SEND_EQ_PRESET) {
        uint32_t customDataSize = EQ_CONFIG_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_EQ_CONFIG;
        customPayload->data[0] = eq->config.eq_pregain;
        customPayload->data[1] =
            kEqualizerOpenSlToOffloadMap[eq->config.preset_id];
        customPayload->data[2] = 0;    // num_of_band must be 0 for preset

        ret = sendCustomPayload(handle, TAG_STREAM_EQUALIZER,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }

    if (flags & OFFLOAD_SEND_EQ_BANDS_LEVEL) {
        uint32_t customDataSize = (EQ_CONFIG_PARAM_LEN +
         (eq->config.num_bands * EQ_CONFIG_PER_BAND_PARAM_LEN)) * sizeof(uint32_t);

        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_EQ_CONFIG;
        index = 0;
        customPayload->data[index++] = eq->config.eq_pregain;
        customPayload->data[index++] = CUSTOM_OPENSL_PRESET;
        customPayload->data[index++] = eq->config.num_bands;
        for (i = 0; i < eq->config.num_bands; i++) {
            customPayload->data[index++] = eq->per_band_cfg[i].filter_type;
            customPayload->data[index++] = eq->per_band_cfg[i].freq_millihertz;
            customPayload->data[index++] = eq->per_band_cfg[i].gain_millibels;
            customPayload->data[index++] = eq->per_band_cfg[i].quality_factor;
            customPayload->data[index++] = eq->per_band_cfg[i].band_idx;
        }
        ret = sendCustomPayload(handle, TAG_STREAM_EQUALIZER,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
done:
    return ret;
}


int ParamDelegator::updatePalParameters(pal_stream_handle_t *handle,
                               struct reverb_params *reverb,
                              unsigned flags)
{
    int ret = 0;
    pal_effect_custom_payload_t *customPayload = NULL;

    ALOGV("%s: flags 0x%x", __func__, flags);

    if (flags & OFFLOAD_SEND_REVERB_ENABLE_FLAG) {
        uint32_t num_kvs = 1;
        pal_key_vector_t *palKeyVector = NULL;
        palKeyVector = (pal_key_vector_t *) calloc (1, sizeof(pal_key_vector_t) +
                                         num_kvs * sizeof(pal_key_value_pair_t));
        EXIT_IF_NO_MEMORY(palKeyVector);
        palKeyVector->num_tkvs = num_kvs;
        palKeyVector->kvp[0].key= REVERB_SWITCH;
        palKeyVector->kvp[0].value = reverb->enable_flag;
        ret = sendKvPayload(handle, TAG_STREAM_REVERB, palKeyVector);
        free(palKeyVector);
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_MODE) {
        uint32_t customDataSize = REVERB_MODE_PARAM_LEN * sizeof(uint32_t);

        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_MODE;
        customPayload->data[0] = reverb->mode;;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_PRESET) {
        uint32_t customDataSize = REVERB_PRESET_PARAM_LEN * sizeof(uint32_t);

        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_PRESET;
        customPayload->data[0] = reverb->preset;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_WET_MIX) {
        // param_id + actual payload
        uint32_t customDataSize = REVERB_WET_MIX_PARAM_LEN * sizeof(uint32_t);

        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                            sizeof(pal_effect_custom_payload_t) +
                                            customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);

        customPayload->paramId = PARAM_ID_REVERB_WET_MIX;
        customPayload->data[0] = reverb->wet_mix;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_GAIN_ADJUST) {
        uint32_t customDataSize = REVERB_GAIN_ADJUST_PARAM_LEN * sizeof(uint32_t);

        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                            sizeof(pal_effect_custom_payload_t) +
                                            customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);

        customPayload->paramId = PARAM_ID_REVERB_GAIN_ADJUST;
        customPayload->data[0] = reverb->gain_adjust;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_ROOM_LEVEL) {
        uint32_t customDataSize = REVERB_ROOM_LEVEL_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                            sizeof(pal_effect_custom_payload_t) +
                                            customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_ROOM_LEVEL;
        customPayload->data[0] = reverb->room_level;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_ROOM_HF_LEVEL) {
        uint32_t customDataSize = REVERB_ROOM_HF_LEVEL_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                            sizeof(pal_effect_custom_payload_t) +
                                            customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_ROOM_HF_LEVEL;
        customPayload->data[0] = reverb->room_hf_level;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_DECAY_TIME) {
        uint32_t customDataSize = REVERB_DECAY_TIME_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                              sizeof(pal_effect_custom_payload_t) +
                                              customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_DECAY_TIME;
        customPayload->data[0] = reverb->decay_time;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_DECAY_HF_RATIO) {
        uint32_t customDataSize = REVERB_DECAY_HF_RATIO_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                              sizeof(pal_effect_custom_payload_t) +
                                              customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_DECAY_HF_RATIO;
        customPayload->data[0] = reverb->decay_hf_ratio;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_REFLECTIONS_LEVEL) {
        uint32_t customDataSize = REVERB_REFLECTIONS_LEVEL_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                            sizeof(pal_effect_custom_payload_t) +
                                            customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);

        customPayload->paramId = PARAM_ID_REVERB_REFLECTIONS_LEVEL;
        customPayload->data[0] = reverb->reflections_level;
        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_REFLECTIONS_DELAY) {
        uint32_t customDataSize = REVERB_REFLECTIONS_DELAY_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                             sizeof(pal_effect_custom_payload_t) +
                                             customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);

        customPayload->paramId = PARAM_ID_REVERB_REFLECTIONS_DELAY;
        customPayload->data[0] = reverb->reflections_delay;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_LEVEL) {
        uint32_t customDataSize = REVERB_LEVEL_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                              sizeof(pal_effect_custom_payload_t) +
                                              customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_LEVEL;
        customPayload->data[0] = reverb->level;
        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);

    }
    if (flags & OFFLOAD_SEND_REVERB_DELAY) {
        uint32_t customDataSize = REVERB_DELAY_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                             sizeof(pal_effect_custom_payload_t) +
                                             customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_DELAY;
        customPayload->data[0] = reverb->delay;
        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_DIFFUSION) {
        uint32_t customDataSize = REVERB_DIFFUSION_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_DIFFUSION;
        customPayload->data[0] = reverb->diffusion;

        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
    if (flags & OFFLOAD_SEND_REVERB_DENSITY) {
        uint32_t customDataSize = REVERB_DENSITY_PARAM_LEN * sizeof(uint32_t);
        customPayload = (pal_effect_custom_payload_t *) calloc (1,
                                           sizeof(pal_effect_custom_payload_t) +
                                           customDataSize);
        EXIT_IF_NO_MEMORY(customPayload);
        customPayload->paramId = PARAM_ID_REVERB_DENSITY;
        customPayload->data[0] = reverb->density;
        ret = sendCustomPayload(handle, TAG_STREAM_REVERB,
                              customPayload, customDataSize);
        free(customPayload);
        customPayload = NULL;
        EXIT_IF_SET_PARAM_FAILS(ret);
    }
done:
    return ret;
}

} // namespace aidl::qti::effects