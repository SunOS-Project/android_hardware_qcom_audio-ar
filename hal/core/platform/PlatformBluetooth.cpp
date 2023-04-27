
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_PlatformBluetooth"

#include <android-base/logging.h>
#include <platform/PlatformBluetooth.h>

#include "PalApi.h"

namespace qti::audio::core {

PlatformBluetooth::PlatformBluetooth() {
    LOG(VERBOSE) << __func__ << ": Enter";
}

PlatformBluetooth::~PlatformBluetooth() {
    LOG(VERBOSE) << __func__ << ": Enter";
}

int32_t PlatformBluetooth::setHfpEnabled(bool enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::getHfpEnabled(bool& enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::setHfpSampleRate(int32_t sampleRate) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::getHfpSampleRate(int32_t& sampleRate) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::setHfpVolume(float volume) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::getHfpVolume(float& volume) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

int32_t PlatformBluetooth::setBtScoEnabled(bool enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::getBtScoEnabled(bool& enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::setBtScoHeadsetDebugName(
    const std::string& debugName) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::setBtScoNrecEnabled(bool enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::getBtScoNrecEnabled(bool& enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::setBtScoWidebandEnabled(bool enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t PlatformBluetooth::getBtScoWidebandEnabled(bool& enabled) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}
int32_t dump(int32_t fd) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit";
    return status;
}

}  // namespace qti::audio::core
