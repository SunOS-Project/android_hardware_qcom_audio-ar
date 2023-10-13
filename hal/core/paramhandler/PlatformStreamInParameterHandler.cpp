/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <android-base/logging.h>
#include <qti-core/PlatformInterface.h>

#include "PalDefs.h"

namespace qti::audio::core {

int32_t AudioStreamInParameterHandler::setParameters(
        const std::vector<std::string>& keyValuePairs) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit ";
    return status;
}

int32_t AudioStreamInParameterHandler::getParameters(const std::string& keys, std::string& values) {
    int32_t status = 0;
    LOG(VERBOSE) << __func__ << ": Enter";
    LOG(VERBOSE) << __func__ << ": Exit ";
    return status;
}

} // namespace qti::audio::core