/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "EffectMain"

#include "effectFactory-impl/EffectFactory.h"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <system/audio_config.h>

/** Default name of effect configuration file. */
static const char* kDefaultConfigName = "audio_effects_config.xml";

extern "C" __attribute__((visibility("default"))) binder_status_t
registerService() {

    LOG(DEBUG) << __func__ ;

    auto configFile = android::audio_find_readable_configuration_file(kDefaultConfigName);
    if (configFile == "") {
        LOG(ERROR) << __func__ << ": config file " << kDefaultConfigName << " not found!";
        return EXIT_FAILURE;
    }
    LOG(INFO) << __func__ << ": start factory with configFile:" << configFile;
    auto effectFactory =
            ndk::SharedRefBase::make<aidl::android::hardware::audio::effect::Factory>(configFile);

    std::string serviceName = std::string() + effectFactory->descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(effectFactory->asBinder().get(), serviceName.c_str());
    LOG(INFO) << __func__ << "status:: " << status;
    return status;
}
