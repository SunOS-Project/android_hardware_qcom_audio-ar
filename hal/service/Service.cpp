/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_NDEBUG 0
#define LOG_TAG "AHAL_QService"

#include <cstdlib>
#include <ctime>
#include <dlfcn.h>

#include <algorithm>

#include<vector>
#include<string>
#include <chrono>

#include <log/log.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <android/binder_ibinder_platform.h>
#include <binder/ProcessState.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include "ConfigManager.h"

// TODO Remove it
static bool gFatalIfMandatoryInterfaceMissing = false;

static bool registerServiceImplementation(const Interface & interface) {
    auto libraryName = interface.libraryName;
    auto interfaceMethod = interface.method;
    void* handle = dlopen(libraryName.c_str(), RTLD_LAZY);
    if (handle == nullptr) {
        const char* error = dlerror();
        ALOGE("Failed to dlopen %s: %s", libraryName.c_str(), error != nullptr ? error : "unknown error");
        return false;
    }
    auto instantiate = reinterpret_cast<binder_status_t (*)()>
                                (dlsym(handle, interfaceMethod.c_str()));
    if (instantiate == nullptr) {
        const char* error = dlerror();
        ALOGE("Factory function %s not found in libName %s: %s", interfaceMethod.c_str(), libraryName.c_str(),
              error != nullptr ? error : "unknown error");
        dlclose(handle);
        return false;
    }
    return (instantiate() == STATUS_OK);
}

void registerInterfaces(const Interfaces &interfaces) {
     for (const auto& interface : interfaces) {
            if (registerServiceImplementation(interface)) {
                ALOGI("successfully registered %s", interface.toString().c_str());
            } else if(interface.mandatory) { //TODO
                LOG_ALWAYS_FATAL_IF(gFatalIfMandatoryInterfaceMissing, "failed to register %s ", interface.toString().c_str());
            } else {
                ALOGW("failed to register optional %s ", interface.toString().c_str());
            }
    }
}

bool registerFromConfigs() {
    auto interfaces = parseInterfaces();
    registerInterfaces(interfaces);
    return !interfaces.empty();
}

void registerDefaultInterfaces() {
    Interfaces defaultInterfaces = {
        {
            .name = "audiohal",
            .libraryName = "libaudiocorehal.qti.so",
            .method = "registerService",
            .mandatory = true
        },
        {
            .name = "audioeffecthal",
            .libraryName = "libaudioeffecthal.qti.so",
            .method = "registerService",
            .mandatory = true
        },
        {
            .name = "sthal",
            .libraryName = "android.hardware.soundtrigger.audio-impl.so",
            .method = "createISoundTriggerFactory",
            .mandatory = true
        },
        {
            .name = "bthal",
            .libraryName = "android.hardware.bluetooth.audio-impl.so",
            .method = "createIBluetoothAudioProviderFactory",
            .mandatory = false
        },
    };

    registerInterfaces(defaultInterfaces);
}

void registerAvailableInterfaces() {
    if (!registerFromConfigs()) {
        ALOGI("registerDefaultInterfaces");
        registerDefaultInterfaces();
    }
}

void setLogSeverity() {
    const std::string kDefaultAudioHALLogLevel{"vendor.audio.hal.loglevel"};
    auto logLevel =
        ::android::base::GetIntProperty<int8_t>(kDefaultAudioHALLogLevel, 0);
    logLevel = 0;

    // system/libbase/include/android-base/logging.h
    android::base::SetMinimumLogSeverity(
        static_cast<::android::base::LogSeverity>(logLevel));
}

int main() {
    auto startTime = std::chrono::steady_clock::now();
    // Random values are used in the implementation.
    std::srand(std::time(nullptr));
    setLogSeverity();

    ABinderProcess_setThreadPoolMaxThreadCount(16);
    ABinderProcess_startThreadPool();

    registerAvailableInterfaces();
    auto endTime = std::chrono::steady_clock::now();
    float timeTaken = std::chrono::duration_cast<std::chrono::duration<float>>(
                         endTime - startTime).count();
    ALOGI("registration took %.2f seconds ", timeTaken);
    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;
}
