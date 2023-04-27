/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <dlfcn.h>
#include <log/log.h>
#include <extensions/AudioExtension.h>

AudioExtensionBase::AudioExtensionBase(std::string library, bool enabled) :
    mLibraryName(library),
    mEnabled (enabled)
{
    ALOGI("opening %s: enabled %s", mLibraryName.c_str(), enabled);
    if (mEnabled) {
        mHandle = dlopen(mLibraryName.c_str(), RTLD_LAZY);
        if (mHandle == nullptr) {
            const char* error = dlerror();
            ALOGE("Failed to dlopen %s: %s", mLibraryName.c_str(), error != nullptr ? error : "unknown error");
        }
    }
}

AudioExtensionBase:: ~AudioExtensionBase() {
    cleanUp();
}

void AudioExtensionBase::cleanUp() {
    if (mHandle != nullptr) {
        dlclose(mHandle);
    }
}

BatteryListenerExtension::BatteryListenerExtension() :
AudioExtensionBase (kBatteryListenerLibrary, isExtensionEnabled(kBatteryListenerProperty)) {
    ALOGI("Enter: %s", __func__);
    if (mHandle != nullptr) {
        dlsym(mHandle, "battery_properties_listener_init");
        dlsym(mHandle, "battery_properties_listener_deinit");
        dlsym(mHandle, "battery_properties_is_charging");
    }
}

A2dpExtension::A2dpExtension() :
AudioExtensionBase (kBatteryListenerLibrary, isExtensionEnabled(kBluetoothProperty)) {
    ALOGI("Enter: %s", __func__);
    if (mHandle != nullptr) {
        dlsym(mHandle, "bt_audio_pre_init");
        dlsym(mHandle, "register_reconfig_cb");
    }
}

HfpExtension::HfpExtension() :
AudioExtensionBase (kBatteryListenerLibrary, isExtensionEnabled(kHfpProperty)) {
    ALOGI("Enter: %s", __func__);
    if (mHandle != nullptr) {
        dlsym(mHandle, "hfp_init");
        dlsym(mHandle, "hfp_is_active");
        dlsym(mHandle, "hfp_get_usecase");
        dlsym(mHandle, "hfp_set_mic_mute");
        dlsym(mHandle, "hfp_set_mic_mute2");
        dlsym(mHandle, "hfp_set_parameters");
    }
}

FmExtension::FmExtension() :
AudioExtensionBase (kFmLibrary) {
    ALOGI("Enter: %s", __func__);
    if (mHandle != nullptr) {
        dlsym(mHandle, "fm_set_parameters");
        dlsym(mHandle, "fm_get_parameters");
    }
}

PerfLockExtension::PerfLockExtension() :
AudioExtensionBase (kHfpLibrary, isExtensionEnabled(kHfpProperty)) {
    ALOGI("Enter: %s", __func__);
    if (mHandle != nullptr) {
        mAcquirePerfLock = reinterpret_cast<AcquirePerfLock> (dlsym(mHandle, "perf_lock_acq"));
        mReleasePerfLock = reinterpret_cast<ReleasePerfLock> (dlsym(mHandle, "perf_lock_rel"));
    }
}

KarokeExtension::KarokeExtension() :
AudioExtensionBase (kKarokeLibrary) {
    ALOGI("Enter: %s", __func__);
    if (mHandle != nullptr) {

    }
}
