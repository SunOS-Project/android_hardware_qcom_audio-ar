/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#pragma once

#include <memory>
#include <string>
#include <cutils/properties.h>

// RAII based classes to dlopen/dysym on init and dlclose on dest.

static std::string kBatteryListenerLibrary = std::string("libbatterylistener.so");
static std::string kHfpLibrary = "libhfp_pal.so";
static std::string kFmLibrary = "libfmpal.so";
static std::string kBluetoothIpcLibrary = "btaudio_offload_if.so";
static std::string kKarokeLibrary = "dummy.so"; // TODO

static std::string kBatteryListenerProperty = "vendor.audio.feature.battery_listener.enable";
static std::string kHfpProperty = "vendor.audio.feature.hfp.enable";
static std::string kPerfLockProperty = "vendor.audio.feature.kpi_optimize.enable";
static std::string kBluetoothProperty = "vendor.audio.feature.a2dp_offload.enable";


static bool isExtensionEnabled(std::string property) {
    return property_get_bool(property.c_str(), false);
}

class AudioExtensionBase {
public:
    AudioExtensionBase(std::string library, bool enabled = true);
    ~AudioExtensionBase();

protected:
    void *mHandle = nullptr;
    bool mEnabled;
    std::string mLibraryName;
private:
    void cleanUp();
};

class BatteryListenerExtension : public AudioExtensionBase {
public :
    BatteryListenerExtension() ;
    ~BatteryListenerExtension();
};

class PerfLockExtension : public AudioExtensionBase {
public :
    PerfLockExtension();
    ~PerfLockExtension();

    // function mapping for dlsym
    using AcquirePerfLock = int (*) (int, int, int*, int);
    using ReleasePerfLock = int (*) (int);

    AcquirePerfLock mAcquirePerfLock = nullptr;
    ReleasePerfLock mReleasePerfLock = nullptr;
};

class A2dpExtension : public AudioExtensionBase {
public :
    A2dpExtension();
    ~A2dpExtension();

};

class HfpExtension : public AudioExtensionBase {
public :
    HfpExtension();
    ~HfpExtension();

    using Init = void (*) ();
    using IsActive = bool (*) ();
    //using GetUsecase = audio_usecase_t (*) ();
    using MicMute = int (*) (bool);
    using MicMute2 = void (*) (bool);
    // using HfpSetParameters = void (*) (std::string keyValue);
    // using HfpGetParameters = void (*) (bool);
    Init mInit;
    IsActive mIsActive;
   // GetUsecase mUsecase;
    MicMute mMicMute;
    MicMute2 mMicMute2;
};

class FmExtension : public AudioExtensionBase {
public :
    FmExtension() ;
    ~FmExtension();

};

class KarokeExtension : public AudioExtensionBase {
public :
    KarokeExtension() ;
    ~KarokeExtension();
};

class AudioExtension {
public:
  static AudioExtension& getInstance() {
        static AudioExtension instance;
        return instance;
  }

private:
    AudioExtension() = default;
    std::unique_ptr<BatteryListenerExtension> mBatteryListenerExtension = std::make_unique<BatteryListenerExtension>();
    std::unique_ptr<A2dpExtension> mA2dpExtension = std::make_unique<A2dpExtension>();
    std::unique_ptr<HfpExtension> mHfpExtension = std::make_unique<HfpExtension>();
    std::unique_ptr<FmExtension> mFmExtension = std::make_unique<FmExtension>();
    std::unique_ptr<KarokeExtension> mKarokeExtension = std::make_unique<KarokeExtension>();
    std::unique_ptr<PerfLockExtension> mPerfLockExtension = std::make_unique<PerfLockExtension>();
};