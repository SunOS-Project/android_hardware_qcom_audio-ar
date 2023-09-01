/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_CoreService"

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android/binder_ibinder_platform.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <core-impl/Config.h>
#include <core-impl/ModuleUsb.h>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/ModulePrimary.h>

#include <cstdlib>
#include <ctime>

auto registerBinderAsService = [](auto&& binder,
                                  const std::string& serviceName) {
    AIBinder_setMinSchedulerPolicy(binder.get(), SCHED_NORMAL,
                                   ANDROID_PRIORITY_AUDIO);
    binder_exception_t status =
        AServiceManager_addService(binder.get(), serviceName.c_str());
    if (status != EX_NONE) {
        LOG(ERROR) << __func__ << " failed to register " << serviceName
                   << " ret:" << status;
        // CHECK_EQ(1, 0);
    }
};

std::shared_ptr<::aidl::android::hardware::audio::core::Module>
    gModuleDefaultAosp;
void registerIModuleDefaultAosp() {
    gModuleDefaultAosp =
        ::aidl::android::hardware::audio::core::Module::createInstance(
            ::aidl::android::hardware::audio::core::Module::Type::DEFAULT);
    const std::string kServiceName = std::string(gModuleDefaultAosp->descriptor)
                                         .append("/")
                                         .append("default");
    registerBinderAsService(gModuleDefaultAosp->asBinder(), kServiceName);
}

std::shared_ptr<::aidl::android::hardware::audio::core::Module>
    gModuleRSubmixAosp;
void registerIModuleRSubmixAosp() {
    gModuleRSubmixAosp =
        ::aidl::android::hardware::audio::core::Module::createInstance(
            ::aidl::android::hardware::audio::core::Module::Type::R_SUBMIX);
    const std::string kServiceName = std::string(gModuleRSubmixAosp->descriptor)
                                         .append("/")
                                         .append("r_submix");
    registerBinderAsService(gModuleRSubmixAosp->asBinder(), kServiceName);
}

std::shared_ptr<::aidl::android::hardware::audio::core::Module> gModuleUsbAosp;
void registerIModuleUsbAosp() {
    gModuleUsbAosp =
        ::aidl::android::hardware::audio::core::Module::createInstance(
            ::aidl::android::hardware::audio::core::Module::Type::USB);
    const std::string kServiceName =
        std::string(gModuleUsbAosp->descriptor).append("/").append("usb");
    registerBinderAsService(gModuleUsbAosp->asBinder(), kServiceName);
}

std::shared_ptr<::aidl::android::hardware::audio::core::Config>
    gConfigDefaultAosp;
void registerIConfigAosp() {
    gConfigDefaultAosp = ndk::SharedRefBase::make<
        ::aidl::android::hardware::audio::core::Config>();
    const std::string kServiceName = std::string(gConfigDefaultAosp->descriptor)
                                         .append("/")
                                         .append("default");
    registerBinderAsService(gConfigDefaultAosp->asBinder(), kServiceName);
}

std::shared_ptr<::qti::audio::core::ModulePrimary> gModuleDefaultQti;
void registerIModuleDefaultQti() {
    gModuleDefaultQti =
        ndk::SharedRefBase::make<::qti::audio::core::ModulePrimary>();
    const std::string kServiceName = std::string(gModuleDefaultQti->descriptor)
                                         .append("/")
                                         .append("default");
    registerBinderAsService(gModuleDefaultQti->asBinder(), kServiceName);
}

extern "C" __attribute__((visibility("default"))) int32_t registerServices() {
    const std::string& kDefaultConfigProp{
        "vendor.audio.core_hal_IConfig_default"};
    const auto& kDefaultConfig =
        ::android::base::GetProperty(kDefaultConfigProp, "aosp");
    if (kDefaultConfig == "qti") {
        // Todo QTI Config
    } else {
        ::registerIConfigAosp();
    }

    const std::string& kDefaultModuleProp{
        "vendor.audio.core_hal_IModule_default"};
    const auto& kDefaultModule =
        ::android::base::GetProperty(kDefaultModuleProp, "qti");

    if (kDefaultModule == "aosp") {
        ::registerIModuleDefaultAosp();
    } else {
        ::registerIModuleDefaultQti();
    }

    ::registerIModuleRSubmixAosp();
    ::registerIModuleUsbAosp();
    return STATUS_OK;
}
