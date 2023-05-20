/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_NDEBUG 0
#define LOG_TAG "AHAL_CoreService"

#include <agm_server_wrapper.h>
#include <aidlservice/Config.h>
#include <aidlservice/Module.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android/binder_ibinder_platform.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <core-impl/Config.h>
#include <core-impl/ModuleUsb.h>
#include <pal_server_wrapper.h>
#include <vendor/qti/hardware/AGMIPC/1.0/IAGM.h>
#include <vendor/qti/hardware/pal/1.0/IPAL.h>

#include <cstdlib>
#include <ctime>

void registerHIDLPALService() {
    /* Platform Adaption Service*/
    using vendor::qti::hardware::pal::V1_0::IPAL;
    using vendor::qti::hardware::pal::V1_0::implementation::PAL;
    android::sp<IPAL> service = new PAL();
    if (android::OK != service->registerAsService()) {
        LOG(ERROR) << __func__ << " LINE:" << __LINE__
                   << " unable to register PAL as HIDL service";
    } else {
        LOG(INFO) << __func__ << " LINE:" << __LINE__
                  << " registered PAL as HIDL service successfully";
    }
    return;
}

void registerHIDLAGMService() {
    using vendor::qti::hardware::AGMIPC::V1_0::IAGM;
    using vendor::qti::hardware::AGMIPC::V1_0::implementation::AGM;
    /* Audio Graph Manager Service*/
    android::sp<IAGM> service = new AGM();
    AGM* temp = static_cast<AGM*>(service.get());
    if (temp->is_agm_initialized()) {
        if (android::OK != service->registerAsService()) {
            LOG(ERROR) << __func__ << " LINE:" << __LINE__
                       << " unable to register AGM as HIDL service";
        }
    }
    LOG(INFO) << __func__ << " LINE:" << __LINE__
              << " registered AGM as HIDL service successfully";
}

auto registerAospModule =
    [](::aidl::android::hardware::audio::core::Module::Type type,
       const std::string& instance) {
        auto module =
            ::aidl::android::hardware::audio::core::Module::createInstance(
                type);
        ndk::SpAIBinder moduleBinder = module->asBinder();
        const std::string moduleName =
            std::string(
                ::aidl::android::hardware::audio::core::IModule::descriptor)
                .append("/")
                .append(instance);
        AIBinder_setMinSchedulerPolicy(moduleBinder.get(), SCHED_NORMAL,
                                       ANDROID_PRIORITY_AUDIO);
        binder_exception_t status =
            AServiceManager_addService(moduleBinder.get(), moduleName.c_str());
        CHECK_EQ(EX_NONE, status);
        return std::make_pair(module, moduleBinder);
    };

void registerAospIConfig() {
    auto config = ndk::SharedRefBase::make<
        ::aidl::android::hardware::audio::core::Config>();
    const std::string& kInterface =
        std::string(::aidl::android::hardware::audio::core::IConfig::descriptor)
            .append("/")
            .append("default");
    ndk::SpAIBinder configBinder = config->asBinder();
    AIBinder_setMinSchedulerPolicy(configBinder.get(), SCHED_NORMAL,
                                   ANDROID_PRIORITY_AUDIO);
    binder_exception_t status =
        AServiceManager_addService(configBinder.get(), kInterface.c_str());
    if (status != EX_NONE) {
        LOG(ERROR) << __func__ << " LINE:" << __LINE__
                   << " failed to register AOSP " << kInterface
                   << std::to_string(status);
    }
    LOG(INFO) << __func__ << " LINE:" << __LINE__
              << " registered successfully AOSP " << kInterface;
    CHECK_EQ(EX_NONE,status);
}

void registerQtiIModuleDefault() {
    auto module = ndk::SharedRefBase::make<::qti::audio::core::Module>(
        qti::audio::core::Module::Type::DEFAULT);
    module->init(module);
    ndk::SpAIBinder moduleBinder = module->asBinder();
    const std::string kInterface =
        std::string(qti::audio::core::Module::descriptor)
            .append("/")
            .append("default");
    AIBinder_setMinSchedulerPolicy(moduleBinder.get(), SCHED_NORMAL,
                                   ANDROID_PRIORITY_AUDIO);
    binder_exception_t status =
        AServiceManager_addService(moduleBinder.get(), kInterface.c_str());
    if (status != EX_NONE) {
        LOG(ERROR) << __func__ << " LINE:" << __LINE__
                   << " failed to register QTI " << kInterface;
    }
    LOG(INFO) << __func__ << " LINE:" << __LINE__
              << " registered successfully QTI " << kInterface;
    CHECK_EQ(EX_NONE,status);
}

extern "C" __attribute__((visibility("default"))) binder_status_t
registerServices() {
    ::registerHIDLAGMService();
    ::registerHIDLPALService();

    const std::string& kDefaultConfigProp{
        "vendor.audio.core_hal_IConfig_default"};
    const auto& kDefaultConfig =
        ::android::base::GetProperty(kDefaultConfigProp, "aosp");
    if (kDefaultConfig == "qti") {
        // Todo QTI Config
    } else {
        ::registerAospIConfig();
    }

    const std::string& kDefaultModuleProp{
        "vendor.audio.core_hal_IModule_default"};
    const auto& kDefaultModule =
        ::android::base::GetProperty(kDefaultModuleProp, "aosp");
    if (kDefaultModule == "qti") {
        ::registerQtiIModuleDefault();
    } else {
        ::registerAospModule(
            ::aidl::android::hardware::audio::core::Module::Type::DEFAULT,
            "default");
    }

    ::registerAospModule(
        ::aidl::android::hardware::audio::core::Module::Type::R_SUBMIX,
        "r_submix");
    ::registerAospModule(
        ::aidl::android::hardware::audio::core::Module::Type::USB, "usb");
    return STATUS_OK;
}
