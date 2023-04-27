/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_CoreService"

#include <cstdlib>
#include <ctime>

#include <aidlservice/Config.h>
#include <aidlservice/Module.h>

#include <android-base/logging.h>
#include <android/binder_ibinder_platform.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

#include <core-impl/ModuleUsb.h>

using qti::audio::core::Config;
using qti::audio::core::Module;

void registerAospModules() {
     auto createModule =
         [](::aidl::android::hardware::audio::core::Module::Type type,
            const std::string& instance) {
             auto module =
                 ::aidl::android::hardware::audio::core::Module::createInstance(
                     type);
             ndk::SpAIBinder moduleBinder = module->asBinder();
             const std::string moduleName =
                 std::string(
                     ::aidl::android::hardware::audio::core::Module::descriptor)
                     .append("/")
                     .append(instance);
             AIBinder_setMinSchedulerPolicy(moduleBinder.get(), SCHED_NORMAL,
                                            ANDROID_PRIORITY_AUDIO);
             binder_status_t status = AServiceManager_addService(
                 moduleBinder.get(), moduleName.c_str());
             CHECK_EQ(STATUS_OK, status);
             return std::make_pair(module, moduleBinder);
         };

     auto modules = {
         createModule(
             ::aidl::android::hardware::audio::core::Module::Type::R_SUBMIX,
             "r_submix"),
         createModule(::aidl::android::hardware::audio::core::Module::Type::USB,
                      "usb")};
     return;
 }

extern "C" __attribute__((visibility("default"))) binder_status_t
registerService()
{
    // Make the default config service
    auto config = ndk::SharedRefBase::make<Config>();
    const std::string configName = std::string() + Config::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(config->asBinder().get(), configName.c_str());
    CHECK_EQ(STATUS_OK, status);

    // Make modules
    auto createModule = [](Module::Type type, const std::string& instance) {
        auto module = ndk::SharedRefBase::make<Module>(type);
        if (type == Module::Type::DEFAULT) {
            module->init(module); // TODO fix it later
        }
        ndk::SpAIBinder moduleBinder = module->asBinder();
        const std::string moduleName = std::string(Module::descriptor).append("/").append(instance);
        AIBinder_setMinSchedulerPolicy(moduleBinder.get(), SCHED_NORMAL, ANDROID_PRIORITY_AUDIO);
        binder_status_t status = AServiceManager_addService(moduleBinder.get(), moduleName.c_str());
        CHECK_EQ(STATUS_OK, status);
        return std::make_pair(module, moduleBinder);
    };
    auto modules = {createModule(Module::Type::DEFAULT, "default")};

    ::registerAospModules();

    return status;
}
