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
#include <core-impl/AudioPolicyConfigXmlConverter.h>
#include <core-impl/ChildInterface.h>
#include <core-impl/Config.h>
#include <core-impl/ModuleUsb.h>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/ModulePrimary.h>
#include <qti-audio-core/ModuleStub.h>

#include <cstdlib>
#include <ctime>

using aidl::android::hardware::audio::core::ChildInterface;

using aidl::android::hardware::audio::core::internal::AudioPolicyConfigXmlConverter;
AudioPolicyConfigXmlConverter gAudioPolicyConverter{
        ::android::audio_get_audio_policy_config_file()};

using AospModule = ::aidl::android::hardware::audio::core::Module;
using AospModuleConfig = ::aidl::android::hardware::audio::core::Module::Configuration;
using AospModuleConfigurationPair = std::pair<std::string, std::unique_ptr<AospModuleConfig>>;
using AospModuleConfigs = std::vector<AospModuleConfigurationPair>;
std::unique_ptr<AospModuleConfigs> gModuleConfigs;
std::vector<ChildInterface<AospModule>> gModuleInstances;
std::shared_ptr<::aidl::android::hardware::audio::core::Config> gConfigDefaultAosp;
std::shared_ptr<::qti::audio::core::ModulePrimary> gModuleDefaultQti;

namespace {

ChildInterface<AospModule> createModule(const std::string &name,
                                        std::unique_ptr<AospModuleConfig> &&config) {
    LOG(DEBUG) << __func__ << "  " << name;
    ChildInterface<AospModule> result;
    {
        auto moduleType = AospModule::typeFromString(name);
        if (!moduleType.has_value()) {
            LOG(ERROR) << __func__ << ": module type \"" << name << "\" is not supported";
            return result;
        }
        auto module = AospModule::createInstance(*moduleType, std::move(config));
        if (module == nullptr) return result;
        result = std::move(module);
    }
    const std::string moduleName =
            std::string().append(AospModule::descriptor).append("/").append(name);
    AIBinder_setMinSchedulerPolicy(result.getBinder(), SCHED_NORMAL, ANDROID_PRIORITY_AUDIO);
    binder_status_t status = AServiceManager_addService(result.getBinder(), moduleName.c_str());
    if (status != STATUS_OK) {
        LOG(ERROR) << __func__ << ": failed to register service for \"" << moduleName << "\"";
        return ChildInterface<AospModule>();
    }
    return result;
};

} // namespace

auto registerBinderAsService = [](auto &&binder, const std::string &serviceName) {
    AIBinder_setMinSchedulerPolicy(binder.get(), SCHED_NORMAL, ANDROID_PRIORITY_AUDIO);
    binder_exception_t status = AServiceManager_addService(binder.get(), serviceName.c_str());
    if (status != EX_NONE) {
        LOG(ERROR) << __func__ << " failed to register " << serviceName << " ret:" << status;
    }
};

void registerIModuleDefaultQti() {
    gModuleDefaultQti = ndk::SharedRefBase::make<::qti::audio::core::ModulePrimary>();
    const std::string kServiceName =
            std::string(gModuleDefaultQti->descriptor).append("/").append("default");
    registerBinderAsService(gModuleDefaultQti->asBinder(), kServiceName);
}

extern "C" __attribute__((visibility("default"))) int32_t registerServices() {
    gConfigDefaultAosp = ndk::SharedRefBase::make<::aidl::android::hardware::audio::core::Config>(
            gAudioPolicyConverter);
    const std::string configIntfName =
            std::string().append(gConfigDefaultAosp->descriptor).append("/default");
    binder_status_t status = AServiceManager_addService(gConfigDefaultAosp->asBinder().get(),
                                                        configIntfName.c_str());
    if (status != STATUS_OK) {
        LOG(ERROR) << "failed to register service for \"" << configIntfName << "\"";
    }

    gModuleConfigs = gAudioPolicyConverter.releaseModuleConfigs();

    registerIModuleDefaultQti();

    // get the modules listed in audio_policy_configuration and register the
    // interfaces except primary
    for (AospModuleConfigurationPair &configPair : *gModuleConfigs) {
        std::string name = configPair.first;
        if (auto instance = createModule(name, std::move(configPair.second)); instance) {
            gModuleInstances.push_back(std::move(instance));
        }
    }
    return STATUS_OK;
}
