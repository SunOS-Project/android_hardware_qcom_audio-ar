/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <audio_policy_engine_configuration.h>
#include <audio_policy_engine_configuration_enums.h>
#include <utils/Errors.h>
#include <utils/XmlConverter.h>

#include <aidl/android/media/audio/common/AudioFlag.h>
#include <aidl/android/media/audio/common/AudioHalEngineConfig.h>
#include <aidl/android/media/audio/common/AudioProductStrategyType.h>
// #include <aidl/android/media/audio/common/AudioAttributes.h>
// #include <aidl/android/media/audio/common/AudioHalAttributesGroup.h>
// #include <aidl/android/media/audio/common/AudioHalCapCriterion.h>
// #include <aidl/android/media/audio/common/AudioHalCapCriterionType.h>
// #include <aidl/android/media/audio/common/AudioHalVolumeCurve.h>
// #include <aidl/android/media/audio/common/AudioHalProductStrategy.h>
// #include <aidl/android/media/audio/common/AudioHalAttributesGroup.h>

#include <string>
#include <unordered_map>

namespace qti::audio::core {

namespace xsd = ::audio::policy::engine::configuration;

class EngineConfigXmlConverter {
   public:
    explicit EngineConfigXmlConverter(const std::string& configFilePath)
        : mConverter(configFilePath, &xsd::read) {
        if (mConverter.getXsdcConfig()) {
            init();
        }
    }

    std::string getError() const { return mConverter.getError(); }
    ::android::status_t getStatus() const { return mConverter.getStatus(); }

    ::aidl::android::media::audio::common::AudioHalEngineConfig&
    getAidlEngineConfig();

   private:
    const std::optional<xsd::Configuration>& getXsdcConfig() {
        return mConverter.getXsdcConfig();
    }
    void init();
    void initProductStrategyMap();
    ::aidl::android::media::audio::common::AudioAttributes
    convertAudioAttributesToAidl(
        const xsd::AttributesType& xsdcAudioAttributes);
    ::aidl::android::media::audio::common::AudioHalAttributesGroup
    convertAttributesGroupToAidl(
        const xsd::AttributesGroup& xsdcAttributesGroup);
    ::aidl::android::media::audio::common::AudioHalCapCriterion
    convertCapCriterionToAidl(const xsd::CriterionType& xsdcCriterion);
    ::aidl::android::media::audio::common::AudioHalCapCriterionType
    convertCapCriterionTypeToAidl(
        const xsd::CriterionTypeType& xsdcCriterionType);
    std::string convertCriterionTypeValueToAidl(
        const xsd::ValueType& xsdcCriterionTypeValue);
    ::aidl::android::media::audio::common::AudioHalVolumeCurve::CurvePoint
    convertCurvePointToAidl(const std::string& xsdcCurvePoint);
    ::aidl::android::media::audio::common::AudioHalProductStrategy
    convertProductStrategyToAidl(
        const xsd::ProductStrategies::ProductStrategy& xsdcProductStrategy);
    int convertProductStrategyNameToAidl(
        const std::string& xsdcProductStrategyName);
    ::aidl::android::media::audio::common::AudioHalVolumeCurve
    convertVolumeCurveToAidl(const xsd::Volume& xsdcVolumeCurve);
    ::aidl::android::media::audio::common::AudioHalVolumeGroup
    convertVolumeGroupToAidl(
        const xsd::VolumeGroupsType::VolumeGroup& xsdcVolumeGroup);

    ::aidl::android::media::audio::common::AudioHalEngineConfig
        mAidlEngineConfig;
    XmlConverter<xsd::Configuration> mConverter;
    std::unordered_map<std::string, xsd::AttributesRefType>
        mAttributesReferenceMap;
    std::unordered_map<std::string, xsd::VolumeRef> mVolumesReferenceMap;
    std::unordered_map<std::string, int> mProductStrategyMap;
    int mNextVendorStrategy = ::aidl::android::media::audio::common::
        AudioHalProductStrategy::VENDOR_STRATEGY_ID_START;
    std::optional<int> mDefaultProductStrategyId;
};

}  // namespace qti::audio::core
