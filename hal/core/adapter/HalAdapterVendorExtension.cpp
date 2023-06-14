/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_NDEBUG 0
#define LOG_TAG "HalAdapterVendorExtension"

#include <aidl/android/media/audio/common/Boolean.h>
#include <aidl/android/media/audio/common/Int.h>
#include <android-base/logging.h>
#include <qti-audio-core/HalAdapterVendorExtension.h>

namespace qti::audio::core {

auto getVectorFromString = [](const std::string& keyString) {
    std::vector<std::string> result;
    size_t i = 0;
    const std::string delimiter{";"};
    /* key1;key2;key3 */
    auto foundPos = keyString.find(delimiter, i);
    while (foundPos != std::string::npos) {
        std::string key = keyString.substr(i, foundPos - i);
        result.push_back(key);
        i = foundPos + 1;
    }
    return result;
};

auto getPairsFromVector = [](const std::vector<std::string>& keyValues) {
    std::vector<std::pair<std::string, std::string>> keyValuePairs;
    for (const auto& keyValue : keyValues) {
        const std::string delimiter{"="};
        auto foundPos = keyValue.find(delimiter, 0);
        if (foundPos == std::string::npos) {
            return keyValuePairs;
        }
        std::string key = keyValue.substr(0, foundPos);
        std::string value = keyValue.substr(foundPos + 1, keyValue.size());
        keyValuePairs.push_back({key, value});
    }
    return keyValuePairs;
};

std::optional<::aidl::android::hardware::audio::core::VendorParameter>
getVendorParameterInt(const std::string& key, const std::string& value) {
    ::aidl::android::hardware::audio::core::VendorParameter param;
    param.id = key;
    try {
        ::aidl::android::media::audio::common::Int parcel;
        parcel.value = std::stoi(value);
        param.ext.setParcelable(parcel);
    } catch (std::invalid_argument const& ex) {
        LOG(ERROR) << "ignoring param: " << key
                   << " due to exception:" << ex.what();
        return std::nullopt;
    } catch (std::out_of_range const& ex) {
        LOG(ERROR) << "ignoring param: " << key
                   << " due to exception:" << ex.what();
        return std::nullopt;
    }
    return param;
}

std::optional<::aidl::android::hardware::audio::core::VendorParameter>
getVendorParameterBool(const std::string& key, const std::string& value) {
    ::aidl::android::hardware::audio::core::VendorParameter param;
    param.id = key;
    try {
        ::aidl::android::media::audio::common::Boolean parcel;
        if (value == "true") {
            parcel.value = true;
        } else {
            parcel.value = false;
        }
        param.ext.setParcelable(parcel);
    } catch (std::invalid_argument const& ex) {
        LOG(ERROR) << "ignoring param: " << key
                   << " due to exception:" << ex.what();
        return std::nullopt;
    } catch (std::out_of_range const& ex) {
        LOG(ERROR) << "ignoring param: " << key
                   << " due to exception:" << ex.what();
        return std::nullopt;
    }
    return param;
}

std::optional<std::string> getStringForVendorParameter(
    const ::aidl::android::hardware::audio::core::VendorParameter& param) {
    std::optional<::aidl::android::media::audio::common::Int> parcel;
    param.ext.getParcelable(&parcel);
    if (!parcel.has_value()) {
        LOG(ERROR) << __func__
                   << " unable to get Int parcelable for key:" << param.id;
        return std::nullopt;
    }
    return param.id + "=" + std::to_string(parcel.value().value);
}

::ndk::ScopedAStatus HalAdapterVendorExtension::parseVendorParameterIds(
    ::aidl::android::media::audio::IHalAdapterVendorExtension::ParameterScope
        in_scope,
    const std::string& in_rawKeys, std::vector<std::string>* _aidl_return) {
    LOG(VERBOSE) << __func__
                 << ::aidl::android::media::audio::toString(in_scope);
    *_aidl_return = getVectorFromString(in_rawKeys);
    return ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus HalAdapterVendorExtension::parseVendorParameters(
    ::aidl::android::media::audio::IHalAdapterVendorExtension::ParameterScope
        in_scope,
    const std::string& in_rawKeysAndValues,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        out_syncParameters,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        out_asyncParameters) {
    LOG(VERBOSE) << __func__
                 << ::aidl::android::media::audio::toString(in_scope);

    const auto& keyValues = getVectorFromString(in_rawKeysAndValues);

    const auto& keyValuesPairs = getPairsFromVector(keyValues);

    std::vector<::aidl::android::hardware::audio::core::VendorParameter> result;

    for (const auto& [key, value] : keyValuesPairs) {
        const auto& param = getVendorParameterInt(key, value);
        param.has_value() ? (void)result.push_back(param.value()) : (void)0;
    }

    *out_syncParameters = result;

    return ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus
HalAdapterVendorExtension::parseBluetoothA2dpReconfigureOffload(
    const std::string& in_rawValue,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    LOG(VERBOSE) << __func__ << "not implemented";
    return ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus
HalAdapterVendorExtension::parseBluetoothLeReconfigureOffload(
    const std::string& in_rawValue,
    std::vector<::aidl::android::hardware::audio::core::VendorParameter>*
        _aidl_return) {
    LOG(VERBOSE) << __func__ << "not implemented";
    return ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus HalAdapterVendorExtension::processVendorParameters(
    ::aidl::android::media::audio::IHalAdapterVendorExtension::ParameterScope
        in_scope,
    const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
        in_parameters,
    std::string* _aidl_return) {
    LOG(VERBOSE) << __func__
                 << ::aidl::android::media::audio::toString(in_scope);
    std::string keyValuesString{""};
    for (const auto& param : in_parameters) {
        const auto& out = getStringForVendorParameter(param);
        if (!out.has_value()) {
            continue;
        }
        if (keyValuesString == "") {
            keyValuesString = out.value();
        } else {
            keyValuesString.append(";").append(out.value());
        }
    }
    *_aidl_return = keyValuesString;
    return ndk::ScopedAStatus::ok();
}

}  // namespace qti::audio::core