/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/BnBluetooth.h>
#include <aidl/android/hardware/audio/core/BnBluetoothA2dp.h>
#include <aidl/android/hardware/audio/core/BnBluetoothLe.h>

namespace qti::audio::core {
class PlatformBluetooth;
class Bluetooth : public ::aidl::android::hardware::audio::core::BnBluetooth {
   public:
    Bluetooth();

   private:
    ndk::ScopedAStatus setScoConfig(const ScoConfig& in_config,
                                    ScoConfig* _aidl_return) override;
    ndk::ScopedAStatus setHfpConfig(const HfpConfig& in_config,
                                    HfpConfig* _aidl_return) override;

    ScoConfig mScoConfig;
    HfpConfig mHfpConfig;
    std::shared_ptr<PlatformBluetooth> mPlatformBluetooth;
};

class BluetoothA2dp
    : public ::aidl::android::hardware::audio::core::BnBluetoothA2dp {
   public:
    BluetoothA2dp() = default;

   private:
    ndk::ScopedAStatus isEnabled(bool* _aidl_return) override;
    ndk::ScopedAStatus setEnabled(bool in_enabled) override;
    ndk::ScopedAStatus supportsOffloadReconfiguration(
        bool* _aidl_return) override;
    ndk::ScopedAStatus reconfigureOffload(
        const std::vector<
            ::aidl::android::hardware::audio::core::VendorParameter>&
            in_parameters) override;

    bool mEnabled = false;
};

class BluetoothLe : public ::aidl::android::hardware::audio::core::BnBluetoothLe {
  public:
    BluetoothLe() = default;

  private:
    ndk::ScopedAStatus isEnabled(bool* _aidl_return) override;
    ndk::ScopedAStatus setEnabled(bool in_enabled) override;
    ndk::ScopedAStatus supportsOffloadReconfiguration(bool* _aidl_return) override;
    ndk::ScopedAStatus reconfigureOffload(
            const std::vector<::aidl::android::hardware::audio::core::VendorParameter>&
                    in_parameters) override;
    bool mEnabled = false;
};
}  // namespace qti::audio::core
