/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/audio/core/BnModule.h>

#include <map>
#include <memory>
#include <set>

#include <aidlservice/Stream.h>
#include <aidlservice/ModuleConfig.h>
#include <qti-audio-core/Platform.h>

namespace qti::audio::core {

class Module : public ::aidl::android::hardware::audio::core::BnModule,
               public std::enable_shared_from_this<Module> {
   public:
    // This value is used for all AudioPatches and reported by all streams.
    static constexpr int32_t kLatencyMs = 10;
    enum Type : int { DEFAULT, R_SUBMIX, USB };

    explicit Module(Type type) : mType(type) {}

    static std::shared_ptr<Module> createInstance(Type type);
    static StreamIn::CreateInstance getStreamInCreator(Type type);
    static StreamOut::CreateInstance getStreamOutCreator(Type type);

   private:
    struct VendorDebug {
        static const std::string kForceTransientBurstName;
        static const std::string kForceSynchronousDrainName;
        bool forceTransientBurst = false;
        bool forceSynchronousDrain = false;
    };
    // Helper used for interfaces that require a persistent instance. We hold
    // them via a strong pointer. The binder token is retained for a call to
    // 'setMinSchedulerPolicy'.
    template <class C>
    struct ChildInterface
        : private std::pair<std::shared_ptr<C>, ndk::SpAIBinder> {
        ChildInterface() {}
        ChildInterface& operator=(const std::shared_ptr<C>& c) {
            return operator=(std::shared_ptr<C>(c));
        }
        ChildInterface& operator=(std::shared_ptr<C>&& c) {
            this->first = std::move(c);
            this->second = this->first->asBinder();
            AIBinder_setMinSchedulerPolicy(this->second.get(), SCHED_NORMAL,
                                           ANDROID_PRIORITY_AUDIO);
            return *this;
        }
        explicit operator bool() const { return !!this->first; }
        C& operator*() const { return *(this->first); }
        C* operator->() const { return this->first; }
        std::shared_ptr<C> getPtr() const { return this->first; }
    };

    ndk::ScopedAStatus setModuleDebug(
        const ::aidl::android::hardware::audio::core::ModuleDebug& in_debug)
        override;
    ndk::ScopedAStatus getTelephony(
        std::shared_ptr<::aidl::android::hardware::audio::core::ITelephony>* _aidl_return) override;
    ndk::ScopedAStatus getBluetooth(
        std::shared_ptr<::aidl::android::hardware::audio::core::IBluetooth>* _aidl_return) override;
    ndk::ScopedAStatus getBluetoothA2dp(
        std::shared_ptr<::aidl::android::hardware::audio::core::IBluetoothA2dp>* _aidl_return) override;
    ndk::ScopedAStatus getBluetoothLe(
        std::shared_ptr<::aidl::android::hardware::audio::core::IBluetoothLe>* _aidl_return) override;
    ndk::ScopedAStatus connectExternalDevice(
        const ::aidl::android::media::audio::common::AudioPort&
            in_templateIdAndAdditionalData,
        ::aidl::android::media::audio::common::AudioPort* _aidl_return)
        override;
    ndk::ScopedAStatus disconnectExternalDevice(int32_t in_portId) override;
    ndk::ScopedAStatus getAudioPatches(
        std::vector<::aidl::android::hardware::audio::core::AudioPatch>* _aidl_return) override;
    ndk::ScopedAStatus getAudioPort(
        int32_t in_portId,
        ::aidl::android::media::audio::common::AudioPort* _aidl_return)
        override;
    ndk::ScopedAStatus getAudioPortConfigs(
        std::vector<::aidl::android::media::audio::common::AudioPortConfig>*
            _aidl_return) override;
    ndk::ScopedAStatus getAudioPorts(
        std::vector<::aidl::android::media::audio::common::AudioPort>*
            _aidl_return) override;
    ndk::ScopedAStatus getAudioRoutes(
        std::vector<::aidl::android::hardware::audio::core::AudioRoute>* _aidl_return) override;
    ndk::ScopedAStatus getAudioRoutesForAudioPort(
        int32_t in_portId,
        std::vector<::aidl::android::hardware::audio::core::AudioRoute>*
            _aidl_return) override;
    ndk::ScopedAStatus openInputStream(
        const ::aidl::android::hardware::audio::core::IModule::
            OpenInputStreamArguments& in_args,
        ::aidl::android::hardware::audio::core::IModule::OpenInputStreamReturn*
            _aidl_return) override;
    ndk::ScopedAStatus openOutputStream(
        const ::aidl::android::hardware::audio::core::IModule::
            OpenOutputStreamArguments& in_args,
        ::aidl::android::hardware::audio::core::IModule::OpenOutputStreamReturn*
            _aidl_return) override;
    ndk::ScopedAStatus getSupportedPlaybackRateFactors(
        SupportedPlaybackRateFactors* _aidl_return) override;
    ndk::ScopedAStatus setAudioPatch(const ::aidl::android::hardware::audio::core::AudioPatch& in_requested,
                                     ::aidl::android::hardware::audio::core::AudioPatch* _aidl_return) override;
    ndk::ScopedAStatus setAudioPortConfig(
        const ::aidl::android::media::audio::common::AudioPortConfig&
            in_requested,
        ::aidl::android::media::audio::common::AudioPortConfig* out_suggested,
        bool* _aidl_return) override;
    ndk::ScopedAStatus resetAudioPatch(int32_t in_patchId) override;
    ndk::ScopedAStatus resetAudioPortConfig(int32_t in_portConfigId) override;
    ndk::ScopedAStatus getMasterMute(bool* _aidl_return) override;
    ndk::ScopedAStatus setMasterMute(bool in_mute) override;
    ndk::ScopedAStatus getMasterVolume(float* _aidl_return) override;
    ndk::ScopedAStatus setMasterVolume(float in_volume) override;
    ndk::ScopedAStatus getMicMute(bool* _aidl_return) override;
    ndk::ScopedAStatus setMicMute(bool in_mute) override;
    ndk::ScopedAStatus getMicrophones(
        std::vector<::aidl::android::media::audio::common::MicrophoneInfo>*
            _aidl_return) override;
    ndk::ScopedAStatus updateAudioMode(
        ::aidl::android::media::audio::common::AudioMode in_mode) override;
    ndk::ScopedAStatus updateScreenRotation(
        ::aidl::android::hardware::audio::core::IModule::ScreenRotation
            in_rotation) override;
    ndk::ScopedAStatus updateScreenState(bool in_isTurnedOn) override;
    ndk::ScopedAStatus getSoundDose(
        std::shared_ptr<::aidl::android::hardware::audio::core::sounddose::ISoundDose>* _aidl_return) override;
    ndk::ScopedAStatus generateHwAvSyncId(int32_t* _aidl_return) override;
    ndk::ScopedAStatus getVendorParameters(
        const std::vector<std::string>& in_ids,
        std::vector<::aidl::android::hardware::audio::core::VendorParameter>* _aidl_return) override;
    ndk::ScopedAStatus setVendorParameters(
        const std::vector<::aidl::android::hardware::audio::core::VendorParameter>& in_parameters,
        bool in_async) override;
    ndk::ScopedAStatus addDeviceEffect(
        int32_t in_portConfigId,
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;
    ndk::ScopedAStatus removeDeviceEffect(
        int32_t in_portConfigId,
        const std::shared_ptr<
            ::aidl::android::hardware::audio::effect::IEffect>& in_effect)
        override;
    ndk::ScopedAStatus getMmapPolicyInfos(
        ::aidl::android::media::audio::common::AudioMMapPolicyType
            mmapPolicyType,
        std::vector<::aidl::android::media::audio::common::AudioMMapPolicyInfo>*
            _aidl_return) override;
    ndk::ScopedAStatus supportsVariableLatency(bool* _aidl_return) override;
    ndk::ScopedAStatus getAAudioMixerBurstCount(int32_t* _aidl_return) override;
    ndk::ScopedAStatus getAAudioHardwareBurstMinUsec(
        int32_t* _aidl_return) override;
    binder_status_t dump(int fd, const char** args, uint32_t numArgs) override;

    void cleanUpPatch(int32_t patchId);
    ndk::ScopedAStatus createStreamContext(
        int32_t in_portConfigId, int64_t in_bufferSizeFrames,
        std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback> asyncCallback,
        std::shared_ptr<::aidl::android::hardware::audio::core::IStreamOutEventCallback> outEventCallback,
        StreamContext* out_context);
    std::vector<::aidl::android::media::audio::common::AudioDevice>
    findConnectedDevices(int32_t portConfigId);
    std::set<int32_t> findConnectedPortConfigIds(int32_t portConfigId);
    ndk::ScopedAStatus findPortIdForNewStream(
        int32_t in_portConfigId,
        ::aidl::android::media::audio::common::AudioPort** port);
    ModuleConfig& getConfig();
    template <typename C>
    std::set<int32_t> portIdsFromPortConfigIds(C portConfigIds);
    void registerPatch(const ::aidl::android::hardware::audio::core::AudioPatch& patch);
    void updateStreamsConnectedState(const ::aidl::android::hardware::audio::core::AudioPatch& oldPatch,
                                     const ::aidl::android::hardware::audio::core::AudioPatch& newPatch);
    bool isMmapSupported();

    // This value is used for all AudioPatches.
    static constexpr int32_t kMinimumStreamBufferSizeFrames = 256;
    // The maximum stream buffer size is 1 GiB = 2 ** 30 bytes;
    static constexpr int32_t kMaximumStreamBufferSizeBytes = 1 << 30;

    const Type mType;
    std::unique_ptr<ModuleConfig> mConfig;
    ::aidl::android::hardware::audio::core::ModuleDebug mDebug;
    VendorDebug mVendorDebug;
    ChildInterface<::aidl::android::hardware::audio::core::ITelephony> mTelephony;
    ChildInterface<::aidl::android::hardware::audio::core::IBluetooth> mBluetooth;
    ChildInterface<::aidl::android::hardware::audio::core::IBluetoothA2dp> mBluetoothA2dp;
    ChildInterface<::aidl::android::hardware::audio::core::IBluetoothLe> mBluetoothLe;
    // ids of device ports created at runtime via 'connectExternalDevice'.
    // Also stores ids of mix ports with dynamic profiles which got populated
    // from the connected port.
    std::map<int32_t, std::vector<int32_t>> mConnectedDevicePorts;
    Streams mStreams;
    // Maps port ids and port config ids to patch ids.
    // Multimap because both ports and configs can be used by multiple patches.
    std::multimap<int32_t, int32_t> mPatches;
    bool mMicMute = false;
    ChildInterface<::aidl::android::hardware::audio::core::sounddose::ISoundDose> mSoundDose;
    std::optional<bool> mIsMmapSupported;

   protected:
    // If the module is unable to populate the connected device port correctly,
    // the returned error code must correspond to the errors of
    // `IModule.connectedExternalDevice` method.
    virtual ndk::ScopedAStatus populateConnectedDevicePort(
        ::aidl::android::media::audio::common::AudioPort* connectedDevicePort,
        const int32_t templateDevicePortId);
    // If the module finds that the patch endpoints configurations are not
    // matched, the returned error code must correspond to the errors of
    // `IModule.setAudioPatch` method.
    virtual ndk::ScopedAStatus checkAudioPatchEndpointsMatch(
        const std::vector<
            ::aidl::android::media::audio::common::AudioPortConfig*>& sources,
        const std::vector<
            ::aidl::android::media::audio::common::AudioPortConfig*>& sinks);
    virtual void onExternalDeviceConnectionChanged(
        const ::aidl::android::media::audio::common::AudioPort& audioPort,
        bool connected);
    virtual ndk::ScopedAStatus onMasterMuteChanged(bool mute);
    virtual ndk::ScopedAStatus onMasterVolumeChanged(float volume);
    void onNewPatchCreation(
        const std::vector<
            ::aidl::android::media::audio::common::AudioPortConfig*>& sources,
        const std::vector<
            ::aidl::android::media::audio::common::AudioPortConfig*>& sinks,
        ::aidl::android::hardware::audio::core::AudioPatch& newPatch);

    bool mMasterMute = false;
    float mMasterVolume = 1.0f;

    Platform& mPlatform{Platform::getInstance()};

   public:
    std::string toStringInternal();
    void dumpInternal() ;
};

}  // namespace qti::audio::core