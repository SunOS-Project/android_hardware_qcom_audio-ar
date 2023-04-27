/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "AHAL_Module"

#include <android-base/logging.h>
#include <android/binder_ibinder_platform.h>

#include <algorithm>
#include <set>
// Todo remove this
#include <Utils.h>
#include <aidl/android/media/audio/common/AudioInputFlags.h>
#include <aidl/android/media/audio/common/AudioOutputFlags.h>
#include <aidlservice/Bluetooth.h>
#include <utils/Helpers.h>
#include <aidlservice/Module.h>
#include <aidlservice/ModuleConfig.h>
#include <aidlservice/SoundDose.h>
#include <aidlservice/StreamStub.h>
#include <aidlservice/Telephony.h>
#include <utils/utils.h>
#include <platform/PlatformModule.h>
#include <system/audio_config.h>

using ::aidl::android::media::audio::common::AudioChannelLayout;
using ::aidl::android::media::audio::common::AudioDevice;
using ::aidl::android::media::audio::common::AudioFormatDescription;
using ::aidl::android::media::audio::common::AudioFormatType;
using ::aidl::android::media::audio::common::AudioInputFlags;
using ::aidl::android::media::audio::common::AudioIoFlags;
using ::aidl::android::media::audio::common::AudioMMapPolicy;
using ::aidl::android::media::audio::common::AudioMMapPolicyInfo;
using ::aidl::android::media::audio::common::AudioMMapPolicyType;
using ::aidl::android::media::audio::common::AudioMode;
using ::aidl::android::media::audio::common::AudioOffloadInfo;
using ::aidl::android::media::audio::common::AudioOutputFlags;
using ::aidl::android::media::audio::common::AudioPort;
using ::aidl::android::media::audio::common::AudioPortConfig;
using ::aidl::android::media::audio::common::AudioPortExt;
using ::aidl::android::media::audio::common::AudioProfile;
using ::aidl::android::media::audio::common::Boolean;
using ::aidl::android::media::audio::common::Int;
using ::aidl::android::media::audio::common::MicrophoneInfo;
using ::aidl::android::media::audio::common::PcmType;

using ::aidl::android::hardware::audio::common::SinkMetadata;
using ::aidl::android::hardware::audio::common::SourceMetadata;
using ::aidl::android::hardware::audio::common::getFrameSizeInBytes;
using ::aidl::android::hardware::audio::common::isBitPositionFlagSet;
using ::aidl::android::hardware::audio::common::isValidAudioMode;

using ::aidl::android::hardware::audio::core::IStreamOut;
using ::aidl::android::hardware::audio::core::IStreamIn;
using ::aidl::android::hardware::audio::core::AudioPatch;
using ::aidl::android::hardware::audio::core::AudioRoute;
using ::aidl::android::hardware::audio::core::IBluetooth;
using ::aidl::android::hardware::audio::core::VendorParameter;
using ::aidl::android::hardware::audio::core::sounddose::ISoundDose;

namespace qti::audio::core {

namespace {

bool findAudioProfile(const AudioPort& port,
                      const AudioFormatDescription& format,
                      AudioProfile* profile) {
    if (auto profilesIt = find_if(port.profiles.begin(), port.profiles.end(),
                                  [&format](const auto& profile) {
                                      return profile.format == format;
                                  });
        profilesIt != port.profiles.end()) {
        *profile = *profilesIt;
        return true;
    }
    return false;
}

}  // namespace

bool Module::generateDefaultPortConfig(const AudioPort& port,
                                       AudioPortConfig& config) {
    config.portId = port.id;
    if (port.profiles.empty()) {
        LOG(ERROR) << __func__ << ": port " << port.id << " has no profiles";
        return false;
    }

    /**
     * Now we may have more than one profile in the Audio,
     * default would be a choice then below chooses first
     **/
    if (port.profiles.empty()) {
        LOG(ERROR) << __func__ << ": given port has no profiles"
                   << port.toString();
        return false;
    }
    const auto& profile = port.profiles[0];

    if (profile.channelMasks.empty()) {
        LOG(ERROR) << __func__ << ": given port has no channel masks"
                   << port.toString();
        return false;
    }

    if (profile.sampleRates.empty()) {
        LOG(ERROR) << __func__ << ": given port has no sample rates"
                   << port.toString();
        return false;
    }

    Int sampleRate;
    sampleRate.value = profile.sampleRates.at(0);
    config.format = profile.format;
    config.sampleRate = sampleRate;
    config.channelMask = profile.channelMasks.at(0);
    config.flags = port.flags;
    config.ext = port.ext;
    return true;
}

void Module::init(std::shared_ptr<Module> module) {
    mPlatformModule = std::make_shared<PlatformModule>(module);
    mPlatformModule->init();

    // TODO remove this dump if not debug
    {
        auto fd = ::open(kAudioHALServiceDumpPath,
                         O_CREAT | O_WRONLY | O_TRUNC,
                         S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        if (fd <= 0) {
            LOG(ERROR) << ": Error " << fd << " unable to open file:"
                       << kAudioHALServiceDumpPath;
        } else {
            int32_t status = dumpInternal(fd);
            LOG(INFO) << (!status ? "dump successful" : "dump failed") << " to "
                      << kAudioHALServiceDumpPath;
            ::close(fd);
        }
    }
}

// static
std::shared_ptr<Module> Module::createInstance(Type type) {
    switch (type) {
        case Module::Type::USB:
            return ndk::SharedRefBase::make<Module>(type);
        case Type::DEFAULT:
        case Type::R_SUBMIX:
        default:
            return ndk::SharedRefBase::make<Module>(type);
    }
}

// static
StreamIn::CreateInstance Module::getStreamInCreator(Type type) {
    switch (type) {
        case Type::USB:
        case Type::DEFAULT:
        case Type::R_SUBMIX:
        default:
            return StreamInStub::createInstance;
    }
}

// static
StreamOut::CreateInstance Module::getStreamOutCreator(Type type) {
    switch (type) {
        case Type::USB:
        case Type::DEFAULT:
        case Type::R_SUBMIX:
        default:
            return StreamOutStub::createInstance;
    }
}

void Module::cleanUpPatch(int32_t patchId) {
    erase_all_values(mPatches, std::set<int32_t>{patchId});
}

ndk::ScopedAStatus Module::createStreamContext(
    int32_t in_portConfigId, int64_t in_bufferSizeFrames,
    std::shared_ptr<::aidl::android::hardware::audio::core::IStreamCallback>
        asyncCallback,
    std::shared_ptr<
        ::aidl::android::hardware::audio::core::IStreamOutEventCallback>
        outEventCallback,
    StreamContext* out_context) {
    if (in_bufferSizeFrames <= 0) {
        LOG(ERROR) << __func__ << ": non-positive buffer size "
                   << in_bufferSizeFrames;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_bufferSizeFrames < kMinimumStreamBufferSizeFrames) {
        LOG(ERROR) << __func__ << ": insufficient buffer size "
                   << in_bufferSizeFrames << ", must be at least "
                   << kMinimumStreamBufferSizeFrames;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    auto& configs = getModuleConfig().mActivePortConfigs;
    auto& portConfig = configs.at(in_portConfigId);
    // Since this is a private method, it is assumed that
    // validity of the portConfigId has already been checked.
    const size_t frameSize = getFrameSizeInBytes(
        portConfig.format.value(), portConfig.channelMask.value());
    if (frameSize == 0) {
        LOG(ERROR) << __func__
                   << ": could not calculate frame size for port config "
                   << portConfig.toString();
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    LOG(DEBUG) << __func__ << ": frame size " << frameSize << " bytes";
    if (frameSize > kMaximumStreamBufferSizeBytes / in_bufferSizeFrames) {
        LOG(ERROR) << __func__ << ": buffer size " << in_bufferSizeFrames
                   << " frames is too large, maximum size is "
                   << kMaximumStreamBufferSizeBytes / frameSize;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    const auto& flags = portConfig.flags.value();
    if ((flags.getTag() == AudioIoFlags::Tag::input &&
         !isBitPositionFlagSet(flags.get<AudioIoFlags::Tag::input>(),
                               AudioInputFlags::MMAP_NOIRQ)) ||
        (flags.getTag() == AudioIoFlags::Tag::output &&
         !isBitPositionFlagSet(flags.get<AudioIoFlags::Tag::output>(),
                               AudioOutputFlags::MMAP_NOIRQ))) {
        StreamContext::DebugParameters params{
            mDebug.streamTransientStateDelayMs,
            mVendorDebug.forceTransientBurst,
            mVendorDebug.forceSynchronousDrain};
        StreamContext temp(std::make_unique<StreamContext::CommandMQ>(
                               1, true /*configureEventFlagWord*/),
                           std::make_unique<StreamContext::ReplyMQ>(
                               1, true /*configureEventFlagWord*/),
                           portConfig.format.value(),
                           portConfig.channelMask.value(),
                           portConfig.sampleRate.value().value,
                           std::make_unique<StreamContext::DataMQ>(
                               frameSize * in_bufferSizeFrames),
                           asyncCallback, outEventCallback, params);
        if (temp.isValid()) {
            *out_context = std::move(temp);
        } else {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
    } else {
        // TODO: Implement simulation of MMAP buffer allocation
    }
    return ndk::ScopedAStatus::ok();
}

std::vector<AudioDevice> Module::findConnectedDevices(int32_t portConfigId) {
    std::vector<AudioDevice> result;
    auto& ports = getModuleConfig().mPorts;
    auto portIds =
        portIdsFromPortConfigIds(findConnectedPortConfigIds(portConfigId));
    for (auto it = portIds.begin(); it != portIds.end(); ++it) {
        if (ports.contains(*it)) {
            auto port = ports[*it];
            if (port.ext.getTag() == AudioPortExt::Tag::device) {
                result.push_back(
                    port.ext.template get<AudioPortExt::Tag::device>().device);
            }
        }
    }
    return result;
}

std::set<int32_t> Module::findConnectedPortConfigIds(int32_t portConfigId) {
    std::set<int32_t> result;
    auto patchIdsRange = mPatches.equal_range(portConfigId);
    auto& patches = getModuleConfig().mPatches;
    for (auto it = patchIdsRange.first; it != patchIdsRange.second; ++it) {
        if (!patches.contains(it->second)) {
            LOG(FATAL) << __func__ << ": patch with id " << it->second
                       << " taken from mPatches "
                       << "not found in the configuration";
        }
        auto patch = patches[it->second];
        if (std::find(patch.sourcePortConfigIds.begin(),
                      patch.sourcePortConfigIds.end(),
                      portConfigId) != patch.sourcePortConfigIds.end()) {
            result.insert(patch.sinkPortConfigIds.begin(),
                          patch.sinkPortConfigIds.end());
        } else {
            result.insert(patch.sourcePortConfigIds.begin(),
                          patch.sourcePortConfigIds.end());
        }
    }
    return result;
}

ndk::ScopedAStatus Module::findPortIdForNewStream(int32_t in_portConfigId,
                                                  AudioPort** port) {
    auto& configs = getModuleConfig().mActivePortConfigs;
    if (!configs.contains(in_portConfigId)) {
        LOG(ERROR) << __func__ << ": existing port config id "
                   << in_portConfigId << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    auto& portConfig = configs.at(in_portConfigId);
    const int32_t portId = portConfig.portId;
    // In our implementation, configs of mix ports always have unique IDs.
    CHECK(portId != in_portConfigId);
    if (!getModuleConfig().mPorts.contains(portId)) {
        LOG(ERROR) << __func__ << ": port id " << portId
                   << " used by port config id " << in_portConfigId
                   << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (mStreams.count(in_portConfigId) != 0) {
        LOG(ERROR) << __func__ << ": port config id " << in_portConfigId
                   << " already has a stream opened on it";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    auto& portForId = getModuleConfig().mPorts[portId];
    if (portForId.ext.getTag() != AudioPortExt::Tag::mix) {
        LOG(ERROR) << __func__ << ": port config id " << in_portConfigId
                   << " does not correspond to a mix port";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    const int32_t maxOpenStreamCount =
        portForId.ext.get<AudioPortExt::Tag::mix>().maxOpenStreamCount;
    if (maxOpenStreamCount != 0 &&
        mStreams.count(portId) >= maxOpenStreamCount) {
        LOG(ERROR)
            << __func__ << ": port id " << portId
            << " has already reached maximum allowed opened stream count: "
            << maxOpenStreamCount;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    *port = &portForId;
    return ndk::ScopedAStatus::ok();
}

template <typename C>
std::set<int32_t> Module::portIdsFromPortConfigIds(C portConfigIds) {
    std::set<int32_t> result;
    auto& portConfigs = getModuleConfig().mActivePortConfigs;
    for (auto it = portConfigIds.begin(); it != portConfigIds.end(); ++it) {
        if (portConfigs.contains(*it)) {
            result.insert(portConfigs.at(*it).portId);
        }
    }
    return result;
}

ModuleConfig& Module::getModuleConfig() {
    if (!mModuleConfig) {
        switch (mType) {
            case Type::R_SUBMIX: {
                // mModuleConfig =
                // std::move(internal::getRSubmixConfiguration());
                break;
            }
            case Type::USB: {
                // mModuleConfig = std::move(internal::getUsbConfiguration());
                break;
            }
            // primary module
            case Type::DEFAULT:
            default: {
                mModuleConfig = std::move(getPrimaryConfiguration());
                break;
            }
        }
    }
    return *mModuleConfig;
}

void Module::registerPatch(const AudioPatch& patch) {
    auto& configs = getModuleConfig().mActivePortConfigs;
    auto do_insert = [&](const std::vector<int32_t>& portConfigIds) {
        for (auto portConfigId : portConfigIds) {
            if (configs.contains(portConfigId)) {
                auto& portConfig = configs.at(portConfigId);
                mPatches.insert(std::pair{portConfigId, patch.id});
                if (portConfig.portId != portConfigId) {
                    mPatches.insert(std::pair{portConfig.portId, patch.id});
                }
            }
        };
    };
    do_insert(patch.sourcePortConfigIds);
    do_insert(patch.sinkPortConfigIds);
}

void Module::updateStreamsConnectedState(const AudioPatch& oldPatch,
                                         const AudioPatch& newPatch) {
    // Streams from the old patch need to be disconnected, streams from the new
    // patch need to be connected. If the stream belongs to both patches, no
    // need to update it.
    std::set<int32_t> idsToDisconnect, idsToConnect;
    idsToDisconnect.insert(oldPatch.sourcePortConfigIds.begin(),
                           oldPatch.sourcePortConfigIds.end());
    idsToDisconnect.insert(oldPatch.sinkPortConfigIds.begin(),
                           oldPatch.sinkPortConfigIds.end());
    idsToConnect.insert(newPatch.sourcePortConfigIds.begin(),
                        newPatch.sourcePortConfigIds.end());
    idsToConnect.insert(newPatch.sinkPortConfigIds.begin(),
                        newPatch.sinkPortConfigIds.end());
    std::for_each(idsToDisconnect.begin(), idsToDisconnect.end(),
                  [&](const auto& portConfigId) {
                      if (idsToConnect.count(portConfigId) == 0) {
                          LOG(DEBUG) << "The stream on port config id "
                                     << portConfigId << " is not connected";
                          mStreams.setStreamIsConnected(portConfigId, {});
                      }
                  });
    std::for_each(
        idsToConnect.begin(), idsToConnect.end(),
        [&](const auto& portConfigId) {
            if (idsToDisconnect.count(portConfigId) == 0) {
                const auto connectedDevices =
                    findConnectedDevices(portConfigId);
                LOG(DEBUG) << "The stream on port config id " << portConfigId
                           << " is connected to: "
                           << ::android::internal::ToString(connectedDevices);
                mStreams.setStreamIsConnected(portConfigId, connectedDevices);
            }
        });
}

ndk::ScopedAStatus Module::setModuleDebug(
    const ::aidl::android::hardware::audio::core::ModuleDebug& in_debug) {
    LOG(DEBUG) << __func__ << ": old flags:" << mDebug.toString()
               << ", new flags: " << in_debug.toString();
    if (mDebug.simulateDeviceConnections !=
            in_debug.simulateDeviceConnections &&
        !mConnExtDevicePortsIds.empty()) {
        LOG(ERROR) << __func__
                   << ": attempting to change device connections simulation "
                   << "while having external devices connected";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    if (in_debug.streamTransientStateDelayMs < 0) {
        LOG(ERROR) << __func__ << ": streamTransientStateDelayMs is negative: "
                   << in_debug.streamTransientStateDelayMs;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    mDebug = in_debug;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getTelephony(
    std::shared_ptr<::aidl::android::hardware::audio::core::ITelephony>*
        _aidl_return) {
    if (!mTelephony) {
        mTelephony = ndk::SharedRefBase::make<Telephony>();
    }
    *_aidl_return = mTelephony.getPtr();
    LOG(DEBUG) << __func__ << ": returning instance of ITelephony: " << _aidl_return->get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getBluetooth(
    std::shared_ptr<IBluetooth>* _aidl_return) {
    if (!mBluetooth) {
        mBluetooth = ndk::SharedRefBase::make<Bluetooth>();
    }
    *_aidl_return = mBluetooth.getPtr();
    LOG(DEBUG) << __func__ << ": returning instance of IBluetooth: " << _aidl_return->get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getBluetoothA2dp(
    std::shared_ptr<::aidl::android::hardware::audio::core::IBluetoothA2dp>*
        _aidl_return) {
    if (!mBluetoothA2dp) {
        mBluetoothA2dp = ndk::SharedRefBase::make<BluetoothA2dp>();
    }
    *_aidl_return = mBluetoothA2dp.getPtr();
    LOG(DEBUG) << __func__ << ": returning instance of IBluetoothA2dp: "
               << _aidl_return->get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getBluetoothLe(std::shared_ptr<::aidl::android::hardware::audio::core::IBluetoothLe>* _aidl_return) {
    if (!mBluetoothLe) {
        mBluetoothLe = ndk::SharedRefBase::make<BluetoothLe>();
    }
    *_aidl_return = mBluetoothLe.getPtr();
    LOG(DEBUG) << __func__ << ": returning instance of IBluetoothLe: " << _aidl_return->get();
    return ndk::ScopedAStatus::ok();
}
std::vector<AudioProfile> Module::getProfilesForDevicePort(
    const AudioPort& in_templateDevicePort) {
    const auto& devicePortExt =
        in_templateDevicePort.ext.get<AudioPortExt::Tag::device>();

    // In debug mode never call, platform API
    if (!mDebug.simulateDeviceConnections &&
        is_usb_device(devicePortExt.device)) {
        return mPlatformModule->getDynamicProfiles(in_templateDevicePort);
    }

    // since mExternalDevicePortProfiles match id with templates AudioPorts
    const auto& extDevPortsProfiles =
        getModuleConfig().mExternalDevicePortProfiles;
    if (extDevPortsProfiles.find(in_templateDevicePort.id) ==
        extDevPortsProfiles.cend()) {
        LOG(ERROR) << __func__
                   << ": no external device port profiles for the port"
                   << in_templateDevicePort.toString();
        return {};
    }
    return extDevPortsProfiles.at(in_templateDevicePort.id);
}

ndk::ScopedAStatus Module::connectExternalDevice(
    const AudioPort& in_templateIdAndAdditionalData, AudioPort* _aidl_return) {
    const int32_t templateId = in_templateIdAndAdditionalData.id;
    const auto& ports = getModuleConfig().mPorts;

    if (!ports.contains(templateId)) {
        LOG(ERROR) << __func__ << ": template port id:" << templateId
                   << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    if (in_templateIdAndAdditionalData.ext.getTag() !=
        AudioPortExt::Tag::device) {
        LOG(ERROR) << __func__ << ": port id " << templateId
                   << " is not a device port";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    if (!in_templateIdAndAdditionalData.profiles.empty()) {
        LOG(ERROR) << __func__ << ": port id " << templateId
                   << " does have AudioProfiles profiles";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    auto& in_templateDevicePort =
        in_templateIdAndAdditionalData.ext.get<AudioPortExt::Tag::device>();
    if (in_templateDevicePort.device.type.connection.empty()) {
        LOG(ERROR) << __func__ << ": port id " << templateId
                   << " is permanently attached";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    /**
     * Check if there is already a connected port with for the same external
     * device.
     **/
    for (auto connectedPortId : mConnExtDevicePortsIds) {
        auto connectedPortEntry = ports.at(connectedPortId);
        if (connectedPortEntry.ext.get<AudioPortExt::Tag::device>().device ==
            in_templateDevicePort.device) {
            LOG(ERROR) << __func__ << ": device "
                       << in_templateDevicePort.device.toString()
                       << " is already connected at the device port id "
                       << connectedPortId;
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
    }

    // lets handle connect when not in debug mode
    if (!mDebug.simulateDeviceConnections &&
        !mPlatformModule->handleDeviceConnectionChange(
            in_templateIdAndAdditionalData, true)) {
        LOG(ERROR) << __func__ << ": failed to connect external device";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    AudioPort newAudioPort;
    newAudioPort = in_templateIdAndAdditionalData;
    newAudioPort.profiles =
        getProfilesForDevicePort(in_templateIdAndAdditionalData);
    if (newAudioPort.profiles.empty()) {
        LOG(ERROR) << __func__
                   << ": failed to add dynamic AudioProfiles for AudioPort:" +
                          newAudioPort.toString();
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    // Assign new id for a new AudioPort
    newAudioPort.id = ++getModuleConfig().mNextPortId;
    mConnExtDevicePortsIds.insert(newAudioPort.id);
    LOG(DEBUG) << __func__ << ": new Audio Port with id:" << newAudioPort.id
               << " created for external "
               << newAudioPort.ext.get<AudioPortExt::Tag::device>().toString();

    std::vector<AudioRoute> newRoutes;
    auto& routes = getModuleConfig().mRoutes;
    for (auto& r : routes) {
        if (r.sinkPortId == templateId) {
            AudioRoute newRoute;
            newRoute.sourcePortIds = r.sourcePortIds;
            newRoute.sinkPortId = newAudioPort.id;
            newRoute.isExclusive = r.isExclusive;
            newRoutes.push_back(std::move(newRoute));
        } else {
            auto& srcs = r.sourcePortIds;
            if (std::find(srcs.begin(), srcs.end(), templateId) != srcs.end()) {
                srcs.push_back(newAudioPort.id);
            }
        }
    }
    routes.insert(routes.end(), newRoutes.begin(), newRoutes.end());

    *_aidl_return =
        getModuleConfig().mPorts.emplace_back(std::move(newAudioPort));

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::disconnectExternalDevice(int32_t in_portId) {
    const auto& ports = getModuleConfig().mPorts;
    if (!ports.contains(in_portId)) {
        LOG(ERROR) << __func__ << ": port id " << in_portId << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    const auto& port = ports.at(in_portId);
    if (port.ext.getTag() != AudioPortExt::Tag::device) {
        LOG(ERROR) << __func__ << ": port id " << in_portId
                   << " is not a device port";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    if (mConnExtDevicePortsIds.count(in_portId) == 0) {
        LOG(ERROR) << __func__ << ": port id " << in_portId
                   << " is not a connected device port";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    const auto& configs = getModuleConfig().mActivePortConfigs;
    auto hasActiveConfigIt = std::find_if(configs.cbegin(), configs.cend(),
                                          [in_portId](auto& config) {
                                              if (config.portId == in_portId) {
                                                  return true;
                                              }
                                              return false;
                                          });
    if (hasActiveConfigIt != configs.cend()) {
        LOG(ERROR) << __func__ << ": port id " << in_portId
                   << " has a active port config with id "
                   << hasActiveConfigIt->id;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    // lets handle disconnect when not in debug mode
    if (!mDebug.simulateDeviceConnections &&
        !mPlatformModule->handleDeviceConnectionChange(port, false)) {
        LOG(ERROR) << __func__ << ": failed to disconnect external device";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    getModuleConfig().mPorts.erase(in_portId);
    mConnExtDevicePortsIds.erase(in_portId);
    LOG(DEBUG) << __func__ << ": connected device port " << in_portId
               << " released";

    auto& routes = getModuleConfig().mRoutes;
    for (auto routesIt = routes.begin(); routesIt != routes.end();) {
        if (routesIt->sinkPortId == in_portId) {
            routesIt = routes.erase(routesIt);
        } else {
            // Note: the list of sourcePortIds can't become empty because there
            // must be the id of the template port in the route.
            erase_if(routesIt->sourcePortIds,
                     [in_portId](auto src) { return src == in_portId; });
            ++routesIt;
        }
    }
    return ndk::ScopedAStatus::ok();
}

void Module::onExternalDeviceConnectionChanged(
    const ::aidl::android::media::audio::common::AudioPort& audioPort __unused,
    bool connected __unused) {
    LOG(DEBUG) << __func__ << ": do nothing and return";
}

ndk::ScopedAStatus Module::onMasterMuteChanged(bool mute __unused) {
    LOG(VERBOSE) << __func__ << ": do nothing and return ok";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::onMasterVolumeChanged(float volume __unused) {
    LOG(VERBOSE) << __func__ << ": do nothing and return ok";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAudioPatches(
    std::vector<AudioPatch>* _aidl_return) {
    *_aidl_return = getModuleConfig().mPatches.getCollection();
    LOG(DEBUG) << __func__ << ": returning " << _aidl_return->size()
               << " patches";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAudioPort(int32_t in_portId,
                                        AudioPort* _aidl_return) {
    auto& ports = getModuleConfig().mPorts;
    if (!ports.contains(in_portId)) {
        LOG(ERROR) << __func__ << ": port id " << in_portId << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    *_aidl_return = ports.at(in_portId);
    LOG(DEBUG) << __func__ << ": returning port by id " << in_portId;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAudioPortConfigs(
    std::vector<AudioPortConfig>* _aidl_return) {
    *_aidl_return = getModuleConfig().mActivePortConfigs.getCollection();
    LOG(DEBUG) << __func__ << ": returning " << _aidl_return->size()
               << " port configs";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAudioPorts(std::vector<AudioPort>* _aidl_return) {
    *_aidl_return = getModuleConfig().mPorts.getCollection();
    LOG(DEBUG) << __func__ << ": returning " << _aidl_return->size()
               << " ports";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAudioRoutes(
    std::vector<AudioRoute>* _aidl_return) {
    *_aidl_return = getModuleConfig().mRoutes;
    LOG(DEBUG) << __func__ << ": returning " << _aidl_return->size()
               << " routes";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAudioRoutesForAudioPort(
    int32_t in_portId, std::vector<AudioRoute>* _aidl_return) {
    auto& ports = getModuleConfig().mPorts;
    if (!ports.contains(in_portId)) {
        LOG(ERROR) << __func__ << ": port id " << in_portId << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    auto& routes = getModuleConfig().mRoutes;
    std::copy_if(routes.begin(), routes.end(),
                 std::back_inserter(*_aidl_return), [&](const auto& r) {
                     const auto& srcs = r.sourcePortIds;
                     return r.sinkPortId == in_portId ||
                            std::find(srcs.begin(), srcs.end(), in_portId) !=
                                srcs.end();
                 });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::openInputStream(
    const OpenInputStreamArguments& in_args,
    OpenInputStreamReturn* _aidl_return) {
    LOG(DEBUG) << __func__ << ": port config id " << in_args.portConfigId
               << ", buffer size " << in_args.bufferSizeFrames << " frames";
    AudioPort* port = nullptr;
    if (auto status = findPortIdForNewStream(in_args.portConfigId, &port);
        !status.isOk()) {
        return status;
    }
    if (port->flags.getTag() != AudioIoFlags::Tag::input) {
        LOG(ERROR) << __func__ << ": port config id " << in_args.portConfigId
                   << " does not correspond to an input mix port";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    StreamContext context;
    if (auto status =
            createStreamContext(in_args.portConfigId, in_args.bufferSizeFrames,
                                nullptr, nullptr, &context);
        !status.isOk()) {
        return status;
    }
    context.fillDescriptor(&_aidl_return->desc);
    std::shared_ptr<StreamIn> stream;
    ndk::ScopedAStatus status =
        getStreamInCreator(mType)(in_args.sinkMetadata, std::move(context),
                                  getModuleConfig().mMicrophones, &stream);
    if (!status.isOk()) {
        return status;
    }
    StreamWrapper streamWrapper(stream);
    AIBinder_setMinSchedulerPolicy(streamWrapper.getBinder().get(),
                                   SCHED_NORMAL, ANDROID_PRIORITY_AUDIO);
    auto patchIt = mPatches.find(in_args.portConfigId);
    if (patchIt != mPatches.end()) {
        streamWrapper.setStreamIsConnected(
            findConnectedDevices(in_args.portConfigId));
    }
    mStreams.insert(port->id, in_args.portConfigId, std::move(streamWrapper));
    _aidl_return->stream = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::openOutputStream(
    const OpenOutputStreamArguments& in_args,
    OpenOutputStreamReturn* _aidl_return) {
    LOG(DEBUG) << __func__ << ": port config id " << in_args.portConfigId
               << ", has offload info? " << (in_args.offloadInfo.has_value())
               << ", buffer size " << in_args.bufferSizeFrames << " frames";
    AudioPort* port = nullptr;
    if (auto status = findPortIdForNewStream(in_args.portConfigId, &port);
        !status.isOk()) {
        return status;
    }
    if (port->flags.getTag() != AudioIoFlags::Tag::output) {
        LOG(ERROR) << __func__ << ": port config id " << in_args.portConfigId
                   << " does not correspond to an output mix port";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    const bool isOffload =
        isBitPositionFlagSet(port->flags.get<AudioIoFlags::Tag::output>(),
                             AudioOutputFlags::COMPRESS_OFFLOAD);
    if (isOffload && !in_args.offloadInfo.has_value()) {
        LOG(ERROR) << __func__ << ": port id " << port->id
                   << " has COMPRESS_OFFLOAD flag set, requires offload info";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    const bool isNonBlocking =
        isBitPositionFlagSet(port->flags.get<AudioIoFlags::Tag::output>(),
                             AudioOutputFlags::NON_BLOCKING);
    if (isNonBlocking && in_args.callback == nullptr) {
        LOG(ERROR) << __func__ << ": port id " << port->id
                   << " has NON_BLOCKING flag set, requires async callback";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    StreamContext context;
    if (auto status =
            createStreamContext(in_args.portConfigId, in_args.bufferSizeFrames,
                                isNonBlocking ? in_args.callback : nullptr,
                                in_args.eventCallback, &context);
        !status.isOk()) {
        return status;
    }
    context.fillDescriptor(&_aidl_return->desc);
    std::shared_ptr<StreamOut> stream;
    ndk::ScopedAStatus status =
        getStreamOutCreator(mType)(in_args.sourceMetadata, std::move(context),
                                   in_args.offloadInfo, &stream);
    if (!status.isOk()) {
        return status;
    }
    StreamWrapper streamWrapper(stream);
    AIBinder_setMinSchedulerPolicy(streamWrapper.getBinder().get(),
                                   SCHED_NORMAL, ANDROID_PRIORITY_AUDIO);
    auto patchIt = mPatches.find(in_args.portConfigId);
    if (patchIt != mPatches.end()) {
        streamWrapper.setStreamIsConnected(
            findConnectedDevices(in_args.portConfigId));
    }
    mStreams.insert(port->id, in_args.portConfigId, std::move(streamWrapper));
    _aidl_return->stream = std::move(stream);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getSupportedPlaybackRateFactors(
    SupportedPlaybackRateFactors* _aidl_return) {
    LOG(DEBUG) << __func__;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Module::setAudioPatch(const AudioPatch& in_requested,
                                         AudioPatch* _aidl_return) {
    LOG(DEBUG) << __func__ << ": requested patch " << in_requested.toString();
    if (in_requested.sourcePortConfigIds.empty()) {
        LOG(ERROR) << __func__ << ": requested patch has empty sources list";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (!all_unique<int32_t>(in_requested.sourcePortConfigIds)) {
        LOG(ERROR) << __func__
                   << ": requested patch has duplicate ids in the sources list";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (in_requested.sinkPortConfigIds.empty()) {
        LOG(ERROR) << __func__ << ": requested patch has empty sinks list";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    if (!all_unique<int32_t>(in_requested.sinkPortConfigIds)) {
        LOG(ERROR) << __func__
                   << ": requested patch has duplicate ids in the sinks list";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    auto& configs = getModuleConfig().mActivePortConfigs;

    std::vector<int32_t> missingIds;
    std::vector<const AudioPortConfig*> sources;
    // Return elements from the vector that have specified ids, also
    // optionally return which ids were not found.
    for (auto id : in_requested.sourcePortConfigIds) {
        if (configs.contains(id)) {
            sources.emplace_back(&(configs.at(id)));
        } else {
            missingIds.emplace_back(id);
        }
    }
    if (!missingIds.empty()) {
        LOG(ERROR) << __func__
                   << ": following source port config ids not found: "
                   << ::android::internal::ToString(missingIds);
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    std::vector<const AudioPortConfig*> sinks;
    for (auto id : in_requested.sinkPortConfigIds) {
        if (configs.contains(id)) {
            sinks.emplace_back(&(configs.at(id)));
        } else {
            missingIds.emplace_back(id);
        }
    }
    if (!missingIds.empty()) {
        LOG(ERROR) << __func__ << ": following sink port config ids not found: "
                   << ::android::internal::ToString(missingIds);
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    // bool indicates whether a non-exclusive route is available.
    // If only an exclusive route is available, that means the patch can not be
    // established if there is any other patch which currently uses the sink
    // port.
    std::map<int32_t, bool> allowedSinkPorts;
    auto& routes = getModuleConfig().mRoutes;
    for (auto src : sources) {
        for (const auto& r : routes) {
            const auto& srcs = r.sourcePortIds;
            if (std::find(srcs.begin(), srcs.end(), src->portId) !=
                srcs.end()) {
                if (!allowedSinkPorts[r.sinkPortId]) {  // prefer non-exclusive
                    allowedSinkPorts[r.sinkPortId] = !r.isExclusive;
                }
            }
        }
    }
    for (auto sink : sinks) {
        if (allowedSinkPorts.count(sink->portId) == 0) {
            LOG(ERROR) << __func__ << ": there is no route to the sink port id "
                       << sink->portId;
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
    }

    if (auto status = checkAudioPatchEndpointsMatch(sources, sinks);
        !status.isOk()) {
        return status;
    }
    auto& patches = getModuleConfig().mPatches;
    AudioPatch* existing = nullptr;
    std::optional<decltype(mPatches)> patchesBackup;
    if (in_requested.id != 0) {
        if (patches.contains(in_requested.id)) {
            existing = &patches[in_requested.id];
            patchesBackup = mPatches;
            cleanUpPatch(existing->id);
        } else {
            LOG(ERROR) << __func__ << ": not found existing patch id "
                       << in_requested.id;
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
    }
    // Validate the requested patch.
    for (const auto& [sinkPortId, nonExclusive] : allowedSinkPorts) {
        if (!nonExclusive && mPatches.count(sinkPortId) != 0) {
            LOG(ERROR)
                << __func__ << ": sink port id " << sinkPortId
                << "is exclusive and is already used by some other patch";
            if (patchesBackup.has_value()) {
                mPatches = std::move(*patchesBackup);
            }
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
    }
    *_aidl_return = in_requested;
    _aidl_return->minimumStreamBufferSizeFrames =
        kMinimumStreamBufferSizeFrames;
    _aidl_return->latenciesMs.clear();
    _aidl_return->latenciesMs.insert(_aidl_return->latenciesMs.end(),
                                     _aidl_return->sinkPortConfigIds.size(),
                                     kLatencyMs);
    AudioPatch oldPatch{};
    if (!existing) {
        _aidl_return->id = getModuleConfig().mNextPatchId++;
        patches.push_back(*_aidl_return);
        existing = &patches.back();
    } else {
        oldPatch = *existing;
        *existing = *_aidl_return;
    }
    registerPatch(*existing);
    updateStreamsConnectedState(oldPatch, *_aidl_return);

    LOG(DEBUG) << __func__ << ": " << (oldPatch.id == 0 ? "created" : "updated")
               << " patch " << _aidl_return->toString();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::setAudioPortConfig(
    const AudioPortConfig& in_requested, AudioPortConfig* out_suggested,
    bool* _aidl_return) {
    LOG(DEBUG) << __func__ << ": requested " << in_requested.toString();

    auto& configs = getModuleConfig().mActivePortConfigs;
    const bool isExistingConfig =
        in_requested.id != 0 ? configs.contains(in_requested.id) : false;
    const auto existing =
        isExistingConfig ? &configs.at(in_requested.id) : nullptr;

    if (in_requested.id != 0 && !isExistingConfig) {
        LOG(ERROR) << __func__ << ": requested port config id "
                   << in_requested.id << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    } else if (isExistingConfig && existing->portId == 0) {
        LOG(ERROR) << __func__ << ": requested port config id "
                   << in_requested.id << " has port Id 0";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    } else if (!isExistingConfig && in_requested.portId == 0) {
        LOG(ERROR) << __func__ << ": input port config does not specify portId";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    const int portId = existing ? existing->portId : in_requested.portId;
    auto& ports = getModuleConfig().mPorts;
    if (!ports.contains(portId)) {
        LOG(ERROR) << __func__
                   << ": input port config points to non-existent portId "
                   << portId;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    auto& port = ports.at(portId);
    if (existing != nullptr) {
        *out_suggested = *existing;
        LOG(DEBUG) << __func__
                   << ": already existing port config: " << existing->id;
    } else {
        AudioPortConfig newConfig;
        if (generateDefaultPortConfig(port, newConfig)) {
            *out_suggested = newConfig;
        } else {
            LOG(ERROR) << __func__
                       << ": unable generate a default config for port "
                       << port.toString();
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
    }

    // From this moment, 'out_suggested' is either an existing port config,
    // or a new generated config. Now attempt to update it according to the
    // specified fields of 'in_requested'.

    bool requestedIsValid = true, requestedIsFullySpecified = true;

    AudioIoFlags portFlags = port.flags;
    if (in_requested.flags.has_value()) {
        if (in_requested.flags.value() != portFlags) {
            LOG(WARNING) << __func__ << ": requested flags "
                         << in_requested.flags.value().toString()
                         << " do not match port's " << portId << " flags "
                         << portFlags.toString();
            requestedIsValid = false;
        }
    } else {
        requestedIsFullySpecified = false;
    }

    AudioProfile portProfile;
    if (in_requested.format.has_value()) {
        const auto& format = in_requested.format.value();
        if (findAudioProfile(port, format, &portProfile)) {
            out_suggested->format = format;
        } else {
            LOG(WARNING) << __func__ << ": requested format "
                         << format.toString() << " is not found in port's "
                         << portId << " profiles";
            requestedIsValid = false;
        }
    } else {
        requestedIsFullySpecified = false;
    }
    if (!findAudioProfile(port, out_suggested->format.value(), &portProfile)) {
        LOG(ERROR) << __func__ << ": port " << portId
                   << " does not support format "
                   << out_suggested->format.value().toString() << " anymore";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    if (in_requested.channelMask.has_value()) {
        const auto& channelMask = in_requested.channelMask.value();
        if (find(portProfile.channelMasks.begin(),
                 portProfile.channelMasks.end(),
                 channelMask) != portProfile.channelMasks.end()) {
            out_suggested->channelMask = channelMask;
        } else {
            LOG(WARNING) << __func__ << ": requested channel mask "
                         << channelMask.toString()
                         << " is not supported for the format "
                         << portProfile.format.toString() << " by the port "
                         << portId;
            requestedIsValid = false;
        }
    } else {
        requestedIsFullySpecified = false;
    }

    if (in_requested.sampleRate.has_value()) {
        const auto& sampleRate = in_requested.sampleRate.value();
        if (find(portProfile.sampleRates.begin(), portProfile.sampleRates.end(),
                 sampleRate.value) != portProfile.sampleRates.end()) {
            out_suggested->sampleRate = sampleRate;
        } else {
            LOG(WARNING) << __func__ << ": requested sample rate "
                         << sampleRate.value
                         << " is not supported for the format "
                         << portProfile.format.toString() << " by the port "
                         << portId;
            requestedIsValid = false;
        }
    } else {
        requestedIsFullySpecified = false;
    }

    if (in_requested.gain.has_value()) {
        // Let's pretend that gain can always be applied.
        out_suggested->gain = in_requested.gain.value();
    }

    if (existing == nullptr && requestedIsValid && requestedIsFullySpecified) {
        out_suggested->id = getModuleConfig().mNextPortId++;
        configs.push_back(*out_suggested);
        *_aidl_return = true;
        LOG(DEBUG) << __func__ << ": created new port config "
                   << out_suggested->toString();
    } else if (existing != nullptr && requestedIsValid) {
        // TODO check later! why update existing?
        // *existing = *out_suggested;
        *_aidl_return = true;
        LOG(DEBUG) << __func__ << ": updated port config "
                   << out_suggested->toString();
    } else {
        LOG(DEBUG) << __func__ << ": not applied; existing config ? "
                   << (existing != nullptr) << "; requested is valid? "
                   << requestedIsValid << ", fully specified? "
                   << requestedIsFullySpecified;
        *_aidl_return = false;
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::resetAudioPatch(int32_t in_patchId) {
    auto& patches = getModuleConfig().mPatches;
    if (patches.contains(in_patchId)) {
        auto patch = patches[in_patchId];
        cleanUpPatch(patch.id);
        updateStreamsConnectedState(patch, AudioPatch{});
        patches.erase(patch.id);
        LOG(DEBUG) << __func__ << ": erased patch " << in_patchId;
        return ndk::ScopedAStatus::ok();
    }
    LOG(ERROR) << __func__ << ": patch id " << in_patchId << " not found";
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus Module::resetAudioPortConfig(int32_t in_portConfigId) {
    auto& configs = getModuleConfig().mActivePortConfigs;
    if (!configs.contains(in_portConfigId)) {
        LOG(ERROR) << __func__ << ": port config id " << in_portConfigId
                   << " not found";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    if (mStreams.count(in_portConfigId) != 0) {
        LOG(ERROR) << __func__ << ": port config id " << in_portConfigId
                   << " has a stream opened on it";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }
    auto patchIt = mPatches.find(in_portConfigId);
    if (patchIt != mPatches.end()) {
        LOG(ERROR) << __func__ << ": port config id " << in_portConfigId
                   << " is used by the patch with id " << patchIt->second;
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
    }

    configs.erase(in_portConfigId);
    LOG(DEBUG) << __func__ << ": erased port config " << in_portConfigId;

    // TODO is default's necessary ?
    auto& initials = getModuleConfig().mDefaultPortConfigs;
    if (initials.contains(in_portConfigId)) {
        LOG(DEBUG) << __func__ << ": reset port config " << in_portConfigId;
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getMasterMute(bool* _aidl_return) {
    *_aidl_return = mMasterMute;
    LOG(DEBUG) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::setMasterMute(bool in_mute) {
    LOG(DEBUG) << __func__ << ": " << in_mute;
    mMasterMute = in_mute;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getMasterVolume(float* _aidl_return) {
    *_aidl_return = mMasterVolume;
    LOG(DEBUG) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::setMasterVolume(float in_volume) {
    LOG(DEBUG) << __func__ << ": " << in_volume;
    if (in_volume >= 0.0f && in_volume <= 1.0f) {
        mMasterVolume = in_volume;
        return ndk::ScopedAStatus::ok();
    }
    LOG(ERROR) << __func__ << ": invalid master volume value: " << in_volume;
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus Module::getMicMute(bool* _aidl_return) {
    *_aidl_return = mMicMute;
    LOG(DEBUG) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::setMicMute(bool in_mute) {
    LOG(DEBUG) << __func__ << ": " << in_mute;
    mMicMute = in_mute;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getMicrophones(
    std::vector<MicrophoneInfo>* _aidl_return) {
    // Todo support this
    /*
    *_aidl_return = getModuleConfig().mMicrophones;
    LOG(DEBUG) << __func__ << ": returning " <<
    ::android::internal::ToString(*_aidl_return);
    */
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Module::updateAudioMode(AudioMode in_mode) {
    // No checks for supported audio modes here, it's an informative
    // notification.
    LOG(DEBUG) << __func__ << ": " << toString(in_mode);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::updateScreenRotation(ScreenRotation in_rotation) {
    LOG(DEBUG) << __func__ << ": " << toString(in_rotation);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::updateScreenState(bool in_isTurnedOn) {
    LOG(DEBUG) << __func__ << ": " << in_isTurnedOn;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getSoundDose(
    std::shared_ptr<ISoundDose>* _aidl_return) {
    if (!mSoundDose) {
        mSoundDose = ndk::SharedRefBase::make<SoundDose>();
    }
    *_aidl_return = mSoundDose.getPtr();
    LOG(DEBUG) << __func__
               << ": returning instance of ISoundDose: " << _aidl_return->get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::generateHwAvSyncId(int32_t* _aidl_return) {
    LOG(DEBUG) << __func__;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

const std::string Module::VendorDebug::kForceTransientBurstName =
    "aosp.forceTransientBurst";
const std::string Module::VendorDebug::kForceSynchronousDrainName =
    "aosp.forceSynchronousDrain";

ndk::ScopedAStatus Module::getVendorParameters(
    const std::vector<std::string>& in_ids,
    std::vector<VendorParameter>* _aidl_return) {
    LOG(DEBUG) << __func__ << ": id count: " << in_ids.size();
    bool allParametersKnown = true;
    for (const auto& id : in_ids) {
        if (id == VendorDebug::kForceTransientBurstName) {
            VendorParameter forceTransientBurst{.id = id};
            forceTransientBurst.ext.setParcelable(
                Boolean{mVendorDebug.forceTransientBurst});
            _aidl_return->push_back(std::move(forceTransientBurst));
        } else if (id == VendorDebug::kForceSynchronousDrainName) {
            VendorParameter forceSynchronousDrain{.id = id};
            forceSynchronousDrain.ext.setParcelable(
                Boolean{mVendorDebug.forceSynchronousDrain});
            _aidl_return->push_back(std::move(forceSynchronousDrain));
        } else {
            allParametersKnown = false;
            LOG(ERROR) << __func__ << ": unrecognized parameter \"" << id
                       << "\"";
        }
    }
    if (allParametersKnown) return ndk::ScopedAStatus::ok();
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

namespace {

template <typename W>
bool extractParameter(const VendorParameter& p, decltype(W::value)* v) {
    std::optional<W> value;
    binder_status_t result = p.ext.getParcelable(&value);
    if (result == STATUS_OK && value.has_value()) {
        *v = value.value().value;
        return true;
    }
    LOG(ERROR) << __func__ << ": failed to read the value of the parameter \""
               << p.id << "\": " << result;
    return false;
}

}  // namespace

ndk::ScopedAStatus Module::setVendorParameters(
    const std::vector<VendorParameter>& in_parameters, bool in_async) {
    LOG(DEBUG) << __func__ << ": parameter count " << in_parameters.size()
               << ", async: " << in_async;
    bool allParametersKnown = true;
    for (const auto& p : in_parameters) {
        if (p.id == VendorDebug::kForceTransientBurstName) {
            if (!extractParameter<Boolean>(p,
                                           &mVendorDebug.forceTransientBurst)) {
                return ndk::ScopedAStatus::fromExceptionCode(
                    EX_ILLEGAL_ARGUMENT);
            }
        } else if (p.id == VendorDebug::kForceSynchronousDrainName) {
            if (!extractParameter<Boolean>(
                    p, &mVendorDebug.forceSynchronousDrain)) {
                return ndk::ScopedAStatus::fromExceptionCode(
                    EX_ILLEGAL_ARGUMENT);
            }
        } else {
            allParametersKnown = false;
            LOG(ERROR) << __func__ << ": unrecognized parameter \"" << p.id
                       << "\"";
        }
    }
    if (allParametersKnown) return ndk::ScopedAStatus::ok();
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus Module::addDeviceEffect(
    int32_t in_portConfigId,
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    if (in_effect == nullptr) {
        LOG(DEBUG) << __func__ << ": port id " << in_portConfigId
                   << ", null effect";
    } else {
        LOG(DEBUG) << __func__ << ": port id " << in_portConfigId
                   << ", effect Binder " << in_effect->asBinder().get();
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Module::removeDeviceEffect(
    int32_t in_portConfigId,
    const std::shared_ptr<::aidl::android::hardware::audio::effect::IEffect>&
        in_effect) {
    if (in_effect == nullptr) {
        LOG(DEBUG) << __func__ << ": port id " << in_portConfigId
                   << ", null effect";
    } else {
        LOG(DEBUG) << __func__ << ": port id " << in_portConfigId
                   << ", effect Binder " << in_effect->asBinder().get();
    }
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Module::getMmapPolicyInfos(
    AudioMMapPolicyType mmapPolicyType,
    std::vector<AudioMMapPolicyInfo>* _aidl_return) {
    LOG(DEBUG) << __func__ << ": mmap policy type " << toString(mmapPolicyType);
    std::set<int32_t> mmapSinks;
    std::set<int32_t> mmapSources;
    auto& ports = getModuleConfig().mPorts;
    for (const auto& port : ports) {
        if (port.flags.getTag() == AudioIoFlags::Tag::input &&
            isBitPositionFlagSet(port.flags.get<AudioIoFlags::Tag::input>(),
                                 AudioInputFlags::MMAP_NOIRQ)) {
            mmapSinks.insert(port.id);
        } else if (port.flags.getTag() == AudioIoFlags::Tag::output &&
                   isBitPositionFlagSet(
                       port.flags.get<AudioIoFlags::Tag::output>(),
                       AudioOutputFlags::MMAP_NOIRQ)) {
            mmapSources.insert(port.id);
        }
    }
    for (const auto& route : getModuleConfig().mRoutes) {
        if (mmapSinks.count(route.sinkPortId) != 0) {
            // The sink is a mix port, add the sources if they are device ports.
            for (int sourcePortId : route.sourcePortIds) {
                if (!ports.contains(sourcePortId)) {
                    // This must not happen
                    LOG(ERROR) << __func__ << ": port id " << sourcePortId
                               << " cannot be found";
                    continue;
                }
                auto sourcePort = ports.at(sourcePortId);
                if (sourcePort.ext.getTag() != AudioPortExt::Tag::device) {
                    // The source is not a device port, skip
                    continue;
                }
                AudioMMapPolicyInfo policyInfo;
                policyInfo.device =
                    sourcePort.ext.get<AudioPortExt::Tag::device>().device;
                // Always return AudioMMapPolicy.AUTO if the device supports
                // mmap for default implementation.
                policyInfo.mmapPolicy = AudioMMapPolicy::AUTO;
                _aidl_return->push_back(policyInfo);
            }
        } else {
            if (!ports.contains(route.sinkPortId)) {
                // This must not happen
                LOG(ERROR) << __func__ << ": port id " << route.sinkPortId
                           << " cannot be found";
                continue;
            }
            auto sinkPort = ports.at(route.sinkPortId);
            if (sinkPort.ext.getTag() != AudioPortExt::Tag::device) {
                // The sink is not a device port, skip
                continue;
            }
            if (count_any(mmapSources, route.sourcePortIds)) {
                AudioMMapPolicyInfo policyInfo;
                policyInfo.device =
                    sinkPort.ext.get<AudioPortExt::Tag::device>().device;
                // Always return AudioMMapPolicy.AUTO if the device supports
                // mmap for default implementation.
                policyInfo.mmapPolicy = AudioMMapPolicy::AUTO;
                _aidl_return->push_back(policyInfo);
            }
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::supportsVariableLatency(bool* _aidl_return) {
    LOG(DEBUG) << __func__;
    *_aidl_return = false;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAAudioMixerBurstCount(int32_t* _aidl_return) {
    if (!isMmapSupported()) {
        LOG(DEBUG) << __func__ << ": mmap is not supported ";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    *_aidl_return = DEFAULT_AAUDIO_MIXER_BURST_COUNT;
    LOG(DEBUG) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::getAAudioHardwareBurstMinUsec(
    int32_t* _aidl_return) {
    if (!isMmapSupported()) {
        LOG(DEBUG) << __func__ << ": mmap is not supported ";
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }
    *_aidl_return = DEFAULT_AAUDIO_HARDWARE_BURST_MIN_DURATION_US;
    LOG(DEBUG) << __func__ << ": returning " << *_aidl_return;
    return ndk::ScopedAStatus::ok();
}

bool Module::isMmapSupported() {
    if (mIsMmapSupported.has_value()) {
        return mIsMmapSupported.value();
    }
    std::vector<AudioMMapPolicyInfo> mmapPolicyInfos;
    if (!getMmapPolicyInfos(AudioMMapPolicyType::DEFAULT, &mmapPolicyInfos)
             .isOk()) {
        mIsMmapSupported = false;
    } else {
        mIsMmapSupported =
            std::find_if(mmapPolicyInfos.begin(), mmapPolicyInfos.end(),
                         [](const auto& info) {
                             return info.mmapPolicy == AudioMMapPolicy::AUTO ||
                                    info.mmapPolicy == AudioMMapPolicy::ALWAYS;
                         }) != mmapPolicyInfos.end();
    }
    return mIsMmapSupported.value();
}

ndk::ScopedAStatus Module::populateConnectedDevicePort(
    AudioPort* audioPort __unused) {
    LOG(DEBUG) << __func__ << ": do nothing and return ok";
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Module::checkAudioPatchEndpointsMatch(
    const std::vector<const AudioPortConfig*>& sources __unused,
    const std::vector<const AudioPortConfig*>& sinks __unused) {
    LOG(DEBUG) << __func__ << ": do nothing and return ok";
    return ndk::ScopedAStatus::ok();
}

binder_status_t Module::dump(int fd, const char** args, uint32_t numArgs) {
    dumpInternal(fd);
    return STATUS_OK;
}

int32_t Module::dumpInternal(const int fd) {
    std::ostringstream os;

    // start write the objects data
    os << "--Module dump start--" << std::endl;

    os << "--Module dump end--" << std::endl;
    // end

    const auto& dumpInfo = os.str();

    if (fd > 0) {  // write to file
        auto dumpInfoSize = dumpInfo.size();
        // TODO remove fd and add fstream support
        auto b = ::write(fd, dumpInfo.c_str(), dumpInfoSize);
        if (b != dumpInfoSize) {
            LOG(ERROR) << __func__ << ": dump failed";
            return -EIO;
        }
    } else {  // Log here
        LOG(INFO) << dumpInfo.c_str();
    }

    // member objects dump here
    int32_t status = 0;
    status = !status ? getModuleConfig().dump(fd) : status;
    status = !status ? mPlatformModule->dump(fd) : status;

    return status;
}

}  // namespace qti::audio::core
