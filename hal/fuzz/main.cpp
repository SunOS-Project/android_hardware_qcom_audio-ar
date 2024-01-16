// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <memory>
#include <string>
#include <vector>
#include <android-base/logging.h>
#include <fuzzer/FuzzedDataProvider.h>

#include "base.h"
#include "provider.h"
#include "gen.h"

#include <qti-audio-core/Module.h>
#include <qti-audio-core/ModulePrimary.h>

using qti::audio::core::Module;
using qti::audio::core::ModulePrimary;

struct AudioDataProvider : public DataProvider {
    AudioDataProvider(const char *data, size_t size): DataProvider(data, size) {}
};

struct AudioStreamCommonFuzzer: public IStreamCommonFuzzer<AudioDataProvider, IStreamCommon> {
    bool closed = false;

    AudioStreamCommonFuzzer(AudioDataProvider *provider, IStreamCommon *stream): IStreamCommonFuzzer(provider, stream) {}

    void fuzz_close() override {
        IStreamCommonFuzzer<AudioDataProvider, IStreamCommon>::fuzz_close();
        closed = true;
    }
};

struct AudioStreamInFuzzer : public IStreamInFuzzer<AudioDataProvider, IStreamIn> {
    AudioStreamInFuzzer(AudioDataProvider *provider, IStreamIn *stream): IStreamInFuzzer(provider, stream) {}

    std::shared_ptr<IStreamCommon> getStreamCommon() {
        std::shared_ptr<IStreamCommon> stream;
        this->target->getStreamCommon(&stream);
        return std::move(stream);
    }
};

struct AudioStreamOutFuzzer : public IStreamOutFuzzer<AudioDataProvider, IStreamOut> {
    AudioStreamOutFuzzer(AudioDataProvider *provider, IStreamOut *stream): IStreamOutFuzzer(provider, stream) {}

    std::shared_ptr<IStreamCommon> getStreamCommon() {
        std::shared_ptr<IStreamCommon> stream;
        this->target->getStreamCommon(&stream);
        return std::move(stream);
    }
};

struct AudioModuleFuzzer : public IModuleFuzzer<AudioDataProvider, Module> {
    AudioModuleFuzzer(AudioDataProvider *provider, Module *mod): IModuleFuzzer(provider, mod) {}

    std::shared_ptr<IStreamIn> streamIn;
    std::shared_ptr<IStreamOut> streamOut;

    IModule::OpenInputStreamReturn fuzz_openInputStream() override {
        auto ret = IModuleFuzzer<AudioDataProvider, Module>::fuzz_openInputStream();
        streamIn = ret.stream;
        return ret;
    }

    IModule::OpenOutputStreamReturn fuzz_openOutputStream() override {
        auto ret = IModuleFuzzer<AudioDataProvider, Module>::fuzz_openOutputStream();
        streamOut = ret.stream;
        return ret;
    }

    // avoid assertion in connectExternalDevice call in get<Tag::device>()
    AudioPort fuzz_connectExternalDevice() override {
        AudioPort in_templateIdAndAdditionalData;
        this->provider->genAudioPort(in_templateIdAndAdditionalData);

        AudioPort _aidl_return;
        if (in_templateIdAndAdditionalData.ext.getTag() == AudioPortExt::Tag::device) {
            this->target->connectExternalDevice(in_templateIdAndAdditionalData, &_aidl_return);
        }
        return std::move(_aidl_return);
  }
};

template <typename T, typename F>
void fuzzStream(AudioDataProvider *provider, std::shared_ptr<T> &stream) {
    if (stream) {
        auto fuzzer = F(provider, stream.get());
        fuzzer.fuzz();

        std::shared_ptr<IStreamCommon> common = fuzzer.getStreamCommon();
        AudioStreamCommonFuzzer commonFuzzer(provider, common.get());
        commonFuzzer.fuzz();
        if (commonFuzzer.closed) {
            commonFuzzer.closed = false;
            stream = nullptr;
        }
    }
}

extern "C" binder_status_t registerService(void);

class AudioFuzzer {
public:
    void setProvider(AudioDataProvider *provider) {
        this->provider = provider;
    }

    void fuzz() {
        if (!modulePrimary) {
            // register agm service
            registerService();
            modulePrimary = ndk::SharedRefBase::make<ModulePrimary>();
        }
        moduleFuzzer = std::make_unique<AudioModuleFuzzer>(provider, modulePrimary.get());
        moduleFuzzer->fuzz();

        fuzzStream<IStreamIn, AudioStreamInFuzzer>(provider, moduleFuzzer->streamIn);
        fuzzStream<IStreamOut, AudioStreamOutFuzzer>(provider, moduleFuzzer->streamOut);
    }

private:
    AudioDataProvider *provider = nullptr;
    std::shared_ptr<Module> modulePrimary = nullptr;
    std::unique_ptr<AudioModuleFuzzer> moduleFuzzer = nullptr;
};

extern "C" int LLVMFuzzerTestOneInput(const char *data, size_t size) {
    static AudioFuzzer fuzzer;

    AudioDataProvider provider(data, size);
    fuzzer.setProvider(&provider);

    fuzzer.fuzz();
    return 0;
}
