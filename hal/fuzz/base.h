// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#pragma once

#include <memory>
#include <qti-audio-core/Module.h>
#include <qti-audio-core/ModulePrimary.h>

using qti::audio::core::Module;
using qti::audio::core::ModulePrimary;

template <typename P, typename T>
class FuzzerBase {
protected:
    P *provider = nullptr;
    T *target = nullptr;

public:
    FuzzerBase() {}
    FuzzerBase(P *provider, T *target): provider(provider), target(target) {}
    ~FuzzerBase() {}
};
