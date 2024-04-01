// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <optional>
#include <string>
#include <vector>
#include <fuzzer/FuzzedDataProvider.h>

#include <aidl/android/media/audio/common/Int.h>

class DataProviderBase {
protected:
    FuzzedDataProvider p;

public:
    DataProviderBase(const char *data, size_t size):
            p(reinterpret_cast<const uint8_t *>(data), size) {}
    virtual ~DataProviderBase() {}

    FuzzedDataProvider &inner() { return p; }

    template <typename T>
    T pick(const T *begin, const T *end) {
        return *(begin + p.ConsumeIntegralInRange<size_t>(0, end - begin - 1));
    }

    // primitive types
    void gen(bool &out) { out = p.ConsumeBool(); }
    void gen(uint8_t &out) { out = p.ConsumeIntegral<uint8_t>(); }
    void gen(float &out) { out = p.ConsumeFloatingPoint<float>(); }
    void gen(double &out) { out = p.ConsumeFloatingPoint<double>(); }
    void gen(int8_t &out) { out = genSignedInteger<uint8_t, int8_t>(); }
    void gen(char16_t &out) { out = genSignedInteger<uint16_t, char16_t>(); }
    void gen(int32_t &out) { out = genSignedInteger<uint32_t, int32_t>(); }
    void gen(int64_t &out) { out = genSignedInteger<uint64_t, int64_t>(); }

    void genInt(aidl::android::media::audio::common::Int &out) {
        out.value = genSignedInteger<uint32_t, int32_t>();
    }

    void genFloat(aidl::android::media::audio::common::Float &out) {
        out.value = p.ConsumeFloatingPoint<float>();
    }

    void genBoolean(aidl::android::media::audio::common::Boolean &out) {
        out.value = p.ConsumeBool();
    }

    void gen(std::string &out) {
        std::string s;
        int size = p.ConsumeIntegralInRange<int>(0, 16);
        for (int i = 0; i < size; ++i) {
            char ch = 'a' + p.ConsumeIntegralInRange<int>(0, 26);
            out.push_back(ch);
        }
        out = s;
    }

    template <typename T, typename F>
    void gen(std::vector<T> &out, F f, int max = 4) {
        std::vector<T> v;
        int size = p.ConsumeIntegralInRange<int>(0, max);
        v.resize(size);
        for (auto &val : v) {
            f(val);
        }
        out = v;
    }

    template <typename T, typename F>
    void gen(std::optional<T> &out, F f) {
        bool v = p.ConsumeBool();
        if (v) {
            T val;
            f(val);
            out = std::optional<T>(val);
        }
    }

private:
    // to avoid ubsan error like:
    // FuzzedDataProvider.h:212:47: runtime error: unsigned integer overflow:
    // 2147483647 - 18446744071562067968 cannot be represented in type 'uint64_t'
    template <typename U, typename I>
    I genSignedInteger() {
        union {
            I i;
            U u;
        } val;
        val.u = p.ConsumeIntegral<U>();
        return val.i;
    }
};
