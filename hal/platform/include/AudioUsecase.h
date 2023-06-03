/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#pragma once

#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>
#include <sstream>
#include <variant>

#include <cutils/properties.h>

#include <PalDefs.h>

#ifndef __BIONIC__
#define __assert2(a, b, c, d) ((void)0)
#endif

namespace qti::audio::core {


class EmptyUsecase {
   public:
    virtual ~EmptyUsecase() = default;
    virtual std::string toString() const { return std::string("INVALID"); }
};

class DeepBufferPlayback : public EmptyUsecase {
   public:
    constexpr static uint32_t kPeriodDurationMs = 40;
    constexpr static uint32_t kPeriodSize = 1920;
    constexpr static uint32_t kPeriodCount = 2;
    constexpr static uint32_t kPlatformDelayUs = (29 * 1000LL);

    // bool open();

    virtual std::string toString() const {
        std::ostringstream os;
        os << " period duration ms: " << std::to_string(kPeriodDurationMs);
        os << " period count: " << std::to_string(kPeriodCount);
        os << " platform delay Us: " << std::to_string(kPlatformDelayUs);
        return os.str();
    }
};

class PCMRecord : public EmptyUsecase {
    private:
     explicit PCMRecord() {}

    public:
     constexpr static uint32_t kCaptureDurationMs = 20;


    virtual std::string toString() const {
        std::ostringstream os;
        os << " single read duration:" << std::to_string(kCaptureDurationMs);
        return os.str();
    }
};

class CompressPlayback : public EmptyUsecase {
   public:
    virtual std::string toString() const {
        return std::string("CompressPlayback");
    }
};

class CompressRecord : public EmptyUsecase {
   public:
    virtual std::string toString() const {
        return std::string("CompressPlayback");
    }
};

class LowLatencyPlayback : public EmptyUsecase {
   public:
    constexpr static uint32_t kPeriodCount = 2;
    constexpr static uint32_t kPlatformDelayUs = (13 * 1000LL);
    virtual std::string toString() const {
        return std::string("CompressPlayback");
    }
};

class AudioUsecase {
    /*
    The variant of the AudioUsecase is decided by stream's portconfig.
    */
   public:
    enum class Tag : uint16_t {
        INVALID = 0,  // empty usecase
        DEEPBUFFER_PLAYBACK,
        PCM_RECORD,
        COMPRESS_PLAYBACK,
        COMPRESS_RECORD,
        LOW_LATENCY_PLAYBACK,
    };

   private:
    std::variant<EmptyUsecase, DeepBufferPlayback, PCMRecord, CompressPlayback,
                 CompressRecord, LowLatencyPlayback>
        mValue;

   public:
    template <typename _Tp>
    static constexpr bool _not_self =
        !std::is_same_v<std::remove_cv_t<std::remove_reference_t<_Tp>>,
                        AudioUsecase>;
    AudioUsecase()
        : mValue(std::in_place_index<static_cast<size_t>(Tag::INVALID)>,
                 EmptyUsecase()) {}

    template <typename _Tp, typename = std::enable_if_t<_not_self<_Tp>>>
    constexpr AudioUsecase(_Tp&& _arg) : mValue(std::forward<_Tp>(_arg)) {}

    template <size_t _Np, typename... _Tp>
    constexpr explicit AudioUsecase(std::in_place_index_t<_Np>, _Tp&&... _args)
        : mValue(std::in_place_index<_Np>, std::forward<_Tp>(_args)...) {}

    template <Tag _tag, typename... _Tp>
    static AudioUsecase make(_Tp&&... _args) {
        return AudioUsecase(std::in_place_index<static_cast<size_t>(_tag)>,
                            std::forward<_Tp>(_args)...);
    }

    template <Tag _tag, typename _Tp, typename... _Up>
    static AudioUsecase make(std::initializer_list<_Tp> _il, _Up&&... _args) {
        return AudioUsecase(std::in_place_index<static_cast<size_t>(_tag)>,
                            std::move(_il), std::forward<_Up>(_args)...);
    }
    Tag getTag() const { return static_cast<Tag>(mValue.index()); }

    template <Tag _tag>
    const auto& get() const {
        if (getTag() != _tag) {
            __assert2(__FILE__, __LINE__, __PRETTY_FUNCTION__,
                      "bad access: a wrong tag");
        }
        return std::get<static_cast<size_t>(_tag)>(mValue);
    }

    template <Tag _tag>
    auto& get() {
        if (getTag() != _tag) {
            __assert2(__FILE__, __LINE__, __PRETTY_FUNCTION__,
                      "bad access: a wrong tag");
        }
        return std::get<static_cast<size_t>(_tag)>(mValue);
    }

    template <Tag _tag, typename... _Tp>
    void set(_Tp&&... _args) {
        mValue.emplace<static_cast<size_t>(_tag)>(std::forward<_Tp>(_args)...);
    }

    inline std::string toString() const;
};

inline std::string AudioUsecase::toString() const {
    std::ostringstream os;
    os << "AudioUsecase{";
    switch (getTag()) {
        case Tag::INVALID:
            os << get<Tag::INVALID>().toString();
            break;
        case Tag::DEEPBUFFER_PLAYBACK:
            os << get<Tag::DEEPBUFFER_PLAYBACK>().toString();
            break;
        case Tag::PCM_RECORD:
            os << get<Tag::PCM_RECORD>().toString();
            break;
        case Tag::COMPRESS_PLAYBACK:
            os << get<Tag::COMPRESS_PLAYBACK>().toString();
            break;
        case Tag::COMPRESS_RECORD:
            os << get<Tag::COMPRESS_RECORD>().toString();
            break;
        case Tag::LOW_LATENCY_PLAYBACK:
            os << get<Tag::LOW_LATENCY_PLAYBACK>().toString();
            break;
        default:
            break;
    }
    os << "}";
    return os.str();
}
}  // namespace qti::audio::core