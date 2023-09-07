LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE:= libqcompostprocbundle
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_RELATIVE_PATH := soundfx
LOCAL_MODULE_OWNER := qti

LOCAL_C_FLAGS += -Werror -Wall -Wextra

LOCAL_SRC_FILES:= \
        OffloadBundleAidl.cpp \
        OffloadBundleContext.cpp \
        BassBoostContext.cpp \
        EqualizerContext.cpp \
        ReverbContext.cpp \
        VirtualizerContext.cpp \
        ParamDelegator.cpp

LOCAL_STATIC_LIBRARIES := libaudioeffecthal_base_impl_static

LOCAL_SHARED_LIBRARIES:= \
    libaudioaidlcommon \
    libbase \
    libbinder_ndk \
    libcutils \
    libfmq \
    liblog \
    libutils \
    android.hardware.common-V2-ndk \
    android.hardware.common.fmq-V1-ndk \
    android.hardware.audio.effect-V1-ndk \
    android.media.audio.common.types-V2-ndk \
    libar-pal

LOCAL_HEADER_LIBRARIES:= \
    libaudioeffectsaidlqti_headers \
    libaudio_system_headers \
    libsystem_headers \
    libacdb_headers

include $(BUILD_SHARED_LIBRARY)
