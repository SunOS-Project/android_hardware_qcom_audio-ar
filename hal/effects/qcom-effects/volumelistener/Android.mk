LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE:= libvolumelistener
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_RELATIVE_PATH := soundfx
LOCAL_MODULE_OWNER := qti

LOCAL_CLANG             := true
LOCAL_TIDY              := true
LOCAL_CFLAGS            += -v -Wall -Wthread-safety

LOCAL_SRC_FILES:= \
        VolumeListener.cpp \
        VolumeListenerContext.cpp \
        GlobalVolumeListenerSession.cpp

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
    libsystem_headers

include $(BUILD_SHARED_LIBRARY)
