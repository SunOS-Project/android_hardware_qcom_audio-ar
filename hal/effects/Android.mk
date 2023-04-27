CURRENT_PATH := $(call my-dir)
ifeq (1, 0)
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE            := libaudioeffecthal.qti
LOCAL_VENDOR_MODULE     := true
LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_C_INCLUDES            := $(LOCAL_PATH)/include

LOCAL_VINTF_FRAGMENTS      := audioeffectservice_qti.xml

LOCAL_SRC_FILES := \
    EffectFactory.cpp \
    EffectMain.cpp

#    defaults: [
#        "latest_android_media_audio_common_types_ndk_shared",
#        "latest_android_hardware_audio_core_ndk_shared",
#    ],
# mk equivalent find a way to fix this in mk file // TODO
#    android.media.audio.common.types-V2-ndk \
#    android.hardware.audio.core-V1-ndk

LOCAL_SHARED_LIBRARIES := \
    libaudioaidlcommon \
    libbase \
    libbinder_ndk \
    libcutils \
    libfmq \
    libstagefright_foundation \
    libutils \
    libtinyxml2 \
    android.hardware.audio.effect-V1-ndk \
    android.media.audio.common.types-V2-ndk

LOCAL_HEADER_LIBRARIES := \
    libaudioaidl_headers \
    libaudio_system_headers \
    libsystem_headers

include $(BUILD_SHARED_LIBRARY)

#include $(CURRENT_PATH)/qcom-effects/Android.mk
endif
include $(CURRENT_PATH)/qcom-effects/Android.mk