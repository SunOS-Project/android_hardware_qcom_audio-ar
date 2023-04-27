LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE            := libaudiocore.extension
LOCAL_VENDOR_MODULE     := true

LOCAL_C_INCLUDES            := $(LOCAL_PATH)/include

LOCAL_SRC_FILES := \
    AudioExtension.cpp

LOCAL_HEADER_LIBRARIES :=  \
    libaudioclient_headers \
    libmedia_helper_headers \
    libexpectedutils_headers

LOCAL_SHARED_LIBRARIES := \
    libaudioaidlcommon \
    libbase \
    libbinder_ndk \
    libcutils \
    libfmq \
    liblog \
    libmedia_helper \
    libstagefright_foundation \
    libutils \
    libxml2 \
    android.hardware.common-V2-ndk \
    android.hardware.common.fmq-V1-ndk \
    android.media.audio.common.types-V2-ndk \
    android.hardware.audio.core-V1-ndk \
    libar-pal

include $(BUILD_STATIC_LIBRARY)
