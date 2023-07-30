LOCAL_PATH := $(call my-dir)
CURRENT_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE  := libaudioplatform.qti
LOCAL_MODULE_OWNER  := qti
LOCAL_MODULE_TAGS   := optional
LOCAL_VENDOR_MODULE := true

LOCAL_C_INCLUDES    := $(LOCAL_PATH)/include
LOCAL_EXPORT_C_INCLUDE_DIRS   := $(LOCAL_PATH)/include

LOCAL_SRC_FILES := \
    PlatformConverter.cpp \
    Platform.cpp \
    AudioUsecase.cpp \
    AudioUsecaseExt.cpp

LOCAL_SHARED_LIBRARIES := \
    libbase \
    libstagefright_foundation \
    android.hardware.audio.core-V1-ndk \
    android.media.audio.common.types-V2-ndk \
    libaudioaidlcommon \
    libar-pal

include $(BUILD_STATIC_LIBRARY)
