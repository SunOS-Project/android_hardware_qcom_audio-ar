LOCAL_PATH := $(call my-dir)

LOCAL_AUDIO_SERVICE_64 := taro kalama anorak pineapple pitti volcano

include $(CLEAR_VARS)

LOCAL_MODULE := libaudio_hal_headers
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/inc

LOCAL_VENDOR_MODULE := true

include $(BUILD_HEADER_LIBRARY)

ifneq ($(TARGET_PROVIDES_AUDIO_HAL),true)
include $(CLEAR_VARS)

ifeq ($(call is-board-platform-in-list,$(LOCAL_AUDIO_SERVICE_64)), true)
ifneq ($(TARGET_BOARD_SUFFIX), _32go)
LOCAL_MODULE       := android.hardware.audio.service_64.rc
else
LOCAL_MODULE       := android.hardware.audio.service.rc
endif
endif

LOCAL_SRC_FILES    := $(LOCAL_MODULE)
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR_ETC)/init
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)

LOCAL_MODULE := audio.primary.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := qti
LOCAL_VENDOR_MODULE := true
LOCAL_ARM_MODE := arm

LOCAL_VINTF_FRAGMENTS := ../configs/common/manifest_non_qmaa.xml

ifeq ($(SOONG_CONFIG_android_hardware_audio_run_64bit), true)
LOCAL_MULTILIB := 64
endif

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_LSM_HIDL)),true)
LOCAL_VINTF_FRAGMENTS += ../configs/common/manifest_non_qmaa_extn.xml
endif

LOCAL_CFLAGS += -Wno-macro-redefined
LOCAL_CFLAGS += -DSOUND_TRIGGER_PLATFORM_NAME=$(TARGET_BOARD_PLATFORM)
LOCAL_CFLAGS += -D_GNU_SOURCE
LOCAL_CFLAGS += -Wall -Werror
LOCAL_CFLAGS += -Wno-unused-variable
LOCAL_CFLAGS += -Wno-format
LOCAL_CFLAGS += -Wno-sign-compare
LOCAL_CFLAGS += -Wno-unused-parameter
LOCAL_CFLAGS += -Wno-unused-label
LOCAL_CFLAGS += -Wno-gnu-designator
LOCAL_CFLAGS += -Wno-typedef-redefinition
LOCAL_CFLAGS += -Wno-shorten-64-to-32
LOCAL_CFLAGS += -Wno-tautological-compare
LOCAL_CFLAGS += -Wno-unused-function
LOCAL_CFLAGS += -Wno-unused-local-typedef
ifeq ($(filter 12 S, $(PLATFORM_VERSION)),)
LOCAL_CFLAGS += -DUSEHIDL7_1
endif

LOCAL_CPPFLAGS += -fexceptions

LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/inc \
    system/media/audio_utils/include \
    external/expat/lib \
    $(call include-path-for, audio-effects)

LOCAL_SRC_FILES := \
    AudioStream.cpp \
    AudioDevice.cpp \
    AudioVoice.cpp \
    audio_extn/soundtrigger.cpp \
    audio_extn/Gain.cpp \
    audio_extn/AudioExtn.cpp

LOCAL_HEADER_LIBRARIES := \
    libhardware_headers \
    qti_audio_kernel_uapi \
    libagm_headers \
    libaudio_extn_headers \
    libagmclient_headers
ifeq ($(QCPATH),)
LOCAL_HEADER_LIBRARIES += libarpal_headers
endif

LOCAL_SHARED_LIBRARIES := \
    libbase \
    liblog \
    libcutils \
    libdl \
    libaudioutils \
    libexpat \
    libhidlbase \
    libprocessgroup \
    libfmq \
    libutils \
    libar-pal \
    android.hidl.allocator@1.0 \
    android.hidl.memory@1.0 \
    libhidlmemory

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_PAL_HIDL)),true)
  LOCAL_SHARED_LIBRARIES += \
    vendor.qti.hardware.pal@1.0-impl \
    vendor.qti.hardware.pal@1.0

  LOCAL_CFLAGS += -DPAL_HIDL_ENABLED
endif

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_AGM_HIDL)),true)
  LOCAL_SHARED_LIBRARIES += \
    vendor.qti.hardware.AGMIPC@1.0-impl \
    vendor.qti.hardware.AGMIPC@1.0 \
    libagm

  LOCAL_CFLAGS += -DAGM_HIDL_ENABLED
endif

ifeq ($(strip $(AUDIO_FEATURE_ENABLED_GEF_SUPPORT)),true)
    LOCAL_CFLAGS += -DAUDIO_GENERIC_EFFECT_FRAMEWORK_ENABLED
ifeq ($(strip $(AUDIO_FEATURE_ENABLED_INSTANCE_ID)), true)
    LOCAL_CFLAGS += -DINSTANCE_ID_ENABLED
endif
    LOCAL_SRC_FILES += audio_extn/Gef.cpp
endif

include $(BUILD_SHARED_LIBRARY)
endif
