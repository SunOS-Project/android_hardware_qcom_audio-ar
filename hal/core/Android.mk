LOCAL_PATH := $(call my-dir)
CURRENT_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE            := libaudiocorehal.qti
LOCAL_VENDOR_MODULE     := true
LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_C_INCLUDES    :=  $(LOCAL_PATH)/include \
                        $(LOCAL_PATH)/adapter/include

# LOCAL_CFLAGS += -Wall -Wextra -Werror -Wthread-safety

# LOCAL_CPPFLAGS += -fexceptions

LOCAL_CFLAGS += -DPAL_HIDL_ENABLED
LOCAL_CFLAGS += -DAGM_HIDL_ENABLED

LOCAL_VINTF_FRAGMENTS   := \
    ../../configs/common/manifest_non_qmaa.xml

LOCAL_SRC_FILES := \
    CoreService.cpp \
    Bluetooth.cpp \
    Module.cpp \
    ModulePrimary.cpp \
    ModuleStub.cpp \
    SoundDose.cpp \
    Stream.cpp \
    StreamStub.cpp \
    Telephony.cpp \
    StreamInPrimary.cpp \
    StreamOutPrimary.cpp \
    HalOffloadEffects.cpp

LOCAL_HEADER_LIBRARIES :=  \
    liberror_headers \
    libaudioclient_headers \
    libaudio_system_headers \
    libmedia_helper_headers


#    defaults: [
#        "latest_android_media_audio_common_types_ndk_shared",
#        "latest_android_hardware_audio_core_ndk_shared",
#    ],
# mk equivalent find a way to fix this in mk file // TODO
#    android.media.audio.common.types-V2-ndk \
#    android.hardware.audio.core-V1-ndk

LOCAL_STATIC_LIBRARIES := \
    libaudio_module_config.qti \
    libaudiocore.extension

LOCAL_WHOLE_STATIC_LIBRARIES := \
    libaudioplatform.qti

LOCAL_SHARED_LIBRARIES := \
    libaudioaidlcommon \
    libbase \
    libbinder_ndk \
    libcutils \
    libdl \
    libhidlbase \
    libhardware \
    libfmq \
    libmedia_helper \
    libstagefright_foundation \
    libutils \
    libaudioutils \
    libxml2 \
    av-audio-types-aidl-ndk \
    android.hardware.common-V2-ndk \
    android.hardware.common.fmq-V1-ndk \
    android.media.audio.common.types-V2-ndk \
    android.hardware.audio.core-V1-ndk \
    android.hardware.audio.core.sounddose-V1-ndk \
    libar-pal \
    libaudioserviceexampleimpl \
    qti-audio-types-aidl-V1-ndk

include $(BUILD_SHARED_LIBRARY)

include $(CURRENT_PATH)/extensions/Android.mk
include $(CURRENT_PATH)/platform/Android.mk
