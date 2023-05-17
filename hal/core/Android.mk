LOCAL_PATH := $(call my-dir)
CURRENT_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE            := libaudiocorehal.qti
LOCAL_VENDOR_MODULE     := true
LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_C_INCLUDES    := $(LOCAL_PATH)/include

LOCAL_VINTF_FRAGMENTS   := \
    ../../configs/common/manifest_non_qmaa.xml

LOCAL_SRC_FILES := \
    CoreService.cpp \
    aidlservice/Bluetooth.cpp \
    aidlservice/Config.cpp \
    aidlservice/Module.cpp \
    aidlservice/SoundDose.cpp \
    aidlservice/Stream.cpp \
    aidlservice/StreamStub.cpp \
    aidlservice/EngineConfigXmlConverter.cpp \
    aidlservice/Telephony.cpp \
    platform/PlatformStream.cpp \
    platform/PlatformBluetooth.cpp \
    platform/PlatformVoice.cpp

LOCAL_HEADER_LIBRARIES :=  \
    libaudioclient_headers \
    libaudio_system_headers \
    audiohalutils_headers \
    libmedia_helper_headers \
    libexpectedutils_headers

#    defaults: [
#        "latest_android_media_audio_common_types_ndk_shared",
#        "latest_android_hardware_audio_core_ndk_shared",
#    ],
# mk equivalent find a way to fix this in mk file // TODO
#    android.media.audio.common.types-V2-ndk \
#    android.hardware.audio.core-V1-ndk

LOCAL_STATIC_LIBRARIES := libaudiohalutils libaudiocore.extension

LOCAL_SHARED_LIBRARIES := \
    libaudioaidlcommon \
    libbase \
    libbinder_ndk \
    libcutils \
    libdl \
    libhidlbase \
    libfmq \
    liblog \
    libmedia_helper \
    libprocessgroup \
    libstagefright_foundation \
    libutils \
    libxml2 \
    libtinyalsav2 \
    libalsautilsv2 \
    android.hardware.common-V2-ndk \
    android.hardware.common.fmq-V1-ndk \
    android.media.audio.common.types-V2-ndk \
    android.hardware.audio.core-V1-ndk \
    android.hardware.audio.core.sounddose-V1-ndk \
    libar-pal \
    libaudioserviceexampleimpl \
    libaudioplatform.qti

include $(BUILD_SHARED_LIBRARY)

include $(CURRENT_PATH)/extensions/Android.mk
