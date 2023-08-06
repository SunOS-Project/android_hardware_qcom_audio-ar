

# Audio HAL process wide props
AUDIO_HAL_PROP := \
vendor.audio.hal.loglevel=0

# core hal default IModule either aosp or qti
# https://cs.android.com/android/platform/superproject/+/master:hardware/interfaces/audio/aidl/android/hardware/audio/core/IModule.aidl
AUDIO_HAL_PROP += \
vendor.audio.core_hal_IModule_default=qti

# core hal default IConfig either aosp or qti
# check below
# https://cs.android.com/android/platform/superproject/+/master:hardware/interfaces/audio/aidl/android/hardware/audio/core/IConfig.aidl
AUDIO_HAL_PROP += \
vendor.audio.core_hal_IConfig_default=aosp

# spf hdr record either true or false
AUDIO_HAL_PROP += \
vendor.audio.hdr.spf.record.enable=false

# spf hdr record either true or false
AUDIO_HAL_PROP += \
vendor.audio.hdr.record.enable=false

#compress offload
AUDIO_HAL_PROP += \
vendor.audio.offload.buffer.size.kb=32

#compress offload
AUDIO_HAL_PROP += \
audio.offload.disable=1

# compress capture feature related
AUDIO_HAL_PROP += \
vendor.audio.compress_capture.enabled=false \
vendor.audio.compress_capture.aac=false
# compress capture end


PRODUCT_VENDOR_PROPERTIES += \
    $(AUDIO_HAL_PROP)