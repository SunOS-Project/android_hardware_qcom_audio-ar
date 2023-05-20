

# Audio HAL process wide props
AUDIO_HAL_PROP := \
vendor.audio.hal.loglevel=0

# core hal default IModule either aosp or qti
# https://cs.android.com/android/platform/superproject/+/master:hardware/interfaces/audio/aidl/android/hardware/audio/core/IModule.aidl
AUDIO_HAL_PROP += \
vendor.audio.core_hal_IModule_default=aosp

# core hal default IConfig either aosp or qti
# check below
# https://cs.android.com/android/platform/superproject/+/master:hardware/interfaces/audio/aidl/android/hardware/audio/core/IConfig.aidl
AUDIO_HAL_PROP += \
vendor.audio.core_hal_IConfig_default=aosp




PRODUCT_VENDOR_PROPERTIES += \
    $(AUDIO_HAL_PROP)