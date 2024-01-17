#Audio product definitions 
include vendor/qcom/opensource/audio-hal/primary-hal/configs/audio-generic-modules.mk
PRODUCT_PACKAGES += $(AUDIO_GENERIC_MODULES)

PRODUCT_PACKAGES_DEBUG += $(MM_AUDIO_DBG)

#----------------------------------------------------------------------
# audio specific
#----------------------------------------------------------------------
TARGET_USES_AOSP := false
TARGET_USES_AOSP_FOR_AUDIO := false
ifeq ($(TARGET_USES_QMAA_OVERRIDE_AUDIO), false)
ifeq ($(TARGET_USES_QMAA),true)
AUDIO_USE_STUB_HAL := true
TARGET_USES_AOSP_FOR_AUDIO := true
-include $(TOPDIR)vendor/qcom/opensource/audio-hal/primary-hal/configs/common/default.mk
else
# Audio hal configuration file
-include $(TOPDIR)vendor/qcom/opensource/audio-hal/primary-hal/configs/$(TARGET_BOARD_PLATFORM)/$(TARGET_BOARD_PLATFORM).mk
endif
else
# Audio hal configuration file
-include $(TOPDIR)vendor/qcom/opensource/audio-hal/primary-hal/configs/$(TARGET_BOARD_PLATFORM)/$(TARGET_BOARD_PLATFORM).mk
endif

# Pro Audio feature
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.audio.pro.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.audio.pro.xml

SOONG_CONFIG_qtiaudio_var00 := false
SOONG_CONFIG_qtiaudio_var11 := false
SOONG_CONFIG_qtiaudio_var22 := false

ifneq ($(BUILD_AUDIO_TECHPACK_SOURCE), true)
    SOONG_CONFIG_qtiaudio_var00 := true
    SOONG_CONFIG_qtiaudio_var11 := true
    SOONG_CONFIG_qtiaudio_var22 := true
endif
ifeq (,$(wildcard $(QCPATH)/mm-audio-noship))
    SOONG_CONFIG_qtiaudio_var11 := true
endif
ifeq (,$(wildcard $(QCPATH)/mm-audio))
    SOONG_CONFIG_qtiaudio_var22 := true
endif