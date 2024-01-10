ifneq ($(AUDIO_USE_STUB_HAL), true)

MY_LOCAL_PATH := $(call my-dir)

include $(MY_LOCAL_PATH)/hal/Android.mk

endif
