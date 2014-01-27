LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := socwatch1_3_external
LOCAL_MODULE_PATH := $(LOCAL_PATH)/src
include $(BUILD_EXTERNAL_KERNEL_MODULE)
