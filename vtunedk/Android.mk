LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := sep3_10
LOCAL_MODULE_PATH := $(LOCAL_PATH)/sepdk
include $(BUILD_EXTERNAL_KERNEL_MODULE)

include $(CLEAR_VARS)
LOCAL_MODULE := pax
# As the sepdk/pax directory is copied to a new location, some
# extra include directives are needed to tell the compiler
# where to find the include directories.
LOCAL_C_INCLUDES := $(addprefix $(LOCAL_PATH)/sepdk/,include inc)
LOCAL_MODULE_PATH := $(LOCAL_PATH)/sepdk/pax
include $(BUILD_EXTERNAL_KERNEL_MODULE)

include $(CLEAR_VARS)
LOCAL_MODULE := vtsspp
LOCAL_MODULE_PATH := $(LOCAL_PATH)/sepdk/src/vtsspp
include $(BUILD_EXTERNAL_KERNEL_MODULE)


