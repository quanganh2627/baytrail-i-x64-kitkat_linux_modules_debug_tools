LOCAL_PATH:=$(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES:= \
	src/ksym_extractor.cpp \
	src/main.cpp \
	src/pw_bt.cpp \
	src/pw_utils.cpp \
	src/pw_parser.cpp \
	src/wulib.cpp   \
	src/ht_wudump.cpp   \
	src/wuwatch.cpp

LOCAL_CFLAGS += -D_ANDROID_=1 -D_NDK_BUILD_=1 -DIS_INTEL_INTERNAL=1 -UNDEBUG

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/inc \
	$(LOCAL_PATH)/../powerdk/include
		
LOCAL_MODULE := wuwatch
LOCAL_MODULE_TAGS := eng debug

LOCAL_SHARED_LIBRARIES := libstlport

# Including this will modify the include path
include external/stlport/libstlport.mk

include $(BUILD_EXECUTABLE)
