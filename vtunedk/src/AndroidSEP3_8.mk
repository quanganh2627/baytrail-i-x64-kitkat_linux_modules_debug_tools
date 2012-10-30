# This makefile is included from vendor/intel/*/AndroidBoard.mk.
vtunedk=1
.PHONY: sep3_8
sep3_8: build_kernel
	TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" vendor/intel/support/debugtools-build.sh -c $(TARGET_DEVICE) -M device/intel/debug_tools/vtunedk/src
