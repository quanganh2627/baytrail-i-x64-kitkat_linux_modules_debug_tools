# This makefile is included from vendor/intel/*/AndroidBoard.mk.
powerdk=1
.PHONY: apwr
apwr: build_kernel
	TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" vendor/intel/support/kernel-build.sh -c $(CUSTOM_BOARD) -M device/intel/PRIVATE/debug_tools/powerdk/src
