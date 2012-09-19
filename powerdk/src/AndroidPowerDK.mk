# This makefile is included from vendor/intel/*/AndroidBoard.mk.
powerdk=1
.PHONY: apwr
apwr: build_kernel
	TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" vendor/intel/support/debugtools-build.sh -c $(CUSTOM_BOARD) -M device/intel/debug_tools/powerdk/src/src
