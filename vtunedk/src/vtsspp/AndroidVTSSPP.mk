# This makefile is included from vendor/intel/*/AndroidBoard.mk.

.PHONY: vtsspp
vtsspp: build_kernel
	TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" vendor/intel/support/debugtools-build.sh -c $(TARGET_DEVICE) -M device/intel/debug_tools/vtunedk/src/vtsspp
