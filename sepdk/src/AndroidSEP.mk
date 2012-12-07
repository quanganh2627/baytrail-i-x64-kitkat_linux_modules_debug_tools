# This makefile is included from vendor/intel/*/AndroidBoard.mk.
sepdk=1
.PHONY: sep
sep: build_kernel
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" \
	TARGET_DEVICE="$(TARGET_DEVICE)" \
	TARGET_BOARD_PLATFORM="$(TARGET_BOARD_PLATFORM)" \
	vendor/intel/support/kernel-build.sh \
	-M device/intel/debug_tools/sepdk/src -f "BOARD_HAVE_SMALL_RAM=$(BOARD_HAVE_SMALL_RAM)"
