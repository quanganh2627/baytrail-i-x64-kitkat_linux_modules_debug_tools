# This makefile is included from vendor/intel/*/AndroidBoard.mk.
vtunedk=1
.PHONY: sep3_8
sep3_8: build_kernel
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" \
	TARGET_DEVICE="$(TARGET_DEVICE)" \
	TARGET_BOARD_PLATFORM="$(TARGET_BOARD_PLATFORM)" \
	KERNEL_SRC_DIR="$(KERNEL_SRC_DIR)" \
	vendor/intel/support/kernel-build.sh \
	-M device/intel/debug_tools/vtunedk/src -f "BOARD_HAVE_SMALL_RAM=$(BOARD_HAVE_SMALL_RAM)"
