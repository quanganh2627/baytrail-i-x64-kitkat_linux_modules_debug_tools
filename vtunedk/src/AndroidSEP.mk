# This makefile is included from vendor/intel/*/AndroidBoard.mk.
vtunedk=true
sep_version=sep3_10
.PHONY: sep3_10
sep3_10: build_kernel
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" \
	TARGET_DEVICE="$(TARGET_DEVICE)" \
	TARGET_BOARD_PLATFORM="$(TARGET_BOARD_PLATFORM)" \
	KERNEL_SRC_DIR="$(KERNEL_SRC_DIR)" \
	PRODUCT_OUT="`pwd`/$(PRODUCT_OUT)" \
	vendor/intel/support/kernel-build.sh \
	-M device/intel/debug_tools/vtunedk/src -f "BOARD_HAVE_SMALL_RAM=$(BOARD_HAVE_SMALL_RAM)"
