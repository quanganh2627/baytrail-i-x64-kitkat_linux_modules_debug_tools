# This makefile is included from vendor/intel/*/AndroidBoard.mk.
pax=1
.PHONY: pax
pax: build_kernel
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" \
	TARGET_DEVICE="$(TARGET_DEVICE)" \
	TARGET_BOARD_PLATFORM="$(TARGET_BOARD_PLATFORM)" \
	KERNEL_SRC_DIR="$(KERNEL_SRC_DIR)" \
	PRODUCT_OUT="`pwd`/$(PRODUCT_OUT)" \
	vendor/intel/support/kernel-build.sh \
	-M device/intel/debug_tools/vtunedk/src/pax
