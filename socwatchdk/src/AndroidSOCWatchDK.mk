# This makefile is included from vendor/intel/*/AndroidBoard.mk.
ifneq ($(TARGET_KERNEL_SOURCE_IS_PRESENT),false)
.PHONY: swdrv
swdrv: build_kernel
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" \
	TARGET_DEVICE="$(TARGET_DEVICE)" \
	TARGET_BOARD_PLATFORM="$(TARGET_BOARD_PLATFORM)" \
	KERNEL_SRC_DIR="$(KERNEL_SRC_DIR)" \
	vendor/intel/support/kernel-build.sh \
	-M device/intel/debug_tools/socwatchdk/src

$(PRODUCT_OUT)/ramdisk.img : swdrv
endif
