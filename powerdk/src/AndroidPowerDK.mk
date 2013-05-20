# This makefile is included from vendor/intel/*/AndroidBoard.mk.
ifneq ($(TARGET_KERNEL_SOURCE_IS_PRESENT),false)
$(eval $(call build_kernel_module,$(call my-dir),apwr))
endif

