# This makefile is included from vendor/intel/*/AndroidBoard.mk.
pax=1
$(eval $(call build_kernel_module,$(call my-dir),pax))

