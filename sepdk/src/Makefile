#
# Version: 1.3
#
# Copyright (C) 2008-2011 Intel Corporation.  All Rights Reserved.
#
#     This file is part of SEP Development Kit
#
#     SEP Development Kit is free software; you can redistribute it
#     and/or modify it under the terms of the GNU General Public License
#     version 2 as published by the Free Software Foundation.
#
#     SEP Development Kit is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with SEP Development Kit; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
#     As a special exception, you may use this file as part of a free software
#     library without restriction.  Specifically, if other files instantiate
#     templates or use macros or inline functions from this file, or you compile
#     this file and link it with other files to produce an executable, this
#     file does not by itself cause the resulting executable to be covered by
#     the GNU General Public License.  This exception does not however
#     invalidate any other reasons why the executable file might be covered by
#     the GNU General Public License.
#

# -------------------- user configurable options ------------------------

# base name of SEP driver
DRIVER_NAME = sep3_7

# location to install driver
INSTALL = .

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
#EXTRA_CFLAGS += -I$(LDDINCDIR) -I$(LDDINCDIR1)
EXTRA_CFLAGS += -I$(M)/../include -I$(M)/inc

# if ARCH variable is set, unset it to avoid conflicts with kbuild
unexport ARCH

# platform details
#MACH := $(shell uname -m)
MACH := i686

ifeq ($(MACH),x86_64)
PLATFORM=x32_64
BUILD_CHIPSET=YES
BUILD_GFX=YES
endif
ifeq ($(MACH),ia64)
PLATFORM=x64
BUILD_CHIPSET=NO
BUILD_GFX=NO
endif
ifeq ($(MACH),i686)
PLATFORM=x32
BUILD_CHIPSET=NO
BUILD_GFX=NO
endif
#KERNEL_VERSION ?= $(shell uname -r)
#SMP := $(shell uname -v | grep SMP)
SMP := SMP
ARITY=up
ifneq ($(SMP),)
ARITY=smp
endif

ifeq ($(PER_USER_MODE),YES)
SEP_PER_USER=-pu
endif
# eventual filename of SEP driver
#DRIVER_FILENAME=$(DRIVER_NAME)$(SEP_PER_USER)-$(PLATFORM)-$(KERNEL_VERSION)$(ARITY).ko

# By default, build the EMON portions of the driver
# unless building IPF, which EMON doesn't support currently
ifeq ($(EMON),)
    ifneq ($(PLATFORM),ia64)
        EMON=YES
    else
        EMON=NO
    endif
endif

# build options ...
ifneq ($(KERNELRELEASE),)
	obj-m := $(DRIVER_NAME).o

ifeq ($(PLATFORM),x32)
	arch-objs :=    core.o    \
			core2.o   \
			sys32.o   \
			corei7_unc.o \
			snbunc_cbo.o \
			snbunc_imc.o \
			wsmexunc_imc.o \
			wsmexunc_qpi.o \
			wsmexunc_wbox.o \
			jktunc_imc.o \
			pci.o    \
			apic.o    \
			pebs.o   \
			snb_power.o
endif
ifeq ($(PLATFORM),x32_64)
	arch-objs :=    core2.o   \
			sys64.o   \
			corei7_unc.o \
			snbunc_cbo.o \
			snbunc_imc.o \
			wsmexunc_imc.o \
			wsmexunc_qpi.o \
			wsmexunc_wbox.o \
			jktunc_imc.o \
			pci.o  \
			apic.o    \
			pebs.o  \
			snb_power.o
endif
ifeq ($(PLATFORM),x64)
	arch-objs :=    montecito.o \
			poulson.o   \
			sys_ia64.o
endif

ifeq ($(PER_USER_MODE),YES)
    EXTRA_CFLAGS += -DSECURE_SEP
endif

ifeq ($(EMON),YES)
    EXTRA_CFLAGS += -DEMON -DEMON_INTERNAL
endif

ifeq ($(BUILD_CHIPSET),YES)
    EXTRA_CFLAGS += -DBUILD_CHIPSET
	chipset-objs :=	pci.o             \
			chap.o            \
			gmch.o
endif

ifeq ($(BUILD_GFX),YES)
    EXTRA_CFLAGS += -DBUILD_GFX
	gfx-objs    :=	gfx.o
endif

	sep3_7-objs :=	lwpmudrv.o        \
			control.o         \
			cpumon.o          \
			eventmux.o        \
			linuxos.o         \
			output.o          \
			pmi.o             \
			sys_info.o        \
			utility.o         \
			$(chipset-objs)   \
			$(gfx-objs)       \
			$(arch-objs)

# targets ...

# Otherwise, we were called directly from the command
# line; invoke the kernel build system.
else
	#KERNEL_SRC_DIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
	KERNEL_SRC_DIR := $(PWD)/../../../../../hardware/intel/linux-2.6
	PATH := $(KERNEL_SRC_DIR)/../../../prebuilt/linux-x86/toolchain/i686-android-linux-4.4.3/bin:$(PATH)
	export PATH

all: default

default:
	ARCH=i386 CROSS_COMPILE=i686-android-linux- $(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) LDDINCDIR=$(PWD)/../include LDDINCDIR1=$(PWD)/inc modules PWD=$(PWD)
	#cp $(DRIVER_NAME).ko $(DRIVER_FILENAME)
	@(cd pax && make all)

endif

install:
	cp $(DRIVER_NAME).ko $(INSTALL)/
	@echo "Installed $(DRIVER_NAME) driver to $(INSTALL)/$(DRIVER_FILENAME) ."
	@(cd pax && make install INSTALL=$(INSTALL)/pax)

clean:
	rm -f *.o .*.o.cmd .*.o.d .*.ko.cmd .*.ko.unsigned.cmd *.gcno
	rm -f $(DRIVER_NAME).ko $(DRIVER_NAME).ko.unsigned $(DRIVER_FILENAME)
	rm -f Module.symvers Modules.symvers *.mod.c modules.order Module.markers
	rm -rf .tmp_versions
	@(cd pax && make clean)
