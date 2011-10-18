# Description: Makefile for the power driver.
# Version: 1.0

# ***********************************************************************************************

 # This file is provided under a dual BSD/GPLv2 license.  When using or
 # redistributing this file, you may do so under either license.

  # GPL LICENSE SUMMARY

  # Copyright(c) 2011 Intel Corporation. All rights reserved.

  # This program is free software; you can redistribute it and/or modify
  # it under the terms of version 2 of the GNU General Public License as
  # published by the Free Software Foundation.

  # This program is distributed in the hope that it will be useful, but
  # WITHOUT ANY WARRANTY; without even the implied warranty of
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  # General Public License for more details.

  # You should have received a copy of the GNU General Public License
  # along with this program; if not, write to the Free Software
  # Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  # The full GNU General Public License is included in this distribution
  # in the file called LICENSE.GPL.

  # Contact Information:
  # Gautam Upadhyaya <gautam.upadhyaya@intel.com>
  # 1906 Fox Drive, Champaign, IL - 61820, USA

  # BSD LICENSE

  # Copyright(c) 2011 Intel Corporation. All rights reserved.
  # All rights reserved.

  # Redistribution and use in source and binary forms, with or without
  # modification, are permitted provided that the following conditions
  # are met:

  #   * Redistributions of source code must retain the above copyright
  #     notice, this list of conditions and the following disclaimer.
  #   * Redistributions in binary form must reproduce the above copyright
  #     notice, this list of conditions and the following disclaimer in
  #     the documentation and/or other materials provided with the
  #     distribution.
  #   * Neither the name of Intel Corporation nor the names of its
  #     contributors may be used to endorse or promote products derived
  #     from this software without specific prior written permission.

  # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  # A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  # ***********************************************************************************************


# ***************************************************************************************
#      Copyright (C) 2011 Intel Corporation.  All Rights Reserved.

#      This file is part of the Power Development Kit, and is free software;
#      you can redistribute it
#      and/or modify it under the terms of the GNU General Public License
#      version 2 as published by the Free Software Foundation.

#      This file is distributed in the hope that it will be useful,
#      but WITHOUT ANY WARRANTY; without even the implied warranty of
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#      GNU General Public License for more details.

#      You should have received a copy of the GNU General Public License
#      along with the Power Development Kit; if not, write to the Free Software
#      Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#      As a special exception, you may use this file as part of a free software
#      library without restriction.  Specifically, if other files instantiate
#      templates or use macros or inline functions from this file, or you compile
#      this file and link it with other files to produce an executable, this
#      file does not by itself cause the resulting executable to be covered by
#      the GNU General Public License.  This exception does not however
#      invalidate any other reasons why the executable file might be covered by
#      the GNU General Public License.
# ***************************************************************************************

# -------------------- user configurable options ------------------------

# base name of APWR driver
DRIVER_NAME = apwr2_0

# location to install driver
INSTALL = .

INCDIR=$(M)/../include
INC2=$(M)/inc

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
EXTRA_CFLAGS += -I$(INCDIR)
EXTRA_CFLAGS += -I$(INC2)

# The driver needs to know what version we have
KERNEL_VERSION=$(shell uname -r | awk 'BEGIN{FS="[^0-9]"} {print $$3}')
ifeq "$(APWR_RED_HAT)" ""
	APWR_RED_HAT="0"
endif
EXTRA_CFLAGS += -DAPWR_RED_HAT=$(APWR_RED_HAT)

# platform details
#MACH := $(shell uname -m)
MACH := i686
ifeq ($(MACH),x86_64)
	PLATFORM=x32_64
endif
ifeq ($(MACH),ia64)
	PLATFORM=x64
endif
ifeq ($(PLATFORM),)
	PLATFORM=x32
endif
#KERNEL_VERSION_FULL ?= $(shell uname -r)
#SMP := $(shell uname -v | grep SMP)
SMP := SMP
ARITY=up
ifneq ($(SMP),)
ARITY=smp
endif

# eventual filename of APWR driver
#DRIVER_FILENAME=$(DRIVER_NAME)-$(PLATFORM)-$(KERNEL_VERSION_FULL)$(ARITY).ko


# build options ...
ifneq ($(KERNELRELEASE),)

# the following macros must be set to y
ifeq ($(strip $(CONFIG_MODULES)),y)
ifeq ($(strip $(CONFIG_MODULE_UNLOAD)),y)
ifeq ($(strip $(CONFIG_TRACEPOINTS)),y)
ifeq ($(strip $(CONFIG_FRAME_POINTER)),y)
ifeq ($(strip $(CONFIG_TIMER_STATS)),y)
	obj-m := $(DRIVER_NAME).o

	apwr2_0-objs :=	src/apwr_driver.o
endif
endif
endif
endif
endif

# targets ...

# Otherwise, we were called directly from the command
# line; invoke the kernel build system.
else
	KERNEL_SRC_DIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

all: default

default:
	@echo "************************************************************"
	@echo "DETECTED KERNEL VERSION $(KERNEL_VERSION)"
	@echo "************************************************************"
	@echo "APWR_RED_HAT=$(APWR_RED_HAT)"
	$(MAKE) -C $(KERNEL_SRC_DIR) M=$(PWD) modules PWD=$(PWD)
	mv $(DRIVER_NAME).ko $(DRIVER_FILENAME)

endif

install:
	@[ $(INSTALL) = "." ] || [ $(INSTALL) = $(PWD) ] || [ $(INSTALL) = $(PWD)/ ] \
         || cp $(DRIVER_FILENAME) $(INSTALL)/
	@echo "Installed $(DRIVER_NAME) driver to $(INSTALL)/$(DRIVER_FILENAME) ."

clean:
	make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
# rm -f *.o .*.o.cmd .*.o.d $(DRIVER_NAME).ko $(DRIVER_FILENAME) .*.ko.cmd
# rm -f Module.symvers Modules.symvers *.mod.c modules.order Module.markers
# rm -rf .tmp_versions
