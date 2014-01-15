#!/bin/sh
###############################################################################
#
# Copyright (C) 2012 Intel Corporation.  All Rights Reserved.
#
# This file is part of SEP Development Kit
#
# SEP Development Kit is free software; you can redistribute it
# and/or modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
#
# SEP Development Kit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SEP Development Kit; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
#
# As a special exception, you may use this file as part of a free software
# library without restriction.  Specifically, if other files instantiate
# templates or use macros or inline functions from this file, or you compile
# this file and link it with other files to produce an executable, this
# file does not by itself cause the resulting executable to be covered by
# the GNU General Public License.  This exception does not however
# invalidate any other reasons why the executable file might be covered by
# the GNU General Public License.
#
###############################################################################

SRC_DIR="$(cd $(dirname "$0"); pwd)"
PRODUCT=${PRODUCT:=mfld_dv10}
ANDROID_REPO=${ANDROID_REPO:=`which adb | sed  's/\(^\/.*\)\/out\/.*/\1/'`}

KDIR=$ANDROID_REPO/out/target/product/$PRODUCT/obj/kernel
if [ -d ${KDIR} ]
then
    TOOLCHAIN=$ANDROID_REPO/prebuilt/linux-x86/toolchain/i686-android-linux-4.4.3/bin/i686-android-linux-
    KERNEL_VERSION=2.6.android-x86
    make -C $SRC_DIR KDIR=$KDIR KERNEL_VERSION=$KERNEL_VERSION MARCH=i386 CROSS_COMPILE=$TOOLCHAIN $*
else
    echo >&2 "Error: $KDIR directory doesn't exist. It is not possible to compile VTSS++ driver."
    echo >&2 "Please try to specify PRODUCT or ANDROID_REPO environment variables."
    echo >&2 "Example: export PRODUCT=mfld_dv10; export ANDROID_REPO=~/umg_repo"
    exit 1
fi
