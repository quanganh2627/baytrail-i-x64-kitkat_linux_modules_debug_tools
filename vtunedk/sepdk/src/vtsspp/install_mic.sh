#!/bin/bash
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

CDIR="$(cd $(dirname "$0") && pwd)"
INSTALL_DIR="${INSTALL_DIR:=/opt/intel/mic/amplxe}"
INSTALL_GID=${INSTALL_GID:=500}
MIC_DIR="${MIC_DIR:=/amplxe}"
NAME=amplxe

if [ ! -d /etc/sysconfig/mic/conf.d  ] ; then
    echo >&2 "Error: /etc/sysconfig/mic/conf.d directory doesn't exist."
    exit 1
fi

if [ ! -r "${CDIR}/support.txt" ] ; then
    echo >&2 "Error: incorrect MIC package."
    exit 1
fi

mkdir -p "${INSTALL_DIR}/device"
if [ ! -d "${INSTALL_DIR}/device" ] ; then
    echo >&2 "Error: Cannot create ${INSTALL_DIR}/device directory."
    exit 1
fi

cp -f "${CDIR}/support.txt" "${INSTALL_DIR}/device/support.txt" || exit 1
cat >"${INSTALL_DIR}/${NAME}.filelist" <<ENDOFFILELIST
dir ${MIC_DIR} 755 0 0
file ${MIC_DIR}/support.txt device/support.txt 644 0 0
ENDOFFILELIST

# Create results directory
mkdir -p "${INSTALL_DIR}/device/results" && echo "dir ${MIC_DIR}/results 777 0 ${INSTALL_GID}" >>"${INSTALL_DIR}/${NAME}.filelist"
# Create all directories
for d in `cd "${CDIR}" && find {bin64,lib64,message} -type d -print`; do
    mkdir -p "${INSTALL_DIR}/device/$d" && echo "dir ${MIC_DIR}/$d 755 0 0" >>"${INSTALL_DIR}/${NAME}.filelist"
done
# Copy all executables
for f in `cd "${CDIR}" && find bin64 -type f -print`; do
    cp -f "${CDIR}/$f" "${INSTALL_DIR}/device/$f" && echo "file ${MIC_DIR}/$f device/$f 755 0 0" >>"${INSTALL_DIR}/${NAME}.filelist"
done
# Copy all shared libraries
for f in `cd "${CDIR}" && find lib64 -type f -name \*.so -print`; do
    cp -f "${CDIR}/$f" "${INSTALL_DIR}/device/$f" && echo "file ${MIC_DIR}/$f device/$f 755 0 0" >>"${INSTALL_DIR}/${NAME}.filelist"
done
# Copy all messages
for f in `cd "${CDIR}" && find message -type f -name \*.xmc -print`; do
    cp -f "${CDIR}/$f" "${INSTALL_DIR}/device/$f" && echo "file ${MIC_DIR}/$f device/$f 644 0 0" >>"${INSTALL_DIR}/${NAME}.filelist"
done
# Copy all txt db files
for f in `cd "${CDIR}" && find lib64 -type f -name \*.txt -print`; do
    cp -f "${CDIR}/$f" "${INSTALL_DIR}/device/$f" && echo "file ${MIC_DIR}/$f device/$f 644 0 0" >>"${INSTALL_DIR}/${NAME}.filelist"
done

echo "Overlay ${INSTALL_DIR} ${INSTALL_DIR}/${NAME}.filelist" > /etc/sysconfig/mic/conf.d/${NAME}.conf
if [ $? -eq 0  ] ; then
    echo >&2 "${NAME} successfully installed."
    (INSTALL_GID=${INSTALL_GID} "${CDIR}/sepdk/src/vtsspp/build_mic.sh" --prebuilt --install)
    if [ $? -ne 0  ] ; then
        echo >&2 ""
        echo >&2 "Manual VTSS++ driver compilation is required:"
        echo >&2 "  unpack MPSS sources into /tmp/mic directory and then do"
        echo >&2 "  KDIR=/tmp/mic/card/kernel ${CDIR}/sepdk/src/vtsspp/build_mic.sh --build clean all"
    fi
    echo >&2 ""
    echo >&2 "MPSS restart is required:"
    echo >&2 "  sudo service mpss restart"
fi
