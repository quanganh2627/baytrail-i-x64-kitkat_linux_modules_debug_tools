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
INSTALL_DIR="${INSTALL_DIR:=/opt/intel/mic/vtsspp}"
INSTALL_GID=${INSTALL_GID:=500}
NAME=vtsspp
ARCH=k1om
KDIR="${KDIR:=/opt/intel/mic/src/card/kernel}"
KIMG="${KIMG:=/lib/firmware/mic/vmlinux}"
TOOLCHAIN="/usr/linux-$ARCH-4.7/bin/x86_64-$ARCH-linux-"

# get kernel version string
if [ -r "${KDIR}/vmlinux" ] ; then
    KIMG="${KDIR}/vmlinux"
fi
KVERMAGIC=`strings ${KIMG} | egrep vermagic=  | uniq`
KERNEL_VERSION=`echo "${KVERMAGIC}" | awk '{ print $1 }' | sed 's/vermagic=//'`

# check whether kernel is for UP or SMP
SMP=`echo "${KVERMAGIC}" | grep SMP`
if [ -z "${SMP}" ] ; then
    echo >&2 "Error: MIC Kernel is not SMP."
    exit 1
fi

test $# = 0 && exec /bin/bash $0 --error
while test $# -gt 0; do
    OPTION=$1; shift
    case "${OPTION}" in
    --prebuilt)
        DRIVER_PATH="${CDIR}/../../prebuilt/${NAME}/${NAME}-${ARCH}-${KERNEL_VERSION}smp.ko"
        if [ ! -r "${DRIVER_PATH}" ] ; then
            # first check if perl installed
            hash perl &> /dev/null
            if [ $? -eq 1 ]; then
                echo >&2 "Error: perl is not installed. So, could not update ${NAME} driver."
                exit 1
            else
                # first grab length of match
                # try to match XX.XX.XX.XX- pattern first
                LX_VER_LEN=`expr "$KERNEL_VERSION" : '[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*-'`
                if [ "$LX_VER_LEN" -eq 0 ] ; then
                    # Now, try to match XX.XX.XX- pattern
                    LX_VER_LEN=`expr "$KERNEL_VERSION" : '[0-9]*\.[0-9]*\.[0-9]*-'`
                fi
                # then extract substring (assuming that it's from the beginning of the string)
                LX_VER=${KERNEL_VERSION:0:$LX_VER_LEN}
                DRIVER_SRC="$(ls ${CDIR}/../../prebuilt/${NAME}/${NAME}-${ARCH}-${LX_VER}*.ko 2>/dev/null | head -n 1)"
                DRIVER_DST="${CDIR}/$(basename ${DRIVER_PATH})"
                if [ ! -z "${DRIVER_SRC}" -a -r "${DRIVER_SRC}" ] ; then
                    echo >&2 "Update version magic: ${KERNEL_VERSION}"
                    (perl "${CDIR}/update_version.pl" "${KERNEL_VERSION}" "${DRIVER_SRC}" "${DRIVER_DST}")
                    if [ $? -eq 0 ]; then
                        echo >&2 "${NAME} driver is ready to install."
                    else
                        exit 1
                    fi
                else
                    echo >&2 "Error: Could not find prebuilt driver in $(dirname ${DRIVER_PATH})"
                    exit 1
                fi
            fi
        else
            cp -f "${DRIVER_PATH}" "${CDIR}" && echo >&2 "${NAME} driver is ready to install."
        fi
        ;;
    --build)
        if [ -d "${KDIR}" ] ; then
            ARGS=""
            VTSS="mic"
            while test $# -gt 0; do
                V=`echo $1 | sed -e 's/^\(..\).*$/\1/'`
                if [ "${V}" == "--" ] ; then
                    break
                fi
                V=`echo $1 | sed -e 's/=.*$//'`
                if [ "${V}" == "VTSS" ] ; then
                    V=`echo $1 | sed -e 's/^.*=//'`
                    VTSS="${VTSS}:${V}"
                else
                    ARGS="${ARGS} $1"
                fi
                shift
            done
            make KDIR="${KDIR}" KERNEL_VERSION="${KERNEL_VERSION}" ARCH="${ARCH}" CROSS_COMPILE="${TOOLCHAIN}" VTSS="${VTSS}" ${ARGS}
        else
            echo >&2 "Error: ${KDIR} directory doesn't exist."
            exit 1
        fi
        ;;
    --install)
        if [ ! -d "/etc/sysconfig/mic/conf.d"  ] ; then
            echo >&2 "Error: /etc/sysconfig/mic/conf.d directory doesn't exist."
            exit 1
        fi
        DRIVER_PATH="${CDIR}/${NAME}-${ARCH}-${KERNEL_VERSION}smp.ko"
        if [ ! -r "${DRIVER_PATH}" ] ; then
            echo >&2 "Error: ${DRIVER_PATH} doesn't exist."
            exit 1
        fi
        mkdir -p "${INSTALL_DIR}"
        if [ ! -d "${INSTALL_DIR}" ] ; then
            echo >&2 "Error: Cannot create ${INSTALL_DIR} directory."
            exit 1
        fi
        cp -f "${DRIVER_PATH}" "${INSTALL_DIR}" || exit 1
        echo "file /lib/modules/${KERNEL_VERSION}/${NAME}.ko $(basename ${DRIVER_PATH}) 644 0 0" > "${INSTALL_DIR}/${NAME}.filelist"
        cat >"${INSTALL_DIR}/${NAME}" <<ENDOFRC
#!/bin/sh

MODULES="/lib/modules/${KERNEL_VERSION}"
GID=${INSTALL_GID}

case "\$1" in
    start)
        if [ ! -f "\${MODULES}/${NAME}.ko" ]; then
            exit 1
        fi
        insmod "\${MODULES}/${NAME}.ko" gid=\${GID} mode=0666
        ;;
    stop)
        rmmod ${NAME}.ko
        ;;
esac
ENDOFRC
        chmod a+x "${INSTALL_DIR}/${NAME}"
        cat >>"${INSTALL_DIR}/${NAME}.filelist" <<ENDOFFILELIST
dir /etc/rc3.d 755 0 0
file /etc/init.d/${NAME} ${NAME} 755 0 0
slink /etc/rc3.d/S97${NAME} ../init.d/${NAME} 777 0 0
ENDOFFILELIST
        echo "Overlay ${INSTALL_DIR} ${INSTALL_DIR}/${NAME}.filelist" > /etc/sysconfig/mic/conf.d/${NAME}.conf
        if [ $? -eq 0  ] ; then
            echo >&2 "${NAME} driver successfully installed for group \"${INSTALL_GID}\"."
        else
            exit 1
        fi
        ;;
    --help)
        cat <<ENDHELP
Usage: $0 [options] [args]

Options:
  --prebuilt Update version magic of prebuilt driver
  --install  Install the driver to MIC card boot image
             (MPSS restart is required)
  --build    Build driver from sources
  --help     Prints this message
ENDHELP
        ;;
    --error|*)
        /bin/bash $0 --help 1>&2
        exit 1
        ;;
    esac
done
exit 0
