/*
    Copyright (C) 2011-2012 Intel Corporation.  All Rights Reserved.

    This file is part of SEP Development Kit

    SEP Development Kit is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    SEP Development Kit is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SEP Development Kit; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    As a special exception, you may use this file as part of a free software
    library without restriction.  Specifically, if other files instantiate
    templates or use macros or inline functions from this file, or you compile
    this file and link it with other files to produce an executable, this
    file does not by itself cause the resulting executable to be covered by
    the GNU General Public License.  This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/


#ifndef _JKTUNC_UBOX_H_INC_
#define _JKTUNC_UBOX_H_INC_

#define DRV_IS_PCI_VENDOR_ID_INTEL            0x8086
#define VENDOR_ID_MASK                        0x0000FFFF
#define DEVICE_ID_MASK                        0xFFFF0000
#define DEVICE_ID_BITSHIFT                    16

#define JKTUNC_SOCKETID_UBOX_DID              0x003CE0
#define JKTUNC_SOCKETID_UBOX_LNID_OFFSET      0x40
#define JKTUNC_SOCKETID_UBOX_GID_OFFSET       0x54
#define JKTUNC_SOCKETID_INTERNAL_BUSES_OFFSET 0xD0
#define IVYTOWN_SOCKETID_UBOX_DID             0x0E1E

extern U32 *JKTUNC_UBOX_package_to_bus_map;

extern VOID JKTUNC_UBOX_Do_Bus_to_Socket_Map(U32);

#endif 
