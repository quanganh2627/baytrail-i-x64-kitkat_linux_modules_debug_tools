/*
    Copyright (C) 2005-2012 Intel Corporation.  All Rights Reserved.
 
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
#ifndef _PCI_H_
#define _PCI_H_

#include "lwpmudrv_defines.h"

/*
 * PCI Config Address macros
 */
#define PCI_ENABLE                          0x80000000

#define PCI_ADDR_IO                         0xCF8
#define PCI_DATA_IO                         0xCFC

#define BIT0                                0x1
#define BIT1                                0x2

/*
 * Macro for forming a PCI configuration address
 */
#define FORM_PCI_ADDR(bus,dev,fun,off)     (((PCI_ENABLE))          |   \
                                            ((bus & 0xFF) << 16)    |   \
                                            ((dev & 0x1F) << 11)    |   \
                                            ((fun & 0x07) <<  8)    |   \
                                            ((off & 0xFF) <<  0))

#if defined(DRV_IA32) || defined(DRV_EM64T)
extern int
PCI_Read_From_Memory_Address (
    U32 addr,
    U32* val
);

extern int
PCI_Write_To_Memory_Address (
    U32 addr,
    U32 val
);

extern int
PCI_Read_Ulong (
    U32 pci_address
);

extern void
PCI_Write_Ulong (
    U32 pci_address,
    U32 value
);
#endif

#endif  
