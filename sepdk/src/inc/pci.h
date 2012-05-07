/*COPYRIGHT**
 * -------------------------------------------------------------------------
 *               INTEL CORPORATION PROPRIETARY INFORMATION
 *  This software is supplied under the terms of the accompanying license
 *  agreement or nondisclosure agreement with Intel Corporation and may not
 *  be copied or disclosed except in accordance with the terms of that
 *  agreement.
 *        Copyright (c) 2007-2011 Intel Corporation.  All Rights Reserved.
 * -------------------------------------------------------------------------
**COPYRIGHT*/

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

#endif  /* _PCI_H_ */
