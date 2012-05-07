/*COPYRIGHT**
// -------------------------------------------------------------------------
//               INTEL CORPORATION PROPRIETARY INFORMATION
//  This software is supplied under the terms of the accompanying license
//  agreement or nondisclosure agreement with Intel Corporation and may not
//  be copied or disclosed except in accordance with the terms< of that
//  agreement.
//        Copyright (c) 2011 Intel Corporation. All Rights Reserved.
// -------------------------------------------------------------------------
**COPYRIGHT*/


#ifndef _SNBUNC_IMC_H_INC_
#define _SNBUNC_IMC_H_INC_

/*
 * Local to this architecture: SNB uncore IMC unit 
 * 
 */
#define SNBUNC_IMC_DESKTOP_DID             0x000100
#define SNBUNC_IMC_MODILE_DID              0x010104
#define NEXT_ADDR_OFFSET                   4
#define SNBUNC_IMC_BAR_ADDR_SHIFT          32
#define DRV_IS_PCI_VENDOR_ID_INTEL         0x8086

#define SNBUNC_IMC_PERF_GLOBAL_CTRL        0x391
#define SNBUNC_IMC_BAR_ADDR_MASK           0x0007FFFFF8000LL

#define IA32_DEBUG_CTRL                    0x1D9
#define MAX_FREE_RUNNING_EVENTS            6


extern DISPATCH_NODE  snbunc_imc_dispatch;

#endif /* _SNBUNC_CBO_H_INC_*/
