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


#ifndef _JKTUNC_IMC_H_INC_
#define _JKTUNC_IMC_H_INC_

/*
 * Local to this architecture: JKT uncore IMC unit 
 * 
 */
#define JKTUNC_IMC0_DID                    0x003CB4
#define JKTUNC_IMC1_DID                    0x003CB5
#define JKTUNC_IMC2_DID                    0x003CB0
#define JKTUNC_IMC3_DID                    0x003CB1
#define NEXT_ADDR_OFFSET                   4
#define IMC_ADDR_SHIFT                     32
#define DRV_IS_PCI_VENDOR_ID_INTEL         0x8086

#define JKTUNC_IMC_PERF_GLOBAL_CTRL        0x391

#define IA32_DEBUG_CTRL                    0x1D9


extern DISPATCH_NODE  jktunc_imc_dispatch;

#endif /* _JKTUNC_CBO_H_INC_*/
