/*****************************************************************************
#                      INTEL CONFIDENTIAL
#
# Copyright 2009-2012 Intel Corporation All Rights Reserved.
#
# This file is provided for internal use only. You are free to modify it
# for your own internal purposes, but you cannot release it to the external
# open source community under any circumstances without SSG.DPD's approval.
#
# If you make significant improvements, fix bugs, or have suggestions for
# additional features or improvements, please provide your changes to
# Robert or Gautam for inclusion in this tool.
#
# Please contact Robert Knight (robert.knight@intel.com) or Gautam Upadhyaya
# (gautam.upadhyaya@intel.com) if you have any questions.
#
*****************************************************************************
*/

#ifndef _PW_ARCH_H_
#define _PW_ARCH_H_

/* **************************************
 * Data structures
 * **************************************
 */
/*
typedef enum {
    NHM=0,
    SNB,
    MFD,
    LEX,
    CLV
} arch_type_t;

#define PW_IS_ATM(type) ( (type) == MFD || (type) == LEX || (type) == CLV )
*/

#define PW_IS_SALTWELL(model) ( (model) == 0x27 || (model) == 0x35 )
#define PW_IS_MFD(model) ( (model) == 0x27 )
#define PW_IS_CLV(model) ( (model) == 0x35 )

#endif // _PW_ARCH_H_
