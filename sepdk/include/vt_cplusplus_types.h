/*COPYRIGHT**
 * -------------------------------------------------------------------------
 *               INTEL CORPORATION PROPRIETARY INFORMATION
 *  This software is supplied under the terms of the accompanying license
 *  agreement or nondisclosure agreement with Intel Corporation and may not
 *  be copied or disclosed except in accordance with the terms of that
 *  agreement.
 *        Copyright (c) 2002-2011 Intel Corporation. All Rights Reserved.
 * -------------------------------------------------------------------------
**COPYRIGHT*/

#ifndef VT_CPLUSPLUS_TYPES_H
#define VT_CPLUSPLUS_TYPES_H

#include <string>
using namespace std;

#if defined (UNICODE)
typedef wstring STLSTRING;
#else
typedef string STLSTRING;
#endif

#if defined(VTSA_SETUP_ASCII_ENVIRONMENT)
typedef string VTSA_STLSTRING;
#else
typedef wstring VTSA_STLSTRING;
#endif

#endif  // VT_CPLUSPLUS_TYPES_H
