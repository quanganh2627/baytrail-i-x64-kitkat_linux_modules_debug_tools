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

#ifndef _WUDUMP_DEFINES_H_
#define _WUDUMP_DEFINES_H_

/*
 * Fields that we don't care
 * about are represented by
 * a "-"
 */
#define DONT_CARE "-"
/*
 * Any process for which we
 * don't have a PID <-> PROC name
 * mapping will be marked
 * as 'unknown'
 */
#define UNKNOWN_PROCESS_NAME "Unknown-Process-Name"

#endif // _WUDUMP_DEFINES_H_
