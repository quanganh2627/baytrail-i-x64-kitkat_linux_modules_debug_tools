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

#ifndef _WUWATCH_DEFINES_H_
#define _WUWATCH_DEFINES_H_ 1

/* **************************************
 * This file contains macros/constants etc. that
 * are specific to the data parser only!
 * **************************************
 */

#define PROC_DIR_NAME "/proc"

#define ACPI_DIR_NAME "/sys/devices/system/cpu/cpu0/cpuidle"
#define ACPI_STATE_STRING "state"
/*
 * Helper macro to express micro-seconds in terms of # of clock
 * ticks for the given target arch.
 * @us: time, in microseconds.
 * @freq_mhz: frequency, in MHz
 * @returns: # of clock ticks
 */
#define CONVERT_US_TO_CLOCK_TICKS(us, freq_mhz) ( (us) * (freq_mhz) )

#endif // _WUWATCH_DEFINES_H_
