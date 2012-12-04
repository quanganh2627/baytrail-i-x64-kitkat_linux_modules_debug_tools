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

#ifndef _WULIB_DEFINES_H_
#define _WULIB_DEFINES_H_
/*
 * How many samples to read at one go?
 */
#define NUM_MSGS_TO_READ 1024
/*
 * Some useful macros.
 */
/*
 * Get string equivalent of a boolean var.
 */
#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )
/*
 * Macros used to determine an actual frequency, given an average one.
 */
#define IS_BRACKETED_BY(upper, lower, freq) ({bool __tmp = ( (upper) >= (freq) && (freq) >= (lower) ) ? true : false; __tmp;})
#define GET_CORRECT_FREQ_BOUND(upper, lower, freq) ({u32 __delta_upper = (upper) - (freq), __delta_lower = (freq) - (lower), __tmp = (__delta_upper < __delta_lower) ? (upper) : (lower); __tmp;})
/*
 * Does this "SCHED_WAKEUP" sample have the same
 * source and destination cpus?
 */
// #define IS_SELF_SCHED_SAMPLE(sample) ( (sample)->cpuidx == (sample)->e_sample.data[1] )
#define IS_SELF_SCHED_SAMPLE(sample) ( GET_CORE_GIVEN_LCPU((sample)->cpuidx) == GET_CORE_GIVEN_LCPU((sample)->e_sample.data[1]) )
/*
 * Is this a valid wakeup cause?
 */
#define IS_INVALID_WAKEUP_CAUSE(c_group) ( (c_group).wakeup_sample == NULL || ( (c_group).wakeup_sample->sample_type == SCHED_SAMPLE && IS_SELF_SCHED_SAMPLE((c_group).wakeup_sample) ) )
// #define IS_INVALID_WAKEUP_CAUSE(c_group) ( (c_group).wakeup_sample == NULL )
/*
 * Helper macro to iterate over a range.
 */
#define FOR_EACH(index, begin, end) \
    for (index = (begin); index <= (end); index = advance_i(index))
/*
 * Helper macro to iterate (in reverse) over a
 * range.
 */
#define FOR_EACH_REVERSE(index, begin, end) \
    for (index = (end); index >= (begin); index = radvance_i(index))

#define ME(w) ( (w).m_me )
#define PER_CPU_PW_LIST(w) ( (w).m_pw_samples )
#define PER_CPU_SAMPLE_LIST(w) ( (w).m_samples )

#define NUMBER_OF_BREAKTYPES_DEPRECATED 5
#define NUMBER_OF_BREAKTYPES 6

#endif // _WULIB_DEFINES_H_
