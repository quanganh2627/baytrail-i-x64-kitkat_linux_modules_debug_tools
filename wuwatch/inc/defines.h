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

/* ****************************************************
 * File containing definitions and constants common to
 * all components of the wuwatch tool.
 * ****************************************************
 */

#ifndef _DEFINES_H_
#define _DEFINES_H_ 1

#define READ 0
#define WRITE 1

#define SUCCESS 0
#define ERROR 1

/* **************************************
 * VERSION information.
 * Format: X.Y.Z
 * **************************************
 */
// We're currently at 3.0.2
#define WUWATCH_VERSION_VERSION 3
#define WUWATCH_VERSION_INTERFACE 0
#define WUWATCH_VERSION_OTHER 2

/* **************************************
 * Compile-time constants.
 * **************************************
 */

#define NUMBER_OF_ARCHITECTURE_TYPES 3
/*
 * Default output file name -- the extensions depend on
 * which program is executing: wuwatch output files have
 * a ".ww1" extension, while wudump output files have a
 * ".txt" extension. The extensions are added in by the
 * respective programs i.e. wuwatch/wudump.
 */
#define DEFAULT_WUWATCH_OUTPUT_FILE_NAME "wuwatch_output"

/*
 * Some useful typedefs/macros
 */

#define INC_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(1)
#define GET_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(0)
#define DEC_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(-1)

/* **************************************
 * Debugging tools.
 * **************************************
 */
extern bool g_do_debugging;
#define db_fprintf(...) do { \
    if (g_do_debugging) { \
        fprintf(__VA_ARGS__); \
    } \
} while(0)
#define db_assert(e, ...) do { \
    if (g_do_debugging && !(e)) { \
	    fprintf(stderr, __VA_ARGS__);	\
	    assert(false);			\
	}					\
} while(0)
#define db_abort(...) do { \
    if (g_do_debugging) { \
        fprintf(stderr, __VA_ARGS__); \
        assert(false); \
    } \
} while(0)
#define db_copy(...) do { \
    if (g_do_debugging) { \
        std::copy(__VA_ARGS__); \
    } \
} while(0)
/*
 * Helper macro to get the string equivalent of a boolean var.
 */
#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )

#endif // _DEFINES_H_
