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

/*
 * File containing declarations/definitions for basic unix domain socket and
 * named pipe based client-server communication.
 * Used with the PRELOAD "hook" library on Linux.
 */

#ifndef _UDS_HPP_
#define _UDS_HPP_

#define WUWATCH_UNIXSTR_PATH "/tmp/wuwatch"
#define WUWATCH_ANDROIDSTR_PATH "/data/uds_wuwatch"

#define WUWATCH_LISTENQ 10

#define PW_SHM_PATH "/pw_shm"
#define PW_SHM_MODE (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)

typedef enum {
    FORK_SYN, /* Client tells server it's been forked */
    FORK_ACK, /* Server ACKs the client SYN */
    EXIT_SYN, /* Client tells server it's about to exit */
    EXIT_ACK, /* Server ACKs the client SYN */
    DO_QUIT /* Server tells client to quit */
} uds_msg_type_t;

typedef struct {
    uds_msg_type_t type;
    int offset;
    union{
        pid_t child_pid;
        int dev_fd;
    };
} uds_msg_t;

typedef struct {
    sem_t sem;
    union {
        uds_msg_t msg;
        int count; // used ONLY in the ZEROth data element
    };
} shm_data_t;

#define SERVER_SHM_OFFSET 0

#endif // _UDS_HPP_ 
