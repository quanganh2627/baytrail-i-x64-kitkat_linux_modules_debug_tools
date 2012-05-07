/*
    Copyright (C) 2005-2011 Intel Corporation.  All Rights Reserved.

    This file is part of SEP Development Kit

    SEP Development Kit is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    SEP Development Kit is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SEP Development Kit; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    As a special exception, you may use this file as part of a free software
    library without restriction.  Specifically, if other files instantiate
    templates or use macros or inline functions from this file, or you compile
    this file and link it with other files to produce an executable, this
    file does not by itself cause the resulting executable to be covered by
    the GNU General Public License.  This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

/*
 *  cvs_id[] = "$Id$"
 */

#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <linux/smp.h>
#include <linux/timer.h>
#if defined(DRV_IA32)
#include <asm/apic.h>
#endif
#include <asm/io.h>
#if defined(DRV_IA32)
#include <asm/msr.h>
#endif
#include <asm/atomic.h>

#include "lwpmudrv_defines.h"
#include "lwpmudrv.h"
#include "lwpmudrv_types.h"

#define  MAX_KMALLOC_SIZE ((1<<17)-1)

// check whether Linux driver should use unlocked ioctls (not protected by BKL)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) && defined(HAVE_UNLOCKED_IOCTL)
#define DRV_USE_UNLOCKED_IOCTL
#endif
#if defined(DRV_USE_UNLOCKED_IOCTL)
#define IOCTL_OP .unlocked_ioctl
#define IOCTL_OP_TYPE long
#define IOCTL_USE_INODE
#else
#define IOCTL_OP .ioctl
#define IOCTL_OP_TYPE int
#define IOCTL_USE_INODE struct   inode  *inode,
#endif

// Information about the state of the driver
typedef struct GLOBAL_STATE_NODE_S  GLOBAL_STATE_NODE;
typedef        GLOBAL_STATE_NODE   *GLOBAL_STATE;
struct GLOBAL_STATE_NODE_S {
    volatile int    cpu_count;
    volatile int    dpc_count;

    int             num_cpus;       // Number of CPUs in the system
    int             active_cpus;    // Number of active CPUs - some cores can be
                                    // deactivated by the user / admin
    int             num_em_groups;
    int             num_descriptors;
    volatile int    current_phase;
};

// Access Macros
#define  GLOBAL_STATE_num_cpus(x)          ((x).num_cpus)
#define  GLOBAL_STATE_active_cpus(x)       ((x).active_cpus)
#define  GLOBAL_STATE_cpu_count(x)         ((x).cpu_count)
#define  GLOBAL_STATE_dpc_count(x)         ((x).dpc_count)
#define  GLOBAL_STATE_num_em_groups(x)     ((x).num_em_groups)
#define  GLOBAL_STATE_num_descriptors(x)   ((x).num_descriptors)
#define  GLOBAL_STATE_current_phase(x)     ((x).current_phase)
#define  GLOBAL_STATE_sampler_id(x)        ((x).sampler_id)


/*
 *
 *
 * CPU State data structure and access macros
 *
 */
typedef struct CPU_STATE_NODE_S  CPU_STATE_NODE;
typedef        CPU_STATE_NODE   *CPU_STATE;
struct CPU_STATE_NODE_S {
    int         apic_id;             // Processor ID on the system bus
    PVOID       apic_linear_addr;    // linear address of local apic
    PVOID       apic_physical_addr;  // physical address of local apic

    PVOID       idt_base;            // local IDT base address
    U64         tsc;
    atomic_t    in_interrupt;

#if defined(DRV_IA32)
    U64         saved_ih;            // saved perfvector to restore
#endif
#if defined(DRV_EM64T)
    PVOID       saved_ih;            // saved perfvector to restore
#endif

    S64        *em_tables;           // holds the data that is saved/restored
                                     // during event multiplexing

    struct timer_list *em_timer;
    U32         current_group;
    S32         trigger_count;
    S32         trigger_event_num;

    DISPATCH    dispatch;
    PVOID       lbr_area;
    PVOID       old_dts_buffer;
    PVOID       dts_buffer;
    U32         accept_interrupt;

    // Chipset counter stuff
    U32         chipset_count_init;  // flag to initialize the last MCH and ICH arrays below.
    U64         last_mch_count[8];
    U64         last_ich_count[8];
    U64         last_mmio_count[32]; // it's only 9 now but the next generation may have 29.
    U64         last_gmch_count[8];

    U64        *pmu_state;           // holds PMU state (e.g., MSRs) that will be
                                     // saved before and restored after collection
    S32         socket_master;
    S32         core_master;
    S32         thr_master;
};

#define CPU_STATE_apic_id(cpu)              (cpu)->apic_id
#define CPU_STATE_apic_linear_addr(cpu)     (cpu)->apic_linear_addr
#define CPU_STATE_apic_physical_addr(cpu)   (cpu)->apic_physical_addr
#define CPU_STATE_idt_base(cpu)             (cpu)->idt_base
#define CPU_STATE_tsc(cpu)                  (cpu)->tsc
#define CPU_STATE_in_interrupt(cpu)         (cpu)->in_interrupt
#define CPU_STATE_saved_ih(cpu)             (cpu)->saved_ih
#define CPU_STATE_saved_ih_hi(cpu)          (cpu)->saved_ih_hi
#define CPU_STATE_dpc(cpu)                  (cpu)->dpc
#define CPU_STATE_em_tables(cpu)            (cpu)->em_tables
#define CPU_STATE_pmu_state(cpu)            (cpu)->pmu_state
#define CPU_STATE_em_dpc(cpu)               (cpu)->em_dpc
#define CPU_STATE_em_timer(cpu)             (cpu)->em_timer
#define CPU_STATE_current_group(cpu)        (cpu)->current_group
#define CPU_STATE_trigger_count(cpu)        (cpu)->trigger_count
#define CPU_STATE_trigger_event_num(cpu)    (cpu)->trigger_event_num
#define CPU_STATE_dispatch(cpu)             (cpu)->dispatch
#define CPU_STATE_lbr(cpu)                  (cpu)->lbr
#define CPU_STATE_old_dts_buffer(cpu)       (cpu)->old_dts_buffer
#define CPU_STATE_dts_buffer(cpu)           (cpu)->dts_buffer
#define CPU_STATE_accept_interrupt(cpu)     (cpu)->accept_interrupt
#define CPU_STATE_msr_value(cpu)            (cpu)->msr_value
#define CPU_STATE_msr_addr(cpu)             (cpu)->msr_addr
#define CPU_STATE_socket_master(cpu)        (cpu)->socket_master
#define CPU_STATE_core_master(cpu)          (cpu)->core_master
#define CPU_STATE_thr_master(cpu)           (cpu)->thr_master


/*
 * For storing data for --read/--write-msr command line options
 */
typedef struct MSR_DATA_NODE_S MSR_DATA_NODE;
typedef        MSR_DATA_NODE  *MSR_DATA;
struct MSR_DATA_NODE_S {
    U64         value;             // Used for emon, for read/write-msr value
    U64         addr;
};

#define MSR_DATA_value(md)   (md)->value
#define MSR_DATA_addr(md)    (md)->addr

/****************************************************************************
 ** Global State variables exported
 ***************************************************************************/
extern   CPU_STATE            pcb;
extern   GLOBAL_STATE_NODE    driver_state;
extern   MSR_DATA             msr_data;

/****************************************************************************
 **  Handy Short cuts
 ***************************************************************************/

/*
 * CONTROL_THIS_CPU()
 *     Parameters
 *         None
 *     Returns
 *         CPU number of the processor being executed on
 *
 */
#define CONTROL_THIS_CPU()     smp_processor_id()

/****************************************************************************
 **  Interface definitions
 ***************************************************************************/

/*
 *  Execution Control Functions
 */

extern VOID
CONTROL_Invoke_Cpu (
    int   cpuid,
    VOID  (*func)(PVOID),
    PVOID ctx
);

/*
 * CONTROL_Invoke_Parallel_Service
 *     Parameters
 *         IN func     - function to be invoked by each core in the system
 *         IN ctx      - pointer to the parameter block for each function invocation
 *         IN blocking - Wait for invoced function to complete
 *         IN exclude  - exclude the current core from executing the code
 *     Returns
 *         None
 *
 *     Description
 *         Invoke the function provided in parallel in either a blocking/non-blocking mode
 *         The current core may be excluded if desired
 *         NOTE - Do not call this function directly from source code.  Use the aliases
 *         CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(), CONTROL_Invoke_Parallel_XS()
 *
 */
extern VOID
CONTROL_Invoke_Parallel_Service (
        VOID   (*func)(PVOID),
        PVOID  ctx,
        int    blocking,
        int    exclude
);

/*
 * CONTROL_Invoke_Parallel()
 *     Parameters
 *        func - name of function to invoke in parallel
 *        ctx  - parameters to pass to the function
 *
 *     Returns
 *        None
 *
 *     Description:
 *        Invoke the function named in parallel, including the CPU that the control is
 *        being invoked on
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel(a,b)      CONTROL_Invoke_Parallel_Service((a),(b),TRUE,FALSE)

/*
 * CONTROL_Invoke_Parallel_NB()
 *     Parameters
 *        func - name of function to invoke in parallel
 *        ctx  - parameters to pass to the function
 *
 *     Returns
 *        None
 *
 *     Description:
 *        Invoke the function named in parallel, including the CPU that the control is
 *        being invoked on.  This is a non blocking call.  Return control to the caller
 *        as soon as we are ready.
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel_NB(a,b)   CONTROL_Invoke_Parallel_Service((a),(b),FALSE,FALSE)

/*
 * CONTROL_Invoke_Parallel_XS()
 *     Parameters
 *        func - name of function to invoke in parallel
 *        ctx  - parameters to pass to the function
 *
 *     Returns
 *        None
 *
 *     Description:
 *        Invoke the function named in parallel, excluding the CPU that the control is
 *        being invoked on.
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel_XS(a,b)   CONTROL_Invoke_Parallel_Service((a),(b),TRUE,TRUE)

/*
 * CONTROL_Invoke_Serial
 *     Parameters
 *         IN func     - function to be invoked by each core in the system
 *         IN ctx      - pointer to the parameter block for each function invocation
 *     Returns
 *         None
 *
 *     Description
 *         Invoke the function provided in serial mode on all cores in the system
 *
 */
extern VOID
CONTROL_Invoke_Serial (
        VOID   (*func)(PVOID),
        PVOID  ctx
);

/*
 * @fn PVOID CONTROL_Allocate_Memory(size)
 *
 * @param    IN size     - size of the memory to allocate
 *
 * @returns  char*       - pointer to the allocated memory block
 *
 * @brief        Allocate and zero memory
 *
 *               Will fail if more than 128KB requested.
 *               CONTROL_Allocate_Large_Memory should be used in that case.
 *               Allocate memory in the GFP_KERNEL pool
 */
extern PVOID
CONTROL_Allocate_Memory (
    size_t    size
);

/*
 * @fn PVOID CONTROL_Allocate_KMemory(size)
 *
 * @param    IN size     - size of the memory to allocate
 *
 * @returns  char*       - pointer to the allocated memory block
 *
 * @brief        Allocate and zero memory
 *
 *               Will fail if more than 128KB requested.
 *               CONTROL_Allocate_Large_Memory should be used in that case.
 *               Allocate memory in the GFP_ATOMIC pool.
 *               This routine should be used jusiciously.
 */
extern PVOID
CONTROL_Allocate_KMemory (
    size_t  size
);

/*
 * @fn CONTROL_Free_Memory(location)
 *
 * @pararm IN   location - pointer to the memory block to free
 *
 * @brief       Frees the memory large block
 *
 *
 *         Frees the memory block and returns NULL.
 *         Does not try to free memory if fed with a NULL pointer
 *         Expected usage:
 *         ptr = CONTROL_Free_Memory(ptr);
 *
 */
extern PVOID
CONTROL_Free_Memory (
    PVOID    location
);

/*
 * @fn PVOID CONTROL_Allocate_Large_Memory(size)
 *
 * @param    IN size     - size of the memory to allocate
 *
 * @returns  char*       - pointer to the allocated memory block
 *
 *     Description
 *         Allocate the memory and zero it out
 *
 */
extern PVOID
CONTROL_Allocate_Large_Memory (
    size_t  size
);

/*
 * @fn CONTROL_Free_Large_Memory(location,size)
 *
 * @pararm IN   location - pointer to the memory block to free
 * @pararm IN   size     - in bytes
 *
 * @brief       Frees the memory large block
 *
 */
extern PVOID
CONTROL_Free_Large_Memory (
    PVOID  location,
    size_t size
);

#endif  /* _CONTROL_H_ */
