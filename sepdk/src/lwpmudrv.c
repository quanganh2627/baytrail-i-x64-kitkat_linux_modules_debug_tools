/*COPYRIGHT**
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
**COPYRIGHT*/

/*
 * cvs_id = "$Id$"
 */

#include "lwpmudrv_defines.h"
#include "lwpmudrv_version.h"

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/device.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_ioctl.h"
#include "lwpmudrv_struct.h"

#if defined(BUILD_GFX)
#include "gfx.h"
#endif

#if defined(BUILD_CHIPSET)
#include "lwpmudrv_chipset.h"
#include "pci.h"
#endif

#if defined(DRV_IA32) || defined(DRV_EM64T)
#include "apic.h"
#endif
#include "cpumon.h"
#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#if defined(DRV_IA32) || defined(DRV_EM64T)
#include "core.h"
#include "core2.h"
#endif
#include "pmi.h"

#if defined(DRV_IA64)
#include "montecito.h"
#include "poulson.h"
#endif
#include "output.h"
#include "linuxos.h"
#include "sys_info.h"
#include "eventmux.h"
#if defined(DRV_IA32) || defined(DRV_EM64T)
#include "pebs.h"
#endif

MODULE_AUTHOR("Copyright(c) 2007-2011 Intel Corporation");
MODULE_VERSION(SEP_NAME"_"SEP_VERSION_STR);
MODULE_LICENSE("Dual BSD/GPL");

typedef struct LWPMU_DEV_NODE_S  LWPMU_DEV_NODE;
typedef        LWPMU_DEV_NODE   *LWPMU_DEV;

struct LWPMU_DEV_NODE_S {
  long              buffer;
  struct semaphore  sem;
  struct cdev       cdev;
};

#define LWPMU_DEV_buffer(dev)      (dev)->buffer
#define LWPMU_DEV_sem(dev)         (dev)->sem
#define LWPMU_DEV_cdev(dev)        (dev)->cdev

/* Global variables of the driver */
SEP_VERSION_NODE        drv_version;
U64                    *read_counter_info     = NULL;
VOID                  **PMU_register_data     = NULL;
#if defined(DRV_IA32) || defined(DRV_EM64T)
#endif
VOID                  **desc_data             = NULL;
DISPATCH                dispatch              = NULL;
#if defined(DRV_IA32) || defined(DRV_EM64T)
#endif
U64                     total_ram             = 0;
U32                     output_buffer_size    = OUTPUT_LARGE_BUFFER;
static  S32             em_groups_count       = 0;
#if defined(DRV_IA32) || defined(DRV_EM64T)
#endif
static  S32             desc_count            = 0;
uid_t                   uid                   = 0;
EVENT_CONFIG            global_ec             = NULL;
DRV_CONFIG              pcfg                  = NULL;
#if defined(DRV_IA32) || defined(DRV_EM64T)
LBR                     lbr                   = NULL;
PWR                     pwr                   = NULL;
#else
RO                      ro                    = NULL;
#endif
LWPMU_DEV               lwpmu_control         = NULL;
LWPMU_DEV               lwmod_control         = NULL;
LWPMU_DEV               lwsamp_control        = NULL;

/* needed for multiple uncores */
U32                     num_devices            = 0;
U32                     cur_device             = 0;
LWPMU_DEVICE            devices                = NULL;
U32                     invoking_processor_id  = 0;

static   struct mutex   ioctl_lock;
volatile int            in_finish_code;

#define  PMU_DEVICES            2   // pmu, mod
#define  OTHER_PMU_DEVICES      1   // mod

#if defined(BUILD_CHIPSET)
CHIPSET_CONFIG          pma               = NULL;
CS_DISPATCH             cs_dispatch       = NULL;
#endif
static S8              *cpu_mask_bits     = NULL;

/*
 *  Global data: Buffer control structure
 */
BUFFER_DESC      cpu_buf    = NULL;
BUFFER_DESC      module_buf = NULL;

static dev_t     lwpmu_DevNum;  /* the major and minor parts for SEP3 base */
static dev_t     lwsamp_DevNum; /* the major and minor parts for SEP3 percpu */

#if defined (DRV_ANDROID)
#define DRV_DEVICE_DELIMITER "_"  
static struct class          *pmu_class   = NULL; 
#endif

extern volatile int      config_done;

CPU_STATE          pcb           = NULL;
size_t             pcb_size      = 0;
U64               *pmu_state     = NULL;

#if defined EMON
static U64              cpu0_TSC;
#endif

#if !defined(DRV_USE_UNLOCKED_IOCTL)
#define MUTEX_LOCK(lock, flags)
#define MUTEX_UNLOCK(lock, flags)
#else
#define MUTEX_LOCK(lock, flags)     mutex_lock(&(lock))
#define MUTEX_UNLOCK(lock, flags)   mutex_unlock(&(lock))
#endif


#if defined(DRV_IA32) || defined(DRV_EM64T)
/*
 * lwpmudrv_PWR_Info
 *
 *     Parameters
 *         IN: in_buf        - pointer to the input buffer
 *         IN: in_buf_len    - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Make a copy of the PWR information that is passed in.
 */
static OS_STATUS
lwpmudrv_PWR_Info (
    IOCTL_ARGS    arg
)
{
    if (DRV_CONFIG_power_capture(pcfg) == FALSE) {
        SEP_PRINT_WARNING("lwpmudrv_PWR_Info: PWR capture has not been configured\n");
        return OS_SUCCESS;
    }
    //
    // First things first: Make a copy of the data for global use.
    //
    pwr = CONTROL_Allocate_Memory((int)arg->w_len);
    if (copy_from_user(pwr, arg->w_buf, arg->w_len)) {
        return OS_FAULT;
    }

    return OS_SUCCESS;
}
#endif

/*
 * lwpmudrv_Initialize_State
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         int
 *
 *     Description
 *         Allocates the memory needed at load time.  Initializes all the
 *         necessary state variables with the default values.
 */
static int
lwpmudrv_Initialize_State (
    VOID
)
{
    /*
     *  Machine Initializations
     *  Abstract this information away into a separate entry point
     *
     *  Question:  Should we allow for the use of Hot-cpu
     *    add/subtract functionality while the driver is executing?
     */
    GLOBAL_STATE_num_cpus(driver_state)          = num_online_cpus();
    GLOBAL_STATE_active_cpus(driver_state)       = num_online_cpus();
    GLOBAL_STATE_cpu_count(driver_state)         = 0;
    GLOBAL_STATE_dpc_count(driver_state)         = 0;
    GLOBAL_STATE_num_em_groups(driver_state)     = 0;
    GLOBAL_STATE_current_phase(driver_state)     = DRV_STATE_UNINITIALIZED;

    SEP_PRINT_DEBUG("lwpmudrv_Initialize_State: num_cpus=%d, active_cpus=%d\n",
                    GLOBAL_STATE_num_cpus(driver_state),
                    GLOBAL_STATE_active_cpus(driver_state));

    return (OS_SUCCESS);
}


/*
 * lwpmudrv_Fill_TSC_Info
 *
 *     Parameters
 *         IN array of U64
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Read the TSC and write into the correct array slot.
 */
atomic_t read_now;
static wait_queue_head_t read_tsc_now;
static VOID
lwpmudrv_Fill_TSC_Info (
    PVOID   param
)
{
    U32      this_cpu;

    preempt_disable();
    this_cpu = CONTROL_THIS_CPU();
    preempt_enable();
    //
    // Wait until all CPU's are ready to proceed
    // This will serve as a synchronization point to compute tsc skews.
    //

    if (atomic_read(&read_now) >= 1) {
        if (atomic_dec_and_test(&read_now) == FALSE) {
            wait_event_interruptible(read_tsc_now, (atomic_read(&read_now) >= 1));
        }
    }
    else {
        atomic_dec(&read_now);
        wake_up_interruptible_all(&read_tsc_now);
    }
    UTILITY_Read_TSC(&CPU_STATE_tsc(&pcb[this_cpu]));
    SEP_PRINT_DEBUG("lwpmudrv_Fill_TSC_Info: this cpu %d --- tsc --- 0x%llx\n",
                    this_cpu, CPU_STATE_tsc(&pcb[this_cpu]));

    return;
}


/*********************************************************************
 *  Internal Driver functions
 *     Should be called only from the lwpmudrv_DeviceControl routine
 *********************************************************************/

/*
 * lwpmudrv_Version
 *
 *     Parameters
 *         IN: out_buf       - pointer to the output buffer
 *         IN: out_buf_len   - size of the output buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_VERSION call.
 *         Returns the version number of the kernel mode sampling.
 */
static OS_STATUS
lwpmudrv_Version (
    IOCTL_ARGS   arg
)
{
    OS_STATUS result = put_user(SEP_VERSION_NODE_sep_version(&drv_version), (U32 *)arg->r_buf);

    return result;
}

/*
 * lwpmudrv_Reserve
 *
 *     Parameters
 *         IN: out_buf       - pointer to the output buffer
 *         IN: out_buf_len   - size of the output buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_RESERVE call.
 *         Sets the state to RESERVED if possible.  Returns BUSY if unable
 *         to reserve the PMU.
 */
static OS_STATUS
lwpmudrv_Reserve (
    IOCTL_ARGS    arg
)
{
    OS_STATUS  status = OS_SUCCESS;
    S32        prev_phase;

    prev_phase = cmpxchg(&GLOBAL_STATE_current_phase(driver_state),
                         DRV_STATE_UNINITIALIZED,
                         DRV_STATE_RESERVED);

    SEP_PRINT_DEBUG("lwpmudrv_Reserve: states --- old_phase = %d; current_phase == %d\n",
            prev_phase, GLOBAL_STATE_current_phase(driver_state));

    status = put_user((prev_phase != DRV_STATE_UNINITIALIZED), (int*)arg->r_buf);

    return status;
}

/*
 * lwpmudrv_Initialize
 *
 *     Parameters
 *         IN: in_buf       - pointer to the input buffer
 *         IN: in_buf_len   - size of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_INIT call.
 *         Sets up the interrupt handler.
 *         Set up the output buffers/files needed to make the driver
 *         operational.
 */
static OS_STATUS
lwpmudrv_Initialize (
    PVOID         in_buf,
    size_t        in_buf_len
)
{
    U32        initialize;
    S32        cpu_num;

    SEP_PRINT_DEBUG("lwpmudrv_Initialize: Entered lwpmudrv_Initialize\n");
    initialize = cmpxchg(&GLOBAL_STATE_current_phase(driver_state),
                         DRV_STATE_RESERVED,
                         DRV_STATE_IDLE);

    if (initialize != DRV_STATE_RESERVED) {
        SEP_PRINT_ERROR("lwpmudrv_Initialize: Sampling is in progress, cannot start a new session.\n");
        return VT_SAM_ERROR;
    }

    /*
     *   Program State Initializations
     */
    pcfg = CONTROL_Allocate_Memory(in_buf_len);
    if (!pcfg) {
        return OS_NO_MEM;
    }

    if (copy_from_user(pcfg, in_buf, in_buf_len)) {
        return -EFAULT;
    }

    if (DRV_CONFIG_use_pcl(pcfg) == TRUE) {
        return OS_SUCCESS;
    }

    pcb_size = GLOBAL_STATE_num_cpus(driver_state)*sizeof(CPU_STATE_NODE);
    pcb      = CONTROL_Allocate_Large_Memory(pcb_size);
    if (!pcb) {
        return OS_NO_MEM;
    }

    pmu_state = CONTROL_Allocate_KMemory(GLOBAL_STATE_num_cpus(driver_state)*sizeof(U64)*2);
    if (!pmu_state) {
        return OS_NO_MEM;
    }

    for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state); cpu_num++) {
        CPU_STATE_accept_interrupt(&pcb[cpu_num]) = 1;
    }

    dispatch = UTILITY_Configure_CPU(DRV_CONFIG_dispatch_id(pcfg));
    if (dispatch == NULL) {
        return OS_FAULT;
    }

    SEP_PRINT_DEBUG("Config : size = %d\n", DRV_CONFIG_size(pcfg));
    SEP_PRINT_DEBUG("Config : counting_mode = %ld\n", DRV_CONFIG_counting_mode(pcfg));
#if defined(DRV_IA32) || defined(DRV_EM64T)
    SEP_PRINT_DEBUG("Config : pebs_mode = %ld\n", DRV_CONFIG_pebs_mode(pcfg));
    SEP_PRINT_DEBUG("Config : pebs_capture = %ld\n", DRV_CONFIG_pebs_capture(pcfg));
    SEP_PRINT_DEBUG("Config : collect_lbrs = %ld\n", DRV_CONFIG_collect_lbrs(pcfg));
#else
    SEP_PRINT_DEBUG("Config : collect_ro = %ld\n", DRV_CONFIG_collect_ro(pcfg));
#endif
    SEP_PRINT_DEBUG("Config : seed_name = %s\n", DRV_CONFIG_seed_name(pcfg));

#if defined(PID_CHECK)
    lwpmudrv_Set_Current_Pid();
#endif

    if (DRV_CONFIG_counting_mode(pcfg) == FALSE) {
        if (cpu_buf == NULL) {
            cpu_buf    = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state)*sizeof(BUFFER_DESC_NODE));
        }
        /*
         * Program the APIC and set up the interrupt handler
         */
        CPUMON_Install_Cpuhooks();
        SEP_PRINT_DEBUG("lwpmudrv_Initialize: Finished Installing cpu hooks\n");
        /*
         * Set up the call back routines based on architecture.
         */
#if defined(DRV_IA32) || defined(DRV_EM64T)
        PEBS_Initialize(pcfg);
#endif

#if defined(DRV_EM64T)
        SYS_Get_GDT_Base((PVOID*)&gdt_desc);
#endif
        /*
         * Allocate the output and control buffers for each CPU in the system
         * Allocate and set up the temp output files for each CPU in the system
         * Allocate and set up the temp outout file for detailing the Modules in the system
         */
        OUTPUT_Initialize(DRV_CONFIG_seed_name(pcfg), DRV_CONFIG_seed_name_len(pcfg));
        SEP_PRINT_DEBUG("lwpmudrv_Initialize: After OUTPUT_Initialize\n");
        SEP_PRINT_DEBUG("lwpmudrv_Initialize: about to install module notification");
        LINUXOS_Install_Hooks();
    }

    return OS_SUCCESS;
}

#if defined(DRV_IA32) || defined(DRV_EM64T)
/*
 * lwpmudrv_Initialize_Num_Devices
 *
 *     Parameters
 *         IN: in_buf       - pointer to the input buffer
 *         IN: in_buf_len   - size of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_INIT_NUM_DEV call.
 *         Init # uncore devices.
 */
static OS_STATUS
lwpmudrv_Initialize_Num_Devices (
    IOCTL_ARGS arg
)
{
    if (copy_from_user(&num_devices, arg->w_buf, arg->w_len)) {
        return OS_FAULT;
    }
    /*
     *   Allocate memory for number of devices
     */
    devices = CONTROL_Allocate_Memory(num_devices * sizeof(LWPMU_DEVICE_NODE));
    cur_device = 0;

    SEP_PRINT_DEBUG("lwpmudrv_Initialize_Num_Devices: num_devices=%d, devices=0x%p\n", num_devices, devices);
    
    return OS_SUCCESS;
}

/*
 * lwpmudrv_Initialize_UNC
 *
 *     Parameters
 *         IN: in_buf       - pointer to the input buffer
 *         IN: in_buf_len   - size of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_INIT call.
 *         Sets up the interrupt handler.
 *         Set up the output buffers/files needed to make the driver
 *         operational.
 */
static OS_STATUS
lwpmudrv_Initialize_UNC (
    PVOID         in_buf,
    U32           in_buf_len
)
{
    DRV_CONFIG  pcfg;

    SEP_PRINT_DEBUG("Entered lwpmudrv_Initialize_UNC\n");
    
    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    /*
     *   Program State Initializations:
     *   Foreach device, copy over pcfg and configure dispatch table
     */
    if (cur_device >= num_devices) {
        SEP_PRINT_ERROR("No more devices to allocate!  Initial call to lwpmudrv_Init_Num_Devices incorrect.");
        return OS_FAULT;
    }
    if (in_buf_len != sizeof(DRV_CONFIG_NODE)) {
        SEP_PRINT_ERROR("Got in_buf_len=%d, expecting size=%d\n", in_buf_len, (int)sizeof(DRV_CONFIG_NODE));
        return OS_FAULT;
    }
    // allocate memory
    LWPMU_DEVICE_pcfg(&devices[cur_device]) = CONTROL_Allocate_Memory(sizeof(DRV_CONFIG_NODE));
    // copy over pcfg
    if (copy_from_user(LWPMU_DEVICE_pcfg(&devices[cur_device]), in_buf, in_buf_len)) {
        SEP_PRINT_ERROR("Failed to copy from user");
        return OS_FAULT;
    }
    // configure dispatch from dispatch_id
    pcfg = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[cur_device]);
    LWPMU_DEVICE_dispatch(&devices[cur_device]) = UTILITY_Configure_CPU(DRV_CONFIG_dispatch_id(pcfg));
    if (LWPMU_DEVICE_dispatch(&devices[cur_device]) == NULL) {
        SEP_PRINT_ERROR("Unable to configure CPU");
        return OS_FAULT;
    }

    SEP_PRINT_DEBUG("LWP: ebc unc = %d\n",DRV_CONFIG_event_based_counts(pcfg));
    SEP_PRINT_DEBUG("LWP Config : unc dispatch id   = %d\n", DRV_CONFIG_dispatch_id(pcfg));

    return OS_SUCCESS;
}
#endif

/*
 * lwpmudrv_Terminate
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMUDRV_IOCTL_TERMINATE call.
 *         Cleans up the interrupt handler and resets the PMU state.
 *
 */
static OS_STATUS
lwpmudrv_Terminate (
    VOID
)
{
    U32  previous_state;
    U32  i;

    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_UNINITIALIZED) {
        return OS_SUCCESS;
    }

    previous_state = cmpxchg(&GLOBAL_STATE_current_phase(driver_state),
                             DRV_STATE_STOPPED,
                             DRV_STATE_UNINITIALIZED);
    if (previous_state != DRV_STATE_STOPPED) {
        SEP_PRINT_ERROR("lwpmudrv_Terminate: Sampling is in progress, cannot terminate.\n");
        return VT_SAM_ERROR;
    }

    GLOBAL_STATE_current_phase(driver_state) = DRV_STATE_UNINITIALIZED;
    if (DRV_CONFIG_use_pcl(pcfg) == TRUE) {
        pcfg = CONTROL_Free_Memory(pcfg);
        goto signal_end;
    }

    if (PMU_register_data) {
        for (i = 0; i < GLOBAL_STATE_num_em_groups(driver_state); i++) {
            CONTROL_Free_Memory(PMU_register_data[i]);
        }
    }

#if defined(DRV_IA32) || defined(DRV_EM64T)
    if (devices) {
        U32 id = 0;
        for (id = 0; id < num_devices; id++) {
            if (LWPMU_DEVICE_PMU_register_data(&devices[id])) {
                for (i = 0; i < EVENT_CONFIG_num_groups_unc(global_ec); i++) {
                    CONTROL_Free_Memory(LWPMU_DEVICE_PMU_register_data(&devices[id])[i]);
                }
                LWPMU_DEVICE_PMU_register_data(&devices[id]) = CONTROL_Free_Memory(LWPMU_DEVICE_PMU_register_data(&devices[id]));
            }
            LWPMU_DEVICE_pcfg(&devices[id]) = CONTROL_Free_Memory(LWPMU_DEVICE_pcfg(&devices[id]));

            if (LWPMU_DEVICE_acc_per_thread(&devices[id])) {
                for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                    LWPMU_DEVICE_acc_per_thread(&devices[id])[i] = CONTROL_Free_Memory(LWPMU_DEVICE_acc_per_thread(&devices[id])[i]);
                }
                LWPMU_DEVICE_acc_per_thread(&devices[id]) = CONTROL_Free_Memory(LWPMU_DEVICE_acc_per_thread(&devices[id]));
            }
            if (LWPMU_DEVICE_prev_val_per_thread(&devices[id])) {
                for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                    LWPMU_DEVICE_prev_val_per_thread(&devices[id])[i] = CONTROL_Free_Memory(LWPMU_DEVICE_prev_val_per_thread(&devices[id])[i]);
                }
                LWPMU_DEVICE_prev_val_per_thread(&devices[id]) = CONTROL_Free_Memory(LWPMU_DEVICE_prev_val_per_thread(&devices[id]));
            }
        }
    }
#endif

    if (desc_data) {
        for (i = 0; i < GLOBAL_STATE_num_descriptors(driver_state); i++) {
            CONTROL_Free_Memory(desc_data[i]);
        }
    }
    PMU_register_data       = CONTROL_Free_Memory(PMU_register_data);
    desc_data               = CONTROL_Free_Memory(desc_data);
    global_ec               = CONTROL_Free_Memory(global_ec);
    pcfg                    = CONTROL_Free_Memory(pcfg);
#if defined(DRV_IA32) || defined(DRV_EM64T)
    lbr                     = CONTROL_Free_Memory(lbr);
#else
    ro                      = CONTROL_Free_Memory(ro);
#endif
    CONTROL_Invoke_Parallel(dispatch->fini, NULL); // must be done before pcb is freed
    pcb                     = CONTROL_Free_Large_Memory(pcb, pcb_size);
    pcb_size                = 0;
    pmu_state               = CONTROL_Free_Memory(pmu_state);
    cpu_mask_bits           = CONTROL_Free_Memory(cpu_mask_bits);

signal_end:
#if defined(PID_CHECK)
    lwpmudrv_Reset_Current_Pid();
#endif
    GLOBAL_STATE_num_em_groups(driver_state)   = 0;
    GLOBAL_STATE_num_descriptors(driver_state) = 0;
    num_devices                                = 0;

    return OS_SUCCESS;
}

#ifdef EMON
/*
 * lwpmudrv_Switch_To_Next_Group
 *
 *     Parameters
 *         UNUSED
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Switch to the next event group.
 *         This function assumes an active collection is frozen
 *         or no collection is active.
 */
static VOID
lwpmudrv_Switch_To_Next_Group (
    PVOID  param
)
{
    U32            this_cpu;
    CPU_STATE      pcpu;

    preempt_disable();
    this_cpu = CONTROL_THIS_CPU();
    pcpu     = &pcb[this_cpu];
    preempt_enable();

    CPU_STATE_current_group(pcpu)++;
    // make the event group list circular
    CPU_STATE_current_group(pcpu) %= EVENT_CONFIG_num_groups(global_ec);

    return;
}
#endif // EMON

/*
 * lwpmudrv_Get_Driver_State
 *
 *     Parameters
 *         IN: arg       - Output buffer args
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_GET_Driver_State call.
 *         Returns the current driver state.
 */
static OS_STATUS
lwpmudrv_Get_Driver_State (
    IOCTL_ARGS    arg
)
{
    OS_STATUS  status = OS_SUCCESS;

    status = put_user(GLOBAL_STATE_current_phase(driver_state), (U32*)arg->r_buf);

    return status;
}

/*
 * lwpmudrv_Pause
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Pause the collection.
 */
static OS_STATUS
lwpmudrv_Pause (
    VOID
)
{
    U32  previous_state;
    int  i;
    int  done = FALSE;
#if defined(DRV_IA32) || defined(DRV_EM64T)
    DRV_CONFIG pcfg_unc = NULL;
    DISPATCH   dispatch_unc = NULL;
    U32        j;
#endif

    previous_state = cmpxchg(&GLOBAL_STATE_current_phase(driver_state),
                             DRV_STATE_RUNNING,
                             DRV_STATE_PAUSING);

    if (previous_state == DRV_STATE_RUNNING) {
        if (DRV_CONFIG_use_pcl(pcfg) == FALSE) {
            for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                CPU_STATE_accept_interrupt(&pcb[i]) = 0;
            }
            while (!done) {
                done = TRUE;
                for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                    if (atomic_read(&CPU_STATE_in_interrupt(&pcb[i]))) {
                        done = FALSE;
                    }
                }
            }
            CONTROL_Invoke_Parallel(dispatch->freeze, NULL);
        }
        /*
         * This means that the PAUSE state has been reached.
         */
        GLOBAL_STATE_current_phase(driver_state) = DRV_STATE_PAUSED;
#if defined(DRV_IA32) || defined(DRV_EM64T)
        for (j = 0; j < num_devices; j++) {
             pcfg_unc = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[j]);
             dispatch_unc = LWPMU_DEVICE_dispatch(&devices[j]);

             if (pcfg_unc                                &&
                 DRV_CONFIG_event_based_counts(pcfg_unc) &&
                 dispatch_unc                            &&
                 dispatch_unc->freeze) {
                    SEP_PRINT_DEBUG("LWP: calling UNC Pause\n");
                    preempt_disable();
                    invoking_processor_id = CONTROL_THIS_CPU();
                    preempt_enable();
                    CONTROL_Invoke_Parallel(dispatch_unc->freeze, (VOID *)&j);
             }
         }
#endif
    }

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Resume
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Resume the collection.
 */
static OS_STATUS
lwpmudrv_Resume (
    VOID
)
{
    U32  previous_state;
    int  i;
#if defined(DRV_IA32) || defined(DRV_EM64T)
    DRV_CONFIG pcfg_unc = NULL;
    DISPATCH   dispatch_unc = NULL;
    U32        j;
#endif

    /*
     * If we are in the process of pausing sampling, wait until the pause has been
     * completed.  Then start the Resume process.
     */
    do {
        previous_state = cmpxchg(&GLOBAL_STATE_current_phase(driver_state),
                                 DRV_STATE_PAUSED,
                                 DRV_STATE_RUNNING);
        /*
         *  This delay probably needs to be expanded a little bit more for large systems.
         *  For now, it is probably sufficient.
         */
        SYS_IO_Delay();
        SYS_IO_Delay();
    } while (previous_state == DRV_STATE_PAUSING);

    if (previous_state == DRV_STATE_PAUSED) {
        if (cpu_mask_bits) {
            for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                CPU_STATE_accept_interrupt(&pcb[i]) = cpu_mask_bits[i] ? 1 : 0;
            }
        }
        else {
            for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                CPU_STATE_accept_interrupt(&pcb[i]) = 1;
            }
        }
        if (DRV_CONFIG_use_pcl(pcfg) == FALSE) {
            CONTROL_Invoke_Parallel(dispatch->restart, (VOID *)(size_t)0);
        }
#if defined(DRV_IA32) || defined(DRV_EM64T)
       for (j = 0; j < num_devices; j++) {
            pcfg_unc = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[j]);
            dispatch_unc = LWPMU_DEVICE_dispatch(&devices[j]);

            if (pcfg_unc                                &&
                DRV_CONFIG_event_based_counts(pcfg_unc) &&
                dispatch_unc                            &&
                dispatch_unc->restart) {
                   SEP_PRINT_DEBUG("LWP: calling UNC Resume\n");
                   CONTROL_Invoke_Parallel(dispatch_unc->restart, (VOID *)&j);
            }
       }
#endif
    }

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Switch_Group
 *
 * Abstract
 *     This routine is called from the user mode code to handle the multiple group
 *     situation.  4 distinct steps are taken:
 *     Step 1: Pause the sampling
 *     Step 2: Increment the current group count
 *     Step 3: Write the new group to the PMU
 *     Step 4: Resume sampling
 *
 * Parameters
 *     None
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Switch_Group (
    VOID
)
{
    S32            idx;
    CPU_STATE      pcpu;

    lwpmudrv_Pause();
    for (idx = 0; idx < GLOBAL_STATE_num_cpus(driver_state); idx++) {
        pcpu = &pcb[idx];
        CPU_STATE_current_group(pcpu)++;
    }
    CONTROL_Invoke_Parallel(dispatch->write, (VOID *)0);
    lwpmudrv_Resume();

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Init_PMU
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Initialize the PMU state.
 */
static OS_STATUS
lwpmudrv_Init_PMU (
    VOID
)
{
#if defined(DRV_IA32) || defined(DRV_EM64T)
    DRV_CONFIG pcfg_unc = NULL;
    DISPATCH   dispatch_unc = NULL;
    U32        i;
#endif

    if (pcfg && DRV_CONFIG_use_pcl(pcfg) == TRUE) {
        return OS_SUCCESS;
    }

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }

    if (GLOBAL_STATE_num_em_groups(driver_state) == 0) {
        SEP_PRINT_ERROR("Number of em groups is not set.\n");
        return OS_SUCCESS;
    }

    // must be done after pcb is created and before PMU is first written to
    CONTROL_Invoke_Parallel(dispatch->init, NULL);

    //
    // Transfer the data into the PMU registers
    //
    CONTROL_Invoke_Parallel(dispatch->write, (VOID *)(size_t)0);

#if defined(DRV_IA32) || defined(DRV_EM64T)
    for (i = 0; i < num_devices; i++) {
        pcfg_unc = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
        dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);
        if (pcfg_unc && DRV_CONFIG_event_based_counts(pcfg_unc) && dispatch_unc && dispatch_unc->write) {
            SEP_PRINT_DEBUG("LWP: calling UNC Init\n");
            preempt_disable();
            invoking_processor_id = CONTROL_THIS_CPU();
            preempt_enable();
            CONTROL_Invoke_Parallel(dispatch_unc->write, (VOID *)&i);
        }
    }
#endif

    SEP_PRINT_DEBUG("lwpmudrv_Init_PMU: IOCTL_Init_PMU - finished initial Write\n");

    return OS_SUCCESS;
}

#ifdef EMON
/*
 * lwpmudrv_Read_MSR
 *
 *     Parameters
 *         IN:  in_buf       - pointer to the MSR address
 *         OUT: out_buf      - pointer to the output buffer
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Read the U64 value at address in in_buf and
 *         write the result into out_buf.
 */
static VOID
lwpmudrv_Read_MSR (
    PVOID param
)
{
#if defined(DRV_IA32) || defined(DRV_EM64T)
    U32       this_cpu;
    MSR_DATA  this_node;
    S64       reg_num;

    preempt_disable();
    this_cpu  = CONTROL_THIS_CPU();
    this_node = &msr_data[this_cpu];
    reg_num = MSR_DATA_addr(this_node);

    if (reg_num == 0) {
      preempt_enable();
      return;
    }
    
    MSR_DATA_value(this_node) = (U64)SYS_Read_MSR((U32)MSR_DATA_addr(this_node));
    preempt_enable();
#endif

    return;
}

/*
 * lwpmudrv_Read_MSR_All_Cores
 *
 *     Parameters
 *         IN:  in_buf       - pointer to the MSR address
 *         OUT: out_buf      - pointer to the output buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Read the U64 value at address into in_buf and write
 *         the result into out_buf.
 *         Returns OS_SUCCESS if the read across all cores succeed,
 *         otherwise OS_FAULT.
 */
static OS_STATUS
lwpmudrv_Read_MSR_All_Cores (
    IOCTL_ARGS    arg
)
{
    U64            *val;
    S32            *reg_num;
    S32             i;
    MSR_DATA        node;


    val     = (U64 *)arg->w_buf;
    reg_num = (U32 *)arg->r_buf;

    if (val == NULL)  {
        SEP_PRINT_ERROR("NULL out_buf\n");
        return OS_SUCCESS;
    }
    
    msr_data = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state)*sizeof(MSR_DATA_NODE));
    if (!msr_data) {
        return OS_NO_MEM;
    }

    for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
        node                = &msr_data[i];
        MSR_DATA_addr(node) = *reg_num;
    }

    CONTROL_Invoke_Parallel(lwpmudrv_Read_MSR, (VOID *)(size_t)0);

    /* copy values to arg array? */
    if (arg->w_len < GLOBAL_STATE_num_cpus(driver_state)) {
        SEP_PRINT_ERROR("Not enough memory allocated in output buffer.\n");
        msr_data = CONTROL_Free_Memory(msr_data);
        return OS_SUCCESS;
    }
    for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
        node = &msr_data[i];
        if (copy_to_user(&val[i], (U64*)&MSR_DATA_value(node), sizeof(U64))) {
            return OS_FAULT;
        }
    }

    msr_data = CONTROL_Free_Memory(msr_data);

    return OS_SUCCESS;
}

#ifdef EMON_INTERNAL
/*
 * lwpmudrv_Write_MSR
 *
 *     Parameters
 *         IN:  in_buf - array containing the MSR address and value to be written
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Read the U64 value at address in in_buf
 *         and write the result into out_buf.
 */
static VOID
lwpmudrv_Write_MSR (
    PVOID param
)
{
#if defined(DRV_IA32) || defined(DRV_EM64T)
    U32       this_cpu;
    MSR_DATA  this_node;
    U32       reg_num;
    U64       val;

    preempt_disable();
    this_cpu  = CONTROL_THIS_CPU();
    this_node = &msr_data[this_cpu];
    reg_num   = (U32)MSR_DATA_addr(this_node);
    val       = (U64)MSR_DATA_value(this_node);
    // don't attempt to write MSR 0
    if (reg_num == 0) {
        preempt_enable();
        return;
    }

    SYS_Write_MSR(reg_num, val);
    preempt_enable();
#endif

    return;
}

/*
 * lwpmudrv_Write_MSR_All_Cores
 *
 *     Parameters
 *         IN:  in_buf - array containing the MSR address and value to be written
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Read the U64 value at address in in_buf and write
 *         the result into out_buf.
 *         Returns OS_SUCCESS if the write across all cores succeeds,
 *         otherwise OS_FAULT.
 */
static OS_STATUS
lwpmudrv_Write_MSR_All_Cores (
    IOCTL_ARGS    arg
)
{
    EVENT_REG       in_buf;
    U32             reg_num;
    U64             val;
    S32             i;
    MSR_DATA        node;

    in_buf  = (EVENT_REG)arg->w_buf;
    reg_num = (U32)EVENT_REG_reg_id(in_buf,0);
    val     = (U64)EVENT_REG_reg_value(in_buf,0);

    msr_data = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state)*sizeof(MSR_DATA_NODE));
    if (!msr_data) {
        return OS_NO_MEM;
    }

    for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
        node                 = &msr_data[i];
        MSR_DATA_addr(node)  = reg_num;
        MSR_DATA_value(node) = val;
    }

    CONTROL_Invoke_Parallel(lwpmudrv_Write_MSR, (VOID *)(size_t)0);

    msr_data = CONTROL_Free_Memory(msr_data);

    return OS_SUCCESS;
}
#endif // EMON_INTERNAL
#endif // EMON

/*
 * lwpmudrv_Read_MSRs
 *
 *     Parameters
 *         IN: out_buf       - pointer to the output buffer
 *         IN: out_buf_len   - length of the output buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Read all the programmed data counters and accumulate them
 *         into a single buffer.
 */
static OS_STATUS
lwpmudrv_Read_MSRs (
    IOCTL_ARGS    arg
)
{
    OS_STATUS  status = OS_SUCCESS;

    if (arg->r_len == 0) {
        return status;
    }
    //
    // Transfer the data in the PMU registers to the output buffer
    //
    read_counter_info = CONTROL_Allocate_Memory(arg->r_len);
    CONTROL_Invoke_Parallel(dispatch->read_data, (VOID *)(size_t)0);
    if (copy_to_user(arg->r_buf, read_counter_info, arg->r_len)) {
        status = OS_FAULT;
    }
    read_counter_info = CONTROL_Free_Memory(read_counter_info);

    return status;
}

#ifdef EMON
/*
 * lwpmudrv_Read_Specific_TSC
 *
 *     Parameters
 *         IN:  param       - pointer to result
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Read the tsc value in the current processor and
 *         write the result into param.
 */
static VOID
lwpmudrv_Read_Specific_TSC (
    PVOID  param
)
{
    U32 this_cpu;

    preempt_disable();
    this_cpu = CONTROL_THIS_CPU();
    if (this_cpu == 0) {
        UTILITY_Read_TSC((U64*)param);
    }
    preempt_enable();

    return;
}

/*
 * lwpmudrv_Read_Counters_And_Switch_Group
 *
 * Abstract
 *     This routine is called from the user mode code to handle the multiple group
 *     situation.  9 distinct steps are taken:
 *     Step 1: Save the previously collected CPU 0 TSC value
 *     Step 2: Read CPU 0's TSC
 *     Step 3: Calculate the difference between the TSCs and save in the first U64 of the buffer
 *     Step 4: Save previous buffer ptr and Increment the buffer
 *     Step 5: Pause the counting PMUs
 *     Step 6: Read the currently programmed data PMUs and copy the data into the output buffer
 *             Restore the old buffer ptr.
 *     Step 7: Increment the current group count
 *     Step 8: Write the new group to the PMU
 *     Step 9: Resume the counting PMUs
 *
 * Parameters
 *     None
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Read_Counters_And_Switch_Group (
    IOCTL_ARGS arg
)
{
    U64            prev_tsc, diff;
    U64           *pBuffer;
    U32            this_cpu;
    char          *orig_r_buf_ptr;

    preempt_disable();
    this_cpu = CONTROL_THIS_CPU();
    preempt_enable();
    // step 1
    prev_tsc = cpu0_TSC;

    // step 2
    // read CPU 0's tsc into the global var cpu0_TSC
    // if running on cpu 0, read the tsc directly, else schedule a dpc

    if (this_cpu == 0) {
        UTILITY_Read_TSC(&cpu0_TSC);
    } 
    else {
        CONTROL_Invoke_Cpu (0, lwpmudrv_Read_Specific_TSC, &cpu0_TSC);
    }

    // step 3
    // get tsc diff (i.e. clocks during this monitor interval)
    diff = cpu0_TSC - prev_tsc;

    // save diff in first slot in buffer
    pBuffer = (U64*)(arg->r_buf);
    *pBuffer = diff;

    // step 4
    orig_r_buf_ptr = arg->r_buf;
    pBuffer += 1;
    arg->r_buf = (char *)pBuffer;
    // step 5
    lwpmudrv_Pause();
    // step 6
    lwpmudrv_Read_MSRs(arg);
    arg->r_buf = orig_r_buf_ptr;

    // step 7
    // for each processor, increment its current group number
    CONTROL_Invoke_Parallel(lwpmudrv_Switch_To_Next_Group, (VOID *)(size_t)0);
    // step 8
    CONTROL_Invoke_Parallel(dispatch->write, (VOID *)(size_t)0);
    // step 9
    lwpmudrv_Resume();

    return OS_SUCCESS;
}

/*
 * @fn        lwpmudrv_Read_And_Reset_Counters
 *
 * @brief     This routine is called from the user mode code to handle overflows
 *            It basically does the same as the lwpmudrv_Read_Counters_And_Switch_Group
 *            routine except it doesn't switch groups.
 *            Step 1: Save the previously collected CPU 0 TSC value
 *            Step 2: Read CPU 0's TSC
 *            Step 3: Calculate the difference between the TSCs and save in the first U64 of the buffer
 *            Step 4: Save previous buffer ptr and Increment the buffer
 *            Step 5: Pause the counting PMUs
 *            Step 6: Read the currently programmed data PMUs and copy the data into the output buffer
 *                    Restore the old buffer ptr.
 *            Step 7: Write the new group to the PMU
 *            Step 8: Resume the counting PMUs
 *
 * @param     None
 *
 * @return    OS_STATUS
 *
 * <I>Special Notes:</I>
 *     <none>
 */
static OS_STATUS
lwpmudrv_Read_And_Reset_Counters (
    IOCTL_ARGS arg
)
{
    U64            prev_tsc, diff;
    U64           *p_buffer;
    U32            this_cpu;
    char          *orig_r_buf_ptr;

    preempt_disable();
    this_cpu = CONTROL_THIS_CPU();
    preempt_enable();
    // step 1
    prev_tsc = cpu0_TSC;

    // step 2
    // read CPU 0's tsc into the global var cpu0_TSC
    // if running on cpu 0, read the tsc directly, else schedule a dpc

    if (this_cpu == 0) {
        UTILITY_Read_TSC(&cpu0_TSC);
    } 
    else {
        CONTROL_Invoke_Cpu (0, lwpmudrv_Read_Specific_TSC, &cpu0_TSC);
    }

    // step 3
    // get tsc diff (i.e. clocks during this monitor interval)
    diff = cpu0_TSC - prev_tsc;

    // save diff in first slot in buffer
    p_buffer = (U64*)(arg->r_buf);
    *p_buffer = diff;

    // step 4
    orig_r_buf_ptr = arg->r_buf;
    p_buffer += 1;
    arg->r_buf = (char *)p_buffer;
    // step 5
    lwpmudrv_Pause();
    // step 6
    lwpmudrv_Read_MSRs(arg);
    arg->r_buf = orig_r_buf_ptr;

    // step 8
    CONTROL_Invoke_Parallel(dispatch->write, (VOID *)(size_t)0);
    // step 9
    lwpmudrv_Resume();

    return OS_SUCCESS;
}
#endif // EMON

/*
 * lwpmudrv_Set_Num_EM_Groups
 *
 *     Parameters
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Configure the event multiplexing group.
 */
static OS_STATUS
lwpmudrv_Set_EM_Config (
    IOCTL_ARGS arg
)
{
    S32  cpu_num;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (arg->w_buf == NULL) {
        SEP_PRINT_DEBUG("lwpmudrv_Set_EM_Config: set_num_em_groups got null pointer\n");
        return OS_NO_MEM;
    }
    if (arg->w_len != sizeof(EVENT_CONFIG_NODE)) {
        SEP_PRINT_ERROR("lwpmudrv_Set_EM_Config: Unknown size of value passed to IOCTL_EM_CONFIG: %lld\n", arg->w_len);
        return OS_NO_MEM;
    }
    global_ec = CONTROL_Allocate_Memory(sizeof(EVENT_CONFIG_NODE));
    if (!global_ec) {
        return OS_NO_MEM;
    }

    if (copy_from_user(global_ec, arg->w_buf, sizeof(EVENT_CONFIG_NODE))) {
        return OS_FAULT;
    }
    for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state); cpu_num++) {
        CPU_STATE_trigger_count(&pcb[cpu_num])     = EVENT_CONFIG_em_factor(global_ec);
        CPU_STATE_trigger_event_num(&pcb[cpu_num]) = EVENT_CONFIG_em_event_num(global_ec) ;
    }
    GLOBAL_STATE_num_em_groups(driver_state) = EVENT_CONFIG_num_groups(global_ec);
    PMU_register_data = CONTROL_Allocate_Memory(GLOBAL_STATE_num_em_groups(driver_state) *
                                                sizeof(VOID *));
    EVENTMUX_Initialize(global_ec);

    em_groups_count = 0;

    return OS_SUCCESS;
}

#if defined(DRV_IA32) || defined(DRV_EM64T)
/*
 * lwpmudrv_Set_EM_Config_UNC
 *
 *     Parameters
 *         IN:  arg->w_buf       - pointer to the input buffer
 *         IN:  arg->w_len       - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Set the number of em groups in the global state node.
 *         Also, copy the EVENT_CONFIG struct that has been passed in,
 *         into a global location for now.
 */
static OS_STATUS
lwpmudrv_Set_EM_Config_UNC (
    IOCTL_ARGS arg
)
{
    SEP_PRINT_DEBUG("enter lwpmudrv_Set_EM_Config_UNC\n");
    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }

    SEP_PRINT_DEBUG("Num Groups UNCORE: %d\n", EVENT_CONFIG_num_groups_unc(global_ec));    

    LWPMU_DEVICE_PMU_register_data(&devices[cur_device]) = CONTROL_Allocate_Memory(EVENT_CONFIG_num_groups_unc(global_ec) *
                                                                                   sizeof(VOID *));
    
    LWPMU_DEVICE_em_groups_count(&devices[cur_device]) = 0;

    return OS_SUCCESS;
}
#endif

/*
 * lwpmudrv_Configure_Events
 *
 *     Parameter - pointer to struct
 *         int    byte_size
 *         char   data[byte_size]
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Copies one group of events into kernel space at
 *         PMU_register_data[em_groups_count].
 */
static OS_STATUS
lwpmudrv_Configure_Events (
    IOCTL_ARGS arg
)
{
    int  uncopied;

    SEP_PRINT_DEBUG("lwpmudrv_Configure_Events: entered.\n");
    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }

    if (em_groups_count >= GLOBAL_STATE_num_em_groups(driver_state)) {
        SEP_PRINT_ERROR("lwpmudrv_Configure_Events: Number of EM groups exceeded the initial configuration.");
        return OS_SUCCESS;
    }

    PMU_register_data[em_groups_count] = CONTROL_Allocate_Memory(arg->w_len);
    if (!PMU_register_data[em_groups_count]) {
        return OS_NO_MEM;
    }
    //
    // Make a copy of the data for global use.
    //
    uncopied = copy_from_user(PMU_register_data[em_groups_count], arg->w_buf, arg->w_len);
    if (uncopied > 0) {
        return OS_NO_MEM;  // TODO: should be some other error code
    }
    em_groups_count++;

    return OS_SUCCESS;
}

#if defined(DRV_IA32) || defined(DRV_EM64T)
/*
 * lwpmudrv_Configure_Events_UNC
 *
 *     Parameters
 *         IN: in_buf        - pointer to the input buffer
 *         IN: in_buf_len    - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Make a copy of the uncore registers that need to be programmed
 *         for the next event set used for event multiplexing
 */
static OS_STATUS
lwpmudrv_Configure_Events_UNC (
    IOCTL_ARGS arg
)
{
    VOID **PMU_register_data_unc;
    S32    em_groups_count_unc;
    ECB    ecb;
    S32    i;
    U32    j;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }

    em_groups_count_unc = LWPMU_DEVICE_em_groups_count(&devices[cur_device]);
    PMU_register_data_unc = LWPMU_DEVICE_PMU_register_data(&devices[cur_device]);
      
    if (em_groups_count_unc >= GLOBAL_STATE_num_em_groups(driver_state)) {
        SEP_PRINT_DEBUG("Number of Uncore EM groups exceeded the initial configuration.");
        return OS_SUCCESS;
    }
 
    //       size is in w_len, data is pointed to by w_buf
    //
    PMU_register_data_unc[em_groups_count_unc] = CONTROL_Allocate_Memory(arg->w_len);
    if (!PMU_register_data_unc[em_groups_count_unc]) {
        return OS_NO_MEM;
    }

    //
    // Make a copy of the data for global use.
    //
    if (copy_from_user(PMU_register_data_unc[em_groups_count_unc], arg->w_buf, arg->w_len)) {
        return OS_NO_MEM;
    }

    // at this point, we know the number of uncore events for this device, 
    // so allocate the results buffer per thread for uncore
    ecb = PMU_register_data_unc[0];
    LWPMU_DEVICE_num_events(&devices[cur_device]) = ECB_num_events(ecb);

    LWPMU_DEVICE_acc_per_thread(&devices[cur_device]) = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state) *
                                                                                sizeof(VOID *));
    LWPMU_DEVICE_prev_val_per_thread(&devices[cur_device]) = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state) *
                                                                                     sizeof(VOID *));
    for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
        LWPMU_DEVICE_acc_per_thread(&devices[cur_device])[i]      = CONTROL_Allocate_Memory((ECB_num_events(ecb) + 1)*sizeof(U64));
        LWPMU_DEVICE_prev_val_per_thread(&devices[cur_device])[i] = CONTROL_Allocate_Memory((ECB_num_events(ecb) + 1)*sizeof(U64));
        // initialize all values to 0
        for (j = 0; j < ECB_num_events(ecb) + 1; j++) {
            LWPMU_DEVICE_acc_per_thread(&devices[cur_device])[i][j]      = 0LL;
            LWPMU_DEVICE_prev_val_per_thread(&devices[cur_device])[i][j] = 0LL;
        }
    }
    
    LWPMU_DEVICE_em_groups_count(&devices[cur_device])++;

    // on to the next device.
    cur_device++;
        
    return OS_SUCCESS;
}
#endif

/*
 * lwpmudrv_Set_Sample_Descriptors
 *
 *     Parameters
 *         IN:  in_buf       - pointer to the input buffer
 *         IN:  in_buf_len   - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Set the number of descriptor groups in the global state node.
 */
static OS_STATUS
lwpmudrv_Set_Sample_Descriptors (
    IOCTL_ARGS    arg
)
{
    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (arg->w_len != sizeof(U32)) {
        SEP_PRINT_WARNING("Unknown size of Sample Descriptors\n");
        return OS_SUCCESS;
    }

    desc_count = 0;
    if (copy_from_user(&GLOBAL_STATE_num_descriptors(driver_state),
                       arg->w_buf, 
                       sizeof(U32))) {
        return OS_FAULT;
    }

    desc_data  = CONTROL_Allocate_Memory(GLOBAL_STATE_num_descriptors(driver_state) *
                                                sizeof(VOID *));
    return OS_SUCCESS;
}

/*
 * lwpmudrv_Configure_Descriptors
 *
 *     Parameters
 *         IN: in_buf        - pointer to the input buffer
 *         IN: in_buf_len    - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Make a copy of the descriptors that need to be read in order
 *         to configure a sample record.
 */
static OS_STATUS
lwpmudrv_Configure_Descriptors (
    IOCTL_ARGS    arg
)
{
    int uncopied;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (desc_count >= GLOBAL_STATE_num_descriptors(driver_state)) {
        SEP_PRINT_WARNING("Number of descriptor groups exceeded the initial configuration.");
        return OS_SUCCESS;
    }

    //
    // First things first: Make a copy of the data for global use.
    //
    if (arg->w_len) {
        desc_data[desc_count] = CONTROL_Allocate_Memory(arg->w_len);
        uncopied = copy_from_user(desc_data[desc_count], arg->w_buf, arg->w_len);
        if (uncopied < 0) {
        // no-op ... eliminates "variable not used" compiler warning
        }
        SEP_PRINT_DEBUG("Added descriptor # %d\n", desc_count);
    }
    desc_count++;

    return OS_SUCCESS;
}

#if defined(DRV_IA32) || defined(DRV_EM64T)
/*
 * lwpmudrv_LBR_Info
 *
 *     Parameters
 *         IN: in_buf        - pointer to the input buffer
 *         IN: in_buf_len    - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Make a copy of the LBR information that is passed in.
 */
static OS_STATUS
lwpmudrv_LBR_Info (
    IOCTL_ARGS    arg
)
{
    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (DRV_CONFIG_collect_lbrs(pcfg) == FALSE) {
        SEP_PRINT_WARNING("lwpmudrv_LBR_Info: LBR capture has not been configured\n");
        return OS_SUCCESS;
    }
    //
    // First things first: Make a copy of the data for global use.
    //
    lbr = CONTROL_Allocate_Memory((int)arg->w_len);
    if (copy_from_user(lbr, arg->w_buf, arg->w_len)) {
        return OS_FAULT;
    }

    return OS_SUCCESS;
}

#ifdef EMON
#define CR4_PCE  0x00000100    //Performance-monitoring counter enable RDPMC  
/*
 * lwpmudrv_Set_CR4_PCE_Bit
 *
 *     Parameters
 *         UNUSED
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Set CR4's PCE bit on the logical processor.
 */
static VOID
lwpmudrv_Set_CR4_PCE_Bit (
    PVOID  param
)
{
#if defined(DRV_IA32) 
  __asm__("movl %%cr4,%%eax\n\t"
          "orl %0,%%eax\n\t"
          "movl %%eax,%%cr4\n"
          : : "irg" (CR4_PCE)
          :"ax");
#endif
#if defined(DRV_EM64T)
  __asm__("movq %%cr4,%%rax\n\t"
          "orq %0,%%rax\n\t"
          "movq %%rax,%%cr4\n"
          : : "irg" (CR4_PCE)
          :"ax");
#endif

    return;
}

/*
 * lwpmudrv_Clear_CR4_PCE_Bit
 *
 *     Parameters
 *         UNUSED
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Clear CR4's PCE bit on the logical processor.
 */
static VOID
lwpmudrv_Clear_CR4_PCE_Bit (
    PVOID  param
)
{
#if defined(DRV_IA32)
  __asm__("movl %%cr4,%%eax\n\t"
          "andl %0,%%eax\n\t"
          "movl %%eax,%%cr4\n"
          : : "irg" (~CR4_PCE)
          :"ax");
#endif
#if defined(DRV_EM64T)
  __asm__("movq %%cr4,%%rax\n\t"
          "andq %0,%%rax\n\t"
          "movq %%rax,%%cr4\n"
          : : "irg" (~CR4_PCE)
          :"ax");
#endif

    return;
}
#endif // EMON

#else /* defined(DRV_IA32) || defined(DRV_EM64T) */

/*
 * @fn OS_STATUS lwpmudrv_RO_Info(arg)
 *
 * @param        IN arg
 *
 * @returns      OS_STATUS
 *
 * @brief        Make a copy of the RO information that is passed in.
 *
 */
static OS_STATUS
lwpmudrv_RO_Info (
    IOCTL_ARGS    arg
)
{
    if (DRV_CONFIG_collect_ro(pcfg) == FALSE) {
        SEP_PRINT_WARNING("lwpmudrv_RO_Info: RO capture has not been configured\n");
        return OS_SUCCESS;
    }
    //
    // First things first: Make a copy of the data for global use.
    //
    ro = CONTROL_Allocate_Memory((int)arg->w_len);
    if (copy_from_user(ro, arg->w_buf, arg->w_len)) {
        return -EFAULT;
    }
    return OS_SUCCESS;
}

#endif /* defined(DRV_IA32) || defined(DRV_EM64T) */

/*
 * lwpmudrv_Start
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMU_IOCTL_START call.
 *         Set up the OS hooks for process/thread/load notifications.
 *         Write the initial set of MSRs.
 */
static OS_STATUS
lwpmudrv_Start (
    VOID
)
{
    OS_STATUS  status = OS_SUCCESS;
    U32        previous_state;
#if defined(DRV_IA32) || defined(DRV_EM64T)
    DRV_CONFIG pcfg_unc = NULL;
    DISPATCH   dispatch_unc = NULL;
    U32        i;
#endif

    /*
     * To Do: Check for state == STATE_IDLE and only then enable sampling
     */
    previous_state = cmpxchg(&GLOBAL_STATE_current_phase(driver_state),
                             DRV_STATE_IDLE,
                             DRV_STATE_RUNNING);
    if (previous_state != DRV_STATE_IDLE) {
        SEP_PRINT_ERROR("lwpmudrv_Start: Unable to start sampling - State is %d\n",
                        GLOBAL_STATE_current_phase(driver_state));
        return VT_SAM_ERROR;
    }
    if (DRV_CONFIG_use_pcl(pcfg) == TRUE) {
        if (DRV_CONFIG_start_paused(pcfg)) {
            GLOBAL_STATE_current_phase(driver_state) = DRV_STATE_PAUSED;
        }
        return status;
    }

    atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
    init_waitqueue_head(&read_tsc_now);

#ifdef EMON
#if defined(DRV_IA32) || defined(DRV_EM64T)
    CONTROL_Invoke_Parallel(lwpmudrv_Set_CR4_PCE_Bit, (PVOID)(size_t)0);
#endif // (DRV_IA32 || DRV_EM64T)
#endif // EMON
    CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info, (PVOID)(size_t)0);

#ifdef EMON
    // initialize the cpu0_TSC var
    cpu0_TSC = CPU_STATE_tsc(&pcb[0]);
#endif

    if (DRV_CONFIG_start_paused(pcfg)) {
        GLOBAL_STATE_current_phase(driver_state) = DRV_STATE_PAUSED;
    }
    else {
        CONTROL_Invoke_Parallel(dispatch->restart, (PVOID)(size_t)0);
    }

#if defined(BUILD_CHIPSET)
    if (DRV_CONFIG_enable_chipset(pcfg)) {
        cs_dispatch->start_chipset();
    }
#endif

#if defined(DRV_IA32) || defined(DRV_EM64T)
    for (i = 0; i < num_devices; i++) {
        pcfg_unc = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
        dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);

        if (pcfg_unc                                && 
            DRV_CONFIG_event_based_counts(pcfg_unc) && 
            dispatch_unc                            && 
            dispatch_unc->restart) {
            SEP_PRINT_DEBUG("LWP: calling UNC Start\n");
            CONTROL_Invoke_Parallel(dispatch_unc->restart, (VOID *)&i);
        }
    }
#endif

    EVENTMUX_Start(global_ec);

#if defined(BUILD_GFX)
    SEP_PRINT_DEBUG("lwpmudrv_Start: enable_gfx=%d\n", (int)DRV_CONFIG_enable_gfx(pcfg));
    if (DRV_CONFIG_enable_gfx(pcfg)) {
        SEP_PRINT_DEBUG("lwpmudrv_Start: got gfx_offset=%d\n", EVENT_DESC_gfx_offset(global_ec));
        GFX_Start();
    }
#endif

    return status;
}

/*
 * @fn lwpmudrv_Prepare_Stop();
 *
 * @param        NONE
 * @return       OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMUDRV_IOCTL_STOP call.
 *         Cleans up the interrupt handler.
 */
static OS_STATUS
lwpmudrv_Prepare_Stop (
    VOID
)
{
    U32 i;
    int done = FALSE;
    U32 current_state = GLOBAL_STATE_current_phase(driver_state);

#if defined(DRV_IA32) || defined(DRV_EM64T)
    DRV_CONFIG pcfg_unc = NULL;
    DISPATCH   dispatch_unc = NULL;
#endif
    SEP_PRINT_DEBUG("lwpmudrv_Prepare_Stop: About to stop sampling\n");
    GLOBAL_STATE_current_phase(driver_state) = DRV_STATE_PREPARE_STOP;

    if (current_state == DRV_STATE_UNINITIALIZED) {
        return OS_SUCCESS;
    }

    if (pcfg == NULL) {
        return OS_SUCCESS;
    }

    if (DRV_CONFIG_use_pcl(pcfg) == TRUE) {
        return OS_SUCCESS;
    }

    if (current_state != DRV_STATE_IDLE          &&
        current_state != DRV_STATE_RESERVED) {
        for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
            CPU_STATE_accept_interrupt(&pcb[i]) = 0;
        }
        while (!done) {
            done = TRUE;
            for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                if (atomic_read(&CPU_STATE_in_interrupt(&pcb[i]))) {
                    done = FALSE;
                }
            }
        }
        CONTROL_Invoke_Parallel(dispatch->freeze, (PVOID)(size_t)0);
        SEP_PRINT_DEBUG("lwpmudrv_Prepare_Stop: Outside of all interrupts\n");

#if defined(BUILD_CHIPSET)
        if (DRV_CONFIG_enable_chipset(pcfg)) {
            cs_dispatch->stop_chipset();
        }
#endif

#if defined(BUILD_GFX)
        SEP_PRINT_DEBUG("lwpmudrv_Stop: enable_gfx=%d\n", (int)DRV_CONFIG_enable_gfx(pcfg));
        if (DRV_CONFIG_enable_gfx(pcfg)) {
            GFX_Stop();
        }
#endif
    }

    if (pcfg == NULL) {
        return OS_SUCCESS;
    }

#if defined(DRV_IA32) || defined(DRV_EM64T)
    for (i = 0; i < num_devices; i++) {
        pcfg_unc = (DRV_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
        dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);

        if (pcfg_unc                                &&
            DRV_CONFIG_event_based_counts(pcfg_unc) &&
            dispatch_unc                            &&
            dispatch_unc->freeze) {
            SEP_PRINT_DEBUG("LWP: calling UNC Stop\n");
            preempt_disable();
            invoking_processor_id = CONTROL_THIS_CPU();
            preempt_enable();
            CONTROL_Invoke_Parallel(dispatch_unc->freeze, (VOID *)&i);
        }
    }
#endif

#ifdef EMON
#if defined(DRV_IA32) || defined(DRV_EM64T)
    CONTROL_Invoke_Parallel(lwpmudrv_Clear_CR4_PCE_Bit, (VOID *)(size_t)0);
#endif // DRV_IA32 || DRV_EM64T
#endif // EMON

#if defined(BUILD_CHIPSET)
    if (DRV_CONFIG_enable_chipset(pcfg) &&
        cs_dispatch && cs_dispatch->fini_chipset) {
        cs_dispatch->fini_chipset();
    }
#endif

    return OS_SUCCESS;
}

/*
 * @fn lwpmudrv_Finish_Stop();
 *
 * @param        NONE
 * @return       OS_STATUS
 *
 *     Description
 *         Local function that handles the LWPMUDRV_IOCTL_STOP call.
 *         Cleans up the interrupt handler.
 */
static OS_STATUS
lwpmudrv_Finish_Stop (
    VOID
)
{
    U32      current_state = GLOBAL_STATE_current_phase(driver_state);
    S32      prev_value;

    prev_value = cmpxchg(&in_finish_code, 0, 1);
    if (prev_value != 0) {
       return OS_SUCCESS;
    }
    if (DRV_CONFIG_counting_mode(pcfg) == FALSE) {
        LINUXOS_Uninstall_Hooks();
        /*
         *  Make sure that the module buffers are not deallocated and that the module flush
         *  thread has not been terminated.
         */
        if (current_state != DRV_STATE_IDLE && current_state != DRV_STATE_RESERVED) {
            LINUXOS_Enum_Process_Modules(TRUE);
        }
        OUTPUT_Flush();
        /*
         * Clean up the interrupt handler via the IDT
         */
        CPUMON_Remove_Cpuhooks();
#if defined(DRV_IA32) || defined(DRV_EM64T)
        PEBS_Destroy(pcfg);
#endif
        EVENTMUX_Destroy(global_ec);
    }
    GLOBAL_STATE_current_phase(driver_state) = DRV_STATE_STOPPED;
    in_finish_code                           = 0;

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Get_Normalized_TSC
 *
 *     Parameters
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 */
static OS_STATUS
lwpmudrv_Get_Normalized_TSC (
    IOCTL_ARGS arg
)
{
    U64 tsc = 0;

    if (pcb != NULL) {
        preempt_disable();
        UTILITY_Read_TSC(&tsc);
        tsc -= TSC_SKEW(CONTROL_THIS_CPU());
        preempt_enable();
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
    if (pcfg && DRV_CONFIG_use_pcl(pcfg) == TRUE) {
        tsc = cpu_clock(CONTROL_THIS_CPU());
    }
#endif
    if (copy_to_user(arg->r_buf, (VOID *)&tsc, sizeof(U64))) {
        SEP_PRINT_DEBUG("lwpmudrv_Get_Normalized_TSC: copy_to_user() failed\n");
    }

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Get_Normalized_TSC_Standalone
 *
 *     Parameters
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Get the tsc value only (meaning you need to allocate
 *         the pcb beforehand).
 */
static OS_STATUS
lwpmudrv_Get_Normalized_TSC_Standalone (
    IOCTL_ARGS arg
)
{
    OS_STATUS status = OS_FAULT;

    pcb_size = GLOBAL_STATE_num_cpus(driver_state)*sizeof(CPU_STATE_NODE);
    pcb      = CONTROL_Allocate_Large_Memory(pcb_size);
    status   = lwpmudrv_Get_Normalized_TSC (arg);
    pcb      = CONTROL_Free_Large_Memory(pcb, pcb_size);
    pcb_size = 0;

    return status;
}

/*
 * lwpmudrv_Get_Num_Cores
 *
 *     Parameters
 *         IN   out_buf       - pointer to the output buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Quickly return the (total) number of cpus in the system.
 */
static OS_STATUS
lwpmudrv_Get_Num_Cores (
    IOCTL_ARGS   arg
)
{
    OS_STATUS result;
    int num = GLOBAL_STATE_num_cpus(driver_state);

    SEP_PRINT_DEBUG("lwpmudrv_Get_Num_Cores: Num_Cores is %d, out_buf is 0x%p\n", num, arg->r_buf);
    result = put_user(num, (int*)arg->r_buf);

    return result;
}

/*
 * lwpmudrv_Set_CPU_Mask
 *
 *     Parameters
 *         IN   in_buf       - pointer to the input buffer
 *         IN   in_buf_len   - length of the input buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Check and make sure that enough space is provided to 
 *         generate the data.
 *         Return the total number of cores in the system.
 */
static OS_STATUS
lwpmudrv_Set_CPU_Mask (
    PVOID         in_buf,
    size_t        in_buf_len
)
{
    U32     cpu_count     = 0;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (in_buf_len == 0 || in_buf == NULL) {
        return VT_SAM_ERROR;
    }

    cpu_mask_bits = CONTROL_Allocate_Memory((int)in_buf_len);
    if (!cpu_mask_bits) {
        return OS_NO_MEM;
    }

    if (copy_from_user(cpu_mask_bits, (S8*)in_buf, (int)in_buf_len)) {
        return OS_FAULT;
    }

    for (cpu_count = 0; cpu_count < (U32)GLOBAL_STATE_num_cpus(driver_state); cpu_count++) {
        CPU_STATE_accept_interrupt(&pcb[cpu_count]) = cpu_mask_bits[cpu_count] ? 1 : 0;
    }


    return OS_SUCCESS;
}


#if defined(DRV_IA32) || defined(DRV_EM64T)
/*
 * @fn lwpmudrv_Get_KERNEL_CS
 *
 * @param    IN   out_buf       - pointer to the output buffer
 * @return   OS_STATUS
 *
 *     Description
 *         Return the value of the Kernel symbol KERNEL_CS.
 */
static OS_STATUS
lwpmudrv_Get_KERNEL_CS (
    IOCTL_ARGS   arg
)
{
    OS_STATUS result;
    int num = __KERNEL_CS;

    SEP_PRINT_DEBUG("lwpmudrv_Get_KERNEL_CS is %d, out_buf is 0x%p\n", num, arg->r_buf);
    result = put_user(num, (int*)arg->r_buf);

    return result;
}
#endif

/*
 * @fn lwpmudrv_Set_UID
 *
 * @param     IN   arg      - pointer to the output buffer
 * @return   OS_STATUS
 *
 *     Description
 *         Receive the value of the UID of the collector process.
 */
static OS_STATUS
lwpmudrv_Set_UID (
    IOCTL_ARGS   arg
)
{
    OS_STATUS result;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    result = get_user(uid, (int*)arg->w_buf);
    SEP_PRINT_DEBUG("lwpmudrv_Set_UID is %d\n", uid);

    return result;
}

/*
 * lwpmudrv_Get_TSC_Skew_Info
 *
 *     Parameters
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 */
static OS_STATUS
lwpmudrv_Get_TSC_Skew_Info (
    IOCTL_ARGS arg
)
{
    S64 *skew_array;
    S32  i;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_STOPPED) {
        return VT_SAM_ERROR;
    }
    skew_array = (S64 *)arg->r_buf;
    // Check if enough space is provided for collecting the data
    SEP_PRINT_DEBUG("lwpmudrv_Get_TSC_Skew_Info dispatched with r_len=%lld\n", arg->r_len);
    if (arg->r_len < GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64)) {
        SEP_PRINT_ERROR("lwpmudrv_Get_TSC_Skew_Info: Buffer too small in Get_TSC_Skew_Info: %lld\n",arg->r_len);
        return OS_FAULT;
    }
    skew_array = CONTROL_Allocate_Memory(arg->r_len);
    if (skew_array == NULL) {
        SEP_PRINT_ERROR("lwpmudrv_Get_TSC_Skew_Info: Unable to allocate memory\n");
        return OS_NO_MEM;
    }

    for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
        skew_array[i] = TSC_SKEW(i);
    }

    if (copy_to_user(arg->r_buf, skew_array, arg->r_len)) {
        return OS_FAULT;
    }
    skew_array = CONTROL_Free_Memory(skew_array);

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Collect_Sys_Config
 *
 *     Parameters
 *         IN   out_buf       - pointer to the U32 result
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Local function that handles the COLLECT_SYS_CONFIG call.
 *         Builds and collects the SYS_INFO data needed.
 *         Writes the result into the argument.
 */
static OS_STATUS
lwpmudrv_Collect_Sys_Config (
    IOCTL_ARGS   arg
)
{
    OS_STATUS  result;
    int num = SYS_INFO_Build();

    SEP_PRINT_DEBUG("lwpmudrv_Collect_Sys_Config: size of sys info is %d\n", num);
    result = put_user(num, (int*)arg->r_buf);

    return result;
}

/*
 * lwpmudrv_Sys_Config
 *
 *     Parameters
 *         IN   out_buf       - pointer to the output buffer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Transfers the VTSA_SYS_INFO data back to the abstraction layer.
 *         The out_buf should have enough space to handle the transfer.
 */
static OS_STATUS
lwpmudrv_Sys_Config (
    IOCTL_ARGS   arg
)
{
    SYS_INFO_Transfer(arg->r_buf, arg->r_len);

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Samp_Read_Num_Of_Core_Counters
 *
 * Abstract
 *     Read memory mapped i/o physical location
 *
 * Parameters
 *     physical_address - physical address in mmio
 *     value            - value at this address
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Samp_Read_Num_Of_Core_Counters (
    IOCTL_ARGS   arg
)
{
#if defined(DRV_IA32) || defined(DRV_EM64T)
    U64           rax, rbx, rcx, rdx,num_basic_functions;
    U32           val = 0;
    OS_STATUS     result;

    UTILITY_Read_Cpuid(0x0,&num_basic_functions,&rbx, &rcx, &rdx);

    if (num_basic_functions >= 0xA) {
         UTILITY_Read_Cpuid(0xA,&rax,&rbx, &rcx, &rdx);
         val    = ((U32)(rax >> 8)) & 0xFF;
    }
    result = put_user(val, (int*)arg->r_buf);
    SEP_PRINT_DEBUG("num of counter is %d\n",val);
    return result;
#else
    return OS_SUCCESS;
#endif
}


#if defined(BUILD_CHIPSET)

/*
 * lwpmudrv_Is_Physical_Address_Free
 *
 * Abstract
 *     Check if physical address is available
 *
 * Parameters
 *     physical_address - physical address
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Is_Physical_Address_Free (
    U32 physical_address
)
{
    U32 value;
    U32 new_value;
    U32 test_value;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    // First attempt read
    //
    PCI_Read_From_Memory_Address(physical_address, &value);

    // Value must be 0xFFFFFFFFF or there is NO chance
    // that this memory location is available.
    //
    if (value != 0xFFFFFFFF)
    {
        return FALSE;
    }

    //
    // Try to write a bit to a zero (this probably
    // isn't too safe, but this is just for testing)
    //
    new_value = 0xFFFFFFFE;

    PCI_Write_To_Memory_Address(physical_address, new_value);

    PCI_Read_From_Memory_Address(physical_address, &test_value);

    // Write back original
    PCI_Write_To_Memory_Address(physical_address, value);

    if (new_value == test_value)
    {
        // The write appeared to change the
        // memory, it must be mapped already
        //
        return FALSE;
    }

    if (test_value == 0xFFFFFFFF)
    {
        // The write did not change the bit, so
        // apparently, this memory must not be mapped
        // to anything.
        //
        return TRUE;
    }

    return FALSE;
}


/*
 * lwpmudrv_Samp_Find_Physical_Address
 *
 * Abstract
 *     Find a free physical address.
 *
 * Parameters
 *     arg - IOCTL argument
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Samp_Find_Physical_Address (
    IOCTL_ARGS    arg
)
{
    CHIPSET_PCI_SEARCH_ADDR_NODE user_addr;
    CHIPSET_PCI_SEARCH_ADDR      search_addr = (CHIPSET_PCI_SEARCH_ADDR)arg->w_buf;
    U32                          addr;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (!search_addr) {
        return OS_FAULT;
    }

    if (!access_ok(VERIFY_WRITE, search_addr, sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
        return OS_FAULT;
    }

    if (copy_from_user(&user_addr, search_addr, sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
        SEP_PRINT_DEBUG("lwpmudrv_Samp_Find_Physical_Address: copy_from_user() failed\n");
        return OS_FAULT;
    }

    if (CHIPSET_PCI_SEARCH_ADDR_start(&user_addr) > CHIPSET_PCI_SEARCH_ADDR_stop(&user_addr)) {
        return OS_INVALID;
    }

    CHIPSET_PCI_SEARCH_ADDR_address(&user_addr) = 0;

    for (addr = CHIPSET_PCI_SEARCH_ADDR_start(&user_addr);
        addr <= CHIPSET_PCI_SEARCH_ADDR_stop(&user_addr);
        addr += CHIPSET_PCI_SEARCH_ADDR_increment(&user_addr)) {
        SEP_PRINT_DEBUG("lwpmudrv_Samp_Find_Physical_Address: addr=%x:",addr);
        if (lwpmudrv_Is_Physical_Address_Free(addr)) {
            CHIPSET_PCI_SEARCH_ADDR_address(&user_addr) = addr;
            break;
        }
    }

    if (copy_to_user(arg->r_buf, (VOID *) &user_addr, sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
        SEP_PRINT_DEBUG("lwpmudrv_Samp_Find_Physical_Address: copy_to_user() failed\n");
    }

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Samp_Read_PCI_Config
 *
 * Abstract
 *     Read PCI configuration space
 *
 * Parameters
 *     arg - IOCTL argument
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Samp_Read_PCI_Config (
    IOCTL_ARGS    arg
)
{
    U32                     pci_address;
    CHIPSET_PCI_CONFIG      rd_pci;

    rd_pci = CONTROL_Allocate_Memory(arg->r_len);
    if (rd_pci == NULL) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Read_PCI_Config: unable to allocate local memory\n");
        return OS_FAULT;
    }

    if (copy_from_user(rd_pci, (CHIPSET_PCI_CONFIG)arg->w_buf, sizeof(CHIPSET_PCI_CONFIG_NODE))) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Read_PCI_Config: unable to read into local memory\n");
        return OS_FAULT;
    }

    SEP_PRINT_DEBUG("lwpmudrv_Samp_Read_PCI_Config: reading PCI address:0x%x:0x%x:0x%x, offset 0x%x\n",
                CHIPSET_PCI_CONFIG_bus(rd_pci),
                CHIPSET_PCI_CONFIG_device(rd_pci),
                CHIPSET_PCI_CONFIG_function(rd_pci),
                CHIPSET_PCI_CONFIG_offset(rd_pci));

    pci_address = FORM_PCI_ADDR(CHIPSET_PCI_CONFIG_bus(rd_pci),
                            CHIPSET_PCI_CONFIG_device(rd_pci),
                            CHIPSET_PCI_CONFIG_function(rd_pci),
                            CHIPSET_PCI_CONFIG_offset(rd_pci));
    CHIPSET_PCI_CONFIG_value(rd_pci) = PCI_Read_Ulong(pci_address);

    if (copy_to_user(arg->r_buf, (VOID *) rd_pci, sizeof(CHIPSET_PCI_CONFIG_NODE))) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Read_PCI_Config: unable to copy to user\n");
        return OS_FAULT;
    }

    SEP_PRINT_DEBUG("lwpmudrv_Samp_Read_PCI_Config: value at this PCI address:0x%x\n",
                    CHIPSET_PCI_CONFIG_value(rd_pci));

    rd_pci = CONTROL_Free_Memory(rd_pci);

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Samp_Write_PCI_Config
 *
 * Abstract
 *     Writes to PCI configuration space
 *
 * Parameters
 *     arg - IOCTL argument
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Samp_Write_PCI_Config (
    IOCTL_ARGS    arg
)
{
    U32 pci_address;
    CHIPSET_PCI_CONFIG wr_pci = (CHIPSET_PCI_CONFIG)arg->w_buf;

    // the following allows "sep -el -pc" to work, since the command must access the
    // the driver ioctls before driver is used for a collection
    if (! (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_UNINITIALIZED ||
           GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_IDLE))
    {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Write_PCI_Config: driver is non-idle or busy\n");
        return VT_SAM_ERROR;
    }

    if (wr_pci == NULL)
    {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Write_PCI_Config: null write buffer\n");
        return OS_INVALID; // OS_FAULT?
    }

    SEP_PRINT_DEBUG("lwpmudrv_Samp_Write_PCI_Config: writing 0x%x to PCI address:0x%x:0x%x:0x%x, offset 0x%x\n",
                    CHIPSET_PCI_CONFIG_bus(wr_pci),
                    CHIPSET_PCI_CONFIG_device(wr_pci),
                    CHIPSET_PCI_CONFIG_function(wr_pci),
                    CHIPSET_PCI_CONFIG_offset(wr_pci));

    pci_address = FORM_PCI_ADDR(CHIPSET_PCI_CONFIG_bus(wr_pci),
                            CHIPSET_PCI_CONFIG_device(wr_pci),
                            CHIPSET_PCI_CONFIG_function(wr_pci),
                            CHIPSET_PCI_CONFIG_offset(wr_pci));
    PCI_Write_Ulong(pci_address, CHIPSET_PCI_CONFIG_value(wr_pci));

    return OS_SUCCESS;
}

/*
 * lwpmudrv_Samp_Chipset_Init
 *
 * Abstract
 *     Initialization chipset configuration.
 *
 * Parameters
 *     arg   - IOCTL arguments
 *
 * Returns
 *     OS_STATUS
 */
static OS_STATUS
lwpmudrv_Samp_Chipset_Init (
    IOCTL_ARGS    arg
)
{
    PVOID         in_buf = arg->w_buf;
    U32           in_buf_len = arg->w_len;

    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Chipset_Init: driver is currently busy!\n");
        return VT_SAM_ERROR;
    }

    if (in_buf == NULL || in_buf_len == 0) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Chipset_Init: Chipset information passed in is null\n");
        return OS_NO_MEM;
    }

    // First things first: Make a copy of the data for global use.
    pma = CONTROL_Allocate_Memory(in_buf_len);

    if (pma == NULL) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Chipset_Init: unable to allocate memory\n");
        return OS_NO_MEM;
    }

    if (copy_from_user(pma, in_buf, in_buf_len)) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Chipset_Init: unable to copy from user\n");
        return OS_FAULT;
    }

#if defined(MY_DEBUG)

    SEP_PRINT("lwpmudrv_Samp_Chipset_Init: Chipset Configuration follows...\n");
    SEP_PRINT("pma->length=%d\n", CHIPSET_CONFIG_length(pma));
    SEP_PRINT("pma->version=%d\n", CHIPSET_CONFIG_major_version(pma));
    SEP_PRINT("pma->processor=%d\n", CHIPSET_CONFIG_processor(pma));
    SEP_PRINT("pma->mch_chipset=%d\n", CHIPSET_CONFIG_mch_chipset(pma));
    SEP_PRINT("pma->ich_chipset=%d\n", CHIPSET_CONFIG_ich_chipset(pma));
    SEP_PRINT("pma->gmch_chipset=%d\n", CHIPSET_CONFIG_gmch_chipset(pma));
    SEP_PRINT("pma->mother_board_time=%d\n", CHIPSET_CONFIG_motherboard_time(pma));
    SEP_PRINT("pma->host_proc_run=%d\n", CHIPSET_CONFIG_host_proc_run(pma));
    SEP_PRINT("pma->noa_chipset=%d\n", CHIPSET_CONFIG_noa_chipset(pma));
    SEP_PRINT("pma->bnb_chipset=%d\n", CHIPSET_CONFIG_bnb_chipset(pma));

    if (CHIPSET_CONFIG_mch_chipset(pma)) {
        SEP_PRINT("pma->mch->phys_add=0x%llx\n", CHIPSET_SEGMENT_physical_address(&CHIPSET_CONFIG_mch(pma)));
        SEP_PRINT("pma->mch->size=%d\n", CHIPSET_SEGMENT_size(&CHIPSET_CONFIG_mch(pma)));
        SEP_PRINT("pma->mch->num_counters=%d\n", CHIPSET_SEGMENT_num_counters(&CHIPSET_CONFIG_mch(pma)));
        SEP_PRINT("pma->mch->total_events=%d\n", CHIPSET_SEGMENT_total_events(&CHIPSET_CONFIG_mch(pma)));
    }

    if (CHIPSET_CONFIG_ich_chipset(pma)) {
        SEP_PRINT("pma->ich->phys_add=0x%llx\n", CHIPSET_SEGMENT_physical_address(&CHIPSET_CONFIG_ich(pma)));
        SEP_PRINT("pma->ich->size=%d\n", CHIPSET_SEGMENT_size(&CHIPSET_CONFIG_ich(pma)));
        SEP_PRINT("pma->ich->num_counters=%d\n", CHIPSET_SEGMENT_num_counters(&CHIPSET_CONFIG_ich(pma)));
        SEP_PRINT("pma->ich->total_events=%d\n", CHIPSET_SEGMENT_total_events(&CHIPSET_CONFIG_ich(pma)));
    }

    if (CHIPSET_CONFIG_gmch_chipset(pma)) {
        SEP_PRINT("pma->gmch->phys_add=0x%llx\n", CHIPSET_SEGMENT_physical_address(&CHIPSET_CONFIG_gmch(pma)));
        SEP_PRINT("pma->gmch->size=%d\n", CHIPSET_SEGMENT_size(&CHIPSET_CONFIG_gmch(pma)));
        SEP_PRINT("pma->gmch->num_counters=%d\n", CHIPSET_SEGMENT_num_counters(&CHIPSET_CONFIG_gmch(pma)));
        SEP_PRINT("pma->gmch->total_events=%d\n", CHIPSET_SEGMENT_total_events(&CHIPSET_CONFIG_gmch(pma)));
        SEP_PRINT("pma->gmch->read_register=0x%x\n", CHIPSET_SEGMENT_read_register(&CHIPSET_CONFIG_gmch(pma)));
        SEP_PRINT("pma->gmch->write_register=0x%x\n", CHIPSET_SEGMENT_write_register(&CHIPSET_CONFIG_gmch(pma)));
    }

#endif

    // Set up the global cs_dispatch table
    cs_dispatch = UTILITY_Configure_Chipset();
    if (cs_dispatch == NULL) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Chipset_Init: unknown chipset family\n");
        return OS_INVALID;
    }

    // Initialize chipset configuration
    if (cs_dispatch->init_chipset()) {
        SEP_PRINT_ERROR("lwpmudrv_Samp_Chipset_Init: failed to initialize the chipset\n");
        return OS_INVALID;
    }

    return OS_SUCCESS;
}

#endif  // BUILD_CHIPSET

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Get_Platform_Info
 *
 * @brief       Reads the MSR_PLATFORM_INFO register if present
 *
 * @param       void
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS
lwpmudrv_Get_Platform_Info (
    IOCTL_ARGS args
)
{
    U64        value = 0;
    OS_STATUS  result;

    if (dispatch && dispatch->platform_info) {
        value = dispatch->platform_info();
    }
    result = put_user(value, (U64*)args->r_buf);

    return result;
}
/* ------------------------------------------------------------------------- */
/*!
 * @fn          void lwpmudrv_Setup_Cpu_Topology (value)
 *
 * @brief       Sets up the per CPU state structures
 *
 * @param       IOCTL_ARGS args
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 *              This function was added to support abstract dll creation. Use
 *              this function to set the value of abnormal_terminate ouside of 
 *              sep_common.               
 */
static OS_STATUS
lwpmudrv_Setup_Cpu_Topology (
    IOCTL_ARGS args
)
{
    S32               cpu_num;
    S32               iter;
    DRV_TOPOLOGY_INFO drv_topology, dt;
    
    if (GLOBAL_STATE_current_phase(driver_state) != DRV_STATE_IDLE) {
        return VT_SAM_ERROR;
    }
    if (args->w_buf == NULL || pcb == NULL ) {
        SEP_PRINT_ERROR("topology information has been misconfigured\n");
        return OS_NO_MEM;
    }

    drv_topology = CONTROL_Allocate_Memory(args->w_len);
    if (drv_topology == NULL) {
        return OS_NO_MEM;
    }

    if (copy_from_user(drv_topology, (DRV_TOPOLOGY_INFO)(args->w_buf), args->w_len)) {
        return EFAULT;
    }
    /*
     *   Topology Initializations
     */
    for (iter = 0; iter < GLOBAL_STATE_num_cpus(driver_state); iter++) {
        dt                                         = &drv_topology[iter];
        cpu_num                                    = DRV_TOPOLOGY_INFO_cpu_number(dt);
        CPU_STATE_socket_master(&pcb[cpu_num])     = DRV_TOPOLOGY_INFO_socket_master(dt);
        CPU_STATE_core_master(&pcb[cpu_num])       = DRV_TOPOLOGY_INFO_core_master(dt);
        CPU_STATE_thr_master(&pcb[cpu_num])        = DRV_TOPOLOGY_INFO_thr_master(dt);
        SEP_PRINT_DEBUG("cpu %d sm = %d cm = %d tm = %d\n",
                  cpu_num,
                  CPU_STATE_socket_master(&pcb[cpu_num]),
                  CPU_STATE_core_master(&pcb[cpu_num]),
                  CPU_STATE_thr_master(&pcb[cpu_num]));
    }
    drv_topology = CONTROL_Free_Memory(drv_topology);
    return OS_SUCCESS;
}

/*******************************************************************************
 *  External Driver functions - Open
 *      This function is common to all drivers
 *******************************************************************************/

static int
lwpmu_Open (
    struct inode *inode,
    struct file *filp
)
{
    SEP_PRINT_DEBUG("lwpmu_Open called on maj:%d, min:%d\n",
            imajor(inode), iminor(inode));
    filp->private_data = container_of(inode->i_cdev, LWPMU_DEV_NODE, cdev);

    return 0;
}

/*******************************************************************************
 *  External Driver functions
 *      These functions are registered into the file operations table that
 *      controls this device.
 *      Open, Close, Read, Write, Release
 *******************************************************************************/

static ssize_t
lwpmu_Read (
    struct file  *filp,
    char         *buf,
    size_t        count,
    loff_t       *f_pos
)
{
    unsigned long retval;

    /* Transfering data to user space */
    SEP_PRINT_DEBUG("lwpmu_Read dispatched with count=%d\n", (S32)count);
    if (copy_to_user(buf, &LWPMU_DEV_buffer(lwpmu_control), 1)) {
        retval = OS_FAULT;
        return retval;
    }
    /* Changing reading position as best suits */
    if (*f_pos == 0) {
        *f_pos+=1;
        return 1;
    }

    return 0;
}

static ssize_t
lwpmu_Write (
    struct file  *filp,
    const  char  *buf,
    size_t        count,
    loff_t       *f_pos
)
{
    unsigned long retval;

    SEP_PRINT_DEBUG("lwpmu_Write dispatched with count=%d\n", (S32)count);
    if (copy_from_user(&LWPMU_DEV_buffer(lwpmu_control), buf+count-1, 1)) {
        retval = OS_FAULT;
        return retval;
    }

    return 1;
}

/*
 * LWPMU_Device_Control
 *
 *     Parameters
 *         IN: inode - pointer to the device object
 *         IN: filp  - pointer to the file object
 *         IN: cmd   - ioctl value (defined in lwpmu_ioctl.h)
 *         IN: arg   - arg or arg pointer
 *
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Worker function that handles IOCTL requests from the user mode.
 */
extern IOCTL_OP_TYPE
lwpmu_Device_Control (
    IOCTL_USE_INODE
    struct   file   *filp,
    unsigned int     cmd,
    unsigned long    arg
)
{
    int              status = OS_SUCCESS;
    IOCTL_ARGS_NODE  local_args;
#if defined(DRV_USE_UNLOCKED_IOCTL)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
    unsigned long    flags;
#endif
#endif

#if !defined(DRV_USE_UNLOCKED_IOCTL)
    SEP_PRINT_DEBUG("lwpmu_DeviceControl(0x%x) called on inode maj:%d, min:%d\n",
            cmd, imajor(inode), iminor(inode));
#endif
    SEP_PRINT_DEBUG("type: %d, subcommand: %d\n",_IOC_TYPE(cmd),_IOC_NR(cmd));

    if (_IOC_TYPE(cmd) != LWPMU_IOC_MAGIC) {
        return OS_ILLEGAL_IOCTL;
    }

    if (cmd ==  LWPMUDRV_IOCTL_GET_DRIVER_STATE) {
        SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_DRIVER_STATE\n");
        if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
            return OS_FAULT;
        }
        status = lwpmudrv_Get_Driver_State(&local_args);
        return status;
    }

    MUTEX_LOCK(ioctl_lock, flags);
    switch (cmd) {

       /*
        * Common IOCTL commands
        */

        case LWPMUDRV_IOCTL_VERSION:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_VERSION\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Version(&local_args);
            break;

        case LWPMUDRV_IOCTL_RESERVE:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_RESERVE\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Reserve(&local_args);
            break;

        case LWPMUDRV_IOCTL_INIT:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_INIT\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Initialize(local_args.w_buf, local_args.w_len);
            break;

        case LWPMUDRV_IOCTL_INIT_PMU:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_INIT_PMU\n");
            status = lwpmudrv_Init_PMU();
            break;

        case LWPMUDRV_IOCTL_SET_CPU_MASK:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_SET_CPU_MASK\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Set_CPU_Mask(local_args.w_buf, local_args.w_len);
            break;

        case LWPMUDRV_IOCTL_START:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_START\n");
            status = lwpmudrv_Start();
            break;

        case LWPMUDRV_IOCTL_STOP:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_STOP\n");
            status = lwpmudrv_Prepare_Stop();
            break;

        case LWPMUDRV_IOCTL_PAUSE:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_PAUSE\n");
            status = lwpmudrv_Pause();
            break;

        case LWPMUDRV_IOCTL_RESUME:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_RESUME\n");
            status = lwpmudrv_Resume();
            break;

        case LWPMUDRV_IOCTL_EM_GROUPS:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_EM_GROUPS\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Set_EM_Config(&local_args);
            break;

        case LWPMUDRV_IOCTL_EM_CONFIG_NEXT:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_EM_CONFIG_NEXT\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Configure_Events(&local_args);
            break;

        case LWPMUDRV_IOCTL_NUM_DESCRIPTOR:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_NUM_DESCRIPTOR\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Set_Sample_Descriptors(&local_args);
            break;

        case LWPMUDRV_IOCTL_DESC_NEXT:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_DESC_NEXT\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Configure_Descriptors(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_NORMALIZED_TSC:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_NORMALIZED_TSC\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_Normalized_TSC(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_NORMALIZED_TSC_STANDALONE:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_NORMALIZED_TSC_STANDALONE\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_Normalized_TSC_Standalone(&local_args);
            break;

        case LWPMUDRV_IOCTL_NUM_CORES:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_NUM_CORES\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_Num_Cores(&local_args);
            break;

        case LWPMUDRV_IOCTL_KERNEL_CS:
#if defined(DRV_IA32) || defined(DRV_EM64T)
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_KERNEL_CS\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_KERNEL_CS(&local_args);
#endif
            break;

        case LWPMUDRV_IOCTL_SET_UID:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_SET_UID\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Set_UID(&local_args);
            break;

        case LWPMUDRV_IOCTL_TSC_SKEW_INFO:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_TSC_SKEW_INFO\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_TSC_Skew_Info(&local_args);
            break;

        case LWPMUDRV_IOCTL_COLLECT_SYS_CONFIG:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_COLLECT_SYS_CONFIG\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Collect_Sys_Config(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_SYS_CONFIG:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_SYS_CONFIG\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Sys_Config(&local_args);
            break;

        case LWPMUDRV_IOCTL_TERMINATE:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_TERMINATE\n");
            status = lwpmudrv_Terminate();
            break;

        case LWPMUDRV_IOCTL_SET_CPU_TOPOLOGY:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_SET_CPU_TOPOLOGY\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Setup_Cpu_Topology(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_NUM_CORE_CTRS:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_NUM_CORE_CTRS\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Samp_Read_Num_Of_Core_Counters(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_PLATFORM_INFO:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_PLATFORM_INFO\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_Platform_Info(&local_args);
            break;

        case LWPMUDRV_IOCTL_READ_MSRS:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_READ_MSRs\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Read_MSRs(&local_args);
            break;

        case LWPMUDRV_IOCTL_SWITCH_GROUP:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_SWITCH_GROUP\n");
            status = lwpmudrv_Switch_Group();
            break;

       /*
        * EMON-specific IOCTL commands
        */

#if defined(EMON)
        case LWPMUDRV_IOCTL_READ_MSR:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_READ_MSR\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Read_MSR_All_Cores(&local_args);
            break;

#if defined(EMON_INTERNAL)
        case LWPMUDRV_IOCTL_WRITE_MSR:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_WRITE_MSR\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Write_MSR_All_Cores(&local_args);
            break;
#endif  // EMON_INTERNAL

        case LWPMUDRV_IOCTL_READ_SWITCH_GROUP:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_READ_SWITCH_GROUP\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Read_Counters_And_Switch_Group(&local_args);
            break;

        case LWPMUDRV_IOCTL_READ_AND_RESET:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_READ_AND_RESET\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Read_And_Reset_Counters(&local_args);
            break;
#endif  // EMON

       /*
        * Platform-specific IOCTL commands (IA64 only)
        */

#if defined(DRV_IA64)
        case LWPMUDRV_IOCTL_RO_INFO:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_RO_INFO\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_RO_Info(&local_args);
            break;
#endif

       /*
        * Platform-specific IOCTL commands (IA32 and Intel64)
        */

#if defined(DRV_IA32) || defined(DRV_EM64T)
        case LWPMUDRV_IOCTL_INIT_UNC:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_INIT_UNC\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Initialize_UNC(local_args.w_buf, local_args.w_len);
            break;

        case LWPMUDRV_IOCTL_EM_GROUPS_UNC:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_EM_GROUPS_UNC\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Set_EM_Config_UNC(&local_args);
            break;

        case LWPMUDRV_IOCTL_EM_CONFIG_NEXT_UNC:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_EM_CONFIG_NEXT_UNC\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Configure_Events_UNC(&local_args);
            break;

        case LWPMUDRV_IOCTL_LBR_INFO:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_LBR_INFO\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_LBR_Info(&local_args);
            break;

        case LWPMUDRV_IOCTL_PWR_INFO:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_PWR_INFO\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_PWR_Info(&local_args);
            break;

        case LWPMUDRV_IOCTL_INIT_NUM_DEV:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_INIT_NUM_DEV\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Initialize_Num_Devices(&local_args);
            break;
#endif

       /*
        * Graphics IOCTL commands
        */

#if defined(BUILD_GFX)
        case LWPMUDRV_IOCTL_SET_GFX_EVENT:
            SEP_PRINT("LWPMUDRV_IOCTL_SET_GFX_EVENT\n");
            SEP_PRINT("lwpmudrv_Device_Control: enable_gfx=%d, gfx_offset=%d\n", (int)DRV_CONFIG_enable_gfx(pcfg), EVENT_DESC_gfx_offset(global_ec));
            status = GFX_Set_Event_Code((IOCTL_ARGS)arg);
            break;
#endif

       /*
        * Chipset IOCTL commands
        */

#if defined(BUILD_CHIPSET)
        case LWPMUDRV_IOCTL_PCI_READ:
            {
                CHIPSET_PCI_ARG_NODE pci_data;

                SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_PCI_READ\n");
                if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                    status = OS_FAULT;
                    goto cleanup;

                }
                if (copy_from_user(&pci_data, (CHIPSET_PCI_ARG)local_args.w_buf, sizeof(CHIPSET_PCI_ARG_NODE))) {
                    status = OS_FAULT;
                    goto cleanup;
                }

                status = PCI_Read_From_Memory_Address(CHIPSET_PCI_ARG_address(&pci_data),
                                               &CHIPSET_PCI_ARG_value(&pci_data));

                if (copy_to_user(((IOCTL_ARGS)arg)->r_buf, &pci_data, sizeof(CHIPSET_PCI_ARG_NODE))) {
                    status =  -EFAULT;
                    goto cleanup;
                }

                break;
            }

        case LWPMUDRV_IOCTL_PCI_WRITE:
            {
                CHIPSET_PCI_ARG_NODE pci_data;

                SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_PCI_WRITE\n");
                if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                    status = OS_FAULT;
                    goto cleanup;
                }

                if (copy_from_user(&pci_data, (CHIPSET_PCI_ARG)local_args.w_buf, sizeof(CHIPSET_PCI_ARG_NODE))) {
                    status = OS_FAULT;
                    goto cleanup;
                }

                status = PCI_Write_To_Memory_Address(CHIPSET_PCI_ARG_address(&pci_data),
                                                     CHIPSET_PCI_ARG_value(&pci_data));
                break;
            }

        case LWPMUDRV_IOCTL_FD_PHYS:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_FD_PHYS\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Samp_Find_Physical_Address(&local_args);
            break;

        case LWPMUDRV_IOCTL_READ_PCI_CONFIG:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_READ_PCI_CONFIG\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Samp_Read_PCI_Config(&local_args);
            break;

        case LWPMUDRV_IOCTL_WRITE_PCI_CONFIG:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_WRITE_PCI_CONFIG\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Samp_Write_PCI_Config(&local_args);
            break;

        case LWPMUDRV_IOCTL_CHIPSET_INIT:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_CHIPSET_INIT\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                SEP_PRINT_ERROR("LWPMUDRV_IOCTL_CHIPSET_INIT: copy from user failed!\n");
                status = OS_FAULT;
                goto cleanup;
            }
            SEP_PRINT_DEBUG("lwpmudrv_Device_Control: enable_chipset=%d\n", (int)DRV_CONFIG_enable_chipset(pcfg));
            status = lwpmudrv_Samp_Chipset_Init(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_CHIPSET_DEVICE_ID:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_CHIPSET_DEVICE_ID\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Samp_Read_PCI_Config(&local_args);
            break;
#endif  // BUILD_CHIPSET

       /*
        * if none of the above, treat as unknown/illegal IOCTL command
        */

        default:
            SEP_PRINT_ERROR("Unknown IOCTL magic:%d number:%d\n",
                    _IOC_TYPE(cmd), _IOC_NR(cmd));
            status = OS_ILLEGAL_IOCTL;
            break;
    }
cleanup:
    MUTEX_UNLOCK(ioctl_lock, flags);

    if (cmd == LWPMUDRV_IOCTL_STOP &&
        GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_PREPARE_STOP) {
        lwpmudrv_Finish_Stop();
    }

    return status;
}

#if defined(HAVE_COMPAT_IOCTL) && defined(DRV_EM64T)
extern long
lwpmu_Device_Control_Compat (
    struct   file   *filp,
    unsigned int     cmd,
    unsigned long    arg
)
{
    int              status = OS_SUCCESS;
    IOCTL_ARGS_NODE  local_args;
#if defined(DRV_USE_UNLOCKED_IOCTL)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
    unsigned long    flags;
#endif
#endif

    SEP_PRINT_DEBUG("Compat: type: %d, subcommand: %d\n",_IOC_TYPE(cmd),_IOC_NR(cmd));

    if (_IOC_TYPE(cmd) != LWPMU_IOC_MAGIC) {
        return OS_ILLEGAL_IOCTL;
    }

    MUTEX_LOCK(ioctl_lock, flags);
    switch (cmd) {
        case LWPMUDRV_IOCTL_VERSION:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_VERSION\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Version(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_DRIVER_STATE:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_DRIVER_STATE\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_Driver_State(&local_args);
            break;

        case LWPMUDRV_IOCTL_GET_NORMALIZED_TSC:
            SEP_PRINT_DEBUG("LWPMUDRV_IOCTL_GET_NORMALIZED_TSC\n");
            if (copy_from_user(&local_args, (IOCTL_ARGS)arg, sizeof(IOCTL_ARGS_NODE))) {
                status = OS_FAULT;
                goto cleanup;
            }
            status = lwpmudrv_Get_Normalized_TSC(&local_args);
            break;

    }
cleanup:
    MUTEX_UNLOCK(ioctl_lock, flags);

    return status;
}
#endif

/*****************************************************************************************
 *
 *   Driver Entry / Exit functions that will be called on when the driver is loaded and
 *   unloaded
 *
 ****************************************************************************************/

/*
 * Structure that declares the usual file access functions
 * First one is for lwpmu_c, the control functions
 */
static struct file_operations lwpmu_Fops = {
    .owner =   THIS_MODULE,
    IOCTL_OP = lwpmu_Device_Control,
#if defined(HAVE_COMPAT_IOCTL) && defined(DRV_EM64T)
    .compat_ioctl = lwpmu_Device_Control_Compat,
#endif
    .read =    lwpmu_Read,
    .write =   lwpmu_Write,
    .open =    lwpmu_Open,
    .release = NULL,
    .llseek =  NULL,
};

/*
 * Second one is for lwpmu_m, the module notification functions
 */
static struct file_operations lwmod_Fops = {
    .owner =   THIS_MODULE,
    IOCTL_OP = NULL,                //None needed
    .read =    OUTPUT_Module_Read,
    .write =   NULL,                //No writing accepted
    .open =    lwpmu_Open,
    .release = NULL,
    .llseek =  NULL,
};

/*
 * Third one is for lwsamp_nn, the sampling functions
 */
static struct file_operations lwsamp_Fops = {
    .owner =   THIS_MODULE,
    IOCTL_OP = NULL,                //None needed
    .read =    OUTPUT_Sample_Read,
    .write =   NULL,                //No writing accepted
    .open =    lwpmu_Open,
    .release = NULL,
    .llseek =  NULL,
};

/*
 * lwpmu_setup_cdev
 *
 *     Parameters
 *         dev        - pointer to the device object
 *         fops       - point to file operations struct
 *         dev_number - major/minor device number
 *
 *     Returns
 *         int
 *
 *     Description
 *         Set up the device object.
 */
static int
lwpmu_setup_cdev (
    LWPMU_DEV  dev,
    struct file_operations *fops,
    dev_t      dev_number
)
{
    cdev_init(&LWPMU_DEV_cdev(dev), fops);
    LWPMU_DEV_cdev(dev).owner = THIS_MODULE;
    LWPMU_DEV_cdev(dev).ops   = fops;

    return cdev_add(&LWPMU_DEV_cdev(dev), dev_number, 1);
}

/*
 * lwpmu_Load
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         int
 *
 *     Description
 *         Load the driver module into the kernel.  Set up the driver object.
 *         Set up the initial state of the driver and allocate the memory
 *         needed to keep basic state information.
 */
static int
lwpmu_Load (
    VOID
)
{
    int   i, result, num_cpus;
    dev_t lwmod_DevNum;
#if defined (DRV_ANDROID)
    char       dev_name[MAXNAMELEN];
#endif

    /* Get one major device number and two minor numbers. */
    /*   The result is formatted as major+minor(0) */
    /*   One minor number is for control (lwpmu_c), */
    /*   the other (lwpmu_m) is for modules */
    SEP_PRINT_DEBUG("lwpmu driver loading...\n");
    SEP_PRINT_DEBUG("lwpmu driver about to register chrdev...\n");

    lwpmu_DevNum = MKDEV(0, 0);
    result = alloc_chrdev_region(&lwpmu_DevNum, 0, PMU_DEVICES, SEP_DRIVER_NAME);
    SEP_PRINT_DEBUG("result of alloc_chrdev_region is %d\n", result);
    if (result<0) {
        SEP_PRINT_ERROR("lwpmu driver failed to alloc chrdev_region!\n");
        return result;
    }
    SEP_PRINT_DEBUG("lwpmu driver: major number is %d\n", MAJOR(lwpmu_DevNum));
    result = lwpmudrv_Initialize_State();
    num_cpus = GLOBAL_STATE_num_cpus(driver_state);
    SEP_PRINT_DEBUG("detected %d CPUs in lwpmudrv_Load\n", num_cpus);

    /* Allocate memory for the control structures */
    lwpmu_control = CONTROL_Allocate_Memory(sizeof(LWPMU_DEV_NODE));
    lwmod_control = CONTROL_Allocate_Memory(sizeof(LWPMU_DEV_NODE));
    lwsamp_control= CONTROL_Allocate_Memory(num_cpus*sizeof(LWPMU_DEV_NODE));
    
    if (!lwsamp_control || !lwpmu_control || !lwmod_control) {
        CONTROL_Free_Memory(lwpmu_control);
        CONTROL_Free_Memory(lwmod_control);
        CONTROL_Free_Memory(lwsamp_control);
        return OS_NO_MEM;
    }

    /* Register the file operations with the OS */

#if defined (DRV_ANDROID)
    pmu_class = class_create(THIS_MODULE, SEP_DRIVER_NAME);
    if (IS_ERR(pmu_class)) {
        SEP_PRINT_ERROR("Error registering SEP control class\n");
    }
    device_create(pmu_class, NULL, lwpmu_DevNum, NULL, SEP_DRIVER_NAME DRV_DEVICE_DELIMITER"c");
#endif

    result = lwpmu_setup_cdev(lwpmu_control,&lwpmu_Fops,lwpmu_DevNum);
    if (result) {
        SEP_PRINT_ERROR("Error %d adding lwpmu as char device\n", result);
        return result;
    }
    /* _c init was fine, now try _m */
    lwmod_DevNum = MKDEV(MAJOR(lwpmu_DevNum),MINOR(lwpmu_DevNum)+1);

#if defined (DRV_ANDROID)
    device_create(pmu_class, NULL, lwmod_DevNum, NULL, SEP_DRIVER_NAME DRV_DEVICE_DELIMITER"m");
#endif

    result = lwpmu_setup_cdev(lwmod_control,&lwmod_Fops,lwmod_DevNum);
    if (result) {
        SEP_PRINT_ERROR("Error %d adding lwpmu as char device\n", result);
        cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
        return result;
    }

    /* allocate one sampling device per cpu */
    lwsamp_DevNum = MKDEV(0, 0);
    result = alloc_chrdev_region(&lwsamp_DevNum, 0, num_cpus, SEP_SAMPLES_NAME);

    if (result<0) {
        SEP_PRINT_ERROR("lwpmu driver failed to alloc chrdev_region!\n");
        return result;
    }

    /* Register the file operations with the OS */
    for (i=0; i<num_cpus; i++) {
#if defined (DRV_ANDROID)
        snprintf(dev_name, MAXNAMELEN, "%s%ss%d", SEP_DRIVER_NAME, DRV_DEVICE_DELIMITER, i);
        device_create(pmu_class, NULL, lwsamp_DevNum+i, NULL, dev_name);
#endif
        result = lwpmu_setup_cdev(lwsamp_control+i,
                                  &lwsamp_Fops,
                                  lwsamp_DevNum+i);
        if (result) {
            SEP_PRINT_ERROR("Error %d adding lwpmu as char device\n", result);
            return result;
        }
        else {
            SEP_PRINT_DEBUG("added sampling device %d\n", i);
        }
    }

    pcb_size = GLOBAL_STATE_num_cpus(driver_state)*sizeof(CPU_STATE_NODE);
    pcb      = CONTROL_Allocate_Large_Memory(pcb_size);
    SYS_INFO_Build();
    pcb      = CONTROL_Free_Large_Memory(pcb, pcb_size);
    pcb_size = 0;
    if (total_ram <= OUTPUT_MEMORY_THRESHOLD) {
        output_buffer_size = OUTPUT_SMALL_BUFFER;
    }

    mutex_init(&ioctl_lock);
    in_finish_code = 0;

    /*
     *  Initialize the SEP driver version (done once at driver load time)
     */
    SEP_VERSION_NODE_major(&drv_version) = SEP_MAJOR_VERSION;
    SEP_VERSION_NODE_minor(&drv_version) = SEP_MINOR_VERSION;
    SEP_VERSION_NODE_api(&drv_version)   = SEP_API_VERSION;

    //
    // Display driver version information
    //
    SEP_PRINT("PMU collection driver v%d.%d.%d%s has been loaded.\n",
              SEP_VERSION_NODE_major(&drv_version),
              SEP_VERSION_NODE_minor(&drv_version),
              SEP_VERSION_NODE_api(&drv_version),
              SEP_DRIVER_MODE);

#if defined(DRV_IA64)
#if defined(PERFMON_V2)
    SEP_PRINT("Using Perfmon v2 infrastructure.\n");
#elif defined(PERFMON_V2_ALT)
    SEP_PRINT("Using alternate Perfmon v2 infrastructure.\n");
#elif defined(PERFMON_V3)
    SEP_PRINT("Using Perfmon v3 infrastructure.\n");
#endif
#endif

#if defined(BUILD_CHIPSET)
    SEP_PRINT("Chipset support is enabled.\n");
#endif

#if defined(BUILD_GFX)
    SEP_PRINT("Graphics support is enabled.\n");
#endif

#if defined(DRV_IA32) || defined(DRV_EM64T)
    SEP_PRINT("IDT vector 0x%x will be used for handling PMU interrupts.\n", CPU_PERF_VECTOR);
#endif

    return result;
}

/*
 * lwpmu_Unload
 *
 *     Parameters
 *         NONE
 *
 *     Returns
 *         NONE
 *
 *     Description
 *         Remove the driver module from the kernel.
 */
static VOID
lwpmu_Unload (
    VOID
)
{
    int i = 0;
    int num_cpus = GLOBAL_STATE_num_cpus(driver_state);

    SEP_PRINT_DEBUG("lwpmu driver unloading...\n");
    SYS_INFO_Destroy();
    OUTPUT_Destroy();
    cpu_buf                 = CONTROL_Free_Memory(cpu_buf);
    module_buf              = CONTROL_Free_Memory(module_buf);
    pcb                     = CONTROL_Free_Large_Memory(pcb, pcb_size);
    pcb_size                = 0;
#if defined (DRV_ANDROID)
    unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
    device_destroy(pmu_class, lwpmu_DevNum);
    device_destroy(pmu_class, lwpmu_DevNum+1);
#endif

    cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
    cdev_del(&LWPMU_DEV_cdev(lwmod_control));
    unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
#if defined (DRV_ANDROID)
    unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
#endif

    for (i = 0; i < num_cpus; i++) {
#if defined (DRV_ANDROID)
        device_destroy(pmu_class, lwsamp_DevNum+i);
#endif
        cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[i]));
    }

#if defined (DRV_ANDROID)
    class_destroy(pmu_class);
#endif
    unregister_chrdev_region(lwsamp_DevNum, num_cpus);
    lwpmu_control  = CONTROL_Free_Memory(lwpmu_control);
    lwmod_control  = CONTROL_Free_Memory(lwmod_control);
    lwsamp_control = CONTROL_Free_Memory(lwsamp_control);

    //
    // Display driver version information
    //
    SEP_PRINT("PMU collection driver v%d.%d.%d%s has been unloaded.\n",
              SEP_VERSION_NODE_major(&drv_version),
              SEP_VERSION_NODE_minor(&drv_version),
              SEP_VERSION_NODE_api(&drv_version),
              SEP_DRIVER_MODE);

    return;
}

/* Declaration of the init and exit functions */
module_init(lwpmu_Load);
module_exit(lwpmu_Unload);
