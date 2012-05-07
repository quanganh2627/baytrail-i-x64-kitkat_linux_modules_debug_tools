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
 *  CVS_Id="$Id$"
 */

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/mempool.h>
#include <linux/slab.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv.h"
#include "control.h"
#include <linux/sched.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)    smp_call_function((func),(ctx),(wait))
#else
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)    smp_call_function((func),(ctx),(retry),(wait))
#endif

/*
 *  Global State Nodes - keep here for now.  Abstract out when necessary.
 */
GLOBAL_STATE_NODE  driver_state;
MSR_DATA           msr_data      = NULL;

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID CONTROL_Invoke_Cpu (func, ctx, arg)
 *
 * @brief       Set up a DPC call and insert it into the queue
 *
 * @param       IN cpu_idx  - the core id to dispatch this function to
 *              IN func     - function to be invoked by the specified core(s)
 *              IN ctx      - pointer to the parameter block for each function invocation
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
extern VOID 
CONTROL_Invoke_Cpu (
    int     cpu_idx, 
    VOID    (*func)(PVOID), 
    PVOID   ctx
)
{
    CONTROL_Invoke_Parallel(func, ctx);

    return;
}

/*
 * CONTROL_Invoke_Parallel_Service
 *     Parameters
 *         IN func     - function to be invoked by each core in the system
 *         IN ctx      - pointer to the parameter block for each function invocation
 *         IN blocking - Wait for invoked function to complete
 *         IN exclude  - exclude the current core from executing the code
 *     Returns
 *         None
 *
 *     Description
 *         Invoke the function provided in parallel in either a blocking/non-blocking mode.
 *         The current core may be excluded if desired.
 *         NOTE - Do not call this function directly from source code.  Use the aliases
 *         CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(), CONTROL_Invoke_Parallel_XS().
 *
 */
extern VOID 
CONTROL_Invoke_Parallel_Service (
        VOID   (*func)(PVOID), 
        PVOID  ctx,
        int    blocking,
        int    exclude
)
{
    GLOBAL_STATE_cpu_count(driver_state) = 0;
    GLOBAL_STATE_dpc_count(driver_state) = 0;

    preempt_disable();
    SMP_CALL_FUNCTION (func, ctx, 0, blocking);

    if (!exclude) {
        func(ctx);
    }
    preempt_enable();

    return;
}

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
)
{
    SEP_PRINT_ERROR("CONTROL_Invoke_Serial is not implemented on Linux\n" );

    return;
}

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
 */
extern PVOID
CONTROL_Allocate_Memory (
    size_t  size
)
{
    PVOID   location = NULL;

    if (size > MAX_KMALLOC_SIZE) {
        SEP_PRINT_ERROR(
               "CONTROL_Allocate_Memory failed: request for %d too large for kmalloc\n", 
               (S32) size);
        return location;
    }

    if (size) {
        location = (PVOID)kmalloc(size, GFP_KERNEL);
        
        if (!location) {
            SEP_PRINT_ERROR("CONTROL_Allocate_Memory() failed for %d bytes\n", (S32) size);
            return NULL;
        }
        memset(location, '\0', size);
        SEP_PRINT_DEBUG("CONTROL_Allocate_Memory() location %p --- size %d\n", 
               location, (S32) size);
    }

    return location;
}

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
 *               Allocate memory in the GFP_Atomic pool
 */
extern PVOID
CONTROL_Allocate_KMemory (
    size_t  size
)
{
    PVOID   location = NULL;

    if (size > MAX_KMALLOC_SIZE) {
        SEP_PRINT_ERROR(
               "CONTROL_Allocate_Memory failed: request for %d too large for kmalloc\n", 
               (S32) size);
        return location;
    }

    if (size) {
        location = (PVOID)kmalloc(size, GFP_ATOMIC);
        
        if (!location) {
            SEP_PRINT_ERROR("CONTROL_Allocate_Memory() failed for %d bytes\n", (S32) size);
            return NULL;
        }
        memset(location, '\0', size);
        SEP_PRINT_DEBUG("CONTROL_Allocate_Memory() location %p --- size %d\n", 
               location, (S32) size);
    }

    return location;
}
/*
 * CONTROL_Free_Memory
 *
 *     Parameters
 *         IN location - pointer to the memory block to free
 *
 *     Returns
 *         Nothing
 *
 *     Description
 *         Frees the memory block
 *
 */
extern PVOID
CONTROL_Free_Memory (
    PVOID  location
)
{
    SEP_PRINT_DEBUG("CONTROL_Free_Memory() location %p\n", location);
    if (location) {
        kfree(location);
    }

    return NULL;
}

/*
 * @fn PVOID CONTROL_Allocate_Large_Memory(size)
 *
 * @param    IN size     - size of the memory to allocate
 *
 * @returns  PVIOD       - pointer to the allocated memory block
 *
 *     Description
 *         Allocate the memory and zero it out
 *
 */
extern PVOID
CONTROL_Allocate_Large_Memory (
    size_t  size
)
{
    PVOID   location = NULL;

    if (size == 0) {
        return location;
    }

    if (size < MAX_KMALLOC_SIZE) {
        location = CONTROL_Allocate_Memory(size);
        return location;
    }
    location = (PVOID)__get_free_pages(GFP_KERNEL, get_order(size));
    
    if (!location) {
        SEP_PRINT_ERROR("CONTROL_Allocate_Memory() failed for %d bytes\n", (S32)size);
        return NULL;
    }
    memset(location, '\0', size);
    SEP_PRINT_DEBUG("CONTROL_Allocate_Large_Memory() location %p --- size %d\n", location, size);

    return location;
}

/*
 * @fn CONTROL_Free_Large_Memory(location)
 *
 * @pararm IN   location - pointer to the memory block to free
 *
 * @brief       Frees the memory large block
 *
 */
extern PVOID
CONTROL_Free_Large_Memory (
    PVOID      location,
    size_t     size
)
{
    SEP_PRINT_DEBUG("CONTROL_Free_Large_Memory() location %p\n", location);
    if (location == NULL) {
        return NULL;
    }
    if (size < MAX_KMALLOC_SIZE) {
        CONTROL_Free_Memory(location);
    }
    else {
        free_pages((unsigned long)location, get_order(size));
    }

    return NULL;
}
