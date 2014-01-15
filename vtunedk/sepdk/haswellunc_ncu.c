/*COPYRIGHT**
    Copyright (C) 2011-2012 Intel Corporation.  All Rights Reserved.

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


#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "haswellunc_ncu.h"
#include "ecb_iterators.h"
#include "pebs.h"

extern U64           *read_counter_info;
extern DRV_CONFIG     pcfg;

/*!
 * @fn          static VOID haswellunc_ncu_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID
haswellunc_ncu_Write_PMU (
    VOID  *param
)
{
    U32 dev_idx = *((U32*)param);

    FOR_EACH_REG_ENTRY_UNC(pecb_unc, dev_idx, i) {
        if (ECB_entries_reg_id(pecb_unc,i) == NCU_PERF_GLOBAL_CTRL) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb_unc,i), 0LL);
            continue;
        }

        SYS_Write_MSR(ECB_entries_reg_id(pecb_unc,i), ECB_entries_reg_value(pecb_unc,i));
#if defined(MYDEBUG)
        SEP_PRINT_DEBUG("haswellunc_ncu_Write_PMU Event_Data_reg = 0x%x --- value 0x%llx\n",
                        ECB_entries_reg_id(pecb_unc,i), ECB_entries_reg_value(pecb_unc,i));
#endif
        if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
            LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb_unc,i);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/*!
 * @fn         static VOID haswellunc_ncu_Disable_PMU(PVOID)
 *
 * @brief      Zero out the global control register.  This automatically disables the
 *             PMU counters.
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
haswellunc_ncu_Disable_PMU (
    PVOID  param
)
{
    SYS_Write_MSR(NCU_PERF_GLOBAL_CTRL, 0LL);

    return;
}

/*!
 * @fn         static VOID haswellunc_ncu_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the CCCR registers
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
haswellunc_ncu_Enable_PMU (
    PVOID   param
)
{
    ECB pecb_unc;
    U32 dev_idx = *((U32*)param);

    pecb_unc = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[0];
    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_RUNNING) {
        SYS_Write_MSR(NCU_PERF_GLOBAL_CTRL, ECB_entries_reg_value(pecb_unc,0));
    }

    return;
}

/*!
 * @fn         static VOID haswellunc_ncu_Read_PMU_Data(PVOID)
 *
 * @brief      Read all the data MSR's into a buffer.
 *             Called by the interrupt handler
 *
 * @param      None
 *
 * @return     None
 */
static void
haswellunc_ncu_Read_PMU_Data(
    PVOID   param
)
{
    S32       start_index, j;
    U64      *buffer    = read_counter_info;
    U32       this_cpu  = CONTROL_THIS_CPU();

    start_index = DRV_CONFIG_num_events(pcfg) * this_cpu;
    FOR_EACH_DATA_REG(pecb_unc,i) {
        j = start_index + ECB_entries_event_id_index(pecb_unc,i);
        buffer[j] = SYS_Read_MSR(ECB_entries_reg_id(pecb_unc,i));
    } END_FOR_EACH_DATA_REG;

    return;
}

/*!
 * @fn         static VOID haswellunc_ncu_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
static VOID
haswellunc_ncu_Clean_Up (
    VOID   *param
)
{
    U32 dev_idx = *((U32*)param);

    FOR_EACH_REG_ENTRY_UNC(pecb_unc, dev_idx, i) {
        if (ECB_entries_clean_up_get(pecb_unc,i)) {
            SYS_Write_MSR(ECB_entries_reg_id(pecb_unc,i), 0LL);
        }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn haswellunc_ncu_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The event id for the which the sample is generated
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *           Uncore PMU does not support sampling, i.e. ignore the id parameter.
 */
static VOID
haswellunc_ncu_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64            *data;
    int             data_index;
    U32             event_id            = 0;
    U32             dev_idx             = id;

    data       = (U64 *)param;
    data_index = 0;

    // Write GroupID
    data[data_index] = 1;
    // Increment the data index as the event id starts from zero
    data_index++;

    FOR_EACH_DATA_REG_UNC(pecb,dev_idx,i) {
        event_id = ECB_entries_event_id_index_local(pecb,i);
        data[data_index + event_id] = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
    } END_FOR_EACH_DATA_REG_UNC;

    return;
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  haswellunc_ncu_dispatch =
{
    NULL,                          // initialize
    NULL,                          // destroy
    haswellunc_ncu_Write_PMU,      // write
    haswellunc_ncu_Disable_PMU,    // freeze
    haswellunc_ncu_Enable_PMU,     // restart
    haswellunc_ncu_Read_PMU_Data,  // read
    NULL,                          // check for overflow
    NULL,
    NULL,
    haswellunc_ncu_Clean_Up,
    NULL,
    NULL,
    NULL,
    haswellunc_ncu_Read_Counts,
    NULL,                          // check_overflow_gp_errata
    NULL,                          // read_ro
    NULL                           // platform_info
};
