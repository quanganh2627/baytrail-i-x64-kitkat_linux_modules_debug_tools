/*COPYRIGHT**
    Copyright 2005-2011 Intel Corporation.  All Rights Reserved.

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
#include "snbunc_imc.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern EVENT_CONFIG   global_ec;
extern U64           *read_counter_info;
extern LBR            lbr;
extern DRV_CONFIG     pcfg;
extern PWR            pwr;
extern U32            invoking_processor_id;
PVOID                 virtual_address;

/*!
 * @fn          static VOID snbunc_imc_Write_PMU(VOID*)
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
snbunc_imc_Write_PMU (
    VOID  *param
)
{
    
    DRV_PCI_DEVICE_ENTRY_NODE  dpden;
    U32                        pci_address;
    U32                        bar_lo;
    U64                        next_bar_offset;
    U64                        bar_hi;
    U64                        physical_address;
    U64                        final_bar;
    U32                        dev_idx   = *((U32*)param);
    ECB                        pecb      = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[0];
    U32                        j;
    U32                        event_id   = 0;
    U32                        offset_delta;
    U32                        tmp_value;
    int                        me      = CONTROL_THIS_CPU();
    
    if (me != invoking_processor_id) {
        return;
    }

    SEP_PRINT_DEBUG("snbunc_imc_Write_PMU Enter\n");
    dpden = ECB_pcidev_entry_node(pecb);
    pci_address = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
                                0);

#if defined(MYDEBUG)
    {
    U32 device_id  = PCI_Read_Ulong(pci_address);
    SEP_PRINT("Bus no = 0x%x\n",DRV_PCI_DEVICE_ENTRY_bus_no(&dpden));
    SEP_PRINT("Dev no = 0x%x\n",DRV_PCI_DEVICE_ENTRY_dev_no(&dpden));
    SEP_PRINT("Func no = 0x%x\n",DRV_PCI_DEVICE_ENTRY_func_no(&dpden));
    SEP_PRINT("value for device id = 0x%x\n",device_id);
    }
#endif

    pci_address = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden));
    bar_lo      = PCI_Read_Ulong(pci_address);
        
    next_bar_offset     = DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden) + NEXT_ADDR_OFFSET;
    pci_address         = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
                                DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
                                next_bar_offset);
    bar_hi              = PCI_Read_Ulong(pci_address);
    final_bar = (bar_hi << SNBUNC_IMC_BAR_ADDR_SHIFT) | bar_lo;
    final_bar &= SNBUNC_IMC_BAR_ADDR_MASK;

    DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb)) = final_bar;
    physical_address     = DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb))
                                 + DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(&ECB_pcidev_entry_node(pecb));
    virtual_address      = ioremap_nocache(physical_address,4096); 
    //Read in the counts into temporary buffer
    FOR_EACH_PCI_DATA_REG(pecb,i,dev_idx,offset_delta) {
            event_id                            = ECB_entries_event_id_index_local(pecb,i);
            tmp_value                           = readl((U32*)((char*)(virtual_address) + offset_delta));
            for ( j = 0; j < (U32)GLOBAL_STATE_num_cpus(driver_state) ; j++) {
                   LWPMU_DEVICE_prev_val_per_thread(&devices[dev_idx])[j][event_id + 1] = tmp_value; // need to account for group id
#if defined(MYDEBUG)
                   DbgPrint("initial value for i =%d is 0x%x\n",i,LWPMU_DEVICE_prev_val_per_thread(&devices[dev_idx])[j][i]);

#endif
            }

            // this is needed for overflow detection of the accumulators.
            if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
                LWPMU_DEVICE_counter_mask(&devices[dev_idx]) = (U64)ECB_entries_max_bits(pecb,i);
            }
    } END_FOR_EACH_PCI_DATA_REG;
#if defined(MYDEBUG)
    SEP_PRINT("BAR address is 0x%llx and virt is 0x%llx phys is 0x%llx\n",DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb)), virtual_address, physical_address);
#endif
    return;
}

/*!
 * @fn         static VOID snbunc_imc_Disable_PMU(PVOID)
 * 
 * @brief      Unmap the virtual address when you stop sampling.
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static void
snbunc_imc_Disable_PMU (
    PVOID  param
)
{
    int me = CONTROL_THIS_CPU();
    
    if (me != invoking_processor_id) {
        return;
    }
  
    SEP_PRINT_DEBUG("snbunc_imc_Disable_PMU : Unmapping the address\n");
    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_STOPPED) {
        iounmap((void*)(UIOP)(virtual_address));
    }
    return;
}

/*!
 * @fn         static VOID snbunc_imc_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
static void
snbunc_imc_Clean_Up (
    VOID   *param
)
{

    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn snbunc_imc_Read_Counts(param, id)
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
snbunc_imc_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64            *data;
    int             data_index;
    U32             event_id            = 0;
    U32             offset_delta;
    U32             dev_idx             = id;

    data       = (U64 *)param;
    data_index = 0;

    // Write GroupID
    data[data_index] = 1;
    // Increment the data index as the event id starts from zero
    data_index++;

    //Read in the counts into temporary buffer
    FOR_EACH_PCI_DATA_REG(pecb,i,dev_idx, offset_delta) {
            event_id                    = ECB_entries_event_id_index_local(pecb,i);
            data[data_index + event_id] = readl((U32*)((char*)(virtual_address) + offset_delta));
    } END_FOR_EACH_PCI_DATA_REG;
}
/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  snbunc_imc_dispatch =
{
    NULL,                        // initialize
    NULL,                        // destroy
    snbunc_imc_Write_PMU,        // write
    snbunc_imc_Disable_PMU,      // freeze
    NULL,                        // restart
    NULL,                        // reinit
    NULL,                        // read
    NULL,                        // check for overflow
    NULL,
    NULL,
    snbunc_imc_Clean_Up,
    NULL,
    NULL,
    NULL,
    snbunc_imc_Read_Counts,//read_counts
    NULL,
    NULL,
    NULL
};
