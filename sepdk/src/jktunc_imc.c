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
#include "jktunc_imc.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern EVENT_CONFIG   global_ec;
extern U64           *read_counter_info;
extern LBR            lbr;
extern DRV_CONFIG     pcfg;
extern PWR            pwr;

/*!
 * @fn          static VOID jktunc_imc_Write_PMU(VOID*)
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
jktunc_imc_Write_PMU (
    VOID  *param
)
{
    U32                        pci_address;
    U32                        device_id;
    U32                        dev_idx   = *((U32*)param);
    U32                        value;
    U32                        vendor_id;
    U32                        busno;
    
    FOR_EACH_REG_ENTRY_UNC(pecb,dev_idx,idx) {
            // otherwise, we have a valid entry
            // now we just need to find the corresponding bus #
            for (busno = 0; busno < 256; busno++) {
                pci_address = FORM_PCI_ADDR(busno,
                                            ECB_entries_dev_no(pecb,idx),
                                            ECB_entries_func_no(pecb,idx),
                                            0);
                value = PCI_Read_Ulong(pci_address);
                vendor_id = value & 0x0000FFFF;
                device_id = (value & 0xFFFF0000) >> 16;

                if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
                  continue;
                }
                if ((device_id != JKTUNC_IMC0_DID) &&
                    (device_id != JKTUNC_IMC1_DID) &&
                    (device_id != JKTUNC_IMC2_DID) &&
                    (device_id != JKTUNC_IMC3_DID)) {
                  continue;
                }
                
                // otherwise, we found the bus # for our device.
                // fill in the corresponding bus #
                ECB_entries_bus_no(pecb,idx) = busno;
/*              SEP_PRINT("Bus no = 0x%x\n",ECB_entries_bus_no(pecb,idx)); */
/*              SEP_PRINT("Dev no = 0x%x\n",ECB_entries_dev_no(pecb,idx)); */
/*              SEP_PRINT("Func no = 0x%x\n",ECB_entries_func_no(pecb,idx)); */

/*              // now program at the corresponding offset */
/*              SEP_PRINT("offset = 0x%x\n", ECB_entries_reg_id(pecb,idx)); */
/*              SEP_PRINT("value = 0x%x\n", ECB_entries_reg_value(pecb,idx)); */

                pci_address = FORM_PCI_ADDR(busno,
                                            ECB_entries_dev_no(pecb,idx),
                                            ECB_entries_func_no(pecb,idx),
                                            ECB_entries_reg_id(pecb,idx));
                PCI_Write_Ulong(pci_address, (U32)ECB_entries_reg_value(pecb,idx));

                // we're zeroing out a data register, which is 48 bits long
                // we need to zero out the upper bits as well
                if (ECB_entries_reg_type(pecb,idx) == DATA) {
                    pci_address = FORM_PCI_ADDR(busno,
                                                ECB_entries_dev_no(pecb,idx),
                                                ECB_entries_func_no(pecb,idx),
                                                (ECB_entries_reg_id(pecb,idx) + NEXT_ADDR_OFFSET));
                    PCI_Write_Ulong(pci_address, (U32)ECB_entries_reg_value(pecb,idx));
                }
            }
    } END_FOR_EACH_REG_ENTRY_UNC;

    return;
}

/*!
 * @fn         static VOID jktunc_imc_Disable_PMU(PVOID)
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
jktunc_imc_Disable_PMU (
    PVOID  param
)
{
    SEP_PRINT_DEBUG("jktunc_imc_Disable_PMU\n");
    return;
}

/*!
 * @fn         static VOID jktunc_imc_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
static void
jktunc_imc_Clean_Up (
    VOID   *param
)
{
    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn jktunc_imc_Read_Counts(param, id)
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
jktunc_imc_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64            *data;
    int             data_index;
    U32             event_id            = 0;
    U32             dev_idx             = id;
    U32             pci_address;
    U64             value_high          = 0;
    U64             value_lo_2          = 0;

    data       = (U64 *)param;
    data_index = 0;

    // Write GroupID
    data[data_index] = 1;
    // Increment the data index as the event id starts from zero
    data_index++;

    //Read in the counts into temporary buffer
    FOR_EACH_DATA_REG_UNC(pecb,dev_idx,i) {
            event_id                    = ECB_entries_event_id_index_local(pecb,i);
            
            // read lower 4 bytes
            pci_address                 = FORM_PCI_ADDR(ECB_entries_bus_no(pecb,i),
                                                        ECB_entries_dev_no(pecb,i),
                                                        ECB_entries_func_no(pecb,i),
                                                        ECB_entries_reg_id(pecb,i));        
            data[data_index + event_id] = 0x00000000FFFFFFFF & PCI_Read_Ulong(pci_address);
            
            // read upper 4 bytes
            pci_address                 = FORM_PCI_ADDR(ECB_entries_bus_no(pecb,i),
                                                        ECB_entries_dev_no(pecb,i),
                                                        ECB_entries_func_no(pecb,i),
                                                        (ECB_entries_reg_id(pecb,i) + NEXT_ADDR_OFFSET));
            value_high                  = (U64)PCI_Read_Ulong(pci_address);
            // Now we have to check if the lower 32 bits overflowed in between the reads
            // reread lower 4 bytes
            pci_address                 = FORM_PCI_ADDR(ECB_entries_bus_no(pecb,i),
                                                        ECB_entries_dev_no(pecb,i),
                                                        ECB_entries_func_no(pecb,i),
                                                        ECB_entries_reg_id(pecb,i));        
            value_lo_2                  = 0x00000000FFFFFFFF & PCI_Read_Ulong(pci_address);         
            if (value_lo_2 < data[data_index + event_id]) {
                // overflow occurred
                // use new lower bits
                data[data_index + event_id] = value_lo_2;
                // reread the top bits as well.
                pci_address                 = FORM_PCI_ADDR(ECB_entries_bus_no(pecb,i),
                                                            ECB_entries_dev_no(pecb,i),
                                                            ECB_entries_func_no(pecb,i),
                                                            (ECB_entries_reg_id(pecb,i) + NEXT_ADDR_OFFSET));
                value_high                  = (U64)PCI_Read_Ulong(pci_address);
            }       
            data[data_index + event_id] |= value_high << IMC_ADDR_SHIFT;

    } END_FOR_EACH_DATA_REG_UNC;

    return;
}
/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  jktunc_imc_dispatch =
{
    NULL,                        // initialize
    NULL,                        // destroy
    jktunc_imc_Write_PMU,        // write
    jktunc_imc_Disable_PMU,      // freeze
    NULL,                        // restart
    NULL,                        // reinit
    NULL,                        // read
    NULL,                        // check for overflow
    NULL,
    NULL,
    jktunc_imc_Clean_Up,
    NULL,
    NULL,
    NULL,
    jktunc_imc_Read_Counts,      //read_counts
    NULL,
    NULL,
    NULL
};
