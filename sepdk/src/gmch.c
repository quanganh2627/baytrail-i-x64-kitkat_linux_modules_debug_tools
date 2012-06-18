/*COPYRIGHT**
 * -------------------------------------------------------------------------
 *               INTEL CORPORATION PROPRIETARY INFORMATION
 *  This software is supplied under the terms of the accompanying license
 *  agreement or nondisclosure agreement with Intel Corporation and may not
 *  be copied or disclosed except in accordance with the terms of that
 *  agreement.
 *        Copyright (c) 2007-2011 Intel Corporation.  All Rights Reserved.
 * -------------------------------------------------------------------------
**COPYRIGHT*/

#include <linux/version.h>
#include <linux/errno.h>
#include <linux/fs.h>

#include "lwpmudrv_defines.h"
#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"
#include "lwpmudrv_chipset.h"
#include "inc/lwpmudrv.h"
#include "inc/control.h"
#include "inc/ecb_iterators.h"
#include "inc/gmch.h"
#include "inc/pci.h"

// lock to allow only one processor to read GMCH counters at interrupt time
static volatile int lock_gmch_read = 0;

// global variables for determining which register offsets to use
static U32 gmch_register_read  = 0;     // value=0 indicates invalid read register
static U32 gmch_register_write = 0;     // value=0 indicates invalid write register

extern DRV_CONFIG        pcfg;
extern CHIPSET_CONFIG    pma;
extern CPU_STATE         pcb;
extern EVENT_CONFIG      global_ec;

/*
 * @fn        gmch_Check_Enabled()
 *
 * @brief     Read GMCH PMON capabilities
 *
 * @param     None
 *
 * @return    GMCH enable bits
 *
 */
static ULONG
gmch_Check_Enabled (
    VOID
)
{
    U32 gmch;
    ULONG enabled_value;

    gmch = FORM_PCI_ADDR(0, 0, 0, 0);
    if (gmch == 0) {
        SEP_PRINT_ERROR("gmch_Check_Enabled: unable to access PCI config space!\n");
        return 0;
    }

    PCI_Write_Ulong( (ULONG)(gmch + GMCH_MSG_CTRL_REG), (ULONG)(GMCH_PMON_CAPABILITIES + gmch_register_read));
    SEP_PRINT_DEBUG("gmch_Check_Enabled: wrote addr=0x%lx register_value=0x%lx\n", (ULONG)(gmch+GMCH_MSG_CTRL_REG), (ULONG)(GMCH_PMON_CAPABILITIES+gmch_register_read));

    enabled_value = PCI_Read_Ulong((ULONG)(gmch + GMCH_MSG_DATA_REG));

    SEP_PRINT_DEBUG("gmch_Check_Enabled: read addr=0x%lx enabled_value=0x%lx\n", (ULONG)(gmch+GMCH_MSG_DATA_REG), enabled_value);

    return enabled_value;
}

/*
 * @fn        gmch_Init_Chipset()
 *
 * @brief     Initialize GMCH Counters.  See note below.
 *
 * @param     None
 *
 * @note      This function must be called BEFORE any other function in this file!
 *
 * @return    VT_SUCCESS if successful, error otherwise
 *
 */
static U32
gmch_Init_Chipset (
    VOID
)
{
    CHIPSET_SEGMENT cs = &CHIPSET_CONFIG_gmch(pma);

    // configure the read/write registers offsets according to usermode setting
    if (cs) {
        gmch_register_read = CHIPSET_SEGMENT_read_register(cs);
        gmch_register_write = CHIPSET_SEGMENT_write_register(cs);;
    }
    if (gmch_register_read == 0 || gmch_register_write == 0) {
        SEP_PRINT_ERROR("Invalid GMCH read/write registers!\n");
        return VT_CHIPSET_CONFIG_FAILED;
    }

    if (DRV_CONFIG_enable_chipset(pcfg)) {
        if (CHIPSET_CONFIG_gmch_chipset(pma)) {
            int i, j;
            U32 gmch;
            // initialize the GMCH counter state
            for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
                pcb[i].chipset_count_init = TRUE;
                for (j = 0; j < 8; j++) {
                    pcb[i].last_gmch_count[j] = 0;
                }
            }
            gmch = FORM_PCI_ADDR(0, 0, 0, 0);
            if (gmch != 0) {
                // disable fixed and GP counters
                PCI_Write_Ulong(gmch + GMCH_MSG_DATA_REG, 0x00000000);
                PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_GLOBAL_CTRL + gmch_register_write);
                // clear fixed counter filter
                PCI_Write_Ulong(gmch + GMCH_MSG_DATA_REG, 0x00000000);
                PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_FIXED_CTR_CTRL + gmch_register_write);
            }
        }
        // always collect processor events
        CHIPSET_CONFIG_processor(pma) = 1;
    }
    else {
        CHIPSET_CONFIG_processor(pma) = 0;
    }

    return VT_SUCCESS;
}

/*
 * @fn        gmch_Start_Counters()
 *
 * @brief     Start the GMCH Counters.
 *
 * @param     None
 *
 * @return    None
 *
 */
static VOID
gmch_Start_Counters (
    VOID
)
{
    // reset and start chipset counters
    if (pma) {
        U32 gmch;
        gmch = FORM_PCI_ADDR(0, 0, 0, 0);
        if (gmch != 0) {
            // enable fixed and GP counters
            PCI_Write_Ulong(gmch + GMCH_MSG_DATA_REG, 0x0001000F);
            PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_GLOBAL_CTRL + gmch_register_write);
            // enable fixed counter filter
            PCI_Write_Ulong(gmch + GMCH_MSG_DATA_REG, 0x00000001);
            PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_FIXED_CTR_CTRL + gmch_register_write);
        }
    }
    else {
        SEP_PRINT_ERROR("gmch_Start_Counters: ERROR pma=NULL\n");
    }

    return;
}

/*
 * @fn        gmch_Read_Counters()
 *
 * @brief     Read the GMCH counters and record them in the sampling data stream.
 *
 * @param     param - pointer to data stream where samples are to be written
 *
 * @return    None
 *
 */
static VOID
gmch_Read_Counters (
    PVOID  param
)
{
    U64            *data;
    U32             gmch;
    U32             gmch_cpu;
    int             i, data_index;
    U64             tmp_data;
    U64            *gmch_data;
//  U64             gmch_data_valid[1]; // tbd: add this to the data stream ...
    U32             counter_data_low;
    U32             counter_data_high;
    U64             counter_data;
    U64             cmd_register_low_read;
    U64             cmd_register_high_read;
    U32             gp_counter_index    = 0;
    U32             this_cpu            = CONTROL_THIS_CPU();
    CHIPSET_SEGMENT gmch_chipset_seg    = &CHIPSET_CONFIG_gmch(pma);
    CHIPSET_EVENT   chipset_events      = CHIPSET_SEGMENT_events(gmch_chipset_seg);

    data       = param;
    data_index = 0;

    if (CHIPSET_CONFIG_gmch_chipset(pma)) {

        //
        // NOTE: reading GMCH counters assumes PCI config space operates
        //       coherently with multiple concurrent readers ... which doesn't
        //       seem to be the case ...
        //       so for now, a lock is used to ensure only one CPU reads
        //       the GMCH counters on interrupt ...
        //

        // place in sample record where GMCH data will be written as gmch_data[0], gmch_data[1], ...
        gmch_data = data + data_index;

        // tbd: initially mark GMCH data as invalid
        // gmch_data_valid[0] = DATA_IS_INVALID;  

        // first cpu to grab the lock gets to read the GMCH counters
        if (cmpxchg(&lock_gmch_read, 0, 1) != 0) {
            return;  // failed to grab lock, so return
        }
        
        // read the GMCH counters and add them into the sample record
        gmch = FORM_PCI_ADDR(0, 0, 0, 0);
        if (gmch == 0) {
            // invalid gmch base address, so release the lock and return
            lock_gmch_read = 0;
            return;
        }

        // iterate through GMCH counters that were configured to collect on the events
        for (i = 0; i < CHIPSET_SEGMENT_total_events(gmch_chipset_seg); i++) {
            U32 event_id = CHIPSET_EVENT_event_id(&chipset_events[i]);
            // read count for fixed GMCH counter event
            if (event_id == 0) {
                PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_FIXED_CTR0 + gmch_register_read);
                data[data_index++] = (U64)PCI_Read_Ulong(gmch + GMCH_MSG_DATA_REG);
            }
            else {
                // read count for general GMCH counter event
                switch (gp_counter_index) {
                    case 0:
                        cmd_register_low_read  = GMCH_PMON_GP_CTR0_L + gmch_register_read; // counter 0 low
                        cmd_register_high_read = GMCH_PMON_GP_CTR0_H + gmch_register_read; // counter 0 high
                        break;

                    case 1:
                        cmd_register_low_read  = GMCH_PMON_GP_CTR1_L + gmch_register_read; // counter 1 low
                        cmd_register_high_read = GMCH_PMON_GP_CTR1_H + gmch_register_read; // counter 1 high
                        break;

                    case 2:
                        cmd_register_low_read  = GMCH_PMON_GP_CTR2_L + gmch_register_read; // counter 2 low
                        cmd_register_high_read = GMCH_PMON_GP_CTR2_H + gmch_register_read; // counter 2 high
                        break;

                    case 3:
                        cmd_register_low_read  = GMCH_PMON_GP_CTR3_L + gmch_register_read; // counter 3 low
                        cmd_register_high_read = GMCH_PMON_GP_CTR3_H + gmch_register_read; // counter 3 high
                        break;

                    default:
                        cmd_register_low_read  = GMCH_PMON_GP_CTR0_L + gmch_register_read; // counter 0 low
                        cmd_register_high_read = GMCH_PMON_GP_CTR0_H + gmch_register_read; // counter 0 high
                        break;
                }
                // read in low value as U32
                PCI_Write_Ulong((ULONG)(gmch + GMCH_MSG_CTRL_REG), (ULONG)cmd_register_low_read);
                counter_data_low = PCI_Read_Ulong((ULONG)(gmch + GMCH_MSG_DATA_REG));
                // read in high value as U32 (note: should not exceed 0x3F = 63 !!!)
                PCI_Write_Ulong((ULONG)(gmch + GMCH_MSG_CTRL_REG), (ULONG)cmd_register_high_read);
                counter_data_high = PCI_Read_Ulong((ULONG)(gmch + GMCH_MSG_DATA_REG));
                // and then add them together to get full U64 value
                counter_data = (U64)counter_data_high;
                data[data_index++] = (counter_data << 32) + counter_data_low;
                gp_counter_index++;
            }
        }

        // initialize the counters on the first interrupt
        if (pcb[this_cpu].chipset_count_init == TRUE) {
            for (i = 0; i < CHIPSET_SEGMENT_total_events(gmch_chipset_seg); i++) {
                pcb[this_cpu].last_gmch_count[i] = gmch_data[i];
            }
        }

        gmch_cpu = (CHIPSET_CONFIG_host_proc_run(pma)) ? this_cpu : 0;
        // tbd: assume the GMCH data is valid
        // gmch_data_valid[0] = DATA_IS_VALID;
        for (i = 0; i < CHIPSET_SEGMENT_total_events(gmch_chipset_seg); i++) {
            U32 event_id = CHIPSET_EVENT_event_id(&chipset_events[i]);
            U64 overflow = (event_id == 0) ? GMCH_PMON_FIXED_CTR_OVF_VAL : GMCH_PMON_GP_CTR_OVF_VAL;
            // get the current count
            tmp_data = gmch_data[i];
            // if the current count is bigger than the previous one, then the counter overflowed
            // so make sure the delta gets adjusted to account for it
            if (gmch_data[i] < pcb[gmch_cpu].last_gmch_count[i]) {
                gmch_data[i] = gmch_data[i] + overflow - pcb[gmch_cpu].last_gmch_count[i];
            }
            // otherwise, counter did not overflow, so compute the delta normally
            else {
                gmch_data[i] = gmch_data[i] - pcb[gmch_cpu].last_gmch_count[i];
            }
            // tbd: if the delta exceeds the overflow value, then mark the GMCH data as invalid
            if (gmch_data[i] >= overflow) {
               // gmch_data_valid[0] = DATA_OUT_OF_RANGE;
               gmch_data[i] = 0;
            }
            // save the current count
            pcb[gmch_cpu].last_gmch_count[i] = tmp_data;
        }
    }

    pcb[this_cpu].chipset_count_init = FALSE;

    // release the lock
    lock_gmch_read = 0;

    return;
}

/*
 * @fn        gmch_Stop_Counters()
 *
 * @brief     Stop the GMCH counters
 *
 * @param     None
 *
 * @return    None
 *
 */
static VOID
gmch_Stop_Counters (
    VOID
)
{
    // stop and reset the chipset counters
    if (pma) {
        U32 gmch;
        gmch = FORM_PCI_ADDR(0, 0, 0, 0);
        if (gmch != 0) {
            // disable fixed and GP counters
            PCI_Write_Ulong(gmch + GMCH_MSG_DATA_REG, 0x00000000);
            PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_GLOBAL_CTRL + gmch_register_write);
            // clear fixed counter filter
            PCI_Write_Ulong(gmch + GMCH_MSG_DATA_REG, 0x00000000);
            PCI_Write_Ulong(gmch + GMCH_MSG_CTRL_REG, GMCH_PMON_FIXED_CTR_CTRL + gmch_register_write);
        }
    }
    else {
        SEP_PRINT_ERROR("gmch_Stop_Counters: pma=NULL\n");
    }

    return;
}

/*
 * @fn        gmch_Fini_Chipset()
 *
 * @brief     Reset GMCH to state where it can be used again.  Called at cleanup phase.
 *
 * @param     None
 *
 * @return    None
 *
 */
static VOID
gmch_Fini_Chipset (
    VOID
)
{
    if (! gmch_Check_Enabled()) {
        SEP_PRINT_WARNING("GMCH is not enabled!\n");
    }

    return;
}

//
// Initialize the GMCH chipset dispatch table
//
CS_DISPATCH_NODE gmch_dispatch =
{
    gmch_Init_Chipset,
    gmch_Start_Counters,
    gmch_Read_Counters,
    gmch_Stop_Counters,
    gmch_Fini_Chipset
};
