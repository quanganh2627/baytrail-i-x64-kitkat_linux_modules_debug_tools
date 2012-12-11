/*COPYRIGHT**
    Copyright (C) 2005-2012 Intel Corporation.  All Rights Reserved.

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

// global variable for tracking number of overflows per GMCH counter
static U32 gmch_overflow[MAX_CHIPSET_COUNTERS];

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
                for (j = 0; j < MAX_CHIPSET_COUNTERS; j++) {
                    pcb[i].last_gmch_count[j] = 0;
                }
            }
            // initialize the GMCH per-counter overflow numbers
            for (i = 0; i < MAX_CHIPSET_COUNTERS; i++) {
                gmch_overflow[i] = 0;  // only used if storing raw counts (i.e., GMCH_COMPUTE_DELTAS is undefined)
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
    U64             val;
    U64            *gmch_data;
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

	// Write GroupID
        data[data_index] = 1;
        // Increment the data index as the event id starts from zero
        data_index++;
        
	// place in sample record where GMCH data will be written as gmch_data[0], gmch_data[1], ...
        gmch_data = data + data_index;

        // decide whether current CPU or default CPU 0 will handle the GMCH read
        gmch_cpu = (CHIPSET_CONFIG_host_proc_run(pma)) ? this_cpu : 0;

        // first cpu to grab the lock gets to read the GMCH counters
        if (cmpxchg(&lock_gmch_read,   // variable to update
                    0,                 // old value to compare
                    1)) {              // new value to use
#if !defined(GMCH_COMPUTE_DELTAS)
            for (i = 0; i < CHIPSET_SEGMENT_total_events(gmch_chipset_seg); i++) {
                // save the previous count as the current count (i.e., unchanged)
                gmch_data[i] = pcb[gmch_cpu].last_gmch_count[i];
            }
#endif
            return;  // failed to grab the lock, so return
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
                gmch_overflow[i] = 0;
            }
        }

        // for all counters - save the count data to the sampling stream
        for (i = 0; i < CHIPSET_SEGMENT_total_events(gmch_chipset_seg); i++) {
            U32 event_id = CHIPSET_EVENT_event_id(&chipset_events[i]);
            U64 overflow = (event_id == 0) ? GMCH_PMON_FIXED_CTR_OVF_VAL : GMCH_PMON_GP_CTR_OVF_VAL;
            // discard higher-order bits (which may contain garbage), only keep relevant lower-order bits
            gmch_data[i] &= overflow;
#if defined(GMCH_COMPUTE_DELTAS)
            // get the current count
            val = gmch_data[i];    // in this context, val represents the non-running count
            // if the current count is bigger than the previous one, then the counter overflowed
            // so make sure the delta gets adjusted to account for it
            if (gmch_data[i] < pcb[gmch_cpu].last_gmch_count[i]) {
                gmch_data[i] = gmch_data[i] + overflow - pcb[gmch_cpu].last_gmch_count[i];
            }
            // otherwise, counter did not overflow, so compute the delta normally
            else {
                gmch_data[i] = gmch_data[i] - pcb[gmch_cpu].last_gmch_count[i];
            }
#else       // just keep track of raw count for this counter
            // if the current count is bigger than the previous one, then the counter overflowed
            if (gmch_data[i] < pcb[gmch_cpu].last_gmch_count[i]) {
                gmch_overflow[i]++;
            }
            gmch_data[i] = gmch_data[i] + gmch_overflow[i]*overflow;
            val = gmch_data[i];  // in this context, val represents the *running* count!
#endif
            // save the current count
            pcb[gmch_cpu].last_gmch_count[i] = val;
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
