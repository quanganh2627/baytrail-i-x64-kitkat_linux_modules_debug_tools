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
 *  cvs_id[] = "$Id$"
 */

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <asm/perfmon.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "poulson.h"
#include "ecb_iterators.h"
#include "cpumon.h"

extern EVENT_CONFIG   global_ec;
extern U64           *read_counter_info;
extern RO             ro;
extern DRV_CONFIG     pcfg;

#if defined(PERFMON_V2_ALT)
    DECLARE_PER_CPU(unsigned long, pfm_syst_info);
#endif

/*
 * poulson_Write_PMU
 *     Parameters
 *         None
 *     Returns
 *         None
 *
 *     Description
 *         Initial write of PMU registers
 *         Walk through the enties and write the value of the register accordingly.
 *         When current_group = 0, then this is the first time this routine is called,
 *         initialize the locks.
 *
 */
static VOID
poulson_Write_PMU (
    VOID  *param
)
{
    U64            val;

    SEP_PRINT_DEBUG("Entered poulson_Write_PMU.\n");

    if (dispatch->hw_errata) {
        dispatch->hw_errata();
    }

    /* Set PMC[0].fr to disable counting. */
#if 0
    /* Per vdk1 sources:
       Mask pmi before setting pmc0.fr=1 to workaround Itanium(R) 2 processor errata.
       Setting pmc0.fr=1 on Itanium(R) 2 processor can cause a pmi.  While pmi is masked via
       the pmv, pmi requests will be ignored (not latched). */
    SYS_Set_PMV_Mask();
#endif
    val = SYS_Read_PMC(IA64_PMC0);
    SYS_Write_PMC(IA64_PMC0, val | (U64)1);
#if defined(MYDEBUG)
//    SEP_PRINT_DEBUG("On CPU%d: Write PMC 0x%x --- value 0x%llx -- read 0x%llx\n",
//                        this_cpu, IA64_PMC0, val | (U64)1, SYS_Read_PMC(IA64_PMC0));
#endif

    FOR_EACH_CCCR_REG(pecb, i) {
        SYS_Write_PMC(ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i));
#if defined(MYDEBUG)
        {
            U64 val      = SYS_Read_PMC(ECB_entries_reg_id(pecb,i));
            U32 this_cpu = CONTROL_THIS_CPU();
            SEP_PRINT_DEBUG("On CPU%d: Write PMC 0x%x --- value 0x%llx -- read 0x%llx\n",
                            this_cpu, ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i), val);
        }
#endif
    } END_FOR_EACH_CCCR_REG;

#if 0
    SYS_Clear_PMV_Mask();
#endif

    FOR_EACH_DATA_REG(pecb, i) {
        SYS_Write_PMD(ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i));
#if defined(MYDEBUG)
        {
            U64 val      = SYS_Read_PMD(ECB_entries_reg_id(pecb,i));
            U32 this_cpu = CONTROL_THIS_CPU();
            SEP_PRINT_DEBUG("On CPU%d: Write PMD 0x%x --- value 0x%llx -- read 0x%llx\n",
                            this_cpu, ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i), val);
        }
#endif
    } END_FOR_EACH_DATA_REG;

    return;
}

/*
 * poulson_Disable_PMU
 *     Parameters
 *         None
 *     Returns
 *         None
 *
 *     Description
 *         Set freeze bit to disable the PMU counters.
 *
 */
static VOID
poulson_Disable_PMU (
    PVOID  param
)
{
#if defined(PERFMON_V2_ALT)
    unsigned long info;
#endif
    U64       val;

    SEP_PRINT_DEBUG("Entered poulson_Disable_PMU.\n");

#if defined(PERFMON_V2_ALT)
    info = 0;

    SYS_Clear_PSR_PP();
    SYS_Clear_PSR_UP();
    SYS_Clear_DCR_PP();

    __get_cpu_var(pfm_syst_info) = info;
#endif

#if 0
    /* Per vdk1 sources:
       Mask pmi before setting pmc0.fr=1 to workaround Itanium(R) 2 processor errata.
       Setting pmc0.fr=1 on Itanium(R) 2 processor can cause a pmi.  While pmi is masked via
       the pmv, pmi requests will be ignored (not latched). */
    SYS_Set_PMV_Mask();
#endif
    val = SYS_Read_PMC(IA64_PMC0);
    SYS_Write_PMC(IA64_PMC0, val | (U64)1);
#if defined(MYDEBUG)
    {
        U64 val2     = SYS_Read_PMC(IA64_PMC0);
        U32 this_cpu = CONTROL_THIS_CPU();
        SEP_PRINT_DEBUG("On CPU%d: Write PMC 0x%x --- value 0x%llx -- read 0x%llx\n",
                        this_cpu, IA64_PMC0, val | (U64)1, val2);
    }
#endif
#if 0
    SYS_Clear_PMV_Mask();
#endif
}

/*
 * poulson_Enable_PMU
 *     Parameters
 *         None
 *     Returns
 *         None
 *
 *     Description
 *         Enable the PMU registers.  Per the software developers manual:
 *
 *             Event collection for a monitor is enabled under the following constraints
 *             on the Poulson processor:
 *                 Monitor Enablei = (not PMC0.fr) and
 *                                   PMCi.plm[PSR.cpl] and
 *                                   PMCi.vm[PSR.vm] and
 *                                   ((not PMCi.pm and PSR.up) or
 *                                   (     PMCi.pm and PSR.pp)) and
 *                                   (not halted or PMCi.ch)
 *
 *         What this code needs to do is:
 *             1) Clear the freeze bit.
 *             2) Clear the overflow bits 4-19.
 *         
 *         PSR.cpl and PSR.up/PSR.pp are external to this routine.  We don't do anything with
 *         them here.  When #1 and #2 intersect with the other conditions, then the monitor is
 *         enabled.
 */
static VOID
poulson_Enable_PMU (
    PVOID   param
)
{
#if defined(PERFMON_V2_ALT)
    unsigned long info;
#endif

    SEP_PRINT_DEBUG("Entered poulson_Enable_PMU.\n");

    if (GLOBAL_STATE_current_phase(driver_state) == DRV_STATE_RUNNING) {
        U64       val;

#if defined(PERFMON_V2_ALT)
        /* Perfmon2 clears the psr's pp/up bits and uses that for control.  This will set those
           bits as we use freeze/unfreeze for control. */
        SYS_Set_PSR_PP();
        SYS_Set_PSR_UP();
        SYS_Set_DCR_PP();

        info = __get_cpu_var(pfm_syst_info);
        info |= PFM_CPUINFO_DCR_PP | PFM_CPUINFO_SYST_WIDE;
         __get_cpu_var(pfm_syst_info) = info;
#endif

        /* Clear the overflow bits 4-19 and the freeze bit (bit 0). */
#if 0
        /* Per vdk1 sources:
           Mask pmi before setting pmc0.fr=1 to workaround Itanium(R) 2 processor errata.
           Setting pmc0.fr=1 on Itanium(R) 2 processor can cause a pmi.  While pmi is masked via
           the pmv, pmi requests will be ignored (not latched). */
        SYS_Set_PMV_Mask();  // Necessary?  We're clearing freeze bit, not setting it.
                             // Yes, system crashes w/o it.
#endif
        val = SYS_Read_PMC(IA64_PMC0);
        SYS_Write_PMC(IA64_PMC0, val & ~((U64)0xffff1));
#if defined(MYDEBUG)
        {
            U64 val2     = SYS_Read_PMC(IA64_PMC0);
            U32 this_cpu = CONTROL_THIS_CPU();
            SEP_PRINT_DEBUG("On CPU%d: Write PMC 0x%x --- value 0x%llx -- read 0x%llx\n",
                            this_cpu, IA64_PMC0, val & ~((U64)0xffff1), val2);
        }
#endif
#if 0
        SYS_Clear_PMV_Mask();
#endif
    }
}

/*
 * poulson_ReInit_Data
 *     Parameters
 *         Dummy
 *     Returns
 *         None
 *
 *     Description
 *         Called by the interrupt handler to reinitialize the counters before
 *         re-enabling the collection
 *
 */
static VOID
poulson_ReInit_Data (
    PVOID param
)
{
    return;
}

/*
 * poulson_Read_PMU_Data
 *     Parameters
 *         None
 *     Returns
 *         None
 *
 *     Description
 *         Read all the PMD registers into a buffer.
 *         Called by the interrupt handler
 *
 */
static VOID
poulson_Read_PMU_Data (
    PVOID   param
)
{
    S32       start_index, j;
    U64      *buffer    = read_counter_info;
    U32       this_cpu  = CONTROL_THIS_CPU();

    SEP_PRINT_DEBUG("Entered poulson_Read_PMU_Data.\n");

    start_index = DRV_CONFIG_num_events(pcfg) * this_cpu;
    SEP_PRINT_DEBUG("On CPU%d: PMU control_data 0x%p, buffer 0x%p\n", this_cpu, PMU_register_data, buffer);
    FOR_EACH_DATA_GENERIC_REG(pecb,i) {
        j = start_index + ECB_entries_event_id_index(pecb,i);
        buffer[j] = SYS_Read_PMD(ECB_entries_reg_id(pecb,i));
        SEP_PRINT_DEBUG("On CPU%d: event_id %d, value 0x%llx\n", this_cpu, ECB_entries_event_id_index(pecb,i), buffer[j]);
    } END_FOR_EACH_DATA_REG;
    return;
}

/*
 * poulson_Check_Overflow
 *     Parameters
 *         INOUT masks
 *     Returns
 *         Event mask of the overflowed registers
 *
 *     Description
 *         Called by the data processing method to figure out which registers have overflowed.
 *
 */
static void
poulson_Check_Overflow (
    DRV_MASKS masks
)
{
    U32              index           = 0;
    U64              overflow_status = 0;
    U32              this_cpu        = CONTROL_THIS_CPU();
    BUFFER_DESC      bd              = &cpu_buf[this_cpu];
    CPU_STATE        pcpu            = &pcb[this_cpu];
    DRV_EVENT_MASK_NODE event_flag;

    // initialize masks 
    DRV_MASKS_masks_num(masks) = 0;

    overflow_status = SYS_Read_PMC(IA64_PMC0);
    SEP_PRINT_DEBUG("Overflow:  cpu: %d, status 0x%llx \n", this_cpu, overflow_status);

    BUFFER_DESC_sample_count(bd) = 0;

    FOR_EACH_DATA_GENERIC_REG(pecb, i) {
        /* Can make this test an assertion. */
        if (IA64_PMC4 <= ECB_entries_reg_id(pecb, i) && ECB_entries_reg_id(pecb, i) <= IA64_PMC19) {
            index = ECB_entries_reg_id(pecb, i);
        } 
        else {
            continue;
        }
        if (overflow_status & ((U64)1 << index)) {
            SEP_PRINT_DEBUG("Overflow:  cpu: %d, index %d\n", this_cpu, index);
            SEP_PRINT_DEBUG("register 0x%x --- val 0x%lx\n",
                            ECB_entries_reg_id(pecb,i),
                            SYS_Read_PMD(ECB_entries_reg_id(pecb,i)));
            SYS_Write_PMD(ECB_entries_reg_id(pecb,i), ECB_entries_reg_value(pecb,i));

            DRV_EVENT_MASK_bitFields1(&event_flag) = (U8) 0;
            if (ECB_entries_dear_value_get(pecb, i)) {
                DRV_EVENT_MASK_dear_capture(&event_flag) = 1;
            }
            if (ECB_entries_iear_value_get(pecb, i)) {
                DRV_EVENT_MASK_iear_capture(&event_flag) = 1;
            }
            if (ECB_entries_btb_value_get(pecb, i)) {
                DRV_EVENT_MASK_btb_capture(&event_flag) = 1;
            }
            if (ECB_entries_ipear_value_get(pecb, i)) {
                DRV_EVENT_MASK_ipear_capture(&event_flag) = 1;
            }

            if (DRV_MASKS_masks_num(masks) < MAX_OVERFLOW_EVENTS) {
                DRV_EVENT_MASK_bitFields1(DRV_MASKS_eventmasks(masks) + DRV_MASKS_masks_num(masks)) = DRV_EVENT_MASK_bitFields1(&event_flag);
                DRV_EVENT_MASK_event_idx(DRV_MASKS_eventmasks(masks) + DRV_MASKS_masks_num(masks)) = ECB_entries_event_id_index(pecb, i);
                DRV_MASKS_masks_num(masks)++;
            }  
            else {
                SEP_PRINT_ERROR("The array for event masks is full.\n");
            }

            SEP_PRINT_DEBUG("Overflow:  cpu: %d, event_flag: %x\n", this_cpu, DRV_EVENT_MASK_bitFields1(&event_flag));
            SEP_PRINT_DEBUG("overflow -- 0x%llx, index 0x%llx\n", overflow_status, (U64)1 << index);
            SEP_PRINT_DEBUG("slot# %d, reg_id 0x%x, index %d\n",
                            i, ECB_entries_reg_id(pecb, i), index);
        }
        if (ECB_entries_event_id_index(pecb, i) == CPU_STATE_trigger_event_num(pcpu)) {
            CPU_STATE_trigger_count(pcpu)--;
        }
    } END_FOR_EACH_DATA_REG;

    /* Clear overflow bits 4-19. */
    SYS_Write_PMC(IA64_PMC0, overflow_status & ~((U64)0xffff0));
    SEP_PRINT_DEBUG("Check Overflow completed %d\n", this_cpu);
}


static VOID
poulson_Initialize (
    VOID  *param
)
{
    SEP_PRINT_DEBUG("Entered poulson_Initialize.\n");
}


static VOID
poulson_Destroy (
    PVOID param
)
{
    SEP_PRINT_DEBUG("Entered poulson_Destroy.\n");
}

static VOID
poulson_Clean_Up (
    VOID   *param
)
{
    FOR_EACH_CCCR_REG(pecb, i) {
        if (ECB_entries_clean_up_get(pecb,i)) {
            SEP_PRINT_DEBUG("clean up set --- RegId --- %x\n", ECB_entries_reg_id(pecb,i));
            SYS_Write_PMC(ECB_entries_reg_id(pecb,i), 0LL);
        }
    } END_FOR_EACH_CCCR_REG;

    FOR_EACH_DATA_REG(pecb, i) {
        if (ECB_entries_clean_up_get(pecb,i)) {
            SEP_PRINT_DEBUG("clean up set --- RegId --- %x\n", ECB_entries_reg_id(pecb,i));
            SYS_Write_PMD(ECB_entries_reg_id(pecb,i), 0LL);
        }
    } END_FOR_EACH_DATA_REG;

    return;
}

/*
 * @FN poulson_Read_ROs(buffer)
 *
 * @param   IN buffer - pointer to the buffer to write the data into
 * @param   IN offset - offset into the buffer at which to start writing data
 * @param   IN size   - amount of data to read/write
 * @return  None
 *
 * @brief   Read all the RO registers into the buffer provided and return
 *
 */
static VOID
poulson_Read_RO (
    VOID   *buffer,
    U32     offset,
    U32     size
)
{
    U32  i;
    U64 *ro_buf    = (U64 *)buffer + offset;
#if defined(MYDEBUG)
    U32  this_cpu  = CONTROL_THIS_CPU();
#endif

    SEP_PRINT_DEBUG("Entered poulson_Read_RO.\n");
    SEP_PRINT_DEBUG("On CPU%d: buffer: %p, offset: %d, size: %d\n", this_cpu, buffer, offset, size);
    for (i = 0; i < size; i++) {
        *ro_buf = SYS_Read_PMD(RO_entries_reg_id(ro, i + offset));
        SEP_PRINT_DEBUG("On CPU%d: reg_id %d, value 0x%llx\n", this_cpu, RO_entries_reg_id(ro, i + offset), *ro_buf);
        ro_buf++;
    }
}


/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  poulson_dispatch =
{
    poulson_Initialize,        // init
    poulson_Destroy,           // fini
    poulson_Write_PMU,         // write
    poulson_Disable_PMU,       // freeze
    poulson_Enable_PMU,        // restart
    poulson_ReInit_Data,       // reinit
    poulson_Read_PMU_Data,     // read_data
    poulson_Check_Overflow,    // check_overflow
    NULL,                      // swap_group
    NULL,                      // read_lbrs
    poulson_Clean_Up,          // clean_up
    NULL,                      // hw_errata
    NULL,                      // read_power
    NULL,                      // check_overflow_errata
    NULL,                      // read_counts
    NULL,                      // check_overflow_gp_errata
    poulson_Read_RO,           // read_ro - used for dear, iear, btb, ipear
    NULL                       // platform_info
};
