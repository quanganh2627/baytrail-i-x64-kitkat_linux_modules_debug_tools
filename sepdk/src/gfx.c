/*COPYRIGHT**
    Copyright 2009-2011 Intel Corporation.

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

#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "lwpmudrv_defines.h"
#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_gfx.h"
#include "rise_errors.h"
#include "lwpmudrv.h"
#include "gfx.h"

static char* virtual_addr    = NULL;
static U32   gfx_code        = GFX_CTRL_DISABLE;
static U32   gfx_counter[GFX_NUM_COUNTERS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile int in_read = 0;

/*
 * GFX_Read
 *     Parameters
 *         S8  *buffer  - buffer to read the count into
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Reads the counters into the buffer provided for the purpose
 *
 */
extern U32
GFX_Read (
    S8 *buffer
)
{
    char *reg_addr = virtual_addr + GFX_PERF_REG;
    U32  *samp     = (U32 *)buffer;
    //    U64  *samp     = (U64 *)buffer;
    U32   i;
    U32   val;

    SEP_PRINT_DEBUG("Entered Read\n");

    /* GFX counting not specified */
    if (virtual_addr == NULL || gfx_code == GFX_CTRL_DISABLE) {
        SEP_PRINT_DEBUG("GFX_Read: virtual_addr=0x%x, gfx_code=0x%x\n", virtual_addr, gfx_code);
        SEP_PRINT_DEBUG("GFX_Read: skipping ... invalid graphics configuration\n");
        return OS_SUCCESS;

    }

    if (in_read == 1) {
        return OS_SUCCESS;
    }

    in_read = 1;

    /* For all counters - get the value of the counter */
    for (i = 0; i < GFX_NUM_COUNTERS; i++) {
        reg_addr += 4;
        val = *(U32 *)(reg_addr);
        SEP_PRINT_DEBUG("Reg 0x%x has value 0x%x\n", i, (val - gfx_counter[i]));
        if (samp) {
            if (val < gfx_counter[i]) {
                samp[i] = val + ((U32)(-1) - gfx_counter[i]);
            }
            else {
                samp[i] = val - gfx_counter[i];
            }
        }
        // Debug code only.. print the register values on entry and exit
        else {
            SEP_PRINT_DEBUG("Reg 0x%x has value 0x%x\n", i, (val - gfx_counter[i]));
        }
        gfx_counter[i] = val;
    }

    in_read = 0;

    return OS_SUCCESS;
}

/*
 * GFX_Set_Event_Code
 *     Parameters
 *         NONE
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Programs the Graphics PMU with the right event code
 *
 */
extern U32
GFX_Set_Event_Code (
    IOCTL_ARGS arg
)
{
    OS_STATUS  result;
    U32        i;
    char      *reg_addr;
    U32        reg_value;

    result = get_user(gfx_code, (int*)arg->w_buf);
    SEP_PRINT("GFX_Set_Event_Code: gfx_code=0x%x\n", gfx_code);

    /* Read the control word for perf counters */
    if (virtual_addr == NULL) {
        virtual_addr = (char *)(UIOP) ioremap_nocache (GFX_BASE_ADDRESS+GFX_BASE_NEW_OFFSET, PAGE_SIZE);
    }

    for (i =  0; i < GFX_NUM_COUNTERS; i++) {
        gfx_counter[i] = 0;
    }

    reg_addr = virtual_addr + GFX_PERF_REG;
    SEP_PRINT("GFX_Set_Event_Code: reg_addr=0x%p\n", reg_addr);

    reg_value = *(U32 *)(reg_addr);
    SEP_PRINT("GFX_Set_Event_Code: reg_value=0x%x\n", reg_value);

    /* Update the perf counter group */
    /* Write the perf counter group with reset = 1 for all counters*/
    reg_value = (gfx_code | GFX_REG_CTR_CTRL);
    *(U32 *)(reg_addr) = reg_value;

    SEP_PRINT("GFX_Set_Event_Code: write reg_value to reg_addr\n");

    return result;
}

/*
 * GFX_Start
 *     Parameters
 *         NONE
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Starts the count of the Graphics PMU
 *
 */
extern OS_STATUS
GFX_Start (
    void
)
{
    U32   reg_value;
    char *reg_addr;

    SEP_PRINT("GFX_Start: inside\n");

    /* GFX counting not specified */
    if (virtual_addr == NULL || gfx_code == GFX_CTRL_DISABLE) {
        SEP_PRINT("GFX_Start: virtual_addr=0x%p, gfx_code=0x%x\n", virtual_addr, gfx_code);
        SEP_PRINT("GFX_Start: skipping ... invalid graphics configuration\n");
        return OS_SUCCESS;
    }

    reg_addr = virtual_addr + GFX_PERF_REG;
    /* Write the perf counter group with reset = 0 for all counters*/
    /* The counter reset control will all be zero regardless */
    SEP_PRINT_DEBUG("Using GFX code 0x%x\n", gfx_code);
    *(U32 *)(reg_addr) = gfx_code;

    reg_value = *(U32 *)reg_addr;
    if (reg_value != gfx_code) {
        SEP_PRINT_ERROR("GFX Control 0x%x, expected 0x%x\n", reg_value, gfx_code);
    }

    /* Initialize the counter values. */
    GFX_Read(NULL);

    SEP_PRINT("GFX_Start: exiting\n");

    return OS_SUCCESS;
}

/*
 * GFX_Stop
 *     Parameters
 *         NONE
 *     Returns
 *         OS_STATUS
 *
 *     Description
 *         Stops the count of the Graphics PMU
 *
 */
extern OS_STATUS
GFX_Stop (
    void
)
{
    char *reg_addr = (char *)virtual_addr + GFX_PERF_REG;

    SEP_PRINT("GFX_Stop: inside\n");

    /* GFX counting not specified */
    if (virtual_addr == NULL || gfx_code == GFX_CTRL_DISABLE) {
        SEP_PRINT("GFX_Stop: virtual_addr=0x%p, gfx_code=0x%x\n", virtual_addr, gfx_code);
        SEP_PRINT("GFX_Stop: skipping ... invalid graphics configuration\n");
        return OS_SUCCESS;
    }

    GFX_Read(NULL);
    *(U32 *)(reg_addr) = GFX_CTRL_DISABLE;
    gfx_code           = GFX_CTRL_DISABLE;
    virtual_addr       = NULL;

    SEP_PRINT("GFX_Stop: exiting\n");

    return OS_SUCCESS;
}
