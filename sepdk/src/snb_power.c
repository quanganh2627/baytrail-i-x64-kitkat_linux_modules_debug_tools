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
#include "snb_power.h"
#include "ecb_iterators.h"


extern DRV_CONFIG     pcfg;

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
snb_power_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U64            *data;
    int             data_index;
    U32             event_id            = 0;


    data       = (U64 *)param;
    data_index = 0;
    
    // Write GroupID        
    data[data_index] =  1;
    // Increment the data index as the event id starts from zero
    data_index++;

    FOR_EACH_DATA_REG_UNC(pecb,id, i) {
        event_id = ECB_entries_event_id_index_local(pecb,i);
        data[data_index + event_id] = SYS_Read_MSR(ECB_entries_reg_id(pecb,i));
    } END_FOR_EACH_DATA_REG_UNC;
}
/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  snb_power_dispatch =
{
    NULL,                        // initialize
    NULL,                        // destroy
    NULL,                        // write
    NULL,                        // freeze
    NULL,                        // restart
    NULL,                        // reinit
    NULL,                        // read
    NULL,                        // check for overflow
    NULL,                        //swap group
    NULL,                        //read lbrs
    NULL,                        //cleanup
    NULL,                        //hw errata
    NULL,                        //read power
    NULL,                        //check overflow errata
    snb_power_Read_Counts,       //read counts 
    NULL,                        //check overflow gp errata
    NULL                         //platform info
};
