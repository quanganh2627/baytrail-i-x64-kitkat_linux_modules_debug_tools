/*COPYRIGHT**
 * -------------------------------------------------------------------------
 *               INTEL CORPORATION PROPRIETARY INFORMATION
 *  This software is supplied under the terms of the accompanying license
 *  agreement or nondisclosure agreement with Intel Corporation and may not
 *  be copied or disclosed except in accordance with the terms of that
 *  agreement.
 *        Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 * -------------------------------------------------------------------------
**COPYRIGHT*/


#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "valleyview_sochap.h"
#include "inc/ecb_iterators.h"
#include "inc/pci.h"

extern U32            invoking_processor_id;
static U32            sochap_overflow[VLV_CHAP_MAX_COUNTERS];
static U32            chap_port_id = 0;


/*!
 * @fn          static VOID write_To_Sideband(U32   bus_no,
                                              U32   dev_no,
                                              U32   func_no,
                                              U32   port_id,
                                              U32   op_code,
                                              U64   mmio_offset,
                                              ULONG value)
 *
 * @brief       Reads VISA/CHAP programming info via sideband
 *
 * @param       bus_no      - bus number
 *              dev_no      - device number
 *              func_no     - function number
 *              port_id     - port id
 *              op_code     - operation code
 *              mmio_offset - mmio offset
 *              value       - data to be written to the register
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static void
write_To_Sideband (
    U32   bus_no,
    U32   dev_no,
    U32   func_no,
    U32   port_id,
    U32   op_code,
    U64   mmio_offset,
    ULONG value
)
{
    U32 mmio_offset_hi;
    U32 mmio_offset_lo;
    U32 mcr_value;
    U32 pci_address;

    mmio_offset_hi = (mmio_offset >> 8) & VLV_VISA_OFFSET_HI_MASK;
    mmio_offset_lo = mmio_offset & VLV_VISA_OFFSET_LO_MASK;
    mcr_value      = (op_code << VLV_VISA_OP_CODE_SHIFT) +
                     (port_id << VLV_VISA_PORT_ID_SHIFT) +
                     (mmio_offset_lo << 8) +
                     (VLV_VISA_BYTE_ENABLES << 4);

    pci_address = FORM_PCI_ADDR(bus_no, dev_no, func_no, VLV_VISA_MDR_REG_OFFSET);
    PCI_Write_Ulong((ULONG)pci_address, (ULONG)value);
    pci_address = FORM_PCI_ADDR(bus_no, dev_no, func_no, VLV_VISA_MCRX_REG_OFFSET);
    PCI_Write_Ulong((ULONG)pci_address, (mmio_offset_hi << 8));
    pci_address = FORM_PCI_ADDR(bus_no, dev_no, func_no, VLV_VISA_MCR_REG_OFFSET);
    PCI_Write_Ulong((ULONG)pci_address, mcr_value);

    return;
}

/*!
 * @fn          static ULONG read_From_Sideband(U32 bus_no,
                                                U32 dev_no,
                                                U32 func_no,
                                                U32 port_id,
                                                U32 op_code,
                                                U64 mmio_offset)
 *
 * @brief       Reads VISA/CHAP programming info via sideband
 *
 * @param       bus_no      - bus number
 *              dev_no      - device number
 *              func_no     - function number
 *              port_id     - port id
 *              op_code     - operation code
 *              mmio_offset - mmio offset
 *
 * @return      data from the CHAP register
 *
 * <I>Special Notes:</I>
 */
static void
read_From_Sideband (
    U32  bus_no,
    U32  dev_no,
    U32  func_no,
    U32  port_id,
    U32  op_code,
    U64  mmio_offset,
    U32 *data_val
)
{
    U32   mmio_offset_hi;
    U32   mmio_offset_lo;
    U32   mcr_value;
    U32   pci_address;

    mmio_offset_hi = (mmio_offset >> 8) & VLV_VISA_OFFSET_HI_MASK;
    mmio_offset_lo = mmio_offset & VLV_VISA_OFFSET_LO_MASK;
    mcr_value      = (op_code << VLV_VISA_OP_CODE_SHIFT) +
                     (port_id << VLV_VISA_PORT_ID_SHIFT) +
                     (mmio_offset_lo << 8) +
                     (VLV_VISA_BYTE_ENABLES << 4);

    pci_address = FORM_PCI_ADDR(bus_no, dev_no, func_no, VLV_VISA_MCRX_REG_OFFSET);
    PCI_Write_Ulong((ULONG)pci_address, (mmio_offset_hi << 8));
    pci_address = FORM_PCI_ADDR(bus_no, dev_no, func_no, VLV_VISA_MCR_REG_OFFSET);
    PCI_Write_Ulong((ULONG)pci_address, mcr_value);
    pci_address = FORM_PCI_ADDR(bus_no, dev_no, func_no, VLV_VISA_MDR_REG_OFFSET);
    *data_val    = PCI_Read_Ulong(pci_address);

    return;
}


/*!
 * @fn          static VOID valleyview_VISA_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the entries and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       param - device index
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Write_PMU (
    VOID  *param
)
{
    U32                        dev_idx  = *((U32*)param);
    int                        me       = CONTROL_THIS_CPU();
    ECB                        pecb     = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[0];
    DRV_PCI_DEVICE_ENTRY       dpden;
    U32                        pci_address;
    U32                        bar_lo;
    U64                        bar_hi;
    U64                        final_bar;
    U64                        physical_address;
    U32                        dev_index       = 0;
    S32                        bar_list[VLV_VISA_MAX_PCI_DEVICES];
    U32                        bar_index       = 0;
    U32                        map_size        = 0;
    U64                        virtual_address = 0;
    U32                        bar_name        = 0;
    DRV_PCI_DEVICE_ENTRY       curr_pci_entry  = NULL;
    U32                        next_bar_offset = 0;
    U64                        mmio_offset     = 0;
    U64                        map_base        = 0;
    U32                        i               = 0;

    if (me != invoking_processor_id) {
        return;
    }

    chap_port_id = 0;
    for (dev_index = 0; dev_index < VLV_VISA_MAX_PCI_DEVICES; dev_index++) {
        bar_list[dev_index] = -1;
    }

    // initialize the CHAP per-counter overflow numbers
    for (i = 0; i < VLV_CHAP_MAX_COUNTERS; i++) {
        sochap_overflow[i] = 0;
    }

    ECB_pcidev_entry_list(pecb) = (DRV_PCI_DEVICE_ENTRY)((S8*)pecb + ECB_pcidev_list_offset(pecb));
    dpden = ECB_pcidev_entry_list(pecb);

    for (dev_index = 0; dev_index < ECB_num_pci_devices(pecb); dev_index++) {
        curr_pci_entry = &dpden[dev_index];
        bar_name       = DRV_PCI_DEVICE_ENTRY_bar_name(curr_pci_entry);
        mmio_offset    = DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(curr_pci_entry);

        if (chap_port_id == 0 && DRV_PCI_DEVICE_ENTRY_prog_type(curr_pci_entry) == UNC_CHAP) {
            chap_port_id = DRV_PCI_DEVICE_ENTRY_port_id(curr_pci_entry);
        }
        if (DRV_PCI_DEVICE_ENTRY_config_type(curr_pci_entry) == UNC_PCICFG) {
            if (bar_name == UNC_SIDEBAND &&
                DRV_PCI_DEVICE_ENTRY_operation(curr_pci_entry) == UNC_OP_WRITE) {
                write_To_Sideband(DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
                                  DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
                                  DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
                                  DRV_PCI_DEVICE_ENTRY_port_id(curr_pci_entry),
                                  DRV_PCI_DEVICE_ENTRY_op_code(curr_pci_entry),
                                  DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(curr_pci_entry),
                                  (ULONG)DRV_PCI_DEVICE_ENTRY_value(curr_pci_entry));
            }
            continue;
        }
        // UNC_MMIO programming
        if (bar_list[bar_name] != -1) {
            bar_index                                            = bar_list[bar_name];
            virtual_address                                      = DRV_PCI_DEVICE_ENTRY_virtual_address(&dpden[bar_index]);
            DRV_PCI_DEVICE_ENTRY_virtual_address(curr_pci_entry) = DRV_PCI_DEVICE_ENTRY_virtual_address(&dpden[bar_index]);
            writel(DRV_PCI_DEVICE_ENTRY_value(curr_pci_entry), (U32*)(((char*)(UIOP)virtual_address)+mmio_offset));
            continue;
        }
        pci_address     = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
                                    DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
                                    DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
                                    DRV_PCI_DEVICE_ENTRY_bar_offset(curr_pci_entry));
        bar_lo          = PCI_Read_Ulong(pci_address);
        next_bar_offset = DRV_PCI_DEVICE_ENTRY_bar_offset(curr_pci_entry)+VLV_VISA_NEXT_ADDR_OFFSET;
        pci_address     = FORM_PCI_ADDR(DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
                                    DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
                                    DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
                                    next_bar_offset);
        bar_hi      = PCI_Read_Ulong(pci_address);
        final_bar   = (bar_hi << VLV_VISA_BAR_ADDR_SHIFT) | bar_lo;
        final_bar  &= VLV_VISA_BAR_ADDR_MASK;
        DRV_PCI_DEVICE_ENTRY_bar_address(curr_pci_entry) = final_bar;
        physical_address = DRV_PCI_DEVICE_ENTRY_bar_address(curr_pci_entry);
        if (physical_address) {
            map_size = VLV_VISA_OTHER_BAR_MMIO_PAGE_SIZE;
            map_base = (mmio_offset/map_size)*map_size;
            if (mmio_offset > map_size) {
                physical_address = physical_address + map_base;
            }
            //NOTE: Enable the following code after verifying GTMMADR programming
            /*
            DRV_PCI_DEVICE_ENTRY_virtual_address(curr_pci_entry) = (U64) (UIOP)ioremap_nocache(physical_address, map_size);
            virtual_address                                      = DRV_PCI_DEVICE_ENTRY_virtual_address(curr_pci_entry);
            writel((U32)DRV_PCI_DEVICE_ENTRY_value(curr_pci_entry), (U32*)(((char*)(UIOP)virtual_address) + (mmio_offset - map_base)));
            iounmap((void*)(UIOP)(DRV_PCI_DEVICE_ENTRY_virtual_address(&dpden[dev_index])));
            bar_list[bar_name] = dev_index;
            */
        }
    }

    return;
}



/*!
 * @fn         static VOID valleyview_VISA_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when sampling/driver stops
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Disable_PMU (
    PVOID  param
)
{
    U32                   me        = CONTROL_THIS_CPU();
    U32                   dev_idx   = *((U32*)param);
    ULONG                 data      = 0;

    if (me != invoking_processor_id) {
        return;
    }

    FOR_EACH_PCI_REG_RAW(pecb, i, dev_idx) {
        if (ECB_entries_reg_type(pecb,i) == CCCR) {
            data = (ULONG)VLV_VISA_CHAP_STOP;
        }
        else if (ECB_entries_reg_type(pecb,i) == DATA) {
            data = 0;
        }
        write_To_Sideband(ECB_entries_bus_no(pecb, i),
                          ECB_entries_dev_no(pecb, i),
                          ECB_entries_func_no(pecb, i),
                          chap_port_id,
                          VLV_CHAP_SIDEBAND_WRITE_OP_CODE,
                          ECB_entries_pci_id_offset(pecb,i),
                          data);
    } END_FOR_EACH_PCI_REG_RAW;

    return;
}

/*!
 * @fn         static VOID valleyview_VISA_Initialize(PVOID)
 *
 * @brief      Initialize any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Initialize (
    VOID  *param
)
{
    return;
}


/*!
 * @fn         static VOID valleyview_VISA_Clean_Up(PVOID)
 *
 * @brief      Reset any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID
valleyview_VISA_Clean_Up (
    VOID   *param
)
{
    return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn valleyview_VISA_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The id refers to the device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *
 */
static VOID
valleyview_VISA_Read_Counts (
    PVOID  param,
    U32    id
)
{
    U32                   event_id    = 0;
    U32                   dev_idx     = id;
    U64                  *data;
    int                   data_index;
    U32                   data_val    = 0;
    U32                   data_reg    = 0;
    U64                   total_count = 0;

    data       = (U64*)param;
    data_index = 0;
    // group id
    data[data_index] = 1;
    data_index++;

    FOR_EACH_PCI_REG_RAW(pecb, i, dev_idx) {
        event_id = ECB_entries_event_id_index_local(pecb, i);

        if (ECB_entries_reg_type(pecb,i) == CCCR) {
            write_To_Sideband(ECB_entries_bus_no(pecb, i),
                              ECB_entries_dev_no(pecb, i),
                              ECB_entries_func_no(pecb, i),
                              chap_port_id,
                              VLV_CHAP_SIDEBAND_WRITE_OP_CODE,
                              ECB_entries_pci_id_offset(pecb,i),
                              (ULONG)VLV_VISA_CHAP_SAMPLE_DATA);

            data_reg           = i + ECB_cccr_pop(pecb);
            if (ECB_entries_reg_type(pecb,data_reg) == DATA) {
                read_From_Sideband(ECB_entries_bus_no(pecb, data_reg),
                                   ECB_entries_dev_no(pecb, data_reg),
                                   ECB_entries_func_no(pecb, data_reg),
                                   chap_port_id,
                                   VLV_CHAP_SIDEBAND_READ_OP_CODE,
                                   ECB_entries_pci_id_offset(pecb,data_reg),
                                   &data_val);
                if (data_val < pcb[0].last_visa_count[i]) {
                    sochap_overflow[i]++;
                }
                pcb[0].last_visa_count[i] = data_val;
                total_count = data_val + sochap_overflow[i]*VLV_CHAP_MAX_COUNT;
                data[data_index+event_id] = total_count;
            }
        }

    } END_FOR_EACH_PCI_REG_RAW;

    return;
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE  valleyview_visa_dispatch =
{
    valleyview_VISA_Initialize,        // initialize
    NULL,                              // destroy
    valleyview_VISA_Write_PMU,         // write
    valleyview_VISA_Disable_PMU,       // freeze
    NULL,                              // restart
    NULL,                              // read
    NULL,                              // check for overflow
    NULL,
    NULL,
    valleyview_VISA_Clean_Up,
    NULL,
    NULL,
    NULL,
    valleyview_VISA_Read_Counts,       // read counts
    NULL,
    NULL
};
