/*
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
*/

/*
 *  cvs_id[] = "$Id: core.h $"
 */

#ifndef _CORE_H_
#define _CORE_H_

/*
 * Local to this architecture: Core Solo, Core Duo
 * Arch Perf monitoring version 2
 */
#define IA32_PMC0               0x0C1
#define IA32_PMC1               0x0C2
#define IA32_PERFEVTSEL0        0x186
#define IA32_PERFEVTSEL1        0x187

#define EM_INT_MASK             0x00100000

extern DISPATCH_NODE  core_dispatch;

extern VOID
CORE_Disable_PMU (
    PVOID  param
);

extern VOID
CORE_Enable_PMU (
    PVOID   param
);

extern VOID
CORE_ReInit_Data (
    PVOID   param
);

extern VOID
CORE_Read_PMU_Data (
    U64   *buffer, 
    int    start, 
    int    stop
);

#endif /* _CORE_H_*/
