/* ***********************************************************************************************

   This file is provided under a dual BSD/GPLv2 license.  When using or
   redistributing this file, you may do so under either license.

   GPL LICENSE SUMMARY

   Copyright(c) 2011 Intel Corporation. All rights reserved.

   This program is free software; you can redistribute it and/or modify
   it under the terms of version 2 of the GNU General Public License as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
   The full GNU General Public License is included in this distribution
   in the file called LICENSE.GPL.

   Contact Information:
   Gautam Upadhyaya <gautam.upadhyaya@intel.com>
   1906 Fox Drive, Champaign, IL - 61820, USA

   BSD LICENSE

   Copyright(c) 2011 Intel Corporation. All rights reserved.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
   * Neither the name of Intel Corporation nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   ***********************************************************************************************
   */

/**
 * apwr_driver.c: Prototype kernel module to trace the following
 * events that are relevant to power:
 *	- entry into a C-state
 *	- change of processor frequency
 *	- interrupts and timers
 */

#define MOD_AUTHOR "Gautam Upadhyaya <gautam.upadhyaya@intel.com>"
#define MOD_DESC "Power driver for Piersol power tool. Adapted from Romain Cledat's codebase."

/*
 * Current driver version is 1.0.1
 * Current driver version is 1.0.2
 * Current driver version is 1.0.3
 * Current driver version is 1.0.4
 * Current driver version is 2.0.0
 * Current driver version is 2.1.1
 * Current driver version is 2.2.0
 * Current driver version is 2.2.1
 * Current driver version is 2.2.2
 */
#define PW_VERSION_VERSION 2
#define PW_VERSION_INTERFACE 2
#define PW_VERSION_OTHER 4

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/smp.h> // For smp_call_function

#include <asm/local.h>
#include <asm/cputime.h> // For ktime
#include <asm/io.h> // For ioremap, read, and write

#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/events/sched.h>
#include <trace/events/syscalls.h>
#include <trace/events/workqueue.h>
#if DO_WAKELOCK_SAMPLE
#include <trace/events/wakelock.h> // Works for the custom kernel enabling wakelock tracepoint event
#endif

#include <linux/hardirq.h> // for "in_interrupt"
#include <linux/interrupt.h> // for "TIMER_SOFTIRQ, HRTIMER_SOFTIRQ"

#include <linux/kallsyms.h>
#include <linux/stacktrace.h>
#include <linux/hash.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/cpufreq.h>
#include <linux/version.h> // for "LINUX_VERSION_CODE"
#include <asm/unistd.h> // for "__NR_execve"
#include <asm/delay.h> // for "udelay"

#include "pw_lock_defs.h"
#include "pw_mem.h" // internally includes "pw_lock_defs.h"
#include "pw_data_structs.h"

/**** CONFIGURATION ****/

#define APWR_VERSION_CODE LINUX_VERSION_CODE

static int PW_max_num_cpus = -1;

static __read_mostly bool PW_is_atm = false;

/* Controls the amount of printks that happen. Levels are:
 *	- 0: no output save for errors and status at end
 *	- 1: single line for each hit (tracepoint, idle notifier...)
 *	- 2: more details
 *	- 3: user stack and kernel stack info
 */
static unsigned int verbosity = 0;

module_param(verbosity, uint, 0);
MODULE_PARM_DESC(verbosity, "Verbosity of output. From 0 to 3 with 3 the most verbose [default=0]");

/*
 * For measuring collection times.
 */
static unsigned long startJIFF, stopJIFF;

#define SUCCESS 0
#define ERROR 1

/*
 * Compile-time flags -- these affect
 * which parts of the driver get
 * compiled in.
 */

/*
 * Do we allow blocking reads?
 */
#define ALLOW_BLOCKING_READ 1
/*
 * Control whether the 'OUTPUT' macro is enabled.
 * Set to: "1" ==> 'OUTPUT' is enabled.
 *         "0" ==> 'OUTPUT' is disabled.
 */
#define DO_DEBUG_OUTPUT 0
/*
 * Do we read the TSC MSR directly to determine
 * TSC (as opposed to using a kernel
 * function call -- e.g. rdtscll)?
 */
#define READ_MSR_FOR_TSC 1
/*
 * Do we support stats collection
 * for the 'PW_IOCTL_STATUS' ioctl?
 */
#define DO_IOCTL_STATS 0
/*
 * Do we check if the special 'B0' MFLD
 * microcode patch has been installed?
 * '1' ==> YES, perform the check.
 * '0' ==> NO, do NOT perform the check.
 */
#define DO_CHECK_BO_MICROCODE_PATCH 1
/*
 * Do we conduct overhead measurements?
 * '1' == > YES, conduct measurements.
 * '0' ==> NO, do NOT conduct measurements.
 */
#define DO_OVERHEAD_MEASUREMENTS 1
/*
 * Do we keep track of IRQ # <--> DEV name mappings?
 * '1' ==> YES, cache mappings.
 * '0' ==> NO, do NOT cache mappings.
 */
#define DO_CACHE_IRQ_DEV_NAME_MAPPINGS 1
/*
 * Do we allow multiple device (names) to
 * map to the same IRQ number? Setting
 * to true makes the driver slower, if
 * more accurate.
 * '1' ==> YES, allow multi-device IRQs
 * '0' ==> NO, do NOT allow.
 */
#define DO_ALLOW_MULTI_DEV_IRQ 0
/*
 * Do we dynamically determine
 * processor frequency (by using
 * the APERF/MPERF ratio and comparing
 * the ratio to a list of available
 * frequencies)? Used in the 'power_frequency'
 * tracepoint.
 * '1' ==> YES
 * '0' ==> NO
 */
#define DO_DYNAMIC_FREQUENCY_MEASUREMENT !DO_LEGACY_P_STATE_MEASUREMENT
/*
 * Do we use APERF, MPERF for
 * dynamic freq calculations?
 * '1' ==> YES, use APERF, MPERF
 * '0' ==> NO, use IA32_FIXED_CTR{1,2}
 */
#define USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY 0
/*
 * Do we use the cpufreq notifier
 * for p-state transitions?
 * Useful on MFLD, where the default
 * TPF seems to be broken.
 */
#define DO_CPUFREQ_NOTIFIER 1
/*
 * Collect S state residency counters
 * The S state residency counters are only available
 * for MFLD now.
 */
// #define DO_S_RESIDENCY_SAMPLE 1
#define DO_S_RESIDENCY_SAMPLE 1
/*
 * Collect S states
 */
#define DO_S_STATE_SAMPLE 0
/*
 * Collect D state residency counters in south complex
 * The D state residency counters are only available
 * for MFLD now.
 */
// #define DO_D_SC_RESIDENCY_SAMPLE 1
#define DO_D_SC_RESIDENCY_SAMPLE 1
/*
 * Collect D states in north complex
 */
// #define DO_D_NC_STATE_SAMPLE 1
#define DO_D_NC_STATE_SAMPLE 1
/*
 * Collect D states in south complex
 */
// #define DO_D_SC_STATE_SAMPLE 1
#define DO_D_SC_STATE_SAMPLE 1
/*
 * Use the predefined SCU IPC functions to access to SCU
 * The intel_scu_ipc driver uses mutex lock which makes the system hanging
 * when used in sched_switch. So, disable it for now.
 */
#define USE_PREDEFINED_SCU_IPC 0

#define DO_GENERATE_CURRENT_FREQ_IN_PARALLEL 0

/*
 * Compile-time constants and
 * other macros.
 */

#define NUM_MAP_BUCKETS_BITS 9
#define NUM_MAP_BUCKETS (1UL << NUM_MAP_BUCKETS_BITS)

// 32 locks for the hash table
#define HASH_LOCK_BITS 5
#define NUM_HASH_LOCKS (1UL << HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

#define HASH_LOCK(i) LOCK(hash_locks[(i) & HASH_LOCK_MASK])
#define HASH_UNLOCK(i) UNLOCK(hash_locks[(i) & HASH_LOCK_MASK])

#define NUM_TIMER_NODES_PER_BLOCK 20

#define TIMER_HASH_FUNC(a) hash_ptr((void *)a, NUM_MAP_BUCKETS_BITS)

/* Macro for printk based on verbosity */
#if DO_DEBUG_OUTPUT
#define OUTPUT(level, ...) do { if(unlikely(level <= verbosity)) printk(__VA_ARGS__); } while(0);
#else
#define OUTPUT(level, ...)
#endif // DO_DEBUG_OUTPUT

#define CPU() (smp_processor_id())
#define TID() (current->pid)
#define PID() (current->tgid)
#define NAME() (current->comm)
#define PKG(c) ( cpu_data(c).phys_proc_id )
#define IT_REAL_INCR() (current->signal->it_real_incr.tv64)

#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )

#define BEGIN_IRQ_STATS_READ(p, c) do{		\
    p = &per_cpu(irq_stat, (c));

#define END_IRQ_STATS_READ(p, c)		\
    }while(0)

#define BEGIN_LOCAL_IRQ_STATS_READ(p) do{	\
    p = &__get_cpu_var(irq_stat);

#define END_LOCAL_IRQ_STATS_READ(p)		\
    }while(0)

/*
 * Circularly increment 'i' MODULO 'l'.
 * ONLY WORKS IF 'l' is (power of 2 - 1) ie.
 * l == (2 ^ x) - 1
 */
#define CIRCULAR_INC(i,l) ({int __tmp1 = (++(i)) & (l); __tmp1;})
/*
 * Circularly decrement 'i'.
 */
#define CIRCULAR_DEC(i,m) ({int __tmp1 = (i); if(--__tmp1 < 0) __tmp1 = (m); __tmp1;})

/*
 * For now, we limit kernel-space backtraces to 20 entries.
 * This decision will be re-evaluated in the future.
 */
// #define MAX_BACKTRACE_LENGTH 20
#define MAX_BACKTRACE_LENGTH TRACE_LEN

/*
 * Record the TSC when we hit a certain point (only the first time).
 * 's' is a pointer to an instance of per_cpu_t.
 */
#define record_hit_full(s,pid,tid,which,what) do{	\
	if(unlikely( (s)->debug_enters == 0)){		\
	    tscval(&((s)->debug_enters));		\
	    (s)->last_pid = pid;			\
	    (s)->last_tid = tid;			\
	    (s)->last_break[(which)] = what;		\
	}						\
    }while(0)

/*
 * For NHM etc.: Base operating frequency
 * ratio is encoded in 'PLATFORM_INFO' MSR.
 */
#define PLATFORM_INFO_MSR_ADDR 0xCE
/*
 * For MFLD -- base operating frequency
 * ratio is encoded in 'CLOCK_CR_GEYSIII_STAT'
 * MSR (internal communication with Peggy Irelan)
 */
#define CLOCK_CR_GEYSIII_STAT_MSR_ADDR 0x198 // '408 decimal'
/*
 * Standard Bus frequency. Valid for
 * NHM/WMR.
 * TODO: frequency for MFLD?
 */
#define BUS_CLOCK_FREQ_KHZ_NHM 133000 /* For NHM/WMR. SNB has 100000 */
#define BUS_CLOCK_FREQ_KHZ_MFLD 100000 /* For MFLD. SNB has 100000 */
/*
 * Try and determine the bus frequency.
 * Used ONLY if the user-program passed
 * us an invalid clock frequency.
 */
#define DEFAULT_BUS_CLOCK_FREQ_KHZ() ({u32 __tmp = (PW_is_atm) ? BUS_CLOCK_FREQ_KHZ_MFLD : BUS_CLOCK_FREQ_KHZ_NHM; __tmp;})
/*
 * MSRs required to enable CPU_CLK_UNHALTED.REF
 * counting.
 */
#define IA32_PERF_GLOBAL_CTRL_ADDR 0x38F
#define IA32_FIXED_CTR_CTL_ADDR 0x38D
/*
 * Standard APERF/MPERF addresses.
 * Required for dynamic freq
 * measurement.
 */
#define MPERF_MSR_ADDR 0xe7
#define APERF_MSR_ADDR 0xe8
/*
 * Fixed counter addresses.
 * Required for dynamic freq
 * measurement.
 */
#define IA32_FIXED_CTR1_ADDR 0x30A
#define IA32_FIXED_CTR2_ADDR 0x30B
/*
 * Toggle between APERF,MPERF and
 * IA32_FIXED_CTR{1,2} for Turbo.
 */
#if USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY
#define CORE_CYCLES_MSR_ADDR APERF_MSR_ADDR
#define REF_CYCLES_MSR_ADDR MPERF_MSR_ADDR
#else // !USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY
#define CORE_CYCLES_MSR_ADDR IA32_FIXED_CTR1_ADDR
#define REF_CYCLES_MSR_ADDR IA32_FIXED_CTR2_ADDR
#endif

#if DO_RCU_OUTPUT_BUFFERS

#define UNPROTECTED_GET_OUTPUT_LIST(cpu) &current_output_set->lists[(cpu)]
/*
 * Aliases for the rcu read lock/unlock calls. These
 * allow a better understanding of what's going on.
 */
#define BEGIN_PRODUCING() rcu_read_lock()
#define STOP_PRODUCING() rcu_read_unlock()

/*
 * Warning: MUST BE RCU-READ-PROTECTED!!!
 */
#define GET_OUTPUT_LIST(cpu) ({output_set_t *curr_set = NULL;	\
	    list_t *list = NULL;				\
	    curr_set = rcu_dereference(current_output_set);	\
	    list = &curr_set->lists[(cpu)];			\
	    list;})

#else // DO_RCU_OUTPUT_BUFFERS

#define UNPROTECTED_GET_OUTPUT_LIST(cpu) lists[(cpu)]
#define BEGIN_PRODUCING() /* NOP */
#define STOP_PRODUCING()  /* NOP */

#define GET_OUTPUT_LIST(cpu) UNPROTECTED_GET_OUTPUT_LIST(cpu)

#endif // DO_RCU_OUTPUT_BUFFERS
/*
 * Size of each 'bucket' for a 'cpu_bitmap'
 */
#define NUM_BITS_PER_BUCKET (sizeof(unsigned long) * 8)
/*
 * Num 'buckets' for each 'cpu_bitmap' in the
 * 'irq_node' struct.
 */
#define NUM_BITMAP_BUCKETS ( (PW_max_num_cpus / NUM_BITS_PER_BUCKET) + 1 )
/*
 * 'cpu_bitmap' manipulation macros.
 */
#define IS_BIT_SET(bit,map) ( test_bit( (bit), (map) ) != 0 )
#define SET_BIT(bit,map) ( test_and_set_bit( (bit), (map) ) )
/*
 * Timer stats accessor macros.
 */
#ifdef CONFIG_TIMER_STATS
	#define TIMER_START_PID(t) ( (t)->start_pid )
	#define TIMER_START_COMM(t) ( (t)->start_comm )
#else
	#define TIMER_START_PID(t) (-1)
	#define TIMER_START_COMM(t) ( "UNKNOWN" )
#endif
/*
 * Helper macro to return time in usecs.
 */
#define CURRENT_TIME_IN_USEC() ({struct timeval tv; \
		do_gettimeofday(&tv);		\
		(unsigned long long)tv.tv_sec*1000000ULL + (unsigned long long)tv.tv_usec;})

/*
 * IPC communication definitions for coulomb counter,
 * S and D state residencies
 */
// IPC register offsets
#define IPC_BASE_ADDRESS                0xFF11C000
#define IPC_CMD_OFFSET                  0x00000000
#define IPC_STS_OFFSET                  0x00000004
#define IPC_SPTR_OFFSET                 0x00000008
#define IPC_DPTR_OFFSET                 0x0000000C
#define IPC_WBUF_OFFSET                 0x00000080
#define IPC_RBUF_OFFSET                 0x00000090
#define IPC_MAX_ADDR                    0x100

// Write 3bytes in IPC_WBUF (2bytes for address and 1byte for value)
#define IPC_ADC_WRITE_1                 0x000300FF
// Write 2bytes in IPC_WBUF (2bytes for address) and read 1byte from IPC_RBUF
#define IPC_ADC_READ_1                  0x000210FF

// IPC commands
#define IPC_MESSAGE_MSIC                0xFF
#define IPC_MESSAGE_CC                  0xEF
#define IPC_MESSAGE_D_RESIDENCY         0xEA
#define IPC_MESSAGE_S_RESIDENCY         0xEB

// IPC subcommands
#define IPC_COMMAND_WRITE               0x0
#define IPC_COMMAND_READ                0x1
#define IPC_COMMAND_START_RESIDENCY     0x0
#define IPC_COMMAND_STOP_RESIDENCY      0x1
#define IPC_COMMAND_DUMP_RESIDENCY      0x2

// IPC commands for S state residency counter
#define S_RESIDENCY_BASE_ADDRESS        0xFFFF71E0
#define S_RESIDENCY_MAX_COUNTERS        0x4
// IPC commands for D state residency counter
#define D_RESIDENCY_BASE_ADDRESS        0xFFFF7000
#define D_RESIDENCY_MAX_COUNTERS        0x78    // 40 LSS * 3 D states = 120

// PCI communication
#define MTX_ENABLE_PCI                      0x80000000
#define MTX_PCI_MSG_CTRL_REG                0x000000D0
#define MTX_PCI_MSG_DATA_REG                0x000000D4
#define MTX_PCI_ADDR(Bus, Device, Function, Offset) ((0x01 << 31) |(((Bus) &0xFF) <<     16)|   \ (((Device)&0x1F) << 11)|(((Function)&0x07) << 8)| (((Offset)&(~0x3)) << 0))
#define MTX_PCI_ADDR_IO                    0x00000CF8
#define MTX_PCI_DATA_IO                    0x00000CFC

// Power management base address for read/control subsystem status
#define PM_BASE_ADDRESS                    0xFF11D000
#define PM_MAX_ADDR                        0x3F

#if DO_S_STATE_SAMPLE || DO_D_SC_STATE_SAMPLE
static void *mmio_pm_base = NULL;             // memory mapped io base address for subsystem power management
#endif
#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
static void *mmio_ipc1_base = NULL;             // memory mapped io base address for IPC-1
#endif
#if DO_S_RESIDENCY_SAMPLE
static void *mmio_s_residency_base = NULL;      // memory mapped io base address for S0ix Residency counters
#endif
#if DO_D_SC_RESIDENCY_SAMPLE
static void *mmio_d_residency_base = NULL;      // memory mapped io base address for D0ix Residency counters
#endif

// Lock used to prevent multiple call to SCU
#if !USE_PREDEFINED_SCU_IPC
static DEFINE_SPINLOCK(ipclock);
#endif

// Required to calculate S0i0 residency counter from non-zero S state counters
#if DO_S_RESIDENCY_SAMPLE
static u64 startJIFF_s_residency;
#endif
#if DO_S_RESIDENCY_SAMPLE
static u64 startJIFF_d_sc_residency;
#endif

// Keep track of jiffies when previously D-state sampled
static unsigned long long prev_sample_usec = 0;

// Hold bits for interesting device residency counters
// The first LSB indicates that the residency counter for LSS0 is collected.
// By default, 40 LSSs are available in MFD now.
static u64 d_sc_mask = 0xFFFFFFFFFFULL;
// Number of devices collecting residency counters
static u32 d_sc_count_num = 40;

// Non-zero value indicates a new sample needs to be generated.
//static atomic_t new_sample_flag = ATOMIC_INIT(0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)    smp_call_function((func),(ctx),(wait))
#else
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)    smp_call_function((func),(ctx),(retry),(wait))
#endif

/*
 * Data structure definitions.
 */

#if DO_RCU_OUTPUT_BUFFERS
typedef struct output_set output_set_t;
struct output_set{
    list_t *lists;
};
#endif

typedef struct tnode tnode_t;
struct tnode{
    struct hlist_node list;
    unsigned long timer_addr;
    pid_t tid, pid;
    u64 tsc;
    u16 trace_sent : 1;
    u16 trace_len : 15;
    unsigned long *trace;
};

typedef struct hnode hnode_t;
struct hnode{
    struct hlist_head head;
};

typedef struct tblock tblock_t;
struct tblock{
    struct tnode *data;
    tblock_t *next;
};

typedef struct per_cpu_mem per_cpu_mem_t;
struct per_cpu_mem{
    tblock_t *block_list;
    hnode_t free_list_head;
};

#define GET_MEM_VARS(cpu) &per_cpu(per_cpu_mem_vars, (cpu))
#define GET_MY_MEM_VARS(cpu) &__get_cpu_var(per_cpu_mem_vars)

/*
 * For IRQ # <--> DEV NAME mappings.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

typedef struct irq_node irq_node_t;
struct irq_node{
    struct hlist_node list;
    struct rcu_head rcu;
    int irq;
    char *name;
    /*
     * We send IRQ # <-> DEV name
     * mappings to Ring-3 ONCE PER
     * CPU. We need a bitmap to let
     * us know which cpus have
     * already had this info sent.
     *
     * FOR NOW, WE ASSUME A MAX OF 64 CPUS!
     * (This assumption is enforced in
     * 'init_data_structures()')
     */
    // u64 cpu_bitmap;
    unsigned long *cpu_bitmap;
};
#define PWR_CPU_BITMAP(node) ( (node)->cpu_bitmap )

typedef struct irq_hash_node irq_hash_node_t;
struct irq_hash_node{
    struct hlist_head head;
};
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS


#define NUM_IRQ_MAP_BITS 6
#define NUM_IRQ_MAP_BUCKETS (1UL << NUM_IRQ_MAP_BITS)
#define IRQ_MAP_HASH_MASK (NUM_IRQ_MAP_BITS - 1)
// #define IRQ_MAP_HASH_FUNC(num) (num & IRQ_MAP_HASH_MASK)
#define IRQ_MAP_HASH_FUNC(a) hash_long((u32)a, NUM_IRQ_MAP_BITS)

#define IRQ_LOCK_MASK HASH_LOCK_MASK

#define IRQ_LOCK(i) LOCK(irq_map_locks[(i) & IRQ_LOCK_MASK])
#define IRQ_UNLOCK(i) UNLOCK(irq_map_locks[(i) & IRQ_LOCK_MASK])


/*
 * For dynamic frequency determination.
 */
typedef struct{
    u64 aperf, mperf;
    u32 prev_req_freq;
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
    u32 prev_act_freq;
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
}per_cpu_freq_data_t;

/*
 * Variable declarations.
 */

/*
 * A list of MSR names.
 * Useful if debugging.
 *
 * UPDATE: REQUIRED by 'read_one_residency(...)'
 */
static const char *msr_names[] = {"C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"};

/*
 * Names for SOFTIRQs.
 * These are taken from "include/linux/interrupt.h"
 */
static const char *pw_softirq_to_name[] = {"HI_SOFTIRQ", "TIMER_SOFTIRQ", "NET_TX_SOFTIRQ", "NET_RX_SOFTIRQ", "BLOCK_SOFTIRQ", "BLOCK_IOPOLL_SOFTIRQ", "TASKLET_SOFTIRQ", "SCHED_SOFTIRQ", "HRTIMER_SOFTIRQ", "RCU_SOFTIRQ"};

/*
 * For microcode PATCH version.
 * ONLY useful for MFLD!
 */
static u32 __read_mostly micro_patch_ver = 0x0;

/*
 * Is the device open right now? Used to prevent
 * concurent access into the same device.
 */
#define DEV_IS_OPEN 0 // see if device is in use
static volatile unsigned long dev_status;

static struct hnode timer_map[NUM_MAP_BUCKETS];

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
static PWCollector_irq_mapping_t *irq_mappings_list = NULL;
static irq_hash_node_t irq_map[NUM_IRQ_MAP_BUCKETS];
static int total_num_irq_mappings = 0;
#endif //  DO_CACHE_IRQ_DEV_NAME_MAPPINGS


#if DO_RCU_OUTPUT_BUFFERS
static atomic_t apwr_device_read_hrtimer_fired = ATOMIC_INIT(0);
static struct hrtimer apwr_device_read_hrtimer;
static output_set_t *current_output_set = NULL;
static int curr_output_set_index = -1;
static output_set_t *output_sets[2] = {NULL, NULL};
#else
static list_t **lists = NULL;
#endif

DEFINE_PER_CPU(per_cpu_t, per_cpu_counts);

DEFINE_PER_CPU(stats_t, per_cpu_stats);

DEFINE_PER_CPU(CTRL_values_t, CTRL_data_values);

static DEFINE_PER_CPU(per_cpu_mem_t, per_cpu_mem_vars);

/*
 * Macro to add newly allocated timer
 * nodes to individual free lists.
 */
#define LINK_FREE_TNODE_ENTRIES(nodes, size, free_head) do{		\
	int i=0;							\
	for(i=0; i<(size); ++i){					\
	    tnode_t *__node = &((nodes)[i]);				\
	    hlist_add_head(&__node->list, &((free_head)->head));	\
	}								\
    }while(0)


#if ALLOW_BLOCKING_READ
wait_queue_head_t read_queue;
#endif


/*
 * Hash locks.
 */
static spinlock_t hash_locks[NUM_HASH_LOCKS];
/*
 * IRQ Map locks.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
static spinlock_t irq_map_locks[NUM_HASH_LOCKS];
#endif

/*
 * Base operating frequency -- required if
 * checking turbo frequencies.
 */
static __read_mostly u32 base_operating_freq = 0x0;
/*
 * A table of available frequencies -- basically
 * the same as what's displayed in the
 * 'scaling_available_frequencies' sysfs file.
 */
static __read_mostly u32 *apwr_available_frequencies = NULL;
/*
 * Store prev APERF/MPERF values -- required
 * for dynamic freq measurements.
 */
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
static DEFINE_PER_CPU(per_cpu_freq_data_t, pcpu_freq_data) = {0, 0, 0};
#else // DO_DYNAMIC_FREQUENCY_MEASUREMENT
static DEFINE_PER_CPU(per_cpu_freq_data_t, pcpu_freq_data) = {0, 0};
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
/*
 * Character device file MAJOR
 * number -- we're now obtaining
 * this dynamically.
 */
static int apwr_dev_major_num = -1;

#if DO_OVERHEAD_MEASUREMENTS
/*
 * Counter to count # of entries
 * in the timer hash map -- used
 * for debugging.
 */
static atomic_t num_timer_entries = ATOMIC_INIT(0);
#endif

/*
 * These are used for the 'hrtimer_start(...)'
 * hack.
 */
static u32 tick_count = 0;
static DEFINE_SPINLOCK(tick_count_lock);
static bool should_probe_on_hrtimer_start = true;

DEFINE_PER_CPU(local_t, sched_timer_found) = LOCAL_INIT(0);

					   /*
					    * MACRO helpers to measure function call
					    * times.
					    */
#if DO_OVERHEAD_MEASUREMENTS

#include "pw_overhead_measurements.h"

					   /*
					    * For each function that you want to profile,
					    * do the following (e.g. function 'foo'):
					    * **************************************************
					    * DECLARE_OVERHEAD_VARS(foo);
					    * **************************************************
					    * This will declare the two variables required
					    * to keep track of overheads incurred in
					    * calling/servicing 'foo'. Note that the name
					    * that you declare here *MUST* match the function name!
					    */

					   DECLARE_OVERHEAD_VARS(timer_init); // for the "timer_init" family of probes
					   DECLARE_OVERHEAD_VARS(timer_expire); // for the "timer_expire" family of probes
					   DECLARE_OVERHEAD_VARS(tps); // for TPS
					   DECLARE_OVERHEAD_VARS(tpf); // for TPF
					   DECLARE_OVERHEAD_VARS(timer_insert); // for "timer_insert"
					   DECLARE_OVERHEAD_VARS(timer_delete); // for "timer_delete"
					   DECLARE_OVERHEAD_VARS(exit_helper); // for "exit_helper"
					   DECLARE_OVERHEAD_VARS(map_find_unlocked_i); // for "map_find_i"
					   DECLARE_OVERHEAD_VARS(get_next_free_node_i); // for "get_next_free_node_i"
					   DECLARE_OVERHEAD_VARS(ti_helper); // for "ti_helper"
					   DECLARE_OVERHEAD_VARS(inter_common); // for "inter_common"
					   DECLARE_OVERHEAD_VARS(irq_insert); // for "irq_insert"
					   DECLARE_OVERHEAD_VARS(find_irq_node_i); // for "find_irq_node_i"

					   /*
					    * Macros to measure overheads
					    */
#define DO_PER_CPU_OVERHEAD_FUNC(func, ...) do{		\
	u64 *__v = &__get_cpu_var(func##_elapsed_time);	\
	u64 tmp_1 = 0, tmp_2 = 0;			\
	local_inc(&__get_cpu_var(func##_num_iters));	\
	tscval(&tmp_1);					\
	{						\
	    func(__VA_ARGS__);				\
	}						\
	tscval(&tmp_2);					\
	*(__v) += (tmp_2 - tmp_1);			\
    }while(0)

#define DO_PER_CPU_OVERHEAD_FUNC_RET(ret, func, ...) do{	\
	u64 *__v = &__get_cpu_var(func##_elapsed_time);		\
	u64 tmp_1 = 0, tmp_2 = 0;				\
	local_inc(&__get_cpu_var(func##_num_iters));		\
	tscval(&tmp_1);						\
	{							\
	    ret = func(__VA_ARGS__);				\
	}							\
	tscval(&tmp_2);						\
	*(__v) += (tmp_2 - tmp_1);				\
    }while(0)


#else // DO_OVERHEAD_MEASUREMENTS

#define DO_PER_CPU_OVERHEAD(v, func, ...) func(__VA_ARGS__)
#define DO_PER_CPU_OVERHEAD_FUNC(func, ...) func(__VA_ARGS__)
#define DO_PER_CPU_OVERHEAD_FUNC_RET(ret, func, ...) ret = func(__VA_ARGS__)

#endif // DO_OVERHEAD_MEASUREMENTS


					   /*
					    * Functions.
					    */

					   /* Helper function to get TSC */
					   static inline void tscval(u64 *v)
{
#if READ_MSR_FOR_TSC
    u64 res;
    rdmsrl(0x10, res);
    *v = res;
#else
    unsigned int aux;
    rdtscpll(*v, aux);
#endif // READ_MSR_FOR_TSC
};

/*
 * PCI io communication functions to read D states in north complex
 */
#if DO_D_NC_STATE_SAMPLE
static unsigned int PCI_ReadU32 (unsigned int pci_address)  {
    outl(pci_address,MTX_PCI_ADDR_IO);
    return inl(MTX_PCI_DATA_IO);
};

static void PCI_WriteU32 (unsigned int pci_address, unsigned int data)  {
    outl(pci_address, MTX_PCI_ADDR_IO);
    outl(data, MTX_PCI_DATA_IO);
    return;
};

static int get_D_NC_states (unsigned long *states)  {
    unsigned int ncaddr = 0x100461f0;
    unsigned int pwr_data = 0;
    PCI_WriteU32(MTX_ENABLE_PCI + MTX_PCI_MSG_CTRL_REG, (ncaddr ));
    if (states != NULL) {
        pwr_data = PCI_ReadU32(MTX_ENABLE_PCI + MTX_PCI_MSG_DATA_REG);
        memcpy(states, &pwr_data, sizeof(unsigned long));

        return SUCCESS;
    }

    return -ERROR;
};
#endif


#if DO_S_STATE_SAMPLE
static int get_S_state (u32 *state)  {
    if (mmio_pm_base != NULL && state != NULL) {
        *state = readl(mmio_pm_base + 0x4);

        return SUCCESS;
    }

    return -ERROR;
};
#endif


#if DO_D_SC_STATE_SAMPLE
static int get_D_SC_states (u32 *states)  {
    if (mmio_pm_base != NULL && states != NULL) {
        states[0] = readl(mmio_pm_base + 0x30);
        states[1] = readl(mmio_pm_base + 0x34);
        states[2] = readl(mmio_pm_base + 0x38);
        states[3] = readl(mmio_pm_base + 0x3C);

        return SUCCESS;
    }

    return -ERROR;
};
#endif


/*
 * IPC communication functions to SCU
 */
#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
// Send IPC command
static inline void ipc_command(u32 cmd)
{
    if (mmio_ipc1_base != NULL)
        writel(cmd, mmio_ipc1_base);
    else
        printk(KERN_ERR "mmio_ipc1_base is NULL!\n");
};

// Write IPC write buffer
static inline void ipc_data_writel(u32 data, u32 offset)
{
    if (mmio_ipc1_base != NULL)
        writel(data, mmio_ipc1_base + IPC_WBUF_OFFSET + offset);
    else
        printk(KERN_ERR "mmio_ipc1_base is NULL!\n");
};

// Read IPC status register
static inline u32 ipc_status(void)
{
    if (mmio_ipc1_base != NULL)
        return readl(mmio_ipc1_base + IPC_STS_OFFSET);

    printk(KERN_ERR "mmio_ipc1_base is NULL!\n");
    return ERROR;
};

// Read one byte from IPC read buffer
static inline u8 ipc_data_readb(u32 offset)
{
    char val = 0;

    if (mmio_ipc1_base != NULL)
        val = readb(mmio_ipc1_base + IPC_RBUF_OFFSET + offset);
    else
        printk(KERN_ERR "mmio_ipc1_base is NULL!\n");

    return val;
};

// Read 4 bytes from IPC read buffer
static inline u32 ipc_data_readl(u32 offset)
{
    u32 val = 0;

    if (mmio_ipc1_base != NULL)
        val = readl(mmio_ipc1_base + IPC_RBUF_OFFSET + offset);
    else
        printk(KERN_ERR "mmio_ipc1_base is NULL!\n");

    return val;
};

// Wait till scu status is busy
static inline int busy_loop(void)
{
    u32 status = 0;
    u32 count = 0;

    status = ipc_status();
    // Check the busy bit
    while (status & 1) {
        udelay(1);
        status = ipc_status();
        count++;

        // SCU is still busy after 1 msec
        if (count > 1000) {
            printk(KERN_INFO "[APWR] IPC is busy.\n");
            return -ERROR;;
        }
    }

    // Check the error bit
    if ((status >> 1) & 1) {
        return -ERROR;
    }

    return SUCCESS;
};
#endif

#if DO_S_RESIDENCY_SAMPLE
// Start S state residency counting
static int start_s_residency_counter(void)
{
    int ret = 0;

    OUTPUT(0, KERN_INFO "[APWR] Start S0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_START_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_START_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    return ret;
};

// Stop S state residency counting
static int stop_s_residency_counter(void)
{
    int ret = 0;

    OUTPUT(0, KERN_INFO "[APWR] Stop S0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_STOP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_STOP_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    return ret;
};

// Dump S state residency counting
static u64 dump_s_residency_counter(void)
{
    int ret;
    u64 delta_usec = 0;

    OUTPUT(0, KERN_INFO "[APWR] Dump S0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_DUMP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_DUMP_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    if(ret != SUCCESS)
        printk(KERN_ERR "Error: dump_s_residency_counter!\n");

#if 0
    if(stop_s_residency_counter() != SUCCESS)
        printk(KERN_ERR "Error: stop_s_residency_counter!\n");
    stopJIFF_s_residency = jiffies;
    if(start_s_residency_counter() != SUCCESS)
        printk(KERN_ERR "Error: start_s_residency_counter!\n");
    startJIFF_s_residency = jiffies;
#endif

    delta_usec = CURRENT_TIME_IN_USEC() - startJIFF_s_residency;

    return delta_usec;
};
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
// Start D residency counting in south complex
static int start_d_sc_residency_counter(void) {
    int ret = 0;

    OUTPUT(0, KERN_INFO "[APWR] Start D0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_START_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_START_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    return ret;
};

// Stop D residency counting in south complex
static int stop_d_sc_residency_counter(void) {
    int ret = 0;

    OUTPUT(0, KERN_INFO "[APWR] Stop D0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_STOP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_STOP_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    return ret;
};

// Dump D residency counting in south complex
static u64 dump_d_sc_residency_counter(void) {
    int ret;
    u64 delta_usec = 0;

    OUTPUT(0, KERN_INFO "[APWR] Dump D0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_DUMP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_DUMP_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    if(ret != SUCCESS)
        printk(KERN_ERR "Error: dump_d_sc_residency_counter!\n");

#if 0
    if(stop_d_sc_residency_counter() != SUCCESS)
        printk(KERN_ERR "Error: stop_d_sc_residency_counter!\n");
    stopJIFF_d_sc_residency = jiffies;
    if(start_d_sc_residency_counter() != SUCCESS)
        printk(KERN_ERR "Error: start_d_sc_residency_counter!\n");
    startJIFF_d_sc_residency = jiffies;
#endif

    delta_usec = CURRENT_TIME_IN_USEC() - startJIFF_d_sc_residency;

    return delta_usec;
};
#endif


/*
 * Initialization and termination routines.
 */
static void destroy_timer_map(void)
{
    /*
     * NOP: nothing to free here -- timer nodes
     * are freed when their corresponding
     * (per-cpu) blocks are freed.
     */
};

static int init_timer_map(void)
{
    int i=0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i){
        INIT_HLIST_HEAD(&timer_map[i].head);
    }

    for(i=0; i<NUM_HASH_LOCKS; ++i){
        spin_lock_init(&hash_locks[i]);
    }

    return SUCCESS;
};


#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static int init_irq_map(void)
{
    int i=0;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
        INIT_HLIST_HEAD(&irq_map[i].head);
    }

    /*
     * Init locks
     */
    for(i=0; i<NUM_HASH_LOCKS; ++i){
        spin_lock_init(&irq_map_locks[i]);
    }

    total_num_irq_mappings = 0;

    return SUCCESS;
};


static void free_irq_node_i(irq_node_t *node)
{
    if(!node)
        return;

    if(node->name)
        pw_kfree(node->name);

    pw_kfree(node);

    return;
};

static void destroy_irq_map(void)
{
    int i=0;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
        struct hlist_head *head = &irq_map[i].head;
        while(!hlist_empty(head)){
            struct irq_node *node = hlist_entry(head->first, struct irq_node, list);
            hlist_del(&node->list);
            free_irq_node_i(node);
        }
    }

    if(irq_mappings_list){
        pw_kfree(irq_mappings_list);
        irq_mappings_list = NULL;
    }
};

#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS


static void free_timer_block(tblock_t *block)
{
    if(!block){
	return;
    }
    if(block->data){
	int i=0;
	for(i=0; i<NUM_TIMER_NODES_PER_BLOCK; ++i){
            /*
             * Check trace, just to be sure
             * (We shouldn't need this -- 'timer_destroy()'
             * explicitly checks and frees call trace
             * arrays).
             */
	    if(block->data[i].trace)
		pw_kfree(block->data[i].trace);
        }
	pw_kfree(block->data);
    }
    free_timer_block(block->next);
    pw_kfree(block);
    return;
};

static tblock_t *allocate_new_timer_block(struct hnode *free_head)
{
    tblock_t *block = pw_kmalloc(sizeof(tblock_t), GFP_ATOMIC);
    if(!block){
	return NULL;
    }
    block->data = pw_kmalloc(sizeof(tnode_t) * NUM_TIMER_NODES_PER_BLOCK, GFP_ATOMIC);
    if(!block->data){
	pw_kfree(block);
	return NULL;
    }
    memset(block->data, 0, sizeof(tnode_t) * NUM_TIMER_NODES_PER_BLOCK);
    if(free_head){
	LINK_FREE_TNODE_ENTRIES(block->data, NUM_TIMER_NODES_PER_BLOCK, free_head);
    }
    block->next = NULL;
    return block;
};

static void destroy_per_cpu_timer_blocks(void)
{
    int cpu = -1;

    for_each_online_cpu(cpu){
	per_cpu_mem_t *pcpu_mem = GET_MEM_VARS(cpu);
	tblock_t *blocks = pcpu_mem->block_list;
	free_timer_block(blocks);
    }
};

static int init_per_cpu_timer_blocks(void)
{
    int cpu = -1;

    for_each_online_cpu(cpu){
	per_cpu_mem_t *pcpu_mem = GET_MEM_VARS(cpu);
	struct hnode *free_head = &pcpu_mem->free_list_head;
	INIT_HLIST_HEAD(&free_head->head);
	if(!(pcpu_mem->block_list = allocate_new_timer_block(free_head))){
	    return -ERROR;
        }
    }

    return SUCCESS;
};

#if DO_RCU_OUTPUT_BUFFERS

static void free_output_set(output_set_t *set)
{
    if(set->lists){
	pw_kfree(set->lists);
    }
    pw_kfree(set);
};

static inline void clear_output_set(output_set_t *set)
{
    int cpu=0, i=0;

    /*
     * We have two buffers (or "segments") for every "list"
     * and one "list" for every cpu.
     */
    for_each_online_cpu(cpu){
	memset(&set->lists[cpu], 0, sizeof(list_t));
	for(i=0; i<NUM_SEGS_PER_LIST; ++i){
	    atomic_set(&set->lists[cpu].segs[i].is_full, EMPTY);
	}
	smp_mb();
    }
};

static output_set_t *get_new_output_set(void)
{
    output_set_t *set = (output_set_t *)pw_kmalloc(sizeof(output_set_t), GFP_KERNEL);
    if(!set){
	printk(KERN_INFO "ERROR: could NOT allocate new list set node!\n");
	return NULL;
    }
    if(!(set->lists = (list_t *)pw_kmalloc(PW_max_num_cpus * sizeof(list_t), GFP_KERNEL))){
	printk(KERN_INFO "ERROR: Could NOT allocate memory for lists for a new set!\n");
	pw_kfree(set);
	return NULL;
    }

    /*
     * Set indices etc.
     */
    clear_output_set(set);

    return set;
};

static void destroy_output_sets(void)
{
    int i=0;
    for(i=0; i<2; ++i)
	if(output_sets[i])
	    free_output_set(output_sets[i]);
};

static int init_output_sets(void)
{
    int i=0;
    for(i=0; i<2; ++i){
	if( (output_sets[i] = get_new_output_set()) == NULL){
	    return -ERROR;
	}
    }

    curr_output_set_index = 0;

    current_output_set = output_sets[curr_output_set_index];
    return SUCCESS;
};

#endif // DO_RCU_OUTPUT_BUFFERS

static void destroy_data_structures(void)
{
    destroy_timer_map();

    destroy_per_cpu_timer_blocks();

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
    destroy_irq_map();
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS

#if DO_RCU_OUTPUT_BUFFERS
    /*
     * Make sure all RCU callbacks etc.
     * are done!
     */
    {
	rcu_barrier();
    }
    destroy_output_sets();
#else
    if(lists) {
	int cpu = 0;
	for_each_online_cpu(cpu) {
	    if (lists[cpu]) {
		pw_kfree(lists[cpu]);
	    }
	}
        pw_kfree(lists);
    }
    lists = NULL;
#endif // DO_RCU_OUTPUT_BUFFERS

};

static int init_data_structures(void)
{
    /*
     * Find the # CPUs in this system.
     */
    PW_max_num_cpus = num_online_cpus();

    /*
     * Enforce an upper limit of 64
     * online cpus -- used in the
     * irq_node->cpu_bitmap data structure.
     */
    if (false && PW_max_num_cpus > sizeof(u64) * 8) { // Total # of bits in 'u64'
        printk(KERN_INFO "ERROR: Detected %d CPUs: MAX of %d ONLINE CPUS currently supported!\n", PW_max_num_cpus, (int)(sizeof(u64) * 8));
        return -ERROR;
    }

#if DO_RCU_OUTPUT_BUFFERS
    {
	if(init_output_sets()){
	    printk(KERN_INFO "ERROR: could NOT initialize the per-cpu output lists!\n");
	    destroy_data_structures();
	    return -ERROR;
	}
    }
#else
    {
	/*
	 * Init the (per-cpu) output buffers.
	 * We have two buffers (or "segments") for every "list"
	 * and one "list" for every cpu.
	 */
	int cpu = 0, i = 0;
	lists = (list_t **)pw_kmalloc(sizeof(list_t *) * PW_max_num_cpus, GFP_KERNEL);
	if(!lists){
	    printk(KERN_INFO "ERROR: could NOT allocate array-of-lists!\n");
	    return -ERROR;
	}
	memset(lists, 0, sizeof(list_t *) * PW_max_num_cpus);

	for_each_online_cpu(cpu) {
	    lists[cpu] = (list_t *)pw_kmalloc(sizeof(list_t), GFP_KERNEL);
	    if (lists[cpu] == NULL) {
		printk(KERN_INFO "ERROR: could NOT allocate list[%d]!\n", cpu);
		destroy_data_structures();
		return -ERROR;
	    }
	    memset(lists[cpu], 0, sizeof(list_t));
	    for (i=0; i<NUM_SEGS_PER_LIST; ++i) {
		atomic_set(&lists[cpu]->segs[i].is_full, EMPTY);
	    }
	}
	smp_mb();
    }
#endif // DO_RCU_OUTPUT_BUFFERS

    /*
     * Init the (per-cpu) free lists
     * for timer mappings.
     */
    if(init_per_cpu_timer_blocks()){
        printk(KERN_INFO "ERROR: could NOT initialize the per-cpu timer blocks!\n");
        destroy_data_structures();
        return -ERROR;
    }

    if(init_timer_map()){
        printk(KERN_INFO "ERROR: could NOT initialize timer map!\n");
        destroy_data_structures();
        return -ERROR;
    }

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
    if(init_irq_map()){
        printk(KERN_INFO "ERROR: could NOT initialize irq map!\n");
        destroy_data_structures();
        return -ERROR;
    }
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS

    return SUCCESS;
};

/*
 * Free list manipulation routines.
 */

static int init_tnode_i(tnode_t *node, unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, int trace_len, unsigned long *trace)
{

    if(node->trace){
        pw_kfree(node->trace);
        node->trace = NULL;
    }

    node->timer_addr = timer_addr; node->tsc = tsc; node->tid = tid; node->pid = pid; node->trace_sent = 0; node->trace_len = trace_len;

    if(trace_len >  0){
        node->trace = pw_kmalloc(sizeof(unsigned long) * trace_len, GFP_ATOMIC);
        if(!node->trace){
            printk(KERN_INFO "ERROR: could NOT allocate memory for backtrace!\n");
            // pw_kfree(node);
            return -ERROR;
        }
        memcpy(node->trace, trace, sizeof(unsigned long) * trace_len); // dst, src
    }

    /*
     * Ensure everyone sees this...
     */
    smp_mb();

    return SUCCESS;
};

static tnode_t *get_next_free_tnode_i(unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, int trace_len, unsigned long *trace)
{
    per_cpu_mem_t *pcpu_mem = GET_MY_MEM_VARS();
    struct hnode *free_head = &pcpu_mem->free_list_head;
    struct hlist_head *head = &free_head->head;

    if(hlist_empty(head)){
	tblock_t *block = allocate_new_timer_block(free_head);
	if(block){
	    block->next = pcpu_mem->block_list;
	    pcpu_mem->block_list = block;
	}
	OUTPUT(3, KERN_INFO "[%d]: ALLOCATED A NEW TIMER BLOCK!\n", CPU());
    }

    if(!hlist_empty(head)){
	struct tnode *node = hlist_entry(head->first, struct tnode, list);
	hlist_del(&node->list);
	/*
	 * 'kmalloc' doesn't zero out memory -- set
	 * 'trace' to NULL to avoid an invalid
	 * 'free' in 'init_tnode_i(...)' just to
	 * be sure (Shouldn't need to have to
	 * do this -- 'destroy_timer()' *should*
	 * have handled it for us).
	 */
	node->trace = NULL;

	if(init_tnode_i(node, timer_addr, tid, pid, tsc, trace_len, trace)){
	    /*
	     * Backtrace couldn't be inited -- re-enqueue
	     * onto the free-list.
	     */
	    node->trace = NULL;
	    hlist_add_head(&node->list, head);
	    return NULL;
	}
	return node;
    }
    return NULL;
};

static void timer_destroy(struct tnode *node)
{
    per_cpu_mem_t *pcpu_mem = GET_MY_MEM_VARS();
    struct hnode *free_head = &pcpu_mem->free_list_head;

    OUTPUT(3, KERN_INFO "DESTROYING %p\n", node);

    if(node->trace){
	pw_kfree(node->trace);
	node->trace = NULL;
    }

    hlist_add_head(&node->list, &((free_head)->head));
};

/*
 * Hash map routines.
 */

static tnode_t *timer_find(unsigned long timer_addr, pid_t tid)
{
    int idx = TIMER_HASH_FUNC(timer_addr);
    tnode_t *node = NULL, *retVal = NULL;
    struct hlist_node *curr = NULL;
    struct hlist_head *head = NULL;

    HASH_LOCK(idx);
    {
	head = &timer_map[idx].head;

	hlist_for_each_entry(node, curr, head, list){
	    if(node->timer_addr == timer_addr && (node->tid == tid || tid < 0)){
		retVal = node;
		break;
	    }
	}
    }
    HASH_UNLOCK(idx);

    return retVal;
};


static void timer_insert(unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, int trace_len, unsigned long *trace)
{
    int idx = TIMER_HASH_FUNC(timer_addr);
    struct hlist_node *curr = NULL;
    struct hlist_head *head = NULL;
    struct tnode *node = NULL, *new_node = NULL;
    bool found = false;

    HASH_LOCK(idx);
    {
        head = &timer_map[idx].head;

        hlist_for_each_entry(node, curr, head, list){
            if(node->timer_addr == timer_addr){
                /*
                 * Update-in-place.
                 */
                OUTPUT(3, KERN_INFO "Timer %p UPDATING IN PLACE! Node = %p, Trace = %p\n", (void *)timer_addr, node, node->trace);
                init_tnode_i(node, timer_addr, tid, pid, tsc, trace_len, trace);
                found = true;
                break;
            }
        }

        if(!found){
            /*
             * Insert a new entry here.
             */
	    new_node = get_next_free_tnode_i(timer_addr, tid, pid, tsc, trace_len, trace);
            if(likely(new_node)){
                hlist_add_head(&new_node->list, &timer_map[idx].head);
#if DO_OVERHEAD_MEASUREMENTS
                {
                    smp_mb();
                    atomic_inc(&num_timer_entries);
                }
#endif
            }else{ // !new_node
                printk(KERN_INFO "ERROR: could NOT allocate new timer node!\n");
            }
        }
    }
    HASH_UNLOCK(idx);

    return;
};

static int timer_delete(unsigned long timer_addr, pid_t tid)
{
    int idx = TIMER_HASH_FUNC(timer_addr);
    tnode_t *node = NULL, *found_node = NULL;
    struct hlist_node *curr = NULL, *next = NULL;
    struct hlist_head *head = NULL;
    int retVal = -ERROR;

    HASH_LOCK(idx);
    {
	head = &timer_map[idx].head;

	hlist_for_each_entry_safe(node, curr, next, head, list){
	    // if(node->timer_addr == timer_addr && node->tid == tid){
            if(node->timer_addr == timer_addr) {
                if (node->tid != tid){
                    OUTPUT(0, KERN_INFO "WARNING: stale timer tid value? node tid = %d, task tid = %d\n", node->tid, tid);
		}
		hlist_del(&node->list);
		found_node = node;
		retVal = SUCCESS;
		OUTPUT(3, KERN_INFO "[%d]: TIMER_DELETE FOUND HRT = %p\n", tid, (void *)timer_addr);
		break;
	    }
	}
    }
    HASH_UNLOCK(idx);

    if(found_node){
	timer_destroy(found_node);
    }

    return retVal;
};

static void delete_all_non_kernel_timers(void)
{
    struct tnode *node = NULL;
    struct hlist_node *curr = NULL, *next = NULL;
    int i=0, num_timers = 0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	{
	    HASH_LOCK(i);
	    {
		hlist_for_each_entry_safe(node, curr, next, &timer_map[i].head, list){
		    if(node->tid != 0){
			++num_timers;
			OUTPUT(3, KERN_INFO "[%d]: Timer %p (Node %p) has TRACE = %p\n", node->tid, (void *)node->timer_addr, node, node->trace);
			hlist_del(&node->list);
			timer_destroy(node);
		    }
		}
	    }
	    HASH_UNLOCK(i);
	}
};


static void delete_timers_for_tid(pid_t tid)
{
    struct tnode *node = NULL;
    struct hlist_node *curr = NULL, *next = NULL;
    int i=0, num_timers = 0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	{
	    HASH_LOCK(i);
	    {
		hlist_for_each_entry_safe(node, curr, next, &timer_map[i].head, list){
		    if(node->tid == tid){
			++num_timers;
			OUTPUT(3, KERN_INFO "[%d]: Timer %p (Node %p) has TRACE = %p\n", tid, (void *)node->timer_addr, node, node->trace);
			hlist_del(&node->list);
			timer_destroy(node);
		    }
		}
	    }
	    HASH_UNLOCK(i);
	}

    OUTPUT(3, KERN_INFO "[%d]: # timers = %d\n", tid, num_timers);
};

static int get_num_timers(void)
{
    tnode_t *node = NULL;
    struct hlist_node *curr = NULL;
    int i=0, num=0;


    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	hlist_for_each_entry(node, curr, &timer_map[i].head, list){
	    ++num;
	    OUTPUT(3, KERN_INFO "[%d]: %d --> %p\n", i, node->tid, (void *)node->timer_addr);
	}

    return num;
};

/*
 * IRQ list manipulation routines.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static void irq_destroy_callback(struct rcu_head *head)
{
    struct irq_node *node = container_of(head, struct irq_node, rcu);

    if(node->name){
	pw_kfree(node->name);
	node->name = NULL;
    }
    if (node->cpu_bitmap) {
        pw_kfree(node->cpu_bitmap);
        node->cpu_bitmap = NULL;
    }
    pw_kfree(node);
};

static irq_node_t *get_next_free_irq_node_i(int cpu, int irq_num, const char *irq_name)
{
    irq_node_t *node = pw_kmalloc(sizeof(irq_node_t), GFP_ATOMIC);

    if(likely(node)){
	memset(node, 0, sizeof(irq_node_t));
	node->irq = irq_num;
	/*
	 * Set current CPU bitmap.
	 */
        node->cpu_bitmap = pw_kmalloc(sizeof(unsigned long) * NUM_BITMAP_BUCKETS, GFP_ATOMIC);
        if (unlikely(!node->cpu_bitmap)) {
            printk(KERN_INFO "ERROR: could NOT allocate a bitmap for the new irq_node!\n");
            pw_kfree(node);
            return NULL;
        }
        memset(node->cpu_bitmap, 0, sizeof(unsigned long) * NUM_BITMAP_BUCKETS);
        if (false) {
            printk(KERN_INFO "BEFORE = %lu (is_set = %d, %u)\n", node->cpu_bitmap[0], IS_BIT_SET(cpu, PWR_CPU_BITMAP(node)), test_bit(cpu, PWR_CPU_BITMAP(node)));
        }
        SET_BIT(cpu, PWR_CPU_BITMAP(node));

        if (false) {
            printk(KERN_INFO "AFTER = %lu (is_set = %d, %u)\n", node->cpu_bitmap[0], IS_BIT_SET(cpu, PWR_CPU_BITMAP(node)), test_bit(cpu, PWR_CPU_BITMAP(node)));
        }

	INIT_HLIST_NODE(&node->list);

	if( !(node->name = pw_kstrdup(irq_name, GFP_ATOMIC))){
	    printk(KERN_INFO "ERROR: could NOT kstrdup irq device name: %s\n", irq_name);
            pw_kfree(node->cpu_bitmap);
	    pw_kfree(node);
	    node = NULL;
	}
    }else{
	printk(KERN_INFO "ERROR: could NOT allocate new irq node!\n");
    }

    return node;

};

/*
 * Check if the given IRQ # <-> DEV Name mapping exists and, if
 * it does, whether this mapping was sent for the given 'cpu'
 * (We need to send each such mapping ONCE PER CPU to ensure it is
 * received BEFORE a corresponding IRQ C-state wakeup).
 */
static bool find_irq_node_i(int cpu, int irq_num, const char *irq_name, int *index, bool *was_mapping_sent)
{
    irq_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    int idx = IRQ_MAP_HASH_FUNC(irq_num);

    *index = idx;

    rcu_read_lock();

    hlist_for_each_entry_rcu(node, curr, &irq_map[idx].head, list){
	if(node->irq == irq_num
#if DO_ALLOW_MULTI_DEV_IRQ
	   && !strcmp(node->name, irq_name)
#endif // DO_ALLOW_MULTI_DEV_IRQ
	   )
	    {
		/*
		 * OK, so the maping exists. But each
		 * such mapping must be sent ONCE PER
		 * CPU to Ring-3 -- have we done so
		 * for this cpu?
		 */
		// *was_mapping_sent = (node->cpu_bitmap & (1 << cpu)) ? true : false;
                *was_mapping_sent = (IS_BIT_SET(cpu, PWR_CPU_BITMAP(node))) ? true : false;
                if (false) {
                    printk(KERN_INFO "FIND_IRQ_NODE: IRQ = %d, CPU = %d, BIT_SET = %d\n", irq_num, cpu, IS_BIT_SET(cpu, PWR_CPU_BITMAP(node)));
                }
		rcu_read_unlock();
		return true;
	    }
    }

    rcu_read_unlock();
    return false;
};

/*
 * Check to see if a given IRQ # <-> DEV Name mapping exists
 * in our list of such mappings and, if it does, whether this
 * mapping has been sent to Ring-3. Take appropriate actions
 * if any of these conditions is not met.
 */
static irq_mapping_types_t irq_insert(int cpu, int irq_num, const char *irq_name)
{
    irq_node_t *node = NULL;
    int idx = -1;
    bool found_mapping = false, mapping_sent = false;
    // int idx = IRQ_MAP_HASH_FUNC(irq_num);

    /*
     * Protocol:
     * (a) if mapping FOUND: return "OK_IRQ_MAPPING_EXISTS"
     * (b) if new mapping CREATED: return "OK_NEW_IRQ_MAPPING_CREATED"
     * (c) if ERROR: return "ERROR_IRQ_MAPPING"
     */

    found_mapping = find_irq_node_i(cpu, irq_num, irq_name, &idx, &mapping_sent);
    if(found_mapping && mapping_sent){
	/*
	 * OK, mapping exists AND we've already
	 * sent the mapping for this CPU -- nothing
	 * more to do.
	 */
	return OK_IRQ_MAPPING_EXISTS;
    }

    /*
     * Either this mapping didn't exist at all,
     * or the mapping wasn't sent for this CPU.
     * In either case, because we're using RCU,
     * we'll have to allocate a new node.
     */

    node = get_next_free_irq_node_i(cpu, irq_num, irq_name);

    if(unlikely(node == NULL)){
	printk(KERN_INFO "ERROR: could NOT allocate node for irq insertion!\n");
	return ERROR_IRQ_MAPPING;
    }

    IRQ_LOCK(idx);
    {
	/*
	 * It is *THEORETICALLY* possible that
	 * a different CPU added this IRQ entry
	 * to the 'irq_map'. For now, disregard
	 * the possiblility (at worst we'll have
	 * multiple entries with the same mapping,
	 * which is OK).
	 */
	bool found = false;
	irq_node_t *old_node = NULL;
	struct hlist_node *curr = NULL;
	if(found_mapping){
	    hlist_for_each_entry(old_node, curr, &irq_map[idx].head, list){
		if(old_node->irq == irq_num
#if DO_ALLOW_MULTI_DEV_IRQ
		   && !strcmp(old_node->name, irq_name)
#endif // DO_ALLOW_MULTI_DEV_IRQ
		   )
		    {
			/*
			 * Found older entry -- copy the 'cpu_bitmap'
			 * field over to the new entry (no need to set this
			 * CPU's entry -- 'get_next_free_irq_node_i() has
			 * already done that. Instead, do a BITWISE OR of
			 * the old and new bitmaps)...
			 */
			OUTPUT(0, KERN_INFO "[%d]: IRQ = %d, OLD bitmap = %lu\n", cpu, irq_num, *(old_node->cpu_bitmap));
			// node->cpu_bitmap |= old_node->cpu_bitmap;
                        /*
                         * UPDATE: new 'bitmap' scheme -- copy over the older
                         * bitmap array...
                         */
                        memcpy(node->cpu_bitmap, old_node->cpu_bitmap, sizeof(unsigned long) * NUM_BITMAP_BUCKETS); // dst, src
                        /*
                         * ...then set the current CPU's pos in the 'bitmap'
                         */
                        SET_BIT(cpu, node->cpu_bitmap);
			/*
			 * ...and then replace the old node with
			 * the new one.
			 */
			hlist_replace_rcu(&old_node->list, &node->list);
			call_rcu(&old_node->rcu, &irq_destroy_callback);
			/*
			 * OK -- everything done.
			 */
			found = true;
			break;
		    }
	    }
	    if(!found){
		printk(KERN_INFO "ERROR: CPU = %d, IRQ = %d, mapping_found but not found!\n", cpu, irq_num);
	    }
	}else{
	    hlist_add_head_rcu(&node->list, &irq_map[idx].head);
	    /*
	     * We've added a new mapping.
	     */
	    ++total_num_irq_mappings;
	}
    }
    IRQ_UNLOCK(idx);
    /*
     * Tell caller that this mapping
     * should be sent to Ring-3.
     */
    return OK_NEW_IRQ_MAPPING_CREATED;
};

/*
 * INTERNAL HELPER: retrieve number of
 * mappings in the IRQ mappings list.
 */
static int get_num_irq_mappings(void)
{
    int retVal = 0;
    int i=0;
    irq_node_t *node = NULL;
    struct hlist_node *curr = NULL;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i)
	hlist_for_each_entry(node, curr, &irq_map[i].head, list){
	    ++retVal;
	    OUTPUT(0, KERN_INFO "[%d]: IRQ Num=%d, Dev=%s\n", i, node->irq, node->name);
	}

    return retVal;

};


#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS


/*
 * Per-CPU OUTPUT buffer manipulation routines.
 */

/*
 * Single consumer: OK to have
 * unprotected access to read indices!
 */
static int last_list_read = -1;
static int last_flush_index = 0;

/*
 * Find the next (non-full) per-cpu output buffer
 * to place a PWCollector_sample struct instance into.
 * ***********************************************************
 * WARNING: MUST BE CALLED FROM WITHIN AN RCU-PROTECTED
 * REGION if 'DO_RCU_OUTPUT_BUFFERS' is set!!!
 * ***********************************************************
 */
static inline seg_t *find_producer_seg(int cpu)
{
    seg_t *seg = NULL;
    int list_index = -1, num_full = 0;
    list_t *list = GET_OUTPUT_LIST(cpu);
    do{
	list_index = list->prod_index;
	seg = &list->segs[list_index];
	smp_mb();
	if(atomic_read(&seg->is_full) == FULL){
	    OUTPUT(0, KERN_INFO "[%d]: Segment %d is FULL! # full = %d\n", cpu, list_index, num_full);
	    list->prod_index = CIRCULAR_INC(list_index, LIST_MASK);
	    continue;
	}
	return seg;
    }while(++num_full < NUM_SEGS_PER_LIST);

    /*
     * Only reaches here if we couldn't find a candidate
     * 'seg'.
     */
    printk(KERN_INFO "WARNING: List for CPU = %d was FULL!\n", cpu);

    return NULL;
};

/*
 * This function is used to signal when
 * any (per-cpu) output buffer is full.
 * Used to wakeup waiting (user-level)
 * reader processes.
 */
static bool any_seg_full(void)
{
    int cpu=0, j=0;
    list_t *list = NULL;
    seg_t *seg = NULL;

    for_each_online_cpu(cpu){
	list = UNPROTECTED_GET_OUTPUT_LIST(cpu);
        /*
         * We need to see if ANY seg
         * in this 'list' is FULL -- we
         * don't need ordering constraints
         * there!
         */
	for(j=0; j<NUM_SEGS_PER_LIST; ++j){
	    seg = &list->segs[j];
            smp_mb();
	    if(atomic_read(&seg->is_full) == FULL){
		last_list_read = cpu - 1; // "-1" because "find_seg()" increments "last_list_read" before reading it!
		return true;
	    }
	}
    }
    return false;
};

/*
 * Try and find a full output buffer.
 */
static inline seg_t *find_seg(void)
{
    int list_index = ++last_list_read;
    int read_index = -1;
    int cpu=0, j=0;
    list_t *list = NULL;
    seg_t *seg = NULL;

    if(list_index == PW_max_num_cpus)
	list_index = 0;

    for_each_online_cpu(cpu){
	/*
	 * "list_index" gives the next
	 * list to check for any FULL segments.
	 */
	list = UNPROTECTED_GET_OUTPUT_LIST(list_index);
        /*
         * 'read_index' defines which
         * SEGMENT to check FIRST -- this
         * ensures that if BOTH segments
         * are FULL (which can happen if we
         * have a slow consumer that isn't reading
         * data fast enough) then the OLDER
         * segment will be returned FIRST.
         */
        read_index = list->read_index;

	for(j=0; j<NUM_SEGS_PER_LIST; ++j){
	    // seg = &list->segs[j];
            seg = &list->segs[read_index];
            read_index = CIRCULAR_INC(read_index, LIST_MASK);
            smp_mb();
	    if(atomic_read(&seg->is_full) == FULL){
                /*
                 * (SIGH -- We shouldn't have
                 * to do this!) -- check OTHER buffer
                 * to see if it has OLDER
                 * data (check needs to be
                 * performed ONLY if OTHER
                 * BUFFER IS ALSO READABLE!).
                 */
                {
                    seg_t *other_seg = &list->segs[read_index]; // safe to do this -- 'read_index' was incremented above
                    if(atomic_read(&other_seg->is_full) == FULL){
                        u64 curr_tsc = SAMPLE(seg, 0).tsc, other_tsc = SAMPLE(other_seg, 0).tsc;
                        if(other_tsc < curr_tsc){
                            printk(KERN_INFO "WARNING: CPU %d has BOTH BUFFERS full. About to return TSC %llu which is NEWER than TSC %llu!\n", list_index, curr_tsc, other_tsc);
                            seg = other_seg;
                            read_index = CIRCULAR_INC(read_index, LIST_MASK);
                        }
                    }
                }
                /*
                 * Make sure we check the OTHER SEG
                 * the next time we do a 'read'
                 */
                list->read_index = read_index;
		last_list_read = list_index;

		return seg;
	    }
	}
	/*
	 * No segment was full for this CPU/List
	 */
	++list_index;
	if(list_index == PW_max_num_cpus)
	    list_index = 0;
	OUTPUT(2, KERN_INFO "List for Cpu %d is EMPTY!\n", cpu);
    }

    last_list_read = list_index;
    OUTPUT(1, KERN_INFO "Warning: EVERY List was EMPTY!\n");

    return NULL;
};

#define for_each_segment(idx) for((idx)=0; (idx) < NUM_SEGS_PER_LIST; ++(idx))

static inline seg_t *find_next_non_empty_seg(void)
{
    int j=0, seg_index = -1;
    list_t *list = NULL;
    seg_t *seg = NULL;

    while(last_flush_index < PW_max_num_cpus){
	list = UNPROTECTED_GET_OUTPUT_LIST(last_flush_index);
	seg_index = list->flush_index;
	for_each_segment(j){
	    seg = &list->segs[seg_index];
	    /*
	     * FORCE read EVERY (non-empty) segment in EVERY list
	     * (including the ones which aren't full yet).
	     * Do this ONLY if every producer has completed!
	     */
	    if(SAMPLE(seg, 0).sample_type != FREE_SAMPLE){
		/* if(!IS_SEG_EMPTY(seg)){ */
		/*
		 * OK, we've reached a non-empty segment.
		 */
		list->flush_index = seg_index;
		return seg;
	    }
	    seg_index = CIRCULAR_INC(seg_index, LIST_MASK);
	}
	++last_flush_index;
    }
    /*
     * All segments were empty for this CPU/list.
     */
    return NULL;
};

/*
 * Helper function that inserts data
 * into the (per-cpu) output buffers.
 */
static inline void produce_pwr_sample(int cpu, const struct PWCollector_sample *sample)
{
    seg_t *seg = NULL;
    bool should_wake = false;
    /*
     * Enter an RCU read-side critical
     * section.
     */
    BEGIN_PRODUCING();
    {
	/*
	 * Find an output segment.
	 */
	seg = find_producer_seg(cpu);
	if(likely(seg)){
	    int seg_index = seg->index;
	    memcpy(&SAMPLE(seg, seg_index), sample, sizeof(PWCollector_sample_t));
	    /*
	     * OK, we've written to the segment.
	     * Now increment indices, check for
	     * full conditions etc.
	     */
	    seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
	    if(!seg->index){ // index has wrapped around ==> FULL segment
		atomic_set(&seg->is_full, FULL);
		should_wake = true;
	    }
	}else{
	    /*
	     * Do NOT exit prematurely. Instead, signal
	     * an error and wait until we leave the
	     * RCU read-side critical section.
	     */
	    printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	}
    }
    STOP_PRODUCING();

    if(should_wake){
#if ALLOW_BLOCKING_READ
	wake_up_interruptible(&read_queue);
#endif
    }
};

/*
 * HELPER template function to illustrate
 * how to 'produce' data into the
 * (per-cpu) output buffers.
 */
static inline void producer_template(int cpu)
{
    /*
     * Template for any of the 'produce_XXX_sample(...)'
     * functions.
     */
    struct PWCollector_sample sample;
    // Populate 'sample' fields in a domain-specific
    // manner. e.g.:
    // sample.foo = bar
    /*
     * OK, computed 'sample' fields. Now
     * write sample into the output buffer.
     */
    produce_pwr_sample(cpu, &sample);
};

/*
 * Insert a C-state sample into a (per-cpu) output buffer.
 */
static inline void produce_c_sample(int cpu, per_cpu_t *pcpu, int last_PID, int last_TID, char sample_type, unsigned long long sample_data)
{
    struct PWCollector_sample sample;

    sample.sample_type = C_STATE;
    sample.cpuidx = cpu;
    sample.tsc = pcpu->tsc;
    memcpy(&RES_COUNT(sample.c_sample, 0), pcpu->residencies, sizeof(u64) * MAX_MSR_ADDRESSES); // dst, src
    sample.c_sample.pid = last_PID;
    sample.c_sample.tid = last_TID;
    sample.c_sample.break_type = sample_type;
    sample.c_sample.c_data = sample_data;

    sample.c_sample.prev_state = pcpu->prev_state;

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};

#if DO_S_RESIDENCY_SAMPLE
/*
 * Insert a S Residency counter sample into a (per-cpu) output buffer.
 */
static inline void produce_s_residency_sample(u64 usec)
{
    seg_t *seg = NULL;
    int seg_index = -1;
    u64 tsc;
    int cpu = CPU();

    /*
     * No residency counters available
     */
    tscval(&tsc);

    if(!(seg = find_producer_seg(cpu))){
        printk(KERN_ERR "Error: No buffer available for s_residency_sample on cpu=%d!\n", cpu);
        return;
    }

    seg_index = seg->index;
    {
        SAMPLE(seg, seg_index).sample_type = S_RESIDENCY;
        SAMPLE(seg, seg_index).cpuidx = cpu;
        SAMPLE(seg, seg_index).tsc = tsc;
        S_RESIDENCY_SAMPLE(seg, seg_index).usec = (unsigned int)usec;
        memcpy(S_RESIDENCY_SAMPLE(seg, seg_index).counters, mmio_s_residency_base, sizeof(u32) * 3);
    }

    /*
     * OK, we've written to the segment.
     * Now increment indices, check for
     * full conditions etc.
     */
    seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
    if(!seg->index){ // index has wrapped around ==> FULL segment
        atomic_set(&seg->is_full, FULL);
        smp_mb();
#if ALLOW_BLOCKING_READ
        wake_up_interruptible(&read_queue);
#endif
    }
};
#endif


#if DO_D_SC_RESIDENCY_SAMPLE
/*
 * Insert a D Residency counter sample into a (per-cpu) output buffer.
 */
static inline void produce_d_sc_residency_sample(u64 usec)
{
    int seg_index = -1;
    seg_t *seg = NULL;
    u64 tsc;
    int cpu = CPU();
    int i,j;
    int start_idx = 0;
    int num = 0;
    int numofsamples = (d_sc_count_num + 5) / 6;  // Currently, 6 devices info can be included into a single sample

    tscval(&tsc);

    for (i=0; i<numofsamples; i++) {
        num = 0;

        if(!(seg = find_producer_seg(cpu))){
            printk(KERN_ERR "Error: No buffer available for d_sc_residency_sample on cpu=%d!\n", cpu);
            return;
        }

        seg_index = seg->index;
        SAMPLE(seg, seg_index).sample_type = D_RESIDENCY;
        SAMPLE(seg, seg_index).cpuidx = cpu;
        SAMPLE(seg, seg_index).tsc = tsc;
        D_RESIDENCY_SAMPLE(seg, seg_index).device_type = PW_SOUTH_COMPLEX;

        for (j=start_idx; j<MAX_LSS_NUM_IN_SC; j++) {
            if ((d_sc_mask >> j) & 0x1) {
                D_RESIDENCY_SAMPLE(seg, seg_index).mask[num] = j;
                D_RESIDENCY_SAMPLE(seg, seg_index).d_residency_counters[num].usec = (unsigned int)usec;
                D_RESIDENCY_SAMPLE(seg, seg_index).d_residency_counters[num].D0i0_CG = readl(mmio_d_residency_base + sizeof(u32)*j);
                D_RESIDENCY_SAMPLE(seg, seg_index).d_residency_counters[num].D0i1 = readl(mmio_d_residency_base + 40*4 + sizeof(u32)*j);
                D_RESIDENCY_SAMPLE(seg, seg_index).d_residency_counters[num].D0i3 = readl(mmio_d_residency_base + 40*4*2 + sizeof(u32)*j);

                start_idx = j+1;
                if(++num == 6)
                    break;
            }
        }
        D_RESIDENCY_SAMPLE(seg, seg_index).num_sampled = num;

        if(true){
            /*
             * OK, we've written to the segment.
             * Now increment indices, check for
             * full conditions etc.
             */
            seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
            if(!seg->index){ // index has wrapped around ==> FULL segment
                atomic_set(&seg->is_full, FULL);
                smp_mb();
#if ALLOW_BLOCKING_READ
                wake_up_interruptible(&read_queue);
#endif
            }
        }
    }
};
#endif


#if DO_S_STATE_SAMPLE
/*
 * Insert a S state sample into a (per-cpu) output buffer.
 */
static inline void produce_s_state_sample(void)
{
    int seg_index = -1;
    seg_t *seg = NULL;
    u64 tsc;
    int cpu = CPU();
    u32 state;

    tscval(&tsc);

    if(get_S_state(&state))
        return;

    if(!(seg = find_producer_seg(cpu))){
        printk(KERN_ERR "Error: No buffer available for s_state_sample on cpu=%d!\n", cpu);
        return;
    }

    seg_index = seg->index;
    {
        SAMPLE(seg, seg_index).sample_type = S_STATE;
        SAMPLE(seg, seg_index).cpuidx = cpu;
        SAMPLE(seg, seg_index).tsc = tsc;
        S_STATE_SAMPLE(seg, seg_index).state = state;
    }

    if(true){
        /*
         * OK, we've written to the segment.
         * Now increment indices, check for
         * full conditions etc.
         */
        seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
        if(!seg->index){ // index has wrapped around ==> FULL segment
            atomic_set(&seg->is_full, FULL);
            smp_mb();
#if ALLOW_BLOCKING_READ
            wake_up_interruptible(&read_queue);
#endif
        }
    }

}
#endif


#if DO_D_NC_STATE_SAMPLE
/*
 * Insert a north complex D state sample into a (per-cpu) output buffer.
 */
static inline void produce_d_nc_state_sample(void)
{
    int seg_index = -1;
    seg_t *seg = NULL;
    u64 tsc;
    int cpu = CPU();
    unsigned long ncstates = 0;

    tscval(&tsc);

    if(get_D_NC_states(&ncstates))
        return;

    if(!(seg = find_producer_seg(cpu))){
        printk(KERN_ERR "Error: No buffer available for d_nc_state_sample on cpu=%d!\n", cpu);
        return;
    }

    seg_index = seg->index;
    {
        SAMPLE(seg, seg_index).sample_type = D_STATE;
        SAMPLE(seg, seg_index).cpuidx = cpu;
        SAMPLE(seg, seg_index).tsc = tsc;
        D_STATE_SAMPLE(seg, seg_index).device_type = PW_NORTH_COMPLEX;
        D_STATE_SAMPLE(seg, seg_index).states[0] = ncstates;
    }

    if(true){
        /*
         * OK, we've written to the segment.
         * Now increment indices, check for
         * full conditions etc.
         */
        seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
        if(!seg->index){ // index has wrapped around ==> FULL segment
            atomic_set(&seg->is_full, FULL);
            smp_mb();
#if ALLOW_BLOCKING_READ
            wake_up_interruptible(&read_queue);
#endif
        }
    }
};
#endif


#if DO_D_SC_STATE_SAMPLE
/*
 * Insert a south complex D state sample into a (per-cpu) output buffer.
 */
static inline void produce_d_sc_state_sample(void)
{
    int seg_index = -1;
    seg_t *seg = NULL;
    u64 tsc;
    int cpu = CPU();
    u32 scstates[4];

    tscval(&tsc);

    if(get_D_SC_states(scstates))
        return;

    if(!(seg = find_producer_seg(cpu))){
        printk(KERN_ERR "Error: No buffer available for d_sc_state_sample on cpu=%d!\n", cpu);
        return;
    }

    seg_index = seg->index;
    {
        SAMPLE(seg, seg_index).sample_type = D_STATE;
        SAMPLE(seg, seg_index).cpuidx = cpu;
        SAMPLE(seg, seg_index).tsc = tsc;
        D_STATE_SAMPLE(seg, seg_index).device_type = PW_SOUTH_COMPLEX;
        memcpy(D_STATE_SAMPLE(seg, seg_index).states, scstates, sizeof(u32)*4);
    }

    if(true){
        /*
         * OK, we've written to the segment.
         * Now increment indices, check for
         * full conditions etc.
         */
        seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
        if(!seg->index){ // index has wrapped around ==> FULL segment
            atomic_set(&seg->is_full, FULL);
            smp_mb();
#if ALLOW_BLOCKING_READ
            wake_up_interruptible(&read_queue);
#endif
        }
    }

}
#endif


#if DO_WAKELOCK_SAMPLE
/*
 * Insert a Wakelock sample into a (per-cpu) output buffer.
 */
static inline void produce_w_sample(int cpu, u64 tsc, w_sample_type_t type, pid_t tid, pid_t pid, const char *name)
{
    int seg_index = -1;
    seg_t *seg = NULL;

    if(!(seg = find_producer_seg(cpu))){
        printk(KERN_ERR "Error: No buffer available for wakelock_sample on cpu=%d!\n", cpu);
	return;
    }

    seg_index = seg->index;
    {
        SAMPLE(seg, seg_index).sample_type = W_STATE;
	SAMPLE(seg, seg_index).cpuidx = cpu;
	SAMPLE(seg, seg_index).tsc = tsc;
	W_SAMPLE(seg, seg_index).type = type;
	W_SAMPLE(seg, seg_index).tid = tid;
	W_SAMPLE(seg, seg_index).pid = pid;
	memcpy(W_SAMPLE(seg, seg_index).name, name, PW_MAX_WAKELOCK_NAME_SIZE); // dst, src
	OUTPUT(0, KERN_INFO "%d , %d -> %s\n", W_SAMPLE(seg, seg_index).tid, W_SAMPLE(seg, seg_index).pid, W_SAMPLE(seg, seg_index).name);
    }

    if(true){
        /*
         * OK, we've written to the segment.
         * Now increment indices, check for
         * full conditions etc.
         */
        seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
        if(!seg->index){ // index has wrapped around ==> FULL segment
            atomic_set(&seg->is_full, FULL);
            smp_mb();
#if ALLOW_BLOCKING_READ
            wake_up_interruptible(&read_queue);
#endif
        }
    }
};
#endif


/*
 * Insert a P-state transition sample into a (per-cpu) output buffer.
 */
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
static inline void produce_p_sample(int cpu, unsigned long long tsc, u32 req_freq, u32 act_freq, u8 is_boundary)
#else
    static inline void produce_p_sample(int cpu, unsigned long long tsc, u32 freq, u8 is_boundary)
#endif
{
    struct PWCollector_sample sample;

    sample.sample_type = P_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
    sample.p_sample.prev_req_frequency = req_freq;
    // sample.p_sample.prev_act_frequency = act_freq;
    sample.p_sample.frequency = act_freq;
#else // DO_DYNAMIC_FREQUENCY_MEASUREMENT
    sample.p_sample.frequency = freq;
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
    sample.p_sample.is_boundary_sample = is_boundary;

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};

/*
 * Insert a K_CALL_STACK sample into a (per-cpu) output buffer.
 */
static inline void produce_k_sample(int cpu, const tnode_t *tentry)
{
    struct PWCollector_sample sample;

    sample.sample_type = K_CALL_STACK;
    sample.sample_len = 1;
    sample.cpuidx = cpu;
    sample.tsc = tentry->tsc;
    sample.k_sample.trace_len = tentry->trace_len;
    /*
     * Generate the "entryTSC" and "exitTSC" values here.
     * Note that this is GUARANTEED to be a k-space call stack.
     */
    {
	sample.k_sample.entry_tsc = tentry->tsc - 1;
	sample.k_sample.exit_tsc = tentry->tsc + 1;
    }
    /*
     * Also populate the trace here!
     */
    if(tentry->trace_len){
	int num = tentry->trace_len;
	int i=0;
	u64 *trace = sample.k_sample.trace;
	if(tentry->trace_len >= TRACE_LEN){
	    OUTPUT(0, KERN_ERR "Warning: kernel trace len = %d > TRACE_LEN = %d! Will need CHAINING!\n", num, TRACE_LEN);
	    num = TRACE_LEN;
	}
	/*
	 * Can't 'memcpy()' -- individual entries in
	 * the 'k_sample_t->trace[]' array are ALWAYS
	 * 64 bits wide, REGARDLESS OF THE UNDERLYING
	 * ARCHITECTURE!
	 */
	// memcpy(sample.k_sample.trace, tentry->trace, sizeof(unsigned long) * num);
	for(i=0; i<num; ++i){
	    trace[i] = tentry->trace[i];
	}
    }
    OUTPUT(3, KERN_INFO "KERNEL-SPACE mapping!\n");

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};

/*
 * Insert an IRQ_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_i_sample(int cpu, int num, const char *name)
{
    struct PWCollector_sample sample;

    sample.sample_type = IRQ_MAP;
    sample.cpuidx = cpu;
    sample.i_sample.irq_num = num;
    memcpy(sample.i_sample.irq_name, name, PW_IRQ_DEV_NAME_LEN); // dst, src

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};

/*
 * Insert a PROC_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_r_sample(int cpu, u64 tsc, r_sample_type_t type, pid_t tid, pid_t pid, const char *name)
{
    struct PWCollector_sample sample;

    sample.sample_type = PROC_MAP;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.r_sample.type = type;
    sample.r_sample.tid = tid;
    sample.r_sample.pid = pid;
    memcpy(sample.r_sample.proc_name, name, PW_MAX_PROC_NAME_SIZE); // dst, src

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};

/*
 * Insert an M_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_m_sample(int cpu, const char *name, unsigned long long begin, unsigned long long sz)
{
    struct PWCollector_sample sample;

    sample.sample_type = M_MAP;
    sample.cpuidx = cpu;
    sample.m_sample.start = begin;
    sample.m_sample.end = (begin+sz);
    sample.m_sample.offset = 0;
    memcpy(sample.m_sample.name, name, PW_MODULE_NAME_LEN); // dst, src

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};


/*
 * Probe functions (and helpers).
 */


/*
 * Generic method to generate a kernel-space call stack.
 * Utilizes the (provided) "save_stack_trace()" function.
 */
int __get_kernel_timerstack(unsigned long buffer[], int len)
{
    struct stack_trace strace;

    strace.max_entries = len; // MAX_BACKTRACE_LENGTH;
    strace.nr_entries = 0;
    strace.entries = buffer;
    strace.skip = 3;

    save_stack_trace(&strace);

    OUTPUT(0, KERN_INFO "[%d]: KERNEL TRACE: nr_entries = %d\n", TID(), strace.nr_entries);

    return strace.nr_entries;
};

/*
 * Generate a kernel-space call stack.
 * Requires the kernel be compiler with frame pointers ON.
 *
 * Returns number of return addresses in the call stack
 * or ZERO, to indicate no stack.
 */
int get_kernel_timerstack(unsigned long buffer[], int len)
{
    return __get_kernel_timerstack(buffer, len);
};


static void timer_init(void *timer_addr)
{
    pid_t tid = TID();
    pid_t pid = PID();
    u64 tsc = 0;
    int trace_len = 0;
    int i=0;
    char symname[KSYM_NAME_LEN];
    unsigned long trace[MAX_BACKTRACE_LENGTH];

    tscval(&tsc);

    /*
     * For accuracy, we ALWAYS collect
     * kernel call stacks.
     */
    if(!tid){
	/*
	 * get kernel timerstack here.
	 * Requires the kernel be compiled with
	 * frame_pointers on.
	 */
	if(INTERNAL_STATE.have_kernel_frame_pointers){
	    trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	    if(false)
		for(i=0; i<trace_len; ++i){
		    //lookup_symbol_name(trace[i], symname);
		    sprint_symbol(symname, trace[i]);
		    OUTPUT(0, KERN_INFO "SYM MAPPING: 0x%lx --> %s\n", trace[i], symname);
		}
	}
	else{
	    trace_len = 0;
	}
	OUTPUT(0, KERN_INFO "KERNEL-SPACE timer init! Timer_addr = %p\n", timer_addr);
    }
    /*
     * Store the timer if:
     * (a) called for a ROOT process (tid == 0) OR
     * (b) we're actively COLLECTING.
     */
    if(!tid || IS_COLLECTING()){
	// timer_insert((unsigned long)timer_addr, tid);
	DO_PER_CPU_OVERHEAD_FUNC(timer_insert, (unsigned long)timer_addr, tid, pid, tsc, trace_len, trace);
    }
};

// #if (KERNEL_VER < 35)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_init(struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#else
    static void probe_hrtimer_init(void *ignore, struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#endif
{
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_timer_init(struct timer_list *timer)
#else
    static void probe_timer_init(void *ignore, struct timer_list *timer)
#endif
{
    /*
     * Debugging ONLY!
     */
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

/*
 * Interval timer state probe.
 * Fired on interval timer initializations
 * (from "setitimer(...)")
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_itimer_state(int which, const struct itimerval *const value, cputime_t expires)
#else
    static void probe_itimer_state(void *ignore, int which, const struct itimerval *const value, cputime_t expires)
#endif
{
    struct hrtimer *timer = &current->signal->real_timer;

    OUTPUT(3, KERN_INFO "[%d]: ITIMER STATE: timer = %p\n", TID(), timer);
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_start(struct hrtimer *hrt)
#else
    static void probe_hrtimer_start(void *ignore, struct hrtimer *hrt)
#endif
{
    int cpu = CPU();
    pid_t tid = TID();
    pid_t pid = PID();
    u64 tsc = 0;
    /* const char *name = TIMER_START_COMM(hrt); */
    int i, trace_len;
    char symname[KSYM_NAME_LEN];
    unsigned long trace[MAX_BACKTRACE_LENGTH];
    void *sched_timer_addr = NULL;
    per_cpu_t *pcpu = NULL;
    bool should_unregister = false;


    if(!should_probe_on_hrtimer_start){
	OUTPUT(3, KERN_INFO "HRTIMER_START: timer = %p\n", hrt);
	return;
    }

    /*
     * Not sure if "save_stack_trace" or "sprint_symbol" can
     * sleep. To be safe, use the "__get_cpu_var" variants
     * here. Note that it's OK if they give us stale values -- we're
     * not looking for an exact match.
     */
    if(tid || local_read(&__get_cpu_var(sched_timer_found)))
	return;

    /*
     * Basic algo: generate a backtrace for this hrtimer_start
     * tracepoint. Then generate symbolic information for each
     * entry in the backtrace array. Check these symbols.
     * If any one of these symbols is equal to "cpu_idle" then
     * we know that this timer is the "tick" timer for this
     * CPU -- store the address (and the backtrace) in
     * the trace map (and also note that we have, in fact, found
     * the tick timer so that we don't repeat this process again).
     */

    if(INTERNAL_STATE.have_kernel_frame_pointers){
	trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	OUTPUT(0, KERN_INFO "[%d]: %.20s TIMER_START for timer = %p. trace_len = %d\n", tid, TIMER_START_COMM(hrt), hrt, trace_len);
	for(i=0; i<trace_len; ++i){
	    sprint_symbol(symname, trace[i]);
	    OUTPUT(3, KERN_INFO "SYM MAPPING: 0x%lx --> %s\n", trace[i], symname);
	    if(strstr(symname, "cpu_idle")){
		printk(KERN_INFO "FOUND CPU IDLE for cpu = %d . TICK SCHED TIMER = %p\n", cpu, hrt);
		local_inc(&__get_cpu_var(sched_timer_found));
		// *timer_found = true;
		sched_timer_addr = hrt;
	    }
	}
    }else{
	OUTPUT(0, KERN_INFO "NO TIMER STACKS!\n");
    }

    if(sched_timer_addr){
	/*
	 * OK, use the safer "get_cpu_var(...)" variants
	 * here. These disable interrupts.
	 */
	pcpu = &get_cpu_var(per_cpu_counts);
	{
	    cpu = CPU();
	    /*
	     * Races should *NOT* happen. Still, check
	     * to make sure.
	     */
	    if(!pcpu->sched_timer_addr){
		pcpu->sched_timer_addr = sched_timer_addr;

		tsc = 0x1 + cpu;

		timer_insert((unsigned long)sched_timer_addr, tid, pid, tsc, trace_len, trace);
		/*
		 * Debugging
		 */
		if(!timer_find((unsigned long)sched_timer_addr, tid)){
		    printk(KERN_INFO "ERROR: could NOT find timer %p in hrtimer_start!\n", sched_timer_addr);
		}
	    }
	}
	put_cpu_var(pcpu);

	LOCK(tick_count_lock);
	{
	    if( (should_unregister = (++tick_count == PW_max_num_cpus))){
		printk(KERN_INFO "[%d]: ALL TICK TIMERS accounted for -- removing hrtimer start probe!\n", cpu);
		should_probe_on_hrtimer_start = false;
	    }
	}
	UNLOCK(tick_count_lock);
    }
};

/*
 * Common function to perform some bookkeeping on
 * IRQ-related wakeups (including (HR)TIMER_SOFTIRQs).
 * Records hits and (if necessary) sends i-sample
 * messages to Ring 3.
 */
static void handle_irq_wakeup_i(int cpu, per_cpu_t *pcpu, int irq_num, const char *irq_name, bool was_hit)
{
    /*
     * First, "record" a "hit".
     */
    record_hit_full(pcpu, 0, 0, IRQ, irq_num);
    /*
     * Then send an i-sample instance
     * to Ring 3 (but only if so configured
     * and if this is first time this
     * particular IRQ was seen on the
     * current CPU).
     */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
    {
	int __ret = -1;
	/*
	 * We only cache device names if they
	 * actually caused a C-state
	 * wakeup.
	 */
	if(was_hit){
	    DO_PER_CPU_OVERHEAD_FUNC_RET(__ret, irq_insert, cpu, irq_num, irq_name);
	    /*
	     * Protocol:
	     * (a) if mapping FOUND (and already SENT for THIS CPU): irq_insert returns "OK_IRQ_MAPPING_EXISTS"
	     * (b) if new mapping CREATED (or mapping exists, but NOT SENT for THIS CPU): irq_insert returns "OK_NEW_IRQ_MAPPING_CREATED"
	     * (c) if ERROR: irq_insert returns "ERROR_IRQ_MAPPING"
	     */
	    if(__ret == OK_NEW_IRQ_MAPPING_CREATED){
		/*
		 * Send mapping info to Ring-3.
		 */
		produce_i_sample(cpu, irq_num, irq_name);
	    }else if(__ret == ERROR_IRQ_MAPPING){
		printk(KERN_INFO "ERROR: could NOT insert [%d,%s] into irq list!\n", irq_num, irq_name);
	    }
	}
    }
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS
};

static void timer_expire(void *timer_addr, pid_t tid)
{
    int cpu = -1;
    per_cpu_t *pcpu = NULL;
    pid_t pid = -1;
    tnode_t *entry = NULL;
    u64 tsc = 0;
    bool found = false;
    bool was_hit = false;

    /*
     * Reduce overhead -- do NOT run
     * if user specifies NO C-STATES.
     */
    if(unlikely(!IS_SLEEP_MODE())){
	return;
    }

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * Atomic context => use __get_cpu_var(...) instead of get_cpu_var(...)
     */
    pcpu = &__get_cpu_var(per_cpu_counts);

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
#endif // DO_IOCTL_STATS


    cpu = CPU();

    if( (entry = (tnode_t *)timer_find((unsigned long)timer_addr, tid))){
	pid = entry->pid;
	tsc = entry->tsc;
	found = true;
    }else{
	/*
	 * Couldn't find timer entry -- PID defaults to TID.
	 */
	pid = tid;
	tsc = 0x1;
	OUTPUT(3, KERN_INFO "Warning: [%d]: timer %p NOT found in list!\n", pid, timer_addr);
    }


    was_hit = pcpu->debug_enters == 0;

    if(!found){
	tsc = PW_max_num_cpus + 1;
	if(tid < 0){
	    /*
	     * Yes, this is possible, especially if
	     * the timer was fired because of a TIMER_SOFTIRQ.
	     * Special case that here.
	     */
	    if(pcpu->was_timer_hrtimer_softirq > 0){
		/*
		 * Basically, fall back on the SOFTIRQ
		 * option because we couldn't quite figure
		 * out the process that is causing this
		 * wakeup. This is a duplicate of the
		 * equivalent code in "inter_common(...)".
		 */
		int irq_num = pcpu->was_timer_hrtimer_softirq;
		const char *irq_name = pw_softirq_to_name[irq_num];
		OUTPUT(3, KERN_INFO "WARNING: could NOT find TID in timer_expire for Timer = %p: FALLING BACK TO TIMER_SOFTIRQ OPTION! was_hit = %s\n", timer_addr, GET_BOOL_STRING(was_hit));
		handle_irq_wakeup_i(cpu, pcpu, irq_num, irq_name, was_hit);
		/*
		 * No further action is required.
		 */
		return;
	    }
	    else{
		/*
		 * tid < 0 but this was NOT caused
		 * by a TIMER_SOFTIRQ.
		 * UPDATE: this is also possible if
		 * the kernel wasn't compiled with the
		 * 'CONFIG_TIMER_STATS' option set.
		 */
		OUTPUT(0, KERN_INFO "WARNING: NEGATIVE tid in timer_expire!\n");
	    }
	}
    }else{
	/*
	 * OK, found the entry. But timers fired
	 * because of 'TIMER_SOFTIRQ' will have
	 * tid == -1. Guard against that
	 * by checking the 'tid' value. If < 0
	 * then replace with entry->tid
	 */
	if(tid < 0){
	    tid = entry->tid;
	}
    }

    record_hit_full(pcpu, pid, tid, TIMER, tsc);

    /*
     * OK, send the TIMER::TSC mapping & call stack to the user
     * (but only if this is for a kernel-space call stack and the
     * user wants kernel call stack info).
     */
    // if(!pid && INTERNAL_STATE.write_to_buffers && found && !entry->trace_sent){
    // if(!pid && INTERNAL_STATE.write_to_buffers && IS_KTIMER_MODE() && found && !entry->trace_sent){
    if(!pid && (IS_COLLECTING() || IS_SLEEPING()) && IS_KTIMER_MODE() && found && !entry->trace_sent){
	produce_k_sample(cpu, entry);
	entry->trace_sent = 1;
    }
};

/*
 * High resolution timer (hrtimer) expire entry probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_expire_entry(struct hrtimer *hrt, ktime_t *now)
#else
    static void probe_hrtimer_expire_entry(void *ignore, struct hrtimer *hrt, ktime_t *now)
#endif
{
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, hrt, TIMER_START_PID(hrt));
};

/*
 * Macro to determine if the given
 * high resolution timer is periodic.
 */
#define IS_INTERVAL_TIMER(hrt) ({					\
	    bool __tmp = false;						\
	    pid_t pid = TIMER_START_PID(hrt);				\
	    ktime_t rem_k = hrtimer_expires_remaining(hrt);		\
	    s64 remaining = rem_k.tv64;					\
	    /* We first account for timers that */			\
	    /* are explicitly re-enqueued. For these */			\
	    /* we check the amount of time 'remaining' */		\
	    /* for the timer i.e.  how much time until */		\
	    /* the timer expires. If this is POSITIVE ==> */		\
	    /* the timer will be re-enqueued onto the */		\
	    /* timer list and is therefore PERIODIC */			\
	    if(remaining > 0){						\
		__tmp = true;						\
	    }else{							\
		/* Next, check for 'itimers' -- these INTERVAL TIMERS are */ \
		/* different in that they're only re-enqueued when their */ \
		/* signal (i.e. SIGALRM) is DELIVERED. Accordingly, we */ \
		/* CANNOT check the 'remaining' time for these timers. Instead, */ \
		/* we compare them to an individual task's 'REAL_TIMER' address.*/ \
		/* N.B.: Call to 'pid_task(...)' influenced by SEP driver code */ \
		struct task_struct *tsk = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID); \
		__tmp = (tsk && ( (hrt) == &tsk->signal->real_timer));	\
	    }								\
	    __tmp; })


/*
 * High resolution timer (hrtimer) expire exit probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_expire_exit(struct hrtimer *hrt)
#else
    static void probe_hrtimer_expire_exit(void *ignore, struct hrtimer *hrt)
#endif
{
    if(!IS_INTERVAL_TIMER(hrt)){
        /*
         * timers are run from hardirq context -- no need
         * for expensive 'get_cpu_var(...)' variants.
         */
        per_cpu_t *pcpu = &__get_cpu_var(per_cpu_counts);
        /*
         * REMOVE the timer from
         * our timer map here (but
         * only if this isn't a 'sched_tick'
         * timer!)
         */
        if((void *)hrt != pcpu->sched_timer_addr){
            int ret = -1;
            DO_PER_CPU_OVERHEAD_FUNC_RET(ret, timer_delete, (unsigned long)hrt, TIMER_START_PID(hrt));
            if(ret){
                OUTPUT(0, KERN_INFO "WARNING: could NOT delete timer mapping for HRT = %p, TID = %d, NAME = %.20s\n", hrt, TIMER_START_PID(hrt), TIMER_START_COMM(hrt));
            }else{
                OUTPUT(3, KERN_INFO "OK: DELETED timer mapping for HRT = %p, TID = %d, NAME = %.20s\n", hrt, TIMER_START_PID(hrt), TIMER_START_COMM(hrt));
                // debugging ONLY!
                if(timer_find((unsigned long)hrt, TIMER_START_PID(hrt))){
                    printk(KERN_INFO "WARNING: TIMER_FIND reports TIMER %p STILL IN MAP!\n", hrt);
                }
            }
        }
    }
};

#define DEFERRABLE_FLAG (0x1)
#define IS_TIMER_DEFERRABLE(t) ( (unsigned long)( (t)->base) & DEFERRABLE_FLAG )


/*
 * Timer expire entry probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_timer_expire_entry(struct timer_list *t)
#else
    static void probe_timer_expire_entry(void *ignore, struct timer_list *t)
#endif
{
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, t, TIMER_START_PID(t));
};


/*
 * Function common to all interrupt tracepoints.
 */
static void inter_common(int irq_num, const char *irq_name)
{
    per_cpu_t *pcpu = NULL;

    bool was_hit = false;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * Reduce overhead -- do NOT run
     * if user specifies NO C-STATES.
     */
    if(unlikely(!IS_SLEEP_MODE())){
        return;
    }

    /*
     * Debugging: make sure we're in
     * interrupt context!
     */
    if(!in_interrupt()){
        printk(KERN_ERR "BUG: inter_common() called from a NON-INTERRUPT context! Got irq: %lu and soft: %lu\n", in_irq(), in_softirq());
        return;
    }

    /*
     * Interrupt context: no need for expensive "get_cpu_var(...)" version.
     */
    pcpu = &__get_cpu_var(per_cpu_counts);

    /*
     * If this is a TIMER or an HRTIMER SOFTIRQ then
     * DO NOTHING (let the 'timer_expire(...)'
     * function handle this for greater accuracy).
     */
    if(irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ){
	pcpu->was_timer_hrtimer_softirq = irq_num;
#if DO_IOCTL_STATS
	/*
	 * Increment counter for timer interrupts as well.
	 */
	local_inc(&pstats->num_timers);
#endif // DO_IOCTL_STATS
	OUTPUT(3, KERN_INFO "(HR)TIMER_SOFTIRQ: # = %d\n", irq_num);
	return;
    }

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
    local_inc(&pstats->num_inters);

    /*
     * Increment counter for timer interrupts as well.
     */
    if(in_softirq() && (irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ))
        local_inc(&pstats->num_timers);
#endif

    /*
     * Check if this interrupt caused a C-state
     * wakeup (we'll use that info to decide
     * whether to cache this IRQ # <-> DEV name
     * mapping).
     */
    was_hit = pcpu->debug_enters == 0;

    /*
     * OK, record a 'hit' (if applicable) and
     * send an i-sample message to Ring 3.
     */
    handle_irq_wakeup_i(CPU(), pcpu, irq_num, irq_name, was_hit);
};

/*
 * IRQ tracepoint.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_irq_handler_entry(int irq, struct irqaction *action)
#else
    static void probe_irq_handler_entry(void *ignore, int irq, struct irqaction *action)
#endif
{
    const char *name = action->name;
    OUTPUT(3, KERN_INFO "NUM: %d\n", irq);
    // inter_common(irq);
    DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};

/*
 * soft IRQ tracepoint.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_softirq_entry(struct softirq_action *h, struct softirq_action *vec)
#else
    static void probe_softirq_entry(void *ignore, struct softirq_action *h, struct softirq_action *vec)
#endif
{
    int irq = -1;
    const char *name = NULL;
    irq = (int)(h-vec);
    name = pw_softirq_to_name[irq];

    OUTPUT(3, KERN_INFO "NUM: %d\n", irq);

    DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};
#else // >= 2.6.38
static void probe_softirq_entry(void *ignore, unsigned int vec_nr)
{
	int irq = (int)vec_nr;
	const char *name = pw_softirq_to_name[irq];

	DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_workqueue_execution(struct task_struct *wq_thread, struct work_struct *work)
#else
static void probe_workqueue_execution(void * ignore, struct task_struct *wq_thread, struct work_struct *work)
#endif // < 2.6.35
{
    per_cpu_t *pcpu = NULL;
    pcpu = &__get_cpu_var(per_cpu_counts);
    record_hit_full(pcpu, 0, 0, WORKQUEUE, 1);

    if (false) {
        printk(KERN_INFO "WORKQUEUE!");
    }
};
#else // >= 2.6.36
static void probe_workqueue_execute_start(void *ignore, struct work_struct *work)
{
    per_cpu_t *pcpu = NULL;
    pcpu = &__get_cpu_var(per_cpu_counts);
    record_hit_full(pcpu, 0, 0, WORKQUEUE, 1);
};
#endif // < 2.6.36

#define C1 APERF
#define CN MAX_MSR_ADDRESSES

#define UNHLT_REF MPERF

static u64 read_one_residency(int cpu, int msr_addr, const char *msr_name, u64 *prev)
{
    u64 curr = 0, delta = 0;

    rdmsrl(msr_addr, curr);

    if(unlikely(curr < *prev)){
	printk(KERN_INFO "ERROR: CPU %d: ROLLOVER in %s! PREV = %llX, CURR = %llX\n", cpu, msr_name, *prev, curr);

	delta = ( (u64)(~0) - *prev ) + (curr + 1);
    }else{
	delta = curr - *prev;
    }

    *prev = curr;

    return delta;
};

/*
 * UPDATE: Use following formula to calculate RES_Cx:
 * (a) OMEGA = T2 - T1
 * (b) BETA = TSC(t2) - TSC(m2)
 * (c) ALPHA = TSC(t1) - TSC(m1)
 * (d) RES_Cx == THETA = (OMEGA - RES_MPERF) + (ALPHA - BETA)
 * (e) RES_C0 = OMEGA - THETA == RES_MPERF + (BETA - ALPHA)
 * Here ALPHA, BETA are the overheads incurred within the residency reading
 * function, and may be calculated by subtracting the TSC just before the MPERF is read
 * from the sample collection TSC.
 */
static void read_residencies(per_cpu_t *pcpu, int cpu)
{
    int i=0, which_c_state = 0, num_deltas = 0;
    u64 prev;
    int msr_addr;
    u64 begin_tsc, end_tsc;
    u64 alpha, beta;
    u64 OMEGA, theta, c0;
    u64 m_delta, c_delta;
    bool is_first = false;
    u64 cx_total = 0;
    u32 clock_multiplier = INTERNAL_STATE.residency_count_multiplier;


    /*
     * Ensure updates are propagated.
     */
    smp_mb();

    is_first = false;

    if(unlikely(PREV_MSR_VAL(pcpu, MPERF) == 0)){
	is_first = true;
    }


    tscval(&begin_tsc);

    msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[MPERF];
    prev = PREV_MSR_VAL(pcpu, MPERF);
    /*
     * Read MPERF, compute DELTA(MPERF)
     */
    m_delta = read_one_residency(cpu, msr_addr, msr_names[MPERF], &prev);

    PREV_MSR_VAL(pcpu, MPERF) = prev;
    /*
     * 'C1' is a DERIVED residency -- we
     * don't read MSRs for it. Instead, we
     * compute its value from the values of
     * OTHER Cx/MPERF/TSC. Reset to zero here.
     */
    RESIDENCY(pcpu, C1) = 0;
    /*
     * Calculate (non-C1) C-state residency.
     */
    for(i=C2; i<CN; ++i){
	RESIDENCY(pcpu, i) = 0;

	if( (msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[i]) <= 0)
	    continue;

	prev = PREV_MSR_VAL(pcpu, i);
	{
	    /*
	     * Safe to compute DELTA here, even on
	     * a first read. The 'reset_per_cpu_msr_residencies()' function
	     * SHOULD have set ALL PREV_MSR_VALs
	     * to ZERO!
	     */
	    c_delta = read_one_residency(cpu, msr_addr, msr_names[i], &prev);
	}
	PREV_MSR_VAL(pcpu, i) = prev;

	if(!is_first && c_delta){
	    ++num_deltas;

	    if(!which_c_state)
		which_c_state = i;

	    OUTPUT(3, KERN_INFO "c_delta = %llu, multiplier = %u, ACTUAL = %llu\n", c_delta, clock_multiplier, (c_delta * clock_multiplier));

	    c_delta *= clock_multiplier;

	    RESIDENCY(pcpu, i) = c_delta;

	    cx_total += c_delta;
	}
    }
    /*
     * Sanity check: we should have at most one
     * C-state residency (could be zero -- in
     * which case we assume the previous break
     * was a C-1 break).
     */
    if(num_deltas > 1){
	OUTPUT(0, KERN_INFO "[%d]: # deltas = %d\n", cpu, num_deltas);
    }

    /*
     * Almost done. Calculate overhead components (only
     * required if we need to calculate C1 residency in
     * the current or next C-state break).
     */
    tscval(&end_tsc);
    OMEGA = end_tsc - pcpu->tsc; /* TSC delta */
    pcpu->tsc = end_tsc;
    beta = end_tsc - begin_tsc; /* Overhead of current read residency call */
    alpha = pcpu->old_alpha; /* Overhead of previous read residency call */
    pcpu->old_alpha = beta;
    /*
     * OK, check to see if we need to compute the
     * (approximate) C1 residency.
     */
    if(!is_first){
	if(!(theta = cx_total)){
	    /*
	     * None of {C3,C6,C7} had non-zero residencies. This
	     * must therefore have been a C1 break.
	     */
	    theta = (OMEGA - m_delta) + (alpha - beta);
	    if((long long)theta < 0){
		/*
		 * Yes, this is still possible, particularly in the case
		 * when, for example, the processor dropped into C1 and
		 * then was (almost immediately) interrupted. In this case,
		 * the C1 residency is so small as to be swallowed up in
		 * the overheads of the various operations (including the
		 * cost of reading all the MSRs). To debug, we can check this
		 * by looking at how much less than zero 'theta' truly is (this
		 * typically comes out to be less than 75 cycles, which is
		 * the approximate cost of a single read of the TSC). Enable
		 * the following printk to perform this check.
		 */
		OUTPUT(3, KERN_INFO "NEGATIVE! Val = %lld\n", (long long)theta);
		theta = 0;
	    }
	    RESIDENCY(pcpu, C1) = theta;
	}else{
	    /*
	     * OK, this was NOT a C1 break; no
	     * computation required.
	     */
	    if(OMEGA < theta){
		OUTPUT(3, KERN_INFO "[%d]: WARNING: OMEGA is less than THETA! Difference = %llu\n", cpu, (theta-OMEGA));
		theta = OMEGA;
	    }
	}
	/*
	 * Estimate C0. This is an estimate because we can't
	 * be sure we have the correct values for 'Cx' (particularly
	 * in the C1 case).
	 */
	RESIDENCY(pcpu, MPERF) = c0 = OMEGA - theta;
    }
};

/*
 * TPS helper -- required for overhead
 * measurements.
 */
static DEFINE_PER_CPU(u64, num_inters) = 0;

				       /*
					* Basically the same as arch/x86/kernel/irq.c --> "arch_irq_stat_cpu(cpu)"
					*/

#define GET_APIC_TIMER_IRQ_COUNT() ( (&__get_cpu_var(irq_stat))->apic_timer_irqs )

				       static u64 my_local_arch_irq_stat_cpu(void)
{
    u64 sum = 0;
    irq_cpustat_t *stats;

    BEGIN_LOCAL_IRQ_STATS_READ(stats);
    {
	// sum = stats->__nmi_count;

#ifdef CONFIG_X86_LOCAL_APIC
	sum += stats->apic_timer_irqs;
	/*
	  sum += stats->irq_spurious_count;
	  sum += stats->apic_perf_irqs;
	  sum += stats->apic_pending_irqs;
	*/
#endif
#ifdef CONFIG_SMP
	sum += stats->irq_resched_count;
	sum += stats->irq_call_count;
	sum += stats->irq_tlb_count;
#endif
	/*
	  #ifdef CONFIG_X86_THERMAL_VECTOR
	  sum += stats->irq_thermal_count;
	  #endif
	  #ifdef CONFIG_X86_MCE_THRESHOLD
	  sum += stats->irq_threshold_count;
	  #endif
	*/

    }
    END_LOCAL_IRQ_STATS_READ(stats);

    return sum;

};

static void tps(unsigned int type, unsigned int state)
{
    per_cpu_t *pcpu = NULL;
    int cpu = -1;
    int last_PID = -1, last_TID = -1;
    unsigned long long last_IRQ = 0, last_TIMER = 0, last_SCHED = 0, last_WORK = 0;
    char sample_type = 'I';
    unsigned long long sample_data = 0;
    bool other_irq_diff = false;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * "get_cpu_var(...)" internally calls "preempt_disable()"
     * See <include/linux/percpu.h>
     */
    pcpu = &get_cpu_var(per_cpu_counts);
    /*
     * "get_cpu_var" already disabled inters for us.
     * Use "__get_cpu_var" versions now.
     */
#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
    {
	local_inc(&pstats->c_breaks);
    }
#endif // DO_IOCTL_STATS

    cpu = CPU();
    // pkg = 0;


    if(true){
	u64 new_inters = my_local_arch_irq_stat_cpu();
	// u64 new_inters = GET_APIC_TIMER_IRQ_COUNT();
	u64 *old_inters = &__get_cpu_var(num_inters);
	if(*old_inters && (*old_inters != new_inters)){
	    OUTPUT(1, KERN_INFO "[%d]: # SMP DIFFERENT! Diff = %llu\n", cpu, (new_inters - *old_inters));
	    other_irq_diff = true;
	}
	*old_inters = new_inters;
    }


    {
	read_residencies(pcpu, cpu);
    }

    /*
     * Reset the "debug_enters" counter.
     * This signals the next timer/interrupt (i.e. C-state break) event
     * to collect information.
     */
    pcpu->debug_enters = 0;

    last_PID = pcpu->last_pid;
    last_TID = pcpu->last_tid;
    last_IRQ = pcpu->last_break[IRQ];
    last_TIMER = pcpu->last_break[TIMER];
    last_WORK = pcpu->last_break[WORKQUEUE];
    last_SCHED = pcpu->last_break[SCHED];

    put_cpu_var(pcpu[cpu]);


    if( (sample_data = last_IRQ) > 0){
	/*
	 * C-state break was caused by an interrupt.
	 */
	// sample_type = 'I';
	sample_type = PW_BREAK_TYPE_I;

#if DO_IOCTL_STATS
	local_inc(&pstats->inters_c_breaks);
#endif
	/*
	 * "last_PID/TID" doesn't make sense here:
	 * it refers to the executing process when
	 * the interrupt was serviced, not necessarily
	 * the process that *CAUSED* the interrupt.
	 */
	last_PID = 0; last_TID = 0;
    }else if( (sample_data = last_TIMER) > 0){ // We're using the last_TIMER to store TSC values on timer breaks: ALWAYS > 0!
	/*
	 * C-state break was caused by a timer.
	 */

	// sample_type = 'T';
	sample_type = PW_BREAK_TYPE_T;

#if DO_IOCTL_STATS
	local_inc(&pstats->timer_c_breaks);
#endif

    }
	else if ( (sample_data = last_WORK) > 0) {
        sample_type = PW_BREAK_TYPE_W;
    }
	else if( (sample_data = last_SCHED) > 0){
		/*
		 * 'last_SCHED' is "source_cpu+1". Adjust
		 * for the "+1" by decrementing 'sample_data'
		 * here.
		 */
		--sample_data;
		sample_type = 'S';
		OUTPUT(3, KERN_INFO "SCHED BREAK!\n");
		sample_type = PW_BREAK_TYPE_S;
	}else if(other_irq_diff){
	/*
	 * We speculate here -- the
	 * wakeup was caused by one of
	 * the following (see '/proc/interrupts'
	 * for meanings):
	 * {LOC,RES,CALL,TLB}
	 */
	// sample_type = 'O';
	sample_type = PW_BREAK_TYPE_IPI;
	sample_data = 0;
    }else{
	/*
	 * Unknown reason for C-state break.
	 */
	// sample_type = '?';
	sample_type = PW_BREAK_TYPE_U;
	sample_data = 0;
	OUTPUT(1, KERN_INFO "UNKNOWN C-STATE BREAK!\n");
	if(false){
	    printk(KERN_INFO "[%d]: UNKNOWN! Other irq = %s\n", cpu, GET_BOOL_STRING(other_irq_diff));
	}
    }

    OUTPUT(3, KERN_INFO "[%d]: Break sample type = %c\n", cpu, sample_type);

    if(INTERNAL_STATE.write_to_buffers){
	produce_c_sample(cpu, pcpu, last_PID, last_TID, sample_type, sample_data);
    }

    pcpu->last_pid = pcpu->last_tid = -1;
    pcpu->last_break[WORKQUEUE] = pcpu->last_break[IRQ] = pcpu->last_break[TIMER] =  pcpu->last_break[SCHED] = 0;
    pcpu->was_timer_hrtimer_softirq = 0;

    pcpu->prev_state = state;

    // Collect S and D state / residency counter samples on CPU0
    if(cpu != 0 || !IS_COLLECTING())
        return;

// Produce S state residency counter sample
// We expect that S state transition will not occur
// as many as C state transition.
// Therefore, we produce the S samples only if the C state
// was in the deepest state (C6 in Medfield)
#if DO_S_RESIDENCY_SAMPLE
        if(PW_is_atm && IS_S_RESIDENCY_MODE() && RESIDENCY(pcpu, C6) > 0){
            u64 usec = dump_s_residency_counter();
            produce_s_residency_sample(usec);
        }
#endif

#if DO_S_STATE_SAMPLE
    if(PW_is_atm && IS_S_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_s_state_sample();
        }
    }
#endif

    /*
     * Controls the sampling frequency to reduce data collection overheads 
     */
    if ((CURRENT_TIME_IN_USEC() - prev_sample_usec) > INTERNAL_STATE.d_state_sample_interval*1000) {
        prev_sample_usec = CURRENT_TIME_IN_USEC();
// Produce D state residency counter sample
#if DO_D_SC_RESIDENCY_SAMPLE
        if(PW_is_atm && IS_D_SC_RESIDENCY_MODE()){
            u64 usec = dump_d_sc_residency_counter();
            if (usec > 0)
                produce_d_sc_residency_sample(usec);
        }
#endif

#if DO_D_NC_STATE_SAMPLE
        if(PW_is_atm && IS_D_NC_STATE_MODE()){
            if(INTERNAL_STATE.write_to_buffers) {
                produce_d_nc_state_sample();
            }
        }
#endif

#if DO_D_SC_STATE_SAMPLE
        if(PW_is_atm && IS_D_SC_STATE_MODE()){
            if(INTERNAL_STATE.write_to_buffers) {
                produce_d_sc_state_sample();
            }
        }
#endif
    }
};

/*
 * C-state break.
 * Read MSR residencies.
 * Also gather information on what caused C-state break.
 * If so configured, write C-sample information to (per-cpu)
 * output buffer.
 */
#if APWR_RED_HAT
/*
 * Red Hat back ports SOME changes from 2.6.37 kernel
 * into 2.6.32 kernel. Special case that here.
 */
static void probe_power_start(unsigned int type, unsigned int state, unsigned int cpu_id)
{
   if(unlikely(!IS_SLEEP_MODE())){
	return;
    }

    // tps_i(type, state);
    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
};
#else
#if LINUX_VERSION_CODE  < KERNEL_VERSION(2,6,38)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_power_start(unsigned int type, unsigned int state)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static void probe_power_start(void *ignore, unsigned int type, unsigned int state)
#else // 2.6.36 <= version < 2.6.38
static void probe_power_start(void *ignore, unsigned int type, unsigned int state, unsigned int cpu_id)
#endif
{
    if(unlikely(!IS_SLEEP_MODE())){
	return;
    }

    // tps_i(type, state);
    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
};
#else // version >= 2.6.38
static void probe_cpu_idle(void *ignore, unsigned int state, unsigned int cpu_id)
{
   if(unlikely(!IS_SLEEP_MODE())){
	return;
    }

   if (state == PWR_EVENT_EXIT) {
       return;
   }

    // tps_i(type, state);
   DO_PER_CPU_OVERHEAD_FUNC(tps, 0 /*type*/, state);
};
#endif // version
#endif // APWR_RED_HAT

/*
 * Tokenize the Frequency table string
 * to extract individual frequency 'steps'
 */
int extract_valid_frequencies(const char *buffer, ssize_t len)
{
    const char *str = NULL;
    char tmp[10];
    int num_toks = 0, i=0, j=0, tmp_len = 0;
    unsigned long freq = 0;

    if(len <= 0 || !buffer)
	return -ERROR;

    /*
     * Step-1: find out number of
     * frequency steps
     */
    for(i=0; i<len; ++i)
	if(buffer[i] == ' ')
	    ++num_toks;

    /*
     * We don't keep a separate "len"
     * field to indicate # of freq
     * steps. Instead, we write a
     * ZERO in the last entry of the
     * 'apwr_available_frequencies' table.
     * Increase the size of the table to
     * accommodate this.
     */
    ++num_toks;

    /*
     * Step-2: allocate memory etc.
     */
    if(!(apwr_available_frequencies = pw_kmalloc(sizeof(u32) * num_toks, GFP_KERNEL))){
	printk(KERN_INFO "ERROR: could NOT allocate memory for apwr_available_frequencies array!\n");
	return -ERROR;
    }

    /*
     * Step-3: extract the actual frequencies.
     */
    for(i=0, j=0, str = buffer; i<len; ++i){
	if(buffer[i] == ' '){
	    memset(tmp, 0, sizeof(tmp));
	    ++num_toks;
	    if( (tmp_len = (buffer+i) - str) > 10){
		// ERROR!
		return -ERROR;
	    }
	    strncpy(tmp, str, min(sizeof(tmp), tmp_len));
	    // fprintf(stderr, "TOKEN = %d\n", atoi(tmp));
	    freq = simple_strtoul(tmp, NULL, 10);
	    apwr_available_frequencies[j++] = (u32)freq;
	    OUTPUT(0, KERN_INFO "FREQ-STEP: %lu\n", freq);
	    str = buffer + i + 1;
	}
    }
    /*
     * Now write the ZERO entry
     * denoting EOF
     */
    apwr_available_frequencies[j] = 0x0;
    /*
     * Special support for MFLD -- if we
     * couldn't get the 'base_operating_freq'
     * then set that now.
     */
    if(true && !base_operating_freq){
	printk(KERN_INFO "WARNING: no \"base_operating_frequency\" set -- using %u\n", apwr_available_frequencies[0]);
	base_operating_freq = apwr_available_frequencies[0];
    }
    return SUCCESS;
};

#define IS_BRACKETED_BY(upper, lower, freq) ({bool __tmp = ( (upper) >= (freq) && (freq) >= (lower) ) ? true : false; __tmp;})
#define GET_CORRECT_FREQ_BOUND(upper, lower, freq) ({u32 __delta_upper = (upper) - (freq), __delta_lower = (freq) - (lower), __tmp = (__delta_upper < __delta_lower) ? (upper) : (lower); __tmp;})

/*
 * Given a frequency calculated using the
 * 'aperf/mperf' ratio, try and figure out
 * what the ACTUAL frequency was.
 */
unsigned long get_actual_frequency(unsigned long avg_freq)
{
    /*
     * Basic algo: iterate through array of
     * available frequencies, comparing 'avg_freq'
     * to each. Stop at FIRST frequency which
     * is LESS than 'avg_freq' (i.e. we're ROUNDING
     * DOWN the 'avg_freq').
     *
     * Note that we should probably be trying to
     * find which of the 'available' frequencies
     * is CLOSEST to 'avg_freq'. For now,
     * we rely on this more primitive method.
     *
     * UPDATE: we're now finding the CLOSEST
     * frequency to the 'avg_freq' value.
     */
    int i=0;

    if(!apwr_available_frequencies || apwr_available_frequencies[0] == 0)
	return 0;
    /*
     * SPECIAL CASE FOR TURBO frequencies:
     * If 'avg_freq' > FIRST entry in array
     * (i.e. the TURBO entry) then just
     * return 'avg_freq'
     * UPDATE: NO -- for now we return the
     * TSC freq + 1 MHz i.e. the first TURBO
     * freq
     */
    /*
    if(avg_freq > apwr_available_frequencies[0])
	return avg_freq;
        */
    if (avg_freq >= apwr_available_frequencies[0]) {
        return apwr_available_frequencies[0]; // in Hz
    }

    /*
     * Try and find the frequency STEP that's
     * closest to 'avg_freq'
     */
    for(i=1; apwr_available_frequencies[i] != 0; ++i){
	u32 upper = apwr_available_frequencies[i-1], lower = apwr_available_frequencies[i];
	if(IS_BRACKETED_BY(upper, lower, avg_freq)){
	    u32 act_freq = GET_CORRECT_FREQ_BOUND(upper, lower, avg_freq);
	    OUTPUT(3, KERN_INFO "AVG = %lu, UPPER = %u, LOWER = %u, CLOSEST = %u, \n", avg_freq, upper, lower, act_freq);
	    return act_freq;
	}
    }

    /*
     * OK, couldn't find a suitable freq -- just
     * return the 'avg_freq'.
     * UPDATE: reaching here implies a frequency
     * LOWER than the LOWEST frequency step. In this
     * case, return this LOWEST frequency.
     */
    return apwr_available_frequencies[--i];
};

#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
static void calc_freq_using_aperf_mperf_ratio(u64 delta_aperf, u64 delta_mperf, u32 *calc_freq, u32 *act_freq)
{
    unsigned long ratio = 0;
    ratio = delta_aperf;
    /*
     * We can't do FP math ops in the kernel
     * so ensure 'delta_mperf' (i.e. the devisor)
     * is LESS than 'delta_aperf' (i.e. the
     * dividend). This ensures the quotient
     * is ALWAYS greater than 1.0. To ensure this,
     * shift the 'delta_mperf' by a known
     * amount.
     */
    delta_mperf >>= APERFMPERF_SHIFT;
    if(delta_mperf){
	ratio = div64_u64(delta_aperf, delta_mperf);
    }

    *calc_freq = ratio * base_operating_freq;
    /*
     * Adjust for the earlier divisor
     * shift.
     */
    *calc_freq >>= APERFMPERF_SHIFT;
    /*
     * 'calc_freq' is an AVERAGE freq -- try
     * and convert that to one of the
     * frequency 'steps' calculated
     * earlier (because users expect to
     * see one of the frequency steps and not
     * an average frequency).
     */
    *act_freq = get_actual_frequency(*calc_freq);
};

static void tpf(unsigned int type, unsigned int state)
{
    int cpu = CPU();
    u64 tsc = 0;
    u64 curr_aperf=0, curr_mperf=0;
    u64 prev_aperf=0, prev_mperf=0;
    u64 delta_aperf = 0, delta_mperf = 0;
    u32 calc_freq = 0, act_freq = 0, prev_req_freq = 0, prev_act_freq = 0;

    per_cpu_freq_data_t *pdata = &__get_cpu_var(pcpu_freq_data);

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * Read TSC value
     */
    tscval(&tsc);

    prev_aperf = pdata->aperf; prev_mperf = pdata->mperf;
    prev_req_freq = pdata->prev_req_freq;
    pdata->prev_req_freq = state;

    prev_act_freq = pdata->prev_act_freq;
    // prev_freq = pdata->freq;

    rdmsrl(CORE_CYCLES_MSR_ADDR, curr_aperf); rdmsrl(REF_CYCLES_MSR_ADDR, curr_mperf);

    pdata->aperf = curr_aperf; pdata->mperf = curr_mperf;

    /*
     * TODO: check for rollovers.
     */
    delta_aperf = curr_aperf - prev_aperf; delta_mperf = curr_mperf - prev_mperf;
    OUTPUT(0, KERN_INFO "[%d]: P-STATE transition: state = %u, DELTA-aperf = %llu, DELTA-mperf = %llu\n", cpu, state, delta_aperf, delta_mperf);

    /*
     * The following freq calculation
     * methodology is taken from
     * 'arch/x86/kernel/cpu/cpufreq/mperf.c'
     * and from
     * 'arch/x86/include/asm/processor.h'
     */
    calc_freq_using_aperf_mperf_ratio(delta_aperf, delta_mperf, &calc_freq, &act_freq);
    pdata->prev_act_freq = act_freq;

    /*
     * OK, check current frequency with
     * previous frequency to see if
     * anything needs to be transmitted
     * to Ring-3
     */
    if(act_freq != prev_act_freq){
	if(INTERNAL_STATE.write_to_buffers){
	    /*
	     * Transmit sample to Ring-3!
	     * (Ensure we transmit the
	     * 'act_freq' and NOT the 'calc_freq'
	     * value -- users expect to see
	     * one of the frequency steps and
	     * not an average frequency).
	     */
	    // printk(KERN_INFO "[%d]: P-STATE TRANSITION: [%u] --> [%u] (%u, %u)\n", cpu, prev_act_freq, calc_freq, act_freq, state);
	    OUTPUT(3, KERN_INFO "[%d]: PREV_REQ = %u, ACT = %u, NEXT_REQ = %u\n", cpu, prev_req_freq, act_freq, state);
	    produce_p_sample(cpu, tsc, prev_req_freq, act_freq, 0); // "0" ==> NOT a boundary sample
	}
    }else{
	OUTPUT(0, KERN_INFO "[%d]: P-STATE NON-TRANS: [%u] --> [%u] (%u, %u)\n", cpu, prev_act_freq, calc_freq, act_freq, state);
    }

#if DO_IOCTL_STATS
    {
	pstats = &get_cpu_var(per_cpu_stats);
	local_inc(&pstats->p_trans);
	put_cpu_var(pstats);
    }
#endif // DO_IOCTL_STATS

};
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT

#if DO_CPUFREQ_NOTIFIER
/*
 * CPUFREQ notifier callback function.
 * Used in cases where the default
 * power frequency tracepoint mechanism
 * is broken (e.g. MFLD).
 */
static int apwr_cpufreq_notifier(struct notifier_block *block, unsigned long val, void *data)
{
    struct cpufreq_freqs *freq = data;
    u32 state = freq->new; // "state" is frequency CPU is ABOUT TO EXECUTE AT

    if(unlikely(!IS_FREQ_MODE())){
	return SUCCESS;
    }

    if(val == CPUFREQ_PRECHANGE){
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
	{
	    DO_PER_CPU_OVERHEAD_FUNC(tpf, 2, state);
	}
#else
	{
	    /*
	     * Legacy method -- use the 'state'
	     * parameter to determine frequency
	     * the proc is ABOUT TO EXECUTE AT.
	     * THIS IS DEPRECATED!!!
	     */
	    int cpu = CPU();
	    u64 tsc;
	    OUTPUT(3, KERN_INFO "[%d]: FREQS = %p\n", cpu, freq);
	    tscval(&tsc);
	    produce_p_sample(cpu, tsc, state, 0); // "0" ==> NOT boundary sample
	}
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
    }
    return SUCCESS;
};

static struct notifier_block apwr_cpufreq_notifier_block = {
    .notifier_call = &apwr_cpufreq_notifier
};

#else // DO_CPUFREQ_NOTIFIER

/*
 * P-state transition probe.
 *
 * "type" is ALWAYS "2" (i.e. "POWER_PSTATE", see "include/trace/power.h")
 * "state" is the NEXT frequency range the CPU is going to enter (see "arch/x86/kernel/cpu/cpufreq/acpi-cpufreq.c")
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_power_frequency(unsigned int type, unsigned int state)
#else
    static void probe_power_frequency(void *ignore, unsigned int type, unsigned int state)
#endif
{
    if(unlikely(!IS_FREQ_MODE())){
	return;
    }
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
    {
	DO_PER_CPU_OVERHEAD_FUNC(tpf, type, state);
    }
#else
    {
	/*
	 * Legacy method -- use the 'state'
	 * parameter to determine frequency
	 * the proc is ABOUT TO EXECUTE AT.
	 * THIS IS DEPRECATED!!!
	 */
	int cpu = CPU();
	u64 tsc;
	tscval(&tsc);
	produce_p_sample(cpu, tsc, state, 0); // "0" ==> NOT boundary sample
    }
#endif
};

#endif // DO_CPUFREQ_NOTIFIER


/*
 * Helper function for "probe_sched_exit"
 * Useful for overhead measurements.
 */
static void exit_helper(struct task_struct *task)
{
    pid_t tid = task->pid;
    if(false){
	printk(KERN_INFO "WARNING: NOT deleting timers for EXITTING TID = %d\n", tid);
	return;
    }
    OUTPUT(3, KERN_INFO "[%d]: SCHED_EXIT\n", tid);

    delete_timers_for_tid(tid);

};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_process_exit(struct task_struct *task)
#else
    static void probe_sched_process_exit(void *ignore, struct task_struct *task)
#endif
{
    pid_t tid = task->pid, pid = task->tgid;
    const char *name = task->comm;
    u64 tsc;


    OUTPUT(3, KERN_INFO "[%d, %d]: %s exitting\n", tid, pid, name);

    DO_PER_CPU_OVERHEAD_FUNC(exit_helper, task);

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING!
     * UPDATE: track if COLLECTION ONGOING OR
     * IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);

    produce_r_sample(CPU(), tsc, PW_PROC_EXIT, tid, pid, name);
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_wakeup(struct rq *rq, struct task_struct *task, int success)
#else
static void probe_sched_wakeup(void *ignore, struct task_struct *task, int success)
#endif
{
    int target_cpu = task_cpu(task), source_cpu = CPU();
    per_cpu_t *pcpu = NULL;

    if (source_cpu == target_cpu) {
        pcpu = &__get_cpu_var(per_cpu_counts);
    } else {
        pcpu = &per_cpu(per_cpu_counts, target_cpu);
    }
    record_hit_full(pcpu, 0, 0, SCHED, (source_cpu+1)); // we add "1" to "source_cpu" because we need a non-zero value in "last_break"!!!
};

#if DO_PROBE_ON_EXEC_SYSCALL
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sys_exit(struct pt_regs *regs, long ret)
#else
static void probe_sys_exit(void *ignore, struct pt_regs *regs, long ret)
#endif
{
    long id = syscall_get_nr(current, regs);
    u64 tsc;

    tscval(&tsc);

    if(id == __NR_execve){
        printk(KERN_INFO "[%d]: EXECVE ENTER! TID = %d, NAME = %.20s\n", CPU(), TID(), NAME());
        produce_r_sample(CPU(), tsc, PW_PROC_EXEC, TID(), PID(), NAME());
    }
};
#endif // DO_PROBE_ON_EXEC_SYSCALL

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_process_fork(struct task_struct *parent, struct task_struct *child)
#else
    static void probe_sched_process_fork(void *ignore, struct task_struct *parent, struct task_struct *child)
#endif
{
    const char *cname = child->comm;
    pid_t ctid = child->pid, cpid = child->tgid;
    u64 tsc;

    tscval(&tsc);

    OUTPUT(3, KERN_INFO "DEBUG: PROCESS_FORK: %d (%.20s) --> %d (%.20s) \n", parent->pid, parent->comm, child->pid, cname);
    /* printk(KERN_INFO "DEBUG: PROCESS_FORK: %d,%d (%.20s) --> %d,%d (%.20s) \n", parent->pid, parent->tgid, parent->comm, ctid, cpid, cname); */

    produce_r_sample(CPU(), tsc, PW_PROC_FORK, ctid, cpid, cname);
};

/*
 * Module load tracepoint.
 * We register a module load event -- extract memory
 * bounds for the module.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_module_load(struct module *mod)
#else
    static void probe_module_load(void *ignore, struct module *mod)
#endif
{
    int cpu = CPU();
    const char *name = mod->name;
    unsigned long module_core = (unsigned long)mod->module_core;
    unsigned long core_size = mod->core_size;


    OUTPUT(0, KERN_INFO "Module %s LOADED! START = 0x%lx, SIZE = %lu\n", name, module_core, core_size);

    produce_m_sample(cpu, name, module_core, core_size);

    return;
};


#if DO_WAKELOCK_SAMPLE
/*
 * Wakelock hooks
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_wake_lock(struct wake_lock *lock)
#else
    static void probe_wake_lock(void *ignore, struct wake_lock *lock)
#endif
{
    u64 tsc;

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING!
     * UPDATE: track if COLLECTION ONGOING OR
     * IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);

    produce_w_sample(CPU(), tsc, PW_WAKE_LOCK, TID(), PID(), lock->name);

    OUTPUT(0, "wake_lock: name=%s, CPU=%d, PID=%d, TSC=%llu\n", lock->name, CPU(), PID(), tsc);
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_wake_unlock(struct wake_lock *lock)
#else
    static void probe_wake_unlock(void *ignore, struct wake_lock *lock)
#endif
{
    u64 tsc;

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING!
     * UPDATE: track if COLLECTION ONGOING OR
     * IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);

    produce_w_sample(CPU(), tsc, PW_WAKE_UNLOCK, TID(), PID(), lock->name);

    OUTPUT(0, "wake_unlock: name=%s, CPU=%d, PID=%d, TSC=%llu\n", lock->name, CPU(), PID(), tsc);
};
#endif


static int register_timer_callstack_probes(void)
{
    int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    {
	OUTPUT(0, KERN_INFO "\tTIMER_INIT_EVENTS");
	ret = register_trace_hrtimer_init(probe_hrtimer_init);
	WARN_ON(ret);
	ret = register_trace_timer_init(probe_timer_init);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tITIMER_STATE_EVENTS");
	ret = register_trace_itimer_state(probe_itimer_state);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tTIMER_START_EVENTS");
	ret = register_trace_hrtimer_start(probe_hrtimer_start);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tSCHED_EXIT_EVENTS");
	ret = register_trace_sched_process_exit(probe_sched_process_exit);
	WARN_ON(ret);
    }

#else

    {
	OUTPUT(0, KERN_INFO "\tTIMER_INIT_EVENTS");
	ret = register_trace_hrtimer_init(probe_hrtimer_init, NULL);
	WARN_ON(ret);
	ret = register_trace_timer_init(probe_timer_init, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tITIMER_STATE_EVENTS");
	ret = register_trace_itimer_state(probe_itimer_state, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tTIMER_START_EVENTS");
	ret = register_trace_hrtimer_start(probe_hrtimer_start, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tSCHED_EVENTS");
	ret = register_trace_sched_process_exit(probe_sched_process_exit, NULL);
	WARN_ON(ret);
    }

#endif // KERNEL_VER
    return SUCCESS;
};

static void unregister_timer_callstack_probes(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    {
	unregister_trace_hrtimer_init(probe_hrtimer_init);
	unregister_trace_timer_init(probe_timer_init);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_itimer_state(probe_itimer_state);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_hrtimer_start(probe_hrtimer_start);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_sched_process_exit(probe_sched_process_exit);

	tracepoint_synchronize_unregister();
    }

#else

    {
	unregister_trace_hrtimer_init(probe_hrtimer_init, NULL);
	unregister_trace_timer_init(probe_timer_init, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_itimer_state(probe_itimer_state, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_hrtimer_start(probe_hrtimer_start, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_sched_process_exit(probe_sched_process_exit, NULL);

	tracepoint_synchronize_unregister();
    }

#endif // KERNEL_VER
};

/*
 * Register all probes which should be registered
 * REGARDLESS OF COLLECTION STATUS.
 */
static int register_permanent_probes(void)
{
    return register_timer_callstack_probes();
};

static void unregister_permanent_probes(void)
{
    unregister_timer_callstack_probes();
};

/*
 * Register all probes which should be registered
 * ONLY FOR AN ONGOING, NON-PAUSED COLLECTION.
 */
static int register_non_pausable_probes(void)
{
    // timer expire
    // irq
    // tps
    // tpf
    int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
            ret = register_trace_timer_expire_entry(probe_timer_expire_entry);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit);
            WARN_ON(ret);
            ret = register_trace_irq_handler_entry(probe_irq_handler_entry);
            WARN_ON(ret);
            ret = register_trace_softirq_entry(probe_softirq_entry);
            WARN_ON(ret);
            ret = register_trace_sched_wakeup(probe_sched_wakeup);
            WARN_ON(ret);
        }
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_power_start(probe_power_start);
            WARN_ON(ret);
        }
        {
            ret = register_trace_workqueue_execution(probe_workqueue_execution);
            WARN_ON(ret);
        }
    }


#else // KERNEL_VER

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
            ret = register_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit, NULL);
            WARN_ON(ret);
            ret = register_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_softirq_entry(probe_softirq_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_sched_wakeup(probe_sched_wakeup, NULL);
            WARN_ON(ret);
        }
        /*
         * ONLY required for "SLEEP" mode i.e. C-STATES
         */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_power_start(probe_power_start, NULL);
            WARN_ON(ret);
        }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
        {
            ret = register_trace_workqueue_execution(probe_workqueue_execution, NULL);
            WARN_ON(ret);
        }
#else // 2.6.36 <= version < 2.6.38
        {
            ret = register_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);
            WARN_ON(ret);
        }
#endif // version < 2.6.36
#else // version >= 2.6.38
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_cpu_idle(probe_cpu_idle, NULL);
            WARN_ON(ret);
        }
        {
            ret = register_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);
            WARN_ON(ret);
        }
#endif // LINUX_VERSION_CODE < 2.6.38
    }

#endif // KERNEL_VER
    return SUCCESS;
};

static void unregister_non_pausable_probes(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    // #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            unregister_trace_timer_expire_entry(probe_timer_expire_entry);
            unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
            unregister_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit);
            unregister_trace_irq_handler_entry(probe_irq_handler_entry);
            unregister_trace_softirq_entry(probe_softirq_entry);
            unregister_trace_sched_wakeup(probe_sched_wakeup);

            tracepoint_synchronize_unregister();
        }
        /*
         * ONLY required for "SLEEP" mode i.e. C-STATES
         */
        {
            unregister_trace_power_start(probe_power_start);

            tracepoint_synchronize_unregister();
        }

        {
            unregister_trace_workqueue_execution(probe_workqueue_execution);

            tracepoint_synchronize_unregister();
        }
    }


#else // KERNEL_VER

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            unregister_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
            unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
            unregister_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit, NULL);
            unregister_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
            unregister_trace_softirq_entry(probe_softirq_entry, NULL);
            unregister_trace_sched_wakeup(probe_sched_wakeup, NULL);

            tracepoint_synchronize_unregister();
        }
        /*
         * ONLY required for "SLEEP" mode i.e. C-STATES
         */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
        {
            unregister_trace_power_start(probe_power_start, NULL);

            tracepoint_synchronize_unregister();
        }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
        {
            unregister_trace_workqueue_execution(probe_workqueue_execution, NULL);

            tracepoint_synchronize_unregister();
        }
#else // 2.6.36 <= version < 2.6.38
        {
            unregister_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);

            tracepoint_synchronize_unregister();
        }
#endif // version < 2.6.36
#else // version >= 2.6.38
        {
            unregister_trace_cpu_idle(probe_cpu_idle, NULL);

            tracepoint_synchronize_unregister();
        }
        {
            unregister_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);

            tracepoint_synchronize_unregister();
        }
#endif // LINUX_VERSION_CODE < 2.6.38
    }

#endif // KERNEL_VER
};

/*
 * Register all probes which must be registered
 * ONLY FOR AN ONGOING (i.e. START/PAUSED) COLLECTION.
 */
static int register_pausable_probes(void)
{
    int ret = 0;
    // sys_exit
    // sched_fork
    // module_load
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tMODULE_LOAD_EVENTS");
	ret = register_trace_module_load(probe_module_load);
	WARN_ON(ret);
    }

    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tSCHED_FORK_EVENTS");
	ret = register_trace_sched_process_fork(probe_sched_process_fork);
	WARN_ON(ret);
    }

    /*
     * ALWAYS required.
     */
#if DO_PROBE_ON_EXEC_SYSCALL
    {
	OUTPUT(0, KERN_INFO "\tsys_exit_EVENTS");
	ret = register_trace_sys_exit(probe_sys_exit);
	WARN_ON(ret);
    }
#endif // DO_PROBE_ON_EXEC_SYSCALL

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_register_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    ret = register_trace_power_frequency(probe_power_frequency);
	    WARN_ON(ret);
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
        OUTPUT(0, KERN_INFO "\tWAKELOCK_EVENTS\n");
        ret = register_trace_wake_lock(probe_wake_lock);
        WARN_ON(ret);

        OUTPUT(0, KERN_INFO "\tWAKEUNLOCK_EVENTS\n");
        ret = register_trace_wake_unlock(probe_wake_unlock);
        WARN_ON(ret);
#endif
    }

#else // KERNEL_VER

    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tMODULE_LOAD_EVENTS");
	ret = register_trace_module_load(probe_module_load, NULL);
	WARN_ON(ret);
    }

    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tSCHED_FORK_EVENTS");
	ret = register_trace_sched_process_fork(probe_sched_process_fork, NULL);
	WARN_ON(ret);
    }

    /*
     * ALWAYS required.
     */
#if DO_PROBE_ON_EXEC_SYSCALL
    {
	OUTPUT(0, KERN_INFO "\tsys_exit_EVENTS");
	ret = register_trace_sys_exit(probe_sys_exit, NULL);
	WARN_ON(ret);
    }
#endif // DO_PROBE_ON_EXEC_SYSCALL

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED!\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_register_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    ret = register_trace_power_frequency(probe_power_frequency, NULL);
	    WARN_ON(ret);
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
        printk(KERN_INFO "\tWAKELOCK_EVENTS");
        ret = register_trace_wake_lock(probe_wake_lock, NULL);
        WARN_ON(ret);

        printk(KERN_INFO "\tWAKEUNLOCK_EVENTS");
        ret = register_trace_wake_unlock(probe_wake_unlock, NULL);
        WARN_ON(ret);
#endif
    }

#endif // KERNEL_VER

    return SUCCESS;
};

static void unregister_pausable_probes(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)

    /*
     * ALWAYS required.
     */
    {
	unregister_trace_module_load(probe_module_load);

	tracepoint_synchronize_unregister();
    }

    /*
     * ALWAYS required.
     */
    {
	unregister_trace_sched_process_fork(probe_sched_process_fork);

	tracepoint_synchronize_unregister();
    }

    /*
     * ALWAYS required.
     */
#if DO_PROBE_ON_EXEC_SYSCALL
    {
	unregister_trace_sys_exit(probe_sys_exit);

	tracepoint_synchronize_unregister();
    }
#endif // DO_PROBE_ON_EXEC_SYSCALL

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED!\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_unregister_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    unregister_trace_power_frequency(probe_power_frequency);

	    tracepoint_synchronize_unregister();
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
        unregister_trace_wake_lock(probe_wake_lock);
        unregister_trace_wake_unlock(probe_wake_unlock);

        tracepoint_synchronize_unregister();
#endif
    }

#else // KERNEL_VER

    /*
     * ALWAYS required.
     */
    {
	unregister_trace_module_load(probe_module_load, NULL);

	tracepoint_synchronize_unregister();
    }

    /*
     * ALWAYS required.
     */
    {
	unregister_trace_sched_process_fork(probe_sched_process_fork, NULL);

	tracepoint_synchronize_unregister();
    }

    /*
     * ALWAYS required.
     */
#if DO_PROBE_ON_EXEC_SYSCALL
    {
	unregister_trace_sys_exit(probe_sys_exit, NULL);

	tracepoint_synchronize_unregister();
    }
#endif // DO_PROBE_ON_EXEC_SYSCALL

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_unregister_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    unregister_trace_power_frequency(probe_power_frequency, NULL);

	    tracepoint_synchronize_unregister();
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
        unregister_trace_wake_lock(probe_wake_lock, NULL);
        unregister_trace_wake_unlock(probe_wake_unlock, NULL);

        tracepoint_synchronize_unregister();
#endif
    }

#endif // KERNEL_VER
};

#if DO_RCU_OUTPUT_BUFFERS
/*
 * Function to dump all output buffers to
 * userspace. Multiple calls to 'device_read(...)'
 * will now return partially-sorted data i.e.
 * it is GUARANTEED that EVERY PWCollector_sample_t
 * element returned via a 'read(...)' call
 * to the device driver will have occurred
 * EARLIER than ANY PWCollector_sample_t element
 * returned from a subsequent call to the
 * 'read(...)' function.
 */
inline int copy_output_set_to_userspace(output_set_t *set, char __user *buffer, size_t length)
{
    int cpu=0, num_bytes_copied = 0, bytes_not_copied = 0;
    char *user_ptr = buffer;

    /*
     * Basic algo: iterate through
     * all output buffers in the current
     * output set and copy every
     * valid output sample to userspace.
     * Return # of bytes actually copied.
     */
    for_each_online_cpu(cpu){
	int i=0;
	list_t *list = &set->lists[cpu];
	for(i=0; i<NUM_SEGS_PER_LIST; ++i){
	    seg_t *seg = &list->segs[i];
	    int size = seg->index * sizeof(struct PWCollector_sample);
	    /*
	     * If buffer full then 'index' == 0.
	     * In that case, set the 'size' manually.
	     */
	    if(atomic_read(&seg->is_full) == FULL){
		size = SEG_SIZE;
	    }
	    /*
	     * 'copy_to_user(...)' is expensive: only copy
	     * data if there's anything to copy!
	     */
	    if(size == 0)
		continue;
	    /*
	     * OK, copy this chunk to userspace.
	     */
	    bytes_not_copied += copy_to_user(user_ptr, (char *)seg->samples, size);
	    user_ptr += size;
	    num_bytes_copied += size;
	}
    }

    num_bytes_copied -= bytes_not_copied;

    OUTPUT(3, KERN_INFO "DEBUG: # bytes = %d\n", num_bytes_copied);

    return num_bytes_copied;
};
#endif // DO_RCU_OUTPUT_BUFFERS

#if DO_PERIODIC_BUFFER_FLUSH
/*
 * Callback function for the timer initialized
 * in 'device_read(...)'. Enables periodic output
 * buffer flushes (required for GRUM support).
 */
static enum hrtimer_restart apwr_device_read_hrtimer_callback(struct hrtimer *hrtimer)
{
    OUTPUT(3, KERN_INFO "APWR_DEVICE_READ_CALLBACK!\n");
    /*
     * Tell a (possibly blocked) reader to unblock
     * and to copy all buffers to user space.
     */
    {
	atomic_set(&apwr_device_read_hrtimer_fired, 1);
	wake_up_interruptible(&read_queue);
    }
    /*
     * We're NOT periodic!
     */
    return HRTIMER_NORESTART;
};
#endif // DO_PERIODIC_BUFFER_FLUSH

/*
 * Service a "read(...)" call from user-space.
 *
 * Returns sample information back to the user.
 * When a user calls the "read" function, the device
 * driver first checks if any (per-cpu) output buffers are full.
 * If yes, then the entire contents of that buffer are
 * copied to the user. If not, then the user blocks, until the
 * buffer-full condition is met.
 */
static ssize_t device_read(struct file *file,	/* see include/linux/fs.h   */
			   char __user * buffer,	/* buffer to be
							 * filled with data */
			   size_t length,	/* length of the buffer     */
			   loff_t * offset)
{
    int bytes_read = 0;
    ssize_t size = SEG_SIZE;
    seg_t *seg = NULL;
    int cmp_size = SEG_SIZE;

    /*
     * First, check if we're actively collecting. If not, AND
     * if this isn't an attempt to drain the (per-cpu) buffers, return
     * with an error.
     *
     * Update: allow STOP and PAUSE, but not CANCEL.
     */
    if(!IS_COLLECTING() && INTERNAL_STATE.cmd == CANCEL){
	OUTPUT(0, KERN_INFO "Warning: (NON-FLUSH) read issued when driver not collecting!\n");
	return -EWOULDBLOCK;
    }

#if DO_RCU_OUTPUT_BUFFERS
    cmp_size = SEG_SIZE /* sizeof each buffer */ * 2 /* # buffers per cpu */ * PW_max_num_cpus /* # cpus */;
#else
    cmp_size = SEG_SIZE;
#endif
    /*
     * For now, require the input length to be
     * EXACTLY 'cmp_size' (for optimization)
     * This requirement will be removed in the future.
     */
    if(length != cmp_size){
	printk(KERN_INFO "Error: requested length (%d) MUST be equal to %d bytes!\n", (int)length, cmp_size);
	return -ERROR;
    }

    OUTPUT(3, KERN_INFO "DEBUG: READ: drain_buffers = %s\n", GET_BOOL_STRING(INTERNAL_STATE.drain_buffers));

    /*
     * Strategy:
     * (1) If we've been signalled to flush our buffers
     * (e.g. if a "STOP" command was issued), then find the
     * first non-empty output buffer and return its contents to the user.
     * (2) Else, wait until a buffer is completely full, and then
     * return its contents to the user.
     */
    if(INTERNAL_STATE.drain_buffers){ // drain the buffers
	seg = find_next_non_empty_seg();
	if(!seg)
	    return 0;
	size = seg->index * sizeof(struct PWCollector_sample);
    }else{
#if ALLOW_BLOCKING_READ

#if DO_PERIODIC_BUFFER_FLUSH
	{
	    /*
	     * We need a timer to wake us up
	     * within a fixed time limit -- helps
	     * to support 'GRUM'
	     */
	    u64 sleep_time_ns = 1000000000; // Timeout: 1 sec
	    hrtimer_init(&apwr_device_read_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	    apwr_device_read_hrtimer.function = &apwr_device_read_hrtimer_callback;
	    hrtimer_start(&apwr_device_read_hrtimer, ns_to_ktime(sleep_time_ns), HRTIMER_MODE_REL);
	}
#endif // DO_PERIODIC_BUFFER_FLUSH

	while(!(seg = find_seg())){
	    if(!IS_COLLECTING() && !IS_SLEEPING()){
		/*
		 * This could happen if, for example,
		 * an async reader thread was blocked
		 * on a "read", but the main thread
		 * issued a "STOP/PAUSE/CANCEL" command.
		 * (a) If CANCEL => Return -1 (and set errno to EWOULDBLOCK).
		 * (b) If STOP => Return as many bytes as possible.
		 * (c) If PAUSE => Block (waiting for RESUME).
		 */
		PWCollector_cmd_t cmd = INTERNAL_STATE.cmd;
		OUTPUT(0, KERN_INFO "Warning: NO buffer FULL, but told to WAKEUP! Cmd = %d\n", cmd);
		if(cmd == CANCEL) // error
		    return -EWOULDBLOCK; // the kernel takes care of setting errno to EWOULDBLOCK.
		else{ // STOP received => return (non-full) buffers
		    if(!(seg = find_next_non_empty_seg()))
			return 0;
		    /*
		     * We didn't have a full buffer -- set
		     * correct size here.
		     */
		    size = seg->index * sizeof(struct PWCollector_sample);
		    break;
		}
	    }
	    OUTPUT(3, KERN_INFO "Waiting...\n");
	    /*
	     * OK, there's no data for us: block until
	     * either there's enough data to return OR
	     * the driver receives a STOP/CANCEL command.
	     */
	    if(wait_event_interruptible(read_queue, ( (!IS_COLLECTING() && !IS_SLEEPING()) || any_seg_full()
#if DO_PERIODIC_BUFFER_FLUSH
						      || (atomic_read(&apwr_device_read_hrtimer_fired) == 1)
#endif
						      ))) // returns -ERESTARTSYS if interrupted by signal, "0" on success
		return -ERESTARTSYS;
	    /*
	     * We've been woken up. This could be because of:
	     * (a) A FULL buffer condition or
	     * (b) DD received a STOP/PAUSE/CANCEL while reader was blocked or
	     * (c) The 'apwr_device_read_hrtimer' elapsed (ONLY FOR 'DO_PERIODIC_BUFFER_FLUSH == 1')
	     * In case (a) we're *guaranteed* a FULL buffer ==> SEG_SIZE
	     * bytes below is valid. If (b) then we probably won't
	     * have a full buffer, but its still OK to set to SEG_SIZE
	     * here because the correct size will be set when the
	     * "find_next_non_empty_seg()" function is called (above).
	     */
	    size = SEG_SIZE;
#if DO_PERIODIC_BUFFER_FLUSH
	    {
		/*
		 * Cancel our timer. Blocks if the callback
		 * function is currently executing, but
		 * that's OK -- we check for that condition
		 * below anyway.
		 */
		hrtimer_cancel(&apwr_device_read_hrtimer);
		/*
		 * Check if we were woken because our
		 * timer fired.
		 */
		if(atomic_read(&apwr_device_read_hrtimer_fired) == 1){
		    /*
		     * We were woken up by our timer (or the callback function
		     * executed after we were woken up) -- break
		     * out of the loop (without checking for
		     * a full buffer) and copy all output buffers
		     * to user space.
		     */
		    atomic_set(&apwr_device_read_hrtimer_fired, 0);
		    break;
		}
	    }
#endif // DO_PERIODIC_BUFFER_FLUSH
	}
#else // ALLOW_BLOCKING_READ
	if(!(seg = find_seg()))
	    return 0; // nothing to read
#endif
    }

#if DO_RCU_OUTPUT_BUFFERS
    {
	/*
	 * OK, we need to allow the 'consumers' (i.e.
	 * the threads writing PWCollector samples
	 * into the output buffers) to continue to
	 * write data -- we do so by (atomically)
	 * switching the output buffer 'set' to
	 * a new one. As an optimization, we keep
	 * TWO output buffer sets, with only one of
	 * them being active at any point of time.
	 * The 'curr_output_set_index' variable
	 * points to the currently active output set.
	 */
	int new_index = 1  - curr_output_set_index;
	output_set_t *old_set = current_output_set, *new_set = output_sets[new_index];
	/*
	 * Tell subsequent consumers to look at the
	 * 'new' output set...
	 */
	rcu_assign_pointer(current_output_set, new_set);
	/*
	 * ...and sleep until all RCU read-side critical
	 * sections which began BEFORE this point are
	 * done (RCU makes those guarantees).
	 */
	synchronize_rcu(); // blocking

	curr_output_set_index = new_index;
	/*
	 * OK, at this point we're guaranteed all
	 * 'readers' are operating on the new
	 * output set. Go ahead and copy ALL elements
	 * of ALL (per-cpu) output buffers from
	 * the old output set into the (user-supplied)
	 * buffer.
	 */
	{
	    bytes_read = copy_output_set_to_userspace(old_set, buffer, length);
	}
	/*
	 * OK, contents have been copied over
	 * to user-space. We can now zero
	 * out the old set in preparation
	 * for the next 'device_read(...)' (when
	 * the current output set will be switched out
	 * and the 'old_set' output set will
	 * be switched back in).
	 */
	clear_output_set(old_set);
    }
#else // !DO_RCU_OUTPUT_BUFFERS
    {

	/*
	 * Now copy data to user.
	 */
	int bytes_not_copied = 0;
	OUTPUT(3, KERN_INFO "About to copy to user-space! Size = %d\n", (int)size);

	if( (bytes_not_copied = copy_to_user(buffer, (char *)seg->samples, size))){ // dst,src
	    printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", bytes_not_copied);
	}

	/*
	 * Done copying -- let producer know
	 * this segment is empty.
	 */
	SAMPLE(seg, 0).sample_type = FREE_SAMPLE;
	atomic_set(&seg->is_full, EMPTY);
	smp_mb();

	bytes_read = size - bytes_not_copied;

    }
#endif // DO_RCU_OUTPUT_BUFFERS


    OUTPUT(3, KERN_INFO "READ bytes_read = %d\n", bytes_read);


    return bytes_read;
};

static unsigned int device_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask = 0;

    poll_wait(filp, &read_queue, wait);

    if(!IS_COLLECTING() || any_seg_full()) // device is readable if: (a) NOT collecting or (b) any buffers is full
	mask = (POLLIN | POLLRDNORM);

    return mask;
};

// "copy_from_user" ==> dst, src
#define EXTRACT_LOCAL_ARGS(l,u) copy_from_user((l), (u), sizeof(struct PWCollector_ioctl_arg))

/*
 * Check if command is valid, given current state.
 */
static inline bool is_cmd_valid(PWCollector_cmd_t cmd)
{
    bool is_collecting = IS_COLLECTING(), is_sleeping = IS_SLEEPING();

    if(is_sleeping){
	/*
	 * If currently PAUSEd, the ONLY command
	 * that's NOT allowed is a subsequent PAUSE.
	 */
	if(cmd == PAUSE)
	    return false;
    }
    else if(is_collecting && (cmd == START || cmd == RESUME))
      return false;
    else if(!is_collecting && (cmd == STOP || cmd == PAUSE || cmd == CANCEL))
      return false;

    return true;
};

#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
/*
 * Get list of available frequencies
 */
void get_frequency_steps(void)
{
    /*
     * Try and determine the 'legal' frequencies
     * i.e. the various (discrete) frequencies
     * the processor could possibly execute at.
     * This must be done ONCE PER COLLECTION (and
     * only at the START of the collection)!
     * Update: do this ONLY if we're computing
     * frequencies dynamically!
     */
    int cpu = 0;
    struct cpufreq_policy *policy;
    ssize_t buffer_len = -1;
    struct freq_attr *freq_attrs = &cpufreq_freq_attr_scaling_available_freqs;
    char buffer[512]; // size is probably overkill -- but we can't be sure how many freq states there are!
    /*
     * (1) Get the current CPUFREQ policy...
     */
    if( (policy = cpufreq_cpu_get(cpu)) == NULL){
	printk(KERN_INFO "WARNING: cpufreq_cpu_get error for CPU = %d\n", cpu);
    }
    else{
	/*
	 * (2) Get the (string) representation of the
	 * various frequencies. The string contains
	 * a number of (space-separated) frequencies (in KHz).
	 */
	if( (buffer_len = freq_attrs->show(policy, buffer)) == -ENODEV){
	    printk(KERN_INFO "WARNING: cpufreq_attrs->show(...) error for CPU = %d\n", cpu);
	}else{
	    /*
	     * (3) Tokenize the string to extract the
	     * actual frequencies. At this point
	     * we can confidently state we've found
	     * the frequencies and we don't need to
	     * repeat this procedure.
	     */
	    OUTPUT(0, KERN_INFO "[%d]: buffer_len = %d, BUFFER = %s\n", cpu, (int)buffer_len, buffer);
	    if(extract_valid_frequencies(buffer, buffer_len)){
		printk(KERN_INFO "ERROR: could NOT determine frequency table!\n");
	    }
	}
    }
};
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT


/*
 * Retrieve the base operating frequency
 * for this CPU. The base frequency acts
 * as a THRESHOLD indicator for TURBO -- frequencies
 * ABOVE this are considered TURBO.
 */
static inline void get_base_operating_frequency(void)
{
    u64 res;
    u16 ratio = 0;

    if(!INTERNAL_STATE.bus_clock_freq_khz){
        printk(KERN_INFO "ERROR: cannot set base_operating_frequency until we have a bus clock frequency!\n");
        return;
    }

    /*
     * Algo:
     * (1) If NHM/WMR/SNB -- read bits 15:8 of 'PLATFORM_INFO_MSR_ADDR'
     * (2) If MFLD/ATM -- read bits 44:40 of 'CLOCK_CR_GEYSIII_STAT' MSR
     * to extract the 'base operating ratio'.
     * To get actual TSC frequency, multiply this ratio
     * with the bus clock frequency.
     */
    if(PW_is_atm){
	rdmsrl(CLOCK_CR_GEYSIII_STAT_MSR_ADDR, res);
	/*
	 * Base operating Freq ratio is
	 * bits 44:40
	 */
	ratio = (res >> 40) & 0x1f;
    }else{
	rdmsrl(PLATFORM_INFO_MSR_ADDR, res);
	/*
	 * Base Operating Freq ratio is
	 * bits 15:8
	 */
	ratio = (res >> 8) & 0xff;
    }

    // base_operating_freq = ratio * BUS_CLOCK_FREQ_KHZ;
    base_operating_freq = ratio * INTERNAL_STATE.bus_clock_freq_khz;
    OUTPUT(3, KERN_INFO "RATIO = 0x%x, FREQ = %u, (RES = %llu)\n", (u32)ratio, base_operating_freq, res);
};

/*
 * Set initial config params.
 * These include MSR addresses, and power
 * collection switches.
 */
int set_config(struct PWCollector_config *remote_config, int size)
{
    int i=0;
    struct PWCollector_config local_config;

    if( (i = copy_from_user(&local_config, remote_config, size))) // "copy_from_user" returns number of bytes that COULD NOT be copied
	return i;
    /*
     * Copy Core/Pkg MSR addresses
     */
    memcpy(INTERNAL_STATE.coreResidencyMSRAddresses, local_config.info.coreResidencyMSRAddresses, sizeof(int) * MAX_MSR_ADDRESSES);
    memcpy(INTERNAL_STATE.pkgResidencyMSRAddresses, local_config.info.coreResidencyMSRAddresses, sizeof(int) * MAX_MSR_ADDRESSES);

    if(true){
	for(i=0; i<MAX_MSR_ADDRESSES; ++i)
	    OUTPUT(0, KERN_INFO "C%d: %d\n", i, INTERNAL_STATE.coreResidencyMSRAddresses[i]);
    }
    /*
     * Set C-state clock multiplier.
     */
    INTERNAL_STATE.residency_count_multiplier = local_config.info.residency_count_multiplier;

    /*
     * Make sure we've got a valid multiplier!
     */
    if((int)INTERNAL_STATE.residency_count_multiplier <= 0)
	INTERNAL_STATE.residency_count_multiplier = 1;

    if(true){
	OUTPUT(0, KERN_INFO "DEBUG: C-state clock multiplier = %u\n", INTERNAL_STATE.residency_count_multiplier);
    }

    /*
     * Set bus clock frequency -- required for
     * Turbo threshold determination / calculation.
     */
    INTERNAL_STATE.bus_clock_freq_khz = local_config.info.bus_clock_freq_khz;
    /*
     * Check if we've got a valid bus clock frequency -- default to
     * BUS_CLOCK_FREQ_KHZ if not.
     */
    if((int)INTERNAL_STATE.bus_clock_freq_khz <= 0)
	// INTERNAL_STATE.bus_clock_freq_khz = BUS_CLOCK_FREQ_KHZ;
	INTERNAL_STATE.bus_clock_freq_khz = DEFAULT_BUS_CLOCK_FREQ_KHZ();

    OUTPUT(0, KERN_INFO "DEBUG: Bus clock frequency = %u KHz\n", INTERNAL_STATE.bus_clock_freq_khz);

    /*
     * The base operating frequency requires the
     * bus frequency -- set it here.
     */
    get_base_operating_frequency();
    /*
     * Get a list of frequencies that
     * the processors may execute at.
     */
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
    get_frequency_steps();
#endif

    /*
     * Set power switches.
     */
    INTERNAL_STATE.collection_switches = local_config.data;
    OUTPUT(0, KERN_INFO "\tCONFIG collection switches = %d\n", INTERNAL_STATE.collection_switches);

    INTERNAL_STATE.d_state_sample_interval = local_config.d_state_sample_interval;
    OUTPUT(0, KERN_INFO "\tCONFIG D-state collection interval (msec) = %d\n", INTERNAL_STATE.d_state_sample_interval);

    d_sc_count_num = 0;
    for (i=0; i<MAX_LSS_NUM_IN_SC; i++) {
        if ((d_sc_mask >> i) & 0x1) {
            d_sc_count_num++;
        }
    }

    return SUCCESS;
};

int check_platform(struct PWCollector_check_platform *remote_check, int size)
{
    struct PWCollector_check_platform *local_check;
    const char *unsupported = "UNSUPPORTED_T1, UNSUPPORTED_T2"; // for debugging ONLY
    int len = strlen(unsupported);
    int max_size = sizeof(struct PWCollector_check_platform);
    int retVal = SUCCESS;

    local_check = pw_kmalloc(max_size, GFP_KERNEL);

    if(!local_check){
	printk(KERN_INFO "ERROR: could NOT allocate memory in check_platform!\n");
	return -ERROR;
    }

    memset(local_check, 0, max_size);

    /*
     * Populate "local_check.unsupported_tracepoints" with a (comma-separated)
     * list of unsupported tracepoints. For now, we just leave this
     * blank, reflecting the fact that, on our development systems,
     * every tracepoints is supported.
     *
     * Update: for debugging, write random data here.
     */
    memcpy(local_check->unsupported_tracepoints, unsupported, len);
    /*
     * Copy everything back to user address space.
     */
    if( (retVal = copy_to_user(remote_check, local_check, size))) // returns number of bytes that COULD NOT be copied
	retVal = -ERROR;

    pw_kfree(local_check);
    return retVal; // all unsupported tracepoints documented
};

/*
 * Return the TURBO frequency threshold
 * for this CPU.
 */
int get_turbo_threshold(struct PWCollector_turbo_threshold *remote_thresh, int size)
{
    struct PWCollector_turbo_threshold local_thresh;

    if(!base_operating_freq)
	return -ERROR;

    local_thresh.threshold_frequency = base_operating_freq;

    if(copy_to_user(remote_thresh, &local_thresh, size)) // returns number of bytes that could NOT be copied.
	return -ERROR;

    return SUCCESS;
};

/*
 * Retrieve device driver version
 */
int get_version(struct PWCollector_version_info *remote_version, int size)
{
    struct PWCollector_version_info local_version;

    local_version.version = PW_VERSION_VERSION;
    local_version.interface = PW_VERSION_INTERFACE;
    local_version.other = PW_VERSION_OTHER;

    /*
     * Copy everything back to user address space.
     */
    return copy_to_user(remote_version, &local_version, size); // returns number of bytes that could NOT be copiled
};

/*
 * Retrieve microcode patch version.
 * Only useful for MFLD
 */
int get_micro_patch_ver(int *remote_ver, int size)
{
    int local_ver = micro_patch_ver;

    /*
     * Copy everything back to user address space.
     */
    return copy_to_user(remote_ver, &local_ver, size); // returns number of bytes that could NOT be copiled
};

static inline void read_sample_residencies(int cpu, u64 samples[])
{
    u32 h=0, l=0;
    u64 res=0;
    int i=0, msr_addr=-1;

    OUTPUT(3, KERN_INFO "Addr = %p\n", samples);

    for(i=0; i<MAX_MSR_ADDRESSES; ++i){
	if( (msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[i]) <= 0)
	    continue;
	rdmsr_on_cpu(cpu, msr_addr, &l, &h);
	{
	    res = h;
	    res <<= 32;
	    res += l;
	}
	samples[i] = res;
	OUTPUT(3, KERN_INFO "\t[%d]: MSR=%d, VAL=0x%llx\n", cpu, i, samples[i]);
    }
};



int get_status(struct PWCollector_status *remote_status, int size)
{
    struct PWCollector_status local_status;
    int cpu, retVal = SUCCESS;
    stats_t *pstats = NULL;
    unsigned long statusJIFF, elapsedJIFF = 0;

    memset(&local_status, 0, sizeof(local_status));

    /*
     * Set # cpus.
     */
    local_status.num_cpus = PW_max_num_cpus;

    /*
     * Set total collection time elapsed.
     */
    {
	statusJIFF = jiffies;
	if(statusJIFF < INTERNAL_STATE.collectionStartJIFF){
	    printk(KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
	    elapsedJIFF = 0; // avoid messy NAN when dividing
	}else{
	    // elapsedJIFF = statusJIFF - startJIFF;
	    elapsedJIFF = statusJIFF - INTERNAL_STATE.collectionStartJIFF;
	}
	OUTPUT(0, KERN_INFO "start = %lu, stop = %lu, elapsed = %lu\n", INTERNAL_STATE.collectionStartJIFF, statusJIFF, elapsedJIFF);
    }
    local_status.time = jiffies_to_msecs(elapsedJIFF);

    /*
     * Set # c-breaks etc.
     * Note: aggregated over ALL cpus,
     * per spec document.
     */
    for_each_online_cpu(cpu){
	pstats = &per_cpu(per_cpu_stats, cpu);
	local_status.c_breaks += local_read(&pstats->c_breaks);
	local_status.timer_c_breaks += local_read(&pstats->timer_c_breaks);
	local_status.inters_c_breaks += local_read(&pstats->inters_c_breaks);
	local_status.p_trans += local_read(&pstats->p_trans);
	local_status.num_inters += local_read(&pstats->num_inters);
	local_status.num_timers += local_read(&pstats->num_timers);
    }

    /*
     * Now copy everything to user-space.
     */
    retVal = copy_to_user(remote_status, &local_status, sizeof(local_status)); // returns number of bytes that COULD NOT be copied

    return retVal;
};

long get_available_frequencies(struct PWCollector_available_frequencies *remote_freqs, int size)
{
    int i=0;
    struct PWCollector_available_frequencies local_freqs;

    if(!apwr_available_frequencies){
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
	printk(KERN_INFO "ERROR: trying to get list of available frequencies WITHOUT setting config?!\n");
#else
	printk(KERN_INFO "ERROR: trying to get a list of available frequencies while operating in LEGACY P-state mode?!\n");
#endif
	return -ERROR;
    }

    memset(local_freqs.frequencies, 0, sizeof(u32) * PW_MAX_NUM_AVAILABLE_FREQUENCIES);

    for(i = 0, local_freqs.num_freqs = 0; apwr_available_frequencies[i] != 0; ++i, ++local_freqs.num_freqs){
	local_freqs.frequencies[i] = apwr_available_frequencies[i];
    }

    if(copy_to_user(remote_freqs, &local_freqs, size)) // returns number of bytes that could NOT be copied.
	return -ERROR;

    return SUCCESS;
};

/*
 * Reset all statistics collected so far.
 * Called from a non-running collection context.
 */
static inline void reset_statistics(void)
{
    int cpu;
    stats_t *pstats = NULL;

    /*
     * Note: no need to lock, since we're only
     * going to be called from a non-running
     * collection, and tracepoints are inserted
     * (just) before a collection starts, and removed
     * (just) after a collection ends.
     */
    for_each_online_cpu(cpu){
	/*
	 * Reset the per cpu stats
	 */
	{
	    pstats = &per_cpu(per_cpu_stats, cpu);
	    local_set(&pstats->c_breaks, 0);
	    local_set(&pstats->timer_c_breaks, 0);
	    local_set(&pstats->inters_c_breaks, 0);
	    local_set(&pstats->p_trans, 0);
	    local_set(&pstats->num_inters, 0);
	    local_set(&pstats->num_timers, 0);
	}
    }
};

/*
 * Reset the (PER-CPU) structs containing
 * MSR residency information (amongst
 * other fields).
 */
void reset_per_cpu_msr_residencies(void)
{
    int cpu;
    per_cpu_t *pcpu = NULL;

    for_each_online_cpu(cpu){
	/*
	 * Reset the per-cpu residencies
	 */
	{
	    pcpu = &per_cpu(per_cpu_counts, cpu);
	    {
		// PREV_MSR_VAL(pcpu, MPERF) = 0;
		memset(pcpu->residencies, 0, sizeof(pcpu->residencies));
		memset(pcpu->prev_msr_vals, 0, sizeof(pcpu->prev_msr_vals));
		pcpu->last_pid = pcpu->last_tid = -1;
		pcpu->last_break[WORKQUEUE] = pcpu->last_break[IRQ] = pcpu->last_break[TIMER] =  pcpu->last_break[SCHED] = 0;
		pcpu->debug_enters = 0;
		pcpu->prev_state = 0;
	    }
	}
    }
    /*
     * Ensure updates are propagated.
     */
    smp_mb();
};


/*
 * Helper function: measure TSC
 * and current CPU operating
 * frequency on the current
 * CPU.
 */
#if 0
static void measure_curr_freq(void *dummy)
{
    int cpu = CPU();
    u64 tsc;
    u32 freq;

    tscval(&tsc);
    /*
     * CANNOT use 'cpufreq_get()' => it
     * uses semaphore locking, which can block, and
     * blocking is DISALLOWED in functions
     * called via 'on_each_cpu(...)'.
     * Use the safer (but potentially less accurate)
     * 'cpufreq_quick_get(...)' function instead.
     */
    // freq = cpufreq_get(cpu);
    freq = cpufreq_quick_get(cpu);
    printk(KERN_INFO "[%d]: %llu --> %u\n", cpu, tsc, freq);

    // freq /= 1000; // P-sample expects MHz

    // produce_p_sample(cpu, tsc, freq, 0 /* is_turbo */);
    produce_p_sample(cpu, tsc, freq);
};
#endif

static void reset_trace_sent_fields(void)
{
    struct tnode *node = NULL;
    struct hlist_node *curr = NULL;
    int i=0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	hlist_for_each_entry(node, curr, &timer_map[i].head, list){
	    node->trace_sent = 0;
	}
};

/*
 * Measure current CPU operating
 * frequency, and push 'P-samples'
 * onto the (per-cpu) O/P buffers.
 * Also determine the various
 * discrete frequencies the processor
 * is allowed to execute at (basically
 * the various frequencies present
 * in the 'scaling_available_frequencies'
 * sysfs file).
 *
 * REQUIRES CPUFREQ DRIVER!!!
 */
static void generate_cpu_frequency_per_cpu(int cpu, bool is_start)
{

	u32 act_freq = 0, req_freq = 0;
	u32 l=0, h=0;
	u64 tsc = 0;
	per_cpu_freq_data_t *pdata = &per_cpu(pcpu_freq_data, cpu);

	/*
	 * Try and get the current operating
	 * frequency -- THIS IS ON A BEST-EFFORT
	 * BASIS!
	 * N.B. -- only do this if we're at
	 * a collection START -- otherwise just
	 * return the (previously computed
	 * frequency).
	 */
	if(is_start){
	    /*
	     * Collection START -- get freq
	     * from the 'cpufreq'
	     * subsystem.
	     */
	    if(!(act_freq = cpufreq_get(cpu))){
		/*
		 * One last try!
		 */
		act_freq = cpufreq_quick_get(cpu); // LESS accurate, but sometimes works (esp. on MFLD)
	    }
	    pdata->aperf = pdata->mperf = 0;
	    /*
	     * Set 'prev_req_freq' here -- this ensures
	     * that when we get the first
	     * (non-boundary) TPF we have SOMETHING
	     * to show (otherwise the user sees
	     * 2 consecutive samples with 0 req states -- the
	     * boundary sample and the first non-boundary
	     * one).
	     */
	    pdata->prev_req_freq = act_freq;
	    req_freq = 0;

#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
	    {
		pdata->prev_act_freq = act_freq;
	    }
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT

	    OUTPUT(0, KERN_INFO "[%d]: INIT FREQ = %u\n", cpu, act_freq);
	}else{
	    /*
	     * Collection STOP -- return previously
	     * calculated freq.
	     */
	    req_freq = pdata->prev_req_freq;
	    OUTPUT(3, KERN_INFO "[%d]: PREV_REQ_FREQ = %u\n", cpu, req_freq);
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
	    {
		act_freq = pdata->prev_act_freq;
	    }
#else
	    if(!(act_freq = cpufreq_get(cpu))){
		/*
		 * One last try!
		 */
		act_freq = cpufreq_quick_get(cpu); // LESS accurate, but sometimes works (esp. on MFLD)
	    }
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
	}

	/*
	 * OK, per-cpu stuff done. Now send
	 * a 'p-sample' to Ring-3
	 * (noting that this is a 'BOUNDARY-MARKER'
	 * sample).
	 */
	{
	    /*
	     * We KNOW TSC exists -- no
	     * need for 'safe' variant.
	     */
	    int ret = rdmsr_on_cpu(cpu, 0x10, &l, &h);
	    if(ret){
		printk(KERN_INFO "WARNING: rdmsr of TSC failed with code %d\n", ret);
	    }
	    tsc = h;
	    tsc <<= 32;
	    tsc += l;
	}
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
	produce_p_sample(cpu, tsc, req_freq, act_freq, 1); // "1" ==> this is a BOUNDARY-MARKER p-sample.
#else // DO_DYNAMIC_FREQUENCY_MEASUREMENT
	produce_p_sample(cpu, tsc, act_freq, 1); // "1" ==> this is a BOUNDARY-MARKER p-sample.
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
	OUTPUT(3, KERN_INFO "[%d]: %llu --> %u\n", cpu, tsc, act_freq);
}
static void generate_cpu_frequency(void *data)
{
    int cpu = CPU();
    bool is_start = *((bool *)data);
    generate_cpu_frequency_per_cpu(cpu, is_start);
}
static void get_current_cpu_frequency(bool is_start)
{
    int cpu = 0;
#if DO_GENERATE_CURRENT_FREQ_IN_PARALLEL
    SMP_CALL_FUNCTION(&generate_cpu_frequency, (void *)&is_start, 0, 1);
    generate_cpu_frequency((void *)&is_start);
#else
    for_each_online_cpu(cpu){
        generate_cpu_frequency_per_cpu(cpu, is_start);
    }
#endif
    if(!is_start){
#if DO_DYNAMIC_FREQUENCY_MEASUREMENT
	/*
	 * We're at collection STOP -- delete
	 * previously allocated frequency
	 * table.
	 */
	if(apwr_available_frequencies){
	    pw_kfree(apwr_available_frequencies);
	    apwr_available_frequencies = NULL;
	}
#endif // DO_DYNAMIC_FREQUENCY_MEASUREMENT
    }
};

/*
 * START/RESUME a collection.
 *
 * (a) (For START ONLY): ZERO out all (per-cpu) O/P buffers.
 * (b) Reset all statistics.
 * (c) Register all tracepoints.
 */
int start_collection(PWCollector_cmd_t cmd)
{
    int cpu=0, i=0;

    {
	if(true && cmd == RESUME){
	    u64 tmp_tsc = 0;
	    tscval(&tmp_tsc);
	    printk(KERN_INFO "RECEIVED RESUME at tsc = %llu\n", tmp_tsc);
	}
    }
    switch(cmd){
    case START:
	/*
	 * Reset the O/P buffers.
	 *
	 * START ONLY
	 */
	{
	    for_each_online_cpu(cpu){
		list_t *list = UNPROTECTED_GET_OUTPUT_LIST(cpu);
		memset(list, 0, sizeof(list_t));
		for(i=0; i<NUM_SEGS_PER_LIST; ++i){
		    atomic_set(&list->segs[i].is_full, EMPTY);
		    smp_mb();
		}
	    }
	    last_list_read = -1;
	    last_flush_index = 0;
	}
	/*
	 * Reset the 'trace_sent' fields
	 * for all trace entries -- this
	 * ensures we send backtraces
	 * once per collection, as
	 * opposed to once per 'insmod'.
	 *
	 * START ONLY
	 */
	{
	    reset_trace_sent_fields();
	}

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
	/*
	 * Reset the list of vars required
	 * to transfer IRQ # <-> Name info.
	 * UPDATE: the 'irq_map' should contain
	 * mappings for only those
	 * devices that actually caused C-state
	 * wakeups DURING THE CURRENT COLLECTION.
	 * We therefore reset the map before
	 * every collection (this also auto resets
	 * the "irq_mappings_list" data structure).
	 *
	 * START ONLY
	 */
	{
	    destroy_irq_map();
	    if(init_irq_map()){
		// ERROR
		printk(KERN_INFO "ERROR: could NOT initialize irq map in start_collection!\n");
		return -ERROR;
	    }
	}
#endif
	/*
	 * Reset collection stats
	 *
	 * START ONLY
	 */
#if DO_IOCTL_STATS
	{
	    reset_statistics();
	}
#endif

    case RESUME: // fall through
	break;
    default: // should *NEVER* happen!
	printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
	return -ERROR;
    }
    /*
     * Reset the (per-cpu) "per_cpu_t" structs that hold MSR residencies
     *
     * START + RESUME
     */
    {
	reset_per_cpu_msr_residencies();
    }

    /*
     * Get START P-state samples.
     *
     * UPDATE: do this ONLY IF
     * USER SPECIFIES FREQ-mode!
     *
     * START + RESUME???
     */
    if(likely(IS_FREQ_MODE())){
	get_current_cpu_frequency(true); // "true" ==> collection START
    }

    INTERNAL_STATE.collectionStartJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = true;
    /*
     * OK, all setup completed. Now
     * register the tracepoints.
     */
    switch(cmd){
    case START:
	register_pausable_probes();
    case RESUME: // fall through
	register_non_pausable_probes();
	break;
    default: // should *NEVER* happen!
	printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
	return -ERROR;
    }
#if 0
    {
	register_all_probes();
    }
#endif
    OUTPUT(0, KERN_INFO "\tREGISTERED all probes!\n");

#if DO_S_RESIDENCY_SAMPLE
    //struct timeval cur_time;
    if(PW_is_atm && IS_S_RESIDENCY_MODE()){
	    start_s_residency_counter();
            //do_gettimeofday(cur_time);
            startJIFF_s_residency = CURRENT_TIME_IN_USEC();
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
    if(PW_is_atm && IS_D_SC_RESIDENCY_MODE()){
	    start_d_sc_residency_counter();
            startJIFF_d_sc_residency = CURRENT_TIME_IN_USEC();
            prev_sample_usec = CURRENT_TIME_IN_USEC();
    }
#endif

    return SUCCESS;
};

/*
 * STOP/PAUSE/CANCEL a (running) collection.
 *
 * (a) Unregister all tracepoints.
 * (b) Reset all stats.
 * (c) Wake any process waiting for full buffers.
 */
int stop_collection(PWCollector_cmd_t cmd)
{
    /*
     * Get S and D state residency counter samples
     */
#if DO_S_RESIDENCY_SAMPLE
    if(PW_is_atm && IS_S_RESIDENCY_MODE()){
        u64 usec = dump_s_residency_counter();
        produce_s_residency_sample(usec);
        stop_s_residency_counter();
    }
#endif

#if DO_S_STATE_SAMPLE
    if(PW_is_atm && IS_S_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_s_state_sample();
        }
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
    if(PW_is_atm && IS_D_SC_RESIDENCY_MODE()){
        u64 usec = dump_d_sc_residency_counter();
        produce_d_sc_residency_sample(usec);
        stop_d_sc_residency_counter();
    }
#endif

#if DO_D_NC_STATE_SAMPLE
    if(PW_is_atm && IS_D_NC_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_d_nc_state_sample();
        }
    }
#endif

#if DO_D_SC_STATE_SAMPLE
    if(PW_is_atm && IS_D_SC_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_d_sc_state_sample();
        }
    }
#endif

    INTERNAL_STATE.collectionStopJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = false;
    {
	if(true && cmd == PAUSE){
	    u64 tmp_tsc = 0;
	    tscval(&tmp_tsc);
	    OUTPUT(0, KERN_INFO "RECEIVED PAUSE at tsc = %llu\n", tmp_tsc);
	}
    }
    {
	unregister_non_pausable_probes();
    }
    if(cmd == STOP || cmd == CANCEL)
	{
	    unregister_pausable_probes();
	}
#if 0
    {
        unregister_all_probes();
    }
#endif

    // Reset the (per-cpu) "per_cpu_t" structs that hold MSR residencies
    {
        reset_per_cpu_msr_residencies();
    }

    /*
     * Reset IOCTL stats
     *
     * STOP/CANCEL ONLY
     */
#if DO_IOCTL_STATS
    if(cmd == STOP || cmd == CANCEL)
	{
	    reset_statistics();
	}
#endif

    /*
     * Get STOP P-state samples
     * (but only for STOP/CANCEL commands)
     *
     * UPDATE: do this ONLY IF
     * USER SPECIFIES FREQ-mode!
     */
    if(cmd == STOP || cmd == CANCEL)
	{
	    if(likely(IS_FREQ_MODE())){
		get_current_cpu_frequency(false); // "false" ==> collection STOP
	    }
	}

    /*
     * There might be a reader thread blocked on a read: wake
     * it up to give it a chance to respond to changed
     * conditions.
     */
    {
        wake_up_interruptible(&read_queue);
    }

    /*
     * Delete all non-kernel timers.
     *
     * STOP/CANCEL ONLY
     */
    if(cmd == STOP || cmd == CANCEL)
	{
	    delete_all_non_kernel_timers();
	}

    OUTPUT(0, KERN_INFO "\tUNREGISTERED all probes!\n");
    return SUCCESS;
};

long handle_cmd(PWCollector_cmd_t cmd)
{
    PWCollector_cmd_t prev_cmd;
    /*
     * Sanity check cmd range.
     */
    if(cmd < START || cmd > MARK){
	printk(KERN_INFO "Error: UNSUPPORTED cmd=%d\n", cmd);
	return -ERROR;
    }
    /*
     * Check to see if there are any invalid
     * command combinations (e.g. START -> START etc.)
     */
    if(!is_cmd_valid(cmd)){
	printk(KERN_INFO "Error: INVALID requested cmd=%d, CURRENT cmd=%d\n", cmd, INTERNAL_STATE.cmd);
	return -ERROR;
    }
    /*
     * OK, we've gotten a valid command.
     * Store it.
     */
    prev_cmd = INTERNAL_STATE.cmd;
    INTERNAL_STATE.cmd = cmd;
    /*
     * Actions based on specific commands here...
     */
    switch(cmd){
    case START:
    case RESUME:
	INTERNAL_STATE.drain_buffers = false;
	// startJIFF = jiffies;
	{
	    if(start_collection(cmd))
		return -ERROR;
	}
	break;
    case STOP:
	INTERNAL_STATE.drain_buffers = true;
    case PAUSE:
    case CANCEL:
	// stopJIFF = jiffies;
	{
	    stop_collection(cmd);
	}
	break;
    default:
	printk(KERN_INFO "Error: UNSUPPORTED cmd=%d\n", cmd);
	/*
	 * Reset "cmd" state to what it was before
	 * this ioctl.
	 */
	INTERNAL_STATE.cmd = prev_cmd;
	return -ERROR;
    }
    OUTPUT(3, KERN_INFO "Debug: Successfully switched mode from %d to %d: IS_COLLECTING = %d\n", prev_cmd, cmd, IS_COLLECTING());
    return SUCCESS;
};

static inline int get_arg_lengths(unsigned long ioctl_param, int *in_len, int *out_len)
{
    ioctl_args_stub_t local_stub, *remote_stub;

    remote_stub = (ioctl_args_stub_t *)ioctl_param;
    if(copy_from_user(&local_stub, remote_stub, sizeof(ioctl_args_stub_t))){
	printk(KERN_INFO "ERROR: could NOT extract local stub!\n");
	return -ERROR;
    }
    OUTPUT(0, KERN_INFO "OK: in_len = %d, out_len = %d\n", local_stub.in_len, local_stub.out_len);
    *in_len = local_stub.in_len; *out_len = local_stub.out_len;
    return SUCCESS;
};


#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
#define MATCH_IOCTL(num, pred) ( (num) == (pred) || (num) == (pred##32) )
#else
#define MATCH_IOCTL(num, pred) ( (num) == (pred) )
#endif

/*
 * Service IOCTL calls from user-space.
 * Handles both 32b and 64b calls.
 */
long unlocked_handle_ioctl_i(unsigned int ioctl_num, struct PWCollector_ioctl_arg *remote_args, unsigned long ioctl_param)
{
    int local_in_len, local_out_len;
    PWCollector_cmd_t cmd;
    int tmp = -1;

    /*
     * (1) Sanity check:
     * Before doing anything, double check to
     * make sure this IOCTL was really intended
     * for us!
     */
    if(_IOC_TYPE(ioctl_num) != APWR_IOCTL_MAGIC_NUM){
	printk(KERN_INFO "ERROR: requested IOCTL TYPE (%d) != APWR_IOCTL_MAGIC_NUM (%d)\n", _IOC_TYPE(ioctl_num), APWR_IOCTL_MAGIC_NUM);
	return -ERROR;
    }
    /*
     * (2) Extract arg lengths.
     */
    if(get_arg_lengths(ioctl_param, &local_in_len, &local_out_len)){
	return -ERROR;
    }
    /*
     * (3) Service individual IOCTL requests.
     */
    if(MATCH_IOCTL(ioctl_num, PW_IOCTL_CONFIG)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_CONFIG\n");
	return set_config((struct PWCollector_config *)remote_args->in_arg, local_in_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_CMD)){
	if(get_user(cmd, ((PWCollector_cmd_t *)remote_args->in_arg))){
	    printk(KERN_INFO "ERROR: could NOT extract cmd value!\n");
	    return -ERROR;
	}
	OUTPUT(0, KERN_INFO "PW_IOCTL_CMD: cmd=%d\n", cmd);
	return handle_cmd(cmd);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_STATUS)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_STATUS\n");
	/*
	 * For now, we assume STATUS information can only
	 * be retrieved for an ACTIVE collection.
	 */
	if(!IS_COLLECTING()){
	    printk(KERN_INFO "\tError: status information requested, but NO COLLECTION ONGOING!\n");
	    return -ERROR;
	}
#if DO_IOCTL_STATS
	return get_status((struct PWCollector_status *)remote_args->out_arg, local_out_len);
#else
	return -ERROR;
#endif
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_CHECK_PLATFORM)) {
	OUTPUT(0, KERN_INFO "PW_IOCTL_CHECK_PLATFORM\n");
	if( (tmp = check_platform((struct PWCollector_check_platform *)remote_args->out_arg, local_out_len)))
	    if(tmp < 0) // ERROR
		return 2; // for PW_IOCTL_CHECK_PLATFORM: >= 2 ==> Error; == 1 => SUCCESS, but not EOF; 0 ==> SUCCESS, EOF
	return tmp;
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_VERSION)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_VERSION\n");
	OUTPUT(3, KERN_INFO "OUT len = %d\n", local_out_len);
	return get_version((struct PWCollector_version_info *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_MICRO_PATCH)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_MICRO_PATCH\n");
	return get_micro_patch_ver((int *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_TURBO_THRESHOLD)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_TURBO_THRESHOLD\n");
	return get_turbo_threshold((struct PWCollector_turbo_threshold *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_AVAILABLE_FREQUENCIES)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_AVAILABLE_FREQUENCIES\n");
	return get_available_frequencies((struct PWCollector_available_frequencies *)remote_args->out_arg, local_out_len);
    }
    else{
	// ERROR!
	printk(KERN_INFO "Invalid IOCTL command = %u\n", ioctl_num);
	return -ERROR;
    }
    /*
     * Should NEVER reach here!
     */
    return -ERROR;
};

/*
 * (1) Handle 32b IOCTLs in 32b kernel-space.
 * (2) Handle 64b IOCTLs in 64b kernel-space.
 */
long device_unlocked_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
    OUTPUT(3, KERN_INFO "64b transfering to handler!\n");
    return unlocked_handle_ioctl_i(ioctl_num, (struct PWCollector_ioctl_arg *)ioctl_param, ioctl_param);
};

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
/*
 * Handle 32b IOCTLs in 64b kernel-space.
 */
long device_compat_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct PWCollector_ioctl_arg remote_args;
    struct PWCollector_ioctl_arg32 *remote_args32 = (struct PWCollector_ioctl_arg32 *)ioctl_param;

    remote_args.in_len = remote_args32->in_len;
    remote_args.in_arg = (char *)((unsigned long)remote_args32->in_arg);
    remote_args.out_len = remote_args32->out_len;
    remote_args.out_arg = (char *)((unsigned long)remote_args32->out_arg);
    /*
    remote_args.in_len = remote_args32->in_len; remote_args.in_arg = remote_args32->in_arg;
    remote_args.out_len = remote_args32->out_len; remote_args.out_arg = remote_args32->out_arg;
    */

    OUTPUT(3, KERN_INFO "32b transfering to handler!\n");

    return unlocked_handle_ioctl_i(ioctl_num, &remote_args, ioctl_param);
};
#endif // COMPAT && x64

/*
 * Service an "open(...)" call from user-space.
 */
static int device_open(struct inode *inode, struct file *file)
{
    /*
     * We don't want to talk to two processes at the same time
     */
    if(test_and_set_bit(DEV_IS_OPEN, &dev_status)){
	// Device is busy
	return -EBUSY;
    }

    try_module_get(THIS_MODULE);
    return SUCCESS;
};

/*
 * Service a "close(...)" call from user-space.
 */
static int device_release(struct inode *inode, struct file *file)
{
    OUTPUT(3, KERN_INFO "Debug: Device Release!\n");
    /*
     * Did the client just try to zombie us?
     */
    if(IS_COLLECTING()){
	printk(KERN_INFO "ERROR: Detected ongoing collection on a device release!\n");
	INTERNAL_STATE.cmd = CANCEL;
	stop_collection(CANCEL);
    }
    module_put(THIS_MODULE);
    /*
     * We're now ready for our next caller
     */
    clear_bit(DEV_IS_OPEN, &dev_status);
    return SUCCESS;
};


struct file_operations Fops = {
    .open = device_open,
    .read = device_read,
    .poll = device_poll,
    // .ioctl = device_ioctl,
    .unlocked_ioctl = &device_unlocked_ioctl,
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
    .compat_ioctl = &device_compat_ioctl,
#endif // COMPAT && x64
    .release = device_release,
};

int register_dev(void)
{
    /*
     * Register the character device (atleast try)
     */
    // int ret = register_chrdev(MAJOR_NUM, DEVICE_FILE_NAME, &Fops);
    int ret = register_chrdev(0, DEVICE_FILE_NAME, &Fops);

    apwr_dev_major_num = ret;

    /*
     * Negative values signify an error
     */
    if (ret < 0) {
	printk(KERN_ALERT "%s failed with %d\n",
	       "Sorry, registering the character device ", ret);
    }else{

	printk(KERN_INFO "Registeration is a success. The major device number is %d.", ret);
	printk(KERN_INFO "If you want to talk to the device driver,\n");
	printk(KERN_INFO "you'll have to create a device file. \n");
	printk(KERN_INFO "We suggest you use:\n");
	// printk(KERN_INFO "mknod %s c %d 0\n", DEVICE_FILE_NAME, MAJOR_NUM);
	printk(KERN_INFO "mknod %s c %d 0\n", DEVICE_FILE_NAME, ret);
	printk(KERN_INFO "The device file name is important, because\n");
	printk(KERN_INFO "the ioctl program assumes that's the\n");
	printk(KERN_INFO "file you'll use.\n");
    }

    return ret;
};

void unregister_dev(void)
{
    /*
     * Unregister the device
     */
    // unregister_chrdev(MAJOR_NUM, DEVICE_FILE_NAME);
    unregister_chrdev(apwr_dev_major_num, DEVICE_FILE_NAME);
};


/*
 * Enable CPU_CLK_UNHALTED.REF counting
 * by setting bits 8,9 in MSR_PERF_FIXED_CTR_CTRL
 * MSR (addr == 0x38d). Also store the previous
 * value of the MSR.
 */
static void enable_ref(void)
{
    int cpu;
    u64 res;
    int ret;

    u32 *data_copy;// [2];
    u32 data[2];

    for_each_online_cpu(cpu){
        /*
         * (1) Do for IA32_FIXED_CTR_CTL
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->fixed_data;
            ret = rdmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, &data[0], &data[1]);
            WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
            memcpy(data_copy, data, sizeof(u32) * 2);
            /*
             * Turn on CPU_CLK_UNHALTED.REF counting.
             *
             * UPDATE: also turn on CPU_CLK_UNHALTED.CORE counting.
             */
            // data[0] |= 0x300;
            data[0] |= 0x330;

            ret = wrmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, data[0], data[1]);
        }
        /*
         * (2) Do for IA32_PERF_GLOBAL_CTRL_ADDR
         */
        if(true){
            data_copy = (&per_cpu(CTRL_data_values, cpu))->perf_data;
            ret = rdmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, &data[0], &data[1]);
            WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
            memcpy(data_copy, data, sizeof(u32) * 2);
            res = data[1];
            res <<= 32;
            res += data[0];
            printk(KERN_INFO "[%d]: READ res = 0x%llx\n", cpu, res);
            /*
             * Turn on CPU_CLK_UNHALTED.REF counting.
             *
             * UPDATE: also turn on CPU_CLK_UNHALTED.CORE counting.
             * Set bits 33, 34
             */
            // data[0] |= 0x330;
            data[1] |= 0x6;
            // data[0] = data[1] = 0x0;

            ret = wrmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, data[0], data[1]);
        }
    }
    if(true)
        for_each_online_cpu(cpu){
            ret = rdmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, &data[0], &data[1]);
            // ret = rdmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, &data[0], &data[1]);
            res = data[1];
            res <<= 32;
            res += data[0];
            printk(KERN_INFO "[%d]: NEW res = 0x%llx\n", cpu, res);
        }
};

static void restore_ref(void)
{
    int cpu;
    u64 res;
    int ret;

    u32 *data_copy;
    u32 data[2];

    for_each_online_cpu(cpu){
        /*
         * (1) Do for IA32_FIXED_CTR_CTL
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->fixed_data;
            memcpy(data, data_copy, sizeof(u32) * 2);

            res = data[1];
            res <<= 32;
            res += data[0];

            OUTPUT(3, KERN_INFO "[%d]: PREV res = 0x%llx\n", cpu, res);
            if( (ret = wrmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, data[0], data[1]))){
                printk(KERN_INFO "ERROR writing PREVIOUS IA32_FIXED_CTR_CLT_ADDR values for CPU = %d!\n", cpu);
            }
        }
        /*
         * (2) Do for IA32_PERF_GLOBAL_CTRL_ADDR
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->perf_data;
            memcpy(data, data_copy, sizeof(u32) * 2);

            res = data[1];
            res <<= 32;
            res += data[0];

            OUTPUT(3, KERN_INFO "[%d]: PREV res = 0x%llx\n", cpu, res);
            if( (ret = wrmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, data[0], data[1]))){
                printk(KERN_INFO "ERROR writing PREVIOUS IA32_PERF_GLOBAL_CTRL_ADDR values for CPU = %d!\n", cpu);
            }
        }
    }
};

/*
 * Check if we're running on ATM.
 */
bool is_atm(void)
{
    unsigned int ecx, edx;
    unsigned int fms, family, model, stepping;

    asm("cpuid" : "=a" (fms), "=c" (ecx), "=d" (edx) : "a" (1) : "ebx");

    family = (fms >> 8) & 0xf;
    model = (fms >> 4) & 0xf;
    stepping = fms & 0xf;

    if (family == 6 || family == 0xf){
        model += ((fms >> 16) & 0xf) << 4;
    }
    printk(KERN_INFO "FMS = 0x%x:%x:%x (%d:%d:%d)\n", family, model, stepping, family, model, stepping);
    /*
     * This check below will need to
     * be updated for each new
     * architecture type!!!
     */
    if (family == 0x6 && model == 0x27) {
        // MFLD
        return true;
    }
    return false;
};

static int __init init_hooks(void)
{
    int ret = 0;

    printk(KERN_INFO "# IRQS = %d\n", NR_IRQS);

    OUTPUT(0, KERN_INFO "Sizeof PWCollector_sample_t = %lu, Sizeof k_sample_t = %lu\n", sizeof(PWCollector_sample_t), sizeof(k_sample_t));

    /*
     * We first check to see if
     * TRACEPOINTS are ENABLED in the kernel.
     * If not, EXIT IMMEDIATELY!
     */
#ifdef CONFIG_TRACEPOINTS
    OUTPUT(0, KERN_INFO "Tracepoints ON!\n");
#else
    printk(KERN_INFO "ERROR: TRACEPOINTS NOT found on system!!!\n");
    return -ERROR;
#endif

    /*
     * Check if we're running on ATM.
     */
    PW_is_atm = is_atm();

    /*
     * For MFLD, we also check
     * if the required microcode patches
     * have been installed. If
     * not then EXIT IMMEDIATELY!
     */
#if DO_CHECK_BO_MICROCODE_PATCH
    {
	/*
	 * Read MSR 0x8b -- if microcode patch
	 * has been applied then the first 12 bits
	 * of the higher order 32 bits should be
	 * >= 0x102.
	 *
	 * THIS CHECK VALID FOR ATM ONLY!!!
	 */
	/*
	 * Do check ONLY if we're ATM!
	 */
	if(PW_is_atm){
	    u64 res;
	    u32 patch_val;

	    rdmsrl(0x8b, res);
	    patch_val = (res >> 32) & 0xfff;
	    if(patch_val < 0x102){
		printk(KERN_INFO "ERROR: B0 micro code path = 0x%x: REQUIRED >= 0x102!!!\n", patch_val);
		return -ERROR;
	    }
	    micro_patch_ver = patch_val;
	    OUTPUT(3, KERN_INFO "patch ver = %u\n", micro_patch_ver);
	}else{
	    OUTPUT(0, KERN_INFO "DEBUG: SKIPPING MICROCODE PATCH check -- NON ATM DETECTED!\n");
	}
    }
#endif

    OUTPUT(3, KERN_INFO "Sizeof node = %lu\n", sizeof(tnode_t));
    OUTPUT(3, KERN_INFO "Sizeof per_cpu_t = %lu\n", sizeof(per_cpu_t));

    startJIFF = jiffies;

    if(init_data_structures())
	return -ERROR;


#if 0
    {
	get_base_operating_frequency();
    }
#endif

    {
	enable_ref();
    }

#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
    if(PW_is_atm){
        // Map the bus memory into CPU space for 6 IPC registers
        mmio_ipc1_base = ioremap_nocache(IPC_BASE_ADDRESS, IPC_MAX_ADDR);

        if (mmio_ipc1_base == NULL) {
                printk(KERN_ERR "ioremap_nocache returns NULL! IPC1 is NOT available\n");
                return -ERROR;
        }
#endif

#if DO_S_RESIDENCY_SAMPLE
        // Map the bus memory into CPU space for 4 S state residency counters
        mmio_s_residency_base = ioremap_nocache(S_RESIDENCY_BASE_ADDRESS, S_RESIDENCY_MAX_COUNTERS*4);

        if (mmio_s_residency_base == NULL) {
                printk(KERN_ERR "ioremap_nocache returns NULL! S Residency counter is NOT available\n");
                return -ERROR;
        }
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
        // Map the bus memory into CPU space for 4(bytes) * 3(states) * 40(LSS) D state residency counters
        mmio_d_residency_base = ioremap_nocache(D_RESIDENCY_BASE_ADDRESS, D_RESIDENCY_MAX_COUNTERS*4);

        if (mmio_d_residency_base == NULL) {
                printk(KERN_ERR "ioremap_nocache returns NULL! D Residency counter is NOT available\n");
                return -ERROR;
        }
#endif

#if DO_S_STATE_SAMPLE || DO_D_SC_STATE_SAMPLE
    if(PW_is_atm){
        // Map the bus memory into CPU space for power management
        mmio_pm_base = ioremap_nocache(PM_BASE_ADDRESS, PM_MAX_ADDR);

        if (mmio_pm_base == NULL) {
                printk(KERN_ERR "ioremap_nocache returns NULL! PM is NOT available\n");
                return -ERROR;
        }
    }
#endif

    {
	/*
	 * Check if kernel-space call stack generation
	 * is possible.
	 */
#ifdef CONFIG_FRAME_POINTER
	OUTPUT(0, KERN_INFO "Frame pointer ON!\n");
	INTERNAL_STATE.have_kernel_frame_pointers = true;
#else
	printk(KERN_INFO "**********************************************************************************************************\n");
	printk(KERN_INFO "Error: kernel NOT compiled with frame pointers -- NO KERNEL-SPACE TIMER CALL TRACES WILL BE GENERATED!\n");
	printk(KERN_INFO "**********************************************************************************************************\n");
	INTERNAL_STATE.have_kernel_frame_pointers = false;
#endif
    }

#if ALLOW_BLOCKING_READ
    {
	init_waitqueue_head(&read_queue);
    }
#endif

    /*
     * "Register" the device-specific special character file here.
     */
    {
	if( (ret = register_dev()) < 0)
	    return ret;
    }

    /*
     * Probes required to cache (kernel) timer
     * callstacks need to be inserted, regardless
     * of collection status.
     */
    {
	// register_timer_callstack_probes();
	register_permanent_probes();
    }

#if 0
    {
	register_all_probes();
    }
#endif

    printk(KERN_INFO "\n--------------------------------------------------------------------------------------------\n");
    printk(KERN_INFO "START Initialized the DRIVER\n");
    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");

    return SUCCESS;
};

static void __exit cleanup_hooks(void)
{
    unsigned long elapsedJIFF = 0, collectJIFF = 0;
    int num_timers = 0, num_irqs = 0;

    {
	unregister_dev();
    }

#if DO_S_RESIDENCY_SAMPLE
    if(mmio_s_residency_base != NULL && PW_is_atm){
	stop_s_residency_counter();

        iounmap(mmio_s_residency_base);
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
    if(mmio_d_residency_base != NULL && PW_is_atm){
	stop_d_sc_residency_counter();

        iounmap(mmio_d_residency_base);
    }
#endif

#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
    if(mmio_ipc1_base != NULL && PW_is_atm){
        iounmap(mmio_ipc1_base);
    }
#endif

#if DO_S_STATE_SAMPLE || DO_D_SC_STATE_SAMPLE
    if(mmio_pm_base != NULL && PW_is_atm){
        iounmap(mmio_pm_base);
    }
#endif


    /*
     * Probes required to cache (kernel) timer
     * callstacks need to be removed, regardless
     * of collection status.
     */
    {
	// unregister_timer_callstack_probes();
	unregister_permanent_probes();
    }

#if 1
    if(IS_COLLECTING()){
	// unregister_all_probes();
	unregister_non_pausable_probes();
	unregister_pausable_probes();
    }
    else if(IS_SLEEPING()){
	unregister_pausable_probes();
    }
#else
    /*
     * Forcibly unregister -- used in debugging.
     */
    {
	unregister_all_probes();
    }
#endif


    {
	num_timers = get_num_timers();
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
	num_irqs = get_num_irq_mappings();
#endif
    }

    {
	destroy_data_structures();
    }

    {
	restore_ref();
    }

    /*
     * Collect some statistics: total execution time.
     */
    stopJIFF = jiffies;
    if(stopJIFF < startJIFF){
	printk(KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
	elapsedJIFF = 0; // avoid messy NAN when dividing
    }else{
	elapsedJIFF = stopJIFF - startJIFF;
    }

    /*
     * Collect some collection statistics: total collection time.
     */
    if(INTERNAL_STATE.collectionStopJIFF < INTERNAL_STATE.collectionStartJIFF){
	OUTPUT(0, KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
	collectJIFF = 0;
    }else{
	collectJIFF = INTERNAL_STATE.collectionStopJIFF - INTERNAL_STATE.collectionStartJIFF;
    }

    printk(KERN_INFO "\n--------------------------------------------------------------------------------------------\n");

    printk(KERN_INFO "STOP Cleaned up the probes\n");
    printk(KERN_INFO "Total time elapsed = %u msecs, Total collection time = %u msecs\n", jiffies_to_msecs(elapsedJIFF), jiffies_to_msecs(collectJIFF));

    printk(KERN_INFO "Total # timers = %d, Total # irq mappings = %d\n", num_timers, num_irqs);

#if DO_OVERHEAD_MEASUREMENTS
    {
	timer_init_print_cumulative_overhead_params("TIMER_INIT");
	timer_expire_print_cumulative_overhead_params("TIMER_EXPIRE");
	timer_insert_print_cumulative_overhead_params("TIMER_INSERT");
	tps_print_cumulative_overhead_params("TPS");
	tpf_print_cumulative_overhead_params("TPF");
	inter_common_print_cumulative_overhead_params("INTER_COMMON");
	irq_insert_print_cumulative_overhead_params("IRQ_INSERT");
	find_irq_node_i_print_cumulative_overhead_params("FIND_IRQ_NODE_I");
	exit_helper_print_cumulative_overhead_params("EXIT_HELPER");
	timer_delete_print_cumulative_overhead_params("TIMER_DELETE");
	/*
	 * Also print stats on timer entries.
	 */
	printk(KERN_INFO "# TIMER ENTRIES = %d\n", atomic_read(&num_timer_entries));
	/*
	 * And some mem debugging stats.
	 */
	printk(KERN_INFO "TOTAL # BYTES ALLOCED = %llu, CURR # BYTES ALLOCED = %llu, MAX # BYTES ALLOCED = %llu\n", TOTAL_NUM_BYTES_ALLOCED(), CURR_NUM_BYTES_ALLOCED(), MAX_NUM_BYTES_ALLOCED());
    }
#endif // DO_OVERHEAD_MEASUREMENTS

    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");
};

module_init(init_hooks);
module_exit(cleanup_hooks);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
