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
 * Current driver version is 2.2.3
 * Current driver version is 2.2.4
 * Current driver version is 2.2.5
 * Current driver version is 2.2.6
 * Current driver version is 2.2.7
 * Current driver version is 3.0.0
 * Current driver version is 3.0.1
 * Current driver version is 3.0.2
 */
#define PW_VERSION_VERSION 3
#define PW_VERSION_INTERFACE 0
#define PW_VERSION_OTHER 3

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/smp.h> // For smp_call_function

#include <asm/local.h>
#include <asm/cputime.h> // For ktime
#include <asm/io.h> // For ioremap, read, and write
#include <asm/intel_scu_ipc.h> // For intel_scu_ipc_command

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
#include <linux/cdev.h>
#include <linux/device.h>
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
 * The driver creates its own device node
 * with 0600 permission
 */
#define CREATE_DEVICE_NODE 1
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
 * Should we print some stats at the end of a collection?
 * '1' ==> YES, print stats
 * '0' ==> NO, do NOT print stats
 */
#define DO_PRINT_COLLECTION_STATS 0
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
 * Do we use APERF, MPERF for
 * dynamic freq calculations?
 * '1' ==> YES, use APERF, MPERF
 * '0' ==> NO, use IA32_FIXED_CTR{1,2}
 */
#define USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY 0
/*
 * PERF_STATUS MSR addr -- bits 12:8, multiplied by the
 * bus clock freq, give the freq the H/W is currently
 * executing at.
 */
#define IA32_PERF_STATUS_MSR_ADDR 0x198
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
 * By default, S-state sample collection is intentionally
 * disabled because it will be always S0 state because the collection
 * will always work when CPU is running.
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
 * By default, South complex D-state sample collection is
 * intentionally disabled because we can obtain the same
 * information from South complex D-state residency counters
 * more accurately.
 */
// #define DO_D_SC_STATE_SAMPLE 1
#define DO_D_SC_STATE_SAMPLE 0
/*
 * Use the predefined SCU IPC functions to access to SCU
 * The intel_scu_ipc driver uses mutex lock which makes the system hanging
 * when used in sched_switch. So, disable it for now.
 */
#define USE_PREDEFINED_SCU_IPC 0

/*
 * Run the p-state sample generation in parallel for all CPUs
 * at the beginning and the end to avoid any delay
 * due to serial execution
 */
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
 * Is this a "root" timer?
 */
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
    #define IS_ROOT_TIMER(tid) ( (tid) == 0 || !is_tid_in_sys_list(tid) )
#else
    #define IS_ROOT_TIMER(tid) ( (tid) == 0 )
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT
/*
 * Compare-and-swap.
 */
#define LOCAL_CAS(l, o, n) local_cmpxchg((l), (o), (n)) == (o)
/*
 * Produce a "sample".
 * @cpu: the cpu on which the underlying event occured.
 * @type: the event type; one of "event_type_t"
 * @should_wake: should we wake any waiting readers?
 */
#define record_hit(cpu, type, should_wake) do { \
	bool should_produce_sample = LOCAL_CAS(&get_cpu_var(is_first_event), 1, 0); \
	put_cpu_var(is_first_event); \
	if (should_produce_sample) { \
		u64 tsc; \
		tscval(&tsc); \
		/*tsc = cpu_clock(cpu);*/ \
		produce_sample_i(cpu, (type), tsc, (should_wake)); \
	} \
} while(0)
/*
 * Produce a "sample", allowing the underlying algorithm to awaken
 * any waiting readers.
 */
#define record_hit_wakeup(cpu, type) record_hit((cpu), (type), true)
/*
 * Produce a "sample", but do NOT call any "sleeping"
 * functions. This macro will NOT try to wake up any
 * tasks/processes that are currently waiting for data.
 */
#define record_hit_no_wakeup(cpu, type) record_hit((cpu), (type), false)
/*
 * Copy a "sample".
 * @cpu: the cpu on which the corresponding sample was generated.
 * @sample: the sample to copy.
 */
#define record_hit_sample(cpu, sample) do { \
	bool should_produce_sample = LOCAL_CAS(&get_cpu_var(is_first_event), 1, 0); \
	put_cpu_var(is_first_event); \
	if (should_produce_sample) { \
		copy_sample_i(cpu, sample, true); \
	} \
} while(0)
/*
 * Copy a "sample", but do NOT call any "sleeping" functions.
 * @cpu: the cpu on which the corresponding sample was generated.
 * @sample: the sample to copy.
 */
#define record_hit_sample_no_wakeup(cpu, sample) do { \
	bool should_produce_sample = LOCAL_CAS(&get_cpu_var(is_first_event), 1, 0); \
	put_cpu_var(is_first_event); \
	if (should_produce_sample) { \
		copy_sample_i(cpu, sample, false); \
	} \
} while(0)

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

// Address for S state residency counters
#define S_RESIDENCY_BASE_ADDRESS        0xFFFF71E0
#define S_RESIDENCY_MAX_COUNTERS        0x4
// Address for D state residency counters
#define D_RESIDENCY_BASE_ADDRESS        0xFFFF7000
#define D_RESIDENCY_MAX_COUNTERS        0x78    // 40 LSS * 3 D states = 120
// Address for cumulative residency counter
#define CUMULATIVE_RESIDENCY_ADDRESS    0xFFFF71EC

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
// memory mapped io base address for subsystem power management
static void *mmio_pm_base = NULL;
#endif
#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
// memory mapped io base address for IPC-1
static void *mmio_ipc1_base = NULL;
// memory mapped io base address for Cumulative Residency counter
static void *mmio_cumulative_residency_base = NULL;
#endif
#if DO_S_RESIDENCY_SAMPLE
// memory mapped io base address for S0ix Residency counters
static void *mmio_s_residency_base = NULL;
#endif
#if DO_D_SC_RESIDENCY_SAMPLE
// memory mapped io base address for D0ix Residency counters
static void *mmio_d_residency_base = NULL;
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
    u16 is_root_timer : 1;
    u16 trace_sent : 1;
    u16 trace_len : 14;
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
 * For tracking device driver loads
 * and removals i.e. insmods and rmmods.
 */
typedef struct mod_node mod_node_t;
struct mod_node{
    struct list_head list;
    struct rcu_head rcu;
    pid_t tid, pid;
    const char *name;
};

/*
 * For syscall nodes
 */
typedef struct sys_node sys_node_t;
struct sys_node{
    struct hlist_node list;
    pid_t tid, pid;
    int ref_count, weight;
};

#define SYS_MAP_BUCKETS_BITS 9
#define NUM_SYS_MAP_BUCKETS (1UL << SYS_MAP_BUCKETS_BITS) // MUST be pow-of-2
#define SYS_MAP_LOCK_BITS 4
#define NUM_SYS_MAP_LOCKS (1UL << SYS_MAP_LOCK_BITS) // MUST be pow-of-2

#define SYS_MAP_NODES_HASH(t) hash_32(t, SYS_MAP_BUCKETS_BITS)
#define SYS_MAP_LOCK_HASH(t) ( (t) & (SYS_MAP_LOCK_BITS - 1) ) // pow-of-2 modulo

#define SYS_MAP_LOCK(index) LOCK(apwr_sys_map_locks[index])
#define SYS_MAP_UNLOCK(index) UNLOCK(apwr_sys_map_locks[index])

#define GET_SYS_HLIST(index) (apwr_sys_map + index)


/*
 * Function declarations (incomplete).
 */
/*
bool is_sleep_syscall_i(long id);
void sys_enter_helper_i(long id, pid_t tid, pid_t pid);
void sys_exit_helper_i(long id, pid_t tid, pid_t pid);
void sched_wakeup_helper_i(struct task_struct *task);
*/

/*
 * Variable declarations.
 */

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

static DEFINE_PER_CPU(u64, num_local_apic_timer_inters) = 0;

/*
 * TPS helper -- required for overhead
 * measurements.
 */
#if DO_IOCTL_STATS
static DEFINE_PER_CPU(u64, num_inters) = 0;
#endif

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
 * Character device file MAJOR
 * number -- we're now obtaining
 * this dynamically.
 */
static int apwr_dev_major_num = -1;
/*
 * Atomic counter used to synchronize TPS probes and
 * sched wakeups on other cores.
 */
#if DO_TPS_EPOCH_COUNTER
static atomic_t tps_epoch = ATOMIC_INIT(0);
#endif // DO_TPS_EPOCH_COUNTER

#if CREATE_DEVICE_NODE
/*
 * Variables to create the character device file
 */
static dev_t apwr_dev;
static struct cdev *apwr_cdev;
static struct class *apwr_class = NULL;
#endif


#if DO_OVERHEAD_MEASUREMENTS
/*
 * Counter to count # of entries
 * in the timer hash map -- used
 * for debugging.
 */
static atomic_t num_timer_entries = ATOMIC_INIT(0);
#endif
/*
 * SORTED list of all device drivers loaded into
 * the kernel since the power driver was loaded.
 */
LIST_HEAD(apwr_mod_list);
/*
 * Spinlock to guard updates to mod list.
 */
static DEFINE_SPINLOCK(apwr_mod_list_lock);
/*
 * The sys map. Individual buckets are unordered.
 */
static struct hlist_head apwr_sys_map[NUM_SYS_MAP_BUCKETS];
/*
 * Spinlock to guard updates to sys map.
 */
static spinlock_t apwr_sys_map_locks[NUM_SYS_MAP_LOCKS];
/*
 * These are used for the 'hrtimer_start(...)'
 * hack.
 */
static u32 tick_count = 0;
static DEFINE_SPINLOCK(tick_count_lock);
static bool should_probe_on_hrtimer_start = true;

DEFINE_PER_CPU(local_t, sched_timer_found) = LOCAL_INIT(0);

static DEFINE_PER_CPU(local_t, num_samples_produced) = LOCAL_INIT(0);
static DEFINE_PER_CPU(local_t, num_samples_dropped) = LOCAL_INIT(0);
static u64 total_num_samples_produced = 0, total_num_samples_dropped = 0;

/*
 * Used to record which wakeup event occured first.
 * Reset on every TPS.
 */
static DEFINE_PER_CPU(local_t, is_first_event) = LOCAL_INIT(1);

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
DECLARE_OVERHEAD_VARS(sys_enter_helper_i);
DECLARE_OVERHEAD_VARS(sys_exit_helper_i);

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

// Read IPC status register
static inline u32 ipc_status(void)
{
    if (mmio_ipc1_base != NULL)
        return readl(mmio_ipc1_base + IPC_STS_OFFSET);

    printk(KERN_ERR "mmio_ipc1_base is NULL!\n");
    return ERROR;
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
            OUTPUT(0, KERN_INFO "[APWR] IPC is busy.\n");
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
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_START_RESIDENCY, NULL, 0, NULL, 0);
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
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_STOP_RESIDENCY, NULL, 0, NULL, 0);
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
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_DUMP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_DUMP_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    if(ret != SUCCESS) {
        OUTPUT(0, KERN_ERR "Error: dump_s_residency_counter!\n");
    }

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
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_START_RESIDENCY, NULL, 0, NULL, 0);
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
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_STOP_RESIDENCY, NULL, 0, NULL, 0);
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
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_DUMP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_DUMP_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    if(ret != SUCCESS) {
        OUTPUT(0, KERN_ERR "Error: dump_d_sc_residency_counter!\n");
    }


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

    for (i=0; i<NUM_HASH_LOCKS; ++i) {
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

static void destroy_irq_map(void)
{
    int i=0;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
        struct hlist_head *head = &irq_map[i].head;
        while(!hlist_empty(head)){
            struct irq_node *node = hlist_entry(head->first, struct irq_node, list);
            hlist_del(&node->list);
            irq_destroy_callback(&node->rcu);
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

void free_sys_node_i(sys_node_t *node)
{
    if (!node) {
	return;
    }
    pw_kfree(node);
};

sys_node_t *alloc_new_sys_node_i(pid_t tid, pid_t pid)
{
    sys_node_t *node = pw_kmalloc(sizeof(sys_node_t), GFP_ATOMIC);
    if (!node) {
	printk(KERN_INFO "ERROR: could NOT allocate new sys node!\n");
	return NULL;
    }
    node->tid = tid; node->pid = pid;
    node->ref_count = node->weight = 1;
    INIT_HLIST_NODE(&node->list);
    return node;
};

int destroy_sys_list(void)
{
    int size = 0, i=0;

    for (i=0; i<NUM_SYS_MAP_BUCKETS; ++i) {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(i);
	int tmp_size = 0;
	while (!hlist_empty(apwr_sys_list)) {
	    sys_node_t *node = hlist_entry(apwr_sys_list->first, struct sys_node, list);
	    hlist_del(&node->list);
	    ++tmp_size;
	    free_sys_node_i(node);
	    ++size;
	}
	if (tmp_size) {
	    OUTPUT(3, KERN_INFO "[%d] --> %d\n", i, tmp_size);
	}
    }

#if DO_PRINT_COLLECTION_STATS
    printk(KERN_INFO "SYS_LIST_SIZE = %d\n", size);
#endif

    return SUCCESS;
};

int init_sys_list(void)
{
    int i=0;

    for (i=0; i<NUM_SYS_MAP_BUCKETS; ++i) {
	INIT_HLIST_HEAD(GET_SYS_HLIST(i));
    }

    for (i=0; i<NUM_SYS_MAP_LOCKS; ++i) {
	spin_lock_init(apwr_sys_map_locks + i);
    }

    return SUCCESS;
};

void destroy_mod_list(void)
{
    int size = 0;
    while (!list_empty(&apwr_mod_list)) {
        struct mod_node *node = list_first_entry(&apwr_mod_list, struct mod_node, list);
        list_del(&node->list);
        pw_kfree(node);
        ++size;
    }
#if DO_PRINT_COLLECTION_STATS
    printk(KERN_INFO "MOD_LIST_SIZE = %d\n", size);
#endif
};

int init_mod_list(void)
{
    // NOP
    return SUCCESS;
};

/*
 * Gather and print some stats on # of dropped samples.
 */
void count_samples_produced_dropped(void)
{
    int cpu = 0;
    total_num_samples_produced = total_num_samples_dropped = 0;
    for_each_online_cpu(cpu) {
        total_num_samples_produced += local_read(&per_cpu(num_samples_produced, cpu));
        total_num_samples_dropped += local_read(&per_cpu(num_samples_dropped, cpu));
    }
};

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

    destroy_mod_list();
    destroy_sys_list();

    {
        /*
         * Print some stats about # samples produced and # dropped.
         */
#if DO_PRINT_COLLECTION_STATS
        printk(KERN_INFO "DEBUG: There were %llu / %llu dropped samples!\n", total_num_samples_dropped, total_num_samples_produced);
#endif
    }
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
	/*
	 * First, allocate space for our 'lists' array.
	 */
	lists = (list_t **)pw_kmalloc(sizeof(list_t *) * PW_max_num_cpus, GFP_KERNEL);
	if (!lists) {
	    printk(KERN_INFO "ERROR: could NOT allocate array-of-lists!\n");
	    return -ERROR;
	}
	memset(lists, 0, sizeof(list_t *) * PW_max_num_cpus);
	/*
	 * Then allocate space for each 'list'.
	 */
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

    if (init_mod_list()) {
        printk(KERN_INFO "ERROR: could NOT init mod list!\n");
        destroy_data_structures();
        return -ERROR;
    }

    if (init_sys_list()) {
        printk(KERN_INFO "ERROR: could NOT initialize syscall map!\n");
        destroy_data_structures();
        return -ERROR;
    }
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
        /*
         * Root timer!
         */
        node->is_root_timer = 1;
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
                    if (node->is_root_timer == 0) {
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
		    if(node->is_root_timer == 0 && node->tid == tid){
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
 * MOD list manipulation routines.
 */

void mod_node_destroy_callback_i(struct rcu_head *head)
{
    struct mod_node *node = container_of(head, struct mod_node, rcu);
    if (node) {
        pw_kfree(node);
    }
};
/*
 * Helper function to check if a given TID corresponds
 * to that of a device driver (which inherits pid,tid of
 * the underlying 'insmod' process).
 */
bool is_tid_in_mod_list(pid_t tid)
{
    mod_node_t *node;
    bool retVal = false;
    rcu_read_lock();
    {
        list_for_each_entry_rcu(node, &apwr_mod_list, list){
            if (node->tid >= tid) {
                retVal = node->tid == tid;
                break;
            }
        }
    }
    rcu_read_unlock();
    return retVal;
};

/*
 * Helper function to add a new device driver record to our
 * list of drivers.
 */
int add_new_module_to_mod_list(pid_t tid, pid_t pid, const struct module *mod)
{
    mod_node_t *node = NULL;

    if (mod == NULL) {
        printk(KERN_INFO "ERROR: CANNOT add NULL module to list!\n");
        return -ERROR;
    }

    node = pw_kmalloc(sizeof(mod_node_t), GFP_KERNEL);
    if (node == NULL) {
        printk(KERN_INFO "ERROR: could NOT allocate a new mod node!\n");
        return -ERROR;
    }
    node->tid = tid; node->pid = pid;
    node->name = mod->name;

    LOCK(apwr_mod_list_lock);
    {
        struct mod_node *__tmp_node = NULL;
        bool inserted = false;
        list_for_each_entry(__tmp_node, &apwr_mod_list, list){
            if (__tmp_node->tid > tid) {
                list_add_tail_rcu(&node->list, &__tmp_node->list);
                inserted = true;
                break;
            }
        }
        if (!inserted) {
            list_add_tail_rcu(&node->list, &apwr_mod_list);
        }
    }
    UNLOCK(apwr_mod_list_lock);
    return SUCCESS;
};

/*
 * Helper function to remove a device driver record from
 * our list of device drivers.
 */
int remove_module_from_mod_list(const struct module *mod)
{
    const char *name = mod->name;
    struct mod_node *node = NULL;
    struct mod_node *tmp_node = NULL;
    bool removed = false;

    LOCK(apwr_mod_list_lock);
    {
        list_for_each_entry_safe(node, tmp_node, &apwr_mod_list, list){
            if (strcmp(node->name, name) == 0) {
                list_del_rcu(&node->list);
                // call_rcu(&node->rcu, &mod_node_destroy_callback_i);
                removed = true;
                break;
            }
        }
    }
    UNLOCK(apwr_mod_list_lock);
    if (!removed) {
        printk(KERN_INFO "ERROR: could NOT remove module = %s from list!\n", name);
        return -ERROR;
    } else {
        synchronize_rcu();
        pw_kfree(node);
    }
    return SUCCESS;
};

/*
 * SYS map manipulation routines.
 */

inline bool is_tid_in_sys_list(pid_t tid)
{
    sys_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    bool found = false;

    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        hlist_for_each_entry (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid) {
		found = true;
		break;
	    }
	}
    }
    SYS_MAP_UNLOCK(lindex);

    return found;
};

inline int check_and_remove_proc_from_sys_list(pid_t tid, pid_t pid)
{
    sys_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    bool found = false;
    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        hlist_for_each_entry (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid && node->ref_count > 0) {
		found = true;
		--node->ref_count;
		break;
	    }
	}
    }
    SYS_MAP_UNLOCK(lindex);

    if (!found) {
	return -ERROR;
    }
    return SUCCESS;
};

inline int check_and_delete_proc_from_sys_list(pid_t tid, pid_t pid)
{
    sys_node_t *node = NULL;
    bool found = false;
    struct hlist_node *curr = NULL;
    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        hlist_for_each_entry (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid) {
		found = true;
		hlist_del(&node->list);
                OUTPUT(3, KERN_INFO "CHECK_AND_DELETE: successfully deleted node: tid = %d, ref_count = %d, weight = %d\n", tid, node->ref_count, node->weight);
		free_sys_node_i(node);
		break;
	    }
	}
    }
    SYS_MAP_UNLOCK(lindex);

    if (!found) {
	return -ERROR;
    }
    return SUCCESS;
};

inline int check_and_add_proc_to_sys_list(pid_t tid, pid_t pid)
{
    sys_node_t *node = NULL;
    bool found = false;
    int retVal = SUCCESS;
    struct hlist_node *curr = NULL;
    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        hlist_for_each_entry (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid) {
		found = true;
		++node->ref_count;
		++node->weight;
		break;
	    }
	}
        if (!found){
	    node = alloc_new_sys_node_i(tid, pid);
	    if (!node) {
		printk(KERN_INFO "ERROR: could NOT allocate new node!\n");
		retVal = -ERROR;
	    } else {
		hlist_add_head(&node->list, apwr_sys_list);
	    }
        }
    }
    SYS_MAP_UNLOCK(lindex);
    return retVal;
};


void print_sys_node_i(sys_node_t *node)
{
    printk(KERN_INFO "SYS_NODE: %d -> %d, %d\n", node->tid, node->ref_count, node->weight);
};



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
static inline seg_t *find_producer_seg(int cpu, bool *should_wake)
{
    seg_t *seg = NULL;
    int list_index = -1, num_full = 0;
    list_t *list = GET_OUTPUT_LIST(cpu);

    {
        local_inc(&get_cpu_var(num_samples_produced));
        put_cpu_var(num_samples_produced);
    }

    do{
        list_index = list->prod_index;
        seg = &list->segs[list_index];
        smp_mb();
        if(atomic_read(&seg->is_full) == FULL){
            OUTPUT(0, KERN_INFO "[%d]: Segment %d is FULL! # full = %d\n", cpu, list_index, num_full);
            list->prod_index = CIRCULAR_INC(list_index, LIST_MASK);
            *should_wake = true;
            continue;
        }
        return seg;
    }while(++num_full < NUM_SEGS_PER_LIST);

    /*
     * Only reaches here if we couldn't find a candidate
     * 'seg'.
     */
    OUTPUT(0, KERN_INFO "WARNING: List for CPU = %d was FULL!\n", cpu);
    {
        local_inc(&get_cpu_var(num_samples_dropped));
        put_cpu_var(num_samples_dropped);
    }

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
        seg = find_producer_seg(cpu, &should_wake);
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
            OUTPUT(0, KERN_ERR "Error: No buffer available for cpu=%d, sample type = %d!\n", cpu, sample->sample_type);
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
 * Helper function that inserts data
 * into the (per-cpu) output buffers.
 */
static inline void produce_pwr_sample_no_wakeup(int cpu, const struct PWCollector_sample *sample)
{
    seg_t *seg = NULL;
    bool dummy;
    /*
     * Enter an RCU read-side critical
     * section.
     */
    BEGIN_PRODUCING();
    {
        /*
         * Find an output segment.
         */
        seg = find_producer_seg(cpu, &dummy);
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
            }
        }else{
            /*
             * Do NOT exit prematurely. Instead, signal
             * an error and wait until we leave the
             * RCU read-side critical section.
             */
            OUTPUT(0, KERN_ERR "Error: No buffer available for cpu=%d, sample type = %d!\n", cpu, sample->sample_type);
        }
    }
    STOP_PRODUCING();
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

void produce_sample_i(int cpu, u32 event_type, u64 tsc, bool should_wake)
{
    PWCollector_sample_t output_sample;
    output_sample.cpuidx = cpu;
    output_sample.sample_type = event_type;
    output_sample.tsc = tsc;

    if (should_wake) {
        produce_pwr_sample(cpu, &output_sample);
    } else {
        produce_pwr_sample_no_wakeup(cpu, &output_sample);
    }

    return;
};

void copy_sample_i(int cpu, PWCollector_sample_t *output_sample, bool should_wake)
{
    if (should_wake) {
        produce_pwr_sample(cpu, output_sample);
    } else {
        produce_pwr_sample_no_wakeup(cpu, output_sample);
    }
};

#if DO_S_RESIDENCY_SAMPLE
/*
 * Insert a S Residency counter sample into a (per-cpu) output buffer.
 */
static inline void produce_s_residency_sample(u64 usec)
{
    u64 tsc;
    int cpu = raw_smp_processor_id();

    PWCollector_sample_t sample;

    /*
     * No residency counters available
     */
    tscval(&tsc);
    sample.sample_type = S_RESIDENCY;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.s_residency_sample.usec = (unsigned int)usec;
    if (usec) {
        memcpy(sample.s_residency_sample.counters, mmio_s_residency_base, sizeof(u32) * 3);
    } else {
        memset(sample.s_residency_sample.counters, 0, sizeof(u32) * 3);
    }

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};
#endif


#if DO_D_SC_RESIDENCY_SAMPLE
/*
 * Insert a D Residency counter sample into a (per-cpu) output buffer.
 */
static inline void produce_d_sc_residency_sample(u64 usec)
{
    u64 tsc, msec;
    int cpu = raw_smp_processor_id();
    int i,j;
    int start_idx = 0;
    int num = 0;
    int numofsamples = (d_sc_count_num + 5) / 6;  // Currently, 6 devices info can be included into a single sample

    tscval(&tsc);
    msec = readl(mmio_cumulative_residency_base);

    for (i=0; i<numofsamples; i++) {
        PWCollector_sample_t sample;
        num = 0;

        sample.sample_type = D_RESIDENCY;
        sample.cpuidx = cpu;
        sample.tsc = tsc;
        sample.d_residency_sample.device_type = PW_SOUTH_COMPLEX;

        for (j=start_idx; j<MAX_LSS_NUM_IN_SC; j++) {
            if ((d_sc_mask >> j) & 0x1) {
                sample.d_residency_sample.mask[num] = j;
                sample.d_residency_sample.d_residency_counters[num].usec = (!usec) ? 0 : (unsigned int)msec*1000;
                sample.d_residency_sample.d_residency_counters[num].D0i0_ACG = (!usec) ? 0 : readl(mmio_d_residency_base + sizeof(u32)*j);
                sample.d_residency_sample.d_residency_counters[num].D0i1 = (!usec) ? 0 : readl(mmio_d_residency_base + 40*4 + sizeof(u32)*j);
                sample.d_residency_sample.d_residency_counters[num].D0i3 = (!usec) ? 0 : readl(mmio_d_residency_base + 40*4*2 + sizeof(u32)*j);

                start_idx = j+1;
                if (++num == 6) {
                    break;
                }
            }
        }
        sample.d_residency_sample.num_sampled = num;
        /*
         * OK, everything computed. Now copy
         * this sample into an output buffer
         */
        produce_pwr_sample(cpu, &sample);
    }
};
#endif


#if DO_S_STATE_SAMPLE
/*
 * Insert a S state sample into a (per-cpu) output buffer.
 */
static inline void produce_s_state_sample(void)
{
    u64 tsc;
    int cpu = CPU();
    u32 state;
    PWCollector_sample_t sample;

    tscval(&tsc);

    if (get_S_state(&state)) {
        return;
    }

    sample.sample_type = S_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.s_state_sample.state = state;

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

}
#endif


#if DO_D_NC_STATE_SAMPLE
/*
 * Insert a north complex D state sample into a (per-cpu) output buffer.
 */
static inline void produce_d_nc_state_sample(void)
{
    u64 tsc;
    int cpu = raw_smp_processor_id();
    unsigned long ncstates = 0;
    PWCollector_sample_t sample;

    tscval(&tsc);

    if (get_D_NC_states(&ncstates)) {
        return;
    }

    sample.sample_type = D_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.d_state_sample.device_type = PW_NORTH_COMPLEX;
    sample.d_state_sample.states[0] = ncstates;

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);

};
#endif


#if DO_D_SC_STATE_SAMPLE
/*
 * Insert a south complex D state sample into a (per-cpu) output buffer.
 */
static inline void produce_d_sc_state_sample(void)
{
    u64 tsc;
    int cpu = raw_smp_processor_id();
    u32 scstates[4];
    PWCollector_sample_t sample;

    tscval(&tsc);

    if (get_D_SC_states(scstates)) {
        return;
    }

    sample.sample_type = D_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.d_state_sample.device_type = PW_SOUTH_COMPLEX;
    memcpy(sample.d_state_sample.states, scstates, sizeof(u32) * 4);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);
};
#endif


#if DO_WAKELOCK_SAMPLE
/*
 * Insert a Wakelock sample into a (per-cpu) output buffer.
 */
static inline void produce_w_sample(int cpu, u64 tsc, w_sample_type_t type, pid_t tid, pid_t pid, const char *wlname, const char *pname)
{
    PWCollector_sample_t sample;

    sample.sample_type = W_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.w_sample.type = type;
    sample.w_sample.tid = tid;
    sample.w_sample.pid = pid;
    memcpy(sample.w_sample.name, wlname, PW_MAX_WAKELOCK_NAME_SIZE); // dst, src
    memcpy(sample.w_sample.proc_name, pname, PW_MAX_PROC_NAME_SIZE); // process name
    OUTPUT(0, KERN_INFO "%d , %d -> %s\n", sample.w_sample.tid, sample.w_sample.pid, sample.w_sample.wlname);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    produce_pwr_sample(cpu, &sample);
};
#endif


/*
 * Insert a P-state transition sample into a (per-cpu) output buffer.
 */
static inline void produce_p_sample(int cpu, unsigned long long tsc, u32 req_freq, u32 act_freq, u8 is_boundary_sample, u64 aperf, u64 mperf)
{
    struct PWCollector_sample sample;

    sample.sample_type = P_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.p_sample.prev_req_frequency = req_freq;
    sample.p_sample.frequency = act_freq;
    sample.p_sample.is_boundary_sample = is_boundary_sample;

    sample.p_sample.unhalted_core_value = aperf;
    sample.p_sample.unhalted_ref_value = mperf;

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
    sample.k_sample.tid = tentry->tid;
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
    u64 tsc;

    tscval(&tsc);

    sample.sample_type = IRQ_MAP;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
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
    u64 tsc;

    tscval(&tsc);

    sample.sample_type = M_MAP;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
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


/*
 * For debugging ONLY!
 */
static void *debug_insmod_timer_addr = 0x0;

static void timer_init(void *timer_addr)
{
    pid_t tid = TID();
    pid_t pid = PID();
    u64 tsc = 0;
    int trace_len = 0;
    unsigned long trace[MAX_BACKTRACE_LENGTH];
    bool is_root_timer = false;

    tscval(&tsc);

    if (false && !strcmp(NAME(), "insmod")) {
        debug_insmod_timer_addr = timer_addr;
        /*
        printk(KERN_INFO "%d -> %s\n", tid, NAME());
        printk(KERN_INFO "MOD-PRESENT = %s, SYS-PRESENT = %s\n", GET_BOOL_STRING(is_tid_in_mod_list(tid)), GET_BOOL_STRING(is_tid_in_sys_list(tid)));
        */
    }
    /*
     * For accuracy, we ALWAYS collect
     * kernel call stacks.
     */
    if ( (is_root_timer = IS_ROOT_TIMER(tid)) ) {
	/*
	 * get kernel timerstack here.
	 * Requires the kernel be compiled with
	 * frame_pointers on.
	 */
	if (INTERNAL_STATE.have_kernel_frame_pointers) {
	    trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	}
	else{
	    trace_len = 0;
	}
	OUTPUT(0, KERN_INFO "KERNEL-SPACE timer init! Timer_addr = %p, tid = %d, pid = %d\n", timer_addr, tid, pid);
    } else {
        trace_len = 0;
    }
    /*
     * Store the timer if:
     * (a) called for a ROOT process (tid == 0) OR
     * (b) we're actively COLLECTING.
     */
    if (is_root_timer || IS_COLLECTING()) {
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
		OUTPUT(0, KERN_INFO "FOUND CPU IDLE for cpu = %d . TICK SCHED TIMER = %p\n", cpu, hrt);
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
		OUTPUT(0, KERN_INFO "[%d]: ALL TICK TIMERS accounted for -- removing hrtimer start probe!\n", cpu);
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
static void handle_irq_wakeup_i(int cpu, int irq_num, const char *irq_name, bool was_hit)
{
    /*
     * Send a sample to Ring-3
     * (but only if collecting).
     */
    if (IS_COLLECTING()) {
        PWCollector_sample_t output_sample;
        u64 sample_tsc, c0;

        tscval(&sample_tsc);

        rdmsrl(INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], c0);

        output_sample.cpuidx = cpu;
        output_sample.sample_type = IRQ_SAMPLE;
        output_sample.tsc = sample_tsc;
        output_sample.e_sample.data[0] = irq_num;
        output_sample.e_sample.data[1] = output_sample.e_sample.data[2] = 0;
        /*
         * 'C0' always goes into LAST element of 'data' array.
         */
        output_sample.e_sample.data[3] = c0;

        record_hit_sample(cpu, &output_sample);
    }
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

#define TRACK_TIMER_EXPIRES 1

static void timer_expire(void *timer_addr, pid_t tid)
{
    int cpu = -1;
    pid_t pid = -1;
    tnode_t *entry = NULL;
    u64 tsc = 0;
    bool found = false;
    bool was_hit = false;
    bool is_root = false;
    int irq_num = -1;

    /*
     * Reduce overhead -- do NOT run
     * if user specifies NO C-STATES.
     */
    if (unlikely(!IS_C_STATE_MODE())) {
	return;
    }

#if !TRACK_TIMER_EXPIRES
    {
        if (IS_COLLECTING()) {
            record_hit_wakeup(CPU(), TIMER_SAMPLE);
        }

        return;
    }
#endif

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif
    /*
     * Atomic context => use __get_cpu_var(...) instead of get_cpu_var(...)
     */
    irq_num = (&__get_cpu_var(per_cpu_counts))->was_timer_hrtimer_softirq;

    was_hit = local_read(&__get_cpu_var(is_first_event)) == 1;

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
#endif // DO_IOCTL_STATS

    cpu = CPU();

    if ( (entry = (tnode_t *)timer_find((unsigned long)timer_addr, tid))) {
	pid = entry->pid;
	tsc = entry->tsc;
	found = true;
        is_root = entry->is_root_timer;
    } else {
	/*
	 * Couldn't find timer entry -- PID defaults to TID.
	 */
	pid = tid;
	tsc = 0x1;
	OUTPUT(3, KERN_INFO "Warning: [%d]: timer %p NOT found in list!\n", pid, timer_addr);
        is_root = pid == 0;
    }

    if (!found) {
	tsc = PW_max_num_cpus + 1;
	if (tid < 0) {
	    /*
	     * Yes, this is possible, especially if
	     * the timer was fired because of a TIMER_SOFTIRQ.
	     * Special case that here.
	     */
            if (irq_num > 0) {
		/*
		 * Basically, fall back on the SOFTIRQ
		 * option because we couldn't quite figure
		 * out the process that is causing this
		 * wakeup. This is a duplicate of the
		 * equivalent code in "inter_common(...)".
		 */
		const char *irq_name = pw_softirq_to_name[irq_num];
		OUTPUT(3, KERN_INFO "WARNING: could NOT find TID in timer_expire for Timer = %p: FALLING BACK TO TIMER_SOFTIRQ OPTION! was_hit = %s\n", timer_addr, GET_BOOL_STRING(was_hit));
		handle_irq_wakeup_i(cpu, irq_num, irq_name, was_hit);
		/*
		 * No further action is required.
		 */
		return;
	    }
	    else {
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
    } else {
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
    /*
     * Now send a sample to Ring-3.
     * (But only if collecting).
     */
    if (IS_COLLECTING()) {
        PWCollector_sample_t output_sample;
        u64 sample_tsc, c0;

        tscval(&sample_tsc);

        rdmsrl(INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], c0);

        output_sample.cpuidx = cpu;
        output_sample.sample_type = TIMER_SAMPLE;
        output_sample.tsc = sample_tsc;
        output_sample.e_sample.data[0] = pid;
        output_sample.e_sample.data[1] = tid;
        output_sample.e_sample.data[2] = tsc;
        /*
         * 'C0' always goes into LAST element of 'data' array.
         */
        output_sample.e_sample.data[3] = c0;

        record_hit_sample(cpu, &output_sample);
    }

    /*
     * OK, send the TIMER::TSC mapping & call stack to the user
     * (but only if this is for a kernel-space call stack AND the
     * user wants kernel call stack info).
     */
    if (is_root && (IS_COLLECTING() || IS_SLEEPING()) && IS_KTIMER_MODE() && found && !entry->trace_sent) {
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
    if(unlikely(!IS_C_STATE_MODE())){
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
    if(false && (irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ)){
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
    was_hit = local_read(&__get_cpu_var(is_first_event)) == 1;

    /*
     * OK, record a 'hit' (if applicable) and
     * send an i-sample message to Ring 3.
     */
    handle_irq_wakeup_i(CPU(), irq_num, irq_name, was_hit);
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
    if (IS_COLLECTING()) {
        record_hit_wakeup(CPU(), WORKQUEUE_SAMPLE);
    }
};
#else // >= 2.6.36
static void probe_workqueue_execute_start(void *ignore, struct work_struct *work)
{
    if (IS_COLLECTING()) {
        record_hit_wakeup(CPU(), WORKQUEUE_SAMPLE);
    }
};
#endif // < 2.6.36

void read_all_c_state_msrs_i(PWCollector_sample_t *sample)
{
    int i=0;
    int msr_addr = 0x0;
    u64 val = 0;

    for (i=MPERF; i<MAX_MSR_ADDRESSES; ++i) {
        RES_COUNT(sample->c_sample, i) = 0x0;
        msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[i];
        if (msr_addr < 0) {
            continue;
        }
        rdmsrl(msr_addr, val);
        RES_COUNT(sample->c_sample, i) = val;
    }

    return;
};

/*
 * Basically the same as arch/x86/kernel/irq.c --> "arch_irq_stat_cpu(cpu)"
 */

static u64 my_local_arch_irq_stats_cpu(void)
{
    u64 sum = 0;
    irq_cpustat_t *stats;

    BEGIN_LOCAL_IRQ_STATS_READ(stats);
    {
#ifdef CONFIG_X86_LOCAL_APIC
        sum += stats->apic_timer_irqs;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        sum += stats->x86_platform_ipis;
#endif // 2,6,34
        sum += stats->apic_perf_irqs;
#ifdef CONFIG_SMP
        sum += stats->irq_call_count;
        sum += stats->irq_resched_count;
        sum += stats->irq_tlb_count;
#endif
    }
    END_LOCAL_IRQ_STATS_READ(stats);
    return sum;
};

static DEFINE_PER_CPU(u64, prev_c6_val) = 0;

/*
 * TPS epoch manipulation functions.
 */
#if DO_TPS_EPOCH_COUNTER

int inc_tps_epoch_i(void)
{
    int retVal = -1;
    /*
     * From "Documentation/memory-barriers.txt": "atomic_inc_return()"
     * has IMPLICIT BARRIERS -- no need to add explicit barriers
     * here!
     */
    retVal = atomic_inc_return(&tps_epoch);
    return retVal;
};

int read_tps_epoch_i(void)
{
    /*
     * Make sure TPS updates have propagated
     */
    smp_mb();
    return atomic_read(&tps_epoch);
};
#endif // DO_TPS_EPOCH_COUNTER

static void tps(unsigned int type, unsigned int state)
{
    int cpu = CPU(), epoch = 0;
    u64 tsc = 0;
    PWCollector_sample_t sample;
    bool was_c6_transition = false;

    bool local_apic_fired = false;
    /*
     * We no longer encode the previous 'hint' param. Instead
     * we send the state the OS is CURRENTLY requesting.
     */
    unsigned int pstate = state;

    tscval(&tsc);

    /*
     * Read all C-state MSRs.
     */
    read_all_c_state_msrs_i(&sample);
    /*
     * We need to capture the 'C6' residency value in order to
     * generate S-state samples (see below).
     */
    {
        u64 *c6_val = &__get_cpu_var(prev_c6_val); // OK if we get interrupted
        u64 curr_c6_val = RES_COUNT(sample.c_sample, C6);
        was_c6_transition = (*c6_val > 0) && (curr_c6_val != *c6_val);
        *c6_val = curr_c6_val;
    }


    /*
     * Check if the local APIC timer raised interrupts.
     * Only required if we're capturing C-state samples.
     */
    if (IS_C_STATE_MODE()) {
        {
            u64 curr_num_local_apic = my_local_arch_irq_stats_cpu();
            u64 *old_num_local_apic = &__get_cpu_var(num_local_apic_timer_inters);
            if (*old_num_local_apic && (*old_num_local_apic != curr_num_local_apic)) {
                local_apic_fired = true;
            }
            *old_num_local_apic = curr_num_local_apic;
        }

        if (local_apic_fired && IS_COLLECTING()) {
            PWCollector_sample_t apic_sample;

            apic_sample.cpuidx = cpu;
            apic_sample.sample_type = IPI_SAMPLE;
            /*
             * We need a 'TSC' for this IPI sample but we don't know
             * WHEN the local APIC timer interrupt was raised. Fortunately, it doesn't
             * matter, because we only need to ensure this sample lies
             * BEFORE the corresponding 'C_STATE' sample in a sorted timeline.
             * We therefore simply subtract one from the C_STATE sample TSC to get
             * the IPI sample TSC.
             */
            apic_sample.tsc = tsc - 1;
            apic_sample.e_sample.data[0] = apic_sample.e_sample.data[1] = apic_sample.e_sample.data[2] = apic_sample.e_sample.data[3] = 0;

            record_hit_sample(cpu, &apic_sample);

        }

        sample.cpuidx = cpu;
        sample.sample_type = C_STATE;
        sample.tsc = tsc;
        sample.c_sample.prev_state = pstate;

#if DO_TPS_EPOCH_COUNTER
        /*
         * We're entering a new TPS "epoch".
         * Increment our counter.
         */
        epoch = inc_tps_epoch_i();
        sample.c_sample.tps_epoch = epoch;
#endif // DO_TPS_EPOCH_COUNTER

        if (IS_COLLECTING()) {
            produce_pwr_sample(cpu, &sample);
            OUTPUT(3, KERN_INFO "[%d] TPS\n", CPU());
        } else {
            OUTPUT(3, KERN_INFO "[%d] TPS -- NOT COLLECTING!\n", CPU());
        }
        /*
         * Reset the "first-hit" variable.
         */
        local_set(&get_cpu_var(is_first_event), 1);
        put_cpu_var(is_first_event);

    } // IS_C_STATE_MODE()

    // Collect S and D state / residency counter samples on CPU0
    if (cpu != 0 || !IS_COLLECTING()) {
        return;
    }

    /*
     * Controls the sampling frequency to reduce data collection overheads
     */
    if ((CURRENT_TIME_IN_USEC() - prev_sample_usec) > INTERNAL_STATE.d_state_sample_interval*1000) {
        prev_sample_usec = CURRENT_TIME_IN_USEC();

#if DO_S_STATE_SAMPLE
        if (PW_is_atm && IS_S_STATE_MODE()) {
            if (INTERNAL_STATE.write_to_buffers) {
                produce_s_state_sample();
            }
        }
#endif

#if DO_D_NC_STATE_SAMPLE
        if (PW_is_atm && IS_D_NC_STATE_MODE()) {
            if (INTERNAL_STATE.write_to_buffers) {
                produce_d_nc_state_sample();
            }
        }
#endif

#if DO_D_SC_STATE_SAMPLE
        if (PW_is_atm && IS_D_SC_STATE_MODE()) {
            if (INTERNAL_STATE.write_to_buffers) {
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
    // tps_i(type, state);
    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
};
#else // version >= 2.6.38
static void probe_cpu_idle(void *ignore, unsigned int state, unsigned int cpu_id)
{
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
    for (i=0; i<len; ++i) {
	if (buffer[i] == ' ') {
	    ++num_toks;
        }
    }

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
    for (i=0, j=0, str = buffer; i<len; ++i) {
	if (buffer[i] == ' ') {
	    memset(tmp, 0, sizeof(tmp));
	    ++num_toks;
	    if ( (tmp_len = (buffer+i) - str) > 10) {
		// ERROR!
		return -ERROR;
	    }
	    strncpy(tmp, str, tmp_len);
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
    if (!base_operating_freq) {
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


static DEFINE_PER_CPU(u32, pcpu_prev_req_freq) = 0;

/*
 * New methodology -- We're now using APERF/MPERF
 * collected within the TPS probe to calculate actual
 * frequencies. Only calculate TSC values within
 * the 'power_frequency' tracepoint.
 */
static void tpf(int cpu, unsigned int type, u32 req_freq)
{
    u64 tsc = 0, aperf = 0, mperf = 0;
    u32 l=0, h=0;
    u32 prev_req_freq = 0;
    u32 prev_act_freq = INTERNAL_STATE.bus_clock_freq_khz;
    u32 perf_status = 0;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * We're not guaranteed that 'cpu' (which is the CPU on which the frequency transition is occuring) is
     * the same as the cpu on which the callback i.e. the 'TPF' probe is executing. This is why we use 'rdmsr_safe_on_cpu()'
     * to read the various MSRs.
     */
    /*
     * Read TSC value
     */
    WARN_ON(rdmsr_safe_on_cpu(cpu, 0x10, &l, &h));
    tsc = (u64)h << 32 | (u64)l;
    /*
     * Read CPU_CLK_UNHALTED.REF and CPU_CLK_UNHALTED.CORE. These required ONLY for AXE import
     * backward compatibility!
     */
    {
        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[APERF], &l, &h));
        aperf = (u64)h << 32 | (u64)l;

        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], &l, &h));
        mperf = (u64)h << 32 | (u64)l;
    }
    /*
     * Read the IA32_PERF_STATUS MSR. Bits 12:8 (on Atom ) or 15:0 (on big-core) of this determines
     * the frequency the H/W is currently running at.
     */
    WARN_ON(rdmsr_safe_on_cpu(cpu, IA32_PERF_STATUS_MSR_ADDR, &l, &h));
    perf_status = l; // We're only interested in the lower 16 bits!

    /*
     * OK, try and get the actual frequency; this is the perf_status value multiplied by the
     * FSB frequency.
     */
    if (PW_is_atm) {
        prev_act_freq *= ((perf_status >> 8) & 0x1f); // bits [12:8]
    } else {
        prev_act_freq *= (perf_status & 0xffff); // bits [15:0]
    }

    /*
     * Retrieve the previous requested frequency, if any. Note that we're GUARANTEED this
     * will be non-zero because it's set in 'get_current_cpu_frequency()' on collection START/STOP.
     */
    prev_req_freq = per_cpu(pcpu_prev_req_freq, cpu);
    per_cpu(pcpu_prev_req_freq, cpu) = req_freq;

    produce_p_sample(cpu, tsc, prev_req_freq, prev_act_freq, 0 /* boundary */, aperf, mperf); // "0" ==> NOT a boundary sample

    OUTPUT(0, KERN_INFO "[%d]: TSC = %llu, OLD_req_freq = %u, NEW_REQ_freq = %u, perf_status = %u, ACT_freq = %u\n", cpu, tsc, prev_req_freq, req_freq, perf_status, prev_act_freq);

#if DO_IOCTL_STATS
    {
	pstats = &get_cpu_var(per_cpu_stats);
	local_inc(&pstats->p_trans);
	put_cpu_var(pstats);
    }
#endif // DO_IOCTL_STATS

};

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
    int cpu = freq->cpu;

    if (unlikely(!IS_FREQ_MODE())) {
	return SUCCESS;
    }

    if (val == CPUFREQ_PRECHANGE) {
    // if (val == CPUFREQ_POSTCHANGE) {
        // printk(KERN_INFO "[%d]: OLD_freq = %u, NEW_freq = %u\n", cpu, freq->old, state);
        DO_PER_CPU_OVERHEAD_FUNC(tpf, cpu, 2, state);
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
    DO_PER_CPU_OVERHEAD_FUNC(tpf, type, state);
};

#endif // DO_CPUFREQ_NOTIFIER


/*
 * Helper function for "probe_sched_exit"
 * Useful for overhead measurements.
 */
static void exit_helper(struct task_struct *task)
{
    pid_t tid = task->pid, pid = task->tgid;

    OUTPUT(3, KERN_INFO "[%d]: SCHED_EXIT\n", tid);
    /*
     * Delete all (non-Kernel) timer mappings created
     * for this thread.
     */
    delete_timers_for_tid(tid);
    /*
     * Delete any sys-node mappings created on behalf
     * of this thread.
     */
    check_and_delete_proc_from_sys_list(tid, pid);

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

inline void sched_wakeup_helper_i(struct task_struct *task)
{
    int target_cpu = task_cpu(task), source_cpu = CPU();
    /*
     * "Self-sched" samples are NOT enqueued because they cannot, by
     * definition, cause wakeups.
     */
    if (target_cpu != source_cpu) {

        PWCollector_sample_t output_sample;
        u64 sample_tsc;

        tscval(&sample_tsc);
        output_sample.cpuidx = source_cpu;
        output_sample.sample_type = SCHED_SAMPLE;
        output_sample.tsc = sample_tsc;
        output_sample.e_sample.data[0] = source_cpu;
        output_sample.e_sample.data[1] = target_cpu;

#if DO_TPS_EPOCH_COUNTER
        output_sample.e_sample.data[2] = read_tps_epoch_i();
#endif // DO_TPS_EPOCH_COUNTER

        /*
         * Sched wakeups should ALWAYS be generated, and they NEVER cause
         * "Self-wakeups", so don't use the "record_hit_XXX()" interface.
         * Instead, copy the sample manually.
         */
        copy_sample_i(source_cpu, &output_sample, false); // "false" ==> don't wake any sleeping readers (required from scheduling context)
        // record_hit_sample_no_wakeup(source_cpu, &output_sample);
    }
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_wakeup(struct rq *rq, struct task_struct *task, int success)
#else
static void probe_sched_wakeup(void *ignore, struct task_struct *task, int success)
#endif
{
    if (likely(IS_COLLECTING())) {
        sched_wakeup_helper_i(task);
    }
};

inline bool is_sleep_syscall_i(long id)
{
    switch (id) {
        case __NR_poll: // 7
        case __NR_select: // 23
        case __NR_nanosleep: // 35
        case __NR_alarm: // 37
        case __NR_setitimer: // 38
        case __NR_rt_sigtimedwait: // 128
        case __NR_futex: // 202
        case __NR_timer_settime: // 223
        case __NR_clock_nanosleep: // 230
        case __NR_epoll_wait: // 232
        case __NR_pselect6: // 270
        case __NR_ppoll: // 271
        case __NR_epoll_pwait: // 281
        case __NR_timerfd_settime: // 286
            return true;
        default:
            break;
    }
    return false;
};

inline void  sys_enter_helper_i(long id, pid_t tid, pid_t pid)
{
    if (check_and_add_proc_to_sys_list(tid, pid)) {
        printk(KERN_INFO "ERROR: could NOT add proc to sys list!\n");
    }
    return;
};

inline void  sys_exit_helper_i(long id, pid_t tid, pid_t pid)
{
    check_and_remove_proc_from_sys_list(tid, pid);
};


#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sys_enter(struct pt_regs *regs, long ret)
#else
static void probe_sys_enter(void *ignore, struct pt_regs *regs, long ret)
#endif
{
    long id = syscall_get_nr(current, regs);
    pid_t tid = TID(), pid = PID();

    if (is_sleep_syscall_i(id)) {
        DO_PER_CPU_OVERHEAD_FUNC(sys_enter_helper_i, id, tid, pid);
    }
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sys_exit(struct pt_regs *regs, long ret)
#else
static void probe_sys_exit(void *ignore, struct pt_regs *regs, long ret)
#endif
{
    long id = syscall_get_nr(current, regs);
    pid_t tid = TID(), pid = PID();

    DO_PER_CPU_OVERHEAD_FUNC(sys_exit_helper_i, id, tid, pid);

    if(id == __NR_execve && IS_COLLECTING()){
        u64 tsc;

        tscval(&tsc);
        OUTPUT(3, KERN_INFO "[%d]: EXECVE ENTER! TID = %d, NAME = %.20s\n", CPU(), TID(), NAME());
        produce_r_sample(CPU(), tsc, PW_PROC_EXEC, TID(), PID(), NAME());
    }
};
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

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
 * Notifier for module loads and frees.
 * We register module load and free events -- extract memory bounds for
 * the module (on load). Also track TID, NAME for tracking device driver timers.
 */
int apwr_mod_notifier(struct notifier_block *block, unsigned long val, void *data)
{
    struct module *mod = data;
    int cpu = CPU();
    const char *name = mod->name;
    unsigned long module_core = (unsigned long)mod->module_core;
    unsigned long core_size = mod->core_size;

    if (val == MODULE_STATE_COMING) {
        OUTPUT(0, KERN_INFO "COMING: tid = %d, pid = %d, name = %s, module_core = %lu\n", TID(), PID(), name, module_core);
        produce_m_sample(cpu, name, module_core, core_size);
        // return add_new_module_to_mod_list(TID(), PID(), mod);
    } else if (val == MODULE_STATE_GOING) {
        OUTPUT(0, KERN_INFO "GOING: tid = %d, pid = %d, name = %s\n", TID(), PID(), name);
        // return remove_module_from_mod_list(mod);
    }
    return SUCCESS;
};

static struct notifier_block apwr_mod_notifier_block = {
    .notifier_call = &apwr_mod_notifier
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
     * ONGOING OR IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);

    produce_w_sample(CPU(), tsc, PW_WAKE_LOCK, TID(), PID(), lock->name, NAME());

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
     * ONGOING OR IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
        return;
    }

    tscval(&tsc);

    produce_w_sample(CPU(), tsc, PW_WAKE_UNLOCK, TID(), PID(), lock->name, NAME());

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
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    {
        WARN_ON(register_trace_sys_enter(probe_sys_enter));
	WARN_ON(register_trace_sys_exit(probe_sys_exit));
    }
#else // LINUX_VERSION
    {
        WARN_ON(register_trace_sys_enter(probe_sys_enter, NULL));
	WARN_ON(register_trace_sys_exit(probe_sys_exit, NULL));
    }
#endif // LINUX_VERSION
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT
    return register_timer_callstack_probes();
};

static void unregister_permanent_probes(void)
{
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    {
        unregister_trace_sys_enter(probe_sys_enter);
	unregister_trace_sys_exit(probe_sys_exit);

	tracepoint_synchronize_unregister();
    }
#else // LINUX_VERSION
    {
        unregister_trace_sys_enter(probe_sys_enter, NULL);
        unregister_trace_sys_exit(probe_sys_exit, NULL);

        tracepoint_synchronize_unregister();
    }
#endif // LINUX_VERSION
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

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
    // module_notifier
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tMOD_NOTIFIER_EVENTS");
        register_module_notifier(&apwr_mod_notifier_block);
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
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT

#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

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
	OUTPUT(0, KERN_INFO "\tMOD_NOTIFIER_EVENTS");
        register_module_notifier(&apwr_mod_notifier_block);
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
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT

#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

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
        OUTPUT(0, KERN_INFO "\tWAKELOCK_EVENTS");
        ret = register_trace_wake_lock(probe_wake_lock, NULL);
        WARN_ON(ret);

        OUTPUT(0, KERN_INFO "\tWAKEUNLOCK_EVENTS");
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
        unregister_module_notifier(&apwr_mod_notifier_block);
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
        unregister_module_notifier(&apwr_mod_notifier_block);
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
	    if (extract_valid_frequencies(buffer, buffer_len)) {
		printk(KERN_INFO "ERROR: could NOT determine frequency table!\n");
	    }
	}
    }
};


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
    get_frequency_steps();

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
     * UPDATE: we're borrowing one of the 'reserved' 64bit values
     * to document the following:
     * (1) Kernel call stacks supported?
     * (2) Kernel compiled with CONFIG_TIMER_STATS?
     * (2) Wakelocks supported?
     */
#ifdef CONFIG_FRAME_POINTER
    local_check->supported_features |= PW_KERNEL_SUPPORTS_CALL_STACKS;
#endif
#ifdef CONFIG_TIMER_STATS
    local_check->supported_features |= PW_KERNEL_SUPPORTS_CONFIG_TIMER_STATS;
#endif
#if DO_WAKELOCK_SAMPLE
    local_check->supported_features |= PW_KERNEL_SUPPORTS_WAKELOCK_PATCH;
#endif

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
	printk(KERN_INFO "ERROR: trying to get list of available frequencies WITHOUT setting config?!\n");
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

    for_each_online_cpu(cpu) {
        /*
         * Reset the per-cpu residencies
         */
        *(&per_cpu(prev_c6_val, cpu)) = 0;
        /*
         * Reset the "first-hit" variable.
         */
        local_set(&per_cpu(is_first_event, cpu), 1);
        /*
         * Reset stats on # samples produced and # dropped.
         */
        local_set(&per_cpu(num_samples_produced, cpu), 0);
        local_set(&per_cpu(num_samples_dropped, cpu), 0);
    }
    /*
     * Reset the TPS atomic count value.
     */
#if DO_TPS_EPOCH_COUNTER
    atomic_set(&tps_epoch, 0);
#endif
    /*
     * Ensure updates are propagated.
     */
    smp_mb();
};


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
 * Run the generation of current p-state sample
 * for all cpus in parallel to avoid the delay
 * due to a serial execution.
 */
static void generate_cpu_frequency_per_cpu(int cpu, bool is_start)
{
    u32 l=0, h=0;
    u64 tsc = 0, aperf = 0, mperf = 0;
    u32 perf_status = 0;
    u8 is_boundary = (is_start) ? 1 : 2; // "0" ==> NOT boundary, "1" ==> START boundary, "2" ==> STOP boundary
    u32 act_freq = INTERNAL_STATE.bus_clock_freq_khz;
    u32 prev_req_freq = 0;

    {
        int ret = rdmsr_safe_on_cpu(cpu, 0x10, &l, &h);
        if(ret){
            OUTPUT(0, KERN_INFO "WARNING: rdmsr of TSC failed with code %d\n", ret);
        }
        tsc = h;
        tsc <<= 32;
        tsc += l;
    }

    /*
     * Read the IA32_PERF_STATUS MSR. Bits 12:8 (Atom) or 15:0 (big-core) of this determines the frequency
     * the H/W is currently running at.
     */
    {
        WARN_ON(rdmsr_safe_on_cpu(cpu, IA32_PERF_STATUS_MSR_ADDR, &l, &h));
        perf_status = l; // We're only interested in the lower 16 bits!
    }

    /*
     * OK, try and get the actual frequency; this is the perf_status value multiplied by the
     * FSB frequency.
     */
    if (PW_is_atm) {
        act_freq *= ((perf_status >> 8) & 0x1f); // bits [12:8]
    } else {
        act_freq *= (perf_status & 0xffff); // bits [15:0]
    }

    /*
     * Retrieve the previous requested frequency.
     */
    if (is_start == false) {
        prev_req_freq = per_cpu(pcpu_prev_req_freq, cpu);
    }
    per_cpu(pcpu_prev_req_freq, cpu) = act_freq;

    /*
     * Also read CPU_CLK_UNHALTED.REF and CPU_CLK_UNHALTED.CORE. These required ONLY for AXE import
     * backward compatibility!
     */
    {
        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[APERF], &l, &h));
        aperf = (u64)h << 32 | (u64)l;
        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], &l, &h));
        mperf = (u64)h << 32 | (u64)l;
    }
    produce_p_sample(cpu, tsc, prev_req_freq, act_freq, is_boundary, aperf, mperf);

};

#if DO_GENERATE_CURRENT_FREQ_IN_PARALLEL
static void generate_cpu_frequency(void *start)
{
    int cpu = raw_smp_processor_id();
    bool is_start = *((bool *)start);

    generate_cpu_frequency_per_cpu(cpu, is_start);
}
#endif

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
static void get_current_cpu_frequency(bool is_start)
{

#if DO_GENERATE_CURRENT_FREQ_IN_PARALLEL
    SMP_CALL_FUNCTION(&generate_cpu_frequency, (void *)&is_start, 0, 1);
    // smp_call_function is executed for all other CPUs except itself.
    // So, run it for itself.
    generate_cpu_frequency((void *)&is_start);
#else
    int cpu = 0;
    for_each_online_cpu(cpu){
        generate_cpu_frequency_per_cpu(cpu, is_start);
    }
#endif
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
            produce_s_residency_sample(0);
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
    if(PW_is_atm && IS_D_SC_RESIDENCY_MODE()){
	    start_d_sc_residency_counter();
            startJIFF_d_sc_residency = CURRENT_TIME_IN_USEC();
            prev_sample_usec = CURRENT_TIME_IN_USEC();
            produce_d_sc_residency_sample(0);
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
        if (usec > 0) {
            produce_s_residency_sample(usec);
        }
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
        if (usec > 0) {
            produce_d_sc_residency_sample(usec);
        }
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

    if(cmd == STOP || cmd == CANCEL) {
        unregister_pausable_probes();
        /*
         * Gather some stats on # of samples produced and dropped.
         */
        {
            count_samples_produced_dropped();
        }
    }



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
    if(cmd == STOP || cmd == CANCEL) {
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
    if (cmd == STOP || cmd == CANCEL) {
        if (likely(IS_FREQ_MODE())) {
            get_current_cpu_frequency(false); // "false" ==> collection STOP
        }
        /*
         * We're at collection STOP -- delete
         * previously allocated frequency
         * table.
         */
        if (apwr_available_frequencies) {
            pw_kfree(apwr_available_frequencies);
            OUTPUT(3, KERN_INFO "FREED AVAILABLE FREQUENCIES!\n");
            apwr_available_frequencies = NULL;
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
    if (cmd == STOP || cmd == CANCEL) {
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

long do_cmd(PWCollector_cmd_t cmd, u64 *remote_output_args, int size)
{
    int retVal = SUCCESS;
    /*
     * Handle the command itself.
     */
    if (handle_cmd(cmd)) {
        return -ERROR;
    }
    /*
     * Then check if the user requested some collection stats.
     */
#if DO_COUNT_DROPPED_SAMPLES
    if (cmd == STOP || cmd == CANCEL) {
        u64 local_args[2] = {total_num_samples_produced, total_num_samples_dropped};
        // u64 local_args[2] = {100, 10}; // for debugging!
        if (copy_to_user(remote_output_args, local_args, size)) // returns number of bytes that could NOT be copied
            retVal = -ERROR;
    }
#endif // DO_COUNT_DROPPED_SAMPLES

    return retVal;
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
	// return handle_cmd(cmd);
        return do_cmd(cmd, (u64 *)remote_args->out_arg, local_out_len);
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
    int ret;

    /*
     * Register the character device
     */
#if CREATE_DEVICE_NODE
    ret = alloc_chrdev_region(&apwr_dev, 0, 1, DEVICE_NAME);
    apwr_dev_major_num = MAJOR(apwr_dev);
    apwr_class = class_create(THIS_MODULE, "apwr");
    if(IS_ERR(apwr_class))
        printk(KERN_ERR "Error registering apwr class\n");

    device_create(apwr_class, NULL, apwr_dev, NULL, DEVICE_NAME);
    apwr_cdev = cdev_alloc();
    apwr_cdev->owner = THIS_MODULE;
    apwr_cdev->ops = &Fops;
    if( cdev_add(apwr_cdev, apwr_dev, 1) < 0 )  {
        printk("Error registering device driver\n");
        return ret;
    }
#else
    ret = register_chrdev(0, DEVICE_FILE_NAME, &Fops);

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
#endif

    return ret;
};

void unregister_dev(void)
{
    /*
     * Unregister the device
     */
#if CREATE_DEVICE_NODE
    unregister_chrdev(apwr_dev_major_num, DEVICE_NAME);
    device_destroy(apwr_class, apwr_dev);
    class_destroy(apwr_class);
    unregister_chrdev_region(apwr_dev, 1);
    cdev_del(apwr_cdev);
#else
    // unregister_chrdev(MAJOR_NUM, DEVICE_FILE_NAME);
    unregister_chrdev(apwr_dev_major_num, DEVICE_FILE_NAME);
#endif
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
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->perf_data;
            ret = rdmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, &data[0], &data[1]);
            WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
            memcpy(data_copy, data, sizeof(u32) * 2);
            res = data[1];
            res <<= 32;
            res += data[0];
            OUTPUT(0, KERN_INFO "[%d]: READ res = 0x%llx\n", cpu, res);
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
    OUTPUT(0, KERN_INFO "FMS = 0x%x:%x:%x (%d:%d:%d)\n", family, model, stepping, family, model, stepping);
    /*
     * This check below will need to
     * be updated for each new
     * architecture type!!!
     */
    if (family == 0x6) {
        if (model == 0x27) {
            // MFLD
            return true;
        } else if (model == 0x35) {
            // CLV
            return true;
        }
    }
    return false;
};

static int __init init_hooks(void)
{
    int ret = 0;

    OUTPUT(0, KERN_INFO "# IRQS = %d\n", NR_IRQS);

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

#if DO_S_RESIDENCY_SAMPLE
        // Map the bus memory into CPU space for 4 S state residency counters
        mmio_s_residency_base = ioremap_nocache(S_RESIDENCY_BASE_ADDRESS, S_RESIDENCY_MAX_COUNTERS*4);

        if (mmio_s_residency_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! S Residency counter is NOT available\n");
            return -ERROR;
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

        mmio_cumulative_residency_base = ioremap_nocache(CUMULATIVE_RESIDENCY_ADDRESS, 4);

        if (mmio_cumulative_residency_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! Cumulative Residency counter is NOT available\n");
            return -ERROR;
        }

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
    printk(KERN_INFO "START Initialized the PWR DRIVER\n");
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
        iounmap(mmio_cumulative_residency_base);
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

    printk(KERN_INFO "STOP Terminated the PWR Driver.\n");
#if DO_PRINT_COLLECTION_STATS
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
        sys_enter_helper_i_print_cumulative_overhead_params("SYS_ENTER_HELPER_I");
        sys_exit_helper_i_print_cumulative_overhead_params("SYS_EXIT_HELPER_I");
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
#endif // DO_PRINT_COLLECTION_STATS

    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");
};

module_init(init_hooks);
module_exit(cleanup_hooks);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
