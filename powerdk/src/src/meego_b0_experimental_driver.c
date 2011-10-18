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
 * pw_driver.c: Prototype kernel module to trace the following
 * events that are relevant to power:
 *	- entry into a C-state
 *	- change of processor frequency
 *	- interrupts and timers
 *
 * LIMITATIONS:
 *      - Timer expires do NOT remove timer::TSC mappings from the list of timers.
 *      - This prototype REQUIRES kernel frame pointers to be compiled in for k-space call stack generation.
 */

#define MOD_AUTHOR "Gautam Upadhyaya <gautam.upadhyaya@intel.com>"
#define MOD_DESC "Prototype collector for Piersol power tool. Adapted from original prototype developed by Romain Cledat."

/*
 * Current driver version is 0.0.10
 *
 * CHANGED TO: 0.1.0
 * CHANGED TO: 1.0.1
 */
#define PW_VERSION_VERSION 1
#define PW_VERSION_INTERFACE 0
#define PW_VERSION_OTHER 1

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>

#include <linux/percpu.h>
#include <asm/local.h>
#include <asm/cputime.h> // For ktime
#include <asm/ptrace.h> // For pt_regs
#include <asm/processor.h> // For task_pt_regs
#include <linux/uaccess.h>
#include <asm/uaccess.h> // For get_user
#include <asm/msr.h> // For rdtscpll and MSR reading
#include <asm/idle.h>
#include <linux/tracepoint.h>
#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/sched.h> // Includes current
#include <linux/hardirq.h> // For the in_interrupt functions
#include <linux/interrupt.h>
#include <linux/kprobes.h>
#include <linux/notifier.h>

#include <linux/preempt.h>

#include <linux/jiffies.h>

#include <linux/kallsyms.h>

#include <asm/desc.h>

#include <linux/stacktrace.h>
#include <asm/stacktrace.h>
// #include <arch/x86/asm/stacktrace.h>
#include <linux/nmi.h>

#include <linux/poll.h>

#include <linux/slab.h>

#include <trace/events/sched.h> // for "sched_process_exit"

#include <linux/hash.h>

/*
 * *****************************************************
 * This device driver is intended to be used with only
 * a SINGLE READER process.
 * THIS IS A DESIGN DECISION!!!
 * *****************************************************
 */


/**** CONFIGURATION ****/

static int PW_max_num_cpus = -1;

/* Controls the amount of printks that happen. Levels are:
 *	- 0: no output save for errors and status at end
 *	- 1: single line for each hit (tracepoint, idle notifier...)
 *	- 2: more details
 *	- 3: user stack and kernel stack info
 */
static unsigned int verbosity = 0;

/**** RESIDENCY COUNTERS ****/

/* MSR counters to read the residency counts */


/*
 * For measuring collection times.
 */
static unsigned long startJIFF, stopJIFF;


/*
 * These are valid for NHM ONLY!
 */
// static const unsigned int msr_counters[RESIDENCY_COUNT] = { 0x10, 0xe7, 0xe8, 0x3fc, 0x3fd, 0x3fe }; // "tsc", "mperf", "aperf", "C3", "C6", "C7"

module_param(verbosity, uint, 0);
MODULE_PARM_DESC(verbosity, "Verbosity of output. From 0 to 3 with 3 the most verbose [default=0]");

/*
 * We're the PWR kernel device driver.
 * Flag needs to be set BEFORE
 * including 'pw_ioctl.h'
 */
#define PW_KERNEL_MODULE 1

#include "pw_ioctl.h" // For IOCTL mechanism
#include <linux/fs.h>
#include <linux/bitops.h> // for "test_and_set_bit(...)" atomic functionality

/*
 * Is the device open right now? Used to prevent
 * concurent access into the same device.
 */
#define DEV_IS_OPEN 0 // see if device is in use
static volatile unsigned long dev_status;

#define SUCCESS 0
#define ERROR 1
#define BUF_LEN 80

enum{
    EMPTY=0,
    FULL
};

#define NUM_SEGS_PER_LIST 2

#define SAMPLE_MASK (NUM_SAMPLES_PER_SEG - 1)
#define LIST_MASK (NUM_SEGS_PER_LIST - 1)

/*
 * Output buffer (also called a "segment" or "seg").
 * Driver writes samples into these. "read(...)" function
 * pulls samples out of these.
 */
typedef struct{
    PWCollector_sample_t samples[NUM_SAMPLES_PER_SEG];
    int index;
    atomic_t is_full;
}seg_t;

/*
 * Per-cpu structure (or "list") of output buffers.
 * Each list has "NUM_SEGS_PER_LIST" (== 2) buffers.
 */
typedef struct{
    seg_t segs[NUM_SEGS_PER_LIST];
    int index;
    int flush_index;
}list_t;


#define SAMPLE(s,i) ( (s)->samples[(i)] )
#define C_SAMPLE(s,i) ( (s)->samples[(i)].c_sample )
#define P_SAMPLE(s,i) ( (s)->samples[(i)].p_sample )
#define K_SAMPLE(s,i) ( (s)->samples[(i)].k_sample )
#define M_SAMPLE(s,i) ( (s)->samples[(i)].m_sample )

static list_t *lists = NULL;

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
 * Define MAX number of trace entries.
 * MUST be > 4 (because # trace hash bits == num trace bits - 4)
 */
// #define NUM_TRACE_ENTRY_BITS 14
#define NUM_TRACE_ENTRY_BITS 8
#define MAX_NUM_TRACE_ENTRIES (1 << NUM_TRACE_ENTRY_BITS)
#define TRACE_ENTRIES_MASK (MAX_NUM_TRACE_ENTRIES - 1)

typedef struct trace_entry trace_entry_t;

typedef struct trace_entry{
    pid_t tid;
    pid_t tgid;
    unsigned long timer_addr;
    unsigned long long tsc;
    int trace_len;
    unsigned long backtrace[MAX_BACKTRACE_LENGTH];

    trace_entry_t *next;
}*hash_entry_t;


// MUST be a pow of 2!
#define NUM_TRACE_HASH_BITS (NUM_TRACE_ENTRY_BITS)
/* #define NUM_TRACE_HASH_BITS (NUM_TRACE_ENTRY_BITS - 2) */
#define NUM_TRACE_HASH_ENTRIES (1 << NUM_TRACE_HASH_BITS)
#define TRACE_HASH_MASK (NUM_TRACE_HASH_ENTRIES - 1)
//#define hash_func_entries(p) ( ((unsigned long)(p) & TRACE_HASH_MASK) )
#define hash_func_entries(p) ( (int) ( hash_ptr( (void *)(p), NUM_TRACE_HASH_BITS ) ) )

static hash_entry_t hash_table[NUM_TRACE_HASH_ENTRIES];

static trace_entry_t *free_list_head = NULL;

// 32 locks for the hash table
#define HASH_LOCK_BITS 5
// 4 locks for the hash table
// #define HASH_LOCK_BITS 2
#define NUM_HASH_LOCKS (1UL << HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

// static spinlock_t hashLocks[NUM_HASH_LOCKS];
static rwlock_t hashLocks[NUM_HASH_LOCKS];

static spinlock_t free_list_lock = SPIN_LOCK_UNLOCKED;

#define HASH_READ_LOCK(x) READ_LOCK(hashLocks[(x) & HASH_LOCK_MASK])
#define HASH_READ_UNLOCK(x) READ_UNLOCK(hashLocks[(x) & HASH_LOCK_MASK])

#define HASH_WRITE_LOCK(x) WRITE_LOCK(hashLocks[(x) & HASH_LOCK_MASK])
#define HASH_WRITE_UNLOCK(x) WRITE_UNLOCK(hashLocks[(x) & HASH_LOCK_MASK])


typedef struct trace_block trace_block_t;

struct trace_block{
    trace_entry_t *block;
    trace_block_t *next;
};

static trace_block_t *trace_entry_blocks_head = NULL, *trace_entry_blocks_tail = NULL;

u64 num_free_trace_entries = 0, total_num_trace_entries = 0;

typedef struct ptnode ptnode_t;
typedef struct ptblock ptblock_t;

typedef struct ptnode{
    union{
	struct{
	    pid_t tid;
	    ptnode_t *timer_list_head;
	};
	unsigned long addr;
    };
    ptnode_t *next;
} *pthash_t;

struct ptblock{
    ptnode_t *block;
    ptblock_t *next;
};

static ptblock_t *ptblocks_head = NULL, *ptblocks_tail = NULL;

static ptnode_t *ptnode_free_list_head = NULL;

u64 num_free_ptnode_entries = 0, total_num_ptnode_entries = 0;

#define PTNODE_HASH_MASK TRACE_HASH_MASK
//#define pt_hash_func_entries(p) ( ((unsigned long)(p) & PTNODE_HASH_MASK) )
#define pt_hash_func_entries(p) ( (int) ( (hash_32)( (u32)(p), NUM_TRACE_HASH_BITS ) ) )

static pthash_t pt_hash_table[NUM_TRACE_HASH_ENTRIES];
static rwlock_t pt_hashLocks[NUM_HASH_LOCKS];

#define PT_HASH_READ_LOCK(x) READ_LOCK(pt_hashLocks[(x) & HASH_LOCK_MASK])
#define PT_HASH_READ_UNLOCK(x) READ_UNLOCK(pt_hashLocks[(x) & HASH_LOCK_MASK])

#define PT_HASH_WRITE_LOCK(x) WRITE_LOCK(pt_hashLocks[(x) & HASH_LOCK_MASK])
#define PT_HASH_WRITE_UNLOCK(x) WRITE_UNLOCK(pt_hashLocks[(x) & HASH_LOCK_MASK])

// static spinlock_t ptnode_free_list_lock = SPIN_LOCK_UNLOCKED;
static DEFINE_SPINLOCK(ptnode_free_list_lock);
/* static DEFINE_RWLOCK(ptlist_lock); */

#define TRACE_BLOCK_SIZE_REALLOC_THRESHOLD 20
#define PTNODE_BLOCK_SIZE_REALLOC_THRESHOLD 20

/*
 * We need an upper limit on how many trace entries
 * we're going to allocate.
 */
#define NUM_TRACE_REALLOCS_ALLOWED 1
#define NUM_PTNODE_REALLOCS_ALLOWED NUM_TRACE_REALLOCS_ALLOWED

static unsigned int num_trace_reallocs = 0, num_ptnode_reallocs = 0;

/*
 * Is this a deferred timer?
 */
#define DEFERRABLE_FLAG (0x1)
#define IS_TIMER_DEFERRABLE(t) ( (unsigned long)( (t)->base) & DEFERRABLE_FLAG )

/*
 * Compiler flags defined here.
 * These are compile-time constants which
 * typically control what gets compiled into
 * the code.
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
 * Do we (k)free memory used by trace entries
 * on a collection STOP/CANCEL? '1' ==> YES
 */
#define FREE_TRACE_MAP_ON_COLLECTION_STOP 0
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
#define DO_OVERHEAD_MEASUREMENTS 0




#if ALLOW_BLOCKING_READ
wait_queue_head_t read_queue;
#endif



/*
 * If we ever run into memory allocation errors then
 * stop (and drop) everything.
 */
static atomic_t should_panic = ATOMIC_INIT(0);
/*
 * Macro to check if PANIC is on.
 */
#define SHOULD_TRACE() (atomic_read(&should_panic) == 0)


/*
 * Structure to hold current CMD state
 * of the device driver. Constantly evolving, but
 * that's OK -- this is internal to the driver
 * and is NOT exported.
 */
typedef struct{
    PWCollector_cmd_t cmd; // indicates which command was specified last e.g. START, STOP etc.
    /*
     * Should we write to our per-cpu output buffers?
     * YES if we're actively collecting.
     * NO if we're not.
     */
    bool write_to_buffers;
    /*
     * Should we "drain/flush" the per-cpu output buffers?
     * (See "device_read" for an explanation)
     */
    bool drain_buffers;
    /*
     * Current methodology for generating kernel-space call
     * stacks relies on following frame pointer: has
     * the kernel been compiled with frame pointers?
     */
    bool have_kernel_frame_pointers;
    /*
     * Core/Pkg MSR residency addresses
     */
    unsigned int coreResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    unsigned int pkgResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    /*
     * On some archs, C-state residency MSRS do NOT count at TSC frequency.
     * For these, we need to apply a "clock multiplier". Record that
     * here.
     */
    unsigned int residency_count_multiplier;
    /*
     * What switches should the device driver collect?
     * Note: different from interface spec:
     * We're moving from bitwise OR to bitwise OR of (1 << switch) values.
     * Use the "POWER_XXX_MASK" masks to set/test switch residency.
     */
    int collection_switches;
    /*
     * Total time elapsed for
     * all collections.
     * Aggregated over all collections -- useful
     * in multiple PAUSE/RESUME scenarios
     */
    unsigned long totalCollectionTime;
    /*
     * Start and stop jiffy values for
     * the current collection.
     */
    unsigned long collectionStartJIFF, collectionStopJIFF;
    // Others...
}internal_state_t;

static internal_state_t INTERNAL_STATE;

#define IS_COLLECTING() (INTERNAL_STATE.cmd == START || INTERNAL_STATE.cmd == RESUME)
#define IS_SLEEPING() (INTERNAL_STATE.cmd == PAUSE)
#define IS_SLEEP_MODE() (INTERNAL_STATE.collection_switches & POWER_SLEEP_MASK)
#define IS_FREQ_MODE() (INTERNAL_STATE.collection_switches & POWER_FREQ_MASK)
#define IS_KTIMER_MODE() (INTERNAL_STATE.collection_switches & POWER_KTIMER_MASK)
#define IS_NON_PRECISE_MODE() (INTERNAL_STATE.collection_switches & POWER_SYSTEM_MASK)



/*
 * Some utility macros...
 */


/* Macro for printk based on verbosity */
#if DO_DEBUG_OUTPUT
#define OUTPUT(level, ...) do { if(unlikely(level <= verbosity)) printk(__VA_ARGS__); } while(0);
#else
#define OUTPUT(level, ...)
#endif // DO_DEBUG_OUTPUT

/*
 * Get a string value for a bool.
 */
#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )


#define CPU() (smp_processor_id())
#define PID() (current->pid)
#define TGID() (current->tgid)
#define NAME() (current->comm)
#define PKG(c) ( cpu_data(c).phys_proc_id )
#define IT_REAL_INCR() (current->signal->it_real_incr.tv64)

// (Safely Copy a long int value from user-space
#define GET_USER_LONG_VALUE(to, from) ({int __tmp_val = -1; pagefault_disable(); __tmp_val = __copy_from_user_inatomic( (to), (from), sizeof(unsigned long) ); pagefault_enable(); __tmp_val;})

#define C_DUMP_FULL(p,c,s,type,data)
#define P_DUMP_FULL(p,c,t,f)

#ifdef CONFIG_X86_32
#define get_bp(bp) asm("movl %%ebp, %0" : "=r" (bp) :)
#else
#define get_bp(bp) asm("movq %%rbp, %0" : "=r" (bp) :)
#endif

#define LOCK(l) {				\
    unsigned long _tmp_l_flags;			\
    spin_lock_irqsave(&(l), _tmp_l_flags);

#define UNLOCK(l)				\
    spin_unlock_irqrestore(&(l), _tmp_l_flags); \
    }

#define READ_LOCK(l) {				\
    unsigned long _tmp_l_flags;			\
    read_lock_irqsave(&(l), _tmp_l_flags);

#define READ_UNLOCK(l)				\
    read_unlock_irqrestore(&(l), _tmp_l_flags);	\
    }

#define WRITE_LOCK(l) {				\
    unsigned long _tmp_l_flags;			\
    write_lock_irqsave(&(l), _tmp_l_flags);

#define WRITE_UNLOCK(l)					\
    write_unlock_irqrestore(&(l), _tmp_l_flags);	\
    }

/**** HELPER FUNCTIONS ****/

/*
 * Per-cpu structure holding MSR residency counts,
 * timer-TSC values etc.
 */
typedef struct per_cpu_struct{
    unsigned int freqState; // 4 bytes
    pid_t last_pid; // 4 bytes
    pid_t last_tid; // 4 bytes
    int last_type; // 4 bytes
    unsigned int prev_state; // 4 bytes
    u64 old_alpha; // 8 bytes
    u64 tsc; // 8 bytes
    u64 residencies[MAX_MSR_ADDRESSES]; // 96 bytes
    u64 prev_msr_vals[MAX_MSR_ADDRESSES]; // 96 bytes
    // u64 debug_enters[3]; // 24 bytes
    u64 debug_enters;
    u64 overheadCount; // 8 bytes
    u64 last_break[3]; // 16 bytes
    unsigned char is_deferrable; // 1 byte
    unsigned char padding[24]; // 24 bytes
    void *sched_timer_addr;
}per_cpu_t; // 196 bytes


DEFINE_PER_CPU(per_cpu_t, per_cpu_counts);

/*
 * Convenience macros for accessing per-cpu residencies
 */
#define RESIDENCY(p,i) ( (p)->residencies[(i)] )
#define PREV_MSR_VAL(p,i) ( (p)->prev_msr_vals[(i)] )

/*
 * Per-cpu structure holding stats information.
 * Eventually, we may want to incorporate these fields within
 * the "per_cpu_t" structure.
 */
typedef struct{
    local_t c_breaks, timer_c_breaks, inters_c_breaks;
    local_t p_trans;
    local_t num_inters, num_timers;
}stats_t;


DEFINE_PER_CPU(stats_t, per_cpu_stats);

/*
 * Struct to hold old IA32_FIXED_CTR_CTRL MSR
 * values (to enable restoring
 * after pw driver terminates). These are
 * used to enable/restore/disable CPU_CLK_UNHALTED.REF
 * counting.
 */
typedef struct{
    u32 data[2];
}CTRL_values_t;

DEFINE_PER_CPU(CTRL_values_t, CTRL_data_values);

/*
 * Helper macro to declare variables required
 * for conducting overhead measurements.
 */
#if DO_OVERHEAD_MEASUREMENTS

#define DECLARE_OVERHEAD_VARS(name)					\
    DEFINE_PER_CPU(u64, name##_elapsed_time);				\
    DEFINE_PER_CPU(local_t, name##_num_iters) = LOCAL_INIT(0);		\
									\
    static inline u64 get_my_cumulative_elapsed_time_##name(void){	\
	return *(&__get_cpu_var(name##_elapsed_time));			\
    }									\
    static inline int get_my_cumulative_num_iters_##name(void){		\
	return local_read(&__get_cpu_var(name##_num_iters));		\
    }									\
									\
    static inline u64 name##_get_cumulative_elapsed_time_for(int cpu){	\
	return *(&per_cpu(name##_elapsed_time, cpu));			\
    }									\
									\
    static inline int name##_get_cumulative_num_iters_for(int cpu){	\
      return local_read(&per_cpu(name##_num_iters, cpu));		\
    }									\
									\
    static inline void name##_get_cumulative_overhead_params(u64 *time,	int *iters){ \
	int cpu = 0;							\
	*time = 0; *iters = 0;						\
	for_each_online_cpu(cpu){					\
	    *iters += name##_get_cumulative_num_iters_for(cpu);		\
	    *time += name##_get_cumulative_elapsed_time_for(cpu);	\
	}								\
	return;								\
    }									\
									\
    static inline void name##_init_overhead_params(void){		\
	int cpu = 0;							\
	for_each_online_cpu(cpu){					\
	    u64 *overhead = &per_cpu(name##_elapsed_time, cpu);		\
	    *overhead = 0;						\
	    local_set(&per_cpu(name##_num_iters, cpu), 0);		\
	}								\
    }

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
DECLARE_OVERHEAD_VARS(inter_common); // for the "IRQ" family of probes.
DECLARE_OVERHEAD_VARS(sched_switch); // for "probe_sched_switch"

#endif // DO_OVERHEAD_MEASUREMENTS


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
}

/*
 * MACRO helper to measure function call
 * times.
 */
#if DO_OVERHEAD_MEASUREMENTS

#define DO_PER_CPU_OVERHEAD(v, func, ...) do{		\
	u64 *__v = &__get_cpu_var(v##_elapsed_time);	\
	u64 tmp_1 = 0, tmp_2 = 0;			\
	local_inc(&__get_cpu_var(v##_num_iters));	\
	tscval(&tmp_1);					\
	{						\
	    func(__VA_ARGS__);				\
	}						\
	tscval(&tmp_2);					\
	*(__v) += (tmp_2 - tmp_1);			\
    }while(0)

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


#else // DO_OVERHEAD_MEASUREMENTS

#define DO_PER_CPU_OVERHEAD(v, func, ...) func(__VA_ARGS__)
#define DO_PER_CPU_OVERHEAD_FUNC(func, ...) func(__VA_ARGS__)

#endif // DO_OVERHEAD_MEASUREMENTS

enum{
    IRQ=0,
    TIMER,
    SCHED
};

/*
 * Find the next (non-full) per-cpu output buffer
 * to place a PWCollector_sample struct instance into.
 */
static inline seg_t *find_producer_seg(int cpu)
{
    list_t *list = &lists[cpu];
    int list_index = -1, num_full = 0;
    seg_t *seg = NULL;

    do{
	list_index = list->index;
	seg = &list->segs[list_index];
	/* if(IS_SEG_FULL(seg)){ */
	if(atomic_read(&seg->is_full) == FULL){
	    OUTPUT(0, KERN_INFO "[%d]: Segment %d is FULL! # full = %d\n", cpu, list_index, num_full);
	    list->index = CIRCULAR_INC(list_index, LIST_MASK);
	    continue;
	}
	return seg;
    }while(++num_full < NUM_SEGS_PER_LIST);

    printk(KERN_INFO "WARNING: List for CPU = %d was FULL!\n", cpu);

    return NULL;
};

/*
 * Insert a C-state sample into a (per-cpu) output buffer.
 */
static inline void produce_c_sample(int cpu, per_cpu_t *pcpu, int last_PID, int last_TID, char sample_type, unsigned long long sample_data)
{
    seg_t *seg = NULL;
    int seg_index = -1;


    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = seg->index;
    {
	SAMPLE(seg, seg_index).sample_type = C_STATE;
	// SAMPLE(seg, seg_index).cpu = cpu;
	SAMPLE(seg, seg_index).cpuidx = cpu;
	SAMPLE(seg, seg_index).tsc = pcpu->tsc;
	/* rdtscll(SAMPLE(seg, seg_index).tsc); */
	{
	    // memcpy(&RES_COUNT(C_SAMPLE(seg, seg_index),0), &pcpu->residencies, sizeof(int) * MAX_MSR_ADDRESSES); // dst, src
	    memcpy(&RES_COUNT(C_SAMPLE(seg, seg_index),0), pcpu->residencies, sizeof(u64) * MAX_MSR_ADDRESSES); // dst, src
	}
	C_SAMPLE(seg, seg_index).pid = last_PID;
	C_SAMPLE(seg, seg_index).tid = last_TID;
	C_SAMPLE(seg, seg_index).break_type = sample_type;
	C_SAMPLE(seg, seg_index).c_data = sample_data;

	C_SAMPLE(seg, seg_index).prev_state = pcpu->prev_state;

    }
    /*
     * OK, we've written to the segment.
     * Now increment indices, check for
     * full conditions etc.
     */
    seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
    if(!seg->index){ // index has wrapped around ==> FULL segment
	atomic_set(&seg->is_full, FULL);
#if ALLOW_BLOCKING_READ
	wake_up_interruptible(&read_queue);
#endif
    }
};

/*
 * Insert a P-state transition sample into a (per-cpu) output buffer.
 */
static inline void produce_p_sample(int cpu, unsigned long long res, unsigned int state)
{
    int seg_index = -1;
    seg_t *seg = NULL;

    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = seg->index;
    {
	SAMPLE(seg, seg_index).sample_type = P_STATE;
	// SAMPLE(seg, seg_index).cpu = cpu;
	SAMPLE(seg, seg_index).cpuidx = cpu;
	SAMPLE(seg, seg_index).tsc = res;
	/* rdtscll(SAMPLE(seg, seg_index).tsc); */
	P_SAMPLE(seg, seg_index).frequency = state;
    }
    /*
     * OK, we've written to the segment.
     * Now increment indices, check for
     * full conditions etc.
     */
    seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
    if(!seg->index){ // index has wrapped around ==> FULL segment
	atomic_set(&seg->is_full, FULL);
#if ALLOW_BLOCKING_READ
	wake_up_interruptible(&read_queue);
#endif
    }
};

/*
 * Insert a K_CALL_STACK sample into a (per-cpu) output buffer.
 */
static inline void produce_k_sample(int cpu, const trace_entry_t *tentry)
{
    int seg_index = -1;
    seg_t *seg = NULL;


    OUTPUT(1, KERN_INFO "KERNEL-SPACE mapping!\n");

    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = seg->index;
    {
	SAMPLE(seg, seg_index).sample_type = K_CALL_STACK;
	SAMPLE(seg, seg_index).sample_len = 1;
	// SAMPLE(seg, seg_index).cpu = cpu;
	SAMPLE(seg, seg_index).cpuidx = cpu;
	SAMPLE(seg, seg_index).tsc = tentry->tsc;
	K_SAMPLE(seg, seg_index).trace_len = tentry->trace_len;
	/*
	 * Generate the "entryTSC" and "exitTSC" values here.
	 * Note that this is GUARANTEED to be a k-space call stack.
	 */
	{
	    K_SAMPLE(seg, seg_index).entry_tsc = tentry->tsc - 1;
	    K_SAMPLE(seg, seg_index).exit_tsc = tentry->tsc + 1;
	}
	/*
	 * Also populate the trace here!
	 */
	if(tentry->trace_len){
	    int num = tentry->trace_len;
	    if(tentry->trace_len >= TRACE_LEN){
		printk(KERN_ERR "Warning: kernel trace len = %d > TRACE_LEN = %d! Will need CHAINING!\n", num, TRACE_LEN);
		num = TRACE_LEN;
	    }
	    memcpy(K_SAMPLE(seg, seg_index).trace, tentry->backtrace, sizeof(unsigned long) * num);
	}
    }
    /*
     * OK, we've written to the segment.
     * Now increment indices, check for
     * full conditions etc.
     */
    seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
    if(!seg->index){ // index has wrapped around ==> FULL segment
	atomic_set(&seg->is_full, FULL);
#if ALLOW_BLOCKING_READ
	wake_up_interruptible(&read_queue);
#endif
    }
};

/*
 * Insert an M_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_m_sample(int cpu, const char *name, unsigned long long begin, unsigned long long sz)
{
    int seg_index = -1;
    seg_t *seg = NULL;

    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = seg->index;
    {
	SAMPLE(seg, seg_index).sample_type = M_MAP;
	SAMPLE(seg, seg_index).cpuidx = cpu;
	M_SAMPLE(seg, seg_index).start = begin;
	M_SAMPLE(seg, seg_index).end = (begin+sz);
	M_SAMPLE(seg, seg_index).offset = 0;
	memcpy(M_SAMPLE(seg, seg_index).name, name, PW_MODULE_NAME_LEN); // dst, src
	OUTPUT(0, KERN_INFO "%s -> (0x%llx,0x%llx)\n", M_SAMPLE(seg, seg_index).name, M_SAMPLE(seg, seg_index).start, M_SAMPLE(seg, seg_index).end);
    }
    /*
     * OK, we've written to the segment.
     * Now increment indices, check for
     * full conditions etc.
     */
    seg->index = CIRCULAR_INC(seg_index, SAMPLE_MASK);
    if(!seg->index){ // index has wrapped around ==> FULL segment
	atomic_set(&seg->is_full, FULL);
#if ALLOW_BLOCKING_READ
	wake_up_interruptible(&read_queue);
#endif
    }
};

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
    strace.skip = 1;

    save_stack_trace(&strace);

    OUTPUT(3, KERN_INFO "[%d]: KERNEL TRACE: nr_entries = %d\n", PID(), strace.nr_entries);

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


/* Record the TSC when we hit a certain point (only the first time) */
/*
#define record_hit(s,point) do { if(unlikely((s)->debug_enters[(point)] == 0)) tscval(&((s)->debug_enters[(point)])); } while(0)
#define record_hit_full(s,point,pid,tid,which,what,d) do{	\
	if(unlikely( (s)->debug_enters[ (point) ] == 0)){	\
	    tscval(&((s)->debug_enters[(point)]));		\
	    (s)->last_pid = pid;				\
	    (s)->last_tid = tid;				\
	    (s)->last_break[(which)] = what;			\
	    (s)->is_deferrable = d;				\
	}							\
    }while(0)
*/

#define record_hit(s) do { if(unlikely((s)->debug_enters == 0)) tscval(&((s)->debug_enters)); } while(0)
#define record_hit_full(s,pid,tid,which,what,d) do{	\
	if(unlikely( (s)->debug_enters == 0)){		\
	    tscval(&((s)->debug_enters));		\
	    (s)->last_pid = pid;			\
	    (s)->last_tid = tid;			\
	    (s)->last_break[(which)] = what;		\
	    (s)->is_deferrable = d;			\
	}						\
    }while(0)


static trace_entry_t *find_trace_i(pid_t tid, unsigned long addr, int *index)
{
    int idx = hash_func_entries(addr);
    trace_entry_t **hash = NULL;
    trace_entry_t *curr = NULL;

    if(true){
	OUTPUT(3, KERN_INFO "Timer Addr = %lu, idx = %d, hash_ptr idx = %lu\n", addr, idx, (hash_ptr((void *)addr, NUM_TRACE_HASH_BITS)));
    }

    *index = idx;


    HASH_READ_LOCK(idx);
    {
	hash = &hash_table[idx];
	curr = *hash;

	while(curr && curr->timer_addr != addr) curr = curr->next;

    }
    HASH_READ_UNLOCK(idx);
    return curr;
};

/*
 * Find the entry corresponding to "timer_addr".
 * Uses a hash-based mechanism for efficiency.
 */
static inline trace_entry_t *find_trace_entry(pid_t tid, unsigned long timer_addr)
{
    int idx = -1;
    trace_entry_t *tentry = find_trace_i(tid, timer_addr, &idx);

    return tentry;
};


static void init_trace_entries(trace_entry_t *entries)
{
    int i=0;
    memset(entries, 0, MAX_NUM_TRACE_ENTRIES);

    for(i=1; i<MAX_NUM_TRACE_ENTRIES; ++i)
	entries[i-1].next = &entries[i];
};

/*
 * Switch from "GFP_KERNEL" --> "GFP_ATOMIC" because
 * this function *COULD* be called from a locked context.
 */
static trace_block_t *alloc_new_trace_block(void)
{
    trace_block_t *trace_block = (trace_block_t *)kmalloc(sizeof(trace_block_t), GFP_ATOMIC);

    if(!trace_block){
	printk(KERN_INFO "ERROR: could NOT allocate a new trace block!\n");
	atomic_set(&should_panic, 1);
	return NULL;
    }

    trace_block->block = (trace_entry_t *)kmalloc(sizeof(trace_entry_t) * MAX_NUM_TRACE_ENTRIES, GFP_ATOMIC);
    if(!trace_block->block){
	printk(KERN_INFO "ERROR: could NOT allocate a new trace entries array!\n");
	atomic_set(&should_panic, 1);
	return NULL;
    }

    total_num_trace_entries += MAX_NUM_TRACE_ENTRIES;

    init_trace_entries(trace_block->block);
    trace_block->next = NULL;

    if(trace_entry_blocks_tail) // conditional, because this routine is also called from "init_data_structures()"
	trace_entry_blocks_tail->next = trace_block;

    trace_entry_blocks_tail = trace_block;

    return trace_block;
};

static void destroy_trace_block(trace_block_t *block)
{
    if(!block)
	return;

    if(block->next)
	destroy_trace_block(block->next);

    kfree(block->block);

    kfree(block);
    return;
};

static trace_entry_t *get_next_free_entry_i(void)
{
    trace_entry_t *curr = NULL;
    trace_block_t *block = NULL;

    LOCK(free_list_lock);
    {
	curr = free_list_head;
	if(curr)
	    free_list_head = curr->next;
	if(--num_free_trace_entries <= TRACE_BLOCK_SIZE_REALLOC_THRESHOLD){
	    /*
	     * On some systems with LOTS of timer-related activity, we
	     * can get a large number of timer inits. We don't
	     * want to be allocating TOO much memory in that
	     * case -- guard against that here.
	     */
	    if(unlikely(++num_trace_reallocs >= NUM_TRACE_REALLOCS_ALLOWED)){
		printk(KERN_INFO "ERROR: too many trace reallocs (%d)\n", num_trace_reallocs);
		atomic_set(&should_panic, 1);
		if(curr){
		    curr->next = free_list_head;
		    free_list_head = curr;
		    ++num_free_trace_entries;
		    curr = NULL;
		}
	    }else{
		OUTPUT(0, KERN_INFO "# free entries = %llu, THRESHOLD = %d, ALLOCATING A NEW BLOCK!\n", num_free_trace_entries, TRACE_BLOCK_SIZE_REALLOC_THRESHOLD);
		block = alloc_new_trace_block();
		if(block){
		    block->block[MAX_NUM_TRACE_ENTRIES-1].next = free_list_head;
		    free_list_head = block->block;
		    num_free_trace_entries += MAX_NUM_TRACE_ENTRIES;
		}else if(curr){
		    printk(KERN_INFO "ERROR: ALLOC ERROR in get_next_free_entry!\n");
		    curr->next = free_list_head;
		    free_list_head = curr;
		    ++num_free_trace_entries;
		    curr = NULL;
		}
	    }
	}
	OUTPUT(3, KERN_INFO "\t # free entries [REMOVE]= %llu\n", num_free_trace_entries);
    }
    UNLOCK(free_list_lock);
    return curr;
};

static void add_to_free_list_i(trace_entry_t *node)
{
    LOCK(free_list_lock);
    {
	node->next = free_list_head;
	free_list_head = node;
	++num_free_trace_entries;
	OUTPUT(3, KERN_INFO "\t # free entries [ADD]= %llu\n", num_free_trace_entries);
    }
    UNLOCK(free_list_lock);
};

/*
 * Insert a new timer into the (hash-based) list of timer::TSC mappings.
 */
static inline int insert_trace_entry(pid_t tid, pid_t tgid, unsigned long timer_addr, unsigned long long tsc, int trace_len, unsigned long trace[])
{
    int idx = -1;
    trace_entry_t *curr = find_trace_i(tid, timer_addr, &idx);
    int retVal = SUCCESS;

    HASH_WRITE_LOCK(idx);
    {
	trace_entry_t **hash = hash_table+idx;
	if(!curr){
	    curr = get_next_free_entry_i();
	    if(curr){
		curr->next = *hash;
		*hash = curr;
	    }
	}
	if(curr){
	    curr->tid = tid; curr->tgid = tgid; curr->timer_addr = timer_addr; curr->tsc = tsc;
	    curr->trace_len = trace_len;
	    if(trace_len)
		memcpy(curr->backtrace, trace, sizeof(unsigned long) * trace_len); // dst, src
	}else{
	    printk(KERN_INFO "ERROR: NO FREE ENTRY exists in insert!\n");
	    retVal = -ERROR;
	}
    }
    HASH_WRITE_UNLOCK(idx);

    return retVal;
};


/*
 * Removes the trace entry for "timer_addr", but
 * ONLY if 'trace->tid' == tid. This is used in
 * the process exit tracepoints to conditionally
 * remove all trace entries for a give TID. We need
 * the conditional check because the following scenario is
 * possible (here 'Tx' is a timer address and 'Py' is a
 * thread/process; Py <-> Tx should be read as:
 * "Process y inits a timer whose addr is Tx").
 *
 * ***************************************************************
 * P1<->T1, T1 expires, P2<->T1, P1 exits [+], T1 expires[#]
 * ***************************************************************
 *
 * [+]: At this point, the P1<->T1 mapping exists, even though
 * the 'T1' timer addr now maps to 'P2'. Unconditionally removing
 * the trace for 'T1' will therefore cause a trace search
 * at [#] to return NULL i.e. we'll remove the P2<->T2 binding.
 */
static inline int remove_trace_entry_for_tid(pid_t tid, unsigned long timer_addr)
{
    int idx = -1;
    trace_entry_t *curr = find_trace_i(tid, timer_addr, &idx), *prev = NULL;
    trace_entry_t **hash = NULL;

    if(!curr){
	printk(KERN_INFO "ERROR: could NOT find addr=0x%lx in remove_trace!\n", timer_addr);
	return -ERROR;
    }

    if(curr->tid != tid){
	OUTPUT(0, KERN_INFO "Warning: Told to remove %lu (0x%lx) for TID = %d; but %lu now maps to TID = %d!\n", timer_addr, timer_addr, tid, timer_addr, curr->tid);
	return -ERROR;
    }

    HASH_WRITE_LOCK(idx);
    {
	hash = hash_table + idx;
	prev = *hash;
	if(prev == curr){
	    *hash = curr->next;
	}else{
	    while(prev->next != curr) prev = prev->next;
	    prev->next = curr->next;
	}
    }
    HASH_WRITE_UNLOCK(idx);

    add_to_free_list_i(curr);

    return SUCCESS;
};


/*
 * Destroy all trace entry mappings -- called
 * when a collection is STOPped or CANCELed.
 */
static int destroy_trace_entry_map(void)
{
    if(trace_entry_blocks_head)
	destroy_trace_block(trace_entry_blocks_head);
    trace_entry_blocks_head = trace_entry_blocks_tail = NULL;
    free_list_head = NULL;

    return SUCCESS;
};

/*
 * Init all trace entry mappings -- called
 * when a collection is STARTed, and also when
 * the driver is first initialized.
 */
static int init_trace_entry_map(void)
{
    total_num_trace_entries = 0;
    /*
     * Init the (hash-based) timer::tsc mapping list here.
     */
    trace_entry_blocks_head = trace_entry_blocks_tail = alloc_new_trace_block(); // (block_t *)malloc(sizeof(trace_entry_blocks_head));
    if(!trace_entry_blocks_head){
	printk(KERN_INFO "ERROR: could NOT allocate a block in INIT TRACE ENTRY MAP!\n");
	return -ERROR;
    }

    free_list_head = trace_entry_blocks_head->block;

    memset(hash_table, 0, sizeof(hash_table));


    num_free_trace_entries = MAX_NUM_TRACE_ENTRIES;


    return SUCCESS;
};

/*
 * ****************************************************************
 * TID <--> TIMER mapping: [BEGIN]
 * ****************************************************************
 */
static void init_ptnodes(ptnode_t *block)
{
    int i=0;
    memset(block, 0, sizeof(ptnode_t) * MAX_NUM_TRACE_ENTRIES);

    for(i=1; i<MAX_NUM_TRACE_ENTRIES; ++i)
	block[i-1].next = &block[i];
};

/*
 * Switch from "GFP_KERNEL" --> "GFP_ATOMIC" because
 * this function *COULD* be called from a locked context.
 */
static inline ptblock_t *alloc_new_pt_block(void)
{
    ptblock_t *ptblock = (ptblock_t *)kmalloc(sizeof(ptblock_t), GFP_ATOMIC);
    if(!ptblock){
	atomic_set(&should_panic, 1);
	return NULL;
    }
    ptblock->block = (ptnode_t *)kmalloc(sizeof(ptnode_t) * MAX_NUM_TRACE_ENTRIES, GFP_ATOMIC);
    if(!ptblock->block){
	kfree(ptblock);
	atomic_set(&should_panic, 1);
	return NULL;
    }

    total_num_ptnode_entries += MAX_NUM_TRACE_ENTRIES;

    init_ptnodes(ptblock->block);
    ptblock->next = NULL;

    if(ptblocks_tail)
	ptblocks_tail->next = ptblock;
    ptblocks_tail = ptblock;

    return ptblock;
};

static void free_ptblock(ptblock_t *block)
{
    if(!block)
	return;

    if(block->next)
	free_ptblock(block->next);

    kfree(block->block);

    kfree(block);
};

static inline void add_to_free_ptnode_list_i(ptnode_t *node)
{
	LOCK(ptnode_free_list_lock);
	{
		node->next = ptnode_free_list_head;
		ptnode_free_list_head = node;
		++num_free_ptnode_entries;
		OUTPUT(3, KERN_INFO "\t PTNODE: # free entries [ADD]= %llu\n", num_free_ptnode_entries);
	}
	UNLOCK(ptnode_free_list_lock);
};

static inline ptnode_t *get_next_free_ptnode(void)
{
	ptnode_t *curr = NULL;
	ptblock_t *block = NULL;
	LOCK(ptnode_free_list_lock);
	{
		curr = ptnode_free_list_head;
		if(curr){
			ptnode_free_list_head = curr->next;
			curr->next = NULL;
			curr->timer_list_head = NULL;
		}
		if(--num_free_ptnode_entries <= PTNODE_BLOCK_SIZE_REALLOC_THRESHOLD){
		    if(unlikely(++num_ptnode_reallocs >= NUM_PTNODE_REALLOCS_ALLOWED)){
			printk(KERN_INFO "ERROR: too many ptnode reallocs (%d)\n", num_ptnode_reallocs);
			atomic_set(&should_panic, 1);
			if(curr){
			    curr->next = ptnode_free_list_head;
			    ptnode_free_list_head = curr;
			    ++num_free_ptnode_entries;
			    curr = NULL;
			}
		    }else{
			OUTPUT(0, KERN_INFO "# free ptnodes = %llu, THRESHOLD = %d, ALLOCATING A NEW PTNODE BLOCK!\n", num_free_ptnode_entries, PTNODE_BLOCK_SIZE_REALLOC_THRESHOLD);
			block = alloc_new_pt_block();
			if(block){
			    block->block[MAX_NUM_TRACE_ENTRIES-1].next = ptnode_free_list_head;
			    ptnode_free_list_head = block->block;
			    num_free_ptnode_entries += MAX_NUM_TRACE_ENTRIES;
			}else if(curr){
			    curr->next = ptnode_free_list_head;
			    ptnode_free_list_head = curr;
			    ++num_free_ptnode_entries;
			    curr = NULL;
			}
		    }
		}
		OUTPUT(3, KERN_INFO "\t PTNODE: # free entries [REMOVE]= %llu\n", num_free_ptnode_entries);
	}
	UNLOCK(ptnode_free_list_lock);
	return curr;
};


static inline ptnode_t *find_pt_node(pid_t tid, int *index)
{
	unsigned long idx = pt_hash_func_entries(tid);
	ptnode_t *node = NULL;

	if(true){
	    OUTPUT(3, KERN_INFO "[%d]: idx = %lu\n", tid, idx);
	}

	*index = idx;

	PT_HASH_READ_LOCK(idx);
	{
		ptnode_t **hash = pt_hash_table+idx;
		node = *hash;
		while(node && node->tid != tid) node = node->next;
	}
	PT_HASH_READ_UNLOCK(idx);
	return node;
};

static inline ptnode_t *find_t_node(ptnode_t *node, unsigned long timer_addr)
{
    ptnode_t *tnode = node->timer_list_head;
    while(tnode && tnode->addr != timer_addr) tnode = tnode->next;

    return tnode;
};

static inline int insert_pt_mapping(pid_t tid, unsigned long timer_addr)
{
	int idx = -1;
	ptnode_t *node = find_pt_node(tid, &idx), *tnode = NULL;
	bool found = node != NULL;

	if(!found){
		node = get_next_free_ptnode();
		if(node){
		    node->tid = tid;
		}
	}

	if(!node){
		printk(KERN_INFO "ERROR: NO FREE ENTRY exists in insert!\n");
		atomic_set(&should_panic, 1);
		return -ERROR;
	}

	if(!found){
		/*
		 * We have to add "node" to the hash
		 * table.
		 */
		PT_HASH_WRITE_LOCK(idx);
		{
			ptnode_t **hash = pt_hash_table+idx;
			node->next = *hash;
			*hash = node;
		}
		PT_HASH_WRITE_UNLOCK(idx);
		OUTPUT(3, KERN_INFO "[%d]: ADDED NEW PT ENTRY!\n", tid);
	}
	/*
	 * OK, node for tid was either already present
	 * or has just been inserted. In either case,
	 * search for "timer_addr".
	 */
	if(!(tnode = find_t_node(node, timer_addr))){
		tnode = get_next_free_ptnode();
		if(tnode){
		    tnode->addr = timer_addr;
		    tnode->next = node->timer_list_head;
		    node->timer_list_head = tnode;
		    OUTPUT(3, KERN_INFO "[%d]: ADDED NEW T NODE!\n", tid);
		}else{
		    atomic_set(&should_panic, 1);
		    printk(KERN_INFO "ERROR: could NOT get next free ptnode!\n");
		}
	}

	return SUCCESS;
};


static void free_t_node(pid_t tid, ptnode_t *node)
{
	if(node->next)
		free_t_node(tid, node->next);
	/*
	 * Also remove the 'trace' entry
	 * corresponding to 'node->addr'
	 */
	{
		remove_trace_entry_for_tid(tid, node->addr);
		OUTPUT(3, KERN_INFO "[%d]: FREED TIMER ENTRY!\n", tid);
	}
	add_to_free_ptnode_list_i(node);
	OUTPUT(3, KERN_INFO "[%d]: FREED T NODE!\n", tid);
};

static inline int remove_pt_mapping(pid_t tid)
{
	int idx = -1;
	ptnode_t *node = find_pt_node(tid, &idx);
	if(!node){
		OUTPUT(3, KERN_INFO "[%d]: NO timer mappings found!\n", tid);
		return -ERROR;
	}
	free_t_node(tid, node->timer_list_head);

	PT_HASH_WRITE_LOCK(idx);
	{
		pthash_t *hash = pt_hash_table + idx;
		ptnode_t *prev = *hash;
		if(prev == node){
			*hash = node->next;
		}else{
			while(prev->next != node) prev = prev->next;
			prev->next = node->next;
		}
	}
	PT_HASH_WRITE_UNLOCK(idx);

	add_to_free_ptnode_list_i(node);

	OUTPUT(3, KERN_INFO "[%d]: FREED PT MAPPING!\n", tid);

	return SUCCESS;
};


static int init_pt_subsys(void)
{
    total_num_ptnode_entries = 0;

    ptblocks_head = ptblocks_tail = alloc_new_pt_block();
    if(!ptblocks_head)
	return -ERROR;
    ptnode_free_list_head = ptblocks_head->block;
    num_free_ptnode_entries = MAX_NUM_TRACE_ENTRIES;

    /*
     * Set # free entries: this MUST be set BEFORE
     * calling 'get_next_free_node(...)'!!!
     */
    num_free_ptnode_entries = MAX_NUM_TRACE_ENTRIES;

    /*
     * Initialize the PT hash table.
     */

    memset(pt_hash_table, 0, sizeof(pt_hash_table));

    return SUCCESS;
};

static void destroy_pt_subsys(void)
{
    free_ptblock(ptblocks_head);
    ptnode_free_list_head = NULL;
    ptblocks_head = ptblocks_tail = NULL;
};

/*
 * ****************************************************************
 * TID <--> TIMER mapping: [END]
 * ****************************************************************
 */

static int init_tracing_subsystem(void)
{
    if(init_trace_entry_map())
	return -ERROR;
    if(init_pt_subsys())
	return -ERROR;
    return SUCCESS;
};

static int destroy_tracing_subsystem(void)
{
    if(destroy_trace_entry_map())
	return -ERROR;

    destroy_pt_subsys();

    return SUCCESS;
};

/*
 * Function common to all timer initialization tracepoints.
 * Measure TSC value and create timer::TSC mapping.
 * Also generates kernel-space call stack (if applicable).
 */
static void timer_init(void *timer)
{
    const char *name = NAME();
    pid_t pid = PID(); /* This is actually the TID! */
    pid_t tgid = TGID(); /* This is actually the PID! */
    unsigned long long tsc = 0;
    int trace_len = 0;
    unsigned long trace[MAX_BACKTRACE_LENGTH];

    tscval(&tsc);

    OUTPUT(2, KERN_INFO "[%d,%d]: %.20s TIMER_INIT: tsc=%llu\n", pid, tgid, name, tsc);
    if(false && !strcmp(name, "a.out")){
	struct task_struct *tsk = current;
	ktime_t k = tsk->signal->it_real_incr;
	OUTPUT(0, KERN_INFO "\t[%d]: %.20s IT_REAL_INCR = %lld\n", pid, name, k.tv64);
	if(k.tv64 != 0)
	    OUTPUT(0, KERN_INFO "\t\tNON-ZERO ITIMER INCREMENT!\n");
    }
    /*
     * For now, force K-TIMER mode.
     */
    //if(!pid && (INTERNAL_STATE.collection_switches & POWER_KTIMER_MASK)){
    /* if(!pid && IS_KTIMER_MODE()){ */
    if(!pid){
	/*
	 * get kernel timerstack here.
	 * Requires the kernel be compiled with
	 * frame_pointers on.
	 *
	 * UPDATE: ONLY CALL THIS IF FRAME POINTERS ARE ENABLED -- CHECK THAT
	 * HERE.
	 */
	if(INTERNAL_STATE.have_kernel_frame_pointers)
	    trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	else
	    trace_len = 0;
	OUTPUT(2, KERN_INFO "KERNEL-SPACE timer INIT! Addr = %p\n", timer);
    }
    /*
     * Store the timer if:
     * (a) called for a ROOT process (tid == 0) OR
     * (b) we're actively COLLECTING.
     */
    if(!pid || IS_COLLECTING()){
	insert_trace_entry(pid, tgid, (unsigned long)timer, tsc, trace_len, trace);
	/*
	 * There could have been a mem allocation error -- check
	 * that first.
	 */
	if(atomic_read(&should_panic))
	    return;
	insert_pt_mapping(pid, (unsigned long)timer);
    }
};

/*
 * High resolution timer (hrtimer) initialization probe.
 * Fired on hrtimer initializations.
 */
#if (KERNEL_VER < 35)
static void probe_hrtimer_init(struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#else
    static void probe_hrtimer_init(void *ignore, struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#endif
{
    if(!SHOULD_TRACE())
	return;

    OUTPUT(3, KERN_INFO "[%d]: HRTIMER_INIT: Timer %p initialized\n", PID(), timer);
    /*
    if(false){
	struct task_struct *tsk = current;
	ktime_t k = tsk->signal->it_real_incr;
	OUTPUT(0, KERN_INFO "\tIT_REAL_INCR = %lld\n", k.tv64);
    }
    */
    /* timer_init(timer); */
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

/*
 * Timer initialization probe.
 * Fired on timer initializations.
 */
#if (KERNEL_VER < 35)
static void probe_timer_init(struct timer_list *timer)
#else
    static void probe_timer_init(void *ignore, struct timer_list *timer)
#endif
{
    if(!SHOULD_TRACE())
	return;

    OUTPUT(2, KERN_INFO "[%d]:TIMER_INIT: Timer %p initialized\n", PID(), timer);
    // timer_init(timer);
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

/*
 * Interval timer state probe.
 * Fired on interval timer initializations
 * (from "setitimer(...)")
 */
#if (KERNEL_VER < 35)
static void probe_itimer_state(int which, const struct itimerval *const value, cputime_t expires)
#else
    static void probe_itimer_state(void *ignore, int which, const struct itimerval *const value, cputime_t expires)
#endif
{
    struct hrtimer *timer = &current->signal->real_timer;

    if(!SHOULD_TRACE())
	return;

    // OUTPUT(3, KERN_INFO "[%d]: %.20s ITIMER STATE: which=%d, value=%p, expires=%lu (TIMER=%p)\n", PID(), NAME(), which, value, expires, timer);
    OUTPUT(3, KERN_INFO "[%d]: ITIMER STATE: timer = %p\n", PID(), timer);

    // timer_init(timer);
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

static u32 tick_count = 0;
static DEFINE_SPINLOCK(tick_count_lock);
static bool should_probe_on_hrtimer_start = true;

DEFINE_PER_CPU(local_t, sched_timer_found) = LOCAL_INIT(0);


#if (KERNEL_VER < 35)
static void probe_hrtimer_start(struct hrtimer *hrt)
#else
    static void probe_hrtimer_start(void *ignore, struct hrtimer *hrt)
#endif
{
    int cpu = CPU();
    pid_t tid = PID();
    pid_t pid = TGID();
    u64 tsc = 0;
    /* const char *name = hrt->start_comm; */
    int i, trace_len;
    char symname[KSYM_NAME_LEN];
    unsigned long trace[MAX_BACKTRACE_LENGTH];
    void *sched_timer_addr = NULL;
    per_cpu_t *pcpu = NULL;
    bool should_unregister = false;

    if(!SHOULD_TRACE())
	return;

    if(!should_probe_on_hrtimer_start){
	OUTPUT(3, KERN_INFO "HRTIMER_START: timer = %p\n", hrt);
	return;
    }

    /*
     * Not sure if "save_stack_trace" or "sprint_symbol" can
     * sleep. To be safe, use the "__get_cpu_var" variants
     * here. Note that it's OK if they give us stale values -- we're
     * not looking for an exact value.
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

    OUTPUT(0, KERN_INFO "[%d]: %.20s TIMER_START for timer = %p\n", tid, hrt->start_comm, hrt);
    if(INTERNAL_STATE.have_kernel_frame_pointers){
	trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	for(i=0; i<trace_len; ++i){
	    //lookup_symbol_name(trace[i], symname);
	    sprint_symbol(symname, trace[i]);
	    OUTPUT(3, KERN_INFO "SYM MAPPING: 0x%lx --> %s\n", trace[i], symname);
	    if(strstr(symname, "cpu_idle")){
		printk(KERN_INFO "FOUND CPU IDLE for cpu = %d . TICK SCHED TIMER = %p\n", cpu, hrt);
		local_inc(&__get_cpu_var(sched_timer_found));
		// *timer_found = true;
		sched_timer_addr = hrt;
	    }
	}
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

		insert_trace_entry(tid, pid, (unsigned long)sched_timer_addr, tsc, trace_len, trace);
		insert_pt_mapping(tid, (unsigned long)sched_timer_addr);
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
 * An EMPTY trace structure.
 */
static const struct trace_entry EMPTY_trace_entry = {
    .tid = 0,
    .tgid = 0,
    .timer_addr = 0,
    .tsc = 0x1,
    .trace_len = 0,
    .next = NULL,
};

/*
 * Function common to all timer expire tracepoints.
 * Write the Timer::TSC mapping to the (per-cpu) output
 * buffers.
 */
static void timer_expire(void* timer_addr, pid_t pid, bool is_deferrable)
{
    int cpu;
    per_cpu_t *pcpu = NULL;
    const trace_entry_t *tentry = NULL;
    pid_t tgid = 0;
    unsigned long long tsc = 0x1; // We need AT LEAST a non-zero TSC value to tell "power_start" that break was caused by a timer.
    bool tentry_found = false;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    if(!in_interrupt()){
	printk(KERN_ERR "BUG: timer_expire called from a NON-INTERRUPT context!\n");
	return;
    }

    /*
     * Atomic context => use __get_cpu_var(...) instead of get_cpu_var(...)
     */
    pcpu = &__get_cpu_var(per_cpu_counts);

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
#endif // DO_IOCTL_STATS

    cpu = CPU();

    if(!timer_addr){ // should *NEVER* happen!
	printk(KERN_INFO "ERROR: NULL timer_addr passed in to timer expire function!\n");
	return;
    }

    if(!(tentry = find_trace_entry(pid, (unsigned long)timer_addr))){
	/*
	 * Couldn't find timer entry -- PID defaults to TID.
	 */
	tgid = pid;
	tentry = &EMPTY_trace_entry;
	OUTPUT(3, KERN_INFO "Warning: [%d]: timer %p NOT found in list!\n", pid, timer_addr);
    }else{
	tentry_found = true;
	tgid = tentry->tgid;
	OUTPUT(3, KERN_INFO "[%d]: timer %p FOUND in list: tid=%d, pid=%d\n", pid, timer_addr, tentry->tid, tgid);
	// tsc = tentry->tsc;
    }
    tsc = tentry->tsc;

    if(!tsc){
	OUTPUT(3, KERN_INFO "ERROR: EMPTY TSC field!\n");
    }

    record_hit_full(pcpu, tgid, pid, TIMER, tsc, is_deferrable);


    if(!pid && !tentry){
	OUTPUT(0, KERN_INFO "KERNEL-SPACE expire: no tentry, using tsc = %llu\n", EMPTY_trace_entry.tsc);
    }

    /*
     * OK, send the TIMER::TSC mapping & call stack to the user
     * (but only if this is for a kernel-space call stack and the
     * user wants kernel call stack info).
     */
    if(!pid && INTERNAL_STATE.write_to_buffers && (INTERNAL_STATE.collection_switches & POWER_KTIMER_MASK))
	produce_k_sample(cpu, tentry);

};

/*
 * High resolution timer (hrtimer) expire entry probe.
 */
#if (KERNEL_VER < 35)
static void probe_hrtimer_expire_entry(struct hrtimer *hrt, ktime_t *now)
#else
    static void probe_hrtimer_expire_entry(void *ignore, struct hrtimer *hrt, ktime_t *now)
#endif
{
    pid_t pid = hrt->start_pid;

    if(!SHOULD_TRACE())
	return;

    OUTPUT(3, KERN_INFO "[%d]: HRTIMER expire: timer = %p\n", pid, hrt);
    // timer_expire(hrt, pid, false); // "hrtimers" are *NEVER* deferrable! (CHECK THIS!!!)
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, hrt, pid, false);
};

/*
 * Timer expire entry probe.
 */
#if (KERNEL_VER < 35)
static void probe_timer_expire_entry(struct timer_list *t)
#else
    static void probe_timer_expire_entry(void *ignore, struct timer_list *t)
#endif
{
    pid_t pid = t->start_pid;

    if(!SHOULD_TRACE())
	return;

    OUTPUT(2, KERN_INFO "[%d]: TIMER_EXPIRE: Timer %p expired\n", pid, t);
    // timer_expire(t, pid, IS_TIMER_DEFERRABLE(t));
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, t, pid, IS_TIMER_DEFERRABLE(t));
};

/*
 * Interval timer expire probe.
 *
 * NOTE: 'itimer_expire' is NOT probed. This is because, internally, itimers
 * are implemented as HRTIMERs. Consequently, EVERY itimer_expire probe WILL
 * be PRECEEDED by a corresponding hrtimer_expire probe.
 */
/*
#if (KERNEL_VER < 35)
static void probe_itimer_expire(int which, struct pid *spid, cputime_t now)
#else
    static void probe_itimer_expire(void *ignore, int which, struct pid *spid, cputime_t now)
#endif
{
    pid_t pid = pid_nr(spid);
    struct task_struct *tsk = pid_task(spid, PIDTYPE_PID);
    struct hrtimer *timer = &tsk->signal->real_timer;
    if(which != ITIMER_REAL)
	OUTPUT(0, KERN_INFO "NON-REAL ITIMER!\n");
    OUTPUT(0, KERN_INFO "ITIMER_EXPIRE in PID = %d, tsk = %p, TID = %d, timer = %p\n", pid, tsk, tsk->pid, timer);

    timer_expire(NULL, pid, false);
};
*/

/*
 * Function common to all interrupt tracepoints.
 */
static void inter_common(int irq_num)
{
    /* Note that this is *always* called from within an interrupt
     * context
     */
    // int cpu;
    per_cpu_t *pcpu = NULL;
    unsigned char is_deferrable = 0;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    if(!in_interrupt()){
	printk(KERN_ERR "BUG: inter_common() called from a NON-INTERRUPT context! Got irq: %lu and soft: %lu\n", in_irq(), in_softirq());
	return;
    }

    pcpu = &__get_cpu_var(per_cpu_counts);

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
    local_inc(&pstats->num_inters);

    /*
     * Increment counter for timer interrupts as well.
     */
    if(in_softirq() && (irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ))
	local_inc(&pstats->num_timers);
#endif

    // cpu = CPU();


    /*
     * Record the TSC values, *ONLY* if this is the *FIRST*
     * interrupt *AFTER* the last TPS.
     * (Record hit internally checks this condition)
     *
     * Note that this version does *NOT* defer timer interrupts
     * to the "expire" probes.
     */
    /* record_hit_full(pcpu, TGID(), PID(), IRQ, irq_num, is_deferrable); // PID() will only be an approximation in this case! */
    record_hit_full(pcpu, 0, 0, IRQ, irq_num, is_deferrable); // PID() will only be an approximation in this case!

    OUTPUT(3, KERN_INFO "*** Interrupt Event ***\n");
    OUTPUT(2, KERN_INFO "\n");
};

/*
 * IRQ tracepoint.
 */
#if (KERNEL_VER < 35)
static void probe_irq_handler_entry(int irq, struct irqaction *action)
#else
    static void probe_irq_handler_entry(void *ignore, int irq, struct irqaction *action)
#endif
{
    if(!SHOULD_TRACE())
	return;
    // inter_common(irq);
    DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq);
};

/*
 * soft IRQ tracepoint.
 */
#if (KERNEL_VER < 35)
static void probe_softirq_entry(struct softirq_action *h, struct softirq_action *vec)
#else
    static void probe_softirq_entry(void *ignore, struct softirq_action *h, struct softirq_action *vec)
#endif
{
    int irq = (int)(h-vec);

    if(!SHOULD_TRACE())
	return;
    // inter_common(irq);
    DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq);
};

#define C1 APERF
#define CN MAX_MSR_ADDRESSES

#define UNHLT_REF MPERF

const char *msr_names[] = {"C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"};

// DEFINE_PER_CPU(local_t, multiple_c_states) = LOCAL_INIT(0);
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
	u64 res;
	int msr_addr;
	u64 begin_tsc, end_tsc;
	u64 alpha, beta;
	u64 OMEGA, theta, c0;
	u64 m_delta, c_delta;
	bool is_first;
	u64 cx_total = 0;
	u32 clock_multiplier = INTERNAL_STATE.residency_count_multiplier;


	is_first = PREV_MSR_VAL(pcpu, MPERF) == 0;

	if(is_first){
	    OUTPUT(3, KERN_INFO "[%d]: PREV_STATE = %u\n", cpu, pcpu->prev_state);
	}

	tscval(&begin_tsc);

	msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[MPERF];
	rdmsrl(msr_addr, res);
	if(unlikely(res < PREV_MSR_VAL(pcpu, MPERF))){
		/*
		 * W've had a rollover!
		 */
	    PREV_MSR_VAL(pcpu, MPERF) = (unsigned long long)(~0) - PREV_MSR_VAL(pcpu, MPERF);
	    res += PREV_MSR_VAL(pcpu, MPERF) + 1;
	    PREV_MSR_VAL(pcpu, MPERF) = 0;
	    printk(KERN_INFO "Warning: ROLLOVER for MPERF counter for CPU %u\n", cpu);
	}
	m_delta = res - PREV_MSR_VAL(pcpu, MPERF);
	PREV_MSR_VAL(pcpu, MPERF) = res;
	/*
	 * Calculate (non-C1) C-state residency.
	 */
	for(i=C1; i<CN; ++i){
		c_delta = RESIDENCY(pcpu, i) = 0;
		if( (msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[i]) <= 0)
		    continue;
		rdmsrl(msr_addr, res);
		if(unlikely(res < PREV_MSR_VAL(pcpu, i))){
		    /*
		     * W've had a rollover!
		     */
		    PREV_MSR_VAL(pcpu, i) = (unsigned long long)(~0) - PREV_MSR_VAL(pcpu, i);
		    res += PREV_MSR_VAL(pcpu, i) + 1;
		    PREV_MSR_VAL(pcpu, i) = 0;
		    OUTPUT(0, KERN_INFO "Error: Cpu %d has a ROLLOVER in %s\n", cpu, msr_names[i]);
		}
		if(!is_first){
			if( (c_delta = res - PREV_MSR_VAL(pcpu, i))){
				++num_deltas;
				if(!which_c_state)
					which_c_state = i;
				OUTPUT(3, KERN_INFO "c_delta = %llu, multiplier = %u, ACTUAL = %llu\n", c_delta, clock_multiplier, (c_delta * clock_multiplier));
				c_delta *= clock_multiplier;
			}
			RESIDENCY(pcpu, i) = c_delta;
			cx_total += c_delta;
		}
		PREV_MSR_VAL(pcpu, i) = res;
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
				OUTPUT(0, KERN_INFO "[%d]: WARNING: OMEGA is less than THETA! Difference = %llu\n", cpu, (theta-OMEGA));
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
static void tps(unsigned int type, unsigned int state)
{
    per_cpu_t *pcpu = NULL;
    int cpu = -1, pkg=-1;
    int last_PID = -1, last_TID = -1;
    unsigned long long last_IRQ = 0, last_TIMER = 0,  last_SCHED = 0;
    unsigned char is_deferrable = 0;
    char sample_type = 'I';
    unsigned long long sample_data = 0;
    int should_add = true;

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
    /*
     * Update: set "pkg" in the
     * ANALYZER instead of the
     * kernel module.
     */
    // pkg = PKG(cpu);
    pkg = 0;

    // read_residency(cpu, pcpu);
    // if(read_residency(cpu, pcpu))
    /*
    if(read_residency(cpu, pcpu, state))
	should_add = false;
    */
    read_residencies(pcpu, cpu);

    /*
     * Reset all "debug_enters" counters.
     * This signals the next timer/interrupt (i.e. C-state break) event
     * to collect information.
     */
    /* for(i=0; i<MAX_DEBUG; ++i) */
    /* 	pcpu->debug_enters[i] = 0; */
    pcpu->debug_enters = 0;

    last_PID = pcpu->last_pid;
    last_TID = pcpu->last_tid;
    last_IRQ = pcpu->last_break[IRQ];
    last_TIMER = pcpu->last_break[TIMER];
    last_SCHED = pcpu->last_break[SCHED];
    is_deferrable = pcpu->is_deferrable;

    // put_cpu_var(*pcpu);
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
	if(sample_data == 0x1) // possible if timer expire had no corresponding timer init: e.g. timer was inited BEFORE driver started.
	    sample_data = 0x0;
	/*
	 * UPDATE: we're doing away with 'D' and 'N'.
	 */
	/*
	if(is_deferrable)
	    sample_type = 'D';
	else
	    sample_type = 'N';
	*/
	// sample_type = 'T';
	sample_type = PW_BREAK_TYPE_T;

	if(last_PID < 0 || last_TID < 0){
	    printk(KERN_INFO "ERROR: SCHED TPS TID == %d\n", last_TID);
	}

#if DO_IOCTL_STATS
	local_inc(&pstats->timer_c_breaks);
#endif

    }else if( (sample_data = last_SCHED) > 0){
	/*
	 * C-state break was caused by the "need_resched" flag
	 * being set AND a context switch actually taking
	 * place.
	 */
	// sample_type = 'S';
	sample_type = PW_BREAK_TYPE_S;
	OUTPUT(3, KERN_INFO "SCHED BREAK!\n");
    }else{
	/*
	 * Unknown reason for C-state break.
	 */
	// sample_type = 'U';
	// sample_type = '?';
	sample_type = PW_BREAK_TYPE_U;
	sample_data = 0;
    }

    if(!should_add){
	OUTPUT(2, KERN_INFO "\tInvalid C-state totals: Break Type = %c\n", sample_type);
    }

    /* if(INTERNAL_STATE.write_to_buffers) */
    if(INTERNAL_STATE.write_to_buffers && should_add)
	produce_c_sample(cpu, pcpu, last_PID, last_TID, sample_type, sample_data);

    else{
	C_DUMP_FULL(pkg, cpu, pcpu, sample_type, sample_data);
    }

    pcpu->last_pid = pcpu->last_tid = -1;
    pcpu->last_break[IRQ] = pcpu->last_break[TIMER] =  pcpu->last_break[SCHED] = 0;
    pcpu->is_deferrable = 0;

    pcpu->prev_state = state;
};

/*
 * C-state break.
 * Read MSR residencies.
 * Also gather information on what caused C-state break.
 * If so configured, write C-sample information to (per-cpu)
 * output buffer.
 */
#if (KERNEL_VER < 35)
static void probe_power_start(unsigned int type, unsigned int state)
#else
    static void probe_power_start(void *ignore, unsigned int type, unsigned int state)
#endif
{
	if(unlikely(!IS_SLEEP_MODE())){
		return;
	}

    if(!SHOULD_TRACE())
	return;

    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
};

/*
 * TPF helper -- required for overhead measurements.
 */
static void tpf(unsigned int type, unsigned int state)
{
    int cpu = CPU();
    unsigned long long res = 0;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    OUTPUT(3, KERN_INFO "probe_power_frequency, cpu = %d, type = %u, state = %u\n", cpu, type, state);

    /*
     * Read TSC value
     */
    // rdtscll(res);
    tscval(&res);

    if(INTERNAL_STATE.write_to_buffers){
	produce_p_sample(cpu, res, state);
    }else{
	P_DUMP_FULL(pkg, cpu, res, state);
    }

#if DO_IOCTL_STATS
    {
	pstats = &get_cpu_var(per_cpu_stats);
	local_inc(&pstats->p_trans);
	put_cpu_var(pstats);
    }
#endif // DO_IOCTL_STATS
};

/*
 * P-state transition probe.
 *
 * "type" is ALWAYS "2" (i.e. "POWER_PSTATE", see "include/trace/power.h")
 * "state" is the NEXT frequency range the CPU is going to enter (see "arch/x86/kernel/cpu/cpufreq/acpi-cpufreq.c")
 */
#if (KERNEL_VER < 35)
static void probe_power_frequency(unsigned int type, unsigned int state)
#else
    static void probe_power_frequency(void *ignore, unsigned int type, unsigned int state)
#endif
{
	if(unlikely(!IS_FREQ_MODE())){
		return;
	}

    if(!SHOULD_TRACE())
	return;

    DO_PER_CPU_OVERHEAD_FUNC(tpf, type, state);
};

/*
 * Thread exit tracepoint.
 */
#if (KERNEL_VER < 35)
static void probe_sched_process_exit(struct task_struct *task)
#else
static void probe_sched_process_exit(void *ignore, struct task_struct *task)
#endif
{
    pid_t pid = task->pid; /* 'PID' is actually the 'TID' !!!*/
    /*
    const char *name = task->comm;
    int cpu = CPU();
    OUTPUT(0, KERN_INFO "[%d]: %.20s EXIT on Cpu = %d\n", pid, name, cpu);
    */

    if(!SHOULD_TRACE())
	return;

    {
	remove_pt_mapping(pid);
    }
};

/*
 * Sched switch helper -- required for
 * overhead measurements.
 */
static void sched_switch(struct task_struct *prev, struct task_struct *next)
{
    int cpu = -1;
    per_cpu_t *pcpu = NULL;
    u64 tsc;
    pid_t ptid = prev->pid, ppid = prev->tgid;
    /* pid_t ntid = next->pid, npid = next->tgid; */
    /* u64 sample_data = ntid << 32 | ptid; */

    OUTPUT(3, KERN_INFO "[%d]: SCHED SWITCH!\n", CPU());

    tscval(&tsc);

    pcpu = &get_cpu_var(per_cpu_counts);
    {
	cpu = CPU();
	record_hit_full(pcpu, ppid, ptid, SCHED, tsc, 0 /* is_deferrable */);
	/* record_hit_full(pcpu, ppid, ptid, SCHED, sample_data, 0 /\* is_deferrable *\/); */
    }
    put_cpu_var(pcpu);
};

/*
 * Scheduler switch tracepoint.
 */
#if (KERNEL_VER < 35)
static void probe_sched_switch(struct rq* rq, struct task_struct *prev, struct task_struct *next)
#else
    static void probe_sched_switch(void *ignore, struct task_struct *prev, struct task_struct *next)
#endif
{
    if(!SHOULD_TRACE())
	return;
    DO_PER_CPU_OVERHEAD_FUNC(sched_switch, prev, next);
};

/*
 * Module load tracepoint.
 * We register a module load event -- extract memory
 * bounds for the module.
 */
#if (KERNEL_VER < 35)
static void probe_module_load(struct module *mod)
#else
    static void probe_module_load(void *ignore, struct module *mod)
#endif
{
    int cpu = CPU();
    const char *name = mod->name;
    unsigned long module_core = (unsigned long)mod->module_core;
    unsigned long core_size = mod->core_size;

    if(!SHOULD_TRACE())
	return;

    printk(KERN_INFO "Module %s LOADED! START = 0x%lx, SIZE = %lu\n", name, module_core, core_size);

    produce_m_sample(cpu, name, module_core, core_size);

    return;
};

static int register_timer_callstack_probes(void)
{
    int ret = 0;

#if (KERNEL_VER < 35)
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
#if (KERNEL_VER < 35)
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
 * Register all tracepoints.
 */
#if (KERNEL_VER < 35)
static int register_all_trace_probes(void)
{
    int ret = 0;


    {
    OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
      ret = register_trace_timer_expire_entry(probe_timer_expire_entry);
      WARN_ON(ret);
      ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
      WARN_ON(ret);
      /* ret = register_trace_itimer_expire(probe_itimer_expire); */
      /* WARN_ON(ret); */
	ret = register_trace_irq_handler_entry(probe_irq_handler_entry);
	WARN_ON(ret);
	ret = register_trace_softirq_entry(probe_softirq_entry);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
	ret = register_trace_power_start(probe_power_start);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	ret = register_trace_power_frequency(probe_power_frequency);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tMODULE_LOAD_EVENTS");
	ret = register_trace_module_load(probe_module_load);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tSCHED_SWITCH_EVENTS");
	ret = register_trace_sched_switch(probe_sched_switch);
	WARN_ON(ret);
    }

    return SUCCESS;
};
#else
static int register_all_trace_probes(void)
{
    int ret = 0;

    {
      OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
      ret = register_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
      WARN_ON(ret);
      ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
      WARN_ON(ret);
      /* ret = register_trace_itimer_expire(probe_itimer_expire, NULL); */
      /* WARN_ON(ret); */
	ret = register_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
	WARN_ON(ret);
	ret = register_trace_softirq_entry(probe_softirq_entry, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
	ret = register_trace_power_start(probe_power_start, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	ret = register_trace_power_frequency(probe_power_frequency, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tMODULE_LOAD_EVENTS");
	ret = register_trace_module_load(probe_module_load, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tSCHED_SWITCH_EVENTS");
	ret = register_trace_sched_switch(probe_sched_switch, NULL);
	WARN_ON(ret);
    }

    return SUCCESS;
};
#endif

/*
 * Unregister all (previously registered) tracepoints.
 */
#if (KERNEL_VER < 35)
static void unregister_all_trace_probes(void)
{

    {
      unregister_trace_timer_expire_entry(probe_timer_expire_entry);
      unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
      /* unregister_trace_itimer_expire(probe_itimer_expire); */
	unregister_trace_irq_handler_entry(probe_irq_handler_entry);
	unregister_trace_softirq_entry(probe_softirq_entry);

      tracepoint_synchronize_unregister();
    }


    {
	unregister_trace_power_start(probe_power_start);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_power_frequency(probe_power_frequency);

	tracepoint_synchronize_unregister();
    }


    {
	unregister_trace_module_load(probe_module_load);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_sched_switch(probe_sched_switch);

	tracepoint_synchronize_unregister();
    }

    return;
};
#else
static void unregister_all_trace_probes(void)
{

    {
      unregister_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
      unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
      /* unregister_trace_itimer_expire(probe_itimer_expire, NULL); */
	unregister_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
	unregister_trace_softirq_entry(probe_softirq_entry, NULL);

      tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_power_start(probe_power_start, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_power_frequency(probe_power_frequency, NULL);

	tracepoint_synchronize_unregister();
    }


    {
	unregister_trace_module_load(probe_module_load, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_sched_switch(probe_sched_switch, NULL);

	tracepoint_synchronize_unregister();
    }

    return;
};
#endif

/*
 * **********************************************
 * ASSUMPTION:
 * We assume there is AT MOST ONE CONSUMER
 * i.e. AT MOST ONE (USER-LEVEL) PROCESS
 * CALLING "read(...)" ON THE DEVICE DRIVER!!!
 * **********************************************
 */

/*
 * Single consumer: OK to have
 * unprotected access to read indices!
 */
static int last_list_read = -1;

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
	list = &lists[cpu];
	for(j=0; j<NUM_SEGS_PER_LIST; ++j){
	    seg = &list->segs[j];
	    if(atomic_read(&seg->is_full) == FULL){
	    /* if(IS_SEG_FULL(seg)){ */
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
	list = &lists[list_index];
	for(j=0; j<NUM_SEGS_PER_LIST; ++j){
	    seg = &list->segs[j];
	    if(atomic_read(&seg->is_full) == FULL){
	    /* if(IS_SEG_FULL(seg)){ */
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

static int last_flush_index = 0;

#define for_each_segment(idx) for((idx)=0; (idx) < NUM_SEGS_PER_LIST; ++(idx))

static inline seg_t *find_next_non_empty_seg(void)
{
    int j=0, seg_index = -1;
    list_t *list = NULL;
    seg_t *seg = NULL;

    while(last_flush_index < PW_max_num_cpus){
	list = &lists[last_flush_index];
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
    int bytes_read = 0, bytes_not_copied = 0;
    ssize_t size = SEG_SIZE;
    seg_t *seg = NULL;


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

    /*
     * Unprotected access to "copy_buffer".
     * But that's OK: we assume only a SINGLE
     * CONSUMER / READING process!
     */
    /*
    if(!copy_buffer){
	printk(KERN_INFO "Error: copy_buffer is NULL!\n");
	return -ERROR;
    }
    */

    /*
     * For now, require the input length to be
     * EXACTLY SEG_SIZE (for optimization)
     * This requirement will be removed in the future.
     */
    if(length != SEG_SIZE){
	printk(KERN_INFO "Error: requested length (%d) MUST be equal to %d bytes!\n", length, SEG_SIZE);
	return -ERROR;
    }

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
	    if(wait_event_interruptible(read_queue, ( (!IS_COLLECTING() && !IS_SLEEPING()) || any_seg_full()))) // returns -ERESTARTSYS if interrupted by signal, "0" on success
		return -ERESTARTSYS;
	    /*
	     * We've been woken up. This could be because of:
	     * (a) A FULL buffer condition or
	     * (b) DD received a STOP/PAUSE/CANCEL while reader was blocked.
	     * In case (a) we're *guaranteed* a FULL buffer ==> SEG_SIZE
	     * bytes below is valid. If (b) then we probably won't
	     * have a full buffer, but its still OK to set to SEG_SIZE
	     * here because the correct size will be set when the
	     * "find_next_non_empty_seg()" function is called (above).
	     */
	    size = SEG_SIZE;
	}
#else // ALLOW_BLOCKING_READ
	if(!(seg = find_seg()))
	    return 0; // nothing to read
#endif
    }

    /*
     * Now copy data to user.
     */
    OUTPUT(2, KERN_INFO "About to copy to user-space!\n");
    // memcpy(copy_buffer, (char *)seg->samples, size);
    if( (bytes_not_copied = copy_to_user(buffer, (char *)seg->samples, size))) // dst,src
	printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", bytes_not_copied);

    /*
     * Done copying -- let producer know
     * this segment is empty.
     */
    SAMPLE(seg, 0).sample_type = FREE_SAMPLE;
    atomic_set(&seg->is_full, EMPTY);

    /*
    if( (bytes_not_copied = copy_to_user(buffer, copy_buffer, size))) // dst,src
	printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", bytes_not_copied);
    */

    bytes_read = size - bytes_not_copied;

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
#define extract_local_args(l,u) copy_from_user((l), (u), sizeof(struct PWCollector_ioctl_arg))

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

    if((int)INTERNAL_STATE.residency_count_multiplier <= 0)
	INTERNAL_STATE.residency_count_multiplier = 1;

    if(true){
	OUTPUT(0, KERN_INFO "DEBUG: C-state clock multiplier = %u\n", INTERNAL_STATE.residency_count_multiplier);
    }

    /*
     * Set power switches.
     */
    INTERNAL_STATE.collection_switches = local_config.data;
    OUTPUT(0, KERN_INFO "\tCONFIG collection switches = %d\n", INTERNAL_STATE.collection_switches);

    /*
     *
     */

    return SUCCESS;
};

int check_platform(struct PWCollector_check_platform *remote_check, int size)
{
    struct PWCollector_check_platform *local_check;
    const char *unsupported = "UNSUPPORTED_T1, UNSUPPORTED_T2"; // for debugging ONLY
    int len = strlen(unsupported);
    int max_size = sizeof(struct PWCollector_check_platform);
    int retVal = SUCCESS;

    local_check = kmalloc(max_size, GFP_KERNEL);

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

    kfree(local_check);
    return retVal; // all unsupported tracepoints documented
};

/*
 * Retrieve device driver versino
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


int get_sample(struct PWCollector_non_precise_sample *remote_sample, int size)
{
    int i=0, retVal = SUCCESS;
    int cpu = CPU();
    unsigned char num_cpus_to_sample;
    struct PWCollector_non_precise_sample local_sample;
    c_sample_t *local_samples = NULL;
    int max_size = PW_max_num_cpus * sizeof(c_sample_t);
    int act_size = max_size;


    /*
     * Dynamically allocating memory is expensive.
     * However, we do NOT expect the user to sample more
     * than once a second, so we leave this as is, for now.
     * Later, we could perform a one-time per-cpu dynamic
     * init of this memory (say, within "init_data_structures()")
     */
    local_samples = kmalloc(max_size, GFP_KERNEL);
    if(!local_samples){
	printk(KERN_INFO "ERROR: Could NOT allocate memory for sample!\n");
	atomic_set(&should_panic, 1);
	return -ERROR;
    }
    memset(local_samples, 0, max_size);


    if( (i = copy_from_user(&local_sample, remote_sample, sizeof(local_sample)))){ // "copy_from_user" returns number of bytes that COULD NOT be copied
	retVal = i;
	goto done;
    }

    num_cpus_to_sample = local_sample.num_cpus_to_sample;
    act_size = num_cpus_to_sample * sizeof(c_sample_t);

    OUTPUT(0, KERN_INFO "# cpus to sample = %d\n", num_cpus_to_sample);
    OUTPUT(0, KERN_INFO "%p\n", remote_sample->samples);

    if(num_cpus_to_sample < PW_max_num_cpus){
	OUTPUT(0, KERN_INFO "Warning: user requests %d samples, which is less than the number of cpus (%d) in the system\n", num_cpus_to_sample, PW_max_num_cpus);
    }

    /*
     * FOR EACH CPU, read sample information.
     * Sampling consists of only reading MSR residency.
     */
    for_each_online_cpu(cpu){
	read_sample_residencies(cpu, &RES_COUNT(local_samples[cpu], 0));
    }

    /*
     * Now copy the buffer back to user-space.
     */
    if( (i = copy_to_user(remote_sample->samples, local_samples, act_size))) // returns number of bytes that COULD NOT be copied
	retVal = i;
    else
	retVal = SUCCESS;

    // Debugging
    OUTPUT(0, KERN_INFO "SAMPLE done!\n");

 done:
    kfree(local_samples);

    return retVal;
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
	// if(statusJIFF < startJIFF){
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

void reset_buffers(void)
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
			    PREV_MSR_VAL(pcpu, MPERF) = 0;
			    pcpu->last_pid = pcpu->last_tid = -1;
			    pcpu->last_break[IRQ] = pcpu->last_break[TIMER] =  pcpu->last_break[SCHED] = 0;
			    pcpu->debug_enters = 0;
			    pcpu->prev_state = 0;
			}
		}
	}
}

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

    switch(cmd){
    case START:
	for_each_online_cpu(cpu){
	    memset(&lists[cpu], 0, sizeof(list_t));
	    for(i=0; i<NUM_SEGS_PER_LIST; ++i){
		atomic_set(&lists[cpu].segs[i].is_full, EMPTY);
	    }
	}
	last_list_read = -1;
	last_flush_index = 0;
	/*
	 * Also allocate trace entry
	 * blocks (if not previously
	 * alloced).
	 */
#if FREE_TRACE_MAP_ON_COLLECTION_STOP
	{
	    OUTPUT(0, KERN_INFO "DEBUG: on Start collection, found trace_entry_blocks_head = %p, free_list_head = %p, ptnode_free_list_head = %p\n", trace_entry_blocks_head, free_list_head, ptnode_free_list_head);
	    if(!trace_entry_blocks_head){
		// if(init_trace_entry_map())
		if(init_tracing_subsystem())
		    return -ERROR;// ptnode_free_list_head
		OUTPUT(0, KERN_INFO "DEBUG: TRACE_BLOCKS ALLOCed on COLLECTION START!! trace_entry_blocks_head = %p, free_list_head = %p, ptnode_free_list_head = %p\n", trace_entry_blocks_head,
		       free_list_head, ptnode_free_list_head);
	    }
	}
#endif
    case RESUME: // fall through
	break;
    default: // should *NEVER* happen!
	printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
	return -ERROR;
    }
    INTERNAL_STATE.collectionStartJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = true;

    // Reset the (per-cpu) output buffers.
    {
	reset_buffers();
    }

#if DO_IOCTL_STATS
    reset_statistics();
#endif

    {
	register_all_trace_probes();
    }
    OUTPUT(0, KERN_INFO "\tREGISTERED all probes!\n");
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
    INTERNAL_STATE.collectionStopJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = false;
    {
	unregister_all_trace_probes();
    }

	// Reset the (per-cpu) output buffers.
	{
		reset_buffers();
	}

#if DO_IOCTL_STATS
    reset_statistics();
#endif

    /*
     * There might be a reader thread blocked on a read: wake
     * it up to give it a chance to respond to changed
     * conditions.
     */
    {
	wake_up_interruptible(&read_queue);
    }
    /*
     * If 'STOP' or 'CANCEL' ==> free up
     * trace entry blocks.
     * Note: there's a race here: it is THEORETICALLY
     * possible that a tracepoint is currently
     * being serviced when the "trace_entry_blocks_head"
     * pointer is freed. For now, do nothing.
     */
#if FREE_TRACE_MAP_ON_COLLECTION_STOP
    {
	if(cmd == STOP || cmd == CANCEL){
	    // destroy_trace_entry_map();
	    destroy_tracing_subsystem();
	    OUTPUT(0, KERN_INFO "DEBUG: TRACE_BLOCKS FREEd on COLLECTION STOP/CANCEL!\n");
	}
    }
#endif

    /*
     * Print out some debugging info.
     */
    {
	OUTPUT(0, KERN_INFO "\t# free trace entries = %llu, total # trace entries = %llu\n", num_free_trace_entries, total_num_trace_entries);
	OUTPUT(0, KERN_INFO "\t# free PTNODE entries = %llu, total # PTNODE entries = %llu\n", num_free_ptnode_entries, total_num_ptnode_entries);
    }
    OUTPUT(0, KERN_INFO "\tUNREGISTERED all probes!\n");
    return SUCCESS;
};

/*
 * Service IOCTL calls from user-space.
 */
int device_ioctl(struct inode *inode,	/* see include/linux/fs.h */
		 struct file *file,	/* ditto */
		 unsigned int ioctl_num,	/* number and param for ioctl */
		 unsigned long ioctl_param)
{
    int retVal = SUCCESS;
    PWCollector_cmd_t cmd, prev_cmd;
    struct PWCollector_ioctl_arg local_args, *user_args;

    /*
     * EVERY ioctl has the same param type ==> extract it here.
     */
    user_args = (struct PWCollector_ioctl_arg *)ioctl_param;
    if(extract_local_args(&local_args, user_args)){ // "copy_from_user" returns number of bytes that COULD NOT BE COPIED
	printk(KERN_ERR "Error: Could NOT copy all bytes in IOCTL!\n");
	return -ERROR;
    }
    /*
     * We've 'extracted' the PWCollector_config structure
     * But we still need to extract "in_arg" and "out_arg".
     * Unfortunately, those are command-dependent, so
     * do that in the "ioctl_num" switch, below.
     */

    /*
     * Switch according to the ioctl called
     */
    switch(ioctl_num){
    case PW_IOCTL_CONFIG:
	OUTPUT(0, KERN_INFO "PW_IOCTL_CONFIG\n");
	if( (retVal = set_config((struct PWCollector_config *)user_args->in_arg, local_args.in_len))){
	    printk(KERN_INFO "Error: could NOT copy %d bytes out of %d bytes in config!\n", retVal, sizeof(struct PWCollector_config));
	    return -ERROR;
	}
	retVal = SUCCESS;
	break;
    case PW_IOCTL_CMD:
	cmd = *((PWCollector_cmd_t *)local_args.in_arg);
	OUTPUT(0, KERN_INFO "PW_IOCTL_CMD: cmd=%d\n", cmd);
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
	retVal = SUCCESS;
	OUTPUT(3, KERN_INFO "Debug: Successfully switched mode from %d to %d: IS_COLLECTING = %d\n", prev_cmd, cmd, IS_COLLECTING());
	break;
	/*
	 * These IOCTLs are unsupported, for now.
	 */
    case PW_IOCTL_STATUS:
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
	if( (retVal = get_status((struct PWCollector_status *)user_args->out_arg, local_args.out_len))){
	    printk(KERN_INFO "\tError retrieving status information!\n");
	    retVal = -ERROR;
	}
#else
	return -ERROR;
#endif
	break;
    case PW_IOCTL_SAMPLE:
	OUTPUT(0, KERN_INFO "PW_IOCTL_SAMPLE\n");
	/*
	 * We can't return a "sample" if we're not actively collecting!
	 * Check this here.
	 */
	if(!IS_COLLECTING()){
	    printk(KERN_INFO "\tError: sample requested, but NO COLLECTION ONGOING!\n");
	    return -ERROR;
	}
	if(!IS_NON_PRECISE_MODE()){
	    printk(KERN_INFO "\tError: sample requested, but NOT IN NON-PRECISE MODE!\n");
	    return -ERROR;
	}
	if( (retVal = get_sample((struct PWCollector_non_precise_sample *)user_args->out_arg, local_args.out_len))){
	    printk(KERN_INFO "\tError retrieving sample information in non-precise mode!\n");
	    return -ERROR;
	}
	retVal = SUCCESS;
	break;
    case PW_IOCTL_CHECK_PLATFORM:
	OUTPUT(0, KERN_INFO "PW_IOCTL_CHECK_PLATFORM\n");
	if( (retVal = check_platform((struct PWCollector_check_platform *)user_args->out_arg, local_args.out_len)))
	    if(retVal < 0) // ERROR
		return 2; // for PW_IOCTL_CHECK_PLATFORM: >= 2 ==> Error; == 1 => SUCCESS, but not EOF; 0 ==> SUCCESS, EOF
	break;
    case PW_IOCTL_VERSION:
	OUTPUT(0, KERN_INFO "PW_IOCTL_VERSION\n");
	if( (retVal = get_version((struct PWCollector_version_info *)user_args->out_arg, local_args.out_len))){
	    printk(KERN_INFO "\tError retrieving version information!\n");
	    return -ERROR;
	}
	retVal = SUCCESS;
	break;
    default:
	printk(KERN_INFO "Invalid IOCTL command = %d\n", ioctl_num);
	return -ERROR;
    }
    return retVal;
};


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
    .ioctl = device_ioctl,
    .release = device_release,
};

int register_dev(void)
{
    /*
     * Register the character device (atleast try)
     */
    int ret = register_chrdev(MAJOR_NUM, DEVICE_FILE_NAME, &Fops);

    /*
     * Negative values signify an error
     */
    if (ret < 0) {
	printk(KERN_ALERT "%s failed with %d\n",
	       "Sorry, registering the character device ", ret);
    }else{

	printk(KERN_INFO "%s The major device number is %d.\n",
	       "Registeration is a success", MAJOR_NUM);
	printk(KERN_INFO "If you want to talk to the device driver,\n");
	printk(KERN_INFO "you'll have to create a device file. \n");
	printk(KERN_INFO "We suggest you use:\n");
	printk(KERN_INFO "mknod %s c %d 0\n", DEVICE_FILE_NAME, MAJOR_NUM);
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
    unregister_chrdev(MAJOR_NUM, DEVICE_FILE_NAME);
};


/*
 * Cleanup procedure: destroy all data structures initialized
 * in the "init_data_structures()" function.
 */
static void destroy_data_structures(void)
{
    /*
    if(copy_buffer)
	kfree(copy_buffer);
    */

    if(lists)
	kfree(lists);

    // destroy_trace_entry_map();
    destroy_tracing_subsystem();

};


/*
 * Initialize data structures here.
 */
static int init_data_structures(void)
{
    int i=0, j=0;

    /*
     * Find the # CPUs in this system.
     */
    PW_max_num_cpus = num_online_cpus();

    /*
     * Init the (per-cpu) output buffers.
     * We have two buffers (or "segments") for every "list"
     * and one "list" for every cpu.
     */
    lists = (list_t *)kmalloc(PW_max_num_cpus * sizeof(list_t), GFP_KERNEL);
    if(!lists){
	printk(KERN_INFO "ERROR: Could NOT allocate memory for lists!\n");
	atomic_set(&should_panic, 1);
	return -ERROR;
    }
    for_each_online_cpu(i){
	memset(&lists[i], 0, sizeof(list_t));
	for(j=0; j<NUM_SEGS_PER_LIST; ++j)
	    atomic_set(&lists[i].segs[j].is_full, EMPTY);
    }

    /*
    copy_buffer = (char *)kmalloc(SEG_SIZE, GFP_KERNEL);
    if(!copy_buffer){
	printk(KERN_INFO "ERROR: Could NOT allocate memory for copy buffer!\n");
	destroy_data_structures();
	return -ERROR;
    }
    */

    // if(init_trace_entry_map()){
    if(init_tracing_subsystem()){
	destroy_data_structures();
	return -ERROR;
    }

    /*
     * Init the state struct here.
     */
    memset(&INTERNAL_STATE, 0, sizeof(INTERNAL_STATE));

    /*
     * Init the overhead measurement vars here.
     */
#if DO_OVERHEAD_MEASUREMENTS
    {
	timer_init_init_overhead_params();
	timer_expire_init_overhead_params();
	tps_init_overhead_params();
    }
#endif // DO_OVERHEAD_MEASUREMENTS

    return SUCCESS;

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
	data_copy = (&per_cpu(CTRL_data_values, cpu))->data;
	ret = rdmsr_safe_on_cpu(cpu, 0x38d, &data[0], &data[1]);
	WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
	memcpy(data_copy, data, sizeof(u32) * 2);
	/*
	 * Turn on CPU_CLK_UNHALTED.REF counting.
	 */
	data[0] |= 0x300;
	/* data[0] = 0x0; */

	ret = wrmsr_safe_on_cpu(cpu, 0x38d, data[0], data[1]);
    }
    if(true)
	for_each_online_cpu(cpu){
	    ret = rdmsr_safe_on_cpu(cpu, 0x38d, &data[0], &data[1]);
	    res = data[1];
	    res <<= 32;
	    res += data[0];
	    OUTPUT(0, KERN_INFO "[%d]: NEW res = 0x%llx\n", cpu, res);
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
	data_copy = (&per_cpu(CTRL_data_values, cpu))->data;
	memcpy(data, data_copy, sizeof(u32) * 2);

	res = data[1];
	res <<= 32;
	res += data[0];

	OUTPUT(3, KERN_INFO "[%d]: PREV res = 0x%llx\n", cpu, res);
	ret = wrmsr_safe_on_cpu(cpu, 0x38d, data[0], data[1]);
    }
};


// static const int MDFcoreResidencyMSRAddresses[] = {0x30b, -1, 0x3f8, -1, 0x3f9, 0x121, 0x3fa, -1, -1, -1};
/*
 * Initialize the device driver.
 * For now, also insert tracepoints here. This
 * will probably change (e.g., wait to insert
 * tracepoints until we get a START command?).
 */
static int __init init_hooks(void)
{
    int i=0, ret;

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

#if DO_CHECK_BO_MICROCODE_PATCH
	{
		/*
		 * Read MSR 0x8b -- if microcode patch
		 * has been applied then the first 12 bits
		 * of the higher order 32 bits should be
		 * >= 0x102.
		 */
		u64 res;
		u32 patch_val;

		rdmsrl(0x8b, res);
		patch_val = (res >> 32) & 0xfff;
		if(patch_val < 0x102){
			printk(KERN_INFO "ERROR: B0 micro code path = 0x%x: REQUIRED >= 0x102!!!\n", patch_val);
		}
	}
#endif

    OUTPUT(0, KERN_INFO "pid_t size = %d\n", sizeof(pid_t));
    OUTPUT(0, KERN_INFO "per_cpu_t size = %d\n", sizeof(per_cpu_t));
    OUTPUT(0, KERN_INFO "c_sample_t size = %d\n", sizeof(c_sample_t));
    OUTPUT(0, KERN_INFO "k_sample_t size = %d\n", sizeof(k_sample_t));
    OUTPUT(0, KERN_INFO "PWCollector_sample size = %d\n", sizeof(struct PWCollector_sample));
    OUTPUT(0, KERN_INFO "platform_info_t size = %d\n", sizeof(platform_info_t));
    OUTPUT(0, KERN_INFO "config size = %d\n", sizeof(struct PWCollector_config));
    OUTPUT(0, KERN_INFO "Max MSR addresses = %d\n", MAX_MSR_ADDRESSES);
    OUTPUT(0, KERN_INFO "Lists size = %d\n", sizeof(list_t));
    OUTPUT(0, KERN_INFO "# Online Cpus = %d\n", num_online_cpus());


    startJIFF = jiffies;


#ifdef CONFIG_STACKTRACE
    OUTPUT(0, KERN_INFO "Stacktrace ON!\n");
#endif

#if FREE_TRACE_MAP_ON_COLLECTION_STOP
    OUTPUT(0, KERN_INFO "FREE_TRACE_MAP = ON!\n");
#endif

    /*
     * Init all locks here
     */
    {
	for(i=0; i<NUM_HASH_LOCKS; ++i){
	    rwlock_init(&hashLocks[i]);
	    rwlock_init(&pt_hashLocks[i]);
	}
    }


    {
	if(init_data_structures())
	    return -ERROR;
    }

    {
	enable_ref();
    }

    /*
     * Init config stuff here...
     */
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
	register_timer_callstack_probes();
    }

    /*
     * Insert all tracepoints.
     *
     * Eventually, we'll move to a model
     * where driver "init" will first register
     * the trace probes, and then immediately
     * unregister them. This will be done to, for
     * example, test which tracepoints are
     * truly supported by our device driver.
     * For now, do nothing (actual probe registration
     * takes place when we start a collection).
     */
#if 0
    {
	register_all_trace_probes();
    }
#endif

    // Debugging only
    {
	u64 res;
	rdmsrl(0x8b, res);

	OUTPUT(0, KERN_INFO "0x8b = 0x%llx\n", res);
    }


    printk(KERN_INFO "\n--------------------------------------------------------------------------------------------\n");
    printk(KERN_INFO "START Initialized the probes\n");
    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");

    return 0;
}

/*
 * Cleanup procedure: remove all previously inserted tracepoints.
 */
static void __exit cleanup_hooks(void)
{
    /* int i=0; */
    unsigned long elapsedJIFF = 0, collectJIFF = 0;

    {
	unregister_dev();
    }

    /*
     * Probes required to cache (kernel) timer
     * callstacks need to be removed, regardless
     * of collection status.
     */
    {
	unregister_timer_callstack_probes();
    }

    /*
     * Remove previously inserted tracepoints here.
     *
     * Unlike the driver "init" (where we don't really
     * register anything), we call "unregister" here
     * (but only if the Ring-3 component didn't send
     * us a STOP command).
     */
#if 1
    if(IS_COLLECTING()){
	unregister_all_trace_probes();
    }
#else
    /*
     * Forcibly unregister -- used in debugging.
     */
    {
	unregister_all_trace_probes();
    }
#endif

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
    printk(KERN_INFO "Total time elapsed = %u msecs; Total collection time = %u msecs\n", jiffies_to_msecs(elapsedJIFF), jiffies_to_msecs(collectJIFF));

    OUTPUT(0, KERN_INFO "\t# free trace entries = %llu, total # trace entries = %llu\n", num_free_trace_entries, total_num_trace_entries);
    OUTPUT(0, KERN_INFO "\t# free PTNODE entries = %llu, total # PTNODE entries = %llu\n", num_free_ptnode_entries, total_num_ptnode_entries);

#if DO_OVERHEAD_MEASUREMENTS
    {
	int num_timer_inits = 0, num_tps = 0, num_timer_expire = 0, num_tpf = 0, num_inter = 0, num_sched_switch = 0;
	u64 total_timer_init = 0, total_tps = 0, total_timer_expire = 0, total_tpf = 0, total_inter = 0, total_sched_switch = 0;

	timer_init_get_cumulative_overhead_params(&total_timer_init, &num_timer_inits);
	tps_get_cumulative_overhead_params(&total_tps, &num_tps);
	timer_expire_get_cumulative_overhead_params(&total_timer_expire, &num_timer_expire);
	tpf_get_cumulative_overhead_params(&total_tpf, &num_tpf);
	inter_common_get_cumulative_overhead_params(&total_inter, &num_inter);
	sched_switch_get_cumulative_overhead_params(&total_sched_switch, &num_sched_switch);

	printk(KERN_INFO "TIMER_INIT: %d iters took %llu cycles!\n", num_timer_inits, total_timer_init);
	printk(KERN_INFO "TPS: %d iters took %llu cycles!\n", num_tps, total_tps);
	printk(KERN_INFO "TIMER_EXPIRE: %d iters took %llu cycles!\n", num_timer_expire, total_timer_expire);
	printk(KERN_INFO "TPF: %d iters took %llu cycles!\n", num_tpf, total_tpf);
	printk(KERN_INFO "INTER_COMMON: %d iters took %llu cycles!\n", num_inter, total_inter);
	printk(KERN_INFO "SCHED_SWITCH: %d iters took %llu cycles!\n", num_sched_switch, total_sched_switch);
    }
#endif // DO_OVERHEAD_MEASUREMENTS

    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");
}

module_init(init_hooks);
module_exit(cleanup_hooks);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
