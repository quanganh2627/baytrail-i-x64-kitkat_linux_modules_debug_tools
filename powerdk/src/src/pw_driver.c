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
 */
#define PW_VERSION_VERSION 0
#define PW_VERSION_INTERFACE 0
#define PW_VERSION_OTHER 10

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
 * For debugging: count times where C-state
 * residencies were INVALID or ZERO.
 */
atomic_t num_c_state_breaks, num_invalid_residencies, num_zero_residencies;

/* Debugging: Records when we hit IDLE_END_NOTIFY, IRQ_ENTER or a TRACEPOINT_BREAK
 *
 * This allows us to see how many C-State wake-ups we actually catch and which ones we miss
 * We also get a better idea of the latencies involved as these are TSC stamped
 */
enum {
    IDLE_END_NOTIFY,
    IRQ_ENTER,
    TRACEPOINT_BREAK,
    MAX_DEBUG
};


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
#define DEV_IS_OPEN 0// see if device is in use
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
    atomic_t head, tail;
    atomic_t is_full;
}seg_t;

#define HEAD(s) ( atomic_read(&(s)->head ) )
#define TAIL(s) ( atomic_read(&(s)->tail ) )
#define IS_SEG_EMPTY(s) ( HEAD(s) == TAIL(s) )
#define IS_SEG_FULL(s) ( HEAD(s) == ((TAIL(s) + 1) & SAMPLE_MASK) )

/*
 * Per-cpu structure (or "list") of output buffers.
 * Each list has "NUM_SEGS_PER_LIST" (== 2) buffers.
 */
typedef struct{
    seg_t segs[NUM_SEGS_PER_LIST];
    int index;
    int flush_index;
}list_t;

/*
 * How many times should we try to insert into
 * completely full lists before we decide
 * to unregister trace probes?
 */
#define LIST_FULL_THRESHOLD 10

#define SAMPLE(s,i) ( (s)->samples[(i)] )
#define C_SAMPLE(s,i) ( (s)->samples[(i)].c_sample )
#define P_SAMPLE(s,i) ( (s)->samples[(i)].p_sample )
#define K_SAMPLE(s,i) ( (s)->samples[(i)].k_sample )

static list_t *lists = NULL;


#define CIRCULAR_INC(i,l) ({int __tmp1 = (++(i)) & (l); __tmp1;})

/*
 * For now, we limit kernel-space backtraces to 20 entries.
 * This decision will be re-evaluated in the future.
 */
#define MAX_BACKTRACE_LENGTH 20

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
#define NUM_TRACE_HASH_BITS (NUM_TRACE_ENTRY_BITS - 4)
/* #define NUM_TRACE_HASH_BITS (NUM_TRACE_ENTRY_BITS - 2) */
#define NUM_TRACE_HASH_ENTRIES (1 << NUM_TRACE_HASH_BITS)
#define TRACE_HASH_MASK (NUM_TRACE_HASH_ENTRIES - 1)
#define hash_func_entries(p) ( ((unsigned long)(p) & TRACE_HASH_MASK) )

static hash_entry_t hash_table[NUM_TRACE_HASH_ENTRIES];

static trace_entry_t *free_list_head = NULL;

// 32 locks for the hash table
// #define HASH_LOCK_BITS 5
// 4 locks for the hash table
#define HASH_LOCK_BITS 2
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

#define TRACE_BLOCK_SIZE_REALLOC_THRESHOLD 20


#define ALLOW_BLOCKING_READ 1

#if ALLOW_BLOCKING_READ
wait_queue_head_t read_queue;
#endif


/*
 * Temporary buffer to hold data. Used within
 * the device "read()" function.
 */
static char *copy_buffer = NULL;


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
#define IS_NON_PRECISE_MODE() (INTERNAL_STATE.collection_switches & POWER_SYSTEM_MASK)
#define IS_KTIMER_MODE() (INTERNAL_STATE.collection_switches & POWER_KTIMER_MASK)


/*
 * Some utility macros...
 */

/* Macro to determine if the user-stack can be obtained */
#define user_stack_valid(regs) (user_mode(regs) && !in_interrupt() && !in_softirq())

/* Macro for printk based on verbosity */
#define OUTPUT(level, ...) do { if(unlikely(level <= verbosity)) printk(__VA_ARGS__); } while(0);

/* Macro to determine if we dump the kernel stack */
#define DUMP_STACK do { if(unlikely(verbosity >= 0)) dump_stack(); } while(0);

#define DUMP_HRTIMER(t, m) OUTPUT(0, KERN_INFO "(%s) Timer [%p]: start-site=%p, function=%p, start-pid=%d\n", m, t, t->start_site, t->function, t->start_pid);

#define CPU() (smp_processor_id())
#define PID() (current->pid)
#define TGID() (current->tgid)
#define NAME() (current->comm)
#define PKG(c) ( cpu_data(c).phys_proc_id )

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
    unsigned long long tsc; // 8 bytes
    unsigned long long residencies[MAX_MSR_ADDRESSES]; // 96 bytes
    unsigned long long prev_msr_vals[MAX_MSR_ADDRESSES]; // 96 bytes
    unsigned long long debug_enters[3]; // 24 bytes
    unsigned long long overheadCount; // 8 bytes
    unsigned long long last_break[2]; // 16 bytes
    unsigned char is_deferrable; // 1 byte
    unsigned char padding[24]; // 24 bytes
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

/* Helper function to get TSC */
static inline void tscval(unsigned long long *v)
{
    unsigned int aux;
    rdtscpll(*v, aux);
}

enum{
    IRQ=0,
    TIMER
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
	if(IS_SEG_FULL(seg)){
	    // if(atomic_read(&seg->is_full) == FULL){
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

    if(false){
	return;
    }

    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = TAIL(seg); // insert at TAIL
    {
	SAMPLE(seg, seg_index).sample_type = C_STATE;
	// SAMPLE(seg, seg_index).cpu = cpu;
	SAMPLE(seg, seg_index).cpuidx = cpu;
	SAMPLE(seg, seg_index).tsc = pcpu->tsc;
	/* rdtscll(SAMPLE(seg, seg_index).tsc); */
	// memcpy(&RES_COUNT(C_SAMPLE(seg, seg_index),0), &pcpu->residencies, sizeof(int) * MAX_MSR_ADDRESSES); // dst, src
	memcpy(&RES_COUNT(C_SAMPLE(seg, seg_index),0), pcpu->residencies, sizeof(u64) * MAX_MSR_ADDRESSES); // dst, src
	C_SAMPLE(seg, seg_index).pid = last_PID;
	C_SAMPLE(seg, seg_index).tid = last_TID;
	C_SAMPLE(seg, seg_index).break_type = sample_type;
	C_SAMPLE(seg, seg_index).c_data = sample_data;

	C_SAMPLE(seg, seg_index).prev_state = pcpu->prev_state;

	if(false){
	    OUTPUT(0, KERN_INFO "C-DUMP: %d\t%16llX\t%16llX\t%16llX\t%16llX\n", cpu, pcpu->tsc, RES_COUNT(C_SAMPLE(seg, seg_index), MPERF), RES_COUNT(C_SAMPLE(seg, seg_index), C3), RES_COUNT(C_SAMPLE(seg, seg_index), C6));
	}
    }
    if(false){
	OUTPUT(0, KERN_INFO "C-DUMP: %d\t%16llX\t%16llX\t%16llX\t%16llX\n", cpu, pcpu->tsc, RESIDENCY(pcpu, MPERF), RESIDENCY(pcpu, C3), RESIDENCY(pcpu, C6));
    }
    /*
     * OK, we've written to the segment.
     * Now increment indices, check for
     * full conditions etc.
     */
    atomic_set(&seg->tail, CIRCULAR_INC(seg_index, SAMPLE_MASK));
    if(IS_SEG_FULL(seg)){
	atomic_set(&seg->is_full, FULL);
	OUTPUT(2, KERN_INFO "[%d]: (C-STATE) seg FULL\n", cpu);
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

    if(false){
	return;
    }

    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = TAIL(seg); // insert at TAIL
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
    atomic_set(&seg->tail, CIRCULAR_INC(seg_index, SAMPLE_MASK));
    if(IS_SEG_FULL(seg)){
	atomic_set(&seg->is_full, FULL);
	OUTPUT(2, KERN_INFO "[%d]: (P-STATE) seg FULL\n", cpu);
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

    if(false){
	return;
    }

    OUTPUT(1, KERN_INFO "KERNEL-SPACE mapping!\n");

    if(!(seg = find_producer_seg(cpu))){
	printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
	return;
    }

    seg_index = TAIL(seg); // insert at TAIL
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
    atomic_set(&seg->tail, CIRCULAR_INC(seg_index, SAMPLE_MASK));
    if(IS_SEG_FULL(seg)){
	atomic_set(&seg->is_full, FULL);
	OUTPUT(2, KERN_INFO "[%d]: (K-STATE) seg FULL\n", cpu);
#if ALLOW_BLOCKING_READ
	wake_up_interruptible(&read_queue);
#endif
    }
};

/*
 * Generate a kernel-space call stack.
 * Follow the frame pointer back along
 * the call stack to get return addresses.
 *
 * THIS HAS BEEN DEPRECATED -- use the "__get_kernel_timerstack(...)"
 * function instead!!!
 */
int __get_kernel_timerstack_frame_pointer(unsigned long bp, unsigned long buffer[], int len)
{
    int index = 0;
    unsigned long *frame = (unsigned long *)bp;
    unsigned long addr = 0;
    static int warn_counter=-1;

    if(!(++warn_counter)) // warn only once per driver load
	printk(KERN_ERR "Warning: using deprecated function: \"__get_kernel_timerstack_frame_pointer(...)\"!\n");

    while(index < len && ((unsigned long)frame & (THREAD_SIZE - 1))){
	addr = *(frame+1);
	buffer[index++] = addr;
	frame = (unsigned long *)(*frame);
    }

    return index;
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
#if 0
    unsigned long bp = 0;

    if(INTERNAL_STATE.have_kernel_frame_pointers){
	if(!bp)
	    get_bp(bp);

	return __get_kernel_timerstack_frame_pointer(bp, buffer, len);
    }else{
	return __get_kernel_timerstack_crawl(buffer, len);
    }
#else
    return __get_kernel_timerstack(buffer, len);
#endif
};


/* Record the TSC when we hit a certain point (only the first time) */
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

#define alloc_trace_entry(t) {						\
	int idx = -1;							\
	LOCK(trace_entry_lock);						\
	idx = trace_entry_index = CIRCULAR_INC(trace_entry_index, TRACE_ENTRIES_MASK); \
	UNLOCK(trace_entry_lock);					\
	(t) = trace_entries + idx;					\
    }

static trace_entry_t *find_trace_i(unsigned long addr, int *index)
{
    int idx = hash_func_entries(addr);
    trace_entry_t **hash = NULL;
    trace_entry_t *curr = NULL;

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
static inline trace_entry_t *find_trace_entry(unsigned long timer_addr)
{
    int idx = -1;
    return find_trace_i(timer_addr, &idx);
};

// For debugging
u64 num_free_trace_entries = 0, total_num_trace_entries = 0;

static void init_trace_entries(trace_entry_t *entries)
{
    int i=0;
    memset(entries, 0, MAX_NUM_TRACE_ENTRIES);

    for(i=1; i<MAX_NUM_TRACE_ENTRIES; ++i)
	entries[i-1].next = &entries[i];
};

static trace_block_t *alloc_new_trace_block(void)
{
    trace_block_t *trace_block = (trace_block_t *)kmalloc(sizeof(trace_block_t), GFP_KERNEL);

    if(!trace_block){
	printk(KERN_INFO "ERROR: could NOT allocate a new trace block!\n");
	return NULL;
    }

    trace_block->block = (trace_entry_t *)kmalloc(sizeof(trace_entry_t) * MAX_NUM_TRACE_ENTRIES, GFP_KERNEL);
    if(!trace_block->block){
	printk(KERN_INFO "ERROR: could NOT allocate a new trace entries array!\n");
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
	    OUTPUT(0, KERN_INFO "# free entries = %llu, THRESHOLD = %d, ALLOCATING A NEW BLOCK!\n", num_free_trace_entries, TRACE_BLOCK_SIZE_REALLOC_THRESHOLD);
	    block = alloc_new_trace_block();
	    block->block[MAX_NUM_TRACE_ENTRIES-1].next = free_list_head;
	    free_list_head = block->block;
	    num_free_trace_entries += MAX_NUM_TRACE_ENTRIES;
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
    trace_entry_t *curr = find_trace_i(timer_addr, &idx);
    int retVal = SUCCESS;

    HASH_WRITE_LOCK(idx);
    {
	trace_entry_t **hash = hash_table+idx;
	if(!curr){
	    curr = get_next_free_entry_i();
	    curr->next = *hash;
	    *hash = curr;
	}
	if(curr){
	    curr->tid = tid; curr->tgid = tgid; curr->timer_addr = timer_addr; curr->tsc = tsc;
	    curr->trace_len = trace_len;
	    memcpy(curr->backtrace, trace, sizeof(unsigned long) * trace_len); // dst, src
	}else{
	    printk(KERN_INFO "ERROR: NO FREE ENTRY exists in insert!\n");
	    retVal = -ERROR;
	}
    }
    HASH_WRITE_UNLOCK(idx);
    /*
      if(curr){
      curr->tid = tid; curr->tgid = tgid; curr->timer_addr = timer_addr; curr->tsc = tsc;
      curr->trace_len = trace_len;
      memcpy(curr->backtrace, trace, sizeof(unsigned long) * trace_len); // dst, src
      }else{
      printk(KERN_INFO "ERROR: NO FREE ENTRY exists in insert!\n");
      retVal = -ERROR;
      }
    */

    return retVal;
};

static inline int remove_trace_entry(unsigned long timer_addr)
{
    int idx = -1;
    trace_entry_t *curr = find_trace_i(timer_addr, &idx), *prev = NULL;
    trace_entry_t **hash = NULL;

    if(!curr){
	printk(KERN_INFO "ERROR: could NOT find addr=0x%lx in remove_trace!\n", timer_addr);
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
	add_to_free_list_i(curr);
    }
    HASH_WRITE_UNLOCK(idx);

    return SUCCESS;
};

/*
 * Find the entry corresponding to "timer_addr".
 * Uses a hash-based mechanism for efficiency.
 */
#if 0
static inline trace_entry_t *find_trace_entry(unsigned long timer_addr, bool insert_if_absent)
{
    unsigned long idx = hash_func_entries(timer_addr);
    trace_entry_t **hash = hash_entries + idx;
    trace_entry_t *curr = *hash;

    while(curr && curr->timer_addr != timer_addr) curr = curr->next;

    if(!curr && insert_if_absent){
	alloc_trace_entry(curr);
	HASH_LOCK(idx);
	{
	    curr->next = *hash;
	    *hash = curr;
	}
	HASH_UNLOCK(idx);
    }
    return curr;
};

/*
 * Insert a new timer into the (hash-based) list of timer::TSC mappings.
 */
static inline void insert_trace_entry(pid_t tid, pid_t tgid, unsigned long timer_addr, unsigned long long tsc, int trace_len, unsigned long trace[])
{
    trace_entry_t *tentry = find_trace_entry(timer_addr, true); // "true" => insert entry if not present

    if(!tid)
	OUTPUT(2, KERN_INFO "KERNEL-SPACE insert entry! Addr = 0x%lx\n", timer_addr);

    tentry->tid = tid; tentry->tgid = tgid; tentry->timer_addr = timer_addr; tentry->tsc = tsc;
    tentry->trace_len = trace_len;
    memcpy(tentry->backtrace, trace, sizeof(unsigned long) * trace_len); // dst, src

    return;
};
#endif


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
    if(false && !pid){
	trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	OUTPUT(3, KERN_INFO "trace_len = %d\n", trace_len);
	DUMP_STACK;
    }
    //if(!pid && (INTERNAL_STATE.collection_switches & POWER_KTIMER_MASK)){
    if(!pid && IS_KTIMER_MODE()){
	/*
	 * get kernel timerstack here.
	 * Requires the kernel be compiled with
	 * frame_pointers on.
	 */
	trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	OUTPUT(2, KERN_INFO "KERNEL-SPACE timer INIT! Addr = %p\n", timer);
	// DUMP_STACK;
    }
    insert_trace_entry(pid, tgid, (unsigned long)timer, tsc, trace_len, trace);
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
    OUTPUT(3, KERN_INFO "[%d]: HRTIMER_INIT: Timer %p initialized\n", PID(), timer);
    if(false && !strcmp(NAME(), "a.out")){
	OUTPUT(0, KERN_INFO "[%d,%d]: %.20s has an HRTIMER INIT! expires time = %llu nsecs\n", PID(), TGID(), NAME(), ktime_to_ns(timer->_expires));
    }
    timer_init(timer);
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
    OUTPUT(2, KERN_INFO "[%d]:TIMER_INIT: Timer %p initialized\n", PID(), timer);
    if(false && !PID())
	DUMP_STACK;
    timer_init(timer);
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

    if(true){
	OUTPUT(3, KERN_INFO "[%d]: %.20s ITIMER STATE: which=%d, value=%p, expires=%lu (TIMER=%p)\n", PID(), NAME(), which, value, expires, timer);
    }

    timer_init(timer);
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
    stats_t *pstats = NULL;
    const trace_entry_t *tentry = NULL;
    pid_t tgid = 0;
    unsigned long long tsc = 0x1; // We need AT LEAST a non-zero TSC value to tell "power_start" that break was caused by a timer.
    bool tentry_found = false;

    if(!in_interrupt()){
	printk(KERN_ERR "BUG: timer_expire called from a NON-INTERRUPT context!\n");
	return;
    }

    /*
     * Atomic context => use __get_cpu_var(...) instead of get_cpu_var(...)
     */
    pcpu = &__get_cpu_var(per_cpu_counts);

    pstats = &__get_cpu_var(per_cpu_stats);

    cpu = CPU();

    if(!timer_addr){ // called from "itimer_expire_entry"
	record_hit(pcpu, TRACEPOINT_BREAK);
	return;
    }


    // tentry = find_trace_entry((unsigned long)timer_addr, false); // "false" => do NOT insert into entry list if not already present
    if(!(tentry = find_trace_entry((unsigned long)timer_addr))){
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

    record_hit_full(pcpu, TRACEPOINT_BREAK, tgid, pid, TIMER, tsc, is_deferrable);


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

    if(tentry_found){
	/*
	 * For now, remove tentry.
	 * Later, we'll want to move to a more sophisticated model: one that
	 * incorporates some degree of locality, reuse distance etc.
	 */
	if(remove_trace_entry((unsigned long)timer_addr))
	    OUTPUT(0, KERN_INFO "ERROR: could NOT remove timer entry for timer %p!\n", timer_addr);
    }
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
    timer_expire(hrt, pid, false); // "hrtimers" are *NEVER* deferrable! (CHECK THIS!!!)
};

#define DEFERRABLE_FLAG (0x1)
#define IS_TIMER_DEFERRABLE(t) ( (unsigned long)( (t)->base) & DEFERRABLE_FLAG )

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
    OUTPUT(2, KERN_INFO "[%d]: TIMER_EXPIRE: Timer %p expired\n", pid, t);
    timer_expire(t, pid, IS_TIMER_DEFERRABLE(t));
};

/*
 * Interval timer expire probe.
 */
#if (KERNEL_VER < 35)
static void probe_itimer_expire(int which, struct pid *pid, cputime_t now)
#else
    static void probe_itimer_expire(void *ignore, int which, struct pid *pid, cputime_t now)
#endif
{
    OUTPUT(2, KERN_INFO "ITIMER_EXPIRE\n");
    if(false)
	timer_expire(NULL, 0, false);
};

/*
 * High resolution timer (hrtimer) cancel probe.
 * Present for debugging only.
 */
#if (KERNEL_VER < 35)
static void probe_hrtimer_cancel(struct hrtimer *hrt)
#else
    static void probe_hrtimer_cancel(void *ignore, struct hrtimer *hrt)
#endif
{
    pid_t pid = hrt->start_pid;
    OUTPUT(2, KERN_INFO "[%d]: HRTIMER_CANCEL! Timer %p canceled\n", pid, hrt);
};

/*
 * Timer cancel probe.
 * Present for debugging only.
 */
#if (KERNEL_VER < 35)
static void probe_timer_cancel(struct timer_list *t)
#else
    static void probe_timer_cancel(void *ignore, struct timer_list *t)
#endif
{
    pid_t pid = t->start_pid;
    OUTPUT(2, KERN_INFO "[%d]: TIMER_CANCEL: Timer %p canceled\n", pid, t);
};

/*
 * Function common to all interrupt tracepoints.
 */
static void inter_common(int irq_num)
{
    /* Note that this is *always* called from within an interrupt
     * context
     */
    int cpu;
    per_cpu_t *pcpu = NULL;
    stats_t *pstats = NULL;
    unsigned char is_deferrable = 0;

    if(!in_interrupt()){
	printk(KERN_ERR "BUG: inter_common() called from a NON-INTERRUPT context! Got irq: %lu and soft: %lu\n", in_irq(), in_softirq());
	return;
    }

    pcpu = &__get_cpu_var(per_cpu_counts);

    pstats = &__get_cpu_var(per_cpu_stats);

    local_inc(&pstats->num_inters);

    /*
     * Increment counter for timer interrupts as well.
     */
    if(in_softirq() && (irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ))
	local_inc(&pstats->num_timers);

    cpu = CPU();

    /*
     * Record the TSC values, *ONLY* if this is the *FIRST*
     * interrupt *AFTER* the last TPS.
     * (Record hit internally checks this condition)
     *
     * Note that this version does *NOT* defer timer interrupts
     * to the "expire" probes.
     */
    record_hit_full(pcpu, TRACEPOINT_BREAK, TGID(), PID(), IRQ, irq_num, is_deferrable); // PID() will only be an approximation in this case!

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
    inter_common(irq);
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
    inter_common((int)(h-vec));
};

/*
 * Do we compute MSR residencies (by taking
 * deltas), or do we just return raw values?
 */
#define COMPUTE_MSR_RESIDENCIES 1

/*
 * Read MSR residency values.
 * Ultimately, MSR addresses will be obtained from the
 * PW_IOCTL_CONFIG ioctl.
 * For now, we use hardcoded NHM values.
 */
static inline int read_residency(int cpu, per_cpu_t *pcpu, unsigned int state)
{
    unsigned long long res;
    int msr_addr = -1;
    int i=0;

    int retVal = SUCCESS;

#if COMPUTE_MSR_RESIDENCIES
    u64 total_c_states = 0, non_c0_states = 0, prev_tsc = pcpu->tsc, tsc_delta = 0;
    unsigned long long prev_msr_vals[MAX_MSR_ADDRESSES]; // 96 bytes

    // In case we need to rollback:
    memcpy(prev_msr_vals, pcpu->prev_msr_vals, sizeof(unsigned long long) * MAX_MSR_ADDRESSES); // dst, src
#endif

    /*
     * Newer methodology: take MSR addresses from (user-input) values.
     */
    for(i=0; i<MAX_MSR_ADDRESSES; ++i){
	if( (msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[i]) <= 0)
	    continue;

	/*
	 * Read the MSR.
	 */
	rdmsrl(msr_addr, res);
	/*
	{
	    u32 h, l;
	    rdmsr_on_cpu(cpu, msr_addr, &l, &h);
	    res = h;
	    res <<= 32;
	    res += l;
	}
	 */
	/*
	 * Now compute the actual residency: subtract
	 * the previous MSR value from the current one.
	 */
#if COMPUTE_MSR_RESIDENCIES
	if(unlikely(res < PREV_MSR_VAL(pcpu, i))){
	    /*
	     * W've had a rollover!
	     */
	    PREV_MSR_VAL(pcpu, i) = (unsigned long long)(~0) - PREV_MSR_VAL(pcpu, i);
	    /* PREV_RES_COUNT(resCount, i) = (unsigned long long)(~0) - PREV_RES_COUNT(resCount, i); */
	    res += PREV_MSR_VAL(pcpu, i) + 1;
	    /* res += PREV_RES_COUNT(resCount, i) + 1; */
	    PREV_MSR_VAL(pcpu, i) = 0;
	    /* PREV_RES_COUNT(resCount, i) = 0; */

	    printk(KERN_INFO "Warning: ROLLOVER for C-State residency counter %u for CPU %u\n", i, cpu);
	}
	RESIDENCY(pcpu, i) = res - PREV_MSR_VAL(pcpu, i);
	PREV_MSR_VAL(pcpu, i) = res;

	if(i != APERF)
	    total_c_states += RESIDENCY(pcpu, i);

	if(i > APERF)
	    non_c0_states += RESIDENCY(pcpu, i);

#else
	/*
	 * Dump RAW values instead of residencies.
	 */
	RESIDENCY(pcpu, i) = res;
#endif // COMPUTE_MSR_RESIDENCIES
    }

    // TSC
    {
	tscval(&res);

	if(res < pcpu->tsc)
	    printk(KERN_INFO "Warning: ROLLOVER in TSC residency counts for cpu=%d!\n", cpu);

	pcpu->tsc = res;
    }

#if COMPUTE_MSR_RESIDENCIES
    {
	tsc_delta = pcpu->tsc - prev_tsc;

	if(unlikely(tsc_delta < total_c_states)){
	    OUTPUT(3, KERN_INFO "ERROR: TSC delta (%llu) < TOTAL C STATES (%llu)!\n", tsc_delta, total_c_states);
	    retVal = -ERROR;
	    /*
	     * We're going to discard this sample, so we must
	     * rollback to the previous state. The next
	     * time "read_residency" is called and MSR residencies
	     * taken, it should appear as if the values in the PREV_MSR_VAL(...)
	     * macro corresponded to the "prev_msr_vals" populated
	     * above.
	     */
	    memcpy(pcpu->prev_msr_vals, prev_msr_vals, sizeof(unsigned long long) * MAX_MSR_ADDRESSES); // dst, src

	    /*
	     * We also need to rollback the TSC...
	     */
	    pcpu->tsc = prev_tsc;

	    // For debugging
	    atomic_inc(&num_invalid_residencies);
	}else{
	    /*
	     * OK, this is a valid sample: set appropriate
	     * C1 value.
	     */
	    RESIDENCY(pcpu, APERF) = tsc_delta - total_c_states;
	    /*
	     * Debugging: check to see if we had ZERO residencies
	     * across the board.
	     */
	    // if(total_c_states == 0 || (RESIDENCY(pcpu, MPERF) == total_c_states))
	    if(non_c0_states == 0)
		atomic_inc(&num_zero_residencies);
	}
    }
#endif

    return retVal;
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
    int i=0;
    per_cpu_t *pcpu = NULL;
    stats_t *pstats = NULL;
    int cpu = -1, pkg=-1;
    int last_PID = -1, last_TID = -1;
    unsigned long long last_IRQ = 0, last_TIMER = 0;
    unsigned char is_deferrable = 0;
    char sample_type = 'I';
    unsigned long long sample_data = 0;
    int should_add = true;

    if(false){
	DUMP_STACK;
    }

    atomic_inc(&num_c_state_breaks);

    /*
     * "get_cpu_var(...)" internally calls "preempt_disable()"
     * See <include/linux/percpu.h>
     */
    pcpu = &get_cpu_var(per_cpu_counts);
    /*
     * "get_cpu_var" already disabled inters for us.
     * Use "__get_cpu_var" versions now.
     */
    pstats = &__get_cpu_var(per_cpu_stats);
    {
	local_inc(&pstats->c_breaks);
    }

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
    if(read_residency(cpu, pcpu, state))
	should_add = false;

    /*
     * Reset all "debug_enters" counters.
     * This signals the next timer/interrupt (i.e. C-state break) event
     * to collect information.
     */
    for(i=0; i<MAX_DEBUG; ++i)
	pcpu->debug_enters[i] = 0;

    last_PID = pcpu->last_pid;
    last_TID = pcpu->last_tid;
    last_IRQ = pcpu->last_break[IRQ];
    last_TIMER = pcpu->last_break[TIMER];
    is_deferrable = pcpu->is_deferrable;

    // put_cpu_var(*pcpu);
    put_cpu_var(pcpu[cpu]);

    if( (sample_data = last_IRQ) > 0){
	/*
	 * C-state break was caused by an interrupt.
	 */
	sample_type = 'I';
	local_inc(&pstats->inters_c_breaks);
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
	if(is_deferrable)
	    sample_type = 'D';
	else
	    sample_type = 'N';
	local_inc(&pstats->timer_c_breaks);
    }else{
	/*
	 * Unknown reason for C-state break.
	 */
	sample_type = 'U';
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
    pcpu->last_break[IRQ] = pcpu->last_break[TIMER] = 0;
    pcpu->is_deferrable = 0;

    pcpu->prev_state = state;
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
    int cpu = CPU();
    unsigned long long res = 0;
    stats_t *pstats = NULL;

    OUTPUT(3, KERN_INFO "probe_power_frequency, cpu = %d, type = %u, state = %u\n", cpu, type, state);

    /*
     * Read TSC value
     */
    rdtscll(res);

    if(INTERNAL_STATE.write_to_buffers){
	produce_p_sample(cpu, res, state);
    }else{
	P_DUMP_FULL(pkg, cpu, res, state);
    }
    /*
     */

    {
	pstats = &get_cpu_var(per_cpu_stats);
	local_inc(&pstats->p_trans);
	put_cpu_var(pstats);
    }
};

/*
 * Register all tracepoints.
 */
#if (KERNEL_VER < 35)
static int register_all_trace_probes(void)
{
    int ret = 0;

    {
	printk(KERN_INFO "\tTIMER_INIT_EVENTS");
	ret = register_trace_hrtimer_init(probe_hrtimer_init);
	WARN_ON(ret);
	ret = register_trace_timer_init(probe_timer_init);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tITIMER_STATE_EVENTS");
	ret = register_trace_itimer_state(probe_itimer_state);
	WARN_ON(ret);
    }

    {
      printk(KERN_INFO "\tTRACE_BREAK_EVENTS");
      ret = register_trace_timer_expire_entry(probe_timer_expire_entry);
      WARN_ON(ret);
      ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
      WARN_ON(ret);
      ret = register_trace_itimer_expire(probe_itimer_expire);
      WARN_ON(ret);
	ret = register_trace_irq_handler_entry(probe_irq_handler_entry);
	WARN_ON(ret);
	ret = register_trace_softirq_entry(probe_softirq_entry);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tTIMER_CANCEL_EVENTS");
	ret = register_trace_hrtimer_cancel(probe_hrtimer_cancel);
	WARN_ON(ret);
	ret = register_trace_timer_cancel(probe_timer_cancel);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tCSTATE_EVENTS");
	ret = register_trace_power_start(probe_power_start);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tPSTATE_EVENTS\n");
	ret = register_trace_power_frequency(probe_power_frequency);
	WARN_ON(ret);
    }
    return SUCCESS;
};
#else
static int register_all_trace_probes(void)
{
    int ret = 0;

    {
	printk(KERN_INFO "\tTIMER_INIT_EVENTS");
	ret = register_trace_hrtimer_init(probe_hrtimer_init, NULL);
	WARN_ON(ret);
	ret = register_trace_timer_init(probe_timer_init, NULL);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tITIMER_STATE_EVENTS");
	ret = register_trace_itimer_state(probe_itimer_state, NULL);
	WARN_ON(ret);
    }

    {
      printk(KERN_INFO "\tTRACE_BREAK_EVENTS");
      ret = register_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
      WARN_ON(ret);
      ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
      WARN_ON(ret);
      ret = register_trace_itimer_expire(probe_itimer_expire, NULL);
      WARN_ON(ret);
	ret = register_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
	WARN_ON(ret);
	ret = register_trace_softirq_entry(probe_softirq_entry, NULL);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tTIMER_CANCEL_EVENTS");
	ret = register_trace_hrtimer_cancel(probe_hrtimer_cancel, NULL);
	WARN_ON(ret);
	ret = register_trace_timer_cancel(probe_timer_cancel, NULL);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tCSTATE_EVENTS");
	ret = register_trace_power_start(probe_power_start, NULL);
	WARN_ON(ret);
    }

    {
	printk(KERN_INFO "\tPSTATE_EVENTS\n");
	ret = register_trace_power_frequency(probe_power_frequency, NULL);
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
	unregister_trace_hrtimer_init(probe_hrtimer_init);
	unregister_trace_timer_init(probe_timer_init);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_itimer_state(probe_itimer_state);

	tracepoint_synchronize_unregister();
    }


    {
      unregister_trace_timer_expire_entry(probe_timer_expire_entry);
      unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
      unregister_trace_itimer_expire(probe_itimer_expire);
	unregister_trace_irq_handler_entry(probe_irq_handler_entry);
	unregister_trace_softirq_entry(probe_softirq_entry);

      tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_hrtimer_cancel(probe_hrtimer_cancel);
	unregister_trace_timer_cancel(probe_timer_cancel);

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
    return;
};
#else
static void unregister_all_trace_probes(void)
{
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
      unregister_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
      unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
      unregister_trace_itimer_expire(probe_itimer_expire, NULL);
	unregister_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
	unregister_trace_softirq_entry(probe_softirq_entry, NULL);

      tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_hrtimer_cancel(probe_hrtimer_cancel, NULL);
	unregister_trace_timer_cancel(probe_timer_cancel, NULL);

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
	    // if(atomic_read(&seg->is_full) == FULL){
	    if(IS_SEG_FULL(seg)){
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
	    // if(atomic_read(&seg->is_full) == FULL){
	    if(IS_SEG_FULL(seg)){
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


#define CIRCULAR_DEC(i,m) ({int __tmp1 = (i); if(--__tmp1 < 0) __tmp1 = (m); __tmp1;})

/*
 * We have two strategies to read information:
 * (a) "memcpy" (in parallel) the output buffers of every CPU
 * onto temporary buffers and then serially copying these temp buffers
 * into user space and
 * (b) obtaining (in parallel) read/write indices for every CPU and
 * then serially copying data directly from the (per-cpu) O/P buffers
 * into user-space.
 *
 * Option (a) above is enabled by setting "DO_PARALLEL_COPY" to 1.
 * To enable option (b), set "DO_PARALLEL_COPY" to 0.
 */

#define DO_PARALLEL_COPY 1
#define DO_PARALLEL_READ_INDICES (1 - DO_PARALLEL_COPY)

#if DO_PARALLEL_COPY

typedef struct{
    PWCollector_sample_t samples[NUM_SEGS_PER_LIST * NUM_SAMPLES_PER_SEG]; // max possible size
    int size;
}per_cpu_read_t;

/*
 * One read struct for every cpu.
 */
static per_cpu_read_t *per_cpu_read_buffs = NULL;

#else

enum{
    CURR_SEG=0,
    OTHER_SEG
};

typedef struct{
    seg_t *segs[2];
    struct{
	int start_indices[2], stop_indices[2], lower_limits[2];
    };
    int size;
    int output_index;
}per_cpu_indices_t;

/*
 * One read struct for every cpu.
 */
static per_cpu_indices_t *per_cpu_read_indices = NULL;

#endif // DO_PARALLEL_COPY

#if DO_PARALLEL_COPY

int copy_from_seg_i(seg_t *seg, PWCollector_sample_t *ptr, int cpu, int head, int tail, u64 tscLimit)
{
    int i=CIRCULAR_DEC(tail, SAMPLE_MASK), size=0, retVal=0;
    int lower_limit = CIRCULAR_DEC(head, SAMPLE_MASK);
    /* int i = tail, size = 0, retVal = 0; */

    OUTPUT(2, KERN_INFO "h=%d, t=%d, i=%d, ll=%d\n", head, tail, i, lower_limit);

    if(head != tail){ // if seg is NOT empty
	// while(i != head && SAMPLE(seg, i).tsc > tscLimit) i = CIRCULAR_DEC(i, SAMPLE_MASK);
	while(i != lower_limit && SAMPLE(seg, i).tsc > tscLimit) i = CIRCULAR_DEC(i, SAMPLE_MASK);

	if(i == lower_limit){ // Nothing to read here
	    OUTPUT(2, KERN_INFO "\t\t[%d]: i = %d ll = %d\n", cpu, i, lower_limit);
	    return 0;
	}

	/* ************************************************
	 * TODO: SHOULD WE NOT ACTUALLY INCLUDE "HEAD"
	 * (what happens, e.g., if the "head" element
	 * is the only one LESS than the TSC limit)?
	 * ************************************************
	 */
	if(i == head){ // OOPS! Nothing to read here
	    OUTPUT(2, KERN_INFO "\t\t[%d]: i = %d, head = %d, tail = %d, TSC[i] = %llu, tscLimit = %llu\n", cpu, i, head, tail, SAMPLE(seg, i).tsc, tscLimit);
	    // return 0;
	}

	/*
	 * Need to copy elements from "head" to "i"
	 * But, it is possible that "i" < "head"!
	 */
	if(i < head){
	    // Go from "head" to END first...
	    int t1 = (NUM_SAMPLES_PER_SEG - head);
	    size = t1 * sizeof(PWCollector_sample_t);
	    memcpy(ptr, &seg->samples[head], size);
	    ptr += t1;
	    retVal += t1;
	    // ...and then go from BEGIN to i
	    size = (i+1) * sizeof(PWCollector_sample_t);
	    memcpy(ptr, seg->samples, size);
	    retVal += (i+1);
	}else{
	    // Go from "head" to "i"
	    int t1 = i - head + 1;
	    size = t1 * sizeof(PWCollector_sample_t);
	    memcpy(ptr, &seg->samples[head], size);
	    retVal += t1;
	}
	/*
	 * Set new "head": this indicates where we're going to read
	 * from next, so set this to "i+1" (+1 since we've already read upto,
	 * and including, i).
	 */
	// seg->head = CIRCULAR_INC(i, SAMPLE_MASK);
	atomic_set(&seg->head, CIRCULAR_INC(i, SAMPLE_MASK));
    }
    OUTPUT(2, KERN_INFO "\t[%d]: i = %d, tail = %d, head = %d, retVal = %d\n", cpu, i, tail, head, retVal);
    return retVal;
};

void copy_partial_seg(void *arg)
{
    u64 tscLimit = *((u64 *)arg);
    int cpu = CPU();
    list_t *list = &lists[cpu];
    int curr_index = list->index, other_index = 1 - curr_index;
    seg_t *curr_seg = &list->segs[curr_index]; // current seg
    seg_t *other_seg = &list->segs[other_index];
    int curr_tail = TAIL(curr_seg), other_tail = TAIL(other_seg);
    int curr_head = HEAD(curr_seg), other_head = HEAD(other_seg);
    int i = CIRCULAR_DEC(curr_tail, SAMPLE_MASK);
    per_cpu_read_t *pcpu = per_cpu_read_buffs + cpu;
    PWCollector_sample_t *ptr = pcpu->samples;

    pcpu->size = 0;

    OUTPUT(3, KERN_INFO "Cpu: %d, TSC LIMIT = 0x%llx, curr_index = %d, other_index = %d\n", cpu, tscLimit, curr_index, other_index);

    /*
     * First copy elements from the current segment...
     */
    {
	i = copy_from_seg_i(curr_seg, ptr, cpu, curr_head, curr_tail, tscLimit);
	pcpu->size += (i * sizeof(PWCollector_sample_t));
    }
    ptr += i;
    /*
     * ...and then copy from the other segment.
     * *********************************************
     * Note that the final data is now UNSORTED!
     * *********************************************
     */
    {
	i = copy_from_seg_i(other_seg, ptr, cpu, other_head, other_tail, tscLimit);
	pcpu->size += (i * sizeof(PWCollector_sample_t));
    }

    OUTPUT(3, KERN_INFO "\t[%d]: i = %d, curr_tail = %d, curr_head = %d\n", cpu, i, curr_tail, curr_head);

};

#else // DO_PARALLEL_COPY

int find_read_indices_i(per_cpu_indices_t *pcpu_i, int seg_index, int cpu, int head, int tail, u64 tscLimit)
{
    int i = CIRCULAR_DEC(tail, SAMPLE_MASK), size = 0, retVal = 0;
    int lower_limit = CIRCULAR_DEC(head, SAMPLE_MASK);
    seg_t *seg = pcpu_i->segs[seg_index];

    OUTPUT(3, KERN_INFO "[%d]: h=%d, t=%d, i=%d, ll=%d\n", cpu, head, tail, i, lower_limit);

    /*
     * There's no need to worry about what happens if
     * we get an empty segment: the default values for
     * "start", "stop" and "lower_limit" have been set
     * in the calling function ("find_read_indices(void *)").
     */

    if(head != tail){ // seg is NOT empty
	while(i != lower_limit && SAMPLE(seg, i).tsc > tscLimit) i = CIRCULAR_DEC(i, SAMPLE_MASK);

	/*
	 * Now all elements from "head" to "i" need
	 * to be copied back to user space.
	 * Do that later, when we have EVERY set
	 * of limits.
	 */
	pcpu_i->start_indices[seg_index] = head;
	pcpu_i->stop_indices[seg_index] = i;
	pcpu_i->lower_limits[seg_index] = lower_limit;

	if(i == lower_limit){ // Nothing to read here
	    OUTPUT(3, KERN_INFO "\t\t[%d]: i = %d ll = %d\n", cpu, i, lower_limit);
	    return 0;
	}
	OUTPUT(3, KERN_INFO "[%d,%d]: head = %d, value = %llu, i = %d, i value = %llu, tsc limit = %llu\n", cpu, seg_index, head, SAMPLE(seg, head).tsc, i, SAMPLE(seg, i).tsc, tscLimit);
	/*
	 * Calculate size:
	 * (a) i < head ==> (s1 + s2) * sizeof(PWCollector_sample_t)
	 *    where s1 = (NUM_SAMPLES_PER_SEG - head) // from "head" to "END"
	 *          s2 = (i+1) // from "BEGIN" to "i"
	 * (b) i >= head ==> (i-head+1) * sizeof(PWCollector_sample_t) // from "head" to "i"
	 */
	if(i < head){
	    int t1 = (NUM_SAMPLES_PER_SEG - head), t2 = (i+1);
	    size = (t1 + t2) * sizeof(PWCollector_sample_t);
	}else{
	    size = (i - head + 1) * sizeof(PWCollector_sample_t);
	}
	retVal = size;
    }
    return retVal;
};

void find_read_indices(void *arg)
{
    u64 tscLimit = *((u64 *)arg);
    int cpu = CPU();
    list_t *list = &lists[cpu];
    int curr_index = list->index, other_index = 1 - curr_index;
    seg_t *curr_seg = &list->segs[curr_index]; // current seg
    seg_t *other_seg = &list->segs[other_index];
    int curr_tail = TAIL(curr_seg), other_tail = TAIL(other_seg);
    int curr_head = HEAD(curr_seg), other_head = HEAD(other_seg);
    int size = 0;
    per_cpu_indices_t *pcpu_i = per_cpu_read_indices + cpu;

    // pcpu_i->size = 0;
    pcpu_i->start_indices[0] = pcpu_i->start_indices[1] = 0;
    pcpu_i->stop_indices[0] = pcpu_i->stop_indices[1] = 0;
    pcpu_i->lower_limits[0] = pcpu_i->lower_limits[1] = 0;
    pcpu_i->segs[CURR_SEG] = curr_seg;
    pcpu_i->segs[OTHER_SEG] = other_seg;

    OUTPUT(3, KERN_INFO "Cpu: %d, TSC LIMIT = 0x%llx, curr_index = %d, other_index = %d\n", cpu, tscLimit, curr_index, other_index);

    /*
     * First find indices for the current segment...
     */
    {
	// size += find_read_indices_i(pcpu_i, curr_index, cpu, curr_head, curr_tail, tscLimit);
	size += find_read_indices_i(pcpu_i, CURR_SEG, cpu, curr_head, curr_tail, tscLimit);
	OUTPUT(3, KERN_INFO "\t[%d]: seg=%d, start=%d, stop=%d\n", cpu, curr_index, pcpu_i->start_indices[curr_index], pcpu_i->stop_indices[curr_index]);
    }
    /*
     * ...and then for the other segment.
     * ***********************************************
     * Note that the final data will now be UNSORTED!
     * ***********************************************
     */
    {
	// size += find_read_indices_i(pcpu_i, other_index, cpu, other_head, other_tail, tscLimit);
	size += find_read_indices_i(pcpu_i, OTHER_SEG, cpu, other_head, other_tail, tscLimit);
	OUTPUT(3, KERN_INFO "\t[%d]: seg=%d, start=%d, stop=%d\n", cpu, other_index, pcpu_i->start_indices[other_index], pcpu_i->stop_indices[other_index]);
    }
    pcpu_i->size = size;
};

/*
 * Copy segment data to user space.
 * Returns # of bytes that COULD NOT be copied.
 * Also, sets total size of attempted copy in "pcpu_i->output_index".
 */
int copy_seg_to_user_i(char *user_ptr, per_cpu_indices_t *pcpu_i, int seg_index, int cpu)
{
    seg_t *seg = pcpu_i->segs[seg_index];
    int head = pcpu_i->start_indices[seg_index], i = pcpu_i->stop_indices[seg_index], lower_limit = pcpu_i->lower_limits[seg_index];
    int size = 0, total_size = 0;
    int tmp_bytes_not_copied = 0, bytes_not_copied = 0;

    pcpu_i->output_index = 0;

    if(i == lower_limit){ // nothing to copy for this segment
	OUTPUT(3, KERN_INFO "\t\t[%d]: NOTHING TO COPY for seg = %d!\n", cpu, seg_index);
	return 0;
    }


    OUTPUT(3, KERN_INFO "[%d,%d]: head = %d, value = %llu, i = %d, i value = %llu\n", cpu, seg_index, head, SAMPLE(seg, head).tsc, i, SAMPLE(seg, i).tsc);

    /*
     * Need to copy elements from "head" to "i"
     * But, it is possible that "i" < "head"!
     */
    if(i < head){
	// Go from "head" to END first...
	int t1 = (NUM_SAMPLES_PER_SEG - head);
	size = t1 * sizeof(PWCollector_sample_t);
	total_size += size;
	{
	    if( (tmp_bytes_not_copied = copy_to_user(user_ptr, (char *)&seg->samples[head], size))){ // dst, src
		printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", tmp_bytes_not_copied);
		bytes_not_copied += tmp_bytes_not_copied;
	    }
	}
	user_ptr += size;
	// ...and then go from BEGIN to "i"
	size = (i+1) * sizeof(PWCollector_sample_t);
	total_size += size;
	{
	    if( (tmp_bytes_not_copied = copy_to_user(user_ptr, (char *)seg->samples, size))){ // dst, src
		printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", tmp_bytes_not_copied);
		bytes_not_copied += tmp_bytes_not_copied;
	    }
	}
    }else{
	// Go from "head" to "i"
	int t1 = i - head + 1;
	size = t1 * sizeof(PWCollector_sample_t);
	total_size = size;
	{
	    if( (tmp_bytes_not_copied = copy_to_user(user_ptr, (char *)&seg->samples[head], size))){ // dst, src
		printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", tmp_bytes_not_copied);
		bytes_not_copied += tmp_bytes_not_copied;
	    }
	}
    }
    pcpu_i->output_index = total_size;
    /*
     * Set new "head": this indicates where we're going to read
     * from next, so set this to "i+1" (+1 since we've already read upto,
     * and including, i).
     */
    atomic_set(&seg->head, CIRCULAR_INC(i, SAMPLE_MASK));
    OUTPUT(3, KERN_INFO "\t\t[%d]: Set HEAD for seg %d to %d\n", cpu, seg_index, atomic_read(&seg->head));

    return bytes_not_copied;
};

#endif // DO_PARALLEL_COPY


/*
 * Service a "read(...)" call from user-space.
 *
 * Returns sample information back to the user.
 * When a user calls the "read" function, the device
 * driver first checks if any (per-cpu) output buffers are full.
 * If yes, then the entire contents of that buffer are
 * copied to the user. If not, then the user blocks, until the
 * buffer-full condition is met.
 *
 * The complete READ semantics are as follows:
 * (a) Read issued when DD is CANCELed: return with -EWOULDBLOCK.
 * (b) Read issued when DD is STOPed: return with
 *     (i) 0 => no more data in buffers or
 *     (ii) 'N' <= (PW_max_num_cpus * SEG_SIZE * 2) (subsequent reads return 0).
 * (c) READ issued when DD is PAUSEd: block, waiting for a full buffer. Note
 * that, because collection is also paused, the calling thread
 * will be blocked indefinitely, unless a corresponding RESUME is
 * issued.
 * (d) Slave in Read & Master issues STOP => Same as (b) above.
 * (e) Slave in Read & Master issues PAUSE => Slave blocks until
 * buffer full (but note that collection is paused => buffer fills up
 * only after a RESUME is received. Thus, Slave is effectively
 * blocked until at least a RESUME is received).
 * (f) Slave in Read & Master issues CANCEL => Read returns with -1 (and errno
 * is set to EWOULDBLOCK).
 * N.B.: (d) -- (f) are only valid for asynchronous reads.
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
    int cpu=0;

    u64 tscLimit = 0;
    char *user_ptr = NULL;
    int seg_index = -1;

    /*
     * First, check if we're actively collecting. If not, AND
     * if this isn't an attempt to drain the (per-cpu) buffers, return
     * with an error.
     *
     * Update: allow STOP and PAUSE, but not CANCEL.
     */
    // if(!IS_COLLECTING() && INTERNAL_STATE.drain_buffers == false){
    if(!IS_COLLECTING() && INTERNAL_STATE.cmd == CANCEL){
	OUTPUT(1, KERN_INFO "Warning: (NON-FLUSH) read issued when driver not collecting!\n");
	return -EWOULDBLOCK;
    }

    /*
     * Unprotected access to "copy_buffer".
     * But that's OK: we assume only a SINGLE
     * CONSUMER / READING process!
     */
    if(!copy_buffer){
	printk(KERN_INFO "Error: copy_buffer is NULL!\n");
	return -ERROR;
    }

    /*
     * For now, require the input length to be
     * EXACTLY SEG_SIZE (for optimization)
     * This requirement will be removed in the future.
     */
    if(false && length != SEG_SIZE){
	printk(KERN_INFO "Error: requested length MUST be equal to %d bytes!\n", SEG_SIZE);
	return -ERROR;
    }

    /*
     * Basic strategy:
     * (1) If we've been signalled to flush our buffers
     * (e.g. if a "STOP" command was issued), then find the
     * first non-empty output buffer and return its contents to the user.
     * (2) Else, wait until a buffer is completely full, and then
     * return its contents to the user.
     */
    if(INTERNAL_STATE.drain_buffers){ // drain the buffers
	OUTPUT(3, KERN_INFO "Debug: Draining buffers...\n");
	seg = find_next_non_empty_seg();
	if(!seg)
	    return 0;
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
		    break;
		}
	    }
	    OUTPUT(2, KERN_INFO "Waiting...\n");
	    /*
	     * OK, there's no data for us: block until
	     * either there's enough data to return OR
	     * the driver receives a STOP/CANCEL command.
	     */
	    if(wait_event_interruptible(read_queue, ( (!IS_COLLECTING() && !IS_SLEEPING()) || any_seg_full()))) // returns -ERESTARTSYS if interrupted by signal, "0" on success
		return -ERESTARTSYS;
	}
#else // ALLOW_BLOCKING_READ
	if(!(seg = find_seg()))
	    return 0; // nothing to read
#endif
    }
    size = 0;
    /*
     * "seg" tells us which buffer to look at
     * to discover the TSC limit.
     */
#if DO_PARALLEL_COPY
    {
	per_cpu_read_t *pcpu = NULL;
	user_ptr = buffer; // in USER SPACE!
	seg_index = CIRCULAR_DEC(TAIL(seg), SAMPLE_MASK);
	tscLimit = SAMPLE(seg, seg_index).tsc;
	OUTPUT(2, KERN_INFO "TSC Limit = %llu\n", tscLimit);
	{
	    on_each_cpu(copy_partial_seg, &tscLimit, 1);
	}
	for_each_online_cpu(cpu){
	    pcpu = per_cpu_read_buffs+cpu;
	    OUTPUT(2, KERN_INFO "[%d]: size = %d bytes\n", cpu, pcpu->size);
	    if(pcpu->size){
		int tmp_bytes_not_copied = 0;
		size += pcpu->size;
		/*
		 * Copy data directly into user-space here.
		 */
		if( (tmp_bytes_not_copied = copy_to_user(user_ptr, (char *)pcpu->samples, pcpu->size))){ // dst, src
		    printk(KERN_INFO "Warning: could NOT copy %d bytes!\n", tmp_bytes_not_copied);
		    bytes_not_copied += tmp_bytes_not_copied;
		}
		user_ptr += pcpu->size;
	    }
	}
    }
#else // DO_PARALLEL_COPY
    {
	per_cpu_indices_t *pcpu_i = NULL;
	user_ptr = buffer; // in USER SPACE!
	seg_index = CIRCULAR_DEC(TAIL(seg), SAMPLE_MASK);
	tscLimit = SAMPLE(seg, seg_index).tsc;
	OUTPUT(3, KERN_INFO "TSC Limit = %llu\n", tscLimit);
	{
	    on_each_cpu(find_read_indices, &tscLimit, 1);
	}
	for_each_online_cpu(cpu){
	    pcpu_i = per_cpu_read_indices + cpu;
	    OUTPUT(3, KERN_INFO "[%d]\n", cpu);
	    if(pcpu_i->size){ // total size for BOTH segs
		seg_index = 0;
		for_each_segment(seg_index){
		    bytes_not_copied += copy_seg_to_user_i(user_ptr, pcpu_i, seg_index, cpu);
		    user_ptr += pcpu_i->output_index;
		    OUTPUT(3, KERN_INFO "\t[%d]: %d (%d)\n", seg_index, pcpu_i->output_index, (user_ptr - buffer));
		}
		size += pcpu_i->size;
	    }
	}
	/* return -1; */
    }
#endif // DO_PARALLEL_COPY

    bytes_read = size - bytes_not_copied;

    OUTPUT(1, KERN_INFO "READ bytes_read = %d\n", bytes_read);
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
     * Set power switches.
     */
    INTERNAL_STATE.collection_switches = local_config.data;
    OUTPUT(0, KERN_INFO "\tCONFIG collection switches = %d\n", INTERNAL_STATE.collection_switches);

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
	pstats = &per_cpu(per_cpu_stats, cpu);
	local_set(&pstats->c_breaks, 0);
	local_set(&pstats->timer_c_breaks, 0);
	local_set(&pstats->inters_c_breaks, 0);
	local_set(&pstats->p_trans, 0);
	local_set(&pstats->num_inters, 0);
	local_set(&pstats->num_timers, 0);
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

    switch(cmd){
    case START:
	for_each_online_cpu(cpu){
	    memset(&lists[cpu], 0, sizeof(list_t));
	    for(i=0; i<NUM_SEGS_PER_LIST; ++i){
		atomic_set(&lists[cpu].segs[i].is_full, EMPTY);
		atomic_set(&lists[cpu].segs[i].head, 0);
		atomic_set(&lists[cpu].segs[i].tail, 0);
	    }
	}
	last_list_read = -1;
	last_flush_index = 0;
    case RESUME: // fall through
	break;
    default: // should *NEVER* happen!
	printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
	return -ERROR;
    }
    INTERNAL_STATE.collectionStartJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = true;
    reset_statistics();
    {
	register_all_trace_probes();
    }
    printk(KERN_INFO "\tREGISTERED all probes!\n");
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
    reset_statistics();
    /*
     * There might be a reader thread blocked on a read: wake
     * it up to give it a chance to respond to changed
     * conditions.
     */
    {
	wake_up_interruptible(&read_queue);
    }
    printk(KERN_INFO "\tUNREGISTERED all probes!\n");
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
		start_collection(cmd);
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
	if( (retVal = get_status((struct PWCollector_status *)user_args->out_arg, local_args.out_len))){
	    printk(KERN_INFO "\tError retrieving status information!\n");
	    retVal = -ERROR;
	}
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
 * Serive an "open(...)" call from user-space.
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
    if(copy_buffer)
	kfree(copy_buffer);

    if(lists)
	kfree(lists);

#if DO_PARALLEL_COPY
    {
	if(per_cpu_read_buffs)
	    kfree(per_cpu_read_buffs);
    }
#else
    {
	if(per_cpu_read_indices)
	    kfree(per_cpu_read_indices);
    }
#endif // DO_PARALLEL_COPY

    if(trace_entry_blocks_head)
	destroy_trace_block(trace_entry_blocks_head);
};

/*
 * Initialize data structures here.
 */
static int init_data_structures(void)
{
    int i=0, j=0;
#if DO_PARALLEL_COPY
    per_cpu_read_t *pcpu = NULL;
#else
    per_cpu_indices_t *pcpu_i = NULL;
#endif
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
	return -ERROR;
    }
    for_each_online_cpu(i){
	memset(&lists[i], 0, sizeof(list_t));
	for(j=0; j<NUM_SEGS_PER_LIST; ++j)
	    atomic_set(&lists[i].segs[j].is_full, EMPTY);
    }

    // copy_buffer = (char *)kmalloc(SEG_SIZE, GFP_KERNEL);
    copy_buffer = (char *)kmalloc(PW_max_num_cpus * SEG_SIZE * 2, GFP_KERNEL);
    if(!copy_buffer){
	printk(KERN_INFO "ERROR: Could NOT allocate memory for copy buffer!\n");
	destroy_data_structures();
	return -ERROR;
    }

    /*
     * Init the (per-cpu) read buffers. These
     * are used to contain information
     * from EVERY output buffer in the new read mechanism.
     */
#if DO_PARALLEL_COPY
    {
	per_cpu_read_buffs = (per_cpu_read_t *)kmalloc(PW_max_num_cpus * sizeof(per_cpu_read_t), GFP_KERNEL);

	if(!per_cpu_read_buffs){
	    printk(KERN_INFO "ERROR: Could NOT allocate memory for per cpu read buffers!\n");
	    destroy_data_structures();
	    return -ERROR;
	}

	for_each_online_cpu(i){
	    pcpu = per_cpu_read_buffs + i;
	    memset(pcpu, 0, sizeof(per_cpu_read_t));
	}
    }
#else
    {
	per_cpu_read_indices = (per_cpu_indices_t *)kmalloc(PW_max_num_cpus * sizeof(per_cpu_indices_t), GFP_KERNEL);

	if(!per_cpu_read_indices){
	    printk(KERN_INFO "ERROR: Could NOT allocate memory for per cpu read indices!\n");
	    destroy_data_structures();
	    return -ERROR;
	}

	for_each_online_cpu(i){
	    pcpu_i = per_cpu_read_indices + i;
	    memset(pcpu_i, 0, sizeof(per_cpu_indices_t));
	}
    }
#endif // DO_PARALLEL_COPY

    /*
     * Init the (hash-based) timer::tsc mapping list here.
     */
    trace_entry_blocks_head = trace_entry_blocks_tail = alloc_new_trace_block(); // (block_t *)malloc(sizeof(trace_entry_blocks_head));
    if(!trace_entry_blocks_head){
	destroy_data_structures();
	return -ERROR;
    }

    free_list_head = trace_entry_blocks_head->block;

    memset(hash_table, 0, sizeof(hash_table));

    // For debugging only
    num_free_trace_entries = MAX_NUM_TRACE_ENTRIES;

    /*
     * Init the state struct here.
     */
    memset(&INTERNAL_STATE, 0, sizeof(INTERNAL_STATE));

    return SUCCESS;

};


/*
 * Initialize the device driver.
 * For now, also insert tracepoints here. This
 * will probably change (e.g., wait to insert
 * tracepoints until we get a START command?).
 */
static int __init init_hooks(void)
{
    int i=0, ret;

    printk(KERN_INFO "pid_t size = %d\n", sizeof(pid_t));
    printk(KERN_INFO "per_cpu_t size = %d\n", sizeof(per_cpu_t));
    printk(KERN_INFO "c_sample_t size = %d\n", sizeof(c_sample_t));
    printk(KERN_INFO "k_sample_t size = %d\n", sizeof(k_sample_t));
    printk(KERN_INFO "PWCollector_sample size = %d\n", sizeof(struct PWCollector_sample));
    printk(KERN_INFO "platform_info_t size = %d\n", sizeof(platform_info_t));
    printk(KERN_INFO "config size = %d\n", sizeof(struct PWCollector_config));
    printk(KERN_INFO "Max MSR addresses = %d\n", MAX_MSR_ADDRESSES);
    printk(KERN_INFO "Lists size = %d\n", sizeof(list_t));
    printk(KERN_INFO "# Online Cpus = %d\n", num_online_cpus());


    startJIFF = jiffies;

    atomic_set(&num_c_state_breaks, 0);
    atomic_set(&num_invalid_residencies, 0);
    atomic_set(&num_zero_residencies, 0);

#ifdef CONFIG_STACKTRACE
    printk(KERN_INFO "Stacktrace ON!\n");
#endif

    /*
     * Init all locks here
     */
    {
	for(i=0; i<NUM_HASH_LOCKS; ++i)
	    rwlock_init(&hashLocks[i]);
    }


    {
	if(init_data_structures())
	    return -ERROR;
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
	printk(KERN_INFO "Frame pointer ON!\n");
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


    printk(KERN_INFO "\nSTART Initialized the probes\n");

    return 0;
}

/*
 * Cleanup procedure: remove all previously inserted tracepoints.
 */
static void __exit cleanup_hooks(void)
{
    unsigned long elapsedJIFF = 0;

    {
	unregister_dev();
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

    printk(KERN_INFO "\nSTOP Cleaned up the probes. Total time elapsed = %u msecs\n", jiffies_to_msecs(elapsedJIFF));

    /*
     * Collect some collection statistics: total collection time.
     */
    if(INTERNAL_STATE.collectionStopJIFF < INTERNAL_STATE.collectionStartJIFF){
	printk(KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
	elapsedJIFF = 0;
    }else{
	elapsedJIFF = INTERNAL_STATE.collectionStopJIFF - INTERNAL_STATE.collectionStartJIFF;
    }
    printk(KERN_INFO "\nTotal collection time = %u msecs\n", jiffies_to_msecs(elapsedJIFF));

    if(false){
	printk(KERN_INFO "\nTOTAL # C-state breaks = %d\n", atomic_read(&num_c_state_breaks));
	printk(KERN_INFO "\n# INVALID C-state residencies = %d\n", atomic_read(&num_invalid_residencies));
	printk(KERN_INFO "\n# ZERO C-state residencies = %d\n", atomic_read(&num_zero_residencies));
    }

    printk(KERN_INFO "\t# free trace entries = %llu, total # trace entries = %llu\n", num_free_trace_entries, total_num_trace_entries);
}

module_init(init_hooks);
module_exit(cleanup_hooks);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
