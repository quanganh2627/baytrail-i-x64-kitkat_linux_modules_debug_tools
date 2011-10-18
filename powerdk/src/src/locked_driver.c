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
 * locked_driver.c: Prototype kernel module to trace the following
 * events that are relevant to power:
 *	- entry into a C-state
 *	- change of processor frequency
 *	- interrupts and timers
 */

#define MOD_AUTHOR "Gautam Upadhyaya <gautam.upadhyaya@intel.com>"
#define MOD_DESC "TLS-based approach to prototype collector for Piersol power tool. Adapted from Romain Cledat's codebase."

/*
 * Current driver version is 1.0.1
 * Current driver version is 1.0.2
 * Current driver version is 1.0.3
 * Current driver version is 1.0.4
 */
#define PW_VERSION_VERSION 1
#define PW_VERSION_INTERFACE 0
#define PW_VERSION_OTHER 4

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

#include <asm/hardirq.h>

#include <linux/preempt.h>

#include <linux/jiffies.h>

#include <linux/kallsyms.h>

#include <asm/desc.h>

#include <linux/stacktrace.h>
#include <asm/stacktrace.h>

#include <linux/hash.h>

#include <trace/events/sched.h> // for "sched_process_exit"

#include <linux/nmi.h>

#include <linux/poll.h>

#include <linux/pid.h>

#include <linux/tick.h> // for "struct tick_sched"

#include <linux/list.h>

#include <linux/slab.h>

#include <asm/cmpxchg.h>

#include "pw_lock_defs.h"
#include "exp_pw_data_structs.h"

/**** CONFIGURATION ****/

static int PW_max_num_cpus = -1;

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
#define DO_OVERHEAD_MEASUREMENTS 0
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
#define DO_ALLOW_MULTI_DEV_IRQ 1


/*
 * Compile-time constants and
 * other macros.
 */

#define NUM_MAP_BUCKETS_BITS 8
#define NUM_MAP_BUCKETS (1UL << NUM_MAP_BUCKETS_BITS)

// 32 locks for the hash table
#define HASH_LOCK_BITS 5
#define NUM_HASH_LOCKS (1UL << HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

#define HASH_READ_LOCK(i) READ_LOCK(hash_locks[(i) & HASH_LOCK_MASK])
#define HASH_READ_UNLOCK(i) READ_UNLOCK(hash_locks[(i) & HASH_LOCK_MASK])

#define HASH_WRITE_LOCK(i) WRITE_LOCK(hash_locks[(i) & HASH_LOCK_MASK])
#define HASH_WRITE_UNLOCK(i) WRITE_UNLOCK(hash_locks[(i) & HASH_LOCK_MASK])


#define NUM_NODES_PER_BLOCK 32

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

#define BEGIN_IRQ_STATS_READ(p, c) do{\
	p = &per_cpu(irq_stat, (c));

#define END_IRQ_STATS_READ(p, c) \
	}while(0)

#define BEGIN_LOCAL_IRQ_STATS_READ(p) do{\
	p = &__get_cpu_var(irq_stat);

#define END_LOCAL_IRQ_STATS_READ(p) \
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
#define record_hit_full(s,pid,tid,which,what,d) do{	\
	if(unlikely( (s)->debug_enters == 0)){		\
	    tscval(&((s)->debug_enters));		\
	    (s)->last_pid = pid;			\
	    (s)->last_tid = tid;			\
	    (s)->last_break[(which)] = what;		\
	    (s)->is_deferrable = d;			\
	}						\
    }while(0)

/*
 * Data structure definitions.
 */

typedef struct node node_t;
struct node{
	node_t *next;
	unsigned long timer_addr;
	pid_t tid, pid;
	u64 tsc;
	u16 trace_sent : 1;
	u16 trace_len : 15;
	unsigned long *trace;
	// unsigned char padding[52];
};

typedef struct block block_t;
struct block{
	/*
	 * A CPU can have more than
	 * one 'block' of nodes.
	 */
	block_t *next;
	/*
	 * Nodes allocated for this
	 * block -- we need a reference
	 * to those if/when we free
	 * up all system resources.
	 */
	node_t *nodes;
};


typedef struct per_cpu_mem per_cpu_mem_t;
struct per_cpu_mem{
	block_t *block_list;
	node_t *free_list;
};

typedef struct node *hash_node_t;

/*
 * For IRQ # <--> DEV NAME mappings.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

typedef struct irq_node irq_node_t;
struct irq_node{
	irq_node_t *next;
	int irq;
	u32 count;
	char *name;
};

typedef struct irq_node *irq_hash_node_t;
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
 * For tracking process PID <-> NAME mappings
 * in sched exit.
 */
typedef struct proc_node proc_node_t;
struct proc_node{
	PWCollector_proc_mapping_t mapping;
	/*
	pid_t pid;
	char name[TASK_COMM_LEN];
	*/
	struct list_head list;
};

#define PROC_MAPPING_PID(p) ( (p)->mapping.pid )
#define PROC_MAPPING_TID(p) ( (p)->mapping.tid )
#define PROC_MAPPING_NAME(p) ( (p)->mapping.name )
#define PROC_MAPPING_LIST(p) ( (p)->list )

/*
 * Variable declarations.
 */

/*
 * A list of MSR names.
 * Useful if debugging.
 */
#if DO_DEBUG_OUTPUT
static const char *msr_names[] = {"C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"};
#endif

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

static DEFINE_PER_CPU(struct per_cpu_mem, per_cpu_mem_vars);

static hash_node_t *timer_map = NULL;

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
static irq_hash_node_t *irq_map = NULL;
static PWCollector_irq_mapping_t *irq_mappings_list = NULL;
static int total_num_irq_mappings = 0;
#endif //  DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static LIST_HEAD(proc_list);
static DEFINE_SPINLOCK(proc_list_lock);
static PWCollector_proc_mapping_t *proc_mappings_list = NULL;
static int total_num_proc_mappings = 0;

static list_t *lists = NULL;

DEFINE_PER_CPU(per_cpu_t, per_cpu_counts);

DEFINE_PER_CPU(stats_t, per_cpu_stats);

DEFINE_PER_CPU(CTRL_values_t, CTRL_data_values);

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
 * Hash locks.
 */
static rwlock_t hash_locks[NUM_HASH_LOCKS];
/*
 * IRQ Map locks.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
static spinlock_t irq_map_locks[NUM_HASH_LOCKS];
#endif

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
DECLARE_OVERHEAD_VARS(timer_insert); // for "timer_insert"
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

#define DO_PER_CPU_OVERHEAD_FUNC_RET(ret, func, ...) do{ \
	u64 *__v = &__get_cpu_var(func##_elapsed_time);	\
	u64 tmp_1 = 0, tmp_2 = 0;			\
	local_inc(&__get_cpu_var(func##_num_iters));	\
	tscval(&tmp_1);					\
	{						\
	    ret = func(__VA_ARGS__);				\
	}						\
	tscval(&tmp_2);					\
	*(__v) += (tmp_2 - tmp_1);			\
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
 * malloc and free helpers
 */
static inline void *pw_kmalloc(size_t size, int flags)
{
	void *ret = NULL;

	if(SHOULD_TRACE()){
		if(!(ret = kmalloc(size, flags))){
			atomic_set(&should_panic, 1);
		}
	}
	return ret;
};

static inline char *pw_kstrdup(const char *str, int flags)
{
	char *ret = NULL;

	if(SHOULD_TRACE()){
		if(!(ret = kstrdup(str, flags))){
			atomic_set(&should_panic, 1);
		}
	}

	return ret;
};

static inline void pw_kfree(void *mem)
{
	kfree(mem);
};


/*
 * Initialization and termination routines.
 */

#define LINK_BLOCK_NODES(nodes, num) do{\
	node_t *array = (nodes); \
	int __idx = 0; \
	for(__idx=1; __idx < (num); ++__idx) \
		array[__idx-1].next = &array[__idx]; \
	array[(num)-1].next = NULL; \
}while(0)

#define GET_MEM_VARS(cpu) &per_cpu(per_cpu_mem_vars, (cpu))
#define GET_MY_MEM_VARS() &__get_cpu_var(per_cpu_mem_vars)

static block_t *allocate_new_block(void)
{
	block_t *block = (block_t *)pw_kmalloc(sizeof(block_t), GFP_ATOMIC);
	if(!block)
		return NULL;

	block->next = NULL;

	block->nodes = (node_t *)pw_kmalloc(sizeof(node_t) * NUM_NODES_PER_BLOCK, GFP_ATOMIC);
	if(!block->nodes){
		pw_kfree(block);
		return NULL;
	}

	memset(block->nodes, 0, sizeof(node_t) * NUM_NODES_PER_BLOCK);

	LINK_BLOCK_NODES(block->nodes, NUM_NODES_PER_BLOCK);

	return block;
};


static void block_free(block_t *block)
{
	int i=0;

	if(!block)
		return;

	block_free(block->next);

	if(block->nodes){
		for(i=0; i<NUM_NODES_PER_BLOCK; ++i){
			if(block->nodes[i].trace)
				pw_kfree(block->nodes[i].trace);
		}
		pw_kfree(block->nodes);
	}
	pw_kfree(block);
	return;
};

static void free_allocated_blocks(int max_block_allocated)
{
	int i=0;

	for(i=0; i<max_block_allocated; ++i){
		per_cpu_mem_t *vars = GET_MEM_VARS(i);
		block_free(vars->block_list);
	}
};

static int init_per_cpu_mem_vars(void)
{
	int cpu = 0;
	block_t *block;

	for_each_online_cpu(cpu){
		per_cpu_mem_t *vars = GET_MEM_VARS(cpu);
		block = allocate_new_block();
		if(!block){
			free_allocated_blocks(cpu);
			return -ERROR;
		}
		vars->block_list = block;
		vars->free_list = block->nodes;
	}

	return SUCCESS;
};

static void destroy_per_cpu_mem_vars(void)
{
	int i=0;

	for_each_online_cpu(i){
		per_cpu_mem_t *vars = GET_MEM_VARS(i);
		block_t *block = vars->block_list;
		block_free(block);
	}

};

static int init_timer_map(void)
{
	int i=0;
	timer_map = (hash_node_t *)pw_kmalloc(sizeof(hash_node_t) * NUM_MAP_BUCKETS, GFP_ATOMIC);
	if(!timer_map)
		return -ERROR;

	for(i=0; i<NUM_MAP_BUCKETS; ++i){
		timer_map[i] = NULL;
	}

	/*
	 * Init locks
	 */
	for(i=0; i<NUM_HASH_LOCKS; ++i)
		rwlock_init(&hash_locks[i]);

	return SUCCESS;
};

static void destroy_timer_map(void)
{
	if(timer_map)
		pw_kfree(timer_map);
	timer_map = NULL;
};

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static int init_irq_map(void)
{
	int i=0;
	irq_map = (irq_hash_node_t *)pw_kmalloc(sizeof(irq_hash_node_t) * NUM_IRQ_MAP_BUCKETS, GFP_KERNEL);
	if(!irq_map)
		return -ERROR;

	for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
		irq_map[i] = NULL;
	}

	/*
	 * Init locks
	 */
	for(i=0; i<NUM_HASH_LOCKS; ++i)
		// rwlock_init(&irq_map_locks[i]);
		spin_lock_init(&irq_map_locks[i]);

	total_num_irq_mappings = 0;

	return SUCCESS;
};

static void free_irq_node_i(irq_node_t *node)
{
	if(!node)
		return;
	free_irq_node_i(node->next);

	if(node->name)
		pw_kfree(node->name);

	pw_kfree(node);

	return;
};

static void destroy_irq_map(void)
{
	int i=0;

	if(irq_map){
		for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i)
			free_irq_node_i(irq_map[i]);

		pw_kfree(irq_map);
	}
	irq_map = NULL;

	if(irq_mappings_list){
		pw_kfree(irq_mappings_list);
		irq_mappings_list = NULL;
	}
};

#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static int init_proc_list(void)
{
	/*
	 * Nothing to do.
	 */
	return SUCCESS;
};

static void destroy_proc_list(void)
{
	while(!list_empty(&proc_list)){
		proc_node_t *node = list_first_entry(&proc_list, struct proc_node, list);
		list_del(&node->list);
		OUTPUT(3, KERN_INFO "[%d, %d]: %s\n", PROC_MAPPING_PID(node), PROC_MAPPING_TID(node), PROC_MAPPING_NAME(node));
		// printk(KERN_INFO "[%d]: %s\n", node->pid, node->name);
		pw_kfree(node);
	}

	if(proc_mappings_list){
		pw_kfree(proc_mappings_list);
		proc_mappings_list = NULL;
	}

	total_num_proc_mappings = 0;
};

static void destroy_data_structures(void)
{
	destroy_per_cpu_mem_vars();
	destroy_timer_map();

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
	destroy_irq_map();
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS

	destroy_proc_list();

	if(lists)
		pw_kfree(lists);
	lists = NULL;

};

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
    lists = (list_t *)pw_kmalloc(PW_max_num_cpus * sizeof(list_t), GFP_KERNEL);
	if(!lists){
		printk(KERN_INFO "ERROR: Could NOT allocate memory for lists!\n");
		return -ERROR;
	}

	for_each_online_cpu(i){
		memset(&lists[i], 0, sizeof(list_t));
		for(j=0; j<NUM_SEGS_PER_LIST; ++j)
			atomic_set(&lists[i].segs[j].is_full, EMPTY);
	}

	if(init_per_cpu_mem_vars()){
		printk(KERN_INFO "ERROR: could NOT initialize per-cpu mem areas!\n");
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

	if(init_proc_list()){
		printk(KERN_INFO "ERROR: could NOT init proc list!\n");
		destroy_data_structures();
		return -ERROR;
	}

	return SUCCESS;
};

/*
 * Free list manipulation routines.
 */

#define IS_FREE_LIST_EMPTY(vars) ( (vars)->free_list == NULL )

static node_t *get_next_free_node_i(void)
{
	node_t *retVal = NULL;
	per_cpu_mem_t *vars = GET_MY_MEM_VARS();
	if(IS_FREE_LIST_EMPTY(vars)){
		/*
		 * OK, we've run out of free nodes -- allocate
		 * a new block of free nodes.
		 */
		block_t *block = vars->block_list;
		block_t *new_block = allocate_new_block();
		if(new_block){
			/*
			 * Add the new block to our block list.
			 */
			{
				new_block->next = block->next;
				block->next = new_block;
			}
			/*
			 * And finally, add the fresh set of nodes
			 * to the free list.
			 */
			vars->free_list = new_block->nodes;
			/* list_add(&new_block->nodes[0].list, head); */
			printk(KERN_INFO "ALLOCATED A NEW BLOCK!\n");
		}
	}
	if(IS_FREE_LIST_EMPTY(vars)){
		printk(KERN_INFO "ERROR: no free nodes to get!\n");
	}else{
		retVal = vars->free_list;
		vars->free_list = retVal->next;
	}
	return retVal;
};

static void free_node_i(node_t *node)
{
	per_cpu_mem_t *vars = GET_MY_MEM_VARS();
	node->next = vars->free_list;
	vars->free_list = node;
};

/*
 * Hash map routines.
 */

/*
 * UNPROTECTED, fast search for the next timer
 * address for a given TID
 */
static node_t *find_timer_addr_for_tid_i(pid_t tid, int *index)
{
	int idx = *index;
	node_t *curr = NULL;

	if(idx < 0)
		idx = 0;

	for(; idx < NUM_MAP_BUCKETS; ++idx){
		curr = timer_map[idx];
		while(curr && curr->tid != tid)
			curr = curr->next;
		if(curr)
			break;
	}

	*index = idx;

	return curr;
};

static node_t *map_find_unlocked_i(unsigned long timer_addr, pid_t tid, int *index)
{
	int idx = TIMER_HASH_FUNC(timer_addr);
	node_t **head = &timer_map[idx];
	node_t *curr = NULL;

	*index = idx;

	smp_mb();

	/*
	 * UNLOCKED check.
	 */
	for(curr = *head; curr && curr->timer_addr != timer_addr; curr = curr->next);

	return curr;
};

static node_t *timer_find(unsigned long timer_addr, pid_t tid)
{
	int idx = -1;
	node_t *node = map_find_unlocked_i(timer_addr, tid, &idx);
	// return map_find_i(timer_addr, tid, &idx);
	return (node && node->tid == tid) ? node : NULL;
};

static void timer_insert(unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, int trace_len, unsigned long *trace)
{
	int idx = -1;
	node_t *curr = NULL;
	// DO_PER_CPU_OVERHEAD_FUNC_RET(curr, map_find_i, timer_addr, tid, &idx);
	DO_PER_CPU_OVERHEAD_FUNC_RET(curr, map_find_unlocked_i, timer_addr, tid, &idx);
	// node_t *curr = map_find_i(timer_addr, tid, &idx);
	//
	if(!curr){
		// curr = get_next_free_node_i();
		DO_PER_CPU_OVERHEAD_FUNC_RET(curr, get_next_free_node_i);
		if(likely(curr)){
			/*
			 * We only copy 'timer_addr' here -- the
			 * other fields are populated below
			 */
			curr->timer_addr = timer_addr; // curr->tsc = tsc; curr->tid = tid; curr->pid = pid; curr->trace_len = trace_len;
			HASH_WRITE_LOCK(idx);
			{
				node_t **head = &timer_map[idx];
				curr->next = *head;
				*head = curr;
			}
			HASH_WRITE_UNLOCK(idx);
		}
	}

	if(curr){
		curr->tsc = tsc; curr->tid = tid; curr->pid = pid; curr->trace_sent = 0; curr->trace_len = trace_len;
		if(curr->trace){
			pw_kfree(curr->trace);
			curr->trace = NULL;
		}
		if(trace_len >  0){
			curr->trace = pw_kmalloc(sizeof(unsigned long) * trace_len, GFP_ATOMIC);
			if(!curr->trace){
				printk(KERN_INFO "ERROR: could NOT allocate memory for backtrace!\n");
				return;
			}
			memcpy(curr->trace, trace, sizeof(unsigned long) * trace_len);
		}
	}else{
		printk(KERN_INFO "ERROR: could NOT insert timer addr = %lu\n", timer_addr);
	}
};

static bool remove_entry(unsigned long timer_addr, pid_t tid)
{
	int idx = -1;
	// node_t *curr = map_find_i(timer_addr, tid, &idx);
	node_t *curr = map_find_unlocked_i(timer_addr, tid, &idx);

	if(!curr){
		printk(KERN_INFO "ERROR: could NOT remove timer addr = %lu\n", timer_addr);
		return false;
	}

	HASH_WRITE_LOCK(idx);
	{
		node_t **head = &timer_map[idx];
		node_t *prev = *head;
		if(curr == prev){
			*head = prev->next;
		}else{
			while(prev && prev->next != curr) prev = prev->next;
			BUG_ON(!prev);
			prev->next = curr->next;
		}
		curr->next = NULL;
	}
	HASH_WRITE_UNLOCK(idx);

	free_node_i(curr);

	return true;
};

/*
 * WARNING: NOT thread-safe!
 */
static int get_num_timers(void)
{
	int num_timers = 0;
	int i=0;

	OUTPUT(3, KERN_INFO "DUMPING timer map...\n");

	for(i=0; i<NUM_MAP_BUCKETS; ++i){
		node_t *curr = timer_map[i];
		for(; curr; curr=curr->next, ++num_timers){
			OUTPUT(3, KERN_INFO "[%d]: %d, 0x%lx\n", i, curr->tid, curr->timer_addr);
		}
	}
	return num_timers;
};

/*
 * IRQ list manipulation routines.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static irq_node_t *find_irq_node_i(int irq_num, const char *irq_name, int *index)
{
	int idx = IRQ_MAP_HASH_FUNC(irq_num);
	irq_node_t **head = &irq_map[idx];
	irq_node_t *curr = NULL;

	*index = idx;

	smp_mb();

#if DO_ALLOW_MULTI_DEV_IRQ
	for(curr = *head; curr && curr->irq <= irq_num; curr = curr->next)
		if(curr->irq == irq_num && !strcmp(curr->name, irq_name))
			return curr;

	return NULL;
#else // DO_ALLOW_MULTI_DEV_IRQ
	for(curr = *head; curr && curr->irq < irq_num; curr = curr->next);

	return (curr && curr->irq == irq_num) ? curr : NULL;
#endif // DO_ALLOW_MULTI_DEV_IRQ
};

static int irq_insert(int irq_num, const char *irq_name)
{
	irq_node_t *curr = NULL, *prev = NULL, **head = NULL;
	int idx = -1;
	// curr = find_irq_node_i(irq_num, irq_name);
	DO_PER_CPU_OVERHEAD_FUNC_RET(curr, find_irq_node_i, irq_num, irq_name, &idx);

	if(curr){
		++curr->count;
		return SUCCESS;
	}

	smp_mb();

	if(! (curr = pw_kmalloc(sizeof(irq_node_t), GFP_ATOMIC))){
		printk(KERN_INFO "ERROR: could NOT allocate memory for a new irq <-> dev mapping!\n");
		return -ERROR;
	}
	memset(curr, 0, sizeof(irq_node_t));

	curr->irq = irq_num;
	if( !(curr->name = pw_kstrdup(irq_name, GFP_ATOMIC))){
		printk(KERN_INFO "ERROR: could NOT kstrdup irq device name: %s\n", irq_name);
		pw_kfree(curr);
		return -ERROR;
	}


	IRQ_LOCK(idx);
	{
		head = &irq_map[idx];
		/*
		 * Search for position to insert 'curr' in the
		 * (sorted) list.
		 * Yes, this is slow, but *should* be quite rare.
		 *
		 * We first check to see if:
		 * (a) the bucket is EMPTY or
		 * (b) 'curr' should be the FIRST node in the list.
		 * Note that these checks are required ONLY because
		 * we're setting the buckets to NULL (instead of
		 * a sentinel node).
		 */
		if( (prev = *head) == NULL || irq_num < prev->irq){
			curr->next = *head;
			*head = curr;
		}else{
			/*
			 * Note: because of the 'if' check above, we're GUARANTEED
			 * that 'prev' is NON-NULL.
			 * Note: we re-assign 'prev' just for completeness.
			 */
			for(prev = *head; prev->next && prev->next->irq < irq_num; prev = prev->next);

			curr->next = prev->next;
			prev->next = curr;
		}
	}
	IRQ_UNLOCK(idx);

	++curr->count;

	++total_num_irq_mappings;

	return SUCCESS;
};

/*
 * UNLOCKED access!
 */
static int get_num_irq_mappings(void)
{
	int retVal = 0;
	int i=0;
	irq_node_t *node = NULL;

	for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i)
		for(node = irq_map[i]; node; node = node->next, ++retVal){
			OUTPUT(3, KERN_INFO "[%d]: IRQ Num=%d, Dev=%s called %d times\n", i, node->irq, node->name, node->count);
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
 */
static inline seg_t *find_producer_seg(int cpu)
{
	list_t *list = &lists[cpu];
	int list_index = -1, num_full = 0;
	seg_t *seg = NULL;

	do{
		list_index = list->index;
		seg = &list->segs[list_index];
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
		SAMPLE(seg, seg_index).cpuidx = cpu;
		SAMPLE(seg, seg_index).tsc = pcpu->tsc;
		{
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
		SAMPLE(seg, seg_index).cpuidx = cpu;
		SAMPLE(seg, seg_index).tsc = res;
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
static inline void produce_k_sample(int cpu, const node_t *tentry)
{
	int seg_index = -1;
	seg_t *seg = NULL;


	OUTPUT(3, KERN_INFO "KERNEL-SPACE mapping!\n");

	if(!(seg = find_producer_seg(cpu))){
		printk(KERN_ERR "Error: No buffer available for cpu=%d!\n", cpu);
		return;
	}

	seg_index = seg->index;
	{
		SAMPLE(seg, seg_index).sample_type = K_CALL_STACK;
		SAMPLE(seg, seg_index).sample_len = 1;
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
				OUTPUT(0, KERN_ERR "Warning: kernel trace len = %d > TRACE_LEN = %d! Will need CHAINING!\n", num, TRACE_LEN);
				num = TRACE_LEN;
			}
			memcpy(K_SAMPLE(seg, seg_index).trace, tentry->trace, sizeof(unsigned long) * num);
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
	 * For now, FORCE KTIMER mode to be TRUE.
	 */
	/* if(!tid  && IS_KTIMER_MODE()){ */
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

#if (KERNEL_VER < 35)
static void probe_hrtimer_init(struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#else
    static void probe_hrtimer_init(void *ignore, struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#endif
{
	if(!SHOULD_TRACE())
		return;
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

#if (KERNEL_VER < 35)
static void probe_timer_init(struct timer_list *timer)
#else
    static void probe_timer_init(void *ignore, struct timer_list *timer)
#endif
{
	if(!SHOULD_TRACE())
		return;
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

    OUTPUT(3, KERN_INFO "[%d]: ITIMER STATE: timer = %p\n", TID(), timer);
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
    pid_t tid = TID();
    pid_t pid = PID();
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

    if(INTERNAL_STATE.have_kernel_frame_pointers){
	trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
    OUTPUT(0, KERN_INFO "[%d]: %.20s TIMER_START for timer = %p. trace_len = %d\n", tid, hrt->start_comm, hrt, trace_len);
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
// static void timer_insert(unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, int trace_len, unsigned long *trace)
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
static const struct node EMPTY_tentry = {
	.next = NULL,
    .timer_addr = 0,
	.tid = 0,
	.pid = 0,
    .tsc = 0x1,
    .trace_sent = 0,
    .trace_len = 0,
    .trace = NULL,
};

static void timer_expire(void *timer_addr, pid_t tid, bool is_deferrable)
{
    int cpu = -1;
    per_cpu_t *pcpu = NULL;
    pid_t pid = -1;
    node_t *entry = NULL;
    u64 tsc = 0;
    bool found = false;

    /*
     * Reduce overhead -- do NOT run
     * if user specifies NO C-STATES.
     */
    if(unlikely(!IS_SLEEP_MODE())){
	return;
    }

    if(true && is_deferrable)
	OUTPUT(3, KERN_INFO "[%d]: DEFERRABLE TIMER!\n", CPU());

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

    if(true){
	if(pcpu->debug_enters == 0){
	    if(is_deferrable){
		OUTPUT(0, KERN_INFO "\tDEFERRABLE TIMER WAKEUP!\n");
	    }
	}
    }

    cpu = CPU();

    if( (entry = (node_t *)timer_find((unsigned long)timer_addr, tid))){
	pid = entry->pid;
	found = true;
    }else{
	/*
	 * Couldn't find timer entry -- PID defaults to TID.
	 */
	pid = tid;
	entry = (node_t *)&EMPTY_tentry;
	OUTPUT(3, KERN_INFO "Warning: [%d]: timer %p NOT found in list!\n", pid, timer_addr);
    }

    tsc = entry->tsc;

    if(!found){
	if(!tid && (timer_addr == pcpu->sched_timer_addr)){
	    /*
	     * Should NEVER happen -- the 'hrtimer_start' probe
	     * should have inserted this address into
	     * the hash map!
	     */
	    tsc = 0x1 + cpu;
	    printk(KERN_INFO "ERROR: break caused by SCHED TIMER for cpu = %d! Setting tsc to %llu\n", cpu, tsc);
	}else{
	    tsc = PW_max_num_cpus + 1;
	}
    }

    record_hit_full(pcpu, pid, tid, TIMER, tsc, is_deferrable);

    /*
     * OK, send the TIMER::TSC mapping & call stack to the user
     * (but only if this is for a kernel-space call stack and the
     * user wants kernel call stack info).
     */
    // if(!pid && INTERNAL_STATE.write_to_buffers && found && !entry->trace_sent){
    if(!pid && INTERNAL_STATE.write_to_buffers && IS_KTIMER_MODE() && found && !entry->trace_sent){
	produce_k_sample(cpu, entry);
	entry->trace_sent = 1;
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
	if(!SHOULD_TRACE())
		return;
    // timer_expire(hrt, hrt->start_pid, false);
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, hrt, hrt->start_pid, false);
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
	if(!SHOULD_TRACE())
		return;
    OUTPUT(1, KERN_INFO "[%d]: TIMER_EXPIRE: Deferrable = %s\n", CPU(), GET_BOOL_STRING(IS_TIMER_DEFERRABLE(t)));
    // timer_expire(t, t->start_pid, IS_TIMER_DEFERRABLE(t));
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, t, t->start_pid, IS_TIMER_DEFERRABLE(t));
};

/*
 * Function common to all interrupt tracepoints.
 */
static void inter_common(int irq_num, const char *irq_name)
{
	per_cpu_t *pcpu = NULL;
	unsigned char is_deferrable = 0;

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

	if(!in_interrupt()){
		printk(KERN_ERR "BUG: inter_common() called from a NON-INTERRUPT context! Got irq: %lu and soft: %lu\n", in_irq(), in_softirq());
		return;
	}

	// printk(KERN_INFO "[%d]: IRQ %d, NAME = %s\n", CPU(), irq_num, irq_name);

	/*
	 * Interrupt context: no need for expensive "get_cpu_var(...)" version.
	 */
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

	/*
	 * Record if this interrupt caused a C-state
	 * wakeup.
	 */
	was_hit = pcpu->debug_enters == 0;
	record_hit_full(pcpu, 0, 0, IRQ, irq_num, is_deferrable);

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
	{
		int __ret = -1;
		/*
		 * We only cache device names if they
		 * actually caused a C-state
		 * wakeup.
		 */
		if(was_hit){
			// __ret = irq_insert(irq_num, irq_name);
			DO_PER_CPU_OVERHEAD_FUNC_RET(__ret, irq_insert, irq_num, irq_name);
			if(__ret){
				printk(KERN_INFO "ERROR: could NOT insert [%d,%s] into irq list!\n", irq_num, irq_name);
			}
		}
	}
#endif
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
	const char *name = action->name;
	if(!SHOULD_TRACE())
		return;
	// printk(KERN_INFO "IRQ: %d->%s\n", irq, name);
	OUTPUT(3, KERN_INFO "NUM: %d\n", irq);
    // inter_common(irq);
	DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
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
	int irq = -1;
	const char *name = NULL;
	if(!SHOULD_TRACE())
		return;
	irq = (int)(h-vec);
	name = pw_softirq_to_name[irq];
	// printk(KERN_INFO "SOFTIRQ: %d->%s\n", irq, name);
	OUTPUT(3, KERN_INFO "NUM: %d\n", irq);
    // inter_common((int)(h-vec));
	DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};


#define C1 APERF
#define CN MAX_MSR_ADDRESSES

#define UNHLT_REF MPERF

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

	tscval(&begin_tsc);

	msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[MPERF];
	// rdmsrl(msr_addr, res);
	{
		u32 l=0, h=0;

		int ret = rdmsr_safe_on_cpu(cpu, msr_addr, &l, &h);
		// WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
		if(ret){
			printk(KERN_INFO "WARNING: rdmsr failed with code %d\n", ret);
		}
		res = h;
		res <<= 32;
		res += l;
		if(unlikely(res < PREV_MSR_VAL(pcpu, MPERF))){
			/* This means we had a rollover */
			PREV_MSR_VAL(pcpu, MPERF) = (unsigned long long)(~0) - PREV_MSR_VAL(pcpu, MPERF);
			res += PREV_MSR_VAL(pcpu, MPERF) + 1;
			PREV_MSR_VAL(pcpu, MPERF) = 0;

			printk(KERN_INFO "Warning: ROLLOVER for MPERF counter for CPU %u\n", cpu);
		}
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
		{
			u32 l=0, h=0;

			int ret = rdmsr_safe_on_cpu(cpu, msr_addr, &l, &h);
			if(ret){
				printk(KERN_INFO "WARNING: rdmsr failed with code %d\n", ret);
			}
			WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
			res = h;
			res <<= 32;
			res += l;
			if(unlikely(res < PREV_MSR_VAL(pcpu, i))){
				/* This means we had a rollover */
				PREV_MSR_VAL(pcpu, i) = (unsigned long long)(~0) - PREV_MSR_VAL(pcpu, i);
				res += PREV_MSR_VAL(pcpu, i) + 1;
				PREV_MSR_VAL(pcpu, i) = 0;

				printk(KERN_WARNING "Rollover for C-State residency counter %u for CPU %u\n", i, cpu);
			}
		}

			/*
		rdmsrl(msr_addr, res);
		if(unlikely(res < PREV_MSR_VAL(pcpu, i))){
			 // W've had a rollover!
			PREV_MSR_VAL(pcpu, i) = (unsigned long long)(~0) - PREV_MSR_VAL(pcpu, i);
			res += PREV_MSR_VAL(pcpu, i) + 1;
			PREV_MSR_VAL(pcpu, i) = 0;
			OUTPUT(0, KERN_INFO "Error: Cpu %d has a ROLLOVER in %s\n", cpu, msr_names[i]);
		}
		*/
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

// DEBUGGING ONLY!
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
    int cpu = -1, pkg=-1;
    int last_PID = -1, last_TID = -1;
    unsigned long long last_IRQ = 0, last_TIMER = 0, last_SCHED = 0;
    unsigned char is_deferrable = 0;
    char sample_type = 'I';
    unsigned long long sample_data = 0;
    bool found = true, other_irq_diff = false;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    if(unlikely(!IS_SLEEP_MODE())){
	return;
    }

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
    pkg = 0;


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
    last_SCHED = pcpu->last_break[SCHED];
    is_deferrable = pcpu->is_deferrable;

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

    }else if( (sample_data = last_SCHED) > 0){
	sample_type = 'S';
	OUTPUT(3, KERN_INFO "SCHED BREAK!\n");
	printk(KERN_INFO "SCHED BREAK: APIC TIMER = %s!\n", GET_BOOL_STRING(other_irq_diff));
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
	found = false;
	if(false){
	    printk(KERN_INFO "[%d]: UNKNOWN! Other irq = %s\n", cpu, GET_BOOL_STRING(other_irq_diff));
	}
    }

    OUTPUT(3, KERN_INFO "[%d]: Break sample type = %c\n", cpu, sample_type);

    if(INTERNAL_STATE.write_to_buffers){
	produce_c_sample(cpu, pcpu, last_PID, last_TID, sample_type, sample_data);
    }

    pcpu->last_pid = pcpu->last_tid = -1;
    pcpu->last_break[IRQ] = pcpu->last_break[TIMER] = pcpu->last_break[SCHED] = 0;
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
	if(!SHOULD_TRACE())
		return;
    // tps_i(type, state);
    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
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

#if DO_IOCTL_STATS
	stats_t *pstats = NULL;
#endif

	if(unlikely(!IS_FREQ_MODE())){
		return;
	}

	if(!SHOULD_TRACE())
		return;

	/*
	 * Read TSC value
	 */
	tscval(&res);

	if(INTERNAL_STATE.write_to_buffers){
		produce_p_sample(cpu, res, state);
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
 * Helper function for "probe_sched_exit"
 * Useful for overhead measurements.
 */
static void exit_helper(struct task_struct *task)
{
	pid_t tid = task->pid;
	node_t *node = NULL;
	int idx = -1;
	// printk(KERN_INFO "[%d]: SCHED_EXIT\n", tid);

	for(node = find_timer_addr_for_tid_i(tid, &idx); node; node = find_timer_addr_for_tid_i(tid, &idx)){
		unsigned long timer_addr =  node->timer_addr;
		bool val = false;
		val = remove_entry(timer_addr, tid);
		OUTPUT(1, KERN_INFO "[%d]: 0x%lx, REMOVED = %s\n", tid, timer_addr, GET_BOOL_STRING(val));
	}

};

#if (KERNEL_VER < 35)
static void probe_sched_process_exit(struct task_struct *task)
#else
	static void probe_sched_process_exit(void *ignore, struct task_struct *task)
#endif
{
	pid_t tid = task->pid, pid = task->tgid;
	const char *name = task->comm;
	proc_node_t *node = NULL;

	if(!SHOULD_TRACE())
		return;

	OUTPUT(3, KERN_INFO "[%d, %d]: %s exitting\n", tid, pid, name);

	DO_PER_CPU_OVERHEAD_FUNC(exit_helper, task);

	/*
	 * Track task exits ONLY IF COLLECTION
	 * ONGOING!
	 */
	if(!IS_COLLECTING()){
		return;
	}

	if(!(node = pw_kmalloc(sizeof(proc_node_t), GFP_ATOMIC))){
		printk(KERN_INFO "ERROR: could NOT allocate memory in probe_sched_process_exit\n");
		return;
	}

	memset(node, 0, sizeof(proc_node_t));

	/*
	node->pid = pid;
	memcpy(node->name, name, sizeof(node->name));
	*/
	PROC_MAPPING_PID(node) = pid; PROC_MAPPING_TID(node) = tid;
	memcpy(PROC_MAPPING_NAME(node), name, MAX_PROC_NAME_SIZE);

	LOCK(proc_list_lock);
	{
		list_add_tail(&node->list, &proc_list);
		++total_num_proc_mappings;
	}
	UNLOCK(proc_list_lock);
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

    OUTPUT(0, KERN_INFO "Module %s LOADED! START = 0x%lx, SIZE = %lu\n", name, module_core, core_size);

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
		OUTPUT(0, KERN_INFO "\tSCHED_EXIT_EVENTS");
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

static int register_all_probes(void)
{
	int ret = 0;

#if (KERNEL_VER < 35)

	{
		OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
		ret = register_trace_timer_expire_entry(probe_timer_expire_entry);
		WARN_ON(ret);
		ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
		WARN_ON(ret);
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

#if 0
	{
		OUTPUT(0, KERN_INFO "\tSCHED_SWITCH_EVENTS");
		ret = register_trace_sched_switch(probe_sched_switch);
		WARN_ON(ret);
	}
#endif

#else

	{
		OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
		ret = register_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
		WARN_ON(ret);
		ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
		WARN_ON(ret);
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

#if 0
	{
		OUTPUT(0, KERN_INFO "\tSCHED_SWITCH_EVENTS");
		ret = register_trace_sched_switch(probe_sched_switch, NULL);
		WARN_ON(ret);
	}
#endif

#endif // KERNEL_VER

	return SUCCESS;
};

static void unregister_all_probes(void)
{
#if (KERNEL_VER < 35)

	{
		unregister_trace_timer_expire_entry(probe_timer_expire_entry);
		unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
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

#if 0
	{
		unregister_trace_sched_switch(probe_sched_switch);

		tracepoint_synchronize_unregister();
	}
#endif

#else

	{
		unregister_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
		unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
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

#if 0
	{
		unregister_trace_sched_switch(probe_sched_switch, NULL);

		tracepoint_synchronize_unregister();
	}
#endif
#endif // KERNEL_VER
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
 * Return a list of IRQ # <-> DEV name
 * mappings to user space.
 */
int get_irq_mappings(struct PWCollector_irq_mapping_block *remote_mappings, int size)
{
	struct PWCollector_irq_mapping_block local_mappings;
	int retVal = SUCCESS;
	int i=0, j=0, offset = -1, len = -1;
	irq_node_t *curr = NULL;

	if(copy_from_user(&offset, &remote_mappings->offset, sizeof(int))){ // "copy_from_user" returns number of bytes that COULD NOT be copied
		return -ERROR;
	}

	OUTPUT(3, KERN_INFO "OFFSET = %d\n", offset);

	memset(&local_mappings, 0, sizeof(local_mappings));

	if(false){
		OUTPUT(3, KERN_INFO "TOTAL # MAPPINGS = %d, IRQ LIST = %p\n", total_num_irq_mappings, irq_mappings_list);
	}

	if(offset < 0 || offset > total_num_irq_mappings){
		printk(KERN_INFO "ERROR: requested offset (%d) is INVALID! Total # irq mappings = %d\n", offset, total_num_irq_mappings);
		return -ERROR;
	}else if(offset == total_num_irq_mappings){
		/*
		 * We're done already.
		 */
		printk(KERN_INFO "WARNING: get irq mappings called extraneously!\n");
		if( (retVal = copy_to_user(remote_mappings, &local_mappings, size))) // returns number of bytes that COULD NOT be copied
			retVal = -ERROR;
		return retVal;
	}

	/*
	 * Initialize the 'mappings' list -- this
	 * needs to be done ONCE PER COLLECTION.
	 */
	if(!irq_mappings_list){
		irq_mappings_list = (PWCollector_irq_mapping_t *)pw_kmalloc(sizeof(PWCollector_irq_mapping_t) * total_num_irq_mappings, GFP_KERNEL);
		if(!irq_mappings_list){
			printk(KERN_INFO "ERROR: could NOT allocate space for mappings list!\n");
			return -ERROR;
		}
		memset(irq_mappings_list, 0, sizeof(PWCollector_irq_mapping_t) * total_num_irq_mappings);
		for(i=0, j=0; i<NUM_IRQ_MAP_BUCKETS && j < total_num_irq_mappings; ++i)
			for(curr = irq_map[i]; curr; curr = curr->next){
				PWCollector_irq_mapping_t *mapping = &irq_mappings_list[j++];
				mapping->irq_num = curr->irq;
				if(curr->name){
					len = strlen(curr->name);
					if(len > MAX_IRQ_NAME_SIZE)
						len = MAX_IRQ_NAME_SIZE;
					memcpy(mapping->irq_name, curr->name, len);
				}
			}
	}

	OUTPUT(3, KERN_INFO "Curr idx = %d\n", offset);

	len = total_num_irq_mappings - offset;
	if(len > MAX_NUM_IRQ_MAPPINGS_PER_BLOCK){
		len = MAX_NUM_IRQ_MAPPINGS_PER_BLOCK;
		retVal = 1; // > 0 ==> SUCCESS, NOT EOF
	}else{
		retVal = 0; // == 0 ==> SUCCESS, EOF
	}

	OUTPUT(3, KERN_INFO "LEN = %d, new offset = %d\n", len, offset+len);

	memcpy(local_mappings.mappings, &irq_mappings_list[offset], sizeof(PWCollector_irq_mapping_t) * len);

	offset += len;

	local_mappings.size = len; local_mappings.offset = offset;

	if(copy_to_user(remote_mappings, &local_mappings, size)) // returns number of bytes that COULD NOT be copied
		retVal = -ERROR; // < 0 ==> ERROR

	return retVal;
};

/*
 * Return a list of PID <-> PROC name
 * mappings (for those processes that
 * exitted during the collection)
 * to user space.
 */
int get_proc_mappings(struct PWCollector_proc_mapping_block *remote_mappings, int size)
{
	struct PWCollector_proc_mapping_block local_mappings;
	int i=0, offset = -1, len = -1;
	int retVal = SUCCESS;
	proc_node_t *node;
	PWCollector_proc_mapping_t *curr = NULL;

	if(IS_COLLECTING()){
		printk(KERN_INFO "ERROR: CANNOT get proc mappings for an ongoing collection!\n");
		return -ERROR;
	}

	if(copy_from_user(&offset, &remote_mappings->offset, sizeof(int))){ // "copy_from_user" returns number of bytes that COULD NOT be copied
		return -ERROR;
	}
	memset(&local_mappings, 0, sizeof(local_mappings));

	OUTPUT(3, KERN_INFO "IN offset = %d, Total # mappings = %d, proc_mappings_list = %p\n", offset, total_num_proc_mappings, proc_mappings_list);

	if(offset < 0 || offset > total_num_proc_mappings){
		printk(KERN_INFO "ERROR: requested offset (%d) is INVALID! Total # proc mappings = %d\n", offset, total_num_proc_mappings);
		return -ERROR;
	}else if(offset == total_num_proc_mappings){
		/*
		 * We're done already.
		 */
		printk(KERN_INFO "WARNING: get proc mappings called extraneously!\n");
		if( (retVal = copy_to_user(remote_mappings, &local_mappings, size))) // returns number of bytes that COULD NOT be copied
			retVal = -ERROR;
		return retVal;
	}

	if(!proc_mappings_list){
		if(!(proc_mappings_list = pw_kmalloc(sizeof(PWCollector_proc_mapping_t) * total_num_proc_mappings, GFP_ATOMIC))){
			printk(KERN_INFO "ERROR: could NOT allocate space for mappings list!\n");
			return -ERROR;
		}
		memset(proc_mappings_list, 0, sizeof(PWCollector_proc_mapping_t) * total_num_proc_mappings);
		i=0;
		LOCK(proc_list_lock);
		{
			list_for_each_entry(node, &proc_list, list){
				curr = &proc_mappings_list[i];
				++i;
				memcpy(curr, &node->mapping, sizeof(PWCollector_proc_mapping_t));
			}
		}
		UNLOCK(proc_list_lock);
	}

	len = total_num_proc_mappings - offset;
	if(len > MAX_NUM_PROC_MAPPINGS_PER_BLOCK){
		len = MAX_NUM_PROC_MAPPINGS_PER_BLOCK;
		retVal = 1; // > 0 ==> SUCCESS, NOT EOF
	}else{
		retVal = 0; // == 0 ==> SUCCESS, EOF
	}

	OUTPUT(3, KERN_INFO "LEN = %d, new offset = %d\n", len, offset+len);

	memcpy(local_mappings.mappings, &proc_mappings_list[offset], sizeof(PWCollector_proc_mapping_t) * len);

	offset += len;

	local_mappings.size = len; local_mappings.offset = offset;

	if(copy_to_user(remote_mappings, &local_mappings, size)) // returns number of bytes that COULD NOT be copied
		retVal = -ERROR; // < 0 ==> ERROR

	return retVal;
	// return -ERROR;
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
	local_samples = pw_kmalloc(max_size, GFP_KERNEL);
	if(!local_samples){
		printk(KERN_INFO "ERROR: Could NOT allocate memory for sample!\n");
		// atomic_set(&should_panic, 1);
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
	pw_kfree(local_samples);

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
		case RESUME: // fall through
			break;
		default: // should *NEVER* happen!
			printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
			return -ERROR;
	}

	/*
	 * Need to reset proc mappings
	 * list BEFORE setting "IS_COLLECTING()"
	 * to true.
	 */
	{
		destroy_proc_list();
		init_proc_list(); // NOP
	}
	INTERNAL_STATE.collectionStartJIFF = jiffies;
	INTERNAL_STATE.write_to_buffers = true;

	// Reset the (per-cpu) output buffers.
	{
		reset_buffers();
	}

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
	/*
	 * Reset the list of vars required
	 * to transfer IRQ # <-> Name info.
	 */
	if(irq_mappings_list){
		pw_kfree(irq_mappings_list);
		irq_mappings_list = NULL;
	}
#endif


#if DO_IOCTL_STATS
	reset_statistics();
#endif

	{
		register_all_probes();
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
		unregister_all_probes();
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
	if(EXTRACT_LOCAL_ARGS(&local_args, user_args)){ // "copy_from_user" returns number of bytes that COULD NOT BE COPIED
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
		case PW_IOCTL_MICRO_PATCH:
			OUTPUT(0, KERN_INFO "PW_IOCTL_MICRO_PATCH\n");
			if( (retVal = get_micro_patch_ver((int *)user_args->out_arg, local_args.out_len))){
				printk(KERN_INFO "\tError retrieving microcode patch version info!\n");
				return -ERROR;
			}
			return SUCCESS;
		case PW_IOCTL_IRQ_MAPPINGS:
			OUTPUT(0, KERN_INFO "PW_IOCTL_IRQ_MAPPINGS\n");
			if( (retVal = get_irq_mappings((struct PWCollector_irq_mapping_block *)user_args->out_arg, local_args.out_len)))
				return retVal;
			break;
		case PW_IOCTL_PROC_MAPPINGS:
			OUTPUT(0, KERN_INFO "PW_IOCTL_PROC_MAPPINGS\n");
			if( (retVal = get_proc_mappings((struct PWCollector_proc_mapping_block *)user_args->out_arg, local_args.out_len)))
				return retVal;
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

static int __init init_hooks(void)
{
	int ret = 0;

	printk(KERN_INFO "# IRQS = %d\n", NR_IRQS);

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
		micro_patch_ver = patch_val;
		OUTPUT(3, KERN_INFO "patch ver = %u\n", micro_patch_ver);
	}
#endif

	printk(KERN_INFO "Sizeof node = %d\n", sizeof(node_t));

	startJIFF = jiffies;

	if(init_data_structures())
		return -ERROR;

	{
		enable_ref();
	}

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

	/*
	 * Probes required to cache (kernel) timer
	 * callstacks need to be removed, regardless
	 * of collection status.
	 */
	{
		unregister_timer_callstack_probes();
	}

#if 1
	if(IS_COLLECTING()){
		unregister_all_probes();
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
	printk(KERN_INFO "Total time elapsed = %u msecs\n", jiffies_to_msecs(elapsedJIFF));

	printk(KERN_INFO "Total # timers = %d, Total # irq mappings = %d\n", num_timers, num_irqs);

#if DO_OVERHEAD_MEASUREMENTS
	{
		timer_init_print_cumulative_overhead_params("TIMER_INIT");
		timer_expire_print_cumulative_overhead_params("TIMER_EXPIRE");
		tps_print_cumulative_overhead_params("TPS");
		inter_common_print_cumulative_overhead_params("INTER_COMMON");
		irq_insert_print_cumulative_overhead_params("IRQ_INSERT");
		find_irq_node_i_print_cumulative_overhead_params("FIND_IRQ_NODE_I");
		exit_helper_print_cumulative_overhead_params("EXIT_HELPER");
	}
#endif // DO_OVERHEAD_MEASUREMENTS

	printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");

};

module_init(init_hooks);
module_exit(cleanup_hooks);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
