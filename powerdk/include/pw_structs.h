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

/*
 * Description: file containing data structures used by the
 * power driver.
 */

#ifndef _DATA_STRUCTURES_H_
#define _DATA_STRUCTURES_H_ 1

#ifndef __KERNEL__
/*
 * Called from Ring-3.
 */
#include <stdint.h> // Grab 'uint64_t' etc.
/*
 * UNSIGNED types...
 */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
/*
 * SIGNED types...
 */
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#endif // __KERNEL__

/*
 * Should we probe on syscall enters and exits?
 * We require this functionality to handle certain
 * device-driver related timers.
 * ********************************************************
 * WARNING: SETTING TO 1 will INVOLVE HIGH OVERHEAD!!!
 * ********************************************************
 */
#define DO_PROBE_ON_SYSCALL_ENTER_EXIT 1
#define DO_PROBE_ON_EXEC_SYSCALL DO_PROBE_ON_SYSCALL_ENTER_EXIT
/*
 * Do we use an RCU-based mechanism
 * to determine which output buffers
 * to write to?
 * Set to: "1" ==> YES
 *         "0" ==> NO
 * ************************************
 * CAUTION: RCU-based output buffer
 * selection is EXPERIMENTAL ONLY!!!
 * ************************************
 */
#define DO_RCU_OUTPUT_BUFFERS 0
/*
 * Do we force the device driver to
 * (periodically) flush its buffers?
 * Set to: "1" ==> YES
 *       : "0" ==> NO
 * ***********************************
 * UPDATE: This value is now tied to the
 * 'DO_RCU_OUTPUT_BUFFERS' flag value
 * because, for proper implementations
 * of buffer flushing, we MUST have
 * an RCU-synchronized output buffering
 * mechanism!!!
 * ***********************************
 */
#define DO_PERIODIC_BUFFER_FLUSH DO_RCU_OUTPUT_BUFFERS
/*
 * Do we use a TPS "epoch" counter to try and
 * order SCHED_WAKEUP samples and TPS samples?
 * (Required on many-core architectures that don't have
 * a synchronized TSC).
 */
#define DO_TPS_EPOCH_COUNTER 1
/*
 * Should the driver count number of dropped samples?
 */
#define DO_COUNT_DROPPED_SAMPLES 1


#define NUM_SAMPLES_PER_SEG 512
// #define NUM_SAMPLES_PER_SEG 1024
#define SEG_SIZE (NUM_SAMPLES_PER_SEG * sizeof(PWCollector_sample_t))

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

/*
 * MSR counter stuff.
 *
 * Ultimately the list of MSRs to read (and the core MSR residency addresses)
 * will be specified by the "runss" tool (via the "PW_IOCTL_CONFIG" ioctl).
 *
 * For now, hardcoded to values for NHM.
 */
enum{
    MPERF=0, // C0
    APERF, // C1
    C2,
    C3,
    C4,
    C5,
    C6,
    C7,
    C8,
    C9,
    /* C10, */
    /* C11, */
    MAX_MSR_ADDRESSES
};

/*
 * Enumeration of possible sample types.
 */
typedef enum {
    FREE_SAMPLE=0, /* Used (internally) to indicate a FREE entry */
    C_STATE, /* Used for c-state samples */
    P_STATE, /* Used for p-state samples */
    K_CALL_STACK, /* Used for kernel-space call trace entries */
    M_MAP, /* Used for module map info samples */
    IRQ_MAP, /* Used for IRQ # <-> DEV name mapping samples */
    PROC_MAP, /* Used for PID <-> PROC name mapping samples */
    S_RESIDENCY, /* Used for S residency counter samples */
    S_STATE, /* Used for S state samples */
    D_RESIDENCY, /* Used for D residency counter samples */
    D_STATE, /* Used for D state samples in north or south complex */
    TIMER_SAMPLE,
    IRQ_SAMPLE,
    WORKQUEUE_SAMPLE,
    SCHED_SAMPLE,
    IPI_SAMPLE,
    TPE_SAMPLE, /*  Used for 'trace_power_end' samples */
    W_STATE, /* Used for wakelock samples */
    SAMPLE_TYPE_END
} sample_type_t;

/*
 * Enumeration of possible C-state sample
 * types.
 */
/*
typedef enum{
    I=0, // interrupt
    D, // deferred timer
    N, // non-deferred timer
    U // unknown
}c_break_type_t;
*/

typedef enum{
    PW_BREAK_TYPE_I=0, // interrupt
    PW_BREAK_TYPE_T, // timer
    PW_BREAK_TYPE_S, // sched-switch : THIS IS DEPRECATED -- DO NOT USE!
    PW_BREAK_TYPE_IPI, // {LOC, RES, CALL, TLB}
    PW_BREAK_TYPE_W, // workqueue
    PW_BREAK_TYPE_B, // begin
    PW_BREAK_TYPE_U // unknown
}c_break_type_t;



/*
 * Structure used to encode C-state sample information.
 */
typedef struct {
    u16 break_type; // instance of 'c_break_type_t'
    u16 prev_state; // "HINT" parameter passed to TPS probe
    pid_t pid; // PID of process which caused the C-state break.
    pid_t tid; // TID of process which caused the C-state break.
    u32 tps_epoch; // Used to sync with SCHED_SAMPLE events
    /*
     * "c_data" is one of the following:
     * (1) If "c_type" == 'I' ==> "c_data" is the IRQ of the interrupt
     * that caused the C-state break.
     * (2) If "c_type" == 'D' || 'N' => "c_data" is the TSC that maps to the
     * user-space call trace ID for the process which caused the C-state break.
     * (3) If "c_type" == 'U' ==> "c_data" is undefined.
     */
    u64 c_data;
    u64 c_state_res_counts[MAX_MSR_ADDRESSES];
} c_sample_t;

#define RES_COUNT(s,i) ( (s).c_state_res_counts[(i)] )

/*
 * Structure used to encode P-state transition information.
 *
 * UPDATE: For TURBO: for now, we only encode WHETHER the CPU is
 * about to TURBO-up; we don't include information on WHICH Turbo
 * frequency the CPU will execute at. See comments in struct below
 * for an explanation on why the 'frequency' field values are
 * unreliable in TURBO mode.
 */
typedef struct {
    /*
     * Field to encode the frequency
     * the CPU was ACTUALLY executing
     * at DURING THE PREVIOUS
     * P-QUANTUM.
     */
    u32 frequency;
    /*
     * Field to encode the frequency
     * the OS requested DURING THE
     * PREVIOUS P-QUANTUM.
     */
    u32 prev_req_frequency;
    /*
     * We encode the frequency at the start
     * and end of a collection in 'boundary'
     * messages. This flag is set for such
     * messages.
     */
    u32 is_boundary_sample;
    u32 padding;
    /*
     * The APERF and MPERF values.
     */
    u64 unhalted_core_value, unhalted_ref_value;
} p_sample_t;

/*
 * The MAX number of entries in the "trace" array of the "k_sample_t" structure.
 * If the actual backtrace is longer, multiple
 * k_sample structs need to be chained together (see "sample_len" in
 * the "sample" struct).
 * This value is carefully chosen to ensure the PWCollector_sample
 * struct size is 128 bytes.
 */
// #define TRACE_LEN 23
#define TRACE_LEN 11

/*
 * Structure used to encode kernel-space call trace information.
 */
typedef struct {
    /*
     * "trace_len" indicates the number of entries in the "trace" array.
     * Note that the actual backtrace may be larger -- in which case the "sample_len"
     * field of the enclosing "struct PWCollector_sample" will be greater than 1.
     */
    u32 trace_len;
    /*
     * We can have root timers with non-zero tids.
     * Account for that possibility here.
     */
    pid_t tid;
    /*
     * The entry and exit TSC values for this kernel call stack.
     * MUST be equal to "[PWCollector_sample.tsc - 1, PWCollector_sample.tsc + 1]" respectively!
     */
    u64 entry_tsc, exit_tsc;
    /*
     * "trace" contains the kernel-space call trace.
     * Individual entries in the trace correspond to the various
     * return addresses in the call trace, shallowest address first.
     * For example: if trace is: "0x10 0x20 0x30 0x40" then
     * the current function has a return address of 0x10, its calling function
     * has a return address of 0x20 etc.
     */
    u64 trace[TRACE_LEN];
} k_sample_t;

/*
 * Max size of a module name. Ideally we'd
 * like to directly include "module.h" (which
 * defines this value), but this code
 * will be shared with Ring-3 code, which is
 * why we redefine it here.
 */
#define PW_MODULE_NAME_LEN (64 - sizeof(unsigned long))

/*
 * Structure used to encode kernel-module map information.
 */
typedef struct{
    /*
     * Offset of current chunk, in case a kernel module is
     * mapped in chunks. DEFAULTS TO ZERO!
     */
    u32 offset;
    /*
     * Compiler would have auto-padded this for us, but
     * we make that padding explicit just in case.
     */
    u32 padding_64b;
    /*
     * The starting addr (in HEX) for this module.
     */
    u64 start;
    /*
     * The ending addr (in HEX) for this module.
     */
    u64 end;
    /*
     * Module NAME. Note that this is NOT the full
     * path name. There currently exists no way
     * of extracting path names from the module
     * structure.
     */
    char name[PW_MODULE_NAME_LEN];
}m_sample_t;

/*
 * MAX size of each irq name (bytes).
 */
#define PW_IRQ_DEV_NAME_LEN	100

/*
 * Structure used to encode IRQ # <-> DEV name
 * mapping information.
 */
typedef struct{
    /*
     * The IRQ #
     */
    int irq_num;
    /*
     * Device name corresponding
     * to 'irq_num'
     */
    char irq_name[PW_IRQ_DEV_NAME_LEN];
}i_sample_t;

/*
 * MAX size of each proc name.
 */
// #define MAX_PROC_NAME_SIZE TASK_COMM_LEN
#define PW_MAX_PROC_NAME_SIZE 16

/*
 * MAX number of logical subsystems in south complex.
 */
#define MAX_LSS_NUM_IN_SC 40

/*
 * MAX number of logical subsystems in north complex.
 */
#define MAX_LSS_NUM_IN_NC 10

/*
 * MAX size of each wakelock name.
 */
#define PW_MAX_WAKELOCK_NAME_SIZE 84


/*
 * The 'type' of the associated
 * 'r_sample'.
 */
typedef enum {
    PW_PROC_FORK, /* Sample encodes a process FORK */
    PW_PROC_EXIT, /* Sample encodes a process EXIT */
    PW_PROC_EXEC /* Sample encodes an EXECVE system call */
} r_sample_type_t;

typedef struct{
    u32 type;
    pid_t tid, pid;
    char proc_name[PW_MAX_PROC_NAME_SIZE];
}r_sample_t;

/*
 * Platform state (a.k.a. S state) residency counter sample
 */
typedef struct{
    u32 usec; // S0 residency = usec - S0i1 - S0i2 - S0i3
    u32 counters[3];  // S0i1, S0i2, S0i3 in order
}s_residency_sample_t;

/*
 * Platform state (a.k.a. S state) sample
 */
typedef struct{
    u32 state; // S-state
}s_state_sample_t;

/*
 * The 'type' of the associated
 * 'd_residency_sample'.
 */
typedef enum{
    PW_NORTH_COMPLEX, // North complex
    PW_SOUTH_COMPLEX  // South complex
}device_type_t;


typedef struct{
    u32 usec;     // D0 residency = usec - D0i0_ACG - D0i1 - D0i3
    u32 D0i0_ACG;
    u32 D0i1;
    u32 D0i3;
}d_residency_t;

/*
 * Device state (a.k.a. D state) residency counter sample
 */
typedef struct{
    char device_type;     // Either NORTH_COMPLEX or SOUTH_COMPLEX
    char num_sampled;
    char mask[6];  // Each bit indicates whether LSS residency is counted or not.
                   // 1 means "counted", 0 means "not counted"
                   // The last byte indicates the number of LSSes sampled
    d_residency_t d_residency_counters[6];  // Limit the number of d_residency data to 128byte which is 2 cache lines
}d_residency_sample_t;

/*
 * Device state (a.k.a. D state) sample from north or south complex
 */
typedef struct{
    char device_type;     // Either NORTH_COMPLEX or SOUTH_COMPLEX
    u32 states[4]; // Each device state is represented in 2 bits
}d_state_sample_t;

/*
 * The 'type' of the associated
 * 'w_sample'.
 */
typedef enum{
    PW_WAKE_LOCK, // Wake lock
    PW_WAKE_UNLOCK // Wake unlock
}w_sample_type_t;

/*
 * Wakelock sample
 */
typedef struct{
    w_sample_type_t type;   // Either WAKE_LOCK or WAKE_UNLOCK
    pid_t tid, pid;
    char name[PW_MAX_WAKELOCK_NAME_SIZE]; // Wakelock name
    char proc_name[PW_MAX_PROC_NAME_SIZE]; // process name
}w_sample_t;

/*
 * Generic "event" sample.
 */
typedef struct {
    u64 data[4];
} event_sample_t;

/*
 * The C/P/K/S sample structure.
 */
typedef struct PWCollector_sample{
    u32 cpuidx;
    u16 sample_type; // The type of the sample: one of "sample_type_t"
    /*
     * "sample_len" is useful when stitching together
     * multiple PWCollector_sample instances.
     * This is used in cases where the kernel-space call
     * trace is very large, and cannot fit within one K-sample.
     * We can stitch together a MAX of
     * 256 K-samples.
     */
    u16 sample_len;
    u64 tsc; // The TSC at which the measurement was taken
    union {
	c_sample_t c_sample;
	p_sample_t p_sample;
	k_sample_t k_sample;
	m_sample_t m_sample;
	i_sample_t i_sample;
	r_sample_t r_sample;
	s_residency_sample_t s_residency_sample;
	s_state_sample_t s_state_sample;
	d_state_sample_t d_state_sample;
	d_residency_sample_t d_residency_sample;
	w_sample_t w_sample;
	event_sample_t e_sample;
    };
}PWCollector_sample_t;


typedef enum{
    START=1,
    DETACH,
    PAUSE,
    RESUME,
    STOP,
    CANCEL,
    SNAPSHOT,
    STATUS,
    MARK
}PWCollector_cmd_t;

/*
 * "Collect-Power" switches.
 * Note: different from interface spec:
 * We're moving from bitwise OR to bitwise OR of (1 << switch) values.
 */
typedef enum {
    // SLEEP=1,
    SLEEP=0,
    KTIMER,
    FREQ,
    COMPONENT,
    SYSTEM,
    PLATFORM_RESIDENCY,
    PLATFORM_STATE,
    DEVICE_SC_RESIDENCY,
    DEVICE_NC_STATE,
    DEVICE_SC_STATE,
    WAKELOCK_STATE,
    POWER_C_STATE,
    MAX_POWER_DATA_MASK
} power_data_t;

#define POWER_SLEEP_MASK (1 << SLEEP)
#define POWER_KTIMER_MASK (1 << KTIMER)
#define POWER_FREQ_MASK (1 << FREQ)
#define POWER_COMPONENT_MASK (1 << COMPONENT)
#define POWER_SYSTEM_MASK (1 << SYSTEM)
#define POWER_S_RESIDENCY_MASK (1 << PLATFORM_RESIDENCY)
#define POWER_S_STATE_MASK (1 << PLATFORM_STATE)
#define POWER_D_SC_RESIDENCY_MASK (1 << DEVICE_SC_RESIDENCY)
#define POWER_D_SC_STATE_MASK (1 << DEVICE_SC_STATE)
#define POWER_D_NC_STATE_MASK (1 << DEVICE_NC_STATE)
#define POWER_WAKELOCK_MASK (1 << WAKELOCK_STATE)
#define POWER_C_STATE_MASK ( 1 << POWER_C_STATE )

/*
 * Platform-specific config struct.
 */
typedef struct{
    int residency_count_multiplier;
    int bus_clock_freq_khz;
    int coreResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    int pkgResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    u64 reserved[3];
}platform_info_t;

/*
 * Config Structure. Includes platform-specific
 * stuff and power switches.
 */
struct PWCollector_config{
    int data;
    u32 d_state_sample_interval;  // This is the knob to control the frequency of D-state data sampling
    // to adjust their collection overhead in the unit of msec.
    platform_info_t info;
};

/*
 * Some constants used to describe kernel features
 * available to the power driver.
 */
#define PW_KERNEL_SUPPORTS_CALL_STACKS (1 << 0)
#define PW_KERNEL_SUPPORTS_CONFIG_TIMER_STATS (1 << 1)
#define PW_KERNEL_SUPPORTS_WAKELOCK_PATCH (1 << 2)

/*
 * Structure to encode unsupported tracepoints and
 * kernel features that enable power collection.
 */
struct PWCollector_check_platform {
    char unsupported_tracepoints[4096];
    /*
     * Bitwise 'OR' of zero or more of the
     * 'PW_KERNEL_SUPPORTS_' constants described
     * above.
     */
    u64 supported_features;
    u64 reserved[3];
};

/*
 * Structure to return status information.
 */
struct PWCollector_status{
    u32 num_cpus;
    u32 time;
    u32 c_breaks;
    u32 timer_c_breaks;
    u32 inters_c_breaks;
    u32 p_trans;
    u32 num_inters;
    u32 num_timers;
};

/*
 * Structure to return version information.
 */
struct PWCollector_version_info{
    int version;
    int interface;
    int other;
};

/*
 * Structure to return specific microcode
 * patch -- for MFLD development steppings.
 */
struct PWCollector_micro_patch_info{
    u32 patch_version;
};

/*
 * MAX number of mappings per block.
 */
#define PW_MAX_NUM_IRQ_MAPPINGS_PER_BLOCK 16
/*
 * MAX size of each irq name (bytes).
 */
#define PW_MAX_IRQ_NAME_SIZE 32
/*
 * Helper struct for IRQ <-> DEV name mappings.
 */
typedef struct {
	int irq_num;
	char irq_name[PW_MAX_IRQ_NAME_SIZE];
}PWCollector_irq_mapping_t;
/*
 * Structure to return IRQ <-> DEV name mappings.
 */
struct PWCollector_irq_mapping_block{
	/*
	 * INPUT param: if >= 0 ==> indicates
	 * the client wants information for
	 * a SPECIFIC IRQ (and does not want
	 * ALL mappings).
	 */
	int requested_irq_num;
	/*
	 * OUTPUT param: records number of
	 * valid entries in the 'mappings'
	 * array.
	 */
	int size;
	/*
	 * INPUT/OUTPUT param: records from which IRQ
	 * entry the client wants mapping info.
	 * Required because the driver
	 * may return LESS than the total number
	 * of IRQ mappings, in which case the client
	 * is expected to call this IOCTL
	 * again, specifying the offset.
	 */
	int offset;
	/*
	 * The array of mappings.
	 */
	PWCollector_irq_mapping_t mappings[PW_MAX_NUM_IRQ_MAPPINGS_PER_BLOCK];
};

/*
 * MAX number of mappings per block.
 */
#define PW_MAX_NUM_PROC_MAPPINGS_PER_BLOCK 32

typedef struct{
	pid_t pid, tid;
	char name[PW_MAX_PROC_NAME_SIZE];
}PWCollector_proc_mapping_t;
/*
 * Structure to return PID <-> PROC name mappings.
 */
struct PWCollector_proc_mapping_block{
	/*
	 * OUTPUT param: records number of
	 * valid entries in the 'mappings'
	 * array.
	 */
	int size;
	/*
	 * INPUT/OUTPUT param: records from which PROC
	 * entry the client wants mapping info.
	 * may return LESS than the total number
	 * of PROC mappings, in which case the client
	 * is expected to call this IOCTL
	 * again, specifying the offset.
	 */
	int offset;
	/*
	 * The array of mappings.
	 */
	PWCollector_proc_mapping_t mappings[PW_MAX_NUM_PROC_MAPPINGS_PER_BLOCK];
};

/*
 * Structure to return TURBO frequency
 * threshold.
 */
struct PWCollector_turbo_threshold{
	u32 threshold_frequency;
};

/*
 * Max # of available frequencies.
 */
#define PW_MAX_NUM_AVAILABLE_FREQUENCIES 16 // should be enough!
/*
 * Structure to return 'available
 * frequencies' i.e. the list of
 * frequencies the processor
 * may execute at.
 */
struct PWCollector_available_frequencies{
	/*
	 * Number of valid entries in the
	 * 'frequencies' array -- supplied
	 * by the DD.
	 */
	u32 num_freqs;
	/*
	 * List of available frequencies, in kHz.
	 */
	u32 frequencies[PW_MAX_NUM_AVAILABLE_FREQUENCIES];
};

/*
 * Wrapper for ioctl arguments.
 * EVERY ioctl MUST use this struct!
 */
struct PWCollector_ioctl_arg{
    int in_len;
    int out_len;
    char *in_arg;
    char *out_arg;
};

#endif // _DATA_STRUCTURES_H_
