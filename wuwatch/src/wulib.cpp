/*****************************************************************************
#                      INTEL CONFIDENTIAL
#
# Copyright 2009-2012 Intel Corporation All Rights Reserved.
#
# This file is provided for internal use only. You are free to modify it
# for your own internal purposes, but you cannot release it to the external
# open source community under any circumstances without SSG.DPD's approval.
#
# If you make significant improvements, fix bugs, or have suggestions for
# additional features or improvements, please provide your changes to
# Robert or Gautam for inclusion in this tool.
#
# Please contact Robert Knight (robert.knight@intel.com) or Gautam Upadhyaya
# (gautam.upadhyaya@intel.com) if you have any questions.
#
*****************************************************************************
*/

/* ******************************************
 * The HT-aware power library.
 * Contains code to read/parse/correlate raw
 * power driver information.
 * ******************************************
 */

#include <stdio.h>
#include <fcntl.h>		/* open */
#include <unistd.h>		/* exit */
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <dirent.h>
#include <errno.h>
//#include <sys/un.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <assert.h>
#include <sys/utsname.h>

#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <list>
#include <deque>
#include <iterator>
#include <algorithm> // for "std::sort"
#include <sstream>

#include "pw_ioctl.h" // IOCTL stuff.
#include "pw_arch.h" // Architecture info.
#include "pw_bt.h"
#include "pw_utils.hpp"
#include "ksym_extractor.hpp"
#include "defines.h"
#include "wulib_defines.h"
#include "wulib.h"

/* *****************************************
 * Debugging tools.
 * *****************************************
 */
extern bool g_do_debugging;
#define db_fprintf(...) do { \
    if (g_do_debugging) { \
        fprintf(__VA_ARGS__); \
    } \
} while(0)
#define db_assert(e, ...) do { \
    if (g_do_debugging && !(e)) { \
	    fprintf(stderr, __VA_ARGS__);	\
	    assert(false);			\
	}					\
} while(0)
#define db_abort(...) do { \
    if (g_do_debugging) { \
        fprintf(stderr, __VA_ARGS__); \
        assert(false); \
    } \
} while(0)
#define db_copy(...) do { \
    if (g_do_debugging) { \
        std::copy(__VA_ARGS__); \
    } \
} while(0)

/* *****************************************
 * Forward declarations.
 * *****************************************
 */
namespace pwr {
    class WuParser;
    struct CGroup;
}

/* *****************************************
 * Some useful typedefs.
 * *****************************************
 */
typedef std::list<PWCollector_sample_t> sample_list_t;
typedef std::vector<std::string> str_vec_t;
typedef std::deque<std::string> str_deq_t;
typedef std::string str_t;
typedef std::vector<FILE *> fp_vec_t;
typedef fp_vec_t::iterator fp_iter_t;
typedef std::vector <int> int_vec_t;

typedef std::pair <int,int> int_pair_t;
typedef std::vector <int_pair_t> pair_vec_t;
typedef std::vector <pwr::CGroup> c_group_vec_t;

typedef std::pair <uint64_t, uint64_t> u64_pair_t;
typedef std::list <u64_pair_t> p_sample_list_t;
typedef std::map <int, p_sample_list_t> p_sample_map_t;
typedef std::map <int, u64_pair_t> core_tsc_map_t;
typedef std::map <int, std::pair <u64, u64> > aperf_mperf_map_t;

/* *****************************************
 * Data structure definitions.
 * *****************************************
 */

/*
 * Template specializations to allow
 * us to 'std::list<>::merge(...)' etc.
 */
namespace std {
    template <> struct less<PWCollector_sample_t> {
        bool operator()(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2) {
            return s1.tsc < s2.tsc;
        };
    };
};


namespace pwr {
    /*
     * Helper struct for TPS group formation.
     */
    struct CGroup {
        int_pair_t group;
        const PWCollector_sample_t *wakeup_sample;

        CGroup(int b, int e);
    };


    /*
     * Parser class -- reads wuwatch data, parses it and converts v3 pwr samples to v2 format.
     */
    class WuParser {
        private:
            /*
             * The combined output file name. Under new
             * scheme, both driver output and sys params
             * info gets dumped into a single file (with
             * a ".ww<X>" extension).
             */
            std::string m_combined_input_file_name;
            /*
             * Directory used to store descendent pid information.
             */
            std::string m_wuwatch_output_dir;
            /*
             * Where should we store intermediate output?
             */
            std::string m_output_file_name;
            /*
             * Where should we read the PRELOAD lib
             * results?
             */
            fp_vec_t m_lib_input_fps;
            /*
             * A list of valid frequencies. We will eventually
             * get this either from the driver, or from
             * the "sys" file system. For now, default to MFLD
             * values.
             * **********************************************
             * WARNING: LAST ENTRY IN ARRAY MUST BE ZERO!!!
             * **********************************************
             */
            std::vector <u32> m_availableFrequenciesKHz;
            /*
             * # cpus on TARGET machine
             * i.e. on machine where experiment
             * was conducted.
             */
            int PW_max_num_cpus;
            /*
             * List of PWCollector_sample_t instances
             * output by the DD, organized by (logical) CPU.
             */
            sample_list_t *m_per_cpu_sample_lists;
            /*
             * List of P-state sample instances output by the DD, organized
             * by (logical) CPU.
             */
            sample_list_t *m_per_cpu_p_sample_lists;
            /*
             * List of SCHED_SAMPLE PWCollector_sample_t instances
             * output by the DD, organized by the (logical)
             * CPU that was targeted by the "sched_wakeup" call.
             * Only valid if we're using the "EPOCH" based
             * TPS <-> SCHED_WAKEUP syncing mechanism.
             */
#if DO_TPS_EPOCH_COUNTER
            sample_list_t *m_per_cpu_sched_sample_lists;
#endif // DO_TPS_EPOCH_COUNTER
            /*
             * Map to hold tid <-> backtrace information.
             * This info is generated by the hook library,
             * and by the kernel (for kernel call stacks).
             */
            trace_pair_map_t m_trace_pair_map;
            /*
             * Convenience vector to hold dynamically allocated
             * (user-space) back traces. Storing those traces
             * here allows us to (quickly) track and deallocate
             * them when the time comes.
             */
            trace_vec_t m_trace_vec;
            /*
             * Map to hold the FIRST and LAST TSCs of each core. Basically
             * the first sample that we see and the last sample that we
             * see for a particular core. We need this to generate
             * begin and end 'Pure C0' samples and also to adjust the
             * boundary P-state sample TSC values.
             */
            core_tsc_map_t m_core_tsc_map;
            /*
             * A list of samples that do NOT need to be post-processed.
             * These samples are returned directly to the caller.
             */
            sample_list_t m_all_samples_list;
            /*
             * A copy of ALL samples returned by the driver. Used ONLY
             * if we want to dump original samples.
             */
            sample_list_t *m_all_orig_samples;
            /*
             * List of samples to output. We have one
             * list for each CORE.
             * (For now -- assume single-core ala MFLD).
             */
            sample_list_t m_output_samples;
            /*
             * List of P-state samples to output.
             * list for each CORE.
             * (For now -- assume single-core ala MFLD).
             */
            sample_list_t m_output_p_samples;
            /*
             * Flag to indicate whether
             * we should dump backtrace
             * information.
             */
            bool m_do_dump_backtraces;
            /*
             * Should we dump TPS groups?
             */
            bool m_do_dump_tps_groups;
            /*
             * Should we try and do a basic sanity test?
             * This test involves summing up all of the
             * C0, C1, C2 ... residencies and ensuring
             * this sum is equal to the difference in
             * the TSC values of the last C-state sample
             * and the first C-state sample i.e. ensure
             * Sigma(Cx, 0<=x<=9) == TSC(last C-state sample) - TSC (first C-state sample)
             */
            bool m_do_cx_c0_sanity_test;
            /*
             * Should we dump original samples i.e. samples as returned by
             * the driver (without any post-processing other than basic
             * sorting wrt tsc)?
             */
            bool m_do_dump_orig_samples;
            /*
             * Should we try and determine C1 residencies?
             */
            bool m_do_check_c1_res;
            /*
             * Did we detect ANY TPF samples? We use this
             * in lieu of a concrete list of options the
             * user passed to the wuwatch collector. Later, we'll
             * probably add just such a list to the
             * 'sys_params_found' section.
             */
            bool m_was_any_tpf_sample_present;
            /*
             * We need to keep track of the total collection time. This is merely
             * the TSC of the last C-state sample - TSC of the first core mwait.
             * We need a bit of extra logic because the collection time is really
             * the MAX of the collection times of the individual cores.
             */
            u64 m_total_collection_time;
            /*
             * Number of online cores to be considered for wakeup purposes. 
             * Same as 'sysInfo.m_coreCount' EXCEPT FOR MEDFIELD (where it
             * is ALWAYS 1).
             */
            int m_wakeupCoreCount;
            /*
             * Helper map to associate threads with cores.
             * Same as 'sysInfo.m_htMap' EXCEPT FOR MEDFIELD!
             */
            std::map <int, int> m_htMap;
            /*
             * Helper map to retrieve all threads belonging to a given core i.e. the 'sibling threads'
             * of a given core.
             * (Basically, the inverse of 'sysInfo.m_htMap'
             */
            std::map <int, std::vector <int> > m_threadSiblingMap;
            /*
             * Should we inject duplicate C-state samples for the various cores in a given
             * package? ONLY FOR CLOVERVIEW/CLOVERTRAIL.
             */
            bool m_do_inject_dup_c_sample; 
            /*
             * Repository for system configuration info.
             */
            SystemInfo& sysInfo;

            /*
             * Friends.
             */
            friend class WuData;

        private:
            int do_read_i();
            int do_parse_i();
            int do_sort_and_collate_i();

            int get_sys_params_i(FILE *, u64&);

            std::string get_required_token_i(std::string , int , int , const char *);

            int fsm_begin_i(const sample_vec_t& samples, int from, int to, c_group_vec_t& c1_groups);
            int fsm_c_half_i(const sample_vec_t& samples, int to, int index, c_group_vec_t& c1_groups);
            int fsm_c_one_i(const sample_vec_t& samples, int to, int index, int group_beg_index, c_group_vec_t& c1_groups);
            int inlined_fsm_i(const sample_vec_t& samples, int from, int to, int core, c_group_vec_t& c1_groups);

            int do_c1_calcs_i(const sample_vec_t& samples, int from, int to, sample_vec_t& c1_output_samples, int core, int which_group, u64& prev_cx_end_tsc);
            int calc_c1_c0_res_i(const sample_vec_t& samples, int prev_beg, int prev_end, int curr_beg, int curr_end, int group_num, u64& c1_delta, u64& c0);
            int calc_last_c1_c0_i(const sample_vec_t& samples, int prev_beg, int prev_end, int curr_end, u64& c1_delta, u64& c0);

            int calc_cx_mwait_hint_i(const sample_vec_t& samples, int from, int to, int& requested_cx);
            int calc_inter_group_cx_c0_res_i(const PWCollector_sample_t& prev_end, const PWCollector_sample_t& curr_end, int& which, u64& cx_delta, u64& c0, const u64 &prev_cx_begin_tsc, const u64& prev_cx_end_tsc);
            int calc_wakeup_cause_i(const sample_vec_t& samples, int from, int to, bool allow_sched_wakeups, PWCollector_sample_t *wakeup_sample);

            int calc_aperf_mperf_deltas_i(const sample_vec_t& samples, int prev_end, int curr_end, aperf_mperf_map_t& aperf_mperf_map, sample_list_t& tmp_p_samples);
            int calc_actual_frequencies_i(sample_list_t& tmp_p_samples, const u64& min_tsc, const u64& max_tsc);
            int calc_actual_frequency_i(const double& aperf_mperf_ratio, u32& actual_freq);

            int advance_i(int index);

            int create_tps_groups_i(const sample_list_t *sorted_list, pair_vec_t& indices, int& num_tps);
            void dump_tps_groups_i(const sample_vec_t& sorted_vec, const pair_vec_t& indices);
            void dump_tps_groups_i(const sample_vec_t& sorted_vec, const c_group_vec_t& indices);

            PWCollector_sample_t create_tps_collector_sample_i(u8 break_type, u8 which_cx, u8 req_cx, u16 cpu, const u64& tsc, const u64& c0, const u64& cx, const PWCollector_sample_t& wu_sample);
            PWCollector_sample_t create_tpf_collector_sample_i(u16 is_boundary, u16 cpu, u32 freq, const u64& tsc, const u64& unhlt_core, const u64& unhlt_ref);

            void dump_pwcollector_sample_i(const PWCollector_sample_t& pwc_sample);

            bool check_sorted_i(int cpu);
            void sort_data_i(int cpu);

            std::vector <int> get_all_threads_of_i(int core);
            int get_core_given_lcpu_i(int lcpu);

#if DO_TPS_EPOCH_COUNTER
            void do_sort_sched_list_i(int core);
            void do_merge_sched_list_i(int core);
#endif // DO_TPS_EPOCH_COUNTER

        public:

            WuParser(SystemInfo&, bool, bool);
            ~WuParser();

            int do_work();

            void set_wuwatch_output_file_name(const std::string&, const std::string&);
    };
} // pwr


/* *****************************************
 * Some useful function declarations.
 * *****************************************
 */
void operator<<(std::ostream& os, const PWCollector_sample_t& samp);
void operator<<(std::ostream& os, const int_pair_t& p);
bool operator==(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2);
bool operator!=(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2);
bool operator<(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2);

/* *****************************************
 * Variable declarations.
 * *****************************************
 */
/*
 * For HT-testing. These correspond to the various
 * "sample_type_t" enum values. We have a long and 
 * a short version of each name.
 */
static const char *s_long_sample_names[] = {"FREE_SAMPLE", "TPS", "TPF", "K_CALL", "M_MAP", "I_MAP", "P_MAP", "S_RES", "S_ST", "D_RES", "D_ST", "TIM", "IRQ", "WRQ", "SCD", "IPI", "TPE", "W_STATE", "SAMPLE_END"};
static const char *s_short_sample_names[] = {"FREE_SAMPLE", "C", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "T", "I", "W", "S", "IPI", "TPE", "WL", "?"};


/* *****************************************
 * Function definitions.
 * *****************************************
 */

/*
 * The parser constructor.
 */
pwr::WuParser::WuParser(SystemInfo& info, bool should_calc_c1, bool should_dump_orig) : sysInfo(info),
    PW_max_num_cpus(-1), m_per_cpu_sample_lists(NULL), m_per_cpu_p_sample_lists(NULL),
#if DO_TPS_EPOCH_COUNTER
    m_per_cpu_sched_sample_lists(NULL),
#endif
    m_do_dump_backtraces(false), m_do_check_c1_res(should_calc_c1),
    m_do_dump_tps_groups(false), m_do_cx_c0_sanity_test(false),
    m_do_dump_orig_samples(should_dump_orig), m_was_any_tpf_sample_present(false),
    m_total_collection_time(0), 
    m_wakeupCoreCount(0), m_do_inject_dup_c_sample(false),
    m_combined_input_file_name(""), m_output_file_name("./parser_output.txt") {};

/*
 * The parser destructor.
 */
pwr::WuParser::~WuParser()
{
    fp_vec_t::iterator iter;
    for (iter = m_lib_input_fps.begin(); iter != m_lib_input_fps.end(); ++iter) {
        db_fprintf(stderr, "Closing %p\n", *iter);
        if (*iter) {
            fclose(*iter);
        }
    }

    if (m_do_dump_orig_samples) {
        delete []m_all_orig_samples;
    }

#if DO_TPS_EPOCH_COUNTER
    delete []m_per_cpu_sched_sample_lists;
#endif // DO_TPS_EPOCH_COUNTER
    delete []m_per_cpu_sample_lists;
    delete []m_per_cpu_p_sample_lists;
};

/*
 * Instance functions for helper
 * structs declared earlier.
 */
pwr::SystemInfo::SystemInfo():m_microPatchVer(0) {};
pwr::SystemInfo::~SystemInfo() {};

pwr::WuData *pwr::WuData::s_data = NULL;

/*
 * INTERNAL API:
 * Create a PWCollector_sample for a given 'wakeup'.
 *
 * @break_type: the wakeup type. One of the 'sample_type_t' enum values.
 * @which_cx: the C-state the core was actually executing at.
 * @req_cx: the C-state the OS had requested. May be different from @which_cx due to promotion/demotion.
 * @cpu: the logical processor that entered the C-state.
 * @tsc: TSC at which the core exitted the C-state.
 * @c0: The C0 residency.
 * @cx: the C-state residency
 * @wu_sample: Description of the wakeup cause.
 *
 * @returns: a PWCollector_sample instance corresponding to the given sleep state.
 */
PWCollector_sample_t pwr::WuParser::create_tps_collector_sample_i(u8 break_type, u8 which_cx, u8 req_cx, u16 cpu, const u64& tsc, const u64& c0, const u64& cx, const PWCollector_sample_t& wu_sample)
{
    PWCollector_sample_t sample;
    int wakeup_cpuidx = wu_sample.cpuidx;

    memset(&sample, 0, sizeof(sample));

    sample.sample_type = C_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;

    sample.c_sample.prev_state = req_cx;
    
    RES_COUNT(sample.c_sample, MPERF) = c0; RES_COUNT(sample.c_sample, which_cx) = cx;
    /*
     * Need to convert 'break_type' to its 'PW_BREAK_TYPE_xxx' equivalent.
     * e.g. "TIMER_SAMPLE" <--> PW_BREAK_TYPE_T etc.
     */
    switch (break_type) {
        case FREE_SAMPLE:
            /* Only happens if we're inserting a 'Ghost' sample. */
            sample.c_sample.break_type = PW_BREAK_TYPE_B;
            break;
        case TIMER_SAMPLE:
            sample.c_sample.break_type = PW_BREAK_TYPE_T;
            sample.c_sample.pid = wu_sample.e_sample.data[0];
            sample.c_sample.tid = wu_sample.e_sample.data[1];
            sample.c_sample.c_data = wu_sample.e_sample.data[2];
            break;
        case IRQ_SAMPLE:
            sample.c_sample.break_type = PW_BREAK_TYPE_I;
            sample.c_sample.pid = sample.c_sample.tid = 0;
            sample.c_sample.c_data = wu_sample.e_sample.data[0];
            break;
        case WORKQUEUE_SAMPLE:
            sample.c_sample.break_type = PW_BREAK_TYPE_W;
            break;
        case SCHED_SAMPLE:
            sample.c_sample.break_type = PW_BREAK_TYPE_S;
            /*
             * For SCHED_WAKEUP events, we need the CPUID of the
             * (Logical) CPU that was TARGETED, and NOT the CPUID
             * of the (Logical) CPU that sent the SCHED_WAKEUP!
             */
            wakeup_cpuidx = wu_sample.e_sample.data[1]; // TARGET cpu!
            sample.c_sample.c_data = wu_sample.e_sample.data[0]; // SOURCE cpu == wu_sample.cpuidx!
            break;
        case IPI_SAMPLE:
            sample.c_sample.break_type = PW_BREAK_TYPE_IPI;
            break;
        default:
            db_fprintf(stderr, "Warning: Break type = %d\n", break_type);
            sample.c_sample.break_type = PW_BREAK_TYPE_U;
            break;
    }
    /*
     * We no longer need the 'EPOCH' field.
     * Use it to store the ACTUAL 'Cx' granted.
     * UPDATE: we also use it to store the CPUID
     * of the wakeup event.
     */
    // sample.c_sample.tps_epoch = which_cx;
    sample.c_sample.tps_epoch = COMBINED_CX_CPUID(which_cx, wakeup_cpuidx);

    /*
     * GCC *should* do a Return-Value-Optimization here!
     */
    return sample;
};
/*
 * INTERNAL API:
 * Create a PWCollector_sample instance for a given P-state transition.
 *
 * @is_boundary: 0 ==> Non-boundary sample; 1 ==> START boundary sample; 2 ==> STOP boundary sample
 * @cpu: the logical processor that entered a new p-state.
 * @freq: the frequency corresponding to the p-state.
 * @tsc: the tsc at which the transition occured.
 * @unhlt_core: the number of unhalted core cycles elapsed.
 * @unhlt_ref: the number of unhalted reference cycles elapsed.
 *
 * @returns: the newly constructed PWCollector_sample instance corresponding
 * to this p-state transition.
 */
PWCollector_sample_t pwr::WuParser::create_tpf_collector_sample_i(u16 is_boundary, u16 cpu, u32 freq, const u64& tsc, const u64& unhlt_core, const u64& unhlt_ref)
{
    PWCollector_sample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.sample_type = P_STATE;
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    sample.p_sample.is_boundary_sample = is_boundary;
    sample.p_sample.frequency = freq;

    sample.p_sample.unhalted_core_value = unhlt_core;
    sample.p_sample.unhalted_ref_value = unhlt_ref;
    /*
     * GCC *should* do a Return-Value-Optimization here!
     */
    return sample;
};


pwr::CGroup::CGroup(int begin, int end) : group(begin, end), wakeup_sample(NULL){};


/**********************************************
  WUDUMP FUNCTIONS
 **********************************************/

/*
 * INTERNAL API:
 * Helper function: tokenize 'line', using delim set
 * 'd', and return the 'tok_num'th token.
 *
 * @line: the string to tokenize
 * @tok_num: the token number to return
 * @max_num_toks: the max # of tokens to expect in @line
 * @d: the (possibly empty) delim set.
 *
 * @returns: the 'tok_num'th token in 'line'
 */
std::string pwr::WuParser::get_required_token_i(std::string line, int tok_num, int max_num_toks, const char *d = NULL) {
    const char *delims = d ? d : " ";
    str_vec_t toks = Tokenizer(line, delims).get_all_tokens();

    db_assert(toks.size() == max_num_toks, "ERROR: invalid # toks = %d\n", toks.size());

    return toks[tok_num - 1];
};

/*
 * INTERNAL API:
 * Function to create "TPS groups". A "TPS group" is a group of C-state
 * (and other) samples that all have the same 'Cx' MSR values. In other words,
 * a TPS group marks the beginning or end of a c-state transition and provides
 * a guarantee that the core never entered "C2" or a deeper sleep state because
 * the 'Cx' MSRs never counted.
 *
 * Requires the input list to
 * (1) Be sorted (in ascending TSC order) and
 * (2) Have elements of both logical CPUs.
 *
 * @sorted_list: the sorted input list of PWCollector samples, sorted wrt TSC
 * @indices: a set of [begin,end] pairs indicating the begin and end of the
 * various TPS groups.
 * @num_tps: the number of c-state samples encountered during the parsing; primarily
 * used in debugging.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::create_tps_groups_i(const sample_list_t *sorted_list, pair_vec_t& indices, int& num_tps)
{
    PWCollector_sample_t last_sample;
    bool is_first = true;
    int begin_index = -1, end_index = -1, curr_index = 0;
    for (sample_list_t::const_iterator citer = sorted_list->begin(); citer != sorted_list->end(); ++citer, ++curr_index) {
        PWCollector_sample_t curr_sample = *citer;
        if (curr_sample.sample_type != C_STATE) {
            continue;
        }
        ++num_tps;
        if (begin_index == -1) {
            begin_index = curr_index;
        }
        if (end_index == -1) {
            end_index = curr_index;
        }
        if (is_first) {
            last_sample = curr_sample;
            is_first = false;
            continue;
        }
        if (curr_sample != last_sample) {
            /*
             * It's possible for the 'group' to consist of a 
             * SINGLE TPS sample, in which case 'end_index' < 'begin_index'.
             * Check for that here.
             */
            if (end_index < begin_index) {
                db_fprintf(stderr, "Warning: end_index = %d, begin_index = %d, SINGLE GROUP?!\n", end_index, begin_index);
                end_index = begin_index;
            }
            indices.push_back(int_pair_t(begin_index, end_index));
            last_sample = curr_sample;
            begin_index = curr_index;
        } else {
            end_index = curr_index;
        }
    }
    if (begin_index > 0 && begin_index != curr_index) {
        /*
         * We have some samples at the end -- create an
         * open-ended TPS group for them.
         */
        if (end_index < begin_index) {
            db_fprintf(stderr, "Warning: end_index = %d, begin_index = %d, SINGLE END GROUP!\n", end_index, begin_index);
            end_index = begin_index;
        }
        indices.push_back(int_pair_t(begin_index, end_index));
    }
    return SUCCESS;
};
/*
 * INTERNAL API:
 * Helper function to iterate over a range. Currently
 * only increments an index.
 */
int pwr::WuParser::advance_i(int index)
{
    return ++index;
};

/*
 * INTERNAL API:
 * A stage in the C1-determination FSM representing when the core is thought to
 * be in a C1 sleep state.
 */
int pwr::WuParser::fsm_c_one_i(const sample_vec_t& samples, int to, int index, int group_beg_index, c_group_vec_t& c1_groups)
{
    if (c1_groups.size() && IS_INVALID_WAKEUP_CAUSE(c1_groups.back())) {
        /*
         * Previous group had UNKNOWN (or "SCHED") wakeup. We posit this isn't
         * a C-state transition at all, and discard this sample.
         */
        db_fprintf(stderr, "Warning: group [%d,%d] had INVALID/NULL wakup cause; DISCARDING! (Cause = %d)\n", c1_groups.back().group.first, c1_groups.back().group.second, c1_groups.back().wakeup_sample ? c1_groups.back().wakeup_sample->sample_type : -1);
        /*
         * Discarding the previous sample ==> core did NOT enter a "sleep" state ==> current
         * C1 groups 'BEGIN' boundary should be pushed back to accomodate the previous
         * sample.
         */
        c1_groups.pop_back();
    }

    c1_groups.push_back(CGroup(group_beg_index, index));

    FOR_EACH(index, advance_i(index), to) {
        if (samples[index].sample_type == C_STATE) { // TPS
            if (WAS_SYSTEM_HYPER_THREADED()) {
                return fsm_c_half_i(samples, to, index, c1_groups);
            } else if (c1_groups.size() && IS_INVALID_WAKEUP_CAUSE(c1_groups.back())) {
                /*
                 * Previous group had UNKNOWN (or "SCHED") wakeup. We posit this isn't
                 * a C-state transition at all, and discard this sample.
                 */
                db_fprintf(stderr, "Warning: group [%d,%d] had INVALID/NULL wakup cause; DISCARDING! (Cause = %d)\n", c1_groups.back().group.first, c1_groups.back().group.second, c1_groups.back().wakeup_sample ? c1_groups.back().wakeup_sample->sample_type : -1);
                c1_groups.pop_back();
                /*
                 * We're starting a new group here!
                 */
                c1_groups.push_back(CGroup(index, index));
            }
        } else { // NTPS
            c1_groups.back().wakeup_sample = &samples[index];
            return fsm_begin_i(samples, index, to, c1_groups);
        }
    }

    return SUCCESS;
};
/*
 * INTERNAL API:
 * A stage in the C1-determination FSM representing when one of two logical processors is
 * in 'mwait' (but the other processor may still be executing).
 */
int pwr::WuParser::fsm_c_half_i(const sample_vec_t& samples, int to, int index, c_group_vec_t& c1_groups)
{
    bool wakeup_cause_calculated = c1_groups.empty() || c1_groups.back().wakeup_sample != NULL;
    const PWCollector_sample_t& beg_sample = samples[index];
    int group_beg_index = index;
    int beg_cpu = beg_sample.cpuidx;
    int curr_cpu = -1;

    FOR_EACH(index, advance_i(group_beg_index), to) {
        const PWCollector_sample_t& curr_sample = samples[index];
        curr_cpu = curr_sample.cpuidx;
        if (curr_sample.sample_type == C_STATE) {
            if (curr_cpu != beg_cpu) { // OTPS
                return fsm_c_one_i(samples, to, index, group_beg_index, c1_groups);
            }
        } else { // NTPS
            if (!wakeup_cause_calculated) {
                c1_groups.back().wakeup_sample = &curr_sample;
                wakeup_cause_calculated = true;
            }
            if (curr_cpu == beg_cpu) { // SNTPS
                return fsm_begin_i(samples, index, to, c1_groups);
            }
        }
    }
    return SUCCESS;
};
/*
 * INTERNAL API:
 * A stage in the C1-determination FSM representing when the core is known to be executing
 * i.e. in the C0 state.
 * This is also the begin state of the FSM.
 */
int pwr::WuParser::fsm_begin_i(const sample_vec_t& samples, int from, int to, c_group_vec_t& c1_groups)
{
    int index = from;
    FOR_EACH(index, from, to) {
        if (samples[index].sample_type == C_STATE) {
            if (WAS_SYSTEM_HYPER_THREADED()) {
                return fsm_c_half_i(samples, to, index, c1_groups);
            } else {
                return fsm_c_one_i(samples, to, index, index, c1_groups);
            }
        }
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Inlined version of our C1 group FSM. Avoids stack corruption on very large
 * data sets.
 *
 * The primary purpose of this FSM is to try and determine when, within a 
 * particular "TPS Group", the core enters "C1". We need an algorithmic
 * approach to C1 determination because we don't have any hardware counters
 * to give us this information.
 * Note that this is applicable only to the samples within a TPS group.
 *
 * @samples: the (sorted) list of PWCollector samples.
 * @from: the start of the current TPS group.
 * @to: the end of the current TPS group.
 * @c1_groups: the set of "C1" groups for the current TPS group, as determined by
 * our FSM-based algorithm.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::inlined_fsm_i(const sample_vec_t& samples, int from, int to, int core, c_group_vec_t& c1_groups)
{
    int index = from, group_beg_index = -1;
    bool wakeup_cause_calculated = false;
    int beg_cpu = -1, curr_cpu = -1;
    const std::vector <int> &threads = m_threadSiblingMap[core];
    std::map <int, bool> c_half_map;
    int num_threads_in_c_half = 0;

    int orig_from = from, orig_to = to;

    typedef enum {
        FSM_BEGIN=0, /* Core is known to be in C0 */
        FSM_HALF, /* 1 of 2 logical procs is known to be in mwait */
        FSM_ONE, /* Core is *thought* to be in C1 */
        FSM_END /* We've reached the end of this TPS group */
    } fsm_state_t;

    fsm_state_t curr_state = FSM_BEGIN;

    for (std::vector<int>::const_iterator citer = threads.begin(); citer != threads.end(); ++citer) {
        c_half_map[*citer] = false;
    }

    while (curr_state != FSM_END) {
        switch (curr_state) {

            case FSM_BEGIN: /* Core is known to be in C0 */
                /* fsm_begin_i */
                FOR_EACH(index, from, to) {
                    if (samples[index].sample_type == C_STATE) {
                        if (WAS_SYSTEM_HYPER_THREADED()) {
                            curr_state = FSM_HALF;
                        } else {
                            curr_state = FSM_ONE;
                            group_beg_index = index;
                        }
                        break;
                    }
                }
                if (curr_state == FSM_BEGIN) {
                    /*
                     * Can only happen if we've iterated to the end of the group!
                     */
                    curr_state = FSM_END;
                }
                break;

            case FSM_HALF: /* 1 of 2 logical procs is known to be in mwait */
                /* fsm_c_half_i */
                wakeup_cause_calculated = c1_groups.empty() || c1_groups.back().wakeup_sample != NULL;
                group_beg_index = index;
                beg_cpu = samples[index].cpuidx;
                num_threads_in_c_half = 1;

                for (std::map<int, bool>::iterator iter = c_half_map.begin(); iter != c_half_map.end(); ++iter) {
                    iter->second = false;
                }
                c_half_map[beg_cpu] = true;

                FOR_EACH(index, advance_i(group_beg_index), to) {
                    const PWCollector_sample_t& curr_sample = samples[index];
                    curr_cpu = curr_sample.cpuidx;
                    if (curr_sample.sample_type == C_STATE) {
                        if (c_half_map[curr_cpu] == false) {
                            c_half_map[curr_cpu] = true;
                            if (++num_threads_in_c_half == threads.size()) {
                                curr_state = FSM_ONE;
                                break;
                            }
                        }
                    } else { // NTPS
                        if (!wakeup_cause_calculated) {
                            c1_groups.back().wakeup_sample = &curr_sample;
                            wakeup_cause_calculated = true;
                        }
                        if (c_half_map[curr_cpu] == true) { // SNTPS
                            curr_state = FSM_BEGIN;
                            from = index; to = to;
                            break;
                            // return fsm_begin_i(samples, index, to, c1_groups);
                        }
                    }
                }
                if (curr_state == FSM_HALF) {
                    /*
                     * Can only happen if we've iterated to the end of the group!
                     */
                    curr_state = FSM_END;
                }
                break;

            case FSM_ONE: /* Core is *thought* to be in C1 */
                /* fsm_c_one_i */
                if (c1_groups.size() && IS_INVALID_WAKEUP_CAUSE(c1_groups.back())) {
                    /*
                     * Previous group had UNKNOWN (or "SCHED") wakeup. We posit this isn't
                     * a C-state transition at all, and discard this sample.
                     */
                    db_fprintf(stderr, "Warning: group [%d,%d] had INVALID/NULL wakup cause; DISCARDING! (Cause = %d)\n", c1_groups.back().group.first, c1_groups.back().group.second, c1_groups.back().wakeup_sample ? c1_groups.back().wakeup_sample->sample_type : -1);
                    /*
                     * Discarding the previous sample ==> core did NOT enter a "sleep" state ==> current
                     * C1 groups 'BEGIN' boundary should be pushed back to accomodate the previous
                     * sample.
                     */
                    c1_groups.pop_back();
                }

                c1_groups.push_back(CGroup(group_beg_index, index));

                FOR_EACH(index, advance_i(index), to) {
                    if (samples[index].sample_type == C_STATE) { // TPS
                        if (WAS_SYSTEM_HYPER_THREADED()) {
                            curr_state = FSM_HALF;
                            break;
                            // return fsm_c_half_i(samples, to, index, c1_groups);
                        } else if (c1_groups.size() && IS_INVALID_WAKEUP_CAUSE(c1_groups.back())) {
                            /*
                             * Previous group had UNKNOWN (or "SCHED") wakeup. We posit this isn't
                             * a C-state transition at all, and discard this sample.
                             */
                            db_fprintf(stderr, "Warning: group [%d,%d] had INVALID/NULL wakup cause; DISCARDING! (Cause = %d)\n", c1_groups.back().group.first, c1_groups.back().group.second, c1_groups.back().wakeup_sample ? c1_groups.back().wakeup_sample->sample_type : -1);
                            c1_groups.pop_back();
                            /*
                             * We're starting a new group here!
                             */
                            c1_groups.push_back(CGroup(index, index));
                        }
                    } else { // NTPS
                        c1_groups.back().wakeup_sample = &samples[index];
                        curr_state = FSM_BEGIN;
                        from = index; to = to;
                        break;
                        // return fsm_begin_i(samples, index, to, c1_groups);
                    }
                }
                if (curr_state == FSM_ONE) {
                    /*
                     * Can only happen if we've iterated to the end of the group!
                     */
                    curr_state = FSM_END;
                }
                break;
            default:
                break;
        }
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Calculate 'C1' and 'C0' residency for a given C1 "Group".
 */
int pwr::WuParser::calc_c1_c0_res_i(const sample_vec_t& samples, int prev_beg, int prev_end, int curr_beg, int curr_end, int group_num, u64& c1_delta, u64& c0)
{
    const PWCollector_sample_t& prev_beg_sample = samples[prev_beg], &prev_end_sample = samples[prev_end], &curr_beg_sample = samples[curr_beg], &curr_end_sample = samples[curr_end];

    u64 tsc_delta = curr_end_sample.tsc - prev_end_sample.tsc;
    
    if (prev_end_sample.cpuidx == curr_beg_sample.cpuidx) {
        /*
         * Case-1: cpuidx(prev_end) == cpuidx(curr_begin)
         * C0 = (MPERF" - MPERF') + (TSC''' - TSC")
         * Where ' ==> prev end
         *       " ==> curr beg
         *       ''' => curr end
         */
        u64 mperf_delta = RES_COUNT(curr_beg_sample.c_sample, MPERF) - RES_COUNT(prev_end_sample.c_sample, MPERF);
        c0 = (curr_end_sample.tsc - curr_beg_sample.tsc) + mperf_delta;
        if ((s64)c0 < 0) {
            c0 = 0;
            assert(false);
        }
    } else {
        /*
         * Case-2: cpuidx(prev_end) != cpuidx(curr_begin)
         * C0 = (MPERF''' - MPERF')
         * Where ' ==> prev end
         *       " ==> curr beg
         *       ''' => curr end
         * UPDATE: to accomodate CLV (where we associate all 4 threads with a single core for wakeup purposes), 
         * we cannot assume "prev_end.cpuidx" == "curr_end.cpuidx". Instead, we must iterate over the previous group
         * until we come to a compatible cpuidx.
         */
        if (m_threadSiblingMap[get_core_given_lcpu_i(curr_beg_sample.cpuidx)].size() > 2) {
            c0 = 0;
            for (int __tmp_idx = prev_end; __tmp_idx >= prev_beg; --__tmp_idx) {
                const PWCollector_sample_t& __tmp_sample = samples[__tmp_idx];
                if (__tmp_sample.cpuidx == curr_end_sample.cpuidx) {
                    c0 = RES_COUNT(curr_end_sample.c_sample, MPERF) - RES_COUNT(__tmp_sample.c_sample, MPERF);
                    break;
                }
            }
            assert(c0 > 0);
        } else {
            if (true && prev_end_sample.cpuidx != curr_end_sample.cpuidx) {
                std::cerr << "prev_end: " << prev_end_sample;
                std::cerr << "curr_end: " << curr_end_sample;
            }
            assert(prev_end_sample.cpuidx == curr_end_sample.cpuidx);

            u64 mperf_delta = RES_COUNT(curr_end_sample.c_sample, MPERF) - RES_COUNT(prev_end_sample.c_sample, MPERF);
            c0 = mperf_delta;
        }
        if ((s64)c0 < 0) {
            c0 = 0;
            assert(false);
        }
    }

    if (c0 > tsc_delta) {
        c0 = tsc_delta;
    }
    /*
     * Common eqn:
     * C1 = TSC-delta - C0
     * Where TSC-delta = TSC''' - TSC'
     *       C0 = as calculated above.
     *       ' ==> prev end
     *       " ==> curr beg
     *       ''' => curr end
     */
    c1_delta = tsc_delta - c0;

    return SUCCESS;
};

/*
 * INTERNAL API:
 * Special case for calculating C1, C0 residencies for the last C1 group.
 */
int pwr::WuParser::calc_last_c1_c0_i(const sample_vec_t& samples, int prev_beg, int prev_end, int curr_end, u64& c1_delta, u64& c0)
{
    const PWCollector_sample_t& prev_beg_sample = samples[prev_beg], &prev_end_sample = samples[prev_end], &curr_end_sample = samples[curr_end];

    u64 tsc_delta = curr_end_sample.tsc - prev_end_sample.tsc;

    assert(prev_end_sample.sample_type == curr_end_sample.sample_type == C_STATE);
    if (prev_end_sample.cpuidx == curr_end_sample.cpuidx) {
        c0 = RES_COUNT(curr_end_sample.c_sample, MPERF) - RES_COUNT(prev_end_sample.c_sample, MPERF);
    } else {
        c0 = RES_COUNT(curr_end_sample.c_sample, MPERF) - RES_COUNT(prev_beg_sample.c_sample, MPERF);
    }
    if (c0 > tsc_delta) {
        c0 = tsc_delta;
    }
    c1_delta = tsc_delta - c0;
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Parse a given "TPS Group", determining possible C1 transitions, calculating
 * C1 and C0 residencies for those transitions and extracting wakeup causes
 * for those transitions.
 *
 * @samples: the (sorted) list of PWCollector samples, as returned by the power driver.
 * @from: the start of the current TPS group.
 * @to: the end of the current TPS group.
 * @c1_output_samples: the list of C1 C-state samples, as determined by our algorithm.
 * @core: the core we're currently considering.
 * @which_group: an index representing which "TPS group" we're currently parsing; used
 * (mostly) for debugging.
 * @prev_cx_end_tsc: the (calculated) TSC when the core LAST entered Cx.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_c1_calcs_i(const sample_vec_t& samples, int from, int to, sample_vec_t& c1_output_samples, int core, int which_group, u64& prev_cx_end_tsc)
{
    int start = from, stop = to;

    c_group_vec_t c1_groups;

    /*
     * OK, we've determined analysis boundaries. Now create
     * the actual 'C1' "Groups" (use an FSM for this).
     */
    // if (fsm_begin_i(samples, start, stop, c1_groups)) {
    if (inlined_fsm_i(samples, start, stop, core, c1_groups)) {
        db_fprintf(stderr, "ERROR retrieving C1 groups!\n");
        return -ERROR;
    }

    /*
     * OK, "C1" groups created. Now calculate C1 residencies
     * and wakeup causes for each of the groups. Also generate
     * output samples for them.
     */
    db_fprintf(stderr, "Group = %d, from = %d, to = %d, start = %d, stop = %d (from tsc = %16llu)\n", which_group, from, to, start, stop, samples[from].tsc);
    db_fprintf(stderr, "# C1 groups = %d\n", c1_groups.size());

    if (m_do_dump_tps_groups) {
        dump_tps_groups_i(samples, c1_groups);
    }

    int size = c1_groups.size();

    if (size == 0) {
        /*
         * Should ONLY be possible for the LAST Cx group!
         */
        db_fprintf(stderr, "Warning: [%d] had ZERO c1 groups!\n", which_group);
    }
    /*
     * It is possible for the LAST C1 group to have an invalid
     * wakeup cause. Account for that here (by removing it
     * from the list of C1 groups to parse).
     */
    if (c1_groups.size() && IS_INVALID_WAKEUP_CAUSE(c1_groups.back())) {
        db_fprintf(stderr, "WARNING: invalid wakup cause for last C1 group  for TPS group %d!\n", which_group);
        c1_groups.pop_back();
        --size;
    }

    prev_cx_end_tsc = 0;

    u64 delta_tsc = samples[to].tsc - samples[from].tsc, c0_res = 0; // for debugging
    u64 c1_res = 0;

    /*
     * Iterate over the C1 groups and calculate residencies. Also construct
     * C-state samples from the C1 transitions.
     */
    for (int i=1; i<size; ++i) {
        int_pair_t prev_c1_group = c1_groups[i-1].group, curr_c1_group = c1_groups[i].group;
        int prev_beg = prev_c1_group.first, prev_end = prev_c1_group.second, curr_beg = curr_c1_group.first, curr_end = curr_c1_group.second;
        db_assert(prev_beg <= prev_end && curr_beg <= curr_end, "ERROR in samples! p-beg = %d, p-end = %d, c-beg = %d, c-end = %d\n", prev_beg, prev_end, curr_beg, curr_end);
        const PWCollector_sample_t& prev_beg_sample = samples[prev_beg], &prev_end_sample = samples[prev_end], &curr_beg_sample = samples[curr_beg], &curr_end_sample = samples[curr_end];
        PWCollector_sample_t wakeup_sample = *(c1_groups[i-1].wakeup_sample);
        u32 cause = wakeup_sample.sample_type;
        u64 c0 = 0, c1 = 0;

        assert(IS_INVALID_WAKEUP_CAUSE(c1_groups[i-1]) == false);

        if (prev_cx_end_tsc == 0) {
            prev_cx_end_tsc = prev_end_sample.tsc;
        }

        if (calc_c1_c0_res_i(samples, prev_beg, prev_end, curr_beg, curr_end, i, c1, c0)) {
            db_fprintf(stderr, "ERROR calculating c1 res!\n");
            continue;
        }
        c0_res += c0; c1_res += c1;

        int requested_cx = prev_end_sample.c_sample.prev_state;

        c1_output_samples.push_back(create_tps_collector_sample_i(cause, APERF, GET_C_STATE_GIVEN_TPS_HINT(requested_cx), core, curr_end_sample.tsc, c0, c1, wakeup_sample));
    }
    /*
     * We need to fix up the last 'C1' group.
     */
    if (size > 0) {
        int_pair_t last_c1_group = c1_groups.back().group;
        const PWCollector_sample_t &prev_end_sample = samples[last_c1_group.second], &curr_end_sample = samples[to];
        const PWCollector_sample_t *wakeup_sample = c1_groups.back().wakeup_sample;

        db_fprintf(stderr, "[%d]: Cx begin tsc = %llu C1 begin tsc = %llu\n", which_group, samples[start].tsc, samples[c1_groups[0].group.first].tsc);
        db_fprintf(stderr, "[%d]: Cx end tsc = %llu C1 end tsc = %llu size = %d (%d, %d)\n", which_group, samples[to].tsc, samples[last_c1_group.second].tsc, size, to, last_c1_group.second);

        if (last_c1_group.second != to) {
            if (IS_INVALID_WAKEUP_CAUSE(c1_groups.back())) {
                db_fprintf(stderr, "[%d]: NULL/INVALID wakeup sample!\n", which_group);
            } else {
                if (prev_cx_end_tsc == 0) {
                    prev_cx_end_tsc = prev_end_sample.tsc;
                }
                db_fprintf(stderr, "[%d]: non-NULL wakeup sample!\n", which_group);

                u32 cause = wakeup_sample->sample_type;
                u64 c0 = 0, c1 = 0;

                if (calc_last_c1_c0_i(samples, last_c1_group.first, last_c1_group.second, to, c1, c0)) {
                    db_fprintf(stderr, "ERROR calculating last c0/c1!\n");
                } else {
                    c0_res += c0; c1_res += c1;
                    int requested_cx = prev_end_sample.c_sample.prev_state;

                    c1_output_samples.push_back(create_tps_collector_sample_i(cause, APERF, GET_C_STATE_GIVEN_TPS_HINT(requested_cx), core, curr_end_sample.tsc, c0, c1, *wakeup_sample));
                }
            }
        }
    }

    if (size > 1) {
        db_fprintf(stderr, "[%d]: size = %d C0-res = %llu C1-res = %llu Delta-TSC = %llu diff = %llu\n", which_group, size, c0_res, c1_res, delta_tsc, (delta_tsc - c0_res - c1_res));
    }

    return SUCCESS;
};

/*
 * INTERNAL API:
 * Read collection/system params from the 'sys_params_found.txt'
 * file -- adapted from 'wudump' code.
 *
 * @in_fp: the file to read.
 * @sys_params_off: the offset (within the wuwatch ".ww1" output file) from
 * which to begin reading system configuration information.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::get_sys_params_i(FILE *in_fp, u64& sys_params_off)
{

#define DOES_LINE_CONTAIN(line, str) ( (line).find(str) != std::string::npos )

    str_deq_t lines;
    /*
     * Make sure the 'init()'
     * function has been called!
     */
    db_assert(m_combined_input_file_name.size() > 0, "ERROR: init not called before get_sys_params()!\n");
    /*
     * Starting offset of system param information is
     * encoded in first 8 bytes of the combined output file.
     * Read that here.
     */
    sys_params_off = 0;
    if (fread(&sys_params_off, sizeof(sys_params_off), 1, in_fp) != 1) {
        perror("fread error in get_sys_params_i()");
        return -ERROR;
    }
    db_fprintf(stderr, "SYS PARAMS OFFSET = %llu\n", sys_params_off);
    /*
     * OK, seek to the offset specified.
     */
    if (fseek(in_fp, sys_params_off, SEEK_SET)) {
        perror("fseek error");
        return -ERROR;
    }
    /*
     * Next line MUST be "--SYS_PARAMS_BEGIN---"!!!
     */
//    char *line = NULL;
//    size_t len = 0;
//    ssize_t read = 0;
//    if ( (read = getline(&line, &len, in_fp)) == -1) {
//        perror("getline error");
//        return -ERROR;
//    }
//    line[read-1] = '\0';

    char line[1024];
    /*
     * We ASSUME 1024 characters is enough!
     */
    if (fgets(line, sizeof(line), in_fp) == NULL) {
        perror("fgets error");
        return -ERROR; 
    }
    /*
     * Get rid of terminating newline
     */
    line[strlen(line) - 1] = '\0';

    db_fprintf(stderr, "NEXT LINE IS: %s\n", line);
    assert(!strcmp(line, "---SYS_PARAMS_BEGIN---"));
    /*
     * OK, we're ready to begin. Start by reading in
     * all the remaining lines in the input file. Then parse
     * them to extract system configuration information.
     */
    LineReader::get_all_lines(in_fp, lines);

    /*
     * Users expect to see a copy of the system config in its entirety.
     * Make a copy here.
     */
    sysInfo.m_lines = lines;

    while(!lines.empty()) {
        str_t line = lines.front(); lines.pop_front();
        db_fprintf(stderr, "%s\n", line.c_str());

        if (line.find("Version") != std::string::npos) {
            /*
             * Found Driver/Wuwatch version info.
             */
            str_vec_t toks = Tokenizer(line, " ").get_all_tokens();
            db_assert(toks.size() == 4, "ERROR: invalid # tokens = %d\n", toks.size());
            str_t ver_str = toks[3];
            if (toks[0] == "Driver") {
                sysInfo.m_driverVersion = ver_str;
            }else if (toks[0] == "Wuwatch") {
                sysInfo.m_wuwatchVersion = ver_str;
            } else if (toks[0] == "OS") {
                std::string os_version = get_required_token_i(line, 4, 4);
                sysInfo.m_osVersion = os_version;
                db_assert(true, "Found os version = %s\n", os_version.c_str());
            } else {
                db_abort("ERROR: invalid version string: %s\n", line.c_str());
            }
        }
        else if (DOES_LINE_CONTAIN(line, "Start TSC")) {
            uint64_t start_tsc = strtoull(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            sysInfo.m_startTsc = start_tsc;
            db_assert(true, "Found start tsc = %llu\n", start_tsc);
        }
        else if (DOES_LINE_CONTAIN(line, "Stop TSC")) {
            uint64_t stop_tsc = strtoull(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            sysInfo.m_stopTsc = stop_tsc;
            db_assert(true, "Found stop tsc = %llu\n", stop_tsc);
        }
        else if (DOES_LINE_CONTAIN(line, "Start Timeval")) {
            uint64_t start_timeval = strtoull(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            sysInfo.m_startTimeval = start_timeval;
            db_assert(true, "Found start timeval = %llu\n", start_timeval);
        }
        else if (DOES_LINE_CONTAIN(line, "Host Name")) {
            std::string host_name = get_required_token_i(line, 4, 4);
            sysInfo.m_hostName = host_name;
            db_assert(true, "Found host name = %s\n", host_name.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "OS Name")) {
            std::string os_name = get_required_token_i(line, 4, 4);
            sysInfo.m_osName = os_name;
            db_assert(true, "Found os name = %s\n", os_name.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "OS Type")) {
            std::string os_type = get_required_token_i(line, 4, 4);
            sysInfo.m_osType = os_type;
            db_assert(true, "Found os type = %s\n", os_type.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Brand")) {
            std::string cpu_brand = get_required_token_i(line, 4, 4);
            sysInfo.m_cpuBrand = cpu_brand;
            db_assert(true, "Found CPU brand = %s\n", cpu_brand.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Family")) {
            sysInfo.m_cpuFamily = atoi(get_required_token_i(line, 4, 4).c_str());
            db_assert(true, "Found CPU Family = %u\n", sysInfo.m_cpuFamily);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Model")) {
            sysInfo.m_cpuModel = atoi(get_required_token_i(line, 4, 4).c_str());
            db_assert(true, "Found CPU model = %u\n", sysInfo.m_cpuModel);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Stepping")) {
            sysInfo.m_cpuStepping = atoi(get_required_token_i(line, 4, 4).c_str());
            db_assert(true, "Found CPU stepping = %u\n", sysInfo.m_cpuStepping);
        }
        else if (DOES_LINE_CONTAIN(line, "Turbo Threshold")) {
            sysInfo.m_turboThreshold = strtoull(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            db_assert(true, "Found turbo threshold = %llu\n", sysInfo.m_turboThreshold);
        }
        else if (line.find("Platform Architecture") != std::string::npos) {
            /*
             * Found the platform architecture identifier.
             */
            int arch_id = atoi(get_required_token_i(line, 4, 4).c_str());
            sysInfo.m_arch = arch_id;
            db_assert(true, "Found platform architecture = %d\n", arch_id);
        }
        else if (line.find("Num CPUs") != std::string::npos) {
            /*
             * Found # cpus on collection
             * machine.
             */
            int num_cpus = atoi(get_required_token_i(line, 4, 4).c_str());
            sysInfo.m_cpuCount = num_cpus;
            db_assert(true, "Found # cpus = %d\n", num_cpus);
        }
        else if (line.find("CPU Topology") != std::string::npos) {
            sysInfo.m_cpuTopology = line.substr(line.find_first_of("=")+2); // +2 ==> 1 for the '=' and 1 for the space after it.
            str_vec_t toks = Tokenizer(line, " ").get_all_tokens();
            assert(toks.size() > 3);
            int size = toks.size();
            /*
             * We don't need the first 3 tokens (i.e. "Cpu Topology =")
             */
            std::map <int, int_vec_t> tmp_map;
            int max_core_id = -1, curr_core_id = -1;
            if (sysInfo.m_cpuCount == 1) {
                sysInfo.m_htMap[0] = 0;
                sysInfo.m_coreCount = 1;
                m_wakeupCoreCount = 1;
            } else {
                for (int i=3; i<size; ) {
                    str_t tokens[5];
                    /*
                     * Format of each logical CPU is:
                     * proc # <space> physical id <space> siblings <space> core id <space> cores <space>"
                     */
                    for (int j=0; j<5; ++j, ++i) {
                        tokens[j] = toks[i];
                    }
                    int proc = atoi(tokens[0].c_str()), phys_id = atoi(tokens[1].c_str()), core_id = atoi(tokens[3].c_str());
                    tmp_map[((phys_id << 16) | core_id)].push_back(proc);
                }
#if 0
                for (std::map <int,int_vec_t>::iterator iter = tmp_map.begin(); iter != tmp_map.end(); ++iter) {
                    int_vec_t procs = iter->second;
                    /*
                     * We assign the first LCPU in the list as the core ID.
                     */
                    int core_id = procs[0];
                    sysInfo.m_htMap[core_id] = core_id;
                    for (int i=1; i<procs.size(); ++i) {
                        /*
                         * Subsequent LCPUs in this list are assigned to the same core.
                         */
                        sysInfo.m_htMap[procs[i]] = core_id;
                    }
                    if (core_id > max_core_id) {
                        max_core_id = core_id;
                    }
                }
#endif
                for (std::map <int, int_vec_t>::iterator iter = tmp_map.begin(); iter != tmp_map.end(); ++iter) {
                    int_vec_t procs = iter->second;
                    int core_id = ++curr_core_id;
                    for (int i=0; i<procs.size(); ++i) {
                        sysInfo.m_htMap[procs[i]] = core_id;
                        if (sysInfo.m_arch != MFD) {
                            m_htMap[procs[i]] = core_id;
                            m_threadSiblingMap[core_id].push_back(procs[i]);
                        }
                    }
                    if (core_id > max_core_id) {
                        max_core_id = core_id;
                    }
                }
                sysInfo.m_coreCount = max_core_id + 1;
                if (sysInfo.m_arch == MFD) {
                    /*
                     * For Cloverview/Clovertrail/Saltwell (basically, all Saltwell derivatives), associate all threads with a single core (for wakeups ONLY).
                     */
                    for (int i=0; i<sysInfo.m_cpuCount; ++i) {
                        m_htMap[i] = 0;
                        m_threadSiblingMap[0].push_back(i);
                    }
                    m_wakeupCoreCount = 1;
                    if (sysInfo.m_coreCount > 1) {
                        m_do_inject_dup_c_sample = true;
                    }
                } else {
                    m_wakeupCoreCount = sysInfo.m_coreCount;
                }
                {
                    int cpu;
                    db_fprintf(stderr, "SystemInfo::HT map dump...\n");
                    for_each_online_cpu(cpu) {
                        db_fprintf(stderr, "%d -> %d\n", cpu, sysInfo.m_htMap[cpu]);
                    }
                    db_fprintf(stderr, "WuParser::HT map dump...\n");
                    for_each_online_cpu(cpu) {
                        db_fprintf(stderr, "%d -> %d\n", cpu, m_htMap[cpu]);
                    }
                    if (g_do_debugging) {
                        fprintf(stderr, "Rev-HT map dump...\n");
                        for (cpu = 0; cpu < sysInfo.m_coreCount; ++cpu) {
                            std::cerr << cpu << ":->";
                            std::copy(m_threadSiblingMap[cpu].begin(), m_threadSiblingMap[cpu].end(), std::ostream_iterator<int>(std::cerr, ","));
                            std::cerr << "\n";
                        }
                    }
                }
            }
            db_assert(true, "Found CPU Topology=%s\n", sysInfo.m_cpuTopology.c_str());
        }
        else if (line.find("TSC Frequency") != std::string::npos) {
            int tsc_freq = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            sysInfo.m_tscFreq = tsc_freq;
            db_assert(true, "Found TSC freq = %d\n", tsc_freq);
            switch (sysInfo.m_arch) {
                case NHM:
                case SNB:
                    sysInfo.m_cStateMult = 1;
                    break;
                case MFD:
                    sysInfo.m_cStateMult = tsc_freq; // in MHz
                    break;
                default:
                    fprintf(stderr, "ERROR: invalid arch type = %u\n", sysInfo.m_arch);
                    abort();
            }
        }
        else if (line.find("Microcode patch version") != std::string::npos) {
            int micro_patch_ver = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            sysInfo.m_microPatchVer = micro_patch_ver;
            db_assert(true, "FOUND micro patch = %d\n", micro_patch_ver);
        }
        else if (line.find("Total Collection Time") != std::string::npos) {
            /*
             * Found collection time.
             */
            sysInfo.m_collectionTime = get_required_token_i(line, 5, 6);
            db_assert(true, "Found ctime = %s\n", sysInfo.m_collectionTime.c_str());
        }
        else if (line.find("Profiled Application") != std::string::npos) {
            /*
             * Found profiled app details.
             */
            str_vec_t toks = Tokenizer(line, " \t").get_all_tokens();
            assert(toks.size() == 8);
            sysInfo.m_appPID = atoi(toks[4].c_str());
            sysInfo.m_appProfiled = toks[7];
        }
        else if (line.find("HARDWARE C-STATE Mappings") != std::string::npos) {
            /*
             * Found ACPI <-> H/W C-STATE mapping info.
             */
            /*
             * We're going to be storing the mapping in
             * a vector -- reserve space for it here.
             */
            sysInfo.m_stateMapping.assign(MAX_MSR_ADDRESSES,99); /* Reserve space for 'MAX_MSR_ADDRESSES' slots, writing '99' into each */
            /*
             * This is NOT the LAST section -- iterate
             * until we get to the "TARGET RESIDENCIES" section.
             */
            while(!lines.empty()) {
                line = lines.front(); lines.pop_front();

                if (line.empty() || line.size() == 1) {
                    continue;
                } else if (line.find("TARGET RESIDENCIES") != std::string::npos) {
                    lines.push_front(line);
                    break;
                }

#define GET_NEXT_TOKEN(tok) ({const char *__tmp = tok.get_next_token(); assert(__tmp != NULL); __tmp;})

                Tokenizer tok(line, " \t");
                int ac = atoi(GET_NEXT_TOKEN(tok));
                str_t hws = GET_NEXT_TOKEN(tok);
                size_t endpos = hws.rfind('C');
                db_assert(endpos != std::string::npos, "MALFORMED ACPI mapping line!\n");
                /*
                 * Everything from 'endpos+1' to EOL
                 * is the H/W C-state number.
                 */
                int hw = atoi(&(hws.c_str())[++endpos]);
                sysInfo.m_stateMapping[ac] = hw;
                db_assert(true, "%d <-> %d\n", ac, hw);
            }
            std::vector<int> *vec = &sysInfo.m_stateMapping;
            db_copy(vec->begin(), vec->end(), std::ostream_iterator<int>(std::cerr, "\n"));
            db_assert(true, "TODO\n");
        }
        else if (line.find("TARGET RESIDENCIES") != std::string::npos) {
            while (!lines.empty()) {
                line = lines.front(); lines.pop_front();

                if (line.empty() || line.size() == 1) {
                    continue;
                } else if (line.find("AVAILABLE FREQUENCIES") != std::string::npos) {
                    lines.push_front(line);
                    break;
                }

                str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
                assert(tokens.size() == 3);
                int state = atoi(&(tokens[0].c_str())[1]);
                // sysInfo.m_targetRes[state] = (atoi(tokens[2].c_str())/1000);
                sysInfo.m_targetRes[state] = (atoi(tokens[2].c_str()));
            }

        }
        else if (DOES_LINE_CONTAIN(line, "AVAILABLE FREQUENCIES")) {
            str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
            assert(tokens.size() > 3);
            for (int i=3; i<tokens.size(); ++i) {
                m_availableFrequenciesKHz.push_back(atoi(tokens[i].c_str()));
            }
            db_copy(m_availableFrequenciesKHz.begin(), m_availableFrequenciesKHz.end(), std::ostream_iterator<u32>(std::cerr, "\n"));
            db_assert(true, "Found available freqs list! Size = %d\n", m_availableFrequenciesKHz.size());
        }
        else if (line.find("DESCENDENT PIDS LIST") != std::string::npos) {
            while (!lines.empty()) {
                int idx = -1;
                line = lines.front();
                lines.pop_front();
                if (!line.size()) {
                    continue;
                }
                if ((idx = line.find("PID")) == std::string::npos || idx != 0) {
                    // Different section?
                    lines.push_front(line);
                    break;
                }
                str_t pid_str = line.substr(4);
                db_fprintf(stderr, "Pid substr = %s\n", pid_str.c_str());
                std::stringstream full_name;
                full_name << m_wuwatch_output_dir << "/lib_output_" << pid_str << ".txt";
                db_fprintf(stderr, "FULL lib name = %s\n", full_name.str().c_str());
                m_lib_input_fps.push_back(fopen(full_name.str().c_str(), "r"));
            }
        }
    }

    /*
     * Also populate the timeline version.
     */
    char wudump_ver[100];
    sprintf(wudump_ver, "%d.%d.%d", WUWATCH_VERSION_VERSION, WUWATCH_VERSION_INTERFACE, WUWATCH_VERSION_OTHER);
    sysInfo.m_wudumpVersion = wudump_ver;

    return SUCCESS;
};


/*
 * INTERNAL API:
 * A function to read samples from the
 * binary file and check that they meet
 * sorting requirements.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_read_i(void)
{
    /*
     * We have a combined input file, containing driver output
     * and also system configuration information.
     */
    FILE *in_fp = fopen(m_combined_input_file_name.c_str(), "rb");
    if(!in_fp){
        perror("fopen error");
        return -ERROR;
    }
    u64 driver_beg_off = sizeof(u64), driver_end_off = 0;
    /*
     * Step (1): read the 'sys_params_found.txt'
     * file.
     */
    {
        if (get_sys_params_i(in_fp, driver_end_off)) {
            db_fprintf(stderr, "ERROR retrieving sys params!\n");
            return -ERROR;
        }
    }

    /*
     * Sanity tests!
     */
    assert(driver_beg_off <= driver_end_off);
    db_assert(sysInfo.m_cpuCount > 0, "ERROR: # cpus = %d\n", sysInfo.m_cpuCount);

    /*
     * Init the (per-cpu) lists.We CANNOT init those until we
     * read the # cpus from the 'sys_params_found.txt' file.
     * For now, we ASSUME MFLD.
     */
    {
        m_per_cpu_sample_lists = new sample_list_t[NUM_ONLINE_CPUS()];
        m_per_cpu_p_sample_lists = new sample_list_t[NUM_ONLINE_CORES()];
#if DO_TPS_EPOCH_COUNTER
        /*
         * Allocate per-core, NOT per-cpu.
         */
        m_per_cpu_sched_sample_lists = new sample_list_t[NUM_ONLINE_CORES()];
#endif
        if (m_do_dump_orig_samples) {
            m_all_orig_samples = new sample_list_t[NUM_ONLINE_CORES()];
        }
    }

    sample_vec_t samples(NUM_MSGS_TO_READ + 1);
    int num_read = 0;
    int cpu = 0;

    int num_k_call_stacks = 0;

    /*
     * Step (2): read the lib_outputXXX.txt files (if any).
     */
    {
        db_fprintf(stderr, "Size = %d\n", m_lib_input_fps.size());
        for(fp_iter_t iter = m_lib_input_fps.begin(); iter != m_lib_input_fps.end(); ++iter) {
            db_fprintf(stderr, "FP = %p\n", *iter);
            if (*iter == NULL) {
                db_fprintf(stderr, "WARNING: NULL fp for trace file; NOT opening!\n");
                continue;
            }
#if WAS_BINARY_DUMP
            Tracer::instance()->deserialize_traces((*iter), m_trace_vec, m_trace_pair_map);
            assert(false);
#else
            str_t ver_str;
            Tracer::instance()->read_traces((*iter), m_trace_vec, m_trace_pair_map, ver_str);
            /*
             * Ensure we get rid of any pesky
             * newlines.
             */
            size_t pos;
            if ( (pos = ver_str.rfind('\n')) != std::string::npos) {
                ver_str.erase(pos, std::string::npos);
            }
            sysInfo.m_hookLibraryVersion = ver_str;
#endif
        }
        /*
         * Make sure 'Tracer' is destroyed.
         */
        Tracer::destroy();
    }

    /*
     * Step (3): read (driver) samples from disk into (per-cpu) output 
     * vectors.
     */
    /*
     * First, we 'seek' back to the start of the driver data.
     */
    if (fseek(in_fp, driver_beg_off, SEEK_SET)) {
        perror("fseek error in do_read_i()");
        return -ERROR;
    }
    u64 num_driver_output_samples = (driver_end_off - driver_beg_off) / sizeof(PWCollector_sample_t);
    size_t num_samples_to_read = 0;
    db_fprintf(stderr, "DRIVER OUTPUT SIZE = %llu\n", num_driver_output_samples);

    /*
     * Then we read in the driver samples.
     */
    while (num_driver_output_samples > 0) {
        num_samples_to_read = std::min(num_driver_output_samples, (u64)NUM_MSGS_TO_READ);
        if ( (num_read = fread(&samples[0], sizeof(PWCollector_sample_t), num_samples_to_read, in_fp)) < num_samples_to_read) {
            perror("fread error while reading driver samples");
            break;
        }
        num_driver_output_samples -= num_read;
        
        for (int i=0; i<num_read; ++i) {
            PWCollector_sample_t *sample = &samples[i];
            sample_type_t sample_type = (sample_type_t)sample->sample_type;
            int coreid = get_core_given_lcpu_i(sample->cpuidx);
            bool is_boundary = sample->sample_type == P_STATE ? sample->p_sample.is_boundary_sample > 0 : false;

            if (m_do_dump_orig_samples) {
                m_all_orig_samples[coreid].push_back(*sample);
            }

            assert(sample->cpuidx < NUM_ONLINE_CPUS());
            /*
             * Specific actions depend on sample types.
             */
            switch (sample->sample_type) {
                case C_STATE:
                    /*
                     * Store the sample. These samples need to be post-processed, so we store them in
                     * a special (per-cpu) list.
                     */
                    m_per_cpu_sample_lists[sample->cpuidx].push_back(*sample);
                    break;
                case P_STATE:
                    /*
                     * For now, don't process TPF samples returned by the driver. Later, we 
                     * might want to include these samples for educational purposes (e.g. when
                     * did the OS request a P-state transition and when was it actually granted
                     * by H/W).
                     * UPDATE: we're now also measuring APERF, MPERF in P-state samples.
                     */
                    m_was_any_tpf_sample_present = true;
                    // m_per_cpu_sample_lists[sample->cpuidx].push_back(*sample);
                    m_per_cpu_p_sample_lists[GET_CORE_GIVEN_LCPU(sample->cpuidx)].push_back(*sample);
                    break;
                case K_CALL_STACK:
                    /*
                     * Store the kernel backtrace.
                     */
                    m_all_samples_list.push_back(*sample);
                    {
                        k_sample_t *ks = &sample->k_sample;
                        pid_t tid = ks->tid;
                        trace_t *trace = new trace_t(0, 0, NULL);
                        /*   
                         * Extract kernel backtrace symbols from
                         * the '/proc/kallsyms' file and append
                         * to SWAPPERs list of traces.
                         */
                        {    
                            std::vector<std::string> ksym_vec;
                            wuwatch::KernelSymbolExtractor::get_backtrace_symbols((const wuwatch::u64 *)ks->trace, ks->trace_len, ksym_vec);

                            trace->num_trace = ks->trace_len; // ksym_vec.size();
                            trace->bt_symbols = (char **)calloc(trace->num_trace, sizeof(char *)); 
                            assert(trace->bt_symbols);
                            for (int i=0; i<trace->num_trace; ++i) {
                                trace->bt_symbols[i] = strdup(ksym_vec[i].c_str());
                            }

                            trace_pair_list_t tlist;
                            tlist.push_back(trace_pair(ks->entry_tsc, ks->exit_tsc, trace));
                            m_trace_pair_map[tid].merge(tlist, std::less<trace_pair_t>());
                            db_fprintf(stderr, "BT: tid = %d, entry tsc = %llu, exit tsc = %llu\n", tid, (unsigned long long)ks->entry_tsc, (unsigned long long)ks->exit_tsc);
                        }
                        ++num_k_call_stacks; // Debugging ONLY!
                    }
                    break;
                case IRQ_MAP:
                case PROC_MAP:
                    /*
                     * 'IRQ_MAP' and 'PROC_MAP' samples need to be returned unchanged.
                     */
                    m_all_samples_list.push_back(*sample);
                    break;
                case TIMER_SAMPLE:
                case IRQ_SAMPLE:
                case WORKQUEUE_SAMPLE:
                    /*
                     * Store the sample in our per-cpu lists. Unlike most other samples (e.g. "S_RESIDENCY", "IRQ_MAP"), but
                     * like "TPS" samples, these need to be post-processed and will NOT be returned to the caller.
                     */
                    m_per_cpu_sample_lists[sample->cpuidx].push_back(*sample);
                    break;
                case SCHED_SAMPLE:
                    /*
                     * Store the sample in our per-cpu lists. Unlike most other samples (e.g. "S_RESIDENCY", "IRQ_MAP"), but
                     * like "TPS" samples, these need to be post-processed and will NOT be returned to the caller. We use
                     * the TARGET cpu to determine which cpu to delegate it to.
                     */
                    {
                        int target_cpu = sample->e_sample.data[1];
#if DO_TPS_EPOCH_COUNTER
                        if (NUM_ONLINE_CORES() > 1) {
                            m_per_cpu_sched_sample_lists[get_core_given_lcpu_i(target_cpu)].push_back(*sample);
                        } else {
                            /*
                             * SINGLE-core system: no need to try and correlate SCHED_WAKEUP samples with 
                             * TPS samples because SCHED_WAKEUP cannot cause wakeups on a single-core
                             * system!
                             */
                            m_per_cpu_sample_lists[target_cpu].push_back(*sample);
                        }
#else
                        m_per_cpu_sample_lists[target_cpu].push_back(*sample);
#endif
                    }
                    break;
                case IPI_SAMPLE:
                case TPE_SAMPLE:
                    /*
                     * Store the sample in our per-cpu lists. Unlike most other samples (e.g. "S_RESIDENCY", "IRQ_MAP"), but
                     * like "TPS" samples, these need to be post-processed and will NOT be returned to the caller.
                     */
                    m_per_cpu_sample_lists[sample->cpuidx].push_back(*sample);
                    break;
                default: /* S_RESIDENCY, S_STATE, D_STATE, W_STATE etc. */
                    /*
                     * All other samples need to be returned unchanged.
                     */
                    m_all_samples_list.push_back(*sample);
                    break;
            }
        }
    }
    /*
     * We don't need to read anything else from the file.
     */
    fclose(in_fp);
    /*
     * "fread" returns '0' on EOF and on ERROR. 
     * Check for errors here.
     */
    if (num_driver_output_samples > 0) {
      return -ERROR;
    }

    /*
     * Print some debugging info.
     */
    for_each_online_cpu(cpu) {
        db_fprintf(stderr, "[%d]: SIZE = %d\n", cpu, (int)m_per_cpu_sample_lists[cpu].size());
    }

    return SUCCESS;
};

#if DO_TPS_EPOCH_COUNTER
/*
 * Utility data structures / functions to account for the EPOCH-based
 * TPS <--> SCHED_WAKEUP synchronization.
 * Useful for big-core only!
 */
struct sched_sample_sorter {
    bool operator()(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2) {
        assert(s1.sample_type == SCHED_SAMPLE && s2.sample_type == SCHED_SAMPLE);
        return s1.e_sample.data[2] < s2.e_sample.data[2];
    };
};

void pwr::WuParser::do_sort_sched_list_i(int core)
{
    sample_list_t& sched_list = m_per_cpu_sched_sample_lists[core];
    sched_list.sort(sched_sample_sorter());
    /*
     * Sanity test -- make sure everything is actually sorted!
     */
    int prev_epoch = -1;
    for (sample_list_t::const_iterator citer = sched_list.begin(); citer != sched_list.end(); ++citer) {
        int curr_epoch = citer->e_sample.data[2];
        if (prev_epoch > curr_epoch) {
            fprintf(stderr, "[%d]: prev epoch = %d, current epoch = %d\n", core, prev_epoch, curr_epoch);
        }
        assert(curr_epoch >= prev_epoch);
        prev_epoch = curr_epoch;
    }
};

void pwr::WuParser::do_merge_sched_list_i(int core)
{
    sample_list_t &sample_list = m_per_cpu_sample_lists[core], &sched_list = m_per_cpu_sched_sample_lists[core];
    if (sample_list.empty()) {
        return;
    }

    sample_list_t::iterator sample_iter = sample_list.begin();

    while (sched_list.size()) {
        const PWCollector_sample& sched_sample = sched_list.front();
        assert(sched_sample.sample_type == SCHED_SAMPLE);
        int sched_epoch = sched_sample.e_sample.data[2];

        for (; sample_iter != sample_list.end(); ++sample_iter) {
            if (sample_iter->sample_type == C_STATE && sample_iter->c_sample.tps_epoch > sched_epoch) {
                /*
                 * Insert the sched sample BEFORE this sample.
                 */
                sample_iter = sample_list.insert(sample_iter, sched_sample);
                break;
            }
        }
        if (sample_iter == sample_list.end()) {
            /*
             * Couldn't find an appropriate location
             * within the list for this SCHED sample;
             * at it at the end.
             */
            sample_list.push_back(sched_sample);
        }
        sched_list.pop_front();
    }
};
#endif // DO_TPS_EPOCH_COUNTER

/*
 * INTERNAL API:
 * The meat of the algorithm. Sort (previously read) samples, 
 * create TPS groupings, measure Cx residencies, determine
 * wakeup causes etc.
 *
 * @returns: 0 on success, -1 on error
 */
int pwr::WuParser::do_parse_i(void)
{
    int cpu = 0, core = 0;
    sample_list_t *merged_list = NULL;
    u64 min_collection_start_tsc = 0, max_collection_stop_tsc = 0;
    std::map <int, sample_list_t> output_c_samples, output_p_samples;

    std::vector <sample_list_t *> merged_lists(NUM_ONLINE_CORES());
#if DO_TPS_EPOCH_COUNTER
    std::vector <sample_list_t *> merged_sched_lists(NUM_ONLINE_CORES());
#endif

    /*
     * Basic algo:
     * (1) (If required) sort data in increasing TSC order.
     * (2) Merge both logical CPU data into a single stream
     *	   (i.e. convert from LCPU --> Core).
     * (3) Create "TPS groups" from the merged stream.
     * (4) Calculate wakeup causes, Cx residencies etc. between
     *	   the various TPS groups.
     */

    /*
     * (1) Check sorting requirements.
     * (2) Merge (sorted) data into a single core-stream.
     */
    for_each_online_cpu(cpu) {
        if (check_sorted_i(cpu) == false) {
            // Sort data here!
            sort_data_i(cpu);
            assert (check_sorted_i(cpu) == true);
        }
        int core_id = get_core_given_lcpu_i(cpu);
        if (merged_lists[core_id] == NULL) {
            merged_lists[core_id] = m_per_cpu_sample_lists + cpu;
        } else {
            merged_lists[core_id]->merge(m_per_cpu_sample_lists[cpu]);
        }
        m_per_cpu_p_sample_lists[core_id].sort();
        core_id = GET_CORE_GIVEN_LCPU(cpu);
        output_p_samples[core_id] = m_per_cpu_p_sample_lists[core_id];
    }
    /*
     * We do a final "post-process" sort to rearrange
     * SCHED wakeup samples from other cores (because
     * the TSC isn't guaranteed to be synchronized across cores, we
     * use an "EPOCH" based synchronization mechanism
     * between SCHED and TPS samples).
     */
#if DO_TPS_EPOCH_COUNTER
    // for_each_online_core(core) {
    for (core=0; core<m_wakeupCoreCount; ++core) {
        /*
         * First, sort the list of sched samples (in EPOCH order).
         */
        do_sort_sched_list_i(core);
        /*
         * Then, add these sched samples to the main 'per_cpu'
         * list of samples (again, in EPOCH order)
         */
        do_merge_sched_list_i(core);
    }
#endif
    
    db_fprintf(stderr, "There are %d online cpus and %d online cores!\n", NUM_ONLINE_CPUS(), NUM_ONLINE_CORES());
    db_fprintf(stderr, "m_wakeupCoreCount = %d\n", m_wakeupCoreCount);
    // for_each_online_core(core) {
    for (core=0; core<m_wakeupCoreCount; ++core) {
        db_assert(merged_lists[core] != NULL, "ERROR: core %d had a NULL list!\n", core);
        db_fprintf(stderr, "Core [%d]: # samples = %d\n", core, merged_lists[core]->size());
    }

    /*
     * Sort and print out the list of original samples, but
     * only if asked to do so.
     */
    if (m_do_dump_orig_samples) {
        db_fprintf(stderr, "DUMPING original samples\n");
        sample_list_t tmp_merged_list;
        // for_each_online_core(core) {
        for (core=0; core<m_wakeupCoreCount; ++core) {
            sample_list_t& samples = m_all_orig_samples[core];
            samples.sort();
            // std::copy(samples.begin(), samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
            tmp_merged_list.merge(samples);
        }
        std::copy(tmp_merged_list.begin(), tmp_merged_list.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
    }


    // for_each_online_core(core) {
    for (core=0; core<m_wakeupCoreCount; ++core) {
        int num_tps = 0;
        pair_vec_t group_indices;
        aperf_mperf_map_t aperf_mperf_map;
        sample_list_t& core_output_c_samples = output_c_samples[core];
        sample_list_t& core_output_p_samples = output_p_samples[core];

        db_fprintf(stderr, "core %d: # tpf samples = %u\n", core, core_output_p_samples.size());

        merged_list = merged_lists[core];

        /*
         * (3) Create "TPS groups".
         */
        if (create_tps_groups_i(merged_list, group_indices, num_tps)) {
            db_fprintf(stderr, "ERROR creating TPS group indices!\n");
            return -ERROR;
        }
        db_fprintf(stderr, "There were %d groups in total! # TPS = %d\n", (int)group_indices.size(), num_tps);

        /*
         * Convenience: convert 'std::list' to 'std::vector'
         */
        sample_vec_t merged_vec(merged_list->size());
        std::copy(merged_list->begin(), merged_list->end(), merged_vec.begin());
        db_fprintf(stderr, "Merged vec size = %d\n", (int)merged_vec.size());

        if (m_do_dump_tps_groups) {
            dump_tps_groups_i(merged_vec, group_indices);
        }

        /*
         * (4) Calculate wakeup causes, Cx residencies etc. between
         *     the various TPS groups.
         */
        int size = group_indices.size();
        u32 prev_actual_freq = 0;
        int first_group_beg = -1, first_group_end = -1;

        if (size == 0) {
            /*
             * No TPS group was found. This can happen if
             *      (a) The user did NOT request C-state information (i.e. did
             *          not pass the "-cs" switch to wuwatch)
             *      (b) No TPS groups were found because the core never entered
             *          a sleep state deeper than C1. THIS IS UNLIKELY!!!
             */
            if (num_tps > 0) {
                /*
                 * User asked for a C-state collection, but core never entered C2 or deeper.
                 * HIGHLY UNLIKELY!!!
                 */
                for (first_group_beg = 0; first_group_beg < merged_vec.size() && merged_vec[first_group_beg].sample_type != C_STATE; ++first_group_beg) {}
                for (first_group_end = merged_vec.size()-1; first_group_end >= 0 && merged_vec[first_group_end].sample_type != C_STATE; --first_group_end) {}
                assert (first_group_beg >= 0 && first_group_end >= 0);
            }
        } else {
            first_group_beg = group_indices[0].first;
            first_group_end = group_indices[0].second;
        }

        /*
         * We need to apply our C1 algorithm to the first group, REGARDLESS
         * OF WHETHER THE USER REQUESTED C1 SAMPLES! This is because the
         * collection START TSC is defined as the first 
         * Core mwait TSC. In other words, the TSC when the core first entered 
         * a sleep state. This first sleep state may (or may not) be a C1 state. 
         * So we need to calculate this state, regardless of whether we're going 
         * to display it.
         *
         * UPDATE: collection START TSC now defined as the first {IRQ,TIMER,TPS,TPF} sample
         * returned by the driver. This is to avoid a large "white space" at the start
         * of the collection in cases where the cores are heavily loaded and are mostly in
         * C0.
         */
        if (merged_vec.empty() == false) {
            if (min_collection_start_tsc == 0 || min_collection_start_tsc > merged_vec.front().tsc) {
                min_collection_start_tsc = merged_vec.front().tsc;
            }
        }
        if (m_do_check_c1_res) {
            /*
             * We need to generate C1 samples. We have two cases:
             * (1) No TPS group was found. See comments above regarding when this
             *     can happen.
             * (2) TPS groups WERE found. In this case, we need to parse the
             *     first TPS group here because the 'for(...)' loop below starts
             *     off with the second group.
             */
            sample_vec_t c1_output_samples;
            u64 dont_care_tsc = 0;

            if (first_group_beg >= 0 && first_group_end > 0) {
                /*
                 * Calculate C1 residencies for the first group.
                 */
                if (do_c1_calcs_i(merged_vec, first_group_beg, first_group_end, c1_output_samples, core, 0, dont_care_tsc)) {
                    db_fprintf(stderr, "ERROR calculating C1 residencies for previous group!\n");
                } else if (c1_output_samples.empty() == false) {
                    /*
                     * We have some initial C1 samples -- calculate the 'start tsc' from those.
                     */
                    u64 first_c1_tsc = c1_output_samples.front().tsc;
                    const c_sample_t *cs = &c1_output_samples.front().c_sample;
                    // u64 &collection_start_tsc = m_core_tsc_map[get_core_given_lcpu_i(c1_output_samples.front().cpuidx)].first;
                    u64 first_c0_res = RES_COUNT(*cs, MPERF), first_c1_res = RES_COUNT(*cs, APERF);
                    u64 collection_start_tsc = first_c1_tsc - first_c0_res - first_c1_res;
                    if (min_collection_start_tsc == 0 || collection_start_tsc < min_collection_start_tsc) {
                        min_collection_start_tsc = collection_start_tsc;
                    }
                    db_fprintf(stderr, "First C1 tsc = %llu, C0 res = %llu, C1 res = %llu, first_mwait_tsc = %llu\n", first_c1_tsc, first_c0_res, first_c1_res, collection_start_tsc);
                    /*
                     * OK, computed collection_start_tsc. Now add these to the list of output samples
                     */
                    if (m_do_check_c1_res) {
                        /*
                         * Used asked for C1 samples.
                         */
                        core_output_c_samples.insert(core_output_c_samples.end(), c1_output_samples.begin(), c1_output_samples.end());
#if 0
                        if (m_do_inject_dup_c_sample) {
                            int other_core = 1 - core; // should ALWAYS be '1'!!!
                            assert(other_core == 1);
                            for (sample_vec_t::iterator c1_iter = c1_output_samples.begin(); c1_iter != c1_output_samples.end(); ++c1_iter) {
                                c1_iter->cpuidx = other_core;
                                output_c_samples[other_core].push_back(*c1_iter);
                            }
                        }
#endif
                    }
                }
            }
        }

        /*
         * We need to keep a running count of the APERF, MPERF values
         * (to determine operating frequencies). Do that for the first
         * group here because the algo below starts off with the
         * second group.
         */
        if (m_was_any_tpf_sample_present) {
            int tpf_begin = -1, tpf_end = -1;
            sample_list_t tmp_p_samples;

            if (num_tps > 0) {
                tpf_end = first_group_end;
            } else {
                /*
                 * No TPS samples present, but we DID have some TPF samples. Find
                 * the last one.
                 */
                for (tpf_end = merged_vec.size()-1; tpf_end >= 0 && merged_vec[tpf_end].sample_type != P_STATE; --tpf_end) {}
            }
            assert(tpf_end >= 0);

#if 0
            if (calc_aperf_mperf_deltas_i(merged_vec, -1, tpf_end, aperf_mperf_map, tmp_p_samples)) { // "-1" because algo starts with 'prev_end+1'
                db_fprintf(stderr, "ERROR cannot calculate aperf/mperf deltas!\n");
                db_assert(false, "ERROR cannot calculate aperf/mperf deltas!\n");
            }
#endif
            core_output_p_samples.insert(core_output_p_samples.end(), tmp_p_samples.begin(), tmp_p_samples.end());
        }

        for (int i=1; i<size; ++i) {
            int prev_beg = group_indices[i-1].first, prev_end = group_indices[i-1].second, curr_begin = group_indices[i].first, curr_end = group_indices[i].second;
            PWCollector_sample_t prev_beg_sample = merged_vec[prev_beg], prev_end_sample = merged_vec[prev_end], curr_begin_sample = merged_vec[curr_begin], curr_end_sample = merged_vec[curr_end];
            int which_cx = -1;
            u64 cx_delta = 0, c0 = 0, c0_c1 = 0;
            u32 cause = SAMPLE_TYPE_END;
            PWCollector_sample_t wakeup_sample;
            int requested_cx = -1;
            double aperf_mperf_ratio = 0.0;
            u32 actual_freq = 0;
            sample_vec_t c1_output_samples;
            sample_list_t tmp_p_samples;
            u64 prev_cx_begin_tsc = prev_end_sample.tsc;
            u64 prev_cx_end_tsc = curr_end_sample.tsc;

            memset(&wakeup_sample, 0, sizeof(wakeup_sample));


            /*
             * Calculate 'C1' groups for the previous TPS "group".
             * Those need to be printed before the 'Cx' etc. from
             * the current group.
             */
            if (m_do_check_c1_res) {
                if (do_c1_calcs_i(merged_vec, curr_begin, curr_end, c1_output_samples, core, i, prev_cx_end_tsc)) {
                    db_fprintf(stderr, "ERROR calculating C1 residencies for previous group!\n");
                }
            }
            if (prev_cx_end_tsc == 0) {
                prev_cx_end_tsc = curr_end_sample.tsc;
            }
            /*
             * Calculate Cx, C0 for this TPS "group".
             * We use the END of the PREV group and the END of the
             * CURR group for these calculations.
             */
            if (calc_inter_group_cx_c0_res_i(prev_end_sample, curr_end_sample, which_cx, cx_delta, c0, prev_cx_begin_tsc, prev_cx_end_tsc)) {
                if (g_do_debugging) {
                    fprintf(stderr, "ERROR calculating residencies! prev_end = %d, curr_end = %d, i = %d, size = %d\n", prev_end, curr_end, i, size);
                    std::cerr << prev_end_sample;
                    std::cerr << curr_end_sample;
                }
                return -ERROR;
            }
            /*
             * Calculate requested Cx state for this current group.
             * We use the TPS samples from the PREVIOUS group for
             * this (under current scheme, TPS samples encode
             * the state the logical CPU is REQUESTING).
             */
            if (calc_cx_mwait_hint_i(merged_vec, prev_beg, prev_end, requested_cx)) {
                if (g_do_debugging) {
                    fprintf(stderr, "ERROR calculating cx mwait hint! prev_beg = %d, prev_end = %d\n", prev_beg, prev_end);
                    std::cerr << prev_beg_sample;
                    std::cerr << prev_end_sample;
                }
                requested_cx = prev_end_sample.c_sample.prev_state;
            }
            /*
             * The only reason we can't determine the 'mwait hint' is if
             * we have only one TPS sample in this group -- this usually happens
             * for the LAST TPS group. For now, just take the mwait from the single
             * TPS sample in this group.
             */
            if (requested_cx < 0) {
                assert (prev_end == prev_beg);
                requested_cx = prev_end_sample.c_sample.prev_state;
            }

            /*
             * Convert 'requested_cx' to the ACTUAL C-state using
             * our mapping table.
             */
            requested_cx = GET_C_STATE_GIVEN_TPS_HINT(requested_cx);
            /*
             * Find wakeup cause for this TPS "group".
             * We use the END of the PREV group and the END of the
             * CURR group for this. Note that we need to go to the
             * END of the CURR group because there is a race between the
             * two LCPUs -- the core might wake up because of, e.g., an IRQ which is
             * being serviced by LCPU-1. In the meantime, LCPU-0, which has
             * nothing else to do, may well hit TPS AGAIN (i.e. BEFORE the IRQ
             * tracepoint on LCPU-1 fires).
             */
            if (calc_wakeup_cause_i(merged_vec, prev_end, curr_end, false, &wakeup_sample)) {
                if (g_do_debugging) {
                    fprintf(stderr, "ERROR calculating wakeup cause!\n");
                    std::cerr << prev_end_sample;
                    std::cerr << curr_end_sample;
                }
                cause = SAMPLE_TYPE_END;
            } else {
                cause = wakeup_sample.sample_type;
                db_fprintf(stderr, "cause = %d\n", cause);
            }
            /*
             * Calculate APERF/MPERF ratio -- we use that to determine if
             * the core moved to a different P-state.
             */
            if (m_was_any_tpf_sample_present) {
#if 0
                if (calc_aperf_mperf_deltas_i(merged_vec, prev_end, curr_end, aperf_mperf_map, tmp_p_samples)) {
                    db_fprintf(stderr, "ERROR cannot calculate aperf/mperf deltas!\n");
                    db_assert(false, "ERROR cannot calculate aperf/mperf deltas!\n");
                }
#endif
                core_output_p_samples.insert(core_output_p_samples.end(), tmp_p_samples.begin(), tmp_p_samples.end());
            }

            /*
             * OK, all 'Cx' calcs done. Now add to list of
             * output samples. Note: we use the END TSC of
             * the CURR group here because that's what the
             * previous versions of 'wudump' used.
             * **************************************************
             * UPDATE: WE SHOULD BE USING THE END TSC of the
             * FIRST C1 GROUP WITHIN THIS CX GROUP INSTEAD!!!
             * **************************************************
             */
            core_output_c_samples.push_back(create_tps_collector_sample_i(cause, which_cx, requested_cx, core, prev_cx_end_tsc, c0, cx_delta, wakeup_sample));
            if (c1_output_samples.size()) {
                /*
                 * And finally, add all the output samples corresponding
                 * to 'C1' samples within this group.
                 */
                core_output_c_samples.insert(core_output_c_samples.end(), c1_output_samples.begin(), c1_output_samples.end());
            }
#if 0
            if (m_do_inject_dup_c_sample) {
                int other_core = 1 - core; // should ALWAYS be '1'!!!
                assert(other_core == 1);
                output_c_samples[other_core].push_back(create_tps_collector_sample_i(cause, which_cx, requested_cx, other_core, prev_cx_end_tsc, c0, cx_delta, wakeup_sample));
                db_fprintf(stderr, "Injected dup sample (%d, %d)\n", core, other_core);
                for (sample_vec_t::iterator c1_iter = c1_output_samples.begin(); c1_iter != c1_output_samples.end(); ++c1_iter) {
                    c1_iter->cpuidx = other_core;
                    output_c_samples[other_core].push_back(*c1_iter);
                }
            }
#endif

            db_fprintf(stderr, "[%d, %d, %d]: c0 = %llu which_cx = %d cx_delta = %llu cause = %s\n", prev_end, curr_begin, curr_end, (unsigned long long)c0, which_cx, (unsigned long long)cx_delta, s_long_sample_names[cause]);
        }
        /*
         * There may be some samples of interest after the last group (e.g. boundary P-state samples etc.).
         * Check for them and handle them appropriately here.
         */
        if (m_was_any_tpf_sample_present && size > 0 && group_indices[size-1].second != merged_vec.size()) {
            sample_list_t tmp_p_samples;
#if 0
            if (calc_aperf_mperf_deltas_i(merged_vec, group_indices[size-1].second, merged_vec.size()-1, aperf_mperf_map, tmp_p_samples)) {
                db_fprintf(stderr, "ERROR cannot calculate aperf/mperf deltas!\n");
                db_assert(false, "ERROR cannot calculate aperf/mperf deltas!\n");
            }
#endif
            core_output_p_samples.insert(core_output_p_samples.end(), tmp_p_samples.begin(), tmp_p_samples.end());
        }

        /*
         * Update: our algorithm allows us to have one UNKNOWN wakeup sample per core if
         * the following conditions are met:
         * (a) HT is enabled and
         * (b) This is the LAST wakeup sample for this core.
         *
         * We check for both conditions here and remove this sample if it has an
         * unknown wakeup reason.
         */
        if (WAS_SYSTEM_HYPER_THREADED() && core_output_c_samples.back().c_sample.break_type == PW_BREAK_TYPE_U) {
            if (g_do_debugging) {
                fprintf(stderr, "Warning: LAST SAMPLE had an UNKNOWN wakeup; removing!\n");
                std::cerr << core_output_c_samples.back();
            }
            core_output_c_samples.pop_back();
        }

        /*
         * OK, created we're done creating C-state samples. But we need to account for "ghost" time
         * which is the C0 time between the collection START and the FIRST C-state sample and also
         * the C0 time between the LAST C-state sample and collection STOP. To do so, we inject two
         * spurious C-state samples with the requisite C0 residency and zero Cx residency (where 
         * 'Cx' is either 'C1' if the user specified C1 residency checking, or the next higher
         * C-state if not).
         */
        u64& collection_start_tsc = m_core_tsc_map[core].first, &collection_stop_tsc = m_core_tsc_map[core].second;

        if (core_output_c_samples.size() > 0) {
            const u64& first_c_state_tsc = core_output_c_samples.front().tsc, &last_c_state_tsc = core_output_c_samples.back().tsc;
            const c_sample_t *first_cs = &core_output_c_samples.front().c_sample, *last_cs = &core_output_c_samples.back().c_sample;
            /*
             * The TSC of the first 'mwait' ever seen by this CORE (and not just a given logical CPU). 
             * This is equal to the TSC of the first output C-state sample - the (C0 + Cx) residencies 
             * of that sample.
             */
            u64 first_mwait_tsc = first_c_state_tsc - RES_COUNT(*first_cs, MPERF) - RES_COUNT(*first_cs, GET_CX_FROM_COMBINED(first_cs->tps_epoch));

            assert (collection_start_tsc == 0 && collection_stop_tsc == 0);

            collection_start_tsc = first_mwait_tsc;
            collection_stop_tsc = last_c_state_tsc;
        }
        /*
         * It's possible for the user to specify a pure P,S,D state collection etc. In this case
         * the 'collection_start' and 'collection_stop' tscs won't be set. Check for that here.
         */
        if (collection_start_tsc == 0 && core_output_p_samples.empty() == false) {
            collection_start_tsc = core_output_p_samples.front().tsc;
        }
        if (collection_stop_tsc == 0 && core_output_p_samples.empty() == false) {
            collection_stop_tsc = core_output_p_samples.back().tsc;
        }
        /*
         * Keep track of the collection start and stop TSCs. These are defined as the minimum
         * and maximum of the collection start/stop TSCs of each core, respectively.
         */
        if (min_collection_start_tsc == 0 || collection_start_tsc <= min_collection_start_tsc) {
            min_collection_start_tsc = collection_start_tsc;
        }
        if (max_collection_stop_tsc == 0 || collection_stop_tsc >= max_collection_stop_tsc) {
            max_collection_stop_tsc = collection_stop_tsc;
        }
    }
    /*
     * OK, all cores have been visited. Now update the collection start, stop TSCs, generate boundary (or "ghost") samples (but only if required),
     * generate the P-state samples and compute the 'sysInfo.m_collectionTime' value.
     */
    // for_each_online_core(core) {
    for (core=0; core<m_wakeupCoreCount; ++core) {
        sample_list_t& core_output_c_samples = output_c_samples[core];
        // sample_list_t& core_output_p_samples = output_p_samples[core];
        /*
         * We need to account for "ghost" time which is the C0 time between the collection START and 
         * the FIRST C-state sample and also the C0 time between the LAST C-state sample and collection
         * STOP. To do so, we inject two spurious C-state samples with the requisite C0 residency and 
         * zero Cx residency (where 'Cx' is either 'C1' if the user specified C1 residency checking, 
         * or the next higher C-state if not).
         */
        u64 &core_collection_start_tsc = m_core_tsc_map[core].first, &core_collection_stop_tsc = m_core_tsc_map[core].second;
        db_fprintf(stderr, "Core %d: start TSC = %llu, overall start TSC = %llu\n", core, core_collection_start_tsc, min_collection_start_tsc);
        if (min_collection_start_tsc > 0 && core_output_c_samples.size() > 0) {
            if (core_collection_start_tsc > min_collection_start_tsc) {
                /*
                 * Add a "ghost" wakeup sample at the beginning to adjust for
                 * C0+C1 time. This wakeup sample will have the following
                 * characteristics:
                 * 1. TSC == core_collection_start_tsc;
                 * 2. C0 res == (core_collection_start_tsc - min_collection_start_tsc)
                 * 3. Cx res == 0
                 * 4. 'Which Cx' == {Lowest sleep state allowed by the user}
                 * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                 * 6. Wakeup Cause == 'BOGUS'
                 * ---------------------------------------------------------------
                 * UPDATE: do this ONLY if the user asked for C-state samples!!!
                 * ---------------------------------------------------------------
                 */
                u64 c0_res = (core_collection_start_tsc - min_collection_start_tsc);
                int which_cx = m_do_check_c1_res ? APERF : C2;
                for (; which_cx < MAX_MSR_ADDRESSES; ++which_cx) {
                    if (sysInfo.m_stateMapping[which_cx] != 99) {
                        break;
                    }
                }
                assert (which_cx >= APERF && which_cx < MAX_MSR_ADDRESSES);
                PWCollector_sample_t tmp_wakeup_sample = {core};
                PWCollector_sample_t ghost_sample = create_tps_collector_sample_i(FREE_SAMPLE, which_cx, which_cx /* requested cx */, core, core_collection_start_tsc, c0_res, 0 /* cx res */, tmp_wakeup_sample);
                core_output_c_samples.insert(core_output_c_samples.begin(), ghost_sample);
#if 0
                   if (m_do_inject_dup_c_sample) {
                   int other_core = 1 - core; // should ALWAYS be '1'!!!
                   assert(other_core == 1);
                   PWCollector_sample_t other_tmp_wakeup_sample = {other_core};
                   PWCollector_sample_t other_ghost_sample = create_tps_collector_sample_i(FREE_SAMPLE, which_cx, which_cx , other_core, core_collection_start_tsc, c0_res, 0, other_tmp_wakeup_sample);
                   output_c_samples[other_core].insert(output_c_samples[other_core].begin(), other_ghost_sample);
                   }
#endif
                // core_collection_start_tsc = min_collection_start_tsc;
            }
        }
        if (max_collection_stop_tsc > 0 && core_output_c_samples.size() > 0) {
            if (core_collection_stop_tsc < max_collection_stop_tsc) {
                db_fprintf(stderr, "Core %d: stop TSC = %llu, max stop TSC = %llu\n", core, core_collection_stop_tsc, max_collection_stop_tsc);
                /*
                 * Add a "ghost" wakeup sample at the end to adjust for
                 * C0+C1 time. This wakeup sample will have the following
                 * characteristics:
                 * 1. TSC == max_collection_stop_tsc;
                 * 2. C0 res == (max_collection_stop_tsc - core_collection_stop_tsc)
                 * 3. Cx res == 0
                 * 4. 'Which Cx' == {Lowest sleep state allowed by the user}
                 * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                 * 6. Wakeup Cause == 'BOGUS'
                 * ---------------------------------------------------------------
                 * UPDATE: do this ONLY if the user asked for C-state samples!!!
                 * ---------------------------------------------------------------
                 */
                u64 c0_res = (max_collection_stop_tsc - core_collection_stop_tsc);
                /*
                 * It's possible there's already a "ghost" sample for this core (introduced above, when we check
                 * the no-C1 case). In this case, don't introduce a new one. Instead, merely modify the previous
                 * sample.
                 */
                int which_cx = m_do_check_c1_res ? APERF : C2;
                for (; which_cx < MAX_MSR_ADDRESSES; ++which_cx) {
                    if (sysInfo.m_stateMapping[which_cx] != 99) {
                        break;
                    }
                }
                assert (which_cx >= APERF && which_cx < MAX_MSR_ADDRESSES);
                PWCollector_sample_t tmp_wakeup_sample = {core};
                PWCollector_sample_t ghost_sample = create_tps_collector_sample_i(FREE_SAMPLE, which_cx, which_cx /* requested cx */, core, max_collection_stop_tsc, c0_res, 0 /* cx res */, tmp_wakeup_sample);
                core_output_c_samples.insert(core_output_c_samples.end(), ghost_sample);
#if 0
                if (m_do_inject_dup_c_sample) {
                    int other_core = 1 - core; // should ALWAYS be '1'!!!
                    assert(other_core == 1);
                    PWCollector_sample_t other_tmp_wakeup_sample = {other_core};
                    PWCollector_sample_t other_ghost_sample = create_tps_collector_sample_i(FREE_SAMPLE, which_cx, which_cx /* requested cx */, other_core, max_collection_stop_tsc, c0_res, 0 /* cx res */, other_tmp_wakeup_sample);
                    output_c_samples[other_core].insert(output_c_samples[other_core].begin(), other_ghost_sample);
                }
#endif
                // core_collection_stop_tsc = max_collection_stop_tsc;
            }
        }
        if (min_collection_start_tsc > 0) {
            core_collection_start_tsc = min_collection_start_tsc;
        }
        if (max_collection_stop_tsc > 0) {
            core_collection_stop_tsc = max_collection_stop_tsc;
        }
        /*
         * OK, now calculate the actual frequencies. We have a list of APERF/MPERF ratios -- use
         * those to calculate the frequency the core was actually executing at. We also ensure
         * the TSC of the first (i.e. 'START boundary' sample isn't less than our 'collection_start_tsc' value, 
         * and that the TSC of the last (i.e. 'END boundary' sample isn't greater than our
         * 'collection_stop_tsc' value. 
         */
#if 0
        if (calc_actual_frequencies_i(core_output_p_samples, core_collection_start_tsc, core_collection_stop_tsc)) {
            db_assert(false, "ERROR cannot calculate actual frequencies!\n");
        }
#endif
        /*
         * OK, all adjustments done. Now add this core's output samples to the
         * list of global samples.
         * First, the C-state samples.
         */
        m_output_samples.insert(m_output_samples.end(), core_output_c_samples.begin(), core_output_c_samples.end());
#if 0
        /*
         * And then, the P-state samples, if any.
         */
        m_output_p_samples.insert(m_output_p_samples.end(), core_output_p_samples.begin(), core_output_p_samples.end());
#endif
        if (m_do_inject_dup_c_sample) {
            int other_core = 1 - core; // should ALWAYS be 1!
            assert(other_core == 1);
            for (sample_list_t::iterator iter = core_output_c_samples.begin(); iter != core_output_c_samples.end(); ++iter) {
                PWCollector_sample_t other_sample = *iter;
                other_sample.cpuidx = other_core;
                m_output_samples.push_back(other_sample);
            }
        }
        /*
         * OK, sample generation done. Now calculate the total elapsed time.
         */
        if (core_collection_start_tsc > 0 && core_collection_stop_tsc > 0) {
            assert(core_collection_start_tsc <= core_collection_stop_tsc);
            u64 core_collection_time = (core_collection_stop_tsc - core_collection_start_tsc);
            db_fprintf(stderr, "Core %d: start TSC = %llu, stop TSC = %llu collection time = %llu\n", core, core_collection_start_tsc, core_collection_stop_tsc, core_collection_time);
            // m_total_collection_time = std::max(m_total_collection_time, core_collection_time);
            /*
             * Note: to maintain compatibility with the 'summary' script output, we add up the elapsed times
             * for all cores and then divide by the number of cores.
             * An alternate approach would be to take the maximum elapsed time of all cores.
             */
            m_total_collection_time += core_collection_time;
            if (m_do_inject_dup_c_sample) {
                m_total_collection_time += core_collection_time;
            }
        }
        if (m_do_inject_dup_c_sample) {
            int other_core = 1 - core; // should ALWAYS be '1'!!!
            assert(other_core == 1);
            m_core_tsc_map[other_core].first = core_collection_start_tsc; m_core_tsc_map[other_core].second = core_collection_stop_tsc;
        }
    }
    for_each_online_core(core) {
        sample_list_t& core_output_p_samples = output_p_samples[core];
        u64 &core_collection_start_tsc = m_core_tsc_map[core].first, &core_collection_stop_tsc = m_core_tsc_map[core].second;
        /*
         * OK, now calculate the actual frequencies. We have a list of APERF/MPERF ratios -- use
         * those to calculate the frequency the core was actually executing at. We also ensure
         * the TSC of the first (i.e. 'START boundary' sample isn't less than our 'collection_start_tsc' value, 
         * and that the TSC of the last (i.e. 'END boundary' sample isn't greater than our
         * 'collection_stop_tsc' value. 
         */
        if (calc_actual_frequencies_i(core_output_p_samples, core_collection_start_tsc, core_collection_stop_tsc)) {
            db_assert(false, "ERROR cannot calculate actual frequencies!\n");
        }
        /*
         * OK, all adjustments done. Now add this core's P-state samples to the
         * list of global samples.
         */
        m_output_p_samples.insert(m_output_p_samples.end(), core_output_p_samples.begin(), core_output_p_samples.end());
    }
    db_fprintf(stderr, "total collection time = %llu\n", m_total_collection_time);
    double avg_collection_time = (double)m_total_collection_time / (double)sysInfo.m_tscFreq /* MHz */; // in usecs
    avg_collection_time /= 1e6; // in secs
    avg_collection_time /= NUM_ONLINE_CORES(); // avg/core
    if (avg_collection_time > 0.0) { // sanity!
        char tmp[10];
        sprintf(tmp, "%.2f", avg_collection_time);
        db_fprintf(stderr, "%s, %f\n", tmp, avg_collection_time);
        sysInfo.m_collectionTime = tmp;
    }
    /*
     * Sanity test: iterate through the C-state samples, ensuring that
     * Cx + C0 == Delta-TSC for each pair of such samples.
     * Update: this should be performed on a per-core basis!
     */
    if (false) {
        sample_list_t::const_iterator citer = m_output_samples.begin();
        u64 prev_tsc = citer->tsc;
        for (++citer; citer != m_output_samples.end(); ++citer) {
            if (g_do_debugging) {
                std::cerr << *citer;
            }
            u64 curr_tsc = citer->tsc;
            const c_sample_t *curr_cs = &citer->c_sample;
            int which_cx = GET_CX_FROM_COMBINED(curr_cs->tps_epoch);
            u64 delta_tsc = curr_tsc - prev_tsc;
            u64 delta_c0 = RES_COUNT(*curr_cs, MPERF), delta_cx = RES_COUNT(*curr_cs, which_cx), sigma = delta_c0 + delta_cx;
            s64 diff = delta_tsc - sigma;
            db_fprintf(stderr, "Delta TSC = %llu, Delta C0 = %llu, Delta Cx = %llu, SIGMA = %llu, DIFF = %lld\n", delta_tsc, delta_c0, delta_cx, sigma, diff);
            if (diff != 0) {
                fprintf(stderr, "ERROR: diff is %lld: delta tsc = %llu, sigma = %llu, prev tsc = %llu\n", diff, delta_tsc, sigma, prev_tsc);
                assert(false);
            }
            prev_tsc = curr_tsc;
        }
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Collate and sort all parsed/read data in preparation for sending back
 * to the client.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_sort_and_collate_i(void)
{
    /*
     * First, make sure the 'all_samples' list is sorted.
     * Once that is done, we'll merge the (already sorted)
     * 'output_samples' and 'output_p_samples' lists with
     * it to create the final (globally sorted) list.
     */
#if 0
    m_all_samples_list.sort();
    m_all_samples_list.merge(m_output_samples);
    m_all_samples_list.merge(m_output_p_samples);
#else // if 1
    m_all_samples_list.insert(m_all_samples_list.end(), m_output_samples.begin(), m_output_samples.end());
    m_all_samples_list.insert(m_all_samples_list.end(), m_output_p_samples.begin(), m_output_p_samples.end());
    m_all_samples_list.sort();
#endif

    return SUCCESS;
};


/*
 * INTERNAL API:
 * Function to pretty print results. All output sent
 * to 'stdout'. Used in debugging only.
 */
void pwr::WuParser::dump_tps_groups_i(const sample_vec_t& sorted_vec, const pair_vec_t& indices)
{
    fprintf(stderr, "---TPS-GROUPS-DUMP-BEGIN---\n");
    int group = 0;
    for (pair_vec_t::const_iterator citer = indices.begin(); citer != indices.end(); ++citer, ++group) {
        int begin = citer->first, end = citer->second, num = 1;
        std::cerr << "Group " << group << std::endl;
        assert(sorted_vec[begin].sample_type == C_STATE);
        std::cerr << sorted_vec[begin];
        for (int i = begin+1; i <= end; ++i) {
            if (true) {
                std::cerr << sorted_vec[i];
                ++num;
            }
        }
        if (num < 2) {
            fprintf(stderr, "SINGLE GROUP\n");
        }
        fprintf(stderr, "*****************************************\n");
    }
    fprintf(stderr, "---TPS-GROUPS-DUMP-END---\n");
};
/*
 * INTERNAL API:
 * Function to pretty print results. All output sent
 * to 'stdout'. Used in debugging only.
 */
void pwr::WuParser::dump_tps_groups_i(const sample_vec_t& sorted_vec, const c_group_vec_t& indices)
{
    fprintf(stderr, "---TPS-GROUPS-DUMP-BEGIN---\n");
    int group = 0;
    for (c_group_vec_t::const_iterator citer = indices.begin(); citer != indices.end(); ++citer, ++group) {
        int begin = citer->group.first, end = citer->group.second, num = 1;
        std::cerr << "Group " << group << std::endl;
        assert(sorted_vec[begin].sample_type == C_STATE);
        std::cerr << sorted_vec[begin];
        for (int i = begin+1; i <= end; ++i) {
            if (true) {
                std::cerr << sorted_vec[i];
                ++num;
            }
        }
        if (num < 2) {
            fprintf(stderr, "SINGLE GROUP\n");
        }
        fprintf(stderr, "*****************************************\n");
    }
    fprintf(stderr, "---TPS-GROUPS-DUMP-END---\n");
};

/*
 * INTERNAL API:
 * Function to check whether elements of a given
 * (logical) CPU are sorted (ascending order wrt TSC
 * values).
 *
 * @cpu: the logical processor in question.
 *
 * @returns: "true" ==> samples are sorted, "false" ==> not sorted.
 */
bool pwr::WuParser::check_sorted_i(int cpu)
{
    sample_list_t samples = m_per_cpu_sample_lists[cpu];
    int num_unsorted = 0;
    bool retVal = true;
    PWCollector_sample_t prev_sample;
    u64 prev_tsc = 0;
    int idx = 0;

    for (sample_list_t::const_iterator citer = samples.begin(); citer != samples.end(); ++citer, ++idx) {
        if (prev_tsc && prev_tsc > citer->tsc) {
            if (g_do_debugging) {
                fprintf(stderr, "[%d]: ERROR: idx = %d, prev_tsc = %llu (type = %s), curr_tsc = %llu (type = %s)\n", cpu, idx, (unsigned long long)prev_sample.tsc, s_long_sample_names[prev_sample.sample_type], (unsigned long long)citer->tsc, s_long_sample_names[citer->sample_type]);
                std::cerr << prev_sample;
                std::cerr << *citer;
            }
            ++num_unsorted;
            retVal = false;
        }
        prev_sample = *citer; prev_tsc = citer->tsc;
    }
    return retVal;
};

/*
 * INTERNAL API:
 * Function to sort elements of a given (logical)
 * CPU.
 *
 * @cpu: the logical processor in question.
 */
void pwr::WuParser::sort_data_i(int cpu)
{
    sample_list_t& c_samples = m_per_cpu_sample_lists[cpu];
    c_samples.sort();
};

std::vector<int> pwr::WuParser::get_all_threads_of_i(int coreid)
{
    const std::vector<int>& siblings = m_threadSiblingMap[coreid];
    std::vector<int> retVal;
    retVal.insert(retVal.begin(), siblings.begin(), siblings.end());

    return retVal;
};

inline int pwr::WuParser::get_core_given_lcpu_i(int cpu)
{
    return m_htMap[cpu];
};

/*
 * INTERNAL API:
 * Function to calculate the 'Cx' and 'C0' residency for a given 'TPS group'.
 *
 * @prev_end: the end sample of the PREVIOUS TPS group.
 * @curr_end: the end sample of the CURRENT TPS group.
 * @which: the specific C-state (i.e. C2, C3 etc.) entered between the end of the previous TPS group
 * and the start of the current one.
 * @cx_delta: the Cx residency.
 * @c0: the C0 residency.
 * @prev_cx_begin_tsc: TSC of the last time the core entered a C-state.
 * @prev_cx_end_tsc: TSC when the core woke up from the C-state entered at @prev_cx_begin_tsc
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::calc_inter_group_cx_c0_res_i(const PWCollector_sample_t& prev_end, const PWCollector_sample_t& curr_end, int& which, u64& cx_delta, u64& c0, const u64& prev_cx_begin_tsc, const u64& prev_cx_end_tsc) {
    assert(prev_end.sample_type == curr_end.sample_type == C_STATE); // sanity check
    const c_sample_t &cs1 = prev_end.c_sample, &cs2 = curr_end.c_sample;

    assert (prev_cx_end_tsc > prev_cx_begin_tsc); // sanity check

    which = -1;

    /*
     * Calculate 'Cx' first because we'll need it to
     * calculate 'C0'.
     */
    for (int i=C9; i>APERF; --i) {
        /*
         * We iterate in reverse because we need to ensure that, on MFLD,
         * we check 'C5' BEFORE we check 'C4' because if the core enters
         * 'C5' then both MSRS (i.e. C4, C5) will increment.
         */
        if ( (cx_delta = RES_COUNT(cs2, i) - RES_COUNT(cs1, i)) > 0) {
            which = i;
            break;
        }
    }
    if (which < 0) {
        if (g_do_debugging) {
            fprintf(stderr, "ERROR: no difference in samples?!\n");
            std::cerr << prev_end;
            std::cerr << curr_end;
        }
        return -ERROR;
    }
    cx_delta *= C_STATE_RES_COUNT_MULTIPLIER();
    /*
     * OK, Cx done. Now calculate C0
     * Formula:
     * C0 = (TSC" - TSC') - Cx
     * Where TSC" ==> 'prev_cx_end_tsc'
     *		 TSC' ==> 'prev_cx_begin_tsc'
     *		 Cx ==> Cx residency for CURRENT TPS group.
     * N.B.: we add the 'C0' component from the
     * current group in order to align with previous 'wudump'
     * semantics.
     */
    u64 tsc_delta = (prev_cx_end_tsc - prev_cx_begin_tsc);
    if (cx_delta < tsc_delta) {
        c0 = tsc_delta - cx_delta;
    } else {
        if (g_do_debugging) {
            int prev_cpu = prev_end.cpuidx, curr_cpu = curr_end.cpuidx;
            fprintf(stderr, "WARNING: NEGATIVE C0! %lld, which_cx = %d, prev cpu = %d curr cpu = %d\n", (long long int)c0, which, prev_cpu, curr_cpu);
            if (prev_cpu == curr_cpu) {
                fprintf(stderr, "SAME-CPU: %d\n", curr_cpu);
            }
            std::cerr << prev_end;
            std::cerr << curr_end;
        }
        cx_delta = tsc_delta;
        c0 = 0;
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Determine what caused the core to exit idle.
 * If "allow_sched_wakeups" is FALSE then do NOT consider
 * "SCHED_WAKEUP" samples as possible causes of wakeups (set to FALSE
 * in a SINGLE-CORE environment).
 *
 * @samples: the sorted list of driver samples.
 * @from: the end of the previous TPS group.
 * @to: the end of the current TPS group.
 * @allow_sched_wakeups: "true" ==> consider "SCHED_WAKEUP" to be a possible
 * wakeup cause; "false" ==> do NOT allow "SCHED_WAKEUP" causes.
 * @wakeup_sample: the calculated wakeup cause.
 *
 * @returns: 0 on success, -1 on error
 */
int pwr::WuParser::calc_wakeup_cause_i(const sample_vec_t& samples, int from, int to, bool allow_sched_wakeups, PWCollector_sample_t *wakeup_sample) {
    /*
     * Basic algo:
     * check for first non-TPS/TPE sample in range (from,to).
     */
    for (int i=from+1; i<to; ++i) {
        const PWCollector_sample_t& sample = samples[i];
        if (sample.sample_type != C_STATE) {
            if (sample.sample_type == SCHED_SAMPLE) {
                int src_cpu = sample.e_sample.data[0], target_cpu = sample.e_sample.data[1];
                if (!allow_sched_wakeups && (get_core_given_lcpu_i(src_cpu) == get_core_given_lcpu_i(target_cpu))) {
                    db_fprintf(stderr, "WARNING: SELF-SCHED found [%d, %d]\n", from, to);
                    continue;
                }
            }
            *wakeup_sample = sample;
            return SUCCESS;
        }
    }
    return -ERROR;
};

/*
 * INTERNAL API:
 * Calculate cx mwait hint parameter i.e. the C-state the OS requested from the hardware.
 * Used in determining whether the hardware promoted/demoted the OS request.
 *
 * @samples: the sorted list of driver PWCollector samples.
 * @from: the start of the PREVIOUS TPS group.
 * @to: the end of the PREVIOUS TPS group.
 * @requested_cx: the (calculated) C-state requested by the OS.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::calc_cx_mwait_hint_i(const sample_vec_t& samples, int from, int to, int& requested_cx) {
    int hints[] = {-1, -1};
    int prev_cpu = -1;
    int i=0, hint_index = -1, num_hints = 0;

    for (i=to; i>= from && hint_index < 1; --i) {
        const PWCollector_sample_t& sample = samples[i];
        int cpu = sample.cpuidx;
        if (sample.sample_type != C_STATE) {
            continue;
        }
        if (prev_cpu == -1) {
            prev_cpu = cpu;
            hints[++hint_index] = sample.c_sample.prev_state;
        } else if (cpu != prev_cpu) {
            hints[++hint_index] = sample.c_sample.prev_state;
        }
    }

    if (i < from) {
        return -ERROR;
    }
    /*
     * The actual requested C-state will be the minimum of 
     * the two the OS requested.
     */
    requested_cx = std::min(hints[0], hints[1]);
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Compute APERF and MPERF deltas over a given TPS group. Used to determine P-state transitions.
 * We use the ratio of APERF-delta/MPERF-delta to determine actual core operating frequencies
 * (more precisely, we use the CPU_CLK_UNHALTED.CORE to CPU_CLK_UNHALTED.REF ratio to determine
 * the operating frequency -- see Intel White Paper dated November 2008: "Intel Turbo Boost Technology
 * in Intel Core Microarchitectures (Nehalem) Based Processors" for more information)
 *
 * @samples: the sorted list of PWCollector samples returned by the power driver.
 * @prev_end: end of the PREVIOUS TPS group.
 * @curr_end: end of the CURRENT TPS group.
 * @aperf_mperf_map: free-running, per-logical-processor list of aperf & mperf ratios.
 * @tmp_p_samples: (generated) temporary list of P-state transition samples.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::calc_aperf_mperf_deltas_i(const sample_vec_t& samples, int prev_end, int curr_end, aperf_mperf_map_t& aperf_mperf_map, sample_list_t& tmp_p_samples)
{
    u64 curr_aperf = 0, curr_mperf = 0;
    u64 delta_aperf = 0, delta_mperf = 0;
    int core = -1;
    bool is_ht = WAS_SYSTEM_HYPER_THREADED();
    u32 prev_bound_freq = 0;

    for (int i=prev_end+1; i<= curr_end; ++i) {
        const PWCollector_sample_t& sample = samples[i];
        int cpu = sample.cpuidx;
        int sample_type = sample.sample_type;
        u16 is_boundary = 0;
        bool do_produce = true;
        u32 freq = 0;

        /*
         * We read the CPU_CLK_UNHALTED.{CORE, REF} counters
         * on every TPS and every TPF probe. Check for those
         * samples here.
         */
        if (sample_type != C_STATE && sample_type != P_STATE) {
            continue;
        }

        if (core == -1) {
            core = get_core_given_lcpu_i(cpu);
        }
        /*
         * Deltas require a baseline. We keep a (free-running) list
         * of aperf, mperf values for this purpose.
         */
        u64& prev_aperf = aperf_mperf_map[cpu].first;
        u64& prev_mperf = aperf_mperf_map[cpu].second;

        if (sample_type == C_STATE) {
            curr_aperf = RES_COUNT(sample.c_sample, APERF);
            curr_mperf = RES_COUNT(sample.c_sample, MPERF);
        } else {
            curr_aperf = sample.p_sample.unhalted_core_value;
            curr_mperf = sample.p_sample.unhalted_ref_value;
            /*
             * Only TPF samples can be boundary samples.
             */
            is_boundary = sample.p_sample.is_boundary_sample;
        }
        if (is_boundary) {
            /*
             * For boundary samples, we rely on the frequency reported by
             * the OS (because we don't have a baseline 'APERF' or 'MPERF'
             * value to construct a delta).
             */
            freq = sample.p_sample.frequency;
            if (is_ht) {
                /*
                 * Core operates at MAX of the frequencies reported
                 * by its logical processors.
                 */
                if (prev_bound_freq > 0) {
                    freq = std::max(prev_bound_freq, freq);
                    prev_bound_freq = 0;
                } else {
                    prev_bound_freq = freq;
                    /*
                     * Don't create a P-state transition sample until we see the boundary
                     * sample from the other logical processor.
                     */
                    do_produce = false;
                }
            }
        } else {
            assert(prev_aperf && prev_mperf); /* Sanity! */
            delta_aperf = curr_aperf - prev_aperf; delta_mperf = curr_mperf - prev_mperf;
            double ratio = (double)delta_aperf / (double)delta_mperf;
            if (calc_actual_frequency_i(ratio, freq)) {
                db_fprintf(stderr, "ERROR calculating actual frequency!\n");
                do_produce = false;
            }
        }
        if (do_produce) {
            tmp_p_samples.push_back(create_tpf_collector_sample_i(is_boundary, core, freq, sample.tsc, delta_aperf, delta_mperf));
        }
        /*
         * OK, computed the deltas. Now update the (free-running) list
         * of baseline APERF, MPERF values.
         * N.B.: 'prev_aperf' and 'prev_mperf' are references, so this
         * *should* update the map!
         */
        prev_aperf = curr_aperf; prev_mperf = curr_mperf;
    }
    
    return SUCCESS;
};

struct PStatePredLess {
    const u64& m_collection_start_tsc;
    PStatePredLess(const u64& t) : m_collection_start_tsc(t) {};
    bool operator()(const PWCollector_sample_t& sample) {
        return sample.tsc < m_collection_start_tsc;
    }
};

struct PStatePredMore {
    const u64& m_collection_stop_tsc;
    PStatePredMore(const u64& t) : m_collection_stop_tsc(t) {};
    bool operator()(const PWCollector_sample_t& sample) {
        return sample.tsc > m_collection_stop_tsc;
    }
};

/*
 * INTERNAL API:
 * Calculate operating frequencies given a list of APERF/MPERF ratios.
 *
 * @tmp_p_samples: the previously constructed list of temporary p-state transition samples.
 * This list is constructed in the 'calc_aperf_mperf_deltas_i()' function.
 * @collection_start_tsc: the Start of the collection.
 * @collection_stop_tsc: the Stop of the collection.
 *
 * @returns: 0 on success, -1 on error
 */
int pwr::WuParser::calc_actual_frequencies_i(sample_list_t& tmp_p_samples, const u64& collection_start_tsc, const u64& collection_stop_tsc)
{
    if (tmp_p_samples.empty()) {
        db_fprintf(stderr, "Warning: empty list of tmp p samples!\n");
        return SUCCESS;
    }
    sample_list_t::iterator curr = tmp_p_samples.begin(), next = curr;
    u32 collection_start_freq = 0, collection_stop_freq = 0;
    u32 beg_freq = tmp_p_samples.front().p_sample.frequency, end_freq = tmp_p_samples.back().p_sample.frequency;

    int core_id = GET_CORE_GIVEN_LCPU(curr->cpuidx);

    /*
     * Adjust the 'cpuidx' field to reflect the actual core and not the logical cpu.
     */
    curr->cpuidx = core_id;
    /*
     * Make sure the frequency returned by the driver is in our
     * list of available frequencies. This is ONLY for turbo frequencies!
     */
    if (curr->p_sample.frequency > m_availableFrequenciesKHz[0]) {
        curr->p_sample.frequency = m_availableFrequenciesKHz[0];
    }
    
    for (++next; next != tmp_p_samples.end(); curr = next, ++next) {
        if (collection_start_freq == 0 && curr->tsc >= collection_start_tsc) {
            collection_start_freq = curr->p_sample.frequency;
        }
        if (collection_stop_freq == 0 && curr->tsc >= collection_stop_tsc) {
            collection_stop_freq = curr->p_sample.frequency;
        }
        /*
         * Adjust the 'cpuidx' field to reflect the actual core and not the logical cpu.
         */
        next->cpuidx = core_id;
        /*
         * Make sure the frequency returned by the driver is in our
         * list of available frequencies. This is ONLY for turbo frequencies!
         */
        if (next->p_sample.frequency > m_availableFrequenciesKHz[0]) {
            next->p_sample.frequency = m_availableFrequenciesKHz[0];
        }
        if (next->p_sample.frequency != curr->p_sample.frequency) {
            /* Retain 'curr' */
        } else {
            /* Remove 'curr' */
            tmp_p_samples.erase(curr);
        }
    }

    if (collection_start_freq == 0) {
        /*
         * Can only happen if the collection_start_tsc was greater than all
         * the P-state TSCs. In this case we ASSUME the frequency doesn't
         * change between the last P-state sample and the collection start.
         */
        collection_start_freq = end_freq;
    }
    if (collection_stop_freq == 0) {
        /*
         * Can only happen if the collection_stop_tsc was greater than all
         * the P-state TSCs. In this case we ASSUME the frequency doesn't
         * change between the last P-state sample and the collection stop.
         */
        collection_stop_freq = end_freq;
    }
    /*
     * OK, we've calculated the p-state transitions. Now elminiate all those
     * that don't fall between our previously calculated collection limits
     */
    tmp_p_samples.remove_if(PStatePredLess(collection_start_tsc));
    tmp_p_samples.remove_if(PStatePredMore(collection_stop_tsc));
    /*
     * OK, done pruning. Final step: add the two boundary samples for collection
     * START and STOP, respectively. Note that we might already have a sample with
     * TSC == collection_{start,stop}_tsc, in which case we merely mark it as a
     * boundary sample. On the other hand, if a boundary sample exists, but it's
     * TSC doesn't satisfy our {min,max} criteria, then we simply modify this boundary
     * so that the TSC's match.
     */
    if (tmp_p_samples.empty()) {
        tmp_p_samples.push_front(create_tpf_collector_sample_i(true, core_id, collection_start_freq, collection_start_tsc, 0 /* delta_aperf */, 0 /* delta_mperf */));
    } else if (tmp_p_samples.front().tsc > collection_start_tsc) {
        if (tmp_p_samples.front().p_sample.is_boundary_sample == 1) {
            tmp_p_samples.front().tsc = collection_start_tsc;
        } else {
            tmp_p_samples.push_front(create_tpf_collector_sample_i(true, core_id, collection_start_freq, collection_start_tsc, 0 /* delta_aperf */, 0 /* delta_mperf */));
        }
    } else {
        assert(tmp_p_samples.front().tsc == collection_start_tsc); // not strictly necessary!
        tmp_p_samples.front().p_sample.is_boundary_sample = 1;
    }
    assert(tmp_p_samples.empty() == false);
    if (tmp_p_samples.back().tsc < collection_stop_tsc) {
        if (tmp_p_samples.back().p_sample.is_boundary_sample < 2) {
            tmp_p_samples.push_back(create_tpf_collector_sample_i(true, core_id, collection_stop_freq, collection_stop_tsc, 0 /* delta_aperf */, 0 /* delta_mperf */));
        } else {
            tmp_p_samples.back().tsc = collection_stop_tsc;
        }
    } else {
        assert(tmp_p_samples.back().tsc == collection_stop_tsc); // not strictly necessary!
        tmp_p_samples.back().p_sample.is_boundary_sample = 2;
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Calculate the actual operating frequency, given an aperf/mperf ratio.
 * @aperf_mperf_ratio: the (previously calculated) APERF/MPERF ratio.
 * @actual_freq: the (calculated) actual operating frequency corresponding to @aperf_mperf_ratio
 *
 * @returns: 0 on success, -1 on error
 */
int pwr::WuParser::calc_actual_frequency_i(const double& aperf_mperf_ratio, u32& actual_freq)
{
    int i=0;
    u32 tsc_freq = sysInfo.m_tscFreq * 1000; // We need TSC freq in KHz, not MHz
    int num_freqs = m_availableFrequenciesKHz.size();

    if (tsc_freq == 0) {
        db_fprintf(stderr, "ERROR: no tsc frequency found!\n");
        return -ERROR;
    }

    u32 calc_freq = (u32)(aperf_mperf_ratio * tsc_freq);

    /*
     * "calc_freq" is an AVERAGE value. We need to return a frequency
     * the user expects to see (i.e. one of the frequency "steps").
     * Compute this frequency here.
     */
    if (num_freqs == 0) {
        db_fprintf(stderr, "NO list of available frequencies?!\n");
        return -ERROR;
    }
    if (calc_freq >= m_availableFrequenciesKHz[0]) {
        /*
         * Indicates TURBO.
         */
        db_fprintf(stderr, "WARNING: TURBO detected!\n");
        actual_freq = m_availableFrequenciesKHz[0];
        return SUCCESS;
    }
    /*
     * Try and find the frequency STEP that's
     * closest to 'calc_freq'
     */
    for (i=1; i<num_freqs; ++i) {
        u32 upper = m_availableFrequenciesKHz[i-1], lower = m_availableFrequenciesKHz[i];
        if(IS_BRACKETED_BY(upper, lower, calc_freq)){
            actual_freq = GET_CORRECT_FREQ_BOUND(upper, lower, calc_freq);
            db_fprintf(stderr, "AVG = %lu, UPPER = %u, LOWER = %u, CLOSEST = %u\n", calc_freq, upper, lower, actual_freq);
            return SUCCESS;
        }
    }

    /*
     * OK, couldn't find a suitable freq -- just
     * return the 'calc_freq'.
     * UPDATE: reaching here implies a frequency
     * LOWER than the LOWEST frequency step. In this
     * case, return this LOWEST frequency.
     */
    actual_freq = m_availableFrequenciesKHz[--i];
    return SUCCESS;
};
/*
 * EXTERNAL API:
 * Set the input directory and file names.
 *
 * @dir: the directory name.
 * @file: the actual file name.
 */
void pwr::WuParser::set_wuwatch_output_file_name(const std::string& dir, const std::string& file)
{
    assert(dir.size() && file.size());
    m_wuwatch_output_dir = dir;
    m_combined_input_file_name = dir + file;
    db_fprintf(stderr, "WUDUMP has input file = %s\n", m_combined_input_file_name.c_str());
};

/*
 * EXTERNAL API:
 * Main function:
 * (1) Read samples.
 * (2) Parse them, exctracting relevent information.
 * (3) Print out results.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_work(void)
{
    if (do_read_i()) {
        fprintf(stderr, "ERROR reading input data!\n");
        return -ERROR;
    }

    if (do_parse_i()) {
        fprintf(stderr, "ERROR parsing data!\n");
        return -ERROR;
    }

    if (do_sort_and_collate_i()) {
        fprintf(stderr, "ERROR soring and collating samples!\n");
        return -ERROR;
    }

    return SUCCESS;
};

/*
 * PRIVATE API
 * Default constructor -- privatized for Singleton-compatibility.
 */
pwr::WuData::WuData() : m_init_complete(false) {};
/*
 * PRIVATE API
 * Default destructor -- privatized for Singleton-compatibility.
 */
pwr::WuData::~WuData() {
    /*
     * Delete any dynamically allocted (userspace) backtrace memory.
     */
    for (trace_vec_t::iterator iter = m_trace_vec.begin(); iter != m_trace_vec.end(); ++iter) {
        delete *iter;
    }
};

/*
 * PUBLIC API
 * Return a (globally persistent) WuData instance.
 */
pwr::WuData *pwr::WuData::instance()
{
    if (!s_data) {
        s_data = new WuData;
    }
    return s_data;
};

/*
 * PUBLIC API
 * Destroy the (globally persistent) WuData instance.
 */
void pwr::WuData::destroy()
{
    delete s_data;
    s_data = NULL;
};


/*
 * PUBLIC API
 * Entry point to reading and parsing process.
 *
 * @dir_name: Directory in which to find wuwatch output file.
 * @file_name: The name of the wuwatch output file.
 *
 * @returns:
 *          0 ==>   OK
 *          -1 ==>  Failure.
 */
int pwr::WuData::do_read_and_process(const std::string& dir_name, const std::string& file_name, bool should_calc_c1, bool should_dump_orig_samples)
{
    WuParser *wuparserObj = new WuParser(sysInfo, should_calc_c1, should_dump_orig_samples);
    wuparserObj->set_wuwatch_output_file_name(dir_name, file_name);
    /*
     * 'WuParser' accesses macros that require 'm_init_complete' to
     * be set.
     */
    m_init_complete = true;

    if (wuparserObj->do_work()) {
        /*
         * Something went wrong. Free up memory, reset the
         * 'init' flag and die.
         */
        m_init_complete = false;
        delete wuparserObj;
        return -ERROR;
    }
    /*
     * OK, copy samples and trace maps etc. over.
     */
    {
        m_samples = sample_vec_t(wuparserObj->m_all_samples_list.size());
        std::copy(wuparserObj->m_all_samples_list.begin(), wuparserObj->m_all_samples_list.end(), m_samples.begin());
        m_trace_pair_map = wuparserObj->m_trace_pair_map;
        m_trace_vec = wuparserObj->m_trace_vec;
    }

    delete wuparserObj;
    return SUCCESS;
};

/*
 * PUBLIC API
 * Entry point to reading and parsing process.
 *
 * @file_name: (Relative or Absolute) Path to the wuwatch output file.
 *
 * @returns:
 *          0 ==>   OK
 *          -1 ==>  Failure.
 */
int pwr::WuData::do_read_and_process(const std::string& wuwatch_file_path, bool should_calc_c1)
{
    std::string dir_name = "", file_name = "";
    extract_dir_and_file_from_path(wuwatch_file_path, dir_name, file_name);
    if (file_name == "" || file_name == ".") {
        file_name = std::string(DEFAULT_WUWATCH_OUTPUT_FILE_NAME) + ".ww1";
    }
    return do_read_and_process(dir_name, file_name, should_calc_c1);
};
/*
 * PUBLIC API
 * Retrieve system configuration information.
 *
 * @returns: the system configuration information.
 */
const pwr::SystemInfo& pwr::WuData::getSystemInfo() const
{
    assert(m_init_complete);
    return sysInfo;
};

/*
 * PUBLIC API
 * Retrieve list of PWCollector samples returned by the driver.
 *
 * @returns: the list of (parsed) PWCollector samples
 */
const sample_vec_t& pwr::WuData::getSamples() const
{
    assert(m_init_complete);
    return m_samples;
};

/*
 * PUBLIC API
 * Retrieve TID <--> Call trace mappings returned by the driver
 * and also mappings generated by the hook library.
 *
 * @returns: the call trace mapping.
 */
const trace_pair_map_t& pwr::WuData::get_tid_backtrace_map() const
{
    assert(m_init_complete);
    return m_trace_pair_map;
};

/*
 * EXTERNAL API:
 * Helper function to pretty-print an individual
 * power sample.
 */
void operator<<(std::ostream& os, const PWCollector_sample_t& sample)
{
    const c_sample_t *cs;
    const r_sample_t *rs;
    const i_sample_t *is;
    int cpu;
    unsigned long long tsc, mperf, c1, c2, c3, c4, c5, c6, c9;

    cpu = sample.cpuidx;
    tsc = sample.tsc;

    os << cpu << "\t" << sample.tsc << "\t" << s_long_sample_names[sample.sample_type];

    if (sample.sample_type == C_STATE) {
        cs = &sample.c_sample;
        mperf = RES_COUNT(*cs, MPERF);
        c1 = RES_COUNT(*cs, APERF);
        c2 = RES_COUNT(*cs, C2);
        c3 = RES_COUNT(*cs, C3);
        c4 = RES_COUNT(*cs, C4);
        c5 = RES_COUNT(*cs, C5);
        c6 = RES_COUNT(*cs, C6);

        c9 = RES_COUNT(*cs, C9);

        os << "\t" << mperf << "\t" << c1;

        switch (pwr::WuData::instance()->getSystemInfo().m_arch) {
            case NHM:
            case SNB:
                os << "\t" << c3 << "\t" << c6;
                break;
            case MFD:
                os << "\t" << c2 << "\t" << c4 << "\t" << c5 << "\t" << c6 << "\t" << c9;
                break;
        }
        os << "\t" << sample.c_sample.prev_state << "\t" << sample.c_sample.tps_epoch;
    }
    else if (sample.sample_type == P_STATE) {
        // os << "\t" << sample.p_sample.unhalted_ref_value << "\t" << sample.p_sample.unhalted_core_value << "\t" << sample.p_sample.frequency << "\t" << GET_BOOL_STRING(sample.p_sample.is_boundary_sample);
        os << "\t" << sample.p_sample.prev_req_frequency << "\t" << sample.p_sample.frequency << "\t" << GET_BOOL_STRING(sample.p_sample.is_boundary_sample);
        // os << "\t" << sample.p_sample.frequency << "\t" << GET_BOOL_STRING(sample.p_sample.is_boundary_sample);
    }
    else if (sample.sample_type == IRQ_MAP) {
        is = &sample.i_sample;
        os << "\t" << is->irq_num << "\t" << is->irq_name;
    }
    else if (sample.sample_type == PROC_MAP) {
        rs = &sample.r_sample;
        os << "\t" << rs->type << "\t" << rs->tid << "\t" << rs->pid << "\t" << rs->proc_name;
    }
    else if (sample.sample_type == IRQ_SAMPLE) {
        os << "\t" << sample.e_sample.data[0];
    }
    else if (sample.sample_type == TIMER_SAMPLE) {
        os << "\t" << sample.e_sample.data[0] << "\t" << sample.e_sample.data[1];
    }
    else if (sample.sample_type == SCHED_SAMPLE) {
        os << "\t" << sample.e_sample.data[0] << "\t" << sample.e_sample.data[1] << "\t" << sample.e_sample.data[2];
    }
    else if (sample.sample_type == IPI_SAMPLE) {
        os << "\t" << sample.e_sample.data[0];
    }
    os << std::endl;
};

/*
 * EXTERNAL API:
 * Helper function to pretty-print a pair.
 */
void operator<<(std::ostream& os, const int_pair_t& pair)
{
    os << pair.first << "\t" << pair.second << std::endl;
};
/*
 * EXTERNAL API:
 * Helper function to compare two c-state samples.
 *
 * @s1, @s2: the two samples to compare.
 *
 * @returns: "true" ==> the two samples are 'EQUAL', "false" otherwise.
 * Equality is defined by comparing the MSR-sets of the two samples.
 */
bool operator==(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2)
{
    const c_sample_t& cs1 = s1.c_sample, cs2 = s2.c_sample;
    /*
     * Unlike the case when we need to know WHICH C-state MSR counter, 
     * iteration order doesn't matter in this case: we only
     * need to know if ANY C-state MSR counted.
     */
    for (int i=C2; i<MAX_MSR_ADDRESSES; ++i) {
        if (RES_COUNT(cs1, i) != RES_COUNT(cs2, i)) {
            return false;
        }
    }
    return true;
};
/*
 * EXTERNAL API:
 * Helper function to compare two c-state samples.
 */
bool operator!=(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2)
{
    return !(s1 == s2);
};

/*
 * EXTERNAL API:
 * Helper function used to sort (in ascending TSC order) the various PWCollector
 * samples returned by the driver.
 *
 * @s1, @s2: the two samples to compare.
 *
 * @returns: "true" ==> s1.tsc < s2.tsc, "false" otherwise.
 */
bool operator<(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2)
{
    return s1.tsc < s2.tsc;
};


