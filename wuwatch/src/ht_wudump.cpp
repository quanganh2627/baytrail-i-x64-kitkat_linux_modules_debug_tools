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

/* **************************************
 * The wuwatch data parser.
 * **************************************
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
#include "ht_wudump_defines.h"
#include "ht_wudump.h"
#include "wulib.h"

/* *******************************************************************
 * Internal data structures. These are declared here instead of in
 * 'ht_wudump.h' because we don't want them to be visible externally.
 * *******************************************************************
 */
/*
 * Data structure to encapsuate S state residency counters
 */
struct s_residency_data {
    uint64_t tsc;
    int cpu;
    uint32_t S0i0;
    uint32_t S0i1;
    uint32_t S0i2;
    uint32_t S0i3;
    s_residency_data(uint64_t&, int&, uint32_t&, uint32_t&, uint32_t&, uint32_t&);
};

struct s_residency_sorter {
    bool operator()(const s_residency_data_t& p1, const s_residency_data_t& p2);
};

/*
 * Data structure to encapsuate D states in North complex
 */
struct d_nc_state_data {
    uint64_t tsc;
    uint32_t states;
    d_nc_state_data(uint64_t&, uint32_t&);
};

struct d_nc_state_sorter {
    bool operator()(const d_nc_state_data_t& p1, const d_nc_state_data_t& p2);
};


/*
 * Data structure to encapsuate D states in Sorth complex
 */
struct d_sc_state_data {
    uint64_t tsc;
    uint32_t states[4];
    d_sc_state_data(uint64_t&, uint32_t*);
};

struct d_sc_state_sorter {
    bool operator()(const d_sc_state_data_t& p1, const d_sc_state_data_t& p2);
};


/*
 * Data structure to encapsuate D state residency counters in Sorth complex
 */
struct d_sc_residency_data {
    uint64_t tsc;
    uint32_t id;   // LSS number
    uint32_t D0i0;
    uint32_t D0i0_ACG;
    uint32_t D0i1;
    uint32_t D0i3;
    d_sc_residency_data(uint64_t&, uint32_t&, uint32_t&, uint32_t&, uint32_t&, uint32_t&);
};

struct d_sc_residency_sorter {
    bool operator()(const d_sc_residency_data_t& p1, const d_sc_residency_data_t& p2);
};

/*
 * Data structure to encapsuate wakelock samples
 */
struct wakelock_data {
    uint64_t tsc;
    uint32_t type;
    pid_t pid;
    pid_t tid;
    std::string name;
    std::string proc_name;
    wakelock_data(uint64_t&, uint32_t&, pid_t&, pid_t&, char*, char*);
};

struct wakelock_sorter {
    bool operator()(const wakelock_data_t& p1, const wakelock_data_t& p2);
};

/*
 * Data structure to encapsulate proc_map samples.
 */
struct proc_struct {
    pid_t tid, pid;
    std::string name;
    u64 init_time, exit_time;
};


/*
 * Helper struct used ONLY to dump out
 * C-state samples.
 */
struct CDumper {
    FILE *output_fp;
    bool m_do_dump_backtraces, m_do_dump_tscs, m_do_raw_output;
    const trace_pair_map_t& m_trace_pair_map;
    const r_sample_map_t& m_r_sample_map;
    const i_sample_map_t& m_i_sample_map;

    CDumper(FILE *fp, const bool& b, const bool& t, const bool& r, const trace_pair_map_t& t_map, const r_sample_map_t& r_map, const i_sample_map_t& i_map);

    const trace_t *bracket_tsc_pair_i(const uint64_t& tsc, const trace_pair_list_t& trace_list) const;
    const trace_t *get_backtrace_given_tsc_tid_i(const u64& tsc, pid_t tid) const;

    void operator()(const PWCollector_sample_t&);
};

/*
 * Helper struct used ONLY to dump out
 * P-state samples.
 */
struct PDumper {
    FILE *output_fp;
    PDumper(FILE *f);
    void operator()(const PWCollector_sample_t&);
};


/* **************************************
 * Variable definitions.
 * **************************************
 */
/*
 * (Short) names of the various wakeup types.
 */
const char *HTWudump::break_type_names_deprecated[] = {"I", "T", "S", "IPI", "BEGIN", "?"};
const char *HTWudump::break_type_names[] = {"I", "T", "S", "IPI", "W", "B", "?"};
/*
 * Short and long versions of names of the various 'sample_type_t'
 * enum values.
 */
const char *HTWudump::s_long_sample_names[] = {"FREE_SAMPLE", "TPS", "TPF", "K_CALL", "M_MAP", "I_MAP", "P_MAP", "S_RES", "S_ST", "D_RES", "D_ST", "TIM", "IRQ", "WRQ", "SCD", "IPI", "TPE", "NULL", "SAMPLE_END"};
const char *HTWudump::s_short_sample_names[] = {"FREE_SAMPLE", "C", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "T", "I", "W", "S", "IPI", "TPE", "NULL", "?"};
/*
 * North and South complex device names.
 */
const char *mfd_nc_device_names[][2] = { 
                                {"GPS", "GFX subsystem"},
                                {"VDPS", "Video Decode subsystem"},
                                {"VEPS", "Video Encode subsystem"},
                                {"DPA", "Display Island A"},
                                {"DPB", "Display Island B"},
                                {"DPC", "Display Island C"},
                                {"GL3", "GL3 Power Island"}, 
                                {"ISP", "ISP Power Island"},
                                {"IPH", "IPH Power Island"},
                                {"MIO", "MIO Power Island"} };


const char *mfd_sc_device_names[][2] = { 
                                {"LSS00", "SDIO0 (HC2)"},
                                {"LSS01", "eMMC0 (HC0a)"},
                                {"LSS02", "NAND controller"},
                                {"LSS03", "MIPI- HSI subsystem"},
                                {"LSS04", "Security subsystem"},
                                {"LSS05", "eMMC1 (HC0b)"},
                                {"LSS06", "USB OTG"},
                                {"LSS07", "USB Host"},
                                {"LSS08", "Audio Engine, Audio Ram0"},
                                {"LSS10", "SRAM controller, SRAM Bank0"},
                                {"LSS12", "SRAM Bank1"},
                                {"LSS13", "SRAM Bank2"},
                                {"LSS14", "SDIO2 (HC1b)"},
                                {"LSS15", "DFX Subsystem"},
                                {"LSS16", "SCU DMA"},
                                {"LSS17", "SPI0/MSIC"},
                                {"LSS18", "SPI1"},
                                {"LSS19", "SPI2"},
                                {"LSS20", "I2C0"},
                                {"LSS21", "I2C1"},
                                {"LSS24", "SC Fabric"},
                                {"LSS25", "Audio Ram1"},
                                {"LSS27", "I2C2"},
                                {"LSS29", "Chabbi AON Registers"},
                                {"LSS30", "SDIO1 (HC1a)"},
                                {"LSS33", "I2C3 (HDMI)"},
                                {"LSS34", "I2C4"},
                                {"LSS35", "I2C5"},
                                {"LSS36", "SSP (SPI3)"},
                                {"LSS37", "GPIO1"},
                                {"LSS39", "GPIO0"},
                                {"LSS40", "KBD"},
                                {"LSS41", "UART2:0"},
                                {"LSS44", "Security TPAC"},
                                {"LSS48", "SSP2"},
                                {"LSS49", "SLIMBUS CTL 1"},
                                {"LSS50", "SLIMBUS CTL 2"},
                                {"LSS51", "SSP0"},
                                {"LSS52", "SSP1"},
                                {"LSS54", "DMA Controller"} };



/* **************************************
 * Function definitions.
 * **************************************
 */

/*
 * The wudump constructor.
 */
HTWudump::HTWudump() : PW_max_num_cpus(-1), m_per_cpu_c_state_samples(NULL), m_per_cpu_p_state_samples(NULL),
    m_do_raw_output(true), m_do_dump_tscs(true), m_do_dump_backtraces(false), m_do_check_c1_res(true),
    m_do_dump_orig_samples(false), m_do_check_c_state_demotions(false),
    /*m_do_delete_driver_files(false),*/ m_was_any_tps_sample_present(false), m_was_any_tpf_sample_present(false),
    m_wuwatch_file_name(""), m_output_file_name("") {};

/*
 * The wudump destructor.
 */
HTWudump::~HTWudump() {
    /*
     * Delete wuwatch o/p file (if requested).
     * UPDATE: disabled for now.
     */
#if 0
    if (m_do_delete_driver_files) {
        std::string combined_input_file_name = m_wuwatch_output_dir + m_wuwatch_file_name;
        if (remove(combined_input_file_name.c_str())) {
            db_fprintf(stderr, "Error while removing file %s\n", combined_input_file_name.c_str());
        }
    }
#endif

    delete []m_per_cpu_c_state_samples;
    delete []m_per_cpu_p_state_samples;
};

/*
 * Instance functions for helper
 * structs declared earlier.
 */

CDumper::CDumper(FILE *fp, const bool& b, const bool& t, const bool& r, const trace_pair_map_t& t_map, const r_sample_map_t& r_map, const i_sample_map_t& i_map) :  output_fp(fp), m_do_dump_backtraces(b), m_do_dump_tscs(t), m_do_raw_output(r), m_trace_pair_map(t_map), m_r_sample_map(r_map), m_i_sample_map(i_map) {};

/*
 * Functor used to print out wakeup information.
 */
void CDumper::operator()(const PWCollector_sample_t& sample)
{
    assert(sample.sample_type == C_STATE);

    std::string payload;
    pid_t tid = 0, pid = 0;
    int irq_num = -1;
    int cpu = sample.cpuidx;
    u64 tsc = sample.tsc;
    static bool is_first = true;
    char cx_str[10];
    char req_str[10];
    const c_sample_t *cs = &sample.c_sample;
    int break_type = cs->break_type;
    int which_cx = GET_CX_FROM_COMBINED(cs->tps_epoch), which_cpuidx = GET_CPUID_FROM_COMBINED(cs->tps_epoch);
    u64 c0_res = RES_COUNT(*cs, MPERF), cx_res = RES_COUNT(*cs, which_cx);
    const char *c0_str = "0";
    std::string str_res_suffix = "";
    const char *res_suffix = NULL;

    if (m_do_raw_output == false) {
        cx_res /= 1000; c0_res /= 1000;
        str_res_suffix = "K";
    } else {
        str_res_suffix = "";
    }

    res_suffix = str_res_suffix.c_str();

    if (is_first) {
        sprintf(req_str, "%s", DONT_CARE);
        is_first = false;
    } else {
        /*
         * We've already mapped the requested mwait using
         * the 'm_tps_state_to_c_state' mapping. No
         * need to do so again.
         */
        sprintf(req_str, "%d", cs->prev_state);
    }

    sprintf(cx_str, "%d", which_cx);

    if (m_do_dump_tscs) {
        // fprintf(output_fp, "%20llu\t%8d\t%7s\t%15s\t%20llu K", tsc, cpu, cx_str, req_str, cx_res / 1000); // print Cx
        fprintf(output_fp, "%20llu\t%8d\t%7s\t%15s\t%20llu %s", tsc, cpu, cx_str, req_str, cx_res, res_suffix); // print Cx, div by 1000 handled above
    } else {
        // fprintf(output_fp, "%8d\t%7s\t%15s\t%20llu K", cpu, cx_str, req_str, cx_res / 1000); // print Cx
        fprintf(output_fp, "%8d\t%7s\t%15s\t%20llu %s", cpu, cx_str, req_str, cx_res, res_suffix); // print Cx, div by 1000 handled above
    }
    fprintf(output_fp, "\t%10s\t%5d", HTWudump::break_type_names[break_type], which_cpuidx);

    if (break_type == PW_BREAK_TYPE_T) { /* Timer wakeup */
        pid = cs->pid;
        tid = cs->tid;
        r_sample_map_t::const_iterator citer = m_r_sample_map.find(tid);
        if (citer != m_r_sample_map.end()) {
            payload = citer->second.second;
        } else {
            payload = UNKNOWN_PROCESS_NAME;
        }
        fprintf(output_fp, "\t%9d\t%6d\t%9s\t %-20s", tid, pid, DONT_CARE, payload.c_str());
        if (m_do_dump_backtraces) {
            /*
             * Also need to extract a back trace here!
             */
            u64 timer_init_tsc = cs->c_data;
            const trace_t *bt = get_backtrace_given_tsc_tid_i(timer_init_tsc, tid);
            if (bt) {
                /*
                 * We have backtrace information -- print it out.
                 */
                fprintf(output_fp, "\n\n");
                for (int i=0; i<bt->num_trace; ++i) {
                    fprintf(output_fp, "%s\n", bt->bt_symbols[i]);
                }
            }
        }
    }
    else if (break_type == PW_BREAK_TYPE_I) { /* IRQ wakeup */
        irq_num = cs->c_data;
        i_sample_map_t::const_iterator citer = m_i_sample_map.find(irq_num);
        if (citer != m_i_sample_map.end()) {
            payload = citer->second;
        } else {
            payload = "???";
        }
        fprintf(output_fp, "\t%9s\t%6s\t%9d\t %-20s", DONT_CARE, DONT_CARE, irq_num, payload.c_str());
    }
    else if (break_type == PW_BREAK_TYPE_S) { /* Scheduler wakeup */
        fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-20d", DONT_CARE, DONT_CARE, DONT_CARE, cs->c_data);
    }
    else { /* Either 'Begin' wakeup (i.e. "ghost" sample) or Unknown wakeup */
        fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-20s", DONT_CARE, DONT_CARE, DONT_CARE, DONT_CARE);
    }

    fprintf(output_fp, "\n");

    if (m_do_dump_tscs) {
        // fprintf(output_fp, "%20s\t%8d\t%7s\t%15s\t%20llu K", DONT_CARE, cpu, c0_str, DONT_CARE, c0_res / 1000); // print C0
        fprintf(output_fp, "%20s\t%8d\t%7s\t%15s\t%20llu %s", DONT_CARE, cpu, c0_str, DONT_CARE, c0_res, res_suffix); // print C0, div by 1000 handled above
    } else{
        // fprintf(output_fp, "%8d\t%7s\t%15s\t%20llu K", cpu, c0_str, DONT_CARE, c0_res / 1000); // print C0
        fprintf(output_fp, "%8d\t%7s\t%15s\t%20llu %s", cpu, c0_str, DONT_CARE, c0_res, res_suffix); // print C0, div by 1000 handled above
    }

    fprintf(output_fp, "\t%10s\t%5s", DONT_CARE, DONT_CARE);
    fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-20s", DONT_CARE, DONT_CARE, DONT_CARE, DONT_CARE);

    fprintf(output_fp, "\n");
};

/*
 * INTERNAL API:
 * Function to bracket a TSC pair.
 * Performs a simple linear search. The search can be optimized (e.g., by
 * using locality and caching), but experiments are required to determine
 * effectiveness of any such optimization.
 *
 * @tsc: the tsc to bracket
 * @trace_list: the list of [begin,end] tsc pairs.
 *
 * @returns: a backtrace (if found), or NULL else.
 */
const trace_t *CDumper::bracket_tsc_pair_i(const uint64_t& tsc, const trace_pair_list_t& trace_list) const
{
    /*
     * We should think about caching this.
     * Or, failing which, at least do a
     * binary search!
     * (Caching is best -- allows us to
     * incorporate temporal assurances
     * i.e. knowledge that 'trace_list'
     * is globally sorted wrt TSC).
     */
    for (trace_pair_list_t::const_iterator iter = trace_list.begin(); iter != trace_list.end(); ++iter) {
        /*
         * For now, we ASSUME no two
         * [Begin,End] pairs will EVER
         * intersect (i.e. all such
         * pairs are mutually disjoint).
         * ***********************************************
         * THIS IS A POSSIBLY INVALID ASSUMPTION!!!
         * ***********************************************
         */
        if (iter->m_begin < tsc && tsc < iter->m_end) {
            return iter->m_bt;
        }
    }
    return NULL;
};

/*
 * INTERNAL API:
 * Function to return a backtrace, given a TSC, TID pair.
 *
 * @tsc: the tsc value
 * @tid: the thread ID
 *
 * @returns: a valid backtrace, if found. NULL otherwise.
 */
const trace_t *CDumper::get_backtrace_given_tsc_tid_i(const u64& tsc, pid_t tid) const
{
    trace_pair_map_t::const_iterator citer = m_trace_pair_map.find(tid);
    if (citer != m_trace_pair_map.end()) {
        return bracket_tsc_pair_i(tsc, citer->second);
    }
    return NULL;
};

PDumper::PDumper(FILE *fp) : output_fp(fp) {};
/*
 * Functor to print out a P-state transition.
 */
void PDumper::operator()(const PWCollector_sample_t& sample)
{
    assert(sample.sample_type == P_STATE);
    u64 tsc = sample.tsc;
    int cpu = sample.cpuidx;
    u32 act_freq = sample.p_sample.frequency / 1000; // O/P requires freq in MHz
    std::string req_str = "";

    if (sample.p_sample.is_boundary_sample) {
        req_str = DONT_CARE;
    } else {
        char tmp[10];
        sprintf(tmp, "%u", act_freq);
        req_str = tmp;
    }

    fprintf(output_fp, "\n%16llu\t%8d\t%16s\t%16d\n", tsc, cpu, req_str.c_str(), act_freq);
};

s_residency_data::s_residency_data(uint64_t& t, int& c, uint32_t& s0, uint32_t& s1, uint32_t& s2, uint32_t& s3) : tsc(t),cpu(c),S0i0(s0),S0i1(s1),S0i2(s2),S0i3(s3) {};

bool s_residency_sorter::operator()(const s_residency_data_t& p1, const s_residency_data_t& p2) {
    return p1.tsc < p2.tsc;
};

d_nc_state_data::d_nc_state_data(uint64_t& t, uint32_t& s) : tsc(t),states(s) {};

bool d_nc_state_sorter::operator()(const d_nc_state_data_t& p1, const d_nc_state_data_t& p2) {
    return p1.tsc < p2.tsc;
};

d_sc_state_data::d_sc_state_data(uint64_t& t, uint32_t* s):tsc(t) {
    memcpy(states, s, sizeof(uint32_t)*4);
};

bool d_sc_state_sorter::operator()(const d_sc_state_data_t& p1, const d_sc_state_data_t& p2) {
    return p1.tsc < p2.tsc;
};

d_sc_residency_data::d_sc_residency_data(uint64_t& t, uint32_t& i, uint32_t& d0i0, uint32_t& d0i0_acg, uint32_t& d0i1, uint32_t& d0i3):tsc(t),id(i),D0i0(d0i0),D0i0_ACG(d0i0_acg),D0i1(d0i1),D0i3(d0i3) {};

bool d_sc_residency_sorter::operator()(const d_sc_residency_data_t& p1, const d_sc_residency_data_t& p2) {
    return p1.tsc < p2.tsc;
};

wakelock_data::wakelock_data(uint64_t& t, uint32_t& tp, pid_t& p_id, pid_t& t_id, char *n, char *pn):tsc(t),type(tp),pid(p_id),tid(t_id),name(n),proc_name(pn) {};

bool wakelock_sorter::operator()(const wakelock_data_t& p1, const wakelock_data_t& p2) {
    return p1.tsc < p2.tsc;
};

/**********************************************
  WUDUMP FUNCTIONS
 **********************************************/

/*
 * INTERNAL API:
 * The meat of the algorithm. Sort (previously read) samples, 
 * create TPS groupings, measure Cx residencies, determine
 * wakeup causes etc.
 *
 * @returns: 0 on success, -1 on error
 */
int HTWudump::do_parse_i(void)
{
    /*
     * Allocate per-cpu lists.
     */
    m_per_cpu_c_state_samples = new sample_vec_t[NUM_ONLINE_CORES()];
    m_per_cpu_p_state_samples = new sample_vec_t[NUM_ONLINE_CORES()];
    /*
     * Get (parsed) samples from our database.
     */
    const sample_vec_t& samples = pwr::WuData::instance()->getSamples();

    uint32_t s_res[4];
    uint32_t d_res[MAX_LSS_NUM_IN_SC][4];

    memset(s_res, 0, sizeof(uint32_t)*4);
    memset(d_res, 0, sizeof(uint32_t)*MAX_LSS_NUM_IN_SC*4);


    for(sample_vec_t::const_iterator citer = samples.begin(); citer != samples.end(); ++citer) {
        int sample_type = citer->sample_type;
        int core = GET_CORE_GIVEN_LCPU(citer->cpuidx);

        switch (sample_type) {
            case C_STATE:
                m_was_any_tps_sample_present = true;
                m_per_cpu_c_state_samples[core].push_back(*citer);
                break;
            case P_STATE:
                m_was_any_tpf_sample_present = true;
                /*
                 * power library converts TPF cpuidx of TPF samples
                 * to core id (for AXE compatibility). Therefore, use
                 * the 'cpuidx' field directly.
                 */
                // m_per_cpu_p_state_samples[core].push_back(*citer);
                m_per_cpu_p_state_samples[citer->cpuidx].push_back(*citer);
                break;
            case K_CALL_STACK:
                /* NOP */
                break;
            case S_RESIDENCY:
                {
                    int cpu = citer->cpuidx;
                    uint64_t tsc = citer->tsc;
                    uint32_t s0i1 = citer->s_residency_sample.counters[0] - s_res[1];
                    uint32_t s0i2 = citer->s_residency_sample.counters[1] - s_res[2];
                    uint32_t s0i3 = citer->s_residency_sample.counters[2] - s_res[3];
                    uint32_t s0i0 = citer->s_residency_sample.usec - s_res[0] - (s0i1 + s0i2 + s0i3);
                    if (citer->s_residency_sample.usec > 0) {
                        m_s_residency_vec.push_back(s_residency_data_t(tsc, cpu, s0i0, s0i1, s0i2, s0i3));
                    }
                    // Residency counter values are cumulative. So, need to calculate the diff.
                    s_res[0] = citer->s_residency_sample.usec;
                    s_res[1] = citer->s_residency_sample.counters[0];
                    s_res[2] = citer->s_residency_sample.counters[1];
                    s_res[3] = citer->s_residency_sample.counters[2];
                }
                break;
                case S_STATE:
                    assert(false);
                    break;
                case D_RESIDENCY:
                    {
                        int idx=0;
                        uint64_t tsc = citer->tsc;
                        if (citer->d_residency_sample.device_type == PW_SOUTH_COMPLEX) {
                            const char *mask = citer->d_residency_sample.mask;
                            uint32_t num = citer->d_residency_sample.num_sampled;

                            for (idx=0; idx<num; idx++) {
                                uint32_t id = mask[idx];
                                if (id < MAX_LSS_NUM_IN_SC) {
                                    uint32_t d0i0_acg = (citer->d_residency_sample.d_residency_counters[idx].D0i0_ACG - d_res[id][1]) * 1000; // Convert in usec
                                    uint32_t d0i1 = (citer->d_residency_sample.d_residency_counters[idx].D0i1 - d_res[id][2]) * 1000;
                                    uint32_t d0i3 = (citer->d_residency_sample.d_residency_counters[idx].D0i3 - d_res[id][3]) * 1000;
                                    uint32_t d0i0 = (citer->d_residency_sample.d_residency_counters[idx].usec - d_res[id][0]) - (d0i0_acg + d0i1 + d0i3);
                                    if (citer->d_residency_sample.d_residency_counters[idx].usec > 0) {
                                        m_d_sc_residency_vec.push_back(d_sc_residency_data_t(tsc, id, d0i0, d0i0_acg, d0i1, d0i3));
                                    }
                                    // Residency counter values are cumulative. So, need to calculate the diff.
                                    d_res[id][0] = citer->d_residency_sample.d_residency_counters[idx].usec;
                                    d_res[id][1] = citer->d_residency_sample.d_residency_counters[idx].D0i0_ACG;
                                    d_res[id][2] = citer->d_residency_sample.d_residency_counters[idx].D0i1;
                                    d_res[id][3] = citer->d_residency_sample.d_residency_counters[idx].D0i3;
                                }
                            }
                        }
                    }
                    break;
                case D_STATE:
                    {
                        uint64_t tsc = citer->tsc;
                        uint32_t *states = const_cast<uint32_t *>(citer->d_state_sample.states);
                        if (citer->d_state_sample.device_type == PW_SOUTH_COMPLEX) {
                            assert(false);
                        } else if (citer->d_state_sample.device_type == PW_NORTH_COMPLEX) {
                            m_d_nc_state_vec.push_back(d_nc_state_data_t(tsc, states[0]));
                        }
                    }
                    break;
                case IRQ_MAP:
                    /*
                     * Store the IRQ # <-> Name mapping.
                     */
                    m_i_sample_map[citer->i_sample.irq_num] = citer->i_sample.irq_name;
                    break;
                case PROC_MAP:
                    /*
                     * Store the PID <-> Proc Name mapping.
                     */
                    switch (citer->r_sample.type) {
                        case PW_PROC_FORK:
                        case PW_PROC_EXEC:
                            m_r_sample_map[citer->r_sample.tid] = r_sample_pair_t(citer->r_sample.pid, citer->r_sample.proc_name);
                            break;
                        case PW_PROC_EXIT:
                            break;
                        default:
                            assert(false);
                    }
                    break;
                case W_STATE:
                    {
                        uint64_t tsc = citer->tsc;
                        uint32_t type = citer->w_sample.type;
                        pid_t pid = citer->w_sample.pid;
                        pid_t tid = citer->w_sample.tid;
                        char *str = const_cast<char *>(citer->w_sample.name);
                        char *pname = const_cast<char *>(citer->w_sample.proc_name);
                        m_w_sample_vec.push_back(wakelock_data_t(tsc, type, pid, tid, str, pname));
                    }
                    break;
                default:
                    assert(false);
                    break;
        }
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Write all output information to the output file.
 *
 * @returns: 0 on success, -1 on error
 */
int HTWudump::do_write_i(void)
{
    int core = -1;
    FILE *output_fp = fopen(m_output_file_name.c_str(), "w");

    if (output_fp == NULL) {
        db_fprintf(stderr, "ERROR opening wudump output file: %s\n", strerror(errno));
        return -ERROR;
    }
    /*
     * Taken from original 'wudump'
     */
    {
        /*
         * Write current time to output_fp.
         */
        time_t rawtime;
        struct tm * timeinfo;
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        std::string currentTime = asctime(timeinfo);
        currentTime.erase(currentTime.find_last_of("\n"));
        fprintf(output_fp,"Data collected %s (wuwatch v%s, driver v%s, hook lib v%s, wudump v%s ) \n", currentTime.c_str(), 
                pwr::WuData::instance()->getSystemInfo().m_wuwatchVersion.c_str(), 
                pwr::WuData::instance()->getSystemInfo().m_driverVersion.c_str(), 
                pwr::WuData::instance()->getSystemInfo().m_hookLibraryVersion.c_str(), 
                pwr::WuData::instance()->getSystemInfo().m_wudumpVersion.c_str());

        fprintf(output_fp, "Processor C-States Found : ");
        {
            /*
             * 'C0' isn't on the list of residencies -- special case it here.
             */
            fprintf(output_fp, "  C0");
            for (std::map<uint32_t, uint32_t>::const_iterator iter = pwr::WuData::instance()->getSystemInfo().m_targetRes.begin();
                    iter != pwr::WuData::instance()->getSystemInfo().m_targetRes.end(); ++iter){
                fprintf(output_fp, "  C%d", iter->first);
            }
        }
        fprintf(output_fp, "\nNumber of Cores = %d", NUM_ONLINE_CORES());
        fprintf(output_fp, "\nNumber of Logical CPU's = %d\n", NUM_ONLINE_CPUS());
        fprintf(output_fp, "\nTSC Frequency (MHz) = %u", pwr::WuData::instance()->getSystemInfo().m_tscFreq);
        /*
         * Per Neha's request: write the START and STOP TSC values.
         */
        {
            fprintf(output_fp, "\nCollection start TSC value = %llu", pwr::WuData::instance()->getSystemInfo().m_startTsc);
            fprintf(output_fp, "\nCollection stop TSC value = %llu", pwr::WuData::instance()->getSystemInfo().m_stopTsc);
        }
        fprintf(output_fp, "\n\n");
        {
            /*
             * Write target residency list.
             */
            fprintf(output_fp, "\nTARGET RESIDENCIES\n");
            for (std::map<uint32_t, uint32_t>::const_iterator iter = pwr::WuData::instance()->getSystemInfo().m_targetRes.begin();
                    iter != pwr::WuData::instance()->getSystemInfo().m_targetRes.end(); ++iter){
                if (m_do_raw_output) {
                    fprintf(output_fp, "C%d = %d\n", iter->first, iter->second);
                } else {
                    fprintf(output_fp,"C%d = %d K\n",iter->first,iter->second / 1000);
                }
            }
        }
        fprintf(output_fp,"\n\nTotal Collection Time = %s (secs)\n\n",pwr::WuData::instance()->getSystemInfo().m_collectionTime.c_str());
    }
    /*
     * End taken from original 'wudump'
     */
    /*
     * OK, we're ready to output
     * sample information -- begin
     * by printing out header
     * info.
     */
    if (m_was_any_tps_sample_present) {
        fprintf(output_fp, "C-State Samples\n");
        if (m_do_dump_tscs) {
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");
            fprintf(output_fp,"\n%32s%16s%16s\t%14s%14s%11s","CORE", "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP");
            fprintf(output_fp, "\n%14s\t%16s\t%8s\t%9s\t%14s\t%12s\t%7s\t%10s\t%6s\t%10s\t%16s\n", "TSC", "ID","C-STATE","C-STATE ","COUNT","EVENT","CPU", "TID","PID","IRQ", "Additional Info");
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");
        } else {
            fprintf(output_fp,"%s\n","****************************************************************************************************************************************************");
            fprintf(output_fp,"\n%8s%16s%16s\t%14s%14s%11s","CORE", "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP");
            fprintf(output_fp, "\n%8s\t%8s\t%9s\t%14s\t%12s\t%7s\t%10s\t%6s\t%10s\t%16s\n", "ID","C-STATE","C-STATE ","COUNT","EVENT","CPU", "TID","PID","IRQ", "Additional Info");
            fprintf(output_fp,"%s\n","****************************************************************************************************************************************************");
        }
        /*
         * Then print out C-state samples...
         */
        for_each_online_core(core) {
            const sample_vec_t& samples = m_per_cpu_c_state_samples[core];
            CDumper cdumper = CDumper(output_fp, m_do_dump_backtraces, m_do_dump_tscs, m_do_raw_output, pwr::WuData::instance()->get_tid_backtrace_map(), m_r_sample_map, m_i_sample_map);
            std::for_each(samples.begin(), samples.end(), cdumper);
        }
    }
    /*
     * ...and then the P-state samples (if present)...
     */
    if (m_was_any_tpf_sample_present) {
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");
        fprintf(output_fp, "P-State Samples\n");
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");
        fprintf(output_fp, "%32s\t%16s\t%16s\n", "Core", "Requested", "Actual");
        fprintf(output_fp, "%16s\t%8s\t%16s\t%16s\n","TSC","ID","Frequency(Mhz)", "Frequency(Mhz)");
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");

        for_each_online_core(core) {
            const sample_vec_t& samples = m_per_cpu_p_state_samples[core];
            std::for_each(samples.begin(), samples.end(), PDumper(output_fp));
        }
    }
    /*
     * And then the other sample types (if present).
     */
    if (m_s_residency_vec.size() > 0) {
        /*
         * Dump S-state residency samples.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "S-State Samples (all state data shown in usecs)\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%16s\t%16s\t%16s\t%16s\n", "TSC", "S0i0", "S0i1", "S0i2", "S0i3");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_s_residency_vec.begin(), m_s_residency_vec.end(), s_residency_sorter());

        /*
         * Sort all s-state samples first.
         */
        for (s_residency_vec_t::iterator iter = m_s_residency_vec.begin(); iter != m_s_residency_vec.end(); ++iter) {
            uint64_t tsc = iter->tsc;
            uint32_t s0i0 = iter->S0i0;
            uint32_t s0i1 = iter->S0i1;
            uint32_t s0i2 = iter->S0i2;
            uint32_t s0i3 = iter->S0i3;
            fprintf(output_fp, "\n%20llu\t%16d\t%16d\t%16d\t%16d\n", tsc, s0i0, s0i1, s0i2, s0i3);
        }
    }


    if (m_d_sc_residency_vec.size() > 0) {
        /*
         * Dump D-state residency samples in South complex.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "South Complex D-State Residency Samples (all state data shown in usecs)\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        printSCDeviceList_i(output_fp);
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%8s\t%16s\t%16s\t%16s\t%16s\n", "TSC", "Device", "D0i0_AON", "D0i0_ACG", "D0i1", "D0i3");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        //std::sort(m_d_sc_residency_vec.begin(), m_d_sc_residency_vec.end(), d_sc_residency_sorter());

        /*
         * Sort all d-state residency samples first.
         */
        for (d_sc_residency_vec_t::iterator iter = m_d_sc_residency_vec.begin(); iter != m_d_sc_residency_vec.end(); ++iter) {
            fprintf(output_fp, "\n%20llu\t%8s\t%16d\t%16d\t%16d\t%16d\n", iter->tsc, mfd_sc_device_names[iter->id][0], iter->D0i0, iter->D0i0_ACG, iter->D0i1, iter->D0i3);
        }
    }


    if (m_d_nc_state_vec.size() > 0) {
        /*
         * Dump D-state in North complex samples.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "North Complex D-State Samples\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        printNCDeviceList_i(output_fp);
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%8s\t%8s\n", "TSC", "Device", "State");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_d_nc_state_vec.begin(), m_d_nc_state_vec.end(), d_nc_state_sorter());

        /*
         * Sort all d-state samples first.
         */
        for (d_nc_state_vec_t::iterator iter = m_d_nc_state_vec.begin(); iter != m_d_nc_state_vec.end(); ++iter) {
            uint64_t tsc = iter->tsc;
            uint32_t states = iter->states;
            for (int i=0; i<MAX_LSS_NUM_IN_NC; i++) {
                fprintf(output_fp, "\n%20llu\t%8s\t%8d\n", tsc, mfd_nc_device_names[i][0], (states >> (i*2)) & 0x3);
            }
        }
    }


    if (m_d_sc_state_vec.size() > 0) {
        /*
         * Dump D-state samples in South complex.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "South Complex D-State Samples\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        printSCDeviceList_i(output_fp);
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%8s\t%8s\n", "TSC", "Device", "State");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_d_sc_state_vec.begin(), m_d_sc_state_vec.end(), d_sc_state_sorter());

        /*
         * Sort all d-state samples first.
         */
        for (d_sc_state_vec_t::iterator iter = m_d_sc_state_vec.begin(); iter != m_d_sc_state_vec.end(); ++iter) {
            int quotient = MAX_LSS_NUM_IN_SC/16;
            int remainder = MAX_LSS_NUM_IN_SC%16;
            uint64_t tsc = iter->tsc;
            for (int i=0; i<quotient; i++) {
                for (int j=0; j<16; j++) {
                    fprintf(output_fp, "\n%20llu\t%8s\t%8d\n", tsc, mfd_sc_device_names[j+16*i][0], (iter->states[i] >> (j*2)) & 0x3);
                }
            }
            for (int i=0; i<remainder; i++) {
                fprintf(output_fp, "\n%20llu\t%8s\t%8d\n", tsc, mfd_sc_device_names[i+16*quotient][0], (iter->states[quotient] >> (i*2)) & 0x3);
            }
        }
    }


    if (m_w_sample_vec.size() > 0) {
        /*
         * Dump Wakelock samples.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "Wakelock Samples\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%8s\t%8s\t%8s\t%32s\t%32s\n", "TSC", "TYPE", "PID", "TID", "Wakelock Name", "Proc Name");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_w_sample_vec.begin(), m_w_sample_vec.end(), wakelock_sorter());

        /*
         * Sort all wakelock samples first.
         */
        for (w_sample_vec_t::iterator iter = m_w_sample_vec.begin(); iter != m_w_sample_vec.end(); ++iter) {
            if (iter->type)
                fprintf(output_fp, "\n%20llu\t%8s\t%8d\t%8d\t%32s\t%32s\n", iter->tsc, "UNLOCK", iter->pid, iter->tid, iter->name.c_str(), iter->proc_name.c_str());
            else
                fprintf(output_fp, "\n%20llu\t%8s\t%8d\t%8d\t%32s\t%32s\n", iter->tsc, "LOCK", iter->pid, iter->tid, iter->name.c_str(), iter->proc_name.c_str());
        }
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
    }
    /*
     * OK, everything done -- now close output file.
     */
    fclose(output_fp);
    return SUCCESS;
};
/*
 * INTERNAL API:
 * Function to print out North Complex device names.
 *
 * @output_fp: the file to print to.
 */
void HTWudump::printNCDeviceList_i(FILE *output_fp)
{
    int i;
    if (pwr::WuData::instance()->getSystemInfo().m_arch == MFD) {
        for (i=0; i<MAX_LSS_NUM_IN_NC; i++) {
            fprintf(output_fp, "%8s : %32s\n", mfd_nc_device_names[i][0], mfd_nc_device_names[i][1]);
        }
    }
};

/*
 * INTERNAL API:
 * Function to print out South Complex device names.
 *
 * @output_fp: the file to print to.
 */
void HTWudump::printSCDeviceList_i(FILE *output_fp)
{
    int i;
    if (pwr::WuData::instance()->getSystemInfo().m_arch == MFD) {
        for (i=0; i<MAX_LSS_NUM_IN_SC; i++) {
            fprintf(output_fp, "%8s : %32s\n", mfd_sc_device_names[i][0], mfd_sc_device_names[i][1]);
        }
    }
};

/*
 * EXTERNAL API:
 * Set the various file names we're going to be using, including
 * the file that the data collector wrote to, and the file
 * that we're going to write to.
 *
 * @dir: the directory name.
 * @file: the file name prefix. It is assumed that wuwatch will have written
 * to a file called @file.ww1, and that we're going to write our information
 * to @file.txt
 */
void HTWudump::set_output_file_name(const std::string& dir, const std::string& file)
{
    assert(dir.size() && file.size());
    m_wuwatch_output_dir = dir;
    m_wuwatch_file_name = file + ".ww1";
    m_output_file_name = dir + file + ".txt";
    db_fprintf(stderr, "HT-wudump has input dir = %s, file = %s, output file = %s\n", m_wuwatch_output_dir.c_str(), m_wuwatch_file_name.c_str(), m_output_file_name.c_str());
};


static const std::string unknown_proc_name = std::string(UNKNOWN_PROCESS_NAME);
/*
 * INTERNAL API:
 * Helper function to retrieve a process name given a thread ID.
 *
 * @tid: the thread ID
 *
 * @returns: (a constant reference to) the process name corresponding to @tid.
 */
const std::string& HTWudump::get_proc_name_given_tid(pid_t tid) const {
    r_sample_map_t::const_iterator citer = m_r_sample_map.find(tid);
    if (citer != m_r_sample_map.end()) {
        return citer->second.second;
    }
    return unknown_proc_name;
};
/*
 * INTERNAL API: 
 * Helper function to retrieve the device name corresponding to an IRQ.
 *
 * @irq: the irq number.
 *
 * @returns: (a constant reference to) the device name corresponding to @irq.
 */
const std::string& HTWudump::get_irq_name_given_num(int irq) const {
    return m_i_sample_map.find(irq)->second;
};

/*
 * EXTERNAL API:
 * Main function:
 * (1) Read samples.
 * (2) Parse them, exctracting relevent information.
 * (3) Print out results.
 *
 * @returns: 0 on success, -1 on error
 */
int HTWudump::do_work(void)
{
    int retVal = SUCCESS;

    /*
     * Read wuwatch output files. We contract out the reading
     * to a power 'library'.
     */
    if (pwr::WuData::instance()->do_read_and_process(m_wuwatch_output_dir, m_wuwatch_file_name, m_do_check_c1_res, m_do_dump_orig_samples)) {
        fprintf(stderr, "ERROR reading in wuwatch data!\n");
        retVal = -ERROR;
        goto done;
    }
    if (false) {
        const sample_vec_t& samples = pwr::WuData::instance()->getSamples();
        std::copy(samples.begin(), samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
    }
    /*
     * Parse information. Create output samples.
     */
    if (do_parse_i()) {
        fprintf(stderr, "ERROR parsing data!\n");
        retVal = -ERROR;
        goto done;
    }
    /*
     * Everything done -- write info.
     */
    if (do_write_i()) {
        fprintf(stderr, "ERROR writing data!\n");
        retVal = -ERROR;
        goto done;
    }

done:
    /*
     * Destroy our power data repository. This is a repository
     * that's created by the power 'library'.
     */
    pwr::WuData::instance()->destroy();

    return retVal;
};
