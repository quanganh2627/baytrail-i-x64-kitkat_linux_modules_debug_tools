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

#include "pw_defines.h"
#include "pw_ioctl.h" // IOCTL stuff.
#include "pw_arch.h" // Architecture info.
#include "pw_bt.h"
#include "pw_utils.hpp"
#include "ksym_extractor.hpp"
#include "defines.h"
#include "ht_wudump_defines.h"
#include "ht_wudump.h"
#include "wulib.h"

/* **************************************
 * Some useful typedefs.
 * **************************************
 */
typedef std::string str_t;
typedef std::vector <std::string> str_vec_t;

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
    uint64_t S0i0;
    uint64_t S0i1;
    uint64_t S0i2;
    uint64_t S0i3;
    uint64_t S3;
    s_residency_data(uint64_t&, int&, uint64_t&, uint64_t&, uint64_t&, uint64_t&, uint64_t&);
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
    uint64_t D0i0;
    uint64_t D0i0_ACG;
    uint64_t D0i1;
    uint64_t D0i3;
    d_sc_residency_data(uint64_t&, uint32_t&, uint64_t&, uint64_t&, uint64_t&, uint64_t&);
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
    uint64_t expires;
    wakelock_data(uint64_t&, uint32_t&, pid_t&, pid_t&, char*, char*, uint64_t&);
};

struct wakelock_sorter {
    bool operator()(const wakelock_data_t& p1, const wakelock_data_t& p2);
};

/*
 * Data structure to encapsuate user wakelock samples
 */
struct userwakelock_data {
    uint64_t tsc;
    uint32_t type;
    uint32_t flag;
    uint32_t count;
    pid_t pid;
    uint32_t uid;
    std::string tag;
    userwakelock_data(uint64_t&, uint32_t&, uint32_t&, uint32_t&, pid_t&, uint32_t&, char*);
};

struct userwakelock_sorter {
    bool operator()(const userwakelock_data_t& p1, const userwakelock_data_t& p2);
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
 * C-state and P-state samples.
 */
struct PerCoreCDumper {
    FILE *output_fp;
    bool m_do_dump_backtraces, m_do_dump_tscs, m_do_raw_output, m_do_c_res_in_clock_ticks;
    const trace_pair_map_t& m_trace_pair_map;
    const r_sample_map_t& m_r_sample_map;
    const i_sample_map_t& m_i_sample_map;
    float m_usecs_per_clock_tick;
    const sample_vec_t& c_samples;
    const sample_vec_t& p_samples;

    PerCoreCDumper(FILE *fp, const bool& b, const bool& t, const bool& r, const bool& c, const sample_vec_t& c_samples, const sample_vec_t& p_samples, const trace_pair_map_t& t_map, const r_sample_map_t& r_map, const i_sample_map_t& i_map);

    const trace_t *bracket_tsc_pair_i(const uint64_t& tsc, const trace_pair_list_t& trace_list) const;
    const trace_t *get_backtrace_given_tsc_tid_i(const u64& tsc, pid_t tid) const;

    void dump_one_c_sample_i(const PWCollector_sample_t&, const std::vector<std::pair<pw_u64_t, pw_u32_t> >&);

    void dump_c_samples();
};

struct PerCorePDumper {
    FILE *output_fp;
    PerCorePDumper(FILE *);
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
const char *HTWudump::break_type_names_deprecated[] = {"I", "T", "S", "IPI", "W", "B", "N", "?"};
const char *HTWudump::break_type_names[] = {"I", "T", "S", "IPI", "W", "B", "-", "A", "?"}; // "PW_BREAK_TYPE_N" lines should be printed with a "-", per Bob's request
// const char *HTWudump::break_type_names[] = {"I", "T", "S", "IPI", "W", "B", "N", "A", "?"}; // "PW_BREAK_TYPE_N" lines should be printed with a "-", per Bob's request
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
                                {"IPH", "IPH Power Island"} };

const char *clv_nc_device_names[][2] = {
                                {"GPS", "GFX subsystem"},
                                {"VDPS", "Video Decode subsystem"},
                                {"VEPS", "Video Encode subsystem"},
                                {"DPA", "Display Island A"},
                                {"DPB", "Display Island B"},
                                {"ISP", "ISP Power Island"},
                                {"IPH", "IPH Power Island"} };

const char *mfd_sc_device_names[][2] = { 
                                {"LSS00", "Storage: SDIO0 (HC2)"},
                                {"LSS01", "Storage: eMMC0 (HC0a)"},
                                {"LSS03", "HSI: HSI DMA"},
                                {"LSS04", "Security: RNG, ROM (64KB), Chaabi, RAM (24KB)"},
                                {"LSS05", "Storage: eMMC1 (HC0b)"},
                                {"LSS06", "USB: USB OTG (ULPI)"},
                                {"LSS08", "Audio: Diamond330, DMA,Fabric, IRAM, DRAM"},
                                {"LSS09", "Audio: DMA1"},
                                {"LSS10", "SRAM: SRAM BANK (16KB), SRAM controller"},
                                {"LSS12", "SRAM: SRAM BANK (16KB+3x32KBKB)"},
                                {"LSS13", "SRAM: SRAM BANK (4x32KB)"},
                                {"LSS14", "SDIO COMMS: SDIO2 (HC1b)"},
                                {"LSS16", "SC: DMA"},
                                {"LSS17", "SC: SPI0"},
                                {"LSS18", "GP: SPI1"},
                                {"LSS19", "GP: SPI2"},
                                {"LSS20", "GP: I2C0"},
                                {"LSS21", "GP: I2C1"},
                                {"LSS27", "GP: I2C2"},
                                {"LSS30", "SDIO COMMS: SDIO1 (HC1a)"},
                                {"LSS33", "GP: I2C3 (HDMI)"},
                                {"LSS34", "GP: I2C4"},
                                {"LSS35", "GP: I2C5"},
                                {"LSS36", "GP: SSP (SPI3)"},
                                {"LSS39", "SC: GPIO0"},
                                {"LSS40", "SC: KBD"},
                                {"LSS41", "SC: UART2:0"},
                                {"LSS44", "Security: Security TPAC"},
                                {"LSS51", "Audio: SSP0"},
                                {"LSS52", "Audio: SSP1"},
                                {"LSS54", "GP: DMA"} };

const char *clv_sc_device_names[][2] = { 
                                {"LSS00", "Storage: SDIO0 (HC2)"},
                                {"LSS01", "Storage: eMMC0 (HC0a)"},
                                {"LSS03", "HSI: HSI DMA + MIPI HSI"},
                                {"LSS04", "Security: RNG, ROM (64KB), Chaabi, RAM (24KB)"},
                                {"LSS05", "Storage: eMMC1 (HC0b)"},
                                {"LSS06", "USB: USB OTG (ULPI)"},
                                {"LSS07", "USB: SPH (Host ULPI1), USB PLL"},
                                {"LSS08", "Audio: Diamond330, DMA,Fabric, IRAM, DRAM, SSP4"},
                                {"LSS09", "Audio: DMA1"},
                                {"LSS14", "SDIO COMMS: SDIO2 (HC1b)"},
                                {"LSS18", "GP: SPI1"},
                                {"LSS19", "GP: SPI2"},
                                {"LSS20", "GP: I2C0"},
                                {"LSS21", "GP: I2C1"},
                                {"LSS27", "GP: I2C2"},
                                {"LSS30", "SDIO COMMS: SDIO1 (HC1a)"},
                                {"LSS33", "GP: I2C3 (HDMI)"},
                                {"LSS34", "GP: I2C4"},
                                {"LSS35", "GP: I2C5"},
                                {"LSS36", "GP: SSP (SPI3)"},
                                {"LSS40", "SC: KBD"},
                                {"LSS41", "SC: UART2:0"},
                                {"LSS51", "Audio: SSP0"},
                                {"LSS52", "Audio: SSP1"},
                                {"LSS54", "GP: DMA"} };

/* **************************************
 * Function definitions.
 * **************************************
 */

/*
 * The wudump constructor.
 */
HTWudump::HTWudump() : PW_max_num_cpus(-1), m_per_cpu_c_state_samples(NULL), m_per_cpu_p_state_samples(NULL),
    m_do_raw_output(true), m_do_dump_tscs(true), m_do_dump_backtraces(false), m_do_check_c1_res(true), m_do_c_res_in_clock_ticks(true),
    m_do_dump_orig_samples(false), m_do_dump_sample_stats(false), m_do_check_c_state_demotions(false),
    /*m_do_delete_driver_files(false),*/ m_was_any_tps_sample_present(false), m_was_any_tpf_sample_present(false),
    m_wuwatch_file_name(""), m_output_file_name("") {};

/*
 * The wudump destructor.
 */
HTWudump::~HTWudump() {
    /*
     * Delete any open 'lib_input_XXX.txt' files we may have.
     */
    fp_vec_t::iterator iter;
    for (iter = m_lib_input_fps.begin(); iter != m_lib_input_fps.end(); ++iter) {
        db_fprintf(stderr, "Closing %p\n", *iter);
        if (*iter) {
            fclose(*iter);
        }
    }
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

static void find_frequencies_i(std::vector<std::pair<pw_u64_t, pw_u32_t> >& frequencies, const pw_u64_t& c0_start_tsc, const pw_u64_t& c0_stop_tsc, sample_vec_t::const_iterator& curr_p, const sample_vec_t::const_iterator& last_p)
{
    /*
     * Basic algo:
     * for-each p-state sample p st c0_start < TSC(p) < c0_stop: add 'p' to list of frequencies.
     * Add first 'p' st. c0_stop <= TSC(p) to list of frequencies.
     */
    pw_u64_t __c0_tsc = c0_start_tsc;
    if (curr_p == last_p) {
        // No point in proceeding if we're already at the end!
        frequencies.push_back(std::pair<pw_u64_t, pw_u32_t>(c0_stop_tsc - __c0_tsc, 0x0));
        return;
    }
    // Step 1: find the first P-state sample whose TSC > c0_start_tsc
    for (; curr_p != last_p && curr_p->tsc < c0_start_tsc; ++curr_p);
    // Step 2: then try to find any ranges
    for (; curr_p != last_p && curr_p->tsc < c0_stop_tsc; ++curr_p) {
        frequencies.push_back(std::pair<pw_u64_t, pw_u32_t>(curr_p->tsc - __c0_tsc, curr_p->p_sample.frequency));
        __c0_tsc = curr_p->tsc;
    }
    // Step 3: add the last P-state sample
    frequencies.push_back(std::pair<pw_u64_t, pw_u32_t>(c0_stop_tsc - __c0_tsc, curr_p->p_sample.frequency));
};

/*
 * Instance functions for helper
 * structs declared earlier.
 */
PerCoreCDumper::PerCoreCDumper(FILE *fp, const bool& b, const bool& t, const bool& r, const bool& c, const sample_vec_t& cs, const sample_vec_t& ps, const trace_pair_map_t& t_map, const r_sample_map_t& r_map, const i_sample_map_t& i_map) :  output_fp(fp), m_do_dump_backtraces(b), m_do_dump_tscs(t), m_do_raw_output(r), m_do_c_res_in_clock_ticks(c), c_samples(cs), p_samples(ps), m_trace_pair_map(t_map), m_r_sample_map(r_map), m_i_sample_map(i_map), 
    m_usecs_per_clock_tick(1.0 / (float)pwr::WuData::instance()->getSystemInfo().m_tscFreq) {};

void PerCoreCDumper::dump_c_samples()
{
    std::vector<PWCollector_sample_t>::const_iterator p_iter = p_samples.begin(), last_p_iter = p_samples.end();
    for (sample_vec_t::const_iterator c_iter = c_samples.begin(); c_iter != c_samples.end(); ++c_iter) {
        std::vector<std::pair<pw_u64_t, pw_u32_t> > frequencies;
        pw_u64_t __c0_stop = c_iter->tsc, __c0_start = c_iter->tsc - RES_COUNT(c_iter->c_sample, MPERF);
        find_frequencies_i(frequencies, __c0_start, __c0_stop, p_iter, last_p_iter);
        dump_one_c_sample_i(*c_iter, frequencies);
    }
};

void PerCoreCDumper::dump_one_c_sample_i(const PWCollector_sample_t& sample, const std::vector<std::pair<pw_u64_t, pw_u32_t> >& frequencies)
{
    assert(sample.sample_type == C_STATE);
    /*
     * Helper macro: convert an integer to a string.
     */
#define GET_STRING_FROM_INT(i) ({std::stringstream __tmp; __tmp << (i); __tmp.str();})
#define GET_PMC_ID_STR(p, m, c) ((p) + ", " + (m) + ", " + (c))
#define GET_PACKAGE_PMC_ID_STR(id) GET_PMC_ID_STR(GET_STRING_FROM_INT(GET_PACKAGE_GIVEN_CORE(id)), "-", "-")
#define GET_MODULE_PMC_ID_STR(id) GET_PMC_ID_STR(GET_STRING_FROM_INT(GET_PACKAGE_GIVEN_CORE(id)), GET_STRING_FROM_INT(GET_MODULE_GIVEN_CORE(id)), "-")
#define GET_CORE_PMC_ID_STR(id) GET_PMC_ID_STR(GET_STRING_FROM_INT(GET_PACKAGE_GIVEN_CORE(id)), "-", GET_STRING_FROM_INT(id))

    std::string payload;
    pid_t tid = 0, pid = 0;
    int irq_num = -1;
    int cpu = sample.cpuidx;
    u64 tsc = sample.tsc;
    static bool is_first = true;
    /*
    char cx_str[10];
    char req_str[10];
    */
    std::string cx_str, req_str;
    const c_sample_t *cs = &sample.c_sample;
    int break_type = cs->break_type;
    // int which_cx = cs->prev_state, which_cpuidx = cs->tps_epoch;
    // int which_cx = GET_CX_FROM_COMBINED(cs->tps_epoch), which_cpuidx = GET_CPUID_FROM_COMBINED(cs->tps_epoch);
    // int req_state = (cs->prev_state >> 8) & 0xff, act_state = (cs->prev_state & 0xff);
    int req_state = GET_REQ_FROM_PREV(cs->prev_state), act_state = GET_ACT_FROM_PREV(cs->prev_state);
    int which_cpuidx = cs->tps_epoch;
    u64 c0_res = RES_COUNT(*cs, MPERF), cx_res = RES_COUNT(*cs, act_state);
    const char *c0_str = "0";
    std::string str_res_suffix = "";
    const char *res_suffix = NULL;
    const trace_t *bt = NULL;
    int act_pkg_id = pwr::WuData::instance()->getSystemInfo().m_abstractPkgMap.find(GET_PACKAGE_GIVEN_CORE(cpu))->second;
    std::string pkg_id_str = GET_STRING_FROM_INT(act_pkg_id), mod_id_str = "-", core_id_str = "-";
    bool is_saltwell = PW_IS_SALTWELL(pwr::WuData::instance()->getSystemInfo().m_cpuModel);
    bool is_pkg_sample = is_saltwell && act_state > APERF; // should really be using the "IS_PACKAGE_SAMPLE()" macro!!!
    bool is_mod_sample = false;
    std::string id_str = "";
    std::vector <std::pair<pw_u64_t, std::string> > c0_freq_mhz_strs;

    if (is_pkg_sample) {
        mod_id_str = core_id_str = "-";
    } else if (is_mod_sample) {
        mod_id_str = GET_STRING_FROM_INT(GET_MODULE_GIVEN_CORE(cpu));
        core_id_str = "-";
    } else {
        // core_id_str = GET_STRING_FROM_INT(cpu);
        core_id_str = GET_STRING_FROM_INT(pwr::WuData::instance()->getSystemInfo().m_abstractCoreMap.find(cpu)->second);
        mod_id_str = "-";
    }

    std::string c0_freq_mhz_str = "", cx_freq_mhz_str = "";

#if 0
    if (is_saltwell) {
        cx_frequency_khz = (act_state < C4) ? c0_frequency_khz : pwr::WuData::instance()->getSystemInfo().m_availableFrequenciesKHz.back();
    } else {
        cx_frequency_khz = (act_state > APERF) ? 0 : pwr::WuData::instance()->getSystemInfo().m_availableFrequenciesKHz.back();
    }
#endif
    /*
     * Get string representations of the frequency values; we require strings because we print out
     * "-" in case the user didn't request P-states.
     */
    assert(frequencies.size());
    db_fprintf(stderr, "TSC = %llu had %u frequencies!\n", sample.tsc, frequencies.size());
    // cx_freq_mhz_str = GET_STRING_FROM_INT(frequencies.front().second / 1000); // convert from KHz to MHz
    for (int i=0; i<frequencies.size(); ++i) {
        pw_u32_t __freq = frequencies[i].second;
        std::string __freq_str = __freq ? GET_STRING_FROM_INT(__freq / 1000) : DONT_CARE;
        c0_freq_mhz_strs.push_back(std::pair<pw_u64_t, std::string>(frequencies[i].first, __freq_str));
        if (i == 0) {
            if (is_saltwell) {
                cx_freq_mhz_str = (act_state < C4) ? __freq_str : GET_STRING_FROM_INT(pwr::WuData::instance()->getSystemInfo().m_availableFrequenciesKHz.back() / 1000);
            } else {
                cx_freq_mhz_str = (act_state > APERF) ? "0" : GET_STRING_FROM_INT(pwr::WuData::instance()->getSystemInfo().m_availableFrequenciesKHz.back() / 1000); 
            }
        }
        // c0_freq_mhz_strs.push_back(std::pair<pw_u64_t, std::string>(frequencies[i].first, GET_STRING_FROM_INT(frequencies[i].second / 1000))); // convert from KHz to MHz
        pw_u64_t& __c0_res = c0_freq_mhz_strs.back().first;
        if (m_do_c_res_in_clock_ticks == false) {
            double tmp_res = (1.0 * __c0_res * m_usecs_per_clock_tick);
            __c0_res = (u64)tmp_res;
        }

        if (m_do_raw_output == false) {
            __c0_res /= 1000;
        }
    }

    if (m_do_c_res_in_clock_ticks == false) {
        double tmp_res = (1.0 * cx_res * m_usecs_per_clock_tick);
        cx_res = (u64)tmp_res;
        tmp_res = (1.0 * c0_res * m_usecs_per_clock_tick);
        c0_res = (u64)tmp_res;
    }

    if (m_do_raw_output == false) {
        cx_res /= 1000; c0_res /= 1000;
        str_res_suffix = "K";
    } else {
        str_res_suffix = "";
    }

    res_suffix = str_res_suffix.c_str();

    if (is_first) {
        // sprintf(req_str, "%s", DONT_CARE);
        req_str = DONT_CARE;
        is_first = false;
    } else {
        /*
         * We've already mapped the requested mwait using
         * the 'm_tps_state_to_c_state' mapping. No
         * need to do so again.
         */
        // sprintf(req_str, "%d", req_state);
        req_str = GET_STRING_FROM_INT(req_state);
    }

    // sprintf(cx_str, "%d", act_state);
    cx_str = GET_STRING_FROM_INT(act_state);
    if (unlikely(PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel) && act_state == APERF && req_state != APERF)) {
        /*
         * CLTP specific 'intermediate' state: state entered by a core (that has NOT requested 'C1') while waiting for the other
         * core's C-state to be resolved. We mark these as 'C1i' or '1i'.
         */
        switch (break_type) {
            /*
               case PW_BREAK_TYPE_N:
               cx_str += 'i';
               */
            case PW_BREAK_TYPE_A: // fall-through
                break;
            default:
                // fprintf(stderr, "%llu -> %llu, %s\n", sample.tsc, cx_res, GET_BOOL_STRING(IS_C1I_FLAG_SET(cx_res)));
                if (m_do_c_res_in_clock_ticks) {
                    assert(IS_C1I_FLAG_SET(cx_res));
                    RESET_C1I_FLAG(cx_res);
                }
                cx_str += 'i';
                break;
        }
    }

    if (m_do_dump_tscs) {
        // fprintf(output_fp, "%20llu\t%8s\t%7s\t%15s\t%20llu %s", TO_ULL(tsc), id_str.c_str(), cx_str, req_str, TO_ULL(cx_res), res_suffix); // print Cx, div by 1000 handled above
        fprintf(output_fp, "%20llu\t%8s%5s%5s%11s%11s%20llu %s", TO_ULL(tsc), pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), cx_str.c_str(), req_str.c_str(), TO_ULL(cx_res), res_suffix); // print Cx, div by 1000 handled above
    } else {
        // fprintf(output_fp, "%8d\t%7s\t%15s\t%20llu %s", cpu, cx_str, req_str, TO_ULL(cx_res), res_suffix); // print Cx, div by 1000 handled above
        fprintf(output_fp, "%8s%5s%5s%11s%11s%20llu %s", pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), cx_str.c_str(), req_str.c_str(), TO_ULL(cx_res), res_suffix); // print Cx, div by 1000 handled above
    }
    fprintf(output_fp, "\t%10s\t%5d", HTWudump::break_type_names[break_type], which_cpuidx);

    if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        assert(break_type != PW_BREAK_TYPE_N); // sanity!
    }

    if (break_type == PW_BREAK_TYPE_T) { /* Timer wakeup */
        pid = cs->pid;
        tid = cs->tid;
        r_sample_map_t::const_iterator citer = m_r_sample_map.find(tid);
        if (citer != m_r_sample_map.end()) {
            payload = citer->second.second;
        } else {
            payload = UNKNOWN_PROCESS_NAME;
        }
        // fprintf(output_fp, "\t%9d\t%6d\t%9s\t %-20s", tid, pid, DONT_CARE, payload.c_str());
        fprintf(output_fp, "\t%9d\t%6d\t%9s\t %-2s\t       %-10s", tid, pid, DONT_CARE, cx_freq_mhz_str.c_str(), payload.c_str());
        if (m_do_dump_backtraces) {
            /*
             * Also need to extract a back trace here!
             */
            u64 timer_init_tsc = cs->c_data;
            bt = get_backtrace_given_tsc_tid_i(timer_init_tsc, tid);
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
        fprintf(output_fp, "\t%9s\t%6s\t%9d\t %-2s\t       %-10s", DONT_CARE, DONT_CARE, irq_num, cx_freq_mhz_str.c_str(), payload.c_str());
    }
    else if (break_type == PW_BREAK_TYPE_S) { /* Scheduler wakeup */
        fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-2s\t       %-10d", DONT_CARE, DONT_CARE, DONT_CARE, cx_freq_mhz_str.c_str(), (int)cs->c_data);
    }
    else { /* Either 'Begin' wakeup (i.e. "ghost" sample) or Unknown wakeup */
        fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-2s\t%8s", DONT_CARE, DONT_CARE, DONT_CARE, cx_freq_mhz_str.c_str(), DONT_CARE);
    }

    // fprintf(output_fp, "\t%2s", cx_freq_mhz_str.c_str());
    
    if (m_do_dump_backtraces && bt) {
        /*
         * We have backtrace information -- print it out.
         */
        fprintf(output_fp, "\n\n");
        for (int i=0; i<bt->num_trace; ++i) {
            fprintf(output_fp, "%s\n", bt->bt_symbols[i]);
        }
    }

    fprintf(output_fp, "\n");

    c0_res = c0_freq_mhz_strs.front().first;
    c0_freq_mhz_str = c0_freq_mhz_strs.front().second;

    if (m_do_dump_tscs) {
        // fprintf(output_fp, "%20s\t%8d\t%7s\t%15s\t%20llu K", DONT_CARE, cpu, c0_str, DONT_CARE, c0_res / 1000); // print C0
        fprintf(output_fp, "%20s\t%8s%5s%5s%11s%11s%20llu %s", DONT_CARE, pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), c0_str, DONT_CARE, TO_ULL(c0_res), res_suffix); // print C0, div by 1000 handled above
    } else{
        // fprintf(output_fp, "%8d\t%7s\t%15s\t%20llu K", cpu, c0_str, DONT_CARE, c0_res / 1000); // print C0
        fprintf(output_fp, "%8s%5s%5s%11s%11s%20llu %s", pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), c0_str, DONT_CARE, TO_ULL(c0_res), res_suffix); // print C0, div by 1000 handled above
        // fprintf(output_fp, "%8s%5s%5s%11s%11s%20llu %s", pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), cx_str, req_str, TO_ULL(cx_res), res_suffix); // print Cx, div by 1000 handled above
    }

    fprintf(output_fp, "\t%10s\t%5s", DONT_CARE, DONT_CARE);
    fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-2s\t%8s", DONT_CARE, DONT_CARE, DONT_CARE, c0_freq_mhz_str.c_str(), DONT_CARE);

    if (unlikely(c0_freq_mhz_strs.size() > 1)) {
        for (int i=1; i<c0_freq_mhz_strs.size(); ++i) {
            fprintf(output_fp, "\n");
            c0_res = c0_freq_mhz_strs[i].first;
            c0_freq_mhz_str = c0_freq_mhz_strs[i].second;
            if (m_do_dump_tscs) {
                fprintf(output_fp, "%20s\t%8s%5s%5s%11s%11s%20llu %s", DONT_CARE, pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), c0_str, DONT_CARE, TO_ULL(c0_res), res_suffix);
            } else {
                fprintf(output_fp, "%8s%5s%5s%11s%11s%20llu %s", pkg_id_str.c_str(), mod_id_str.c_str(), core_id_str.c_str(), c0_str, DONT_CARE, TO_ULL(c0_res), res_suffix);
            }
            fprintf(output_fp, "\t%10s\t%5s", DONT_CARE, DONT_CARE);
            fprintf(output_fp, "\t%9s\t%6s\t%9s\t %-2s\t%8s", DONT_CARE, DONT_CARE, DONT_CARE, c0_freq_mhz_str.c_str(), DONT_CARE);
        }
    }

    // fprintf(output_fp, "\t%2s", c0_freq_mhz_str.c_str());

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
const trace_t *PerCoreCDumper::bracket_tsc_pair_i(const uint64_t& tsc, const trace_pair_list_t& trace_list) const
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
const trace_t *PerCoreCDumper::get_backtrace_given_tsc_tid_i(const u64& tsc, pid_t tid) const
{
    trace_pair_map_t::const_iterator citer = m_trace_pair_map.find(tid);
    if (citer != m_trace_pair_map.end()) {
        return bracket_tsc_pair_i(tsc, citer->second);
    }
    return NULL;
};

PerCorePDumper::PerCorePDumper(FILE *fp) : output_fp(fp) {};
void PerCorePDumper::operator()(const PWCollector_sample_t& sample)
{
    assert(sample.sample_type == P_STATE);
    u64 tsc = sample.tsc;
    int cpu = sample.cpuidx;
    u32 act_freq = sample.p_sample.frequency / 1000; // O/P requires freq in MHz
    std::string req_str = "";

    if (sample.p_sample.is_boundary_sample) {
        req_str = DONT_CARE;
    } else {
        char tmp[11];
        snprintf(tmp, sizeof(tmp), "%u", act_freq);
        req_str = tmp;
    }

    fprintf(output_fp, "\n%16llu\t%8d\t%16s\t%16d\n", TO_ULL(tsc), cpu, req_str.c_str(), act_freq);
};



s_residency_data::s_residency_data(uint64_t& t, int& c, uint64_t& s0i0, uint64_t& s0i1, uint64_t& s0i2, uint64_t& s0i3, uint64_t& s3) : tsc(t),cpu(c),S0i0(s0i0),S0i1(s0i1),S0i2(s0i2),S0i3(s0i3),S3(s3) {};

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

d_sc_residency_data::d_sc_residency_data(uint64_t& t, uint32_t& i, uint64_t& d0i0, uint64_t& d0i0_acg, uint64_t& d0i1, uint64_t& d0i3):tsc(t),id(i),D0i0(d0i0),D0i0_ACG(d0i0_acg),D0i1(d0i1),D0i3(d0i3) {};

bool d_sc_residency_sorter::operator()(const d_sc_residency_data_t& p1, const d_sc_residency_data_t& p2) {
    return p1.tsc < p2.tsc || p1.id < p2.id;
};

wakelock_data::wakelock_data(uint64_t& t, uint32_t& tp, pid_t& p_id, pid_t& t_id, char *n, char *pn, uint64_t& to):tsc(t),type(tp),pid(p_id),tid(t_id),name(n),proc_name(pn),expires(to) {};

bool wakelock_sorter::operator()(const wakelock_data_t& p1, const wakelock_data_t& p2) {
    return p1.tsc < p2.tsc;
};

userwakelock_data::userwakelock_data(uint64_t& t, uint32_t& tp, uint32_t& fg, uint32_t& ct, pid_t& p_id, uint32_t& u_id, char *n):tsc(t),type(tp),flag(fg),count(ct),pid(p_id),uid(u_id),tag(n) {};

bool userwakelock_sorter::operator()(const userwakelock_data_t& p1, const userwakelock_data_t& p2) {
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
 * @returns: 0 on PW_SUCCESS, -1 on error
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

    /*
     * The power library does NOT parse any 'lib_input_XXX.txt' files (i.e. files
     * containing userspace call traces). Do so now.
     *
     * Step (1): Retrieve the list of descendent pids and open the file(s) containing
     * their backtraces.
     */
    const str_vec_t& descendents = pwr::WuData::instance()->getSystemInfo().m_descendentPids;
    for (str_vec_t::const_iterator citer = descendents.begin(); citer != descendents.end(); ++citer) {
        std::stringstream full_name;
        full_name << m_wuwatch_output_dir << "/lib_output_" << *citer << ".txt";
        db_fprintf(stderr, "FULL lib name = %s\n", full_name.str().c_str());
        m_lib_input_fps.push_back(fopen(full_name.str().c_str(), "r"));
    }
    /*
     * Step (2): read the lib_outputXXX.txt files (if any).
     */
    {
        db_fprintf(stderr, "Size = %lu\n", TO_UL(m_lib_input_fps.size()));
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
            m_hookLibraryVersion = ver_str;
            // sysInfo.m_hookLibraryVersion = ver_str;
#endif
        }
        /*
         * Make sure 'Tracer' is destroyed.
         */
        Tracer::destroy();
    }

    std::vector<pw_u64_t> __prev_tscs(NUM_ONLINE_CORES());

    for(sample_vec_t::const_iterator citer = samples.begin(); citer != samples.end(); ++citer) {
        int sample_type = citer->sample_type;
        // int core = GET_CORE_GIVEN_LCPU(citer->cpuidx);
        pw_u32_t core = citer->cpuidx;
        PWCollector_sample_t sample;

        db_fprintf(stderr, "core = %u, IS_PACKAGE_SAMPLE = %s, IS_LCPU_SAMPLE = %s, IS_MODULE_SAMPLE = %s, IS_CORE_SAMPLE = %s\n", core, GET_BOOL_STRING(IS_PACKAGE_SAMPLE(core)), GET_BOOL_STRING(IS_LCPU_SAMPLE(core)), GET_BOOL_STRING(IS_MODULE_SAMPLE(core)), GET_BOOL_STRING(IS_CORE_SAMPLE(core)));

        switch (sample_type) {
            case C_STATE:
                m_was_any_tps_sample_present = true;
                // m_per_cpu_c_state_samples[core].push_back(*citer);
                sample = *citer;
                /*
                 * Also add the frequency value. Note that if the user requested P-states then we are GUARANTEED to have seen the
                 * P-state samples before the C-state samples, so the 'm_per_cpu_p_state_samples[]' value will be non-empty.
                 * TODO: handle 'PACKAGE' C-state samples!!!
                 */
                // fprintf(stderr, "core = %u, IS_PACKAGE_SAMPLE = %s, IS_LCPU_SAMPLE = %s, IS_MODULE_SAMPLE = %s, IS_CORE_SAMPLE = %s\n", core, GET_BOOL_STRING((core & PACKAGE_CPUIDX_MASK) == PACKAGE_CPUIDX_MASK), GET_BOOL_STRING(IS_LCPU_SAMPLE(core)), GET_BOOL_STRING(IS_MODULE_SAMPLE(core)), GET_BOOL_STRING(IS_CORE_SAMPLE(core)));
                if (IS_CORE_SAMPLE(core)) {
                    m_per_cpu_c_state_samples[core].push_back(sample);
                    if (true) {
                        // Sanities!
                        if (__prev_tscs[core] != 0x0) {
                            pw_u64_t __curr_tsc = sample.tsc;
                            pw_u32_t __curr_req = GET_REQ_FROM_PREV(sample.c_sample.prev_state), __curr_act = GET_ACT_FROM_PREV(sample.c_sample.prev_state);
                            pw_u64_t __c0_res = RES_COUNT(sample.c_sample, MPERF), __cx_res = RES_COUNT(sample.c_sample, __curr_act);
                            pw_u64_t __tsc_delta = __curr_tsc - __prev_tscs[core];
                            if (__curr_act == APERF) {
                                RESET_C1I_FLAG(__cx_res);
                            }
                            // Sanity: __tsc_delta = __c0_res + __cx_res
                            // fprintf(stderr, "Core %d: curr_tsc = %llu tsc_delta = %llu cx_total = %llu\n", core, __curr_tsc, __tsc_delta, (__c0_res + __cx_res));
                            // assert(__tsc_delta == (__c0_res + __cx_res));
                            // fprintf(stderr, "OK: core %d: curr_tsc = %llu, tsc_delta = %llu, cx_total = %llu\n", core, __curr_tsc, __tsc_delta, (__c0_res + __cx_res));
                            if (__tsc_delta != (__c0_res + __cx_res)) {
                                fprintf(stderr, "ERROR: Core %d: curr_tsc = %llu tsc_delta = %llu cx_total = %llu [req = %u, act = %u]\n", core, __curr_tsc, __tsc_delta, (__c0_res + __cx_res), __curr_req, __curr_act);
                            }
                        }
                        __prev_tscs[core] = sample.tsc;
                    }
                } else {
                    // db_abort("TODO: handle package c-state samples!\n");
                }
                break;
            case P_STATE:
                m_was_any_tpf_sample_present = true;
                m_per_cpu_p_state_samples[core].push_back(*citer);
                break;
            case K_CALL_STACK:
                {
                    const k_sample_t *ks = &citer->k_sample;
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
                }
                break;
            case M_MAP:
                /*
                 * We only get these samples if given a ".ww1" file that was generated by the AXE power component as
                 * part of the integrity testing infrastructure. For now, silently discard these samples.
                 */
                break;
            case S_RESIDENCY:
                {
                    int cpu = citer->cpuidx;
                    uint64_t tsc = citer->tsc;

                    if (citer->s_residency_sample.data[0] + citer->s_residency_sample.data[1] +
                        citer->s_residency_sample.data[2] + citer->s_residency_sample.data[3] + 
                        citer->s_residency_sample.data[4] > 0) {
                        uint64_t s0i0 = citer->s_residency_sample.data[0];
                        uint64_t s0i1 = citer->s_residency_sample.data[1];
                        uint64_t s0i2 = citer->s_residency_sample.data[2];
                        uint64_t s0i3 = citer->s_residency_sample.data[3];
                        uint64_t s3 = citer->s_residency_sample.data[4];

                        m_s_residency_vec.push_back(s_residency_data_t(tsc, cpu, s0i0, s0i1, s0i2, s0i3, s3));
                    }
                }
                break;
                case S_STATE:
                    assert(false);
                    break;
                case D_RESIDENCY:
                    {
                        uint32_t idx=0;
                        uint64_t tsc = citer->tsc;
                        if (citer->d_residency_sample.device_type == PW_SOUTH_COMPLEX) {
                            const u16* mask = citer->d_residency_sample.mask;
                            uint32_t num = citer->d_residency_sample.num_sampled;

                            for (idx=0; idx<num; idx++) {
                                uint32_t id = mask[idx];
                                if (id < getNumberOfDevices(PW_SOUTH_COMPLEX)) {
                                    if (citer->d_residency_sample.d_residency_counters[idx].data[0] > 0) {
                                        uint64_t d0i0_acg = citer->d_residency_sample.d_residency_counters[idx].data[1] * 1000; // Convert in usec
                                        uint64_t d0i1 = citer->d_residency_sample.d_residency_counters[idx].data[2] * 1000;
                                        uint64_t d0i3 = citer->d_residency_sample.d_residency_counters[idx].data[3] * 1000;
                                        uint64_t d0i0 = citer->d_residency_sample.d_residency_counters[idx].data[0] * 1000 - (d0i0_acg + d0i1 + d0i3);
                                        m_d_sc_residency_vec.push_back(d_sc_residency_data_t(tsc, id, d0i0, d0i0_acg, d0i1, d0i3));
                                    }
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
                        uint64_t timeout = citer->w_sample.expires;
                        m_w_sample_vec.push_back(wakelock_data_t(tsc, type, pid, tid, str, pname, timeout));
                    }
                    break;
                case DEV_MAP:
                    // NOP, for now
                    break;
                case U_STATE:
                    {
                        uint64_t tsc = citer->tsc;
                        uint32_t type = citer->u_sample.type;
                        uint32_t flag = citer->u_sample.flag;
                        pid_t pid = citer->u_sample.pid;
                        uint32_t uid = citer->u_sample.uid;
                        uint32_t count = citer->u_sample.count;
                        char *str = const_cast<char *>(citer->u_sample.tag);
                        m_u_sample_vec.push_back(userwakelock_data_t(tsc, type, flag, count, pid, uid, str));
                    }
                    break;
                case PKG_MAP:
                    {
                        m_pkgmap_sample_map[citer->pkg_sample.uid] = citer->pkg_sample.pkg_name;
                    }
                    break;
                default:
                    fprintf(stderr, "Unknown sample type = %d\n", sample_type);
                    assert(false);
                    break;
        }
    }
    /*
     * Post-processing: calculate H/W P-state residencies.
     */
#if 0
    {
        int core = -1;
        for_each_online_core(core) {
            std::vector<PWCollector_sample_t>::iterator c_iter = m_per_cpu_c_state_samples[core].begin();
            for (std::vector<PWCollector_sample_t>::const_iterator p_iter = m_per_cpu_p_state_samples[core].begin(); p_iter != m_per_cpu_p_state_samples[core].end(); ++p_iter) {
                for (; c_iter != m_per_cpu_c_state_samples[core].end() && c_iter->tsc <= p_iter->tsc; ++c_iter) {
                    if (IS_CORE_SAMPLE(c_iter->cpuidx)) {
                        RES_COUNT(c_iter->c_sample, C9) = p_iter->p_sample.frequency;
                    }
                }
            }
        }
    }
#endif
#if 0
    if (false) {
        int core = -1;
        for_each_online_core(core) {
            std::vector<PWCollector_sample_t>::iterator p_iter = m_per_cpu_p_state_samples[core].begin(), last_p_iter = m_per_cpu_p_state_samples[core].end();
            for (std::vector<PWCollector_sample_t>::iterator c_iter = m_per_cpu_c_state_samples[core].begin(); c_iter != m_per_cpu_c_state_samples[core].end(); ++c_iter) {
                std::vector<std::pair<pw_u64_t, pw_u32_t> > frequencies;
                pw_u64_t __c0_stop = c_iter->tsc, __c0_start = c_iter->tsc - RES_COUNT(c_iter->c_sample, MPERF);
                find_frequencies_i(frequencies, __c0_start, __c0_stop, p_iter, last_p_iter);
                db_fprintf(stderr, "TSC = %llu had %u frequencies!\n", c_iter->tsc, frequencies.size());
                pw_u64_t __from_tsc = __c0_start, __to_tsc = __from_tsc;
                for (int i=0; i<frequencies.size(); ++i) {
                    __to_tsc = __from_tsc + frequencies[i].first;
                    db_fprintf(stderr, "[%llu, %llu] -> %u\n", __from_tsc, __to_tsc, frequencies[i].second);
                    __from_tsc = __to_tsc;
                }
                if (unlikely(frequencies.size() > 1)) {
                    // assert(false);
                }
            }
        }
    }
#endif
    return PW_SUCCESS;
};

/*
 * INTERNAL API:
 * Write all output information to the output file.
 *
 * @returns: 0 on PW_SUCCESS, -1 on error
 */
int HTWudump::do_write_i(void)
{
    int core = -1;
    FILE *output_fp = fopen(m_output_file_name.c_str(), "w");

    if (output_fp == NULL) {
        db_fprintf(stderr, "ERROR opening wudump output file: %s\n", strerror(errno));
        return -PW_ERROR;
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
            fprintf(output_fp, "\nCollection start TSC value = %llu", TO_ULL(pwr::WuData::instance()->getSystemInfo().m_startTsc));
            fprintf(output_fp, "\nCollection stop TSC value = %llu", TO_ULL(pwr::WuData::instance()->getSystemInfo().m_stopTsc));
        }
        fprintf(output_fp, "\n\n");
        {
            /*
             * Write if core C-states are independent. Currently, always independent, EXCEPT for Saltwell-derived cores.
             */
            fprintf(output_fp, "Cores have independent C-states = %s\n\n", PW_IS_SALTWELL(pwr::WuData::instance()->getSystemInfo().m_cpuModel) ? "false" : "true");
        }
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
        const char *c_res_type_str = (m_do_c_res_in_clock_ticks) ? "(clock ticks)" : "(usecs)";
        // const char *proc_type_str = (PW_IS_SALTWELL(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) ? "Package" : "Core";
        // const char * const proc_type_str = "Core";
        const char * const proc_type_str = "P, M, C";
#if 0
        if (m_do_dump_tscs) {
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");
            fprintf(output_fp,"\n%32s%16s%16s\t%14s%14s%11s","CORE", "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP");
            fprintf(output_fp, "\n%14s\t%16s\t%8s\t%9s\t%14s\t%12s\t%7s\t%10s\t%6s\t%10s\t%16s\n", "TSC", "ID","C-STATE","C-STATE ", c_res_type_str,"EVENT","CPU", "TID","PID","IRQ", "Additional Info");
            fprintf(output_fp,"%s\n","*************************************************************************************************************************************************************************");
        } else {
            fprintf(output_fp,"%s\n","****************************************************************************************************************************************************");
            fprintf(output_fp,"\n%8s%16s%16s\t%14s%14s%11s","CORE", "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP");
            fprintf(output_fp, "\n%8s\t%8s\t%9s\t%14s\t%12s\t%7s\t%10s\t%6s\t%10s\t%16s\n", "ID","C-STATE","C-STATE ", c_res_type_str,"EVENT","CPU", "TID","PID","IRQ", "Additional Info");
            fprintf(output_fp,"%s\n","****************************************************************************************************************************************************");
        }
#endif
        {
            const char *c_res_type_str = (m_do_c_res_in_clock_ticks) ? "(clock ticks)" : "(usecs)";
            if (m_do_dump_tscs) {
                fprintf(output_fp,"%s\n","***************************************************************************************************************************************************************************************");
                /*
                fprintf(output_fp,"\n%32s%16s%16s\t%12s%14s%11s%53s", proc_type_str, "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP", "Frequency");
                fprintf(output_fp, "\n%14s\t%16s\t%8s\t%9s\t%12s\t%10s\t%5s\t%8s\t%5s\t%8s\t%6s\t%22s\n", "TSC", "ID","C-STATE","C-STATE ",c_res_type_str,"EVENT","CPU", "TID","PID","IRQ", "(MHz)", "Additional Info");
                */
                fprintf(output_fp,"\n%32s%5s%5s%11s%11s\t%12s%14s%11s%53s", "Pkg", "Mod", "Core", "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP", "Frequency");
                fprintf(output_fp, "\n%14s\t%16s%5s%5s%11s\t%9s\t%12s\t%10s\t%5s\t%8s\t%5s\t%8s\t%6s\t%22s\n", "TSC", "ID", "ID", "ID", "C-STATE","C-STATE ",c_res_type_str,"EVENT","CPU", "TID","PID","IRQ", "(MHz)", "Additional Info");
                fprintf(output_fp,"%s\n","***************************************************************************************************************************************************************************************");
            } else {
                fprintf(output_fp,"%s\n","***************************************************************************************************************************************************************");
                /*
                fprintf(output_fp,"\n%8s%16s%16s\t%12s%16s%13s%53s", proc_type_str, "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP", "Frequency");
                fprintf(output_fp, "\n%8s\t%8s\t%9s\t%14s\t%12s\t%7s\t%10s\t%6s\t%10s\t%6s\t%22s\n", "ID","C-STATE","C-STATE ",c_res_type_str,"EVENT","CPU", "TID","PID","IRQ", "(MHz)", "Additional Info");
                */
                fprintf(output_fp,"\n%8s%5s%5s%10s%12s\t%12s%16s%13s%49s", "Pkg", "Mod", "Core", "ACTUAL", "REQUESTED","RESIDENCY", "WAKEUP", "WAKEUP", "Frequency");
                fprintf(output_fp, "\n%8s%5s%5s%11s\t%9s\t%14s\t%12s\t%7s\t%10s\t%6s\t%10s\t%6s\t%22s\n", "ID","ID", "ID", "C-STATE","C-STATE ",c_res_type_str,"EVENT","CPU", "TID","PID","IRQ", "(MHz)", "Additional Info");
                fprintf(output_fp,"%s\n","***************************************************************************************************************************************************************");
            }
        }
        /*
         * Then print out C-state samples...
         */
        for_each_online_core(core) {
            PerCoreCDumper cpdumper(output_fp, m_do_dump_backtraces, m_do_dump_tscs, m_do_raw_output, m_do_c_res_in_clock_ticks, m_per_cpu_c_state_samples[core], m_per_cpu_p_state_samples[core], m_trace_pair_map, m_r_sample_map, m_i_sample_map);
            cpdumper.dump_c_samples();
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
            PerCorePDumper pdumper(output_fp);
            std::for_each(samples.begin(), samples.end(), pdumper);
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
        fprintf(output_fp, "%16s\t%16s\t%16s\t%16s\t%16s\t%16s\n", "TSC", "S0i0", "S0i1", "S0i2", "S0i3", "S3");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_s_residency_vec.begin(), m_s_residency_vec.end(), s_residency_sorter());

        /*
         * Sort all s-state samples first.
         */
        for (s_residency_vec_t::iterator iter = m_s_residency_vec.begin(); iter != m_s_residency_vec.end(); ++iter) {
            uint64_t tsc = iter->tsc;
            uint64_t s0i0 = iter->S0i0;
            uint64_t s0i1 = iter->S0i1;
            uint64_t s0i2 = iter->S0i2;
            uint64_t s0i3 = iter->S0i3;
            uint64_t s3 = iter->S3;
            fprintf(output_fp, "\n%20llu\t%16llu\t%16llu\t%16llu\t%16llu\t%16llu\n", TO_ULL(tsc), TO_ULL(s0i0), TO_ULL(s0i1), TO_ULL(s0i2), TO_ULL(s0i3), TO_ULL(s3));
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
        std::sort(m_d_sc_residency_vec.begin(), m_d_sc_residency_vec.end(), d_sc_residency_sorter());

        /*
         * Sort all d-state residency samples first.
         */
        for (d_sc_residency_vec_t::iterator iter = m_d_sc_residency_vec.begin(); iter != m_d_sc_residency_vec.end(); ++iter) {
            const char *devname = NULL;
            if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                devname = mfd_sc_device_names[iter->id][0];
            } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                devname = clv_sc_device_names[iter->id][0];
            }
            fprintf(output_fp, "\n%20llu\t%8s\t%16llu\t%16llu\t%16llu\t%16llu\n", TO_ULL(iter->tsc), devname, iter->D0i0, iter->D0i0_ACG, iter->D0i1, iter->D0i3);
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
            for (int i=0; i<getNumberOfDevices(PW_NORTH_COMPLEX); i++) {
                const char *devname = NULL;
                if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    devname = mfd_nc_device_names[i][0];
                } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    devname = clv_nc_device_names[i][0];
                }
                fprintf(output_fp, "\n%20llu\t%8s\t%8d\n", TO_ULL(tsc), devname, (states >> (i*2)) & 0x3);
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
            int quotient = getNumberOfDevices(PW_SOUTH_COMPLEX)/16;
            int remainder = getNumberOfDevices(PW_SOUTH_COMPLEX)%16;
            uint64_t tsc = iter->tsc;
            for (int i=0; i<quotient; i++) {
                for (int j=0; j<16; j++) {
                    const char *devname = NULL;
                    if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                        devname = mfd_sc_device_names[j+16*i][0];
                    } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                        devname = clv_sc_device_names[j+16*i][0];
                    }
                    fprintf(output_fp, "\n%20llu\t%8s\t%8d\n", TO_ULL(tsc), devname, (iter->states[i] >> (j*2)) & 0x3);
                }
            }
            for (int i=0; i<remainder; i++) {
                const char *devname = NULL;
                if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    devname = mfd_sc_device_names[i+16*quotient][0];
                } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    devname = clv_sc_device_names[i+16*quotient][0];
                }
                fprintf(output_fp, "\n%20llu\t%8s\t%8d\n", TO_ULL(tsc), devname, (iter->states[quotient] >> (i*2)) & 0x3);
            }
        }
    }

    if (m_w_sample_vec.size() > 0) {
        /*
         * Dump Wakelock samples.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "Kernel Wakelock Samples\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%16s\t%8s\t%8s\t%32s\t%20s\t%32s\n", "TSC", "TYPE", "PID", "TID", "Wakelock Name", "Expires", "Proc Name");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_w_sample_vec.begin(), m_w_sample_vec.end(), wakelock_sorter());

        /*
         * Sort all wakelock samples first.
         */
        for (w_sample_vec_t::iterator iter = m_w_sample_vec.begin(); iter != m_w_sample_vec.end(); ++iter) {
            switch (iter->type) {
                case 0:
                    fprintf(output_fp, "\n%20llu\t%16s\t%8d\t%8d\t%32s\t%20s\t%32s\n", TO_ULL(iter->tsc), "LOCK", iter->pid, iter->tid, iter->name.c_str(), "-", iter->proc_name.c_str());
                    break;
                case 1:
                    fprintf(output_fp, "\n%20llu\t%16s\t%8d\t%8d\t%32s\t%20s\t%32s\n", TO_ULL(iter->tsc), "UNLOCK", iter->pid, iter->tid, iter->name.c_str(), "-", iter->proc_name.c_str());
                    break;
                case 2:
                    fprintf(output_fp, "\n%20llu\t%16s\t%8d\t%8d\t%32s\t%20llu\t%32s\n", TO_ULL(iter->tsc), "LOCK_TIMEOUT", iter->pid, iter->tid, iter->name.c_str(), TO_ULL(iter->expires), iter->proc_name.c_str());
                    break;
                case 3:
                    fprintf(output_fp, "\n%20llu\t%16s\t%8s\t%8s\t%32s\t%20s\t%32s\n", TO_ULL(iter->tsc), "EXISTING_LOCK", "UNKNOWN", "UNKNOWN", iter->name.c_str(), "-", "-");
                    break;
                default:
                    db_fprintf(stderr, "Unrecognized wakelock trace type = %d\n", iter->type);
            }
        }
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
    }

    if (m_u_sample_vec.size() > 0) {
        /*
         * Dump User Wakelock samples.
         */
        fprintf(output_fp, "\n");
        fprintf(output_fp, "************************************************************************************************************************************************************\n");
        fprintf(output_fp, "User Wakelock Samples\n");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
        fprintf(output_fp, "%16s\t%8s\t%16s\t%8s\t%8s\t%8s\t%32s\t%32s\n", "TSC", "TYPE", "FLAG", "COUNT", "PID", "UID", "TAG", "PACKAGE_NAME");
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");

        // Do sorting
        std::sort(m_u_sample_vec.begin(), m_u_sample_vec.end(), userwakelock_sorter());

        /*
         * Sort all wakelock samples first.
         */
        for (u_sample_vec_t::iterator iter = m_u_sample_vec.begin(); iter != m_u_sample_vec.end(); ++iter) {
            std::string flagname;
            switch(iter->flag) {
                case PW_WAKE_PARTIAL: 
                    flagname = "PARTIAL";
                    break;
                case PW_WAKE_FULL:
                    flagname = "FULL";
                    break;
                case PW_WAKE_SCREEN_DIM:
                    flagname = "SCREEN_DIM";
                    break;
                case PW_WAKE_SCREEN_BRIGHT:
                    flagname = "SCREEN_BRIGHT";
                    break;
                case PW_WAKE_PROXIMITY_SCREEN_OFF:
                    flagname = "SCREEN_OFF";
                    break;
            }
            if (m_pkgmap_sample_map.find(iter->uid) == m_pkgmap_sample_map.end()) {
                char val[10]; 
                snprintf(val, sizeof(val), "%u", iter->uid);
                m_pkgmap_sample_map[iter->uid] = val;
            } 
            if (iter->type) {
                fprintf(output_fp, "\n%20llu\t%8s\t%16s\t%8d\t%8d\t%8d\t%32s\t%32s\n", iter->tsc, "RELEASE", flagname.c_str(), iter->count, iter->pid, iter->uid, iter->tag.c_str(), m_pkgmap_sample_map[iter->uid].c_str());
            }
            else {
                fprintf(output_fp, "\n%20llu\t%8s\t%16s\t%8d\t%8d\t%8d\t%32s\t%32s\n", iter->tsc, "ACQUIRE", flagname.c_str(), iter->count, iter->pid, iter->uid, iter->tag.c_str(), m_pkgmap_sample_map[iter->uid].c_str());
            } 
        }
        fprintf(output_fp, "\n************************************************************************************************************************************************************\n");
    }

    /*
     * OK, everything done -- now close output file.
     */
    fclose(output_fp);
    return PW_SUCCESS;
};

/*
 * INTERNAL API:
 * Function to return the number of devices.
 */
uint32_t HTWudump::getNumberOfDevices(device_type_t ctype)
{
    uint32_t max_lss_num = 0; 
    if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        if (ctype == PW_NORTH_COMPLEX) {
            max_lss_num = MFD_MAX_LSS_NUM_IN_NC; 
        } else if (ctype == PW_SOUTH_COMPLEX) {
            max_lss_num = MFD_MAX_LSS_NUM_IN_SC; 
        }
    } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        if (ctype == PW_NORTH_COMPLEX) {
            max_lss_num = CLV_MAX_LSS_NUM_IN_NC; 
        } else if (ctype == PW_SOUTH_COMPLEX) {
            max_lss_num = CLV_MAX_LSS_NUM_IN_SC; 
        }
    }
    return max_lss_num;
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
    if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        for (i=0; i<getNumberOfDevices(PW_NORTH_COMPLEX); i++) {
            fprintf(output_fp, "%8s : %32s\n", mfd_nc_device_names[i][0], mfd_nc_device_names[i][1]);
        }
    } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        for (i=0; i<getNumberOfDevices(PW_NORTH_COMPLEX); i++) {
            fprintf(output_fp, "%8s : %32s\n", clv_nc_device_names[i][0], clv_nc_device_names[i][1]);
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
    if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        for (i=0; i<getNumberOfDevices(PW_SOUTH_COMPLEX); i++) {
            fprintf(output_fp, "%8s : %32s\n", mfd_sc_device_names[i][0], mfd_sc_device_names[i][1]);
        }
    } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
        for (i=0; i<getNumberOfDevices(PW_SOUTH_COMPLEX); i++) {
            fprintf(output_fp, "%8s : %32s\n", clv_sc_device_names[i][0], clv_sc_device_names[i][1]);
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
 * @returns: 0 on PW_SUCCESS, -1 on error
 */
int HTWudump::do_work(void)
{
    int retVal = PW_SUCCESS;

    /*
     * Read wuwatch output files. We contract out the reading
     * to a power 'library'.
     */
    std::string combined_input_file_name = m_wuwatch_output_dir + m_wuwatch_file_name;
    if (pwr::WuData::instance()->do_read_and_process(combined_input_file_name, m_do_check_c1_res, false /* from AXE */, m_do_dump_orig_samples, m_do_dump_sample_stats)) {
        fprintf(stderr, "ERROR reading in wuwatch data!\n");
        retVal = -PW_ERROR;
        goto done;
    }
    /*
     * Parse information. Create output samples.
     */
    if (do_parse_i()) {
        fprintf(stderr, "ERROR parsing data!\n");
        retVal = -PW_ERROR;
        goto done;
    }
    /*
     * Everything done -- write info.
     */
    if (do_write_i()) {
        fprintf(stderr, "ERROR writing data!\n");
        retVal = -PW_ERROR;
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
