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

#ifndef _HT_WUDUMP_H_
#define _HT_WUDUMP_H_

/* **************************************
 * File containing declarations for the
 * wuwatch parsing tool (aka 'wudump').
 * This version is HT-aware.
 * **************************************
 */

/* **************************************
 * Forward declarations.
 * **************************************
 */
class HTWudump;
struct CDumper;
struct PDumper;
struct proc_struct;

/* **************************************
 * Some useful typedefs.
 * Most of these could be moved into 'ht_wudump.cpp'
 * **************************************
 */
typedef std::vector<PWCollector_sample_t> sample_vec_t;
typedef std::list<PWCollector_sample_t> sample_list_t;
typedef std::string str_t;
typedef std::vector <int> int_vec_t;

typedef std::pair <int,int> int_pair_t;
typedef std::vector <int_pair_t> pair_vec_t;

typedef std::pair<uint64_t, uint64_t> p_sample_pair_t;
typedef std::list<p_sample_pair_t> p_sample_list_t;
typedef std::map<int, p_sample_list_t> p_sample_map_t;
typedef std::pair<pid_t, std::string> r_sample_pair_t;
typedef std::map<pid_t, r_sample_pair_t> r_sample_map_t;
typedef std::map<int, std::string> i_sample_map_t;
typedef struct pcpu_discarded_data pcpu_discarded_data_t;

typedef struct s_residency_data s_residency_data_t;
typedef std::vector<s_residency_data_t> s_residency_vec_t;
typedef struct d_sc_residency_data d_sc_residency_data_t;
typedef std::vector<d_sc_residency_data_t> d_sc_residency_vec_t;
typedef struct d_sc_state_data d_sc_state_data_t;
typedef std::vector<d_sc_state_data_t> d_sc_state_vec_t;
typedef struct d_nc_state_data d_nc_state_data_t;
typedef std::vector<d_nc_state_data_t> d_nc_state_vec_t;
typedef struct wakelock_data wakelock_data_t;
typedef std::vector<wakelock_data_t> w_sample_vec_t;
typedef std::list<proc_struct> r_sample_list_t;


/* **************************************
 * The (HT-aware) data parser.
 * **************************************
 */
class HTWudump {
    private:
        /*
         * The combined output file name. Under new
         * scheme, both driver output and sys params
         * info gets dumped into a single file (with
         * a ".ww<X>" extension).
         */
        std::string m_wuwatch_file_name;
        /*
         * Directory in which the wuwatch output file and files
         * containing hook library call traces are located.
         */
        std::string m_wuwatch_output_dir;
        /*
         * Where should we store intermediate output?
         */
        std::string m_output_file_name;
        /*
         * String equivalents of the PW_BREAK_TYPE_XXX
         * break type enum values.
         * We have 2 versions of this array: the first corresponds
         * to older drivers (deprecated), which did NOT support
         * the 'workqueue' break-type. The second corresponds
         * to newer drivers which DO support that break-type.
         * We use the 'm_driverVersion' field of the "SystemInfo"
         * class to decide which array to use.
         */
        static const char* break_type_names_deprecated[]; //= {"I", "T", "S", "IPI", "?"};
        static const char* break_type_names[]; //= {"I", "T", "S", "IPI", "W", "?"};
        /*
         * For HT-testing. These correspond to the various
         * "sample_type_t" enum values. We have a long and 
         * a short version of each name.
         */
        static const char *s_long_sample_names[];
        static const char *s_short_sample_names[];
        /*
         * # cpus on TARGET machine
         * i.e. on machine where experiment
         * was conducted.
         */
        int PW_max_num_cpus;
        /*
         * Map to hold TID <-> call trace mappings.
         */
        trace_pair_map_t m_trace_pair_map;
        /*
         * Map to hold PID <-> PROC Name mappings.
         */
        r_sample_map_t m_r_sample_map;
        /*
         * Map to hold IRQ # <-> DEV name mappings.
         */
        i_sample_map_t m_i_sample_map;
        /*
         * Hold S-residency sample info.
         */
        s_residency_vec_t m_s_residency_vec;
        /*
         * Hold D-residency in South Complex sample info.
         */
        d_sc_residency_vec_t m_d_sc_residency_vec;
        /*
         * Hold D-residency in South Complex sample info.
         */
        d_sc_state_vec_t m_d_sc_state_vec;
        /*
         * Hold D-residency in North Complex sample info.
         */
        d_nc_state_vec_t m_d_nc_state_vec;
        /*
         * Hold Wakelock sample info.
         */
        w_sample_vec_t m_w_sample_vec;
        /*
         * (Per-core) List of C-state samples to output.
         */
        sample_vec_t *m_per_cpu_c_state_samples;
        /*
         * (Per-core) List of P-state samples to output.
         */
        sample_vec_t *m_per_cpu_p_state_samples;
        /*
         * Flag to indicate whether
         * we should output RAW data.
         */
        bool m_do_raw_output;
        /*
         * Flag to indicate whether we should dump unprocessed
         * samples returned by the driver.
         */
        bool m_do_dump_orig_samples;
        /*
         * Special flag to dump TSC -- this
         * is different from 'raw' because
         * here we still give residency numbers
         * in units of thousands (unlike 'raw'
         * mode where we dump the actual
         * residency numbers).
         * UPDATE: this now defaults to TRUE
         */
        bool m_do_dump_tscs;
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
         * Should we try and determine C1 residencies?
         */
        bool m_do_check_c1_res;
        /*
         * Should we check for C-state DEMOTIONS?
         */
        bool m_do_check_c_state_demotions;
        /*
         * Should we delete wuwatch files after we're
         * done?
         * UPDATE: disabled for now
         */
        // bool m_do_delete_driver_files;
        /*
         * Did we detect any TPS samples? We use this
         * in lieu of a concrete list of options the
         * user passed to the wuwatch collector. Later, 
         * we'll probably add just such a list to the
         * 'sys_params_found' section.
         */
        bool m_was_any_tps_sample_present;
        /*
         * Did we detect any TPF samples?
         */
        bool m_was_any_tpf_sample_present;

        /*
         * Friends.
         */
        friend int parse_args(char ***,int);
        friend void operator<<(std::ostream& os, const PWCollector_sample_t& sample);
        friend struct CDumper;
        friend struct PDumper;

    private:
        int do_parse_i();
        int do_write_i();

        void printNCDeviceList_i(FILE *);
        void printSCDeviceList_i(FILE *);

        const trace_t *bracket_tsc_pair_i(const uint64_t& , const trace_pair_list_t& ) const;

        int advance_i(int index);

    public:

        HTWudump();
        ~HTWudump();

        int do_work();

        void set_output_file_name(const std::string&, const std::string&);

        const std::string& get_proc_name_given_tid(pid_t) const;
        const std::string& get_irq_name_given_num(int) const;
        const trace_t *get_backtrace_given_tsc_tid(const u64&, pid_t) const;
};


extern void wu_exit(int);

extern HTWudump *g_wudumpObj;

#endif // _HT_WUDUMP_H_
