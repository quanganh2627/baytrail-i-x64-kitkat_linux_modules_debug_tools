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
 *  File containing declarations for the
 *  wuwatch data collector tool (aka
 *  'wuwatch').
 * **************************************
 */

#ifndef WUWATCH_H
#define WUWATCH_H

/* **************************************
 * Some useful typedefs.
 * **************************************
 */
typedef std::pair<pid_t, std::string> pid_name_pair_t;
typedef std::map<int,std::string> acpi_mapping_t;
typedef std::vector<int> int_vec_t;
typedef int_vec_t fd_vec_t;
typedef std::map<pid_t, int> offset_map_t;
typedef std::vector<std::string> str_vec_t;
typedef std::map<std::string,bool> str_map_t;
typedef std::map <int,int> int_map_t;

/* **************************************
 * The data collector.
 * **************************************
 */
class Wuwatch
{
    private:
        /*
         * Variable declarations/definitions
         */
        pid_t profiled_app_pid;
        std::string profiled_app_name;
        /*
         * (Relative or Absolute) Path to the device driver object file.
         * e.g. "/data/wuwatch/apwr2_2.ko"
         * e.g. "./driver/apwr2_2.ko"
         */
        std::string m_driver_path;
        /*
         * The combined output file name. Under new
         * scheme, both driver output and sys params
         * info gets dumped into a single file (with
         * a ".ww<X>" extension).
         */
        std::string m_output_file_name;
        /*
         * CPU topology information for target machine.
         */
        std::string m_cpu_topology_str;
        /*
         * Number of logical procs on target machine.
         */
        int m_max_num_cpus;
        /*
         * Device driver fd.
         */
        int m_dev_fd;
        /*
         * Fd for communication with PRELOAD library.
         */
        int fork_listenfd;
        /*
         * SHM area for communication with PRELOAD library.
         */
#if !_ANDROID_
        shm_data_t *fork_shm_data;
#endif
        /*
         * FILE name for named pipe.
         */
        std::string uds_file_name;
        /*
         * File pointer for output file.
         */
        FILE *m_output_fp;
        /*
         * Thread ID of internal 'reader' thread i.e. the
         * thread that reads data from the power driver.
         */
        pthread_t reader_tid;
        /*
         * Barrier for communication with the reader
         * thread.
         */
        // pthread_barrier_t reader_barrier;
        /*
         * Pid of the application that's been forked.
         */
        pid_t child_pid;
        /*
         * Device driver version string -- retrieved from
         * the power driver.
         */
        std::string m_driver_version_string;
        /*
         * IAFW patch version number -- retrieved from
         * the power driver.
         */
        int micro_patch_ver;
        /*
         * (Calculated) TSC frequency, in MHz.
         */
        u32 tsc_freq_MHz;
        /*
         * The target architecture type -- one of 'NHM', 
         * 'SNB' or 'MFD'
         */
        arch_type_t target_arch_type;
        /*
         * A list of pids to wait on -- useful only if we're
         * trying to "hook" the "daemon()" syscall.
         */
        int_vec_t descendent_pids;
        /*
         * Collection START TSC.
         */
        u64 m_initialTSC;
        /*
         * Collection STOP TSC.
         */
        u64 m_finalTSC;
        /*
         * Collection START timeval.
         */
        u64 initialTimeval;
        /*
         * Turbo threshold frequency, in MHz.
         */
        u32 turboThreshold;
        /*
         * cpu 'brand' information, as retrieved from the
         * 'cpuid' instruction.
         */
        char cpu_brand[16];
        /*
         * CPU F.M.S information.
         */
        u32 cpu_family, cpu_model, cpu_stepping;
        /*
         * 'hostname()' information.
         */
        struct utsname un;

        /* **************************************
         * Switches to determine data collection 
         * functionality.
         * **************************************
         */
        /*
         * Collect C-State data
         */
        int c_state_collection;
        /*
         * Collect P-State data
         */
        int p_state_collection;
        /*
         * Collect S-State residency counters
         */
        int s_residency_collection;
        /*
         * Collect S-State samples
         * UPDATE: disabled for now.
         */
        // int s_state_collection;
        /*
         * Collect D-State residency counters
         */
        int d_residency_collection;
        /*
         * Collect north complex D-State samples
         */
        int d_nc_state_collection;
        /*
         * Collect south complex D-State samples
         * UPDATE: disabled for now.
         */
        // int d_sc_state_collection;
        /*
         * Collect wakelock samples
         */
        int w_state_collection;
        /*
         * D-State sample interval in msec
         */
        int d_state_sample_interval_msecs;

        /*
         * Load the driver if not already loaded.
         */
        int m_do_force_dd_load;
        /*
         * Collect call stacks for kernel timers that
         * cause processor wakeups.
         */
        int m_do_collect_kernel_backtrace;
        /*
         * List of SHM offsets -- also serves
         * as a list of which descendent processes
         * are still running.
         */
        offset_map_t offsets;
        /*
         * A list of frequencies the processor(s) may execute at.
         * Obtained from the driver via an IOCTL.
         */
        u32 m_available_frequencies[PW_MAX_NUM_AVAILABLE_FREQUENCIES];

        /*
         * String representation of the various 'arch_type_t' values.
         */
        static const char *g_arch_type_names[];
        /*
         * A list of MSR addresses.
         * ***************************************************************************************
         * CAUTION: ELEMENTS BELOW SHOULD BE SORTED IN SAME ORDER AS "arch_type_t" ENUM VALUES!!!
         * ***************************************************************************************
         */
        static const int s_coreResidencyMSRAddresses[][MAX_MSR_ADDRESSES];
        /*
         * A list of target residencies, in micro-seconds.
         * ***************************************************************************************
         * CAUTION: ELEMENTS BELOW SHOULD BE SORTED IN SAME ORDER AS "arch_type_t" ENUM VALUES!!!
         * ***************************************************************************************
         */
        static const int s_target_residencies_us[][MAX_MSR_ADDRESSES];
        /*
         * A list of bus clock frequencies, in KHz.
         */
        static const int s_busClockFreqKHz[];
        /*
         * Measured collection time, in msecs.
         */
        double collection_time_msecs;
        /*
         * (User-supplied) Timeout (in secs) for
         * system collection mode.
         */
        int system_collection_mode_secs;

        friend int parse_args(char ***,int);


    private:

        /*
         * Internal helper struct to pass args to the
         * reader thread.
         */
        typedef struct args {
            int m_dev_fd;
            FILE *m_output_fp;
            args(int d=-1, FILE *f=NULL):m_dev_fd(d), m_output_fp(f) {};
        } args_t;
        /*
         * Internal helper struct to pass args to the
         * reader thread.
         */
        struct reader_thread_arg {
            Wuwatch *who;
            args_t *args;

            reader_thread_arg(Wuwatch *w, args_t *a) : who(w), args(a) {};
        };

        /*
         * Wrapper for the reader thread function. Required
         * because C++ doesn't play nicely with pthreads.
         */
        static void *reader_thread(void *);
        /*
         * The reader thread function. This function retrieves information
         * from the power driver.
         */
        void *reader_thread_i(args_t *);

        /*
         * Functions to load a driver, open a connection to it and
         * then, when the collection is over, to close the connection.
         */
        int do_insmod_i(void);
        int open_dd_i(void);
        int close_dd_i(void);

        /*
         * Helper function to parse '/proc/cpuinfo' and extract
         * CPU 'topology' information.
         */
        std::string get_cpu_topology_i(void);

    public:
        /*
         * Constructor.
         */
        Wuwatch();
        /*
         * Destructor.
         */
        ~Wuwatch();
        /*
         * Various instance functions.
         * Some of these are no longer used
         * and are deprecated.
         */
        void do_exit(void);
        void *do_ioctl_i(int , int , void *, int , bool );
        void do_ioctl_start_stop(int , bool );
        int do_ioctl_driver_version_i(int , std::string& );
        int do_ioctl_micro_patch(int , int& );
        int do_ioctl_irq_mappings(int , /*FILE *out_fp,*/ std::vector<PWCollector_irq_mapping_t>& );
        bool irq_sorter(const PWCollector_irq_mapping_t& , const PWCollector_irq_mapping_t&);
        int do_ioctl_proc_mappings(int , std::map<pid_t, std::string>& );
        int get_arch_type(arch_type_t&);
        unsigned long long rdtsc(void);
        unsigned int get_tsc_freq_MHz_i();
        void do_ioctl_config(int , int *, int, u32 );
        int do_ioctl_available_frequencies(int, u32 *);
        int do_ioctl_get_turbo_threshold(int , u32 *);
        int do_ioctl_check_platform_i(int, u64&);

        void sigint_handler(int);
        void start_stop_timer(double *, bool);
        void usage(void);

        int get_next_pid_i(DIR *, std::string&);
        int readline(const std::string& , std::string& );
        int parse_stat_line_i(const std::string& , std::string& );
        int extract_pid_tid_name_i(const std::string& , const str_vec_t& , str_map_t& , const char *, std::vector<PWCollector_sample_t>& );
        int get_all_tasks_for_proc_i(std::string , str_vec_t& );
        int get_initial_proc_map_i(FILE *);
        void setup_collection();
        void extract_acpi_c_state_mappings(acpi_mapping_t& );
        void cleanup_collection(bool should_write_results);
        int  fork_child(char **);
        void init_fork_listener(void);
        void destroy_fork_listener(void);
        void init_shm();
        void destroy_shm();
        void shm_send(int , uds_msg_type_t );
        inline int inc_dec_num_shm_clients(int );
        void send_quit_to_all_i(const offset_map_t& );
        void uds_send(int , uds_msg_type_t , int , int );
        int  work(char **);
        void unlink_fork_listener(void);

        void set_output_file_name(const std::string&, const std::string&);
        void set_driver_path(const std::string&);

        int get_driver_version(std::string&);

};

extern void wu_exit(int);

extern Wuwatch *g_wuwatchObj;

#endif // WUWATCH_H
