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

#ifndef _WULIB_H_
#define _WULIBH_H_

/* *****************************************
 * Declarations for the (HT-aware) power
 * library.
 * *****************************************
 */

/* *****************************************
 * Some useful typedefs.
 * *****************************************
 */
#if defined(_WIN32)
#define SPRINTF(b1, s1, ...) sprintf_s(b1, s1, __VA_ARGS__)
#else
#define SPRINTF(b1, s1, ...) sprintf(b1, s1, __VA_ARGS__)
#endif

typedef std::vector<PWCollector_sample_t> sample_vec_t;
typedef std::pair <pid_t, std::string> r_sample_pair_t;
typedef std::map <pid_t, r_sample_pair_t> r_sample_map_t;
typedef std::map <int, std::string> i_sample_map_t;

namespace pwr {
    /*
     * Container for system params extracted
     * from the 'sys_params_found.txt' file.
     */
    class SystemInfo {
        public:
            std::string m_hookLibraryVersion;
            std::string m_driverVersion;
            std::string m_wuwatchVersion;
            std::string m_appProfiled;
            std::string m_collectionTime;
            std::string m_wudumpVersion;
            pw_s32_t m_cpuCount;
            pw_s32_t m_coreCount;
            pw_s32_t m_moduleCount;
            pw_s32_t m_packageCount;
            pw_u32_t m_tscFreq;
            pw_u32_t m_busClockFreq;
            float m_busClockFreqMHz;
            pw_u32_t m_microPatchVer;
            pw_u32_t m_cStateMult;
            pw_u32_t m_perfBitsLow;
            pw_u32_t m_perfBitsHigh;
            pw_s32_t m_driverMajor;
            pw_s32_t m_driverMinor;
            pw_s32_t m_driverOther;
            pw_u32_t m_wasAnyThreadSet;
            pw_u32_t m_wasAutoDemoteEnabled;
            pw_u32_t m_collectionSwitches;
            pid_t m_appPID;
            std::map <pw_u32_t,pw_u32_t> m_targetRes;
            std::vector <pw_u32_t> m_availableFrequenciesKHz;
            std::vector <int> m_stateMapping;
            std::map <int, std::string> m_irqNamesMap;
            std::map <pid_t, std::string> m_pidNamesMap;
            std::map <int,int> m_htMap; // mapping of abstract thread <-> abstract core #
            std::map <int,int> m_abstractThreadMap; // mapping of abstract thread # to actual thread #: DUMMY MAPPING because abstract == actual
            std::map <int,int> m_abstractCoreMap; // mapping of abstract core # to actual core #
            std::map <int,int> m_abstractModMap; // mapping of abstract module # to actual module #
            std::map <int,int> m_abstractPkgMap; // mapping of abstract Package # to actual Package #: DUMMY MAPPING because abstract == actual
            pw_u64_t m_startTsc;
            pw_u64_t m_stopTsc;
            pw_u64_t m_startTimeval;
            std::string m_hostName;
            std::string m_osName;
            std::string m_osType;
            std::string m_osVersion;
            std::string m_cpuBrand;
            pw_u32_t m_cpuFamily;
            pw_u32_t m_cpuModel;
            pw_u32_t m_cpuStepping;
            pw_u64_t m_turboThreshold;
            std::string m_cpuTopology;
            pw_u32_t m_totalSamples;
            pw_u32_t m_droppedSamples;
            std::vector <std::string> m_descendentPids;
            /*
             * A list of lines in the 'sys_params' section.
             * Users may either access these or access individual
             * members of the 'WuData' class.
             */
            std::deque <std::string> m_lines;

            SystemInfo();
            ~SystemInfo();
    };
    /*
     * Repository for all wuwatch data. Includes helper functions for:
     * (1) Reading in wuwatch and hook library data.
     * (2) Parsing data and converting from HT-aware format to standard wudump format.
     * (3) Returning converted samples and backtrace information.
     */
    class WuData
    {
        private:
            /*
             * Were we initialized? Did we read a power trace file? etc.
             */
            bool m_init_complete;
            /*
             * The list of PWCollector_sample instances.
             */
            sample_vec_t m_samples;
            /*
             * Map to hold tid <-> backtrace information.
             * This info is generated by the hook library,
             * and by the kernel (for kernel call stacks).
             */
            // trace_pair_map_t m_trace_pair_map;
            /*
             * Convenience vector to hold dynamically allocated
             * (user-space) back traces. Storing those traces
             * here allows us to (quickly) track and deallocate
             * them when the time comes. We make this private because
             * applications should never need to access it directly. Instead, they
             * should access the 'm_trace_pair_map' above.
             */
            // trace_vec_t m_trace_vec;
            /*
             * Repository for system configuration information.
             */
            SystemInfo sysInfo;

        private:
            WuData();
            ~WuData();

            static WuData *s_data;

        public:
            /*
             * The public API.
             */
            /*
             * Return the globally persistent WuData instance.
             */
            static WuData *instance();
            /*
             * Destroy the globally persistent WuData instance.
             */
            void destroy();
            /*
             * Main entry point. Use this function to read wuwatch output and prime the
             * various data structures (list of samples, trace map, system information etc.)
             */
            int do_read_and_process(const std::string& file_name, bool should_calc_c1, bool is_from_axe=false, bool should_dump_orig=false, bool should_dump_sample_stats=false);
            /*
             * Main entry point. Use this function to process previously read samples.
             * Called from the power interface.
             */
            int do_process(std::list <PWCollector_sample_t>&, SystemInfo&);
            /*
             * Main entry point. use this function to process previously read samples.
             * Called from the power interface.
             */
            int do_process(const std::vector<char>& data, SystemInfo& systemInfo);
            /*
             * Accessor function: retrieve (previously read) system configuration information.
             * REQUIRES that 'do_read_and_process()' should have been called in the past.
             */
            const SystemInfo& getSystemInfo() const;
            /*
             * Accessor function: retrieve (previously read and parsed) power samples.
             * REQUIRES that 'do_read_and_process()' should have been called in the past.
             */
            const sample_vec_t& getSamples() const;
            /*
             * Accessor function: retrieve (previously read) user/kernel TID <-> backtrace information.
             * REQUIRES that 'do_read_and_process()' should have been called in the past.
             */
            // const trace_pair_map_t& get_tid_backtrace_map() const;
    };
} // pwr

std::ostream& operator<<(std::ostream& os, const PWCollector_sample_t& sample);

/* *********************************************************
 * Helper macros for the various 'SystemInfo' fields.
 * *********************************************************
 */
/*
 * The "MULTIPLIER" field from "arch_msr_config.txt"
 */
#define C_STATE_RES_COUNT_MULTIPLIER() (pwr::WuData::instance()->getSystemInfo().m_cStateMult)
/*
 * Was this a C-state DEMOTION?
 * "Demotion" ==> H/W grants SHALLOWER C-state than OS requst.
 */
#define WAS_C_STATE_DEMOTION(req, act) ( (act) < (req) )
/*
 * The number of online cpus.
 */
#define NUM_ONLINE_CPUS()  ( pwr::WuData::instance()->getSystemInfo().m_cpuCount )
/*
 * The number of online cores.
 * Always <= # online cpus.
 */
#define NUM_ONLINE_CORES() ( pwr::WuData::instance()->getSystemInfo().m_coreCount )
/*
 * The number of online modules.
 */
#define NUM_ONLINE_MODULES() ( pwr::WuData::instance()->getSystemInfo().m_moduleCount )
/*
 * The number of online packages.
 */
#define NUM_ONLINE_PACKAGES() ( pwr::WuData::instance()->getSystemInfo().m_packageCount )
/*
 * The number of logical processors per core.
 */
#define NUM_CPUS_PER_CORE() ( NUM_ONLINE_CPUS() / NUM_ONLINE_CORES() )
/*
 * Macro used to determine number of cores per package.
 */
#define NUM_CORES_PER_PACKAGE() ( NUM_ONLINE_CORES() / NUM_ONLINE_PACKAGES() )
/*
 * Macro used to retrieve package information, given core information.
 * Relies on integer division
 */
#define GET_PACKAGE_GIVEN_CORE(core) ( (core) / NUM_CORES_PER_PACKAGE() )
/*
 * Macro used to retrive a core's module ID.
 * Forced to '0', for now!
 */
#define GET_MODULE_GIVEN_CORE(core) (0)
/*
 * Macro used to determine if the underlying system
 * is Hyper Threaded. System is HT enabled if #cpus > # cores.
 */
#define WAS_SYSTEM_HYPER_THREADED() ( NUM_ONLINE_CPUS() > NUM_ONLINE_CORES() )
/*
 * Macro used to determine which CORE the given LCPU
 * maps to.
 */
#define GET_CORE_GIVEN_LCPU(lcpu) ( pwr::WuData::instance()->getSystemInfo().m_htMap.find((const int)lcpu)->second )
/*
 * Macro used to convert between 'mwait' hints and the
 * actual C-states.
 */
#define GET_C_STATE_GIVEN_TPS_HINT(hint) ( pwr::WuData::instance()->getSystemInfo().m_stateMapping[(hint)] )
/*
 * Helper macro to iterate over a range of cpus.
 */
#define for_each_online_cpu(cpu) for (cpu=0; cpu<NUM_ONLINE_CPUS(); ++cpu)
/*
 * Helper macro to iterate over a range of cores.
 */
#define for_each_online_core(core) for (core=0; core<NUM_ONLINE_CORES(); ++core)
/*
 * Helper macro to iterate over all logical processors in a given core.
 */
#define for_each_thread_in_core(coreid, threadid) for (threadid = NUM_CPUS_PER_CORE() * (coreid); threadid < ((coreid) + 1) * NUM_CPUS_PER_CORE(); ++threadid)
/*
 * Get lower 16 bits of any value. Useful for masking ops.
 */
#define GET_16_BIT_MASK(x) ( (x) & 0xffff )
/*
 * Combine the actual 'Cx' field and H/W 'cpuidx' values into a 32 bit value.
 * REQUIRES: Cx and cpuid are 16-bit values i.e. <= 2^16
 */
#define COMBINED_CX_CPUID(cx, cpuid) ( ( GET_16_BIT_MASK(cx) << 16 ) | GET_16_BIT_MASK(cpuid) )
/*
 * Extract the 'Cx' value from the combined value.
 */
#define GET_CX_FROM_COMBINED(combined) ( ( (combined) & (0xffff0000) ) >> 16 )
/*
 * Extract the wakeup event 'cpuidx' value from the combined value.
 */
#define GET_CPUID_FROM_COMBINED(combined) ( GET_16_BIT_MASK(combined) )
/*
 * Extract 'req_state' from 'sample->c_sample.prev_state'
 */
#define GET_REQ_FROM_PREV(prev_state) ( ( ( prev_state ) >> 8 ) & 0xff )
/*
 * Extract 'act_state' from 'sample->c_sample.prev_state'
 */
#define GET_ACT_FROM_PREV(prev_state) ( (prev_state) & 0xff )
/*
 * Bit values to add to each 'PWCollector_sample_t->cpuidx' value to indicate the type of the associated sample.
 * We set the two most significant bits per the following algo:
 * 00 ==> Core sample
 * 01 ==> Package sample
 * 10 ==> logical CPU sample
 * 11 ==> Module sample
 */
#define CORE_CPUIDX_MASK (0x00 << 30)
#define PACKAGE_CPUIDX_MASK (0x1 << 30)
#define LCPU_CPUIDX_MASK (0x2 << 30)
#define MODULE_CPUIDX_MASK (0x3 << 30)
/*
 * Helper macros for the above bit values.
 */
#define IS_PACKAGE_SAMPLE(cpuidx) ( ( (cpuidx) & PACKAGE_CPUIDX_MASK ) == PACKAGE_CPUIDX_MASK )
#define GET_SAMPLE_PACKAGE_ID(cpuidx) ( (cpuidx) & ~PACKAGE_CPUIDX_MASK )

#define IS_LCPU_SAMPLE(cpuidx) ( ( (cpuidx) & LCPU_CPUIDX_MASK ) == LCPU_CPUIDX_MASK )
#define GET_SAMPLE_LCPU_ID(cpuidx) ( (cpuidx) & ~LCPU_CPUIDX_MASK )

#define IS_MODULE_SAMPLE(cpuidx) ( ( (cpuidx) & MODULE_CPUIDX_MASK ) == MODULE_CPUIDX_MASK )
#define GET_SAMPLE_MODULE_ID(cpuidx) ( (cpuidx) & ~MODULE_CPUIDX_MASK )

#define IS_CORE_SAMPLE(cpuidx) !( IS_PACKAGE_SAMPLE(cpuidx) || IS_LCPU_SAMPLE(cpuidx) || IS_MODULE_SAMPLE(cpuidx) )
#define GET_SAMPLE_CORE_ID(cpuidx) ( cpuidx )

/*
 * Macros to set, test and reset whether a given 'C1' residency is actually
 * a 'C1i' residency. Valid for CLTP ONLY!
 */
#define SET_C1I_FLAG(res) ( (res) |= 0x8000000000000000ULL )
#define IS_C1I_FLAG_SET(res) ( (res) & 0x8000000000000000ULL )
#define RESET_C1I_FLAG(res) ( (res) &= 0x7FFFFFFFFFFFFFFFULL )

#endif // _WULIB_H_
