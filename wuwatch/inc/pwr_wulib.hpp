#ifndef _WULIB_H_
#define _WULIBH_H_


/*
 * Some useful typedefs.
 */
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
            uint32_t m_arch;
            uint32_t m_cpuCount;
            uint32_t m_coreCount;
            uint32_t m_tscFreq;
            uint32_t m_microPatchVer;
            uint32_t m_cStateMult;
            pid_t m_appPID;
            std::map <uint32_t,uint32_t> m_targetRes;
            std::vector<int> m_stateMapping;
            std::map <int, std::string> m_irqNamesMap;
            std::map <pid_t, std::string> m_pidNamesMap;
            std::map <int,int> m_htMap;
            uint64_t m_startTsc;
            uint64_t m_startTimeval;
            std::string m_hostName;
            std::string m_osName;
            std::string m_osType;
            std::string m_osVersion;
            std::string m_cpuBrand;
            uint32_t m_cpuFamily;
            uint32_t m_cpuModel;
            uint32_t m_cpuStepping;
            uint64_t m_turboThreshold;
            std::string m_cpuTopology;
            /*
             * A list of lines in the 'sys_params' section.
             * Users may either access these or access individual
             * members of the 'WuData' class.
             */
            std::deque <std::string> m_lines;

            friend class WuData;

        private:
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
            int do_read_and_process(const std::string& dir_name, const std::string& file_name);
            /*
             * Main entry point. Use this function to read wuwatch output and prime the
             * various data structures (list of samples, trace map, system information etc.)
             */
            int do_read_and_process(const std::string& file_name);
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
    };
} // pwr

void operator<<(std::ostream& os, const PWCollector_sample_t& sample);

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
 * Macro used to determine if the underlying system
 * is Hyper Threaded. System is HT enabled if #cpus > # cores.
 */
#define WAS_SYSTEM_HYPER_THREADED() ( NUM_ONLINE_CPUS() > NUM_ONLINE_CORES() )
/*
 * Macro used to determine which CORE the given LCPU
 * maps to.
 */
#define GET_CORE_GIVEN_LCPU(lcpu) ( pwr::WuData::instance()->getSystemInfo().m_htMap.find(lcpu)->second )
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


#endif // _WULIB_H_
