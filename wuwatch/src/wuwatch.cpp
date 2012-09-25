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
 * The wuwatch data collector.
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
#if !_ANDROID_
    #include <sys/un.h>
#endif
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <assert.h>
#include <sys/utsname.h>
#include <sys/syscall.h>
#include <ctype.h>

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

#include "uds.hpp" // for UDS server stuff.
#include "pw_ioctl.h" // IOCTL stuff.
#include "pw_arch.h" // Architecture info.
#include "defines.h"
#include "wuwatch_defines.h"
#include "wuwatch.h"

/*
 * Required syscall wrapper for manual insertion
 * of the power driver.
 */
#define load_module(...) syscall(__NR_init_module, __VA_ARGS__)

/* **************************************
 * Data structure definitions.
 * **************************************
 */
namespace pwr {
    struct barrier {
        bool wasInitialized;
        int numThreads;
        pthread_mutex_t lock;
        pthread_cond_t cond;

        int init(int num_threads);
        int wait();
    };

    struct StringUtils {
        static int getline(FILE *fp, std::string& line);
        static void trim(std::string& line);
    };
}

/* **************************************
 * Variable declarations/definitions.
 * **************************************
 */
/*
 * Barrier used to communicate between parent and reader threads.
 */
static pwr::barrier s_barrier;
/*
 * String representation of the various arch types.
 */
const char *Wuwatch::g_arch_type_names[NUMBER_OF_ARCHITECTURE_TYPES] = {"NHM", "SNB", "MFD"};
/*
 * A list of MSR addresses and target residencies. We differentiate between internal 
 * and external customers for these (external customers do NOT see a 'C5' on MFD).
 * ***************************************************************************************
 * CAUTION: ELEMENTS BELOW SHOULD BE SORTED IN SAME ORDER AS "arch_type_t" ENUM VALUES!!!
 * ***************************************************************************************
 */
#if IS_INTEL_INTERNAL
const int Wuwatch::s_coreResidencyMSRAddresses[][MAX_MSR_ADDRESSES] = {
    {0x30b, 0x30a, -1, 0x3fc, -1, -1, 0x3fd, 0x3fe, -1, -1}, /* NHM/WMR */
    {0x30b, 0x30a, -1, 0x3fc, -1, -1, 0x3fd, 0x3fe, -1, -1}, /* SNB */
    {0xe7, 0xe8, 0x3f8, -1, 0x3f9, 0x121, 0x3fa, -1, -1, -1}, /* ATM */
};
const int Wuwatch::s_target_residencies_us[][MAX_MSR_ADDRESSES] = {
    {-1, 6, -1, 80, -1, -1, 800, -1, -1, -1}, /* NHM/WMR */
    {-1, 1, -1, 211, -1, -1, 345, 345, -1, -1}, /* SNB */
    {-1, 4, 80, -1, 400, 400, 560, -1, -1, -1}, /* ATM */
};
#else // NON-INTEL
const int Wuwatch::s_coreResidencyMSRAddresses[][MAX_MSR_ADDRESSES] = {
    {0x30b, 0x30a, -1, 0x3fc, -1, -1, 0x3fd, 0x3fe, -1, -1}, /* NHM/WMR */
    {0x30b, 0x30a, -1, 0x3fc, -1, -1, 0x3fd, 0x3fe, -1, -1}, /* SNB */
    {0xe7, 0xe8, 0x3f8, -1, 0x3f9, -1, 0x3fa, -1, -1, -1}, /* ATM */
};
const int Wuwatch::s_target_residencies_us[][MAX_MSR_ADDRESSES] = {
    {-1, 6, -1, 80, -1, -1, 800, -1, -1, -1}, /* NHM/WMR */
    {-1, 1, -1, 211, -1, -1, 345, 345, -1, -1}, /* SNB */
    {-1, 4, 80, -1, 400, -1, 560, -1, -1, -1}, /* ATM */
};
#endif // IS_INTEL_INTERNAL
/*
 * A list of bus clock frequencies
 */
// const int Wuwatch::s_busClockFreqKHz[] = {133333, 100000, 100000};
const int Wuwatch::s_busClockFreqKHz[] = {133000, 100000, 100000};

/* **************************************
 * Function definitions.
 * **************************************
 */
int pwr::barrier::init(int num_threads)
{
    if (pthread_mutex_init(&lock, NULL) || pthread_cond_init(&cond, NULL)) {
        perror("pthread_mutex_init or pthread_cond_init error");
        return -ERROR;
    }
    numThreads = num_threads;
    wasInitialized = true;
    return SUCCESS;
};

int pwr::barrier::wait()
{
    if (!wasInitialized) {
        return -ERROR;
    }
    if (pthread_mutex_lock(&lock)) {
        perror("pthread_mutex_lock error");
        return -ERROR;
    }
    if (--numThreads <= 0) {
        db_fprintf(stderr, "Count = %d, unblocking!\n", numThreads);
        pthread_cond_broadcast(&cond);
    } else {
        db_fprintf(stderr, "Count = %d, blocking!\n", numThreads);
        pthread_cond_wait(&cond, &lock); // OK to wait just once!
        db_fprintf(stderr, "OK, unblocked\n");
    }
    pthread_mutex_unlock(&lock);
    return SUCCESS;
};

int pwr::StringUtils::getline(FILE *fp, std::string& line)
{
    char tmp_buf[1024];
    size_t tmp_size = 0;
    tmp_buf[0] = '\0';

    line.clear();

    if (feof(fp)) {
        return -1;
    }

    do {
        if (fgets(tmp_buf, sizeof(tmp_buf), fp) == NULL) {
            if (!feof(fp)) {
                return -1;
            }
            break;
        }
        line.append(tmp_buf);
        tmp_size = strlen(tmp_buf);
        /*
         * Sanities.
         */
        if (tmp_size < 1) {
            tmp_size = 1;
        } else if (tmp_size >= sizeof(tmp_buf)) {
            tmp_size = sizeof(tmp_buf) - 1;
        }
    } while (tmp_buf[tmp_size-1] != '\n');

    /*
     * Make sure we NULL terminate the line!
     */
    if (line.size() > 0) {
        line[line.size()-1] = '\0';
    }
    trim(line);

    return (int)line.size();
};

void pwr::StringUtils::trim(std::string& line)
{
    /*
     * Trim any remaining leading or trailing whitespace. We can't
     * use 'boost::trim()' here because of constraints on the power
     * library (must be statically compiled/linked for Android, e.g.).
     */
    if (line.size() == 0) {
        return;
    }
    std::string::size_type begin = line.find_first_not_of(" \t\n");
    std::string::size_type end = line.find_last_not_of(" \t\n");
    if (begin != std::string::npos) {
        line = line.substr(begin, (end-begin+1));
    }
};
/*
 * The wuwatch constructor.
 */
Wuwatch::Wuwatch()
{

    profiled_app_pid    = -1;
    m_max_num_cpus        = -1;
    m_dev_fd              = -1;
    fork_listenfd       = -1;
    m_cpu_topology_str  = "";
    uds_file_name       = "";
    m_output_fp         = NULL;
    child_pid           = -1;
    micro_patch_ver     = 0;
    tsc_freq_MHz        = 0;
    target_arch_type  = NHM;

    c_state_collection      = 0;
    p_state_collection      = 0;
    s_residency_collection  = 0;
    // s_state_collection      = 0;
    d_residency_collection  = 0;
    d_nc_state_collection   = 0;
    // d_sc_state_collection   = 0;
    w_state_collection      = 0;
    m_do_force_dd_load	            = 0;
    m_do_collect_kernel_backtrace   = 0;

    d_state_sample_interval_msecs   = 100;


    collection_time_msecs           = 0.0;
    system_collection_mode_secs     = 0;

    m_initialTSC = 0;
    m_finalTSC = 0;
    initialTimeval = 0;
    turboThreshold = 0;

    m_available_frequencies[0] = 0;

};
/*
 * The wuwatch destructor.
 */
Wuwatch::~Wuwatch() {
    if (m_dev_fd > 0) {
        close(m_dev_fd);
    }
};

/*
 * INTERNAL API:
 * Loads the power driver (basically, replicates 'insmod').
 * 
 * @returns: 0 on success, -1 on failure
 */
int Wuwatch::do_insmod_i(void)
{
    const char *dd_path_c_str = m_driver_path.c_str();
    char *buffer = NULL;
    struct stat st_buf;
    char options = '\0';
    /*
     * Basic algo:
     * read the driver file from disk into a buffer.
     * Then call the 'init_module(...)' syscall to
     * load it. Our power driver doesn't require any
     * insmod options, which is why we use a single
     * 'char' value.
     */
    int fd = open(dd_path_c_str, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "ERROR: could NOT open driver file: %s. Error string = %s\n", dd_path_c_str, strerror(errno));
        return -ERROR;
    }
    /*
     * We need to know the size of the driver before
     * we can allocate buffer space for it.
     */
    if (fstat(fd, &st_buf)) {
        perror("fstat error");
        close(fd);
        return -ERROR;
    }
    ssize_t size = st_buf.st_size;
    buffer = new char[size];
    /*
     * OK, now read the driver into our buffer.
     */
    ssize_t rd_size = read(fd, buffer, size);
    close(fd);
    /*
     * No fancy error corrections. Just DIE!
     */
    if (rd_size < size) {
        fprintf(stderr, "ERROR reading in power driver: %s --> %s\n", dd_path_c_str, strerror(errno));
        delete []buffer;
        return -ERROR;
    }

    /*
     * OK, we've read the power driver file into memory.
     * Now load it.
     */
    if (load_module(buffer, size, &options)) {
        switch (errno) {
            case EPERM:
                fprintf(stderr, "ERROR loading power driver: you do NOT have sufficient permissions to load a device driver! Consider re-running as root.\n");
                break;
            default:
                perror("load module error");
        }
        delete []buffer;
        return -ERROR;
    }

    delete []buffer;
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Open a connection to the power driver.
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::open_dd_i()
{
    /*
     * Open connection to device driver.
     */
    if (m_dev_fd > 0) {
        return SUCCESS;
    }
    while ( (m_dev_fd = open(DEVICE_FILE_NAME, 0)) < 0) {
        switch (errno) {
            case EPERM:
                fprintf(stderr, "ERROR: you do NOT have permission to access the APWR2 character device file: %s\n", DEVICE_FILE_NAME);
                return -ERROR;
            case ENOENT:
                /*
                 * We couldn't find the DD -- we'll try to do a manual
                 * 'insmod' here, but ONLY if the user has allowed
                 * us to do so (i.e. specified the "-f/--force-dd-load"
                 * option)
                 */
                if (m_do_force_dd_load) {
                    fprintf(stderr, "WARNING: device driver (apwr3) NOT loaded -- trying to manually insert it!\n");
                    if (do_insmod_i()) {
                        fprintf(stderr, "ERROR: could NOT manually load apwr2 device driver!\n");
                        return -ERROR;
                    }
                    fprintf(stderr, "Device driver loaded successfully!\n");
                } else {
                    fprintf(stderr, "ERROR: device driver (apwr3) NOT loaded!\n");
                    return -ERROR;
                }
                break;
            default:
                perror("ERROR on requested access to APWR2 character device file");
                return -ERROR;
        }
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Perform an IOCTL.
 *
 * @fd: device driver fd.
 * @ioctl_num: specific IOCTL to perform.
 * @data: (pointer-to) data to transmit/receive to/from driver.
 * @size: size of data pointed to by @data.
 * @is_in: true ==> input IOCTL, out ==> output IOCTL
 *
 * @returns: either the data itself (if "is_in" == true) or
 *           pointer to data returned by driver.
 */
void *Wuwatch::do_ioctl_i(int fd, int ioctl_num, void *data, int size, bool is_in)
{
    int i=0;
    struct PWCollector_ioctl_arg ioctl_arg;

    memset(&ioctl_arg, 0, sizeof(ioctl_arg));
    if (is_in) {
        ioctl_arg.in_len = size;
        ioctl_arg.in_arg = (char *)data;
    } else {
        ioctl_arg.out_len = size;
        ioctl_arg.out_arg = (char *)data;
    }

    if ( (i = ioctl(fd, ioctl_num, &ioctl_arg)) < 0) {
        perror("ioctl error");
        return NULL;
    }
    /*
     * Can't just return the "out_arg"
     * Because then nothing differentiates error
     * conditions from "is_in == true" (out_arg will be NULL
     * in this case, too!).
     */
    return is_in ? data : ioctl_arg.out_arg;
};

/*
 * INTERNAL API:
 * Start (or stop) a collection.
 * 
 * @fd: device driver fd.
 * @enable: true ==> START a collection, false ==> STOP a collection.
 */
void Wuwatch::do_ioctl_start_stop(int fd, bool enable)
{
    int i = -1;
    struct PWCollector_ioctl_arg ioctl_arg;
    int cmd = enable ? START : STOP;

#if DO_COUNT_DROPPED_SAMPLES
    {
        u64 array[2];
        struct PWCollector_ioctl_arg ioctl_Arg;

        ioctl_arg.in_len = sizeof(cmd);
        ioctl_arg.in_arg = (char *)&cmd;

        ioctl_arg.out_len = sizeof(array);
        ioctl_arg.out_arg = (char *)array;

        if (false) {
            fprintf(stderr, "%d\n", sizeof(array));
            assert(false);
        }

        if (ioctl(fd, PW_IOCTL_CMD, &ioctl_arg) < 0) {
            perror("ioctl error");
            return;
        }
        if (cmd == STOP && array[1] != 0) {
            fprintf(stderr, "\nWARNING: There were %llu samples dropped out of a total of %llu!\n\n", array[1], array[0]);
        }
    }
#else
    {
        do_ioctl_i(fd, PW_IOCTL_CMD, &cmd, sizeof(int), true); // "true" ==> INPUT param.
    }
#endif // DO_COUNT_DROPPED_SAMPLES

};

/*
 * INTERNAL API:
 * Get device driver VERSION info.
 *
 * @fd: device driver fd.
 * @ver_str: (reference to) the version string.
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::do_ioctl_driver_version_i(int fd, std::string& ver_str)
{
    PWCollector_version_info version;

    if (do_ioctl_i(fd, PW_IOCTL_VERSION, &version, sizeof(PWCollector_version_info), false) == NULL) // "false" ==> OUTPUT param.
        return -ERROR;

    db_fprintf(stderr, "DEVICE DRIVER VERSION = %d.%d.%d\n", version.version, version.interface, version.other);

    char version_string[1024];
    sprintf(version_string, "%d.%d.%d", version.version, version.interface, version.other);
    ver_str = std::string(version_string);
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Get microcode patch verion
 * (Useful for MFLD ONLY!)
 *
 * @fd: device driver fd.
 * @patch_ver: (reference to) the IAFW microcode patch version number.
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::do_ioctl_micro_patch(int fd, int& patch_ver)
{
    int tmp_patch_ver = -1;

    if (do_ioctl_i(fd, PW_IOCTL_MICRO_PATCH, &tmp_patch_ver, sizeof(tmp_patch_ver), false) == NULL) // "false" ==> OUTPUT param.
        return -ERROR;

    db_fprintf(stderr, "DRIVER RETURNS PATCH VERSION = %d\n", tmp_patch_ver);

    patch_ver = tmp_patch_ver;

    return SUCCESS;
}

static void cpuid(unsigned info, unsigned *eax, unsigned *ebx, unsigned *ecx, unsigned *edx)
{
    *eax = info;
    __asm volatile
    ("mov %%ebx, %%edi; cpuid; mov %%ebx, %%esi; mov %%edi, %%ebx;"
     :"+a" (*eax), "=S" (*ebx), "=c" (*ecx), "=d" (*edx) : :"edi");
}

/*
 * INTERNAL API:
 * Get CPU arch details. Adapted
 * from code taken from
 * 'turbostat'.
 *
 * @returns: an arch_type_t instance indicating
 * underlying arch.
 */
int Wuwatch::get_arch_type(arch_type_t& type)
{
    unsigned int eax, ebx, ecx, edx, max_level;
    char brand[16];
    unsigned int fms;

    eax = ebx = ecx = edx = 0;

    //asm("cpuid" : "=a" (max_level), "=b" (ebx), "=c" (ecx), "=d" (edx) : "a" (0));
    cpuid(0x0, &max_level, &ebx, &ecx, &edx);

    sprintf(cpu_brand, "%.4s%.4s%.4s", (char *)&ebx, (char *)&edx, (char *)&ecx);

    db_assert(!strncmp(cpu_brand, "GenuineIntel", 12), "CPUID: %s != GenuineIntel\n", cpu_brand);

    if (strncmp(cpu_brand, "GenuineIntel", 12)) {
        fprintf(stderr, "CPUID: %s != GenuineIntel\n", cpu_brand);
        wu_exit(-ERROR);
    }

    //asm("cpuid" : "=a" (fms), "=c" (ecx), "=d" (edx) : "a" (1) : "ebx");
    cpuid(0x1, &fms, &ebx, &ecx, &edx);
    cpu_family = (fms >> 8) & 0xf;
    cpu_model = (fms >> 4) & 0xf;
    cpu_stepping = fms & 0xf;
    if (cpu_family == 6 || cpu_family == 0xf)
        cpu_model += ((fms >> 16) & 0xf) << 4;

        db_fprintf(stderr, "CPUID %s %d levels family:model:stepping 0x%x:%x:%x (%d:%d:%d)\n",
                cpu_brand, max_level, cpu_family, cpu_model, cpu_stepping, cpu_family, cpu_model, cpu_stepping);

    if (cpu_family != 0x6) {
        db_fprintf(stderr, "ERROR: unsupported cpu family = 0x%x\n", cpu_family);
        return -ERROR;
    }
    if (cpu_model == 0x1a /*NHM */ || cpu_model == 0x2c /* WMR */) {
        type = NHM;
    } else if (cpu_model == 0x2a || cpu_model == 0x2d) {
        type = SNB;
    } else {
        type = MFD;
    }
    return SUCCESS;
};

/*
 * INTERNAL API:
 * Helper function to read
 * the TSC.
 *
 * @returns: the TSC value.
 */
unsigned long long Wuwatch::rdtsc(void)
{
    unsigned long long x;
    unsigned int a, d;

    __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));

    return ((unsigned long long)a) | (((unsigned long long)d) << 32);
};

/*
 * INTERNAL API:
 * Extract the TSC frequency.
 * Algo ref: 'Intel Processor Identification and the CPUID Instruction'
 * Application Note 485.
 * August, 2009.
 * ***************************************************************************************
 *
 * (1) Execute the CPUD instruction with an input value of EAX=0 and
 * ensure the vendor ID string is "GenuineIntel".
 * (2) Execute the CPUID instruction with EAX=1 to load the EDX register
 * with the feature flags.
 * (3) Ensure the TSC feature flag (EDX bit 4) is set. This indicates the
 * processor supports the Time-Stamp Counter and RTSC instruction.
 * (4) Read the TSC at the beginning of the reference period.
 * (5) Read the TS at the end of the reference period.
 * (6) Compute the TSC delta from the beginning and ending of the
 * reference period.
 * (7) Compute the actual frequency by dividing the TSC delta
 * by the reference period.
 *
 * Actual Freq = (Ending TSC value - Beginning TSC value) / reference period.
 *
 * ***************************************************************************************
 * Note that we do NOT do steps (1) -- (3).
 *
 * @returns: the frequency at which the TSC counts, in MHz.
 */
unsigned int Wuwatch::get_tsc_freq_MHz_i()
{
    unsigned long long tsc1, tsc2, tsc_delta;
    struct timeval tv;
    double elapsedTime = 0.0;
    unsigned int freq = 0;

    /*
     * Step (4)
     */
    {
        if (gettimeofday(&tv, NULL)) {
            perror("gettimeofday error");
            return 0;
        }
        elapsedTime = -(tv.tv_sec * 1e6 + tv.tv_usec);

        tsc1 = rdtsc();
    }
    sleep(1);
    /*
     * Step (5)
     */
    {
        if (gettimeofday(&tv, NULL)) {
            perror("gettimeofday error");
            return 0;
        }
        elapsedTime += (tv.tv_sec * 1e6 + tv.tv_usec);

        tsc2 = rdtsc();
    }

    /*
     * Step (6)
     */
    {
        tsc_delta = tsc2 - tsc1;
    }
    /*
     * Step (7)
     */
    {
        freq = (unsigned int)(1.0 * (tsc_delta / elapsedTime)); // check this??
    }

    return freq;
};

/*
 * INTERNAL API:
 * Send configuration information to the power driver.
 *
 * @fd: device driver fd.
 * @switches: an array of collection switches.
 * @num: the number of collection switches specified by @switches.
 * @interval: the D-state sampling interval.
 */
void Wuwatch::do_ioctl_config(int fd, int *switches, int num, u32 interval)
{
    int i=0;
    struct PWCollector_ioctl_arg ioctl_arg;
    struct PWCollector_config config;
    int len = sizeof(config);
    bool does_support_s_d_states = false;
    u32 busClockFreqKHz = 0x0;
    int residency_count_multiplier = 0;

    memset(&config, 0, len);

    for (i=0; i<num; ++i) {
        config.data |= (switches[i] << i);
    }

    config.d_state_sample_interval = interval;

    /*
     * Do arch-specific config stuff here.
     */
    {
        arch_type_t arch = target_arch_type;
        bool is_nhm_wmr_snb = false;
        const int *coreResAddrs = s_coreResidencyMSRAddresses[arch];
        busClockFreqKHz = s_busClockFreqKHz[arch];
        switch(arch) {
            case NHM:
            case SNB:
                residency_count_multiplier = 1;
                does_support_s_d_states = false;
                break;
            case MFD:
                residency_count_multiplier = tsc_freq_MHz;
                does_support_s_d_states = true;
                break;
            default:
                fprintf(stderr, "INVALID arch type %d!\n", arch);
                wu_exit(-ERROR);
        }

        /*
         * GU: moved the S,D state checks here to accomodate
         * future arch requirements.
         */
        if (does_support_s_d_states == false) {

            if (s_residency_collection) {
                fprintf(stderr, "Warning: -sr/--s-residency switch not supported on this platform. No sr data will be collected.\n");
                switches[PLATFORM_RESIDENCY] = 0;
            }

            if (d_residency_collection) {
                fprintf(stderr, "Warning: dres switch not supported on this platform. No dres data will be collected.\n");
                switches[DEVICE_SC_RESIDENCY] = 0;
            }

            if (d_nc_state_collection) {
                fprintf(stderr, "Warning: -dn/--nc-states switch not supported on this platform. No dn data will be collected.\n");
                switches[DEVICE_NC_STATE] = 0;
            }

#if 0
            if (d_sc_state_collection) {
                fprintf(stderr, "Warning: -ds/--sc-states switch not supported on this platform. No ds data will be collected.\n");
                switches[DEVICE_SC_STATE] = 0;
            }
#endif

#if 0
            if (s_state_collection) {
                fprintf(stderr, "Warning: s-state collection is NOT supported on this platform! No s-state data will be collected!\n");
                switches[PLATFORM_STATE] = 0;
            }
#endif

            if (w_state_collection) {
                fprintf(stderr, "Warning: wakelock switch is only supported on Android. No wakelock data will be collected.\n");
                switches[WAKELOCK_STATE] = 0;
            }

        }

        memcpy(&config.info.coreResidencyMSRAddresses, coreResAddrs, sizeof(int) * MAX_MSR_ADDRESSES); // dst, src

        db_fprintf(stderr, "DEBUG: <PWR> ARCH type = %s\n", g_arch_type_names[arch]);
        db_fprintf(stderr, "DEBUG: <PWR> busClockFreqKhz = %u\n", busClockFreqKHz);
        db_fprintf(stderr, "DEBUG: <PWR> residency_count_multiplier = %d\n", residency_count_multiplier);

        config.info.bus_clock_freq_khz = busClockFreqKHz;
        config.info.residency_count_multiplier = residency_count_multiplier;
    }

    do_ioctl_i(fd, PW_IOCTL_CONFIG, &config, len, true); // "true" ==> INPUT param.

    return;
};
/*
 * INTERNAL API:
 * Queries the power driver for a list of frequencies the CPUs may legally
 * execute at.
 *
 * @fd: the device driver fd.
 * @available_frequencies: an array of frequencies, to be populated by the driver. It is
 * ASSUMED that this array is large enough for our purposes.
 *
 * @returns: 0 on success, -1 on error.
 */
int Wuwatch::do_ioctl_available_frequencies(int fd, u32 *available_frequencies)
{
    struct PWCollector_available_frequencies freqs;

    if (do_ioctl_i(fd, PW_IOCTL_AVAILABLE_FREQUENCIES, &freqs, sizeof(freqs), false) == NULL) { // "false" ==> OUTPUT param.
        db_abort("ERROR: could NOT retrieve available frequencies!\n");
        return -ERROR;
    }
    /*
     * Sanity test!
     */
    // assert(freqs.num_freqs <= PW_MAX_NUM_AVAILABLE_FREQUENCIES);
    if (freqs.num_freqs >= PW_MAX_NUM_AVAILABLE_FREQUENCIES) {
        db_fprintf(stderr, "ERROR: num freqs = %d\n", (int)freqs.num_freqs);
        return -ERROR;
    }

    db_fprintf(stderr, "Available frequencies...\n");
    for (int i=0; i<freqs.num_freqs; ++i) {
        db_fprintf(stderr, "%u\t", freqs.frequencies[i]);
    }
    db_fprintf(stderr, "\n");
    /*
     * Copy frequencies into output array.
     */
    assert(freqs.num_freqs <= 16);
    memcpy(available_frequencies, freqs.frequencies, sizeof(u32) * freqs.num_freqs);
    available_frequencies[freqs.num_freqs] = 0; // last entry MUST be ZERO!!!
    return SUCCESS;
};

/*
 * IOCTL to get TURBO threshold frequency.
 * @fd: the DD fd.
 * @turbo_thresh: the TURBO threshold frequency, in KHz.
 */
int Wuwatch::do_ioctl_get_turbo_threshold(int fd, u32 *turbo_thresh) {

    PWCollector_turbo_threshold thresh;
    memset(&thresh, 0, sizeof(thresh));
    if(do_ioctl_i(fd, PW_IOCTL_TURBO_THRESHOLD, &thresh, sizeof(PWCollector_turbo_threshold), false) == NULL)
        return -ERROR;
    *turbo_thresh = thresh.threshold_frequency;
    return SUCCESS;
};

int Wuwatch::do_ioctl_check_platform_i(int fd, u64& supported_features) {
    PWCollector_check_platform platform;
    memset(&platform, 0, sizeof(platform));
    if (do_ioctl_i(fd, PW_IOCTL_CHECK_PLATFORM, &platform, sizeof(PWCollector_check_platform), false) == NULL) { // "false" ==> OUTPUT param.
        return -ERROR;
    }
    supported_features = platform.supported_features;
    return SUCCESS;
};
/*
 * INTERNAL API:
 * Static wrapper around the reader thread function. Required because C++ doesn't play
 * nicely with the PTHREAD API.
 *
 * @arg: args to send to the reader thread.
 *
 * @returns: the reader thread return value (usually NULL).
 */
void *Wuwatch::reader_thread(void *arg)
{
    reader_thread_arg *reader_arg = (reader_thread_arg *)arg;
    Wuwatch *who = reader_arg->who;
    args_t *args = reader_arg->args;

    delete reader_arg;

    who->reader_thread_i(args);
     
    return NULL;
};
/*
 * INTERNAL API:
 * A thread to read data from the power driver.
 *
 * @args: an instance of 'args_t', containing various information required
 * for the thread to read information from the power driver, including
 * the device driver fd.
 *
 * @returns: don't care info (never queried).
 */
void *Wuwatch::reader_thread_i(args_t *args)
{
    int i=0, num=0;
    int fd = args->m_dev_fd;
    FILE *fp = args->m_output_fp;
    int array_size = NUM_SAMPLES_PER_SEG;
    // int array_size = 1024;
    PWCollector_sample_t *samples = new PWCollector_sample_t[array_size];
    int read_size = array_size * sizeof(PWCollector_sample_t);

    delete args;

    /*
    i = pthread_barrier_wait(&reader_barrier);
    if (i && i != PTHREAD_BARRIER_SERIAL_THREAD) {
        perror("pthread_barrier_wait error");
        delete []samples;
        wu_exit(-ERROR);
    }
    */
    if (s_barrier.wait()) {
        fprintf(stderr, "barrier_wait error in reader thread!\n");
        delete []samples;
        wu_exit(-ERROR);
    }

    do {
        i = read(fd, (void *)samples, read_size);
        num = i / sizeof(PWCollector_sample_t);

        db_fprintf(stderr, "DEBUG: <PWR> device driver returned %d bytes (sizeof = %d, num = %d)\n", i, sizeof(PWCollector_sample_t), num);

        if (i > 0) {
            size_t tmp = 0;
            // assert(num <= array_size); // Not strictly required!
            if (num >= array_size) {
                db_fprintf(stderr, "ERROR: num = %d\n", (int)num);
                delete []samples;
                wu_exit(-ERROR);
            }
            if ( (tmp = fwrite(samples, sizeof(PWCollector_sample_t), num, fp)) < num) {
                perror("fwrite error");
                delete []samples;
                wu_exit(-ERROR);
            }
        }
    } while (i > 0);

    delete []samples;

//    pthread_exit(PTHREAD_CANCELED);
    pthread_exit(0);

    return NULL;
}
/*
 * INTERNAL API:
 * Parses the "/proc/cpuinfo" file to retrieve the CPU topology of
 * the host machine.
 *
 * @returns: a string representing the topology of the host machine.
 * Format of this string is:
 * logical proc # <space> physical id <space> siblings <space> core id <space> cores <space>"
 */
std::string Wuwatch::get_cpu_topology_i(void)
{
    std::ostringstream topology;
    // char *line = NULL;
    std::string str_line;
    int proc = -1, phys_id = -1, core_id = -1, core_no, sibling;
    int retVal = SUCCESS;

    std::map <int, int_vec_t> tmp_map;

    m_max_num_cpus = 0;

    FILE *fp = fopen("/proc/cpuinfo", "r");
    if (!fp) {
        fprintf(stderr, "fopen error for /proc/cpuinfo: %s\n", strerror(errno));
        retVal = -ERROR;
    } else {
        size_t len = 0;
        ssize_t read = 0;

        while ((read = pwr::StringUtils::getline(fp, str_line)) != -1) {
            if (read < 3) {
                continue;
            }
            // line[read-1] = '\0';
            const char *line = str_line.c_str();
            /*
             * Format is:
             * proc # <space> physical id <space> siblings <space> core id <space> cores <space>"
             */
            if (sscanf(line, "processor       : %d", &proc)) {
                topology << proc << " ";
                ++m_max_num_cpus;
                continue;
            }
            if (sscanf(line, "physical id     : %d", &phys_id)) {
                topology << phys_id << " "; 
                continue;
            }
            if (sscanf(line, "siblings        : %d", &sibling)) {
                topology << sibling << " "; 
                continue;
            }
            if (sscanf(line, "core id         : %d", &core_id)) {
                topology << core_id << " ";
                assert(proc >= 0 && phys_id >= 0);
                tmp_map[(phys_id << 16 | core_id)].push_back(proc);
                proc = phys_id = core_id = -1;
                continue;
            }
            if (sscanf(line, "cpu cores       : %d", &core_no)) {
                topology << core_no << " "; 
                continue;
            }
        }
        fclose(fp);
    }
    if (proc >= 0) {
        /*
         * Bug on Lexington PR1 systems -- "/proc/cpuinfo" contains only "processor" field
         * and none of the other fields (i.e. no "physical id", "siblings" etc.).
         * Special case that here.
         */
        phys_id = 0; sibling = 0; core_id = 0; core_no = 0;
        topology << phys_id << " " << sibling << " " << core_id << " " << core_no << " ";
    }

    return topology.str();
};
/*
 * (Thin) wrapper around the wuwatch SIGINT signal handler.
 */
void sigint_handler_wrapper(int signum)
{
    g_wuwatchObj->sigint_handler(signum);
};
/*
 * Handle SIGINT (^-C).
 * Note: this supersedes the handler defined
 * in 'hook_lib' because this is registered AFTER
 * that one. Effectively, then, while programs-of-interest
 * will have SIGINT trapped by the 'hook_lib', 'wuwatch'
 * will trap (and handle) SIGINT on its own.
 */
void Wuwatch::sigint_handler(int signum)
{
    fprintf(stderr, "wuwatch SIGINT received!\n");
    /*
     * We *MIGHT* have some descendent pids.
     * Tell them to quit.
     * N.B.: It's OK if we never had any descendents
     * (e.g. if we were run in "system mode").
     */
    send_quit_to_all_i(offsets);
};
/*
 * INTERNAL API:
 * Start/Stop the timer counting how long the collection takes.
 *
 * @elapsedTime: (Pointer to) elapsed time.
 * @is_start: "true" ==> collection START, "stop" ==> collection STOP.
 */
void Wuwatch::start_stop_timer(double *elapsedTime, bool is_start)
{
    struct timeval tv;

    if (gettimeofday(&tv, NULL)) {
        perror("gettimeofday error");
        wu_exit(-ERROR);
    }
    if (is_start)
        *elapsedTime = -(tv.tv_sec * 1e6 + tv.tv_usec); // in usecs
    else
        *elapsedTime += (tv.tv_sec * 1e6 + tv.tv_usec); // in usecs
}

/*
 * INTERNAL API:
 * Helper function used by the "get_initial_proc_map_i()" function 
 * to troll the "/proc" directory and retrieve PID <-> PROC name mappings
 * at the start of a collection.
 * *********************************************
 * ATTN: NOT REENTRANT!
 * (Uses 'readdir')
 * *********************************************
 *
 * @dir: pointer to the dir structure initialized by "get_initial_proc_map_i()"
 * @pid_name: (reference to) the Proc name for the next PID in the directory 
 * specified by @dir
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::get_next_pid_i(DIR *dir, std::string& pid_name)
{
    static struct dirent *ent = NULL;
    /*
     * Basic algo -- iterate through the
     * dir until we get a directory corresponding
     * to a PID. Return its name.
     */
    for (;;) {
        ent = readdir(dir);
        if (!ent || !ent->d_name) {
            return -ERROR;
        }
        const char *dname = ent->d_name;
        if (*dname > '0' && *dname <= '9') { // we assume no malformed names!
            pid_name = std::string(dname);
            return SUCCESS;
        }
    }
    return -ERROR;
}
/*
 * INTERNAL API:
 * Helper function to read a line from a given file.
 *
 * @path: the file to read from.
 * @line: (reference to) the line that's read from @path.
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::readline(const std::string& path, std::string& line)
{
    int retVal = SUCCESS;

    std::ifstream in_stream(path.c_str());

    if (!in_stream.is_open()) {
        db_fprintf(stderr, "Error: could NOT open file %s: %s\n", path.c_str(), strerror(errno));
        return -ERROR;
    }

    if (!in_stream.eof())
        std::getline(in_stream, line);
    else {
        db_fprintf(stderr, "MALFORMED comm file: path = %s!\n", path.c_str());
        retVal = -ERROR;
    }

    in_stream.close();

    return retVal;
}
/*
 * INTERNAL API:
 * Helper function to extract a process name from the /proc/XXX/stat file.
 *
 * @line: the line to parse.
 * @proc_name: the extracted process name.
 *
 * @returns: 0 on success, -1 on error.
 */
int Wuwatch::parse_stat_line_i(const std::string& line, std::string& proc_name)
{
    int from = line.find('('), to = line.rfind(')');
    if (from == std::string::npos || to == std::string::npos)
        return -ERROR;
    ++from;
    proc_name = line.substr(from, (to-from));
    return SUCCESS;
}
/*
 * INTERNAL API:
 * Helper function to extract task data from the "/proc" file system.
 *
 * @pid_name: the process name (returned from a previous call to @get_next_pid_i)
 * @tid_names: the list of tasks belonging to @pid_name
 * @pid_map: helper map to determine if we've visited @pid_name before.
 * @dir_path: path to the proc fs (usually "/proc").
 * @proc_samples: the vector of PWCollector_sample_t instances containing PID <-> Proc name
 * mappings for all tasks present when the collection is started.
 *
 * @returns: 0 on success, -1 on error.
 */
int Wuwatch::extract_pid_tid_name_i(const std::string& pid_name, const str_vec_t& tid_names, str_map_t& pid_map, const char *dir_path, std::vector<PWCollector_sample_t>& proc_samples)
{
    std::string path = dir_path;
    pid_t pid = atoi(pid_name.c_str());
    std::string proc_name;
    r_sample_t rs;
    PWCollector_sample_t sample;

    /*
     * Do NOT do anything if we've seen this
     * 'pid_name' before.
     */
    if (pid_map[pid_name] == true) {
        return SUCCESS;
    }

    pid_map[pid_name] = true;

    /*
     * Some flavours of Linux don't seem to store
     * a "/proc/XXX/comm" file -- use the 'stat'
     * file instead.
     */
    path += "/" + pid_name + "/stat";

    {
        std::string line;
        if (readline(path, line))
            return -ERROR;
        if (parse_stat_line_i(line, proc_name)) {
            db_fprintf(stderr, "ERROR: MALFORMED stat file %s\n", path.c_str());
            return -ERROR;
        }
    }
    int len = proc_name.length();

    if (len >= PW_MAX_PROC_NAME_SIZE)
        len = PW_MAX_PROC_NAME_SIZE - 1;

    const char *c_proc_name = proc_name.c_str();

    /*
     * These fields are common to ALL (initial)
     * PROC_MAP samples.
     */
    sample.sample_len = sample.cpuidx = 0;
    sample.tsc = rdtsc();
    sample.sample_type = PROC_MAP;

    for (str_vec_t::const_iterator iter = tid_names.begin(); iter != tid_names.end(); ++iter) {
        rs.type = PW_PROC_FORK;
        rs.tid = atoi(iter->c_str()); rs.pid = pid;
        memset(rs.proc_name, 0, sizeof(rs.proc_name));
        memcpy(rs.proc_name, c_proc_name, len);
        sample.r_sample = rs;
        proc_samples.push_back(sample);
    }

    return SUCCESS;
}
/*
 * INTERNAL API:
 * Return a list of tasks for a given process.
 *
 * @pid_name: the process name
 * @tasks: the (retrieved) list of tasks for @pid_name
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::get_all_tasks_for_proc_i(std::string pid_name, str_vec_t& tasks)
{
    std::string task_file_name = "/proc/" + pid_name + "/task";
    struct dirent *ent = NULL;
    DIR *dir = opendir(task_file_name.c_str());
    if (!dir) {
        db_fprintf(stderr, "ERROR opening dir %s: %s\n", task_file_name.c_str(), strerror(errno));
        return -ERROR;
    }
    for (ent = readdir(dir); ent; ent = readdir(dir)) {
        if (!strcmp(ent->d_name, ".") || !strcmp(ent->d_name, ".."))
            continue;
        tasks.push_back(ent->d_name);
    }
    closedir(dir);

    return SUCCESS;
}
/*
 * INTERNAL API:
 * Parse the "/proc" psuedo-filesystem
 * to extract PID <-> PROC name
 * mappings.
 *
 * @fp: the output file pointer
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::get_initial_proc_map_i(FILE *fp)
{
    DIR *proc_dir = opendir(PROC_DIR_NAME);
    str_map_t pid_map;
    std::vector<PWCollector_sample_t> proc_samples;
    int num_samples = 0;
    if (proc_dir) {
        /*
         * Before we troll the "/proc" directory, make sure we add an entry
         * for the "swapper" process (which isn't in /proc").
         */
        {
            PWCollector_sample_t sample;
            memset(&sample, 0, sizeof(sample));
            r_sample_t *rs = &sample.r_sample;
            /*
             * These fields are common to ALL (initial)
             * PROC_MAP samples.
             */
            sample.sample_len = sample.cpuidx = 0;
            sample.tsc = rdtsc();
            sample.sample_type = PROC_MAP;

            rs->type = PW_PROC_FORK;
            rs->tid = 0; rs->pid = 0;
            memcpy(rs->proc_name, "swapper", strlen("swapper"));
            proc_samples.push_back(sample);
        }
        std::string pid_name;
        for (int i = get_next_pid_i(proc_dir, pid_name); i == SUCCESS; i = get_next_pid_i(proc_dir, pid_name)) {
            pid_name_pair_t pair;
            r_sample_t rs;
            str_vec_t tasks;
            get_all_tasks_for_proc_i(pid_name, tasks);
            extract_pid_tid_name_i(pid_name, tasks, pid_map, PROC_DIR_NAME, proc_samples);
        }
        closedir(proc_dir);
        
        /*
         * Dump ALL samples at the same time...
         */
        if ( (num_samples = proc_samples.size()) > 0) {
            if (fwrite(&proc_samples[0], sizeof(PWCollector_sample_t), num_samples, fp) < num_samples) {
                perror("fwrite error");
                wu_exit(-ERROR);
            }
            db_fprintf(stderr, "OK: wrote %d samples!\n", num_samples);
        }
    } else {
        char error_msg[1024];
        sprintf(error_msg, "opendir error in wuwatch::cleanup_collection; could not open \"%s\" directory", PROC_DIR_NAME);
        db_fprintf(stderr, "%s: %s\n", error_msg, strerror(errno));
        return -ERROR;
    }
    return SUCCESS;
}

/*
 * EXTERNAL API:
 * Perform wuwatch-specific setup.
 */
void Wuwatch::setup_collection()
{
    /*
     * Get # of processors in system.
     * UPDATE: 'sysconf()' silently fails in native Android builds. Use CPU
     * topology information instead.
     */
    // m_max_num_cpus = sysconf(_SC_NPROCESSORS_CONF);
    m_cpu_topology_str = get_cpu_topology_i();

    signal(SIGINT, sigint_handler_wrapper);

    db_fprintf(stderr, "# cpus = %d\n", m_max_num_cpus);


    /*
     * Make sure we know where to write our results!
     */
    assert(m_output_file_name.size());

    /*
     * Get current arch type.
     */
    {
        if (get_arch_type(target_arch_type)) {
            db_fprintf(stderr, "ERROR retrieving arch type!\n");
            wu_exit(-ERROR);
        }
        db_fprintf(stderr, "TARGET ARCH TYPE = %d\n", target_arch_type);
    }

    /*
     * Make sure we have a connection to the DD!
     */
    if (open_dd_i()) {
        db_fprintf(stderr, "ERROR: could NOT open connection to the device driver!\n");
        wu_exit(-ERROR);
    }

    /*
     * Get driver version information.
     */
    {
        if (do_ioctl_driver_version_i(m_dev_fd, m_driver_version_string)) {
            db_fprintf(stderr, "ERROR: could NOT retrieve DRIVER VERSION information!\n");
            wu_exit(-ERROR);
        }
        db_fprintf(stderr, "DRIVER_VERSION: %s\n", m_driver_version_string.c_str());
    }
    /*
     * Get microcode patch version
     * (Only useful for MFLD).
     */
    if (target_arch_type == MFD) {
        int patch_ver = -1;
        if (do_ioctl_micro_patch(m_dev_fd, patch_ver)) {
            db_fprintf(stderr, "ERROR: could NOT retrieve microcode patch version!\n");
            wu_exit(-ERROR);
        }
        micro_patch_ver = patch_ver;
        db_fprintf(stderr, "PATCH VERSION = %d\n", micro_patch_ver);
    }

    /*
     * Open handle to output file(s). We have a single
     * file that contains driver output AND system
     * configuration information.
     */
    if ( (m_output_fp = fopen(m_output_file_name.c_str(), "wb")) == NULL) {
        fprintf(stderr, "ERROR creating wuwatch output file %s: %s\n", m_output_file_name.c_str(), strerror(errno));
        wu_exit(-ERROR);
    }
    /*
     * We leave some space at the HEAD of the output
     * file to record the ENDING OFFSET of driver
     * data. This allows wudump to 'fseek(...)'
     * immediately to the start of the system parameters
     * section and allows us to use a single output
     * file for both sets of data.
     */
    if (fseek(m_output_fp, sizeof(u64), SEEK_SET)) {
        perror("fseek error");
        wu_exit(-ERROR);
    }

    /*
     * Get the TSC frequency
     */
    {
        tsc_freq_MHz = get_tsc_freq_MHz_i();
    }

    /*
     * Set CONFIG switches -- we're only interested
     * in C-states and P-states, for now.
     * UPDATE: and in kernel call stacks.
     */
    {
        int switches[MAX_POWER_DATA_MASK];

        memset(switches, 0, MAX_POWER_DATA_MASK * sizeof(int));

        if (c_state_collection) {
            switches[SLEEP] = 1; /* Use TPS tracepoint */
            switches[POWER_C_STATE] = 1; /* GET C state samples */
            if (m_do_collect_kernel_backtrace) {
                switches[KTIMER] = 1; /* GET K-SAMPLES */
            }
        }

        if (p_state_collection) {
            switches[FREQ] = 1; /* GET P state samples */
        }

        if (s_residency_collection) {
            /*
             * REQUIRES TPS.
             */
            switches[SLEEP] = 1; /* Use TPS tracepoint */
            switches[PLATFORM_RESIDENCY] = 1; /* GET S state residency counter samples */
        }

        if (d_residency_collection) {
            /*
             * REQUIRES TPS.
             */
            switches[SLEEP] = 1; /* Use TPS tracepoint */
            switches[DEVICE_SC_RESIDENCY] = 1; /* GET D state residency counter samples in south complex */
        }

        if (d_nc_state_collection) {
            /*
             * REQUIRES TPS.
             */
            switches[SLEEP] = 1; /* Use TPS tracepoint */
            switches[DEVICE_NC_STATE] = 1; /* GET D state state samples in north complex */
        }

        /*
         * South complex D-state sample collection is intentionally disabled
         * because we can obtain the same information from South complex
         * D-state residency counters more accurately.
         * Therefore, use D-state residency collection which provides 
         * more useful information because it counts how much time is spent
         * for each state. 
         */
#if 0
        if (d_sc_state_collection) {
            /*
             * REQUIRES TPS.
             */
            switches[SLEEP] = 1; /* Use TPS tracepoint */
            switches[DEVICE_SC_STATE] = 1; /* GET D state state samples in south complex */
        }
#endif
        /*
         * S-state sample collection is intentionally disabled
         * because it will be always S0 state because the collection
         * will always work when CPU is running.
         * Instead, use S-state residency collection which provides 
         * more useful information because it counts how much time is spent
         * for each S states.
         */
#if 0
        if (s_state_collection) {
            /*
             * REQUIRES TPS.
             */
            switches[SLEEP] = 1; /* Use TPS tracepoint */
            switches[PLATFORM_STATE] = 1; /* GET S state samples */
        }
#endif

        if (w_state_collection) {
            /*
             * First, make sure the wakelock patch has been applied.
             */
            u64 supported_features = 0;
            if (do_ioctl_check_platform_i(m_dev_fd, supported_features)) {
                /*
                 * For whatever reason, the IOCTL didn't succeed. We conservatively assume
                 * that wakelocks are NOT supported.
                 */
                fprintf(stderr, "ERROR: kernel check for wakelock tracking returns error; wakelock data will NOT be collected!\n");
                switches[WAKELOCK_STATE] = 0;
            }
            if (supported_features & PW_KERNEL_SUPPORTS_WAKELOCK_PATCH) {
                switches[WAKELOCK_STATE] = 1; /* GET Wakelock samples */
            } else {
                fprintf(stderr, "Warning: The OS is NOT configured to track wakelocks; wakelock data will NOT be collected!\n");
                switches[WAKELOCK_STATE] = 0;
            }
        }

        do_ioctl_config(m_dev_fd, switches, MAX_POWER_DATA_MASK, d_state_sample_interval_msecs);
    }

    /*
     * Get list of "available frequencies".
     * Should only be called AFTER config!
     */
    {
        do_ioctl_available_frequencies(m_dev_fd, m_available_frequencies);
    }

    /*
     * Get the turbo threshold
     */
    {
        do_ioctl_get_turbo_threshold(m_dev_fd, &turboThreshold);
    }


    /*
     * Init the barrier -- ensures the thread doesn't
     * execute until we want/need it to.
     */
    /*
    if (pthread_barrier_init(&reader_barrier, NULL, 2)) {
        perror("pthread_barrier_init error");
        wu_exit(-ERROR);
    }
    */
    if (s_barrier.init(2)) {
        fprintf(stderr, "barrier_init error");
        wu_exit(-ERROR);
    }

    /*
     * Create the reader thread.
     */
    if (pthread_create(&reader_tid, NULL, &Wuwatch::reader_thread, new reader_thread_arg(this, new args(m_dev_fd, m_output_fp)))) {
        perror("pthread_create error");
        wu_exit(-ERROR);
    }

};

/*
 * INTERNAL API:
 * Extract information on ACPI <--> C-state mappings
 * i.e. mapping between C-states as described by the ACPI system and
 * those described by the OS.
 *
 * @mapping: the list of mappings populated by this function. Mappings
 * are in the form ACPI[acpi_num] = OS_mapping.
 */
void Wuwatch::extract_acpi_c_state_mappings(acpi_mapping_t& mapping)
{
    switch (target_arch_type) {
        case NHM:
            mapping[0] = "C0";
            mapping[1] = "NHM-C1";
            mapping[2] = "NHM-C3";
            mapping[3] = "NHM-C6";
            break;
        case SNB:
            mapping[0] = "C0";
            mapping[1] = "SNB-C1";
            mapping[2] = "SNB-C3";
            mapping[3] = "SNB-C6";
            mapping[4] = "SNB-C7";
            break;
        case MFD:
            mapping[0] = "C0";
            mapping[1] = "ATM-C1";
            mapping[2] = "ATM-C2";
            mapping[3] = "ATM-C4";
            mapping[4] = "ATM-C6";
            mapping[5] = "ATM-C6"; /* ATM-S0i1 */
            mapping[6] = "ATM-C6"; /* ATM-LpAudio */
            mapping[7] = "ATM-C6"; /* ATM-S0i3 */
            // mapping[8] = "ATM-C6";
            break;
        default:
            db_fprintf(stderr, "ERROR: could NOT extract ACPI <-> C-STATE mappings for unknown arch type = %d\n", target_arch_type);
    }
}

/*
 * EXTERNAL API:
 * Perform wuwatch-specific cleanup.
 */
void Wuwatch::cleanup_collection(bool should_write_results)
{
    db_fprintf(stderr, "CLEANUP_COLLECTION!\n");

    if (pthread_join(reader_tid, NULL)) {
        perror("pthread_join error");
        wu_exit(-ERROR);
    }

    if (should_write_results) {

        /*
         * We need to let 'wudump' know where we've written
         * the system config info ('wudump' needs to read that
         * BEFORE reading driver output). We do that by storing
         * its offset at the HEAD of the (combined) output file.
         */
        u64 curr_pos = ftell(m_output_fp);
        rewind(m_output_fp); // == fseek(fp, 0L, SEEK_SET);
        /*
         * Write sys params info offset...
         */
        if (fwrite(&curr_pos, sizeof(curr_pos), 1, m_output_fp) != 1) {
            perror("fwrite error");
            wu_exit(-ERROR);
        }
        /*
         * ...and then go back to the end of the file
         * (so that we can start writing the actual
         * system configuration).
         */
        if (fseek(m_output_fp, 0, SEEK_END)) {
            perror("fseek error");
            wu_exit(-ERROR);
        }
        db_fprintf(stderr, "OK: wrote offset = %llu\n", curr_pos);
        /*
         * Sanity test!
         */
        if (ftell(m_output_fp) != curr_pos) {
            fprintf(stderr, "ftel = %u, curr = %llu\n", ftell(m_output_fp), curr_pos);
        }
        assert(ftell(m_output_fp) == curr_pos);

        db_fprintf(stderr, "CLEANING up collection!\n");

        /*
         * Write all of the sys params we discovered
         * to the 'sys_params_found' file.
         */
        {
            // (0) HEADER string
            fprintf(m_output_fp, "---SYS_PARAMS_BEGIN---\n");
            // (1) DRIVER version info
            fprintf(m_output_fp, "Driver Version = %s\n", m_driver_version_string.c_str());
            // (2) WUWATCH version info
            fprintf(m_output_fp, "Wuwatch Version = %d.%d.%d\n", WUWATCH_VERSION_VERSION, WUWATCH_VERSION_INTERFACE, WUWATCH_VERSION_OTHER);
            // (3) Collection time -- UPDATE: printing SECONDS (precision == 2 SIGNIFICANT DIGITS)
            fprintf(m_output_fp, "Total Collection Time = %.2f (secs)\n", collection_time_msecs / 1e3);
            fprintf(m_output_fp, "Start TSC = %llu\n", m_initialTSC);
            fprintf(m_output_fp, "Start Timeval = %llu\n", initialTimeval);
            fprintf(m_output_fp, "Stop TSC = %llu\n", m_finalTSC);

            // System information
            char hostname[128];
            if (gethostname(hostname, 128) != -1)
            {
                fprintf(m_output_fp, "Host Name = %s\n", hostname);
            } else {
                fprintf(m_output_fp, "Host Name = UNKNOWN\n");
            }
            if (uname(&un) == 0)
            {
                fprintf(m_output_fp, "OS Name = %s\n", un.sysname);
                fprintf(m_output_fp, "OS Type = %s\n", un.machine);
                fprintf(m_output_fp, "OS Version = %s\n", un.release);
            } else {
                fprintf(m_output_fp, "OS Name = UNKNOWN\n");
                fprintf(m_output_fp, "OS Type = UNKNOWN\n");
                fprintf(m_output_fp, "OS Version = UNKNOWN\n");
            }

            // Hardware configuration
            fprintf(m_output_fp, "CPU Brand = %s\n", cpu_brand);
            fprintf(m_output_fp, "CPU Family = %u\n", cpu_family);
            fprintf(m_output_fp, "CPU Model = %u\n", cpu_model);
            fprintf(m_output_fp, "CPU Stepping = %u\n", cpu_stepping);
            fprintf(m_output_fp, "Turbo Threshold = %llu\n", (unsigned long long)turboThreshold * 1000);
            fprintf(m_output_fp, "Num CPUs = %d\n", m_max_num_cpus);
            // (4) Platform architecture: MUST be before "topology"!!!
            fprintf(m_output_fp, "Platform Architecture = %d\n", target_arch_type);
            fprintf(m_output_fp, "CPU Topology = %s\n", m_cpu_topology_str.c_str());
            // (5) TSC Frequency
            fprintf(m_output_fp, "TSC Frequency (MHz) = %u\n", tsc_freq_MHz);
            // (6) MICROCODE patch version (ONLY for MFLD!)
            if (target_arch_type == MFD) {
                fprintf(m_output_fp, "Microcode patch version = %d\n", micro_patch_ver);
            }
            // (8) PROFILED application details
            fprintf(m_output_fp, "Profiled Application: pid = %d name = %s\n", profiled_app_pid, profiled_app_name.length() ? profiled_app_name.c_str() : "SYSTEM-COLLECTION-MODE");
            // (9) ACPI <-> Hardware C-STATE mappings
            fprintf(m_output_fp, "\nACPI <---> HARDWARE C-STATE Mappings:\n\n");
            {
                acpi_mapping_t mappings;
                extract_acpi_c_state_mappings(mappings);

                for (acpi_mapping_t::iterator iter = mappings.begin(); iter != mappings.end(); ++iter) {
                    fprintf(m_output_fp, "%d\t%s\n", (*iter).first, (*iter).second.c_str());
                }
            }
            // (10) list of target residencies.
            fprintf(m_output_fp, "\nTARGET RESIDENCIES:\n\n");
            {
                const int *residencies = s_target_residencies_us[target_arch_type];
                for (int i=0; i<MAX_MSR_ADDRESSES; ++i) {
                    if (residencies[i] != -1) {
                        fprintf(m_output_fp, "C%d = %u\n", i, CONVERT_US_TO_CLOCK_TICKS(residencies[i], tsc_freq_MHz));
                    }
                }
            }
            // (11) list of available frequencies.
            fprintf(m_output_fp, "\nAVAILABLE FREQUENCIES = ");
            {
                for (int i=0; i<PW_MAX_NUM_AVAILABLE_FREQUENCIES && m_available_frequencies[i] != 0; ++i) {
                    fprintf(m_output_fp, "%u ", m_available_frequencies[i]);
                }
                fprintf(m_output_fp, "\n");
            }
            // (12) list of descendents
            fprintf(m_output_fp, "\nDESCENDENT PIDS LIST:\n\n");
            if (descendent_pids.size()) {
                for (int_vec_t::const_iterator citer = descendent_pids.begin(); citer != descendent_pids.end(); ++citer) {
                    fprintf(m_output_fp, "PID %d\n", *citer);
                }
                descendent_pids.clear();
            }

            /*
             * UPDATE: under new scheme, ALL IRQ <-> DEV name and
             * PID <-> PROC name mappings are encapsulated within
             * 'PWCollctor_sample_t' messages and are written to
             * disk as part of the driver output. We therefore
             * do NOT need to write those in the 'sys_params_found.txt'
             * file. We're therefore DONE with the sys_params
             * file.
             */
        }
        fclose(m_output_fp);
    } else {
        /*
         * We had an error somewhere and need to discard
         * all results.
         */
        db_fprintf(stderr, "WARNING: wuwatch detected an error -- flushing all collected data\n");
        fflush(m_output_fp); // not really required, but doesn't hurt us
        fclose(m_output_fp);
        if (unlink(m_output_file_name.c_str())) {
            perror("unlink error when removing damaged wuwatch output file");
        }
    }
};

/*
 * INTERNAL API:
 * Initialize a file descriptor to use when communicating with 
 * the PRELOAD "hook" library. Information on newly forked (or terminated)
 * processes will be communicated to wuwatch via this file descriptor.
 * Used ONLY if NOT running on Android!
 */
void Wuwatch::init_fork_listener(void)
{
#if !_ANDROID_
    struct sockaddr_un servaddr;

    std::string uds_file_name_str = WUWATCH_UNIXSTR_PATH;
    /*
     * Update: actual UNIX domain socket file
     * name will be "/tmp/wuwatch-{$USER}" to
     * avoid perm issues.
     */
    {
        const char *user = getenv("USER");
        if (user) {
            uds_file_name_str += "-";
            uds_file_name_str += user;
        }
    }

    uds_file_name = uds_file_name_str;
    db_fprintf(stderr, "UNIX Domain Socket file name = %s\n", uds_file_name.c_str());

    if ( (fork_listenfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        perror("socket error");
        unlink(uds_file_name.c_str());
        wu_exit(-ERROR);
    }


    // unlink(WUWATCH_UNIXSTR_PATH);
    unlink(uds_file_name.c_str());
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sun_family = AF_LOCAL;
    // strcpy(servaddr.sun_path, WUWATCH_UNIXSTR_PATH);
    strcpy(servaddr.sun_path, uds_file_name.c_str());

    if (bind(fork_listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) {
        perror("bind error");
        wu_exit(-ERROR);
    }

    if (listen(fork_listenfd, WUWATCH_LISTENQ)) {
        perror("listen error");
        wu_exit(-ERROR);
    }
#endif // !_ANDROID_
}
/*
 * INTERNAL API:
 * Destroy the (previously initialized) fd for "fork" requests.
 */
void Wuwatch::destroy_fork_listener(void)
{
#if !_ANDROID_
    close(fork_listenfd);
#endif
}
/*
 * INTERNAL API:
 * Unlink the backing file used for fork "PRELOAD" requests.
 */
void Wuwatch::unlink_fork_listener(void)
{
#if !_ANDROID_
    unlink(uds_file_name.c_str());
#endif
}
/*
 * INTERNAL API:
 * Initialize shmem area used to communicate with the PRELOAD "hook" library.
 * Used ONLY if NOT running on Android!
 */
void Wuwatch::init_shm()
{
#if !_ANDROID_
    struct stat buf;
    int len = sysconf(_SC_PAGE_SIZE), num = len / sizeof(shm_data_t);

    db_fprintf(stderr, "%d, %d, %d\n", len, sizeof(shm_data_t), num);

    int fd = shm_open(PW_SHM_PATH, O_RDWR | O_CREAT, PW_SHM_MODE);
    if (fd < 0) {
        perror("shm_open error");
        wu_exit(-ERROR);
    }
    if (fstat(fd, &buf)) {
        perror("fstat error");
        wu_exit(-ERROR);
    }
    db_fprintf(stderr, "LEN = %d\n", buf.st_size);
    if (ftruncate(fd, len)) {
        perror("ftruncate error");
        shm_unlink(PW_SHM_PATH);
        wu_exit(-ERROR);
    }
    if (fstat(fd, &buf)) {
        perror("fstat error");
        shm_unlink(PW_SHM_PATH);
        wu_exit(-ERROR);
    }
    db_fprintf(stderr, "LEN = %d\n", buf.st_size);

    if ( (fork_shm_data = (shm_data_t *)mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == NULL) {
        perror("mmap error");
        shm_unlink(PW_SHM_PATH);
        wu_exit(-ERROR);
    }
    close(fd);
    /*
     * The 'SERVER' sem should be inited
     * with a value of '1'
     */
    {
        if (sem_init(&fork_shm_data[0].sem, 1, 1)) {
            perror("sem_init error");
            shm_unlink(PW_SHM_PATH);
            wu_exit(-ERROR);
        }
        fork_shm_data[0].count = 1; // "1" for wuwatch itself
    }
    for (int i=1; i<num; ++i) {
        if (sem_init(&fork_shm_data[i].sem, 1, 0)) {
            perror("sem_init error");
            shm_unlink(PW_SHM_PATH);
            wu_exit(-ERROR);
        }
        memset(&fork_shm_data[i].msg, 0, sizeof(uds_msg_t));
    }
#endif // !_ANDROID_
};
/*
 * INTERNAL API:
 * Destroy the previously initialized shmem area.
 * Used ONLY if NOT running on Android!
 */
void Wuwatch::destroy_shm()
{
#if !_ANDROID_
    shm_unlink(PW_SHM_PATH);
#endif // !_ANDROID_
};
/*
 * INTERNAL API:
 * Helper function to transmit information using the (previously mmaped)
 * shared mem region.
 * Used ONLY if NOT running on Android!
 *
 * @offset: offset within the shmem region to use (each client gets a unique shmem offset
 * that functions as a "mailbox address" -- clients read data starting at this offset).
 * @type: the specific message type.
 */
void Wuwatch::shm_send(int offset, uds_msg_type_t type)
{
#if !_ANDROID_
    shm_data_t *data = fork_shm_data + offset;
    uds_msg_t msg;

    msg.type = type;
    msg.dev_fd = -1;

    memcpy(&data->msg, &msg, sizeof(msg));

    sem_post(&data->sem);
#endif // !_ANDROID_
    return;
};
/*
 * INTERNAL API:
 * Helper function to send a QUIT message to all connected clients.
 * Used ONLY if NOT running on Android!
 *
 * @offsets: the list of shared mem offsets to write to.
 */
void Wuwatch::send_quit_to_all_i(const offset_map_t& offsets)
{
#if !_ANDROID_
    if (offsets.empty()) {
        return;
    }
    /*
     * OK, iterate through list of descendants, telling
     * each to exit by sending them a "DO_QUIT" message.
     */
    for (offset_map_t::const_iterator iter = offsets.begin(); iter != offsets.end(); ++iter) {
        shm_send(iter->second, DO_QUIT);
    }
    for (offset_map_t::const_iterator iter = offsets.begin(); iter != offsets.end(); ++iter) {
        /*
         * No, we can't 'waitpid()' on a proc that
         * isn't our child. Which is why we don't
         * explicitly check error conditions here.
         */
        int status = -1;
        waitpid(iter->first, &status, 0);
    }
#endif // !_ANDROID_
};
/*
 * INTERNAL API:
 * Helper function to keep track of how many "clients" (i.e. forked application processes)
 * we need to wait for.
 * Used ONLY if NOT running on Android!
 *
 * @num: the number of clients to increment or decrement.
 *
 * @returns: 0 on success, -1 on error.
 */
inline int Wuwatch::inc_dec_num_shm_clients(int num)
{
#if !_ANDROID_
    shm_data_t *data = fork_shm_data + SERVER_SHM_OFFSET;
    if (sem_wait(&data->sem)) {
        perror("sem_wait error");
        destroy_shm();
        wu_exit(-ERROR);
    }
    data->count += num;
    int retVal = data->count;
    if (sem_post(&data->sem)) {
        perror("sem_post error");
        destroy_shm();
        wu_exit(-ERROR);
    }
#else // !_ANDROID_
    int retVal = SUCCESS;
#endif

    return retVal;
};
/*
 * INTERNAL API:
 * Helper function to transmit information to the PRELOAD "hook" library.
 *
 * @fd: the file descriptor to send data on.
 * @type: the specific message type.
 * @m_dev_fd: the device driver fd; PRELOAD lib instances may use this to simulate "close-on-exec", if required.
 * @offset: the shmem offset that the client will use henceforth.
 */
void Wuwatch::uds_send(int fd, uds_msg_type_t type, int m_dev_fd, int offset)
{
    uds_msg_t msg;
    msg.type = type;
    msg.offset = offset;
    msg.dev_fd = m_dev_fd;

    if (write(fd, &msg, sizeof(msg)) != sizeof(msg)) {
        perror("write error");
        wu_exit(-ERROR);
    }
}
/*
 * INTERNAL API:
 * enable self-pipe trick. Required because 'sigtimedwait()' doesn't
 * seem to work on Android
 */
#if _ANDROID_
static int s_sigchld_pipe[2];
void sigchld_handler(int signum)
{
    char c = 'a';
    write(s_sigchld_pipe[1], &c, sizeof(c));
};

static void timed_waitpid(int secs, pid_t child_pid)
{
    fd_set fds;
    struct timeval tv;
    int max_fd = -1;

    /*
     * First, initialize the pipe itself.
     */
    if (pipe(s_sigchld_pipe)) {
        perror("error initializing self pipe");
        wu_exit(-ERROR);
    }
    /*
     * Then set up the signal handler. For simplicity, we use
     * 'signal' instead of 'sigaction'.
     */
    signal(SIGCHLD, &sigchld_handler);

    /*
     * And finally, setup and call 'select'.
     */
    FD_ZERO(&fds);
    FD_SET(s_sigchld_pipe[0], &fds);
    max_fd = s_sigchld_pipe[0];

    tv.tv_sec = secs; tv.tv_usec = 0;

    switch (select(max_fd+1, &fds, NULL, NULL, &tv)) {
        case -1:
            perror("select error");
            wu_exit(-ERROR);
        case 0: // timeout!
            if (kill(child_pid, SIGKILL)) {
                perror("kill error");
                wu_exit(-ERROR);
            }
            break;
        default:
            break;
    }
};
#else // !_ANDROID_
static void timed_waitpid(int secs, pid_t child_pid)
{
    sigset_t mask, orig_mask;
    struct timespec sig_ts;

    sigemptyset(&mask);
    sigaddset(&mask, SIGCHLD); /* The signal we're waiting for */

    if (sigprocmask(SIG_BLOCK, &mask, &orig_mask)) {
        perror("sigprocmask error");
        wu_exit(-ERROR);
    }

    sig_ts.tv_sec = secs;
    sig_ts.tv_nsec = 0;

    if (sigtimedwait(&mask, NULL, &sig_ts) < 0) {
        if (errno == EAGAIN /* Timeout */ || errno == EINTR /* Something OTHER than SIGCHLD was delivered */) {
            /*
             * OK, need to terminate child.
             * For now, simply send it a SIGKILL
             */
            if (kill(child_pid, SIGKILL)) {
                perror("kill error");
                wu_exit(-ERROR);
            }
        } else {
            perror("sigtimedwait error");
            wu_exit(-ERROR);
        }
    }
};
#endif

/*
 * INTERNAL API:
 * Helper function to fork (and then exec) the application to be profiled.
 *
 * @argv: the (user-supplied) list of arguments to use when execing the application.
 *
 * @returns: 0 on success, -1 on error.
 */
int Wuwatch::fork_child(char *argv[])
{
    int connfd;
    int ret_code = SUCCESS;

    socklen_t clilen;
    uds_msg_t msg;
    fd_set RFDS;
    struct timeval tv, *timeout;
    int maxfd = -1;
    fd_vec_t selectfds;

    if (system_collection_mode_secs > 0) {
        timeout = &tv;
    } else {
        timeout = NULL;
    }

    /*
     * Init the server
     */
    {
        init_fork_listener();
        init_shm();
    }

    child_pid = fork();

    if (!child_pid) {
        /* child */
        {
            /*
             * Make sure the child doesn't see the
             * device fd.
             */
            {
                db_fprintf(stderr, "[%d]: NEW CHILD FORKED: closing device fd = %d\n", getpid(), m_dev_fd);
                close(m_dev_fd);
            }
            /*
             * The child doesn't need to listen for
             * fork requests.
             */
            destroy_fork_listener();
        }
        execvp(argv[0], argv);
        /*
         * If control reaches here ==> an error
         * occurred. Exit immediately.
         */
        fprintf(stderr, "ERROR executing program %s: %s\n", argv[0], strerror(errno));
        wu_exit(-ERROR);
    } else {
        /* parent */
        int status, i;
        int offset = SERVER_SHM_OFFSET, tmp_offset = -1;
        if (child_pid == -1) {
            perror("fork");
            wu_exit(-ERROR);
        }
        /*
         * Record the app PID + name here. We'll write this
         * to our "sys_params_found.txt" file later.
         */
        {
            profiled_app_pid = child_pid;
            std::string tmp_name = std::string(argv[0]);
            int i=0;
            /*
             * SPECIAL CHECK: remove leading "./" from
             * the app name (if any)
             */
            if (tmp_name.length() && tmp_name[0] == '.') {
                for (; !isalnum(tmp_name[i]); ++i);
            }
            profiled_app_name = tmp_name.substr(i);
        }

        /*
         * Wait for the (forked) application to terminate.
         * ************************************************************
         * NOTE THAT THIS WILL BE DONE ONLY IF WUWATCH WAS LAUNCHED
         * WITH THE HOOK LIB AS THE PRELOAD.
         * ************************************************************
         */
        if (getenv("LD_PRELOAD") != NULL) { /* 'wuwatch' was launched with a PRELOAD -- we ASSUME this is the hook lib! */
#if _ANDROID_
            /*
             * This code path should NOT have been taken under android!
             */
            assert(false);
#endif
            /*
             * 'wuwatch' will run in 'server-mode'. In this mode
             * it listens for new applications being forked
             * and takes appropriate actions (stores PIDs etc.).
             * Setup the server "select" loop here.
             */
            FD_ZERO(&RFDS);
            FD_SET(fork_listenfd, &RFDS);
            maxfd = fork_listenfd;
            /*
             * We set the timeout ONE-TIME ONLY! This is
             * because in Linux the timeout struct is updated
             * with the amount of time remaining until
             * the timer elapses. THIS BEHAVIOUR IS LINUX-SPECIFIC!
             */
            tv.tv_sec = system_collection_mode_secs; tv.tv_usec = 0;
            /*
             * Loop until either the timout expires, or
             * all descendant processes exit.
             */
            for (;;) {
                int numfds = -1;
                fd_set rfds;
                memcpy(&rfds, &RFDS, sizeof(rfds)); // dst, src
                if ( (numfds = select(maxfd+1, &rfds, NULL, NULL, timeout)) == -1) {
                    perror("select error");
                    wu_exit(-ERROR);
                }
                if (numfds == 0) {
                    /* TIMEOUT */
                    fprintf(stderr, "TIMEOUT!\n");
                    send_quit_to_all_i(offsets);
                    sleep(1);
                    break;
                }
                /*
                 * NOT a timeout -- iterate through
                 * list of FDs checking to see
                 * which one unblocked.
                 */
                db_fprintf(stderr, "DEBUG: SECS = %d, USECS = %d\n", tv.tv_sec, tv.tv_usec);
#if !_ANDROID_
                if (FD_ISSET(fork_listenfd, &rfds)) {
                    /*
                     * We have a new connection
                     */
                    struct sockaddr_un cliaddr;
                    clilen = sizeof(cliaddr);
                    if ( (connfd = accept(fork_listenfd, (struct sockaddr *)&cliaddr, &clilen)) < 0) {
                        if (errno == EINTR)
                            continue;
                        else {
                            perror("accept error");
                            wu_exit(-ERROR);
                        }
                    }
                    FD_SET(connfd, &RFDS);
                    maxfd = (connfd > maxfd) ? connfd : maxfd;
                    selectfds.push_back(connfd);
                    db_fprintf(stderr, "CONNECTED! fd = %d, maxfd = %d\n", connfd, maxfd);
                    if (--numfds <= 0)
                        continue;
                }
#endif // !_ANDROID_
                /*
                 * A previously accepted connection unblocked -- iterate
                 * through the list of connections to check which one(s)
                 * have data for us.
                 */
                for (fd_vec_t::iterator iter = selectfds.begin(); iter != selectfds.end() && numfds > 0; ++iter) {
                    int childfd = *iter;
                    if (FD_ISSET(childfd, &rfds)) {
                        /*
                         * OK, read available data...
                         */
                        if (read(childfd, &msg, sizeof(msg)) != sizeof(msg)) {
                            db_fprintf(stderr, "read error in server for fd = %d: %s\n", childfd, strerror(errno));
                            /*
                             * Usually happens because 'hook_lib_initialize()' is
                             * (sometimes) called TWICE. For now, tolerate it.
                             */
                            FD_CLR(childfd, &RFDS);
                            close(childfd);
                            --numfds;
                            selectfds.erase(iter);
                            continue;
                        }
                        /*
                         * ...and do some work on it.
                         */
                        switch(msg.type) {
                            case FORK_SYN:
                                db_fprintf(stderr, "NEW PROC FORKED: id = %d\n", msg.child_pid);
                                /*
                                 * The same proc could have done a 'FORK_SYN' on
                                 * us more than once (because the 'hook_lib_initialize()'
                                 * function may be called more than once). Check
                                 * for that BEFORE we decrement the # clients
                                 * ref count and allocate a new offset etc.
                                 */
                                if (offsets.find(msg.child_pid) == offsets.end()) {
                                    /*
                                     * 'fork()' in parent would have incremented
                                     * the # clients refcount -- decrement now.
                                     */
                                    DEC_NUM_SHM_CLIENTS();
                                    tmp_offset = ++offset;
                                    offsets[msg.child_pid] = offset;
                                    descendent_pids.push_back(msg.child_pid);
                                } else {
                                    tmp_offset = offsets[msg.child_pid];
                                }
                                uds_send(childfd, FORK_ACK, m_dev_fd, offset);
                                break;
                            case EXIT_SYN:
                                db_fprintf(stderr, "OLD PROC EXITTED: fd = %d, id = %d, offset = %d\n", childfd, msg.child_pid, offsets[msg.child_pid]);
                                if (offsets.find(msg.child_pid) != offsets.end()) {
                                    offsets.erase(offsets.find(msg.child_pid));
                                    uds_send(childfd, EXIT_ACK, -1, -1);
                                } else {
                                    db_fprintf(stderr, "ERROR: no mapping for %d found!\n", msg.child_pid);
                                }
                                break;
                            default:
                                fprintf(stderr, "ERROR: INVALID msg type = %d\n", msg.type);
                                wu_exit(-ERROR);
                        }
                        /*
                         * We no longer need the child fd
                         * (all further comm via the SHM
                         * interface).
                         */
                        FD_CLR(childfd, &RFDS);
                        close(childfd);
                        --numfds;
                        selectfds.erase(iter);
                    }
                }
                /*
                 * Check list of FDS to see if we still
                 * need to wait...
                 */
                int num_clients = GET_NUM_SHM_CLIENTS();
                db_fprintf(stderr, "# CLIENTS = %d, OFFSETS map size = %d\n", num_clients, offsets.size());
                if (num_clients == 0 && offsets.empty()) {
                    /*
                     * OK, all descendants exitted
                     */
                    db_fprintf(stderr, "ALL CHILD PROCS EXITTED!\n");
                    break;
                }
            }
        }
        else {
            /*
             * wuwatch was NOT invoked with a PRELOAD hook library.
             * Update: if the user specifies a timeout value in addition to the application to profile
             * then wait for it to expire.
             * Note that if we launch with the PRELOAD lib then this is auto handled above. We
             * only need the code below if the user doesn't specify a PRELOAD library
             * (this is the default on Android, for example).
             */
            if (system_collection_mode_secs > 0) {
                timed_waitpid(system_collection_mode_secs, child_pid);
            }
            /*
             * OK, either the child exitted, or the collection timed out.
             * In either case, waiting on the child shouldn't cause us
             * to hang.
             */
            if (waitpid(child_pid, &status, 0) == -1) {
                perror("wait");
                wu_exit(-ERROR);
            }
            /*
             * We need to check the child's return/exit status
             * (to check for errors).
             */
            if (WIFEXITED(status)) {
                ret_code = WEXITSTATUS(status);
                db_fprintf(stderr, "Child exitted: ret_code = %d\n", ret_code);
                if (ret_code != SUCCESS) {
                    ret_code = -ERROR;
                }
            }
        }

        /*
         * Destroy server and write list of
         * descendent pids to disk. For now,
         * write this list to a dedicated
         * file. Later, we'll look into adding that
         * information to the 'sys_params_found.txt'
         * file instead.
         */
        {
            destroy_shm();
            destroy_fork_listener();
            /*
             * We also need to 'unlink' the backing socket.
             */
            unlink_fork_listener();

            db_fprintf(stderr, "DESTROYED server...\n");
            db_fprintf(stderr, "# descendent pids = %d\n", descendent_pids.size());
        }
    }

    return ret_code;
};


/*
 * INTERNAL API:
 * Stop a collection and perform some bookkeeping.
 */
void Wuwatch::do_exit(void)
{
    /*
     * Stop the collection (take a snapshot
     * of the system time after that). Also
     * calculate elapsed time.
     */
    {
        do_ioctl_start_stop(m_dev_fd, false);
    }

    {
        m_finalTSC = rdtsc();
        start_stop_timer(&collection_time_msecs, false); // "false" ==> STOP timer
        collection_time_msecs /= 1e3; // convert from "usecs" to "msecs"

        db_fprintf(stderr, "Total Collection Time = %f msecs\n", collection_time_msecs);
    }
}

/*
 * EXTERNAL API:
 * Start the power data collection.
 * If instructed to do so by the user, fork (and exec) an application process.
 * Alternately, sleep for a prescribed amount of time.
 * After the application has exitted (or the prescribed timeout has elapsed),
 * stop the power data collection.
 *
 * @argv: (user-supplied) arguments describing the application to be profiled
 * (or NULL, if no application is to be profiled).
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::work(char **argv)
{
    int ret_code = SUCCESS;
    /*
     * OK, we're ready to start the collection. Before we do so, create (and output) 
     * the initial proc map. We defer this for as long as possible because we don't
     * want too much time to elapse between when the snapshot of the proc mappings
     * was taken and when the collection starts.
     */
    if (get_initial_proc_map_i(m_output_fp)) {
        fprintf(stderr, "ERROR: could NOT create initial proc map!\n");
        wu_exit(-ERROR);
    }
    /*
     * Take a snapshot of the initial TSC and 'timeval' values. We want
     * to take this after we're done trolling the "/proc" fs.
     */
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        m_initialTSC = rdtsc();
        initialTimeval = (1000000ULL * tv.tv_sec + tv.tv_usec) * 10;
    }

    /*
     * Start the collection (take a snapshot
     * of the system time before that).
     */
    {
        start_stop_timer(&collection_time_msecs, true); // "true" ==> START timer
    }

    {
        do_ioctl_start_stop(m_dev_fd, true);
    }

    /*
    int i = pthread_barrier_wait(&reader_barrier);
    if (i && i != PTHREAD_BARRIER_SERIAL_THREAD) {
        perror("pthread_barrier_wait error");
        wu_exit(-ERROR);
    }
    */
    if (s_barrier.wait()) {
        fprintf(stderr, "barrier_wait error");
        wu_exit(-ERROR);
    }


    if (*argv) {
        /*
         * There's an application to
         * be profiled.
         * Fork the child (and
         * wait for it to terminate).
         */
        ret_code = fork_child(argv);
    } else {
        /*
         * Two options:
         * (a) SYSTEM COLLECTION MODE (timed).
         * (b) No timeout -- collect until SIGINT.
         */
        if (system_collection_mode_secs > 0) {
            fprintf(stderr, "SYSTEM COLLECTION MODE: time = %d seconds\n", system_collection_mode_secs);
            /*
             * OK, sleep for 'system_collection_mode_secs' seconds.
             */
            sleep(system_collection_mode_secs);
        } else {
            pause(); /* wait for SIGINT */
        }
    }

    do_exit();
    return ret_code;
};

/*
 * EXTERNAL API:
 * Sets the data collector output file name, based on user-supplied inputs.
 *
 * @dir_name: The directory name.
 * @file_name: The output file name.
 */
void Wuwatch::set_output_file_name(const std::string& dir_name, const std::string& file_name)
{
    assert(dir_name.size() && file_name.size());
    /*
     * Make sure the directory specified by 'dir_name'
     * actually exists. We do this by issuing a (possibly
     * gratuitous) "mkdir" instruction and ignoring the
     * return value.
     */
    if (mkdir(dir_name.c_str(), (S_IRWXU | S_IRWXG | S_IRWXO))) {
        switch (errno) {
            case EEXIST: /* NOP */
                break;
            default:
                fprintf(stderr, "ERROR creating result dir %s: %s\n", dir_name.c_str(), strerror(errno));
                wu_exit(-ERROR);
        }
    } else {
        /*
         * "mkdir" doesn't have an error ==> a dir was actually created.
         */
        db_fprintf(stderr, "OK, created output dir = %s\n", dir_name.c_str());
    }
    /*
     * We always append a special extension to wuwatch files.
     */
    m_output_file_name = dir_name + file_name + ".ww1";
    db_fprintf(stderr, "WUWATCH has output file = %s\n", m_output_file_name.c_str());
};
/*
 * EXTERNAL API:
 * Set path to the device driver. Required if we want to
 * dynamically load the driver.
 *
 * @name: path to the driver (or empty, if the user wishes us
 * to use the default path).
 */
void Wuwatch::set_driver_path(const std::string& name)
{
    if (name.size()) {
        m_driver_path = name;
    } else {
        m_driver_path = "./driver/apwr2_2.ko";
    }
};
/*
 * EXTERNAL API:
 * Retrieve power driver version information.
 *
 * @ver_str: the device driver version retrieved from the power driver.
 *
 * @returns: 0 on success, -1 on error.
 */
int Wuwatch::get_driver_version(std::string& ver_str)
{
    /*
     * Get driver version information.
     */
    if (open_dd_i()) {
        db_fprintf(stderr, "ERROR: could NOT retrieve DRIVER VERSION information!\n");
        return -ERROR;
    }
    {
        if (do_ioctl_driver_version_i(m_dev_fd, ver_str)) {
            db_fprintf(stderr, "ERROR: could NOT retrieve DRIVER VERSION information!\n");
            return -ERROR;
        }
    }
    return SUCCESS;
};
