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
#if _NDK_BUILD_
#include <sys/system_properties.h>
#endif

#include "pw_defines.h"
#include "pw_ioctl.h" // IOCTL stuff.
#include "uds.hpp" // for UDS server stuff.
#include "pw_arch.h" // Architecture info.
#include "pw_parser.h" // Config file parser
#include "defines.h"
#include "wuwatch_defines.h"
#include "wuwatch.h"

/*
 * Required syscall wrapper for manual insertion
 * of the power driver.
 */
#if !_NDK_BUILD_
#define load_module(...) syscall(SYS_init_module, __VA_ARGS__)
#define delete_module(...) syscall(SYS_delete_module, __VA_ARGS__)
#else
#define load_module(...) syscall(__NR_init_module, __VA_ARGS__)
#define delete_module(...) syscall(__NR_delete_module, __VA_ARGS__)
#endif

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
 * A list of North complex device names for Medfield.
 */
const char *pw_mfd_nc_device_names[][2] = { 
                                {"GPS", "GFX subsystem"},
                                {"VDPS", "Video Decode subsystem"},
                                {"VEPS", "Video Encode subsystem"},
                                {"DPA", "Display Island A"},
                                {"DPB", "Display Island B"},
                                {"DPC", "Display Island C"},
                                {"GL3", "GL3 Power Island"}, 
                                {"ISP", "ISP Power Island"},
                                {"IPH", "IPH Power Island"} };
/*
 * A list of North complex device names for Clovertrail.
 */
const char *pw_clv_nc_device_names[][2] = { 
                                {"GPS", "GFX subsystem"},
                                {"VDPS", "Video Decode subsystem"},
                                {"VEPS", "Video Encode subsystem"},
                                {"DPA", "Display Island A"},
                                {"DPB", "Display Island B"},
                                {"ISP", "ISP Power Island"},
                                {"IPH", "IPH Power Island"} };
/*
 * A list of South complex device names for Medfield.
 */
const char *pw_mfd_sc_device_names[][2] = { 
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
/*
 * A list of South complex device names for Clovertrail.
 */
const char *pw_clv_sc_device_names[][2] = { 
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

/*
 * A list of UID to pkg name mappings predefined on Android.
 * We can get the full list from 
 * system/core/include/private/android_filesystem_config.h.
 */
int android_predefined_uid_pkg_num = 29;
const char *android_predefined_uid_pkg_mappings[][2] = { 
                                {"0", "root"},
                                {"1000", "system server"},
                                {"1001", "radio"},
                                {"1002", "bluetooth"},
                                {"1003", "graphics"},
                                {"1004", "input"},
                                {"1005", "audio"},
                                {"1006", "camera"},
                                {"1007", "log"},
                                {"1008", "compass"},
                                {"1009", "mountd"},
                                {"1010", "WIFI"},
                                {"1011", "adbd"},
                                {"1012", "install"},
                                {"1013", "media server"},
                                {"1014", "DHCP client"},
                                {"1015", "SDcard write"},
                                {"1016", "VPN"},
                                {"1017", "keystore"},
                                {"1018", "USB"},
                                {"1019", "DRM server"},
                                {"1020", "Multicast DNS"},
                                {"1021", "GPS daemon"},
                                {"1023", "Internal media write"},
                                {"1024", "MTP USB"},
                                {"1026", "DRM RPC"},
                                {"1027", "NFC"},
                                {"1028", "SDcard read"},
                                {"1029", "Smart card"} };

/* **************************************
 * Function definitions.
 * **************************************
 */
int pwr::barrier::init(int num_threads)
{
    if (pthread_mutex_init(&lock, NULL) || pthread_cond_init(&cond, NULL)) {
        perror("pthread_mutex_init or pthread_cond_init error");
        return -PW_ERROR;
    }
    numThreads = num_threads;
    wasInitialized = true;
    return PW_SUCCESS;
};

int pwr::barrier::wait()
{
    if (!wasInitialized) {
        return -PW_ERROR;
    }
    if (pthread_mutex_lock(&lock)) {
        perror("pthread_mutex_lock error");
        return -PW_ERROR;
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
    return PW_SUCCESS;
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
    max_num_cpus        = -1;
    m_dev_fd              = -1;
    fork_listenfd       = -1;
    uds_file_name       = "";
    m_output_fp         = NULL;
    child_pid           = -1;

    m_driver_major      = -1;
    m_driver_minor      = -1;
    m_driver_other      = -1;

    micro_patch_ver     = 0;
    tsc_freq_MHz        = 0;

    c_state_collection      = 0;
    p_state_collection      = 0;
    s_residency_collection  = 0;
    // s_state_collection      = 0;
    d_residency_collection  = 0;
    d_nc_state_collection   = 0;
    // d_sc_state_collection   = 0;
    w_state_collection      = 0;
    u_state_collection      = 0;
    m_collectionSwitches   = 0;
    m_do_force_dd_load	    = 0;

    m_do_collect_kernel_backtrace   = 0;

    d_state_sample_interval_msecs   = 100;


    collection_time_msecs           = 0.0;
    system_collection_mode_secs     = 0;

    m_initialTSC = 0;
    m_finalTSC = 0;
    initialTimeval = 0;
    m_initialTime = 0;
    turboThreshold = 0;

    m_totalSamples = 0;
    m_droppedSamples = 0;

    m_available_frequencies[0] = 0;

    m_wasAutoDemoteEnabled = m_wasAnyThreadSet = 0;

    m_targetArchRec = NULL;

    m_mmap_addr = NULL;
    m_mmap_size = 0;
    m_buff_size = 0;
};

/*
 * The wuwatch destructor.
 */
Wuwatch::~Wuwatch() {
    if (m_dev_fd > 0) {
        close(m_dev_fd);
    }
    delete m_targetArchRec;
};

/*
 * Helper function to provide "popen"-like functionality.
 */
FILE *Wuwatch::popen_ro(const char *command, pid_t& pid) {
    int pdes_pipe[2];
    if (pipe(pdes_pipe)) {
        perror("pipe error");
        return NULL;
    }
    
    pid = fork();
    if (pid < 0) {
        perror("fork error");
        return NULL;
    } else if (pid == 0) {
        /*
         * Child.
         */
        close(pdes_pipe[0]); // child doesn't need to read from pipe
        /*
         * If child writes to stdout then
         * parent should be able to read
         * the fp.
         */
        dup2(pdes_pipe[1], fileno(stdout));
        /*
         * Android systems have "/system/bin/sh", not "/bin/sh"
         * We try both. We first try the "/bin/sh" exec.
         * We then try the "/system/bin/sh" exec.
         * Note that, by definition, if the first exec succeeds
         * then the second one WILL NOT EXECUTE!
         */
        if (execl("/bin/sh", "sh", "-c", command, (char *)NULL) && execl("/system/bin/sh", "sh", "-c", command, (char *)NULL)) {
            db_fprintf(stderr, "Command %s could not be executed\n", command);
            return NULL;
        }
    }
    /*
     * Parent
     */
    close(pdes_pipe[1]); // parent doesn't need to write to pipe
    FILE *fp = fdopen(pdes_pipe[0], "r");
    db_fprintf(stderr, "Parent: fd = %d, fp = %p\n", pdes_pipe[0], fp);
    return fp;
};

/*
 * Helper function to provide "pclose"-like functionality.
 */
int Wuwatch::pclose_ro(FILE *fp, pid_t pid) {
    if (pid == -1) {
        db_fprintf(stderr, "ERROR: invalid pid in pclose_ro!\n");
        return -PW_ERROR;
    }
    int status = -1;
    if (waitpid(pid, &status, 0) < 0) {
        perror("waitpid error");
        return -PW_ERROR;
    }
    if (fp != NULL) {
        fclose(fp);
    }
    return PW_SUCCESS;
};

/*
 * INTERNAL helper function: get the android version string
 * from system property database.
 *
 * @version: Android version returned.
 *
 * @returns: PW_SUCCESS on OK
 *         : -PW_ERROR on ERROR
 */
int Wuwatch::get_android_version_i(std::string& version)
{
    /*
     * Following taken from "PWCollector::parse_boot_config_file_i()"
     */
    std::string command = "getprop ro.build.version.release";
    char tmp[1024];
    version = "UNKNOWN";

    pid_t pid = -1;
    FILE *fp = popen_ro(command.c_str(), pid);

    if (!fp) {
        db_fprintf(stderr, "ERROR: could not get the system property with getprop!\n");
	return -PW_ERROR;
    }

    memset(tmp, 0, sizeof(tmp));

    fgets(tmp, sizeof(tmp), fp);

    // need to get rid of terminating NEWLINE (if any)
    int len = strlen(tmp);
    if (len <= 0) {
        db_fprintf(stderr, "ERROR: invalid strlen\n");
        pclose_ro(fp, pid);
        return -PW_ERROR;
    }
    if (tmp[len-1] == '\n') {
	tmp[len-1] = '\0';
    }

    // pclose(fp);
    pclose_ro(fp, pid);

    version = "Android_";
    version += tmp;

    db_fprintf(stderr, "GETPROP ro.build.version.release = %s\n", version.c_str());
    return PW_SUCCESS;
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
        return -PW_ERROR;
    }
    /*
     * We need to know the size of the driver before
     * we can allocate buffer space for it.
     */
    if (fstat(fd, &st_buf)) {
        perror("fstat error");
        close(fd);
        return -PW_ERROR;
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
        return -PW_ERROR;
    }

    /*
     * OK, we've read the power driver file into memory.
     * Now load it.
     */
    if (load_module(buffer, size, &options)) {
        switch (errno) {
            case EPERM:
                fprintf(stderr, "ERROR loading power driver: you do NOT have sufficient permissions to load the device driver! Consider re-running as root.\n");
                break;
            default:
                perror("load module error");
        }
        delete []buffer;
        return -PW_ERROR;
    }

    delete []buffer;
    return PW_SUCCESS;
};

/*
 * INTERNAL API:
 * Unloads the power driver (basically, replicates 'rmmod').
 * 
 * @returns: 0 on success, -1 on failure
 */
int Wuwatch::do_rmmod_i(void)
{
    if (delete_module(PW_DRV_NAME)) {
        switch (errno) {
            case EPERM:
                fprintf(stderr, "ERROR unloading power driver: you do NOT have sufficient permissions to unload the device driver (%s)! Consider re-running as root.\n", PW_DRV_NAME);
                break;
            case EBUSY:
                fprintf(stderr, "ERROR unloading power driver: the driver (%s) is in use.\n", PW_DRV_NAME);
                break;
            case ENOENT:
                fprintf(stderr, "ERROR unloading power driver: the driver (%s) does NOT exist.\n", PW_DRV_NAME);
                break;
            default:
                fprintf(stderr, "ERROR: could NOT unload device driver!\n");
        }
        return -PW_ERROR;
    }

    return PW_SUCCESS;
};

/*
 * INTERNAL API:
 * Open a connection to the power driver.
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::open_dd_i()
{
    int devfd; 
    /*
     * Open connection to device driver.
     */
    if (m_dev_fd > 0) {
        return PW_SUCCESS;
    }

    /*
     * We couldn't find the DD -- we'll try to do a manual
     * 'insmod' here, but ONLY if the user has allowed
     * us to do so (i.e. specified the "-f/--force-dd-load"
     * option)
     */
    if (m_do_force_dd_load) {
        if ((devfd = open(PW_DEVICE_FILE_NAME, 0)) > 0) {
            close(devfd);
            fprintf(stderr, "Found the existing device driver. Unloading device driver (%s)...\n", PW_DRV_NAME);
            do_rmmod_i();
        }

        /*
         * Sanity check to confirm if the driver is unloaded.
         */
        if ((devfd = open(PW_DEVICE_FILE_NAME, 0)) > 0) {
            fprintf(stderr, "ERROR: an old device driver is likely to be still loaded! Please remove it manually with 'rmmod'.\n");
            close(devfd);
            return -PW_ERROR;
        }

        fprintf(stderr, "Loading device driver (%s)...\n", m_driver_path.c_str());
        if (do_insmod_i()) {
            fprintf(stderr, "ERROR: could NOT load device driver!\n");
            return -PW_ERROR;
        }
        fprintf(stderr, "Device driver loaded successfully!\n");
    }

    if ((m_dev_fd = open(PW_DEVICE_FILE_NAME, 0)) < 0) {
        switch (errno) {
            case EPERM:
                fprintf(stderr, "ERROR: you do NOT have permission to access the character device file: %s\n", PW_DEVICE_FILE_NAME);
                return -PW_ERROR;
            case ENOENT:
                fprintf(stderr, "ERROR: device driver (%s) NOT loaded!\n", m_driver_path.c_str());
                return -PW_ERROR;
            default:
                perror("ERROR on requested access to character device file");
                return -PW_ERROR;
        }
    }
    return PW_SUCCESS;
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
    int cmd = enable ? PW_START : PW_STOP;

#if DO_COUNT_DROPPED_SAMPLES
    {
        u64 array[2];
        struct PWCollector_ioctl_arg ioctl_arg;

        ioctl_arg.in_len = sizeof(cmd);
        ioctl_arg.in_arg = (char *)&cmd;

        ioctl_arg.out_len = sizeof(array);
        ioctl_arg.out_arg = (char *)array;

        if (false) {
            fprintf(stderr, "%lu\n", TO_UL(sizeof(array)));
            assert(false);
        }

        if (ioctl(fd, PW_IOCTL_CMD, &ioctl_arg) < 0) {
            perror("ioctl error");
            return;
        }
        if (cmd == PW_STOP) {
            m_totalSamples = array[0];
            m_droppedSamples = array[1];

            if (m_droppedSamples != 0) {
                fprintf(stderr, "\nWARNING: There were %llu samples dropped out of a total of %llu!\n\n", TO_ULL(array[1]), TO_ULL(array[0]));
            }
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
int Wuwatch::do_ioctl_driver_version_i(int fd)
{
    PWCollector_version_info version;

    if (do_ioctl_i(fd, PW_IOCTL_VERSION, &version, sizeof(PWCollector_version_info), false) == NULL) // "false" ==> OUTPUT param.
        return -PW_ERROR;

    db_fprintf(stderr, "DEVICE DRIVER VERSION = %d.%d.%d\n", version.version, version.inter, version.other);


    m_driver_major = version.version;
    m_driver_minor = version.inter;
    m_driver_other = version.other;

    char version_string[1024];
    sprintf(version_string, "%d.%d.%d", version.version, version.inter, version.other);
    m_driver_version_string = std::string(version_string);

    return PW_SUCCESS;
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
        return -PW_ERROR;

    db_fprintf(stderr, "DRIVER RETURNS PATCH VERSION = %d\n", tmp_patch_ver);

    patch_ver = tmp_patch_ver;

    return PW_SUCCESS;
}
static void cpuid(unsigned info, unsigned *eax, unsigned *ebx, unsigned *ecx, unsigned *edx)
{
   *eax = info;
   __asm volatile
   ("mov %%ebx, %%edi; cpuid; mov %%ebx, %%esi; mov %%edi, %%ebx;"
    :"+a" (*eax), "=S" (*ebx), "=c" (*ecx), "=d" (*edx) : : "edi");
}

/*
 * INTERNAL API:
 * Get CPU arch details. Adapted
 * from code taken from
 * 'turbostat'.
 */
void Wuwatch::retrieve_target_arch_details_i()
{
    unsigned int eax, ebx, ecx, edx, max_level;
    unsigned int fms;

    eax = ebx = ecx = edx = 0;

    cpuid(0x0, &max_level, &ebx, &ecx, &edx);

    sprintf(cpu_brand, "%.4s%.4s%.4s", (char *)&ebx, (char *)&edx, (char *)&ecx);

    db_assert(!strncmp(cpu_brand, "GenuineIntel", 12), "CPUID: %s != GenuineIntel\n", cpu_brand);

    if (strncmp(cpu_brand, "GenuineIntel", 12)) {
        fprintf(stderr, "CPUID: %s != GenuineIntel\n", cpu_brand);
        wu_exit(-PW_ERROR);
    }

    cpuid(0x1, &fms, &ebx, &ecx, &edx);

    m_cpuFamily = (fms >> 8) & 0xf;
    m_cpuModel = (fms >> 4) & 0xf;
    m_cpuStepping = fms & 0xf;
    if (m_cpuFamily == 6 || m_cpuFamily == 0xf) {
        m_cpuModel += ((fms >> 16) & 0xf) << 4;
    }

    if (false) {
        db_fprintf(stderr, "CPUID %s %d levels family:model:stepping 0x%x:%x:%x (%d:%d:%d)\n",
                cpu_brand, max_level, m_cpuFamily, m_cpuModel, m_cpuStepping, m_cpuFamily, m_cpuModel, m_cpuStepping);
    }
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
    // sleep(1);
    usleep(1000); // 1 msec
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
 * @interval: the D-state sampling interval.
 */
void Wuwatch::do_ioctl_config(int fd, u32 interval)
{
    int i=0;
    struct PWCollector_config config;
    int len = sizeof(config);
    bool does_support_s_d_states = false;
    u32 busClockFreqKHz = 0x0;
    int residency_count_multiplier = 0;

    /*
     * Do arch-specific config stuff here.
     */
    {
        std::vector <int> coreResAddrs(MAX_MSR_ADDRESSES), pkgResAddrs(MAX_MSR_ADDRESSES);
        const std::vector <CRec>& crecs = m_targetArchRec->get_c_states();
        for (std::vector<CRec>::const_iterator citer = crecs.begin(); citer != crecs.end(); ++citer) {
            switch (citer->m_type) {
                case PW_MSR_PACKAGE:
                    /*
                    pkgResAddrs[citer->num] = citer->msr_addr;
                    break;
                    */
                case PW_MSR_CORE:
                case PW_MSR_THREAD:
                    coreResAddrs[citer->num] = citer->msr_addr;
                default: // fall-through
                    break;
            }
        }
        /*
        for (int i=0; i<MAX_MSR_ADDRESSES; ++i) {
            coreResAddrs[i] = crecs[i].msr_addr;
        }
        */
        busClockFreqKHz = m_targetArchRec->get_bus_freq();
        residency_count_multiplier = 1; // This is now don't care for the driver!
        if (PW_IS_SALTWELL(m_cpuModel)) {
            does_support_s_d_states = true;
        } else {
            does_support_s_d_states = false;
        }
        /*
         * GU: moved the S,D state checks here to accomodate
         * future arch requirements.
         */
        if (does_support_s_d_states == false) {

            if (s_residency_collection) {
                fprintf(stderr, "Warning: -sr/--s-residency switch not supported on this platform. No sr data will be collected.\n");
                RESET_COLLECTION_SWITCH(m_collectionSwitches, PW_PLATFORM_RESIDENCY);
            }

            if (d_residency_collection) {
                fprintf(stderr, "Warning: dres switch not supported on this platform. No dres data will be collected.\n");
                RESET_COLLECTION_SWITCH(m_collectionSwitches, PW_DEVICE_SC_RESIDENCY);
            }

            if (d_nc_state_collection) {
                fprintf(stderr, "Warning: -dn/--nc-states switch not supported on this platform. No dn data will be collected.\n");
                RESET_COLLECTION_SWITCH(m_collectionSwitches, PW_DEVICE_NC_STATE);
            }

#if 0
            if (d_sc_state_collection) {
                fprintf(stderr, "Warning: -ds/--sc-states switch not supported on this platform. No ds data will be collected.\n");
                switches[PW_DEVICE_SC_STATE] = 0;
            }
#endif

#if 0
            if (s_state_collection) {
                fprintf(stderr, "Warning: s-state collection is NOT supported on this platform! No s-state data will be collected!\n");
                switches[PW_PLATFORM_STATE] = 0;
            }
#endif

            if (w_state_collection) {
                fprintf(stderr, "Warning: kernel wakelock switch is only supported on Android. No kernel wakelock data will be collected.\n");
                RESET_COLLECTION_SWITCH(m_collectionSwitches, PW_WAKELOCK_STATE);
            }

            if (u_state_collection) {
                fprintf(stderr, "Warning: user wakelock switch is only supported on Android. No user wakelock data will be collected.\n");
                RESET_COLLECTION_SWITCH(m_collectionSwitches, PW_WAKELOCK_STATE);
            }
        }

        memset(&config, 0, len);

        config.data = m_collectionSwitches;

        config.d_state_sample_interval = interval;

        memcpy(&config.info.coreResidencyMSRAddresses, &coreResAddrs[0], sizeof(int) * MAX_MSR_ADDRESSES); // dst, src
        memcpy(&config.info.pkgResidencyMSRAddresses, &pkgResAddrs[0], sizeof(int) * MAX_MSR_ADDRESSES); // dst, src

        // db_fprintf(stderr, "DEBUG: <PWR> ARCH type = %s\n", g_arch_type_names[arch]);
        db_fprintf(stderr, "DEBUG: <PWR> busClockFreqKhz = %u\n", busClockFreqKHz);
        db_fprintf(stderr, "DEBUG: <PWR> residency_count_multiplier = %d\n", residency_count_multiplier);

        config.info.bus_clock_freq_khz = busClockFreqKHz;
        config.info.residency_count_multiplier = residency_count_multiplier;
    }

    db_fprintf(stderr, "DEBUG: <PWR> config.data = 0x%x\n", config.data);
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
        return -PW_ERROR;
    }
    /*
     * Sanity test!
     */
    assert(freqs.num_freqs <= PW_MAX_NUM_AVAILABLE_FREQUENCIES);

    db_fprintf(stderr, "Available frequencies...\n");
    for (u32 i=0; i<freqs.num_freqs; ++i) {
        db_fprintf(stderr, "%u\t", freqs.frequencies[i]);
    }
    db_fprintf(stderr, "\n");
    /*
     * Copy frequencies into output array.
     */
    if (freqs.num_freqs <= PW_MAX_NUM_AVAILABLE_FREQUENCIES) {
        memcpy(available_frequencies, freqs.frequencies, sizeof(u32) * freqs.num_freqs);
    } else {
        available_frequencies[0] = 0; // last entry MUST be ZERO!!!
        return -PW_ERROR;
    }
    available_frequencies[freqs.num_freqs] = 0; // last entry MUST be ZERO!!!
    return PW_SUCCESS;
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
        return -PW_ERROR;
    *turbo_thresh = thresh.threshold_frequency;
    return PW_SUCCESS;
};

int Wuwatch::do_ioctl_check_platform_i(int fd, u32& supported_kernel_features, u32& supported_arch_features) {
    PWCollector_check_platform platform;
    memset(&platform, 0, sizeof(platform));
    supported_kernel_features = supported_arch_features = 0;
    if (do_ioctl_i(fd, PW_IOCTL_CHECK_PLATFORM, &platform, sizeof(PWCollector_check_platform), false) == NULL) { // "false" ==> OUTPUT param.
        return -PW_ERROR;
    }
    supported_kernel_features = platform.supported_kernel_features;
    supported_arch_features = platform.supported_arch_features;
    return PW_SUCCESS;
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

    return who->reader_thread_i(args);
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

    delete args;

/*
    i = pthread_barrier_wait(&reader_barrier);
    if (i && i != PTHREAD_BARRIER_SERIAL_THREAD) {
        perror("pthread_barrier_wait error");
        wu_exit(-PW_ERROR);
    }
*/
    if (s_barrier.wait()) {
        fprintf(stderr, "barrier_wait error in reader thread!\n");
        //delete []samples;
        wu_exit(-PW_ERROR);
    }

    {
        int bytes_read = 0;
        std::vector <char> read_buff(m_buff_size);

        for (bytes_read = read(m_dev_fd, &read_buff[0], m_buff_size); bytes_read > 0; bytes_read = read(m_dev_fd, &read_buff[0], m_buff_size)) {
            db_fprintf(stderr, "OK: client receives %d bytes\n", bytes_read);
            if (fwrite(&read_buff[0], sizeof(char), bytes_read, fp) != bytes_read) {
                perror("fwrite error");
                wu_exit(-PW_ERROR);
            }
        }
        if (bytes_read < 0) {
            perror("Error reading from device driver");
            wu_exit(-PW_ERROR);
        }
    }

    //pthread_exit(PTHREAD_CANCELED);
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
    std::string str_line;
    //char *line = NULL;
    int proc = -1, phys_id = -1, core_id = -1, core_no, sibling;
    int retVal = PW_SUCCESS;

    std::map <int, int_vec_t> tmp_map;
    int_map_t cpu_map;

    FILE *fp = fopen("/proc/cpuinfo", "r");
    if (!fp) {
        fprintf(stderr, "fopen error for /proc/cpuinfo: %s\n", strerror(errno));
        retVal = -PW_ERROR;
    } else {
        size_t len = 0;
        ssize_t read = 0;

        while ((read = pwr::StringUtils::getline(fp, str_line)) != -1) {
            if (read < 3) {
                continue;
            }
            //line[read-1] = '\0';
            const char *line = str_line.c_str(); 
            /*
             * Format is:
             * proc # <space> physical id <space> siblings <space> core id <space> cores <space>"
             */
            if (sscanf(line, "processor       : %d", &proc)) {
                topology << proc << " ";
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

    for (std::map<int,int_vec_t>::iterator iter = tmp_map.begin(); iter != tmp_map.end(); ++iter) {
        int_vec_t procs = iter->second;
        /*
         * We assign the first LCPU in the list as the core ID.
         */
        int core_id = procs[0];
        cpu_map[core_id] = core_id;
        for (u32 i=1; i<procs.size(); ++i) {
            /*
             * Subsequent LCPUs in this list are assigned to the same core.
             */
            cpu_map[procs[i]] = core_id;
        }
    }
    
    return topology.str();
};
/*
 * (Thin) wrapper around the wuwatch SIGINT signal handler.
 */
void sigint_handler_wrapper(int signum)
{
    g_wuwatchObj->sig_handler(signum);
};
/*
 * (Thin) wrapper around the wuwatch SIGHUP signal handler.
 */
void sighup_handler_wrapper(int signum)
{
    g_wuwatchObj->sig_handler(signum);
};
/*
 * Handle SIGINT (^-C) or SIGHUP.
 * Note: this supersedes the handler defined
 * in 'hook_lib' because this is registered AFTER
 * that one. Effectively, then, while programs-of-interest
 * will have SIGINT/SIGHUP trapped by the 'hook_lib', 'wuwatch'
 * will trap (and handle) SIGINT/SIGHUP on its own.
 */
void Wuwatch::sig_handler(int signum)
{
    fprintf(stderr, "wuwatch SIGNAL %d received!\n", signum);
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
        wu_exit(-PW_ERROR);
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
            return -PW_ERROR;
        }
        const char *dname = ent->d_name;
        if (*dname > '0' && *dname <= '9') { // we assume no malformed names!
            pid_name = std::string(dname);
            return PW_SUCCESS;
        }
    }
    return -PW_ERROR;
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
    int retVal = PW_SUCCESS;

    std::ifstream in_stream(path.c_str());

    if (!in_stream.is_open()) {
        db_fprintf(stderr, "Error: could NOT open file %s: %s\n", path.c_str(), strerror(errno));
        return -PW_ERROR;
    }

    if (!in_stream.eof())
        std::getline(in_stream, line);
    else {
        db_fprintf(stderr, "MALFORMED comm file: path = %s!\n", path.c_str());
        retVal = -PW_ERROR;
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
    if (from == (int)std::string::npos || to == (int)std::string::npos)
        return -PW_ERROR;
    ++from;
    proc_name = line.substr(from, (to-from));
    return PW_SUCCESS;
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
        return PW_SUCCESS;
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
            return -PW_ERROR;
        if (parse_stat_line_i(line, proc_name)) {
            db_fprintf(stderr, "ERROR: MALFORMED stat file %s\n", path.c_str());
            return -PW_ERROR;
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

    return PW_SUCCESS;
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
        return -PW_ERROR;
    }
    for (ent = readdir(dir); ent; ent = readdir(dir)) {
        if (!strcmp(ent->d_name, ".") || !strcmp(ent->d_name, ".."))
            continue;
        tasks.push_back(ent->d_name);
    }
    closedir(dir);

    return PW_SUCCESS;
};
#define GET_CONSTANT_POOL_INDEX(pool, index, str, out) do { pw_u32_t __idx; \
        db_fprintf(stderr, "Retrieving idx for str = %s\n", (str)); \
        if ((pool).find(str) == (pool).end()) { \
            __idx = ++(index); \
            (pool)[str] = __idx; \
        } else { \
            __idx = (pool)[(str)]; \
        } \
        out = __idx;} while (0)

static constant_pool_msg_t *convert_wakelock_to_constant_pool_i(constant_pool_msg_t *cp_msg, w_wakelock_msg_t *w_msg, u_wakelock_msg_t *u_msg, size_t *msg_len, const PWCollector_sample_t *sample)
{
    static int constant_pool_index = -1;
    const w_sample_t *ws = NULL;
    const u_sample_t *us = NULL;
    size_t len = 0;
    const char *wl_name = NULL;
    int cp_index = -1;
    static std::map <std::string, pw_u32_t> constantPool;

    switch (sample->sample_type) {
        case W_STATE:
            ws = &sample->w_sample;
            wl_name = ws->name;
            GET_CONSTANT_POOL_INDEX(constantPool, constant_pool_index, wl_name, cp_index);
            /*
             * Tell the post-processing algo that this is a "PW_WAKE_LOCK_INITIAL" W_STATE sample.
             */
            assert(ws->type == PW_WAKE_LOCK_INITIAL);
            cp_index = PW_SET_INITIAL_W_STATE_MAPPING_MASK(cp_index);
            db_fprintf(stderr, "%s has pool index = %d\n", wl_name, cp_index);
            w_msg->type = ws->type;
            w_msg->tid = ws->tid;
            w_msg->pid = ws->pid;
            w_msg->constant_pool_index = cp_index; // Should point to whatever is in the 'constant_pool_msg' above
            w_msg->expires = ws->expires;
            memcpy(w_msg->proc_name, ws->proc_name, PW_MAX_PROC_NAME_SIZE); // process name
            break;
        case U_STATE:
            us = &sample->u_sample;
            wl_name = us->tag;
            GET_CONSTANT_POOL_INDEX(constantPool, constant_pool_index, wl_name, cp_index);
            u_msg->type = us->type;
            u_msg->flag = us->flag;
            u_msg->pid = us->pid;
            u_msg->uid = us->uid;
            u_msg->count = us->count;
            u_msg->constant_pool_index = cp_index;
            break;
        default:
            assert(false);
         
    }
    if (wl_name == NULL) {
        if (cp_msg) {
            free(cp_msg);
            cp_msg = NULL;
        }
        return NULL;
    }
    len = strlen(wl_name);
    *msg_len = PW_CONSTANT_POOL_MSG_HEADER_SIZE + len + 1;
    constant_pool_msg_t *tmp_msg = (constant_pool_msg_t *)realloc(cp_msg, *msg_len);
    if (tmp_msg == NULL && cp_msg) {
        free(cp_msg);
    }
    cp_msg = tmp_msg;
    if (cp_msg) {
        cp_msg->entry_type = sample->sample_type; // This is either a KERNEL or USERSPACE wakelock constant pool mapping
        cp_msg->entry_len = (pw_u16_t)len;
        cp_msg->entry_index = cp_index;
        memcpy(cp_msg->entry, wl_name, len+1);
    }
    return cp_msg;
};
/*
 * INTERNAL API:
 * "Serialize" data by converting it from wuwatch v3.0.0 format to wuwatch v3.1.0 format.
 *
 * @from, @to: the start, stop limits of the data to "serialize".
 * @data: (reference to) converted data.
 *
 * @returns: 0 on success, -1 on error
 */
template <typename InputIterator> static int serialize_i(InputIterator from, InputIterator to, std::vector<char>& data)
{
    InputIterator dummy;
    char __tmp[sizeof(*dummy)];
    PWCollector_msg_t msg;
    int payload_size = 0;
    char *p_payload = NULL;
    constant_pool_msg_t *cp_msg = NULL;
    w_wakelock_msg_t w_msg;
    u_wakelock_msg_t u_msg;
    int retVal = PW_SUCCESS;
    size_t msg_len;

    for(; from != to; ++from) {
        PWCollector_sample_t sample = *from;
        msg.cpuidx = sample.cpuidx;
        msg.tsc = sample.tsc;
        msg.data_type = sample.sample_type;

        switch (sample.sample_type) {
            case M_MAP:
                msg.data_len = payload_size = sizeof(sample.m_sample);
                p_payload = (char *)&sample.m_sample;
                break;
            case PROC_MAP:
                msg.data_len = payload_size = sizeof(sample.r_sample);
                p_payload = (char *)&sample.r_sample;
                break;
            case DEV_MAP:
                msg.data_len = payload_size = sizeof(sample.dev_sample);
                p_payload = (char *)&sample.dev_sample;
                break;
            case PKG_MAP:
                msg.data_len = payload_size = sizeof(sample.pkg_sample);
                p_payload = (char *)&sample.pkg_sample;
                break;
            case W_STATE:
            case U_STATE:
                /*
                 * Check if the sample.w_sample.name (or sample.u_sample.tag) is in our constant pool. If not, add it.
                 * Retrieve it's index into the pool.
                 */
                cp_msg = convert_wakelock_to_constant_pool_i(cp_msg, &w_msg, &u_msg, &msg_len, &sample);
                /*
                 * We need to send TWO messages; the first is for the constant pool entry...
                 */
                msg.data_type = CONSTANT_POOL_ENTRY;
                msg.data_len = payload_size = msg_len;
                p_payload = (char *)cp_msg;
                db_fprintf(stderr, "Converted to [%d: %s]; # bytes = %u\n", cp_msg->entry_index, cp_msg->entry, msg_len);
                memcpy(__tmp, &msg, PW_MSG_HEADER_SIZE);
                memcpy(__tmp + PW_MSG_HEADER_SIZE, p_payload, payload_size);
                data.insert(data.end(), __tmp, __tmp + (PW_MSG_HEADER_SIZE + payload_size));
                db_fprintf(stderr, "OK: wrote constant pool info; now writing wakelock msg...\n");
                /*
                 * ...and then the actual wakelock sample itself.
                 */
                msg.data_type = sample.sample_type;
                if (sample.sample_type == W_STATE) {
                    msg.data_len = payload_size = sizeof(w_msg);
                    p_payload = (char *)&w_msg;
                } else {
                    msg.data_len = payload_size = sizeof(u_msg);
                    p_payload = (char *)&u_msg;
                }
                break;
            default:
                fprintf(stderr, "ERROR: unsupported sample type = %d in serialize!\n", sample.sample_type);
                retVal = -PW_ERROR;
                goto out;
        }
        memcpy(__tmp, &msg, PW_MSG_HEADER_SIZE);
        memcpy(__tmp + PW_MSG_HEADER_SIZE, p_payload, payload_size);
        data.insert(data.end(), __tmp, __tmp + (PW_MSG_HEADER_SIZE + payload_size));
    }
out:
    if (cp_msg) {
        free(cp_msg);
    }
    return retVal;
};
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
    size_t num_samples = 0;
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
        for (int i = get_next_pid_i(proc_dir, pid_name); i == PW_SUCCESS; i = get_next_pid_i(proc_dir, pid_name)) {
            pid_name_pair_t pair;
            str_vec_t tasks;
            get_all_tasks_for_proc_i(pid_name, tasks);
            extract_pid_tid_name_i(pid_name, tasks, pid_map, PROC_DIR_NAME, proc_samples);
        }
        closedir(proc_dir);
        
        /*
         * Dump ALL samples at the same time...
         */
        if ( (num_samples = proc_samples.size()) > 0) {
            if (WUWATCH_VERSION(m_driver_major, m_driver_minor, m_driver_other) >= WUWATCH_VERSION(3, 1, 0)) {
                /*
                 * We expect new, 'PWCollector_msg' samples -- convert the existing samples
                 * to this new format before writing to disk.
                 */
                std::vector <char> buffer;
                if (serialize_i(proc_samples.begin(), proc_samples.end(), buffer)) {
                    db_fprintf(stderr, "ERROR serializing samples!\n");
                    wu_exit(-PW_ERROR);
                }
                db_fprintf(stderr, "OK: serialized proc map samples: size = %d (# proc samples = %d)\n", buffer.size(), proc_samples.size());
                if (fwrite(&buffer[0], sizeof(char), buffer.size(), fp) < buffer.size()) {
                    perror("fwrite error");
                    wu_exit(-PW_ERROR);
                }
            } else {
                if (fwrite(&proc_samples[0], sizeof(PWCollector_sample_t), num_samples, fp) < num_samples) {
                    perror("fwrite error");
                    wu_exit(-PW_ERROR);
                }
                db_fprintf(stderr, "OK: wrote %u samples!\n", (unsigned int)num_samples);
            }
        }
    } else {
        char error_msg[1024];
        sprintf(error_msg, "opendir error in wuwatch::cleanup_collection; could not open \"%s\" directory", PROC_DIR_NAME);
        db_fprintf(stderr, "%s: %s\n", error_msg, strerror(errno));
        return -PW_ERROR;
    }
    return PW_SUCCESS;
}

/*
 * INTERNAL API:
 * Retrieve a list of device number <-> name mappings.
 * Applicable to MFD/CLV only.
 *
 * @fp: the output file pointer
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::get_initial_dev_map_i(FILE *fp)
{
    std::vector <PWCollector_sample_t> dev_samples;
    int num_samples = 0;
    int max_lss_num_in_nc = 0;
    int max_lss_num_in_sc = 0;
    if (PW_IS_MFD(m_cpuModel)) {
        max_lss_num_in_nc = MFD_MAX_LSS_NUM_IN_NC;
        max_lss_num_in_sc = MFD_MAX_LSS_NUM_IN_SC;
    } else if (PW_IS_CLV(m_cpuModel)) {
        max_lss_num_in_nc = CLV_MAX_LSS_NUM_IN_NC;
        max_lss_num_in_sc = CLV_MAX_LSS_NUM_IN_SC;
    }

    for (int i=0; i<max_lss_num_in_nc; ++i) {
        PWCollector_sample_t sample;
        dev_sample_t *dev = &sample.dev_sample;

        memset(&sample, 0, sizeof(sample));
        /*
         * These fields are common to ALL DEV_MAP samples.
         */
        sample.sample_len = sample.cpuidx = 0;
        sample.tsc = rdtsc();
        sample.sample_type = DEV_MAP;

        dev->dev_num = i;
        dev->dev_type = PW_NORTH_COMPLEX;

        if (PW_IS_MFD(m_cpuModel)) {
            memcpy(dev->dev_short_name, pw_mfd_nc_device_names[i][0], sizeof(dev->dev_short_name));
            memcpy(dev->dev_long_name, pw_mfd_nc_device_names[i][1], sizeof(dev->dev_long_name));
        } else if (PW_IS_CLV(m_cpuModel)) {
            memcpy(dev->dev_short_name, pw_clv_nc_device_names[i][0], sizeof(dev->dev_short_name));
            memcpy(dev->dev_long_name, pw_clv_nc_device_names[i][1], sizeof(dev->dev_long_name));
        }

        dev_samples.push_back(sample);
    }
    for (int i=0; i<max_lss_num_in_sc; ++i) {
        PWCollector_sample_t sample;
        dev_sample_t *dev = &sample.dev_sample;

        memset(&sample, 0, sizeof(sample));
        /*
         * These fields are common to ALL DEV_MAP samples.
         */
        sample.sample_len = sample.cpuidx = 0;
        sample.tsc = rdtsc();
        sample.sample_type = DEV_MAP;

        dev->dev_num = i;
        dev->dev_type = PW_SOUTH_COMPLEX;

        if (PW_IS_MFD(m_cpuModel)) {
            memcpy(dev->dev_short_name, pw_mfd_sc_device_names[i][0], sizeof(dev->dev_short_name));
            memcpy(dev->dev_long_name, pw_mfd_sc_device_names[i][1], sizeof(dev->dev_long_name));
        } else if (PW_IS_CLV(m_cpuModel)) {
            memcpy(dev->dev_short_name, pw_clv_sc_device_names[i][0], sizeof(dev->dev_short_name));
            memcpy(dev->dev_long_name, pw_clv_sc_device_names[i][1], sizeof(dev->dev_long_name));
        }

        dev_samples.push_back(sample);
    }
    num_samples = dev_samples.size();
    if (num_samples > 0) {
        if (WUWATCH_VERSION(m_driver_major, m_driver_minor, m_driver_other) >= WUWATCH_VERSION(3, 1, 0)) {
            /*
             * We expect new, 'PWCollector_msg' samples -- convert the existing samples
             * to this new format before writing to disk.
             */
            std::vector <char> buffer;
            if (serialize_i(dev_samples.begin(), dev_samples.end(), buffer)) {
                db_fprintf(stderr, "ERROR serializing samples!\n");
                wu_exit(-PW_ERROR);
            }
            db_fprintf(stderr, "OK: serialized dev map samples: size = %d (# dev samples = %d)\n", buffer.size(), dev_samples.size());
            if (fwrite(&buffer[0], sizeof(char), buffer.size(), fp) < buffer.size()) {
                perror("fwrite error");
                wu_exit(-PW_ERROR);
            }
        } else {
            if (fwrite(&dev_samples[0], sizeof(PWCollector_sample_t), num_samples, fp) != num_samples) {
                perror("fwrite error in get_initial_dev_map_i()");
                wu_exit(-PW_ERROR);
            }
        }
    }
    return PW_SUCCESS;
};
/*
 * INTERNAL API:
 * Helper function to extract wakelocks from the "/proc/wakelocks" file system.
 *
 * @returns: 0 on success, -1 on error.
 */
int Wuwatch::extract_wakelock_name_i(std::vector<PWCollector_sample_t>& wl_samples)
{
    std::string wl_path = "/proc/wakelocks";
    str_vec_t wl_names;
    w_sample_t ws;
    PWCollector_sample_t sample;
    std::string line;

    std::ifstream in_stream(wl_path.c_str());

    if (!in_stream.is_open()) {
        db_fprintf(stderr, "Error: could NOT open file %s: %s\n", wl_path.c_str(), strerror(errno));
        return -PW_ERROR;
    }

    while (!in_stream.eof()) {
        std::getline(in_stream, line);


        str_vec_t tokens;
        std::istringstream sstream(line);
        std::string token;
        while(!sstream.eof()) {
            std::getline(sstream, token, '\t');
            tokens.push_back(token); 
        }
   
        // The fifth column (non-zero) indicates whether the lock is active or not.
        if (tokens.size() < 5) {
            break; 
        }

        if(tokens[4].compare("active_since") && tokens[4].compare("0")) {
            if (tokens[0].size() > 3) {
                wl_names.push_back(tokens[0].substr(1, tokens[0].size()-2));
                db_fprintf(stderr, "WAKE_LOCK_INITIAL: %s\n", tokens[0].substr(1, tokens[0].size()-2).c_str());
            }
        }
    }
    in_stream.close();

    /*
     * These fields are common to ALL (initial)
     * wakelock samples.
     */
    sample.sample_len = sample.cpuidx = 0;
    sample.tsc = rdtsc();
    sample.sample_type = W_STATE;

    for (str_vec_t::const_iterator iter = wl_names.begin(); iter != wl_names.end(); ++iter) {
        ws.type = PW_WAKE_LOCK_INITIAL;
        ws.tid = ws.pid = 0;
        memset(ws.proc_name, 0, sizeof(ws.proc_name));
        memcpy(ws.name, iter->c_str(), sizeof(ws.name));
        sample.w_sample = ws;
        wl_samples.push_back(sample);
    }

    return PW_SUCCESS;
}
/*
 * INTERNAL API:
 * Parse the "/proc/wakelocks" psuedo-filesystem
 * to extract active wakelocks
 *
 * @fp: the output file pointer
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::get_active_wakelocks_i(FILE *fp)
{
    std::vector<PWCollector_sample_t> wl_samples;
    size_t num_samples = 0;

    if (extract_wakelock_name_i(wl_samples) == PW_SUCCESS) {
        /*
         * Dump ALL samples at the same time...
         */
        if ( (num_samples = wl_samples.size()) > 0) {
            if (WUWATCH_VERSION(m_driver_major, m_driver_minor, m_driver_other) >= WUWATCH_VERSION(3, 1, 0)) {
                /*
                 * We expect new, 'PWCollector_msg' samples -- convert the existing samples
                 * to this new format before writing to disk.
                 */
                std::vector <char> buffer;
                if (serialize_i(wl_samples.begin(), wl_samples.end(), buffer)) {
                    db_fprintf(stderr, "ERROR serializing samples!\n");
                    wu_exit(-PW_ERROR);
                }
                db_fprintf(stderr, "OK: serialized wakelock samples: size = %d (# w samples = %d)\n", buffer.size(), wl_samples.size());
                if (fwrite(&buffer[0], sizeof(char), buffer.size(), fp) < buffer.size()) {
                    perror("fwrite error");
                    wu_exit(-PW_ERROR);
                }
            } else {
                if (fwrite(&wl_samples[0], sizeof(PWCollector_sample_t), num_samples, fp) < num_samples) {
                    perror("fwrite error");
                    wu_exit(-PW_ERROR);
                }
                db_fprintf(stderr, "OK: wrote %u samples!\n", (unsigned int)num_samples);
            }
        }
    } else {
        char error_msg[1024];
        sprintf(error_msg, "Could not read active wakelocks from /proc/wakelocks");
        db_fprintf(stderr, "%s: %s\n", error_msg, strerror(errno));
        return -PW_ERROR;
    }
    return PW_SUCCESS;
}
/*
 * INTERNAL API:
 * Parse the "/data/system/packages.list"
 * to extract uid and package mappings
 *
 * @fp: the output file pointer
 *
 * @returns: 0 on success, -1 on error
 */
int Wuwatch::get_uid_pkg_map_i(FILE *fp)
{
    std::vector<PWCollector_sample_t> pkg_samples;
    std::string pkgmap_path = "/data/system/packages.list";
    std::string line;

    /*
     * Add the predefined UID and package mappings first
     */
    for (int i=0; i<android_predefined_uid_pkg_num; i++) {
        PWCollector_sample_t sample;
        pkg_sample_t *ps = &sample.pkg_sample;

        memset(&sample, 0, sizeof(sample));
        /*
         * These fields are common to ALL DEV_MAP samples.
         */
        sample.sample_len = sample.cpuidx = 0;
        sample.tsc = rdtsc();
        sample.sample_type = PKG_MAP;

        ps->uid = (u32)atoi(android_predefined_uid_pkg_mappings[i][0]);
        memcpy(ps->pkg_name, android_predefined_uid_pkg_mappings[i][1], sizeof(ps->pkg_name));
        pkg_samples.push_back(sample);
        db_fprintf(stderr, "UID-Package name map: <%s, %s>\n", android_predefined_uid_pkg_mappings[i][0], 
                android_predefined_uid_pkg_mappings[i][1]);
    }

    /*
     * Add the UID and package mappings for pure user applications
     */
    std::ifstream in_stream(pkgmap_path.c_str());

    if (!in_stream.is_open()) {
        db_fprintf(stderr, "Error: could NOT open file %s: %s\n", pkgmap_path.c_str(), strerror(errno));
        return -PW_ERROR;
    }

    while (!in_stream.eof()) {
        std::getline(in_stream, line);

        str_vec_t tokens;
        std::istringstream sstream(line);
        std::string token;
        while(!sstream.eof()) {
            std::getline(sstream, token, ' ');
            tokens.push_back(token); 
        }
   
        // The first column should be the package name and the second column should be UID.
        if (tokens.size() > 1) {
            PWCollector_sample_t sample;
            pkg_sample_t *ps = &sample.pkg_sample;

            memset(&sample, 0, sizeof(sample));
            /*
             * These fields are common to ALL DEV_MAP samples.
             */
            sample.sample_len = sample.cpuidx = 0;
            sample.tsc = rdtsc();
            sample.sample_type = PKG_MAP;

            ps->uid = (u32)atoi(tokens[1].c_str());
            memcpy(ps->pkg_name, tokens[0].c_str(), sizeof(ps->pkg_name));
            pkg_samples.push_back(sample);
            db_fprintf(stderr, "UID-Package name map: <%s, %s>\n", tokens[1].c_str(), tokens[0].c_str());
        }
    }
    in_stream.close();

    size_t num_samples = 0;

    if ( (num_samples = pkg_samples.size()) > 0) {
        if (WUWATCH_VERSION(m_driver_major, m_driver_minor, m_driver_other) >= WUWATCH_VERSION(3, 1, 0)) {
            /*
             * We expect new, 'PWCollector_msg' samples -- convert the existing samples
             * to this new format before writing to disk.
             */
            std::vector <char> buffer;
            if (serialize_i(pkg_samples.begin(), pkg_samples.end(), buffer)) {
                db_fprintf(stderr, "ERROR serializing samples!\n");
                wu_exit(-PW_ERROR);
            }
            db_fprintf(stderr, "OK: serialized uid package map samples: size = %d (# pkg samples = %d)\n", buffer.size(), pkg_samples.size());
            if (fwrite(&buffer[0], sizeof(char), buffer.size(), fp) < buffer.size()) {
                perror("fwrite error");
                wu_exit(-PW_ERROR);
            }
        } else {
            if (fwrite(&pkg_samples[0], sizeof(PWCollector_sample_t), num_samples, fp) < num_samples) {
                perror("fwrite error");
                wu_exit(-PW_ERROR);
            }
            db_fprintf(stderr, "OK: wrote %u samples!\n", (unsigned int)num_samples);
        }
    }

    return PW_SUCCESS;
}
/*
 * INTERNAL API:
 * Produce user wakelock samples by reading Android system logs in /data/logs/aplog or /logs/aplog.
 */
void Wuwatch::produce_user_wakelock_samples(FILE *outfp)
{
    std::vector <PWCollector_sample_t> userwakelock_samples;
    std::string str_line;
    //char *line = NULL;
    int month, day, hour, minute, second, millisec;
    int pid, tid;
    char flag;
    char str[1024];
    int retVal = PW_SUCCESS;
    int num_samples = 0;
    struct dirent *ent = NULL;
    str_vec_t aplogs;
    DIR *dir = NULL;
    FILE *fp = NULL;
    std::string logdirname = "/logs/"; 

    fp = fopen("/logs/aplog", "r");
    if (!fp) {
        db_fprintf(stderr, "fopen error for %saplog: %s\n", logdirname.c_str(), strerror(errno));
        fp = fopen("/data/logs/aplog", "r");
        logdirname = "/data/logs/"; 
        if (!fp) {
            fprintf(stderr, "fopen error for aplog: %s\n", strerror(errno));
            retVal = -PW_ERROR;
        } 
    }

    dir = opendir(logdirname.c_str());

    if (!dir) {
        perror("ERROR opening log dir");
        wu_exit(-PW_ERROR);
    }
    for (ent = readdir(dir); ent; ent = readdir(dir)) {
        if (ent->d_type != DT_UNKNOWN && ent->d_type != DT_DIR
            && !strncmp(ent->d_name, "aplog.", 6)) {

            /*
             * Get start and end time values for each aplog file
             * and compare them with collection time values
             * so that we know which log files have wakelock data
             */
#if 0
            char aplogfile[128];
            sprintf(aplogfile, "%s%s", logdirname.c_str(), ent->d_name);

            FILE *logfp = fopen(aplogfile, "r");
            if (logfp) {
                size_t len = 0;
                ssize_t read = 0;
                char firstline[512];
                char lastline[512];

                while ((read = pwr::StringUtils::getline(logfp, str_line)) != -1) {
                    //line[read-1] = '\0';
                    const char *line = str_line.c_str();
                    if (read > 14 && line[2] == '-' && line[5] == ' ') {
                        memcpy(firstline, line, 512);
                        break;
                    }
                }
                while ((read = pwr::StringUtils::getline(logfp, str_line)) != -1) {
                    //line[read-1] = '\0';
                    const char *line = str_line.c_str();
                    if (read > 14 && line[2] == '-' && line[5] == ' ') {
                        memcpy(lastline, line, 512);
                        break;
                    }
                }

                if (strncmp(firstline, m_initialTimeStr, 14) <= 0 &&
                    strncmp(lastline, m_initialTimeStr, 14) >= 0) {
                    aplogs.push_back(ent->d_name);
                    printf("Selected aplog file: %s\n", aplogfile);
                }
                fclose(logfp);
            }
#endif

            aplogs.push_back(ent->d_name);
        }
    }
    closedir(dir);

    aplogs.push_back("aplog");

    for (str_vec_t::const_iterator iter = aplogs.begin(); iter != aplogs.end(); ++iter) {
        char aplogfile[128];
        snprintf(aplogfile, sizeof(aplogfile), "%s%s", logdirname.c_str(), iter->c_str());
        FILE *fp = fopen(aplogfile, "r");
        if (!fp) {
            fprintf(stderr, "fopen error for %s: %s\n", aplogfile, strerror(errno));
            retVal = -PW_ERROR;
        } else {
            size_t len = 0;
            ssize_t read = 0;

            while ((read = pwr::StringUtils::getline(fp, str_line)) != -1) {
                if (read < 1) {
                    continue;
                }
                //line[read-1] = '\0';
                const char *line = str_line.c_str();
                if (sscanf(line, "%2d-%2d %2d:%2d:%2d.%3d %d %d %c %s", &month, &day, &hour, &minute, &second, &millisec, 
                    &pid, &tid, &flag, str)) {
                    if (!strncmp(str, "WAKELOCK_", 9)) {
                        str_line = str_line.substr(51);
                        std::string tag, flagstr;
                        unsigned long long ts;
                        u_sample_flag_t flag;
                        u_sample_type_t type;
                        int count;
                        int pid;
                        int uid;
                        char timestr[20];

                        snprintf(timestr, sizeof(timestr), "%02d-%02d %02d:%02d:%02d", month, day, hour, minute, second);
                        sscanf(str_line.substr(0, str_line.find(',')).c_str(), "TIMESTAMP=%llu", &ts);
                        str_line = str_line.substr(str_line.find(',')+2);
                        tag = str_line.substr(4, str_line.find(',')-4).c_str();
                        str_line = str_line.substr(str_line.find(',')+2);
                        flagstr = str_line.substr(5, str_line.find(',')-5).c_str();
                        str_line = str_line.substr(str_line.find(',')+2);
                        sscanf(str_line.substr(0, str_line.find(',')).c_str(), "COUNT=%d", &count);
                        str_line = str_line.substr(str_line.find(',')+2);
                        sscanf(str_line.substr(0, str_line.find(',')).c_str(), "PID=%d", &pid);
                        str_line = str_line.substr(str_line.find(',')+2);
                        sscanf(str_line.substr(0, str_line.find('\n')).c_str(), "UID=%d", &uid);

                        //fprintf(stderr,"%llu, %llu, %llu, %s, %s\n", m_initialTime, ts, m_finalTime, timestr, m_initialTimeStr);

                        if (ts >= m_initialTime && ts <= m_finalTime
                            && strncmp(timestr, m_initialTimeStr, 14) >= 0) {
                         
                            db_fprintf(stderr,"%s, %llu, %s, %s, %d, %d, %d\n", str, ts, tag.c_str(), flagstr.c_str(), count, pid, uid);
                            if (!strncmp(str, "WAKELOCK_ACQUIRE", 16)) {
                                type = PW_WAKE_ACQUIRE;
                            } else if (!strncmp(str, "WAKELOCK_RELEASE", 16)) {
                                type = PW_WAKE_RELEASE;
                            }
                            if (!strncmp(flagstr.c_str(), "PARTIAL_WAKE_LOCK", 17)) {
                                flag = PW_WAKE_PARTIAL;
                            } else if (!strncmp(flagstr.c_str(), "FULL_WAKE_LOCK", 14)) {
                                flag = PW_WAKE_FULL;
                            } else if (!strncmp(flagstr.c_str(), "SCREEN_DIM_WAKE_LOCK", 20)) {
                                flag = PW_WAKE_SCREEN_DIM;
                            } else if (!strncmp(flagstr.c_str(), "SCREEN_BRIGHT_WAKE_LOCK", 23)) {
                                flag = PW_WAKE_SCREEN_BRIGHT;
                            } else if (!strncmp(flagstr.c_str(), "PROXIMITY_SCREEN_OFF_WAKE_LOCK", 30)) {
                                flag = PW_WAKE_PROXIMITY_SCREEN_OFF;
                            }

                            PWCollector_sample_t sample;
                            u_sample_t *uw = &sample.u_sample;

                            memset(&sample, 0, sizeof(sample));
                            sample.sample_len = sample.cpuidx = 0;
                            sample.tsc = (ts - m_initialTime) / 1000.0 * tsc_freq_MHz + m_initialTSC;
                            sample.sample_type = U_STATE;

                            uw->type = type;
                            uw->flag = flag;
                            uw->count = count;
                            uw->pid = pid;
                            uw->uid = uid;
                            memcpy(uw->tag, tag.c_str(), sizeof(uw->tag));
                            userwakelock_samples.push_back(sample);
                        }
                    }
                }
            }
        }
        fclose(fp);
    }

    num_samples = userwakelock_samples.size();

    if (num_samples > 0) {
        if (WUWATCH_VERSION(m_driver_major, m_driver_minor, m_driver_other) >= WUWATCH_VERSION(3, 1, 0)) {
            /*
             * We expect new, 'PWCollector_msg' samples -- convert the existing samples
             * to this new format before writing to disk.
             */
            std::vector <char> buffer;
            if (serialize_i(userwakelock_samples.begin(), userwakelock_samples.end(), buffer)) {
                db_fprintf(stderr, "ERROR serializing samples!\n");
                wu_exit(-PW_ERROR);
            }
            db_fprintf(stderr, "OK: serialized userwakelock samples: size = %d (# userwakelock samples = %d)\n", buffer.size(), userwakelock_samples.size());
            if (fwrite(&buffer[0], sizeof(char), buffer.size(), outfp) < buffer.size()) {
                perror("fwrite error");
                wu_exit(-PW_ERROR);
            }
        } else {
            if (fwrite(&userwakelock_samples[0], sizeof(PWCollector_sample_t), num_samples, outfp) != num_samples) {
                perror("fwrite error in produce_user_wakelock_samples()");
                wu_exit(-PW_ERROR);
            }
        }
    }
};

/*
 * EXTERNAL API:
 * Perform wuwatch-specific setup.
 */
void Wuwatch::setup_collection()
{
    /*
     * Get # of processors in system.
     */
    // max_num_cpus = sysconf(_SC_NPROCESSORS_CONF);
    max_num_cpus = sysconf(_SC_NPROCESSORS_ONLN);

    signal(SIGINT, sigint_handler_wrapper);
    signal(SIGHUP, sighup_handler_wrapper);

    db_fprintf(stderr, "# cpus = %d\n", max_num_cpus);

    /*
     * Make sure we have a valid config file!
     */
    assert(m_config_file_path.size());

    /*
     * Make sure we know where to write our results!
     */
    assert(m_output_file_name.size());

    /*
     * Get target FMS, architecture.
     */
    {
        retrieve_target_arch_details_i();
        db_fprintf(stderr, "TARGET FMS = %u.%u.%u\n", m_cpuFamily, m_cpuModel, m_cpuStepping);
    }
    /*
     * Parse the config file.
     */
    {
        m_targetArchRec = PWParser::get_c_states_info(m_config_file_path, m_cpuFamily, m_cpuModel, m_cpuStepping);
        if (m_targetArchRec == NULL) {
            db_fprintf(stderr, "ERROR retrieving target arch rec!\n");
            wu_exit(-PW_ERROR);
        } else if (g_do_debugging) {
            std::cerr << (*m_targetArchRec) << std::endl;
        }
    }


    /*
     * Make sure we have a connection to the DD!
     */
    if (open_dd_i()) {
        db_fprintf(stderr, "ERROR: could NOT open connection to the device driver!\n");
        wu_exit(-PW_ERROR);
    }

    /*
     * Get driver version information.
     */
    {
        if (do_ioctl_driver_version_i(m_dev_fd)) {
            db_fprintf(stderr, "ERROR: could NOT retrieve DRIVER VERSION information!\n");
            wu_exit(-PW_ERROR);
        }
        db_fprintf(stderr, "DRIVER_VERSION: %s\n", m_driver_version_string.c_str());
    }
    /*
     * Get microcode patch version
     * (Only useful for MFLD).
     */
    if (PW_IS_SALTWELL(m_cpuModel)) {
        int patch_ver = -1;
        if (do_ioctl_micro_patch(m_dev_fd, patch_ver)) {
            db_fprintf(stderr, "ERROR: could NOT retrieve microcode patch version!\n");
            wu_exit(-PW_ERROR);
        }
        micro_patch_ver = patch_ver;
        db_fprintf(stderr, "PATCH VERSION = %d\n", micro_patch_ver);
    }
    /*
     * Get the driver buffer size. We need this if we want to 'mmap' the per-cpu
     * output buffers.
     */
    if (WUWATCH_VERSION(m_driver_major, m_driver_minor, m_driver_other) >= WUWATCH_VERSION(3, 1, 0)) {
        unsigned long mmap_size = 0, buff_size = 0;

        if (do_ioctl_i(m_dev_fd, PW_IOCTL_MMAP_SIZE, &mmap_size, sizeof(mmap_size), false) == NULL) { // "false" ==> OUTPUT param.
            wu_exit(-PW_ERROR);
        }
        if (do_ioctl_i(m_dev_fd, PW_IOCTL_BUFFER_SIZE, &buff_size, sizeof(buff_size), false) == NULL) { // "false" ==> OUTPUT param.
            wu_exit(-PW_ERROR);
        }
        db_fprintf(stderr, "OK: received mmap_size = %lu, buffer_size = %lu\n", mmap_size, buff_size);
        m_mmap_size = mmap_size; m_buff_size = buff_size;
        if (false) {
            db_fprintf(stderr, "Doing an MMAP...\n");
            m_mmap_addr = mmap(NULL, m_mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, m_dev_fd, 0);
            db_fprintf(stderr, "addr = 0x%lx\n", m_mmap_addr);
            if (m_mmap_addr == MAP_FAILED) {
                db_fprintf(stderr, "ERROR mapping driver buffers: %s\n", strerror(errno));
                wu_exit(-PW_ERROR);
            }
            db_fprintf(stderr, "OK: mmaped addr = %p\n", m_mmap_addr);
        }
    }

    /*
     * Open handle to output file(s). We have a single
     * file that contains driver output AND system
     * configuration information.
     */
    if ( (m_output_fp = fopen(m_output_file_name.c_str(), "wb")) == NULL) {
        fprintf(stderr, "ERROR creating wuwatch output file %s: %s\n", m_output_file_name.c_str(), strerror(errno));
        wu_exit(-PW_ERROR);
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
        wu_exit(-PW_ERROR);
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

        if (c_state_collection) {
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_SLEEP); /* Use TPS tracepoint */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_POWER_C_STATE); /* GET C state samples */
            if (m_do_collect_kernel_backtrace) {
                SET_COLLECTION_SWITCH(m_collectionSwitches, PW_KTIMER); /* GET K-SAMPLES */
            }
        }

        if (p_state_collection) {
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_FREQ); /* GET P state samples */
        }

        if (s_residency_collection) {
            /*
             * REQUIRES TPS.
             */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_SLEEP); /* Use TPS tracepoint */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_PLATFORM_RESIDENCY); /* GET S state residency counter samples */
        }

        if (d_residency_collection) {
            /*
             * REQUIRES TPS.
             */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_SLEEP); /* Use TPS tracepoint */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_DEVICE_SC_RESIDENCY); /* GET D state residency counter samples in south complex */
        }

        if (d_nc_state_collection) {
            /*
             * REQUIRES TPS.
             */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_SLEEP); /* Use TPS tracepoint */
            SET_COLLECTION_SWITCH(m_collectionSwitches, PW_DEVICE_NC_STATE); /* GET D state state samples in north complex */
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
            switches[PW_SLEEP] = 1; /* Use TPS tracepoint */
            switches[PW_DEVICE_SC_STATE] = 1; /* GET D state state samples in south complex */
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
            switches[PW_SLEEP] = 1; /* Use TPS tracepoint */
            switches[PW_PLATFORM_STATE] = 1; /* GET S state samples */
        }
#endif

        /*
         * Get some information on kernel and arch features.
         */
        u32 supported_kernel_features = 0, supported_arch_features = 0;
        if (do_ioctl_check_platform_i(m_dev_fd, supported_kernel_features, supported_arch_features)) {
            /*
             * For whatever reason, the IOCTL didn't succeed. We conservatively assume
             * that NO kernel/arch features are supported!
             */
            fprintf(stderr, "ERROR: kernel check for supported data returns an error!\n");
            supported_kernel_features = supported_arch_features = 0;
        }

        db_fprintf(stderr, "Supported arch features = %u\n", supported_arch_features);

        m_wasAnyThreadSet = (supported_arch_features & PW_ARCH_ANY_THREAD_SET) ? 1 : 0;
        m_wasAutoDemoteEnabled = (supported_arch_features & PW_ARCH_AUTO_DEMOTE_ENABLED) ? 1 : 0;

        if (m_wasAutoDemoteEnabled ) {
            db_fprintf(stderr, "ANY_THREAD is SET\n");
        } else {
            db_fprintf(stderr, "DEBUG: ANY_THREAD is NOT SET\n");
        }

        if (m_wasAnyThreadSet) {
            db_fprintf(stderr, "AUTO_DEMOTE is ENABLED\n");
        } else {
            db_fprintf(stderr, "AUTO_DEMOTE is NOT ENABLED\n");
        }

        if (w_state_collection) {
            /*
             * First, make sure the wakelock patch has been applied.
             */
            if (supported_kernel_features & PW_KERNEL_SUPPORTS_WAKELOCK_PATCH) {
                SET_COLLECTION_SWITCH(m_collectionSwitches, PW_WAKELOCK_STATE); /* GET Wakelock samples */
            } else {
                fprintf(stderr, "Warning: The OS is NOT configured to track kernel wakelocks; kernel wakelock data will NOT be collected!\n");
                RESET_COLLECTION_SWITCH(m_collectionSwitches, PW_WAKELOCK_STATE);
                w_state_collection = 0;
            }
        }

        do_ioctl_config(m_dev_fd, d_state_sample_interval_msecs);
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
     * Tell the post-processor how to map (North and South Complex) device numbers to names.
     * (But ONLY if Medfield).
     */
    if (PW_IS_SALTWELL(m_cpuModel) && (d_residency_collection == 1 ||
        d_nc_state_collection == 1) && get_initial_dev_map_i(m_output_fp)) {
        fprintf(stderr, "ERROR retrieving Dev # <-> Name mappings\n");
        wu_exit(-PW_ERROR);
    }


    /*
     * Init the barrier -- ensures the thread doesn't
     * execute until we want/need it to.
     */
/*
    if (pthread_barrier_init(&reader_barrier, NULL, 2)) {
        perror("pthread_barrier_init error");
        wu_exit(-PW_ERROR);
    }
*/
    if (s_barrier.init(2)) {
        fprintf(stderr, "barrier_init error");
        wu_exit(-PW_ERROR);
    }

    /*
     * Create the reader thread.
     */
    if (pthread_create(&reader_tid, NULL, &Wuwatch::reader_thread, new reader_thread_arg(this, new args(m_dev_fd, m_output_fp)))) {
        perror("pthread_create error");
        wu_exit(-PW_ERROR);
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
    const std::vector <CRec>& crecs = m_targetArchRec->get_c_states();
    int j = 0;
    /*
     * Special case the 'C0' mapping.
     */
    mapping[j++] = "C0";
    /*
     * Also need to special case the Saltwell mappings (OS can request C-states
     * that aren't tracked because they're basically aliases for S0ix states).
     */
    if (PW_IS_SALTWELL(m_cpuModel)) {
        mapping[j++] = "C1";
        mapping[j++] = "C2";
        mapping[j++] = "C4";
        mapping[j++] = "C6";
        mapping[j++] = "C7";
        mapping[j++] = "C8";
        mapping[j++] = "C9";
    } else {
        for (std::vector<CRec>::const_iterator citer = crecs.begin(); citer != crecs.end(); ++citer) {
            if (citer->tres > 0) {
                char __tmp[10];
                snprintf(__tmp, sizeof(__tmp), "C%d", citer->num);
                mapping[j++] = __tmp;
            }
        }
    }
    /*
    for (int i=0, j=0; i<MAX_MSR_ADDRESSES; ++i) {
        const CRec *crec = &crecs[i];
        if (crec->msr_addr > 0x0) {
            char __tmp[10];
            snprintf(__tmp, sizeof(__tmp), "C%d", i);
            mapping[j++] = __tmp;
        }
    }
    */
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
        wu_exit(-PW_ERROR);
    }

    /* 
     * Read Android system logs to extract user-level wakelock data
     */
    if (u_state_collection) {
        // produce user wakelock traces 
        produce_user_wakelock_samples(m_output_fp);
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
            wu_exit(-PW_ERROR);
        }
        /*
         * ...and then go back to the end of the file
         * (so that we can start writing the actual
         * system configuration).
         */
        if (fseek(m_output_fp, 0, SEEK_END)) {
            perror("fseek error");
            wu_exit(-PW_ERROR);
        }
        db_fprintf(stderr, "OK: wrote offset = %llu\n", TO_ULL(curr_pos));
        /*
         * Sanity test!
         */
        if (ftell(m_output_fp) != (long)curr_pos) {
            fprintf(stderr, "ftel = %ld, curr = %llu\n", ftell(m_output_fp), TO_ULL(curr_pos));
        }
        assert(ftell(m_output_fp) == (long)curr_pos);

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
            fprintf(m_output_fp, "Start TSC = %llu\n", TO_ULL(m_initialTSC));
            fprintf(m_output_fp, "Start Timeval = %llu\n", TO_ULL(initialTimeval));
            fprintf(m_output_fp, "Stop TSC = %llu\n", TO_ULL(m_finalTSC));
            fprintf(m_output_fp, "Start Time = %llu\n", m_initialTime);
            fprintf(m_output_fp, "Stop Time = %llu\n", m_finalTime);

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
#if _ANDROID_
                // On Android, we get the OS name differently.
                std::string android_os_name;
                get_android_version_i(android_os_name);
                fprintf(m_output_fp, "OS Name = %s(kernel:%s)\n", android_os_name.c_str(), un.release);
#else // !ANDROID
                fprintf(m_output_fp, "OS Name = %s\n", un.sysname);
#endif
                fprintf(m_output_fp, "OS Type = %s\n", un.machine);
                fprintf(m_output_fp, "OS Version = %s\n", un.release);
            } else {
                fprintf(m_output_fp, "OS Name = UNKNOWN\n");
                fprintf(m_output_fp, "OS Type = UNKNOWN\n");
                fprintf(m_output_fp, "OS Version = UNKNOWN\n");
            }

            // Hardware configuration
            fprintf(m_output_fp, "CPU Brand = %s\n", cpu_brand);
            fprintf(m_output_fp, "CPU Family = %u\n", m_cpuFamily);
            fprintf(m_output_fp, "CPU Model = %u\n", m_cpuModel);
            fprintf(m_output_fp, "CPU Stepping = %u\n", m_cpuStepping);
            fprintf(m_output_fp, "CPU C-states Clock Rate = %u\n", m_targetArchRec->get_cx_clock_rate());
            fprintf(m_output_fp, "CPU Bus Frequency (KHz) = %u\n", m_targetArchRec->get_bus_freq());
            fprintf(m_output_fp, "CPU Perf Status Bits = %u %u\n", m_targetArchRec->m_perf_bits_low, m_targetArchRec->m_perf_bits_high);
            fprintf(m_output_fp, "CPU C-states =");
            {
                const char *msr_type_str[] = {"Package", "Core", "Thread"};
                const std::vector <CRec>& crecs = m_targetArchRec->get_c_states();
                for (std::vector<CRec>::const_iterator citer = crecs.begin(); citer != crecs.end(); ++citer) {
                    fprintf(m_output_fp, " C%d %s", citer->num, msr_type_str[citer->m_type]);
                }
                /*
                for (int i=0; i<MAX_MSR_ADDRESSES; ++i) {
                    const CRec *crec = &crecs[i];
                    if (crec->msr_addr > 0x0) {
                        fprintf(m_output_fp, " C%d %s", i, msr_type_str[crec->m_type]);
                    }
                }
                */
                fprintf(m_output_fp, "\n");
            }
            fprintf(m_output_fp, "Turbo Threshold = %llu\n", (unsigned long long)turboThreshold * 1000);
            fprintf(m_output_fp, "Bus clock frequency (KHz) = %llu\n", (unsigned long long)m_targetArchRec->get_bus_freq());
            fprintf(m_output_fp, "Num CPUs = %d\n", max_num_cpus);
            fprintf(m_output_fp, "CPU Topology = %s\n", get_cpu_topology_i().c_str());
            // (5) TSC Frequency
            fprintf(m_output_fp, "TSC Frequency (MHz) = %u\n", tsc_freq_MHz);
            // (6) MICROCODE patch version (ONLY for MFLD!)
            if (PW_IS_SALTWELL(m_cpuModel)) {
                fprintf(m_output_fp, "Microcode patch version = %d\n", micro_patch_ver);
            }
            // (7) Auto-demote and Any_thread information
            {
                fprintf(m_output_fp, "Any thread bit set = %u\n", m_wasAnyThreadSet);
                fprintf(m_output_fp, "Auto demote enabled = %u\n", m_wasAutoDemoteEnabled);
            }
            // (8) Collection switches
            {
                fprintf(m_output_fp, "Collection switches = %u\n", m_collectionSwitches);
            }
            // (9) PROFILED application details
            fprintf(m_output_fp, "Profiled Application: pid = %d name = %s\n", profiled_app_pid, profiled_app_name.length() ? profiled_app_name.c_str() : "SYSTEM-COLLECTION-MODE");
            // (10) ACPI <-> Hardware C-STATE mappings
            fprintf(m_output_fp, "\nACPI <---> HARDWARE C-STATE Mappings:\n\n");
            {
                acpi_mapping_t mappings;
                extract_acpi_c_state_mappings(mappings);

                for (acpi_mapping_t::iterator iter = mappings.begin(); iter != mappings.end(); ++iter) {
                    fprintf(m_output_fp, "%d\t%s\n", (*iter).first, (*iter).second.c_str());
                }
            }
            // (11) list of target residencies.
            fprintf(m_output_fp, "\nTARGET RESIDENCIES:\n\n");
            {
                const std::vector <CRec>& crecs = m_targetArchRec->get_c_states();
                for (std::vector<CRec>::const_iterator citer = crecs.begin(); citer != crecs.end(); ++citer) {
                    if (citer->tres > 0) {
                        fprintf(m_output_fp, "C%d = %u\n", citer->num, CONVERT_US_TO_CLOCK_TICKS(citer->tres, tsc_freq_MHz));
                    }
                }
                /*
                for (int i=1; i<MAX_MSR_ADDRESSES; ++i) { // start from '1' because we never print out the 'C0' target residency
                    const CRec *crec = &crecs[i];
                    if (crec->msr_addr > 0x0) {
                        fprintf(m_output_fp, "C%d = %u\n", i, CONVERT_US_TO_CLOCK_TICKS(crec->tres, tsc_freq_MHz));
                    }
                }
                */
            }
            // (12) list of available frequencies.
            fprintf(m_output_fp, "\nAVAILABLE FREQUENCIES = ");
            {
                for (int i=0; i<PW_MAX_NUM_AVAILABLE_FREQUENCIES && m_available_frequencies[i] != 0; ++i) {
                    fprintf(m_output_fp, "%u ", m_available_frequencies[i]);
                }
                fprintf(m_output_fp, "\n");
            }
            // (13) Dropped sample statistics
            fprintf(m_output_fp, "\nTOTAL SAMPLES = %llu DROPPED SAMPLES = %llu\n", m_totalSamples, m_droppedSamples);
            // (14) list of descendents
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

    std::string uds_file_name_str = 
#if _ANDROID_
        WUWATCH_ANDROIDSTR_PATH;
#else // !ANDROID
        WUWATCH_UNIXSTR_PATH;
#endif
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
        wu_exit(-PW_ERROR);
    }


    // unlink(WUWATCH_UNIXSTR_PATH);
    unlink(uds_file_name.c_str());
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sun_family = AF_LOCAL;
    // strcpy(servaddr.sun_path, WUWATCH_UNIXSTR_PATH);
    strcpy(servaddr.sun_path, uds_file_name.c_str());

    if (bind(fork_listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) {
        perror("bind error");
        wu_exit(-PW_ERROR);
    }

    if (listen(fork_listenfd, WUWATCH_LISTENQ)) {
        perror("listen error");
        wu_exit(-PW_ERROR);
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
#endif // !_ANDROID_
}
/*
 * INTERNAL API:
 * Unlink the backing file used for fork "PRELOAD" requests.
 */
void Wuwatch::unlink_fork_listener(void)
{
#if !_ANDROID_
    unlink(uds_file_name.c_str());
#endif // !_ANDROID_
}
/*
 * INTERNAL API:
 * Initialize shmem area used to communicate with the PRELOAD "hook" library.
 */
void Wuwatch::init_shm()
{
#if !_ANDROID_
    struct stat buf;
    int len = sysconf(_SC_PAGE_SIZE), num = len / sizeof(shm_data_t);

    db_fprintf(stderr, "%d, %lu, %d\n", len, TO_UL(sizeof(shm_data_t)), num);

    int fd = shm_open(PW_SHM_PATH, O_RDWR | O_CREAT, PW_SHM_MODE);
    if (fd < 0) {
        perror("shm_open error");
        wu_exit(-PW_ERROR);
    }
    if (fstat(fd, &buf)) {
        perror("fstat error");
        wu_exit(-PW_ERROR);
    }
    db_fprintf(stderr, "LEN = %d\n", (int)buf.st_size);
    if (ftruncate(fd, len)) {
        perror("ftruncate error");
        shm_unlink(PW_SHM_PATH);
        wu_exit(-PW_ERROR);
    }
    if (fstat(fd, &buf)) {
        perror("fstat error");
        shm_unlink(PW_SHM_PATH);
        wu_exit(-PW_ERROR);
    }
    db_fprintf(stderr, "LEN = %d\n", (int)buf.st_size);

    if ( (fork_shm_data = (shm_data_t *)mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == NULL) {
        perror("mmap error");
        shm_unlink(PW_SHM_PATH);
        wu_exit(-PW_ERROR);
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
            wu_exit(-PW_ERROR);
        }
        fork_shm_data[0].count = 1; // "1" for wuwatch itself
    }
    for (int i=1; i<num; ++i) {
        if (sem_init(&fork_shm_data[i].sem, 1, 0)) {
            perror("sem_init error");
            shm_unlink(PW_SHM_PATH);
            wu_exit(-PW_ERROR);
        }
        memset(&fork_shm_data[i].msg, 0, sizeof(uds_msg_t));
    }
#endif // !_ANDROID_
}
/*
 * INTERNAL API:
 * Destroy the previously initialized shmem area.
 */
void Wuwatch::destroy_shm()
{
#if !_ANDROID_
    shm_unlink(PW_SHM_PATH);
#endif // !_ANDROID_
}
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
        wu_exit(-PW_ERROR);
    }
}
/*
 * INTERNAL API:
 * Helper function to transmit information using the (previously mmaped)
 * shared mem region.
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
    return;
#endif // !_ANDROID_
};
/*
 * INTERNAL API:
 * Helper function to send a QUIT message to all connected clients.
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
        wu_exit(-PW_ERROR);
    }
    data->count += num;
    int retVal = data->count;
    if (sem_post(&data->sem)) {
        perror("sem_post error");
        destroy_shm();
        wu_exit(-PW_ERROR);
    }
#else // !_ANDROID_
    int retVal = PW_SUCCESS;
#endif
    return retVal;
};

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
        wu_exit(-PW_ERROR);
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
            wu_exit(-PW_ERROR);
        case 0: // timeout!
            if (kill(child_pid, SIGKILL)) {
                perror("kill error");
                wu_exit(-PW_ERROR);
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
        wu_exit(-PW_ERROR);
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
                wu_exit(-PW_ERROR);
            }
        } else {
            perror("sigtimedwait error");
            wu_exit(-PW_ERROR);
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
    int ret_code = PW_SUCCESS;

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
        wu_exit(-PW_ERROR);
    } else {
        /* parent */
        int status;
        int offset = SERVER_SHM_OFFSET, tmp_offset = -1;
        if (child_pid == -1) {
            perror("fork");
            wu_exit(-PW_ERROR);
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
                    wu_exit(-PW_ERROR);
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
                db_fprintf(stderr, "DEBUG: SECS = %lu, USECS = %lu\n",(unsigned long)tv.tv_sec, (unsigned long)tv.tv_usec);
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
                            wu_exit(-PW_ERROR);
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
                                wu_exit(-PW_ERROR);
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
                db_fprintf(stderr, "# CLIENTS = %d, OFFSETS map size = %lu\n", num_clients, TO_UL(offsets.size()));
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
                wu_exit(-PW_ERROR);
            }
            /*
             * We need to check the child's return/exit status
             * (to check for errors).
             */
            if (WIFEXITED(status)) {
                ret_code = WEXITSTATUS(status);
                db_fprintf(stderr, "Child exitted: ret_code = %d\n", ret_code);
                if (ret_code != PW_SUCCESS) {
                    ret_code = -PW_ERROR;
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
            db_fprintf(stderr, "# descendent pids = %lu\n", TO_UL(descendent_pids.size()));
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

//#ifdef HAVE_POSIX_CLOCKS
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        m_finalTime = (unsigned long long)now.tv_sec*1000000000LL + now.tv_nsec;
//#else
//        struct timeval tv;
//        gettimeofday(&tv, NULL);
//        m_finalTime = (1000000ULL * tv.tv_sec + tv.tv_usec);
//#endif

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
    int ret_code = PW_SUCCESS;

    /*
     * Enable user level wakelock trace
     * by setting wakelock.trace to 1 in system property
     */
    if (u_state_collection) {
        if(system("setprop wakelock.trace 1") == -1) {
            fprintf(stderr, "ERROR: could NOT set a system property for user wakelock tracing!\n");
            wu_exit(-PW_ERROR);
        } 

#if _NDK_BUILD_
        char prop_val[PROP_VALUE_MAX];
        if(__system_property_get("wakelock.trace", prop_val) == 0 || atoi(prop_val) != 1) {
            fprintf(stderr, "ERROR: could NOT enable user wakelock tracing!\n");
            wu_exit(-PW_ERROR);
        } else {
            db_fprintf(stderr, "INFO: set a system property for user wakelock tracing\n");
        }
#endif
    }

    /*
     * OK, we're ready to start the collection. Before we do so, create (and output) 
     * the initial proc map. We defer this for as long as possible because we don't
     * want too much time to elapse between when the snapshot of the proc mappings
     * was taken and when the collection starts.
     */
    if (get_initial_proc_map_i(m_output_fp)) {
        fprintf(stderr, "ERROR: could NOT create initial proc map!\n");
        wu_exit(-PW_ERROR);
    }
    /*
     * Get active kernel wakelock names before the collection starts.
     */
    if (w_state_collection) {
        if (get_active_wakelocks_i(m_output_fp)) {
            fprintf(stderr, "ERROR: could NOT get active kernel wakelocks!\n");
            wu_exit(-PW_ERROR);
        }

        if (get_uid_pkg_map_i(m_output_fp)) {
            fprintf(stderr, "ERROR: could NOT get UID-package name mappings!\n");
            wu_exit(-PW_ERROR);
        }
    }

    /*
     * Take a snapshot of the initial TSC and 'timeval' values. We want
     * to take this after we're done trolling the "/proc" fs.
     */
    {
        struct timeval tv;
        struct tm *ptm;
        time_t rawtime;
        gettimeofday(&tv, NULL);

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        m_initialTSC = rdtsc();
        initialTimeval = (1000000ULL * tv.tv_sec + tv.tv_usec) * 10;
        ptm = localtime(&tv.tv_sec); 
        // For non-native Android build, the env var "TZ" 
        // must be set to get the right timezone value
        // e.g. TZ=CST6CDT for central time
        strftime(m_initialTimeStr, sizeof(m_initialTimeStr), "%m-%d %H:%M:%S", ptm); 

        m_initialTime = (unsigned long long)now.tv_sec*1000000000LL + now.tv_nsec;
    }

    /*
     * Start the collection (take a snapshot
     * of the system time before that).
     */
    {
        start_stop_timer(&collection_time_msecs, true); // "true" ==> START timer
    }

    /*
     * For debugging: correlate TSC and POSIX monotonic clocks.
     */
    if (true) {
        struct timespec ts;
        pw_u64_t tsc, nsecs;
        if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
            perror("clock_gettime error");
            wu_exit(-PW_ERROR);
        }
        tsc = rdtsc();
        nsecs = (pw_u64_t)ts.tv_sec * 1000000000ULL + (pw_u64_t)ts.tv_nsec;
        db_fprintf(stderr, "COLLECTION START: tsc = %llu CLK_GETTIME = %llu RATIO = %.10f\n", tsc, nsecs, (double)nsecs / (double)tsc);
    }

    {
        do_ioctl_start_stop(m_dev_fd, true);
    }

/*
    int i = pthread_barrier_wait(&reader_barrier);
    if (i && i != PTHREAD_BARRIER_SERIAL_THREAD) {
        perror("pthread_barrier_wait error");
        wu_exit(-PW_ERROR);
    }
*/
    if (s_barrier.wait()) {
        fprintf(stderr, "barrier_wait error");
        wu_exit(-PW_ERROR);
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

#if 0
    /* 
     * Read Android system logs to extract user-level wakelock data
     */
    if (u_state_collection) {
        /*
         * Disable user level wakelock trace
         * by setting wakelock.trace to 0 in system property
         */
        if(system("setprop wakelock.trace 0") == -1) {
            fprintf(stderr, "ERROR: could NOT disable user wakelock tracing!\n");
            wu_exit(-PW_ERROR);
        } 
    }
#endif

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
                wu_exit(-PW_ERROR);
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
 * Sets the data collector config file name, based on user-supplied inputs.
 *
 * @path: path to the config file.
 */
void Wuwatch::set_config_file_path(const std::string& path)
{
    assert(path.size());
    m_config_file_path = path;
    db_fprintf(stderr, "WUWATCH has config file = %s\n", m_config_file_path.c_str());
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
        m_driver_path += "./driver/";
        m_driver_path += PW_DRV_NAME;
        m_driver_path += ".ko";
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
        return -PW_ERROR;
    }
    {
        if (do_ioctl_driver_version_i(m_dev_fd)) {
            db_fprintf(stderr, "ERROR: could NOT retrieve DRIVER VERSION information!\n");
            return -PW_ERROR;
        }
    }
    ver_str = std::string(m_driver_version_string);
    return PW_SUCCESS;
};
