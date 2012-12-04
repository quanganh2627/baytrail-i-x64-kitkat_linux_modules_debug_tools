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
 * ENTRY POINT for the wuwatch tool.
 * **************************************
 */

#include <stdio.h>
#include <fcntl.h>		/* open */
#include <unistd.h>		/* exit */
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>
#include <semaphore.h>
#include <dirent.h>
#include <assert.h>
#include <sys/utsname.h>
#include <ctype.h>

#include <map>
#include <vector>
#include <list>
#include <deque>
#include <iterator>

#include "pw_defines.h"
#include "pw_ioctl.h" // IOCTL stuff.
#include "pw_arch.h" // Architecture info.
#include "uds.hpp" // for UDS server stuff.
#include "pw_bt.h"
#include "pw_utils.hpp"
#include "ht_wudump.h"
#include "wuwatch.h"
#include "defines.h"

/* **************************************
 * Compile-time constants and other
 * MACROs.
 * **************************************
 */

#define RUN_NEITHER 0x0 // 00
#define RUN_WUWATCH 0x1 // 01
#define RUN_WUDUMP 0x2 // 10
#define RUN_BOTH 0x3 // 11


/* **************************************
 * Function declarations.
 * **************************************
 */
bool is_collection_configured_for(int which);
void find_app_to_run(char **args,int argc);
int parse_args(char ***argv,int argc);
int run_wuwatch(char **args);
int run_wudump();
void cleanup();
void wu_exit(int num);
void usage(void);
void get_wuwatch_dir(std::string&);
void extract_dir_and_file_from_path_i(const std::string& path, std::string& dir, std::string& file);


/* **************************************
 * Variable declarations.
 * **************************************
 */
/*
 * Where should we write the driver
 * results?
 */
std::string g_output_file("");
/*
 * Where's the config file located?
 */
std::string g_config_file("");
/*
 * Which phase to run -- collection or parsing?
 */
int g_program_to_run = RUN_BOTH;
/*
 * Global flag to enable verbose debugging messages.
 */
bool g_do_debugging = false;
/*
 * The global wuwatch (collector) instance.
 */
Wuwatch *g_wuwatchObj = NULL;
/*
 * The global wudump (parser) instance.
 */
HTWudump *g_wudumpObj = NULL;



/* **************************************
 * Function definitions.
 * **************************************
 */

/*
 * Check to see if the user requested the given phase.
 */
bool is_collection_configured_for(int which)
{
    if ((g_program_to_run & which) == which) {
        return true;
    }
    return false;
};

/*
 * Parse CLI args to determine which phase to run
 * (collection phase == WUWATCH, parsing phase == WUDUMP)
 */
void find_app_to_run(char **args,int argc)
{
    /*
     *find out which applications do we need to run
     *None: run both wuwatch and wudump
     */

    g_program_to_run = RUN_NEITHER;

    for (int i=1;i<argc;i++) {

        if (!strcmp(args[i],"--wudump") || !strcmp(args[i],"-d")) {
            g_program_to_run |= RUN_WUDUMP;
            if (g_wudumpObj == NULL) {
                g_wudumpObj = new HTWudump();
            }
        }
        else if (!strcmp(args[i],"--wuwatch") || !strcmp(args[i],"-w")) {
            g_program_to_run |= RUN_WUWATCH;
            if (g_wuwatchObj == NULL) {
                g_wuwatchObj = new Wuwatch();
            }
        }
    }

    if (g_program_to_run == RUN_NEITHER) {
        g_program_to_run = RUN_BOTH;
        g_wuwatchObj = new Wuwatch();
        g_wudumpObj  = new HTWudump();
    }
};

/*
 * Print CLI usage information.
 */
void usage(void)
{

    fprintf(stderr, "\nUse this program to collect power data samples and dump them in text format. By default both watch and dump operations are performed.\n");
    fprintf(stderr, "\nUsage: ./wuwatch <options> <program-to-analyze>\n\n");

    fprintf(stderr,  "\t-h, --help:\tPrint this help message.\n");
    fprintf(stderr, "\t-v, --version:\tPrint wuwatch and driver version information.\n");
    fprintf(stderr,  "\t-w, --wuwatch:\tOnly run wuwatch. Do NOT dump the data.\n");
    fprintf(stderr,  "\t-d, --wudump: \tOnly run wudump to dump existing collected data.\n");
    fprintf(stderr,  "\t-o, --output-file [file name]: Specify an output file name. [file name] may optionally include an absolute or relative path. If no path is specified, [file name] will be created in the current directory\n");
    // fprintf(stderr,  "\t-o, --output-dir [output directory]: Specify a directory in which to store results(wuwatch) or access input files(wudump). PWD is used by default.\n");
    // Per Bob's request, we hide this switch to users.
    //fprintf(stderr,  "\t-f, --force-dd-load [Full path to power driver]: Load the device driver if it isn't already loaded. Requires a path argument (./driver is used by default).\n");
    // fprintf(stderr, "\t-ddb, --do-debug: Turn on debug printfs (useful for debugging)\n");
    fprintf(stderr, "\t-c, --config-file [config file name]: Specify the config file. [config file name] may optionally include an absolute or relative path.\n");

    fprintf(stderr,"\nCollection (i.e. wuwatch) configuration options:\n\n");

    fprintf(stderr,  "\t-i, --interval[msecs]:\t\tSpecify a d-state sampling interval, in ms. Interval defaults to 100 msec if this option is not specified. Applies to the -dn switch\n");
    fprintf(stderr,  "\t-t, --time[secs]:     \t\tRun collection for [time-in-seconds] seconds. wuwatch will collect indefinitely if this option is not specified (use a SIGINT to terminate indefinite collections)\n");
    fprintf(stderr,  "\t-cs,--c-states:   \t\tCollect sleep states samples.\n");
    fprintf(stderr,  "\t-ps,--p-states:   \t\tCollect frequency information.\n");
    fprintf(stderr,  "\t-kb, --kernel-backtrace: \tCollect backtrace information for kernel timers that cause processor wakeups. Requires the \"-cs\" option be selected.\n");
    fprintf(stderr,  "\t-ss,--s-states:   \t\tCollect s-state residency values.\n");
    fprintf(stderr,  "\t-ds,--d-states:   \t\tCollect south-complex d-state residency values.\n");
    fprintf(stderr,  "\t-dn,--nc-samples:\t\tCollect north-complex d-state samples.\n");
    fprintf(stderr,  "\t-wl,--wakelocks: \t\tCollect user & kernel wakelock information on Android.\n\n");

    fprintf(stderr, "Data dumper (i.e. wudump) options:\n\n");
    // fprintf(stderr, "\t-dc1, --do-c1:	\tInclude C1 percentages in the results.\n");
    fprintf(stderr, "\t-nc1, --no-c1:	\tDo NOT calculate C1 percentages. Selecting this option assigns all C1 values to C0.\n");
    fprintf(stderr, "\t-nts, --no-tsc:   \tDo NOT Dump TSC results along with residency values. Applicable only to C State samples. \n");
    fprintf(stderr, "\t-bkt, --backtrace:\tDump call stack information. Applicable only to C State samples.\n");
    // fprintf(stderr, "\t-clk, --clock-ticks:\tResidency information should be in clock ticks, instead of micro seconds. Applicable only to C State samples.\n");
    fprintf(stderr, "\t-us,  --usecs:\t\tResidency information should be in micro seconds instead of clock ticks. Applicable only to C State samples.\n");
    // fprintf(stderr, "\t-del, --delete-files:\tDelete intermediate wuwatch o/p files. These files can be used to import collected data into amplxe.\n\t\t\t\tThis switch can be used ONLY if BOTH watch and dump operations are performed.\n\n");

};

/*
 * Used to track debug builds.
 */
#ifdef BUILD_REF
    static const pw_u64_t g_buildRef = BUILD_REF;
#else
    static const pw_u64_t g_buildRef = 0x0;
#endif
#ifdef BUILD_TIMESTAMP
    static const pw_u64_t g_buildTimestamp = BUILD_TIMESTAMP;
#else
    static const pw_u64_t g_buildTimestamp = 0x0;
#endif

/*
 * Parse CLI args.
 */
int parse_args(char ***argv,int argc)
{
    int dint_set = 0;
    char **args = *argv;

    /*
     * Sanity check: MUST have at least one argument!
     */
    if (argc < 2) {
        fprintf(stderr, "\nERROR: invalid/no arguments specified!\n");
        usage();
        wu_exit(-PW_ERROR);
    }

    find_app_to_run(args,argc);

    while (*++args) {
        if (!strcmp(*args, "-h") || !strcmp(*args, "--help")) {
            usage();
            wu_exit(PW_SUCCESS);
        }
        else if (!strcmp(*args, "-v") || !strcmp(*args, "--version")) {
            std::string driver_ver_str;
            fprintf(stdout, "Wuwatch Version = %d.%d.%d\n", WUWATCH_VERSION_VERSION, WUWATCH_VERSION_INTERFACE, WUWATCH_VERSION_OTHER);
            /*
             * We *might* already have allocated a 'wuwatch' object -- check
             * for that first.
             */
            Wuwatch wObj = (g_wuwatchObj) ? *g_wuwatchObj : Wuwatch();
            if (wObj.get_driver_version(driver_ver_str)) {
                db_fprintf(stderr, "ERROR retrieving driver version string!\n");
            } else {
                fprintf(stdout, "Driver Version = %s\n", driver_ver_str.c_str());
            }
            db_fprintf(stdout, "Build ref = %llu, Build timestamp = %llu\n", g_buildRef, g_buildTimestamp);
            wu_exit(PW_SUCCESS);
        }
        else if (!strcmp(*args, "-t") || !strcmp(*args, "--time")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // system collection mode
            if (*++args == NULL) {
                fprintf(stderr, "ERROR: time option requires a value!\n");
                usage();
                return -PW_ERROR;
            }
            g_wuwatchObj->system_collection_mode_secs = atoi(*args);
            if (g_wuwatchObj->system_collection_mode_secs <= 0) {
                fprintf(stderr,"\nERROR: Collection run time must be atleast atleast 1 second.\n\n"); 
                return -PW_ERROR;
            }
            db_fprintf(stderr, "System mode! %d\n", g_wuwatchObj->system_collection_mode_secs);
        }
        else if (!strcmp(*args, "-o") || !strcmp(*args, "--output-file")) {
            // output file string found
            if (*++args == NULL) {
                fprintf(stderr, "ERROR: output-file option requires a value!\n");
                usage();
                return -PW_ERROR;
            }
            g_output_file = *args;
        }
        else if (!strcmp(*args, "-f") || !strcmp(*args, "--force-dd-load")) {
            // User is requesting us to load DD if not already loaded
            char **old_args = args;
            std::string dd_path;
            if (*++args != NULL && **args != '-') {
                db_fprintf(stderr, "DD path = %s\n", *args);
                dd_path = *args;
            } else {
                fprintf(stderr, "DEFAULT dir specified!\n");
                dd_path = "./driver/apwr3_1.ko";
                args = old_args;
            }
            g_wuwatchObj->m_do_force_dd_load = 1;
            g_wuwatchObj->set_driver_path(dd_path);
        }
        else if (!strcmp(*args, "-c") || !strcmp(*args, "--config-file")) {
            if (*++args == NULL) {
                fprintf(stderr, "ERROR: output-file option requires a value!\n");
                usage();
                return -PW_ERROR;
            }
            g_config_file = *args;
        }
        else if (!strcmp(*args, "-ddb") || !strcmp(*args, "--do-debug")) {
            g_do_debugging = true;
        }
        else if (!strcmp(*args, "-i") || !strcmp(*args, "--interval")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            int isValidNum = 1;
            if (*++args == NULL) {
                fprintf(stderr, "ERROR: dint option requires a value!\n");
                usage();
                return -PW_ERROR;
            }
            std::string str = *args;
            for (u32 i=0; i<str.length(); ++i) {
                if (!isdigit(str[i]))
                    isValidNum = 0;
            }
            if (!isValidNum) {
                fprintf(stderr, "D-State sample interval (-i) must be non-negative\n");
                return -PW_ERROR;
            }
            // D-state sample interval
            g_wuwatchObj->d_state_sample_interval_msecs = atoi(str.c_str());
            dint_set = 1;
            db_fprintf(stderr, "D-State sample interval=%d\n",g_wuwatchObj->d_state_sample_interval_msecs);
        }
#if 0
        else if (!strcmp(*args, "-del") || !strcmp(*args, "--delete-wuwatch-files")) {
            if (is_collection_configured_for(RUN_BOTH)) {
                fprintf(stderr, "Wuwatch files can only be deleted if both collector and dumper are run.\n");
            } else {
                db_fprintf(stderr, "Delete driver o/p files!\n");
                g_wudumpObj->m_do_delete_driver_files = true;
            }
        }
#endif
        else if (!strcmp(*args, "-cs") || !strcmp(*args, "--c-states")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect C-State Samples
            g_wuwatchObj->c_state_collection = 1;
            db_fprintf(stderr, "Collecting C-State samples\n");
        }
        else if (!strcmp(*args, "-ps") || !strcmp(*args, "--p-states")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect P-State Samples
            g_wuwatchObj->p_state_collection = 1;
            db_fprintf(stderr, "Collecting P-State samples\n");
        }
        else if (!strcmp(*args, "-kb") || !strcmp(*args, "--kernel-backtrace")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect kernel call traces
            g_wuwatchObj->m_do_collect_kernel_backtrace = 1;
            db_fprintf(stderr, "Collecting kernel backtrace information\n");
        }
        else if (!strcmp(*args, "-ss") || !strcmp(*args, "--s-states")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect S-State residency counters
            g_wuwatchObj->s_residency_collection = 1;
            db_fprintf(stderr, "Collecting S-State residency counters\n");
        }
        else if (!strcmp(*args, "-ds") || !strcmp(*args, "--d-states")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect D-State residency counters
            g_wuwatchObj->d_residency_collection = 1;
            db_fprintf(stderr, "Collecting D-State residency counters\n");
        }
        else if (!strcmp(*args, "-dn") || !strcmp(*args, "--nc-samples")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect D-State samples from north complex
            g_wuwatchObj->d_nc_state_collection = 1;
            db_fprintf(stderr, "Collecting north complex D-State samples\n");

            /*
             * South complex D-state sample collection is intentionally disabled
             * because we can obtain the same information from South complex
             * D-state residency counters more accurately.
             * Therefore, use D-state residency collection which provides 
             * more useful information because it counts how much time is spent
             * for each state. 
            else if (!strcmp(*args, "-ds") || !strcmp(*args, "--sc-states")) {
            if (is_collection_configured_for(RUN_WUWATCH)) {
                usage();
                return -PW_ERROR;
            }
            // Collect D-State samples from south complex
            g_wuwatchObj->d_sc_state_collection = 1;
            db_fprintf(stderr, "Collecting south complex D-State samples\n");
            */
        }
        else if (!strcmp(*args, "-kl") || !strcmp(*args, "--kernelwakelocks")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect wakelock samples
            g_wuwatchObj->w_state_collection = 1;

            /*
             * S-state sample collection is intentionally disabled
             * because it will be always S0 state because the collection
             * will always work when CPU is running.
             * Instead, use S-state residency collection which provides 
             * more useful information because it counts how much time is spent
             * for each S states. 
             }else if(!strcmp(*args, "-sstates")) {
            // Collect S-State samples
            s_state_collection = 1;
            db_fprintf(stderr, "Collecting S-State samples\n");
            */
        }
        else if (!strcmp(*args, "-ul") || !strcmp(*args, "--userwakelocks")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect wakelock samples
            g_wuwatchObj->u_state_collection = 1;
        }
        else if (!strcmp(*args, "-wl") || !strcmp(*args, "--wakelocks")) {
            if (is_collection_configured_for(RUN_WUWATCH) == false) {
                usage();
                return -PW_ERROR;
            }
            // Collect wakelock samples
            g_wuwatchObj->u_state_collection = 1;
            g_wuwatchObj->w_state_collection = 1;
        }
        else if (!strcmp(*args, "-nr") || !strcmp(*args, "--no-raw")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                usage();
                return -PW_ERROR;
            }
            db_fprintf(stderr, "NO RAW!\n");
            g_wudumpObj->m_do_raw_output = false;
        }
        else if (!strcmp(*args, "--dump-orig-samples")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                fprintf(stderr, "ERROR: cannot dump samples if you don't run the parsing phase!\n");
                return -PW_ERROR;
            }
            g_wudumpObj->m_do_dump_orig_samples = true;
        }
        else if (!strcmp(*args, "--dump-sample-stats")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                fprintf(stderr, "ERROR: cannot dump sample stats if you don't run the parsing phase!\n");
                return -PW_ERROR;
            }
            g_wudumpObj->m_do_dump_sample_stats = true;
        }
        else if (!strcmp(*args, "-nc1") || !strcmp(*args, "--no-c1")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                usage();
                return -PW_ERROR;
            }
            g_wudumpObj->m_do_check_c1_res = false;
        }
        else if (!strcmp(*args, "-nts") || !strcmp(*args, "--no-tsc")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                usage();
                return -PW_ERROR;
            }
            db_fprintf(stderr, "NO TSC!\n");
            g_wudumpObj->m_do_dump_tscs = false;
        }
        else if (!strcmp(*args, "-bkt") || !strcmp(*args, "--backtrace")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                usage();
                return -PW_ERROR;
            }
            db_fprintf(stderr, "BT!\n");
            g_wudumpObj->m_do_dump_backtraces = true;
        }
        else if (!strcmp(*args, "-us") || !strcmp(*args, "--usecs")) {
            if (is_collection_configured_for(RUN_WUDUMP) == false) {
                usage();
                return -PW_ERROR;
            }
            db_fprintf(stderr, "MICRO SECONDS!\n");
            g_wudumpObj->m_do_c_res_in_clock_ticks = false;
        }
        else if (!strcmp(*args, "-d") || !strcmp(*args, "--wudump")) {
            /* Handled in 'find_app_to_run(...)' above */
        }
        else if (!strcmp(*args, "-w") || !strcmp(*args, "--wuwatch")) {
            /* Handled in 'find_app_to_run(...)' above */
        }
        else {
            // program to analyze
            break;
        }
    }

    *argv = args;

    if (is_collection_configured_for(RUN_WUWATCH) == true) {
        /*
         * Perform some basic sanity-checking of user-supplied args here.
         */
        if (dint_set == 1) {
            /*
             * User gave us a sampling interval -- check to see if there's anything to sample!
             * UPDATE: for external release, the '-i' switch controls ONLY the NC D-state sampling interval!
             */
            // if (g_wuwatchObj->d_residency_collection == 0 && g_wuwatchObj->d_nc_state_collection == 0 /*&& g_wuwatchObj->d_sc_state_collection == 0*/) {
            if (g_wuwatchObj->d_nc_state_collection == 0 /*&& g_wuwatchObj->d_sc_state_collection == 0*/) {
                fprintf(stderr, "Warning: '-i/--interval' only works with D-States sample collection switches: -dn/--nc-samples'.\n");
            }
        }
        /*
         * Some options depend on the user selecting the "-cs" option.
         * Check that here.
         */
        if (g_wuwatchObj->c_state_collection != 1) {
            /*
             * Kernel calltrace collection REQUIRES c-state collection!
             */
            if (g_wuwatchObj->m_do_collect_kernel_backtrace == 1) {
                fprintf(stderr, "ERROR: kernel call trace collection REQUIRES c-state collection!\n");
                wu_exit(-PW_ERROR);
            }
        }
        
        /*
         * Print out a list of collections specified.
         */
        fprintf(stdout, "WUWATCH configured to collect the following data...\n");
        if (g_wuwatchObj->c_state_collection) {
            fprintf(stdout, "\tC-state data\n");
        }
        if (g_wuwatchObj->p_state_collection) {
            fprintf(stdout, "\tP-state data\n");
        }
        if (g_wuwatchObj->s_residency_collection) {
            fprintf(stdout, "\tS-state residency data\n");
        }
        if (g_wuwatchObj->d_residency_collection) {
            fprintf(stdout, "\tD-state residency data\n");
        }
        if (g_wuwatchObj->d_nc_state_collection) {
            fprintf(stdout, "\tNorth Complex D-state data\n");
        }
#if 0
        if (g_wuwatchObj->d_sc_state_collection) {
            fprintf(stdout, "\tSouth Complex D-state data\n");
        }
#endif
        if (g_wuwatchObj->w_state_collection) {
            fprintf(stdout, "\tKernel-level Wakelock data\n");
        }

        if (g_wuwatchObj->u_state_collection) {
            fprintf(stdout, "\tUser-level Wakelock data\n");
        }
    }


    return PW_SUCCESS;
};


/*
 * Get O/P dir info -- read the file "output_dir.txt"
 * (which is auto set in the 'runExecs' script file
 * based on user-supplied information).
 */
void get_wuwatch_dir(std::string& dir)
{
    FILE *fp = fopen("output_dir.txt", "r");
    if (!fp) {
        db_fprintf(stderr, "fopen error for output dir file: %s\n", strerror(errno));
        dir = "./";
    } else {
        size_t len = 0;
        ssize_t read=0;
/*
        char *line = NULL;

        read = getline(&line, &len, fp);
        line[read-1] = '\0';
*/
        char line[1024];
        /*
         * We ASSUME 1024 characters is enough!
         */
        if (fgets(line, sizeof(line), fp) == NULL) {
            perror("fgets error");
            fclose(fp);
            exit(-PW_ERROR);
        }
        fclose(fp);
        /*
         * Under LINUX, its OK to have multiple "/" before the actual file name. 
         * Thus, the path "/tmp////file1.txt" is equivalent to
         * "/tmp/file1.txt". This is why we don't check for trailing "/" in the 
         * output dir argument passed to us by the user.
         */
        dir = std::string(line);
        //free(line);
        /*
         * Sanity check -- we need the directory path to end in a
         * "/". Check to make sure it does. If it doesn't, add it.
         */
        read = dir.find_last_of('/');
        if (read == (ssize_t)std::string::npos || read != ((ssize_t)dir.size() - 1)) {
            db_fprintf(stderr, "WARNING: no trailing '/' detected in directory path: %s\n", dir.c_str());
            dir += "/";
        }
    }
};


/*
 * Run the data collector (aka 'wuwatch')
 */
int run_wuwatch(char **args)
{

    int ret_code = PW_SUCCESS;
    /*
     * Perform wuwatch-specific setup.
     */
    g_wuwatchObj->setup_collection();
    {
        ret_code = g_wuwatchObj->work(args);
    }
    /*
     * Perform wuwatch-specific cleanup.
     */
    g_wuwatchObj->cleanup_collection(ret_code == PW_SUCCESS);

    return ret_code;
};

/*
 * Run the data parser (aka 'wudump')
 */
int run_wudump()
{
    fprintf(stderr, "Processing result data.\n");
    return g_wudumpObj->do_work();
};

/*
 * Global exit function.
 */
void wu_exit(int num)
{
    cleanup();
    exit(num);
};
/*
 * Free resources. Currently only deletes instance variables.
 */
void cleanup()
{
    delete g_wudumpObj;
    delete g_wuwatchObj;
};
/*
 * Setup a collection.
 */
void setup()
{
    std::string output_dir_name, output_file_name;
    std::string config_file_path;
    if (g_output_file.size() == 0) {
        get_wuwatch_dir(output_dir_name);
        output_file_name = DEFAULT_WUWATCH_OUTPUT_FILE_NAME;
    } else {
        /*
         * We've been given a path -- extract dir and file names
         * from it.
         */
        extract_dir_and_file_from_path_i(g_output_file, output_dir_name, output_file_name);
        db_fprintf(stderr, "OUTPUT FILE: %s, DIR: %s, output_file_name: %s\n", g_output_file.c_str(), output_dir_name.c_str(), output_file_name.c_str());
    }
    if (g_config_file.size() == 0) {
        config_file_path = std::string("./") + std::string(DEFAULT_WUWATCH_CONFIG_FILE_NAME);
    } else {
        /*
         * We've been given a path -- no need to parse it!
         */
        config_file_path = g_config_file;
    }
    if (g_wuwatchObj) {
        g_wuwatchObj->set_output_file_name(output_dir_name, output_file_name);
        g_wuwatchObj->set_config_file_path(config_file_path);
    }
    if (g_wudumpObj) {
        g_wudumpObj->set_output_file_name(output_dir_name, output_file_name);
    }
};

/*
 * Helper function to extract dir and file names
 * from a path.
 */
void extract_dir_and_file_from_path_i(const std::string& path, std::string& dir, std::string& file)
{
    int size = path.size();
    assert(size > 0);
    extract_dir_and_file_from_path(path, dir, file);
    if (file == "" || file == ".") {
        file = DEFAULT_WUWATCH_OUTPUT_FILE_NAME;
    }
};

/*
 * The main function.
 */
int main(int argc, char *argv[])
{
    /*
     * Parse user-supplied args.
     */
    if (parse_args(&argv, argc)) {
        db_fprintf(stderr, "ERROR parsing args!\n");
        wu_exit(-PW_ERROR);
    }
    /*
     * We're done parsing args -- initialize everything.
     */
    setup();
    {
        int wuwatch_ret_code = PW_SUCCESS;
        /*
         * OK, we're done with setup etc.
         * Now run the two phases.
         */

        if (is_collection_configured_for(RUN_WUWATCH) == true) {
            /*
             * User asked us to run the data collector.
             */
            wuwatch_ret_code = run_wuwatch(argv);
            if (wuwatch_ret_code) {
                db_fprintf(stderr, "ERROR running wuwatch!\n");
            }
        }

        if (is_collection_configured_for(RUN_WUDUMP) == true && wuwatch_ret_code == PW_SUCCESS) {
            /*
             * User asked us to parse data.
             */
            if (run_wudump()) {
                db_fprintf(stderr, "ERROR running wudump!\n");
            }
        }
    }
    /*
     * All done! Free up resources.
     */
    cleanup();
};
