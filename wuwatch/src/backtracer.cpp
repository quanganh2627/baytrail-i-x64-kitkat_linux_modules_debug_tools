/*****************************************************************************
#                         INTEL CONFIDENTIAL
#
# Copyright 2009-2011 Intel Corporation All Rights Reserved.
#
# The source code contained or described herein and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors.  Title to the Material remains with Intel
# Corporation or its suppliers and licensors.  The Material contains
# trade secrets and proprietary and confidential information of Intel
# or its suppliers and licensors.  The Material is protected by worldwide
# copyright and trade secret laws and treaty provisions.  No part of the
# Material may be used, copied, reproduced, modified, published, uploaded,
# posted, transmitted, distributed, or disclosed in any way without
# Intel's prior express written permission.
#
# No license under any patent, copyright, trade secret or other
# intellectual property right is granted to or conferred upon you by
# disclosure or delivery of the Materials, either expressly, by
# implication, inducement, estoppel or otherwise.  Any license under such
# intellectual property rights must be express and approved by Intel in
# writing.
#
****************************************************************************
*/

/*
 * BACKTRACER
 * Functions to parse WUWATCH and HOOK LIB output
 * and 'bracket' backtraces i.e. correlate C-STATE
 * information collected by the pwr driver with
 * source code for the various benchmarks run.
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

#include <string.h>
#include <sys/time.h>

#include <assert.h>

#include <errno.h>

#include <map>
#include <vector>
#include <list>
#include <deque>

#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>


#include <sstream> // for std::stringstream

#include "pw_ioctl.h"
#include "ksym_extractor.hpp"
#include "pw_utils.hpp"
#include "pw_bt.h"


/*
 * VERSION information.
 * Format: X.Y.Z
 */
// Backtracer is currently at 1.0.0
#define BACKTRACER_VERSION_VERSION 1
#define BACKTRACER_VERSION_INTERFACE 0
#define BACKTRACER_VERSION_OTHER 0

#define SUCCESS 0
#define ERROR 1

/*
 * Config params and compile-time
 * directives.
 */
/*
 * How many samples to read at one go?
 */
#define NUM_MSGS_TO_READ 1024

/*
 * Forward Declarations.
 */
struct bt_info;

/*
 * Debug printfs etc.
 */
#define DB_FPRINTF(...) do {\
	if(env_pw_do_debug_output){ \
		fprintf(__VA_ARGS__); \
	} \
}while(0)

#define DB_ASSERT(e, ...) do {\
	if(env_pw_do_debug_output && !(e)){ \
		fprintf(stderr, __VA_ARGS__); \
		assert(false); \
	} \
}while(0)

#define DB_ABORT(...) do {\
	if(env_pw_do_debug_output){ \
		fprintf(stderr, __VA_ARGS__); \
		assert(false); \
	} \
}while(0)

/*
 * Some useful typedefs.
 */
typedef std::vector<PWCollector_sample_t> sample_vec_t;
typedef std::vector<bt_info> bt_info_vec_t;
typedef std::string str_t;
typedef std::ifstream ifstream_t;
typedef std::vector<str_t> str_vec_t;
typedef std::map<pid_t, bool> pid_map_t;

/*
 * Variable declarations.
 */

/*
 * Where should we read the driver
 * results?
 */
static char *input_file_name = NULL;
static char *output_dir_name = NULL;
/*
 * What are the "processes of interest"
 * (if any)?
 */
static pid_map_t pids_of_interest;
/*
 * Should we do debug printfs etc?
 * Populated via an (external) environment
 * variable.
 */
static bool env_pw_do_debug_output=false;


/*
 * Helper data structures.
 */
struct bt_info{
    pid_t m_tid;
    u64 m_begin, m_end;

    bt_info(pid_t t, const u64& b, const u64& e):m_tid(t), m_begin(b), m_end(e){};
};

/*
 * Functions.
 */

/*
 * Get O/P dir info -- read the file "output_dir.txt"
 * (which is auto set in the 'runExecs' script file
 * based on user-supplied information).
 */
static void find_output_dirname(void)
{
    char *line = NULL;
    bool found_output = false;
    FILE *fp = fopen("output_dir.txt", "r");
    if(!fp){
	// perror("fopen error for output dir file");
        DB_FPRINTF(stderr, "fopen error for output dir file: %s\n", strerror(errno));
	line = const_cast<char *>(".");
    }else{
	size_t len = 0;
	ssize_t read=0;

	read = getline(&line, &len, fp);
	line[read-1] = '\0';

	found_output = true;

	fclose(fp);
    }

    /*
     * Under LINUX, its OK to have multiple "/"
     * before the actual file name. Thus, the path
     * "/tmp////file1.txt" is equivalent to
     * "/tmp/file1.txt".
     * This is why we don't check for trailing "/"
     * in the output dir argument passed to us
     * by the user.
     */
    output_dir_name = strdup(line);

    if(found_output)
	free(line);
};

/*
 * Sets driver output file name.
 */
static void set_input_filename(void)
{
    if(!output_dir_name)
	find_output_dirname();

    input_file_name = strdup((std::string(output_dir_name).append("/driver_output.txt")).c_str());
    DB_FPRINTF(stderr, "DRIVER file name = %s\n", input_file_name);
};


/*
 * Searches for (but does NOT open) files 
 * output by the hook library.
 *
 * UPDATE: 'popen' doesn't work under
 * Android -- we ask that 'wuwatch'
 * store the app pid + descendent pids
 * in a file and read that file, instead.
 * Currently we use a special 'tmp' file.
 * In the future we might want to use the
 * 'sys_params_found.txt' file instead.
 */
void find_lib_files(void)
{
    if(!output_dir_name)
	find_output_dirname();

    /*
     * We read the PIDs from the "__tmp_descendent_pids.txt" file.
     */
    std::ifstream pid_input_file;
    char input_file_name[1024];
    sprintf(input_file_name, "%s/__tmp_descendent_pids.txt", output_dir_name);
    pid_input_file.open(input_file_name);
    DB_ASSERT(pid_input_file.is_open(), "ERROR: could NOT open descendent pids file in wudump! File name = %s\n", input_file_name);
    if(!pid_input_file.is_open()){
        fprintf(stderr, "ERROR: could NOT open descendent pids file in wudump!\n");
        exit(-1);
    }
    while(pid_input_file.good()){
        std::string line;
        getline(pid_input_file, line);
        if(!line.size())
            continue;
	pids_of_interest[atoi(line.c_str())] = true;
    }
    pid_input_file.close();
};

void check_environ_vars()
{
    const char *env = getenv("PW_DO_DEBUG_OUTPUT");
    env_pw_do_debug_output = env && tolower(env[0]) == 'y';

    return;
};

int bracket_sample(const u64& tsc, const trace_pair_list_t& pair_list)
{
    /*
     * Simple linear search because we expect
     * that 'pair_list' will be in ASCENDING
     * TSC order, so hits *should* occur in
     * the first few iterations.
     * TODO: should we only have to check
     * the FIRST entry?
     * UPDATE: NO! Even a lowly "printf(...)"
     * can cause a sleep event!
     */
    for(trace_pair_list_t::const_iterator citer = pair_list.begin(); citer != pair_list.end() && tsc <= citer->m_end; ++citer){
	/*
	 * For now, we ASSUME no two
	 * [Begin,End] pairs will EVER
	 * intersect (i.e. all such 
	 * pairs are mutually disjoint).
	 * ***********************************************
	 * THIS IS A POSSIBLY INVALID ASSUMPTION!!!
	 * ***********************************************
	 */
	if(citer->m_begin < tsc && tsc < citer->m_end){
	    DB_FPRINTF(stderr, "%llu < %llu < %llu\n", citer->m_begin, tsc, citer->m_end);
	    return SUCCESS;
	}
    }
    return -ERROR;
};

int do_work(FILE *driver_input_fp)
{
    PWCollector_sample_t *samples = NULL;
    int i=0;
    trace_vec_t trace_vec;
    int retVal = -ERROR;
    /*
     * Map to hold tid <-> backtrace information.
     * This info is generated by the hook library, 
     * and by the kernel (for kernel call stacks).
     */
    trace_pair_map_t pair_map;
    /*
     * Step (1): Read the lib_XXX.txt files
     * (if any).
     */
    {
	for(pid_map_t::iterator iter = pids_of_interest.begin(); iter != pids_of_interest.end(); ++iter){
	    std::stringstream full_name;
	    char *lib_file_name = NULL;
	    full_name << output_dir_name << "/lib_output_" << iter->first << ".txt";
	    lib_file_name = strdup(full_name.str().c_str());
	    DB_FPRINTF(stderr, "FULL lib name = %s\n", lib_file_name);
	    FILE *fp = fopen(lib_file_name, "r");
	    // DB_ASSERT(fp, "ERROR: could NOT open lib file %s\n", lib_file_name);
	    if(!fp){
		DB_FPRINTF(stderr, "ERROR: could NOT open lib file %s: %s\n", lib_file_name, strerror(errno));
		free(lib_file_name);
		exit(-ERROR);
	    }
	    DB_FPRINTF(stderr, "%p\n", fp);
#if WAS_BINARY_DUMP
	    Tracer::instance()->deserialize_traces(fp, trace_vec, pair_map);
#else
	    str_t ver_str; // not used; present for wudump-compatibility
	    Tracer::instance()->read_traces(fp, trace_vec, pair_map, ver_str);
#endif

	    fclose(fp);
	    free(lib_file_name);

	    // DB_FPRINTF(stderr, "[%d]: TRACE_PAIR LIST SIZE = %d\n", iter->first, pair_map[iter->first].size());
	}
	/*
	 * Make sure 'Tracer' is destroyed.
	 */
	Tracer::destroy();
    }
    /*
     * Step (2): Read driver output samples
     * into (per-thread) buffers.
     */
    {
	PWCollector_sample_t *sample = NULL;
	samples  = new PWCollector_sample_t[NUM_MSGS_TO_READ];
	do{
	    i = fread(samples, sizeof(PWCollector_sample_t), NUM_MSGS_TO_READ, driver_input_fp);
	    for(int j=0; j < i && (sample = samples + j); ++j){
		/*
		 * OK, we've read some samples. Now see if
		 * we can successfully bracket them.
		 * NOTE: we can't just check the 'pids_of_interest'
		 * map to see which TIDs we must bracket because
		 * a given process can have multiple threads.
		 * Instead, we look at the 'pair_map' (which DOES
		 * store backtraces on a per-thread basis).
		 */
		if(sample->sample_type == C_STATE /* We're only interested in C-state samples...*/
		   // && pids_of_interest[sample->c_sample.tid] == true /* ... caused by "processes of interest" */){
		   && pair_map[sample->c_sample.tid].size() > 0 /* ... caused by "processes of interest" */){
		    /*
		     * OK, this is a C-STATE sample with TID of interest.
		     * Try and "bracket" it.
		     */
		    if(bracket_sample(sample->c_sample.c_data, pair_map[sample->c_sample.tid]) == SUCCESS){
			/*
			 * SUCCESS! We don't need to proceed further because we know
			 * that:
			 * (a) We obtained C-STATE samples from the driver and
			 * (b) at least one of the C-STATE samples was caused by
			 *     a "process of interest" (i.e. the benchmark app run by
			 *     'wuwatch') and
			 * (c) at least one of the resultant C-STATE samples was caused
			 *     by a timer for which we have a valid (Ring-3) backtrace.
			 * This satisfies all of our success criteria.
			 */
			DB_FPRINTF(stderr, "SUCCESS: bracketed a TSC value!\n");
			retVal = SUCCESS;
			break;
		    }
		}
	    }
	}while(i > 0 && retVal != SUCCESS);
	/*
	 * Make sure we clean up after ourselves!
	 */
	delete []samples;
    }

    /*
     * And finally, cleanup.
     */
    {
	int len = trace_vec.size();
	for(int i=0; i<len; ++i){
	    trace_t *tr = trace_vec[i];
	    delete tr;
	}
	trace_vec.clear();
    }

    return retVal;
};


/*
 * Get input file names. These comprise
 * the file containing device driver
 * output and the file(s) output by
 * the hook library.
 */
void init()
{
    /*
     * Check for debugging environment
     * variables.
     */
    check_environ_vars();
    /*
     * Get driver output file
     * name.
     */
    set_input_filename();
    /*
     * Get hook lib output file
     * name(s).
     */
    find_lib_files();
};

/*
 * Close any previously opened
 * file descriptors and free
 * file name strings.
 */
void destroy()
{
    free(output_dir_name);
    free(input_file_name);
};

int main(int argc, char *argv[])
{
    int result = -ERROR;
    init();
    {
	FILE *fp = fopen(input_file_name, "rb");
	if(!fp){
	    DB_FPRINTF(stderr, "ERROR: fopen error: %s\n", strerror(errno));
	    exit(-ERROR);
	}
	result = do_work(fp);
	fclose(fp);
    }
    destroy();

    if(result){
	DB_FPRINTF(stderr, "ERROR: could NOT associate a backtrace!\n");
	exit(-ERROR);
    }
    exit(SUCCESS);
};
