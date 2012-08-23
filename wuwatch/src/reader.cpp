/* ***************************************************************************************
 * Copyright (C) 2011 Intel Corporation. All rights reserved.
 * Other names and brands may be claimed sa the property of others.
 * Internal use only -- Do Not Distribute
 * ***************************************************************************************
 */


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <string>
#include <string.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iterator>

#include <sys/time.h>
#include <assert.h>

// #include <boost/thread.hpp>
#include <pthread.h>

#include "pw_ioctl.h"
#include "ksym_extractor.hpp"

/*
 * This program reads the binary file output by the
 * 'execer' program and dumps its contents to stderr.
 */

/*
 * Where should we read the driver
 * results?
 */
static char *input_file_name = NULL;

/*
 * How many CPUs do we have
 * on the current system?
 * (Required for sorting verification)
 */
static int max_num_cpus = -1;

typedef std::vector<PWCollector_sample_t> sample_vec_t;

/*
 * For sorting verification...
 */
static sample_vec_t *per_cpu_sample_vecs = NULL;

/*
 * How many samples to read at one go?
 */
#define NUM_MSGS_TO_READ 1024

/*
 * Should we check if the output results
 * are sorted? Set using an input 'arg'
 * switch.
 */
static bool arg_should_check_sorted = false;
/*
 * Are we reading MFLD results? Set
 * using an input 'arg' switch.
 */
static bool arg_is_mfld_output = false;

/*
 * String equivalents of the PW_BREAK_TYPE_XXX
 * break type enum values.
 */
static const char *break_type_names[] = {"I", "T", "S", "IPI", "?"};
/*
 * For HT-testing.
 */
static const char *s_long_sample_names[] = {"FREE_SAMPLE", "TPS", "TPF", "K_CALL", "M_MAP", "I_MAP", "P_MAP", "S_RES", "S_ST", "D_RES", "D_ST", "TIM", "IRQ", "WRQ", "SCD", "IPI", "TPE", "NULL", "SAMPLE_END"};

struct sample_sorter{
    bool operator()(const PWCollector_sample_t& p1, const PWCollector_sample_t& p2){
	return p1.tsc < p2.tsc;
    };
};

bool sample_less(const PWCollector_sample_t& p1, const PWCollector_sample_t& p2){
    return p1.tsc < p2.tsc;
};

struct timer{
    struct timeval tv;
    double elapsedTime;
    double *ep;
    timer(double *e=NULL):elapsedTime(0.0), ep(e){
	if(gettimeofday(&tv, NULL)){
	    perror("gettimeofday error");
	    exit(-1);
	}
	elapsedTime = -(tv.tv_sec * 1e6 + tv.tv_usec);
    };
    ~timer(){
	if(gettimeofday(&tv, NULL)){
	    perror("gettimeofday error");
	    exit(-1);
	}
	elapsedTime += (tv.tv_sec * 1e6 + tv.tv_usec);
	if(ep)
	    *ep = elapsedTime;
	else
	    fprintf(stderr, "ELAPSED TIME = %f msecs\n", elapsedTime / 1e3);
    };
};

/*
 * Get O/P dir info -- read the file "output_dir.txt"
 * (which is auto set in the 'runExecs' script file
 * based on user-supplied information).
 */
static void set_input_filename(void)
{
    char *line = NULL;
    FILE *fp = fopen("output_dir.txt", "r");
    if(!fp){
	perror("fopen error for output dir file");
	line = const_cast<char *>(".");
    }else{
	size_t len = 0;
	ssize_t read=0;

	read = getline(&line, &len, fp);
	line[read-1] = '\0';

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
    std::string input_dir_str(line);
    input_dir_str.append("/driver_output.txt");
    input_file_name = strdup(input_dir_str.c_str());
    if(true)
	fprintf(stderr, "READER INPUT FILE NAME = %s\n", input_file_name);
};

static void get_system_stats(void)
{
    /*
     * Get # of processors in system.
     */
    max_num_cpus = sysconf(_SC_NPROCESSORS_CONF);
};

static void initialize()
{
    get_system_stats();

    per_cpu_sample_vecs = new sample_vec_t[max_num_cpus];

    set_input_filename();
};

static void cleanup()
{
    delete []per_cpu_sample_vecs;

    free(input_file_name);
};


int dump_ht_testing_sample(PWCollector_sample_t *sample)
{
    c_sample_t *cs;
    int cpu, sample_type;
    unsigned long long tsc, mperf, c1, c2, c3, c4, c5, c6, c7;

    sample_type = sample->sample_type;
    cpu = sample->cpuidx;
    tsc = sample->tsc;

    fprintf(stderr, "%s: %8d\t%16llu", s_long_sample_names[sample_type], cpu, tsc);

    switch(sample_type){
        case C_STATE:
            cs = &sample->c_sample;
            mperf = RES_COUNT(*cs, MPERF);
            c1 = RES_COUNT(*cs, APERF);
            c2 = RES_COUNT(*cs, C2);
            c3 = RES_COUNT(*cs, C3);
            c4 = RES_COUNT(*cs, C4);
            c5 = RES_COUNT(*cs, C5);
            c6 = RES_COUNT(*cs, C6);
            c7 = RES_COUNT(*cs, C7);
            fprintf(stderr, "t%16llX\t%16llX\t%16llX\t%16llX\n", c2, c4, c5, c6);
            break;
        default:
            /* NOOP */
            break;
    }
};


/*
 * A function to read samples from the
 * binary file.
 */
void do_read(FILE *fp)
{
    PWCollector_sample_t *samples = new PWCollector_sample_t[NUM_MSGS_TO_READ];
    int i=0, num=0;
    sample_vec_t sample_vec;

    fprintf(stderr, "COLLECTING...\n");
    do{
	/*
	 * Read a block of 'NUM_MSGS_TO_READ' samples. The return value
	 * indicates the NUMBER of samples read.
	 */
	i = fread(samples, sizeof(PWCollector_sample_t), NUM_MSGS_TO_READ, fp);
	for(int j=0; j<i; ++j){
	    dump_ht_testing_sample(samples+j);
	    sample_vec.push_back(samples[j]);
	}
    }while(i > 0);

    /*
     * DUMP the samples...
     */
    /*
    fprintf(stderr, "DUMPING...\n");
    sample_vec_t::iterator iter = sample_vec.begin();
    for(; iter != sample_vec.end(); ++iter)
	dump_sample(&(*iter));
    */

    fprintf(stderr, "# samples = %d\n", (int)sample_vec.size());

    /*
     * SORT the samples...
     */
    fprintf(stderr, "SORTING...\n");
    double elapsedTime = 0.0;
    {
	timer t1(&elapsedTime);
	std::sort(sample_vec.begin(), sample_vec.end(), sample_sorter());
	// std::sort(sample_vec.begin(), sample_vec.end(), sample_less);
    }
    elapsedTime /= 1e3; // in msecs
    fprintf(stderr, "ELAPSED time = %f msecs\n", elapsedTime);
    fprintf(stderr, "Avg time per sample = %f msecs/sample\n", elapsedTime / sample_vec.size());

    /*
    fprintf(stderr, "POST-SORT DUMPING...\n");
    iter = sample_vec.begin();
    for(; iter != sample_vec.end(); ++iter)
	dump_sample(&(*iter));
    */

    delete []samples;
};

struct sorting_args{
    int me;
    const sample_vec_t& samples;
    bool *is_sorted;

    sorting_args(int m, const sample_vec_t& samp, bool *is):me(m), samples(samp), is_sorted(is){};
};

// void per_cpu_sorting_checker(int me, const sample_vec_t& samples, bool* is_sorted)
void *per_cpu_sorting_checker(void *args)
{
    sorting_args *sargs = (sorting_args *)args;
    int me = sargs->me;
    const sample_vec_t& samples = sargs->samples;
    bool *is_sorted = sargs->is_sorted;
    fprintf(stderr, "Thread %d GOT SIZE = %d\n", me, (int)samples.size());
    int size = samples.size();

    for(int i=1; i<size; ++i){
	if(samples[i-1].tsc > samples[i].tsc){
	    *is_sorted = false;
	    pthread_exit(PTHREAD_CANCELED);
	}
    }

    *is_sorted = true;
    pthread_exit(PTHREAD_CANCELED);
};

/*
 * A function to read samples from the
 * binary file and check that they meet
 * sorting requirements.
 */
void do_sorting_check(FILE *fp)
{
    PWCollector_sample_t *samples = new PWCollector_sample_t[NUM_MSGS_TO_READ];
    int i=0;
    sample_vec_t sample_vec;

    fprintf(stderr, "COLLECTING...\n");
    do{
	/*
	 * Read a block of 'NUM_MSGS_TO_READ' samples. The return value
	 * indicates the NUMBER of samples read.
	 */
	i = fread(samples, sizeof(PWCollector_sample_t), NUM_MSGS_TO_READ, fp);
	if(i > 0){

	    fprintf(stderr, "CPU = %d\n", samples[i].cpuidx);

	    sample_vec_t *output_vec = &per_cpu_sample_vecs[samples[i].cpuidx];
	    for(int j=0; j<i; ++j){
		PWCollector_sample_t *sample = samples + j;
		assert(sample->cpuidx < max_num_cpus);
		if(sample->sample_type == C_STATE){
		    per_cpu_sample_vecs[sample->cpuidx].push_back(samples[j]);
		}
	    }
	}
    }while(i > 0);

    for(i=0; i<max_num_cpus; ++i){
	fprintf(stderr, "[%d]: SIZE = %d\n", i, (int)per_cpu_sample_vecs[i].size());
    }

    /*
     * Now check each (per-cpu) vector
     * for unsorted elements.
     */
    bool *is_sorted = new bool[max_num_cpus];
    // boost::thread *threads = new boost::thread[max_num_cpus];
    pthread_t *threads = new pthread_t[max_num_cpus];

    for(i=0; i<max_num_cpus; ++i){
	is_sorted[i] = false;
	// threads[i] = boost::thread(per_cpu_sorting_checker, i, per_cpu_sample_vecs[i], &is_sorted[i]);
	sorting_args *sargs = new sorting_args(i, per_cpu_sample_vecs[i], &is_sorted[i]);
	if(pthread_create(&threads[i], NULL, per_cpu_sorting_checker, sargs)){
	    perror("pthread_create error");
	    exit(-1);
	}
    }

    for(i=0; i<max_num_cpus; ++i){
	// threads[i].join();
	if(pthread_join(threads[i], NULL)){
	    perror("pthread_join error");
	    exit(-1);
	}
	if(!is_sorted[i]){
	    fprintf(stderr, "ERROR: Thread %d reports NON-SORTED!\n", i);
	}
    }

    delete []threads;
    delete []is_sorted;
    delete []samples;
};

void usage(void)
{
    fprintf(stderr, "USAGE: ./reader <OPTIONS>\n");
    fprintf(stderr, "where <OPTIONS> are:\n");
    fprintf(stderr, "\t-t, --test-sorted: Test if C-state samples are sorted by TSC value\n");
    fprintf(stderr, "\t-m, --mfld:\t\tUse Medfield-specific options when parsing the data\n");
    fprintf(stderr, "\t-h, --help:\t\tPrint this usage message\n");
    exit(1);
};

void parse_args(char *argv[])
{
    char **tmp = argv;
    while(*++tmp){
	if(!strcmp(*tmp, "-t") || !strcmp(*tmp, "--test-sorted")){
	    fprintf(stderr, "TEST_SORTED!\n");
	    arg_should_check_sorted = true;
        }
        else if(!strcmp(*tmp, "-m") || !strcmp(*tmp, "--mfld")){
            fprintf(stderr, "MFLD!\n");
            arg_is_mfld_output = true;
        }
        else if(!strcmp(*tmp, "-h") || !strcmp(*tmp, "--help")){
            usage();
        }
        else{
	    usage();
	}
    }
};

int main(int argc, char *argv[])
{

    /*
     * Setup
     */
    parse_args(argv);
    {
	initialize();
    }
    // FILE *fp = fopen("driver_output.txt", "rb");
    FILE *fp = fopen(input_file_name, "rb");
    if(!fp){
	perror("fopen error");
	exit(-1);
    }

    if(arg_should_check_sorted){
	do_sorting_check(fp);
    }
    else{
	do_read(fp);
    }

    fclose(fp);
    /*
     * Cleanup
     */
    {
	cleanup();
    }
};
