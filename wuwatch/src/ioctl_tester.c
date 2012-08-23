/* ***************************************************************************************
 * Copyright (C) 2011 Intel Corporation. All rights reserved.
 * Other names and brands may be claimed sa the property of others.
 * Internal use only -- Do Not Distribute
 * ***************************************************************************************
 */

#include <stdio.h>
#include <fcntl.h>		/* open */
#include <unistd.h>		/* exit */
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>

/*
  #define DEVICE_FILE_NAME "/home/gupadhya/char_dev"
  #include "chardev.h"
  #include "data_structures.h"
*/
#include "pw_ioctl.h" // IOCTL stuff.

static int max_num_cpus = -1;


typedef enum{
    false=0,
    true
}bool;

/*
 * MSR counter stuff.
 *
 * Ultimately the list of MSRs to read (and the core MSR residency addresses)
 * will be specified by the "runss" tool (via the "PW_IOCTL_CONFIG" ioctl).
 *
 * For now, hardcoded to values for NHM.
 */
/*
  enum{
  TSC=0,
  MPERF,
  APERF,
  C3,
  C6,
  C7,
  RESIDENCY_COUNT
  };
*/

enum{
    MPERF=0, // C0
    APERF, // C1
    C2,
    C3,
    C4,
    C5,
    C6,
    C7,
    C8,
    C9,
    C10,
    C11,
    MAX_MSR_ADDRESSES
};

void *do_ioctl_i(int fd, int ioctl_num, void *data, int size, bool is_in)
{
    int i=0;
    struct PWCollector_ioctl_arg ioctl_arg;

    memset(&ioctl_arg, 0, sizeof(ioctl_arg));
    if(is_in){
	ioctl_arg.in_len = size;
	ioctl_arg.in_arg = data;
    }else{
	ioctl_arg.out_len = size;
	ioctl_arg.out_arg = data;
    }

    if( (i = ioctl(fd, ioctl_num, &ioctl_arg)) < 0){
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

void do_ioctl_start_stop(int fd, bool enable)
{
    int i = -1;
    struct PWCollector_ioctl_arg ioctl_arg;
    int cmd = enable ? START : STOP;

    do_ioctl_i(fd, PW_IOCTL_CMD, &cmd, sizeof(int), true); // "true" ==> INPUT param.

};

/*
 * Core MSR residency addresses for NHM.
 */
static const int NHMcoreResidencyMSRAddresses[] = {0xe7, 0xe8, -1, 0x3fc, -1, -1, 0x3fd, 0x3fe, -1, -1, -1, -1};

void do_ioctl_config(int fd, power_data_t *switches, int num)
{
    int i=0;
    struct PWCollector_ioctl_arg ioctl_arg;
    platform_info_t info;
    struct PWCollector_config config;
    int len = sizeof(config);

    memset(&info, 0, sizeof(info));
    memset(&config, 0, len);

    /*
     * Copy MSR addresses.
     * For now, leave PKG addresses blank
     */
    memcpy(info.coreResidencyMSRAddresses, NHMcoreResidencyMSRAddresses, sizeof(int) * MAX_MSR_ADDRESSES);

    memcpy(&config.info, &info, sizeof(info));

    for(i=0; i<num; ++i){
	config.data |= (switches[i] << i);
    }

    do_ioctl_i(fd, PW_IOCTL_CONFIG, &config, len, true); // "true" ==> INPUT param.

};

void do_ioctl_check_platform(int fd)
{
    int i=0;
    struct PWCollector_ioctl_arg ioctl_arg;
    struct PWCollector_check_platform check_platform, *tmp;
    int len = sizeof(check_platform);

    memset(&check_platform, 0, len);

    if( (tmp = do_ioctl_i(fd, PW_IOCTL_CHECK_PLATFORM, &check_platform, len, false)) == NULL){ // "false" ==> OUTPUT param.
	return;
    }

    check_platform = *tmp;
    fprintf(stderr, "List of unsupported tracepoints: %s\n", check_platform.unsupported_tracepoints);
    
};


void do_ioctl_sample(int fd)
{
    int i=0;
    struct PWCollector_non_precise_sample non_precise, *tmp;
    //int len = sizeof(non_precise);
    int len = sizeof(unsigned char) + max_num_cpus * sizeof(c_sample_t);
    unsigned long long tsc, mperf, c3, c6, c7;

    memset(&non_precise, 0, sizeof(non_precise));

    non_precise.num_cpus_to_sample = max_num_cpus;
    // non_precise.num_cpus_to_sample = 4;
    non_precise.samples = calloc(max_num_cpus, sizeof(c_sample_t));

    if( (tmp = do_ioctl_i(fd, PW_IOCTL_SAMPLE, &non_precise, len, false)) == NULL){ // "false" ==> OUTPUT param
	return;
    }
    non_precise = *tmp;

    //for(i=0; i<max_num_cpus; ++i){
    for(i=0; i<non_precise.num_cpus_to_sample; ++i){
	int j=0;
	c_sample_t *cs = non_precise.samples + i;
	tsc = 0x0;
	mperf = RES_COUNT(*cs, MPERF);
	c3 = RES_COUNT(*cs, C3);
	c6 = RES_COUNT(*cs, C6);
	c7 = RES_COUNT(*cs, C7);
	fprintf(stderr, "C-STATE: 0\t%d\t%d\t%16llX\t%16llX\t%16llX\t%16llX\t%16llX\n", 0, i, tsc, mperf,  c3,  c6, c7);
	/*
	 */
	/*
	  fprintf(stderr, "CPU %d\n", i);
	  for(j=0; j<MAX_MSR_ADDRESSES; ++j)
	  fprintf(stderr, "\tMSR=%d, VAL=0x%llx\n", j, RES_COUNT(*cs, j));
	*/
    }
};

void do_ioctl_status(int fd)
{
    int i=0;
    struct PWCollector_ioctl_arg ioctl_arg;
    struct PWCollector_status status, *tmp;
    int len = sizeof(status);

    memset(&status, 0, len);

    if( (tmp = do_ioctl_i(fd, PW_IOCTL_STATUS, &status, len, false)) == NULL){ // "false" ==> OUTPUT param.
	return;
    }

    status = *tmp;
    fprintf(stderr, "Total # cpus = %d\n", status.num_cpus);
    fprintf(stderr, "Total collection time = %lu msecs\n", status.time);
    fprintf(stderr, "Total # c-breaks = %lu\n", status.c_breaks);
    fprintf(stderr, "Total # timer_c_breaks = %lu\n", status.timer_c_breaks);
    fprintf(stderr, "Total # inters_c_breaks = %lu\n", status.inters_c_breaks);
    fprintf(stderr, "Total # p-state trans = %lu\n", status.p_trans);
    fprintf(stderr, "Total # interrupts = %lu\n", status.num_inters);
    fprintf(stderr, "Total # timer inters = %lu\n", status.num_timers);
    
};

void test_ioctls(void)
{
    int fd;
    int i=0;
    pid_t child_pid;
    pthread_t tid;

    max_num_cpus = sysconf(_SC_NPROCESSORS_CONF);

    if( (fd = open(DEVICE_FILE_NAME, 0)) < 0){
	perror("open error");
	exit(-1);
    }

    /*
     * Multiple starts/stops testing here.
     */
#if 0
    {
	do_ioctl_start_stop(fd, true);
	do_ioctl_start_stop(fd, true);
	do_ioctl_start_stop(fd, false);
	do_ioctl_start_stop(fd, false);
    }
#endif
    /*
     * Config testing here.
     */
#if 0
    {
	int max = SYSTEM+1;
	power_data_t switches[TOTAL];

	memset(switches, 0, TOTAL * sizeof(int));

	switches[SLEEP] = 1;
	switches[KTIMER] = 1;
	switches[SYSTEM] = 1;

	do_ioctl_config(fd, switches, TOTAL);
    }
#endif
    /*
     * Check platform testing here
     */
#if 0
    {
	do_ioctl_check_platform(fd);
    }
#endif
    /*
     * Sample testing here.
     * Requires a running collection
     * and "SAMPLE" mode for config.
     */
#if 0
    {
	power_data_t switches[TOTAL];
	memset(switches, 0, (TOTAL * sizeof(int)));
	switches[SYSTEM] = 1;

	do_ioctl_start_stop(fd, true);
	do_ioctl_config(fd, switches, TOTAL);
	for(i=0; i<10; ++i){
	    sleep(1);
	    do_ioctl_sample(fd);
	}
	do_ioctl_start_stop(fd, false);
    }
#endif
    /*
     * Status testing here.
     * Requires a running collection.
     */
#if 1
    {
	do_ioctl_start_stop(fd, true);
	for(i=0; i<4; ++i){
	    sleep(1);
	    do_ioctl_status(fd);
	}
	do_ioctl_start_stop(fd, false);
    }
#endif

    close(fd);
};

int main(int argc, char *argv[])
{
    test_ioctls();
};
