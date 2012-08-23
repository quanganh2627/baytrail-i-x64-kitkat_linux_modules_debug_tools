/* ***************************************************************************************
 * Copyright (C) 2011 Intel Corporation. All rights reserved.
 * Other names and brands may be claimed sa the property of others.
 * Internal use only -- Do Not Distribute
 * ***************************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <sys/types.h>
#include <sys/wait.h>

/*
 * for "sched_setaffinity"
 * IMPORTANT: compile with "-D_GNU_SOURCE"
 */
#include <sched.h>
/*
 * for "gettid()"
 */
#include <unistd.h>
#include <sys/syscall.h>

#define gettid() (pid_t)syscall(SYS_gettid)

#define LOW 1000000
#define HIGH 10000000
#define NUM_ITERS 2000
// #define NUM_ITERS 5000
#define GET_RANGE(x,y) ( (y) - (x) + 1 )
#define GET_NEXT_RAND(s, x, y) ( (x) + (int)((double)GET_RANGE( (x),(y) ) * (double)rand_r( &(s) ) / (RAND_MAX + 1.0) ) )

void foo(void)
{
	unsigned long next = 0;
	unsigned int seed = 1;
	int numIters = NUM_ITERS;
	struct timespec ts;
	while(--numIters >= 0){
		next = GET_NEXT_RAND(seed, LOW, HIGH);

		ts.tv_sec = next / 1000000000L;
		ts.tv_nsec = next % 1000000000L;

		nanosleep(&ts, NULL);

		/*
		 * Debugging: change affinity (but
		 * only once).
		 */
		if(false && numIters == 500){
			cpu_set_t mask;
			/*
			 * Set affinity here
			 */
			{
				CPU_ZERO(&mask);
				CPU_SET(4, &mask);

				if(sched_setaffinity(gettid(), sizeof(cpu_set_t), &mask)){
					perror("sched_setaffinity error");
					exit(-1);
				}
			}
		}

		ts.tv_sec = next / 1000000000L;
		ts.tv_nsec = next % 1000000000L;

		nanosleep(&ts, NULL);
	}
	return;
};

int main(int argc, char *argv[])
{
	int i=0;
	int me_ = 3;
	cpu_set_t mask;
	/*
	 * Set affinity here
	 */
	if(false){
		CPU_ZERO(&mask);
		CPU_SET(me_, &mask);

		if(sched_setaffinity(gettid(), sizeof(cpu_set_t), &mask)){
			perror("sched_setaffinity error");
			exit(-1);
		}
	}
#if 0
	pid_t child_pid = fork();
	if(!child_pid){
		/* Child */
		foo();
	}else{
		/* Parent */
		int status = -1;
		if(waitpid(child_pid, &status, 0) < 0){
			perror("waitpid error");
			exit(-1);
		}
	}
#else // if 1
	foo();
#endif
	fprintf(stderr, "Done!\n");
};
