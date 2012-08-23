/* ***************************************************************************************
 * Copyright (C) 2011 Intel Corporation. All rights reserved.
 * Other names and brands may be claimed sa the property of others.
 * Internal use only -- Do Not Distribute
 * ***************************************************************************************
 */

#include <stdio.h>
#include <assert.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <poll.h>
#include <semaphore.h>
#include <dlfcn.h>
#include <execinfo.h> // for backtrace
#include <sys/un.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

#include <map>
#include <vector>
#include <list>
#include <deque>
#include <string>
#include <algorithm>

#include "uds.hpp"

#include "hook_lib.h"


/*
 * VERSION information.
 * Format: X.Y.Z
 */
// We're currently at 2.2.6
#define HOOK_LIB_VERSION_VERSION 2
#define HOOK_LIB_VERSION_INTERFACE 2
#define HOOK_LIB_VERSION_OTHER 6

#ifndef SUCCESS
#define SUCCESS 0
#define ERROR 1
#endif

#define READ_FD 0
#define WRITE_FD 1

/*
 * Set to "1" to have debug fprintfs, asserts etc.
 */
#define DO_DEBUG_OUTPUT 0

#if DO_DEBUG_OUTPUT

#define db_fprintf(...) fprintf(__VA_ARGS__)
#define db_assert(e,...) do{			\
	if(!(e)){				\
	    fprintf(stderr, __VA_ARGS__);	\
	    assert(false);			\
	}					\
    }while(0)
#define db_abort(...) do{			\
	fprintf(__VA_ARGS__);			\
	assert(false);				\
    }while(0)

#else // DO_DEBUG_OUTPUT

#define db_fprintf(...) {}
#define db_assert(...) {}
#define db_abort(...) {}

#endif // DO_DEBUG_OUTPUT


/*
 * Do we use 'sched_getcpu()' to
 * determine which CPU the code is 
 * currently executing on? Setting
 * to 0 defaults CPU to 0.
 * ************************************************
 * SET TO ZERO FOR ANDROID!!!
 * ************************************************
 * '1' ==> YES, use sched_getcpu()
 * '0' ==> NO, do not use it.
 */
#define DO_USE_SCHED_GETCPU 0

static int uds_sockfd = -1, shm_offset = -1;
static shm_data_t *fork_shm_data;
static pthread_t uds_client_thread_id;
static bool uds_client_status = false;

static trace_map_t *trace_map = NULL;

static const char *uds_file_name = NULL;

//#define NEW_TSC_PAIR(b,e,n) ({tsc_pair_t *__tmp1 = (tsc_pair_t *)malloc(sizeof(tsc_pair_t)); __tmp1->begin = (b); __tmp1->end = (e); __tmp1->next = (n); __tmp1;})

#define NUM_HASH_LOCK_BITS 4 // 16 locks
#define NUM_HASH_LOCKS (1UL << NUM_HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

static pthread_mutex_t hashLocks[NUM_HASH_LOCKS];

#define LOCK(l) if(pthread_mutex_lock(&(l))){	\
	perror("pthread_mutex_lock error");	\
	exit(-ERROR);				\
    }

#define UNLOCK(l) if(pthread_mutex_unlock(&(l))){	\
	perror("pthread_mutex_unlock error");		\
	exit(-ERROR);					\
    }

#define HASH_LOCK(i) LOCK(hashLocks[(i) & HASH_LOCK_MASK])
#define HASH_UNLOCK(i) UNLOCK(hashLocks[(i) & HASH_LOCK_MASK])

static pw_atomic_t do_nothing = PW_ATOMIC_INIT(0);
static pw_atomic_t lib_inited = PW_ATOMIC_INIT(0);

unsigned long long rdtsc(void)
{
    unsigned a, d;

    __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));

    return ((unsigned long long)a) | (((unsigned long long)d) << 32);;
};

static void vec_clear(trace_t *trace)
{
    delete trace;
};

static void map_clear(std::pair<pid_t, trace_vec_t> p)
{
    trace_vec_t vec = p.second;
    for_each(vec.begin(), vec.end(), vec_clear);
    p.second.clear();
};

struct map_dumper{
    FILE *fp;
    map_dumper(FILE *f):fp(f){};
    void operator()(std::pair<pid_t, trace_vec_t> p){
	trace_vec_t vec = p.second;
	for_each(vec.begin(), vec.end(), trace_dumper(fp));
    };
};

static inline trace_t *find_trace_i(pid_t tid, void *ret_addr)
{
    trace_t *retVal = NULL;
    HASH_LOCK(0);
    {
	// trace_vec_t *vec = &((*trace_map)[tid]);
	trace_vec_t vec = (*trace_map)[tid];
	trace_vec_t::iterator iter = vec.begin();
	for(; iter != vec.end(); ++iter){
	    trace_t *tmp = *iter;
	    if(tmp->tid != tid){
		assert(false);
	    }
	    if(tmp->tid == tid && tmp->ret_addr == ret_addr){
		// fprintf(stderr, "FOUND: [%d,%d], [%p,%p]\n", tid, tmp->tid, ret_addr, tmp->ret_addr);
		retVal = tmp;
		break;
	    }
	}
    }
    HASH_UNLOCK(0);
    return retVal;
};

static inline trace_t *insert_trace_i(pid_t pid, pid_t tid, void *ret_addr)
{
    trace_t *trace = new trace_t(pid, tid, ret_addr);
    assert(trace);
    HASH_LOCK(0);
    {
	(*trace_map)[tid].push_back(trace);
    }
    HASH_UNLOCK(0);
    return trace;
};

/*
 * Generate a "backtrace" and add it to the trace entry.
 * This is not perfect but will do, for now.
 */
static inline void do_backtrace(trace_t *trace)
{
    void *trace_buff[MAX_BACKTRACE_SIZE];
    int i=0, numTrace = 0;

    trace->num_trace = numTrace = backtrace(trace_buff, MAX_BACKTRACE_SIZE);
    if(!(trace->trace_symbols = backtrace_symbols(trace_buff, numTrace))){
	perror("backtrace_symbols error");
	exit(-ERROR);
    }
    /*
     * Code below present for debugging ONLY!
     */
#if 0
    {
	Dl_info dl;
	for(int i=0; i<numTrace; ++i){
	    if(!dladdr(trace_buff[i], &dl)){
		perror("dladdr error");
		exit(-ERROR);
	    }
	    fprintf(stderr, "SName = %s, FName = %s\n", dl.dli_sname, dl.dli_fname);
	}
    }
#endif
};

static inline void insert_trace(pid_t tid, void *ret_addr, int cpu, const uint64_t& begin, const uint64_t& end)
{
    /*
     * Return IMMEDIATELY if told to do so.
     */
    if(pw_atomic_read(&do_nothing)){
	return;
    }

    trace_t *trace = find_trace_i(tid, ret_addr);
    if(!trace){
	pid_t pid = getpid();
	trace = insert_trace_i(pid, tid, ret_addr);
	/*
	 * Also do a "backtrace" and add to the "trace" array here.
	 */
	do_backtrace(trace);
    }
    // trace->head_tsc = NEW_TSC_PAIR(begin, end, trace->head_tsc);
    /*
     * IF check is for DEBUGGING ONLY!
     */
#if 0
    if(trace->num_tsc_pairs % 10000 == 0){
	fprintf(stderr, "%d\n", trace->num_tsc_pairs);
    }
#endif
#if 0
    if(trace->head_tsc){
	trace->num_tsc_pairs++;
	return;
    }
#endif
    trace->head_tsc = new tsc_pair_t(cpu, begin, end, trace->head_tsc);
    assert(trace->head_tsc);
    trace->num_tsc_pairs++;
};

static inline void dump_traces(FILE *fp)
{
    // fprintf(stderr, "trace map size = %d\n", trace_map->size());
    for_each(trace_map->begin(), trace_map->end(), map_dumper(fp));
};


static inline void iterate_traces(FILE *fp)
{
#if 0
    int i=0;
    tsc_pair_t *pair = NULL;

    for(i=0; i<1024 && traces[i].ret_addr != NULL; ++i){
	fprintf(fp, "[%d,%d]: %p\n", traces[i].pid, traces[i].tid, traces[i].ret_addr);
	fprintf(fp, "\t STACK-TRACE: return addrs...\n");
	for(int j=0; j<traces[i].num_trace; ++j)
	    fprintf(fp, "\t\t%lu\n", traces[i].trace[j]);
	fprintf(fp, "\t STACK-TRACE: symbols...\n");
	for(int j=0; j<traces[i].num_trace; ++j)
	    fprintf(fp, "\t\t%s\n", traces[i].trace_symbols[j]);
	for_each_tsc_pair(pair, traces[i].head_tsc){
	    fprintf(fp, "\t(%d): [%llu,%llu]\n", pair->cpu, pair->begin, pair->end);
	}
	fprintf(fp, "\n");
	{
	    free(traces[i].trace);
	    free(traces[i].trace_symbols);
	}
	delete traces[i].head_tsc;
    }
#endif
};

#if DO_USE_SCHED_GETCPU
#define GETCPU() sched_getcpu()
#else
#define GETCPU() 0
#endif

#define PREAMBLE()				\
    uint64_t begin_tsc, end_tsc;		\
    pid_t tid = 0;				\
    int tsc_cpu = -1;				\
    do{						\
	begin_tsc = rdtsc();			\
	/*tsc_cpu = sched_getcpu();*/		\
	tsc_cpu = GETCPU();			\
	tid = gettid();				\
    }while(0)


#define POSTAMBLE() do{							\
	end_tsc = rdtsc();						\
	insert_trace(tid, ret_addr, tsc_cpu, begin_tsc, end_tsc);	\
	/* Tracer::instance()->insert_trace(tid, ret_addr, tsc_cpu, begin_tsc, end_tsc);*/ \
    }while(0)


/*
 * Hook for "sleep".
 */
unsigned int sleep(unsigned int seconds)
{
    typedef unsigned int (*sleep_func)(unsigned int);
    static sleep_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "[%d]: SLEEP!\n", getpid());

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (sleep_func)dlsym(RTLD_NEXT, "sleep");
	}

	retVal = underlying(seconds);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "nanosleep".
 */
int nanosleep(const struct timespec *req, struct timespec *rem)
{
    typedef int (*nanosleep_func)(const struct timespec *, struct timespec *);
    static nanosleep_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    // fprintf(stderr, "NANOSLEEP!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (nanosleep_func)dlsym(RTLD_NEXT, "nanosleep");
	}

	retVal = underlying(req, rem);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "usleep".
 */
int usleep(useconds_t usec)
{
    typedef int(*usleep_func)(int);
    static usleep_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    // fprintf(stderr, "USLEEP!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (usleep_func)dlsym(RTLD_NEXT, "usleep");
	}

	retVal = underlying(usec);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "clock_nanosleep".
 */
int clock_nanosleep(clockid_t clock_id, int flags, const struct timespec *request, struct timespec *remain)
{
    typedef int(*clock_nanosleep_func)(clockid_t, int, const struct timespec *, struct timespec *);
    static clock_nanosleep_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "CLOCK_NANOSLEEP!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (clock_nanosleep_func)dlsym(RTLD_NEXT, "clock_nanosleep");
	}

	retVal = underlying(clock_id, flags, request, remain);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "setitimer".
 */
int setitimer(__itimer_which_t which, const struct itimerval *new_value, struct itimerval *old_value)
{
    typedef int (*setitimer_func)(int, const struct itimerval *, struct itimerval *);
    static setitimer_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "SETITIMER!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (setitimer_func)dlsym(RTLD_NEXT, "setitimer");
	}

	retVal = underlying(which, new_value, old_value);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "timer_settime"
 */
int timer_settime(timer_t timerid, int flags, const struct itimerspec *new_value, struct itimerspec *old_value)
{
    typedef int (*timer_settime_func)(timer_t, int, const struct itimerspec *, struct itimerspec *);
    static timer_settime_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "TIMER_SETTIME!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (timer_settime_func)dlsym(RTLD_NEXT, "timer_settime");
	}

	retVal = underlying(timerid, flags, new_value, old_value);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "sem_timedwait"
 */
int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout)
{
    typedef int (*sem_timedwait_func)(sem_t *, const struct timespec *);
    static sem_timedwait_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "SEM_TIMEDWAIT!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (sem_timedwait_func)dlsym(RTLD_NEXT, "sem_timedwait");
	}

	retVal = underlying(sem, abs_timeout);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "sigtimedwait"
 */
int sigtimedwait(const sigset_t *set, siginfo_t *info, const struct timespec *timeout)
{
    typedef int (*sigtimedwait_func)(const sigset_t *, siginfo_t *, const struct timespec *);
    static sigtimedwait_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "SIGTIMEDWAIT!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (sigtimedwait_func)dlsym(RTLD_NEXT, "sigtimedwait");
	}

	retVal = underlying(set, info, timeout);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "select".
 */
int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout)
{
    typedef int(*select_func)(int, fd_set *, fd_set *, fd_set *, struct timeval *);
    static select_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "SELECT!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (select_func)dlsym(RTLD_NEXT, "select");
	}

	retVal = underlying(nfds, readfds, writefds, exceptfds, timeout);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "pselect".
 */
int pselect(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, const struct timespec *timeout, const sigset_t *sigmask)
{
    typedef int(*pselect_func)(int, fd_set *, fd_set *, fd_set *, const struct timespec *, const sigset_t *);
    static pselect_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "PSELECT!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (pselect_func)dlsym(RTLD_NEXT, "pselect");
	}

	retVal = underlying(nfds, readfds, writefds, exceptfds, timeout, sigmask);

    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "alarm".
 */
unsigned int alarm(unsigned int seconds)
{
    typedef unsigned int (*alarm_func)(unsigned int);
    static alarm_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "ALARM!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (alarm_func)dlsym(RTLD_NEXT, "alarm");
	}

	retVal = underlying(seconds);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "ualarm".
 */
useconds_t ualarm(useconds_t seconds, useconds_t interval)
{
    typedef unsigned int (*ualarm_func)(useconds_t, useconds_t);
    static ualarm_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "UALARM\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (ualarm_func)dlsym(RTLD_NEXT, "ualarm");
	}

	retVal = underlying(seconds, interval);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "poll".
 */
int poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
    typedef int (*poll_func)(struct pollfd *, nfds_t, int);
    static poll_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    // db_fprintf(stderr, "POLL!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (poll_func)dlsym(RTLD_NEXT, "poll");
	}

	retVal = underlying(fds, nfds, timeout);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "ppoll".
 */
int ppoll(struct pollfd *fds, nfds_t nfds, const struct timespec *timeout, const sigset_t *sigmask)
{
    typedef int (*ppoll_func)(struct pollfd *, nfds_t, const struct timespec *, const sigset_t *);
    static ppoll_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "PPOLL!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (ppoll_func)dlsym(RTLD_NEXT, "ppoll");
	}

	retVal = underlying(fds, nfds, timeout, sigmask);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "epoll_wait".
 */
int epoll_wait(int epfd, struct epoll_event *events, int maxevents, int timeout)
{
    typedef int (*epoll_wait_func)(int, struct epoll_event *, int, int);
    static epoll_wait_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "EPOLL_WAIT\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (epoll_wait_func)dlsym(RTLD_NEXT, "epoll_wait");
	}

	retVal = underlying(epfd, events, maxevents, timeout);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "epoll_pwait"
 */
int epoll_pwait(int epfd, struct epoll_event *events, int maxevents, int timeout, const sigset_t *sigmask)
{
    typedef int (*epoll_pwait_func)(int, struct epoll_event *, int, int, const sigset_t *);
    static epoll_pwait_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "EPOLL_PWAIT!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (epoll_pwait_func)dlsym(RTLD_NEXT, "epoll_pwait");
	}

	retVal = underlying(epfd, events, maxevents, timeout, sigmask);
    }
    POSTAMBLE();

    return retVal;
};

/*
 * Hook for "pthread_cond_timedwait"
 */
int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *abstime)
{
    typedef int (*pthread_cond_timedwait_func)(pthread_cond_t *, pthread_mutex_t *, const struct timespec *);
    static pthread_cond_timedwait_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "COND TIMEDWAIT!\n");

    PREAMBLE();
    {
	if(!underlying){
	    underlying = (pthread_cond_timedwait_func)dlsym(RTLD_NEXT, "pthread_cond_timedwait");
	}

	retVal = underlying(cond, mutex, abstime);
    }
    POSTAMBLE();

    return retVal;
};

int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine) (void *), void *arg)
{
    typedef int (*pthread_create_func)(pthread_t *, const pthread_attr_t *, void *(*start_routine)(void *), void *);
    static pthread_create_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "PTHREAD_CREATE HOOK!\n");

    // PREAMBLE();
    {
	if(!underlying){
	    underlying = (pthread_create_func)dlsym(RTLD_NEXT, "pthread_create");
	}

	retVal = underlying(thread, attr, start_routine, arg);
    }
    // POSTAMBLE();

    return retVal;
};

#if 0
int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex)
{
    typedef int (*pthread_cond_wait_func)(pthread_cond_t *, pthread_mutex_t *);
    static pthread_cond_wait_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    db_fprintf(stderr, "WAIT!\n");
    // PREAMBLE();
    {
	if(!underlying){
	    underlying = (pthread_cond_wait_func)dlsym(RTLD_NEXT, "pthread_cond_wait");
	}

	retVal = underlying(cond, mutex);
    }
    // POSTAMBLE();

    return retVal;
};
#endif

/*
 * Hook for "pthread_cond_destroy".
 * Present only to avoid freezes
 * on cond destroy invocations.
 */
int pthread_cond_destroy(pthread_cond_t *cond)
{
    typedef int (*pthread_cond_destroy_func)(pthread_cond_t *);
    static pthread_cond_destroy_func underlying = NULL;

    if(!underlying){
	underlying = (pthread_cond_destroy_func)dlsym(RTLD_NEXT, "pthread_cond_destroy");
    }

    return underlying(cond);
};

/*
 * Hook for "pthread_cond_broadcast".
 * Present only to avoid freezes
 * on cond broadcast invocations.
 */
int pthread_cond_broadcast(pthread_cond_t *cond)
{
    typedef int (*pthread_cond_broadcast_func)(pthread_cond_t *);
    static pthread_cond_broadcast_func underlying = NULL;

    if(!underlying){
	underlying = (pthread_cond_broadcast_func)dlsym(RTLD_NEXT, "pthread_cond_broadcast");
    }

    return underlying(cond);
};

void hook_lib_initialize();
void __terminate();

void __initialize()
{
    db_fprintf(stderr, "[%d]: HELPER\n", getpid());
};

inline int inc_dec_num_shm_clients(int num)
{
    if(!fork_shm_data){
	return -ERROR;
    }
    shm_data_t *data = fork_shm_data + SERVER_SHM_OFFSET;
    if(sem_wait(&data->sem)){
	perror("sem_wait error");
	exit(-ERROR);
    }
    data->count += num;
    int retVal = data->count;
    if(sem_post(&data->sem)){
	perror("sem_post error");
	exit(-ERROR);
    }

    db_fprintf(stderr, "Debug: incremented in hook_lib: value = %d\n", retVal);
    return retVal;
};

#define INC_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(1)
#define GET_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(0)
#define DEC_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(-1)

#define DO_NOTHING() pw_atomic_read(&do_nothing)
#define SHOULD_HOOK() ( pw_atomic_read(&do_nothing) == 0 )
// #define SHOULD_HOOK() true

/*
 * We need to hijack 'fork' to ensure
 * the PRELOAD lib gets loaded in the
 * child context. Note that this is
 * really required ONLY if the
 * child does NOT do an exec (since
 * 'exec' auto loads the PRELOAD lib
 * in the child context), but it's
 * OK to duplicate it here. Note also
 * that we do NOT actually trace calls to 'fork'
 * (hence no "PREAMBLE()" and "POSTAMBLE()").
 */
pid_t fork(void)
{
    typedef pid_t (*fork_func)(void);
    static fork_func underlying = NULL;
    void *ret_addr = __builtin_return_address(0);
    int retVal = -1;

    void *handle = NULL;

    {
	if(!underlying){
	    underlying = (fork_func)dlsym(RTLD_NEXT, "fork");
	}

	db_fprintf(stderr, "Trying to inc in %d, addr = 0x%lx\n", getpid(), fork_shm_data);

	if(pw_atomic_read(&do_nothing) == 0){
	    INC_NUM_SHM_CLIENTS();
	}

	retVal = underlying();

	if(retVal == -1 && SHOULD_HOOK()){
	    // ERROR!
	    DEC_NUM_SHM_CLIENTS();
	}

	{
	    if(!retVal && SHOULD_HOOK()){
		/* Child */
		char *env = getenv("LD_PRELOAD");
		db_fprintf(stderr, "FORK: pid = %d, ppid = %d, env = %p\n", getpid(), getppid(), env);
		__initialize();
		/*
		 * Make sure a new instance of
		 * the 'hook_lib' gets initialized.
		 */
		hook_lib_initialize();
		// handle = dlopen("/home/gupadhya/programs/single_seg_separate_hook_lib/libhook_lib.so.1", RTLD_NOW);
		// typedef void (*constructor_func)(void);
		// constructor_func cons = (constructor_func)dlsym(handle, "hook_lib_initialize");
		// cons();
	    }
	}
    }

    return retVal;
};

pid_t vfork(void)
{
    db_fprintf(stderr, "VFORK!\n");
    assert(false);
    exit(-ERROR);
};

int clone(int (*fn)(void *), void *child_stack, int flags, void *arg, ...)
{
    db_fprintf(stderr, "CLONE!\n");
    assert(false);
    exit(-ERROR);
};

int daemon(int nochdir, int noclose)
{
    db_fprintf(stderr, "DAEMON!\n");
#if 0
    switch(fork()){
    case -1:
	// ERROR
	return -ERROR;
    case 0:
	// CHILD
	break;
    default:
	// PARENT
	exit(0);
    }
#endif
#if 1
    pid_t pid = fork();
    if(pid < 0){
	// ERROR
	return -ERROR;
    }
    if(pid > 0){
	// PARENT
	exit(0);
    }
    /*
     * CHILD i.e. the DAEMON process.
     * For now, do nothing.
     */
    if(setsid() == -1){
	perror("setsid error");
	exit(-ERROR);
    }
    if(!nochdir){
	if(chdir("/")){
	    perror("chdir error");
	    exit(-ERROR);
	}
    }
    if(!noclose){
	int fd = open("/dev/null", O_RDWR, 0);
	if(fd > 0){
	    dup2(fd, STDIN_FILENO);
	    dup2(fd, STDOUT_FILENO);
	    dup2(fd, STDERR_FILENO);
	    if(fd > 2){
		close(fd);
	    }
	}else{
	    return -ERROR;
	}
    }
    return SUCCESS;
#endif
};

void _exit(int status)
{
    typedef void (*_exit_func)(int) __attribute__((noreturn));
    _exit_func underlying = (_exit_func)dlsym(RTLD_NEXT, "_exit");
    fprintf(stderr, "_EXIT!\n");
    /*
     * "_exit(...)" exits IMMEDIATELY (doesn't allow
     * the library destructor to fire). To get
     * around this, we call the termination
     * routines manually.
     */
    __terminate();
    underlying(status);
};

/*
 * Where should the lib write its
 * results?
 */
static char *output_file_name = NULL;

/*
 * Get O/P dir info -- read the file "output_dir.txt"
 * (which is auto set in the 'runExecs' script file
 * based on user-supplied information).
 */
#if 0
static void set_output_filename(void)
{
    char *line = NULL;
    char filename[1024];
    FILE *fp = fopen("output_dir.txt", "r");
    if(!fp){
	perror("fopen error for output dir file");
	{
	    /*
	     * Debugging
	     */
	    const char *pwd = getenv("PWD");
	    db_fprintf(stderr, "pid = %d\n", getpid());
	    db_fprintf(stderr, "PWD in HOOK_LIB::set_output_filename() = %s\n", pwd);
	}
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
    std::string output_dir_str(line);
    output_dir_str.append("/lib_output_");
    char pid_str[10];
    sprintf(pid_str, "%d", getpid());
    output_dir_str.append(pid_str);
    output_dir_str.append(".txt");
    output_file_name = strdup(output_dir_str.c_str());
    // db_fprintf(stderr, "OUTPUT FILE NAME = %s\n", output_file_name);
};
#else
/*
 * Be specific about WHICH directory
 * to troll for the 'driver_output.txt' file.
 * Required to get "DreamChess" working (and
 * hooked).
 */
static void set_output_filename(void)
{
    char *line = NULL;
    char filename[1024];
    char *pwd = getenv("PWD");
    assert(pwd);
    sprintf(filename, "%s/output_dir.txt", pwd);
    // db_fprintf(stderr, "FILE NAME = %s\n", filename);
    FILE *fp = fopen(filename, "r");
    if(fp)
    {
	size_t len = 0;
	ssize_t read=0;

	read = getline(&line, &len, fp);
	line[read-1] = '\0';

	fclose(fp);
    }
    else{
        db_fprintf(stderr, "Warning: could NOT find the \"output_dir.txt\" file -- defaulting to PWD!\n");
        {
            /*
             * Debugging
             */
            db_fprintf(stderr, "pid = %d\n", getpid());
            db_fprintf(stderr, "PWD in HOOK_LIB::set_output_filename() = %s\n", pwd);
        }
        // line = const_cast<char *>(".");
        line = pwd;
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
    std::string output_dir_str(line);
    output_dir_str.append("/lib_output_");
    char pid_str[10];
    sprintf(pid_str, "%d", getpid());
    output_dir_str.append(pid_str);
    output_dir_str.append(".txt");
    output_file_name = strdup(output_dir_str.c_str());
    db_fprintf(stderr, "OUTPUT FILE NAME = %s\n", output_file_name);
};
#endif // if 1

void sigint_handler(int signum)
{
    db_fprintf(stderr, "DEBUG: <PWR> SIGINT received!\n");
    exit(1);
};

void sigusr1_handler(int signum)
{
    db_fprintf(stderr, "DEBUG: <PWR> SIGUSR1 received!\n");
    exit(1);
};

int init_uds_socket()
{
    struct sockaddr_un servaddr;

    std::string uds_file_name_str = WUWATCH_UNIXSTR_PATH;
    /*
     * Update: actual UNIX domain socket file
     * name will be "/tmp/wuwatch-{$USER}" to
     * avoid perm issues.
     */
    {
	const char *user = getenv("USER");
	if(user){
	    uds_file_name_str += "-";
	    uds_file_name_str += user;
	}
    }

    uds_file_name = uds_file_name_str.c_str();
    db_fprintf(stderr, "UNIX Domain Socket file name = %s\n", uds_file_name);

    if( (uds_sockfd = socket(AF_LOCAL, SOCK_STREAM, 0)) < 0){
	perror("socket error");
	return -ERROR;
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sun_family = AF_LOCAL;
    // strcpy(servaddr.sun_path, WUWATCH_UNIXSTR_PATH);
    strcpy(servaddr.sun_path, uds_file_name);

    if(connect(uds_sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr))){
	if(false){
	    perror("connect error");
	}
	return -ERROR;
    }
    return SUCCESS;
};


void destroy_uds_socket()
{
    close(uds_sockfd);
};

void init_shm()
{
    int fd = -1;
    int len = sysconf(_SC_PAGE_SIZE), num = len / sizeof(shm_data_t);

    if( (fd = shm_open(PW_SHM_PATH, O_RDWR | O_CREAT, PW_SHM_MODE)) < 0){
	perror("open error");
	exit(-ERROR);
    }
    if( (fork_shm_data = (shm_data_t *)mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == NULL){
	perror("mmap error");
	exit(-ERROR);
    }
    close(fd);
};

void destroy_shm()
{
    if(!fork_shm_data)
	return;
    int len = sysconf(_SC_PAGE_SIZE);
    munmap(fork_shm_data, len);
    fork_shm_data = NULL;
    // NOP
};

void send_syn(int fd, uds_msg_type_t type, pid_t pid)
{
    uds_msg_t msg;

    msg.type = type;
    msg.child_pid = pid;

    if(write(fd, &msg, sizeof(msg)) != sizeof(msg)){
	perror("write error");
	exit(-ERROR);
    }
};

void get_ack(int fd, uds_msg_type_t type, int *data, int *offset)
{
    uds_msg_t msg;

    if(read(fd, &msg, sizeof(msg)) != sizeof(msg)){
	// perror("read error");
	db_fprintf(stderr, "read error for type = %d in PID = %d: %s\n", type, getpid(), strerror(errno));
	exit(-ERROR);
    }
    assert(msg.type == type);
    if(data){
	*data = msg.dev_fd;
    }
    if(offset){
	*offset = msg.offset;
    }
    return;
};

#define GET_MY_SHM_DATA() (fork_shm_data + shm_offset)

void *uds_client_thread(void *dummy)
{
    pid_t pid = getpid();
    int dev_fd = -1;
    /*
     * Send FORK_SYN...
     */
    send_syn(uds_sockfd, FORK_SYN, pid);
    /*
     * ...and wait for ACK...
     */
    get_ack(uds_sockfd, FORK_ACK, &dev_fd, &shm_offset);
    /*
     * ...before closing the dev_fd and sockfd
     */
    db_fprintf(stderr, "CLIENT %d [TID = %d] got dev_fd = %d, offset = %d\n", pid, gettid(), dev_fd, shm_offset);
    // close(dev_fd); close(uds_sockfd);
    close(uds_sockfd); 
    /*
     * OK, we've given the server a chance to init
     * our entry. Go ahead and mmap it.
     */
    // init_shm();
    /*
     * Now wait for either the server
     * or the parent thread to
     * contact us
     */
    shm_data_t *data = GET_MY_SHM_DATA();
    uds_msg_type_t type;
    sem_wait(&data->sem);
    {
	/*
	 * Somebody sent us a
	 * message.
	 */
	type = data->msg.type;
    }
    sem_post(&data->sem);
    db_fprintf(stderr, "%d: GOT msg type = %d\n", pid, type);
    switch(type){
    case EXIT_SYN:
	/*
	 * Notification from parent thread
	 * to quit (sent from '__terminate()'. 
	 * Send 'EXIT_SYN' to the server
	 * (but connect to it first!).
	 */
	if(init_uds_socket() == SUCCESS){
	    send_syn(uds_sockfd, EXIT_SYN, pid);
	    get_ack(uds_sockfd, EXIT_ACK, NULL, NULL);
	    destroy_uds_socket();
	}
	break;
    case DO_QUIT:
	/*
	 * 'wuwatch' told us to quit
	 */
	db_fprintf(stderr, "[%d]: RECEIVED A QUIT message from server!\n", pid);
	uds_client_status = false; // '__terminate()' should NOT do a join on us!
	// __terminate();
	exit(SUCCESS);
	break;
    default:
	db_fprintf(stderr, "INVALID msg type = %d\n", type);
	exit(-ERROR);
    }
    pthread_exit(PTHREAD_CANCELED);
};


/*
 * This is a GCC-specific attribute.
 * The compiler ensures the code within the "constructor"
 * is executed BEFORE any lib code.
 *
 * Initialize locks etc. here.
 */
void __attribute__ ((constructor)) hook_lib_initialize(void)
{
    db_fprintf(stderr, "[%d]: CONSTRUCTOR!\n", getpid());
    /*
     * Init should be ONE-TIME ONLY!
     */
    if(false && PW_CAS(&lib_inited.count, 0, 1) == false){
        /*
         * Someone else has already initialized
         * this instance of the hook lib
         */
        db_fprintf(stderr, "[%d] WARNING: lib instance (0x%lx) ALREADY INITIALIZED!\n", getpid(), &lib_inited.count);
        return;
    }else{
        db_fprintf(stderr, "[%d] OK: lib instance (0x%lx) NOW INITIALIZED!\n", getpid(), &lib_inited.count);
    }
    trace_map = new trace_map_t;
    assert(trace_map);
    /*
     * Init the hash locks here.
     */
    for(int i=0; i<NUM_HASH_LOCKS; ++i){
        if(pthread_mutex_init(&hashLocks[i], NULL)){
            perror("pthread_mutex_init error");
            exit(-ERROR);
        }
    }

    set_output_filename();

    signal(SIGINT, &sigint_handler);
    // signal(SIGUSR1, &sigusr1_handler);

    if(init_uds_socket() == SUCCESS){
        init_shm();
        if(pthread_create(&uds_client_thread_id, NULL, &uds_client_thread, NULL)){
            perror("pthread_create error");
            exit(-ERROR);
        }
        uds_client_status = true;
        /*
         * We should ideally wait for the
         * thread to signal it's sent
         * the 'FORK_SYN' to 'wuwatch'
         */
    }
};

/*
 * Should the O/P be in binary format?
 */
#define DO_BINARY_DUMP DO_DUMP_BINARY_TRACE 

void __terminate()
{
    FILE *fp = NULL;
    db_fprintf(stderr, "[%d]: __TERMINATE!\n", getpid());
#if 0
    {
	char tmp_name[100];
	sprintf(tmp_name, "%d_%d.txt", getpid(), gettid());
	FILE *tmp_fp = fopen(tmp_name, "a");
	if(!tmp_fp){
	    perror("fopen error");
	    exit(-1);
	}
	db_fprintf(tmp_fp, "%d->%d\n", gettid(), gettid());
	// fflush(tmp_fp);
	fclose(tmp_fp);
    }
#endif
    /*
     * Lock everyone else out
     */
    if(PW_CAS(&do_nothing.count, 0, 1) == false){
        /*
         * Someone else has already called
         * terminate.
         */
        return;
    }

    if(false && PW_CAS(&lib_inited.count, 1, 0) == false){
        db_fprintf(stderr, "[%d] WARNING: Was this instance of the lib not initialized?!\n", getpid());
    }

    if(true){
        if(!output_file_name){
            db_fprintf(stderr, "Error: NO OUTPUT FILE NAME!\n");
            exit(-ERROR);
        }
#if 0
        {
            char tmp_name[100];
            sprintf(tmp_name, "%d_%d.txt", getpid(), gettid());
            FILE *tmp_fp = fopen(tmp_name, "a");
            if(!tmp_fp){
                perror("fopen error");
                exit(-1);
            }
            db_fprintf(tmp_fp, "%d\n", gettid());
            // fflush(tmp_fp);
            fclose(tmp_fp);
        }
#endif
        if( (fp = fopen(output_file_name, "w")) == NULL){
            perror("fopen error in hook lib destructor");
            // exit(-ERROR);
            return;
        }

#if DO_BINARY_DUMP
        {
            Tracer::instance()->serialize_traces(trace_map, fp);
        }
#else
        {
            /*
             * FIRST, dump the version info.
             */
            {
                char ver_str[1024];
                sprintf(ver_str, "%d.%d.%d", HOOK_LIB_VERSION_VERSION, HOOK_LIB_VERSION_INTERFACE, HOOK_LIB_VERSION_OTHER);
                // db_fprintf(fp, "Hook Library Version = %s\n", ver_str);
                fprintf(fp, "Hook Library Version = %s, %d\n", ver_str, gettid());
                fflush(fp);
            }

            dump_traces(fp);
        }
#endif // DO_BINARY_DUMP


        if(trace_map){
	    /*
	      for_each(trace_map->begin(), trace_map->end(), map_clear);
	      trace_map->clear();
	    */

            delete trace_map;
        }

        fclose(fp);

        free(output_file_name);
    }

    /*
     * Also tell the helper thread
     * to shut down.
     */
    if(uds_client_status){
        /*
         * OK, tell worker thread to exit...
         */
        shm_data_t *data = GET_MY_SHM_DATA();
        data->msg.type = EXIT_SYN;
        sem_post(&data->sem);
        /*
         * ...and wait until it does.
         */
        if(pthread_join(uds_client_thread_id, NULL)){
            perror("pthread_join error");
            exit(-ERROR);
        }
    }
    destroy_uds_socket();
    destroy_shm();
};

/*
 * This is a GCC-specific attribute.
 * The compiler ensures the code within the "destructor"
 * is executed AFTER any lib code.
 *
 * Print statistics, print [TID::stack-index] bindings
 * and free memory here.
 */
void __attribute__ ((destructor)) hook_lib_terminate(void)
{
    __terminate();

    /*
     * ALL DONE!
     */
    // exit(SUCCESS);
};
