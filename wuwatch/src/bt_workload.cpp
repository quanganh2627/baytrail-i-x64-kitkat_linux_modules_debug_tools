#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <string.h>
#include <poll.h>
#include <sys/time.h>
#include <signal.h>
#include <errno.h>
#include <semaphore.h>
#include <assert.h>

#include <vector>
#include <string>


#define SUCCESS 0
#define ERROR 1

/*
 * Debug printfs etc.
 */
#define DB_FPRINTF(...) do {			\
	if(env_pw_do_debug_output){		\
	    fprintf(__VA_ARGS__);		\
	}					\
    }while(0)

#define DB_ASSERT(e, ...) do {			\
	if(env_pw_do_debug_output && !(e)){	\
	    fprintf(stderr, __VA_ARGS__);	\
	    assert(false);			\
	}					\
    }while(0)

#define DB_ABORT(...) do {			\
	if(env_pw_do_debug_output){		\
	    fprintf(stderr, __VA_ARGS__);	\
	    assert(false);			\
	}					\
    }while(0)

#define TRACE_FUNC() DB_FPRINTF(stderr, "%s\n", __FUNCTION__)
#define TRACE_ITER(i) DB_FPRINTF(stderr, "Iter = %d\n", i)

#define LOCK(l) {				\
    if(pthread_mutex_lock(&(l))){		\
	perror("pthread_mutex_lock error");	\
	exit(-ERROR);				\
    }						\

#define UNLOCK(l)				\
    if(pthread_mutex_unlock(&(l))){		\
	perror("pthread_mutex_unlock error");	\
	exit(-ERROR);				\
    }						\
    }

/*
 * Useful typedefs.
 */
typedef std::string str_t;
typedef int (*work_func_t)(void);

/*
 * Data structures.
 */
struct option{
    str_t long_name;
    work_func_t work_func;
    bool is_multi_threaded;
};

/*
 * Benchmark function declarations.
 */
int sleep_func(void); // "-s"
int usleep_func(void); // "-u"
int nanosleep_func(void); // "-n"
int clock_nanosleep_func(void); // "-c"
int select_func(void); // "-l"
int pselect_func(void); // "-e"
int poll_func(void); // "-p"
int ppoll_func(void); // "-o"
int timedwait_func(void); // "-t"
int alarm_func(void); // "-r"
int itimer_func(void); // "-i"
int ualarm_func(void); // "-m"
int sem_timedwait_func(void); // "-f"
int sigtimedwait_func(void); // "-g"
int daemon_func(void); // "-d"


/*
 * Variable declarations.
 */
/*
 * Should we do debug printfs etc?
 * Populated via an (external) environment
 * variable.
 */
static bool env_pw_do_debug_output=false;
/*
 * Mutex for the "pthread_cond" family of
 * benchmarks.
 */
static pthread_mutex_t global_mutex = PTHREAD_MUTEX_INITIALIZER;
/*
 * Condition variable for the "pthread_cond" 
 * family of benchmarks.
 */
static pthread_cond_t global_cond = PTHREAD_COND_INITIALIZER;
/*
 * How many options are there?
 */
static int total_num_options = -1;
/*
 * The list of options.
 */
static option options[] = {						\
    {"--sleep", &sleep_func, true},					\
    {"--usleep", &usleep_func, true},					\
    {"--nanosleep", &nanosleep_func, true},				\
    {"--clock_nanosleep", &clock_nanosleep_func, true},			\
    {"--select", &select_func, true},					\
    {"--pselect", &pselect_func, true},					\
    {"--poll", &poll_func, true},					\
    {"--ppoll", &ppoll_func, true},					\
    {"--timedwait", &timedwait_func, true},				\
    {"--alarm", &alarm_func, false},					\
    {"--itimer", &itimer_func, false},					\
    {"--ualarm", &ualarm_func, false},					\
    {"--sem_timedwait", &sem_timedwait_func, true},			\
    {"--sigtimedwait", &sigtimedwait_func, true},			\
    {"--daemon", &daemon_func, false},					\
    {"", NULL, false} /* MUST ALWAYS BE LAST ENTRY IN OPTIONS ARRAY!!! */ \
};

/*
 * Benchmark functions (and helpers).
 */
int sleep_func(void)
{
    int num_iters = 5;
    int num_secs = 1;

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        sleep(num_secs);
    }
    return SUCCESS;
};

int usleep_func(void)
{
    int num_iters = 10;
    int num_usecs = 500000; // 0.5 secs

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        usleep(num_usecs);
    }

    return SUCCESS;
};

int nanosleep_func(void)
{
    int num_iters = 10;
    int num_secs = 0, num_nsecs = 500000000; // 0.5 secs

    TRACE_FUNC();

    struct timespec ts;

    ts.tv_sec = num_secs; ts.tv_nsec = num_nsecs;

    for(int i=0; i<num_iters; ++i){
        nanosleep(&ts, NULL);
    }

    return SUCCESS;
};

int clock_nanosleep_func(void)
{
    int num_iters = 10;
    int num_secs = 0, num_nsecs = 500000000; // 0.5 secs
    struct timespec ts;

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        ts.tv_sec = num_secs;
        ts.tv_nsec = num_nsecs;

        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
    }

    return SUCCESS;
};

int select_func(void)
{
    int num_iters = 10;
    int num_secs = 0, num_usecs = 500000; // 0.5 secs
    struct timeval tv;

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        tv.tv_sec = num_secs; tv.tv_usec = num_usecs;
        select(0, NULL, NULL, NULL, &tv);
    }

    return SUCCESS;
};

int pselect_func(void)
{
    int num_iters = 10;
    int num_secs = 0, num_nsecs = 500000000; // 0.5 secs
    struct timespec ts;

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        ts.tv_sec = num_secs; ts.tv_nsec = num_nsecs;
        pselect(0, NULL, NULL, NULL, &ts, NULL);
    }

    return SUCCESS;
};

int poll_func(void)
{
    int num_iters = 10;
    int num_msecs = 500; // 0.5 secs

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        poll(NULL, 0, num_msecs);
    }

    return SUCCESS;
};

int ppoll_func(void)
{
    int num_iters = 10;
    int num_secs = 0, num_nsecs = 500000000; // 0.5 secs
    struct timespec ts;

    TRACE_FUNC();

    for(int i=0; i<num_iters; ++i){
        ts.tv_sec = num_secs; ts.tv_nsec = num_nsecs;
        ppoll(NULL, 0, &ts, NULL);
    }

    return SUCCESS;
};

int timedwait_func(void)
{
    int num_iters = 5;
    uint32_t secs = 1, nsecs = 0; // 1 sec
    struct timeval tv;
    struct timespec ts;

    TRACE_FUNC();

    memset(&ts, 0, sizeof(ts));
    memset(&tv, 0, sizeof(tv));

    for(int i=0; i<num_iters; ++i){
        LOCK(global_mutex);
        {
            /*
             * time arg to timedwait is NOT
             * a delta -- it is actually an
             * abs time value. Use "gettimeofday"
             * to get the current time, and then
             * add the desired sleep interval
             * to that.
             */
            if(gettimeofday(&tv, NULL)){
                perror("gettimeofday error");
                exit(-ERROR);
            }
            /*
	     */
            ts.tv_sec = tv.tv_sec + secs; ts.tv_nsec = tv.tv_usec * 1000 + nsecs;

            /*
             * Ideally you'd like to check the return values from the timedwait, but
             * we don't because we're not really worried about error values here.
             */
            pthread_cond_timedwait(&global_cond, &global_mutex, &ts);
        }
        UNLOCK(global_mutex);
    }
    return SUCCESS;
};

void sigalrm_handler(int signum)
{
    /* NOP */
};

typedef enum{
    ALARM=0,
    UALARM,
    ITIMER
}alarm_type_t;

int do_alarm_or_itimer(const alarm_type_t& type)
{
    int num_iters = 5;
    int num_secs = 1, num_usecs = 0;
    struct itimerval itv;
    struct timeval tv;
    bool do_alarm = false;
    /*
     * 'alarm()', 'ualarm() and 'setitimer()'
     * all rely on sending a SIGALRM. Register
     * a handler first.
     */
    signal(SIGALRM, &sigalrm_handler);
    /*
     * Setup the interval timer / alarm, ualarm
     */
    switch(type){
    case ALARM:
	do_alarm = true;
	alarm(num_secs);
	break;
    case UALARM:
	/*
	 * CANNOT have a '1 second' timeout -- hangs!
	 */
	ualarm(999999, 999999); // wakeup after 0.999 seconds and every 0.999 seconds after that
	break;
    case ITIMER:
	/*
	 * Set timer interval
	 */
	itv.it_value.tv_sec = num_secs; itv.it_value.tv_usec = num_usecs; // Timer will expire after [sec, usecs]...
	itv.it_interval.tv_sec = num_secs; itv.it_interval.tv_usec = num_usecs; // ... and every [secs, usecs] after that

	if(setitimer(ITIMER_REAL, &itv, NULL)){
	    perror("setitimer error");
	    return -ERROR;
	}
	break;
    default:
	DB_FPRINTF(stderr, "ERROR: unknown alarm type = %d\n", type);
	return -ERROR;
    }
    /*
     * Now loop 'num_iters' times, taking care
     * to call 'pause()' to avoid having to
     * busy-wait.
     */
    for(int i=0; i<num_iters; ++i){
        pause();
        /*
         * If we get here ==> the alarm/ualarm/itimer fired.
         * Reset it.
         */
        if(do_alarm){
            alarm(num_secs);
        } else {
            /*
             * Interval timer / ualarm; nothing to do.
             */
        }
    }
    /*
     * Benchmark done. We need to cleanup...
     */
    if(type == ITIMER){
        /*
         * Unregister interval timer.
         */
        memset(&itv.it_value, 0, sizeof(struct timeval));
        setitimer(ITIMER_REAL, &itv, NULL);
    }
    return SUCCESS;
};

int alarm_func(void)
{
    TRACE_FUNC();
    return do_alarm_or_itimer(ALARM);
};

int itimer_func(void)
{
    TRACE_FUNC();
    return do_alarm_or_itimer(ITIMER);
};

int ualarm_func(void)
{
    TRACE_FUNC();
    return do_alarm_or_itimer(UALARM);
};

int sem_timedwait_func(void)
{
    int num_iters = 5;
    int num_secs = 1, num_nsecs = 0;
    struct timeval tv;
    struct timespec ts;
    sem_t sem;

    TRACE_FUNC();

    if(sem_init(&sem, 0, 0)){
        perror("sem_init error");
        exit(-ERROR);
    }

    for(int i=0; i<num_iters; ++i){

        if(clock_gettime(CLOCK_REALTIME, &ts)){
            perror("clock_gettime error");
            exit(-ERROR);
        }
        ts.tv_sec += num_secs; ts.tv_nsec += num_nsecs;

        if(ts.tv_nsec >= (long)(1000 * 1e6)){
            fprintf(stderr, "EINVAL!\n");
        }

        if(sem_timedwait(&sem, &ts) && errno != ETIMEDOUT){
            perror("sem_timedwait error");
        }
    }

    return SUCCESS;
};

int sigtimedwait_func(void)
{
    int num_iters = 10;
    // int num_secs = 0, num_nsecs = 500000000; // 0.5 secs
    int num_secs = 1, num_nsecs = 0; // 0.5 secs
    struct timespec ts;
    sigset_t set;

    TRACE_FUNC();

    sigemptyset(&set);

    for(int i=0; i<num_iters; ++i){
        ts.tv_sec = num_secs; ts.tv_nsec = num_nsecs;
        sigtimedwait(&set, NULL, &ts);
    }


    return SUCCESS;
};

int daemon_func(void)
{
    int num_iters = 5;
    int num_secs = 1;

    TRACE_FUNC();

    if(daemon(0, 0)){
        perror("daemon error");
        exit(-ERROR);
    }
    /*
     * OK, we're in the daemon.
     * Call "sleep" to get some
     * backtraces...
     */
    for(int i=0; i<num_iters; ++i){
        sleep(num_secs);
    }

    return SUCCESS;
};


/*
 * Helper functions.
 */
void check_environ_vars()
{
    const char *env = getenv("PW_DO_DEBUG_OUTPUT");
    env_pw_do_debug_output = env && tolower(env[0]) == 'y';
};

int get_num_options(void)
{
    int i=0;
    for(i=0; options[i].work_func; ++i);
    return i;
};

int is_valid_option(const str_t& opt)
{
    for(int i=0; i<total_num_options; ++i)
        if(options[i].long_name == opt)
            return i;
    return -ERROR;
};

int parse_cmdline_args(int argc, char *argv[])
{
    int opt_index = -1;
    DB_ASSERT(argc == 2, "USAGE: ./bt_workload [benchmark name]\n");
    if(argc != 2){
        fprintf(stderr, "USAGE: ./bt_workload [benchmark name]\n");
        return -ERROR;
    }
    return is_valid_option(argv[1]);
};

/*
 * Initialization and termination routines.
 */
void init()
{
    check_environ_vars();
    total_num_options = get_num_options();
};

void terminate()
{
};

struct thread_args{
    int who;
    work_func_t work_func;

    thread_args(int w, work_func_t f):who(w), work_func(f){};
};

void *thread_func(void *arg)
{
    thread_args *targs = (thread_args *)arg;
    int me_ = targs->who;
    work_func_t work_func = targs->work_func;
    // DB_FPRINTF(stderr, "Thread %d STARTED\n", me_);
    work_func();
    // DB_FPRINTF(stderr, "Thread %d STOPPED\n", me_);
    delete targs;
    pthread_exit(PTHREAD_CANCELED);
};

void dispatch(const option& option)
{
    pthread_t *tids = NULL;
    int num_cpus = sysconf(_SC_NPROCESSORS_CONF);
    work_func_t work_func = option.work_func;

    if (option.is_multi_threaded) {
	DB_FPRINTF(stderr, "Benchmark %s is MULTI-THREADED\n", option.long_name.c_str());
	tids = new pthread_t[num_cpus];
	for(int i=0; i<num_cpus; ++i)
	    // if(pthread_create(&tids[i], NULL, thread_func, (void *)work_func)){
	    if(pthread_create(&tids[i], NULL, thread_func, new thread_args(i, work_func))){
		perror("pthread_create error");
		exit(-ERROR);
	    }
	for(int i=0; i<num_cpus; ++i)
	    if(pthread_join(tids[i], NULL)){
		perror("pthread_join error");
	    }
    } else {
	work_func();
    }
};

int main(int argc, char *argv[])
{
    int benchmark_idx = -1;
    work_func_t work_func;
    init();
    {
	if(false){
	    dispatch(options[0]);
	    exit(SUCCESS);
	}
        if( (benchmark_idx = parse_cmdline_args(argc, argv)) >= 0){
            // options[benchmark_idx].work_func();
	    dispatch(options[benchmark_idx]);
        } else {
            fprintf(stderr, "USAGE: ./bt_workload [BENCHMARK NAME]\n");
            fprintf(stderr, "Where [BENCHMARK NAME] is one of:\n");
            for(int i=0; i<total_num_options; ++i){
                fprintf(stderr, "\t%s\n", options[i].long_name.c_str());
            }
        }
    }
    terminate();
};
