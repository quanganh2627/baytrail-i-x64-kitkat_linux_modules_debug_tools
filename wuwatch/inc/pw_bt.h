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

/*
 * File containing declarations/definitions required for 
 * "back trace" generation and tracking.
 */

#ifndef _PW_BT_H_
#define _PW_BT_H_

#include <stdint.h> // for "uint64_t"
/*
 * The following shenanigans are required for "gettid()"
 * because Glibc, in its infinite wisdom, 
 * does not provide a wrapper for "gettid()"! x-(
 */
#include <unistd.h>
#include <sys/syscall.h>

/* **************************************
 * Macros and compile-time constants.
 * **************************************
 */
/*
 * GLIBC doesn't have a native 'gettid()' call -- dup
 * it here.
 */
#define gettid() (pid_t)syscall(SYS_gettid)
/*
 * How many frame addresses do we want for the backtrace?
 */
#define MAX_BACKTRACE_SIZE 20

/* **************************************
 * Typedefs and forward declarations.
 * **************************************
 */
typedef struct TSC_pair tsc_pair_t;
typedef struct trace trace_t;
typedef struct trace_pair trace_pair_t;

typedef std::vector<trace_t *> trace_vec_t;
typedef std::map<pid_t,trace_vec_t> trace_map_t;
typedef std::list<trace_t *> trace_list_t;
typedef std::list<trace_pair_t> trace_pair_list_t;
typedef std::map<pid_t, trace_pair_list_t> trace_pair_map_t;

/* **************************************
 * Data structures.
 * **************************************
 */

/*
 * Struct to hold a <cpu, tsc begin, tsc end> tuple.
 * Part of a linked list.
 */
struct TSC_pair {
    int cpu;
    uint64_t begin, end;
    tsc_pair_t *next;
    TSC_pair(int&, const uint64_t&, const uint64_t&, tsc_pair_t *);
    ~TSC_pair();
};

#define for_each_tsc_pair(p, l) for(p=l; p; p=p->next)

/*
 * Struct to hold backtrace information. This
 * struct is initialized per thread, per return address.
 */
struct trace {
    pid_t pid;
    pid_t tid;
    void *ret_addr;
    unsigned char num_trace;
    char **trace_symbols;
    /*
     * Deserializing uses 'bt_symbols'
     * instead of 'trace_symbols'
     */
    char **bt_symbols;
    int num_tsc_pairs;
    tsc_pair_t *head_tsc;

    void dump_trace(FILE *fp);

    trace(pid_t p=-1, pid_t t=-1, void *r=NULL);
    ~trace();

    void serialize(FILE *fp);
    void deserialize(FILE *fp);
    void deserialize(FILE *fp, trace_pair_map_t&);

    void read(std::deque<std::string>&, trace_pair_map_t&);

private:
    int common_size_i();
};

/*
 * Helper struct to encode <tsc begin, tsc end, ptr-to-backtrace> tuples.
 * Especially useful when trying to bracket TSC pairs.
 */
struct trace_pair {
    uint64_t m_begin, m_end;
    trace_t *m_bt;

    trace_pair(const uint64_t& b, const uint64_t& e, trace_t *t);
};

/*
 * Template specialization for
 * "std::less": used in list 'merge'
 * operation below.
 */
namespace std {
    template <> struct less<trace_pair_t> {
	bool operator()(const trace_pair_t& t1, const trace_pair_t& t2){
	    return t1.m_begin < t2.m_begin;
	};
    };
};

/*
 * Helper class to perform disk I/O:
 * writes individual 'struct trace'
 * values to disk as ASCII text or
 * in binary format.
 * Uses Singleton pattern.
 */
class Tracer {
 public:
    static Tracer *instance();
    static void destroy();

    void serialize_traces(trace_map_t*, FILE *fp=stderr);
    void deserialize_traces(FILE *fp=stderr, trace_vec_t *output=NULL);
    void deserialize_traces(FILE *, trace_vec_t&, trace_pair_map_t&);
    void deserialize_traces(FILE *, trace_list_t&, trace_pair_map_t&);

    void read_traces(FILE *, trace_vec_t&, trace_pair_map_t&, std::string&);

 private:
    Tracer();
    ~Tracer();
    static Tracer *s_tracer;
    FILE *fp;
};

/*
 * Helper struct to dump traces.
 */
struct trace_dumper {
    FILE *fp;
    trace_dumper(FILE *f=stderr);
    void operator()(trace_t *);
};

#endif // _PW_BT_H_
