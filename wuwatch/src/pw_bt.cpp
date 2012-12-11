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

/* *****************************************
 * Backtrace tracking support functionality
 * *****************************************
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>

#include <map>
#include <vector>
#include <list>
#include <deque>
#include <string>
#include <algorithm>
#include <iostream>

#include <sstream> // for std::stringstream

#include "pw_bt.h"
#include "pw_utils.hpp"

/*
 * Convert 'u64' to 'unsigned long long'
 * Required to get around pesky "invalid format" gcc compiler
 * warnings.
 */
#define TO_ULL(x) (unsigned long long)(x)
/*
 * Convert an arg to 'unsigned long'
 */
#define TO_UL(x) (unsigned long)(x)

typedef std::string str_t;
typedef std::deque<str_t> str_deq_t;
typedef std::vector<str_t> str_vec_t;

Tracer *Tracer::s_tracer = NULL;

Tracer::Tracer(){
};

Tracer::~Tracer(){
};

/*
 * Singleton create function.
 */
Tracer *Tracer::instance(){
    if(!s_tracer){
	s_tracer = new Tracer;
    }
    return s_tracer;
};

/*
 * Singleton destroy function.
 */
void Tracer::destroy(){
    if(s_tracer){
	delete s_tracer;
    }
    s_tracer = NULL;
};

/*
 * INTERNAL helper: write data to disk.
 * @d: ptr to data to write to disk
 * @size: size (in bytes) of data
 * @fp: the output file.
 */
void __write(void *d, int size, FILE *fp)
{
    if(fwrite(d, size, 1, fp) != 1){
	perror("fwrite error");
	exit(-1);
    }
};

/*
 * Helper macro.
 */
#define write_data(d,f) __write(&(d), sizeof(d), f)

/*
 * INTERNAL helper: read data from disk
 * @d: memory location into which data must be read.
 * @size: # of bytes to read.
 * @fp: the input file.
 */
void __read(void *d, int size, FILE *fp)
{
    if(fread(d, size, 1, fp) != 1){
	perror("fread error(1)");
	exit(-1);
    }
};

/*
 * Helper macro
 */
#define read_data(d, f) __read(&(d), sizeof(d), f)

/*
 * INTERNAL helper: write a string to disk.
 * @str: the string to write
 * @fp: the output file.
 */
static inline void write_string(char *str, FILE *fp)
{
    size_t num = strlen(str);
    if(fwrite(str, sizeof(char), num, fp) != num){
	perror("fwrite error");
	exit(-1);
    }
};

/*
 * INTERNAL helper: read a string from disk.
 * @len: the length of the string to read
 * @fp: the input file.
 * @returns: ptr to the string.
 */
static inline char *read_string(int len, FILE *fp)
{
    /*
     * Sanity: make sure the string being read is not TOO large!
     * It's OK to make this check here: 'read_string' is only called
     * to deserialize backtrace information, and it's reasonable
     * to assume a single entry in a call trace won't be larger than
     * 1024 bytes!
     */
    if (len > 1024) {
        fprintf(stderr, "Warning: trace was %d bytes long; truncating to 1024 bytes!\n", len);
        len = 1024;
    }
    char *retVal = (char *)malloc(len+1);
    size_t num = (size_t)len;

    if (!retVal) {
        perror("malloc error");
        return NULL;
    }

    memset(retVal, 0, sizeof(char) * (len+1));

    if(fread(retVal, sizeof(char), num, fp) != num){
	perror("fread error(2)");
	exit(-1);
    }

    return retVal;
};

/*
 * HELPER struct: write a trace to disk.
 * Function operator used by "std::for_each()"
 */
struct trace_serializer{
    FILE *fp;
    trace_serializer(FILE *f):fp(f){};
    void operator()(trace_t *trace){
	trace->serialize(fp);
    };
};

/*
 * HELPER struct: write a vector of
 * traces to disk.
 * Function operator used by "std::for_each()"
 */
struct map_serializer{
    FILE *fp;
    map_serializer(FILE *f):fp(f){};
    void operator()(std::pair<pid_t, trace_vec_t>p){
	trace_vec_t vec = p.second;
        std::vector<trace_t *>::iterator first = vec.begin();
        std::vector<trace_t *>::iterator last = vec.end();
        for (; first != last; ++first)
           (*first)->serialize(fp);
	//for_each(vec.begin(), vec.end(), trace_serializer(fp));
    };
};

/*
 * HELPER struct: count number of unique traces in
 * the trace map.
 * Function operator used by "std::for_each()"
 */
struct key_counter{
    int *key_count;
    key_counter(int *k):key_count(k){};
    void operator()(std::pair<pid_t, trace_vec_t>p){
	(*key_count) += p.second.size();
    };
};

/*
 * Function to serialize a map of traces to disk.
 * @trace_map: the map of traces to serialize
 * @fp: the output file.
 */
void Tracer::serialize_traces(trace_map_t *trace_map, FILE *fp){
    int count=0;
    fprintf(stderr, "TRACE MAP size = %lu\n", TO_UL(trace_map->size()));
    for_each(trace_map->begin(), trace_map->end(), key_counter(&count));
    fprintf(stderr, "Count = %d\n", count);
    // write_data(size, fp);
    write_data(count, fp);
    for_each(trace_map->begin(), trace_map->end(), map_serializer(fp));
};

/*
 * Function to deserialize a set of traces.
 * @fp: the input file.
 * @output: the list of deserialized traces.
 */
void Tracer::deserialize_traces(FILE *fp, trace_vec_t *output){
    int num_traces = -1;
    read_data(num_traces, fp);
    /*
     * Sanity: assume we can't have more than 1024 traces to deserialize.
     */
    assert(num_traces <= 1024);

    fprintf(stderr, "Num_Traces = %d\n", num_traces);
    for(int i=0; i<num_traces; ++i){
	// while(!feof(fp)){
	// fprintf(stderr, "DDD: FP=%p\n", fp);
	trace_t *trace = new trace_t;
	trace->deserialize(fp);
	if(output){
	    output->push_back(trace);
	}
	else{
	    trace->dump_trace(stderr);
	    delete trace;
	}
    }
};

/*
 * Function to deserialize traces.
 * @fp: the input file.
 * @trace_vec: the vector of deserialized traces.
 * @pair_map: deserialized [tid <-> trace vec] mappings.
 */
void Tracer::deserialize_traces(FILE *fp, trace_vec_t& trace_vec, trace_pair_map_t& pair_map)
{
    int num_traces = -1;
    read_data(num_traces, fp);
    /*
     * Sanity: assume we can't have more than 1024 traces to deserialize.
     */
    assert(num_traces <= 1024);

    fprintf(stderr, "Num_Traces = %d\n", num_traces);
    for(int i=0; i<num_traces; ++i){
	trace_t *trace = new trace_t;
	trace->deserialize(fp, pair_map);
	trace_vec.push_back(trace);
    }
};

/*
 * Function to deserialize traces.
 * @fp: the input file.
 * @trace_list: the list of deserialized traces.
 * @pair_map: deserialized [tid <-> trace vec] mappings.
 */
void Tracer::deserialize_traces(FILE *fp, trace_list_t& trace_list, trace_pair_map_t& pair_map)
{
    int num_traces = -1;
    read_data(num_traces, fp);
    /*
     * Sanity: assume we can't have more than 1024 traces to deserialize.
     */
    assert(num_traces <= 1024);

    fprintf(stderr, "Num_Traces = %d\n", num_traces);
    for(int i=0; i<num_traces; ++i){
	trace_t *trace = new trace_t;
	trace->deserialize(fp, pair_map);
	trace_list.push_back(trace);
    }
};

/*
 * INTERNAL helper: remove the first string from 'lines'
 * and return it.
 * @lines: the list of strings
 * @returns: the first string in 'lines'
 */
static std::string get_next_line(std::deque<std::string>& lines)
{
    std::string retVal;

    if(!lines.empty()){
	retVal = lines.front();
	lines.pop_front();
    }

    return retVal;
};

/*
 * INTERNAL helper: peek at the first string in 'lines' (do NOT
 * remove it). Return this string.
 * @lines: the list of strings.
 * @returns: the first string in 'lines'
 */
static std::string peek_next_line(const std::deque<std::string>& lines)
{
    std::string retVal;

    if(!lines.empty()){
	retVal = lines.front();
    }

    return retVal;
};

/*
 * Helper function used by the NEW WUDUMP (special requirements).
 * This function reads all of the traces present in the
 * plain-text input file.
 * In addition to the trace information,
 * it also reads the hook library version.
 *
 * @fp: the input file.
 * @trace_vec: the list of traces read from @fp
 * @pair_map: [tid <-> trace vector] mappings
 * @ver_str: the hook_lib version.
 */
void Tracer::read_traces(FILE *fp, trace_vec_t& trace_vec, trace_pair_map_t& pair_map, std::string& ver_str)
{
    size_t pos;

    str_deq_t lines;
    LineReader::get_all_lines(fp, lines);
    /*
     * First line is VERSION.
     */
    if (lines.empty()) {
        return;
    }

    str_t line = lines.front(); lines.pop_front();

    if(line.find("Hook Library Version") == std::string::npos){
	assert(false);
    }
    pos = line.rfind('=');
    assert(pos != std::string::npos);

    ver_str = line.substr(pos+2, std::string::npos);

    while(!lines.empty()){
	/*
	 * Make sure we're not dealing
	 * with a single "\n" or with an
         * empty line.
	 */
        str_t front = peek_next_line(lines);
        if (front == "\n" || front == "") {
	    lines.pop_front();
	    continue;
	}
	trace_t *trace = new trace_t;
	trace->read(lines, pair_map);
	trace_vec.push_back(trace);
    }
};

TSC_pair::TSC_pair(int& c, const uint64_t& b, const uint64_t& e, tsc_pair_t *n=NULL):cpu(c), begin(b), end(e), next(n){};
TSC_pair::~TSC_pair(){
    if(next){
	delete next;
    }
    next = NULL;
};

trace::trace(pid_t p, pid_t t, void *r):pid(p), tid(t), ret_addr(r), num_trace(0), trace_symbols(NULL), bt_symbols(NULL), head_tsc(NULL), num_tsc_pairs(0){};

trace::~trace(){
    if(trace_symbols){
	free(trace_symbols);
    }

    if(bt_symbols){
	for(int i=0; i<num_trace; ++i){
	    free(bt_symbols[i]);
	}
    }

    trace_symbols = NULL;
    free(bt_symbols);

    if(head_tsc){
	delete head_tsc;
    }
};

/*
 * Function to serialize a single trace.
 * @fp: the output file.
 */
void trace::serialize(FILE *fp){
    tsc_pair_t *curr = NULL;
    std::stringstream stream;
    // PID, TID
    stream << pid << " " << tid;
    // RET ADDR
    stream << " " << ret_addr;
    // # trace entries
    stream << " " << num_trace;
    // Individual backtrace symbols
    for(int j=0; j<num_trace; ++j){
	// fprintf(stderr, "%s\n", trace_symbols[j]);
	stream << " " << trace_symbols[j];
    }

    stream << " " << num_tsc_pairs;
    // Individual TSC pairs
    for_each_tsc_pair(curr, head_tsc){
	// CPU, BEGIN, END
	stream << " " << curr->cpu << " " << curr->begin << " " << curr->end;
    }

    /*
     * Need a terminating space.
     */
    stream << " ";

    std::string str = stream.str();
    int dlen = str.size();
    const char *data = str.c_str();
    write_data(dlen, fp);
    if(fwrite(data, dlen, 1, fp) != 1){
	perror("fwrite error");
	exit(-1);
    }
};
/*
 * Helper funtion. Checks to ensure 'tok' is present
 * in 'line'.
 * @line: the haystack.
 * @tok: the needle.
 */
static void assert_find(const std::string& line, const std::string& tok)
{
    if (true && line.find(tok) == std::string::npos) {
        fprintf(stderr, "Line = %s, tok = %s\n", line.c_str(), tok.c_str());
    }
    assert(line.find(tok) != std::string::npos);
};

/*
 * This function reads a single trace from a
 * plain-text input file.
 */
void trace::read(std::deque<std::string>& lines, trace_pair_map_t& pair_map)
{
    uint64_t begin, end;
    str_t line = get_next_line(lines);
    /*
     * Trace entries are identified (and separated)
     * by:
     * pid <#> tid <#>
     */
    assert_find(line, "pid");
    str_vec_t toks;
    Tokenizer tok(line, " \t");
    /*
     * Step 1: retrieve pid/tid
     */
    toks = tok.get_all_tokens();
    if(toks.size() != 4){
	fprintf(stderr, "Line = %s, Size = %lu\n", line.c_str(), TO_UL(toks.size()));
    }
    assert(toks.size() == 4);
    pid = atoi(toks[1].c_str()); tid = atoi(toks[3].c_str());

    /*
     * In a plain-text output dump, the
     * return address is NOT written.
     *
     * Step 2: retrieve symbolic
     * information.
     */
    line = get_next_line(lines);
    assert_find(line, "symbol\\");

    str_vec_t bt;

    while((line = get_next_line(lines)).find("\\symbol") == std::string::npos){
	/*
	 * These symbols may have a
	 * '\n' at the end. Force remove them
	 * here.
	 */
	bt.push_back(line);
    }

    int numTrace = num_trace = bt.size();
    bt_symbols = (char **)calloc(numTrace, sizeof(char *));
    if (!bt_symbols) {
        perror("calloc error");
        return;
    }
    for(int j=0; j<numTrace; ++j){
    	bt_symbols[j] = strdup(bt[j].c_str());
    }


    /*
     * Subsequent entries are (possibly)
     * TSC pairs. All such pairs are
     * in the format:
     * ts <cpu #> <beg TSC> <end TSC>
     */
    trace_pair_list_t tlist;
    while( (line = peek_next_line(lines)).size() && line.substr(0, 2) == "ts"){
	lines.pop_front();
	Tokenizer tok(line, " \t");
	bt.clear();
	bt = tok.get_all_tokens();
	begin = strtoull(bt[2].c_str(), NULL, 10);
	end = strtoull(bt[3].c_str(), NULL, 10);
	tlist.push_front(trace_pair(begin, end, this));
    }

    pair_map[tid].merge(tlist, std::less<trace_pair_t>());

};

/*
 * Function to deserialize a single trace.
 * @fp: the input file.
 * @pair_map: a list of TID <-> trace mappings.
 */
void trace::deserialize(FILE *fp, trace_pair_map_t& pair_map){
    int len = 0, cpu;
    uint64_t begin, end;

    read_data(len, fp);

    /*
     * Sanity: assume the total size of the trace can't be more than 64KB!
     */
    assert(len <= 65536);

    char *data = new char[len+1];

    if(fread(data, len, 1, fp) != 1){
	perror("fread error");
	exit(-1);
    }

    std::stringstream stream(data);

    // fprintf(stderr, "%s\n", stream.str().c_str());

    // PID, TID
    stream >> pid; stream >> tid;

    // RET ADDR
    stream >> ret_addr;
    // # trace entries
    stream >> num_trace;


    int numTrace = num_trace;
    bt_symbols = (char **)calloc(numTrace, sizeof(char *));
    if (!bt_symbols) {
        perror("calloc error");
        return;
    }
    for(int j=0; j<numTrace; ++j){
	/*
	 * Need to read TWO strings
	 * for each backtrace symbol here. Symbols
	 * are in the format:
	 * <Name> [Addr]
	 * and the space between the name and addr throws
	 * 'stringstream' off.
	 */
	std::string tmp_str_1, tmp_str_2;
	stream >> tmp_str_1; stream >> tmp_str_2;
	tmp_str_1.append(" " + tmp_str_2);
	bt_symbols[j] = strdup(tmp_str_1.c_str());
    }

    stream >> num_tsc_pairs;

    head_tsc = NULL;

    trace_pair_list_t tlist;

    for(int j=0; j<num_tsc_pairs; ++j){
	stream >> cpu; /* Don't care */
	stream >> begin; stream >> end;
	tlist.push_front(trace_pair(begin, end, this));
    }

    pair_map[tid].merge(tlist, std::less<trace_pair_t>());

    delete []data;
};


/*
 * Function to deserialize a single trace.
 * @fp: the input file.
 */
void trace::deserialize(FILE *fp){
    int len = 0, cpu;
    uint64_t begin, end;

    // PID, TID
    read_data(pid, fp); read_data(tid, fp);
    // RET ADDR
    read_data(ret_addr, fp);
    // # trace entries
    read_data(num_trace, fp);
    int numTrace = num_trace;
    /*
     * Sanity: assume the number of entries in the call stack
     * can't be more than 1024.
     */
    assert(numTrace <= 1024);
    bt_symbols = (char **)calloc(numTrace, sizeof(char *));
    for(int j=0; j<numTrace; ++j){
	read_data(len, fp);
        /*
         * Sanity: assume each line in the backtrace cannot be more than 1024 characters long.
         * We introduce an extra check here even though 'len' is checked within 'read_string'
         * because Klockwork complains otherwise.
         */
        assert(len <= 1024);
	bt_symbols[j] = read_string(len, fp);
    }
    read_data(num_tsc_pairs, fp);
    /*
     * Sanity: assume there can be at most 1024 tsc pairs.
     */
    assert(num_tsc_pairs <= 1024);
    for(int j=0; j<num_tsc_pairs; ++j){
	read_data(cpu, fp);
	read_data(begin, fp); read_data(end, fp);
	head_tsc = new tsc_pair_t(cpu, begin, end, head_tsc);
    }
};


/*
 * Helper function: print a trace to disk in
 * the WUDUMP-specific output format.
 * @fp: the output file.
 */
void trace::dump_trace(FILE *fp){
    tsc_pair_t *pair = NULL;

    fprintf(fp, "pid %d tid %d\n", pid, tid);
    fprintf(fp, "symbol\\\n");
    for(int j=0; j<num_trace; ++j)
	fprintf(fp, "%s\n", trace_symbols[j]);
    fprintf(fp, "\\symbol\n");
    for_each_tsc_pair(pair, head_tsc){
	fprintf(fp, "ts %d %16llu %16llu\n", pair->cpu, TO_ULL(pair->begin), TO_ULL(pair->end));
    }
    fprintf(fp, "\n");
};

/*
 * Helper function: dump 'trace' to disk.
 * @trace: the trace to dump.
 * @fp: the output file.
 */
void dump(trace_t *trace, FILE *fp)
{
    trace->dump_trace(fp);
};

trace_dumper::trace_dumper(FILE *f):fp(f){};

/*
 * Function operator.
 * Required for 'std::for_each()'
 */
void trace_dumper::operator()(trace_t *trace){
    trace->dump_trace(fp);
};

trace_pair::trace_pair(const uint64_t& b, const uint64_t& e, trace_t *t):m_begin(b), m_end(e), m_bt(t){};
