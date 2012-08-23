/* ***************************************************************************************
 * Copyright (C) 2011 Intel Corporation. All rights reserved.
 * Other names and brands may be claimed sa the property of others.
 * Internal use only -- Do Not Distribute
 * ***************************************************************************************
 */

#include "pw_bt.h"
#include <string>
#include <algorithm>
#include <vector>

#include "pw_structs.h"

typedef std::vector<FILE *> fp_vec_t;
typedef fp_vec_t::iterator fp_iter_t;

typedef struct per_cpu_tsc{
	uint64_t begin, end;
	trace_t *trace;
	per_cpu_tsc(uint64_t&, uint64_t&, trace_t *);
}per_cpu_tsc_t;

per_cpu_tsc::per_cpu_tsc(uint64_t& b, uint64_t& e, trace_t *t):begin(b), end(e), trace(t){};

typedef std::vector<per_cpu_tsc_t> per_cpu_vec_t;

per_cpu_vec_t  *per_cpu_tscs;

int *per_cpu_indices;

int max_num_cpus = -1;

FILE *dev_fp, *lib_fp;


/*
 * How many samples to read at one go?
 */
#define NUM_MSGS_TO_READ 100

typedef std::pair<int,uint64_t> cpu_tsc_pair_t;

typedef std::vector<cpu_tsc_pair_t> cpu_tsc_vec_t;

/*
 * Function to aid in sorting cpu_tsc_pair elements.
 */
bool cpu_tsc_pair_less(const cpu_tsc_pair_t& p1, const cpu_tsc_pair_t& p2)
{
	return p1.second < p2.second;
};

/*
 * Pretty-print a sample to stderr.
 */
void get_dd_sample_for(PWCollector_sample_t *sample, cpu_tsc_vec_t& tsc_vec, pid_t tid)
{
	c_sample_t *cs;
	int cpu;

	cpu = sample->cpuidx;
	if(sample->sample_type != C_STATE){
		return;
	}
	cs = &sample->c_sample;
	if(cs->tid == tid)
		tsc_vec.push_back(cpu_tsc_pair_t(cpu,cs->c_data));
	return;
};

/*
 * Timing related stuff.
 */
#include <sys/time.h>
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
 * A function to read samples from the
 * binary file.
 */
void do_read_dd(cpu_tsc_vec_t& tsc_vec, pid_t tid)
{
	PWCollector_sample_t *samples = new PWCollector_sample_t[NUM_MSGS_TO_READ];
	int i=0, num=0;

	do{
		/*
		 * Read a block of 'NUM_MSGS_TO_READ' samples. The return value
		 * indicates the NUMBER of samples read.
		 */
		i = fread(samples, sizeof(PWCollector_sample_t), NUM_MSGS_TO_READ, dev_fp);
		for(int j=0; j<i; ++j){
			get_dd_sample_for(samples+j, tsc_vec, tid);
		}
	}while(i > 0);

	/*
	 * Sort the samples.
	 */
	std::sort(tsc_vec.begin(), tsc_vec.end(), cpu_tsc_pair_less);

	delete []samples;
};

struct bracketer{
	trace_t *trace;
	bracketer(trace_t *t):trace(t){};
	void operator()(cpu_tsc_pair_t& p){
		int cpu=p.first;
		uint64_t tsc=p.second;
		tsc_pair_t *curr = trace->head_tsc;
		while(curr){
			if(curr->begin < tsc && curr->end > tsc){
				return;
			}
			curr = curr->next;
		}
		fprintf(stderr, "Tsc=%llu NOT FOUND!\n", tsc);
	};
};


void extract_per_cpu_tscs(trace_t *trace)
{
	tsc_pair_t *curr = NULL, *head_tsc = trace->head_tsc;
	for_each_tsc_pair(curr, head_tsc){
		per_cpu_tscs[curr->cpu].push_back(per_cpu_tsc_t(curr->begin, curr->end, trace));
	}
};

bool per_cpu_less_than_func(const per_cpu_tsc_t& t1, const per_cpu_tsc_t& t2){
	return t1.end < t2.end;
};

bool per_cpu_bracket_func(const per_cpu_tsc_t& t1, const uint64_t& s1)
{
	return s1 >= t1.begin && s1 <= t1.end;
};

static int num_iters = 0;

bool binary_search(per_cpu_vec_t vec, int from, int to, uint64_t& what)
{
	if(from > to)
		return false;

	++num_iters;

	int mid = (to+from)/2;
	per_cpu_tsc_t mid_val = vec[mid];
	if(per_cpu_bracket_func(mid_val, what))
		return true;
	if(what > mid_val.end)
		return binary_search(vec, mid+1, to, what);
	return binary_search(vec, from, mid-1, what);
};

bool linear_search(per_cpu_vec_t vec, int& from, const int to, uint64_t& what)
{
	++num_iters;

	for(; from<=to && !per_cpu_bracket_func(vec[from], what); ++from);

	return (from <= to);
};

#define DO_BINARY_SEARCH 0

void per_cpu_bracketer(const cpu_tsc_pair_t& p)
{
	int cpu = p.first;
	uint64_t tsc = p.second;
	int size = per_cpu_tscs[cpu].size();
#if DO_BINARY_SEARCH
	{
		if(binary_search(per_cpu_tscs[cpu], 0, size-1, tsc)){
			fprintf(stderr, "[%d]: Found!\n", cpu);
		}else{
			fprintf(stderr, "[%d]: NOT FOUND!\n", cpu);
		}
	}
#else // DO_BINARY_SEARCH
	{
		int old_from = per_cpu_indices[cpu];
		if(linear_search(per_cpu_tscs[cpu], per_cpu_indices[cpu], size-1, tsc)){
			fprintf(stderr, "[%d]: Found! from=%d\n", cpu, per_cpu_indices[cpu]);
		}else{
			fprintf(stderr, "[%d]: NOT FOUND! Old from = %d\n", cpu, old_from);
			per_cpu_indices[cpu] = old_from;
		}
		// per_cpu_indices[cpu] = 0;
	}
#endif
};

void sort_per_cpu_tscs(void)
{
	for(int i=0; i<max_num_cpus; ++i){
		per_cpu_vec_t *vec = per_cpu_tscs + i;
		std::sort(vec->begin(), vec->end(), per_cpu_less_than_func);
	}
};

void dump_per_cpu_tsc_entry(per_cpu_tsc_t& t1)
{
	fprintf(stderr, "\t[%llu,%llu] --> %p\n", t1.begin, t1.end, t1.trace);
};

void dump_per_cpu_tscs(void)
{
	for(int i=0; i<max_num_cpus; ++i){
		per_cpu_vec_t *vec = per_cpu_tscs + i;
		if(vec->empty())
			continue;
		fprintf(stderr, "CPU = %d\n", i);
		std::for_each(vec->begin(), vec->end(), dump_per_cpu_tsc_entry);
	}
};

static char *output_dir_name = NULL;

static void find_output_dirname(void)
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
	output_dir_name = strdup(line);
};

/*
 * Where should we read the driver
 * results?
 */
static char *driver_input_file_name = NULL;

/*
 * Where should we read the PRELOAD lib
 * results?
 */
static fp_vec_t lib_input_fps;

/*
 * Get O/P dir info -- read the file "output_dir.txt"
 * (which is auto set in the 'runExecs' script file
 * based on user-supplied information).
 */
static void set_input_driver_filename(void)
{
	if(!output_dir_name)
		find_output_dirname();

	std::string input_dir_str(output_dir_name);
	input_dir_str.append("/driver_output.txt");
	driver_input_file_name = strdup(input_dir_str.c_str());
	if(true)
		fprintf(stderr, "PARSER: DRIVER INPUT FILE NAME = %s\n", driver_input_file_name);
};


void find_and_open_lib_files(void)
{
	if(!output_dir_name)
		find_output_dirname();

	char command[1024];
	sprintf(command, "ls %s/lib_output_*.txt", output_dir_name);
	FILE *fp = popen(command, "r");
	char *line = NULL;
	size_t len = 0;
	ssize_t read = 0;

	while( (read = getline(&line, &len, fp)) != -1){
		line[read-1] = '\0';
		fprintf(stderr, "%s\n", line);
		lib_input_fps.push_back(fopen(line, "r"));
		// vec.push_back(NULL);
	}
	if(line)
		free(line);

	fclose(fp);

	if(false){
		exit(1);
	}
};

void init()
{
	/*
	 * Get # of processors in system.
	 */
	max_num_cpus = sysconf(_SC_NPROCESSORS_CONF);

	per_cpu_tscs = new per_cpu_vec_t[max_num_cpus];

	per_cpu_indices = new int[max_num_cpus];
	memset(per_cpu_indices, 0, sizeof(int) * max_num_cpus);

	for(int i=0; i<max_num_cpus; ++i)
		fprintf(stderr, "%d\n", per_cpu_indices[i]);

	set_input_driver_filename();

	if(!(dev_fp = fopen(driver_input_file_name, "rb"))){
		perror("fopen error for driver file");
		// exit(-1);
	}

	if(false){
		lib_fp = fopen("lib_output_1653.txt", "rb");
	}else{

		find_and_open_lib_files();

		if(true){
			fp_vec_t::iterator iter;
			for(iter = lib_input_fps.begin(); iter != lib_input_fps.end(); ++iter){
				fprintf(stderr, "FP = %p\n", *iter);
			}
		}
	}

};

void destroy()
{
	delete []per_cpu_tscs;

	delete []per_cpu_indices;

	fclose(dev_fp);

	fp_vec_t::iterator iter;
	for(iter = lib_input_fps.begin(); iter != lib_input_fps.end(); ++iter){
		fprintf(stderr, "Closing %p\n", *iter);
		if(*iter)
			fclose(*iter);
	}
};

int main(int argc, char *argv[])
{
	trace_vec_t trace_vec;


	init();
	{
		if(false){
			Tracer::instance()->deserialize_traces(lib_fp, &trace_vec);
			exit(1);
		}
		for(fp_iter_t iter = lib_input_fps.begin(); iter != lib_input_fps.end(); ++iter)
			Tracer::instance()->deserialize_traces((*iter), &trace_vec);
		// Tracer::instance()->deserialize_traces(lib_fp, &trace_vec);
		// Tracer::instance()->deserialize_traces(lib_fp);

		for_each(trace_vec.begin(), trace_vec.end(), trace_dumper());
		for_each(trace_vec.begin(), trace_vec.end(), extract_per_cpu_tscs);
		// fprintf(stderr, "BEFORE...\n");
		// dump_per_cpu_tscs();
		sort_per_cpu_tscs();
		// fprintf(stderr, "AFTER...\n");
		// dump_per_cpu_tscs();

		for(int i=0; i<max_num_cpus; ++i)
			fprintf(stderr, "[%d]: %d\n", i, per_cpu_tscs[i].size());


		trace_t *trace = trace_vec[0];
		pid_t tid = trace->tid;

		// std::vector<uint64_t> tsc_vec;
		cpu_tsc_vec_t tsc_vec;

		do_read_dd(tsc_vec, tid);


		fprintf(stderr, "DD size = %d, # tsc pairs = %d\n", tsc_vec.size(), trace->num_tsc_pairs);

		double elapsedTime = 0.0;
		{
			timer t(&elapsedTime);
			// for_each(tsc_vec.begin(), tsc_vec.end(), bracketer(trace));
			for_each(tsc_vec.begin(), tsc_vec.end(), per_cpu_bracketer);
		}
		fprintf(stderr, "# iters = %d\n", num_iters);
		fprintf(stderr, "Time required to check each element = %f msecs / element\n", elapsedTime / 1e3 / tsc_vec.size());
	}
	destroy();
};
