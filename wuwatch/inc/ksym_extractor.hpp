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

#ifndef _KSYM_EXTRACTOR_HPP_
#define _KSYM_EXTRACTOR_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory> // for "auto_ptr"

namespace wuwatch
{

	typedef unsigned long long u64;
    // forward declaration -- defined in 'ksym_extractor.cpp'
    struct k_symbol;

    /*
     * Helper class to extract kernel symbolic
     * information.
     */
    class KernelSymbolExtractor{
	typedef std::vector<std::string> str_vec_t;
	typedef std::vector<k_symbol> sym_vec_t;
	typedef std::map<unsigned long, const k_symbol *> sym_map_t;

	friend class std::auto_ptr<KernelSymbolExtractor>;

    private:
	/*
	 * Data structures to hold symbolic
	 * mapping information.
	 */
	sym_vec_t m_sym_vec;
	sym_map_t m_sym_map;

	/*
	 * Static instance vars.
	 */
	static KernelSymbolExtractor *s_instance;
	static std::auto_ptr<KernelSymbolExtractor> s_auto_ptr;

    private:
	KernelSymbolExtractor();
	~KernelSymbolExtractor();
	/*
	 * INTERNAL function: parse /proc/kallsyms file
	 */
	void parse_kallsyms_file_i();
	/*
	 * INTERNAL function: get backtrace for each return addr in "addrs"
	 */
	// void get_backtrace_symbols_i(const unsigned long *addrs, int num_addrs, str_vec_t& out_vec);
	void get_backtrace_symbols_i(const u64 *addrs, int num_addrs, str_vec_t& out_vec);
	/*
	 * INTERNAL function: cached linear search for "addr" in "sym_vec"
	 */
	const k_symbol *linear_search_i(const sym_vec_t& sym_vec, int num_syms, unsigned long addr, int& last_index);

    public:
	/*
	 * PUBLIC API: dump all symbolic info
	 */
	static void dump_all_symbols();

	/*
	 * PUBLIC API: get (kernel-space) backtrace info for each return address in "addrs"
	 */
	// static void get_backtrace_symbols(const unsigned long *addrs, int num_addrs, str_vec_t& out_vec);
	static void get_backtrace_symbols(const u64 *addrs, int num_addrs, str_vec_t& out_vec);
    };
}; // wuwatch

#endif // _KSYM_EXTRACTOR_HPP_
