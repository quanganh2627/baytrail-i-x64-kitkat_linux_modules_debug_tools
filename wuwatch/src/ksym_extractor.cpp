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
 * Kernel symbol extraction functionality.
 * *****************************************
 */

#include <algorithm>

#include <stdio.h>
#include <string.h>
#include <vector>
#include <deque>
#include <iterator>
#include <iostream>
#include <algorithm>

#include "pw_utils.hpp"
#include "ksym_extractor.hpp"

#define SYMS_FILE "/proc/kallsyms"

/*
 * Do we add a '\n' to every
 * kernel backtrace symbol?
 */
#define ADD_NEWLINE_TO_K_BACKTRACE 0

namespace wuwatch
{
    /*
     * Static "KernelSymbolExtractor" variables.
     */
    KernelSymbolExtractor *KernelSymbolExtractor::s_instance = NULL;
    std::auto_ptr<KernelSymbolExtractor> KernelSymbolExtractor::s_auto_ptr;

    /*
     * Helper (INTERNAL) struct -- defined in
     * this file to avoid global exposure.
     * Note that we could have declared
     * 'k_symbol' as an INNER class instead (within
     * the KernelSymbolExtractor class declaration in the
     * header file). I prefer
     * this mechanism, though -- it leaves 'k_symbol' totally
     * opaque to the external world.
     */
    struct k_symbol{
	unsigned long m_addr;
	std::string m_sym_name, m_mod_name;

	k_symbol(unsigned long addr, const std::string& name):m_addr(addr), m_sym_name(name){};
	k_symbol(unsigned long addr, const std::string& name, const std::string& mod_name):m_addr(addr), m_sym_name(name), m_mod_name(mod_name){};
    };

    /*
     * Helper (INTERNAL) function -- dump a "k_symbol" instance
     * to an ostream. Useful for debugging.
     * @ out: the output stream.
     * @ks: the "k_symbol" instance to dump.
     *
     * @returns: an output stream.
     */
    std::ostream& operator<<(std::ostream& out, const k_symbol& ks)
    {
	char addr_str[100];
	memset(addr_str, 0, sizeof(char) * 100);
	sprintf(addr_str, "0x%lx", ks.m_addr);
	out << addr_str << "\t" << ks.m_sym_name << "\t" << ks.m_mod_name;
	return out;
    };

    /*
     * Helper (INTERNAL) function -- compare two "k_symbol"
     * instances. REQUIRED for 'std::less(...)' and 'std::sort(...)'
     * @k1, k2: the two "k_symbol" instances to compare.
     *
     * @returns: TRUE ==> k1.m_addr < k2.m_addr
     *           FALSE ==> else.
     */
    bool operator<(const k_symbol& k1, const k_symbol& k2)
    {
	return k1.m_addr < k2.m_addr;
    };

    /*
     * The KernelSymbolExtractor class.
     */
    KernelSymbolExtractor::KernelSymbolExtractor(){};
    KernelSymbolExtractor::~KernelSymbolExtractor(){};

    /*
     * Read the 'kallsyms' file to extract kernel symbolic
     * information. Uses 'LineReader' and 'StringTokenizer'
     * utility classes.
     */
    void KernelSymbolExtractor::parse_kallsyms_file_i(){
	str_vec_t vec;
	LineReader::get_all_lines(SYMS_FILE, vec);
	for(str_vec_t::iterator iter = vec.begin(); iter != vec.end(); ++iter){
	    std::string line = *iter;
	    Tokenizer tok(line);
	    str_vec_t toks = tok.get_all_tokens(" \t");
	    // FIRST token is the addr (in HEX)
	    unsigned long addr = strtoul(toks[0].c_str(), NULL, 16);
	    // SECOND token is DON'T CARE (for now)
	    // THIRD token is the symbol name
	    std::string sym_name = toks[2];
	    // (Optional) FOURTH token is the module name
	    if(toks.size() > 3){
		// std::cerr << "Fourth token is " << toks[3] << std::endl;
		std::string mod_name = toks[3];
		m_sym_vec.push_back(k_symbol(addr, sym_name, mod_name));
	    }else{
		m_sym_vec.push_back(k_symbol(addr, sym_name));
	    }
	}
	/*
	 * Finally, sort all samples (by 'm_addr').
	 */
	std::sort(m_sym_vec.begin(), m_sym_vec.end(), std::less<k_symbol>());
    };

    /*
     * Simple linear search algorithm, with (primitive) index
     * caching mechanism. REQUIRES SORTED INPUT!
     *
     * @sym_vec: the (SORTED) vector of kernel-space symbols.
     * @num_syms: the size of 'sym_vec'. Equivalent to 'sym_vec.size()'
     * @addr: the (kernel-space) return address to search for.
     * @last_index: the index from which to start searching 'sym_vec'.
     *
     * @returns: the symbol entry corresponding to 'addr', if found. NULL else.
     * 
     * @requires: REQUIRES that 'sym_vec' be sorted in ASCENDING order!!!
     */
    const k_symbol *KernelSymbolExtractor::linear_search_i(const sym_vec_t& sym_vec, int num_syms, unsigned long addr, int& last_index){
	if(addr < sym_vec[0].m_addr || addr > sym_vec[num_syms-1].m_addr){
	    return NULL;
	}

	/*
	 * Basic algo: find the FIRST symbol whose return address
	 * is LESS than the input address. If a previous
	 * index has been specified then iterate DOWN from that
	 * index, else iterate DOWN from the END of the list.
	 */
	if(last_index < 0){
	    last_index = num_syms;
	}

	while(--last_index >= 0 && sym_vec[last_index].m_addr > addr){};

	if(last_index >= 0){
	    return &sym_vec[last_index];
	}

	return NULL;
    };


    /*
     * INTERNAL function to get backtrace symbolic information.
     *
     * @addrs: the list of backtrace return addresses.
     * @num_addrs: the number of return addresses in the 'addrs' array.
     * @out_vec: (reference to) the output vector of (std::string) symbols.
     */
    // void KernelSymbolExtractor::get_backtrace_symbols_i(const unsigned long *addrs, int num_addrs, str_vec_t& out_vec){
    void KernelSymbolExtractor::get_backtrace_symbols_i(const u64 *addrs, int num_addrs, str_vec_t& out_vec){
	if(!addrs || num_addrs <= 0){
	    return;
	}

	/*
	 * We need a copy of 'addrs' (because, amongst other
	 * reasons, we want to sort the return addresses) -- copy
	 * the 'addrs' array into a vector here.
	 */
	std::vector<unsigned long> vec(num_addrs);
	std::copy(addrs, addrs+num_addrs, vec.begin());
	int num_syms = m_sym_vec.size();

	/*
	 * Sort the list of return addresses.
	 */
	std::sort(vec.begin(), vec.end(), std::less<unsigned long>());
	/*
	 * OK, now 'match' the addresses (by bracketing them
	 * within an appropriate pair of symbols). To do so, we
	 * use a simple linear search with a caching mechanism.
	 * Because of the way the search is performed, we iterate
	 * from the LAST address (the LARGEST one) to the FIRST one
	 * (the SMALLEST).
	 * Note that we also cache all addr <---> symbolic info 
	 * mappings.
	 */
	int last_index = -1; // used to store the index of the last symbolic match.
	for(std::vector<unsigned long>::reverse_iterator riter = vec.rbegin(); riter != vec.rend(); ++riter){
	    unsigned long addr = *riter;
	    if(!m_sym_map[addr]){
		m_sym_map[addr] = linear_search_i(m_sym_vec, num_syms, addr, last_index);
	    }
	}
	/*
	 * Return addresses have been matched with an appropriate
	 * symbol. We now convert that mapping into an actual
	 * string. This string includes the following:
	 * (a) the function name
	 * (b) an offset within the function
	 * (c) (OPTIONAL) the device driver (kernel module) name, if any.
	 *
	 * UPDATE: added '\n' at end of each string.
	 */
	for(int i=0; i<num_addrs; ++i){
	    unsigned long addr = addrs[i];
	    const k_symbol *sym = m_sym_map[addr];
	    unsigned offset = (sym) ? addr - sym->m_addr : 0;
	    const char *sym_name = (sym) ? sym->m_sym_name.c_str() : NULL;
	    const char *mod_name = (sym) ? sym->m_mod_name.c_str() : NULL;
	    std::string ret_str;
	    char tmp_str[1024];
	    memset(tmp_str, 0, sizeof(char) * 1024);
	    {
#if ADD_NEWLINE_TO_K_BACKTRACE
		sprintf(tmp_str, "%s+0x%x %s [<%lx>]\n", sym_name, offset, mod_name, addr);
#else
		sprintf(tmp_str, "%s+0x%x %s [<%lx>]", sym_name, offset, mod_name, addr);
#endif // ADD_NEWLINE_TO_K_BACKTRACE
	    }
	    out_vec.push_back(std::string(tmp_str));
	}
    };

    /*
     * PUBLIC API: retrieves (kernel-space) symbolic information for
     * a list of return addresses.
     *
     * @addrs: the list of return addresses
     * @num_addrs: the number of return addresses in the 'addrs' array.
     * @out_vec: (reference to) the output vector of (std::string) symbols.
     */
    void KernelSymbolExtractor::get_backtrace_symbols(const u64 *addrs, int num_addrs, str_vec_t& out_vec){
	if(!s_instance){
	    s_instance = new KernelSymbolExtractor();
	    s_instance->parse_kallsyms_file_i();
	    s_auto_ptr = std::auto_ptr<KernelSymbolExtractor>(s_instance);
	}

	s_instance->get_backtrace_symbols_i(addrs, num_addrs, out_vec);
    };

    /*
     * PUBLIC API: dump all symbolic information.
     * Used for taking snapshots of the "/proc/kallsyms" file.
     */
    void KernelSymbolExtractor::dump_all_symbols(){
	if(!s_instance){
	    s_instance = new KernelSymbolExtractor();
	    s_instance->parse_kallsyms_file_i();
	    s_auto_ptr = std::auto_ptr<KernelSymbolExtractor>(s_instance);
	}
    };
}; // wuwatch
