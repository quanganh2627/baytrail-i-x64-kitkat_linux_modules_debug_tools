/* **********************************************************************
 * Copyright (c) 2011 Intel Corporation.  All rights reserved.
 * Other names and brands may be claimed as the property of others.
 * Internal Use Only - Do Not Distribute
 * **********************************************************************
 */

/* ***************************************************************************
 * POWER INTERFACE.
 * This is an INTERNAL include file. It declares structures used by the
 * PWParser, which is the parser that parses (user-supplied) architecture
 * config files.
 *
 * AUTHOR: Gautam Upadhyaya <gautam.upadhyaya@intel.com>
 * ***************************************************************************
 */

#ifndef _PWR_PARSER_HPP_
#define _PWR_PARSER_HPP_ 1

#include <vector>
#include <string>

typedef struct PWArch_rec PWArch_rec_t;

typedef std::vector<PWArch_rec_t *> arch_vec_t;

class PWLexer;

enum {
    FAMILY=0,
    MODEL,
    STEPPING
};

typedef enum {
    PW_MSR_PACKAGE,
    PW_MSR_CORE,
    PW_MSR_THREAD
} msr_type_t;

/*
 * Helper structure to store c-state arch information.
 */
struct CRec {
    CRec() : num(-1), m_type(PW_MSR_PACKAGE), msr_addr(0), tres(0) {};
    int num; /* C-state number e.g. c0 */
    msr_type_t m_type; /* The MSR type */
    int msr_addr; /* The MSR address */
    int tres; /* The C-state target residency */
};

/*
 * Helper struct to store Family.Model.Stepping
 * values.
 */
struct fms_t {
    fms_t();
    fms_t(int f, int m, int s);
    int m_family; /* The family */
    int m_model; /* The model */
    int m_stepping; /* The stepping */
};

/*
 * This structure encapsulates all of the (architecture-specific)
 * information supplied by the user.
 */
struct PWArch_rec {
    std::string m_name; /* OPTIONAL: architecture name (e.g. "NHM") */
    // int m_family, m_model, m_stepping; /* Family, Model, Stepping values */
    /*
     * Multiple CPUs can share the same MSRs. 
     * This vector encodes Family, Model, Stepping 
     * values for all such CPUs.
     */
    std::vector <fms_t> m_fms_values;
    int m_bus_freq; /* Bus Frequency */
    pw_u32_t m_cx_clock_rate; /* Rate at which the Cx MSRs count, in KHz. */
    pw_u16_t m_perf_bits_low, m_perf_bits_high; /* The bit-range within the IA32_PERF_STATUS MSR */
    /*
     * The list of c-states, one for each c-state
     * specified in the arch config file.
     */
    // CRec *m_c_states;
    std::vector <CRec> m_c_states;
    /*
     * Internal field used for debugging -- stores the
     * "family.model.stepping" value in convenient string
     * format.
     */
    std::string m_mod_str;

    PWArch_rec();
    PWArch_rec(const PWArch_rec&);
    ~PWArch_rec();

    const std::string& get_name() const;
    void set_name(const std::string&);

    const std::vector<fms_t>& get_fms_values() const;
    void add_fms_value(const fms_t&);

    const std::string& get_mod_str() const;
    void set_mod_str(const std::string&);

    // CRec *get_c_states() const;
    std::vector<CRec>& get_c_states();

    int get_bus_freq() const;
    void set_bus_freq(int);

    int get_cx_clock_rate() const;
    void set_cx_clock_rate(int);

    void get_perf_bits(int&, int&) const;
    void set_perf_bits(int, int);

    void dump();
};

extern std::ostream& operator<<(std::ostream& str, const PWArch_rec& rec);

/*
 * A parser class to parse the (user-supplied) arch
 * config files. Uses PWLexer to tokenize the file.
 * The PUBLIC API consists of a single function which
 * accepts a 'required' {family.model.stepping} value, 
 * searches for entries in the config file which 
 * correspond to the 'required' architecture, and returns
 * a list of "PWArch_rec" structs encoding config information
 * for that architecture.
 */
class PWParser {
private:
    PWParser(const std::string& config_file_path);
    ~PWParser();

    int m_config_fd;
    PWArch_rec_t *m_curr_rec;
    PWLexer *m_lexer;

    bool rec_i();
    bool arch_i();
    bool parse_model_i();
    bool model_i();
    bool bus_freq_i();
    bool cx_clock_rate_sub_i();
    bool cx_clock_rate_i();
    bool perf_i();
    bool c_state_i();
    bool c_states_i();

    PWArch_rec_t *program_i(int fam, int mod, int step);

    bool is_number_i(const char *);
    int extract_number_i(const char *);
    int extract_non_negative_dec_integer_i();

    bool expect_i(const char *);

    int calc_input_file_size_i();

    void destroy_i();

public:
    /*
     * This is the PUBLIC API.
     */
    static PWArch_rec_t *get_c_states_info(const std::string& config_file_path, int fam, int mod, int step=-1);

};

#endif // _PWR_PARSER_HPP_
