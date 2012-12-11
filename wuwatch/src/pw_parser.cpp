/* **********************************************************************
 * Copyright (c) 2011 Intel Corporation.  All rights reserved.
 * Other names and brands may be claimed as the property of others.
 * Internal Use Only - Do Not Distribute
 * **********************************************************************
 */


/* ***************************************************************************
 * POWER INTERFACE.
 * This file contains a parser to parse a (user-supplied) arch config
 * file.
 * Standard recursive-descent like parser. Parser uses the following
 * shortened (EBNF-like) grammar:
 * ---------------------------------------------------------------------
 * NOTE: comments ARE allowed, but are NOT mentioned in the grammar.
 * ---------------------------------------------------------------------
 *
 * prog := (rec)+
 * rec := arch model (bus)? (c_clock) ? perf c_states
 * arch := "ARCH" Name
 * model := "MODEL" model_sub
 * model_sub := Num DOT Num (model)?
 * bus := "BUS_FREQ" Num
 * c_clock := "CX_CLOCK_RATE" c_clock_sub
 * c_clock_sub := "TSC" | Num
 * perf := "PERF_BITS" Num : Num
 * c_states := (c_state)+
 * c_state := c_tok c_type Num Num
 * c_tok := 'c'Num | 'C'Num
 * c_type := 'Package' | 'Core' | 'Thread'
 *
 * AUTHOR: Gautam Upadhyaya <gautam.upadhyaya@intel.com>
 * ***************************************************************************
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h> // for "strtoul"
#include <string.h> // for "memcpy"
#include <assert.h>

#include <string>
#include <iostream>
#include <iterator>

#include "pw_defines.h"
#include "pw_structs.h"
#include "pw_parser.h"

/*
#if IS_INTEL_INTERNAL
    #define PW_ARCH_FILE_NAME "intel_private_arch_msr_config.txt"
#else
    #define PW_ARCH_FILE_NAME "arch_msr_config.txt"
#endif // IS_INTEL_INTERNAL
*/


#define EOL '\0'
#define SPACE ' '
#define TAB '\t'
#define NEWLINE '\n'
#define HASH '#'

#define IS_WHITESPACE(c) ( ((c) >= 0x0 && (c) <= 0x20) || (c) == HASH )
#define IS_VALID_CHAR(c) !(IS_WHITESPACE(c))

#define PW_DEFAULT_STEPPING_VALUE 0x0

/*
 * (Helper) Lexer class to tokenize the
 * (user-supplied) arch config file.
 */
class PWLexer {
private:
    std::string m_buffer;
    int m_curr_index, m_curr_len;
    std::string m_curr_tok;

    void strip_whitespace_i();
    void get_next_token_i();

public:
    PWLexer(const std::string&, const int&);
    ~PWLexer();

    /*
     * These functions constitute the Lexer API.
     * They are called by the Parser.
     */
    void advance();
    const std::string& get_curr_token() const;
    bool is_EOF() const;
    void lookahead(int, std::vector<std::string>&);
    const std::string peek();
};

fms_t::fms_t() : m_family(0), m_model(0), m_stepping(PW_DEFAULT_STEPPING_VALUE) {};
fms_t::fms_t(int f, int m, int s) : m_family(f), m_model(m), m_stepping(s) {};

PWArch_rec::PWArch_rec():m_name(""), m_fms_values(), m_bus_freq(0), m_cx_clock_rate(0), m_perf_bits_low(0), m_perf_bits_high(0), m_mod_str("")
{
    // m_c_states = new CRec[MAX_MSR_ADDRESSES];
};

PWArch_rec::PWArch_rec(const PWArch_rec& rec):m_name(rec.m_name), m_fms_values(rec.m_fms_values), m_bus_freq(rec.m_bus_freq), m_cx_clock_rate(rec.m_cx_clock_rate),
					      m_perf_bits_low(rec.m_perf_bits_low), m_perf_bits_high(rec.m_perf_bits_high), m_mod_str(rec.m_mod_str), m_c_states(rec.m_c_states)
{
};

PWArch_rec::~PWArch_rec()
{
    /*
     * We do NOT check for non-NULL values
     * before deletion -- 'delete' handles NULL
     * pointers!
     */
    /*
    if (m_c_states != NULL) {
        delete []m_c_states;
        m_c_states = NULL;
    }
    */
};

const std::string& PWArch_rec::get_name() const
{
    return m_name;
};

void PWArch_rec::set_name(const std::string& name)
{
    m_name = std::string(name);
};


const std::vector<fms_t>& PWArch_rec::get_fms_values() const
{
    return m_fms_values;
};

void PWArch_rec::add_fms_value(const fms_t& fms_value)
{
    m_fms_values.push_back(fms_value);
};

const std::string& PWArch_rec::get_mod_str() const
{
    return m_mod_str;
};

void PWArch_rec::set_mod_str(const std::string& str)
{
    m_mod_str = std::string(str);
};

std::vector<CRec>& PWArch_rec::get_c_states()
{
    return m_c_states;
};

int PWArch_rec::get_bus_freq() const
{
    return m_bus_freq;
};

void PWArch_rec::set_bus_freq(int freq)
{
    m_bus_freq = freq;
};

int PWArch_rec::get_cx_clock_rate() const
{
    return m_cx_clock_rate;
};

void PWArch_rec::set_cx_clock_rate(int rate)
{
    m_cx_clock_rate = rate;
};

void PWArch_rec::get_perf_bits(int& low, int& high) const
{
    low = m_perf_bits_low; high = m_perf_bits_high;
};

void PWArch_rec::set_perf_bits(int low, int high)
{
    m_perf_bits_low = low; m_perf_bits_high = high;
};

std::ostream& operator<<(std::ostream& str, const PWArch_rec& rec)
{
    const char *msr_type_str[] = {"Package", "Core", "Thread"};
    str << rec.m_name << "\n";
    for (int i=0; i<rec.m_fms_values.size(); ++i) {
        str << "\tF.M.S: " << rec.m_fms_values[i].m_family << "." << rec.m_fms_values[i].m_model << "." << rec.m_fms_values[i].m_stepping << "\n";
    }
    str << "\tBus Freq: " << rec.m_bus_freq << "\n";
    str << "\tPERF_BITS: [" << rec.m_perf_bits_high << ":" << rec.m_perf_bits_low << "]\n";
    str << "\n\tC-states:\n";
    for (std::vector<CRec>::const_iterator citer = rec.m_c_states.begin(); citer != rec.m_c_states.end(); ++citer) {
        str << "\tC" << citer->num << ": Type = " << msr_type_str[citer->m_type] << ", addr = " << citer->msr_addr << ", target res = " << citer->tres << "\n";
    }
    /*
    for (int i=0; i<MAX_MSR_ADDRESSES; ++i) {
        const CRec *crec = &rec.m_c_states[i];
        if (crec->msr_addr > 0x0) {
            str << "\tC" << i << ": Type = " << msr_type_str[crec->m_type] << ", addr = " << crec->msr_addr << ", target res = " << crec->tres << "\n";
        }
    }
    */
    return str;
};

/*
 * Use 'fstat' to calculate the size
 * of the (user-supplied) config file.
 */
int PWParser::calc_input_file_size_i()
{
    struct stat stat_buff;

    memset(&stat_buff, 0, sizeof(struct stat));

    if (fstat(m_config_fd, &stat_buff)) {
	db_fprintf(stderr, "fstat error: %s\n", strerror(errno));
	return -PW_ERROR;
    }

    return stat_buff.st_size;
};


/*
 * From current position, advance pointer until
 * we come to the beginning of the next VALID token.
 * Does NOT extract the token.
 */
void PWLexer::strip_whitespace_i()
{
    /*
     * Advance current index to start of next token.
     * Special care must be taken for '#': update to
     * next NEWLINE, if found.
     */
    while (m_curr_index < m_curr_len && IS_WHITESPACE(m_buffer[m_curr_index])) {
	if (m_buffer[m_curr_index] == HASH) {
	    while (m_buffer[++m_curr_index] != NEWLINE && m_curr_index < m_curr_len) {;}
	} else {
	    ++m_curr_index;
	}
    }
};

/*
 * Get the next valid (i.e. non-whitespace)
 * token. Note that this does NOT return
 * the token -- instead, it sets the
 * 'm_curr_tok' (member) variable to point
 * to this token.
 */
void PWLexer::get_next_token_i()
{
    /*
     * Algo: from m_curr_index, extract every valid
     * character until a DELIM
     */
    int prev_index = m_curr_index;

    if (is_EOF()) {
	return;
    }

    while (m_curr_index < m_curr_len && IS_VALID_CHAR(m_buffer[m_curr_index])) ++m_curr_index;

    /*
     * 'm_curr_tok' points to the new token.
     */
    m_curr_tok = m_buffer.substr(prev_index, (m_curr_index - prev_index));

    /*
     * Now advance current index to start of next token
     */
    strip_whitespace_i();
};

/*
 * PUBLIC API: extract the next valid token.
 */
void PWLexer::advance()
{
    get_next_token_i();
};

/*
 * PUBLIC API: return the current token.
 */
const std::string& PWLexer::get_curr_token() const
{
    return m_curr_tok;
};

/*
 * PUBLIC API: are we at EOF?
 */
bool PWLexer::is_EOF() const
{
    return m_curr_index >= m_curr_len;
};


/*
 * PUBLIC API: N-token lookahead
 */
void PWLexer::lookahead(int num, std::vector<std::string>& tokens)
{
    int prev_index = m_curr_index;

    for (int i=0; i<num; ++i) {
	advance();
	/*
	 * We do NOT need to construct an
	 * 'std::string' -- since the
	 * tokens vec is declared to be holding 'std::string',
	 * an implicit construction will take place anyway.
	 * However, we leave it as is in the interests
	 * of code clarity and debugging.
	 */
	tokens.push_back(std::string(m_curr_tok));
    }

    m_curr_index = prev_index;
};

/*
 * PUBLIC API: 'peek' at the next token
 * (euphamism for 1-token lookahead).
 */
const std::string PWLexer::peek()
{
    std::vector<std::string> vec;
    lookahead(1, vec);
    return vec[0];
};

PWLexer::PWLexer(const std::string& buff, const int& len):m_buffer(buff), m_curr_len(len), m_curr_tok(""), m_curr_index(0)
{
    /*
     * Bootstrap the process by stripping any
     * leading white space.
     */
    strip_whitespace_i();
};

PWLexer::~PWLexer()
{
};


PWParser::PWParser(const std::string& config_file_path) : m_lexer(NULL), m_curr_rec(NULL)
{
    /*
     * The 'arch_msr_config.txt' file is present
     * in the product bin directory. Get
     * the absolute path to that directory,
     * and construct arch path.
     */
    m_config_fd = open(config_file_path.c_str(), O_RDONLY);
    if (m_config_fd < 0) {
        fprintf(stderr, "ERROR opening config file \"%s\": %s\n", config_file_path.c_str(), strerror(errno));
	db_perror("open error in parser");
	return;
    }

    ssize_t len = 0;
    int file_size = calc_input_file_size_i();

    if (file_size <= 0) {
	db_fprintf(stderr, "ERROR: <PWR> Parser reports invalid file size!\n");
	return;
    }

    char *buffer = new char[file_size+1];

    if ( (len = read(m_config_fd, buffer, file_size)) < file_size) {
	perror("read error");
	delete []buffer;
	return;
    }
    buffer[len-1] = '\0';

    m_lexer = new PWLexer(std::string(buffer), file_size);

    m_curr_rec = new PWArch_rec_t;

    delete []buffer;
    close(m_config_fd);
};

PWParser::~PWParser()
{
    destroy_i();
};

/*
 * Check the current token -- ensure it's equal
 * to 'token'.
 */
bool PWParser::expect_i(const char *token)
{
    std::string curr_tok = m_lexer->get_curr_token();
    if (strcmp(token, curr_tok.c_str())) {
	db_fprintf(stderr, "ERROR: <PWR> curr token = %s, expected %s\n", curr_tok.c_str(), token);
	db_abort("ERROR: <PWR> curr token = %s, expected %s\n", curr_tok.c_str(), token);
	return false;
    }
    m_lexer->advance();
    return true;
};

/*
 * The ARCH non-terminal.
 */
bool PWParser::arch_i()
{
    /*
     * arch := "ARCH" Name
     */
    if (!expect_i("ARCH")) // should *NEVER* happen!
	return false;

    // m_lexer->advance();
    /*
     * We don't care what the
     * arch name is. For debugging, 
     * just dump the value.
     */
    m_curr_rec->set_name(m_lexer->get_curr_token());
    m_lexer->advance();
    return true;
};

/*
 * INTERNAL function: deallocate resources.
 */
void PWParser::destroy_i()
{
    delete m_lexer;
    delete m_curr_rec;

    m_lexer = NULL; m_curr_rec = NULL;

    return;
};

/*
 * Given a string representation of a (hex or decimal) number,
 * convert it to an integer.
 */
int PWParser::extract_number_i(const char *token)
{
    /*
     * Attempt to find 'base' -- if 'x' or 'X' present
     * then assume HEX else assume DECIMAL.
     */
    unsigned long base = strchr(token, 'x') || strchr(token, 'X') ? 16 : 10;
    unsigned long val = strtoul(token, NULL, base);

    return (int)val;
};

/*
 * Is the input string a (hex or decimal) number?
 */
bool PWParser::is_number_i(const char *token)
{
    if (!token)
	return false;
    int len = strlen(token);
    // Special case: check for "-"
    if (len == 1 && token[0] == '-')
	return false;
    /*
     * We expect HEX numbers to be prefixed
     * by a "0x" or "0X"
     */
    bool is_hex = false;
    const char *start_ptr = token;
    if (*start_ptr == '-') ++start_ptr;
    if (len > 2 && token[0] == '0' && tolower(token[1]) == 'x') {
	is_hex = true;
	start_ptr += 2;
    }
    int (*is_num)(int) = is_hex ? &isxdigit : &isdigit;
    while (*start_ptr) {
	if (!is_num(*start_ptr++)) {
	    return false;
	}
    }

    return true;
};

/*
 * The MODEL_SUB non-terminal.
 * Extracts the model number.
 */
bool PWParser::parse_model_i()
{
    /*
     * Format: Num DOT Num (DOT Num)?
     * i.e. "X.Y" or "X.Y.Z" (the stepping is optional)
     */
    std::string mod_str(m_lexer->get_curr_token());
    std::string toks[3];
    int family, model, stepping;
    int prev_pos = 0, pos = 0, len = mod_str.length();

    if ( (pos = mod_str.find('.', 0)) < 0) {
        db_fprintf(stderr, "ERROR: <PWR> pos=%d\n", pos);
        return false;
    }
    toks[0] = mod_str.substr(0, pos);
    if (!is_number_i(toks[0].c_str())) {
        db_fprintf(stderr, "ERROR: <PWR> Expected a number for model str::substr; got %s\n", toks[0].c_str());
        return false;
    }
    family = extract_number_i(toks[0].c_str());

    prev_pos = ++pos;
    pos = mod_str.find('.', prev_pos);

    if (pos != std::string::npos) {
        toks[1] = mod_str.substr(prev_pos, (pos - prev_pos));
        toks[2] = mod_str.substr(++pos);
    } else {
        pos = prev_pos;
        toks[1] = mod_str.substr(pos);
    }
    if (!is_number_i(toks[1].c_str())) {
	db_fprintf(stderr, "ERROR: <PWR> Expected a number for model str::substr; got %s\n", toks[1].c_str());
	return false;
    }
    model = extract_number_i(toks[1].c_str());

    if (toks[2].size()) {
        if (!is_number_i(toks[2].c_str())) {
            db_fprintf(stderr, "ERROR: <PWR> Expected a number for model str::substr; got %s\n", toks[2].c_str());
            return false;
        }
        stepping = extract_number_i(toks[2].c_str());
    } else {
        stepping = PW_DEFAULT_STEPPING_VALUE;
    }

    db_fprintf(stderr, "mod_str = %s, toks[0] = %s, toks[1] = %s, toks[2] = %s\n", mod_str.c_str(), toks[0].c_str(), toks[1].c_str(), toks[2].c_str());
    /*
     * OK, we've sanity checked the MODEL field.
     * Now set the appropriate fields (after checking to 
     * see if a 'stepping' has been supplied).
     */
    // m_curr_rec->add_fms_value(fms_t(extract_number_i(toks[0].c_str()), extract_number_i(toks[1].c_str())));
    m_curr_rec->add_fms_value(fms_t(family, model, stepping));

    /*
     * Also set the "mod_str"
     */
    m_curr_rec->set_mod_str(mod_str);

    return true;
};

/*
 * The MODEL non-terminal.
 */
bool PWParser::model_i()
{
    /*
     * model := "MODEL" Num DOT Num DOT Num
     */
    if (!expect_i("MODEL"))
	return false;
    /*
     * Extract model number
     */
    parse_model_i();
    m_lexer->advance();
    return true;
};

/*
 * Similar to 'extract_number_i()', but checks/typecasts
 * for negative or non-integral numbers.
 */
int PWParser::extract_non_negative_dec_integer_i()
{

    std::string num_str(m_lexer->get_curr_token());

    db_fprintf(stderr, "Num Str = %s\n", num_str.c_str());
    db_assert(num_str.size(), "ERROR: <PWR> Expected a non-empty BUS FREQUENCY!\n");

    if (num_str.empty())
	return false;

    bool is_double = num_str.find('.') != std::string::npos;

    if (is_double) {
	db_fprintf(stderr, "DEBUG: <PWR> Found DOUBLE value = %s, WILL TRUNCATE TO LOWER INTEGER i.e. floor(%s)!\n", num_str.c_str(), num_str.c_str());
    }

    int retVal = atoi(num_str.c_str());

    db_assert(retVal >= 0, "ERROR: <PWR> Bus Frequency MUST be NON-NEGATIVE!\n");

    if (retVal < 0) {
	retVal = 0;
    }

    m_lexer->advance();

    return retVal;
};

/*
 * The BUS non-terminal.
 */
bool PWParser::bus_freq_i()
{
    /*
     * bus := "BUS_FREQ" Num
     */
    if (!expect_i("BUS_FREQ")) {
	return false;
    }
    /*
     * Extract bus freq
     */
    {
	int freq = extract_non_negative_dec_integer_i();
	db_fprintf(stderr, "DEBUG: <PWR> Bus Freq = %d MHz\n", freq);
	m_curr_rec->set_bus_freq(freq);
    }

    return true;
};

/*
 * The 'c_clock_sub' non-terminal
 */
bool PWParser::cx_clock_rate_sub_i()
{
    /*
     * c_clock_sub := "TSC" | Num
     */
    int mult = 0;
    if (m_lexer->get_curr_token() == "TSC") {
        m_lexer->advance();
    } else {
        mult = extract_non_negative_dec_integer_i();
    }
    db_fprintf(stderr, "DEBUG: <PWR> Clock Multiplier = %d\n", mult);
    m_curr_rec->set_cx_clock_rate(mult);

    return true;
};
/*
 * The 'c_clock' non-terminal.
 */
bool PWParser::cx_clock_rate_i()
{
    /*
     * c_clock := "CX_CLOCK_RATE" Num
     */
    if (!expect_i("CX_CLOCK_RATE"))
	return false;

    return cx_clock_rate_sub_i();
};

/*
 * The perf non-terminal.
 */
bool PWParser::perf_i()
{
    /*
     * perf := "PERF_BITS" Num ':' Num
     */
    if (!expect_i("PERF_BITS"))
        return false;

    std::string token(m_lexer->get_curr_token());
    ssize_t pos = token.find(':', 0);
    if (pos == std::string::npos) {
        fprintf(stderr, "ERROR: malformed PERF non-terminal?!\n");
        return false;
    }
    int high = atoi(token.substr(0, pos).c_str());
    int low = atoi(token.substr(++pos).c_str());
    m_curr_rec->set_perf_bits(low, high);

    // db_fprintf(stderr, "PERF_BITS = [%d:%d]\n", high, low);
    db_fprintf(stderr, "PERF_BITS = [%d:%d]\n", m_curr_rec->m_perf_bits_high, m_curr_rec->m_perf_bits_low);

    m_lexer->advance();
    return true;
};

/*
 * The C_STATE non-terminal.
 */
bool PWParser::c_state_i()
{
    /*
     * c_state := c_tok Number Number
     * c_tok := 'C'Num | 'c'Num
     */
    const char *c_name = m_lexer->get_curr_token().c_str();
    /*
     * First token should be C-state name/number.
     */
    m_curr_rec->get_c_states().push_back(CRec());
    CRec *cs = &m_curr_rec->get_c_states().back();
    /*
     * Extract the C-state number.
     */
    int val = atoi(&c_name[1]);
    assert (val >= 0 && val < MAX_MSR_ADDRESSES);
    // CRec *cs = &m_curr_rec->get_c_states()[val];
    cs->num = val;
    m_lexer->advance();
    /*
     * Second token is MSR type (Package/Core/Thread)
     */
    std::string msr_type = m_lexer->get_curr_token();
    m_lexer->advance();
    db_assert(true, "Type = %s\n", msr_type.c_str());
    msr_type_t c_type = PW_MSR_PACKAGE;
    if (msr_type == "Core") {
        c_type = PW_MSR_CORE;
    } else if (msr_type == "Thread") {
        c_type = PW_MSR_THREAD;
    } else if (msr_type != "Package") {
        db_fprintf(stderr, "ERROR: MSR type MUST be one of: \"Package\", \"Core\", or \"Thread\"!\n");
        return false;
    }
    cs->m_type = c_type;

    /*
     * Third token is MSR address.
     */
    const char *tok_1 = m_lexer->get_curr_token().c_str();
    {
	if (!is_number_i(tok_1)) {
	    db_fprintf(stderr, "ERROR: <PWR> Expected MSR address (or -1) for c-state %s; got %s\n", c_name, tok_1);
	    // destroy_i();
	    return false;
	}
	val = extract_number_i(tok_1);
	if (val < 0) {
	    db_fprintf(stderr, "WARNING: <PWR> Detected MSR address = %d for C%d (NAME=%s, MODEL=%s). Resetting to ZERO!\n", val, cs->num, 
		       m_curr_rec->get_name().c_str(), m_curr_rec->get_mod_str().c_str());
	    val = 0;
	}
    }
    cs->msr_addr = val;
    m_lexer->advance();
    /*
     * Third token is target residency.
     */
    const char *tok_2 = m_lexer->get_curr_token().c_str();
    {
	if (!is_number_i(tok_2)) {
	    db_fprintf(stderr, "ERROR: <PWR>Expected target residency (or -1) for c-state %s; got %s\n", c_name, tok_2);
	    // destroy_i();
	    return false;
	}
	val = extract_number_i(tok_2);
	if (val < 0) {
	    db_fprintf(stderr, "WARNING: <PWR> Detected target residency = %d for C%d (NAME=%s, MODEL=%s).Resetting to ZERO!\n", val, 
		       cs->num, m_curr_rec->get_name().c_str(), m_curr_rec->get_mod_str().c_str());
	    val = 0;
	}
    }
    cs->tres = val;
    m_lexer->advance();

    return true;
};

/*
 * The C_STATES non-terminal.
 */
bool PWParser::c_states_i()
{
    /*
     * c_states := (c_state)+
     */
    /*
     * MUST have at least 1 C-state...
     */
    if (!c_state_i()) {
	return false;
    }
    /*
     * ...followed by zero or more of them.
     */
    while (!m_lexer->is_EOF() && tolower(m_lexer->get_curr_token().c_str()[0]) == 'c') {
	if (!c_state_i()) {
	    return false;
        }
    }

    return true;
};


/*
 * The REC non-terminal.
 */
bool PWParser::rec_i()
{
    /*
     * rec := arch model (bus)? (c_clock)? (perf) c_states
     */

    // if (m_lexer->get_curr_token() == "ARCH")
    if (!arch_i()) {
	return false;
    }

    if (!model_i()) {
	return false;
    }

    /*
     * Peek at next token -- if MODEL
     * then this is another model specifier.
     */
    while (m_lexer->get_curr_token() == "MODEL") {
	if (!model_i()) {
	    return false;
	}
    }

    /*
     * "BUS_FREQ" and "CX_CLOCK_RATE" are 
     * optional -- 'peek' before matching
     * them.
     */
    if (m_lexer->get_curr_token() == "BUS_FREQ") {
	if (!bus_freq_i()) {
	    return false;
	}
    }
    if (m_lexer->get_curr_token() == "CX_CLOCK_RATE") {
	if (!cx_clock_rate_i()) {
	    return false;
	}
    }

    if (!perf_i())
        return false;

    if (!c_states_i())
	return false;

    return true;
};

/*
 * The PROGRAM non-terminal.
 */
PWArch_rec_t *PWParser::program_i(int fam, int mod, int step)
{
    /*
     * Instantiating the m_lexer should have
     * bootstrapped the process by stripping out
     * any leading white space chars. We therefore
     * go straight to reading records.
     *
     * program := (rec)+
     */
    /*
     * Check if 'm_lexer' was instantiated before
     * proceeding.
     */
    if (!m_lexer) {
	return NULL;
    }
    /*
     * 'rec_i()' expects a VALID token -- bootstrap
     * the process by asking lexer to extract
     * the first token.
     */
    m_lexer->advance();
    /*
     * Some entries in the config file may not have a stepping. In this case, we return a match if
     * the family.model numbers match with what the user requires. To check this, we store the
     * LAST entry whose family, model numbers match with user requirements.
     */
    PWArch_rec_t last_fm_match_rec;
    /*
     * Algo: apply the (EBNF-like) grammar
     * specified at the top of this file to the
     * (user-supplied) arch config file. Keep
     * extracting 'records' from this file
     * UNTIL either:
     * (a) EOF or
     * (b) The specified <Family>.<Model>.<Stepping> is found.
     * UPDATE: We check ONLY the <Family> and <Model> and NOT the
     * <Stepping> value.
     */
    while (!m_lexer->is_EOF()) {
	assert(rec_i() == true);
	/*
	 * Did we get the correct REC?
	 * This is defined by equality with the
	 * supplied FAMILY.MODEL.STEPPING value.
	 */
	const std::vector<fms_t>& fms_values = m_curr_rec->get_fms_values();

	for (std::vector<fms_t>::const_iterator iter = fms_values.begin(); iter != fms_values.end(); ++iter) {
            if (iter->m_family == fam && iter->m_model == mod) {
                last_fm_match_rec = *m_curr_rec;
                if (step == PW_DEFAULT_STEPPING_VALUE || step == iter->m_stepping) {
                    db_fprintf(stderr, "DEBUG: <PWR> FOUND REQUIRED ARCH ENTRY!\n");
                    return new PWArch_rec(*m_curr_rec);
                }
            }
        }

	delete m_curr_rec;
	m_curr_rec = new PWArch_rec_t;
    }
    /*
     * OK, no exact match exists; check if we had a partial match (i.e. did we have an entry
     * where the 'family' and 'model' numbers were the same as what the user requires?
     */
    if (last_fm_match_rec.m_fms_values.size() > 0) {
        db_fprintf(stderr, "DEBUG: <PWR> FOUND closest match entry!\n");
        return new PWArch_rec(last_fm_match_rec);
    }
    return NULL;
};

/*
 * PUBLIC API: Given a {Family,Model,Stepping} value ('required'),
 * parse the (user-supplied) arch config file to determine MSR
 * addresses and target residencies.
 */
PWArch_rec_t *PWParser::get_c_states_info(const std::string& config_file_path, int fam, int mod, int step)
{
    if (step <= 0) {
        step = PW_DEFAULT_STEPPING_VALUE;
    }
    db_fprintf(stderr, "REQUIRED FMS = %d.%d.%d\n", fam,mod,step);
    return PWParser(config_file_path).program_i(fam, mod, step);
};


#if 0
#include <iostream>

bool g_do_debugging = true;

void foo(void)
{
    int curr_index = 0;
    int prev_index = curr_index;
    std::string buffer("ARCH NHM");
    std::string curr_tok("");
    int curr_len = buffer.length();

    for (int i=0; curr_index < curr_len; ++i) {

	prev_index = curr_index;

	while (curr_index < curr_len && IS_VALID_CHAR(buffer[curr_index])) ++curr_index;


	curr_tok = buffer.substr(prev_index, (curr_index - prev_index));
	std::cerr << curr_tok << std::endl;

	while (curr_index < curr_len && IS_WHITESPACE(buffer[curr_index])) {
	    if (buffer[curr_index] == HASH) {
		while (buffer[++curr_index] != NEWLINE && curr_index < curr_len);
            } else {
                ++curr_index;
            }
	}
    }
};

int main(int argc, char *argv[])
{
    // foo();
    // PWParser *parser = PWParser::instance();
    {
	const int req[] = {6,44,0x2};
	PWArch_rec_t *rec = PWParser::get_c_states_info(req[0], req[1], req[2]);
        /*
        */
        /*
	const int req[] = {6,42};
	PWArch_rec_t *rec = PWParser::get_c_states_info(req[0], req[1]);
        */
        if (rec) {
            std::cerr << *rec;
        } else {
            fprintf(stderr, "NULL rec!\n");
        }
	delete rec;
    }
    // PWParser::destroy();
};
#endif
