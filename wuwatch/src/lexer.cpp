#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>

#include <readline/readline.h>
#include <readline/history.h>

#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <cctype> // for "isalnum"
#include <sstream> // for "std::stringstream"
#include <fstream>

#include "pw_structs.h"

#define SUCCESS 0
#define ERROR 1


typedef std::string str_t;

class Lexer{
    str_t m_buffer;
    int m_len, m_curr, m_next;

    public:
    Lexer(const str_t& str):m_buffer(str), m_len(str.size()), m_curr(0), m_next(0){};
    int get_curr_token(str_t& token);
    int get_next_token(str_t& token);
    void peek_next_token(str_t& token);
    int get_special_token_string(str_t& buffer, const str_t& end_tok);

    private:
    int strip_leading_whitespace_i();
    bool is_special_token_i();
    int get_next_regular_token_i(str_t& token);
    int get_next_special_token_i(str_t& token);
};

class Parser{
    Lexer *m_lexer;
    str_t m_curr_tok;
    str_t m_output_buffer;
    public:
    Parser(const str_t& str):m_lexer(new Lexer(str)){};
    ~Parser(){
        delete m_lexer;
    };
    public:
    int prog();
    const str_t& get_output() const;
    private:
    /*
     * Top-level rules.
     */
    int loadStat_i();
    int quitStat_i();
    int printStat_i();
    int wuStat_i();
    private:
    /*
     * Lower-level rules.
     */
    int filterStat_i();
    int filterExpr_i();
    int relExpr_i();
    int logicalOp_i();
    int relOp_i();
    int lvalue_i();
    int rvalue_i();
    int wuTrail_i();
    int printTrail_i();
    int printTrailSub_i();
    int any_i();
    int quoted_any_i();
    int LETTER_i(char);
    int NUM_i(char);
    int ID_i(bool is_lvalue);
    int FIELD_NAME_i(bool is_lvalue);
    int FILE_NAME_i();
    private:
    /*
     * Helpers...
     */
    int get_curr_token_i(str_t& token);
    int get_next_token_i(str_t& token);
    void peek_next_token_i(str_t& token);
    int consume_i(const str_t& expected);
    int expect_i(const str_t& expected);
    void output_i();
    void output_i(const str_t& token);
    void advance_i(const str_t& func_name);
    void peek_i();
};

typedef enum{
    FILTER=0,
    MAP,
    PRINT
}code_block_t;

class CodeBlock{
    const code_block_t m_type;
    str_t m_code;
    public:
    CodeBlock(code_block_t type, const str_t& code):m_type(type), m_code(code){};
    const code_block_t get_type() const;
    const str_t& get_code() const;
};

const code_block_t CodeBlock::get_type() const {
    return m_type;
};

const str_t& CodeBlock::get_code() const {
    return m_code;
};

class Execer{
    static Execer *m_execer;
    std::vector<PWCollector_sample_t> m_sample_vec;
    std::list<CodeBlock> m_code_blocks;
    str_t output_code;
    public:
    static Execer *instance();
    static void destroy();
    public:
    void do_load_file(const str_t&);
    void do_filter(const str_t&);
    void do_exec();
    public:
    void add_code_block(const CodeBlock&);
    private:
    Execer();
    ~Execer();
    private:
    void generate_header_code_i(void);
    void generate_filter_code_i(const str_t&);
    void generate_print_code_i(const str_t&);
    void generate_work_and_main_code_i(bool, bool);
    void dump_code_i(void);
    void compile_output_file_i();
    void exec_i();
    void send_samples_i(int);
};

template <typename T> class SingletonResMon{
    T *m_t;
    public:
    SingletonResMon(){
        m_t = T::instance();
    };
    ~SingletonResMon(){
        T::destroy();
    };
};

#define EMPTY_STRING ""

#define WHITESPACE " \t"
#define SPECIAL_TOKS "()|&<=>!+-.,\""
#define TOKEN_DELIM " \t()|&<=>!+-.,\"" // WHITESPACE + SPECIAL_TOKS

#define UNDERSCORE '_'

#define LPAREN "("
#define RPAREN ")"
#define OR  "||"
#define AND "&&"
#define EQ  "=="
#define NE  "!="
#define GT  ">"
#define GE  ">="
#define LT  "<"
#define LE  "<="
#define PLUS  "+"
#define MINUS  "-"
#define PIPE  "|>"
#define DOT  "."
#define COMMA ","
#define QUOTE "\""

int Lexer::strip_leading_whitespace_i(){
    m_curr = m_buffer.find_first_not_of(WHITESPACE, m_next);
    m_next = m_curr;

    if(m_curr == std::string::npos)
        return -ERROR;

    return SUCCESS;
};

int Lexer::get_curr_token(str_t& token){
    if(m_curr == std::string::npos)
        return -ERROR;

    token = m_buffer.substr(m_curr, (m_next - m_curr));

    return SUCCESS;
};

/*
 * Use a non-conventional token delimiting
 * string to decide the next token -- useful
 * when we need to extract ALL characters
 * UNTIL a special char (e.g. QUOTE).
 */
int Lexer::get_special_token_string(str_t& buffer, const str_t& end_tok){
    m_curr = m_buffer.find_first_not_of(end_tok, m_next);
    m_next = m_buffer.find_first_of(end_tok, m_curr);
    if(m_next == std::string::npos)
        return -ERROR;
    buffer = m_buffer.substr(m_curr, (m_next - m_curr));
    return SUCCESS;
};

int Lexer::get_next_token(str_t& token){

    if(strip_leading_whitespace_i())
        return -ERROR;

    return is_special_token_i() ? get_next_special_token_i(token) : get_next_regular_token_i(token);
};

void Lexer::peek_next_token(str_t& token){

    int curr = m_curr, next = m_next;

    if(strip_leading_whitespace_i())
        return;

    if(is_special_token_i())
        get_next_special_token_i(token);
    else
        get_next_regular_token_i(token);

    m_curr = curr; m_next = next;
};

/*
 * ASSUMPTION: first char is
 * NOT WHITESPACE! Call 'strip_leading_whitespace_i()'
 * BEFORE you call this function!
 */
bool Lexer::is_special_token_i(){
    if(false && std::string(SPECIAL_TOKS).find(m_buffer[m_curr]) != std::string::npos){
        fprintf(stderr, "SPECIAL TOK = %c\n", m_buffer[m_curr]);
    }
    return (std::string(SPECIAL_TOKS).find(m_buffer[m_curr]) != std::string::npos);
};

/*
 * ASSUMPTION: 'strip_leading_whitespace_i()'
 * has been called BEFORE calling this
 * function!!!
 */
int Lexer::get_next_regular_token_i(str_t& token){
    m_curr = m_buffer.find_first_not_of(TOKEN_DELIM, m_next);
    m_next = m_buffer.find_first_of(TOKEN_DELIM, m_curr);

    if(m_curr == std::string::npos)
        return -ERROR;

    token = m_buffer.substr(m_curr, (m_next - m_curr));

    return SUCCESS;
};

/*
 * ASSUMPTION: 'strip_leading_whitespace_i()'
 * has been called BEFORE calling this
 * function!!!
 */
int Lexer::get_next_special_token_i(str_t& token){

    /*
     * Special case of the special case (I):
     * PARENS should be a single token i.e. "()"
     * should be TWO tokens -- '(' and ')'.
     */
    if(m_buffer[m_curr] == '(' || m_buffer[m_curr] == ')'){
        ++m_next;
    }
    /*
     * Special case of the special case (II):
     * QUOTE MUST be a SINGLE token itself, regardless
     * of what follows i.e. ", should be two
     * tokens.
     */
    else if(m_buffer[m_curr] == '"'){
        ++m_next;
    }
    else{
        m_curr = m_buffer.find_first_of(SPECIAL_TOKS, m_next);
        m_next = m_buffer.find_first_not_of(SPECIAL_TOKS, m_curr);

        if(m_curr == std::string::npos)
            return -ERROR;
    }

    token = m_buffer.substr(m_curr, (m_next - m_curr));

    return SUCCESS;
};

#define ADVANCE_TOK() advance_i(__FUNCTION__)
#define PEEK_TOK() peek_i()

#define IS_ERROR(f) ( (f) == -ERROR )

int Parser::prog(){

    advance_i(__FUNCTION__);

    if(m_curr_tok == "load"){
        if(IS_ERROR(loadStat_i()))
            return -ERROR;
    }
    else if(m_curr_tok == "quit"){
        if(IS_ERROR(quitStat_i()))
            return -ERROR;
    }
    else if(m_curr_tok == "print"){
        if(IS_ERROR(printStat_i()))
            return -ERROR;
    }
    else if(IS_ERROR(wuStat_i()))
        return -ERROR;
    /*
     * OK, parsing succeeded -- start
     * the code generation + execution.
     */
    Execer::instance()->do_exec();
    return SUCCESS;

    /*
       if(m_curr_tok == "load")
       return loadStat_i();
       else if(m_curr_tok == "quit")
       return quitStat_i();
       else if(m_curr_tok == "print")
       return printStat_i();
       return wuStat_i();
       */
};

const str_t& Parser::get_output() const {
    return m_output_buffer;
};

#define PARSE_ERROR(str) do{						\
    fprintf(stderr, "PARSE ERROR in Function %s: %s", __FUNCTION__, str); \
}while(0)

int Parser::loadStat_i(){
    fprintf(stderr, "LOAD found!\n");
    consume_i("load");
    ADVANCE_TOK();
    if(FILE_NAME_i()){
        PARSE_ERROR("Could NOT extract file name for load statement!\n");
        return -ERROR;
    }
    fprintf(stderr, "LOAD found file name = %s\n", m_output_buffer.c_str());
    Execer::instance()->do_load_file(m_output_buffer);
    return SUCCESS;
};

int Parser::quitStat_i(){
    fprintf(stderr, "QUIT found!\n");
    if(true){
        exit(SUCCESS);
    }
    return -ERROR;
};

int Parser::printTrailSub_i(){
    if(IS_ERROR(FIELD_NAME_i(true))){
        PARSE_ERROR("Could NOT extract a valid field name!\n");
        fprintf(stderr, "TOKEN = %s\n", m_curr_tok.c_str());
        return -ERROR;
    }
    PEEK_TOK();
    while(m_curr_tok.size()){
        ADVANCE_TOK();
        expect_i(COMMA);
        ADVANCE_TOK();
        if(IS_ERROR(FIELD_NAME_i(true))){
            PARSE_ERROR("Could NOT extract a valid field name!\n");
            return -ERROR;
        }
        PEEK_TOK();
    }
    return SUCCESS;
};

int Parser::any_i(){
    if(IS_ERROR(m_lexer->get_special_token_string(m_curr_tok, QUOTE))){
        PARSE_ERROR("Could NOT get special token string for \"any_i()\" rule!\n");
        return -ERROR;
    }
    output_i();
    return SUCCESS;
};

int Parser::quoted_any_i(){
    expect_i(QUOTE);
    do{
        fprintf(stderr, "\tCurr = %s\n", m_curr_tok.c_str());
        ADVANCE_TOK();
        output_i();
    }while(m_curr_tok != QUOTE);
    return SUCCESS;
};

int Parser::printTrail_i(){
    fprintf(stderr, "CURRENT tok = %s\n", m_curr_tok.c_str());
    if(m_curr_tok == QUOTE){
        if(IS_ERROR(quoted_any_i()))
            return -ERROR;
        PEEK_TOK();
        if(!m_curr_tok.size()){
            return SUCCESS;
        }
        ADVANCE_TOK();
        if(IS_ERROR(expect_i(COMMA))){
            fprintf(stderr, "ERROR: expected %s, got %s\n", COMMA, m_curr_tok.c_str());
            return -ERROR;
        }
        ADVANCE_TOK();
    }
    return printTrailSub_i();
};

int Parser::printStat_i(){
    fprintf(stderr, "PRINT found!\n");
    consume_i("print");
    PEEK_TOK();
    if(m_curr_tok.size()){
        ADVANCE_TOK();
        if(IS_ERROR(printTrail_i())){
            return -ERROR;
        }
    }
    Execer::instance()->add_code_block(CodeBlock(PRINT, m_output_buffer));
    fprintf(stderr, "(print)OUTPUT BUFFER = %s\n", m_output_buffer.c_str());
    if(false){
        exit(1);
    }
    return SUCCESS;
};

int Parser::wuStat_i(){
    fprintf(stderr, "WUSTAT found!\n");
    if(m_curr_tok == "filter"){
        if(IS_ERROR(filterStat_i())){
            PARSE_ERROR("filterstat error!\n");
            // fprintf(stderr, "filterstat error!\n");
            return -ERROR;
        }
        fprintf(stderr, "OK, filter parsed SUCCESSFULLY!\n");
        Execer::instance()->add_code_block(CodeBlock(FILTER, m_output_buffer));
        /*
         * Output buffer should be deleted now
         * that a rule has been successfully matched.
         */
        {
            m_output_buffer.clear();
        }
        // Execer::instance()->do_filter(m_output_buffer);
        PEEK_TOK();
        if(m_curr_tok != EMPTY_STRING){
            return wuTrail_i();
        }
        return SUCCESS;
    }
    return -ERROR;
};

int Parser::filterStat_i(){
    if(m_curr_tok == "filter"){
        ADVANCE_TOK();
        if(IS_ERROR(expect_i(LPAREN))){
            fprintf(stderr, "ERROR: expected %s but got %s\n", LPAREN, m_curr_tok.c_str());
            return -ERROR;
        }
        ADVANCE_TOK();
        if(IS_ERROR(filterExpr_i())){
            PARSE_ERROR("filterExpr error!\n");
            return -ERROR;
        }
        ADVANCE_TOK();
        if(IS_ERROR(expect_i(RPAREN))){
            fprintf(stderr, "ERROR: expected %s but got %s\n", RPAREN, m_curr_tok.c_str());
            return -ERROR;
        }
    }
    return SUCCESS;
};

int Parser::filterExpr_i(){
    str_t token;
    if(m_curr_tok[0] == '('){
        output_i();
        ADVANCE_TOK();
        if(IS_ERROR(filterExpr_i())){
            return -ERROR;
        }
        ADVANCE_TOK();
        if(IS_ERROR(expect_i(RPAREN))){
            fprintf(stderr, "ERROR: expected %s but got %s\n", RPAREN, m_curr_tok.c_str());
            return -ERROR;
        }
        return SUCCESS;
    }
    if(IS_ERROR(relExpr_i()))
        return -ERROR;
    PEEK_TOK();
    if(m_curr_tok == OR || m_curr_tok == AND){
        ADVANCE_TOK();
        if(IS_ERROR(logicalOp_i())){
            return -ERROR;
        }
        ADVANCE_TOK();
        return filterExpr_i();
    }
    return SUCCESS;
};

int Parser::relExpr_i(){
    if(IS_ERROR(lvalue_i())){
        fprintf(stderr, "NON lvalue: %s!\n", m_curr_tok.c_str());
        return -ERROR;
    }
    ADVANCE_TOK();
    if(IS_ERROR(relOp_i()))
        return -ERROR;
    ADVANCE_TOK();
    return rvalue_i();
};


int Parser::logicalOp_i(){
    if(expect_i(OR) && expect_i(AND)){
        fprintf(stderr, "ERROR: expected %s or %s but got %s\n", OR, AND, m_curr_tok.c_str());
        return -ERROR;
    }
    return SUCCESS;
};
// EQ | NE | LT | LE | GT | GE
int Parser::relOp_i(){
    if(expect_i(EQ) && expect_i(NE) && expect_i(LT) && expect_i(LE) && expect_i(GT) && expect_i(GE)){
        fprintf(stderr, "ERROR: expected %s, %s, %s, %s, %s or %s but got %s\n", EQ, NE, LT, LE, GT, GE, m_curr_tok.c_str());
        return -ERROR;
    }
    return SUCCESS;
};

int Parser::LETTER_i(char c){
    return ((isalpha(c) || c == UNDERSCORE) ? SUCCESS : -ERROR);
};

int Parser::NUM_i(char c){
    return isdigit(c) ? SUCCESS : -ERROR;
};

int Parser::ID_i(bool is_lvalue){
    int size = m_curr_tok.size();
    if(size == 0 || LETTER_i(m_curr_tok[0]) == -ERROR)
        return -ERROR;
    for(int i=1; i<size; ++i)
        if(LETTER_i(m_curr_tok[i]) == -ERROR && NUM_i(m_curr_tok[i]) == -ERROR)
            return -ERROR;
    // m_output_buffer += m_curr_tok;
    if(is_lvalue){
        output_i("sample.");
    }
    output_i();
    return SUCCESS;
};

int Parser::FIELD_NAME_i(bool is_lvalue){
    if(ID_i(is_lvalue))
        return -ERROR;
    PEEK_TOK();
    while(m_curr_tok == DOT){
        ADVANCE_TOK();
        // m_output_buffer += DOT;
        expect_i(DOT);
        ADVANCE_TOK();
        // if(ID_i(is_lvalue))
        if(ID_i(false))
            return -ERROR;
        PEEK_TOK();
    }
    return SUCCESS;
};

int Parser::FILE_NAME_i(){
    return FIELD_NAME_i(false);
};

int Parser::lvalue_i(){
    return FIELD_NAME_i(true);
    /*
       int size = m_curr_tok.size();
       if(size == 0)
       return -ERROR;
       if(!isalpha(m_curr_tok[0]) || !isalnum(m_curr_tok[size-1]))
       return -ERROR;
       for(int i=1; i<size-1; ++i){
       if(!isalnum(m_curr_tok[i]) && m_curr_tok[i] != '.')
       return -ERROR;
       }
       return SUCCESS;
       */
};

int Parser::rvalue_i(){
    int size = m_curr_tok.size();
    if(size == 0)
        return -ERROR;
    for(int i=0; i<size; ++i){
        // if(!isalnum(m_curr_tok[i]))
        if(LETTER_i(m_curr_tok[i]) == -ERROR && NUM_i(m_curr_tok[i]) == -ERROR)
            return -ERROR;
    }
    // m_output_buffer += m_curr_tok;
    output_i();
    return SUCCESS;
};

int Parser::wuTrail_i(){
    ADVANCE_TOK();
    consume_i(PIPE);
    ADVANCE_TOK();
    if(m_curr_tok == "print")
        return printStat_i();
    return wuStat_i();
    // fprintf(stderr, "ERROR: multiple wuStat NOT supported (yet)!\n");
    // return -ERROR;
};

int Parser::get_curr_token_i(str_t& token){
    if(!m_lexer)
        return -ERROR;
    return m_lexer->get_curr_token(token);
};

int Parser::get_next_token_i(str_t& token){
    if(!m_lexer)
        return -ERROR;
    return m_lexer->get_next_token(token);
};

void Parser::peek_next_token_i(str_t& token){
    if(m_lexer)
        m_lexer->peek_next_token(token);
}

int Parser::expect_i(const str_t& expected){
    if(consume_i(expected)){
        return -ERROR;
    }
    // m_output_buffer += expected;
    output_i(expected);
    return SUCCESS;
};

int Parser::consume_i(const str_t& expected){
    if(m_curr_tok != expected){
        // fprintf(stderr, "ERROR: expected %s, got %s\n", expected.c_str(), m_curr_tok.c_str());
        return -ERROR;
    }
    return SUCCESS;
};

void Parser::output_i(){
    output_i(m_curr_tok);
};

void Parser::output_i(const str_t& token){
    m_output_buffer += token;
};

void Parser::advance_i(const str_t& func_name){
    if(get_next_token_i(m_curr_tok)){
        fprintf(stderr, "ERROR: parse error in function %s\n", func_name.c_str());
        assert(false);
    }
};

void Parser::peek_i(){
    m_curr_tok = "";
    peek_next_token_i(m_curr_tok);
};

Execer *Execer::m_execer = NULL;

Execer::Execer(){};
Execer::~Execer(){};

Execer *Execer::instance(){
    if(!m_execer){
        m_execer = new Execer;
    }
    return m_execer;
};

void Execer::destroy(){
    delete m_execer;
};

/*
 * How many samples to read at one go?
 */
#define NUM_MSGS_TO_READ 1024

void Execer::do_load_file(const str_t& fname){
    fprintf(stderr, "Execer told to load file = %s\n", fname.c_str());

    FILE *fp = fopen(fname.c_str(), "r");
    if(fp == NULL){
        perror("fopen error");
        exit(-1);
    }
    PWCollector_sample_t *samples = new PWCollector_sample_t[NUM_MSGS_TO_READ];
    int i=0, num=0;

    m_sample_vec.clear(); // just in case!

    fprintf(stderr, "COLLECTING...\n");
    do{
        /*
         * Read a block of 'NUM_MSGS_TO_READ' samples. The return value
         * indicates the NUMBER of samples read.
         */
        i = fread(samples, sizeof(PWCollector_sample_t), NUM_MSGS_TO_READ, fp);
        for(int j=0; j<i; ++j){
            m_sample_vec.push_back(samples[j]);
        }
    }while(i > 0);
};

void Execer::exec_i(){
    int p_in[2], p_out[2];
    if(pipe(p_in) || pipe(p_out)){
        perror("pipe error");
        exit(-1);
    }
    pid_t child = fork();
    if(child < 0){
        perror("fork error");
        exit(-1);
    }
    else if(!child){
        // Close child's 'p_in' WRITE end 
        close(p_in[1]);
        // Close child's 'p_out' READ end
        close(p_out[0]);
        // Dup2 child's 'p_in' READ end with 'stdin'
        if(dup2(p_in[0], fileno(stdin)) == -1){
            perror("dup2 error in child");
            exit(-1);
        }
        // Dup2 child's 'p_out' WRITE end with 'stdout'
        if(dup2(p_out[1], fileno(stdout)) == -1){
            perror("dup2 error in child");
            exit(-1);
        }
        // OK, everything setup -- now exec child func
        // execv(...)
        // child_func();
        char *child_args[] = {"./__tmp1", NULL};
        execvp(child_args[0], child_args);
    }
    else{
        // parent
        // Close parent's 'p_in' READ end
        close(p_in[0]);
        // Close parent's 'p_out' WRITE end
        close(p_out[1]);

        send_samples_i(p_in[1]);
        int status = -1;
        if(waitpid(child, &status, 0) == -1){
            perror("waitpid error");
            exit(-1);
        }
    }
};

void Execer::send_samples_i(int out_fd){
    int i=0, size = m_sample_vec.size();

    for(int i=0; i<size; i+= 1024){
        fprintf(stderr, "i = %d\n", i);
        int num = (size-i) > 1024 ? 1024 : (size-i);
        if(write(out_fd, &m_sample_vec[i], (num * sizeof(PWCollector_sample_t))) <= 0){
            perror("fwrite error");
            exit(-1);
        }
        fprintf(stderr, "...done with i = %d, Num = %d\n", i, num);
    }
    fprintf(stderr, "OK, sent all samples\n");
    close(out_fd);
};

void Execer::do_exec(){
    str_t target_filter_code, target_print_code;
    bool is_first = true;
    while(!m_code_blocks.empty()){
        const CodeBlock& block = m_code_blocks.front();
        // const str_t& code = block.get_code();
        str_t code(block.get_code());
        m_code_blocks.pop_front();
        switch(block.get_type()){
            case FILTER:
                if(!is_first)
                    target_filter_code += " && ";
                target_filter_code += code;
                break;
            case PRINT:
                target_print_code = code;
                break;
            default:
                fprintf(stderr, "ERROR: invalid block type = %d\n", block.get_type());
                exit(-1);
        }
        is_first = false;
    }
    // fprintf(stderr, "TARGET CODE = %s\n", target_filter_code.c_str());
    if(target_filter_code.size() == 0 && target_print_code.size() == 0){
        // Nothing to do!
        return;
    }
    {
        generate_header_code_i();
    }

    bool was_custom_filter = false, was_custom_print = false;

    if(target_filter_code.size()){
        was_custom_filter = true;
        generate_filter_code_i(target_filter_code);
    }
    if(target_print_code.size()){
        // We have a custom print string
        was_custom_print = true;
        generate_print_code_i(target_print_code);
    }

    {
        generate_work_and_main_code_i(was_custom_filter, was_custom_print);
    }

    {
        dump_code_i();
    }
    {
        compile_output_file_i();
    }
    {
        exec_i();
    }
    if(false){
        assert(false);
    }
};


void Execer::generate_print_code_i(const str_t& printString){
    bool has_custom_format = false;
    int num_fields = 1, len = printString.size();
    // Check if user passed in a format string
    for(int i=0; i<len && !(has_custom_format = (printString[i] == '"')); ++i);
    if(false){
        fprintf(stderr, "Has custom format = %d\n", has_custom_format);
        exit(1);
    }
    // Count how many fields there are to be printed.
    for(int i=0; i<len; ++i)
        if(printString[i] == ',')
            ++num_fields;
    fprintf(stderr, "There were %d tokens!\n", num_fields);

    output_code += "\nvoid dump_samples(const sample_list_t& output_list){\n";
    output_code += "for(sample_list_t::const_iterator iter = output_list.begin(); iter != output_list.end(); ++iter){\n";
    output_code += "const PWCollector_sample_t& sample = *iter;\n";
    output_code += "fprintf(stderr, ";
    if(!has_custom_format){
        output_code += "\"%d";
        while(--num_fields > 0){
            output_code += "\\t%d";
        }
        output_code += "\\n\", ";
    }
    output_code += printString;
    output_code += ");\n";
    output_code += "}\n";
    // output_code += "fflush(stdout);\n";
    output_code += "};";
    fprintf(stderr, "GENERATED print string = %s\n", output_code.c_str());

    if(false){
        exit(1);
    }
};

void Execer::generate_filter_code_i(const str_t& filterString){
    output_code += "\n";
    output_code += "void filter_struct::operator()(const PWCollector_sample_t& sample){\n";
    output_code += "if(" + filterString + "){\n";
    output_code += "output_list.push_back(sample);\n";
    output_code += "}\n";
    output_code += "};";
};

void Execer::dump_code_i(){
    std::ofstream output_stream("__tmp1.cpp");
    if(!output_stream.is_open()){
        fprintf(stderr, "ERROR: could NOT open output file stream!\n");
        exit(-1);
    }
    output_stream << output_code;
    output_stream.close();
    /*
     * Prepare for subsequent calls
     * to 'exec'
     */
    output_code.clear();
};

void Execer::compile_output_file_i(){
    const char *exec_string = "g++ -g -I../../../include __tmp1.cpp -o __tmp1";
    if(system(exec_string)){
        perror("system error");
        exit(-1);
    }
};

void Execer::do_filter(const str_t& filterString){
    fprintf(stderr, "FILTER STRING = %s\n", filterString.c_str());

    generate_filter_code_i(filterString);
    compile_output_file_i();
    exec_i();
};

void Execer::add_code_block(const CodeBlock& block){
    m_code_blocks.push_back(block);
};

void Execer::generate_work_and_main_code_i(bool use_custom_filter_func, bool use_custom_print_func){
    output_code += "\nvoid do_work(){\n";
    output_code += "PWCollector_sample_t samples[1024];\n";
    output_code += "int i=0;\n";
    output_code += "sample_list_t output_list;\n";
    output_code += "filter_struct fstruct(output_list);\n";
    output_code += "do{\ni = fread(samples, sizeof(PWCollector_sample_t), 1024, stdin);\n";
    if(use_custom_filter_func){
        output_code += "if(i > 0)\nstd::for_each(samples, samples+i, fstruct);\n";
    }else{
        output_code += "for(int j=0; j<i; ++j) output_list.push_back(samples[j]);\n";
    }
    output_code += "}while(i>0);\n";
    output_code += "fprintf(stderr, \"OUTPUT list size = %d\\n\", output_list.size());\n";
    if(use_custom_print_func){
        output_code += "dump_samples(output_list);\n";
    }else{
        output_code += "default_dump_samples(output_list);\n";
    }
    output_code += "};\n";
    output_code += "\n\nint main(int argc, char *argv[]){\ndo_work();\n};\n";
};

const char *header_code = " \
#include <stdio.h> \n\
#include <stdlib.h> \n\
#include <unistd.h> \n\
#include <stdint.h> \n\
#include <list> \n\
#include <algorithm> \n\
#include \"pw_structs.h\" \n\
      typedef std::list<PWCollector_sample_t> sample_list_t; \n\
      struct filter_struct{ \n\
          sample_list_t& output_list; \n\
          filter_struct(sample_list_t& out):output_list(out){}; \n\
          void operator()(const PWCollector_sample_t&); \n\
      };\n\
      void dump_samples(const sample_list_t& output_list); \n\
      void default_dump_samples(const sample_list_t& output_list) \n\
{ \n\
    PWCollector_sample_t *sample = NULL; \n\
    for(sample_list_t::const_iterator iter = output_list.begin(); iter != output_list.end(); ++iter){ \n\
        const PWCollector_sample_t& sample = *iter; \n\
        int cpu = sample.cpuidx; \n\
        switch(sample.sample_type){ \n\
            case C_STATE: \n\
                          fprintf(stdout, \"C-STATE: %d\\n\", cpu); \n\
            break; \n\
            case P_STATE: \n\
                          fprintf(stderr, \"P-STATE: %d\\n\", cpu); \n\
            break; \n\
            case K_CALL_STACK: \n\
                               fprintf(stderr, \"K-CALL-STACK: %d\\n\", cpu); \n\
            break; \n\
            case M_MAP: \n\
                        fprintf(stderr, \"M-MAP: %d\\n\", cpu); \n\
            break; \n\
            case IRQ_MAP: \n\
                          fprintf(stderr, \"IRQ-MAP: %d\\n\", cpu); \n\
            break; \n\
            case PROC_MAP: \n\
                           fprintf(stderr, \"PROC-MAP: %d\\n\", cpu); \n\
            break; \n\
            default: \n\
                     fprintf(stderr, \"ERROR: UNKNOWN type = %d\\n\", sample.sample_type); \n\
            exit(-1); \n\
        } \n\
    } \n\
    fflush(stdout); \n\
}; \n\
      ";

      void Execer::generate_header_code_i(void){
          output_code += header_code;
      };

/*
   int get_next_line(str_t& line)
   {
   if(!std::cin.eof()){
   std::getline(std::cin, line);
   }
   return SUCCESS;
   };
   */

int get_next_line(str_t& line)
{
    char *rline = readline("lexer: ");
    if(!rline){
        return -ERROR;
    }
    line = std::string(rline);
    free(rline);
    return SUCCESS;
};

static SingletonResMon<Execer> s_execer_singleResMon;


int main(int argc, char *argv[])
{
    std::string file_name;
    switch(argc){
        case 2:
            file_name = argv[1];
        case 1:
            break;
        default:
            fprintf(stderr, "Usage: ./lexer <driver file name>\n");
            fprintf(stderr, "where <driver file name> is optional\n");
            exit(-1);
    }
    if(file_name.size()){
        fprintf(stderr, "DRIVER file name = %s\n", file_name.c_str());
        Execer::instance()->do_load_file(file_name);
    }

    str_t line;
    while(get_next_line(line) == SUCCESS){ // forever
        Parser parser(line);
        if(parser.prog()){
            fprintf(stderr, "PARSER error!\n");
            // break;
        }else{
            fprintf(stderr, "OUTPUT buffer = %s\n", parser.get_output().c_str());
            add_history(line.c_str());
        }
    }
    /*
    */
};
