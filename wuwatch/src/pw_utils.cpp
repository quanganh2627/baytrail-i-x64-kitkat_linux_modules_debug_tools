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
 * Utility function definitions.
 * *****************************************
 */

#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <iostream>
#include <fstream>

#include "pw_utils.hpp"

/*
 * The line reader class.
 */
LineReader::LineReader(){};
LineReader::~LineReader(){};

/*
 * Retrieve all lines from a file.
 */
void LineReader::get_all_lines(const char *file, std::vector<std::string>& lines)
{
    if(!file){
	return;
    }

    std::ifstream in_stream(file);

    if(!in_stream.is_open()){
	std::cerr << "Error: could NOT open file" << file <<"!" << std::endl;
	return;
    }

    std::string line;
    while(!in_stream.eof()){
	std::getline(in_stream, line);
	if(line.length()){
	    lines.push_back(line);
	}
    }

    in_stream.close();
};

/*
 * Retrieve all lines.
 */
void LineReader::get_all_lines(const char *file, std::deque<std::string>& lines)
{
    if(!file){
	return;
    }

    std::ifstream in_stream(file);

    if(!in_stream.is_open()){
	std::cerr << "Error: could NOT open file" << file <<"!" << std::endl;
	return;
    }

    std::string line;
    while(!in_stream.eof()){
	std::getline(in_stream, line);
	if(line.length()){
	    lines.push_back(line);
	}
    }

    in_stream.close();
};

void LineReader::get_all_lines(FILE *fp, std::vector<std::string>& lines)
{
    if(!fp){
	return;
    }
    std::string line;
    char tmp_buf[1024];
    size_t tmp_size = 0;

    while (!feof(fp)) {
        line.clear();
        do {
            tmp_buf[0] = '\0';
            if (fgets(tmp_buf, sizeof(tmp_buf), fp) == NULL) {
                break;
            }
            line.append(tmp_buf);
            tmp_size = strlen(tmp_buf);
            /*
             * Sanities.
             */
            if (tmp_size < 1) {
                tmp_size = 1;
            } else if (tmp_size >= sizeof(tmp_buf)) {
                tmp_size = sizeof(tmp_buf) - 1;
            }
        } while (tmp_buf[tmp_size-1] != '\n');
        /*
         * We do NOT want the terminating '\n' or any trailing whitespace.
         */
        trim(line);
        lines.push_back(line);
    }
};

void LineReader::get_all_lines(FILE *fp, std::deque<std::string>& lines)
{
    if(!fp){
	return;
    }
    std::string line;
    char tmp_buf[1024];
    size_t tmp_size = 0;

    while (!feof(fp)) {
        line.clear();
        do {
            tmp_buf[0] = '\0';
            if (fgets(tmp_buf, sizeof(tmp_buf), fp) == NULL) {
                break;
            }
            line.append(tmp_buf);
            tmp_size = strlen(tmp_buf);
            /*
             * Sanities.
             */
            if (tmp_size < 1) {
                tmp_size = 1;
            } else if (tmp_size >= sizeof(tmp_buf)) {
                tmp_size = sizeof(tmp_buf) - 1;
            }
        } while (tmp_buf[tmp_size-1] != '\n');
        /*
         * We do NOT want the terminating '\n' or any trailing whitespace.
         */
        trim(line);
        lines.push_back(line);
    }
};

void LineReader::trim(std::string& line)
{
    /*
     * Trim any remaining leading or trailing whitespace. We can't
     * use 'boost::trim()' here because of constraints on the power
     * library (must be statically compiled/linked for Android, e.g.).
     */
    if (line.size() == 0) {
        return;
    }
    std::string::size_type begin = line.find_first_not_of(" \t\n");
    std::string::size_type end = line.find_last_not_of(" \t\n");
    if (begin != std::string::npos) {
        line = line.substr(begin, (end-begin+1));
    }
};

/*
 * The string tokenizer class.
 * @str: the string to tokenize
 * @delim: the (possibly empty) DELIMS set to use in tokenizing 'str'
 */
Tokenizer::Tokenizer(const std::string& str, const char *delim):m_buffer(str), m_delims(delim), m_curr(0), m_next(0)
{
    if(m_delims && !m_buffer.empty()){
	m_curr = m_buffer.find_first_not_of(m_delims, m_next);
	m_next = m_buffer.find_first_of(m_delims, m_curr);

	m_vec.push_back(m_buffer.substr(m_curr, (m_next - m_curr)));

	m_curr = m_buffer.find_first_not_of(m_delims, m_next);
	m_next = m_buffer.find_first_of(m_delims, m_curr);

	while(m_curr != std::string::npos || m_next != std::string::npos){
	    m_vec.push_back(m_buffer.substr(m_curr, (m_next - m_curr)));
	    m_curr = m_buffer.find_first_not_of(m_delims, m_next);
	    m_next = m_buffer.find_first_of(m_delims, m_curr);
	}
	/*
	 * We may want to tokenize based on a different delim
	 * set. Reset the 'm_curr' and 'm_next' indices to
	 * account for that.
	 */
	m_curr = m_next = 0;
    }
};

/*
 * Tokenize the backing store string, using the (possibly empty) DELIMS 
 * set passed in to the 'Tokenizer' constructor.
 */
const char *Tokenizer::get_next_token()
{
    return get_next_token(m_delims);
};

/*
 * Tokenize the backing store string, using the (possibly empty)
 * DELIMS set specified by 'delims'.
 * ******************************************************************************
 * BASIC ALGO:
 * ******************************************************************************
 * The "m_curr" pointer points
 * to the first VALID character. The
 * "m_next" pointer points to the first NON-VALID character
 * AFTER "m_curr" (where a char is considered INVALID if it belongs to
 * any of the DELIM sets). Extract the substring bounded
 * by [m_curr, m_next).
 * We use the 'find_first_not_of(...)' function to extract
 * "m_curr" and the 'first_first_of(...)' function to
 * retrieve "m_next". These functions are documented in the
 * STL standard.
 *
 * @delims: the (possibly empty) DELIMS set.
 * @returns: the next token (if it exists), or NULL.
 */
const char *Tokenizer::get_next_token(const char *delims)
{
    if(!delims || m_buffer.empty()){
	return NULL;
    }

    m_curr = m_buffer.find_first_not_of(delims, m_next);
    m_next = m_buffer.find_first_of(delims, m_curr);

    if(m_curr == std::string::npos){
	return NULL;
    }

    return m_buffer.substr(m_curr, (m_next - m_curr)).c_str();
};

/*
 * Return ALL tokens for the current backing store string and DELIMS set.
 * @returns: a vector of tokens.
 */
const std::vector<std::string>& Tokenizer::get_all_tokens() const{
    return m_vec;
};

/*
 * Tokenizes the backing store string, using 'delims' as
 * the DELIMS set.
 * ******************************************************************************
 * BASIC ALGO:
 * ******************************************************************************
 * Maintain two ptrs. The "m_curr" pointer points
 * to the first VALID character. The
 * "m_next" pointer points to the first NON-VALID character
 * AFTER "m_curr" (where a char is considered INVALID if it belongs to
 * any of the DELIM sets). Extract the substring bounded
 * by [m_curr, m_next). Wash, rinse, repeat.
 * We use the 'find_first_not_of(...)' function to extract
 * "m_curr" and the 'first_first_of(...)' function to
 * retrieve "m_next". These functions are documented in the
 * STL standard.

 * @delims: the (possibly empty) DELIMS set.
 * @returns: a (possibly empty) vector of tokens.
 */
const std::vector<std::string> Tokenizer::get_all_tokens(const char *delims)
{
    if(delims == m_delims || m_buffer.empty()){
	return m_vec;
    }

    str_vec_t vec;

    if(!delims){
	return vec;
    }

    /*
     * Store previous values -- allows us
     * to "rewind" effects of current function
     * call.
     */
    int curr = m_curr, next = m_next;

    m_curr = m_next = 0;

    m_curr = m_buffer.find_first_not_of(delims, m_next);
    m_next = m_buffer.find_first_of(delims, m_curr);

    vec.push_back(m_buffer.substr(m_curr, (m_next - m_curr)));

    m_curr = m_buffer.find_first_not_of(delims, m_next);
    m_next = m_buffer.find_first_of(delims, m_curr);

    while(m_curr != std::string::npos || m_next != std::string::npos){
	vec.push_back(m_buffer.substr(m_curr, (m_next - m_curr)));
	m_curr = m_buffer.find_first_not_of(delims, m_next);
	m_next = m_buffer.find_first_of(delims, m_curr);
    }

    m_curr = curr; m_next = next;
    return vec;
};

/*
 * EXTERNAL API:
 * Extract directory and file name information from a path.
 *
 * @path: the path to parse.
 * @dir: the extracted directory information.
 * @file: the extracted file information.
 */
void extract_dir_and_file_from_path(const std::string& path, std::string& dir, std::string& file)
{
    int size = path.size();
    assert(size > 0);
    /*
     * The file name is defined as everything AFTER the LAST '/'
     * (or the ENTIRE STRING, if no "/" is specified).
     */
    char last_char = path.c_str()[size-1];
    int last_slash = path.find_last_of("/");
    if (last_slash == std::string::npos) {
        /*
         * No "/" found -- either the 'path' supplied
         * was the file name itself or it was "."
         * In either case, the directory is "./".
         */
        dir = "./";
        file = path;
    } else if (last_slash == (size-1)) {
        // Could happen, for example, if the user specified a "-o /"
        dir = path;
        file = "";
    } else {
        // A DIR component WAS specified -- extract it here.
        dir = path.substr(0, last_slash+1);
        file = path.substr(last_slash+1);
    }
};
